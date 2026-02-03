#include "hp_vector_device.h"
#include <Arduino.h>
#include <atomic>
#include <cctype>
#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "screen.h"

// ------------------------------------------------------------
// Pin assignment (Heltec WiFi Kit 32 V3 / WiFi LoRa 32 V3)
// ------------------------------------------------------------
static const HPVectorDevice::Pins kHpPins = 
{
    20, // dav
    19, // rfd
    -1, // ds (unused for immediate mode)
    -1, // rd (unused)
    -1, // xack (unused)
    {1, 2, 3, 4, 5, 6, 7, 39, 40, 41, 42, 37, 33, 47, 48}
};

static HPVectorDevice g_hp(kHpPins);

// ------------------------------------------------------------
// Protocol / mode state
// ------------------------------------------------------------
enum class UIMode
{
    Binary,
    Console
};

struct Stats
{
    std::atomic<uint32_t> packets_ok{0};
    std::atomic<uint32_t> packets_bad_crc{0};
    std::atomic<uint32_t> parse_resyncs{0};
    std::atomic<uint32_t> playback_timeouts{0};
    std::atomic<uint32_t> words_received{0};
    std::atomic<uint32_t> frames_rendered{0};
    std::atomic<uint32_t> frames_received{0};  // New frames swapped in
};

static Stats g_stats;

// Double-buffered frame data using shared_ptr for atomic swap.
// Staging buffer accumulates incoming words until commit.
// Active buffer is what the drawing task renders (atomically swapped).
static std::vector<uint16_t> g_staging;
static std::shared_ptr<std::vector<uint16_t>> g_active;
static std::mutex g_stagingMutex;  // Protects g_staging only
static std::mutex g_activeMutex;   // Protects g_active only (very brief holds)

// FreeRTOS task handles
static TaskHandle_t g_screenTask = nullptr;
static TaskHandle_t g_loopTask = nullptr;
static TaskHandle_t g_serialTask = nullptr;

// OLED status data
static char g_modeLine[32] = "";
static char g_fpsLine[32] = "";
static char g_vecLine[32] = "";
static uint32_t g_screenLastFrames = 0;
static uint32_t g_screenLastMs = 0;

// Total vectors drawn (for stats overlay)
static std::atomic<uint64_t> g_totalVectorsDrawn{0};

static size_t activeWordCount()
{
    std::lock_guard<std::mutex> lock(g_activeMutex);
    return g_active ? g_active->size() : 0;
}

static std::atomic<bool> g_loopEnabled{false};
static std::atomic<bool> g_swapEnabled{true};  // DEBUG: 'd' key toggles this
static std::atomic<uint16_t> g_loopHz{60};
static std::atomic<uint32_t> g_lastLoopMs{0};
static std::atomic<uint32_t> g_lastFrameUs{0};  // Microseconds for accurate FPS
static std::atomic<UIMode> g_mode{UIMode::Binary};
static std::atomic<uint32_t> g_lastRxMs{0};
static std::atomic<uint32_t> g_lastIdleMs{0};
static std::atomic<bool> g_idleActive{false};

static uint16_t clampCoord(int v)
{
    if (v < 0)
        return 0;
    if (v > 2047)
        return 2047;
    return (uint16_t)v;
}

// Escape detection (pause +++ pause)
static constexpr uint32_t kGuardMs = 500;
static uint32_t g_lastByteMs = 0;
static bool g_guardPrimed = false;
static uint8_t g_plusRun = 0;
static bool g_waitingGuard2 = false;
static uint32_t g_guard2StartMs = 0;

// ------------------------------------------------------------
// CRC16-CCITT (poly 0x1021, init 0xFFFF)
// ------------------------------------------------------------
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b)
        {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// ------------------------------------------------------------
// Escape sequence detector
// ------------------------------------------------------------
static void resetEscapeDetector()
{
    g_guardPrimed = false;
    g_plusRun = 0;
    g_waitingGuard2 = false;
    g_guard2StartMs = 0;
    g_lastByteMs = millis();
}

static void feedEscapeDetector(uint8_t byte, uint32_t nowMs)
{
    const uint32_t delta = nowMs - g_lastByteMs;

    if (g_waitingGuard2)
    {
        g_waitingGuard2 = false;
        g_guardPrimed = false;
        g_plusRun = 0;
    }

    if (delta >= kGuardMs)
    {
        g_guardPrimed = true;
        g_plusRun = 0;
    }

    if (g_guardPrimed)
    {
        if (byte == '+')
        {
            g_plusRun++;
            if (g_plusRun == 3)
            {
                g_waitingGuard2 = true;
                g_guard2StartMs = nowMs;
            }
            else if (g_plusRun > 3)
            {
                g_guardPrimed = false;
                g_plusRun = 0;
                g_waitingGuard2 = false;
            }
        }
        else
        {
            g_guardPrimed = false;
            g_plusRun = 0;
        }
    }

    g_lastByteMs = nowMs;
}

static bool escapeGuard2Satisfied(uint32_t nowMs)
{
    if (!g_waitingGuard2)
        return false;
    if ((nowMs - g_guard2StartMs) >= kGuardMs)
    {
        resetEscapeDetector();
        return true;
    }
    return false;
}

// ------------------------------------------------------------
// Playback helpers
// ------------------------------------------------------------
static bool playbackActiveOnce()
{
    // Get atomic snapshot of active buffer
    std::shared_ptr<std::vector<uint16_t>> local;
    {
        std::lock_guard<std::mutex> lock(g_activeMutex);
        local = g_active;
    }

    // Calculate FPS for stats overlay using microseconds for accuracy
    const uint32_t nowUs = (uint32_t)esp_timer_get_time();
    const uint32_t lastUs = g_lastFrameUs.exchange(nowUs);
    uint32_t fps_x10 = 0;
    if (lastUs > 0 && nowUs > lastUs)
    {
        const uint32_t deltaUs = nowUs - lastUs;
        if (deltaUs > 0)
        {
            fps_x10 = 10000000u / deltaUs;  // 10M / deltaUs gives FPS * 10
        }
    }

    // Calculate vectors per frame
    const uint32_t wordsPerFrame = local ? (uint32_t)local->size() : 0;
    const uint32_t vecPerFrame = wordsPerFrame / 4u;

    // Draw the frame data if any
    if (local && !local->empty())
    {
        size_t count = 0;
        for (uint16_t w : *local)
        {
            if (!g_hp.SendWord(w))
            {
                g_stats.playback_timeouts.fetch_add(1, std::memory_order_relaxed);
                g_loopEnabled.store(false, std::memory_order_relaxed);
                return false;
            }
            // vTaskDelay lets IDLE task run to feed watchdog every 32 words
            if (++count % 32 == 0)
            {
                //vTaskDelay(1);
            }
        }
        g_totalVectorsDrawn.fetch_add(vecPerFrame, std::memory_order_relaxed);
    }
    const uint64_t totalVec = g_totalVectorsDrawn.load(std::memory_order_relaxed);

    // Reset intensity to max before drawing stats overlay
    g_hp.SendWord(0x63FFu);  // Intensity max (param 000, value 0x3FF)
    g_hp.SendWord(0x6FFFu);  // Extra bits set (safety for undocumented behavior)
    g_hp.SendWord(0x67FFu);  // Focus max (param 001)
    g_hp.SendWord(0x7000u);  // Writing rate slow (solid lines)

    g_hp.SendWord(g_hp.MakeXWord(0, false));
    g_hp.SendWord(g_hp.MakeYWord(0, false));

    // Always draw stats overlay at top of screen
    char line1[64];
    char line2[64];
    char line3[64];
    snprintf(line1, sizeof(line1), "FPS/s: %lu.%lu",
             (unsigned long)(fps_x10 / 10u),
             (unsigned long)(fps_x10 % 10u));
    snprintf(line2, sizeof(line2), "Vec/frame: %lu",
             (unsigned long)vecPerFrame);
    snprintf(line3, sizeof(line3), "Total: %llu",
             (unsigned long long)totalVec);\

    #if USE_OSD
        g_hp.DrawTextAtSized(20, 1950, line1, 1, 0);
        g_hp.DrawTextAtSized(20, 1880, line2, 1, 0);
        g_hp.DrawTextAtSized(20, 1810, line3, 1, 0);
    #endif

    g_stats.frames_rendered.fetch_add(1, std::memory_order_relaxed);
    return true;
}

static uint16_t estimateTextWidth(uint8_t size, size_t len)
{
    const uint16_t base = 24;
    const uint16_t scale = (uint16_t)(size + 1);
    return (uint16_t)(len * base * scale);
}

static void drawIdleSplash()
{
    // Starfield background.
    const float cx = 1024.0f;
    const float cy = 1024.0f;
    const float focal = 1400.0f;
    const float zNear = 0.2f;
    const float zFar = 2.2f;
    const int count = 120;

    const float t = millis() * 0.001f;
    const float zSpan = zFar - zNear;
    const float speed = 0.5f;

    auto rand01 = [](uint32_t &seed) -> float {
        seed = seed * 1664525u + 1013904223u;
        return (seed >> 8) * (1.0f / 16777216.0f);
    };

    // Line clipping function (Liang-Barsky algorithm)
    auto clipLine = [](float &x0, float &y0, float &x1, float &y1,
                       float xmin, float xmax, float ymin, float ymax) -> bool {
        float t0 = 0.0f, t1 = 1.0f;
        float dx = x1 - x0;
        float dy = y1 - y0;
        
        auto clipTest = [&](float p, float q) -> bool {
            if (p == 0.0f) return q >= 0.0f;
            float r = q / p;
            if (p < 0.0f) {
                if (r > t1) return false;
                if (r > t0) t0 = r;
            } else {
                if (r < t0) return false;
                if (r < t1) t1 = r;
            }
            return true;
        };
        
        if (!clipTest(-dx, x0 - xmin)) return false;  // Left
        if (!clipTest(dx, xmax - x0)) return false;   // Right
        if (!clipTest(-dy, y0 - ymin)) return false;  // Bottom
        if (!clipTest(dy, ymax - y0)) return false;   // Top
        
        if (t1 < 1.0f) {
            x1 = x0 + t1 * dx;
            y1 = y0 + t1 * dy;
        }
        if (t0 > 0.0f) {
            x0 = x0 + t0 * dx;
            y0 = y0 + t0 * dy;
        }
        return true;
    };

    // Initialize star passcount values (90% get 1, 10% get 4 for brightness variation)
    static std::vector<int> starPasscounts;
    static bool passcountsInitialized = false;
    if (!passcountsInitialized)
    {
        passcountsInitialized = true;
        uint32_t seed = 0x9876543u;
        for (int i = 0; i < count; ++i)
        {
            seed = seed * 1664525u + 1013904223u;
            float randVal = (seed >> 8) * (1.0f / 16777216.0f);
            int passcount = (randVal < 0.1f) ? 4 : 1;  // 10% get passcount=4, rest get 1
            starPasscounts.push_back(passcount);
        }
    }

    for (int i = 0; i < count; ++i)
    {
        uint32_t seed = 0x12345678u ^ (uint32_t)(i * 747796405u);
        const float angle = rand01(seed) * 6.2831853f;
        const float radius = rand01(seed) * 0.35f;
        const float x = cosf(angle) * radius;
        const float y = sinf(angle) * radius;
        const float zBase = rand01(seed) * zSpan;
        float z = zBase - fmodf(t * speed, zSpan);
        if (z < 0.0f)
            z += zSpan;
        z += zNear;
        const float px = cx + (x * focal) / z;
        const float py = cy + (y * focal) / z;
        
        // Calculate screen velocity-based trail: project where the star was a moment ago
        const float zPrev = z + speed * 0.05f;  // Where it was 0.05s ago
        const float pxPrev = cx + (x * focal) / zPrev;
        const float pyPrev = cy + (y * focal) / zPrev;
        
        // Trail points from previous position (creating motion blur)
        const float dx = px - pxPrev;
        const float dy = py - pyPrev;
        
        // Scale trail intensity by proximity (cubic falloff for dramatic effect)
        const float falloff = 1.0f - (z - zNear) / (zFar - zNear);
        const float intensity = falloff * falloff * falloff;
        
        const float tx = px + dx * intensity * 3.0f;  // Amplify trail
        const float ty = py + dy * intensity * 3.0f;
        
        // Clip line to screen bounds [0, 2047]
        float x0 = px, y0 = py, x1 = tx, y1 = ty;
        if (clipLine(x0, y0, x1, y1, 0.0f, 2047.0f, 0.0f, 2047.0f))
        {
            // Draw the line multiple times based on star's passcount for brightness variation
            const int passcount = starPasscounts[i];
            for (int pass = 0; pass < passcount; ++pass)
            {
                g_hp.MoveTo(clampCoord((int)x0), clampCoord((int)y0));
                g_hp.LineTo(clampCoord((int)x1), clampCoord((int)y1));
            }
        }
    }

    // Static background stars (initialized once)
    static std::vector<std::pair<uint16_t, uint16_t>> staticStars;
    static std::vector<bool> staticStarsVisible;
    static bool staticStarsInitialized = false;
    static uint32_t lastTwinkleMs = 0;
    
    if (!staticStarsInitialized)
    {
        staticStarsInitialized = true;
        uint32_t seed = 0xABCDEF01u;
        auto rand01 = [](uint32_t &s) -> float {
            s = s * 1664525u + 1013904223u;
            return (s >> 8) * (1.0f / 16777216.0f);
        };
        for (int i = 0; i < 50; ++i)
        {
            uint16_t x = (uint16_t)(rand01(seed) * 2047.0f);
            uint16_t y = (uint16_t)(rand01(seed) * 2047.0f);
            staticStars.emplace_back(x, y);
            staticStarsVisible.push_back(true);  // Start all visible
        }
        lastTwinkleMs = millis();
    }
    
    // Update twinkle state every 200ms
    const uint32_t nowMs = millis();
    if (nowMs - lastTwinkleMs >= 200)
    {
        lastTwinkleMs = nowMs;
        uint32_t seed = nowMs ^ 0xDEADBEEF;
        for (size_t i = 0; i < staticStarsVisible.size(); ++i)
        {
            seed = seed * 1664525u + 1013904223u;
            float randVal = (seed >> 8) * (1.0f / 16777216.0f);
            staticStarsVisible[i] = (randVal > 0.3f);  // 70% chance to be visible
        }
    }
    
    // Draw static stars as small dots (only if visible)
    for (size_t i = 0; i < staticStars.size(); ++i)
    {
        if (staticStarsVisible[i])
        {
            g_hp.MoveTo(staticStars[i].first, staticStars[i].second);
            g_hp.LineTo(staticStars[i].first, staticStars[i].second);
        }
    }

    // Title and subtitle text.
    const char *title = "HP 1345a";
    const char *subtitle1 = "ESP32 Bridge";
    const char *subtitle2 = "(C)2026 Plummer's Software LLC";
    const char *url = "http://github.com/davepl/vector";
    const uint16_t titleY = 1536;  // 1/4 down from top in HP coords
    const uint16_t subtitleY = 512; // 3/4 down from top

    // Manual tweaks to centering because the estimate isn't very accurate!

    const uint16_t titleW = estimateTextWidth(3, strlen(title)) - 100;
    const uint16_t subtitle1W = estimateTextWidth(3, strlen(subtitle1)) - 150;
    const uint16_t subtitle2W = estimateTextWidth(1, strlen(subtitle2)) + 100;
    const uint16_t urlW = estimateTextWidth(0, strlen(url)) + 250;

    const uint16_t titleX     = (2047u > titleW) ? (uint16_t)((2047u - titleW) / 2u) : 0u;
    const uint16_t subtitle1X = (2047u > subtitle1W) ? (uint16_t)((2047u - subtitle1W) / 2u): 0u;
    const uint16_t subtitle2X = (2047u > subtitle2W) ? (uint16_t)((2047u - subtitle2W) / 2u): 0u;
    const uint16_t urlX       = (2047u > urlW) ? (uint16_t)((2047u - urlW) / 2u): 0u;

    // We overdraw some of these multiple times to make them brighter

    g_hp.DrawTextAtSized(titleX, titleY, title, 3, 0);
    g_hp.DrawTextAtSized(titleX, titleY, title, 3, 0);
    g_hp.DrawTextAtSized(titleX, titleY, title, 3, 0);
    g_hp.DrawTextAtSized(titleX, titleY, title, 3, 0);
    g_hp.DrawTextAtSized(subtitle1X, subtitleY, subtitle1, 3, 0);
    g_hp.DrawTextAtSized(subtitle1X, subtitleY, subtitle1, 3, 0);
    g_hp.DrawTextAtSized(subtitle2X, subtitleY - 120, subtitle2, 1, 0);
    g_hp.DrawTextAtSized(urlX, subtitleY - 220, url, 0, 0);
}

// ------------------------------------------------------------
// Drawing task - runs on Core 0, continuously renders frames
// ------------------------------------------------------------
static void LoopTask(void *param)
{
    (void)param;
    while (true)
    {
        const uint32_t nowMs = millis();
        const uint32_t lastRx = g_lastRxMs.load(std::memory_order_relaxed);
        const bool idleTimeout = (lastRx == 0) || ((nowMs - lastRx) > 500u);
        if (idleTimeout)
        {
            g_idleActive.store(true, std::memory_order_relaxed);
            g_loopEnabled.store(false, std::memory_order_relaxed);  // Stop looping on timeout
        }
        if (g_idleActive.load(std::memory_order_relaxed) &&
            !g_loopEnabled.load(std::memory_order_relaxed))
        {
            g_lastIdleMs.store(nowMs, std::memory_order_relaxed);
            drawIdleSplash();
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (!g_loopEnabled.load(std::memory_order_relaxed))
        {
            // Even when not looping, draw stats overlay periodically
            playbackActiveOnce();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        const uint16_t hz = g_loopHz.load(std::memory_order_relaxed);
        const uint32_t now = millis();
        if (hz > 0)
        {
            const uint32_t interval = 1000u / hz;
            const uint32_t last = g_lastLoopMs.load(std::memory_order_relaxed);
            if (last != 0 && (now - last) < interval)
            {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
        }
        g_lastLoopMs.store(now, std::memory_order_relaxed);
        playbackActiveOnce();
        // Always yield after frame to feed watchdog
        vTaskDelay(1);
    }
}

// ------------------------------------------------------------
// Buffer control helpers
// ------------------------------------------------------------
static void swapStagingToActive()
{
    // Always count received frames for Rx stat
    g_stats.frames_received.fetch_add(1, std::memory_order_relaxed);
    
    // DEBUG: If swap disabled, don't actually swap (to test if swapping causes flicker)
    if (!g_swapEnabled.load(std::memory_order_relaxed))
        return;
    
    // Move staging to new active (no copy - just pointer transfer)
    std::shared_ptr<std::vector<uint16_t>> newActive;
    {
        std::lock_guard<std::mutex> lock(g_stagingMutex);
        // Move the staging vector contents to new shared_ptr (no data copy)
        newActive = std::make_shared<std::vector<uint16_t>>(std::move(g_staging));
        g_staging.clear();  // Reset staging for next frame
    }
    // Swap active pointer (hold active mutex very briefly)
    {
        std::lock_guard<std::mutex> lock(g_activeMutex);
        g_active = newActive;
    }
    g_idleActive.store(false, std::memory_order_relaxed);
}

static void clearBuffers(bool clearActive, bool clearStaging)
{
    if (clearActive)
    {
        std::lock_guard<std::mutex> lock(g_activeMutex);
        g_active.reset();
    }
    if (clearStaging)
    {
        std::lock_guard<std::mutex> lock(g_stagingMutex);
        g_staging.clear();
    }
}

// ------------------------------------------------------------
// Binary packet parser
// ------------------------------------------------------------
class BinaryParser
{
public:
    void Reset()
    {
        state_ = State::Sync;
        preambleIndex_ = 0;
        payload_.clear();
        payloadNeeded_ = 0;
        expectCrc_ = false;
        cmd_ = 0;
        flags_ = 0;
        lenWords_ = 0;
    }

    void ProcessByte(uint8_t byte)
    {
        switch (state_)
        {
        case State::Sync:
            handleSync(byte);
            break;
        case State::Cmd:
            cmd_ = byte;
            state_ = State::Flags;
            break;
        case State::Flags:
            flags_ = byte;
            state_ = State::Len0;
            break;
        case State::Len0:
            lenWords_ = byte;
            state_ = State::Len1;
            break;
        case State::Len1:
            lenWords_ |= (uint16_t)byte << 8;
            payloadNeeded_ = (size_t)lenWords_ * 2u;
            expectCrc_ = (flags_ & 0x01) != 0;
            payload_.clear();
            if (payloadNeeded_ > kMaxPayloadBytes)
            {
                g_stats.parse_resyncs.fetch_add(1, std::memory_order_relaxed);
                Reset();
                break;
            }
            if (payloadNeeded_ == 0)
            {
                state_ = expectCrc_ ? State::Crc0 : State::Dispatch;
            }
            else
            {
                state_ = State::Payload;
            }
            break;
        case State::Payload:
            payload_.push_back(byte);
            if (payload_.size() >= payloadNeeded_)
            {
                state_ = expectCrc_ ? State::Crc0 : State::Dispatch;
            }
            break;
        case State::Crc0:
            crcBytes_[0] = byte;
            state_ = State::Crc1;
            break;
        case State::Crc1:
            crcBytes_[1] = byte;
            state_ = State::Dispatch;
            break;
        case State::Dispatch:
            Reset();
            break;
        }

        if (state_ == State::Dispatch)
        {
            dispatchPacket();
            Reset();
        }
    }

private:
    static constexpr size_t kMaxPayloadBytes = 128 * 1024;
    enum class State
    {
        Sync,
        Cmd,
        Flags,
        Len0,
        Len1,
        Payload,
        Crc0,
        Crc1,
        Dispatch
    };

    void handleSync(uint8_t byte)
    {
        static const uint8_t preamble[4] = {0xA5, 0x5A, 0xC3, 0x3C};
        if (byte == preamble[preambleIndex_])
        {
            preambleIndex_++;
            if (preambleIndex_ == 4)
            {
                state_ = State::Cmd;
                preambleIndex_ = 0;
            }
        }
        else
        {
            if (preambleIndex_ != 0)
            {
                g_stats.parse_resyncs.fetch_add(1, std::memory_order_relaxed);
            }
            preambleIndex_ = (byte == preamble[0]) ? 1 : 0;
        }
    }

    void dispatchPacket()
    {
        if (expectCrc_)
        {
            std::vector<uint8_t> crcBuf;
            crcBuf.reserve(4 + payload_.size());
            crcBuf.push_back(cmd_);
            crcBuf.push_back(flags_);
            crcBuf.push_back((uint8_t)(lenWords_ & 0xFF));
            crcBuf.push_back((uint8_t)(lenWords_ >> 8));
            crcBuf.insert(crcBuf.end(), payload_.begin(), payload_.end());

            const uint16_t received =
                (uint16_t)crcBytes_[0] | ((uint16_t)crcBytes_[1] << 8);
            const uint16_t computed = crc16_ccitt(crcBuf.data(), crcBuf.size());
            if (computed != received)
            {
                g_stats.packets_bad_crc.fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }

        handleCommand();
    }

    void handleCommand()
    {
        switch (cmd_)
        {
        case 0x01: // WRITE_WORDS (append)
            appendWords(false);
            break;
        case 0x02: // REPLACE_WORDS
            appendWords(true);
            break;
        case 0x03: // CLEAR
            g_loopEnabled.store(false, std::memory_order_relaxed);
            clearBuffers(true, true);
            break;
        case 0x04: // COMMIT_ONCE
            g_loopEnabled.store(false, std::memory_order_relaxed);
            swapStagingToActive();
            break;
        case 0x05:
        { // COMMIT_LOOP
            uint16_t hz = 60;
            if (lenWords_ >= 1)
            {
                hz = (uint16_t)((payload_[1] << 8) | payload_[0]);
            }
            swapStagingToActive();
            g_loopHz.store(hz, std::memory_order_relaxed);
            g_loopEnabled.store(true, std::memory_order_relaxed);
            g_lastLoopMs.store(0, std::memory_order_relaxed);
            break;
        }
        case 0x06: // STOP_LOOP
            g_loopEnabled.store(false, std::memory_order_relaxed);
            break;
        case 0x07:
        { // SET_MODE
            uint16_t modeWord = 0;
            if (lenWords_ >= 1)
            {
                modeWord = (uint16_t)((payload_[1] << 8) | payload_[0]);
            }
            (void)modeWord;
            break;
        }
        case 0x08: // DEBUG: Toggle swap enable
        {
            bool newState = !g_swapEnabled.load(std::memory_order_relaxed);
            g_swapEnabled.store(newState, std::memory_order_relaxed);
            Serial.printf("\n*** SWAP %s ***\n", newState ? "ENABLED" : "DISABLED");
            break;
        }
        default:
            break;
        }

        g_lastRxMs.store(millis(), std::memory_order_relaxed);
        g_stats.packets_ok.fetch_add(1, std::memory_order_relaxed);
    }

    void appendWords(bool replace)
    {
        const size_t wordCount = lenWords_;
        if (wordCount == 0)
            return;

        std::vector<uint16_t> words;
        words.reserve(wordCount);
        for (size_t i = 0; i < wordCount; ++i)
        {
            const size_t idx = i * 2;
            const uint16_t w =
                (uint16_t)payload_[idx] | ((uint16_t)payload_[idx + 1] << 8);
            words.push_back(w);
        }

        {
            std::lock_guard<std::mutex> lock(g_stagingMutex);
            if (replace)
            {
                g_staging = std::move(words);  // Move, don't copy
            }
            else
            {
                g_staging.insert(g_staging.end(), words.begin(), words.end());
            }
        }

        g_stats.words_received.fetch_add((uint32_t)wordCount,
                                         std::memory_order_relaxed);
    }

    State state_ = State::Sync;
    uint8_t preambleIndex_ = 0;
    uint8_t cmd_ = 0;
    uint8_t flags_ = 0;
    uint16_t lenWords_ = 0;
    size_t payloadNeeded_ = 0;
    bool expectCrc_ = false;
    uint8_t crcBytes_[2] = {0, 0};
    std::vector<uint8_t> payload_;
};

static BinaryParser g_parser;

// ------------------------------------------------------------
// Console mode
// ------------------------------------------------------------
static void consolePrintln(const char *msg)
{
    Serial.print(msg);
    Serial.print("\r\n");
}

static void consolePrintf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    Serial.vprintf(fmt, args);
    va_end(args);
    Serial.print("\r\n");
}

static void enterConsoleMode()
{
    g_mode.store(UIMode::Console, std::memory_order_relaxed);
    g_hp.SetQuiet(false);
    consolePrintln("");
    consolePrintln("== HP1345A console ==");
    consolePrintln("Commands: ?,help, stat, clear, once, loop <hz>, stop, arm, "
                   "disarm, bin, r, pentest, bittest <n>, rawpen");
    Serial.print("> ");
}

static void returnToBinaryMode()
{
    g_hp.SetQuiet(true);
    g_mode.store(UIMode::Binary, std::memory_order_relaxed);
    g_parser.Reset();
    resetEscapeDetector();
}

// ------------------------------------------------------------
// OLED status task - runs on Core 1
// ------------------------------------------------------------
static void ScreenTask(void *param)
{
    (void)param;
    g_screenLastMs = millis();
    while (true)
    {
        const UIMode mode = g_mode.load(std::memory_order_relaxed);
        const uint32_t nowMs = millis();
        const uint32_t frames = g_stats.frames_rendered.load(std::memory_order_relaxed);
        const uint32_t dt = nowMs - g_screenLastMs;
        uint32_t fps_x10 = 0;
        if (dt > 0)
        {
            fps_x10 = (uint32_t)(((frames - g_screenLastFrames) * 10000u) / dt);
        }
        g_screenLastFrames = frames;
        g_screenLastMs = nowMs;

        const size_t wordsPerFrame = activeWordCount();
        const uint32_t vecPerFrame = (uint32_t)(wordsPerFrame / 4u);
        const uint32_t vecPerSec = (uint32_t)((vecPerFrame * (uint64_t)fps_x10) / 10u);
        snprintf(g_modeLine, sizeof(g_modeLine), "Mode: %s",
                 (mode == UIMode::Binary) ? "Binary" : "Console");
        snprintf(g_fpsLine, sizeof(g_fpsLine), "FPS: %lu",
                 (unsigned long)(round(fps_x10 / 10.0)));
        snprintf(g_vecLine, sizeof(g_vecLine), "Vec/s: %lu",
                 (unsigned long)vecPerSec);

        g_screen.Clear();
        g_screen.DrawBorder();
        g_screen.DrawLines(g_modeLine, g_fpsLine, g_vecLine);
        g_screen.Render();

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ------------------------------------------------------------
// Serial task - runs on Core 0, handles incoming data
// ------------------------------------------------------------
static void handleConsoleLine(const char *line);
static void pollConsoleInput();

static void SerialTask(void *param)
{
    (void)param;
    uint32_t lastOkPackets = g_stats.packets_ok.load(std::memory_order_relaxed);
    
    while (true)
    {
        const UIMode mode = g_mode.load(std::memory_order_relaxed);
        const uint32_t now = millis();
        bool sawPacket = false;

        if (mode == UIMode::Binary)
        {
            // Read all available bytes as fast as possible
            int avail = Serial.available();
            if (avail > 0)
            {
                // Process all currently available bytes in one burst
                while (avail > 0)
                {
                    const uint8_t b = (uint8_t)Serial.read();
                    const uint32_t tnow = millis();
                    
                    feedEscapeDetector(b, tnow);
                    g_parser.ProcessByte(b);
                    avail--;
                    const uint32_t okNow =
                        g_stats.packets_ok.load(std::memory_order_relaxed);
                    if (okNow != lastOkPackets)
                    {
                        sawPacket = true;
                        lastOkPackets = okNow;
                    }
                }
                // Yield after processing batch to let IDLE0 feed watchdog
                // At 921600 baud (~92 bytes/ms) with 8KB buffer, 1ms delay is safe
                vTaskDelay(1);
            }
            else
            {
                // Only delay when no data available
                vTaskDelay(1);
            }
            if (escapeGuard2Satisfied(now))
            {
                enterConsoleMode();
            }
        }
        else
        {
            pollConsoleInput();
            //vTaskDelay(1);
        }

        if (sawPacket)
        {
            g_lastRxMs.store(now, std::memory_order_relaxed);
        }
    }
}

static void handleConsoleLine(const char *line)
{
    char buf[96];
    size_t len = 0;
    for (const char *p = line; *p && len < sizeof(buf) - 1; ++p)
    {
        if (*p == '\r' || *p == '\n')
            continue;
        buf[len++] = (char)tolower((unsigned char)*p);
    }
    buf[len] = '\0';
    if (len == 0)
    {
        Serial.print("> ");
        return;
    }

    auto startsWith = [&](const char *prefix) -> bool
    {
        const size_t l = strlen(prefix);
        return len >= l && strncmp(buf, prefix, l) == 0;
    };

    if (buf[0] == '?' || startsWith("help"))
    {
        consolePrintln("Commands: ?,help, stat, clear, once, loop <hz>, stop, arm, "
                       "disarm, bin, r, pentest, bittest <n>, rawpen");
    }
    else if (startsWith("stat"))
    {
        const auto mode = g_mode.load(std::memory_order_relaxed);
        const bool loop = g_loopEnabled.load(std::memory_order_relaxed);
        uint16_t hz = g_loopHz.load(std::memory_order_relaxed);
        size_t stagingSize = 0, activeSize = 0;
        {
            std::lock_guard<std::mutex> lock(g_stagingMutex);
            stagingSize = g_staging.size();
        }
        {
            std::lock_guard<std::mutex> lock(g_activeMutex);
            activeSize = g_active ? g_active->size() : 0;
        }
        consolePrintf("mode=%s  loop=%s hz=%u  staging=%u words  active=%u words",
                      (mode == UIMode::Binary) ? "binary" : "console",
                      loop ? "on" : "off", (unsigned)hz, (unsigned)stagingSize,
                      (unsigned)activeSize);
        consolePrintf(
            "packets: ok=%lu crc_fail=%lu resync=%lu",
            (unsigned long)g_stats.packets_ok.load(std::memory_order_relaxed),
            (unsigned long)g_stats.packets_bad_crc.load(std::memory_order_relaxed),
            (unsigned long)g_stats.parse_resyncs.load(std::memory_order_relaxed));
        consolePrintf(
            "playback: frames=%lu timeouts=%lu  words_rx=%lu",
            (unsigned long)g_stats.frames_rendered.load(std::memory_order_relaxed),
            (unsigned long)g_stats.playback_timeouts.load(std::memory_order_relaxed),
            (unsigned long)g_stats.words_received.load(std::memory_order_relaxed));
    }
    else if (startsWith("clear"))
    {
        g_loopEnabled.store(false, std::memory_order_relaxed);
        clearBuffers(true, true);
        consolePrintln("Cleared buffers.");
    }
    else if (startsWith("once"))
    {
        g_loopEnabled.store(false, std::memory_order_relaxed);
        swapStagingToActive();
        consolePrintln("Played active buffer once.");
    }
    else if (startsWith("loop"))
    {
        uint16_t hz = 60;
        char *endp = nullptr;
        const char *num = strchr(buf, ' ');
        if (num)
        {
            hz = (uint16_t)strtoul(num + 1, &endp, 10);
        }
        swapStagingToActive();
        g_loopHz.store(hz, std::memory_order_relaxed);
        g_loopEnabled.store(true, std::memory_order_relaxed);
        g_lastLoopMs.store(0, std::memory_order_relaxed);
        consolePrintf("Looping active buffer at %u Hz (0=fast).", (unsigned)hz);
    }
    else if (startsWith("stop"))
    {
        g_loopEnabled.store(false, std::memory_order_relaxed);
        consolePrintln("Loop stopped.");
    }
    else if (startsWith("arm"))
    {
        g_hp.SetTransfersEnabled(true);
        consolePrintln("Bus armed (transfers enabled).");
    }
    else if (startsWith("disarm"))
    {
        g_hp.SetTransfersEnabled(false);
        consolePrintln("Bus disarmed (transfers paused).");
    }
    else if (startsWith("bin") || startsWith("r"))
    {
        consolePrintln("Returning to binary mode.");
        returnToBinaryMode();
        return;
    }
    else if (startsWith("pentest"))
    {
        // Draw a pattern that alternates pen up/down to test bit 11 (GPIO 37)
        consolePrintln("Pen test: Drawing 5 separate horizontal lines...");
        consolePrintln("If wiring is correct, you should see 5 SEPARATE lines.");
        consolePrintln("If bit 11 is stuck, you'll see one connected zigzag.");
        
        g_loopEnabled.store(false, std::memory_order_relaxed);
        
        // Clear and draw test pattern directly
        const uint16_t PEN_UP = 0x0000;   // bit 11 = 0 (move)
        const uint16_t PEN_DOWN = 0x0800; // bit 11 = 1 (draw)
        
        for (int i = 0; i < 5; i++)
        {
            uint16_t y = 500 + i * 200;  // Y positions: 500, 700, 900, 1100, 1300
            
            // Move to start (pen UP)
            g_hp.SendWord(200 | PEN_UP);        // X low
            g_hp.SendWord(y | PEN_UP);          // Y low  
            
            // Draw to end (pen DOWN)
            g_hp.SendWord(1800 | PEN_DOWN);     // X high
            g_hp.SendWord(y | PEN_DOWN);        // Y high
        }
        consolePrintln("Pen test complete.");
    }
    else if (startsWith("bittest"))
    {
        // Test individual data bits
        char *endp = nullptr;
        const char *num = strchr(buf, ' ');
        if (!num)
        {
            consolePrintln("Usage: bittest <bit_number 0-14>");
            consolePrintln("Bit 11 (GPIO 37) = pen state");
            Serial.print("> ");
            return;
        }
        int bit = (int)strtol(num + 1, &endp, 10);
        if (bit < 0 || bit > 14)
        {
            consolePrintln("Bit must be 0-14");
            Serial.print("> ");
            return;
        }
        
        uint16_t word = (1u << bit);
        consolePrintf("Sending word 0x%04X (bit %d set)", word, bit);
        consolePrintf("GPIO for bit %d = pin %d", bit, kHpPins.data[bit]);
        
        // Send several times to make it visible on scope
        for (int i = 0; i < 100; i++)
        {
            g_hp.SendWord(word);
            g_hp.SendWord(0x0000);
        }
        consolePrintln("Done - check with scope on that GPIO.");
    }
    else if (startsWith("rawpen"))
    {
        // Send raw words with explicit pen control for debugging
        consolePrintln("Sending raw pen test...");
        consolePrintln("Move to (100,100) pen UP, then draw to (1900,100) pen DOWN");
        
        // Pen UP move
        g_hp.SendWord(100);          // X=100, bit11=0 (pen up)
        g_hp.SendWord(100);          // Y=100, bit11=0 (pen up)
        
        // Pen DOWN draw  
        g_hp.SendWord(1900 | 0x0800); // X=1900, bit11=1 (pen down)
        g_hp.SendWord(100 | 0x0800);  // Y=100, bit11=1 (pen down)
        
        consolePrintln("If pen works: single horizontal line at Y=100");
        consolePrintln("If pen stuck DOWN: line from origin to (100,100) to (1900,100)");
        consolePrintln("If pen stuck UP: nothing visible");
    }
    else
    {
        consolePrintln("Unknown command. Type 'help' for list.");
    }

    Serial.print("> ");
}

static void pollConsoleInput()
{
    static char lineBuf[96];
    static size_t lineLen = 0;
    while (Serial.available() > 0)
    {
        const int c = Serial.read();
        if (c < 0)
            break;
        if (c == '\r' || c == '\n')
        {
            Serial.print("\r\n");
            lineBuf[lineLen] = '\0';
            handleConsoleLine(lineBuf);
            lineLen = 0;
            continue;
        }
        if (c == '\b' || c == 0x7F)
        {
            if (lineLen > 0)
            {
                lineLen--;
                Serial.print("\b \b");
            }
            continue;
        }
        if (lineLen < sizeof(lineBuf) - 1)
        {
            lineBuf[lineLen++] = (char)c;
            Serial.write((char)c);
        }
    }
}

// ------------------------------------------------------------
// Arduino setup/loop
// ------------------------------------------------------------

// Serial baud rate - must match client hptest.py and platformio.ini monitor_speed
static constexpr uint32_t kSerialBaud = 921600;

void setup()
{
    // Increase RX buffer to handle large packets at high baud rate
    Serial.setRxBufferSize(8192);
    Serial.begin(kSerialBaud);
    delay(100);

    g_hp.SetQuiet(true);
    g_hp.SetServiceCallback(nullptr);
    g_hp.ConfigurePins();
    g_hp.InitializeDisplay();
    g_hp.SetPenInvert(false);  // HP1345A: bit11=1 means pen DOWN

    // Initialize OLED and start screen task on Core 1
    const bool oledOk = g_screen.Init();
    if (oledOk)
    {
        g_screen.Clear();
        g_screen.DrawBorder();
        g_screen.Render();
        xTaskCreatePinnedToCore(
            ScreenTask,
            "ScreenTask",
            4096,
            nullptr,
            1,  // Low priority
            &g_screenTask,
            1); // Core 1
    }

    // Start drawing/loop task on Core 1 - completely isolated from serial
    xTaskCreatePinnedToCore(
        LoopTask,
        "LoopTask",
        8192,
        nullptr,
        2,  // Medium priority for rendering
        &g_loopTask,
        1); // Core 1 - dedicated to drawing

    // Start serial task on Core 0 - handles all serial I/O
    xTaskCreatePinnedToCore(
        SerialTask,
        "SerialTask",
        8192,
        nullptr,
        9,  // Higher priority to drain serial buffer
        &g_serialTask,
        0); // Core 0 - dedicated to serial

    resetEscapeDetector();
    g_parser.Reset();

    Serial.printf("HP1345A streamer ready (binary) - built %s %s\n", __DATE__, __TIME__);
    Serial.println("Send pause+++pause to enter console.");
}

void loop()
{
    // All work is handled by FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(100));
}
