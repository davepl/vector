#include "hp_vector_device.h"
#include <Arduino.h>
#include <atomic>
#include <cctype>
#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
};

static Stats g_stats;

// Double-buffered frame data using shared_ptr for atomic swap.
// Staging buffer accumulates incoming words until commit.
// Active buffer is what the drawing task renders (atomically swapped).
static std::vector<uint16_t> g_staging;
static std::shared_ptr<std::vector<uint16_t>> g_active;
static std::mutex g_bufMutex;

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
    std::lock_guard<std::mutex> lock(g_bufMutex);
    return g_active ? g_active->size() : 0;
}

static std::atomic<bool> g_loopEnabled{false};
static std::atomic<uint16_t> g_loopHz{60};
static std::atomic<uint32_t> g_lastLoopMs{0};
static std::atomic<uint32_t> g_lastFrameMs{0};
static std::atomic<UIMode> g_mode{UIMode::Binary};

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
        std::lock_guard<std::mutex> lock(g_bufMutex);
        local = g_active;
    }

    // Calculate FPS for stats overlay
    const uint32_t nowMs = millis();
    const uint32_t lastMs = g_lastFrameMs.exchange(nowMs);
    uint32_t fps_x10 = 0;
    if (lastMs > 0 && nowMs > lastMs)
    {
        fps_x10 = 10000u / (nowMs - lastMs);
    }

    // Calculate vectors per frame and per second
    const uint32_t wordsPerFrame = local ? (uint32_t)local->size() : 0;
    const uint32_t vecPerFrame = wordsPerFrame / 4u;
    const uint64_t totalVec = g_totalVectorsDrawn.load(std::memory_order_relaxed);

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
            // vTaskDelay lets IDLE task run to feed watchdog
            if (++count % 128 == 0)
            {
                vTaskDelay(1);
            }
        }
        g_totalVectorsDrawn.fetch_add(vecPerFrame, std::memory_order_relaxed);
    }

    // Reset intensity to max before drawing stats overlay
    g_hp.SendWord(0x63FFu);  // Intensity max (param 000, value 0x3FF)
    g_hp.SendWord(0x6FFFu);  // Extra bits set (safety for undocumented behavior)
    g_hp.SendWord(0x67FFu);  // Focus max (param 001)
    g_hp.SendWord(0x7000u);  // Writing rate slow (solid lines)

    g_hp.SendWord(g_hp.MakeXWord(0, false));
    g_hp.SendWord(g_hp.MakeYWord(0, false));

    // Always draw stats overlay at top of screen
    char statsLine[64];
    snprintf(statsLine, sizeof(statsLine), "FPS:%lu.%lu Vec/f:%lu Tot:%llu",
             (unsigned long)(fps_x10 / 10u),
             (unsigned long)(fps_x10 % 10u),
             (unsigned long)vecPerFrame,
             (unsigned long long)(totalVec + vecPerFrame));
    g_hp.DrawTextAtSized(20, 1950, statsLine, 1, 0);

    g_stats.frames_rendered.fetch_add(1, std::memory_order_relaxed);
    return true;
}

// ------------------------------------------------------------
// Drawing task - runs on Core 0, continuously renders frames
// ------------------------------------------------------------
static void LoopTask(void *param)
{
    (void)param;
    while (true)
    {
        if (!g_loopEnabled.load(std::memory_order_relaxed))
        {
            // Even when not looping, draw stats overlay periodically
            playbackActiveOnce();
            vTaskDelay(pdMS_TO_TICKS(1));
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
    }
}

// ------------------------------------------------------------
// Buffer control helpers
// ------------------------------------------------------------
static void swapStagingToActive()
{
    std::lock_guard<std::mutex> lock(g_bufMutex);
    g_active = std::make_shared<std::vector<uint16_t>>(g_staging);
}

static void clearBuffers(bool clearActive, bool clearStaging)
{
    std::lock_guard<std::mutex> lock(g_bufMutex);
    if (clearActive)
        g_active.reset();
    if (clearStaging)
        g_staging.clear();
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
        default:
            break;
        }

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
            std::lock_guard<std::mutex> lock(g_bufMutex);
            if (replace)
            {
                g_staging = words;
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
        snprintf(g_fpsLine, sizeof(g_fpsLine), "FPS: %lu.%lu",
                 (unsigned long)(fps_x10 / 10u),
                 (unsigned long)(fps_x10 % 10u));
        snprintf(g_vecLine, sizeof(g_vecLine), "Vec/s: %lu",
                 (unsigned long)vecPerSec);

        g_screen.Clear();
        g_screen.DrawBorder();
        g_screen.DrawLines(g_modeLine, g_fpsLine, g_vecLine);
        g_screen.Render();

        vTaskDelay(pdMS_TO_TICKS(500));
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
    while (true)
    {
        const UIMode mode = g_mode.load(std::memory_order_relaxed);
        const uint32_t now = millis();

        if (mode == UIMode::Binary)
        {
            // Read all available bytes without delay to prevent buffer overflow
            int avail = Serial.available();
            while (avail > 0)
            {
                const uint8_t b = (uint8_t)Serial.read();
                const uint32_t tnow = millis();
                feedEscapeDetector(b, tnow);
                g_parser.ProcessByte(b);
                avail--;
                
                // Yield every 256 bytes to let other tasks run
                static int yieldCount = 0;
                if (++yieldCount >= 256)
                {
                    yieldCount = 0;
                    taskYIELD();
                }
            }
            if (escapeGuard2Satisfied(now))
            {
                enterConsoleMode();
            }
        }
        else
        {
            pollConsoleInput();
        }

        // Only delay if no data was available
        if (Serial.available() == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
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
            std::lock_guard<std::mutex> lock(g_bufMutex);
            stagingSize = g_staging.size();
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
void setup()
{
    // Increase RX buffer to handle large packets at high baud rate
    Serial.setRxBufferSize(4096);
    Serial.begin(921600);
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

    // Start drawing/loop task on Core 0
    xTaskCreatePinnedToCore(
        LoopTask,
        "LoopTask",
        8192,
        nullptr,
        3,  // Higher priority for smooth rendering
        &g_loopTask,
        0); // Core 0

    // Start serial task on Core 0
    xTaskCreatePinnedToCore(
        SerialTask,
        "SerialTask",
        8192,
        nullptr,
        2,  // Medium priority
        &g_serialTask,
        0); // Core 0

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
