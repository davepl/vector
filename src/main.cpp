#include <Arduino.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <cctype>
#include <mutex>
#include <vector>
#include "hp_vector_device.h"

// ------------------------------------------------------------
// Pin assignment (Heltec WiFi Kit 32 V3 / WiFi LoRa 32 V3)
// ------------------------------------------------------------
static const HPVectorDevice::Pins kHpPins = {
    20,  // dav
    19,  // rfd
    -1,  // ds (unused for immediate mode)
    -1,  // rd (unused)
    -1,  // xack (unused)
    {1, 2, 3, 4, 5, 6, 7, 39, 40, 41, 42, 37, 33, 47, 48}
};

static HPVectorDevice g_hp(kHpPins);

// ------------------------------------------------------------
// Protocol / mode state
// ------------------------------------------------------------
enum class UIMode { Binary, Console };

struct Stats {
    std::atomic<uint32_t> packets_ok{0};
    std::atomic<uint32_t> packets_bad_crc{0};
    std::atomic<uint32_t> parse_resyncs{0};
    std::atomic<uint32_t> playback_timeouts{0};
    std::atomic<uint32_t> words_received{0};
    std::atomic<uint32_t> frames_rendered{0};
};

static Stats g_stats;

static std::vector<uint16_t> g_staging;
static std::vector<uint16_t> g_active;
static std::mutex g_bufMutex;

static std::atomic<bool> g_loopEnabled{false};
static std::atomic<uint16_t> g_loopHz{60};
static std::atomic<uint32_t> g_lastLoopMs{0};
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
static uint16_t crc16_ccitt(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
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

    // If we were waiting for the second guard and a byte arrives early, abort.
    if (g_waitingGuard2) {
        g_waitingGuard2 = false;
        g_guardPrimed = false;
        g_plusRun = 0;
    }

    if (delta >= kGuardMs) {
        g_guardPrimed = true;
        g_plusRun = 0;
    }

    if (g_guardPrimed) {
        if (byte == '+') {
            g_plusRun++;
            if (g_plusRun == 3) {
                g_waitingGuard2 = true;
                g_guard2StartMs = nowMs;
            } else if (g_plusRun > 3) {
                g_guardPrimed = false;
                g_plusRun = 0;
                g_waitingGuard2 = false;
            }
        } else {
            g_guardPrimed = false;
            g_plusRun = 0;
        }
    }

    g_lastByteMs = nowMs;
}

static bool escapeGuard2Satisfied(uint32_t nowMs)
{
    if (!g_waitingGuard2) return false;
    if ((nowMs - g_guard2StartMs) >= kGuardMs) {
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
    std::vector<uint16_t> local;
    {
        std::lock_guard<std::mutex> lock(g_bufMutex);
        local = g_active;
    }
    if (local.empty()) return true;

    for (uint16_t w : local) {
        if (!g_hp.SendWord(w)) {
            g_stats.playback_timeouts.fetch_add(1, std::memory_order_relaxed);
            g_loopEnabled.store(false, std::memory_order_relaxed);
            return false;
        }
    }
    g_stats.frames_rendered.fetch_add(1, std::memory_order_relaxed);
    return true;
}

static void serviceLoopPlayback()
{
    if (!g_loopEnabled.load(std::memory_order_relaxed)) return;

    const uint16_t hz = g_loopHz.load(std::memory_order_relaxed);
    const uint32_t now = millis();
    if (hz > 0) {
        const uint32_t interval = (hz == 0) ? 0 : (1000u / hz);
        const uint32_t last = g_lastLoopMs.load(std::memory_order_relaxed);
        if (last != 0 && (now - last) < interval) return;
    }
    g_lastLoopMs.store(now, std::memory_order_relaxed);
    playbackActiveOnce();
}

// ------------------------------------------------------------
// Buffer control helpers
// ------------------------------------------------------------
static void swapStagingToActive()
{
    std::lock_guard<std::mutex> lock(g_bufMutex);
    g_active = g_staging;
}

static void clearBuffers(bool clearActive, bool clearStaging)
{
    std::lock_guard<std::mutex> lock(g_bufMutex);
    if (clearActive) g_active.clear();
    if (clearStaging) g_staging.clear();
}

// ------------------------------------------------------------
// Binary packet parser
// ------------------------------------------------------------
class BinaryParser {
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
        switch (state_) {
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
            if (payloadNeeded_ > kMaxPayloadBytes) {
                g_stats.parse_resyncs.fetch_add(1, std::memory_order_relaxed);
                Reset();
                break;
            }
            if (payloadNeeded_ == 0) {
                state_ = expectCrc_ ? State::Crc0 : State::Dispatch;
            } else {
                state_ = State::Payload;
            }
            break;
        case State::Payload:
            payload_.push_back(byte);
            if (payload_.size() >= payloadNeeded_) {
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
            // Should never get here; handled below.
            Reset();
            break;
        }

        if (state_ == State::Dispatch) {
            dispatchPacket();
            Reset();
        }
    }

private:
    static constexpr size_t kMaxPayloadBytes = 128 * 1024; // generous cap (64K words)
    enum class State { Sync, Cmd, Flags, Len0, Len1, Payload, Crc0, Crc1, Dispatch };

    void handleSync(uint8_t byte)
    {
        static const uint8_t preamble[4] = {0xA5, 0x5A, 0xC3, 0x3C};
        if (byte == preamble[preambleIndex_]) {
            preambleIndex_++;
            if (preambleIndex_ == 4) {
                state_ = State::Cmd;
                preambleIndex_ = 0;
            }
        } else {
            if (preambleIndex_ != 0) {
                g_stats.parse_resyncs.fetch_add(1, std::memory_order_relaxed);
            }
            preambleIndex_ = (byte == preamble[0]) ? 1 : 0;
        }
    }

    void dispatchPacket()
    {
        if (expectCrc_) {
            std::vector<uint8_t> crcBuf;
            crcBuf.reserve(4 + payload_.size());
            crcBuf.push_back(cmd_);
            crcBuf.push_back(flags_);
            crcBuf.push_back((uint8_t)(lenWords_ & 0xFF));
            crcBuf.push_back((uint8_t)(lenWords_ >> 8));
            crcBuf.insert(crcBuf.end(), payload_.begin(), payload_.end());

            const uint16_t received = (uint16_t)crcBytes_[0] | ((uint16_t)crcBytes_[1] << 8);
            const uint16_t computed = crc16_ccitt(crcBuf.data(), crcBuf.size());
            if (computed != received) {
                g_stats.packets_bad_crc.fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }

        handleCommand();
    }

    void handleCommand()
    {
        switch (cmd_) {
        case 0x01:  // WRITE_WORDS (append)
            appendWords(false);
            break;
        case 0x02:  // REPLACE_WORDS
            appendWords(true);
            break;
        case 0x03:  // CLEAR
            g_loopEnabled.store(false, std::memory_order_relaxed);
            clearBuffers(true, true);
            break;
        case 0x04:  // COMMIT_ONCE
            g_loopEnabled.store(false, std::memory_order_relaxed);
            swapStagingToActive();
            playbackActiveOnce();
            break;
        case 0x05: { // COMMIT_LOOP
            uint16_t hz = 60;
            if (lenWords_ >= 1) {
                hz = (uint16_t)((payload_[1] << 8) | payload_[0]);
            }
            swapStagingToActive();
            g_loopHz.store(hz, std::memory_order_relaxed);
            g_loopEnabled.store(true, std::memory_order_relaxed);
            g_lastLoopMs.store(0, std::memory_order_relaxed);
            break;
        }
        case 0x06:  // STOP_LOOP
            g_loopEnabled.store(false, std::memory_order_relaxed);
            break;
        case 0x07: { // SET_MODE
            uint16_t modeWord = 0;
            if (lenWords_ >= 1) {
                modeWord = (uint16_t)((payload_[1] << 8) | payload_[0]);
            }
            if (modeWord != 0) {
                // unsupported; ignore but do not count as error
            }
            break;
        }
        default:
            // Unknown command; ignore to stay in sync.
            break;
        }

        g_stats.packets_ok.fetch_add(1, std::memory_order_relaxed);
    }

    void appendWords(bool replace)
    {
        const size_t wordCount = lenWords_;
        if (wordCount == 0) return;

        std::vector<uint16_t> words;
        words.reserve(wordCount);
        for (size_t i = 0; i < wordCount; ++i) {
            const size_t idx = i * 2;
            const uint16_t w = (uint16_t)payload_[idx] | ((uint16_t)payload_[idx + 1] << 8);
            words.push_back(w);
        }

        {
            std::lock_guard<std::mutex> lock(g_bufMutex);
            if (replace) {
                g_staging = words;
            } else {
                g_staging.insert(g_staging.end(), words.begin(), words.end());
            }
        }

        g_stats.words_received.fetch_add((uint32_t)wordCount, std::memory_order_relaxed);
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
static void enterConsoleMode()
{
    g_mode.store(UIMode::Console, std::memory_order_relaxed);
    g_hp.SetQuiet(false);
    Serial.println();
    Serial.println("== HP1345A console ==");
    Serial.println("Commands: ?,help, stat, clear, once, loop <hz>, stop, arm, disarm, bin, r");
    Serial.print("> ");
}

static void returnToBinaryMode()
{
    g_hp.SetQuiet(true);
    g_mode.store(UIMode::Binary, std::memory_order_relaxed);
    g_parser.Reset();
    resetEscapeDetector();
}

static void handleConsoleLine(const char* line)
{
    // Simple whitespace trimming and lowercase.
    char buf[96];
    size_t len = 0;
    for (const char* p = line; *p && len < sizeof(buf) - 1; ++p) {
        if (*p == '\r' || *p == '\n') continue;
        buf[len++] = (char)tolower((unsigned char)*p);
    }
    buf[len] = '\0';
    if (len == 0) {
        Serial.print("> ");
        return;
    }

    auto startsWith = [&](const char* prefix) -> bool {
        const size_t l = strlen(prefix);
        return len >= l && strncmp(buf, prefix, l) == 0;
    };

    if (buf[0] == '?' || startsWith("help")) {
        Serial.println("Commands: ?,help, stat, clear, once, loop <hz>, stop, arm, disarm, bin, r");
    } else if (startsWith("stat")) {
        const auto mode = g_mode.load(std::memory_order_relaxed);
        const bool loop = g_loopEnabled.load(std::memory_order_relaxed);
        uint16_t hz = g_loopHz.load(std::memory_order_relaxed);
        size_t stagingSize = 0, activeSize = 0;
        {
            std::lock_guard<std::mutex> lock(g_bufMutex);
            stagingSize = g_staging.size();
            activeSize = g_active.size();
        }
        Serial.printf("mode=%s  loop=%s hz=%u  staging=%u words  active=%u words\n",
                      (mode == UIMode::Binary) ? "binary" : "console",
                      loop ? "on" : "off",
                      (unsigned)hz,
                      (unsigned)stagingSize,
                      (unsigned)activeSize);
        Serial.printf("packets: ok=%lu crc_fail=%lu resync=%lu\n",
                      (unsigned long)g_stats.packets_ok.load(std::memory_order_relaxed),
                      (unsigned long)g_stats.packets_bad_crc.load(std::memory_order_relaxed),
                      (unsigned long)g_stats.parse_resyncs.load(std::memory_order_relaxed));
        Serial.printf("playback: frames=%lu timeouts=%lu  words_rx=%lu\n",
                      (unsigned long)g_stats.frames_rendered.load(std::memory_order_relaxed),
                      (unsigned long)g_stats.playback_timeouts.load(std::memory_order_relaxed),
                      (unsigned long)g_stats.words_received.load(std::memory_order_relaxed));
    } else if (startsWith("clear")) {
        g_loopEnabled.store(false, std::memory_order_relaxed);
        clearBuffers(true, true);
        Serial.println("Cleared buffers.");
    } else if (startsWith("once")) {
        g_loopEnabled.store(false, std::memory_order_relaxed);
        swapStagingToActive();
        playbackActiveOnce();
        Serial.println("Played active buffer once.");
    } else if (startsWith("loop")) {
        uint16_t hz = 60;
        // Parse optional number after space.
        char* endp = nullptr;
        const char* num = strchr(buf, ' ');
        if (num) {
            hz = (uint16_t)strtoul(num + 1, &endp, 10);
        }
        swapStagingToActive();
        g_loopHz.store(hz, std::memory_order_relaxed);
        g_loopEnabled.store(true, std::memory_order_relaxed);
        g_lastLoopMs.store(0, std::memory_order_relaxed);
        Serial.printf("Looping active buffer at %u Hz (0=fast).\n", (unsigned)hz);
    } else if (startsWith("stop")) {
        g_loopEnabled.store(false, std::memory_order_relaxed);
        Serial.println("Loop stopped.");
    } else if (startsWith("arm")) {
        g_hp.SetTransfersEnabled(true);
        Serial.println("Bus armed (transfers enabled).");
    } else if (startsWith("disarm")) {
        g_hp.SetTransfersEnabled(false);
        Serial.println("Bus disarmed (transfers paused).");
    } else if (startsWith("bin") || startsWith("r")) {
        Serial.println("Returning to binary mode.");
        returnToBinaryMode();
        return;
    } else {
        Serial.println("Unknown command. Type 'help' for list.");
    }

    Serial.print("> ");
}

static void pollConsoleInput()
{
    static char lineBuf[96];
    static size_t lineLen = 0;
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c < 0) break;
        if (c == '\r' || c == '\n') {
            lineBuf[lineLen] = '\0';
            handleConsoleLine(lineBuf);
            lineLen = 0;
            continue;
        }
        if (lineLen < sizeof(lineBuf) - 1) {
            lineBuf[lineLen++] = (char)c;
        }
    }
}

// ------------------------------------------------------------
// Arduino setup/loop
// ------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(50);

    g_hp.SetQuiet(true);  // suppress prints during binary mode
    g_hp.SetServiceCallback(nullptr);
    g_hp.ConfigurePins();
    g_hp.InitializeDisplay();

    resetEscapeDetector();
    g_parser.Reset();

    Serial.println("HP1345A streamer ready (binary). Send pause+++pause to enter console.");
}

void loop()
{
    const UIMode mode = g_mode.load(std::memory_order_relaxed);
    const uint32_t now = millis();

    if (mode == UIMode::Binary) {
        while (Serial.available() > 0) {
            const uint8_t b = (uint8_t)Serial.read();
            const uint32_t tnow = millis();
            feedEscapeDetector(b, tnow);
            g_parser.ProcessByte(b);
        }
        if (escapeGuard2Satisfied(now)) {
            enterConsoleMode();
        }
    } else {
        pollConsoleInput();
    }

    serviceLoopPlayback();
}
