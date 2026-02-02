#include <Arduino.h>
#include "driver/gpio.h"
#include <math.h>
#include <algorithm>
#include <stdio.h>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include "screen.h"

// -------------------------
// ESP32-S3 -> HP1345A Wiring
// -------------------------
// HP 1345A rear connector signals used in immediate mode:
//   - Data: D0..D14
//   - DAV (host output, active low)
//   - RFD (display output, active low)
//
// IMPORTANT: Protect ESP32 input pins (RFD, XACK) from 5V.
// Use a resistor divider (e.g., 10k series from HP->GPIO, 20k GPIO->GND).
//
// Also: HP pin 7 (DISCONNECT SENSE) must be tied to GND on your cable harness.

// -------------------------
// Pin assignment (Heltec WiFi Kit 32 V3 / WiFi LoRa 32 V3)
//
// Notes:
// - Avoid OLED pins: GPIO17 (SDA), GPIO18 (SCL), GPIO21 (RST)
// - Avoid Vext Ctrl: GPIO36
// - Avoid flash/PSRAM pins: GPIO33-38, GPIO26
// - Avoid CP2102 UART pins: GPIO43/44
// - GPIO0/3/45/46 are strapping pins; used here ONLY as outputs.
//
// Data bus: 15 pins
// D0..D14 -> {8,2,3,4,5,6,7,39,40,41,42,45,46,47,48}
//
// Control pins:
// DAV/WR -> GPIO20 (OUT)  (Immediate mode uses DAV)
// RFD    -> GPIO19 (IN)   (Immediate mode reads RFD)
// DS     -> GPIO37 (OUT)  (704 mode later)
// RD     -> GPIO38 (OUT)  (704 mode later)
// XACK   -> GPIO39 (IN)   (704 mode later)
// -------------------------

static const int PIN_DAV  = 20;
static const int PIN_RFD  = 19;

// Present for later, not used in this bring-up:
static const int PIN_DS   = -1;
static const int PIN_RD   = -1;
static const int PIN_XACK = -1;

// D0..D14 bit order: bit0 -> D0 ... bit14 -> D14
// Avoid strapping pins GPIO0/3/45/46 so ESP32 can be programmed while HP is powered.
static const int DATA_PINS[15] = {
    1,  2,  3,  4,  5,  6,
    7,  39, 40, 41, 42, 37,   // D11: GPIO37 (pen bit)
    33, 47, 48                 // D12: GPIO33
};

// HP1345A Word Format (from PHK's reverse-engineering):
// -------------------------------------------------------
// Bits 14:13 determine command type:
//   00 = Plot mode (vectors)
//   01 = Graph mode
//   10 = Text mode
//   11 = Set Condition
//
// For Plot mode (bits 14:13 = 00):
//   Bit 12: 0 = X coordinate (stores value, no action)
//           1 = Y coordinate (triggers move/draw using stored X and this Y)
//   Bit 11: 0 = pen up (move), 1 = pen down (draw) — only meaningful on Y word
//   Bits 10:0 = 11-bit coordinate (0–2047)
//
// Drawing rule: Send X word first (stores X), then Y word (executes with pen state).
// The pen bit on X is ignored; only the pen bit on the Y word matters.

static inline uint16_t coordMax() { return 2047u; }  // 11-bit coordinates

// Status counters
static uint32_t g_okCount = 0;
static uint32_t g_timeoutCount = 0;
static uint32_t g_lastTimeoutReportMs = 0;
static std::atomic<uint32_t> g_lastOkMs{0};
static std::atomic<bool> g_connected{false};
static std::thread g_screenThread;
static std::atomic<bool> g_runTransfers{true};
static std::atomic<bool> g_abortWaits{false};
// RFD polarity: active-low (confirmed working)
static bool g_rfdActiveLow = true;
// Pen sense: false = bit11=1 for draw (PHK), true = bit11=0 for draw (inverted)
static std::atomic<bool> g_penInvert{false};
static std::atomic<bool> g_verbose{false};
static std::atomic<bool> g_traceHandshake{false};
static std::atomic<bool> g_echoRx{false};
static std::atomic<uint8_t> g_testIndex{0};
static uint32_t g_fpsFrames = 0;
static uint32_t g_fpsLastMs = 0;
static char g_fpsText[16] = "FPS:0";
static uint32_t g_frameVectorCount = 0;
static uint32_t g_lastVectorCount = 0;
static char g_statsText[24] = "FPS:0 V:0";

static void printHelp()
{
    Serial.println("Commands:");
    Serial.println("  h  help");
    Serial.println("  s  status");
    Serial.println("  d  pulse DAV low (wiring check)");
    Serial.println("  w  toggle D11 (pen bit) for wiring check");
    Serial.println("  p  toggle pen up/down sense");
    Serial.println("  t  toggle transfers on/off");
    Serial.println("  v  toggle verbose logging");
    Serial.println("  b  toggle handshake trace (per-word)");
    Serial.println("  x  toggle RX echo");
    Serial.println("  0..8  select test pattern");
    Serial.println("  n  next test pattern");
}

// Forward declarations for text helpers (defined later).
static void moveTo(uint16_t x, uint16_t y);

static void reportInputs()
{
    Serial.printf("inputs: RFD=%d\n", digitalRead(PIN_RFD));
    if (PIN_XACK >= 0) {
        Serial.printf("inputs: XACK=%d\n", digitalRead(PIN_XACK));
    }
}
static void printBusStatus()
{
    Serial.printf("DAV(GPIO%d)=%d  RFD(GPIO%d)=%d  coordMax=%u  test=%u  verbose=%u\n",
                  PIN_DAV,
                  digitalRead(PIN_DAV),
                  PIN_RFD,
                  digitalRead(PIN_RFD),
                  (unsigned)coordMax(),
                  (unsigned)g_testIndex.load(std::memory_order_relaxed),
                  (unsigned)g_verbose.load(std::memory_order_relaxed));

    Serial.printf("trace: handshake=%u\n", (unsigned)g_traceHandshake.load(std::memory_order_relaxed));
}

static void pulseDavLowMs(uint32_t holdMs)
{
    // Force DAV output low briefly for wiring verification.
    // Use a DMM/scope on HP connector pin 10: it should drop near 0V during the pulse.
    digitalWrite(PIN_DAV, HIGH);
    delay(5);
    digitalWrite(PIN_DAV, LOW);
    delay(holdMs);
    digitalWrite(PIN_DAV, HIGH);
}

static void toggleD11ForWiringCheck()
{
    // Toggle D11 (pen bit) slowly so user can probe GPIO26 and HP pin D11
    // D11 = DATA_PINS[11] = GPIO26
    const int pin = DATA_PINS[11];
    Serial.printf("Toggling D11 (pen bit) on GPIO%d - probe this pin and HP D11\n", pin);
    Serial.println("Press any key to stop...");
    while (!Serial.available()) {
        digitalWrite(pin, HIGH);
        Serial.print("D11=HIGH ");
        delay(500);
        digitalWrite(pin, LOW);
        Serial.print("D11=LOW ");
        delay(500);
    }
    while (Serial.available()) Serial.read(); // flush
    Serial.println("\nD11 toggle stopped.");
}

static void serviceSerialCommands(bool allowDestructive = true)
{
    // Keep command handling lightweight so we can call it from tight loops.
    // If allowDestructive is false, we only honor safe toggles.
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c == '\r' || c == '\n') continue;

        if (g_echoRx.load(std::memory_order_relaxed)) {
            Serial.printf("[RX '%c' 0x%02X]\n", (c >= 32 && c <= 126) ? (char)c : '?', (unsigned)c);
        }

        if (c == 'h' || c == 'H' || c == '?') {
            printHelp();
            continue;
        }
        if (c == 's' || c == 'S') {
            printBusStatus();
        } else if (c == 'v' || c == 'V') {
            const bool nv = !g_verbose.load(std::memory_order_relaxed);
            g_verbose.store(nv, std::memory_order_relaxed);
            Serial.printf("Verbose %s\n", nv ? "ON" : "OFF");
        } else if (c == 'b' || c == 'B') {
            const bool nv = !g_traceHandshake.load(std::memory_order_relaxed);
            g_traceHandshake.store(nv, std::memory_order_relaxed);
            Serial.printf("Handshake trace %s\n", nv ? "ON" : "OFF");
        } else if (c == 'x' || c == 'X') {
            const bool nv = !g_echoRx.load(std::memory_order_relaxed);
            g_echoRx.store(nv, std::memory_order_relaxed);
            Serial.printf("RX echo %s\n", nv ? "ON" : "OFF");
        } else if (c == 'n' || c == 'N') {
            const uint8_t next = (uint8_t)((g_testIndex.load(std::memory_order_relaxed) + 1) % 9);
            g_testIndex.store(next, std::memory_order_relaxed);
            Serial.printf("Test pattern = %u\n", (unsigned)next);
        } else if (c >= '0' && c <= '8') {
            const uint8_t idx = (uint8_t)(c - '0');
            g_testIndex.store(idx, std::memory_order_relaxed);
            Serial.printf("Test pattern = %u\n", (unsigned)idx);
        } else if (c == 'p' || c == 'P') {
            // Toggle pen sense at runtime.
            const bool nv = !g_penInvert.load(std::memory_order_relaxed);
            g_penInvert.store(nv, std::memory_order_relaxed);
            Serial.printf("Pen sense: %s (bit11=1 means %s)\n",
                          nv ? "INVERTED" : "NORMAL",
                          nv ? "pen UP" : "pen DOWN");
        } else if (c == 't' || c == 'T') {
            const bool newVal = !g_runTransfers.load(std::memory_order_relaxed);
            g_runTransfers.store(newVal, std::memory_order_relaxed);
            if (!newVal) {
                // Stop any in-flight waits quickly and release DAV.
                g_abortWaits.store(true, std::memory_order_relaxed);
                digitalWrite(PIN_DAV, HIGH);
            } else {
                g_abortWaits.store(false, std::memory_order_relaxed);
            }
            Serial.printf("Transfers %s (abort=%d)\n",
                          newVal ? "ENABLED" : "PAUSED",
                          (int)g_abortWaits.load(std::memory_order_relaxed));
        } else if (!allowDestructive) {
            // Ignore everything else while inside handshake waits.
        } else if (c == 'd' || c == 'D') {
            Serial.println("Pulsing DAV low for 500ms...");
            pulseDavLowMs(500);
            printBusStatus();
        } else if (c == 'w' || c == 'W') {
            toggleD11ForWiringCheck();
        } else {
            Serial.printf("Unknown command '%c' (0x%02X). Send 'h' for help.\n", (char)c, (unsigned)c);
        }
    }
}

// Timeouts (microseconds)
static const uint32_t TIMEOUT_RFD_LOW_US  = 500000; // 0.5s
static const uint32_t TIMEOUT_RFD_HIGH_US = 500000; // 0.5s

static inline int rfd_raw() { return digitalRead(PIN_RFD); }
static inline bool rfd_asserted()
{
    // "Asserted" means "ready".
    return g_rfdActiveLow ? (rfd_raw() == 0) : (rfd_raw() != 0);
}
static inline bool rfd_deasserted() { return !rfd_asserted(); }

static void writeDataBus15(uint16_t word)
{
    word &= 0x7FFF; // only D0..D14

    // Bit-bang 15 GPIOs.
    // This is intentionally straightforward for first bring-up.
    for (int bit = 0; bit < 15; ++bit) {
        const int pin = DATA_PINS[bit];
        const int level = (word >> bit) & 1;
        digitalWrite(pin, level);
    }
}

static bool waitForRfdAsserted(uint32_t timeout_us)
{
    const uint32_t start = micros();
    uint32_t lastPrint = 0;
    while (!rfd_asserted()) {
        serviceSerialCommands(false);
        if (g_abortWaits.load(std::memory_order_relaxed)) {
            if (g_verbose.load(std::memory_order_relaxed)) {
                Serial.println("Wait aborted (transfers paused).");
            }
            return false;
        }
        if ((micros() - start) > timeout_us) {
            Serial.printf("Timeout waiting for RFD asserted, raw RFD=%d (active-%s)\n",
                          rfd_raw(),
                          g_rfdActiveLow ? "LOW" : "HIGH");
            return false;
        }
        if (g_verbose.load(std::memory_order_relaxed)) {
            uint32_t now = micros();
            if (now - lastPrint > 250000) { // every 250ms
                Serial.printf("Waiting for RFD asserted, raw RFD=%d (active-%s), elapsed=%lu us\n",
                              rfd_raw(),
                              g_rfdActiveLow ? "LOW" : "HIGH",
                              now - start);
                lastPrint = now;
            }
        }
        delayMicroseconds(10);
    }
    return true;
}

static bool waitForRfdDeasserted(uint32_t timeout_us)
{
    const uint32_t start = micros();
    uint32_t lastPrint = 0;
    while (!rfd_deasserted()) {
        serviceSerialCommands(false);
        if (g_abortWaits.load(std::memory_order_relaxed)) {
            if (g_verbose.load(std::memory_order_relaxed)) {
                Serial.println("Wait aborted (transfers paused).");
            }
            return false;
        }
        if ((micros() - start) > timeout_us) {
            Serial.printf("Timeout waiting for RFD deasserted, raw RFD=%d (active-%s)\n",
                          rfd_raw(),
                          g_rfdActiveLow ? "LOW" : "HIGH");
            return false;
        }
        if (g_verbose.load(std::memory_order_relaxed)) {
            uint32_t now = micros();
            if (now - lastPrint > 250000) { // every 250ms
                Serial.printf("Waiting for RFD deasserted, raw RFD=%d (active-%s), elapsed=%lu us\n",
                              rfd_raw(),
                              g_rfdActiveLow ? "LOW" : "HIGH",
                              now - start);
                lastPrint = now;
            }
        }
        delayMicroseconds(10);
    }
    return true;
}

static bool sendOneWordImmediate(uint16_t word)
{
    if (!g_runTransfers.load(std::memory_order_relaxed)) return false;

    const bool trace = g_traceHandshake.load(std::memory_order_relaxed);
    
    // Wait until display indicates ready-for-data (RFD asserted)
    if (trace) Serial.printf("Waiting for RFD asserted...\n");
    if (!waitForRfdAsserted(TIMEOUT_RFD_LOW_US)) return false;

    // Put data on bus BEFORE asserting DAV
    if (trace) Serial.printf("RFD asserted, sending word 0x%04X\n", word);
    writeDataBus15(word);

    // Assert DAV low (data valid)
    if (trace) Serial.printf("Asserting DAV low...\n");
    digitalWrite(PIN_DAV, LOW);

    // Wait for display to accept it (RFD deasserts)
    if (trace) Serial.printf("Waiting for RFD deasserted...\n");
    if (!waitForRfdDeasserted(TIMEOUT_RFD_HIGH_US)) {
        if (trace) Serial.printf("RFD timeout, releasing DAV...\n");
        digitalWrite(PIN_DAV, HIGH);
        return false;
    }

    // Release DAV high to complete the transfer
    if (trace) Serial.printf("RFD deasserted, releasing DAV...\n");
    digitalWrite(PIN_DAV, HIGH);

    return true;
}

static bool sendWord(uint16_t word)
{
    const bool ok = sendOneWordImmediate(word);
    if (ok) {
        g_okCount++;
        g_lastOkMs.store(millis(), std::memory_order_relaxed);
        g_connected.store(true, std::memory_order_relaxed);
    } else {
        g_timeoutCount++;
        const uint32_t lastOk = g_lastOkMs.load(std::memory_order_relaxed);
        if (lastOk == 0 || (millis() - lastOk) > 1000) {
            g_connected.store(false, std::memory_order_relaxed);
        }
        const uint32_t now = millis();
        if (now - g_lastTimeoutReportMs > 1000) {
            g_lastTimeoutReportMs = now;
            Serial.println("TIMEOUT waiting for RFD transition. Check DISCONNECT SENSE=GND, RFD wiring, and divider.");
        }
    }
    return ok;
}

static void ScreenThread()
{
    while (true) {
        const bool connected = g_connected.load(std::memory_order_relaxed);
        g_screen.Clear();
        g_screen.DrawBorder();
        g_screen.DrawStatus(connected);
        g_screen.Render();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// HP1345A Plot-mode word builders.
// X word: bits 14:12 = 000, bit 11 = pen (for upcoming Y), bits 10:0 = x coordinate
// Y word: bit 14:13 = 00, bit 12 = 1, bit 11 = pen, bits 10:0 = y coordinate
// HP1345A Plot-mode word builders (bits 14:13 = 00)
// This mode drew the starburst correctly
// g_penInvert flips pen sense for testing

static inline uint16_t makeXWord(uint16_t x, bool penDown)
{
    // Plot mode, X coordinate (bit12=0), pen bit on bit11, 11-bit coord
    const bool pen = g_penInvert.load(std::memory_order_relaxed) ? !penDown : penDown;
    const uint16_t penBit = pen ? 0x0800u : 0x0000u;
    return penBit | (x & 0x07FFu);
}

static inline uint16_t makeYWord(uint16_t y, bool penDown)
{
    // Plot mode, Y coordinate (bit12=1), pen bit (bit11), 11-bit coord
    const bool pen = g_penInvert.load(std::memory_order_relaxed) ? !penDown : penDown;
    const uint16_t penBit = pen ? 0x0800u : 0x0000u;
    return 0x1000u | penBit | (y & 0x07FFu);
}

// HP1345A Text command word (bits 14:13 = 10 = 0x4000)
// B12..B11 = S1..S0 (size), B10..B9 = R1..R0 (rotation),
// B8 = ES (establish size/rotation), B7..B0 = character code
static inline uint16_t makeTextWord(uint8_t ch, bool setSize, uint8_t size, uint8_t rot)
{
    const uint16_t s1 = (size >> 1) & 0x1;
    const uint16_t s0 = size & 0x1;
    const uint16_t r1 = (rot >> 1) & 0x1;
    const uint16_t r0 = rot & 0x1;
    return 0x4000u |
           (s1 << 12) |
           (s0 << 11) |
           (r1 << 10) |
           (r0 << 9)  |
           (setSize ? 0x0100u : 0x0000u) |
           (uint16_t)ch;
}

static void drawTextAt(uint16_t x, uint16_t y, const char *text)
{
    if (!text || !*text) return;
    // 1X size, 0° rotation
    const uint8_t size = 0;
    const uint8_t rot = 0;
    moveTo(x, y);
    bool first = true;
    for (const char *p = text; *p; ++p) {
        const uint8_t ch = (uint8_t)(*p);
        sendWord(makeTextWord(ch, first, size, rot));
        first = false;
    }
}

static void updateFpsText()
{
    const uint32_t now = millis();
    g_fpsFrames++;
    if (g_fpsLastMs == 0) {
        g_fpsLastMs = now;
        return;
    }
    const uint32_t dt = now - g_fpsLastMs;
    if (dt >= 1000) {
        const float fps = (g_fpsFrames * 1000.0f) / (float)dt;
        const unsigned fpsInt = (unsigned)lroundf(fps);
        snprintf(g_fpsText, sizeof(g_fpsText), "FPS:%u", fpsInt);
        snprintf(g_statsText, sizeof(g_statsText), "FPS:%u V:%lu", fpsInt, (unsigned long)g_lastVectorCount);
        g_fpsFrames = 0;
        g_fpsLastMs = now;
    }
}

// HP1345A Set Condition word (bits 14:13 = 11 = 0x6000)
// Used to initialize display state (intensity, focus, etc.)
// The test page sets these; we must replicate to get bright vectors.
// Based on HP docs: bits 12:10 select parameter, bits 9:0 are value
//   000 = intensity (0-1023, higher = brighter)
//   001 = focus  
//   010 = X offset
//   011 = Y offset
//   100 = writing rate (vector speed)
static void initHP1345A()
{
    Serial.println("Initializing HP1345A display state...");
    
    // Try multiple intensity-related commands to maximize brightness
    // Set Condition format: 0x6000 | (param << 10) | value
    
    // Intensity max (param 000, value 0x3FF)
    sendWord(0x63FFu);
    
    // Also try setting all possible intensity values
    // Maybe there's an additional brightness/blanking control
    sendWord(0x6FFFu);  // Try with more bits set
    
    // Set focus to sharp (try higher value)
    sendWord(0x67FFu);  // Focus param 001, max value
    
    // Set writing rate - low for solid lines
    sendWord(0x7000u);
    
    // Move to origin to establish known position
    sendWord(makeXWord(0, false));
    sendWord(makeYWord(0, false));
    
    Serial.println("HP1345A initialized.");
}

// Send X then Y. The Y word triggers the actual move or draw.
// Track current position so lineTo can send both endpoints
static uint16_t g_curX = 0;
static uint16_t g_curY = 0;

static void moveTo(uint16_t x, uint16_t y)
{
    sendWord(makeXWord(x, false));
    sendWord(makeYWord(y, false));  // pen up = move
    g_curX = x;
    g_curY = y;
}

static void lineTo(uint16_t x, uint16_t y)
{
    // Send start point with pen up, then end point with pen down
    sendWord(makeXWord(g_curX, false));
    sendWord(makeYWord(g_curY, false));  // establish start point
    sendWord(makeXWord(x, true));
    sendWord(makeYWord(y, true));        // draw to end point
    g_frameVectorCount++;
    g_curX = x;
    g_curY = y;
}

static inline uint16_t clampCoord(int32_t v)
{
    if (v < 0) return 0;
    const uint16_t maxv = coordMax();
    if (v > (int32_t)maxv) return maxv;
    return (uint16_t)v;
}

static inline uint16_t coordFromNorm(float n, uint16_t center, uint16_t radius)
{
    const float v = (float)center + (n * (float)radius);
    return clampCoord((int32_t)lroundf(v));
}

static void TestCase0()
{
    // Square with an X
    const uint16_t maxc = coordMax();
    const uint16_t m = 80;
    const uint16_t minv = m;
    const uint16_t maxv = maxc - m;

    moveTo(minv, minv);
    lineTo(maxv, minv);
    lineTo(maxv, maxv);
    lineTo(minv, maxv);
    lineTo(minv, minv);

    moveTo(minv, minv);
    lineTo(maxv, maxv);
    moveTo(minv, maxv);
    lineTo(maxv, minv);
}

static void TestCase1()
{
    // Concentric squares
    const uint16_t maxc = coordMax();
    const uint16_t steps = 6;
    const uint16_t margin = 60;
    const uint16_t span = maxc - (margin * 2);
    const uint16_t step = span / (steps * 2);

    for (uint16_t i = 0; i < steps; ++i) {
        const uint16_t minv = margin + (i * step);
        const uint16_t maxv = maxc - margin - (i * step);
        moveTo(minv, minv);
        lineTo(maxv, minv);
        lineTo(maxv, maxv);
        lineTo(minv, maxv);
        lineTo(minv, minv);
    }
}

static void TestCase2()
{
    // Starburst
    const uint16_t maxc = coordMax();
    const uint16_t cx = maxc / 2;
    const uint16_t cy = maxc / 2;
    const uint16_t r = (maxc / 2) - 40;
    const uint16_t rays = 36;
    const float twoPi = 6.283185307f;

    for (uint16_t i = 0; i < rays; ++i) {
        const float a = twoPi * ((float)i / (float)rays);
        const uint16_t x = coordFromNorm(cosf(a), cx, r);
        const uint16_t y = coordFromNorm(sinf(a), cy, r);
        moveTo(cx, cy);
        lineTo(x, y);
    }
}

static void TestCase3()
{
    // Rectangular spiral
    const uint16_t maxc = coordMax();
    uint16_t left = 60;
    uint16_t right = maxc - 60;
    uint16_t top = 60;
    uint16_t bottom = maxc - 60;
    const uint16_t step = 35;

    moveTo(left, top);
    while (left + step < right && top + step < bottom) {
        lineTo(right, top);
        lineTo(right, bottom);
        lineTo(left, bottom);
        left += step;
        top += step;
        right -= step;
        bottom -= step;
        lineTo(left, top);
    }
}

static void TestCase4()
{
    // Circle + crosshairs
    const uint16_t maxc = coordMax();
    const uint16_t cx = maxc / 2;
    const uint16_t cy = maxc / 2;
    const uint16_t r = (maxc / 2) - 40;
    const uint16_t segments = 80;
    const float twoPi = 6.283185307f;

    const uint16_t x0 = coordFromNorm(1.0f, cx, r);
    const uint16_t y0 = coordFromNorm(0.0f, cy, r);
    moveTo(x0, y0);
    for (uint16_t i = 1; i <= segments; ++i) {
        const float a = twoPi * ((float)i / (float)segments);
        const uint16_t x = coordFromNorm(cosf(a), cx, r);
        const uint16_t y = coordFromNorm(sinf(a), cy, r);
        lineTo(x, y);
    }

    moveTo(cx - r, cy);
    lineTo(cx + r, cy);
    moveTo(cx, cy - r);
    lineTo(cx, cy + r);
}

static void TestCase5()
{
    // Ripple surface (static mesh for max speed)
    const uint16_t maxc = coordMax();
    const float width = (float)maxc;
    const float height = (float)maxc;

    const int GRID_RADIUS = 24;
    const int GRID_STEP = 2;
    const float BASE_TILT_X = 0.85993f; // ~55 deg
    const float RIPPLE_FREQ = 0.65f;
    const float RIPPLE_SPEED = 4.0f;
    const float RIPPLE_DECAY = 0.015f;
    const float HEIGHT_SCALE = 20.0f;
    const float AMPLITUDE_FALLOFF = 0.06f;
    const float FOV = 1200.0f;
    const float VIEWER_DISTANCE = 40.0f;

    struct GridPoint { float x; float y; };
    struct ProjPoint { float x; float y; float depth; bool valid; };
    struct Quad { float depth; int i; int j; };
    struct Segment { uint16_t x0; uint16_t y0; uint16_t x1; uint16_t y1; };
    struct WordPair { uint16_t xWord; uint16_t yWord; };

    static bool cached = false;
    static int gridSize = 0;
    static std::vector<GridPoint> grid;
    static std::vector<ProjPoint> proj;
    static std::vector<Quad> quads;
    static std::vector<Segment> segments;
    static std::vector<WordPair> words;
    static uint32_t cachedVectorCount = 0;

    if (!cached) {
        gridSize = (GRID_RADIUS * 2) / GRID_STEP + 1;
        grid.resize(gridSize * gridSize);
        proj.resize(gridSize * gridSize);
        quads.reserve((gridSize - 1) * (gridSize - 1));

        int idx = 0;
        for (int xi = -GRID_RADIUS; xi <= GRID_RADIUS; xi += GRID_STEP) {
            for (int yi = -GRID_RADIUS; yi <= GRID_RADIUS; yi += GRID_STEP) {
                grid[idx++] = { (float)xi, (float)yi };
            }
        }

        const float t = 0.0f;
        const float cosx = cosf(BASE_TILT_X);
        const float sinx = sinf(BASE_TILT_X);
        const float cosz = 1.0f;
        const float sinz = 0.0f;

        auto rippleHeight = [&](float x, float y) -> float {
            const float r = sqrtf(x * x + y * y);
            const float wave = cosf(r * RIPPLE_FREQ - t * RIPPLE_SPEED);
            const float envelope = expf(-r * RIPPLE_DECAY);
            const float amplitude = HEIGHT_SCALE / (1.0f + AMPLITUDE_FALLOFF * r);
            return wave * envelope * amplitude;
        };

        // Project points
        for (int i = 0; i < gridSize * gridSize; ++i) {
            const float x = grid[i].x;
            const float y = grid[i].y;
            const float z = rippleHeight(x, y);

            const float xSpin = x * cosz - y * sinz;
            const float ySpin = x * sinz + y * cosz;
            const float zSpin = z;

            const float yTilt = ySpin * cosx - zSpin * sinx;
            const float zTilt = ySpin * sinx + zSpin * cosx;
            const float depth = VIEWER_DISTANCE + zTilt;
            if (depth <= 0.1f) {
                proj[i] = {0, 0, 0, false};
                continue;
            }
            const float factor = FOV / depth;
            const float px = xSpin * factor + (width / 2.0f);
            const float py = -yTilt * factor + (height / 2.0f);
            proj[i] = {px, py, depth, true};
        }

        // Build quads (painter's order, far to near)
        quads.clear();
        for (int i = 0; i < gridSize - 1; ++i) {
            for (int j = 0; j < gridSize - 1; ++j) {
                const int idx00 = i * gridSize + j;
                const int idx10 = (i + 1) * gridSize + j;
                const int idx11 = (i + 1) * gridSize + (j + 1);
                const int idx01 = i * gridSize + (j + 1);
                if (!proj[idx00].valid || !proj[idx10].valid || !proj[idx11].valid || !proj[idx01].valid) {
                    continue;
                }
                const float avgDepth = (proj[idx00].depth + proj[idx10].depth + proj[idx11].depth + proj[idx01].depth) * 0.25f;
                quads.push_back({avgDepth, i, j});
            }
        }
        std::sort(quads.begin(), quads.end(), [](const Quad &a, const Quad &b) { return a.depth > b.depth; });

        segments.clear();
        segments.reserve(quads.size() * 4);
        for (const Quad &q : quads) {
            const int i = q.i;
            const int j = q.j;
            const ProjPoint &p00 = proj[i * gridSize + j];
            const ProjPoint &p10 = proj[(i + 1) * gridSize + j];
            const ProjPoint &p11 = proj[(i + 1) * gridSize + (j + 1)];
            const ProjPoint &p01 = proj[i * gridSize + (j + 1)];

            auto toSeg = [&](const ProjPoint &a, const ProjPoint &b) {
                const uint16_t x0 = clampCoord((int32_t)lroundf(a.x));
                const uint16_t y0 = clampCoord((int32_t)lroundf(a.y));
                const uint16_t x1 = clampCoord((int32_t)lroundf(b.x));
                const uint16_t y1 = clampCoord((int32_t)lroundf(b.y));
                segments.push_back({x0, y0, x1, y1});
            };

            toSeg(p00, p10);
            toSeg(p10, p11);
            toSeg(p11, p01);
            toSeg(p01, p00);
        }

        words.clear();
        words.reserve(segments.size() * 2);
        for (const Segment &s : segments) {
            words.push_back({makeXWord(s.x0, false), makeYWord(s.y0, false)});
            words.push_back({makeXWord(s.x1, true), makeYWord(s.y1, true)});
        }

        cachedVectorCount = (uint32_t)segments.size();
        cached = true;
    }

    for (const WordPair &p : words) {
        sendWord(p.xWord);
        sendWord(p.yWord);
    }
    g_frameVectorCount += cachedVectorCount;
}

static void TestCase6()
{
    // Minimal axis sanity test: one horizontal line and one vertical line.
    const uint16_t maxc = coordMax();
    const uint16_t cx = maxc / 2;
    const uint16_t cy = maxc / 2;
    const uint16_t m = 80;

    // Horizontal line across the middle
    moveTo(m, cy);
    lineTo(maxc - m, cy);

    // Vertical line across the middle
    moveTo(cx, m);
    lineTo(cx, maxc - m);
}

static void TestCase7()
{
    // Sombrero plot: 3D wireframe of sinc(r) = sin(r)/r
    // Classic "off at an angle" perspective view
    const uint16_t maxc = coordMax();
    const uint16_t cx = maxc / 2;
    const uint16_t cy = maxc / 2;
    const float baseScale = maxc * 0.28f;    // Narrower overall
    const float heightScale = maxc * 0.18f;  // Taller peak
    
    const int gridSize = 12;        // Points per radial line
    const int numRadials = 12;      // Number of radial spokes
    const float maxR = 3.0f * 3.14159f;  // ~3 periods
    
    // Viewing angle parameters - classic perspective
    const float elevAngle = 0.45f;    // ~25 deg elevation (tilt toward viewer)
    const float rotAngle = 0.6f;      // ~35 deg rotation around Z
    const float cosElev = cosf(elevAngle);
    const float sinElev = sinf(elevAngle);
    const float cosRot = cosf(rotAngle);
    const float sinRot = sinf(rotAngle);
    
    // Draw radial lines (spokes)
    for (int spoke = 0; spoke < numRadials; ++spoke) {
        const float angle = (2.0f * 3.14159f * spoke) / numRadials;
        const float cosA = cosf(angle);
        const float sinA = sinf(angle);
        
        bool first = true;
        for (int i = 0; i <= gridSize; ++i) {
            const float r = (maxR * i) / gridSize;
            
            // sinc(r) = sin(r)/r, with sinc(0) = 1
            float z;
            if (r < 0.001f) {
                z = 1.0f;
            } else {
                z = sinf(r) / r;
            }
            
            // 3D coordinates
            const float x3d = r * cosA;
            const float y3d = r * sinA;
            const float z3d = z;
            
            // Rotate around Z axis, then tilt (elevation)
            const float xRot = x3d * cosRot - y3d * sinRot;
            const float yRot = x3d * sinRot + y3d * cosRot;
            const float xProj = xRot;
            const float yProj = yRot * cosElev + z3d * sinElev;
            
            // Map to screen coordinates (x→horizontal, y→vertical)
            const uint16_t sx = clampCoord((int32_t)(cx + xProj * baseScale / maxR));
            const uint16_t sy = clampCoord((int32_t)(cy - yProj * heightScale));
            
            if (first) {
                moveTo(sx, sy);
                first = false;
            } else {
                lineTo(sx, sy);
            }
        }
    }
    
    // Draw concentric circles at various radii
    const int numCircles = 5;
    const int circlePoints = 24;
    for (int c = 1; c <= numCircles; ++c) {
        const float r = (maxR * c) / numCircles;
        
        // sinc(r) for this radius
        float z = sinf(r) / r;
        
        bool first = true;
        for (int i = 0; i <= circlePoints; ++i) {
            const float angle = (2.0f * 3.14159f * i) / circlePoints;
            const float x3d = r * cosf(angle);
            const float y3d = r * sinf(angle);
            const float z3d = z;
            
            // Same projection as spokes
            const float xRot = x3d * cosRot - y3d * sinRot;
            const float yRot = x3d * sinRot + y3d * cosRot;
            const float xProj = xRot;
            const float yProj = yRot * cosElev + z3d * sinElev;
            
            const uint16_t sx = clampCoord((int32_t)(cx + xProj * baseScale / maxR));
            const uint16_t sy = clampCoord((int32_t)(cy - yProj * heightScale));
            
            if (first) {
                moveTo(sx, sy);
                first = false;
            } else {
                lineTo(sx, sy);
            }
        }
    }
}

static void TestCase8()
{
    // Checkerboard (outlined squares)
    const uint16_t maxc = coordMax();
    const uint16_t margin = 120;
    const int rows = 6;
    const int cols = 6;
    const uint16_t cellW = (maxc - (margin * 2)) / cols;
    const uint16_t cellH = (maxc - (margin * 2)) / rows;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (((r + c) & 1) == 0) {
                const uint16_t x0 = margin + (uint16_t)(c * cellW);
                const uint16_t y0 = margin + (uint16_t)(r * cellH);
                const uint16_t x1 = x0 + cellW;
                const uint16_t y1 = y0 + cellH;
                moveTo(x0, y0);
                lineTo(x1, y0);
                lineTo(x1, y1);
                lineTo(x0, y1);
                lineTo(x0, y0);
            }
        }
    }
}

static void RunTestCase(uint8_t idx)
{
    switch (idx) {
        case 0: TestCase0(); break;
        case 1: TestCase1(); break;
        case 2: TestCase2(); break;
        case 3: TestCase3(); break;
        case 4: TestCase4(); break;
        case 5: TestCase5(); break;
        case 6: TestCase6(); break;
        case 7: TestCase7(); break;
        case 8: TestCase8(); break;
        default: TestCase0(); break;
    }
}

static void configurePins()
{
    // Data bus D0..D14: Outputs low idle
    for (int i = 0; i < 15; ++i) {
        pinMode(DATA_PINS[i], OUTPUT);
        digitalWrite(DATA_PINS[i], LOW);
    }

    // DAV output, idle HIGH (inactive)
    pinMode(PIN_DAV, OUTPUT);
    digitalWrite(PIN_DAV, HIGH);

    // RFD input (direct from HP, ESP32 is 5V tolerant - remove level shifter)
    pinMode(PIN_RFD, INPUT);
   
    // Unused for now (but set safe if defined)
    if (PIN_DS >= 0) {
        pinMode(PIN_DS, INPUT);
    }
    if (PIN_RD >= 0) {
        pinMode(PIN_RD, INPUT);
    }
    if (PIN_XACK >= 0) {
        pinMode(PIN_XACK, INPUT);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(200);

    Serial.println();
    Serial.println("HP1345A Immediate-Mode Handshake Exerciser");
    Serial.println("Wiring reminders:");
    Serial.println(" - HP DISCONNECT SENSE pin must be tied to GND");
    Serial.println(" - Protect RFD input with resistor divider (5V -> 3.3V)");
    Serial.println(" - Tie grounds together (HP GND to ESP32 GND)");
    Serial.println();

    configurePins();

    Serial.println("Send 'd' to pulse DAV low for 500ms (wiring check on HP pin 10).");
    Serial.println("Send 's' to print bus status.");
    Serial.println("Send 'h' for help.");

    Serial.println("Initializing OLED...");
    if (g_screen.Init()) {
        g_screen.Clear();
        g_screen.DrawBorder();
        g_screen.Render();
        Serial.println("OLED ready.");
    } else {
        Serial.println("OLED init failed. Scanning common I2C pin pairs first (avoid blocking init)...");
        const int sdaPins[] = {17, 8, 21};
        const int sclPins[] = {18, 9, 22};
        bool found = false;
        for (size_t i = 0; i < (sizeof(sdaPins) / sizeof(sdaPins[0])); ++i) {
            const int sda = sdaPins[i];
            const int scl = sclPins[i];
            Serial.printf("Scan SDA=%d SCL=%d\n", sda, scl);
            // Try common OLED addresses first to avoid scan issues on some boards.
            const uint8_t tryAddrs[] = {0x3C, 0x3D};
            for (size_t a = 0; a < (sizeof(tryAddrs) / sizeof(tryAddrs[0])); ++a) {
                const uint8_t addr = tryAddrs[a];
                Serial.printf("Try OLED init at 0x%02X on SDA=%d SCL=%d\n", addr, sda, scl);
                if (g_screen.InitWithPins(sda, scl, addr)) {
                    g_screen.Clear();
                    g_screen.DrawBorder();
                    g_screen.Render();
                    Serial.println("OLED ready after direct init.");
                    found = true;
                    break;
                }
            }
            if (found) break;

            const uint8_t addr = g_screen.Scan(sda, scl);
            if (addr != 0) {
                Serial.printf("I2C device found at 0x%02X on SDA=%d SCL=%d\n", addr, sda, scl);
                Serial.println("Initializing OLED on detected bus...");
                if (g_screen.InitWithPins(sda, scl, addr)) {
                    g_screen.Clear();
                    g_screen.DrawBorder();
                    g_screen.Render();
                    Serial.println("OLED ready after scan.");
                    found = true;
                    break;
                }
                Serial.println("OLED init failed on detected bus.");
            } else {
                Serial.printf("No I2C device found on SDA=%d SCL=%d\n", sda, scl);
            }
        }
    if (!found) {
        Serial.println("No OLED detected. Set SCREEN_SDA/SCREEN_SCL/SCREEN_ADDR or confirm OLED controller.");
    }
    }

    // Start screen update thread after OLED init path completes.
    g_screenThread = std::thread(ScreenThread);
    g_screenThread.detach();

    Serial.printf("DAV=%d (OUT), RFD=%d (IN)\n", PIN_DAV, PIN_RFD);
    Serial.print("DATA pins D0..D14: ");
    for (int i = 0; i < 15; ++i) {
        Serial.printf("%d%s", DATA_PINS[i], (i == 14) ? "\n" : ",");
    }

    Serial.printf("Initial RFD level = %d (0=ready/LOW)\n", digitalRead(PIN_RFD));
    Serial.println("HP1345A Plot-mode word format: X word=0x0xxx, Y word=0x1yyy (pen=bit11)");
    
    // Initialize HP1345A display state (intensity, focus, etc.)
    // This replicates what the built-in test page does.
    initHP1345A();
    
    Serial.println("Starting transfers...\n");
}

void loop()
{
    // Always service commands first (and frequently during waits).
    serviceSerialCommands(true);

    if (g_runTransfers.load(std::memory_order_relaxed)) {
        g_frameVectorCount = 0;
        RunTestCase(g_testIndex.load(std::memory_order_relaxed));
        g_lastVectorCount = g_frameVectorCount;
        updateFpsText();
        // Top-left text: position accounts for 1X character height (~36)
        const uint16_t x = 12;n
        const uint16_t y = coordMax() - 40;
        drawTextAt(x, y, g_statsText);
    } else {
        delay(20);
        return;
    }
}
