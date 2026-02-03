#include "hp_vector_device.h"

#include <math.h>

namespace {
constexpr uint32_t kTimeoutRfdAssertUs = 500000;
constexpr uint32_t kTimeoutRfdDeassertUs = 500000;
}  // namespace

HPVectorDevice::HPVectorDevice(Pins pins) : pins_(pins) {}

// Configure the GPIO pins used by the HP1345A bus.
void HPVectorDevice::ConfigurePins()
{
    for (int i = 0; i < 15; ++i) {
        pinMode(pins_.data[i], OUTPUT);
        digitalWrite(pins_.data[i], LOW);
    }

    pinMode(pins_.dav, OUTPUT);
    digitalWrite(pins_.dav, HIGH);

    pinMode(pins_.rfd, INPUT);

    if (pins_.ds >= 0) pinMode(pins_.ds, INPUT);
    if (pins_.rd >= 0) pinMode(pins_.rd, INPUT);
    if (pins_.xack >= 0) pinMode(pins_.xack, INPUT);
}

// Send display setup commands (brightness, focus, writing rate).
void HPVectorDevice::InitializeDisplay()
{
    if (!Quiet()) Serial.println("Initializing HP1345A display state...");

    SendWord(0x63FFu);  // Intensity max (param 000, value 0x3FF)
    SendWord(0x6FFFu);  // Extra bits set (safety for undocumented behavior)
    SendWord(0x67FFu);  // Focus max (param 001)
    SendWord(0x7000u);  // Writing rate slow (solid lines)

    SendWord(MakeXWord(0, false));
    SendWord(MakeYWord(0, false));

    if (!Quiet()) Serial.println("HP1345A initialized.");
}

// Register a lightweight callback to service commands during waits.
void HPVectorDevice::SetServiceCallback(void (*service)())
{
    service_ = service;
}

// Enable or disable transfers; disabling also aborts waits.
void HPVectorDevice::SetTransfersEnabled(bool enabled)
{
    transfersEnabled_.store(enabled, std::memory_order_relaxed);
    abortWaits_.store(!enabled, std::memory_order_relaxed);
    if (!enabled) {
        digitalWrite(pins_.dav, HIGH);
    }
}

bool HPVectorDevice::TransfersEnabled() const
{
    return transfersEnabled_.load(std::memory_order_relaxed);
}

// Force any in-flight waits to exit quickly.
void HPVectorDevice::AbortWaits(bool abort)
{
    abortWaits_.store(abort, std::memory_order_relaxed);
}

void HPVectorDevice::SetVerbose(bool verbose)
{
    verbose_.store(verbose, std::memory_order_relaxed);
}

bool HPVectorDevice::Verbose() const
{
    return verbose_.load(std::memory_order_relaxed);
}

void HPVectorDevice::SetTraceHandshake(bool trace)
{
    traceHandshake_.store(trace, std::memory_order_relaxed);
}

bool HPVectorDevice::TraceHandshake() const
{
    return traceHandshake_.load(std::memory_order_relaxed);
}

void HPVectorDevice::SetPenInvert(bool invert)
{
    penInvert_.store(invert, std::memory_order_relaxed);
}

bool HPVectorDevice::PenInvert() const
{
    return penInvert_.load(std::memory_order_relaxed);
}

void HPVectorDevice::SetQuiet(bool quiet)
{
    quiet_.store(quiet, std::memory_order_relaxed);
}

bool HPVectorDevice::Quiet() const
{
    return quiet_.load(std::memory_order_relaxed);
}

void HPVectorDevice::SetRfdActiveLow(bool activeLow)
{
    rfdActiveLow_ = activeLow;
}

bool HPVectorDevice::RfdActiveLow() const
{
    return rfdActiveLow_;
}

bool HPVectorDevice::IsConnected() const
{
    return connected_.load(std::memory_order_relaxed);
}

int HPVectorDevice::RfdRaw() const
{
    return digitalRead(pins_.rfd);
}

int HPVectorDevice::DavRaw() const
{
    return digitalRead(pins_.dav);
}

// Print current input states.
void HPVectorDevice::ReportInputs(Stream& out) const
{
    out.printf("inputs: RFD=%d\n", RfdRaw());
    if (pins_.xack >= 0) {
        out.printf("inputs: XACK=%d\n", digitalRead(pins_.xack));
    }
}

// Print bus status summary (for debugging).
void HPVectorDevice::PrintBusStatus(Stream& out, uint8_t testIndex) const
{
    out.printf("DAV(GPIO%d)=%d  RFD(GPIO%d)=%d  coordMax=%u  test=%u  verbose=%u\n",
               pins_.dav,
               DavRaw(),
               pins_.rfd,
               RfdRaw(),
               (unsigned)CoordMax(),
               (unsigned)testIndex,
               (unsigned)Verbose());
    out.printf("trace: handshake=%u\n", (unsigned)TraceHandshake());
}

// Coordinate maximum for 11-bit plotting.
uint16_t HPVectorDevice::CoordMax() const
{
    return 2047u;
}

// Clamp integer coordinates to the display range.
uint16_t HPVectorDevice::ClampCoord(int32_t v) const
{
    if (v < 0) return 0;
    const uint16_t maxv = CoordMax();
    if (v > (int32_t)maxv) return maxv;
    return (uint16_t)v;
}

// Convert normalized [-1,1] to screen coordinates.
uint16_t HPVectorDevice::CoordFromNorm(float n, uint16_t center, uint16_t radius) const
{
    const float v = (float)center + (n * (float)radius);
    return ClampCoord((int32_t)lroundf(v));
}

// Build an X plot word (bit12=0) with pen state.
uint16_t HPVectorDevice::MakeXWord(uint16_t x, bool penDown) const
{
    const bool pen = penInvert_.load(std::memory_order_relaxed) ? !penDown : penDown;
    const uint16_t penBit = pen ? 0x0800u : 0x0000u;
    return penBit | (x & 0x07FFu);
}

// Build a Y plot word (bit12=1) with pen state.
uint16_t HPVectorDevice::MakeYWord(uint16_t y, bool penDown) const
{
    const bool pen = penInvert_.load(std::memory_order_relaxed) ? !penDown : penDown;
    const uint16_t penBit = pen ? 0x0800u : 0x0000u;
    return 0x1000u | penBit | (y & 0x07FFu);
}

// Build a text word (bits 14:13 = 10).
uint16_t HPVectorDevice::MakeTextWord(uint8_t ch, bool setSize, uint8_t size, uint8_t rot) const
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

// Send a single word with handshake and bookkeeping.
bool HPVectorDevice::SendWord(uint16_t word)
{
    const bool ok = SendOneWordImmediate(word);
    if (ok) {
        okCount_++;
        lastOkMs_.store(millis(), std::memory_order_relaxed);
        connected_.store(true, std::memory_order_relaxed);
    } else {
        timeoutCount_++;
        const uint32_t lastOk = lastOkMs_.load(std::memory_order_relaxed);
        if (lastOk == 0 || (millis() - lastOk) > 1000) {
            connected_.store(false, std::memory_order_relaxed);
        }
        const uint32_t now = millis();
        if (!Quiet() && now - lastTimeoutReportMs_ > 1000) {
            lastTimeoutReportMs_ = now;
            Serial.println("TIMEOUT waiting for RFD transition. Check DISCONNECT SENSE=GND, RFD wiring, and divider.");
        }
    }
    return ok;
}

// Move the beam without drawing.
void HPVectorDevice::MoveTo(uint16_t x, uint16_t y)
{
    SendWord(MakeXWord(x, false));
    SendWord(MakeYWord(y, false));
    curX_ = x;
    curY_ = y;
}

// Draw a vector from the current point to (x,y).
void HPVectorDevice::LineTo(uint16_t x, uint16_t y)
{
    SendWord(MakeXWord(curX_, false));
    SendWord(MakeYWord(curY_, false));
    SendWord(MakeXWord(x, true));
    SendWord(MakeYWord(y, true));
    curX_ = x;
    curY_ = y;
    vectorCount_++;
}

uint32_t HPVectorDevice::VectorCount() const
{
    return vectorCount_;
}

void HPVectorDevice::ResetVectorCount()
{
    vectorCount_ = 0;
}

void HPVectorDevice::AddVectorCount(uint32_t count)
{
    vectorCount_ += count;
}

// Draw text with explicit size/rotation.
void HPVectorDevice::DrawTextAtSized(uint16_t x, uint16_t y, const char* text, uint8_t size, uint8_t rot)
{
    if (!text || !*text) return;
    MoveTo(x, y);
    bool first = true;
    for (const char* p = text; *p; ++p) {
        const uint8_t ch = (uint8_t)(*p);
        SendWord(MakeTextWord(ch, first, size, rot));
        first = false;
    }
}

// Draw 1X, 0° text at a position.
void HPVectorDevice::DrawTextAt(uint16_t x, uint16_t y, const char* text)
{
    DrawTextAtSized(x, y, text, 0, 0);
}

// Pulse DAV low for a wiring check.
void HPVectorDevice::PulseDavLowMs(uint32_t holdMs)
{
    digitalWrite(pins_.dav, HIGH);
    delay(5);
    digitalWrite(pins_.dav, LOW);
    delay(holdMs);
    digitalWrite(pins_.dav, HIGH);
}

bool HPVectorDevice::RfdAsserted() const
{
    const int raw = RfdRaw();
    return rfdActiveLow_ ? (raw == 0) : (raw != 0);
}

bool HPVectorDevice::WaitForRfdAsserted(uint32_t timeoutUs)
{
    const uint32_t start = micros();
    uint32_t lastPrint = 0;
    while (!RfdAsserted()) {
        if (service_) service_();
        if (abortWaits_.load(std::memory_order_relaxed)) {
            if (!Quiet() && Verbose()) Serial.println("Wait aborted (transfers paused).");
            return false;
        }
        if ((micros() - start) > timeoutUs) {
            if (!Quiet()) {
                Serial.printf("Timeout waiting for RFD asserted, raw RFD=%d (active-%s)\n",
                              RfdRaw(),
                              rfdActiveLow_ ? "LOW" : "HIGH");
            }
            return false;
        }
        if (!Quiet() && Verbose()) {
            const uint32_t now = micros();
            if (now - lastPrint > 250000) {
                Serial.printf("Waiting for RFD asserted, raw RFD=%d (active-%s), elapsed=%lu us\n",
                              RfdRaw(),
                              rfdActiveLow_ ? "LOW" : "HIGH",
                              now - start);
                lastPrint = now;
            }
        }
        delayMicroseconds(10);
    }
    return true;
}

bool HPVectorDevice::WaitForRfdDeasserted(uint32_t timeoutUs)
{
    const uint32_t start = micros();
    uint32_t lastPrint = 0;
    while (RfdAsserted()) {
        if (service_) service_();
        if (abortWaits_.load(std::memory_order_relaxed)) {
            if (!Quiet() && Verbose()) Serial.println("Wait aborted (transfers paused).");
            return false;
        }
        if ((micros() - start) > timeoutUs) {
            if (!Quiet()) {
                Serial.printf("Timeout waiting for RFD deasserted, raw RFD=%d (active-%s)\n",
                              RfdRaw(),
                              rfdActiveLow_ ? "LOW" : "HIGH");
            }
            return false;
        }
        if (!Quiet() && Verbose()) {
            const uint32_t now = micros();
            if (now - lastPrint > 250000) {
                Serial.printf("Waiting for RFD deasserted, raw RFD=%d (active-%s), elapsed=%lu us\n",
                              RfdRaw(),
                              rfdActiveLow_ ? "LOW" : "HIGH",
                              now - start);
                lastPrint = now;
            }
        }
        delayMicroseconds(10);
    }
    return true;
}

// Drive the 15-bit data bus.
void HPVectorDevice::WriteDataBus15(uint16_t word)
{
    word &= 0x7FFF;
    for (int bit = 0; bit < 15; ++bit) {
        const int pin = pins_.data[bit];
        const int level = (word >> bit) & 1;
        digitalWrite(pin, level);
    }
}

// Perform a full DAV/RFD handshake for one word.
bool HPVectorDevice::SendOneWordImmediate(uint16_t word)
{
    if (!TransfersEnabled()) return false;

    const bool trace = TraceHandshake() && !Quiet();
    if (trace) Serial.printf("Waiting for RFD asserted...\n");
    if (!WaitForRfdAsserted(kTimeoutRfdAssertUs)) return false;

    if (trace) Serial.printf("RFD asserted, sending word 0x%04X\n", word);
    WriteDataBus15(word);

    if (trace) Serial.printf("Asserting DAV low...\n");
    digitalWrite(pins_.dav, LOW);

    if (trace) Serial.printf("Waiting for RFD deasserted...\n");
    if (!WaitForRfdDeasserted(kTimeoutRfdDeassertUs)) {
        if (trace) Serial.printf("RFD timeout, releasing DAV...\n");
        digitalWrite(pins_.dav, HIGH);
        return false;
    }

    if (trace) Serial.printf("RFD deasserted, releasing DAV...\n");
    digitalWrite(pins_.dav, HIGH);
    return true;
}
