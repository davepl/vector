#pragma once

#include <Arduino.h>
#include <array>
#include <atomic>
#include <cstdint>

// HP1345A immediate-mode vector interface.
class HPVectorDevice {
public:
    // GPIO pin mapping for the HP1345A bus.
    struct Pins {
        int dav;
        int rfd;
        int ds;
        int rd;
        int xack;
        std::array<int, 15> data;
    };

    // Construct the device with a fixed pin mapping.
    explicit HPVectorDevice(Pins pins);

    // Configure GPIO directions and idle levels for the HP bus.
    void ConfigurePins();
    // Initialize the display state (intensity, focus, writing rate).
    void InitializeDisplay();

    // Set a callback that can service serial commands during waits.
    void SetServiceCallback(void (*service)());

    // Control whether transfers are allowed to proceed.
    void SetTransfersEnabled(bool enabled);
    bool TransfersEnabled() const;
    // Abort any in-flight waits (used when transfers are paused).
    void AbortWaits(bool abort);

    // Debug/behavior toggles.
    void SetVerbose(bool verbose);
    bool Verbose() const;
    void SetTraceHandshake(bool trace);
    bool TraceHandshake() const;
    void SetPenInvert(bool invert);
    bool PenInvert() const;
    void SetRfdActiveLow(bool activeLow);
    bool RfdActiveLow() const;

    // Status queries.
    bool IsConnected() const;
    int RfdRaw() const;
    int DavRaw() const;
    void ReportInputs(Stream& out) const;
    void PrintBusStatus(Stream& out, uint8_t testIndex) const;

    // Coordinate helpers.
    uint16_t CoordMax() const;
    uint16_t ClampCoord(int32_t v) const;
    uint16_t CoordFromNorm(float n, uint16_t center, uint16_t radius) const;

    // Low-level word builders.
    // Word format summary:
    //   Bits 14:13: 00=Plot, 01=Graph, 10=Text, 11=Set Condition.
    //   Plot X: bit12=0, pen bit on bit11, bits10:0 = coord.
    //   Plot Y: bit12=1, pen bit on bit11, bits10:0 = coord.
    uint16_t MakeXWord(uint16_t x, bool penDown) const;
    uint16_t MakeYWord(uint16_t y, bool penDown) const;
    uint16_t MakeTextWord(uint8_t ch, bool setSize, uint8_t size, uint8_t rot) const;

    // Send a single 15-bit word with DAV/RFD handshake.
    bool SendWord(uint16_t word);

    // High-level drawing helpers.
    void MoveTo(uint16_t x, uint16_t y);
    void LineTo(uint16_t x, uint16_t y);
    void DrawTextAtSized(uint16_t x, uint16_t y, const char* text, uint8_t size, uint8_t rot);
    void DrawTextAt(uint16_t x, uint16_t y, const char* text);

    // Wiring diagnostics.
    void PulseDavLowMs(uint32_t holdMs);
    void ToggleD11ForWiringCheck(Stream& out);

private:
    bool RfdAsserted() const;
    bool WaitForRfdAsserted(uint32_t timeoutUs);
    bool WaitForRfdDeasserted(uint32_t timeoutUs);
    void WriteDataBus15(uint16_t word);
    bool SendOneWordImmediate(uint16_t word);

    Pins pins_;
    void (*service_)() = nullptr;

    std::atomic<bool> transfersEnabled_{true};
    std::atomic<bool> abortWaits_{false};
    std::atomic<bool> penInvert_{false};
    std::atomic<bool> verbose_{false};
    std::atomic<bool> traceHandshake_{false};
    bool rfdActiveLow_{true};

    uint32_t okCount_ = 0;
    uint32_t timeoutCount_ = 0;
    uint32_t lastTimeoutReportMs_ = 0;
    std::atomic<uint32_t> lastOkMs_{0};
    std::atomic<bool> connected_{false};

    uint16_t curX_ = 0;
    uint16_t curY_ = 0;
};
