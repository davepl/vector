#pragma once

#include <stdint.h>

// Optional overrides (set via build_flags): SCREEN_SDA, SCREEN_SCL, SCREEN_ADDR, SCREEN_RST_PIN, SCREEN_VEXT_PIN
#ifndef SCREEN_ADDR
#define SCREEN_ADDR 0x3C
#endif
#ifndef SCREEN_SDA
#define SCREEN_SDA 17
#endif
#ifndef SCREEN_SCL
#define SCREEN_SCL 18
#endif
#ifndef SCREEN_RST_PIN
#define SCREEN_RST_PIN 21
#endif
#ifndef SCREEN_VEXT_PIN
#define SCREEN_VEXT_PIN 36
#endif

class Screen {
public:
    Screen(uint8_t sda, uint8_t scl, uint8_t addr, int8_t rstPin, int8_t vextPin);

    bool Init();
    bool InitWithPins(uint8_t sda, uint8_t scl, uint8_t addr);
    uint8_t Scan(uint8_t sda, uint8_t scl);
    void Clear();
    void DrawBorder();
    void DrawStatus(bool connected);
    void DrawLines(const char* line1, const char* line2, const char* line3);
    void Render();

private:
    void PowerOn();

    uint8_t m_sda;
    uint8_t m_scl;
    uint8_t m_addr;
    int8_t m_rstPin;
    int8_t m_vextPin;
};

extern Screen g_screen;
