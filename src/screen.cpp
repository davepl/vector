#include "screen.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <mutex>
#include <string.h>

static const uint8_t kScreenWidth = 128;
static const uint8_t kScreenHeight = 64;

static Adafruit_SSD1306 g_display(kScreenWidth, kScreenHeight, &Wire, -1);
static std::mutex g_displayMutex;

Screen g_screen(SCREEN_SDA, SCREEN_SCL, SCREEN_ADDR, SCREEN_RST_PIN, SCREEN_VEXT_PIN);

Screen::Screen(uint8_t sda, uint8_t scl, uint8_t addr, int8_t rstPin, int8_t vextPin)
    : m_sda(sda),
      m_scl(scl),
      m_addr(addr),
      m_rstPin(rstPin),
      m_vextPin(vextPin)
{
}

void Screen::PowerOn()
{
    if (m_vextPin >= 0) {
        pinMode(m_vextPin, OUTPUT);
        digitalWrite(m_vextPin, LOW); // Vext is active-low on Heltec boards
        delay(5);
    }
    if (m_rstPin >= 0) {
        pinMode(m_rstPin, OUTPUT);
        digitalWrite(m_rstPin, HIGH);
        delay(2);
        digitalWrite(m_rstPin, LOW);
        delay(10);
        digitalWrite(m_rstPin, HIGH);
        delay(2);
    }
}

bool Screen::Init()
{
    return InitWithPins(m_sda, m_scl, m_addr);
}

bool Screen::InitWithPins(uint8_t sda, uint8_t scl, uint8_t addr)
{
    PowerOn();
    Wire.begin(sda, scl);
    Wire.setTimeOut(50);
    Wire.setClock(400000);

    if (!g_display.begin(SSD1306_SWITCHCAPVCC, addr)) {
        return false;
    }

    g_display.clearDisplay();
    g_display.display();
    return true;
}

uint8_t Screen::Scan(uint8_t sda, uint8_t scl)
{
    PowerOn();
    Wire.begin(sda, scl);
    Wire.setTimeOut(50);
    Wire.setClock(400000);
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 0x7F; ++addr) {
        Wire.beginTransmission(addr);
        const uint8_t err = Wire.endTransmission();
        if (err == 0) {
            found = addr;
            break;
        }
        delay(1);
    }
    return found;
}

void Screen::Clear()
{
    std::lock_guard<std::mutex> lock(g_displayMutex);
    g_display.clearDisplay();
}

void Screen::DrawBorder()
{
    std::lock_guard<std::mutex> lock(g_displayMutex);
    g_display.drawRect(0, 0, kScreenWidth, kScreenHeight, SSD1306_WHITE);
}

void Screen::DrawStatus(bool connected)
{
    const char *text = connected ? "Connected: YES" : "Connected: NO";
    const uint8_t textSize = 1;
    const uint8_t charWidth = 6 * textSize;  // 5x7 font + 1px spacing
    const uint8_t charHeight = 8 * textSize;
    const uint8_t len = (uint8_t)strlen(text);
    const int16_t x = (kScreenWidth - (len * charWidth)) / 2;
    const int16_t y = 2;

    std::lock_guard<std::mutex> lock(g_displayMutex);
    g_display.setTextSize(textSize);
    g_display.setTextColor(SSD1306_WHITE);
    g_display.setCursor(x, y);
    g_display.print(text);
}

void Screen::Render()
{
    std::lock_guard<std::mutex> lock(g_displayMutex);
    g_display.display();
}
