# HP1345A Vector Display Driver (ESP32-S3)

Drive an HP 1345A vector display in **immediate mode** from a Heltec WiFi Kit 32 V3 (ESP32‑S3). This project bit‑bangs the 15‑bit data bus and handshake lines, renders a set of vector test patterns, and shows status on the onboard OLED.

## What this is for
- Bring up the HP1345A immediate‑mode interface (D0–D14 + DAV/RFD).
- Exercise the vector display with multiple geometric test patterns.
- Provide a small OLED status panel (connected/disconnected).

## Hardware you need
- HP 1345A vector display (immediate mode).
- Heltec WiFi Kit 32 V3 (ESP32‑S3).
- Resistor divider for RFD (5V -> 3.3V).
- Common ground between HP and ESP32.
- Optional: scope or DMM for wiring verification.

## Wiring
The HP1345A immediate‑mode bus uses **D0–D14** (15 bits). D15 and the Option‑704 lines (DS/RD/XACK/SYNC) are **not used**.

### Data bus (HP → ESP32)
| HP Signal | ESP32 GPIO |
|---|---|
| D0 | GPIO1 |
| D1 | GPIO2 |
| D2 | GPIO3 |
| D3 | GPIO4 |
| D4 | GPIO5 |
| D5 | GPIO6 |
| D6 | GPIO7 |
| D7 | GPIO39 |
| D8 | GPIO40 |
| D9 | GPIO41 |
| D10 | GPIO42 |
| D11 | GPIO37 |
| D12 | GPIO33 |
| D13 | GPIO47 |
| D14 | GPIO48 |

### Control lines
| HP Signal | ESP32 GPIO | Direction |
|---|---|---|
| DAV (active‑low) | GPIO20 | OUT |
| RFD (active‑low) | GPIO19 | IN |
| DISCONNECT SENSE | GND | tie to ground |
| GND | GND | common ground |

### RFD level shifting (required)
HP RFD is 5V TTL. **Do not** connect directly to ESP32.

Suggested divider:
```
HP RFD ---- 10k ----+---- ESP32 GPIO19
                   |
                  20k
                   |
                  GND
```

## OLED (onboard)
The code uses the onboard 0.96" OLED (SSD1306 class) via I2C:
- SDA = GPIO17
- SCL = GPIO18
- RST = GPIO21
- Vext = GPIO36 (active‑low enable)

If your board revision differs, set `SCREEN_SDA`, `SCREEN_SCL`, `SCREEN_ADDR`, `SCREEN_RST_PIN`, or `SCREEN_VEXT_PIN` via build flags.

## Build & flash (PlatformIO)
1. Install PlatformIO (VSCode or CLI).
2. Connect the ESP32 board.
3. Build/flash:
   ```
   pio run -t upload
   ```
4. Monitor serial output:
   ```
   pio device monitor
   ```

Serial runs at **115200**.

## Serial commands
Type into the serial monitor:
- `h` help
- `s` status
- `d` pulse DAV low (wiring check)
- `w` toggle D11 (pen bit) for wiring check
- `p` toggle pen up/down sense
- `t` toggle transfers on/off
- `v` toggle verbose logging
- `b` toggle per‑word handshake trace
- `x` toggle RX echo
- `0..9` select test pattern
- `n` next test pattern

## Test patterns
There are multiple test cases (0–9). Examples include:
- Geometric primitives (square, spiral, circle)
- Starburst
- Ripple/sombrero mesh (cached for speed)
- Spirograph
- Checkerboard
- Starfield and Matrix‑style text

The top‑left text overlay shows **FPS** and **V** (vectors per frame).

## Troubleshooting
**RFD never goes low**
- Ensure **DAV is high** (idle).
- Check DISCONNECT SENSE is tied to HP ground.
- Confirm your RFD divider wiring and common ground.

**No display / blank**
- Verify correct data bus wiring.
- Use the `d` command to pulse DAV and check at HP pin 10 with a scope.
- Try toggling the pen bit with `w` and probe D11.

**OLED blank**
- Confirm Vext (GPIO36) is driven LOW.
- Check SDA/SCL pins and address (0x3C/0x3D).

## Project layout
```
src/
  main.cpp              // Test patterns, loop, serial UI
  hp_vector_device.*    // HP1345A immediate‑mode bus + handshake
  screen.*              // OLED wrapper (Adafruit GFX / SSD1306)
platformio.ini
```

## Safety notes
The HP1345A contains **high voltages**. Do not open or service it while powered. Always power down before rewiring.
