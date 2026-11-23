# Waveshare ESP32-C6-LCD-1.47 Real-Time Motor Engine & Telemetry System

A production-grade, single-core multithreaded Zephyr RTOS application for the **Waveshare ESP32-C6-LCD-1.47** development board (ESP32-C6 RISC-V HP Core @ 160 MHz).

---

## 1. Hardware Architecture & Pinout

The **Waveshare ESP32-C6-LCD-1.47** integrates a 1.47" ST7789 display (172x320) and a MicroSD (TF) card slot sharing hardware SPI bus 2 (`spi2`), plus breakout header GPIOs for motor control and telemetry sensors.

### On-board Peripherals (SPI2)
| Function | GPIO Pin | Description |
| :--- | :--- | :--- |
| **SPI SCLK** | `GPIO 7` | Shared SPI2 Clock (LCD + MicroSD) |
| **SPI MOSI** | `GPIO 6` | Shared SPI2 MOSI Data Out (LCD + MicroSD) |
| **SPI MISO** | `GPIO 5` | Shared SPI2 MISO Data In (MicroSD) |
| **LCD CS** | `GPIO 14` | ST7789 Display Chip Select |
| **LCD DC** | `GPIO 15` | ST7789 Data / Command Control |
| **LCD RST** | `GPIO 21` | ST7789 Hardware Reset |
| **LCD BL** | `GPIO 22` | LCD Backlight Control |
| **SD CS** | `GPIO 4` | MicroSD Card Chip Select (FAT32) |
| **RGB LED** | `GPIO 8` | On-board WS2812 User RGB LED |
| **Boot Key** | `GPIO 9` | On-board User Button |

### External Breakout Header Connections
| Function | GPIO Pin | Description |
| :--- | :--- | :--- |
| **Motor PWM** | `GPIO 18` | Hardware LEDC PWM Channel 0 (25 kHz, Active-Low) |
| **FG Tachometer** | `GPIO 19` | Motor Frequency Generator Tachometer Pulse Input (Falling Edge Interrupt) |
| **I2C SCL** | `GPIO 20` | I2C0 Clock for INA219 Current / Power Sensor |
| **I2C SDA** | `GPIO 23` | I2C0 Data for INA219 Current / Power Sensor |

### Breadboard Physical Pinout & Wiring Diagram

Top view of the **Waveshare ESP32-C6-LCD-1.47** seated across a standard breadboard center channel (USB-C at bottom), showing the on-board reserved pins and the external connections used by this project:

```text
  Breadboard (-) GND Rail ───────┬───────────────────────────────────────────────┬──── 12V PSU (-)
  Breadboard (+) 3V3 Rail ───────┼───────┬───────────────────────────────────────┼──┐
                                 │       │                                       │  │
                        ┌────────┴───────┴────────┐                              │  │
                        │    ESP32-C6-LCD-1.47    │                              │  │
  5V (USB VBUS) ───────o│ 5V                  GND │o── (Common GND)              │  │
  Common GND    ───────o│ GND                 3V3 │o─────────────────────────────┼──┘
  (Onboard RGB) ───────o│ IO8 (WS2812)       IO16 │o── UART0 TX                  │
  (Boot Button) ───────o│ IO9 (BOOT)         IO17 │o── UART0 RX                  │
  (SD Card CS)  ───────o│ IO4 (SD_CS)        IO18 │o──► Motor PWM (25 kHz)       │
  (SPI MISO)    ───────o│ IO5 (SD_MISO)      IO19 │o◄── Motor FG (10k Pull-Up)   │
  (SPI MOSI)    ───────o│ IO6 (MOSI)         IO20 │o──► INA219 SCL               │
  (SPI SCLK)    ───────o│ IO7 (SCLK)         IO21 │o── (Onboard LCD RST)         │
  (LCD CS)      ───────o│ IO14 (LCD_CS)      IO22 │o── (Onboard LCD BL)          │
  (LCD DC)      ───────o│ IO15 (LCD_DC)      IO23 │o──► INA219 SDA               │
                        │         [USB-C]         │                              │
                        └─────────────────────────┘                              │
                                                                                 │
  ┌──────────────────────────────────────────────────────────────────────┐       │
  │  Breadboard Peripheral Connections                                   │       │
  │                                                                      │       │
  │  [INA219 @ 0x40]                 [4-Wire BLDC Pump / Motor]          │       │
  │   VCC  ◄── 3V3 Rail               VCC (Red)   ◄── INA219 VIN-        │       │
  │   GND  ◄── GND Rail               PWM (Blue)  ◄── IO18 (GPIO 18)     │       │
  │   SCL  ◄── IO20 (GPIO 20)         FG  (Yel)   ──► IO19 (+ 10k to 3V3)│       │
  │   SDA  ◄── IO23 (GPIO 23)         GND (Black) ──► GND Rail ──────────┴───────┘
  │   VIN+ ◄── 12V PSU (+)
  │   VIN- ──► Motor VCC (Red)
  └──────────────────────────────────────────────────────────────────────┘
```
  12V Motor PSU (+) ──────────────────────────┐
                                              ▼
  +-----------------------------------+   +-------------------+
  |  Waveshare ESP32-C6-LCD-1.47      |   |  INA219 (0x40)    |
  |                                   |   |                   |
  |  [3V3] ───────────────────────────┼──►| VCC          VIN+ |◄── 12V PSU (+)
  |  [GND] ──────┬────────────────────┼──►| GND          VIN- |──┐
  |              │                    |   |                   |  │
  |  [GPIO 20] ──┼────────────────────┼──►| SCL               |  │
  |  [GPIO 23] ──┼────────────────────┼──►| SDA               |  │
  |              │                    |   +-------------------+  │
  |              │                    |                          │
  |              │                    |   +-------------------+  │
  |              │                    |   |  4-Wire Pump/Fan  |  │
  |              │                    |   |                   |  │
  |              │                    |   | VCC (Red)         |◄─┘
  |  [GPIO 18] ──┼────────────────────┼──►| PWM (Blue/White)  |
  |  [GPIO 19] ◄─┼────────────────────┼───┤ FG  (Yellow)      |
  |              ├────────────────────┼──►| GND (Black)       |
  +--------------┼--------------------+   +-------------------+
                 │
                 ▼
  12V Motor PSU (-) / Common Breadboard GND Rail
```

> **Wiring Notes:**
> - **Verify Silkscreen:** Pin positions on the left/right headers can vary slightly between Waveshare board revisions; always match the **GPIO number printed on the board silkscreen** (`IO18`, `IO19`, `IO20`, `IO23`, `3V3`, `GND`).
> - **Common Ground:** Tie `GND` of the 12V motor power supply, `GND` of the INA219, `GND` of the motor/pump, and `GND` of the ESP32-C6 together on the breadboard ground rail.
> - **3.3V Logic Only:** ESP32-C6 GPIOs are **3.3V** (not 5V tolerant). Power the INA219 from `3V3` so `SDA`/`SCL` pull-ups stay at 3.3V, and never pull `FG` (`IO19`) up to 12V.
> - **Avoid On-board Reserved Pins:** `IO4`, `IO5`, `IO6`, `IO7`, `IO8`, `IO9`, `IO14`, `IO15`, `IO21`, and `IO22` are wired to the on-board ST7789 LCD, MicroSD slot, WS2812 RGB LED, or Boot button.

---

## 2. Software Architecture

Unlike the dual-core AMP sysbuild on ESP32-S3, the **ESP32-C6 architecture is a unified, single-core multithreaded system**. All subsystems run concurrently on the 160 MHz RISC-V core using Zephyr's preemptive priority scheduler:

```
[Hardware Interrupts / Peripherals]
  ├── GPIO 19 Falling-Edge ISR  ──> [Thread 1: Motor Speed & Profile (Priority 2)]
  │                                       │ (motor_speed_msgq)
  │                                       ▼
  ├── I2C0 INA219 Sensor        ──> [Thread 2: Sensor Aggregator (Priority 4)]
  │                                       ├── (csv_msgq) ──> [Thread 4: FAT32 SD Logger (Priority 7)]
  │                                       └── (telemetry_spinlock)
  │                                                ▼
  ├── ST7789 172x320 LCD        <── [Thread 3: LCD Display Engine (Priority 5)]
  │
  ├── Wi-Fi 6 + HTTP + SSE      <── [Thread 5: HTTP Web Server (Priority 8)]
  │
  ├── Non-blocking Download     <── [Thread 6: Background CSV Downloader (Priority 9)]
  │
  └── pool.ntp.org (SNTP)       <── [Thread 7: NTP Sync Client (Priority 10)]
```

### Key Features
1. **Real-time Closed-Loop Timing**:
   - Hardware LEDC PWM generator operates completely independent of CPU load.
   - GPIO edge interrupts guarantee zero pulse drops on the FG tachometer line up to 10,000+ RPM.
   - 4-stage profile (Low &rarr; Ramp Up &rarr; High &rarr; Drop) phase-aligned to wall-clock `00:00:00`.
2. **ST7789 1.47" Color Display (172x320)**:
   - 3 stacked horizontal gauges on left: PWM Target % (Cyan), Speed RPM % (Green), Current mA % (Amber).
   - Right status widget: Date (`MM/DD`), 24-hr clock (`HH:MM:SS`), and 4-color stage countdown (`MM' SS''`).
3. **Daily Rolling CSV Storage**:
   - Filesystem: FAT32 on MicroSD card at `/SD:/telemetry_YYYY-MM-DD.csv`.
   - Auto-pruning to preserve the latest 7 days of logs.
4. **Non-blocking Web Server**:
   - Live browser dashboard with Canvas telemetry graphs.
   - Real-time Server-Sent Events (SSE) stream (`/events`).
   - Dynamic profile reconfiguration REST API (`/api/profile/config`, `/api/profile/start`, `/api/profile/stop`).
   - Non-blocking chunked CSV downloads (`/api/download/csv`).

---

## 3. Wi-Fi Configuration

Wi-Fi credentials are kept private and out of version control using `app/credentials.conf`:

1. Copy the template:
   ```bash
   cp app/credentials.conf.template app/credentials.conf
   ```
2. Edit `app/credentials.conf` with your network credentials:
   ```ini
   CONFIG_APP_WIFI_SSID="YourNetworkSSID"
   CONFIG_APP_WIFI_PSK="YourNetworkPassword"
   ```
`credentials.conf` is ignored by Git and automatically merged into the build by `app/CMakeLists.txt`.

---

## 4. Building & Flashing

From the `ESP32-C6-LCD-147` directory:

### 1. Build Firmware
```bash
cd ~/zephyr-playground/ESP32-C6-LCD-147

# Pristine build:
uv run west build -p always -b esp32c6_lcd_1_47/esp32c6/hpcore app

# Incremental build:
uv run west build app
```

*(Note: Ensure `ZEPHYR_BASE` and `ZEPHYR_SDK_INSTALL_DIR` are set in your environment as described in the workspace root README, e.g. via `zephyr.fish` or `~/.bashrc`).*

### 2. Flash Firmware
```bash
uv run west flash
```

### 3. Monitor Serial Console
```bash
uv run west espressif monitor
```
*(Press `Ctrl + ]` to exit the monitor)*
