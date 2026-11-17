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
