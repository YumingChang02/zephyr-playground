# Waveshare ESP32-S3-GEEK Zephyr Application

This folder contains the Zephyr application for the **Waveshare ESP32-S3-GEEK** development board.

---

## Hardware Overview

The **ESP32-S3-GEEK** is a compact USB stick-form dev board featuring:
- **MCU**: Espressif ESP32-S3 (Xtensa Dual-core 32-bit LX7, up to 240 MHz)
- **Memory**: 16 MB Flash, 2 MB PSRAM, 512 KB SRAM
- **Display**: 1.14" 65K color IPS LCD (ST7789V driver, 240x136, SPI interface)
- **Storage**: MicroSD card slot (4-bit SDHC bus)
- **USB**: USB Type-A port connected to ESP32-S3 native USB (JTAG/Serial & USB OTG)
- **Wireless**: 2.4 GHz Wi-Fi (802.11 b/g/n) & Bluetooth 5 (LE)
- **Buttons**: Boot button (`button0` on GPIO 0)
- **I/O Header**: UART0, I2C0, and GPIOs on a 1.25mm connector

---

## Zephyr Board Support

- **Target Identifier**: `esp32s3_geek/esp32s3/procpu`
- **Device Tree**: Located in `zephyrproject/zephyr/boards/waveshare/esp32s3_geek/`
- **Chosen Nodes**:
  - Console: `usb_serial` (native USB CDC ACM)
  - Display: `st7789p3` (ST7789V controller)
  - SD Card: `sdhc0` (`zephyr,sdmmc-disk`)
  - Wi-Fi / BT: `wifi`, `esp32_bt_hci`

---

## Application Architecture

The application runs a 4-thread RTOS pipeline demonstrating preemptive prioritization, inter-thread messaging, hardware timer synchronization, FAT32 SD card CSV logging with daily auto-rotation, NTP real-time clock tracking, and an embedded Wi-Fi web server:

```text
[Hardware Timer ISR]
       │ (k_sem_give)
       ▼
[Thread 1: Speed Sampler (Priority 3)]
       │ (motor_speed_msgq)
       ▼
[Thread 2: Sensor Aggregator (Priority 5)]
       ├── (csv_msgq)       ──> [Thread 3: SD Card FAT32 CSV Logger (Priority 7)]
       └── (k_condvar)      ──> [Thread 4: Wi-Fi, NTP & Multi-Client SSE Web Server (Priority 8)]
```

| Thread | Priority | Stack | Function |
| :--- | :---: | :---: | :--- |
| **Thread 1: Speed Sampler** | 3 (High) | 2 KB | Unblocks on 1 Hz hardware counter ISR (`timer_tick_sem`), reads mock motor FG pulse count, calculates RPM, and pushes into `motor_speed_msgq`. |
| **Thread 2: Sensor Aggregator** | 5 (Med) | 2 KB | Drains `motor_speed_msgq`, combines motor speed with mock DS18B20 temp (`°C`), INA219 bus voltage (`V`), and motor current (`mA`), timestamps via NTP, and notifies web and SD card threads. |
| **Thread 3: SD Card CSV Logger** | 7 (Low) | 4 KB | Mounts the SD card as FAT32 (auto-formatting on boot if unformatted), logs structured CSV records, and auto-rotates to a new file daily (`/SD:/telemetry_YYYY-MM-DD.csv`). |
| **Thread 4: Wi-Fi / NTP / Web Server** | 8 (Background) | 6 KB | Connects to Wi-Fi station, synchronizes wall-clock time via SNTP, serves zero-CDN canvas dashboard at `GET /`, and streams live telemetry + recent CSV history (up to 1 hour) to multiple clients at `GET /events`. |

### Processor / Core Allocation Note
In Zephyr RTOS on ESP32-S3, hardware peripherals (Wi-Fi MAC/PHY hardware interrupts, 4-bit SDMMC controller) and Espressif proprietary binary blobs (`libnet80211.a`, `libpp.a`, `libphy.a`) are bound to PRO CPU (Core 0). In Zephyr's architecture, AMP splits the chip into two separate applications via IPC, but Wi-Fi cannot run independently on APP CPU because the Espressif Wi-Fi driver and hardware interrupts are hardwired to PRO CPU. Running all threads (or dedicating prioritized RTOS threads) on PRO CPU provides high performance without inter-core IPC bottlenecks.

---

## Web Server & SSE Endpoints

1. **`GET /` (Interactive Canvas Dashboard)**:
   - Self-contained dark-theme HTML5 dashboard (zero CDN dependencies).
   - Dual Canvas charts with **interactive hover inspection**: hovering or touching either chart draws a vertical crosshair and displays a dynamic floating tooltip showing exact timestamp, RPM, temperature, voltage, and current values.
   - Status indicators for Wi-Fi connection, active SSE client count, NTP sync state, and SD card logging.
2. **`GET /events` (Server-Sent Events Stream)**:
   - Supports **multiple simultaneous clients** (`MAX_SSE_CLIENTS = 4`) multiplexed using non-blocking I/O (`zsock_poll`).
   - **Recent History Replay**: When a client connects, reads the last data (clipped to max 1 hour = 3600 samples) from the current day's CSV file on the SD card, sends it over SSE (tagged with `"hist":true`), and then seamlessly continues streaming live telemetry events!

---

## Wi-Fi Configuration

Wi-Fi credentials are kept private and out of version control using `credentials.conf`:

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

## Building and Flashing

From this folder (`/home/markchang/zephyr/ESP32-S3-GEEK`):

### 1. Build
```bash
# Clean / pristine build:
uv run west build -p always -b esp32s3_geek/esp32s3/procpu app

# Incremental build:
uv run west build app
```

### 2. Flash
Connect the board to your USB port and run:
```bash
uv run west flash
```

### 3. Monitor Serial Console
```bash
uv run west espressif monitor
```
*(Press `Ctrl + ]` to exit the monitor)*

