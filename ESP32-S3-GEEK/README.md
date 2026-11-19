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

## Zephyr Board Support & Dual-Core Architecture

The ESP32-S3 SoC contains **two symmetrical 32-bit Xtensa LX7 cores** running up to 240 MHz:
- **Core 0 (`PRO_CPU`)**: Primary core. Boots first from ROM, handles system initialization, executes proprietary Espressif binary blobs (`libnet80211.a`, `libpp.a`, `libphy.a`), manages Wi-Fi MAC/baseband hardware interrupts, and drives high-throughput peripherals like the 4-bit SDMMC host.
- **Core 1 (`APP_CPU`)**: Secondary core. Started by `PRO_CPU` in Asymmetric Multiprocessing (AMP) mode. Ideal for dedicated hard real-time tasks (motor FG pulse counting, PWM generation, and I2C sensor acquisition) isolated from networking bursts.

In Zephyr RTOS, the board is split into two distinct target qualifiers:
- **`esp32s3_geek/esp32s3/procpu`**: Builds firmware targeting Core 0.
- **`esp32s3_geek/esp32s3/appcpu`**: Builds firmware targeting Core 1.

### Flash Partitioning & Inter-Core Memory Map
When running in AMP mode, the 16 MB flash and internal SRAM are divided according to `partitions_0x0_amp_16M.dtsi`:

| Region | Physical Address / Offset | Size | Purpose |
| :--- | :---: | :---: | :--- |
| **Bootloader (MCUboot)** | `0x000000` | 64 KB | Second-stage bootloader |
| **System Partition** | `0x010000` | 64 KB | System & partition metadata |
| **PRO_CPU Primary (`slot0_partition`)** | `0x020000` | 5,952 KB | Primary application binary (Core 0: Wi-Fi, SD card, HTTP/SSE) |
| **PRO_CPU Secondary (`slot1_partition`)** | `0x5F0000` | 5,952 KB | OTA update scratch slot for PRO_CPU |
| **APP_CPU Primary (`slot0_appcpu_partition`)** | `0xBC0000` | 1,984 KB | Secondary application binary (Core 1: Sensor & Motor Control) |
| **APP_CPU Secondary (`slot1_appcpu_partition`)** | `0xDB0000` | 1,984 KB | OTA update scratch slot for APP_CPU |
| **Storage (`storage_partition`)** | `0xFB0000` | 192 KB | Non-volatile storage (NVS / LittleFS) |
| **Shared SRAM (`shm0`)** | `0x3FCE5400` | 16 KB | Uncached internal SRAM for zero-copy inter-core data packets |
| **Inter-Processor Mailbox (`ipm0`)** | `0x3FCE9400` | 8 bytes | Hardware inter-core doorbell interrupts (`espressif,esp32-ipm`) |

---

## Application Architecture (Multi-Core AMP via Sysbuild)

The firmware runs a **true Asymmetric Multiprocessing (AMP)** pipeline across both Xtensa cores:
- **Core 1 (`APP_CPU` / `remote`)**: Executes dedicated hard real-time tasks (hardware counter ISR at 1~10 Hz, GPIO FG pulse counter, 25 kHz LEDC PWM motor drive, and INA219 I2C sampling) with zero jitter. Telemetry packets are sent to Core 0 via Inter-Processor Mailbox (`ipm0` Channel 2).
- **Core 0 (`PRO_CPU` / `app`)**: Ingests IPM telemetry packets, manages FAT32 SD card CSV logging with daily auto-rotation, synchronizes wall-clock time via background NTP, and hosts the multi-client SSE Web Dashboard over Wi-Fi.

```text
 ┌────────────────────────────────────────────────────────────────────────┐
 │                      Core 1: APP_CPU (remote)                          │
 │                                                                        │
 │  [Hardware Timer ISR (1~10 Hz)]                                        │
 │         │ (k_sem_give)                                                 │
 │         ▼                                                              │
 │  [Thread 1: Motor Speed & PWM (Priority 3)]                            │
 │         │ (motor_speed_msgq)                                           │
 │         ▼                                                              │
 │  [Thread 2: Sensor Aggregator (Priority 5)]                            │
 │         │ (ipm_send: Channel 2)                                        │
 │         ▼                                                              │
 │  [Thread 3: 3-Bar LCD Display Engine (Priority 9)]                     │
 │         └── ST7789 SPI Display: PWM %, FG %, Power % (10 Hz)          │
 └─────────┼──────────────────────────────────────────────────────────────┘
           │ Hardware Inter-Processor Mailbox (ipm0)
           ▼
 ┌────────────────────────────────────────────────────────────────────────┐
 │                      Core 0: PRO_CPU (app)                             │
 │                                                                        │
 │  [IPM Receive Callback (ISR)]                                          │
 │         ├── (csv_msgq)  ──> [Thread 1: SD Card FAT32 Logger (Prio 7)]  │
 │         │                        └── Daily rotation (max 7 kept)       │
 │         └── (condvar)   ──> [Thread 2: Wi-Fi & Web Server (Prio 8)]    │
 │                                      ├── Dashboard (GET /)             │
 │                                      ├── CSV Download (GET /download)  │
 │                                      └── SSE Stream (GET /events)      │
 │                                      │ (signals on IP acquired)        │
 │                                      ▼                                 │
 │                               [Thread 3: Background NTP (Prio 10)]     │
 └────────────────────────────────────────────────────────────────────────┘
```

| Core | Subsystem / Thread | Priority | Function |
| :--- | :--- | :---: | :--- |
| **APP_CPU** | **Thread 1: Motor Speed & PWM** | 3 (High) | Unblocks on counter ISR (`timer_tick_sem` at 1~10 Hz). Reads falling-edge pulses from GPIO 6, computes motor RPM, and drives 25 kHz PWM on GPIO 14. |
| **APP_CPU** | **Thread 2: Sensor Aggregator** | 5 (Med) | Samples bus voltage and current from INA219 via I2C0 (`0x40`), packages `telemetry_packet_t`, and transmits via IPM Channel 2. |
| **APP_CPU** | **Thread 3: LCD Display Engine** | 9 (Low) | Drives onboard 1.14" ST7789 LCD over SPI at 10 Hz. Renders 3 horizontal gauge bars: PWM Target %, FG RPM %, and Power (W) %. |
| **PRO_CPU** | **Thread 1: SD Card CSV Logger** | 7 (Low) | Mounts SD card as FAT32, writes telemetry CSV records, flushes periodically, and auto-rotates daily (`/SD:/telemetry_YYYY-MM-DD.csv`) with automatic pruning (`MAX_CSV_FILES_RETAINED = 7`). |
| **PRO_CPU** | **Thread 2: Wi-Fi & Web Server** | 8 (Background) | Connects to Wi-Fi with auto-reconnect supervisor, serves standalone canvas dashboard at `GET /`, direct file download at `GET /download`, and streams live SSE + CSV history at `GET /events`. |
| **PRO_CPU** | **Thread 3: Background NTP Client** | 10 (Lowest) | Synchronizes wall-clock time from `pool.ntp.org` to give precise absolute timestamps to CSV logs and Web UI. |

---

## Web Server & HTTP Endpoints

1. **`GET /` (Interactive Canvas Dashboard)**:
   - Self-contained dark-theme HTML5 dashboard (zero CDN dependencies).
   - Dual Canvas charts with **interactive hover inspection**: hovering or touching either chart draws a vertical crosshair and displays a dynamic floating tooltip showing exact timestamp, RPM, temperature, voltage, and current values.
   - Includes a one-click **`📥 Download CSV`** button in the dashboard header.
   - Status indicators for Wi-Fi connection, active SSE client count, NTP sync state, and SD card logging.
2. **`GET /download` or `GET /telemetry.csv` (CSV File Download)**:
   - Streams the active day's CSV log directly from the SD card to the browser as an attachment (`Content-Disposition: attachment; filename="telemetry_YYYY-MM-DD.csv"`).
3. **`GET /events` (Server-Sent Events Stream)**:
   - Supports **multiple simultaneous clients** (`MAX_SSE_CLIENTS = 4`) multiplexed using non-blocking I/O (`zsock_poll`).
   - **Zero-Blocking Real-Time Broadcast**: Pure real-time telemetry streaming directly from Core 1 IPM frames without disk access latency or lock contention.
4. **`POST /api/profile/config`, `/start`, `/stop`, `/pwm` (REST Control APIs)**:
   - Stage profile configuration and execution via HTTP `POST` (with `GET` fallback).
   - Non-blocking execution sends IPM command packets immediately to Core 1 (`APP_CPU`).

---

## SD Card CSV Storage & Rotation Limit

- **Daily Rotation**: Log files are formatted as `/SD:/telemetry_YYYY-MM-DD.csv` based on wall-clock time from NTP.
- **Retention Limit (`MAX_CSV_FILES_RETAINED`)**: Configured via `#define MAX_CSV_FILES_RETAINED 7` in [app/src/main.c](file:///home/markchang/zephyr-playground/ESP32-S3-GEEK/app/src/main.c).
- **Auto-Pruning**: At each midnight rotation, the SD card logger enumerates all daily CSV files chronologically and automatically deletes the oldest logs when the total count exceeds `MAX_CSV_FILES_RETAINED`, preventing the FAT32 filesystem from filling up.

---

## Onboard 1.14" ST7789 LCD Screen (APP_CPU Gauge & Time Display)

The Waveshare ESP32-S3-GEEK features an onboard 240x136 IPS LCD display driven by Core 1 (`APP_CPU`) via SPI at a low priority (`Priority 9`):

### Visual Screen Layout (Stacked Bars + Right Time Widget)
- **Left Area (X = 4..161, Width 158 px) — 3 Vertically Stacked Bars with No Spacing**:
  - **Top Bar (Cyan `0x07FF`, Y = 2..45, Height 44 px)**: PWM Target (`0% ~ 100%`)
  - **Middle Bar (Bright Green `0x07E0`, Y = 46..89, Height 44 px)**: Motor FG Speed (`0% ~ 100%` of max RPM)
  - **Bottom Bar (Amber/Orange `0xFD20`, Y = 90..133, Height 44 px)**: Power Consumption (`0% ~ 100%` of max Watts)
  - Bars touch directly at Y = 45/46 and Y = 89/90 without dead space.
- **Right Area (X = 166..235, Width 70 px, Y = 2..133) — Current Time Widget**:
  - Top Indicator: `NTP` (emerald green when synced) or `SYNC` (amber)
  - **Line 1**: `MM/DD` (e.g. `11/17` in cool mint/cyan)
  - **Line 2**: `HH:MM:SS` (e.g. `10:45:00` in warm gold)
  - Bottom Tag: `UTC`
  - Current wall-clock time is synced from `PRO_CPU` to `APP_CPU` via Inter-Processor Mailbox (`ipm0` Channel 4) every 500 ms.

### Configurable Scale Macros
You can easily adjust the maximum RPM and maximum Power scales in [remote/src/main.c](file:///home/markchang/zephyr-playground/ESP32-S3-GEEK/remote/src/main.c):
```c
#define CONFIG_APP_MAX_FG_RPM   6000    /* 100% scale for Motor FG RPM (default: 6000 RPM) */
#define CONFIG_APP_MAX_POWER_W  25      /* 100% scale for Power in Watts (default: 25 W)   */
```

### Hardware Pinout Allocation
- **Display SPI2**: SCLK = GPIO 12, MOSI = GPIO 11, CS = GPIO 10, DC = GPIO 8, RST = GPIO 9
- **LCD Backlight**: GPIO 7 (controlled via `gpio-leds` node `lcd-bl`)
- **Motor PWM Output**: GPIO 14 (external 3-pin header)
- **Motor FG Pulse Input**: GPIO 6 (external 3-pin header, pull-up, falling edge ISR)
- **INA219 Current/Voltage Sensor**: I2C0 (SDA = GPIO 4, SCL = GPIO 5, address `0x40`)

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

## Building and Flashing Workflows

---

### Workflow 2: Unified Multi-Core AMP with Sysbuild (`PRO_CPU` + `APP_CPU`) (Active Default)

The active application implements a multi-core AMP architecture:
- `APP_CPU` (Core 1) is dedicated exclusively to real-time motor control (LEDC PWM, FG counter) and INA219 I2C sampling.
- `PRO_CPU` (Core 0) handles Wi-Fi, FatFS SD logging, NTP synchronization, and the HTTP/SSE web server.
- Zephyr's **`sysbuild`** orchestrates compiling MCUboot, `remote` (`APP_CPU`), and `app` (`PRO_CPU`), and flashing everything in a single step.

#### 1. Directory Structure
```text
ESP32-S3-GEEK/
├── sysbuild.cmake                  <-- Sysbuild orchestration script
├── Kconfig.sysbuild                <-- Sysbuild Kconfig source
├── include/
│   └── telemetry.h                 <-- Shared telemetry packet struct & IPM IDs
├── app/                            <-- Primary application (PRO_CPU: Wi-Fi, SD, HTTP/SSE)
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── app.overlay
│   ├── sysbuild.cmake
│   ├── Kconfig.sysbuild
│   ├── credentials.conf (local)
│   └── src/main.c
└── remote/                         <-- Real-time application (APP_CPU: FG Counter, PWM, I2C)
    ├── CMakeLists.txt
    ├── prj.conf
    ├── app.overlay
    └── src/main.c
```

#### 2. Build Both Cores with Sysbuild
Run from `~/zephyr-playground/ESP32-S3-GEEK`:
```bash
# Pristine / clean build:
uv run west build --sysbuild -p always -b esp32s3_geek/esp32s3/procpu app

# Incremental build:
uv run west build --sysbuild app

# Fast incremental rebuild of all images via ninja:
uv run ninja -C build
```

Sysbuild automatically:
1. Configures and compiles MCUboot bootloader.
2. Configures and compiles `esp32s3_geek/esp32s3/appcpu` in `build/remote/`.
3. Configures and compiles `esp32s3_geek/esp32s3/procpu` in `build/app/`.
4. Packages and signs both ELF binaries with MCUboot partition offsets.

#### 3. Flash All Cores in One Step
Connect the ESP32-S3-GEEK USB stick and run:
```bash
uv run west flash
```
`west flash` automatically writes:
- MCUboot bootloader and partition table to `0x0000`
- `PRO_CPU` image to `slot0_partition` (`0x020000`)
- `APP_CPU` image to `slot0_appcpu_partition` (`0xBC0000`)

#### 4. Monitor Serial Console
```bash
uv run west espressif monitor
```
*(Press `Ctrl + ]` to exit).*

---

### Workflow 1: Single-Core Pipeline on PRO_CPU (Legacy Alternative)

If running solely on Core 0 without multi-core AMP:
```bash
uv run west build -p always -b esp32s3_geek/esp32s3/procpu app
uv run west flash
```

---

### Workflow 3: Independent Core Target Builds (Discrete Compiling & Flashing)

If you prefer developing, compiling, and testing each core independently without `sysbuild`:

#### 1. Build & Flash PRO_CPU (Core 0)
```bash
# Build into a dedicated build directory:
uv run west build -d build/procpu -p always -b esp32s3_geek/esp32s3/procpu app

# Flash to primary slot (0x20000):
uv run west flash -d build/procpu
```

#### 2. Build & Flash APP_CPU (Core 1)
```bash
# Build into a separate build directory:
uv run west build -d build/appcpu -p always -b esp32s3_geek/esp32s3/appcpu remote

# Flash specifically to slot0_appcpu_partition offset (0xBC0000):
uv run west flash -d build/appcpu --esp-app-address 0xbc0000
```

---

## Inter-Core Communication (IPM & Shared Memory)

In AMP multi-core mode, data exchange and console routing between `PRO_CPU` and `APP_CPU` is managed via Zephyr's Inter-Processor Mailbox (`ipm0`) and Shared Memory (`shm0`):

1. **Devicetree Binding**:
   Both `esp32s3_geek_esp32s3_procpu.dts` and `esp32s3_geek_esp32s3_appcpu.dts` define:
   ```dts
   chosen {
       zephyr,ipc = &ipm0;
       zephyr,ipc_shm = &shm0;
   };
   ```

2. **Dedicated IPM Channels**:
   - **Channel 2 (`IPM_CHANNEL_APP_TELEMETRY`)**: Transmits packed 28-byte sensor samples (`telemetry_packet_t`) from Core 1 to Core 0 at 1~10 Hz with zero jitter.
   - **Channel 3 (`IPM_CHANNEL_APP_PRINT`)**: Transmits `printk` log messages (`ipc_print_packet_t`) from Core 1 to Core 0.

3. **Inter-Core Signaling & Print Routing**:
   - **`APP_CPU` (Sender)**: `printk()` is mapped to `app_printk()`, which formats the string and dispatches it over IPM Channel 3:
     ```c
     ipm_send(ipm, 1, IPM_CHANNEL_APP_PRINT, pkt.msg, strlen(pkt.msg) + 1);
     ```
   - **`PRO_CPU` (Receiver & Console Output)**:
     - The IPM ISR enqueues the message into `ipc_print_msgq` (`K_NO_WAIT`).
     - A dedicated background thread (`thread_ipc_console`, Priority 6) pulls messages from `ipc_print_msgq` and prints them out via the hardware UART/USB console.

---

## Serial Console & Multi-Core Monitoring

Core 0 (`PRO_CPU`) directly owns the serial console hardware. `APP_CPU` prints are seamlessly routed through IPM Channel 3:
* To monitor serial output:
  ```bash
  uv run west espressif monitor
  ```
* Output from both `APP_CPU` and `PRO_CPU` appears interleaved with core tags on the serial monitor.
* Press `Ctrl + ]` to exit the monitor.

### Structured Wi-Fi Status Logs
During boot and connection cycles, `PRO_CPU` provides detailed Wi-Fi diagnostics:
1. **Connection Initiation**: Logs attempt counter, target SSID, security type (`WPA2-PSK`/`Open`), station MAC address, and frequency band.
2. **Association Result**:
   - **Success**: Displays association confirmation, textual status (`wifi_conn_status_txt`), and transition to DHCP request.
   - **Failure**: Reports error status code, 802.11 status/reason codes (e.g. wrong password, AP not found), and 10s retry timer.
3. **DHCP & Network Banner**: Once an IPv4 lease is granted, prints IP address, subnet mask, default gateway, web dashboard URL, and SSE endpoint URL.
4. **Disconnection & Reconnect**: Identifies disconnection reason (`wifi_disconn_reason_txt`) and 802.11 reason code, resetting state and triggering auto-reconnect.
5. **Periodic Supervisor**: Every 30 seconds, outputs uptime, live Wi-Fi state (`IP` / `Associated (Waiting for DHCP)` / `Disconnected (Retrying)`), NTP sync status, and total attempt count.


