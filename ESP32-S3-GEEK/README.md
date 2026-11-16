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

## Application Architecture

The active firmware runs a multi-threaded RTOS pipeline supporting a **1~10 Hz update cycle** (`CONFIG_APP_SAMPLE_RATE_HZ`), real hardware drivers (GPIO FG pulse counter, 25 kHz LEDC PWM, INA219 I2C voltage/current sensor), FAT32 SD card CSV logging with daily auto-rotation, a dedicated background NTP client, and an embedded Wi-Fi web dashboard:

```text
[Hardware Timer ISR (1~10 Hz)]
       │ (k_sem_give)
       ▼
[Thread 1: Motor Speed & PWM (Priority 3)]
       │ (motor_speed_msgq)
       ▼
[Thread 2: Sensor Aggregator (Priority 5)]
       ├── (csv_msgq)        ──> [Thread 3: SD Card FAT32 CSV Logger (Priority 7)]
       └── (k_condvar)       ──> [Thread 4: Wi-Fi & Multi-Client SSE Server (Priority 8)]
                                        │ (k_sem_give on IP acquired)
                                        ▼
                                 [Thread 5: Background NTP Client (Priority 10)]
```

| Thread | Priority | Stack | Function |
| :--- | :---: | :---: | :--- |
| **Thread 1: Motor Speed & PWM** | 3 (High) | 2 KB | Unblocks on hardware counter ISR (`timer_tick_sem` at 1~10 Hz). Reads falling-edge pulses from GPIO 6 (FG feedback pin), computes motor RPM, drives motor PWM at 25 kHz on GPIO 7, and pushes into `motor_speed_msgq`. |
| **Thread 2: Sensor Aggregator** | 5 (Med) | 2 KB | Drains `motor_speed_msgq`, samples bus voltage and current from INA219 via I2C0 (`0x40`), timestamps via synchronized NTP clock, and updates live telemetry cache. |
| **Thread 3: SD Card CSV Logger** | 7 (Low) | 4 KB | Mounts SD card as FAT32 (auto-formatting on startup if unformatted), writes telemetry CSV records, flushes periodically to protect flash, and auto-rotates daily (`/SD:/telemetry_YYYY-MM-DD.csv`). |
| **Thread 4: Wi-Fi & Web Server** | 8 (Background) | 6 KB | Connects to Wi-Fi with auto-reconnect supervisor, serves standalone canvas dashboard at `GET /`, and streams live events + up to 1 hour CSV history to multiple clients at `GET /events`. |
| **Thread 5: Background NTP Client** | 10 (Lowest) | 3 KB | Decoupled background synchronizer. Synchronizes wall-clock time from `pool.ntp.org` upon network connection and repeats every 1 hour to eliminate RTC drift without blocking the web server or sensor loop. |

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

## Building and Flashing Workflows

Depending on your architecture goals, Zephyr supports three build and execution workflows for the ESP32-S3:

---

### Workflow 1: Unified Preemptive Pipeline on PRO_CPU (Current Default)

The active application implements a 5-thread real-time preemptive pipeline on `PRO_CPU`. Because Zephyr RTOS uses strict priority preemption, Thread 1 (Priority 3) and Thread 2 (Priority 5) sample sensors and pulse counters at 1~10 Hz with **microsecond latency**, preempting background Wi-Fi (Priority 8) and SD card writes (Priority 7) without requiring inter-core IPC marshaling.

From this folder (`~/zephyr-playground/ESP32-S3-GEEK`):

#### 1. Build
```bash
# Pristine / clean build:
uv run west build -p always -b esp32s3_geek/esp32s3/procpu app

# Incremental build:
uv run west build app

# Fast C/C++ source rebuild (bypasses Python checks):
uv run ninja -C build
```

#### 2. Flash
Connect the ESP32-S3-GEEK USB stick to your computer and run:
```bash
uv run west flash
```

#### 3. Monitor Serial Console
```bash
uv run west espressif monitor
```
*(Press `Ctrl + ]` to exit the monitor).*

---

### Workflow 2: Unified Multi-Core AMP with Sysbuild (`PRO_CPU` + `APP_CPU`)

When dedicating `APP_CPU` (Core 1) exclusively to real-time motor control and sensor acquisition while `PRO_CPU` (Core 0) handles Wi-Fi, FatFS SD logging, and the web server, Zephyr's **`sysbuild`** orchestrates compiling and flashing both cores in a single pass.

#### 1. Multi-Core Directory Layout
```text
ESP32-S3-GEEK/
├── CMakeLists.txt                  <-- Root CMake file
├── sysbuild.cmake                  <-- Sysbuild orchestration script
├── app/                            <-- Primary application (PRO_CPU: Wi-Fi, SD, HTTP/SSE)
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── app.overlay
│   └── src/main.c
└── remote/                         <-- Secondary application (APP_CPU: FG Counter, PWM, I2C)
    ├── CMakeLists.txt
    ├── prj.conf
    ├── app.overlay
    └── src/main.c
```

#### 2. Configuring `sysbuild.cmake`
In `ESP32-S3-GEEK/sysbuild.cmake`, register the secondary core as an external Zephyr project:
```cmake
# Automatically derive remote board qualifier from primary target:
string(REPLACE "procpu" "appcpu" REMOTE_CPU "${BOARD_QUALIFIERS}")
string(CONFIGURE "${BOARD}/${REMOTE_CPU}" IPM_REMOTE_BOARD)

# Add APP_CPU application to sysbuild
ExternalZephyrProject_Add(
    APPLICATION app_cpu_firmware
    SOURCE_DIR ${APP_DIR}/remote
    BOARD ${IPM_REMOTE_BOARD}
)

# Ensure APP_CPU firmware compiles alongside PRO_CPU
add_dependencies(app app_cpu_firmware)
sysbuild_add_dependencies(CONFIGURE app app_cpu_firmware)
```

#### 3. Build Both Cores with Sysbuild
Run a single `west build` command with the `--sysbuild` flag:
```bash
uv run west build --sysbuild -p always -b esp32s3_geek/esp32s3/procpu app
```
Sysbuild automatically:
1. Configures and compiles `esp32s3_geek/esp32s3/appcpu` in `build/app_cpu_firmware/`.
2. Configures and compiles `esp32s3_geek/esp32s3/procpu` in `build/app/`.
3. Packages both ELF binaries and generates a unified multi-image flash specification.

#### 4. Flash Both Cores in One Step
```bash
uv run west flash
```
`west flash` automatically writes:
- MCUboot bootloader and partition table to `0x0000`
- `PRO_CPU` image to `slot0_partition` (`0x020000`)
- `APP_CPU` image to `slot0_appcpu_partition` (`0xBC0000`)

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

In AMP multi-core mode, data exchange between `PRO_CPU` and `APP_CPU` is managed via Zephyr's Inter-Processor Mailbox (`ipm0`) and 16 KB Shared SRAM (`shm0`):

1. **Devicetree Binding**:
   Both `esp32s3_geek_esp32s3_procpu.dts` and `esp32s3_geek_esp32s3_appcpu.dts` define:
   ```dts
   chosen {
       zephyr,ipc = &ipm0;
       zephyr,ipc_shm = &shm0;
   };
   ```

2. **Shared Telemetry Struct in `shm0`**:
   Place the shared sensor struct in the uncached `shm0` address block (`0x3FCE5400`):
   ```c
   #define SHARED_MEM_ADDR 0x3FCE5400
   static volatile telemetry_packet_t *shm_telemetry = (volatile telemetry_packet_t *)SHARED_MEM_ADDR;
   ```

3. **Inter-Core Signaling via IPM**:
   * **`APP_CPU` (Sender)**: After updating sensor data in `shm0`, sends a hardware interrupt pulse:
     ```c
     const struct device *ipm = DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));
     ipm_send(ipm, 0, 2, NULL, 0);  /* Channel 2 (application data channel) */
     ```
   * **`PRO_CPU` (Receiver)**: Registers an IPM callback to wake up the SD card logger and SSE broadcaster:
     ```c
     static void ipm_callback(const struct device *dev, void *user_data, uint32_t id, volatile void *data)
     {
         if (id == 2) {
             /* Read from shm_telemetry, write to SD card, and stream to SSE */
         }
     }

     ipm_register_callback(ipm, ipm_callback, NULL);
     ipm_set_enabled(ipm, 1);
     ```

---

## Serial Console & Multi-Core Monitoring

Both cores route their standard `printk()` output through the ESP32-S3 hardware:
* To monitor serial output:
  ```bash
  uv run west espressif monitor
  ```
* Messages printed by `APP_CPU` and `PRO_CPU` will be interleaved with core tags on the serial monitor.
* Press `Ctrl + ]` to exit the monitor.

