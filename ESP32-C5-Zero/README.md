# Waveshare ESP32-C5-Zero Zephyr Application

This folder contains the Zephyr application for the **Waveshare ESP32-C5-Zero** development board.

---

## Hardware Overview

The **ESP32-C5-Zero** is an ultra-compact mini development board based on the Espressif ESP32-C5 SoC:
- **MCU**: Espressif ESP32-C5 (32-bit RISC-V Single-core processor up to 240 MHz)
- **Wireless**: **Dual-band Wi-Fi 6 (2.4 GHz and 5 GHz)** + Bluetooth 5 (LE) + IEEE 802.15.4 (Zigbee / Thread)
- **Memory**: 4 MB / 8 MB Flash, 400 KB SRAM
- **USB**: USB Type-C port connected directly to ESP32-C5 native USB Serial/JTAG
- **Form Factor**: Castellated holes + pin header pins (Zero footprint)
- **Buttons**: Boot button & Reset button

---

## Zephyr Board Configuration

- **Target Identifier**: `esp32c5_devkitc/esp32c5/hpcore`
- **Architecture**: RISC-V 32-bit (`riscv64-zephyr-elf` toolchain)
- **Devicetree Overlay ([app/app.overlay](file:///home/markchang/zephyr-playground/ESP32-C5-Zero/app/app.overlay))**:
  - Directs serial console to native USB Serial/JTAG (`zephyr,console = &usb_serial;`)
  - Configures Timer Group 0 (`&timer0` -> `counter`) and maps it to the portable `app-timer` alias

---

## Multi-Threaded Architecture

The application runs a 4-thread RTOS pipeline demonstrating preemptive prioritization, inter-thread messaging, hardware timer synchronization, and an embedded Wi-Fi web server:

```
[Hardware Timer ISR]
       │ (k_sem_give)
       ▼
[Thread 1: Speed Sampler (Priority 3)]
       │ (motor_speed_msgq)
       ▼
[Thread 2: Sensor Aggregator (Priority 5)]
       ├── (telemetry_msgq)  ──> [Thread 3: Background Logger (Priority 7)]
       └── (k_condvar_signal) ──> [Thread 4: Wi-Fi HTTP / SSE Server (Priority 8)]
```

| Thread | Priority | Stack | Function |
| :--- | :---: | :---: | :--- |
| **Thread 1: Speed Sampler** | 3 (High) | 2 KB | Unblocks on 1 Hz hardware counter ISR (`timer_tick_sem`), reads mock motor FG pulse count, calculates RPM, and pushes into `motor_speed_msgq`. |
| **Thread 2: Sensor Aggregator** | 5 (Med) | 2 KB | Drains `motor_speed_msgq`, combines motor speed with mock DS18B20 temp (`°C`), INA219 bus voltage (`V`), and motor current (`mA`), pushes into `telemetry_msgq`, and notifies the web server via `telemetry_cv`. |
| **Thread 3: Background Logger** | 7 (Low) | 2 KB | Drains `telemetry_msgq` and prints formatted sensor telemetry to console. |
| **Thread 4: Wi-Fi HTTP / SSE** | 8 (Background) | 4 KB | Connects to Wi-Fi, requests DHCP IP, and listens on port 80 to serve web requests. |

---

## Web Server Endpoints

1. **`GET /` (Live Chart.js Dashboard)**:
   - Serves an interactive HTML5 page styled with a dark theme.
   - Dynamically pulls **Chart.js** from CDN (`https://cdn.jsdelivr.net/npm/chart.js`).
   - Renders live scrolling graphs for **Motor Speed (RPM)** and **Motor Temperature & Current**.
   - Contains dynamic metrics cards displaying real-time values and connection state.
2. **`GET /events` (Server-Sent Events Stream)**:
   - Sets `Content-Type: text/event-stream`.
   - Supports **multiple simultaneous clients** (`MAX_SSE_CLIENTS = 4`) multiplexed using non-blocking I/O (`zsock_poll`).
   - Streams live JSON data frames on every timer tick without client-side polling:
     ```json
     data: {"id":4,"rpm":2940,"temp":41.8,"volt":12.04,"curr":850,"uptime":4000,"clients":2}
     ```
   - Automatically detects client disconnections (tab closed, TCP FIN) and frees slots for new connections.

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

From inside `~/zephyr-playground/ESP32-C5-Zero`:

### 1. Build
```bash
# Pristine build:
uv run west build -p always -b esp32c5_devkitc/esp32c5/hpcore app

# Incremental build:
uv run west build app
```

### 2. Flash
Connect the ESP32-C5-Zero to your computer using a USB Type-C cable, then run:
```bash
uv run west flash
```

> **Note**: If the board does not automatically enter flash mode, hold the **BOOT** button, press and release the **RESET** button, and then release **BOOT**.

### 3. Monitor Serial Console
```bash
uv run west espressif monitor
```
*(Press `Ctrl + ]` to exit the monitor)*

Watch the serial output for Wi-Fi connection and assigned DHCP IP address:
```text
[wifi_mgr] Wi-Fi connected to SSID: ...
[wifi_mgr] IPv4 address assigned: 192.168.1.150
[http_srv] HTTP Server listening on port 80...
```

### 4. View Dashboard in Browser
Open your browser and navigate to:
```
http://192.168.1.150/
```
The page will establish an SSE stream to `/events` and display real-time live graphs!

