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

