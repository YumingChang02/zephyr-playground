# Zephyr Multi-Board Workspace

This repository is a unified Zephyr workspace and Python environment located at `/home/markchang/zephyr`. It supports building and flashing across multiple hardware boards using a single shared Python virtual environment and Zephyr tree.

---

## Workspace Structure

```text
/home/markchang/zephyr/
├── pyproject.toml                  <-- Shared uv package configuration
├── uv.lock                         <-- Shared lockfile
├── .venv/                          <-- Shared Python virtual environment
├── README.md                       <-- Workspace & workflow guide
├── agents.md                       <-- Agent instructions & guidelines
├── docs/
│   └── startup.md                  <-- Fresh Arch Linux machine bring-up guide
├── zephyrproject/                  <-- Shared Zephyr tree (OS, HALs, modules)
│   ├── .west/
│   ├── modules/
│   └── zephyr/
│
├── ESP32-S3-GEEK/                  <-- Waveshare ESP32-S3-GEEK project
│   ├── app/
│   │   ├── CMakeLists.txt
│   │   ├── prj.conf
│   │   └── src/
│   │       └── main.c
│   └── build/                      <-- Build output artifacts
│
└── ESP32-C6-LCD-1.47/              <-- Future board project
```

---

## Environment & Tool Execution Rules

- **Always run commands via `uv run`**:
  Do not manually add `.venv/bin` to `PATH`. When you run `uv run west <command>` inside any project subfolder, `uv` automatically walks up the directory tree to find and use `/home/markchang/zephyr/.venv`.
- **`ZEPHYR_BASE` Environment Variable**:
  Configured in `~/.config/fish/conf.d/zephyr.fish`:
  ```fish
  set -gx ZEPHYR_BASE /home/markchang/zephyr/zephyrproject/zephyr
  ```
  In bash / sh:
  ```bash
  export ZEPHYR_BASE=/home/markchang/zephyr/zephyrproject/zephyr
  ```

---

## Workspace Initialization (`west init` & `west update`)

To initialize or reconstruct the shared `zephyrproject/` source tree from scratch:

1. **Sync Python Virtual Environment**:
   ```bash
   cd /home/markchang/zephyr
   uv sync
   ```

2. **Initialize West Workspace**:
   ```bash
   uv run west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.5.0 zephyrproject
   ```

3. **Fetch Modules & Binary Blobs**:
   ```bash
   cd zephyrproject
   uv run west update
   uv run west zephyr-export
   uv run west blobs fetch hal_espressif
   ```

*(See [docs/startup.md](file:///home/markchang/zephyr/docs/startup.md) for the complete OS-level Arch Linux bring-up guide).*

---

## Toolchain Management (`west sdk`)

Zephyr builds require the Zephyr SDK host tools (`dtc`, etc.) and the cross-compiler toolchain matching your board's MCU architecture.

### Check installed toolchains
```bash
uv run west sdk list
```

### Install toolchains for your targets
Instead of downloading the entire multi-gigabyte bundle, install only the toolchains needed:

| Target Chip Family | Architecture | Toolchain Name | Installation Command |
|---|---|---|---|
| **ESP32-S3** | Xtensa | `xtensa-espressif_esp32s3_zephyr-elf` | `uv run west sdk install -t xtensa-espressif_esp32s3_zephyr-elf` |
| **ESP32-C6 / ESP32-C3** | RISC-V | `riscv64-zephyr-elf` | `uv run west sdk install -t riscv64-zephyr-elf` |
| **ESP32 (Original)** | Xtensa | `xtensa-espressif_esp32_zephyr-elf` | `uv run west sdk install -t xtensa-espressif_esp32_zephyr-elf` |
| **nRF52 / STM32 / RP2040** | ARM Cortex-M | `arm-zephyr-eabi` | `uv run west sdk install -t arm-zephyr-eabi` |

*(To install all available toolchains: `uv run west sdk install`)*

---

## End-to-End Flow for Adding a New Board

Follow these steps whenever creating and working on a new board project:

### Step 1: Verify / Install Target Toolchain
Check if the toolchain for your board is installed:
```bash
uv run west sdk list
```
If missing, install it (example for ESP32-C6):
```bash
uv run west sdk install -t riscv64-zephyr-elf
```

### Step 2: Create the Project Directory Structure
Navigate to `/home/markchang/zephyr` and create the new project folder:
```bash
cd /home/markchang/zephyr
mkdir -p <BOARD_DIR>/app/src
```

### Step 3: Create Minimal Application Files

1. **`<BOARD_DIR>/app/CMakeLists.txt`**:
   ```cmake
   cmake_minimum_required(VERSION 3.20.0)
   find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
   project(my_board_app)

   target_sources(app PRIVATE src/main.c)
   ```

2. **`<BOARD_DIR>/app/prj.conf`**:
   ```ini
   CONFIG_PRINTK=y
   CONFIG_GPIO=y
   ```

3. **`<BOARD_DIR>/app/src/main.c`**:
   ```c
   #include <zephyr/kernel.h>
   #include <zephyr/sys/printk.h>

   int main(void)
   {
       printk("Hello from Zephyr on new board!\n");
       while (1) {
           k_sleep(K_MSEC(1000));
       }
       return 0;
   }
   ```

### Step 4: Identify Board Target Identifier
Search Zephyr for the official board target name:
```bash
uv run west boards | grep -i <chip_or_board>
```
For example:
- Waveshare ESP32-S3-GEEK: `esp32s3_geek/esp32s3/procpu`
- ESP32-C6 DevKit: `esp32c6_devkitc/esp32c6/hpcore`

### Step 5: Build
Inside `<BOARD_DIR>`:
- **First / Clean build (always use `-p always` or `-p auto`):**
  ```bash
  cd /home/markchang/zephyr/<BOARD_DIR>
  uv run west build -p always -b <board_target> app
  ```
- **Subsequent incremental builds:**
  ```bash
  uv run west build app
  ```

### Step 6: Flash & Serial Monitor
1. **Flash firmware:**
   ```bash
   uv run west flash
   ```
2. **Open serial monitor (Espressif chips):**
   ```bash
   uv run west espressif monitor
   ```
   *(Or generic monitor: `picocom -b 115200 /dev/ttyACM0`)*

---

## Troubleshooting Common Errors

### 1. `Could not find a package configuration file provided by "Zephyr-sdk"`
- **Cause**: Zephyr SDK is not installed or not registered in CMake package registry (`~/.cmake/packages/Zephyr-sdk`).
- **Fix**: Run:
  ```bash
  uv run west sdk install -t <toolchain_name>
  ```
  This downloads host tools (`dtc`), downloads the compiler, and registers the SDK with CMake.

### 2. `ninja: error: loading 'build.ninja': No such file or directory`
- **Cause**: A previous CMake configuration failed, leaving an incomplete `build/` directory. Running `west build` again assumes an incremental build and immediately calls `ninja`, which cannot find the missing `build.ninja`.
- **Fix**: Force a pristine rebuild so CMake re-generates the build system:
  ```bash
  uv run west build -p always -b <board_target> app
  ```
  *(Or manually delete the build folder: `rm -rf build`)*

### 3. Missing Binary Blobs (Espressif WiFi / Bluetooth)
- **Cause**: Some Espressif hardware features require proprietary binary blobs not checked into git.
- **Fix**: Run:
  ```bash
  uv run west blobs fetch hal_espressif
  ```
