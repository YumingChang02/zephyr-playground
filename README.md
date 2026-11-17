# Zephyr Multi-Board Workspace

This repository is a unified Zephyr workspace and Python environment located at `~/zephyr-playground`. It supports building and flashing across multiple hardware boards using a single shared Python virtual environment and Zephyr tree.

---

## Workspace Structure

```text
~/zephyr-playground/
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
├── zephyr-sdk/                     <-- Project-local Zephyr SDK & toolchains
│   ├── cmake/
│   ├── gnu/
│   ├── hosttools/
│   └── setup.sh
│
├── ESP32-S3-GEEK/                  <-- Waveshare ESP32-S3-GEEK project (Xtensa)
│   ├── app/
│   │   ├── CMakeLists.txt
│   │   ├── prj.conf
│   │   ├── app.overlay
│   │   └── src/
│   │       └── main.c
│   └── build/                      <-- Build output artifacts
│
├── ESP32-C5-Zero/                  <-- Waveshare ESP32-C5-Zero project (RISC-V)
│   ├── app/
│   │   ├── CMakeLists.txt
│   │   ├── prj.conf
│   │   ├── app.overlay
│   │   └── src/
│   │       └── main.c
│   └── build/                      <-- Build output artifacts
│
└── ESP32-C6-LCD-147/               <-- Waveshare ESP32-C6-LCD-1.47 project (RISC-V)
    ├── app/
    │   ├── CMakeLists.txt
    │   ├── prj.conf
    │   ├── app.overlay
    │   └── src/
    │       ├── main.c
    │       └── telemetry.h
    └── build/                      <-- Build output artifacts
```

---

## Environment & Tool Execution Rules

- **Always run commands via `uv run`**:
  Do not manually add `.venv/bin` to `PATH`. When you run `uv run west <command>` inside any project subfolder, `uv` automatically walks up the directory tree to find and use `~/zephyr-playground/.venv`.
- **Environment Variables (`ZEPHYR_BASE` & `ZEPHYR_SDK_INSTALL_DIR`)**:
  Configured in `~/.config/fish/conf.d/zephyr.fish`:
  ```fish
  set -gx ZEPHYR_BASE $HOME/zephyr-playground/zephyrproject/zephyr
  set -gx ZEPHYR_SDK_INSTALL_DIR $HOME/zephyr-playground/zephyr-sdk
  ```
  In bash / sh (`~/.bashrc`):
  ```bash
  export ZEPHYR_BASE="$HOME/zephyr-playground/zephyrproject/zephyr"
  export ZEPHYR_SDK_INSTALL_DIR="$HOME/zephyr-playground/zephyr-sdk"
  ```

---

## Workspace Initialization (`west init`, `west update`, & SDK setup)

To initialize or reconstruct the shared `zephyrproject/` source tree and local `zephyr-sdk` from scratch:

1. **Sync Python Virtual Environment**:
   ```bash
   cd ~/zephyr-playground
   uv sync
   ```

2. **Initialize West Workspace**:
   ```bash
   uv run west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.5.0 zephyrproject
   ```

3. **Fetch Modules & Binary Blobs**:
   ```bash
   cd ~/zephyr-playground/zephyrproject
   uv run west update
   uv run west zephyr-export
   uv run west blobs fetch hal_espressif
   ```

4. **Install Local Zephyr SDK & Cross Compilers**:
   Install the Zephyr SDK host tools (`dtc`, etc.) and desired compilers directly into `~/zephyr-playground/zephyr-sdk`:
   ```bash
   cd ~/zephyr-playground
   uv run west sdk install -d ~/zephyr-playground/zephyr-sdk -t xtensa-espressif_esp32s3_zephyr-elf riscv64-zephyr-elf
   ```
   *(This automatically extracts host tools, downloads the target toolchains, and registers the CMake package registry at `~/.cmake/packages/Zephyr-sdk`).*

*(See [docs/startup.md](file:///home/markchang/zephyr-playground/docs/startup.md) for the complete OS-level Arch Linux bring-up guide).*

---

## Toolchain Management (`west sdk`)

Zephyr builds require the Zephyr SDK host tools (`dtc`, OpenOCD) and architecture-specific GCC toolchains. In this repository, the SDK is kept self-contained under `~/zephyr-playground/zephyr-sdk` and tracked via the `ZEPHYR_SDK_INSTALL_DIR` environment variable.

### Check installed toolchains
```bash
uv run west sdk list
```
*(When `ZEPHYR_SDK_INSTALL_DIR` is set, `west sdk list` will display all toolchains installed in `~/zephyr-playground/zephyr-sdk`).*

### Install toolchains into local SDK
Install only the toolchains needed for your target chips into `~/zephyr-playground/zephyr-sdk`:

| Target Chip Family | Architecture | Toolchain Name | Installation Command |
|---|---|---|---|
| **ESP32-S3** | Xtensa | `xtensa-espressif_esp32s3_zephyr-elf` | `uv run west sdk install -d ~/zephyr-playground/zephyr-sdk -t xtensa-espressif_esp32s3_zephyr-elf` |
| **ESP32-C5 / C6 / C3** | RISC-V | `riscv64-zephyr-elf` | `uv run west sdk install -d ~/zephyr-playground/zephyr-sdk -t riscv64-zephyr-elf` |
| **ESP32 (Original)** | Xtensa | `xtensa-espressif_esp32_zephyr-elf` | `uv run west sdk install -d ~/zephyr-playground/zephyr-sdk -t xtensa-espressif_esp32_zephyr-elf` |
| **nRF52 / STM32 / RP2040** | ARM Cortex-M | `arm-zephyr-eabi` | `uv run west sdk install -d ~/zephyr-playground/zephyr-sdk -t arm-zephyr-eabi` |

Alternatively, you can run the bundled setup script inside `zephyr-sdk/` directly:
```bash
cd ~/zephyr-playground/zephyr-sdk
./setup.sh -t <toolchain_name> -c
```

---

## End-to-End Flow for Adding a New Board

Follow these steps whenever creating and working on a new board project:

### Step 1: Verify / Install Target Toolchain
Check if the toolchain for your board is installed:
```bash
uv run west sdk list
```
If missing, install it (example for ESP32-C6 / ESP32-C5):
```bash
uv run west sdk install -d ~/zephyr-playground/zephyr-sdk -t riscv64-zephyr-elf
```

### Step 2: Create the Project Directory Structure
Navigate to `~/zephyr-playground` and create the new project folder:
```bash
cd ~/zephyr-playground
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
In Zephyr Hardware Model v2 (Zephyr 3.7+), board targets use the format `<board>/<soc>/<cpu>` (qualifiers).

By default, `west boards` only outputs the base board names (e.g. `esp32s3_geek`). To get the **full target identifiers** needed for `-b`:

1. **List all valid target combinations (`-a` / `--all-targets`):**
   ```bash
   uv run west boards -a | grep -i <chip_or_board>
   ```
   *Example:*
   ```bash
   uv run west boards -a | grep -i esp32s3_geek
   # Output:
   # esp32s3_geek/esp32s3/procpu
   # esp32s3_geek/esp32s3/appcpu
   ```

2. **Inspect target qualifiers for a specific board:**
   ```bash
   uv run west boards -a --board <board_name>
   # or format with qualifiers:
   uv run west boards -f "{name}: {qualifiers}" -n <board_name>
   ```

Common ESP32 board targets:
- Waveshare ESP32-S3-GEEK: `esp32s3_geek/esp32s3/procpu`
- ESP32-C6 DevKit: `esp32c6_devkitc/esp32c6/hpcore`

### Step 5: Build
Inside `<BOARD_DIR>`:
- **First / Clean build (always use `-p always` or `-p auto`):**
  ```bash
  cd ~/zephyr-playground/<BOARD_DIR>
  uv run west build -p always -b <board_target> app
  ```
- **Subsequent incremental builds:**
  ```bash
  uv run west build app
  ```
- **Fast incremental builds (source code changes only):**
  ```bash
  uv run ninja -C build
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

## Ninja: Fast Incremental Builds & Inspection

### What is Ninja?
**Ninja** is a lightweight, ultra-fast build tool designed specifically to execute build commands generated by CMake. 
In Zephyr, `west build` is a high-level Python wrapper that invokes CMake to configure the project, which in turn generates `build/build.ninja`. When `west build` compiles your code, it actually runs `ninja` behind the scenes.

Because `ninja` is installed inside the shared Python virtual environment (`.venv`), you invoke it using:
```bash
uv run ninja -C <BOARD_DIR>/build
```

---

### When to Use Ninja vs. `west build`

| Scenario | Recommended Command | Why |
| :--- | :--- | :--- |
| **Editing C/C++ code (`.c`, `.h`)** | `uv run ninja -C build` | **Sub-second rebuilds**: Skips `west` Python overhead and Zephyr module dependency checking; immediately invokes the compiler on only the modified files. |
| **First build of a project** | `uv run west build -p always -b <target> app` | Configures the board, parses Kconfig, and runs CMake to generate the initial `build.ninja`. |
| **Modifying `prj.conf` (Kconfig)** | `uv run west build app` | Requires Kconfig parser to re-evaluate symbols and update `autoconf.h`. |
| **Modifying `*.overlay` (Devicetree)** | `uv run west build app` | Requires `dtc` and Zephyr devicetree scripts to regenerate `devicetree_generated.h`. |
| **Modifying `CMakeLists.txt`** | `uv run west build app` | Requires CMake configuration step to re-run. |
| **Compiler diagnostic / flags debug** | `uv run ninja -C build -v` | Shows the exact compiler invocation, flags (`-I`, `-D`, `-O`), and linker commands. |

---

### Useful Ninja Commands

All commands can be run from the board directory or workspace root:

```bash
# 1. Fast incremental build (from board directory)
uv run ninja -C build

# 2. Verbose build (inspect exact gcc flags, includes, and defines)
uv run ninja -C build -v

# 3. Memory & footprint reports
uv run ninja -C build rom_report    # Detailed breakdown of Flash/ROM usage per symbol
uv run ninja -C build ram_report    # Detailed breakdown of SRAM/RAM usage per symbol

# 4. Clean compiled objects (keeps CMake cache and configuration intact)
uv run ninja -C build clean

# 5. List available build targets
uv run ninja -C build help
```

---

## Troubleshooting Common Errors

### 1. `Could not find a package configuration file provided by "Zephyr-sdk"`
- **Cause**: Zephyr SDK is not installed or not registered in CMake package registry (`~/.cmake/packages/Zephyr-sdk`), or `ZEPHYR_SDK_INSTALL_DIR` is not set.
- **Fix**: Run:
  ```bash
  # Ensure ZEPHYR_SDK_INSTALL_DIR is exported
  export ZEPHYR_SDK_INSTALL_DIR="$HOME/zephyr-playground/zephyr-sdk"

  # If not yet installed, install host tools & target compiler into workspace
  uv run west sdk install -d ~/zephyr-playground/zephyr-sdk -t <toolchain_name>

  # Or re-register existing SDK with CMake package registry:
  cd ~/zephyr-playground/zephyr-sdk && ./setup.sh -c
  ```

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
