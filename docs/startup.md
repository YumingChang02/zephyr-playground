# Environment Setup & Bring-Up Guide (Arch Linux)

This guide documents the complete end-to-end process to bootstrap a fresh **Arch Linux** machine into a fully functional Zephyr RTOS development environment for multi-board projects (ESP32-S3, ESP32-C6, etc.), supporting both **Fish** and **Bash** shells.

---

## 1. Arch Linux System Dependencies

Update the system and install required base development tools, build systems, and hardware flashing utilities:

```bash
sudo pacman -Syu --needed \
    base-devel \
    git \
    cmake \
    ninja \
    gperf \
    ccache \
    dfu-util \
    dtc \
    wget \
    curl \
    python \
    xz \
    file \
    make \
    gcc \
    picocom \
    uv
```

*(Note: If `uv` is not yet available in your configured repos, install it via `curl -LsSf https://astral.sh/uv/install.sh | sh`)*

### Serial Port Permissions (udev / user groups)
In Arch Linux, access to serial ports (`/dev/ttyUSB*`, `/dev/ttyACM*`) belongs to the `uucp` group:

```bash
sudo usermod -aG uucp $USER
```
> **Note**: Log out and log back in (or run `newgrp uucp`) for group permissions to take effect.

---

## 2. Workspace Setup & Python Environment (`uv`)

Navigate to your home directory (or chosen workspace parent) and clone or enter the project repository:

```bash
cd ~
# If cloning fresh:
# git clone <your-repo-url> zephyr
cd ~/zephyr-playground
```

Synchronize the Python virtual environment using `uv`. This reads `pyproject.toml` and `uv.lock` to install the exact pinned versions of `west`, `esptool`, and all Zephyr Python dependencies:

```bash
uv sync
```

---

## 3. Shell Environment Configuration (`ZEPHYR_BASE` & `ZEPHYR_SDK_INSTALL_DIR`)

Zephyr tools require:
- `ZEPHYR_BASE` pointing to the Zephyr kernel source directory (`~/zephyr-playground/zephyrproject/zephyr`).
- `ZEPHYR_SDK_INSTALL_DIR` pointing to the workspace-local Zephyr SDK (`~/zephyr-playground/zephyr-sdk`).

### Option A: Fish Shell (Recommended)

Create an autoloaded fish configuration file in `~/.config/fish/conf.d/zephyr.fish`:

```fish
mkdir -p ~/.config/fish/conf.d
cat << 'EOF' > ~/.config/fish/conf.d/zephyr.fish
set -gx ZEPHYR_BASE ~/zephyr-playground/zephyrproject/zephyr
set -gx ZEPHYR_SDK_INSTALL_DIR ~/zephyr-playground/zephyr-sdk
EOF
```

Reload fish configuration or open a new terminal:
```fish
source ~/.config/fish/conf.d/zephyr.fish
```

### Option B: Bash Shell

Append the variables to your `~/.bashrc`:

```bash
cat << 'EOF' >> ~/.bashrc

# Zephyr RTOS Environment
export ZEPHYR_BASE="$HOME/zephyr/zephyrproject/zephyr"
export ZEPHYR_SDK_INSTALL_DIR="$HOME/zephyr/zephyr-sdk"
EOF
```

Reload bash configuration or open a new terminal:
```bash
source ~/.bashrc
```

---

## 4. Initialize Zephyr Workspace (`west`)

Pull the Zephyr OS repository, HALs, and external modules:

```bash
cd ~/zephyr-playground

# Initialize west manifest pointing to zephyrproject
# If zephyrproject folder already exists with a .west manifest:
cd ~/zephyr-playground/zephyrproject
uv run west update
uv run west zephyr-export

# Fetch proprietary binary blobs (e.g. Espressif Wi-Fi / Bluetooth firmware)
uv run west blobs fetch hal_espressif
```

> **If initializing completely from scratch without existing `zephyrproject`:**
> ```bash
> cd ~/zephyr-playground
> uv run west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.5.0 zephyrproject
> cd zephyrproject
> uv run west update
> uv run west zephyr-export
> uv run west blobs fetch hal_espressif
> ```

---

## 5. Install Zephyr SDK & Toolchains

Zephyr uses `west sdk install` to fetch host tools (`dtc`, OpenOCD) and architecture-specific GCC compilers directly from official releases, registering the SDK with CMake package registries. In this workspace, the SDK is installed directly under `~/zephyr-playground/zephyr-sdk`.

### Check currently installed SDKs & toolchains
```bash
uv run west sdk list
```

### Install Toolchains for Target Chips
To avoid downloading gigabytes of unused architectures, install only the toolchains you intend to target into the workspace-local `zephyr-sdk`:

- **For ESP32-S3 (Xtensa)**:
  ```bash
  uv run west sdk install -d ~/zephyr-playground/zephyr-sdk -t xtensa-espressif_esp32s3_zephyr-elf
  ```
- **For ESP32-C5 / ESP32-C6 / ESP32-C3 (RISC-V)**:
  ```bash
  uv run west sdk install -d ~/zephyr-playground/zephyr-sdk -t riscv64-zephyr-elf
  ```
- **For ARM Cortex-M (nRF52, STM32, RP2040)**:
  ```bash
  uv run west sdk install -d ~/zephyr-playground/zephyr-sdk -t arm-zephyr-eabi
  ```
- **Multi-target install in one command (e.g., S3 and C5/C6)**:
  ```bash
  uv run west sdk install -d ~/zephyr-playground/zephyr-sdk -t xtensa-espressif_esp32s3_zephyr-elf riscv64-zephyr-elf
  ```

### Install Hardware Debug / JTAG udev Rules (Optional but Recommended)
Install OpenOCD udev rules so JTAG debuggers and hardware probes can be accessed without `sudo`:

```bash
sudo cp ~/zephyr-playground/zephyr-sdk/hosttools/sysroots/x86_64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
```

---

## 6. Verify with a Test Build

Test your setup by building the ESP32-S3-GEEK application:

```bash
cd ~/zephyr-playground/ESP32-S3-GEEK

# Always use -p always (pristine) on first build
uv run west build -p always -b esp32s3_geek/esp32s3/procpu app
```

Expected output:
```text
-- west build: generating a build system
-- Found toolchain: zephyr 1.0.1 (~/zephyr-playground/zephyr-sdk)
...
[262/262] Linking C executable zephyr/zephyr.elf
Merged 10 ELF sections.
Successfully created ESP32-S3 image.
```

Generated binaries will be located at:
- `build/zephyr/zephyr.bin`
- `build/zephyr/zephyr.elf`

---

## 7. Flashing & Serial Monitor

Plug in your board via USB-C:

1. **Flash firmware:**
   ```bash
   uv run west flash
   ```
2. **Monitor serial console (Espressif chips):**
   ```bash
   uv run west espressif monitor
   ```
   *(Exit monitor with `Ctrl + ]`)*
3. **Alternative generic monitor:**
   ```bash
   picocom -b 115200 /dev/ttyACM0
   ```
   *(Exit picocom with `Ctrl + A` followed by `Ctrl + X`)*

---

## 8. Common Gotchas & Troubleshooting

| Error | Root Cause | Solution |
|---|---|---|
| `Could not find a package configuration file provided by "Zephyr-sdk"` | Zephyr SDK is not installed or CMake registry is empty. | Run `uv run west sdk install -t <toolchain>` to install and register SDK. |
| `ninja: error: loading 'build.ninja': No such file or directory` | Previous CMake configuration failed midway, leaving empty `build/`. | Run build with `-p always` (or delete `build/` folder). |
| `Permission denied: '/dev/ttyACM0'` | User not member of serial group in Arch. | Run `sudo usermod -aG uucp $USER` and re-login. |
| `Cannot load extension commands` | Running `west` outside workspace without `ZEPHYR_BASE`. | Ensure `ZEPHYR_BASE` is exported in your shell config (`.bashrc` or `zephyr.fish`). |
| Missing Wi-Fi / Bluetooth PHY blobs | Proprietary blobs not pulled into HAL module. | Run `uv run west blobs fetch hal_espressif`. |

