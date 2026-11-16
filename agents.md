# Agent Guidelines & Environment Setup

## Python & Package Management
- **Always use `uv` directly (`uv run ...`)** to execute commands and tools. Never mutate the user's `PATH` or add `.venv/bin` to shell paths (`fish_add_path` or `export PATH`).
- The shared Python workspace is located at the root `~/zephyr-playground/` with `pyproject.toml`, `uv.lock`, and `.venv`.
- To sync dependencies:
  ```bash
  cd ~/zephyr-playground
  uv sync
  ```
- To run commands inside any project subfolder (e.g. `ESP32-S3-GEEK`, `ESP32-C6-LCD-1.47`):
  ```bash
  uv run west <command>
  ```
  `uv` automatically discovers the parent workspace (`~/zephyr-playground`) and its `.venv`.

## Zephyr Development Workflow
- **Workspace Layout**:
  - Shared Zephyr tree: `~/zephyr-playground/zephyrproject`
  - Shared Python virtualenv & tools: `~/zephyr-playground/.venv`
  - Board / App directories: `~/zephyr-playground/ESP32-S3-GEEK`, `~/zephyr-playground/ESP32-C5-Zero`, etc.
- **Environment**:
  - `ZEPHYR_BASE` is set in `~/.config/fish/conf.d/zephyr.fish` (`set -gx ZEPHYR_BASE $HOME/zephyr-playground/zephyrproject/zephyr`).
- **Toolchains (`west sdk`)**:
  - Check installed: `uv run west sdk list`
  - Install specific toolchain: `uv run west sdk install -t <toolchain>` (e.g., `xtensa-espressif_esp32s3_zephyr-elf`, `riscv64-zephyr-elf`)
- **Building**:
  - Always use `-p always` or `-p auto` on the initial build of a board to avoid `build.ninja` missing errors from incomplete CMake setups:
    ```bash
    uv run west build -p always -b <board_target> app
    ```
  - Subsequent incremental builds:
    ```bash
    uv run west build app
    ```
- **Flashing & Monitoring**:
  ```bash
  uv run west flash
  uv run west espressif monitor
  ```
