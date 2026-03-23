# 🚀 Setup Guide - STM32-Control-Coche-Marcos

This guide will help you set up the development environment and compile the STM32 firmware.

## Prerequisites

### Required Software
- **STM32CubeIDE** (version 1.14.0 or later, with CubeMX ≥ 6.17.0) - [Download here](https://www.st.com/en/development-tools/stm32cubeide.html)
- **Git** - For cloning the repository
- **arm-none-eabi-gcc** *(optional, for command-line builds)*:
  - Linux: `sudo apt-get install gcc-arm-none-eabi`
  - macOS: `brew install arm-none-eabi-gcc`
  - Windows: install via [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

### Hardware
- STM32G474RE microcontroller (NUCLEO-G474RE board recommended)
- ST-Link programmer/debugger (included in NUCLEO board)
- USB cable

## Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/florinzgz/STM32-Control-Coche-Marcos.git
cd STM32-Control-Coche-Marcos
```

### 2. Generate Drivers (HAL + CMSIS)

The `Drivers/` folder is **not included** in the repository (it's gitignored). You must generate it before building.

**Option A — Setup script (recommended, no CubeMX needed):**
```bash
# Linux / macOS
./setup_drivers.sh

# Windows
setup_drivers.bat
```
The script downloads the STM32G4xx HAL Driver v1.2.4, CMSIS headers, and device files from GitHub, then runs `fix_build.sh` automatically.

**Option B — STM32CubeMX code generation:**
1. Open **STM32CubeIDE**
2. Import the project (see step 3 below)
3. Open `STM32-Control-Coche-Marcos.ioc`
4. Click **Project → Generate Code** (Alt+K)
5. Run `./fix_build.sh` (Linux/macOS) or `fix_build.bat` (Windows) to fix any `.cproject` changes

### 3. Build the Project

#### Using STM32CubeIDE:
1. **File → Open Projects from File System...**
2. Click **Directory...** and select the **root repository folder** (the one containing `.project` and `.cproject`)
3. Click **Finish**
4. If a `Debug/` folder already exists, **delete it first**
5. Right-click on the project → **Refresh** (F5)
6. **Project → Clean...** → select the project → **Clean**
7. Right-click on project → **Build Project** (or press **Ctrl+B**)

> ⚠️ **Common mistake**: Do not import a subfolder — import the root folder that contains `.project`.

#### Using Command Line (requires arm-none-eabi-gcc):
```bash
make clean
make all
```
Expected output: firmware binary at `build/STM32G474RE.elf` (~53 KB text).

### 4. Flash to STM32

1. Connect your STM32G474RE board via USB
2. In STM32CubeIDE: **Run → Debug** (F11)
3. Or **Run → Run** (Ctrl+F11) to flash without debugging

## Project Structure

```
STM32-Control-Coche-Marcos/
├── Core/                          # Application code
│   ├── Inc/                       # Header files (main.h, motor_control.h, etc.)
│   ├── Src/                       # Source files (main.c, motor_control.c, etc.)
│   └── Startup/                   # Assembly startup code
├── Drivers/                       # STM32 HAL + CMSIS (generated, gitignored)
│   ├── STM32G4xx_HAL_Driver/     # HAL driver sources and headers
│   └── CMSIS/                     # ARM CMSIS core + STM32G4xx device headers
├── docs/                          # Documentation
├── analysis_artifacts/            # Static analysis tools and stubs
├── Makefile                       # Command-line build script
├── fix_build.sh / .bat           # Automatic .cproject repair tool
├── setup_drivers.sh / .bat       # HAL driver download tool
├── validate_cproject.sh / .bat   # Project health check (read-only)
├── STM32-Control-Coche-Marcos.ioc # CubeMX configuration
├── STM32G474RETX_FLASH.ld        # Linker script
├── .project                       # Eclipse/CubeIDE project descriptor
└── .cproject                      # CDT build configuration
```

## Troubleshooting

### "Cannot find HAL headers"
- **Solution**: The `Drivers/` folder must be generated. Run `./setup_drivers.sh` (Linux) / `setup_drivers.bat` (Windows), or open the `.ioc` file in STM32CubeIDE and click **Project → Generate Code** (Alt+K).

### "undefined reference to HAL_XXX"
- **Solution**: Make sure the `Drivers/` folder has been generated (see above) and the project includes it in the build path.

### "No ST-Link detected"
- **Solution**: 
  1. Check USB cable connection
  2. Install ST-Link drivers (included with STM32CubeIDE)
  3. Try a different USB port

### Build errors after cloning
- **Solution**: Generate the `Drivers/` folder by running `./setup_drivers.sh` or using CubeMX code generation on the `.ioc` file (see step 2 above).

### Circular dependency / "Building file:" empty / `arm-none-eabi-gcc ""`
If you see errors like:
```
make: Circular Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.o <- ...
Building file:
arm-none-eabi-gcc "" ...
```
- **Cause**: The `.cproject` file contains element IDs identical to their `superClass` template IDs. Eclipse CDT confuses instances with templates and generates broken `subdir.mk` rules. This happens when **STM32CubeMX overwrites `.cproject`** during code generation.
- **Quick fix** — run the helper script:
  ```bash
  # Linux / macOS
  ./fix_build.sh

  # Windows
  fix_build.bat
  ```
  The script removes the stale `Debug/` directory and fixes broken `.cproject` element IDs.
- **Manual fix**:
  1. Restore the correct `.cproject`: `git checkout .cproject`
  2. **Delete the `Debug/` folder**
  3. In STM32CubeIDE: right-click project → **Refresh** (F5)
  4. **Project → Clean...** → select project → **Clean**
  5. **Build** again (**Ctrl+B**)
- **Prevention**: After every CubeMX code generation (Alt+K), run `./fix_build.sh` before building.

### FDCAN init errors with newer HAL versions
- **Note**: The project uses HAL Driver v1.2.4, which simplified the FDCAN_InitTypeDef by removing per-element Message RAM fields (MessageRAMOffset, RxFifo0ElmtsNbr, etc.). The HAL now configures Message RAM automatically. If you see FDCAN compile errors, make sure you're using HAL v1.2.4 (provided by `setup_drivers.sh`).

### ESP32-S3: "Could not open COMx" / PermissionError
- **Cause**: The serial port is in use by another program, or the board is not in download mode.
- **Solution**:
  1. Close any serial monitor connected to the ESP32 port.
  2. Enter download mode: hold **BOOT**, press **RESET**, release **BOOT**.
  3. Run `pio run -t upload` from the `esp32/` directory.
  4. On Linux, add your user to the `dialout` group: `sudo usermod -aG dialout $USER`

## Validation Tools

Run these to check project health before building:
```bash
# Read-only health check (11 tests)
./validate_cproject.sh

# Auto-fix common issues
./fix_build.sh
```

## Next Steps

1. ✅ Import the project into STM32CubeIDE
2. ✅ Build the project
3. ✅ Flash to your STM32 board
4. 📖 Read the documentation in `docs/` folder

## Support

For issues or questions:
- Open an issue on GitHub
- Check the documentation in the `docs/` folder
- Run `./validate_cproject.sh` to diagnose build issues

## License

MIT License - See [LICENSE](LICENSE) file for details

---

**Author**: florinzgz  
**Project**: STM32-Control-Coche-Marcos  
**Version**: 1.0.0
