# 🔧 Build Instructions — STM32-Control-Coche-Marcos

Complete step-by-step guide for building the firmware from a fresh clone.

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Clone and Setup](#2-clone-and-setup)
3. [Build with Command Line](#3-build-with-command-line)
4. [Build with STM32CubeIDE](#4-build-with-stm32cubeide)
5. [Verified Build Output](#5-verified-build-output)
6. [Common Mistakes](#6-common-mistakes)
7. [Troubleshooting Reference](#7-troubleshooting-reference)

---

## 1. Prerequisites

### For command-line builds (Linux/macOS)

| Tool | Version | Install |
|------|---------|---------|
| `arm-none-eabi-gcc` | ≥ 10.0 | `sudo apt install gcc-arm-none-eabi` (Ubuntu/Debian) |
| `make` | ≥ 4.0 | `sudo apt install make` |
| `git` | ≥ 2.0 | `sudo apt install git` |
| `curl` or `wget` | any | Usually pre-installed |
| `unzip` | any | `sudo apt install unzip` |

### For STM32CubeIDE builds

| Tool | Version | Notes |
|------|---------|-------|
| **STM32CubeIDE** | ≥ 1.14.0 | [Download](https://www.st.com/en/development-tools/stm32cubeide.html) |
| **STM32CubeMX** | ≥ 6.17.0 | Bundled with STM32CubeIDE ≥ 1.14.0 |
| **STM32Cube FW_G4** | ≥ 1.6.2 | Installed via CubeIDE's package manager |
| `com.st.stm32cube.common.mx` plugin | any | Bundled with CubeIDE (opens .ioc files) |

> **Check your CubeMX version**: In STM32CubeIDE, go to **Help → About STM32CubeIDE** and verify the CubeMX version is ≥ 6.17.0. If not, update via **Help → Check for Updates**.

---

## 2. Clone and Setup

```bash
git clone https://github.com/florinzgz/STM32-Control-Coche-Marcos.git
cd STM32-Control-Coche-Marcos
```

### Install HAL Drivers (required before any build)

The `Drivers/` folder is gitignored and must be generated locally:

```bash
# Linux / macOS
./setup_drivers.sh
# Answer 'y' when prompted

# Windows
setup_drivers.bat
```

This downloads three packages from GitHub:
1. **STM32CubeG4 v1.6.2** — CMSIS core headers
2. **stm32g4xx_hal_driver v1.2.4** — HAL driver sources
3. **cmsis_device_g4 v1.2.4** — STM32G4xx device headers

After completion, verify:
```bash
ls Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c    # HAL source
ls Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal.h    # HAL header
ls Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g4xx.h  # Device header
ls Drivers/CMSIS/Include/core_cm4.h                     # ARM core header
```

---

## 3. Build with Command Line

```bash
# Full clean build
make clean
make all

# Expected output:
#   text    data     bss     dec     hex filename
#  53552      72    7696   61320    ef88 build/STM32G474RE.elf
```

Output files:
- `build/STM32G474RE.elf` — ELF binary (for debugging)
- `build/STM32G474RE.hex` — Intel HEX (for flashing)
- `build/STM32G474RE.bin` — Raw binary

### Run validation (optional)

```bash
# Read-only health check
./validate_cproject.sh

# Auto-fix .cproject issues
./fix_build.sh
```

---

## 4. Build with STM32CubeIDE

### Step 1: Import the project

1. Open **STM32CubeIDE**
2. **File → Open Projects from File System...**
3. Click **Directory...** and navigate to the **root repository folder**
   - ⚠️ This is the folder containing `.project` and `.cproject`
   - ⚠️ Do NOT import `Core/` or any subfolder
4. Ensure `STM32-Control-Coche-Marcos` appears in the project list
5. Click **Finish**

### Step 2: Generate or install drivers

**Option A** — If you already ran `./setup_drivers.sh`:
- Right-click the project → **Refresh** (F5)
- The `Drivers/` folder should appear in the project explorer

**Option B** — Generate via CubeMX:
1. Double-click `STM32-Control-Coche-Marcos.ioc` in the project explorer
2. The CubeMX editor should open (requires `com.st.stm32cube.common.mx` plugin)
3. Click **Project → Generate Code** (Alt+K)
4. **Important**: Run `./fix_build.sh` after code generation to repair `.cproject`

### Step 3: Clean and build

1. Delete `Debug/` folder if it exists (right-click → Delete)
2. **Project → Clean...** → select the project → **Clean**
3. **Project → Build Project** (Ctrl+B)
4. Build should complete with 0 errors, 0 warnings

### Step 4: Flash and debug

1. Connect NUCLEO-G474RE board via USB
2. **Run → Debug** (F11) — flashes and starts debugger
3. Or **Run → Run** (Ctrl+F11) — flash only

---

## 5. Verified Build Output

The following was confirmed on a CI/build environment:

| Metric | Value |
|--------|-------|
| **Compiler** | arm-none-eabi-gcc 13.2.1 |
| **Optimization** | -O2 |
| **Warnings** | -Wall -Wextra -Werror (zero warnings) |
| **Text (code)** | 53,552 bytes |
| **Data** | 72 bytes |
| **BSS** | 7,696 bytes |
| **Total** | 61,320 bytes |
| **Flash usage** | ~10.2% of 512 KB |
| **HAL Driver** | v1.2.4 (STM32G4xx) |
| **CMSIS** | v1.2.4 (cmsis_device_g4) |
| **Unit tests** | 419 tests, all passing |
| **Static analysis** | cppcheck + flawfinder — clean |
| **Validation** | 11/11 checks pass |

---

## 6. Common Mistakes

### ❌ Importing the wrong folder
**Symptom**: "No projects found" or project doesn't build.
**Fix**: Import the ROOT folder (the one with `.project` and `.cproject`), not `Core/` or any subfolder.

### ❌ Missing Drivers/ folder
**Symptom**: "Cannot find stm32g4xx_hal.h" or "No such file or directory".
**Fix**: Run `./setup_drivers.sh` or generate code from the `.ioc` file.

### ❌ Outdated CubeMX version
**Symptom**: `.ioc` file opens but generates incompatible code.
**Fix**: Update STM32CubeIDE to get CubeMX ≥ 6.17.0. The `.ioc` specifies `MxCube.Version=6.17.0`.

### ❌ Missing `com.st.stm32cube.common.mx.startCubeMx` plugin
**Symptom**: Double-clicking `.ioc` shows "No editor found" or "Plugin not found".
**Fix**: This plugin is bundled with STM32CubeIDE. If missing, reinstall STM32CubeIDE.

### ❌ Stale Debug/ directory
**Symptom**: Circular dependency errors, `arm-none-eabi-gcc ""`.
**Fix**: Delete the `Debug/` directory and run `./fix_build.sh`.

### ❌ CubeMX overwrites .cproject
**Symptom**: Build breaks after clicking "Generate Code" (Alt+K).
**Fix**: Always run `./fix_build.sh` after CubeMX code generation.

### ❌ Using old HAL version with new FDCAN API
**Symptom**: `FDCAN_InitTypeDef has no member named MessageRAMOffset`.
**Fix**: Use HAL v1.2.4 (provided by `setup_drivers.sh`). This version simplified FDCAN init — Message RAM is configured automatically.

---

## 7. Troubleshooting Reference

| Error | Cause | Fix |
|-------|-------|-----|
| `arm-none-eabi-gcc: command not found` | Toolchain not installed | Install gcc-arm-none-eabi |
| `No rule to make target 'build/adc.o'` | Missing peripheral stubs | Pull latest code (stubs added) |
| `Cannot find stm32g4xx_hal.h` | Drivers/ not generated | Run `./setup_drivers.sh` |
| `Circular dependency dropped` | Broken .cproject element IDs | Run `./fix_build.sh` |
| `FDCAN_InitTypeDef has no member` | HAL version mismatch | Use HAL v1.2.4 via setup script |
| CubeMX "Plugin not found" | STM32CubeIDE incomplete | Reinstall STM32CubeIDE |
| `.ioc` file version mismatch | CubeMX < 6.17.0 | Update STM32CubeIDE |

---

## Build Confirmation Checklist

After a successful build, verify:

- [ ] `build/STM32G474RE.elf` exists
- [ ] No `arm-none-eabi-gcc ""` errors
- [ ] No circular dependency warnings
- [ ] All HAL sources compiled (20 .o files from Drivers/)
- [ ] All Core sources compiled (25 .o files from Core/Src/)
- [ ] `./validate_cproject.sh` reports 11/11 checks pass
- [ ] Size output shows ~53 KB text, ~7.6 KB BSS
