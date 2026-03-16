# STM32CubeIDE Project Configuration Guide

This document explains how the STM32CubeIDE project is structured, which
files control what, and how to ensure CubeIDE loads the correct project
and compiles the correct `main.c`.

---

## 1. Which `.ioc` file exists and its purpose

| Item | Value |
|------|-------|
| **File** | `STM32-Control-Coche-Marcos.ioc` |
| **Path** | Repository root (`./STM32-Control-Coche-Marcos.ioc`) |
| **CubeMX version** | 6.17.0 |
| **Firmware package** | STM32Cube FW_G4 V1.6.2 |
| **Target MCU** | STM32G474RETx (LQFP64) |
| **Toolchain** | STM32CubeIDE |

The `.ioc` file matches the expected STM32CubeMX configuration.  It
defines all peripheral initialisations used by the firmware:

- **ADC1** – Pedal accelerator (PA3, channel 4)
- **FDCAN1** – CAN bus at 500 kbps (PB8 RX / PB9 TX)
- **I2C1** – Sensor bus (PB6 SCL / PB7 SDA)
- **IWDG** – Independent watchdog
- **TIM1** – 4-channel PWM for front traction motors (20 kHz)
- **TIM2** – Quadrature encoder for steering
- **TIM3** – 2-channel PWM for steering motor (20 kHz)
- **TIM8** – 4-channel PWM for rear traction motors (20 kHz)

Key CubeMX project-manager settings:

| Setting | Value | Meaning |
|---------|-------|---------|
| `KeepUserCode` | `true` | Preserve code between `USER CODE` markers |
| `NoMain` | `true` | **Do NOT regenerate `main.c`** (protects custom firmware) |
| `DeletePrevious` | `true` | Remove old generated files before regeneration |
| `MainLocation` | `Core/Src` | Init code target directory |
| `UnderRoot` | `false` | Code goes into `Core/` subdirectories |

> **Important**: `NoMain` is set to `true` because the current `main.c`
> is a fully custom 36 KB firmware file with no `/* USER CODE BEGIN/END */`
> markers.  If CubeMX were allowed to regenerate `main.c` it would
> **overwrite the entire file** with a minimal template, destroying all
> vehicle-control logic.

---

## 2. `.project` and `.cproject` source-folder references

### `.project`

Standard Eclipse/CDT project descriptor.  Key facts:

- **Project name**: `STM32-Control-Coche-Marcos`
- **Natures**: `MCUProjectNature`, `cnature`, `managedBuildNature`
- **Builders**: `genmakebuilder`, `ScannerConfigBuilder`

No source-folder overrides exist here; all source paths are defined in
`.cproject`.

### `.cproject`

The CDT build configuration defines three source entries (line 63‑67):

| Source entry | Excludes | Notes |
|--------------|----------|-------|
| `Core/Src` | `test_*.c` files | Main firmware sources |
| `Core/Startup` | *(none)* | Startup assembly (generated) |
| `Drivers` | `CMSIS/Device/ST/STM32G4xx/Source`, unused HAL modules, template files | HAL + CMSIS drivers |

Include paths (`-I`):

```
../Core/Inc
../Drivers/STM32G4xx_HAL_Driver/Inc
../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
../Drivers/CMSIS/Device/ST/STM32G4xx/Include
../Drivers/CMSIS/Include
```

**Conclusion**: both `.project` and `.cproject` correctly reference
`Core/Src` and `Core/Inc`.

---

## 3. Duplicate `main.c` files

**There are NO duplicate `main.c` files.**

The only `main.c` in the entire repository is:

```
Core/Src/main.c   (36.5 KB – full vehicle-control firmware)
```

No hidden folders, no secondary `Core` directories, no copies in
`Drivers/`, `Middlewares/`, `esp32/`, `docs/`, or anywhere else contain a
`main.c`.

---

## 4. Could CubeIDE load a different project folder or cached copy?

**No, provided you import the project correctly.**

- The `.project` file in the repo root uniquely identifies the project as
  `STM32-Control-Coche-Marcos`.
- The build path in `.cproject` is
  `${workspace_loc:/STM32-Control-Coche-Marcos}/Debug`.
- There is no `.metadata/` directory in the repo (it is git-ignored), so
  no stale workspace state is committed.

**How to avoid loading a wrong copy:**

1. In STM32CubeIDE choose **File → Import → Existing Projects into
   Workspace**.
2. Set **Root Directory** to the cloned repository folder.
3. Ensure exactly one project (`STM32-Control-Coche-Marcos`) is listed
   and checked.
4. Do **not** check "Copy projects into workspace" – work in-place.
5. If the IDE complains about a name conflict, delete the old project
   from the workspace first (**right-click → Delete**, uncheck "Delete
   project contents on disk").

---

## 5. Could CubeMX code generation overwrite the updated `main.c`?

**Previously yes – now fixed.**

The `.ioc` file now has `ProjectManager.NoMain=true`, which tells
STM32CubeMX **not to generate `main.c`** when code is regenerated.

The custom `main.c` contains no `/* USER CODE BEGIN/END */` markers, so
the `KeepUserCode=true` setting alone was **not sufficient** to protect
it.  Setting `NoMain=true` is the correct safeguard.

> **If you change pin assignments in CubeMX**, regeneration will update
> `stm32g4xx_hal_msp.c`, `stm32g4xx_it.c`, and HAL config headers – but
> it will **not touch `main.c`**.  You must manually update the
> `MX_*_Init()` functions in `main.c` to match the new `.ioc` settings.

---

## 6. Is the build system compiling the correct `main.c`?

**Yes.**

- The **Makefile** (standalone ARM GCC build) explicitly lists
  `Core/Src/main.c` in its `C_SOURCES` variable.
- The **`.cproject`** (CubeIDE managed build) includes `Core/Src` as a
  source entry, which automatically picks up `main.c`.
- No other `main.c` exists anywhere in the project.

Both build systems compile exactly one `main.c`: `Core/Src/main.c`.

---

## 7. Does any file outside `Core/Src` contain another `main.c`?

**No.**

Searched locations with zero hits:

- `Drivers/` (not even present – generated on demand)
- `Middlewares/` (does not exist)
- `esp32/` (ESP32 companion firmware uses `main.cpp`, not `main.c`)
- `docs/`, hidden directories, and all other paths

---

## 8. Stale build artifacts

**No stale build artifacts exist in the repository.**

- `Debug/`, `Release/`, and `build/` directories do **not** exist.
- `.gitignore` correctly excludes `Debug/`, `Release/`, `build/`, and
  all compiled object files (`*.o`, `*.elf`, `*.bin`, `*.hex`, etc.).
- No `.metadata/` or IDE workspace cache is committed.

---

## 9. Startup assembly file (`Core/Startup/startup_stm32g474retx.s`)

**Status: FIXED** — replaced with the official STM32CubeG4 v1.5.1 version.

The previous startup file had critical issues:

| Issue | Severity | Detail |
|-------|----------|--------|
| Truncated vector table | **Critical** | Only 68 entries — missing 35 IRQ vectors (positions 47–101) for ADC3, DMA2, UART4/5, TIM5-7/20, SPI3/4, I2C3/4, HRTIM, FDCAN2/3, CORDIC, FMAC, etc. |
| Wrong handler positions | **Critical** | COMP1_2_3 at vector position 50 (should be 64); FPU at 67 (should be 81) |
| Incorrect entries | **Critical** | USB_HP/USB_LP duplicated at positions 60–61 (should be DMA2_Channel5, ADC4) |
| Missing `__libc_init_array` | **High** | C runtime static initializers not called before `main()` |
| Inefficient data-copy loop | Low | Reloaded constant addresses inside loop body |

The fixed file matches the official
`Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g474xx.s`
from STM32CubeG4 v1.5.1 byte-for-byte (102 IRQ vectors, proper
Reset_Handler with `__libc_init_array`).

---

## Summary of answers

| Question | Answer |
|----------|--------|
| **Which `main.c` is correct?** | `Core/Src/main.c` – the only one in the repository (36.5 KB custom firmware). |
| **Which `main.c` does CubeIDE use?** | `Core/Src/main.c` – the `.cproject` source entry points there. |
| **Is the `.ioc` file being applied?** | Yes.  CubeMX reads `STM32-Control-Coche-Marcos.ioc` from the project root.  It controls peripheral init code generation but **no longer regenerates `main.c`** (`NoMain=true`). |
| **How to ensure CubeIDE loads the correct project?** | Import with *File → Import → Existing Projects into Workspace*, point to the repo root, do **not** copy into workspace, and delete any old workspace entries with the same name first.  After import, run `setup_drivers.sh` (or `.bat`) to download the HAL/CMSIS `Drivers/` folder, then build. |

---

## Quick-start after a fresh clone

```bash
# 1. Clone the repository
git clone <repo-url>
cd STM32-Control-Coche-Marcos

# 2. Download HAL + CMSIS drivers (STM32CubeG4 v1.5.1)
bash setup_drivers.sh        # Linux/macOS
# or: setup_drivers.bat      # Windows

# 3. Open STM32CubeIDE
#    File → Import → Existing Projects into Workspace
#    Root directory = this repo folder
#    Uncheck "Copy projects into workspace"

# 4. Build (in IDE or command line)
make all                     # standalone ARM GCC build
```
