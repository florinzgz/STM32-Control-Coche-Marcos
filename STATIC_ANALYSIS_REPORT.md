# Static Analysis Report — Dual Firmware

> **Generated**: 2026-03-18  
> **Tools**: Clang-Tidy 18.1.3, CppCheck 2.13.0  
> **Repository**: STM32-Control-Coche-Marcos

---

## Table of Contents

1. [Tool Setup Summary](#tool-setup-summary)
2. [ESP32-S3 — Analysis Results](#esp32-s3--analysis-results)
3. [STM32 G474 — Analysis Results](#stm32-g474--analysis-results)
4. [Recommendations](#recommendations)

---

## Tool Setup Summary

| Tool | Version | Status |
|------|---------|--------|
| Clang / Clang-Tidy | Ubuntu LLVM 18.1.3 | ✅ Installed |
| CppCheck | 2.13.0 | ✅ Installed |
| PlatformIO CLI | 6.1.19 | ✅ Installed |
| ARM GCC (`arm-none-eabi-gcc`) | 13.2.1 | ✅ Installed (for STM32 compile_commands.json) |
| Bear / compiledb | 3.1.3 / 0.10.7 | ✅ Installed |

### Configuration Files Created

| File | Purpose |
|------|---------|
| `esp32/.clangd` | clangd configuration for ESP32-S3 project (Arduino/C++17), isolated from STM32 |
| `.clangd` (root) | clangd configuration for STM32 G474 project (ARM C), isolated from ESP32 |
| `esp32/compile_commands.json` | Compilation database for ESP32 sources (33 entries) |
| `compile_commands.json` (root) | Compilation database for STM32 Core sources (19 entries) |

> **Note**: `compile_commands.json` files are added to `.gitignore` (build artifacts).

---

## ESP32-S3 — Analysis Results

**Project**: ESP32-S3 HMI firmware (Arduino + PlatformIO)  
**Language**: C++17  
**Source files analyzed**: 33 .cpp files, 35 .h headers  

### CppCheck Summary

| Severity | Count |
|----------|-------|
| **Errors** | 0 |
| **Warnings** | 5 |
| **Style** | 16 (unique, excl. unusedFunction) |
| **Performance** | 2 |
| **unusedFunction** | 13 (many are Arduino framework entry points like `setup()`/`loop()`) |

### CppCheck — Warnings (potential bugs)

| File | Line | Description | Check ID |
|------|------|-------------|----------|
| `screens/engineering_screen.cpp` | 334 | `%u` format specifier used with `signed int` argument (×4 occurrences) | `invalidPrintfArgType_uint` |
| `screens/engineering_screen.cpp` | 676 | `%u` format specifier used with `signed int` argument | `invalidPrintfArgType_uint` |

### CppCheck — Style / Performance Issues

| File | Line | Description | Check ID |
|------|------|-------------|----------|
| `config_store.cpp` | 82 | Variable `inaDefault` scope can be reduced; can be `const` | `variableScope`, `constVariable` |
| `config_store.cpp` | 89 | Variable `tmpDefault` scope can be reduced; can be `const` | `variableScope`, `constVariable` |
| `main.cpp` | 550 | `if (showOverlay) showOverlay=false` is logically equivalent to `showOverlay=false` | `duplicateConditionalAssign` |
| `main.cpp` | 56 | C-style pointer casting | `cstyleCast` |
| `screens/drive_screen.h` | 76 | `DriveScreen::drawSpeed` can be `const` | `functionConst` |
| `screens/engineering_screen.h` | 56 | `EngineeringScreen::drawMainMenu` can be `static` | `functionStatic` |
| `screens/engineering_screen.h` | 61–62 | `drawSensorMapIna`/`drawSensorMapTemp` can be `const` | `functionConst` |
| `screens/engineering_screen.h` | 63 | `drawFactoryDefaults` can be `static` | `functionStatic` |
| `screens/engineering_screen.cpp` | 597 | Same expression in both branches of ternary: `(i == 6) ? COL_DARK_GRAY : COL_DARK_GRAY` | `duplicateExpressionTernary` |
| `screens/pin_screen.h` | 69 | `PinScreen::drawDots` can be `const` | `functionConst` |

### Clang-Tidy Summary

| Category | Count |
|----------|-------|
| **Total warnings** | 1,763 |
| **Compiler errors** | 29 (missing Arduino/ESP-IDF headers — expected in host analysis) |

### Clang-Tidy — Top Issues by Category

| Check | Count | Severity |
|-------|-------|----------|
| `misc-use-anonymous-namespace` | 314 | Style |
| `readability-identifier-length` | 300 | Style |
| `misc-include-cleaner` | 229 | Style |
| `misc-const-correctness` | 198 | Style |
| `modernize-avoid-c-arrays` | 191 | Modernize |
| `readability-braces-around-statements` | 170 | Style |
| `readability-uppercase-literal-suffix` | 91 | Style |
| `cert-err33-c` | 41 | Cert |
| `modernize-use-nodiscard` | 37 | Modernize |
| **`bugprone-narrowing-conversions`** | **34** | **Bug-prone** |
| `modernize-use-auto` | 29 | Modernize |
| **`bugprone-easily-swappable-parameters`** | **29** | **Bug-prone** |
| `readability-implicit-bool-conversion` | 28 | Style |
| `readability-convert-member-functions-to-static` | 24 | Style |
| **`bugprone-branch-clone`** | **12** | **Bug-prone** |
| `readability-function-cognitive-complexity` | 9 | Style |
| **`bugprone-switch-missing-default-case`** | **2** | **Bug-prone** |

### Clang-Tidy — High-Priority Bugprone Findings

#### Narrowing Conversions (34 occurrences)
- `audio_manager.cpp:65,67` — `int` → `int16_t`
- `led_controller.cpp:101,103,104,106` — `int` → `int8_t`
- `screens/drive_screen.cpp:224,228` — `int` → `int16_t`, `int8_t`
- `screens/engineering_screen.cpp` — Multiple lines, `int` → `int16_t`

#### Branch Clones (12 occurrences)
- `led_controller.cpp:143` — switch has 5 consecutive identical branches
- `led_controller.cpp:161` — switch has 3 consecutive identical branches
- `main.cpp:168` — switch has 5 consecutive identical branches
- `main.cpp:352,379` — if with identical then/else branches
- `screen_manager.cpp:145,149` — switch with identical branches
- `engineering_screen.cpp:183,597` — repeated branch bodies

#### Missing Default Case
- `boot_screen.cpp:229` — switch on non-enum without default
- `engineering_screen.cpp:443` — switch on non-enum without default

---

## STM32 G474 — Analysis Results

**Project**: STM32G474RE motor control firmware  
**Language**: C11  
**Source files analyzed**: 19 .c files, 17 .h headers  

### CppCheck Summary

| Severity | Count |
|----------|-------|
| **Errors** | 0 |
| **Warnings** | 0 |
| **Style** | 18 (unique, excl. unusedFunction) |
| **Performance** | 0 |
| **unusedFunction** | ~91 (many are HAL callbacks, ISR handlers, and public API — expected for embedded) |

### CppCheck — Style Issues (non-trivial)

| File | Line | Description | Check ID |
|------|------|-------------|----------|
| `motor_control.c` | 258 | Struct member `Motor_t::power` is never used | `unusedStructMember` |
| `service_mode.c` | 234, 245, 256 | Redundant condition: `i < 32` is redundant since `i < 25` is sufficient (×3) | `redundantCondition` |
| `stm32g4xx_hal_msp.c` | 9–183 | Multiple HAL_MspInit parameters can be `const` (×7) | `constParameterPointer` |
| `syscalls.c` | 64 | Parameter `ptr` in `_write` can be pointer-to-const | `constParameterPointer` |

> **Note**: The `constParameterPointer` findings on HAL MSP functions are false positives — these are HAL callback signatures defined by ST and cannot be changed.

### Clang-Tidy Summary

| Category | Count |
|----------|-------|
| **Total warnings** | 859 |
| **Compiler errors** | 6 (missing HAL headers — expected, Drivers/ is gitignored) |

### Clang-Tidy — Top Issues by Category

| Check | Count | Severity |
|-------|-------|----------|
| `readability-uppercase-literal-suffix` | 395 | Style |
| `readability-braces-around-statements` | 211 | Style |
| `readability-identifier-length` | 107 | Style |
| `misc-include-cleaner` | 57 | Style |
| **`bugprone-narrowing-conversions`** | **26** | **Bug-prone** |
| `readability-function-cognitive-complexity` | 12 | Style |
| **`bugprone-reserved-identifier`** | **12** | **Bug-prone** |
| **`bugprone-easily-swappable-parameters`** | **10** | **Bug-prone** |
| **`bugprone-branch-clone`** | **9** | **Bug-prone** |
| **`bugprone-too-small-loop-variable`** | **5** | **Bug-prone** |
| `misc-unused-parameters` | 4 | Style |
| `performance-no-int-to-ptr` | 3 | Performance |
| `misc-redundant-expression` | 3 | Style |
| **`bugprone-switch-missing-default-case`** | **1** | **Bug-prone** |

### Clang-Tidy — High-Priority Bugprone Findings

#### Reserved Identifiers (12 occurrences)
Header guards using double-underscore prefix (`__MOTOR_CONTROL_H`, `__SAFETY_SYSTEM_H`, etc.) — these are reserved by the C standard.

**Affected headers**:
- `encoder_reader.h` → `__ENCODER_READER_H`
- `eps_params.h` → `__EPS_PARAMS_H`
- `error_log.h` → `__ERROR_LOG_H`
- `motor_control.h` → `__MOTOR_CONTROL_H`
- `safety_system.h` → `__SAFETY_SYSTEM_H`
- `service_mode.h` → `__SERVICE_MODE_H`
- `steering_cal_store.h` → `__STEERING_CAL_STORE_H`
- `steering_centering.h` → `__STEERING_CENTERING_H`
- `vehicle_physics.h` → `__VEHICLE_PHYSICS_H`

#### Narrowing Conversions (26 occurrences)
- `motor_control.c:1128,1139` — `int` → `float`
- `motor_control.c:1169` — `int` → `int8_t`
- `motor_control.c:1270,1301,1308,1313,1314,1382` — `int` → `float`
- `motor_control.c:1302` — `int` → `int8_t` (×2)
- `motor_control.c:1608–1622` — `int` → `int16_t` (×7)
- `motor_control.c:1785–1797` — `int` → `int16_t` (×8)

#### Branch Clones (9 occurrences)
- `can_handler.c:106` — switch has **10** consecutive identical branches
- `main.c:314,337` — repeated branch body in conditional chain
- `motor_control.c:962,1234` — repeated branch body in conditional chain

#### Too-Small Loop Variables (5 occurrences)
- `can_handler.c:922` — `uint8_t` loop variable with `int` upper bound

#### Missing Default Case
- `can_handler.c:900` — switch on non-enum without default case

---

## Recommendations

### Critical (should fix)

1. **`invalidPrintfArgType_uint` in ESP32 engineering_screen.cpp** — Using `%u` with `signed int` can produce unexpected output on negative values. Cast arguments to `unsigned int` or use `%d`.

2. **Reserved identifiers in STM32 headers** — Double-underscore prefixes (`__MOTOR_CONTROL_H`) are reserved by the C standard. Rename to single-underscore or no-underscore convention (`MOTOR_CONTROL_H`).

3. **`duplicateExpressionTernary` in ESP32 engineering_screen.cpp:597** — `(i == 6) ? COL_DARK_GRAY : COL_DARK_GRAY` — identical branches suggest copy-paste error.

4. **Branch clone in STM32 can_handler.c:106** — 10 consecutive identical switch branches suggest missing per-case logic.

### Important (recommended)

5. **Narrowing conversions** — 60 total across both projects. Use explicit casts where intentional to document behavior.

6. **Unused struct member** `Motor_t::power` — Dead code in safety-critical firmware should be removed.

7. **Redundant conditions** in `service_mode.c` — `i < 32` when `i < 25` is the real bound.

8. **`duplicateConditionalAssign` in ESP32 main.cpp:550** — Simplify `if (showOverlay) showOverlay = false` to `showOverlay = false`.

### Low Priority (style)

9. `readability-uppercase-literal-suffix` — Use `0xFF` instead of `0xff`, `1.0F` instead of `1.0f` (395 in STM32, 91 in ESP32).
10. `readability-braces-around-statements` — Add braces to single-line if/for/while bodies.
11. `modernize-avoid-c-arrays` — Replace C-style arrays with `std::array` in ESP32 code where appropriate.
12. `misc-const-correctness` — Add `const` qualifiers where possible (198 in ESP32).

---

*Report generated by automated static analysis. No source code was modified.*
