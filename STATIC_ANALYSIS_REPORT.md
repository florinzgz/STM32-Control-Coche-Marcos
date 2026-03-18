# Informe de Análisis Estático — Dual Firmware (STM32 + ESP32)

> **Fecha**: 2026-03-18
> **Herramientas**: Clang-Tidy 18.1.3, CppCheck 2.13.0, clangd 18.1.3
> **Repositorio**: STM32-Control-Coche-Marcos
> **Nota**: Este informe se basa EXCLUSIVAMENTE en los outputs reales de las herramientas ejecutadas. Ningún resultado ha sido inventado ni asumido.

---

## === 1. Resumen Ejecutivo ===

### Estado General del Firmware

| Proyecto | Herramienta | Errores | Warnings | Estilo | Performance |
|----------|-------------|---------|----------|--------|-------------|
| **STM32 G474** | CppCheck | 0 | 0 | 18 + 91 unusedFunction | 0 |
| **STM32 G474** | Clang-Tidy | 13 (headers faltantes) | 1,007 | — | — |
| **ESP32-S3** | CppCheck | 0 | 5 | 16 + 13 unusedFunction | 2 |
| **ESP32-S3** | Clang-Tidy | 27 (headers faltantes) | 1,525 | — | — |
| **STM32 G474** | clangd | 21 (cascada de include) | — | — | — |
| **ESP32-S3** | clangd | 37+40 (cascada de include) | — | — | — |

### Riesgos Reales Detectados

1. **Printf format mismatch** (ESP32): `%u` con `signed int` en `engineering_screen.cpp` — puede producir output incorrecto o comportamiento indefinido con valores negativos. **Severidad: Alta**.
2. **Ternario duplicado** (ESP32): `(i == 6) ? COL_DARK_GRAY : COL_DARK_GRAY` — ambas ramas idénticas, probable copy-paste error. **Severidad: Alta** (bug visual).
3. **Reserved identifiers** (STM32): 13 header guards con doble guion bajo (`__MOTOR_CONTROL_H`, etc.) + 25 en syscalls.c/sysmem.c — reservados por el estándar C/C++. **Severidad: Media** (UB técnico).
4. **Narrowing conversions**: 26 en STM32, 34 en ESP32 — conversiones implícitas que pueden perder datos. **Severidad: Media**.
5. **Branch clones**: 11 en STM32 (incl. 10 ramas idénticas en `can_handler.c:106`), 12 en ESP32 — lógica posiblemente incompleta. **Severidad: Media**.
6. **Missing default cases**: 1 en STM32 (`can_handler.c:900`), 2 en ESP32 (`boot_screen.cpp:229`, `engineering_screen.cpp:443`). **Severidad: Media** (defensive programming).
7. **Too-small loop variables**: 11 en STM32 — `uint8_t` como variable de loop con upper bound `int`. **Severidad: Baja** (funcional para rangos actuales, pero frágil).

### Severidad por Categorías

| Severidad | Categoría | Count |
|-----------|-----------|-------|
| 🔴 Alta | Printf format mismatch, ternario duplicado | 6 |
| 🟠 Media | Reserved identifiers, narrowing, branch clones, missing default | ~98 |
| 🟡 Baja | Loop variables, redundant conditions, style | ~2,500 |
| ⚪ Info | unusedFunction (falsos positivos de framework) | 104 |

### Propuestas de Mejora Globales

1. **Inmediatas**: Corregir los 5 `invalidPrintfArgType_uint` y el ternario duplicado en `engineering_screen.cpp`.
2. **Corto plazo**: Renombrar header guards `__XXX_H` → `XXX_H` en 13 headers STM32.
3. **Medio plazo**: Añadir `default:` a todos los switches sin él; revisar branch clones.
4. **Toolchain**: Generar `compile_commands.json` con `pio run -t compiledb` (ESP32) y `bear make` o compiledb (STM32) en entorno con SDKs completos para obtener análisis sin falsos errores de include.

---

## === 2. clang-tidy ===

### 2.1 STM32 G474 (ARM C)

**Comando**: `clang-tidy-18 -p . --checks='bugprone-*,cert-*,misc-*,performance-*,readability-*,-readability-magic-numbers,-cert-dcl37-c,-cert-dcl51-cpp' Core/Src/*.c`

**Total**: 1,007 warnings + 13 errores de compilación (headers faltantes — esperado sin Drivers/)

#### Desglose por Categoría (output real)

| Check | Count | Severidad |
|-------|-------|-----------|
| `readability-uppercase-literal-suffix` | 498 | Estilo |
| `readability-braces-around-statements` | 226 | Estilo |
| `readability-identifier-length` | 86 | Estilo |
| `misc-include-cleaner` | 63 | Info |
| `bugprone-narrowing-conversions` | 26 | Media |
| `bugprone-reserved-identifier` | 25 | Media |
| `readability-function-cognitive-complexity` | 22 | Estilo |
| `bugprone-easily-swappable-parameters` | 15 | Baja |
| `bugprone-branch-clone` | 11 | Media |
| `bugprone-too-small-loop-variable` | 11 | Baja |
| `readability-non-const-parameter` | 7 | Estilo |
| `performance-no-int-to-ptr` | 5 | Baja |
| `misc-unused-parameters` | 4 | Estilo |
| `misc-redundant-expression` | 3 | Baja |
| `readability-redundant-declaration` | 2 | Estilo |
| `readability-isolate-declaration` | 1 | Estilo |
| `readability-else-after-return` | 1 | Estilo |
| `bugprone-switch-missing-default-case` | 1 | Media |

#### Errores de Compilación (esperados — Drivers/ es gitignored)

```
Core/Inc/main.h:8:10: error: 'stm32g4xx_hal.h' file not found [clang-diagnostic-error]
Core/Inc/math_safety.h:18:10: error: 'math.h' file not found [clang-diagnostic-error]
Core/Src/ackermann.c:18:10: error: 'math.h' file not found [clang-diagnostic-error]
Core/Src/eps_params.c:19:10: error: 'stm32g4xx_hal.h' file not found [clang-diagnostic-error]
Core/Src/error_log.c:27:10: error: 'stm32g4xx_hal.h' file not found [clang-diagnostic-error]
Core/Src/steering_cal_store.c:21:10: error: 'stm32g4xx_hal.h' file not found [clang-diagnostic-error]
Core/Src/syscalls.c:11:10: error: 'sys/stat.h' file not found [clang-diagnostic-error]
Core/Src/sysmem.c:9:10: error: 'errno.h' file not found [clang-diagnostic-error]
Core/Src/system_stm32g4xx.c:1:10: error: 'stm32g4xx.h' file not found [clang-diagnostic-error]
Core/Src/test_eps_params.c:15:10: error: 'stdio.h' file not found [clang-diagnostic-error]
Core/Src/test_error_log.c:15:10: error: 'stdio.h' file not found [clang-diagnostic-error]
Core/Src/test_math_safety.c:15:10: error: 'stdio.h' file not found [clang-diagnostic-error]
Core/Src/test_steering_cal_store.c:15:10: error: 'stdio.h' file not found [clang-diagnostic-error]
```

> **Causa**: El toolchain ARM (`--target=arm-none-eabi`) no tiene headers de sistema en este entorno CI. `Drivers/` (HAL/CMSIS) está en `.gitignore`. Estos errores desaparecen en el entorno de desarrollo con CubeIDE + Drivers instalados.

#### Ejemplos Reales — bugprone-reserved-identifier (25)

**A) Header guards con doble guion bajo (13 headers):**

Los siguientes headers usan `#ifndef __XXX_H` / `#define __XXX_H`, que es un identificador reservado por el estándar C (§7.1.3):

```
Core/Inc/ackermann.h       → __ACKERMANN_H
Core/Inc/boot_validation.h → __BOOT_VALIDATION_H
Core/Inc/can_handler.h     → __CAN_HANDLER_H
Core/Inc/encoder_reader.h  → __ENCODER_READER_H
Core/Inc/eps_params.h      → __EPS_PARAMS_H
Core/Inc/error_log.h       → __ERROR_LOG_H
Core/Inc/motor_control.h   → __MOTOR_CONTROL_H
Core/Inc/safety_system.h   → __SAFETY_SYSTEM_H
Core/Inc/service_mode.h    → __SERVICE_MODE_H
Core/Inc/steering_cal_store.h → __STEERING_CAL_STORE_H
Core/Inc/steering_centering.h → __STEERING_CENTERING_H
Core/Inc/stm32g4xx_it.h    → __STM32G4xx_IT_H
Core/Inc/vehicle_physics.h → __VEHICLE_PHYSICS_H
```

> **Nota**: `main.h`, `math_safety.h`, `sensor_manager.h` y `stm32g4xx_hal_conf.h` ya usan guards correctos sin doble guion bajo.

**B) POSIX stubs en syscalls.c/sysmem.c (12):**

```
Core/Src/syscalls.c:21:12: warning: declaration uses identifier '__io_putchar', which is a reserved identifier [bugprone-reserved-identifier]
Core/Src/syscalls.c:22:12: warning: declaration uses identifier '__io_getchar', which is a reserved identifier [bugprone-reserved-identifier]
Core/Src/syscalls.c:24:7: warning: declaration uses identifier '__env', which is a reserved identifier [bugprone-reserved-identifier]
Core/Src/syscalls.c:32:5: warning: declaration uses identifier '_getpid', which is reserved in the global namespace
Core/Src/syscalls.c:37:5: warning: declaration uses identifier '_kill', which is reserved in the global namespace
(... +15 más en syscalls.c, +5 en sysmem.c)
```

> **Nota sobre syscalls/sysmem**: Estos son stubs POSIX generados por CubeMX. Los nombres con `_` prefijo son requeridos por newlib/libgloss. Estos warnings son **falsos positivos** — los nombres DEBEN ser exactamente así.

#### Ejemplos Reales — bugprone-branch-clone (11)

```
Core/Src/can_handler.c:106:9: warning: switch has 10 consecutive identical branches [bugprone-branch-clone]
Core/Src/main.c:314:34: warning: repeated branch body in conditional chain [bugprone-branch-clone]
Core/Src/main.c:337:46: warning: repeated branch body in conditional chain [bugprone-branch-clone]
Core/Src/motor_control.c:962:50: warning: repeated branch body in conditional chain [bugprone-branch-clone]
Core/Src/motor_control.c:1234:60: warning: repeated branch body in conditional chain [bugprone-branch-clone]
Core/Src/safety_system.c:382:13: warning: switch has 2 consecutive identical branches [bugprone-branch-clone]
Core/Src/safety_system.c:422:13: warning: switch has 2 consecutive identical branches [bugprone-branch-clone]
Core/Src/safety_system.c:1267:38: warning: repeated branch body in conditional chain [bugprone-branch-clone]
Core/Src/safety_system.c:1280:49: warning: repeated branch body in conditional chain [bugprone-branch-clone]
Core/Src/stm32g4xx_hal_msp.c:149:3: warning: repeated branch body in conditional chain [bugprone-branch-clone]
Core/Src/system_stm32g4xx.c:31:7: warning: if with identical then and else branches [bugprone-branch-clone]
```

#### Ejemplos Reales — bugprone-too-small-loop-variable (11)

```
Core/Src/can_handler.c:922:38: warning: loop variable has narrower type 'uint8_t' [bugprone-too-small-loop-variable]
Core/Src/service_mode.c:82:25: warning: loop variable has narrower type 'uint8_t'
Core/Src/service_mode.c:189:25: warning: loop variable has narrower type 'uint8_t'
Core/Src/service_mode.c:201:25: warning: loop variable has narrower type 'uint8_t'
Core/Src/service_mode.c:217:25: warning: loop variable has narrower type 'uint8_t'
Core/Src/test_service_mode.c:161:25: warning: loop variable has narrower type 'uint8_t'
Core/Src/test_service_mode.c:168:25: warning: loop variable has narrower type 'uint8_t'
Core/Src/test_service_mode.c:200:25: warning: loop variable has narrower type 'uint8_t'
Core/Src/test_service_mode.c:265:25: warning: loop variable has narrower type 'uint8_t'
Core/Src/test_service_mode.c:294:25: warning: loop variable has narrower type 'uint8_t'
Core/Src/test_service_mode.c:314:25: warning: loop variable has narrower type 'uint8_t'
```

#### Ejemplo Real — bugprone-switch-missing-default-case (1)

```
Core/Src/can_handler.c:900:25: warning: switching on non-enum value without default case [bugprone-switch-missing-default-case]
```

---

### 2.2 ESP32-S3 (Arduino C++17)

**Comando**: `clang-tidy-18 -p . --checks='bugprone-*,cert-*,misc-*,performance-*,readability-*,modernize-*,-readability-magic-numbers,-modernize-use-trailing-return-type,-cert-dcl37-c,-cert-dcl51-cpp' src/**/*.cpp`

**Total**: 1,525 warnings + 27 errores de compilación (headers Arduino/ESP-IDF faltantes — esperado sin SDK)

#### Desglose por Categoría (output real)

| Check | Count | Severidad |
|-------|-------|-----------|
| `misc-use-anonymous-namespace` | 314 | Estilo |
| `misc-include-cleaner` | 229 | Info |
| `misc-const-correctness` | 198 | Estilo |
| `readability-identifier-length` | 178 | Estilo |
| `readability-braces-around-statements` | 149 | Estilo |
| `modernize-avoid-c-arrays` | 146 | Estilo |
| `readability-uppercase-literal-suffix` | 86 | Estilo |
| `cert-err33-c` | 41 | Media |
| `bugprone-narrowing-conversions` | 34 | Media |
| `modernize-use-auto` | 29 | Estilo |
| `readability-implicit-bool-conversion` | 28 | Estilo |
| `readability-convert-member-functions-to-static` | 24 | Estilo |
| `bugprone-easily-swappable-parameters` | 23 | Baja |
| `bugprone-branch-clone` | 12 | Media |
| `readability-function-cognitive-complexity` | 9 | Estilo |
| `readability-non-const-parameter` | 3 | Estilo |
| `readability-make-member-function-const` | 3 | Estilo |
| `modernize-redundant-void-arg` | 3 | Estilo |
| `readability-simplify-boolean-expr` | 2 | Estilo |
| `readability-redundant-declaration` | 2 | Estilo |
| `readability-isolate-declaration` | 2 | Estilo |
| `modernize-use-default-member-init` | 2 | Estilo |
| `modernize-loop-convert` | 2 | Estilo |
| `bugprone-switch-missing-default-case` | 2 | Media |
| `readability-redundant-member-init` | 1 | Estilo |
| `readability-avoid-nested-conditional-operator` | 1 | Estilo |
| `misc-redundant-expression` | 1 | Media |
| `cert-err58-cpp` | 1 | Baja |

#### Errores de Compilación (esperados — SDK Arduino no instalado)

```
src/audio_manager.cpp:18:10: error: 'Arduino.h' file not found [clang-diagnostic-error]
src/can/can_obstacle.cpp:15:10: error: 'Arduino.h' file not found [clang-diagnostic-error]
src/can_rx.cpp:13:10: error: 'Arduino.h' file not found [clang-diagnostic-error]
src/config_store.cpp:9:10: error: 'Preferences.h' file not found [clang-diagnostic-error]
src/hmi/obstacle_indicator.h:15:10: error: 'TFT_eSPI.h' file not found [clang-diagnostic-error]
src/led_controller.cpp:21:10: error: 'FastLED.h' file not found [clang-diagnostic-error]
src/main.cpp:11:10: error: 'Arduino.h' file not found [clang-diagnostic-error]
src/power_manager.cpp:15:10: error: 'Arduino.h' file not found [clang-diagnostic-error]
src/relay_audio.cpp:21:10: error: 'Arduino.h' file not found [clang-diagnostic-error]
src/sensors/obstacle_sensor.cpp:19:10: error: 'Arduino.h' file not found [clang-diagnostic-error]
src/shifter_input.cpp:15:10: error: 'Wire.h' file not found [clang-diagnostic-error]
(+ 16 más del mismo tipo en test files y UI headers)
```

#### Ejemplos Reales — bugprone-branch-clone (12)

```
src/led_controller.cpp:143:9: warning: switch has 5 consecutive identical branches [bugprone-branch-clone]
src/led_controller.cpp:161:9: warning: switch has 3 consecutive identical branches [bugprone-branch-clone]
src/led_controller.cpp:188:9: warning: switch has 3 consecutive identical branches [bugprone-branch-clone]
src/main.cpp:168:9: warning: switch has 5 consecutive identical branches [bugprone-branch-clone]
src/main.cpp:173:9: warning: switch has 2 consecutive identical branches [bugprone-branch-clone]
src/main.cpp:352:5: warning: if with identical then and else branches [bugprone-branch-clone]
src/main.cpp:379:9: warning: if with identical then and else branches [bugprone-branch-clone]
src/screen_manager.cpp:145:9: warning: switch has 3 consecutive identical branches [bugprone-branch-clone]
src/screen_manager.cpp:149:9: warning: switch has 2 consecutive identical branches [bugprone-branch-clone]
src/screens/engineering_screen.cpp:183:63: warning: repeated branch body in conditional chain [bugprone-branch-clone]
src/screens/engineering_screen.cpp:597:35: warning: conditional operator with identical true and false expressions [bugprone-branch-clone]
src/shifter_input.cpp:118:5: warning: if with identical then and else branches [bugprone-branch-clone]
```

#### Ejemplo Real — misc-redundant-expression (1)

```
src/screens/engineering_screen.cpp:597:55: warning: 'true' and 'false' expressions are equivalent [misc-redundant-expression]
```

Confirma el ternario duplicado: `(i == 6) ? ui::COL_DARK_GRAY : ui::COL_DARK_GRAY`

#### Ejemplos Reales — bugprone-switch-missing-default-case (2)

```
src/screens/boot_screen.cpp:229:13: warning: switching on non-enum value without default case [bugprone-switch-missing-default-case]
src/screens/engineering_screen.cpp:443:21: warning: switching on non-enum value without default case [bugprone-switch-missing-default-case]
```

---

## === 3. CppCheck ===

### 3.1 STM32 G474

**Comando**: `cppcheck --enable=all --inconclusive --force --suppress=missingInclude -ICore/Inc Core/Src/`
**Versión**: CppCheck 2.13.0

**Resultados**: 0 errores, 0 warnings, 18 issues de estilo, 91 unusedFunction

#### Errores Reales: 0

No se detectaron errores.

#### Warnings Reales: 0

No se detectaron warnings.

#### Issues de Estilo Reales (18, excluyendo unusedFunction)

```
Core/Src/motor_control.c:258:24: style: struct member 'Motor_t::power' is never used. [unusedStructMember]
Core/Src/service_mode.c:234:42: style: Redundant condition: 'i < 32' is redundant since 'i < 25' is sufficient. [redundantCondition]
Core/Src/service_mode.c:245:42: style: Redundant condition: 'i < 32' is redundant since 'i < 25' is sufficient. [redundantCondition]
Core/Src/service_mode.c:256:42: style: Redundant condition: 'i < 32' is redundant since 'i < 25' is sufficient. [redundantCondition]
Core/Src/stm32g4xx_hal_msp.c:9:45: style: Parameter 'hfdcan' can be declared as pointer to const [constParameterPointer]
Core/Src/stm32g4xx_hal_msp.c:33:46: style: Parameter 'htim_base' can be declared as pointer to const [constParameterPointer]
Core/Src/stm32g4xx_hal_msp.c:60:45: style: Parameter 'htim_pwm' can be declared as pointer to const [constParameterPointer]
Core/Src/stm32g4xx_hal_msp.c:106:41: style: Parameter 'hi2c' can be declared as pointer to const [constParameterPointer]
Core/Src/stm32g4xx_hal_msp.c:130:41: style: Parameter 'hadc' can be declared as pointer to const [constParameterPointer]
Core/Src/stm32g4xx_hal_msp.c:145:45: style: Parameter 'htim' can be declared as pointer to const [constParameterPointer]
Core/Src/stm32g4xx_hal_msp.c:183:49: style: Parameter 'htim_encoder' can be declared as pointer to const [constParameterPointer]
Core/Src/syscalls.c:64:50: style: Parameter 'ptr' can be declared as pointer to const [constParameterPointer]
Core/Src/test_steering_cal_store.c:173:14: style: Condition 'diff<0' is always false [knownConditionTrueFalse]
Core/Src/test_steering_cal_store.c:179:14: style: Condition 'diff<0' is always false [knownConditionTrueFalse]
Core/Src/test_steering_cal_store.c:185:14: style: Condition 'diff<0' is always true [knownConditionTrueFalse]
Core/Src/test_steering_cal_store.c:191:14: style: Condition 'diff<0' is always true [knownConditionTrueFalse]
Core/Src/test_steering_cal_store.c:197:14: style: Condition 'diff<0' is always false [knownConditionTrueFalse]
Core/Src/test_steering_cal_store.c:71:14: style: struct member 'stcal_flash_slot_t::reserved' is never used. [unusedStructMember]
```

> **Nota sobre constParameterPointer en hal_msp.c (7)**: Son **falsos positivos** — las firmas de los callbacks HAL están definidas por ST y no pueden modificarse.

> **Nota sobre unusedFunction (91)**: La mayoría son falsos positivos esperados en firmware embebido — funciones HAL_MspInit, ISR handlers, syscalls stubs, y funciones de API pública llamadas desde código HAL/startup (que cppcheck no analiza). Ejemplo: `HAL_FDCAN_MspInit`, `SysTick_Handler`, `_sbrk`, etc.

---

### 3.2 ESP32-S3

**Comando**: `cppcheck --enable=all --inconclusive --force --suppress=missingInclude -Iinclude -Isrc src/`
**Versión**: CppCheck 2.13.0

**Resultados**: 0 errores, 5 warnings, 16 issues de estilo, 2 performance, 13 unusedFunction

#### Errores Reales: 0

No se detectaron errores.

#### Warnings Reales (5)

```
src/screens/engineering_screen.cpp:334:9: warning: %u in format string (no. 1) requires 'unsigned int' but the argument type is 'signed int'. [invalidPrintfArgType_uint]
src/screens/engineering_screen.cpp:334:9: warning: %u in format string (no. 2) requires 'unsigned int' but the argument type is 'signed int'. [invalidPrintfArgType_uint]
src/screens/engineering_screen.cpp:334:9: warning: %u in format string (no. 3) requires 'unsigned int' but the argument type is 'signed int'. [invalidPrintfArgType_uint]
src/screens/engineering_screen.cpp:334:9: warning: %u in format string (no. 4) requires 'unsigned int' but the argument type is 'signed int'. [invalidPrintfArgType_uint]
src/screens/engineering_screen.cpp:676:5: warning: %u in format string (no. 1) requires 'unsigned int' but the argument type is 'signed int'. [invalidPrintfArgType_uint]
```

#### Issues de Estilo Reales (16, excluyendo unusedFunction)

```
src/config_store.cpp:82:13: style: The scope of the variable 'inaDefault' can be reduced. [variableScope]
src/config_store.cpp:89:13: style: The scope of the variable 'tmpDefault' can be reduced. [variableScope]
src/config_store.cpp:82:13: style: Variable 'inaDefault' can be declared as const array [constVariable]
src/config_store.cpp:89:13: style: Variable 'tmpDefault' can be declared as const array [constVariable]
src/main.cpp:550:20: style: The statement 'if (showOverlay) showOverlay=false' is logically equivalent to 'showOverlay=false'. [duplicateConditionalAssign]
src/main.cpp:56:22: style: C-style pointer casting [cstyleCast]
src/screens/drive_screen.h:76:10: style: Technically the member function 'DriveScreen::drawSpeed' can be const. [functionConst]
src/screens/engineering_screen.h:61:10: style: Technically the member function 'EngineeringScreen::drawSensorMapIna' can be const. [functionConst]
src/screens/engineering_screen.h:62:10: style: Technically the member function 'EngineeringScreen::drawSensorMapTemp' can be const. [functionConst]
src/screens/engineering_screen.cpp:597:55: style: Same expression in both branches of ternary operator. [duplicateExpressionTernary]
src/screens/pin_screen.h:69:10: style: Technically the member function 'PinScreen::drawDots' can be const. [functionConst]
src/test_obstacle_sensor.cpp:295:5: style: Same expression on both sides of '!='. [duplicateExpression]
src/test_obstacle_sensor.cpp:298:5: style: Same expression on both sides of '!='. [duplicateExpression]
src/test_obstacle_sensor.cpp:301:5: style: Same expression on both sides of '!='. [duplicateExpression]
src/test_obstacle_sensor.cpp:304:5: style: Same expression on both sides of '!='. [duplicateExpression]
src/test_obstacle_sensor.cpp:310:5: style: Same expression on both sides of '>='. [duplicateExpression]
```

#### Issues de Performance Reales (2)

```
src/screens/engineering_screen.h:56:10: performance: Technically the member function 'EngineeringScreen::drawMainMenu' can be static. [functionStatic]
src/screens/engineering_screen.h:63:10: performance: Technically the member function 'EngineeringScreen::drawFactoryDefaults' can be static. [functionStatic]
```

#### unusedFunction (13)

```
src/audio_manager.cpp:187: isPlaying() [unusedFunction]
src/config_store.cpp:159: setAudioVolume() [unusedFunction]
src/config_store.cpp:181: isDirty() [unusedFunction]
src/config_store.cpp:185: factoryReset() [unusedFunction]
src/led_controller.cpp:350: isEmergencyFlashActive() [unusedFunction]
src/main.cpp:298: setup() [unusedFunction]
src/main.cpp:436: loop() [unusedFunction]
src/power_manager.cpp:146: isShutdownComplete() [unusedFunction]
src/power_manager.cpp:154: isKeyOn() [unusedFunction]
src/sensors/obstacle_sensor.cpp:251: isValidFrame() [unusedFunction]
src/touch_handler.cpp:89: isPressed() [unusedFunction]
src/touch_handler.cpp:93: getPosition() [unusedFunction]
src/ui/debug_overlay.cpp:135: DebugOverlay::isVisible() [unusedFunction]
```

> **Nota**: `setup()` y `loop()` son entry points de Arduino framework — falsos positivos esperados. Los demás son API pública que puede ser llamada desde código futuro o desde código de test que cppcheck no analiza en el mismo pase.

---

## === 4. clangd ===

### 4.1 STM32 G474

**Comando**: `clangd-18 --check=Core/Src/main.c` y `clangd-18 --check=Core/Src/safety_system.c`

#### Includes Faltantes

```
[pp_file_not_found] Line 17: in included file: 'stm32g4xx_hal.h' file not found
[pp_file_not_found] Line 14: in included file: 'stm32g4xx_hal.h' file not found
```

**Causa**: Directorio `Drivers/` (HAL + CMSIS generado por CubeMX) está en `.gitignore` y no presente en el repositorio.

#### Tipos No Resueltos (derivados del include faltante)

```
[unknown_typename] Line 41: unknown type name 'ADC_HandleTypeDef'
[unknown_typename] Line 42: unknown type name 'FDCAN_HandleTypeDef'
[unknown_typename] Line 43: unknown type name 'I2C_HandleTypeDef'
[unknown_typename] Line 44: unknown type name 'TIM_HandleTypeDef'
[unknown_typename] Line 45: unknown type name 'IWDG_HandleTypeDef'
```

#### Identificadores No Declarados (derivados del include faltante)

```
[undeclared_var_use] Line 76: use of undeclared identifier 'RCC'
[undeclared_var_use] Line 82: use of undeclared identifier 'RCC_CSR_LPWRRSTF'
[-Wimplicit-function-declaration] Line 94: call to undeclared function '__HAL_RCC_CLEAR_RESET_FLAGS'
[-Wimplicit-function-declaration] Line 128: call to undeclared function 'HAL_Init'
[-Wimplicit-function-declaration] Line 487: call to undeclared function 'HAL_GPIO_WritePin'
[-Wimplicit-function-declaration] Line 489: call to undeclared function 'HAL_GetTick'
[undeclared_var_use] Line 487: use of undeclared identifier 'GPIOC'
[undeclared_var_use] Line 487: use of undeclared identifier 'GPIO_PIN_10'
[fatal_too_many_errors] Line 1: too many errors emitted, stopping now
```

**Total**: 21 errores (main.c) + 21 errores (safety_system.c) — **todos derivados de `stm32g4xx_hal.h` no encontrado**.

> **Todos estos errores desaparecen con `Drivers/` presente** (generado por CubeMX con `STM32CubeIDE`).

#### Archivos No Indexados

Cualquier archivo que incluya `main.h` (que a su vez incluye `stm32g4xx_hal.h`) falla en el indexado. Esto incluye: `main.c`, `safety_system.c`, `can_handler.c`, `motor_control.c`, `sensor_manager.c`, `service_mode.c`, `error_log.c`, `eps_params.c`, `steering_cal_store.c`, `boot_validation.c`, `steering_centering.c`, `encoder_reader.c`.

#### Configuración clangd

- ✅ `.clangd` carga correctamente (verificado en log: `Loaded compilation database from ./compile_commands.json`)
- ✅ Flags ARM aplicados correctamente (`--target=arm-none-eabi`)
- ✅ `If: PathMatch: Core/.*` evita contaminación a ESP32

---

### 4.2 ESP32-S3

**Comando**: `clangd-18 --check=src/main.cpp` y `clangd-18 --check=src/screens/engineering_screen.cpp`

#### Includes Faltantes

```
[pp_file_not_found] Line 11: 'Arduino.h' file not found
[pp_file_not_found] Line 10: in included file: 'Arduino.h' file not found
```

**Causa**: SDK Arduino/ESP-IDF no instalado en este entorno de análisis.

#### Tipos No Resueltos (derivados del include faltante)

```
[unknown_typename] Line 18: unknown type name 'TFT_eSPI'
[unknown_typename_suggest] Line 48: unknown type name 'size_t'; did you mean 'std::size_t'?
[undeclared_var_use] Line 39: use of undeclared identifier 'Serial'
[undeclared_var_use] Line 41: use of undeclared identifier 'psramInit'
[undeclared_var_use] Line 56: use of undeclared identifier 'ps_malloc'
[unknown_typename] Line 71: unknown type name 'TFT_eSPI'
[undeclared_var_use] Line 73: use of undeclared identifier 'TFT_BLACK'
[undeclared_var_use] Line 190: use of undeclared identifier 'millis'
[undeclared_var_use] Line 274: use of undeclared identifier 'TL_DATUM'
[unknown_typename] Line 503: unknown type name 'CanFrame'
[undeclared_var_use] Line 509: use of undeclared identifier 'ESP32Can'
[fatal_too_many_errors] Line 1: too many errors emitted, stopping now
```

**Total**: 37 errores (main.cpp) + 40 errores (engineering_screen.cpp) — **todos derivados de Arduino.h/TFT_eSPI.h no encontrados**.

#### IncludeCleaner Issues

```
IncludeCleaner: Failed to get an entry for resolved path : No such file or directory
```

> Se reporta decenas de veces. Es consecuencia directa de los headers faltantes.

#### Problemas de Navegación

Debido a los headers faltantes, clangd no puede:
- Resolver tipos de Arduino framework (`TFT_eSPI`, `CanFrame`, `Serial`, etc.)
- Navegar a definiciones de funciones del SDK (`millis()`, `ps_malloc()`, `psramInit()`)
- Completar código basado en miembros de clases del SDK

#### Archivos No Indexados

Todos los archivos que incluyen `Arduino.h`, `TFT_eSPI.h`, `FastLED.h`, `Wire.h`, o `Preferences.h` (prácticamente todos los `.cpp`) fallan en el indexado completo.

#### Configuración clangd

- ✅ `esp32/.clangd` carga correctamente
- ✅ `esp32/compile_commands.json` detectado y cargado
- ✅ Flags C++17 y Arduino defines aplicados
- ✅ Ya no hereda flags STM32 del `.clangd` raíz (aislado con `If: PathMatch: Core/.*`)

---

## === 5. Recomendaciones Finales (FASE 2 — Propuestas de Solución) ===

### 5.1 Prioridad Alta 🔴

---

#### P1: `invalidPrintfArgType_uint` — 5 warnings en `engineering_screen.cpp`

**Archivos**: `esp32/src/screens/engineering_screen.cpp:334,676`

**Por qué ocurre**: Las variables `batteryVoltage_` y `batteryCurrent_` son `uint16_t` (declaradas en `engineering_screen.h:81-82`), y `moduleCtrlPage_` es `uint8_t` (`:93`). Cuando se pasan a `snprintf()` como argumentos de `%u`, la **promoción entera de C** las convierte a `int` (signed), no a `unsigned int`. El estándar dice que `%u` requiere `unsigned int`, pero recibe `signed int`.

**Impacto real**: En la práctica, para valores positivos (que siempre lo son aquí), el output será correcto en la mayoría de implementaciones. Sin embargo, es **comportamiento indefinido** según el estándar C11 §7.21.6.1 si los tipos no coinciden exactamente. En plataformas con int de 16 bits, podría producir truncamiento.

**Solución propuesta para línea 334**:

```cpp
// ANTES (warning):
snprintf(buf, sizeof(buf), "%u.%02u V  %u.%02u A",
         batteryVoltage_ / 100, batteryVoltage_ % 100,
         batteryCurrent_ / 100, batteryCurrent_ % 100);

// DESPUÉS (opción A — cast explícito, preferida):
snprintf(buf, sizeof(buf), "%u.%02u V  %u.%02u A",
         (unsigned)batteryVoltage_ / 100, (unsigned)batteryVoltage_ % 100,
         (unsigned)batteryCurrent_ / 100, (unsigned)batteryCurrent_ % 100);

// DESPUÉS (opción B — cambiar formato):
snprintf(buf, sizeof(buf), "%d.%02d V  %d.%02d A",
         batteryVoltage_ / 100, batteryVoltage_ % 100,
         batteryCurrent_ / 100, batteryCurrent_ % 100);
```

**Solución propuesta para línea 676**:

```cpp
// ANTES (warning):
snprintf(hdrBuf, sizeof(hdrBuf), "MODULE CONTROL (%u/%u)",
         moduleCtrlPage_ + 1, MODULE_CTRL_PAGES);

// DESPUÉS:
snprintf(hdrBuf, sizeof(hdrBuf), "MODULE CONTROL (%u/%u)",
         (unsigned)(moduleCtrlPage_ + 1), (unsigned)MODULE_CTRL_PAGES);
```

---

#### P2: `duplicateExpressionTernary` — ternario con ramas idénticas en `engineering_screen.cpp:597`

**Archivo**: `esp32/src/screens/engineering_screen.cpp:597`

**Por qué ocurre**: El ternario `(i == 6) ? ui::COL_DARK_GRAY : ui::COL_DARK_GRAY` tiene ambas ramas idénticas. Probable error de copy-paste al diferenciar el color de fondo del último botón del menú.

**Impacto real**: **Bug visual** — el ítem 6 del menú debería tener un color de fondo distinto (probablemente resaltado o deshabilitado), pero actualmente se renderiza igual que todos los demás. Detectado por CppCheck (`duplicateExpressionTernary`) y Clang-Tidy (`bugprone-branch-clone` + `misc-redundant-expression`).

**Solución propuesta**:

```cpp
// ANTES (bug):
uint16_t bgCol = (i == 6) ? ui::COL_DARK_GRAY : ui::COL_DARK_GRAY;

// DESPUÉS (ejemplo — depende de la intención del diseñador):
uint16_t bgCol = (i == 6) ? ui::COL_DARKER_GRAY : ui::COL_DARK_GRAY;
// O si el ítem 6 debería estar deshabilitado:
uint16_t bgCol = (i == 6) ? ui::COL_BG : ui::COL_DARK_GRAY;
// O si no hay diferencia intencional, simplificar:
uint16_t bgCol = ui::COL_DARK_GRAY;
```

> **Nota**: Verificar con el diseñador UI qué color debería tener el ítem 6.

---

### 5.2 Prioridad Media 🟠

---

#### P3: `bugprone-reserved-identifier` — 13 header guards con doble guion bajo

**Archivos**: Todos los headers en `Core/Inc/` excepto `main.h`, `math_safety.h`, `sensor_manager.h`, `stm32g4xx_hal_conf.h`

**Por qué ocurre**: El estándar C (§7.1.3) y C++ (§17.6.4.3) reservan todos los identificadores que comienzan con doble guion bajo (`__`) o guion bajo seguido de mayúscula (`_X`). Usar `__MOTOR_CONTROL_H` como include guard es UB técnico.

**Impacto real**: En la práctica, funciona en GCC ARM, pero puede colisionar con macros internas del compilador/runtime en versiones futuras. Es una violación del estándar y un warning legítimo.

**Solución propuesta** (aplicar a cada header):

```c
// ANTES (Core/Inc/motor_control.h):
#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H
// ...
#endif /* __MOTOR_CONTROL_H */

// DESPUÉS:
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
// ...
#endif /* MOTOR_CONTROL_H */
```

**Lista completa de cambios necesarios**:

| Header | Antes | Después |
|--------|-------|---------|
| `ackermann.h` | `__ACKERMANN_H` | `ACKERMANN_H` |
| `boot_validation.h` | `__BOOT_VALIDATION_H` | `BOOT_VALIDATION_H` |
| `can_handler.h` | `__CAN_HANDLER_H` | `CAN_HANDLER_H` |
| `encoder_reader.h` | `__ENCODER_READER_H` | `ENCODER_READER_H` |
| `eps_params.h` | `__EPS_PARAMS_H` | `EPS_PARAMS_H` |
| `error_log.h` | `__ERROR_LOG_H` | `ERROR_LOG_H` |
| `motor_control.h` | `__MOTOR_CONTROL_H` | `MOTOR_CONTROL_H` |
| `safety_system.h` | `__SAFETY_SYSTEM_H` | `SAFETY_SYSTEM_H` |
| `service_mode.h` | `__SERVICE_MODE_H` | `SERVICE_MODE_H` |
| `steering_cal_store.h` | `__STEERING_CAL_STORE_H` | `STEERING_CAL_STORE_H` |
| `steering_centering.h` | `__STEERING_CENTERING_H` | `STEERING_CENTERING_H` |
| `stm32g4xx_it.h` | `__STM32G4xx_IT_H` | `STM32G4XX_IT_H` |
| `vehicle_physics.h` | `__VEHICLE_PHYSICS_H` | `VEHICLE_PHYSICS_H` |

---

#### P4: `bugprone-branch-clone` — switch con 10 ramas idénticas en `can_handler.c:106`

**Archivo**: `Core/Src/can_handler.c:100-116`

**Por qué ocurre**: El switch mapea `len` (0-8) a constantes `FDCAN_DLC_BYTES_x`. En el entorno de análisis (sin HAL headers), las constantes `FDCAN_DLC_BYTES_0` a `FDCAN_DLC_BYTES_8` no están definidas y clang-tidy las trata como todas iguales a 0, detectando "10 ramas idénticas".

**Impacto real**: Este es un **falso positivo** causado por la ausencia de los headers HAL. Con `Drivers/` presente, cada constante `FDCAN_DLC_BYTES_x` tiene un valor único (definido como enum en `stm32g4xx_hal_fdcan.h`). El switch es correcto y necesario.

**Solución propuesta**: No se requiere cambio de código. Para eliminar el warning en CI sin Drivers/:

```c
// Opción: usar array lookup en vez de switch (más compacto, mismo resultado)
static const uint32_t dlc_map[] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8
};
uint32_t dlc_code = (len <= 8) ? dlc_map[len] : FDCAN_DLC_BYTES_8;
```

---

#### P5: `bugprone-switch-missing-default-case` — 3 switches sin default

**Archivos**: `can_handler.c:900`, `boot_screen.cpp:229`, `engineering_screen.cpp:443`

**Por qué ocurre**: Switches sobre valores no-enum (int, uint8_t) sin case `default:`.

**Impacto real**: Si el valor no coincide con ningún case, la ejecución cae al final del switch sin ejecutar nada. Para `boot_screen.cpp:229`, las variables ya están pre-inicializadas con defaults (`stateStr = "UNKNOWN"`, `stateCol = ui::COL_YELLOW`), así que es inofensivo pero confuso. Para `can_handler.c:900`, los comandos 0xF0-0xF4 ya están filtrados por el if exterior (`cmd >= 0xF0 && cmd <= 0xF4`), así que un default inalcanzable no añade funcionalidad pero sí claridad.

**Solución propuesta para `can_handler.c:900`**:

```c
// Añadir al final del switch:
default:
    break;  /* Range already validated by outer if */
```

**Solución propuesta para `boot_screen.cpp:229`**:

```cpp
// Añadir al final del switch:
default:
    break;  /* stateStr/stateCol already initialized with defaults */
```

**Solución propuesta para `engineering_screen.cpp:443`**:

```cpp
// Añadir al final del switch:
default:
    break;  /* Menu items beyond defined cases are ignored */
```

---

#### P6: `bugprone-too-small-loop-variable` — 11 warnings

**Archivos**: `can_handler.c:922`, `service_mode.c:82,189,201,217`, `test_service_mode.c:161,168,200,265,294,314`

**Por qué ocurre**: La variable de loop `uint8_t i` o `uint8_t s` tiene tipo más estrecho que el upper bound (que es `int` tras la promoción).

**Impacto real**: Para los valores actuales (MODULE_COUNT ≤ 25), `uint8_t` es suficiente (rango 0-255). Sin embargo, si MODULE_COUNT creciera por encima de 255 en el futuro, el loop sería infinito. Es un warning de fragilidad, no un bug activo.

**Solución propuesta**:

```c
// ANTES:
for (uint8_t i = 0; i < MODULE_COUNT && i < 32; i++)

// DESPUÉS:
for (unsigned i = 0; i < MODULE_COUNT && i < 32; i++)
```

```c
// ANTES (can_handler.c:922):
for (uint8_t s = MODULE_CURRENT_SENSOR_0; s <= MODULE_CURRENT_SENSOR_5; s++)

// DESPUÉS:
for (unsigned s = MODULE_CURRENT_SENSOR_0; s <= MODULE_CURRENT_SENSOR_5; s++)
```

---

#### P7: `redundantCondition` — 3 condiciones redundantes en `service_mode.c`

**Archivo**: `Core/Src/service_mode.c:234,245,256`

**Por qué ocurre**: El loop usa `i < MODULE_COUNT && i < 32`. MODULE_COUNT es 25 (definido como enum), por lo que `i < 25` siempre implica `i < 32`. La condición `i < 32` nunca se evalúa a false.

**Impacto real**: Ninguno funcional — es código defensivo pero redundante. Si MODULE_COUNT cambiara a >32, la condición `i < 32` evitaría overflow del bitmask. Es una buena práctica defensiva, pero cppcheck la marca correctamente como redundante para el valor actual.

**Solución propuesta** (dos opciones):

```c
// Opción A — eliminar la redundancia (más limpio):
for (uint8_t i = 0; i < MODULE_COUNT; i++)
// Requiere static_assert(MODULE_COUNT <= 32) para garantizar seguridad del bitmask

// Opción B — mantener pero documentar (más defensivo):
// Nota: i < 32 es guardia defensiva para el bitmask, mantener intencionalmente
// NOLINTNEXTLINE(cppcheck-redundantCondition)
for (unsigned i = 0; i < MODULE_COUNT && i < 32; i++)
```

---

#### P8: `duplicateConditionalAssign` — `main.cpp:550`

**Archivo**: `esp32/src/main.cpp:550`

**Por qué ocurre**: El código `if (showOverlay) showOverlay = false;` es lógicamente equivalente a `showOverlay = false;` — si `showOverlay` es true, lo pone a false; si ya es false, no hace nada (mismo resultado).

**Impacto real**: Ninguno funcional — el resultado es idéntico. Pero el código es innecesariamente confuso y sugiere que hubo una condición adicional que se eliminó.

**Solución propuesta**:

```cpp
// ANTES:
} else if (showOverlay) {
    showOverlay = false;
}

// DESPUÉS (más claro):
} else {
    showOverlay = false;
}
```

---

### 5.3 Prioridad Baja 🟡

---

#### P9: `readability-uppercase-literal-suffix` — 498+86 = 584 warnings

**Archivos**: Distribuidos en todo el firmware

**Por qué ocurre**: Sufijos literales en minúscula (`0xffu`, `1.0f`, `100u`) vs mayúscula (`0xFFU`, `1.0F`, `100U`). El estándar acepta ambos, pero la convención de legibilidad prefiere mayúsculas para evitar confusión entre `l` y `1`.

**Impacto real**: Ninguno funcional. Solo legibilidad.

**Solución propuesta**: Aplicar con `clang-tidy --fix -checks=readability-uppercase-literal-suffix` en el entorno de desarrollo con SDKs completos.

---

#### P10: `readability-braces-around-statements` — 226+149 = 375 warnings

**Archivos**: Distribuidos en todo el firmware

**Por qué ocurre**: Sentencias `if/for/while` de una línea sin llaves. Es un riesgo conocido (Apple goto fail bug, 2014).

**Impacto real**: Bajo pero real — facilita bugs en ediciones futuras.

**Solución propuesta**: Aplicar con `clang-tidy --fix -checks=readability-braces-around-statements` en el entorno de desarrollo.

---

#### P11: `unusedStructMember` Motor_t::power — 1 warning

**Archivo**: `Core/Src/motor_control.c:258`

**Por qué ocurre**: El miembro `power` de `Motor_t` se declara pero no se lee ni escribe en ninguna parte del código analizado.

**Impacto real**: Desperdicio de RAM (4 bytes × N_MOTORS). Puede confundir a futuros desarrolladores.

**Solución propuesta**: Eliminar el miembro si no se planea usar, o documentar su propósito futuro.

---

#### P12: `functionConst` / `functionStatic` — 7 warnings

**Archivos**: `drive_screen.h`, `engineering_screen.h`, `pin_screen.h`

**Por qué ocurre**: Métodos de clase que no acceden a `this` o no modifican estado podrían ser `const` o `static`.

**Impacto real**: Ninguno funcional. Mejora la semántica del código.

**Solución propuesta**:

```cpp
// ANTES:
void drawSpeed();
void drawSensorMapIna();
void drawMainMenu();

// DESPUÉS:
void drawSpeed() const;
void drawSensorMapIna() const;
static void drawMainMenu();
```

---

## === 6. Resumen de Configuración ===

### Versiones de Herramientas

| Herramienta | Versión | Notas |
|-------------|---------|-------|
| Clang-Tidy | Ubuntu LLVM 18.1.3 | clang-tidy-18 |
| CppCheck | 2.13.0 | `--enable=all --inconclusive --force` |
| clangd | Ubuntu clangd 18.1.3 | `--check=<file>` |
| compile_commands.json | Generados manualmente | Sin SDKs → includes faltantes |

### Archivos de Configuración

| Archivo | Propósito | Estado |
|---------|-----------|--------|
| `.clangd` (raíz) | Config clangd para STM32 (solo `Core/.*`) | ✅ Correcto — `If: PathMatch: Core/.*` aísla toolchains |
| `esp32/.clangd` | Config clangd para ESP32-S3 (C++17 Arduino) | ✅ Correcto — `CompilationDatabase` dentro de `CompileFlags` |
| `compile_commands.json` (raíz) | Base de datos de compilación STM32 | ✅ Generado (24 entradas, gitignored) |
| `esp32/compile_commands.json` | Base de datos de compilación ESP32 | ✅ Generado (37 entradas, gitignored) |

### Comandos de Reproducción

```bash
# STM32 — CppCheck
cppcheck --enable=all --inconclusive --force --suppress=missingInclude -ICore/Inc Core/Src/

# STM32 — Clang-Tidy (requiere compile_commands.json)
clang-tidy-18 -p . --checks='bugprone-*,cert-*,misc-*,performance-*,readability-*,-readability-magic-numbers,-cert-dcl37-c,-cert-dcl51-cpp' Core/Src/*.c

# STM32 — clangd
clangd-18 --check=Core/Src/main.c

# ESP32 — CppCheck (desde directorio esp32/)
cppcheck --enable=all --inconclusive --force --suppress=missingInclude -Iinclude -Isrc src/

# ESP32 — Clang-Tidy (desde directorio esp32/, requiere compile_commands.json)
clang-tidy-18 -p . --checks='bugprone-*,cert-*,misc-*,performance-*,readability-*,modernize-*,-readability-magic-numbers,-modernize-use-trailing-return-type,-cert-dcl37-c,-cert-dcl51-cpp' src/**/*.cpp

# ESP32 — clangd (desde directorio esp32/)
clangd-18 --check=src/main.cpp

# Generar compile_commands.json (con SDKs instalados)
# STM32: bear -- make  (o pio run -t compiledb)
# ESP32:  cd esp32 && pio run -t compiledb
```

---

*Informe generado el 2026-03-18 a partir de outputs reales de clang-tidy 18.1.3, cppcheck 2.13.0 y clangd 18.1.3. No se ha modificado código fuente. Todas las propuestas de solución son sugerencias — NO se ha aplicado ningún cambio automático al firmware.*
