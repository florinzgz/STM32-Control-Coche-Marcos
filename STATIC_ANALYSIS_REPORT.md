# Informe de Análisis Estático — Dual Firmware

> **Fecha**: 2026-03-18
> **Herramientas**: Clang-Tidy 18.1.3, CppCheck 2.13.0, clangd 18.1.3
> **Repositorio**: STM32-Control-Coche-Marcos
> **Nota**: Este informe se basa EXCLUSIVAMENTE en los outputs reales de las herramientas ejecutadas. Ningún resultado ha sido inventado ni asumido.

---

## 1. Resumen Ejecutivo

### Estado General

| Proyecto | Herramienta | Errores | Warnings | Estilo | Performance |
|----------|-------------|---------|----------|--------|-------------|
| **STM32 G474** | CppCheck | 0 | 0 | 18 + 91 unusedFunction | 0 |
| **STM32 G474** | Clang-Tidy | 6 (headers faltantes) | 865 | — | — |
| **ESP32-S3** | CppCheck | 0 | 5 | 16 + 13 unusedFunction | 2 |
| **ESP32-S3** | Clang-Tidy | 25 (headers faltantes) | 1,317 | — | — |

### Riesgos Reales Detectados

1. **Printf format mismatch** (ESP32): `%u` con `signed int` en `engineering_screen.cpp` — puede producir output incorrecto con valores negativos.
2. **Ternario duplicado** (ESP32): `(i == 6) ? COL_DARK_GRAY : COL_DARK_GRAY` — ambas ramas idénticas, probable copy-paste error.
3. **Reserved identifiers** (STM32): 13 header guards con doble guion bajo (`__MOTOR_CONTROL_H`, etc.) — reservados por el estándar C.
4. **Narrowing conversions**: 26 en STM32, 34 en ESP32 — conversiones implícitas que pueden perder datos.
5. **Branch clones**: 10 ramas idénticas consecutivas en `can_handler.c:106` — lógica posiblemente incompleta.
6. **Toolchain cross-contamination** (corregido): `.clangd` raíz añadía flags STM32 al proyecto ESP32.

### Recomendaciones Basadas en Resultados Reales

1. Corregir los 5 warnings de `invalidPrintfArgType_uint` en `engineering_screen.cpp`.
2. Corregir el ternario duplicado en `engineering_screen.cpp:597`.
3. Renombrar header guards de `__XXX_H` a `XXX_H` en todos los headers STM32.
4. Revisar el switch con 10 ramas idénticas en `can_handler.c:106`.
5. Añadir `default:` a los switches sin él en `can_handler.c:900`, `boot_screen.cpp:229`, `engineering_screen.cpp:443`.

---

## 2. Verificación de compile_commands.json

| Proyecto | Estado | Método de Generación | Entradas |
|----------|--------|---------------------|----------|
| STM32 G474 | ✅ Generado | Script Python desde flags del Makefile | 24 archivos .c |
| ESP32-S3 | ✅ Generado | Script Python desde flags de platformio.ini | 37 archivos .cpp |

> **Nota**: `pio run -t compiledb` no puede ejecutarse en este entorno (sin acceso a internet para descargar el SDK de Espressif). Los `compile_commands.json` se generaron manualmente a partir de los flags reales definidos en `Makefile` y `platformio.ini`. Ambos están en `.gitignore`.

---

## 3. clang-tidy

### 3.1 STM32 G474 (ARM C)

**Total**: 865 warnings + 6 errores de compilación (headers HAL faltantes — esperado sin Drivers/)

#### Desglose por Categoría (output real)

| Check | Count |
|-------|-------|
| `readability-uppercase-literal-suffix` | 395 |
| `readability-braces-around-statements` | 211 |
| `readability-identifier-length` | 107 |
| `misc-include-cleaner` | 61 |
| `bugprone-narrowing-conversions` | 26 |
| `bugprone-reserved-identifier` | 13 |
| `readability-function-cognitive-complexity` | 12 |
| `bugprone-easily-swappable-parameters` | 10 |
| `bugprone-branch-clone` | 10 |
| `bugprone-too-small-loop-variable` | 5 |
| `misc-unused-parameters` | 4 |
| `performance-no-int-to-ptr` | 3 |
| `misc-redundant-expression` | 3 |
| `readability-redundant-declaration` | 2 |
| `readability-isolate-declaration` | 1 |
| `readability-else-after-return` | 1 |
| `bugprone-switch-missing-default-case` | 1 |

#### Errores de Compilación (esperados — Drivers/ es gitignored)

```
Core/Src/ackermann.c:18:10: error: 'math.h' file not found
Core/Src/eps_params.c:19:10: error: 'stm32g4xx_hal.h' file not found
Core/Src/error_log.c:27:10: error: 'stm32g4xx_hal.h' file not found
Core/Src/steering_cal_store.c:21:10: error: 'stm32g4xx_hal.h' file not found
Core/Inc/main.h:8:10: error: 'stm32g4xx_hal.h' file not found
Core/Inc/math_safety.h:18:10: error: 'math.h' file not found
```

#### Ejemplos Reales — bugprone-reserved-identifier (13)

Todos los headers Core/Inc/ usan `#define __XXX_H` como include guard:

```
Core/Inc/ackermann.h:16:9: warning: declaration uses identifier '__ACKERMANN_H', which is a reserved identifier [bugprone-reserved-identifier]
Core/Inc/can_handler.h:10:9: warning: declaration uses identifier '__CAN_HANDLER_H', which is a reserved identifier [bugprone-reserved-identifier]
Core/Inc/motor_control.h:2:9: warning: declaration uses identifier '__MOTOR_CONTROL_H', which is a reserved identifier [bugprone-reserved-identifier]
Core/Inc/safety_system.h:9:9: warning: declaration uses identifier '__SAFETY_SYSTEM_H', which is a reserved identifier [bugprone-reserved-identifier]
Core/Inc/encoder_reader.h:14:9: warning: declaration uses identifier '__ENCODER_READER_H' [bugprone-reserved-identifier]
Core/Inc/eps_params.h:22:9: warning: declaration uses identifier '__EPS_PARAMS_H' [bugprone-reserved-identifier]
Core/Inc/error_log.h:30:9: warning: declaration uses identifier '__ERROR_LOG_H' [bugprone-reserved-identifier]
Core/Inc/service_mode.h:29:9: warning: declaration uses identifier '__SERVICE_MODE_H' [bugprone-reserved-identifier]
Core/Inc/steering_cal_store.h:30:9: warning: declaration uses identifier '__STEERING_CAL_STORE_H' [bugprone-reserved-identifier]
Core/Inc/steering_centering.h:25:9: warning: declaration uses identifier '__STEERING_CENTERING_H' [bugprone-reserved-identifier]
Core/Inc/stm32g4xx_it.h:9:9: warning: declaration uses identifier '__STM32G4xx_IT_H' [bugprone-reserved-identifier]
Core/Inc/vehicle_physics.h:9:9: warning: declaration uses identifier '__VEHICLE_PHYSICS_H' [bugprone-reserved-identifier]
Core/Inc/boot_validation.h:27:9: warning: declaration uses identifier '__BOOT_VALIDATION_H' [bugprone-reserved-identifier]
```

#### Ejemplos Reales — bugprone-branch-clone (10)

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
```

#### Ejemplos Reales — bugprone-too-small-loop-variable (5)

```
Core/Src/can_handler.c:922:38: warning: loop variable has narrower type 'uint8_t' than iteration's upper bound 'int' [bugprone-too-small-loop-variable]
Core/Src/service_mode.c:82:25: warning: loop variable has narrower type 'uint8_t' than iteration's upper bound 'int'
Core/Src/service_mode.c:189:25: warning: loop variable has narrower type 'uint8_t' than iteration's upper bound 'int'
Core/Src/service_mode.c:201:25: warning: loop variable has narrower type 'uint8_t' than iteration's upper bound 'int'
Core/Src/service_mode.c:217:25: warning: loop variable has narrower type 'uint8_t' than iteration's upper bound 'int'
```

#### Ejemplo Real — bugprone-switch-missing-default-case (1)

```
Core/Src/can_handler.c:900:25: warning: switching on non-enum value without default case may not cover all cases [bugprone-switch-missing-default-case]
```

---

### 3.2 ESP32-S3 (Arduino C++17)

**Total**: 1,317 warnings + 25 errores de compilación (headers Arduino/ESP-IDF faltantes — esperado sin SDK instalado)

#### Desglose por Categoría (output real)

| Check | Count |
|-------|-------|
| `readability-identifier-length` | 251 |
| `misc-include-cleaner` | 225 |
| `misc-use-anonymous-namespace` | 181 |
| `readability-braces-around-statements` | 163 |
| `modernize-avoid-c-arrays` | 117 |
| `misc-const-correctness` | 113 |
| `cert-err33-c` | 41 |
| `modernize-use-nodiscard` | 37 |
| `bugprone-narrowing-conversions` | 34 |
| `readability-implicit-bool-conversion` | 25 |
| `readability-convert-member-functions-to-static` | 24 |
| `bugprone-easily-swappable-parameters` | 24 |
| `readability-uppercase-literal-suffix` | 22 |
| `modernize-use-auto` | 19 |
| `bugprone-branch-clone` | 12 |
| `readability-function-cognitive-complexity` | 6 |
| `modernize-use-default-member-init` | 4 |
| `readability-make-member-function-const` | 3 |
| `readability-simplify-boolean-expr` | 2 |
| `readability-redundant-declaration` | 2 |
| `readability-non-const-parameter` | 2 |
| `readability-isolate-declaration` | 2 |
| `modernize-loop-convert` | 2 |
| `bugprone-switch-missing-default-case` | 2 |
| `readability-redundant-member-init` | 1 |
| `readability-avoid-nested-conditional-operator` | 1 |
| `misc-redundant-expression` | 1 |
| `cert-err58-cpp` | 1 |

#### Errores de Compilación (esperados — SDK Arduino no instalado)

```
esp32/src/audio_manager.cpp:18:10: error: 'Arduino.h' file not found
esp32/src/can/can_obstacle.cpp:15:10: error: 'Arduino.h' file not found
esp32/src/can_rx.cpp:13:10: error: 'Arduino.h' file not found
esp32/src/config_store.cpp:9:10: error: 'Preferences.h' file not found
esp32/src/hmi/obstacle_indicator.h:15:10: error: 'TFT_eSPI.h' file not found
esp32/src/led_controller.cpp:21:10: error: 'FastLED.h' file not found
esp32/src/main.cpp:11:10: error: 'Arduino.h' file not found
(+ 18 más del mismo tipo)
```

#### Ejemplos Reales — bugprone-branch-clone (12)

```
esp32/src/led_controller.cpp:143:9: warning: switch has 5 consecutive identical branches [bugprone-branch-clone]
esp32/src/led_controller.cpp:161:9: warning: switch has 3 consecutive identical branches [bugprone-branch-clone]
esp32/src/led_controller.cpp:188:9: warning: switch has 3 consecutive identical branches [bugprone-branch-clone]
esp32/src/main.cpp:168:9: warning: switch has 5 consecutive identical branches [bugprone-branch-clone]
esp32/src/main.cpp:173:9: warning: switch has 2 consecutive identical branches [bugprone-branch-clone]
esp32/src/main.cpp:352:5: warning: if with identical then and else branches [bugprone-branch-clone]
esp32/src/main.cpp:379:9: warning: if with identical then and else branches [bugprone-branch-clone]
esp32/src/screen_manager.cpp:145:9: warning: switch has 3 consecutive identical branches [bugprone-branch-clone]
esp32/src/screen_manager.cpp:149:9: warning: switch has 2 consecutive identical branches [bugprone-branch-clone]
esp32/src/screens/engineering_screen.cpp:183:63: warning: repeated branch body in conditional chain [bugprone-branch-clone]
esp32/src/screens/engineering_screen.cpp:597:35: warning: conditional operator with identical true and false expressions [bugprone-branch-clone]
```

#### Ejemplo Real — misc-redundant-expression (1)

```
esp32/src/screens/engineering_screen.cpp:597:55: warning: 'true' and 'false' expressions are equivalent [misc-redundant-expression]
```

Este warning confirma el ternario duplicado: `(i == 6) ? ui::COL_DARK_GRAY : ui::COL_DARK_GRAY`

#### Ejemplos Reales — bugprone-switch-missing-default-case (2)

```
esp32/src/screens/boot_screen.cpp:229:13: warning: switching on non-enum value without default case may not cover all cases [bugprone-switch-missing-default-case]
esp32/src/screens/engineering_screen.cpp:443:21: warning: switching on non-enum value without default case may not cover all cases [bugprone-switch-missing-default-case]
```

---

## 4. CppCheck

### 4.1 STM32 G474

**CppCheck 2.13.0** — `cppcheck --enable=all --inconclusive --force --suppress=missingInclude -ICore/Inc Core/Src/`

**Resultados**: 0 errores, 0 warnings, 18 issues de estilo, 91 unusedFunction

#### Issues de Estilo Reales (no unusedFunction)

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

> **Nota sobre unusedFunction (91)**: La mayoría son falsos positivos esperados en firmware embebido — funciones HAL_MspInit, ISR handlers, syscalls stubs, y funciones de API pública llamadas desde código HAL/startup (que cppcheck no analiza). Ejemplo: `HAL_FDCAN_MspInit`, `SysTick_Handler`, `_sbrk`, etc.

> **Nota sobre constParameterPointer en hal_msp.c (7)**: Son falsos positivos — las firmas de los callbacks HAL están definidas por ST y no pueden modificarse.

---

### 4.2 ESP32-S3

**CppCheck 2.13.0** — `cppcheck --enable=all --inconclusive --force --suppress=missingInclude -Iinclude -Isrc src/`

**Resultados**: 0 errores, 5 warnings, 16 issues de estilo, 2 performance, 13 unusedFunction

#### Warnings Reales (5)

```
src/screens/engineering_screen.cpp:334:9: warning: %u in format string (no. 1) requires 'unsigned int' but the argument type is 'signed int'. [invalidPrintfArgType_uint]
src/screens/engineering_screen.cpp:334:9: warning: %u in format string (no. 2) requires 'unsigned int' but the argument type is 'signed int'. [invalidPrintfArgType_uint]
src/screens/engineering_screen.cpp:334:9: warning: %u in format string (no. 3) requires 'unsigned int' but the argument type is 'signed int'. [invalidPrintfArgType_uint]
src/screens/engineering_screen.cpp:334:9: warning: %u in format string (no. 4) requires 'unsigned int' but the argument type is 'signed int'. [invalidPrintfArgType_uint]
src/screens/engineering_screen.cpp:676:5: warning: %u in format string (no. 1) requires 'unsigned int' but the argument type is 'signed int'. [invalidPrintfArgType_uint]
```

Todos en `engineering_screen.cpp` — snprintf con `%u` pero argumento `signed int`.

#### Issues de Estilo Reales (16, sin unusedFunction)

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

> **Nota**: `setup()` y `loop()` son entry points de Arduino framework — falsos positivos esperados.

---

## 5. clangd Diagnostics

### 5.1 STM32 G474

**Ejecutado**: `clangd --check=Core/Src/main.c` y `clangd --check=Core/Src/safety_system.c`

#### Includes Faltantes

```
[pp_file_not_found] Line 17: in included file: 'stm32g4xx_hal.h' file not found
```

**Causa**: Directorio `Drivers/` (HAL + CMSIS generado por CubeMX) está en `.gitignore` y no presente en el repositorio. Esto es esperado.

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
[-Wimplicit-function-declaration] Line 487: call to undeclared function 'HAL_GPIO_WritePin'
[-Wimplicit-function-declaration] Line 489: call to undeclared function 'HAL_GetTick'
[undeclared_var_use] Line 487: use of undeclared identifier 'GPIOC'
[undeclared_var_use] Line 487: use of undeclared identifier 'GPIO_PIN_10'
```

> **Todos estos errores desaparecen con `Drivers/` presente** (generado por CubeMX).

#### Configuración clangd

- ✅ `.clangd` carga correctamente
- ✅ `compile_commands.json` detectado y cargado
- ✅ Flags ARM aplicados correctamente
- ✅ `If: PathMatch: Core/.*` evita contaminación a ESP32

### 5.2 ESP32-S3

**Ejecutado**: `clangd --check=src/main.cpp` y `clangd --check=src/screens/engineering_screen.cpp`

#### Includes Faltantes

```
[pp_file_not_found] Line 11: 'Arduino.h' file not found
[pp_file_not_found] Line 8: in included file: 'cstdint' file not found
```

**Causa**: SDK Arduino/ESP-IDF no instalado en este entorno de análisis. Los includes se resolverían con PlatformIO (`pio run -t compiledb` genera un `compile_commands.json` con paths completos al SDK).

#### Tipos No Resueltos (derivados del include faltante)

```
[unknown_typename] Line 18: unknown type name 'TFT_eSPI'
[unknown_typename] Line 21: unknown type name 'int16_t'
[undeclared_var_use] Line 39: use of undeclared identifier 'Serial'
[undeclared_var_use] Line 41: use of undeclared identifier 'psramInit'
```

#### IncludeCleaner Issues

```
IncludeCleaner: Failed to get an entry for resolved path : No such file or directory
```

> Se reporta múltiples veces. Es consecuencia directa de los headers faltantes.

#### Configuración clangd

- ✅ `esp32/.clangd` carga correctamente
- ✅ `esp32/compile_commands.json` detectado y cargado
- ✅ Flags C++17 y Arduino defines aplicados
- ✅ **CORREGIDO**: Ya no hereda flags STM32 del `.clangd` raíz (solucionado con `If: PathMatch: Core/.*`)

---

## 6. Resumen de Configuración de Herramientas

### Versiones

| Herramienta | Versión |
|-------------|---------|
| Clang-Tidy | Ubuntu LLVM 18.1.3 |
| CppCheck | 2.13.0 |
| clangd | Ubuntu clangd 18.1.3 |
| ARM GCC | arm-none-eabi-gcc 13.2.1 |
| compiledb | 0.10.7 |

### Archivos de Configuración

| Archivo | Propósito | Estado |
|---------|-----------|--------|
| `.clangd` (raíz) | Config clangd para STM32 (solo `Core/.*`) | ✅ Corregido — `CompilationDatabase` movido dentro de `CompileFlags`, añadido `If: PathMatch` |
| `esp32/.clangd` | Config clangd para ESP32-S3 (C++17 Arduino) | ✅ Corregido — `CompilationDatabase` movido dentro de `CompileFlags` |
| `compile_commands.json` (raíz) | Base de datos de compilación STM32 | ✅ Generado (24 entradas, gitignored) |
| `esp32/compile_commands.json` | Base de datos de compilación ESP32 | ✅ Generado (37 entradas, gitignored) |

### Correcciones Realizadas en .clangd

1. **`CompilationDatabase`** estaba como key de nivel superior — clangd 18 lo requiere dentro de `CompileFlags`. Movido a `CompileFlags.CompilationDatabase`.
2. **Cross-contamination de toolchains**: `.clangd` raíz añadía `-DSTM32G474xx --target=arm-none-eabi -ICore/Inc` a TODOS los archivos, incluyendo los de ESP32. Corregido añadiendo `If: PathMatch: Core/.*` para que solo aplique a ficheros STM32.

---

## 7. Recomendaciones (basadas SOLO en resultados reales)

### Prioridad Alta

| # | Issue | Herramienta | Archivos | Acción |
|---|-------|-------------|----------|--------|
| 1 | `invalidPrintfArgType_uint` (5) | CppCheck | `engineering_screen.cpp:334,676` | Cambiar `%u` por `%d` o castear argumentos a `unsigned int` |
| 2 | `duplicateExpressionTernary` | CppCheck + Clang-Tidy | `engineering_screen.cpp:597` | Corregir ternario: `(i == 6) ? COL_DARK_GRAY : COL_DARK_GRAY` — una rama debería ser diferente |
| 3 | `bugprone-reserved-identifier` (13) | Clang-Tidy | Todos los headers en Core/Inc/ | Renombrar `__XXX_H` a `XXX_H` (doble guion bajo reservado por el estándar C) |

### Prioridad Media

| # | Issue | Herramienta | Archivos | Acción |
|---|-------|-------------|----------|--------|
| 4 | `bugprone-branch-clone` — 10 ramas idénticas | Clang-Tidy | `can_handler.c:106` | Revisar switch: ¿lógica por implementar o simplificar a default? |
| 5 | `bugprone-switch-missing-default-case` (3) | Clang-Tidy | `can_handler.c:900`, `boot_screen.cpp:229`, `engineering_screen.cpp:443` | Añadir case `default:` |
| 6 | `bugprone-too-small-loop-variable` (5) | Clang-Tidy | `can_handler.c:922`, `service_mode.c:82,189,201,217` | Cambiar `uint8_t` a `int` o `unsigned` para variable de loop |
| 7 | `redundantCondition` (3) | CppCheck | `service_mode.c:234,245,256` | Eliminar `i < 32` redundante (ya se chequea `i < 25`) |
| 8 | `duplicateConditionalAssign` | CppCheck | `main.cpp:550` | Simplificar `if (showOverlay) showOverlay = false` → `showOverlay = false` |

### Prioridad Baja (estilo)

| # | Issue | Count | Nota |
|---|-------|-------|------|
| 9 | `readability-uppercase-literal-suffix` | 395+22 | Usar `0xFFU` en vez de `0xff`, `1.0F` en vez de `1.0f` |
| 10 | `readability-braces-around-statements` | 211+163 | Añadir llaves a if/for/while de una línea |
| 11 | `unusedStructMember` Motor_t::power | 1 | Eliminar miembro no usado |
| 12 | `functionConst` / `functionStatic` | 7 | Marcar métodos como const/static donde aplique |

---

*Informe generado a partir de outputs reales de clang-tidy, cppcheck y clangd. No se ha modificado código fuente.*
