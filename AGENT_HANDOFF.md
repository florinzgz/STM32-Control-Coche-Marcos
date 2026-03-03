# PROMPT DE CONTINUACIÓN PARA EL SIGUIENTE AGENTE

> **Fecha:** 2026-02-22
> **Repositorio:** `florinzgz/STM32-Control-Coche-Marcos`
> **Branch activo:** `copilot/update-firmware-comparison`
> **Último commit conocido:** `c72c1cb` (verificar con `git log --oneline -1` al iniciar)
> **Documento maestro:** `docs/PROJECT_MASTER_STATUS.md` ← TODA PR debe actualizar este archivo

---

## 1. QUÉ ES ESTE PROYECTO

Firmware de control de un coche eléctrico infantil con arquitectura dual:
- **STM32G474RE** (170 MHz, Cortex-M4 FPU): Controlador de motores, autoridad de seguridad, sensores
- **ESP32-S3** (240 MHz, PSRAM 8MB): HMI (pantalla táctil 480×320 TFT ST7796), audio DFPlayer, LEDs WS2812B, lectura de periféricos de usuario (palanca marchas MCP23017, sensor obstáculo TOFSense-M LiDAR)

Comunicación exclusiva por **CAN bus a 500 kbps** (CAN 2.0A, 11-bit IDs, TJA1051 transceiver).

---

## 2. ARQUITECTURA DE DIRECTORIOS

```
/
├── Core/Inc/           ← Headers STM32 (16 archivos .h)
├── Core/Src/           ← Fuentes STM32 (18 archivos .c, incluye 2 tests)
├── Drivers/            ← HAL STM32G4xx + CMSIS (generado por CubeMX)
├── esp32/
│   ├── include/        ← can_ids.h (contrato CAN frozen rev 1.3)
│   └── src/
│       ├── screens/    ← 7 pantallas (boot, standby, drive, safe, error, engineering)
│       ├── ui/         ← 11 widgets UI (car, pedal, gear, battery, mode, obstacle, led, debug)
│       ├── sensors/    ← obstacle_sensor.cpp/h (TOFSense-M LiDAR UART driver)
│       ├── can/        ← can_obstacle.cpp/h (TX frame 0x208)
│       ├── hmi/        ← obstacle_indicator.cpp/h (indicador estado sensor en boot)
│       └── *.cpp/h     ← main, can_rx, vehicle_data, screen_manager, led_controller,
│                         power_manager, audio_manager, shifter_input, touch_handler, config_store
├── docs/               ← 57 archivos .md de documentación
├── Makefile            ← Build STM32 con arm-none-eabi-gcc
└── platformio.ini      ← Build ESP32 con PlatformIO (Arduino C++17)
```

---

## 3. ESTADO ACTUAL COMPLETO DEL CÓDIGO — QUÉ FUNCIONA

### STM32 (directorio `Core/`)

| Módulo | Archivo(s) | Estado |
|--------|-----------|--------|
| Control PWM 4 motores tracción (TIM1, 20 kHz) | `motor_control.c/h` | ✅ Completo |
| Control PWM motor dirección (TIM8, PID P-only) | `motor_control.c/h` | ✅ Completo |
| Ackermann geometría + corrección diferencial torque | `ackermann.c/h`, `motor_control.c` | ✅ Completo |
| EPS torque-assist dirección (smoothstep, speed-dep) | `motor_control.c`, `eps_params.c/h` | ✅ Completo |
| Calibración centro dirección (sensor inductivo PB5) | `steering_centering.c/h`, `steering_cal_store.c/h` | ✅ Completo |
| Encoder dirección (TIM2 quadratura, E6B2-CWZ6C 4800 CPR) | `encoder_reader.c/h` | ✅ Completo |
| Máquina de estados (7 estados: BOOT→STANDBY→ACTIVE⇄DEGRADED→SAFE→ERROR + LIMP_HOME) | `safety_system.c/h` | ✅ Completo |
| 3 niveles degradación (L1=70%, L2=50%, L3=40% potencia) | `safety_system.c/h` | ✅ Completo |
| ABS por rueda (modulación pulso 80ms) | `safety_system.c` | ✅ Completo |
| TCS por rueda (reducción progresiva + recuperación) | `safety_system.c` | ✅ Completo |
| Protección sobrecorriente (25A, escalación a SAFE) | `safety_system.c` | ✅ Completo |
| Protección sobretemperatura (80°C warn, 90°C crit, 130°C/motor) | `safety_system.c`, `motor_control.c` | ✅ Completo |
| Protección batería (20V warn, 18V crit) | `safety_system.c` | ✅ Completo |
| **Detección obstáculos (5 zonas)** | `safety_system.c/h` | ✅ Completo |
| **Detección reacción infantil (pedal drop → zonas más estrictas)** | `safety_system.c` | ✅ Completo |
| Plausibilidad sensor obstáculo (stuck, stale, implausible) | `safety_system.c` | ✅ Completo |
| Sensores (6×INA226, 5×DS18B20, pedal ADC, 4×velocidad rueda) | `sensor_manager.c/h` | ✅ Completo |
| Modo servicio (25 módulos, enable/disable, factory restore) | `service_mode.c/h` | ✅ Completo |
| Validación arranque (6 checks pre-ACTIVE) | `boot_validation.c/h` | ✅ Completo |
| Matemáticas seguras (NaN/Inf sanitization) | `math_safety.c/h` + test | ✅ Completo |
| CAN handler (RX decode, TX status, ACK, bus-off recovery) | `can_handler.c/h` | ✅ Completo |
| Sistema marchas (P/R/N/D1/D2, speed-gated) | `motor_control.c` | ✅ Completo |
| Frenado dinámico + Park hold + BTS7960 brake | `motor_control.c` | ✅ Completo |
| Relé LED en PB10 | `can_handler.c`, `main.c` | ✅ Completo |
| Secuencia relés (Main→Traction→Direction, non-blocking) | `safety_system.c` | ✅ Completo |
| Main loop (10/50/100/1000 ms tareas) + watchdog (500ms) | `main.c` | ✅ Completo |

### ESP32 (directorio `esp32/`)

| Módulo | Archivo(s) | Estado |
|--------|-----------|--------|
| CAN RX decode (todos los IDs STM32) | `can_rx.cpp/h` | ✅ Completo |
| CAN IDs contrato (frozen rev 1.3) | `include/can_ids.h` | ✅ Completo |
| VehicleData store (12 structs + service) | `vehicle_data.h/cpp` | ✅ Completo |
| Screen Manager (6 pantallas + frame limiter 20 FPS) | `screen_manager.cpp/h` | ✅ Completo |
| Boot Screen | `screens/boot_screen.cpp/h` | ✅ Completo |
| Standby Screen (temps + fault flags) | `screens/standby_screen.cpp/h` | ✅ Completo |
| Drive Screen (telemetría completa, partial redraw) | `screens/drive_screen.cpp/h` | ✅ Completo |
| Safe Screen | `screens/safe_screen.cpp/h` | ✅ Completo |
| Error Screen | `screens/error_screen.cpp/h` | ✅ Completo |
| **Engineering Screen (menú oculto código 8989)** | `screens/engineering_screen.cpp/h` | ✅ Completo |
| UI Components (car, pedal, gear, battery, steering, obstacle, mode, led, debug) | `ui/*.cpp/h` | ✅ Completo |
| WS2812B LED controller (FastLED, 28 front + 16 rear) | `led_controller.cpp/h` | ✅ Completo |
| Power Manager (llave contacto GPIO 40/41) | `power_manager.cpp/h` | ✅ Completo |
| Audio Manager (DFPlayer UART2 GPIO 43/44) | `audio_manager.cpp/h` | ✅ Completo |
| Sensor obstáculo TOFSense-M (GPIO 18 UART1 RX, 921600 bps, 5 zonas) | `sensors/obstacle_sensor.cpp/h` | ✅ Completo |
| CAN TX obstáculo (0x208, DLC 5, 66ms) | `can/can_obstacle.cpp/h` | ✅ Completo |
| Indicador obstáculo en boot | `hmi/obstacle_indicator.cpp/h` | ✅ Completo |
| **Palanca marchas MCP23017 I2C (GPIO 8/9)** | `shifter_input.cpp/h` | ✅ Completo |
| **Touch handler centralizado (debounce, tap/long-press)** | `touch_handler.cpp/h` | ✅ Completo |
| **Config store NVS (CRC32, mode/brightness/led/volume)** | `config_store.cpp/h` | ✅ Completo |
| Heartbeat ESP32 (0x011 cada 100ms) | `main.cpp` | ✅ Completo |
| ACK tracking (non-blocking, 200ms timeout) | `main.cpp` | ✅ Completo |
| Envío CAN 0x102 gear + mode flags | `main.cpp` | ✅ Completo |
| Runtime monitor / debug overlay | `ui/runtime_monitor.cpp/h`, `ui/debug_overlay.cpp/h` | ✅ Completo |

---

## 4. IMPLEMENTACIONES RECIENTES (COMMIT e570505 → c72c1cb)

Las siguientes funcionalidades fueron añadidas por el agente anterior y **todavía NO están reflejadas en `docs/PROJECT_MASTER_STATUS.md`**. El siguiente agente DEBE actualizar ese documento.

### 4.1 — Detección obstáculos ampliada a 5 zonas (STM32)
**Archivos modificados:** `Core/Src/safety_system.c`, `Core/Inc/safety_system.h`
**Qué cambió:**
- Se añadieron 2 zonas nuevas al sistema de 3 zonas original:
  - `OBSTACLE_CAUTION_MM = 1500` → scale = 0.85 (línea 142)
  - `OBSTACLE_ALERT_MM = 4000` → scale = 0.95 (línea 143)
- `Obstacle_Update()` ahora computa `dyn_caution` y `dyn_alert` con speed-dependent thresholds (líneas 1580-1587)
- La cadena if/else en target_scale tiene 5 niveles (líneas 1591-1604)
- Defines públicos en header: `OBSTACLE_CAUTION_MM_PUB`, `OBSTACLE_ALERT_MM_PUB`

### 4.2 — Detección reacción infantil (STM32)
**Archivos modificados:** `Core/Src/safety_system.c`, `Core/Inc/safety_system.h`
**Qué cambió:**
- Nuevo bloque de código en `Obstacle_Update()` (líneas 1611-1646)
- Monitorea si el niño suelta el pedal rápido (>10% drop en 500ms)
- Durante 2 segundos tras detección, tightens warning zone (0.7→0.5) y caution zone (0.85→0.7)
- 6 nuevos `#define`: `CHILD_REACTION_THRESHOLD`, `CHILD_REACTION_MIN_PEDAL`, `CHILD_REACTION_WINDOW_MS`, `CHILD_REACTION_BOOST_MS`, `CHILD_REACTION_WARNING_SCALE`, `CHILD_REACTION_CAUTION_SCALE`
- 4 nuevas variables static: `child_prev_pedal_pct`, `child_pedal_sample_tick`, `child_reaction_active`, `child_reaction_start`
- Inicializadas en `Safety_Init()` (líneas 613-616)

### 4.3 — Palanca marchas MCP23017 (ESP32)
**Archivos creados:** `esp32/src/shifter_input.cpp`, `esp32/src/shifter_input.h`
**Archivos modificados:** `esp32/src/main.cpp`, `esp32/src/screens/drive_screen.cpp`
**Qué hace:**
- Driver I2C para MCP23017 en GPIO 8 (SDA) / GPIO 9 (SCL), dirección 0x20, 400 kHz
- Lee Puerto A pins GPA0-GPA4 (active-low, pull-ups internos, one-hot encoding)
- GPA0=Park, GPA1=Reverse, GPA2=Neutral, GPA3=Forward, GPA4=Forward_D2
- Validación: `__builtin_popcount(active) != 1` → default Neutral
- Poll cada 50ms, API: `shifter::init()`, `shifter::update()`, `shifter::getGear()`, `shifter::getGearRaw()`
- `main.cpp`: envía CAN 0x102 con gear actual cuando cambia (100ms debounce)
- `drive_screen.cpp`: lee `shifter::getGearRaw()` en vez de hardcode `Gear::N` (líneas 101-113)

### 4.4 — Touch handler centralizado (ESP32)
**Archivos creados:** `esp32/src/touch_handler.cpp`, `esp32/src/touch_handler.h`
**Archivos modificados:** `esp32/src/main.cpp`
**Qué hace:**
- Reemplaza el touch directo de `tft.getTouch()` por sistema centralizado
- Detecta TAP (200ms debounce), LONG_PRESS (3s), RELEASE
- `main.cpp` loop: llama `touch::update()`, consume `touch::getEvent()`, despacha a:
  - `screenManager.onTouch(x,y)` para código secreto / engineering screen
  - `ui::LedToggle::hitTest()` para toggle LED relay
  - `ui::ModeIcons::hitTest()` para cambiar modo 4x4/4x2/360° → envía CAN 0x102
- Persiste cambios de LED y modo en NVS vía `config_store`

### 4.5 — Engineering Screen (ESP32)
**Archivos creados:** `esp32/src/screens/engineering_screen.cpp`, `esp32/src/screens/engineering_screen.h`
**Archivos modificados:** `esp32/src/screen_manager.cpp`, `esp32/src/screen_manager.h`
**Qué hace:**
- Menú oculto activado por código secreto "8989" (4 taps alternando izquierda-derecha en <2s)
- `screen_manager.cpp` detecta el patrón en `checkSecretCode()` y transiciona a engineering screen
- 5 submenús: Fault Viewer, Module Enable/Disable, Pedal Calibration, Encoder Calibration, Factory Restore
- Fault Viewer: muestra `ServiceData.faultMask/enabledMask/disabledMask` en hex
- Factory Restore: envía `SERVICE_CMD (0x110)` con action `0xFF` al STM32
- Botón BACK en cada submenú, touch dispatch via `handleTouch(x,y)`

### 4.6 — Config Store NVS (ESP32)
**Archivos creados:** `esp32/src/config_store.cpp`, `esp32/src/config_store.h`
**Archivos modificados:** `esp32/src/main.cpp`
**Qué hace:**
- Persistencia en NVS ESP32 usando `Preferences` (nativo, sin librería externa)
- Struct Config: driveMode (uint8, masked 0x03), brightness (uint8), ledEnabled (bool), audioVolume (uint8, max 30)
- CRC32 (ISO-HDLC) sobre campos de datos (excluyendo crc32 field)
- `init()` → `load()` → valida CRC → usa defaults si falla
- Setters individuales con bounds checking: `setDriveMode()` (& 0x03), `setAudioVolume()` (clamp ≤30)
- `factoryReset()` → clear NVS → reinitialize defaults
- `main.cpp` `setup()`: carga config, aplica `currentModeFlags` y `ledLocalState`
- `main.cpp` `loop()`: persiste cambios de LED y modo al tocar iconos

### 4.7 — Zona obstáculo actualizada (ESP32 UI)
**Archivos modificados:** `esp32/src/sensors/obstacle_sensor.cpp`, `esp32/src/sensors/obstacle_sensor.h`, `esp32/src/ui/ui_common.h`
**Qué cambió:**
- `distanceToZone()` ahora retorna 0-4 (era 0-3): zone 1 = caution (1000-1500mm)
- `Reading.zone` comment actualizado a "0-4"
- `proximityColor()` en `ui_common.h` tiene 5 bandas de color:
  - 0cm → GRAY, >150cm → GREEN, >100cm → CYAN, >50cm → YELLOW, >20cm → ORANGE, <20cm → RED

---

## 5. QUÉ FALTA POR HACER (PENDIENTE)

### 5.1 — Actualizar `docs/PROJECT_MASTER_STATUS.md` — PRIORIDAD MÁXIMA
El documento maestro NO refleja las implementaciones 4.1-4.7. Según las reglas del proyecto (§7), toda PR es inválida si no actualiza este documento. Hay que:
- **Sección 1**: Añadir shifter_input, touch_handler, config_store, engineering_screen a la tabla de responsabilidades ESP32
- **Sección 2**: Añadir 5-zone obstacle, child reaction, shifter, touch, engineering, NVS como features completadas
- **Sección 3**: Actualizar la limitación "Detección obstáculos (3 zonas)" → ya son 5 zonas
- **Sección 3**: Actualizar "Drive screen gear display is not CAN-driven" → ahora lee de shifter_input
- **Sección 3**: Eliminar "ESP32 obstacle source not yet implemented" → ya existe obstacle_sensor + can_obstacle
- **Sección 4**: Marcar como resueltos: #1 (ESP32 obstacle sensor driver), #7 (ESP32 mode/gear feedback)

### 5.2 — Funcionalidades pendientes del código (ordenadas por fase del PROJECT_MASTER_STATUS)

| # | Feature | Fase | Estado | Notas |
|---|---------|------|--------|-------|
| 1 | CAN ID 0x209 parsing en STM32 | Phase 3 | ❌ Pendiente | `case CAN_ID_OBSTACLE_SAFETY: break;` — solo tiene break vacío |
| 2 | PID steering tuning (I/D terms) | Phase 3 | ❌ Pendiente | ki=0.0, kd=0.0 hardcoded |
| 3 | Calibración persistente STM32 (encoder zero, sensor offsets) | Phase 4 | ❌ Pendiente | No hay flash/EEPROM write en STM32 |
| 4 | Service mode persistente STM32 | Phase 4 | ❌ Pendiente | RAM-only, se pierde al reiniciar |
| 5 | Sensor pedal redundante | Phase 3 | ❌ Pendiente | Single ADC channel PA3 |
| 6 | Hot-plug DS18B20 | Phase 3 | ❌ Pendiente | ROM search solo en init |
| 7 | DriveScreen mode flags from CAN | Phase 4 | ⚠️ Parcial | Gear ahora viene de shifter, pero mode flags (4x4/360°) siguen locales |
| 8 | Audio feedback integrado | Phase 5 | ⚠️ Parcial | AudioManager existe pero solo welcome/farewell implementados |
| 9 | Lighting control | Phase 5 | ⚠️ Parcial | LED strip funciona, pero no hay lógica de luces de freno/reversa real |

### 5.3 — Problemas detectados no resueltos
1. **Engineering screen no tiene salida al modo normal** — una vez activada, `engineeringActive_ = true` no tiene mecanismo para volver a `false`. Los botones BACK dentro del engineering screen solo navegan entre submenús, pero no hay forma de volver a las pantallas normales (drive/standby/etc). Falta un "exit engineering mode" trigger (ej: otro código secreto, o timeout, o botón EXIT en el menú principal).
2. **sendGearCommand() y sendModeCommand() pueden conflictar** — ambos envían CAN 0x102 pero con diferente contenido de byte 0. Si se llaman en el mismo frame podría haber colisión.
3. **config_store::save() en cada touch** — cada cambio de LED o modo escribe NVS, lo cual tiene wear limitado (~100K cycles). Considerar dirty flag con save periódico.
4. **Steering PID es P-only** — kp=0.09, ki=0.0, kd=0.0. Funcional pero sin integral/derivativo.

---

## 6. PROTOCOLO CAN COMPLETO (500 kbps, 11-bit IDs)

### ESP32 → STM32
| CAN ID | DLC | Rate | Contenido |
|--------|-----|------|-----------|
| 0x011 | 1 | 100 ms | Heartbeat: alive counter |
| 0x100 | 1 | 50 ms | Throttle: demand 0-100% |
| 0x101 | 2 | 50 ms | Steering: angle (0.1° units) |
| 0x102 | 2 | On-demand | Mode/Gear: byte0=mode_flags (bit0=4x4, bit1=tank), byte1=gear (0-4) |
| 0x110 | 2 | On-demand | Service CMD: byte0=action (0=disable, 1=enable, 0xFF=factory), byte1=module_id |
| 0x120 | 1 | On-demand | LED relay: byte0 (0=OFF, 1=ON) |
| 0x208 | 5 | ~66 ms | Obstacle: distance_mm (LE u16), zone (u8), health (u8), rolling_counter (u8) |
| 0x209 | — | Reserved | Obstacle safety state (accepted but not parsed by STM32) |

### STM32 → ESP32
| CAN ID | DLC | Rate | Contenido |
|--------|-----|------|-----------|
| 0x001 | 4 | 100 ms | Heartbeat: counter, system_state, fault_flags, error_code |
| 0x103 | 3 | On-demand | CMD ACK: cmd_id_low, result (0=OK/1=REJECTED/2=INVALID/3=BLOCKED), system_state |
| 0x200 | 8 | 100 ms | Speed: 4× wheel speed (u16 LE, 0.1 km/h units) |
| 0x201 | 8 | 100 ms | Current: 4× motor current (u16 LE, 0.01 A units) |
| 0x202 | 5 | 1000 ms | Temp: 5× temperature (s8 °C) |
| 0x203 | 3 | 100 ms | Safety: ABS active, TCS active, error code |
| 0x204 | 3 | 100 ms | Steering: angle (s16 LE, 0.1° units), calibrated flag |
| 0x205 | 4 | 100 ms | Traction: 4× per-wheel scale (u8, 0-100%) |
| 0x206 | 5 | 1000 ms | Temp Map: 5× mapped temperatures (FL/FR/RL/RR/AMB) |
| 0x207 | 4 | 100 ms | Battery: current (u16 LE, 0.01 A), voltage (u16 LE, 0.01 V) |
| 0x20A | 2 | 1000 ms | Lights: relay state + reserved |
| 0x300 | 2 | On-demand | Diagnostic: error code + subsystem |
| 0x301 | 4 | 1000 ms | Service Faults: 32-bit bitmask |
| 0x302 | 4 | 1000 ms | Service Enabled: 32-bit bitmask |
| 0x303 | 4 | 1000 ms | Service Disabled: 32-bit bitmask |

---

## 7. PRINCIPALES INVARIANTES DE SEGURIDAD (NO ROMPER)

1. **STM32 es la autoridad de seguridad** — todos los comandos ESP32 pasan por `Safety_Validate*()` gates
2. **CAN loss → LIMP_HOME** (20% torque, 5 km/h cap) — el vehículo permanece conducible
3. **Obstacle scale se computa localmente en STM32** — datos CAN son solo advisory
4. **No immobilization** — solo slowdown controlado, nunca parada total inmediata
5. **Reverse escape preserved** — cuando forward está bloqueado por obstáculo, reverse sigue permitido
6. **Critical modules (CAN timeout, emergency stop, watchdog, relay main) nunca deshabilitables** vía service mode
7. **Factory restore NO elimina faults reales** — solo re-habilita módulos, faults reaparecen en siguiente check
8. **Emergency stop es irrecuperable** — requiere reset del MCU
9. **Watchdog (IWDG ~500ms)** — resetea MCU si main loop se bloquea
10. **Gear changes speed-gated** — cambio solo permitido a ≤1 km/h

---

## 8. CÓMO COMPILAR

### STM32
```bash
# Requiere arm-none-eabi-gcc en PATH
cd /repo
make
# Output: STM32G474RE.elf, STM32G474RE.bin
```

### ESP32
```bash
# Requiere PlatformIO
cd /repo/esp32
pio run -e esp32s3
# Output: .pio/build/esp32s3/firmware.bin
```

**Nota:** Las herramientas de cross-compile NO están instaladas en el sandbox. Solo se puede verificar sintaxis con gcc/g++ del host y stubs.

---

## 9. TESTS EXISTENTES

Solo 2 test files en C (no hay framework formal):
- `Core/Src/test_math_safety.c` — Tests para `math_safety.c` (NaN/Inf handling)
- `Core/Src/test_steering_cal_store.c` — Tests para `steering_cal_store.c`

No hay tests para ESP32. No hay CI/CD configurado (no `.github/workflows/`).

---

## 10. GPIO MAP RESUMIDO

### STM32G474RE
| Función | Pin(es) | Periférico |
|---------|---------|-----------|
| PWM Tracción FL/FR/RL/RR | PA8/PA9/PA10/PA11 | TIM1 CH1-4 |
| PWM Dirección | PC8 | TIM8 CH3 |
| Dirección motores | PC0-PC4 | GPIO output |
| Enable motores | PC5-PC7, PC13, PC9 | GPIO output |
| Relé principal | PC10 | GPIO output |
| Relé tracción | PC11 | GPIO output |
| Relé dirección | PC12 | GPIO output |
| Relé LED | PB10 | GPIO output |
| Encoder A/B | PA15/PB3 | TIM2 encoder mode |
| Sensor centro | PB5 | GPIO input (EXTI) |
| Velocidad ruedas | PA0-PA2, PB15 | GPIO input (EXTI) |
| OneWire DS18B20 | PB0 | GPIO bitbang |
| Pedal ADC | PA3 | ADC1 CH4 |
| CAN TX/RX | PB8/PB9 | FDCAN1 |
| I2C SDA/SCL | PB6/PB7 | I2C1 (INA226, TCA9548A) |

### ESP32-S3
| Función | Pin(es) |
|---------|---------|
| CAN TX/RX | GPIO 4/5 |
| Obstacle UART1 RX | GPIO 18 |
| Shifter I2C SDA/SCL | GPIO 8/9 |
| TFT MOSI/SCLK/CS/DC/RST | GPIO 13/14/10/39/38 |
| Touch CS | GPIO 21 |
| WS2812B data | GPIO 47/48 |
| Power sense/hold | GPIO 40/41 |
| TFT backlight | GPIO 42 |
| Audio TX/RX | GPIO 19/20 |

---

## 11. REGLAS OBLIGATORIAS PARA CONTRIBUIR

1. **Toda PR debe actualizar `docs/PROJECT_MASTER_STATUS.md`** si cambia arquitectura, features, limitaciones, backlog o estado de fases
2. **No implementar features de fases futuras** sin que la fase actual esté completada (ver §5 del PROJECT_MASTER_STATUS)
3. **No eliminar safety checks** sin justificación técnica
4. **No modificar CAN IDs existentes** — solo añadir nuevos
5. **Zero heap allocation en ESP32 UI** — solo buffers stack fijos con snprintf
6. **No usar `String` class** en ESP32
7. **Float sanitization** — toda operación PWM-affecting debe pasar por `sanitize_float()`
8. **Parcial redraw** — UI solo redibuja elementos que cambiaron

---

## 12. ARCHIVOS CLAVE PARA ENTENDER EL SISTEMA RÁPIDO

Si tienes poco tiempo, lee estos archivos en este orden:
1. `docs/PROJECT_MASTER_STATUS.md` — Estado maestro obligatorio
2. `esp32/include/can_ids.h` — Contrato CAN completo (IDs, enums, timing)
3. `Core/Inc/safety_system.h` — State machine, error types, obstacle states
4. `esp32/src/main.cpp` — Loop principal ESP32, integración de todos los módulos
5. `Core/Src/main.c` — Loop principal STM32, scheduling de tareas
6. `esp32/src/screen_manager.cpp` — Routing de pantallas + código secreto engineering
7. `Core/Src/safety_system.c` — 5-zone obstacle + child reaction (líneas 1550-1650)

---

## 13. SIGUIENTE PASO RECOMENDADO

**Actualizar `docs/PROJECT_MASTER_STATUS.md`** para reflejar todas las implementaciones de la sección 4 de este documento. Esto es OBLIGATORIO antes de cualquier otra implementación nueva. Luego, continuar con las features pendientes de la sección 5.2 siguiendo el orden de fases del proyecto.
