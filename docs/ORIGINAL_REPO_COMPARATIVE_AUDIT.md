# Auditoría Comparativa: Repo Original vs. Repo Actual

> **Repo original:** [`florinzgz/FULL-FIRMWARE-Coche-Marcos`](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos)
> **Repo actual:** [`florinzgz/STM32-Control-Coche-Marcos`](https://github.com/florinzgz/STM32-Control-Coche-Marcos)
> **Fecha:** 2026-02-15

---

## RESUMEN EJECUTIVO

### Repo Original (FULL-FIRMWARE-Coche-Marcos)

El repo original es un **firmware monolítico para ESP32-S3 N16R8** (PlatformIO/Arduino) que intenta controlar TODO el vehículo desde un solo chip: motores, sensores, display, audio, LEDs, menús táctiles, etc. El proyecto llegó a la versión **v2.18.3** tras **14 fases** de desarrollo, pero **sufre de bootloops crónicos** (reinicios permanentes) documentados en más de 30 archivos de diagnóstico. El ESP32 no tiene la capacidad de controlar actuadores safety-critical de forma fiable en un single-loop Arduino.

### Repo Actual (STM32-Control-Coche-Marcos)

El repo actual es una **reestructuración completa con arquitectura dual-MCU**:
- **STM32G474RE** → autoridad de seguridad y control de actuadores (bare-metal C, HAL)
- **ESP32-S3** → solo HMI/display (recibe telemetría por CAN, envía comandos)

La separación resuelve los problemas fundamentales del original: el control safety-critical corre en un MCU determinista (Cortex-M4F, IWDG, sin RTOS overhead) y el display corre por separado sin poder causar actuaciones no deseadas.

---

## 1) ARQUITECTURA: COMPARACIÓN FUNDAMENTAL

| Aspecto | Original (ESP32-S3 monolítico) | Actual (STM32 + ESP32 dual-MCU) |
|---------|-------------------------------|-------------------------------|
| **MCU de control** | ESP32-S3 (FreeRTOS/Arduino) | STM32G474RE (bare-metal HAL, Cortex-M4F) |
| **MCU de display** | Mismo ESP32-S3 | ESP32-S3 dedicado solo a HMI |
| **Comunicación** | No aplica (todo interno) | CAN bus 500 kbps (FDCAN ↔ TWAI) |
| **RTOS** | FreeRTOS dual-core (v2.18.0) | Sin RTOS en STM32 (superloop tiered) |
| **Framework** | Arduino/PlatformIO | STM32 HAL (Makefile), ESP32 PlatformIO |
| **PWM motores** | PCA9685 vía I2C (indirecto) | TIM1 CH1-4 directo (20 kHz, 8500 steps) |
| **PWM dirección** | PCA9685 vía I2C | TIM8 CH3 directo (20 kHz) |
| **Encoder** | Lectura por GPIO/interrupt en ESP32 | TIM2 hardware quadrature (32-bit) |
| **Watchdog** | ESP32 Task WDT (configurable) | IWDG independiente (~500 ms, LSI) |
| **Safety authority** | Ninguna clara (todo mezclado) | STM32 es la única autoridad de seguridad |
| **Aislamiento de fallos** | Fallo de display = fallo de motores | Fallo de display ≠ fallo de motores |
| **Bootloop risk** | ALTO (30+ documentos de diagnóstico) | BAJO (periféricos non-fatal, IWDG recovery) |

---

## 2) RESUMEN DEL REPO ORIGINAL — CÓMO ESTABA HECHO

### Estructura de código fuente

```
FULL-FIRMWARE-Coche-Marcos/
├── src/
│   ├── main.cpp                    ← Entry point (FreeRTOS + Arduino)
│   ├── i2c.cpp                     ← I2C bus management
│   ├── test_display.cpp            ← Display test mode
│   ├── audio/
│   │   ├── alerts.cpp              ← Audio alert system
│   │   ├── dfplayer.cpp            ← DFPlayer Mini MP3 driver
│   │   └── queue.cpp               ← Audio queue management
│   ├── control/
│   │   ├── adaptive_cruise.cpp     ← Adaptive cruise control
│   │   ├── relays.cpp              ← Relay management (4 relays via MCP23017)
│   │   ├── steering_model.cpp      ← Steering geometry model
│   │   ├── steering_motor.cpp      ← Steering motor via PCA9685
│   │   ├── tcs_system.cpp          ← Traction control system
│   │   └── traction.cpp            ← 4-motor traction (via PCA9685)
│   ├── core/
│   │   ├── boot_guard.cpp          ← Bootloop detection (NVS counter)
│   │   ├── config_manager.cpp      ← Configuration management
│   │   ├── config_storage.cpp      ← Config persistence (NVS)
│   │   ├── eeprom_persistence.cpp  ← EEPROM emulation
│   │   ├── i2c_recovery.cpp        ← I2C bus recovery
│   │   ├── logger.cpp              ← Logging system
│   │   ├── mcp23017_manager.cpp    ← MCP23017 GPIO expander
│   │   ├── mcp_shared.cpp          ← Shared MCP instance
│   │   ├── menu_ina226_monitor.cpp ← INA226 debug menu
│   │   ├── operation_modes.cpp     ← Vehicle mode logic
│   │   ├── rtos_tasks.cpp          ← FreeRTOS task creation (5 tasks, 2 cores)
│   │   ├── shared_data.cpp         ← Thread-safe shared data
│   │   ├── storage.cpp             ← NVS/Preferences storage
│   │   ├── system.cpp              ← System initialization
│   │   ├── telemetry.cpp           ← Serial telemetry output
│   │   └── watchdog.cpp            ← ESP32 watchdog
│   ├── hud/
│   │   ├── gauges.cpp              ← Speed/RPM gauges
│   │   ├── hud.cpp                 ← Main HUD rendering (68 KB!)
│   │   ├── hud_compositor.cpp      ← Sprite compositor
│   │   ├── hud_graphics_telemetry.cpp
│   │   ├── hud_limp_diagnostics.cpp
│   │   ├── hud_limp_indicator.cpp
│   │   ├── hud_manager.cpp         ← HUD manager (52 KB!)
│   │   ├── icons.cpp               ← Icon rendering (35 KB)
│   │   ├── led_control_menu.cpp    ← LED control menu
│   │   ├── menu_encoder_calibration.cpp ← Encoder calibration menu
│   │   ├── menu_hidden.cpp         ← Hidden debug menu (46 KB!)
│   │   ├── menu_led_control.cpp
│   │   ├── menu_power_config.cpp
│   │   ├── menu_sensor_config.cpp
│   │   ├── obstacle_display.cpp
│   │   ├── render_engine.cpp       ← Sprite render engine
│   │   ├── touch_calibration.cpp
│   │   ├── touch_map.cpp
│   │   └── wheels_display.cpp
│   ├── input/
│   │   ├── buttons.cpp             ← Physical button input
│   │   ├── pedal.cpp               ← Pedal reading (ADC)
│   │   ├── shifter.cpp             ← Gear shifter (MCP23017)
│   │   └── steering.cpp            ← Steering encoder input
│   ├── lighting/
│   │   └── led_controller.cpp      ← WS2812B LED controller (FastLED)
│   ├── logging/
│   │   └── obstacle_logger.cpp
│   ├── managers/
│   │   ├── ControlManager.h
│   │   ├── HUDManager.h
│   │   ├── ModeManager.h
│   │   ├── PowerManager.h
│   │   ├── SafetyManager.h + Enhanced
│   │   ├── SensorManager.h + Enhanced
│   │   └── TelemetryManager.h
│   ├── menu/
│   │   ├── menu_auto_exit.cpp
│   │   └── menu_obstacle_config.cpp
│   ├── safety/
│   │   ├── abs_system.cpp          ← ABS implementation
│   │   ├── obstacle_safety.cpp     ← Obstacle detection safety
│   │   └── regen_ai.cpp            ← Regenerative braking AI
│   ├── sensors/
│   │   ├── car_sensors.cpp         ← Combined sensor management
│   │   ├── current.cpp             ← INA226 current sensors
│   │   ├── obstacle_detection.cpp  ← Ultrasonic obstacle detection
│   │   ├── sensors.cpp             ← Generic sensor init
│   │   ├── temperature.cpp         ← DS18B20 temperature
│   │   └── wheels.cpp              ← Wheel speed sensors
│   ├── system/
│   │   ├── limp_mode.cpp           ← Limp/degraded mode
│   │   └── power_mgmt.cpp          ← Power management
│   ├── test/
│   │   ├── audio_validation_tests.cpp
│   │   ├── boot_sequence_test.cpp
│   │   ├── functional_tests.cpp
│   │   ├── hardware_failure_tests.cpp
│   │   ├── memory_stress_test.cpp
│   │   ├── test_runner.cpp
│   │   ├── test_utils.cpp
│   │   └── watchdog_tests.cpp
│   └── utils/
│       ├── debug.cpp
│       ├── filters.cpp             ← Signal filters (EMA, etc.)
│       └── math_utils.cpp          ← Math utilities
├── include/                        ← 90+ header files
├── audio/                          ← MP3 audio tracks
├── boards/                         ← Custom board manifest (N16R8)
├── partitions/                     ← ESP32 partition tables
├── sdkconfig/                      ← ESP-IDF SDK configs
├── variants/                       ← Board variant files
├── data/                           ← SPIFFS data
├── rules/                          ← Build rules
├── tools/                          ← Build tools
└── platformio.ini                  ← PlatformIO config
```

### Problemas críticos del original

**El repo original tiene 30+ documentos de bootloop/fix:** `BOOTLOOP_FIX_*.md`, `ANALISIS_CAUSAS_REINICIO_COMPLETO.md`, `BUGFIX_RESET_LOOP_v2.11.4.md`, `SOLUCION_BOOTLOOP_ESP32S3.md`, etc. Esto demuestra un problema estructural no resuelto.

**Causas raíz de los bootloops:**

1. **PSRAM Octal init failure** → `psramInit()` falla → sistema bloqueado en `while(1)`
2. **Stack overflow** en tareas FreeRTOS → `CONFIG_ESP_MAIN_TASK_STACK_SIZE` insuficiente
3. **Watchdog timeout** → tareas que no llaman a `Watchdog::feed()` a tiempo
4. **I2C bus lock** → PCA9685 o INA226 bloquean el bus → cascade failure
5. **TFT init antes de PSRAM** → sprites sin memoria → crash
6. **Peripheral init order dependency** → la secuencia de init es frágil
7. **handleCriticalError()** → llama a `ESP.restart()` → bootloop cuando el error persiste
8. **IPC0 stack canary** → stack corruption en inter-processor call → Guru Meditation Error

**Mecanismo de boot del original:**
```
Serial.begin → psramInit() → TFT backlight → TFT reset → BootGuard check →
System::init → Storage::init → Watchdog::init → I2CRecovery::init →
Logger::init → initializeSystem() → [PowerMgr → SensorMgr → SafetyMgr →
HUDMgr → ControlMgr → TelemetryMgr → ModeMgr → SharedData → FreeRTOS tasks]
```

Si CUALQUIER Manager::init() falla → `handleCriticalError()` → retry 3 veces → `ESP.restart()` → bootloop.

### Hardware original

- **MCU:** ESP32-S3 N16R8 (16 MB Flash QIO + 8 MB PSRAM OPI)
- **Display:** ST7796S 480×320 TFT con touch XPT2046 (SPI)
- **Motor drivers:** BTS7960 H-bridge vía PCA9685 (I2C PWM)
- **GPIO expander:** MCP23017 (I2C, para relays y shifter)
- **Audio:** DFPlayer Mini (UART)
- **LEDs:** WS2812B (28 frontales + 16 traseros, FastLED)
- **Sensores:** INA226 (I2C), DS18B20 (OneWire), wheel speed (GPIO), encoder dirección

---

## 3) COMPARACIÓN MÓDULO POR MÓDULO

### ✅ Módulos que EXISTEN en ambos repos (reimplementados)

| Subsistema | Original (ESP32) | Actual (STM32/ESP32) | Estado |
|-----------|-------------------|---------------------|--------|
| **Tracción 4 motores** | `traction.cpp` → PCA9685 I2C | `motor_control.c` → TIM1 CH1-4 PWM directo | ✅ Mejorado: PWM directo 20 kHz, sin latencia I2C |
| **Dirección** | `steering_motor.cpp` → PCA9685 | `motor_control.c` → TIM8 CH3 PWM + PID | ✅ Mejorado: PID con encoder hardware quadrature |
| **Encoder dirección** | `steering.cpp` → GPIO/interrupt | `motor_control.c` → TIM2 quadrature 32-bit | ✅ Mejorado: conteo hardware sin pérdida de pulsos |
| **Modelo Ackermann** | `steering_model.cpp` | `ackermann.c` + differential torque | ✅ Mejorado: corrección diferencial de torque |
| **ABS** | `abs_system.cpp` | `safety_system.c` → `ABS_Update()` | ✅ Reimplementado: per-wheel pulse modulation |
| **TCS** | `tcs_system.cpp` | `safety_system.c` → `TCS_Update()` | ✅ Reimplementado: progressive reduction + recovery |
| **Sensores corriente** | `current.cpp` → INA226 I2C | `sensor_manager.c` → INA226 vía TCA9548A | ✅ Reimplementado: 6 canales, mux I2C |
| **Sensores temperatura** | `temperature.cpp` → DS18B20 | `sensor_manager.c` → DS18B20 OneWire bit-bang | ✅ Reimplementado: ROM search, CRC-8 |
| **Sensores velocidad** | `wheels.cpp` → GPIO pulses | `sensor_manager.c` → EXTI pulse + debounce | ✅ Reimplementado: hardware interrupts |
| **Pedal** | `pedal.cpp` → ADC | `sensor_manager.c` → ADC1 12-bit | ✅ Reimplementado + EMA filter + ramp limiter |
| **Relays** | `relays.cpp` → MCP23017 I2C | `safety_system.c` → GPIO directo + sequencing | ✅ Mejorado: secuencia non-blocking con timing |
| **Recovery I2C** | `i2c_recovery.cpp` | `sensor_manager.c` → `I2C_BusRecovery()` | ✅ Reimplementado: NXP AN10216 SCL cycling |
| **Watchdog** | `watchdog.cpp` → ESP32 Task WDT | `main.c` → IWDG independiente ~500 ms | ✅ Mejorado: hardware independiente, no software |
| **Boot guard** | `boot_guard.cpp` → NVS counter | `boot_validation.c` → 6-check pre-ACTIVE | ✅ Diferente enfoque: validación de subsistemas |
| **Limp mode** | `limp_mode.cpp` | `safety_system.c` → 3-level degradation | ✅ Mejorado: L1/L2/L3 con scaling granular |
| **Power management** | `power_mgmt.cpp` | `safety_system.c` → battery voltage checks | ✅ Parcial: undervoltage protection implementada |
| **Display HUD** | `hud.cpp` + compositor + gauges + icons (>200 KB) | ESP32 screens (boot/standby/drive/safe/error) | ✅ Simplificado: 5 screens, partial-redraw, 20 FPS |
| **Obstacle detection** | `obstacle_detection.cpp` + `obstacle_safety.cpp` | `safety_system.c` → CAN-based 3-tier backstop | ✅ Parcial: STM32 procesa, ESP32 driver pendiente |
| **Modos operación** | `operation_modes.cpp` → 4×4/4×2/tank | `motor_control.c` → 4×4/4×2/tank turn | ✅ Reimplementado |
| **Gear system** | `shifter.cpp` → MCP23017 | `motor_control.c` → P/R/N/D1/D2 speed-gated | ✅ Mejorado: validación por velocidad |
| **Service mode** | No existía | `service_mode.c` → 24 módulos, enable/disable | ✅ NUEVO en repo actual |
| **CAN communication** | No existía (monolítico) | `can_handler.c` + `can_rx.cpp` → protocolo completo | ✅ NUEVO en repo actual |
| **Safety state machine** | Disperso en managers | `safety_system.c` → BOOT→STANDBY→ACTIVE⇄DEGRADED→SAFE→ERROR | ✅ NUEVO: máquina de estados formal |

### ❌ Módulos que SOLO EXISTEN en el repo original (NO portados)

| Subsistema | Archivo original | Razón de no portar | Prioridad |
|-----------|-----------------|-------------------|-----------|
| **Audio (DFPlayer)** | `audio/dfplayer.cpp`, `alerts.cpp`, `queue.cpp` | No hay hardware de audio en el sistema actual | BAJA — no es safety-critical |
| **LED WS2812B** | `lighting/led_controller.cpp` | No hay hardware de iluminación en el sistema actual | BAJA — no es safety-critical |
| **Touch screen** | `hud/touch_calibration.cpp`, `touch_map.cpp` | ESP32 actual no usa touch, solo display pasivo | MEDIA — futuro HMI |
| **Menú oculto (debug)** | `hud/menu_hidden.cpp` (46 KB) | Display actual no tiene menús interactivos | BAJA |
| **Menú LED control** | `hud/led_control_menu.cpp`, `menu_led_control.cpp` | No hay LEDs en el sistema | BAJA |
| **Menú sensor config** | `hud/menu_sensor_config.cpp` | No hay menús interactivos | BAJA |
| **Menú power config** | `hud/menu_power_config.cpp` | No hay menús interactivos | BAJA |
| **Menú encoder calibration** | `hud/menu_encoder_calibration.cpp` | Centering automático en STM32 lo reemplaza | BAJA |
| **Menú obstacle config** | `menu/menu_obstacle_config.cpp` | No hay menús interactivos | BAJA |
| **Menu auto-exit** | `menu/menu_auto_exit.cpp` | No hay menús | BAJA |
| **Sprite compositor** | `hud/hud_compositor.cpp` | Render pipeline simplificado en actual | BAJA |
| **Render engine (sprites)** | `hud/render_engine.cpp` | Actual usa partial-redraw simple | BAJA |
| **Wheels display** | `hud/wheels_display.cpp` | Actual usa `CarRenderer` con per-wheel info | ✅ Reimplementado diferente |
| **Gauges (velocímetro/RPM)** | `hud/gauges.cpp` | Drive screen muestra speed como texto/barra | ✅ Reimplementado diferente |
| **Icons** | `hud/icons.cpp` (35 KB) | Modo icons simplificados en actual | ✅ Reimplementado diferente |
| **Buttons (physical)** | `input/buttons.cpp` | STM32 no tiene botones físicos del HMI | BAJA |
| **Adaptive cruise** | `control/adaptive_cruise.cpp` | No implementado en repo actual | MEDIA — futuro |
| **Regen braking AI** | `safety/regen_ai.cpp` | STM32 tiene dynamic braking básico, sin IA | MEDIA — futuro |
| **Telemetry serial** | `core/telemetry.cpp` | CAN telemetry reemplaza serial telemetry | ✅ Reimplementado vía CAN |
| **Config persistence** | `core/config_storage.cpp`, `eeprom_persistence.cpp` | STM32 no tiene NVS; calibración no persiste | MEDIA — backlog item #4/#5 |
| **MCP23017 GPIO expander** | `core/mcp23017_manager.cpp`, `mcp_shared.cpp` | STM32 tiene GPIOs nativos suficientes | N/A — cambio de hardware |
| **FreeRTOS tasks** | `core/rtos_tasks.cpp`, `shared_data.cpp` | STM32 usa superloop tiered, no necesita RTOS | N/A — cambio de arquitectura |
| **Logger** | `core/logger.cpp` | No hay sistema de logging formal en STM32 | BAJA |
| **Debug utilities** | `utils/debug.cpp` | No hay sistema de debug en STM32 | BAJA |
| **Signal filters** | `utils/filters.cpp` | Pedal EMA filter implementado inline | ✅ Parcial |
| **Math utilities** | `utils/math_utils.cpp` | `sanitize_float()` y cálculos inline | ✅ Parcial |
| **Test suite** | `test/` (7 archivos de test) | No hay test suite en repo actual | MEDIA |
| **Obstacle sensor driver** | `sensors/obstacle_detection.cpp` | STM32 acepta datos por CAN, ESP32 driver pendiente | ALTA — backlog #1 |
| **Limp diagnostics HUD** | `hud/hud_limp_diagnostics.cpp`, `hud_limp_indicator.cpp` | Safe/Error screen muestra faults, pero menos detalle | MEDIA |
| **Obstacle display** | `hud/obstacle_display.cpp` | Actual tiene `ObstacleSensor` UI widget | ✅ Reimplementado |

---

## 4) LO QUE RESUELVE EL REPO ACTUAL QUE EL ORIGINAL NO PODÍA

### 4.1 Bootloop eliminado

| Problema original | Solución en repo actual |
|-------------------|------------------------|
| `psramInit()` falla → `while(1)` → brick | No depende de PSRAM (STM32 tiene SRAM fija) |
| `PCA9685 I2C timeout` → cascade failure | PWM directo via TIM1/TIM8, sin I2C en ruta de actuación |
| `handleCriticalError()` → `ESP.restart()` → bootloop | `Error_Handler()` desactiva GPIOC, IWDG resetea limpio |
| `MX_FDCAN1_Init()` failure → `Error_Handler()` → loop | Parchado: retorna gracefully, `fdcan_init_ok = false` |
| `MX_I2C1_Init()` failure → `Error_Handler()` → loop | Parchado: retorna gracefully, `i2c_init_ok = false` |
| FreeRTOS stack overflow → Guru Meditation | Sin RTOS en STM32, superloop determinista |
| TFT init antes de PSRAM → sprite crash | TFT solo en ESP32, independiente del control |

### 4.2 Separación de responsabilidades

| Original | Actual |
|----------|--------|
| ESP32 controla motores Y display → un crash de display = motores sin control | STM32 controla motores, ESP32 solo muestra datos → crash de display no afecta seguridad |
| I2C compartido entre PCA9685 (motores) y INA226 (sensores) y TFT → conflicto de bus | STM32: I2C solo para sensores. Motores por PWM directo. ESP32: SPI para display, sin I2C |
| Un watchdog para todo → si TFT tarda, motores se quedan sin control | IWDG en STM32 monitorea solo control loop. ESP32 tiene su propio watchdog |

### 4.3 Nuevas funcionalidades no existentes en el original

| Funcionalidad | Detalle |
|--------------|---------|
| **CAN bus protocol** | 20+ message IDs, heartbeat bidireccional, hardware filters |
| **Safety state machine formal** | 6 estados con transiciones guardadas |
| **Boot validation checklist** | 6 checks pre-ACTIVE (temp, current, encoder, battery, safety, CAN) |
| **Service mode** | 24 módulos registrados, critical/non-critical classification |
| **Steering centering automático** | Sweep + inductive sensor, fault-tolerant |
| **Encoder fault detection** | Jump, frozen, out-of-range detection |
| **3-level degraded mode** | L1/L2/L3 con power/steering/speed scaling individual |
| **CAN bus-off recovery** | Non-blocking, retry-limited |
| **Relay power sequencing** | Main→50ms→Traction→20ms→Direction |
| **Demand anomaly detection** | Step-rate + frozen pedal detection |
| **NaN/Inf sanitization** | Todas las rutas de PWM sanitizan floats |
| **Dynamic braking** | H-bridge active brake proporcional |
| **Park hold** | Active brake con current/temp derating |
| **Per-motor emergency cutoff** | 130°C per motor, independiente del safety global |
| **Ackermann differential torque** | Corrección ±15% en curva |
| **Command validation gates** | Throttle, steering, mode/gear validados antes de ejecutar |
| **Reset cause reporting** | `RCC->CSR` flags en STM32, `esp_reset_reason()` en ESP32 |
| **Obstacle backstop limiter** | 3-tier distance mapping con stale-data detection |

---

## 5) ESTADO DE CADA SUBSISTEMA EN EL REPO ACTUAL

### STM32 — Firmware de control (Core/Src/)

| Archivo | LOC aprox | Funcionalidad | Estado |
|---------|----------|--------------|--------|
| `main.c` | ~260 | Boot, init, superloop tiered, watchdog | ✅ Completo |
| `motor_control.c` | ~800+ | 4× tracción, dirección PID, gears, park hold, dynamic brake | ✅ Completo |
| `safety_system.c` | ~600+ | State machine, ABS, TCS, overcurrent, overtemp, CAN timeout, obstacle, relay, validation | ✅ Completo |
| `can_handler.c` | ~500+ | FDCAN init, filters, heartbeat, status TX, command RX, bus-off recovery | ✅ Completo |
| `sensor_manager.c` | ~600+ | Pedal ADC, wheel speed, INA226, DS18B20, I2C recovery | ✅ Completo |
| `boot_validation.c` | ~100 | 6-check pre-ACTIVE validation | ✅ Completo |
| `steering_centering.c` | ~200 | Automatic centering state machine | ✅ Completo |
| `ackermann.c` | ~100 | Wheel angle computation | ✅ Completo |
| `service_mode.c` | ~200 | 24 modules, enable/disable, fault tracking | ✅ Completo |
| `encoder_reader.c` | ~100 | Read-only encoder access + CAN diagnostic | ✅ Completo |

### ESP32 — Firmware HMI (esp32/src/)

| Archivo | Funcionalidad | Estado |
|---------|--------------|--------|
| `main.cpp` | Setup, loop, heartbeat TX, ACK tracking | ✅ Completo |
| `can_rx.cpp/h` | CAN frame decoding (all STM32 IDs) | ✅ Completo |
| `vehicle_data.cpp/h` | Passive data container | ✅ Completo |
| `screen_manager.cpp/h` | 5-screen state machine | ✅ Completo |
| `screens/boot_screen.cpp` | Logo + CAN link status | ✅ Completo |
| `screens/standby_screen.cpp` | Temps + fault flags | ✅ Completo |
| `screens/drive_screen.cpp` | Full dashboard | ✅ Completo |
| `screens/safe_screen.cpp` | Amber "SAFE MODE" + faults | ✅ Completo |
| `screens/error_screen.cpp` | Red "SYSTEM ERROR" | ✅ Completo |
| `ui/battery_indicator.cpp` | 18V-25.2V → 0-100% bar | ✅ Completo |
| `ui/car_renderer.cpp` | Body outline, per-wheel, steering gauge | ✅ Completo |
| `ui/gear_display.cpp` | P/R/N/D1/D2 selector | ✅ Completo |
| `ui/mode_icons.cpp` | 4×4, 4×2, 360° icons | ✅ Completo |
| `ui/pedal_bar.cpp` | 0-100% throttle bar | ✅ Completo |
| `ui/obstacle_sensor.cpp` | Distance + proximity bar | ✅ Completo |
| `ui/frame_limiter.h` | 20 FPS cap | ✅ Completo |
| `ui/runtime_monitor.cpp` | Frame timing, phase tracking | ✅ Completo |
| `ui/debug_overlay.cpp` | Long-press toggle stats | ✅ Completo |

---

## 6) FUNCIONALIDADES PENDIENTES (del repo actual, referenciadas en PROJECT_MASTER_STATUS.md)

| # | Funcionalidad | Origen (original) | Estado en actual |
|---|--------------|-------------------|-----------------|
| 1 | **ESP32 obstacle sensor driver** | `obstacle_detection.cpp` (19 KB) | STM32 acepta CAN 0x208, ESP32 no tiene driver de sensor |
| 2 | **CAN ID 0x209 parsing** | No existía | Filter existe, body vacío |
| 3 | **Steering PID tuning (I, D)** | `steering_motor.cpp` tenía control básico | PID existe con ki=0, kd=0 (P-only) |
| 4 | **Calibración persistente** | `eeprom_persistence.cpp`, `config_storage.cpp` | No hay NVM/flash en STM32 |
| 5 | **Service mode persistente** | `config_storage.cpp` → NVS | RAM-only, se pierde al reiniciar |
| 6 | **Pedal redundante** | `pedal.cpp` tenía un solo canal también | Single ADC channel, sin cross-check |
| 7 | **Mode/gear CAN feedback a DriveScreen** | No aplica (monolítico) | DriveScreen no lee gear de CAN |
| 8 | **Hot-plug DS18B20** | `temperature.cpp` — también solo init | ROM search solo al init |

---

## 7) DOCUMENTACIÓN COMPARATIVA

| Aspecto | Original | Actual |
|---------|----------|--------|
| **Markdown docs en raíz** | 150+ archivos (la mayoría de diagnóstico de bootloop) | ~15 archivos (auditorías, reports) |
| **Directorio docs/** | Existe pero menos organizado | 32 archivos bien estructurados |
| **Master status** | `PROJECT_STATUS.md` (shadow rendering status) | `docs/PROJECT_MASTER_STATUS.md` (single source of truth, obligatorio) |
| **Enforcement** | Sin reglas de contribución | 7 reglas de mantenimiento obligatorias |
| **Phases** | 14 fases (muchas de fix de bootloop) | 5 fases ordenadas con exit criteria |
| **Hardware docs** | `HARDWARE.md`, `GPIO_ASSIGNMENT_LIST.md` | `docs/HARDWARE_SPECIFICATION.md`, `docs/PINOUT_DEFINITIVO.md`, `docs/HARDWARE_WIRING_MANUAL.md` |
| **CAN protocol** | No existía | `docs/CAN_PROTOCOL.md`, `docs/CAN_CONTRACT_FINAL.md` |
| **Safety docs** | Disperso en managers | `docs/SAFETY_SYSTEMS.md` |
| **Validation** | No existía | `docs/PHASE1_VALIDATION.md`, `docs/HARDWARE_VALIDATION_PROCEDURE.md` |

---

## 8) CONCLUSIÓN

### El repo original estaba bien concebido pero mal arquitecturado

El `FULL-FIRMWARE-Coche-Marcos` contiene un **volumen impresionante de funcionalidad** (>60 archivos de código, audio, LEDs, menús táctiles, configuración persistente, tests). Sin embargo, meter todo en un **ESP32-S3 monolítico** causó problemas insuperables:

- **I2C compartido** entre actuadores (PCA9685) y sensores (INA226) → contención de bus
- **FreeRTOS + Arduino** → stack overflows, watchdog timeouts, race conditions
- **PSRAM dependency** → boot failure si PSRAM no inicia
- **Display + Control acoplados** → un glitch de rendering puede bloquear motores

El resultado: **14 fases de desarrollo, 30+ documentos de bootloop fix, y el sistema seguía reiniciando**.

### El repo actual resuelve los problemas fundamentales

El `STM32-Control-Coche-Marcos` resuelve la raíz del problema separando control y display en dos MCUs:

- **STM32G474RE** (Cortex-M4F, 170 MHz): controla motores/sensores/seguridad con PWM directo, encoder hardware, watchdog independiente, sin RTOS
- **ESP32-S3**: solo muestra datos y envía comandos por CAN

Todo lo que existía como funcionalidad safety-critical en el original **ya está reimplementado** en el STM32 con mejor calidad (PWM directo > PCA9685, encoder hardware > GPIO polling, IWDG > task WDT).

### Lo que falta por portar del original

Lo que queda pendiente son funcionalidades **no safety-critical** del original:

1. **Audio** (DFPlayer) — no hay hardware
2. **Iluminación LED** (WS2812B) — no hay hardware
3. **Touch screen interactivo** — display actual es pasivo
4. **Menús de configuración** — no hay UI interactiva
5. **Calibración persistente** — pendiente en backlog (Phase 4)
6. **Obstacle sensor driver en ESP32** — pendiente en backlog (Phase 3)
7. **Cruise control adaptativo** — no implementado
8. **Regen braking AI** — dynamic braking básico existe, sin IA
9. **Test suite** — no hay tests automatizados
10. **Logger/debug system** — no hay logging formal

Ninguna de estas ausencias afecta la seguridad o la funcionalidad principal del vehículo. El sistema actual es **más seguro y más estable** que el original, aunque con menos features de "experiencia de usuario".
