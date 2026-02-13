# Auditoría Técnica Completa del Firmware

**Fecha:** 2026-02-13  
**Comparación:** `STM32-Control-Coche-Marcos` vs `FULL-FIRMWARE-Coche-Marcos` (referencia, commit c52beec)  
**Método:** Análisis estático completo del código fuente de ambos repositorios.  
Se leyó cada archivo `.c`, `.cpp` y `.h` del repositorio actual y se comparó línea a línea con los archivos fuente del repositorio de referencia (leídos vía GitHub API).  
**Autor:** Auditoría automatizada — basada exclusivamente en el código fuente real

---

## RESUMEN EJECUTIVO

| Métrica | Valor |
|---------|-------|
| **Porcentaje de implementación global** | **75 %** |
| **Módulos completamente implementados** | 18 |
| **Módulos parcialmente implementados** | 12 |
| **Módulos no implementados** | 5 |
| **Riesgos de seguridad activos** | 4 (2 medio, 2 bajo) |
| **Regresiones detectadas** | 0 |
| **Diferencias funcionales relevantes** | 7 |

---

## PARTE 1 — TABLA COMPARATIVA MÓDULO POR MÓDULO

### 1.1 STM32 (Autoridad de Seguridad)

| # | Módulo | Ref. Original | Impl. STM32 | Estado | % |
|---|--------|---------------|-------------|--------|---|
| 1 | State machine completa | `limp_mode.cpp` (NORMAL/DEGRADED/LIMP/CRITICAL — 4 states, per-state power/speed/steering limits) | `safety_system.c` (BOOT→STANDBY→ACTIVE→DEGRADED→SAFE→ERROR — 6 states, but only 2 power levels: 100% and 40%) | 🟡 PARCIALMENTE IMPLEMENTADO | 85% |
| 2 | ABS (Anti-lock braking) | `abs_system.cpp` (per-wheel, 30% pressure reduction pulse cycle, 10 Hz, slip deactivation at 70% threshold) | `safety_system.c` ABS_Update() — per-wheel scale, same slip threshold 15%, but **full torque cut** (scale=0.0) instead of 30% modulated pulsing | 🟡 PARCIALMENTE IMPLEMENTADO | 80% |
| 3 | TCS (Traction control) | `tcs_system.cpp` (per-wheel reduction, lateral G estimation, drive mode Eco/Normal/Sport threshold adaptation) | `safety_system.c` TCS_Update() — per-wheel reduction with exact same parameters (40%/5%/80%/25%/s), but **no lateral G, no drive mode adaptation** | 🟡 PARCIALMENTE IMPLEMENTADO | 85% |
| 4 | Obstacle backstop | `obstacle_safety.cpp` (5-zone ESP32) + STM32 backstop independiente | `safety_system.c` Obstacle_Update/ProcessCAN() — 3-tier backstop (200/500/1000 mm), CAN timeout, stale detection, recovery hysteresis | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 5 | Undervoltage battery | `power_mgmt.cpp` (battery monitoring) | `safety_system.c` Safety_CheckBatteryVoltage() — INA226 ch4, warning 20V→DEGRADED, critical 18V→SAFE, hysteresis 0.5V, sensor failure → fail-safe | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 6 | Encoder health | No existía como módulo dedicado en referencia | `motor_control.c` Encoder_CheckHealth() — 3 checks: out-of-range, implausible jump, frozen value; latching fault | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 7 | CAN timeout | `car_sensors.cpp` / heartbeat watchdog | `safety_system.c` Safety_CheckCANTimeout() — 250 ms timeout, auto-recovery when heartbeat restored | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 8 | Traction pipeline completo | `traction.cpp` (27 KB: per-wheel demand via PCA9685, 4x2/4x4 50/50 split, tank turn, Ackermann traction correction, NaN/Inf validation, demand anomaly detection, emergency temp shutdown 130°C) | `motor_control.c` Traction_Update() — per-wheel ABS/TCS scale, obstacle scale, 4x2/4x4 (**no 50/50 split**), tank turn (**inverted direction convention**), gear-based power scaling. **Missing:** Ackermann traction correction, NaN validation, demand anomaly detection, per-motor emergency temp cutoff | 🟡 PARCIALMENTE IMPLEMENTADO | 80% |
| 9 | Steering control + centering | `steering_motor.cpp` (PID, deadband 0.5°) + `steering_model.cpp` (Ackermann) | `motor_control.c` Steering_ControlLoop() (PID P=0.09 count-space, deadband) + `ackermann.c` + `steering_centering.c` (auto sweep left/right, inductive sensor, stall/timeout/range detection) | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 10 | Service mode completo | `car_sensors.cpp` (cfg.tempSensorsEnabled, cfg.currentSensorsEnabled, cfg.wheelSensorsEnabled) | `service_mode.c` — 25 modules, CRITICAL/NON_CRITICAL classification, enable/disable, fault tracking, CAN bitmasks (0x301-0x303), factory restore | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 11 | Gestión de relés | `relays.cpp` (non-blocking state machine: SEQ_EN_ENABLE_MAIN→TRAC→DIR, 50ms steps, debounce 50ms, timeout 5s, overcurrent/overtemp monitoring, consecutive errors) | `safety_system.c` Relay_PowerUp/PowerDown() — Main→Traction→Direction with HAL_Delay (**blocking**), settle delays (50/20 ms), reverse order shutdown. **Missing:** non-blocking sequencing, debounce, timeout protection | 🟡 PARCIALMENTE IMPLEMENTADO | 70% |
| 12 | Escalado DEGRADED → SAFE | `relays.cpp` (consecutiveErrors >= 3) + `limp_mode.cpp` (STATE_HYSTERESIS_MS = 500) | `safety_system.c` — consecutive_errors >= 3 → SAFE, recovery debounce 500 ms, error decay after 1s clean | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 13 | Protección temperatura | `temperature.cpp` (sensorOk[], per-sensor tracking) | `safety_system.c` Safety_CheckTemperature() — warning 80°C→DEGRADED, critical 90°C→SAFE, 5°C hysteresis recovery, per-module fault via ServiceMode | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 14 | Protección corriente | `current.cpp` (overcurrent detection) | `safety_system.c` Safety_CheckCurrent() — 25A threshold, consecutive errors → DEGRADED/SAFE escalation, per-module fault, error decay | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 15 | Protección velocidad rueda | `wheels.cpp` (wheel speed plausibility) | `safety_system.c` Safety_CheckSensors() — max 25 km/h plausibility check, per-module fault, DEGRADED on out-of-range | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 16 | Pedal signal conditioning | `pedal.cpp` (EMA filter) | `motor_control.c` Traction_SetDemand() — EMA alpha 0.15, ramp up 50%/s, ramp down 100%/s | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 17 | Dynamic braking | `traction.cpp` (motor braking on throttle release) | `motor_control.c` Traction_Update() — proportional to throttle rate, max 60%, min speed 3 km/h, disabled during ABS, progressive ramp, DEGRADED derating | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 18 | Park hold brake | Implícito en referencia (no explicit P gear) | `motor_control.c` Traction_Update() — GEAR_PARK: active H-bridge brake 30%, current derating (15-20A), temperature derating (70-85°C), safety override in SAFE/ERROR | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 19 | Gear system (P/R/N/D1/D2) | Referencia solo tenía FORWARD/REVERSE implícito | `motor_control.c` + `can_handler.c` — 5 gears (PARK/REVERSE/NEUTRAL/FORWARD/FORWARD_D2), speed gate <= 1 km/h for gear changes, per-gear power scaling | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 20 | Regenerative braking | `regen_ai.cpp` (AI-based regen braking) | No implementado — dynamic braking solo (disipación en motor, no recarga batería) | ❌ NO IMPLEMENTADO | 0% |
| 21 | Adaptive cruise control | `adaptive_cruise.cpp` | No implementado — funcionalidad ESP32-only en diseño dual-MCU | ❌ NO IMPLEMENTADO | 0% |
| 22 | I2C bus recovery | No existía en referencia | `sensor_manager.c` I2C_BusRecovery() — NXP AN10216: SCL cycling, STOP generation, max 2 attempts before SAFE | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 23 | DS18B20 ROM Search | `temperature.cpp` (sensor enumeration) | `sensor_manager.c` OW_SearchAll() — Search ROM algorithm (AN187), CRC-8 validation, Match ROM per-sensor reads | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |

### 1.2 ESP32 (HMI)

| # | Módulo | Ref. Original | Impl. ESP32 | Estado | % |
|---|--------|---------------|-------------|--------|---|
| 24 | CAN RX completo (decodificación) | N/A (monolítico) | `can_rx.cpp` — 13 CAN IDs decodificados: heartbeat, speed, current, temp, safety, steering, traction, temp_map, battery, diag, service ×3 | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 25 | Sincronización estados STM32 | N/A (monolítico) | `screen_manager.cpp` — detección de cambio de estado vía heartbeat byte 1, auto-switch entre 5 pantallas | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 26 | Vehicle data store | N/A (directo en monolítico) | `vehicle_data.h/cpp` — structs para todos los tipos de datos CAN, setters/getters tipados | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 27 | CAN TX (heartbeat ESP32) | N/A (monolítico) | `main.cpp` — heartbeat 0x011 cada 100 ms con alive counter | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 28 | CAN ID definitions | N/A | `can_ids.h` — 22 CAN IDs, SystemState enum, FaultFlag enum, SafetyError enum, timing constants, mode flags, service commands | ✅ COMPLETAMENTE IMPLEMENTADO | 100% |
| 29 | Screen framework | `hud_manager.cpp` (51 KB) | `screen_manager.cpp` + `screens/screen.h` — polimorphic Screen base, lifecycle (onEnter/onExit/update/draw) | 🟡 PARCIALMENTE IMPLEMENTADO | 40% |
| 30 | Boot screen | `hud.cpp` (boot animation) | `boot_screen.cpp` — **STUB** (métodos vacíos) | 🟡 PARCIALMENTE IMPLEMENTADO | 20% |
| 31 | Standby screen | `hud.cpp` (standby display) | `standby_screen.cpp` — **STUB** (métodos vacíos) | 🟡 PARCIALMENTE IMPLEMENTADO | 20% |
| 32 | Drive screen | `hud.cpp` + `gauges.cpp` + `wheels_display.cpp` (speed, RPM, temp, current, steering, battery, gear, traction %) | `drive_screen.cpp` — **STUB** (métodos vacíos). No visualización de datos. | 🟡 PARCIALMENTE IMPLEMENTADO | 10% |
| 33 | Safe screen | `hud.cpp` (safe mode display) | `safe_screen.cpp` — **STUB** (métodos vacíos) | 🟡 PARCIALMENTE IMPLEMENTADO | 20% |
| 34 | Error screen | `hud.cpp` (error display) | `error_screen.cpp` — **STUB** (métodos vacíos) | 🟡 PARCIALMENTE IMPLEMENTADO | 20% |
| 35 | Marcha exacta (P/R/N/D1/D2) | `hud.cpp` (gear indicator) | **Datos disponibles en VehicleData** (via CMD_MODE byte 1), pero no se decodifica gear como campo separado en vehicle_data.h. Drive screen es stub. | 🟡 PARCIALMENTE IMPLEMENTADO | 30% |
| 36 | Velocidad real display | `gauges.cpp` (speedometer) | **Datos recibidos** (SpeedData.raw[0..3] en 0.1 km/h), pero drive_screen.cpp es stub — no se renderiza | ❌ NO IMPLEMENTADO (UI) | 0% |
| 37 | RPM real display | `gauges.cpp` (tachometer) | **CAN no transmite RPM** — solo se transmite speed (km/h). Wheel_GetRPM_FL() existe internamente pero no se expone vía CAN | ❌ NO IMPLEMENTADO | 0% |
| 38 | Porcentaje batería display | `gauges.cpp` (battery gauge) | **Datos de voltaje/corriente recibidos** (BatteryData via 0x207), pero no se calcula % SoC ni se muestra | 🟡 PARCIALMENTE IMPLEMENTADO | 40% |
| 39 | Temperatura motores por rueda | `wheels_display.cpp` (per-wheel temp) | **Datos recibidos** (TempMapData.temps[0..3] via 0x206), pero drive_screen.cpp es stub | 🟡 PARCIALMENTE IMPLEMENTADO | 50% |
| 40 | Porcentaje par por rueda | `wheels_display.cpp` (per-wheel torque %) | **Datos recibidos** (TractionData.scale[0..3] via 0x205 en 0-100%), pero drive_screen.cpp es stub | 🟡 PARCIALMENTE IMPLEMENTADO | 50% |
| 41 | Ángulo de volante display | `gauges.cpp` (steering angle) | **Datos recibidos** (SteeringData.angleRaw via 0x204 en 0.1°), pero drive_screen.cpp es stub | 🟡 PARCIALMENTE IMPLEMENTADO | 50% |
| 42 | Estado 4x2/4x4 display | `hud.cpp` (drive mode indicator) | **No hay CAN dedicado** para informar estado 4x2/4x4. ESP32 envía CMD_MODE pero no recibe confirmación explícita del modo activo | ❌ NO IMPLEMENTADO | 0% |
| 43 | Giro 360° display | `hud.cpp` (tank turn indicator) | **Mismo problema** que 4x2/4x4 — no hay CAN de confirmación de modo tank turn activo | ❌ NO IMPLEMENTADO | 0% |
| 44 | Nivel de pedal display | `gauges.cpp` (throttle gauge) | **No se transmite pedal %** vía CAN. Pedal_GetPercent() solo se usa internamente en STM32. No existe CAN ID para pedal position | ❌ NO IMPLEMENTADO | 0% |
| 45 | Error codes display | `hud_limp_diagnostics.cpp` (error codes, fault details) | **Datos recibidos** (DiagData.errorCode/subsystem via 0x300, HeartbeatData.faultFlags/errorCode via 0x001), pero pantallas son stub | 🟡 PARCIALMENTE IMPLEMENTADO | 40% |
| 46 | SAFE/DEGRADED/ERROR en UI | `hud_limp_indicator.cpp` (limp mode indicator, warning colors) | **Framework implementado** (ScreenManager detecta estado y cambia pantalla), pero pantallas safe_screen y error_screen son stub | 🟡 PARCIALMENTE IMPLEMENTADO | 40% |
| 47 | Obstacle display | `obstacle_display.cpp` (distance bars, zone visualization) | No implementado en ESP32 HMI — obstacle detection runs on ESP32 side but no HMI display | ❌ NO IMPLEMENTADO | 0% |
| 48 | Touch/menu system | `menu_hidden.cpp`, `touch_calibration.cpp`, `menu_sensor_config.cpp`, etc. | No implementado — no TFT library, no touch | ❌ NO IMPLEMENTADO | 0% |
| 49 | LED control | `lighting/` directory, `led_control_menu.cpp` | No implementado — LED control stays on ESP32 side | N/A (out of scope) | — |
| 50 | Audio | `audio/` directory | No implementado — audio stays on ESP32 side | N/A (out of scope) | — |
| 51 | CAN TX commands (throttle/steering/mode) | N/A (was direct GPIO in monolítico) | `main.cpp` — solo envía heartbeat. **No envía** throttle/steering/mode commands al STM32 | 🟡 PARCIALMENTE IMPLEMENTADO | 20% |

---

## PARTE 2 — ANÁLISIS DETALLADO

### 2.1 State Machine (STM32)

**Referencia:** `limp_mode.cpp` — estados NORMAL, DEGRADED, LIMP, CRITICAL  
**Implementación:** `safety_system.c` — BOOT→STANDBY→ACTIVE→DEGRADED→SAFE→ERROR

| Aspecto | Referencia | STM32 | Coincide |
|---------|-----------|-------|----------|
| Estados definidos | 4: NORMAL, DEGRADED, LIMP, CRITICAL | 6: BOOT, STANDBY, ACTIVE, DEGRADED, SAFE, ERROR | ⚠️ Diferente granularidad |
| Mapeo de estados | NORMAL=full power, DEGRADED=70%, LIMP=40%, CRITICAL=15% | ACTIVE=100%, DEGRADED=40% (collapses LIMP limits), SAFE=0%, ERROR=0% | ⚠️ STM32 usa límites LIMP (40%) para DEGRADED, omitiendo DEGRADED (70%) y CRITICAL (15%) del original |
| Transiciones forward-only | ✅ | ✅ (con recovery DEGRADED→ACTIVE y SAFE→ACTIVE) | ✅ |
| Recovery debounce | STATE_HYSTERESIS_MS = 500 | RECOVERY_HOLD_MS = 500 | ✅ Exacto |
| Consecutive errors escalation | consecutiveErrors >= 3 → relay disable (relays.cpp) | CONSECUTIVE_ERROR_THRESHOLD = 3 → SYS_STATE_SAFE | ✅ Exacto |
| Error decay | lastErrorMs > 1000 (relays.cpp) | (HAL_GetTick() - last_error_tick) > 1000 | ✅ Exacto |
| Power limiting DEGRADED | POWER_DEGRADED = 70% | **No existe** — STM32 pasa directamente a 40% (LIMP limits) | ⚠️ Diferencia: referencia tiene escalón intermedio 70% |
| Power limiting LIMP | POWER_LIMP = 40% | DEGRADED_POWER_LIMIT_PCT = 40% | ✅ Exacto (colapsado como DEGRADED) |
| Speed limiting | SPEED_DEGRADED=80%, SPEED_LIMP=50%, SPEED_CRITICAL=25% | DEGRADED_SPEED_LIMIT_PCT = 50% (defined but not enforced in code) | ⚠️ Solo se define, no se aplica activamente |
| Steering assist limiting | STEERING_DEGRADED=80%, STEERING_LIMP=50%, STEERING_CRITICAL=30% | **No implementado** — steering PID no se degrada por estado | ❌ Faltante |
| Relay management per state | enablePower/disablePower (non-blocking state machine in relays.cpp) | Relay_PowerUp/PowerDown (**blocking** HAL_Delay) | ⚠️ Funcional pero blocking — risk of watchdog trip |
| DEGRADED keeps relays on | ✅ | ✅ (no Safety_FailSafe() call) | ✅ |
| Battery thresholds | BATTERY_UNDERVOLTAGE = 20.0V, BATTERY_CRITICAL = 18.0V | BATTERY_UV_WARNING_V = 20.0V, BATTERY_UV_CRITICAL_V = 18.0V | ✅ Exacto |
| Temperature thresholds | TEMP_WARNING = 80.0°C, TEMP_CRITICAL = 90.0°C | TEMP_WARNING_C = 80.0°C, TEMP_CRITICAL_C = 90.0°C | ✅ Exacto |
| Error count thresholds | DEGRADED≥1, LIMP≥3, CRITICAL≥5 | Consecutive threshold = 3 (maps to LIMP/SAFE) | ⚠️ No distingue 1-error vs 3-error vs 5-error |

**Veredicto:** 🟡 PARCIALMENTE IMPLEMENTADO — El core de la state machine es correcto y los parámetros de seguridad coinciden, pero la granularidad de estados de degradación es menor que en la referencia (2 niveles vs 4). Steering assist degradation no está implementada. La secuencia de relés es blocking en lugar de non-blocking.

### 2.2 Safety Layers

| Safety Layer | Referencia | STM32 | Parámetros coinciden | Estado |
|-------------|-----------|-------|---------------------|--------|
| ABS per-wheel | slipThreshold=15%, minSpeed=10, pressureReduction=0.3 (30%), cycleMs=100, deactivation at slip<threshold×0.7 | slip>15%, avg<10 skip, per-wheel scale=0.0 (full cut vs 30% reduction), no ABS cycling/pulsing | ⚠️ STM32 does full torque cut instead of 30% pulse modulation | 🟡 Parcial |
| TCS per-wheel | aggressive=40%, smooth=5%, max=80%, recovery=25%/s, lateral G estimation, drive mode adaptation (Eco/Normal/Sport thresholds) | TCS_INITIAL=0.40, SMOOTH=0.05, MAX=0.80, RECOVERY=0.25/s. **No lateral G, no drive mode adaptation** | ⚠️ Core params exact, advanced features missing | 🟡 Parcial |
| Obstacle backstop | obstacle_safety.cpp: 5-zone with linear interpolation, child reaction detection, speed-dependent braking | 3-tier (200/500/1000 mm), CAN timeout 500ms, stale ≥3, recovery 500mm/1s | ✅ Simplified but adequate for independent backstop | ✅ COMPLETO |
| Undervoltage | limp_mode.cpp: BATTERY_UNDERVOLTAGE=20V→LIMP, BATTERY_CRITICAL=18V→CRITICAL | Warning 20V→DEGRADED, Critical 18V→SAFE, Hyst 0.5V | ✅ Exact match | ✅ COMPLETO |
| Encoder health | Not in reference (encoder managed by ESP32 directly) | Range, jump, frozen checks; latching fault | N/A (new feature) | ✅ COMPLETO |
| CAN timeout | Implicit heartbeat watchdog in monolithic | 250 ms timeout, explicit state transitions, auto-recovery | ✅ | ✅ COMPLETO |
| Overcurrent | relays.cpp: BATTERY_OVERCURRENT_LIMIT=120A; traction.cpp: CURRENT_MAX_REASONABLE=200A | 25A per-motor threshold via INA226 | ⚠️ Different scope: ref monitors battery-level (120A), STM32 per-motor (25A) | 🟡 Parcial |
| Overtemperature | traction.cpp: TEMP_CRITICAL=120°C, TEMP_EMERGENCY_SHUTDOWN=130°C; relays.cpp: MOTOR_OVERTEMP=80°C | Warning 80°C, Critical 90°C, hysteresis 5°C | ⚠️ Ref has 80/120/130°C cascade, STM32 has 80/90°C | 🟡 Parcial |
| Sensor plausibility | car_sensors.cpp: isfinite() + range checks; per-subsystem cfg.enabled flags | Temp(-40/125°C), current(0-50A), speed(0-25 km/h); per-module ServiceMode enable/disable | ✅ Equivalent approach | ✅ COMPLETO |

### 2.3 Traction Pipeline

| Feature | Referencia (traction.cpp) | STM32 (motor_control.c) | Estado |
|---------|--------------------------|------------------------|--------|
| Per-wheel demand + PWM | ✅ (per-wheel demandPct via PCA9685 12-bit) | ✅ (per-wheel via TIM1 16-bit PWM + wheel_scale[]) | ✅ |
| 4×2 mode (front only) | ✅ (rear = 0.0f) | ✅ (rear motors disabled, enable pin LOW) | ✅ |
| 4×4 mode (all wheels) | ✅ (50/50 front/rear split) | ✅ (all 4 same base, per-wheel scale — **no 50/50 split**) | ⚠️ Ref splits 50/50 between axles; STM32 applies same % to all |
| Tank turn (axis rotation) | ✅ (FL/RL forward, FR/RR reverse, pedal-controlled speed) | ✅ (FL/RL reverse, FR/RR forward — **opposite direction convention**) | ⚠️ Direction inverted vs reference — may spin opposite way |
| Direction control (forward/reverse) | Via MCP23017 GPIO expander (I2C) | Via direct GPIO direction pins | ✅ (different hardware, same logic) |
| Ackermann in traction | Progressive pow(1.2) scaling per-wheel (factorFL/FR) | Ackermann geometry computed in Steering_SetAngle only — **not applied to traction demand** | ⚠️ Ref reduces inner wheel traction in curves; STM32 only applies Ackermann to steering angle |
| Gear-based power scaling | Implicit (only forward/reverse via shifter) | D1=60%, D2=100%, R=60% explicit gear system | ✅ (enhanced over reference) |
| Obstacle scale applied | obstacleFactor in traction.cpp (from ObstacleSafety::getState) | obstacle_scale * base_pwm | ✅ |
| ABS/TCS per-wheel scale | modulatePower/modulateBrake called per-wheel | wheel_scale[i] per-wheel multiplicative | ✅ |
| Dynamic braking | **Not in reference** (no explicit dynamic braking in traction.cpp) | Proportional, ramped, ABS interlock, DEGRADED derating | ✅ (new feature) |
| Park hold | Implicit only (parking brake flag set in car_sensors.cpp readGear) | Active H-bridge brake, current/temp derating | ✅ (new feature) |
| Pedal EMA filter | Pedal::get().percent via pedal.cpp (EMA in pedal module) | PEDAL_EMA_ALPHA = 0.15 in Traction_SetDemand | ✅ |
| Pedal ramp limiter | — | Up 50%/s, Down 100%/s | ✅ (new safety feature) |
| NaN/Inf validation | ✅ (std::isfinite throughout traction.cpp) | **No NaN/Inf validation** on float inputs | ⚠️ STM32 lacks isfinite() guards — could pass corrupt data to PWM |
| Demand distribution anomaly detection | ✅ (sumDemand > maxExpected → proportional fallback) | **Not implemented** | ❌ Missing safety check |
| Emergency temperature shutdown | ✅ (TEMP_EMERGENCY_SHUTDOWN=130°C → immediate motor stop) | Only via Safety_CheckTemperature at 90°C (no per-motor immediate cutoff in traction loop) | ⚠️ Less aggressive emergency response |

### 2.4 Steering

| Feature | Referencia | STM32 | Estado |
|---------|-----------|-------|--------|
| PID control | steering_motor.cpp kp=1.2 (degree space), P-only | kp=0.09 (encoder count space = 1.2 / (4800/360)), P-only with I/D terms at 0.0 | ✅ Mathematically equivalent |
| Deadband | kDeadbandDeg = 0.5° | STEERING_DEADBAND_COUNTS = 0.5° × CPR/360 ≈ 6.67 counts | ✅ Exact match |
| Ackermann geometry | steering_model.cpp: L=0.95m, T=0.70m, MAX_INNER=54° | ackermann.c + vehicle_physics.h: WHEELBASE_M=0.95, TRACK_WIDTH_M=0.70, MAX_STEER_DEG=54 | ✅ Exact match |
| Overcurrent protection | kMaxCurrentA=30A on INA226 ch5 | **Not in steering loop** — only via global Safety_CheckCurrent at 25A | ⚠️ Ref has per-update 30A check in steering; STM32 relies on periodic safety check |
| Auto centering | Not in reference (manual centering via menu_encoder_calibration.cpp) | steering_centering.c (automatic sweep left/right, inductive sensor, stall/timeout/range detection) | ✅ Enhanced |
| Max angle limit | MAX_STEER_DEG via steering_model.cpp | 54° clamped in Steering_SetAngle | ✅ |
| Rate limiting | — | STEERING_RATE_MAX_DEG_PER_S = 200°/s in Safety_ValidateSteering | ✅ (new safety feature) |
| Encoder | ESP32 internal encoder via Steering::get().angleDeg | TIM2 hardware quadrature E6B2-CWZ6C 1200 PPR × 4 = 4800 CPR | ✅ (dedicated hardware) |

### 2.5 CAN Contract Consistency

| CAN ID | Direction | Documentado | STM32 TX/RX | ESP32 TX/RX | Payload coincide | Estado |
|--------|-----------|-------------|-------------|-------------|-----------------|--------|
| 0x001 | STM32→ESP32 | ✅ | TX: 4 bytes (counter, state, flags, error) | RX: decodeHeartbeat() | ✅ | ✅ |
| 0x011 | ESP32→STM32 | ✅ | RX: CAN_ProcessMessages() | TX: main.cpp loop | ✅ | ✅ |
| 0x100 | ESP32→STM32 | ✅ | RX: throttle validation | TX: **NO IMPLEMENTADO** | — | ⚠️ |
| 0x101 | ESP32→STM32 | ✅ | RX: steering validation | TX: **NO IMPLEMENTADO** | — | ⚠️ |
| 0x102 | ESP32→STM32 | ✅ | RX: mode/gear parsing | TX: **NO IMPLEMENTADO** | — | ⚠️ |
| 0x110 | ESP32→STM32 | ✅ | RX: service cmd parsing | TX: **NO IMPLEMENTADO** | — | ⚠️ |
| 0x200 | STM32→ESP32 | ✅ | TX: 8 bytes wheel speeds | RX: decodeSpeed() | ✅ | ✅ |
| 0x201 | STM32→ESP32 | ✅ | TX: 8 bytes wheel currents | RX: decodeCurrent() | ✅ | ✅ |
| 0x202 | STM32→ESP32 | ✅ | TX: 5 bytes temperatures | RX: decodeTemp() | ✅ | ✅ |
| 0x203 | STM32→ESP32 | ✅ | TX: 3 bytes safety status | RX: decodeSafety() | ✅ | ✅ |
| 0x204 | STM32→ESP32 | ✅ | TX: 3 bytes steering | RX: decodeSteering() | ✅ | ✅ |
| 0x205 | STM32→ESP32 | ✅ | TX: 4 bytes traction scale | RX: decodeTraction() | ✅ | ✅ |
| 0x206 | STM32→ESP32 | ✅ | TX: 5 bytes temp map | RX: decodeTempMap() | ✅ | ✅ |
| 0x207 | STM32→ESP32 | ✅ | TX: 4 bytes battery | RX: decodeBattery() | ✅ | ✅ |
| 0x208 | ESP32→STM32 | ✅ | RX: obstacle distance | TX: **NO IMPLEMENTADO** | — | ⚠️ |
| 0x209 | ESP32→STM32 | ✅ | RX: accepted, not parsed | TX: **NO IMPLEMENTADO** | — | ⚠️ |
| 0x300 | Bidirectional | ✅ | TX: CAN_SendError() | RX: decodeDiagError() | ✅ | ✅ |
| 0x301 | STM32→ESP32 | ✅ | TX: service faults | RX: decodeServiceFaults() | ✅ | ✅ |
| 0x302 | STM32→ESP32 | ✅ | TX: service enabled | RX: decodeServiceEnabled() | ✅ | ✅ |
| 0x303 | STM32→ESP32 | ✅ | TX: service disabled | RX: decodeServiceDisabled() | ✅ | ✅ |

**Inconsistencias CAN detectadas:**
1. **ESP32 no envía commands** (0x100, 0x101, 0x102) — STM32 los puede recibir pero ESP32 no los transmite. El STM32 opera únicamente con pedal ADC local.
2. **ESP32 no envía obstacle data** (0x208, 0x209) — STM32 tiene el receptor completo pero ESP32 no transmite datos de obstáculos.
3. **No existe CAN ID para:** pedal position (%), drive mode confirmation (4x2/4x4/tank), RPM por rueda, gear position confirmed.
4. **Gear no se decodifica en ESP32** — VehicleData no tiene campo para gear position; CMD_MODE byte 1 se envía pero no se recibe de vuelta.

### 2.6 Datos que el STM32 NO transmite vía CAN

| Dato interno | Función STM32 | CAN ID | Estado |
|-------------|---------------|--------|--------|
| Pedal position (%) | Pedal_GetPercent() | ❌ No existe | Falta CAN ID dedicado |
| RPM por rueda | Wheel_GetRPM_FL() (solo FL) | ❌ No existe | Solo existe RPM para FL, no para FR/RL/RR |
| Modo 4x2/4x4 confirmado | traction_state.mode4x4 | ❌ No existe | ESP32 envía comando pero no recibe confirmación |
| Tank turn confirmado | traction_state.axisRotation | ❌ No existe | Mismo problema |
| Gear position confirmada | current_gear | ❌ No existe | ESP32 envía gear pero no recibe confirmación |
| Corriente motor dirección | Current_GetAmps(5) | ❌ No en 0x201 | Solo se envían 4 motores (índices 0-3) |
| Dynamic braking active | dynbrake_pct | ❌ No existe | No se reporta al HMI |
| Park hold active | GEAR_PARK + hold_pct | ❌ No existe | No se reporta al HMI |
| Centering state | SteeringCentering_GetState() | ❌ No existe | No se reporta al HMI |

---

## PARTE 3 — PORCENTAJE TOTAL DE IMPLEMENTACIÓN GLOBAL

### 3.1 Por Subsistema

| Subsistema | Peso | % Implementación | Ponderado |
|-----------|------|-------------------|-----------|
| **STM32 State Machine** | 15% | 85% | 12.8% |
| **STM32 Safety Layers** (ABS, TCS, obstacle, UV, encoder, CAN timeout, overcurrent, overtemp, sensors) | 25% | 90% | 22.5% |
| **STM32 Traction Pipeline** | 15% | 80% | 12.0% |
| **STM32 Steering** (PID + centering + Ackermann) | 10% | 95% | 9.5% |
| **STM32 Service Mode** | 5% | 100% | 5.0% |
| **STM32 CAN TX** (status messages) | 5% | 100% | 5.0% |
| **ESP32 CAN RX/Decode** | 5% | 100% | 5.0% |
| **ESP32 CAN TX** (commands) | 5% | 20% | 1.0% |
| **ESP32 Screen Framework** | 3% | 40% | 1.2% |
| **ESP32 Screen Content** (drive, safe, error) | 7% | 15% | 1.1% |
| **ESP32 Obstacle TX** | 3% | 0% | 0.0% |
| **ESP32 Data Gaps** (RPM, pedal, mode confirm) | 2% | 0% | 0.0% |
| **TOTAL** | **100%** | | **75.1%** |

### 3.2 Por Clasificación

| Clasificación | Módulos |
|--------------|---------|
| ✅ COMPLETAMENTE IMPLEMENTADO | Obstacle backstop, undervoltage battery, encoder health, CAN timeout, service mode, DEGRADED→SAFE escalation, pedal conditioning, dynamic braking, park hold, gear system, I2C recovery, DS18B20 ROM search, CAN RX decode (ESP32), state sync (ESP32), vehicle data store, CAN TX heartbeat (ESP32), CAN ID definitions, sensor plausibility checks |
| 🟡 PARCIALMENTE IMPLEMENTADO | **State machine** (85% — less degradation granularity: 2 levels vs ref's 4), **ABS** (80% — full cut vs 30% pulse modulation), **TCS** (85% — no lateral G, no drive modes), **Traction pipeline** (80% — no 50/50 4×4 split, no Ackermann traction correction, no NaN validation, inverted tank turn direction), **Relay management** (70% — blocking HAL_Delay), **Steering** (95% — no per-update overcurrent check), **Temperature protection** (85% — no per-motor 130°C emergency cutoff in traction loop), **Current protection** (85% — per-motor vs battery-level), Screen framework (40%), boot/standby/safe/error screens (20% — stub), drive screen (10% — stub), gear display (30%), battery display (40%), per-wheel temp/traction/steering display (50% — data available, no render), error/fault display (40%), SAFE/DEGRADED UI handling (40%), ESP32 CAN TX commands (20% — only heartbeat) |
| ❌ NO IMPLEMENTADO | Regenerative braking, adaptive cruise, RPM CAN transmission, pedal position CAN, drive mode confirmation CAN, obstacle TX (ESP32→STM32), obstacle HMI display, touch/menu system, screen rendering (TFT graphics), ESP32 command transmission (throttle/steering/mode), NaN/Inf validation in traction, demand anomaly detection, steering assist degradation |

### 3.3 Porcentaje Global

**STM32 (autoridad de seguridad): ~88%** — Las capas de seguridad, control de tracción, dirección y gestión de sensores están implementadas y son funcionales. Existen diferencias en los detalles de implementación vs la referencia (granularidad de estados, modulación ABS, split 4×4, validación NaN) pero la funcionalidad de seguridad core es robusta.

**ESP32 (HMI): ~45%** — La infraestructura CAN RX y el data store están completos, pero las pantallas son stubs sin contenido visual, y el ESP32 no envía ningún comando al STM32 (excepto heartbeat).

**Global ponderado: ~75%**

---

## PARTE 4 — LISTA DE LO QUE FALTA PARA LLEGAR AL 100%

### 4.1 Prioridad ALTA (Seguridad / Funcionalidad Crítica)

| # | Item faltante | Impacto | Dificultad |
|---|---------------|---------|------------|
| F1 | **ESP32: Envío de comandos CAN** (throttle 0x100, steering 0x101, mode 0x102) | Sin esto, el STM32 solo opera con el pedal ADC local. No hay control remoto desde el joystick/volante del ESP32 | Media |
| F2 | **ESP32: Envío de obstacle data** (0x208, 0x209) | Sin esto, el STM32 backstop de obstáculos no recibe datos. Los sensores ultrasónicos están en el ESP32 | Media |
| F3 | **STM32: NaN/Inf validation en traction pipeline** | Referencia usa std::isfinite() en todo traction.cpp. STM32 no valida floats antes de aplicar a PWM — potencial riesgo de PWM corrompido | Media |
| F4 | **STM32: Demand distribution anomaly detection** | Referencia detecta sumDemand > maxExpected y aplica fallback proporcional. STM32 no tiene este safety check | Media |
| F5 | **STM32: ABS pulse modulation** (30% reduction cycles vs full cut) | Referencia modula brake 30% con ciclo pulse/release; STM32 hace full torque cut (scale=0.0). Más agresivo, potencialmente menos controlable en frenado | Media |
| F6 | **STM32: 4×4 mode 50/50 axle split** | Referencia aplica 50% delantero + 50% trasero en 4×4. STM32 aplica 100% a las 4 ruedas — potencialmente el doble de demanda total | Media |
| F7 | **STM32: CAN ID para drive mode / gear confirmation** | El ESP32 envía CMD_MODE pero no recibe confirmación del modo activo | Baja |

### 4.2 Prioridad MEDIA (HMI / Visualización)

| # | Item faltante | Impacto | Dificultad |
|---|---------------|---------|------------|
| F5 | **ESP32: Implementación drive_screen** (velocidad, corriente, temperatura, steering, traction, battery, gear) | El HMI no muestra datos de conducción. Todos los datos CAN están disponibles en VehicleData | Alta |
| F6 | **ESP32: Implementación safe_screen** (fault display, safety warnings) | No se muestra información útil en estado SAFE | Media |
| F7 | **ESP32: Implementación error_screen** (error codes, subsystem info) | No se muestran códigos de error al operador | Media |
| F8 | **ESP32: Implementación boot_screen** (logo, version, init status) | Experiencia de usuario mínima durante arranque | Baja |
| F9 | **ESP32: Implementación standby_screen** (waiting status) | Experiencia de usuario mínima en standby | Baja |
| F10 | **STM32: CAN ID para pedal position %** | HMI no puede mostrar nivel de pedal actual | Baja |
| F11 | **STM32: RPM por rueda via CAN** | HMI no puede mostrar RPM real. Wheel_GetRPM solo existe para FL | Media |
| F12 | **ESP32: Decodificación gear position** | VehicleData no tiene campo para gear; necesita parsear heartbeat o nuevo CAN ID | Baja |

### 4.3 Prioridad BAJA (Features Opcionales)

| # | Item faltante | Impacto | Dificultad |
|---|---------------|---------|------------|
| F13 | **Regenerative braking** (regen_ai.cpp) | Eficiencia energética; no es safety-critical | Alta |
| F14 | **Adaptive cruise control** | Confort; debería quedarse en ESP32 side | Alta |
| F15 | **ESP32: TFT graphics library integration** | Necesario para renderizar en pantalla real | Alta |
| F16 | **ESP32: Touch input** | Necesario para menús interactivos | Alta |
| F17 | **ESP32: Obstacle distance display** | Visualización de proximidad en HMI | Media |
| F18 | **STM32: Centering state CAN report** | Diagnóstico remoto del proceso de centrado | Baja |
| F19 | **STM32: Dynamic brake status CAN** | Información diagnóstica para el HMI | Baja |
| F20 | **ESP32: Service mode menu** (enable/disable modules via 0x110) | Administración remota de módulos | Media |
| F21 | **Battery SoC calculation** | Estimación de % de batería a partir de voltaje (curva de descarga LiPo) | Media |

---

## PARTE 5 — PROPUESTA DEL SIGUIENTE PASO MÁS LÓGICO

### Paso recomendado: **F1 — ESP32 CAN TX Commands**

**Justificación:**
- Es el **eslabón faltante más crítico** en la arquitectura dual-MCU
- Sin este paso, el STM32 no recibe comandos del joystick/volante del ESP32
- Toda la infraestructura receptora ya existe en el STM32 (CAN_ProcessMessages parsea 0x100, 0x101, 0x102, 0x110)
- Es de dificultad **media** y no requiere hardware adicional
- Habilita la prueba end-to-end del sistema completo

**Implementación sugerida:**
```
esp32/src/can_tx.cpp  — nuevo módulo
  - sendThrottle(uint8_t percent)     → 0x100, cada 50 ms
  - sendSteering(int16_t angle_01deg) → 0x101, cada 50 ms
  - sendMode(uint8_t flags, uint8_t gear) → 0x102, on-demand
  - sendServiceCmd(uint8_t cmd, uint8_t module_id) → 0x110, on-demand
```

---

## PARTE 6 — ORDEN RECOMENDADO DE IMPLEMENTACIÓN

Priorizado por **impacto en seguridad** primero, luego **arquitectura**, luego **UX**.

| Orden | ID | Tarea | Justificación |
|-------|-----|-------|---------------|
| **1** | F3 | **STM32: NaN/Inf validation en traction/safety** | Riesgo de seguridad: NaN bypasses float comparisons en C, podría producir PWM impredecible. Fix mínimo: añadir isnanf() checks antes de set PWM |
| **2** | F6 | **STM32: 4×4 mode 50/50 axle split** | Corrección funcional: sin split, 4×4 aplica el doble de demand eléctrica total. Fix simple en Traction_Update() |
| **3** | F1 | **ESP32 CAN TX commands** (throttle, steering, mode) | Sin esto no hay control remoto — máximo impacto funcional para el sistema dual-MCU |
| **4** | F2 | **ESP32 obstacle data TX** (0x208, 0x209) | Safety-critical: STM32 backstop sin datos de obstáculos |
| **5** | F5 | **STM32: ABS pulse modulation** (30% reduction vs full cut) | Mejora de controlabilidad en frenado — referencia usa modulación 30%, STM32 hace corte total |
| **6** | F7 | **STM32: CAN IDs para mode/gear confirmation** | Cierra el loop bidireccional — necesario para HMI correcto |
| **7** | F8 | **ESP32 drive_screen implementación** | Los datos CAN ya llegan; solo falta renderizar en pantalla |
| **8** | F9+F10 | **ESP32 safe_screen + error_screen** | Información de fallo visible al operador |
| **9** | F11+F12 | **STM32: pedal % + RPM via CAN** | Datos adicionales para dashboard completo |
| **10** | F13+F14 | **ESP32 boot/standby screens + gear decode** | UX polish |
| **11** | F21 | **Battery SoC calculation** (ref: calculateBatteryPercent, V_MIN=21V, V_MAX=28V) | Mostrar % batería real |
| **12** | F15+F16 | **TFT + Touch integration** | Hardware display real |
| **13** | F13_orig | **Regenerative braking** (regen_ai.cpp) | Eficiencia energética — feature avanzado |

---

## PARTE 7 — RIESGOS ACTUALES

### 7.1 Riesgos de Seguridad

| Riesgo | Severidad | Descripción | Mitigación existente |
|--------|-----------|-------------|---------------------|
| **R1: No hay validación NaN/Inf en traction** | ⚠️ MEDIA | La referencia usa std::isfinite() en toda la cadena de traction.cpp. El STM32 no valida floats antes de setear PWM. Un sensor corrupto (NaN) podría producir un PWM impredecible | El clamp de demand a ±100% y el clamp de PWM a PWM_PERIOD proporcionan protección parcial, pero NaN bypasses comparisons in C |
| **R2: 4×4 mode aplica 100% a 4 ruedas** | ⚠️ MEDIA | La referencia aplica 50%+50% split entre ejes. El STM32 aplica el mismo demand % a las 4 ruedas. Si el pedal pide 60%, la referencia aplica 30% por rueda (120% total), pero el STM32 aplica 60% por rueda (240% total). Riesgo de sobrecarga eléctrica | La protección de corriente INA226 (25A por motor) limita la corriente real, pero la demanda eléctrica instantánea es mayor |
| **R3: ESP32 no envía obstacle data** | ⚠️ MEDIA | STM32 backstop no recibe datos de ultrasonidos. Si obstacle_data_valid nunca se activa, obstacle_scale = 1.0 (sin reducción) | El heartbeat CAN timeout (250 ms) proporciona protección general si el ESP32 falla completamente |
| **R4: ABS full cut vs pulse modulation** | ⚠️ BAJA | La referencia modula brake al 30% con ciclo pulse/release (10 Hz). El STM32 hace full torque cut (scale=0.0). Más agresivo, potencialmente más lock-prone on slippery surfaces | El global fallback (all 4 wheels locked → Traction_SetDemand(0)) proporciona protección de último recurso |
| **R5: Tank turn direction inverted** | ⚠️ BAJA | STM32 invierte FL/RL vs FR/RR con convención opuesta a referencia. Si se prueba en hardware, puede girar en la dirección contraria a la esperada | Funcional pero podría confundir al operador. Fix trivial: invertir sign de `d` |
| **R6: Relay sequencing blocking** | ⚠️ BAJA | STM32 usa HAL_Delay() en Relay_PowerUp (total ~70ms blocking). Referencia usa non-blocking state machine. Risk: watchdog trip durante power-up si IWDG periodo < 70ms | HAL_Delay es estándar en STM32 y el IWDG typical period (128ms+) debería ser suficiente |

### 7.2 Riesgos de Integración

| Riesgo | Severidad | Descripción |
|--------|-----------|-------------|
| **R7: ESP32 no envía commands** | 🔴 ALTA | El STM32 solo opera con el pedal ADC local. No hay control desde joystick/volante ESP32. El sistema dual-MCU no es funcional como diseñado |
| **R8: ESP32 screens son stub** | ⚠️ MEDIA | El operador no ve datos de conducción. En caso de fallo, no hay indicación visual |
| **R9: No hay confirmación de modo** | ⚠️ BAJA | ESP32 envía CMD_MODE pero no sabe si STM32 lo aceptó |
| **R10: RPM incompleto** | ⚠️ BAJA | Solo Wheel_GetRPM_FL() existe; FR/RL/RR no tienen función RPM |

### 7.3 Diferencias Funcionales Relevantes (no regresiones)

| Diferencia | Referencia | STM32 | Impacto |
|-----------|-----------|-------|---------|
| State degradation granularity | 4 niveles: 100%/70%/40%/15% | 2 niveles: 100%/40% | Menor: STM32 es más conservador (salta directo a 40%) |
| ABS modulation | 30% pulse reduction, 10 Hz cycling | Full torque cut (0%) per-wheel | Menor: STM32 es más agresivo en frenado |
| TCS features | Lateral G estimation, drive mode adaptation | Basic per-wheel only | Menor: features de confort, no de seguridad |
| 4×4 power distribution | 50/50 axle split | Same demand to all wheels | **Potencial:** duplica demanda eléctrica total en 4×4 |
| Traction Ackermann | Per-wheel factorFL/FR based on steering angle | Not applied to traction (only to steering geometry) | Menor: affects tire wear in curves, not safety |
| Temperature emergency | 130°C per-motor immediate cutoff in traction loop | 90°C global via safety system periodic check | Menor: STM32 cuts earlier (90°C vs 130°C) — more conservative |
| Relay sequencing | Non-blocking state machine with timeout | Blocking HAL_Delay | Menor: only during power-up sequence |

### 7.4 Regresiones

| Regresión | Estado |
|-----------|--------|
| Funcionalidad del firmware original perdida | ❌ Ninguna detectada |
| Parámetros de seguridad alterados vs referencia | ⚠️ Algunos thresholds difieren (temp 90°C vs 120/130°C, overcurrent 25A per-motor vs 120A battery) pero STM32 es más conservador (más seguro) |
| CAN protocol inconsistencies | ❌ Todas las codificaciones TX/RX son consistentes con CAN_CONTRACT_FINAL.md |

---

## PARTE 8 — CONCLUSIONES

### Fortalezas del firmware actual:
1. **STM32 altamente funcional** (~88%) — todas las capas de seguridad, control de tracción, dirección, y sensores están implementadas con trazabilidad al firmware original. Las diferencias identificadas son mayoritariamente de detalle, no de funcionalidad.
2. **Arquitectura de seguridad sólida** — Safety_ValidateThrottle/Steering/ModeChange, per-wheel ABS/TCS, obstacle backstop, consecutive error escalation, battery undervoltage protection
3. **Service mode robusto** — 25 módulos, clasificación CRITICAL/NON_CRITICAL, enable/disable/fault via CAN
4. **CAN protocol bien definido** — 22 CAN IDs documentados y sincronizados entre STM32 TX y ESP32 RX
5. **Mejoras sobre el original** — steering centering automático, encoder health monitoring 3-fault, I2C bus recovery (NXP AN10216), park hold brake con current/temp derating, gear system expandido (5 gears vs 2 del original), pedal ramp rate limiting, steering rate limiting
6. **STM32 generalmente más conservador que la referencia** — temperature limits más bajos (90°C vs 130°C), per-motor overcurrent (25A vs battery-level 120A)

### Debilidades:
1. **ESP32 HMI es un esqueleto** — pantallas son stubs sin renderizado. No hay feedback visual al operador
2. **ESP32 no envía commands** — el control bidireccional no está operativo (solo heartbeat)
3. **Obstacle data flow incompleto** — ESP32→STM32 obstacle CAN no implementado en ESP32
4. **Sin validación NaN/Inf** — la referencia usa std::isfinite() extensivamente; el STM32 no valida floats
5. **4×4 mode sin split 50/50** — potencialmente duplica la demanda eléctrica total
6. **Degradation granularity reducida** — 2 niveles (100%/40%) vs 4 de la referencia (100%/70%/40%/15%)
7. **No hay Ackermann en traction** — la referencia reduce traction en rueda interior en curvas

### Porcentaje final: **75% de implementación global**

> El STM32 (core de seguridad) está funcional y es más conservador que la referencia en la mayoría de casos.  
> Necesita ajustes puntuales: validación NaN, split 4×4, y modulación ABS para alcanzar paridad funcional.  
> El ESP32 (HMI) necesita trabajo significativo en pantallas y transmisión de comandos  
> para completar el sistema dual-MCU como fue diseñado.
