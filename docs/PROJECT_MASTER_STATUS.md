# PROJECT MASTER STATUS — Single Source of Truth

> **This document is mandatory.** Every PR must update it before being considered complete.
> Last updated: 2026-02-16

---

## 1) CURRENT SYSTEM ARCHITECTURE

The system is a dual-MCU vehicle control platform communicating over CAN bus.

### STM32G474RE Responsibilities

The STM32 is the **safety authority** and sole actuator controller. All actuator commands originate from or are validated by the STM32 before reaching hardware.

| Responsibility | Module / Function | File |
|---|---|---|
| System clock (170 MHz from HSI + PLL) | `SystemClock_Config()` | `Core/Src/main.c` |
| Main control loop (10 ms / 50 ms / 100 ms / 1000 ms tiers) | `main()` while-loop | `Core/Src/main.c` |
| 4× traction motor PWM (TIM1 CH1–CH4, 20 kHz) | `Motor_Init()`, `Traction_Update()` | `Core/Src/motor_control.c` |
| Steering motor PWM + PID (TIM8 CH3, 20 kHz) | `Steering_ControlLoop()`, `PID_Compute()` | `Core/Src/motor_control.c` |
| Steering encoder (TIM2 quadrature, E6B2-CWZ6C, 4800 CPR) | `Steering_Init()`, `Encoder_CheckHealth()` | `Core/Src/motor_control.c` |
| Automatic steering centering (inductive sensor on PB5) | `SteeringCentering_Step()` | `Core/Src/steering_centering.c` |
| Ackermann geometry computation | `Ackermann_ComputeWheelAngles()` | `Core/Src/ackermann.c` |
| Safety state machine (BOOT→STANDBY→ACTIVE⇄DEGRADED→SAFE→ERROR) | `Safety_SetState()`, `Safety_GetState()` | `Core/Src/safety_system.c` |
| ABS (per-wheel pulse modulation, 80 ms cycle) | `ABS_Update()` | `Core/Src/safety_system.c` |
| TCS (per-wheel progressive reduction + recovery) | `TCS_Update()` | `Core/Src/safety_system.c` |
| Overcurrent protection (25 A threshold, consecutive-error escalation) | `Safety_CheckCurrent()` | `Core/Src/safety_system.c` |
| Overtemperature protection (80 °C warning, 90 °C critical) | `Safety_CheckTemperature()` | `Core/Src/safety_system.c` |
| Per-motor emergency temperature cutoff (130 °C per motor) | Logic in `Traction_Update()` | `Core/Src/motor_control.c` |
| Battery undervoltage (20 V warning, 18 V critical) | `Safety_CheckBatteryVoltage()` | `Core/Src/safety_system.c` |
| Sensor plausibility checks | `Safety_CheckSensors()` | `Core/Src/safety_system.c` |
| Encoder fault detection (range, jump, frozen) | `Encoder_CheckHealth()` | `Core/Src/motor_control.c` |
| CAN heartbeat timeout (250 ms) | `Safety_CheckCANTimeout()` | `Core/Src/safety_system.c` |
| CAN bus-off detection and recovery | `CAN_CheckBusOff()` | `Core/Src/can_handler.c` |
| Obstacle backstop limiter (3-tier distance mapping) | `Obstacle_Update()`, `Obstacle_ProcessCAN()` | `Core/Src/safety_system.c` |
| Relay power sequencing (Main → Traction → Direction) | `Relay_PowerUp()`, `Relay_SequencerUpdate()` | `Core/Src/safety_system.c` |
| Command validation gate (throttle, steering, mode) | `Safety_ValidateThrottle()`, `Safety_ValidateSteering()`, `Safety_ValidateModeChange()` | `Core/Src/safety_system.c` |
| Pedal ADC reading (ADC1, 12-bit, PA3) | `Pedal_Update()` | `Core/Src/sensor_manager.c` |
| 4× wheel speed (EXTI pulse counting with debounce) | `Wheel_FL_IRQHandler()` etc., `Wheel_ComputeSpeed()` | `Core/Src/sensor_manager.c` |
| 6× INA226 current/voltage via TCA9548A I2C mux | `Current_ReadAll()`, `Voltage_GetBus()` | `Core/Src/sensor_manager.c` |
| 5× DS18B20 temperature (OneWire bit-bang, ROM search) | `Temperature_ReadAll()`, `OW_SearchAll()` | `Core/Src/sensor_manager.c` |
| I2C bus recovery (NXP AN10216 SCL clock cycling) | `I2C_BusRecovery()` | `Core/Src/sensor_manager.c` |
| Boot validation checklist (6 pre-ACTIVE checks) | `BootValidation_Run()` | `Core/Src/boot_validation.c` |
| Service mode (module enable/disable/fault tracking) | `ServiceMode_Init()`, `ServiceMode_DisableModule()` | `Core/Src/service_mode.c` |
| Independent watchdog (IWDG, ~500 ms) | `MX_IWDG_Init()`, `HAL_IWDG_Refresh()` | `Core/Src/main.c` |
| Granular 3-level degradation (L1/L2/L3 with per-level scaling) | `Safety_SetDegradedLevel()` | `Core/Src/safety_system.c` |
| Dynamic braking (H-bridge active brake on throttle release) | Logic in `Traction_Update()` | `Core/Src/motor_control.c` |
| BTS7960 active brake (PWM=100 % hold when demand=0) | `BTS7960_BRAKE_PWM` in `Traction_Update()`, `Steering_ControlLoop()`, `Steering_Neutralize()` | `Core/Src/motor_control.c` |
| Park hold (active brake in gear P with current/temp derating) | Logic in `Traction_Update()` | `Core/Src/motor_control.c` |
| Gear system (P/R/N/D1/D2, speed-gated changes) | `Traction_SetGear()` | `Core/Src/motor_control.c` |
| Pedal EMA filter + ramp rate limiter | Logic in `Traction_SetDemand()` | `Core/Src/motor_control.c` |
| Demand anomaly detection (step-rate, frozen pedal) | Logic in `Traction_SetDemand()` | `Core/Src/motor_control.c` |
| NaN/Inf float sanitization | `sanitize_float()` | `Core/Src/motor_control.c`, `Core/Src/can_handler.c` |
| Ackermann differential torque correction | `compute_ackermann_differential()` | `Core/Src/motor_control.c` |
| Encoder read-only interface (hardware integrated, not used for control) | `Encoder_GetRawCount()`, `Encoder_GetDelta()`, `Encoder_SendDiagnostic()` | `Core/Src/encoder_reader.c` |

### ESP32-S3 Responsibilities

The ESP32 is the **HMI controller**. It receives telemetry from the STM32 over CAN and displays it on a 480×320 TFT. It sends commands (throttle, steering, mode/gear, service) to the STM32 for validation.

| Responsibility | Module / Class | File |
|---|---|---|
| CAN bus polling and frame decoding | `can_rx::poll()`, static decoders per CAN ID | `esp32/src/can_rx.cpp` |
| Vehicle data storage (passive container) | `VehicleData` class | `esp32/src/vehicle_data.cpp` |
| Screen state machine (BOOT/STANDBY/DRIVE/SAFE/ERROR) | `ScreenManager` class | `esp32/src/screen_manager.cpp` |
| Boot screen (title + CAN link status) | `BootScreen` | `esp32/src/screens/boot_screen.cpp` |
| Standby screen (temperatures + fault flags) | `StandbyScreen` | `esp32/src/screens/standby_screen.cpp` |
| Drive screen (speed, torque%, temps, steering, battery, pedal, gear) | `DriveScreen` | `esp32/src/screens/drive_screen.cpp` |
| Safe screen (fault display, actuators inhibited notice) | `SafeScreen` | `esp32/src/screens/safe_screen.cpp` |
| Error screen (unrecoverable error display) | `ErrorScreen` | `esp32/src/screens/error_screen.cpp` |
| Battery indicator (18 V–25.2 V → 0–100 %, color gradient) | `BatteryIndicator` | `esp32/src/ui/battery_indicator.cpp` |
| Pedal bar (0–100 % throttle visualization) | `PedalBar` | `esp32/src/ui/pedal_bar.cpp` |
| Gear display (P/R/N/D1/D2 selector) | `GearDisplay` | `esp32/src/ui/gear_display.cpp` |
| Mode icons (4×4, 4×2, 360° tank turn) | `ModeIcons` | `esp32/src/ui/mode_icons.cpp` |
| Car renderer (body outline + per-wheel torque/temp + steering gauge) | `CarRenderer` | `esp32/src/ui/car_renderer.cpp` |
| Obstacle sensor display (distance + proximity bar) | `ObstacleSensor` | `esp32/src/ui/obstacle_sensor.cpp` |
| Frame limiter (20 FPS rendering cap) | `FrameLimiter` | `esp32/src/ui/frame_limiter.h` |
| Runtime monitor (optional, frame timing + zone tracking) | `RuntimeMonitor` | `esp32/src/ui/runtime_monitor.cpp` |
| Debug overlay (long-press toggle, semi-transparent stats) | `DebugOverlay` | `esp32/src/ui/debug_overlay.cpp` |
| ESP32 heartbeat transmission (0x011 every 100 ms) | Logic in `loop()` | `esp32/src/main.cpp` |
| Command ACK tracking (non-blocking, 200 ms timeout) | `ackBeginWait()`, `ackCheck()` | `esp32/src/main.cpp` |

### CAN Communication

- **Physical layer:** FDCAN1 on STM32 (PB8/PB9), ESP32-TWAI (GPIO4/GPIO5), 500 kbps Classic CAN
- **Protocol:** Standard 11-bit IDs, 8-byte max payload
- **RX filtering:** STM32 accepts only known ESP32 IDs (0x011, 0x100–0x102, 0x110, 0x208–0x209) via FDCAN hardware filters; all other IDs rejected

**Data flows:**

| Direction | CAN ID | Rate | Content |
|---|---|---|---|
| STM32 → ESP32 | 0x001 | 100 ms | Heartbeat: alive counter, system state, fault flags, error code |
| STM32 → ESP32 | 0x200 | 100 ms | 4× wheel speeds (0.1 km/h units) |
| STM32 → ESP32 | 0x201 | 100 ms | 4× motor currents (0.01 A units) |
| STM32 → ESP32 | 0x202 | 1000 ms | 5× temperatures (°C, signed) |
| STM32 → ESP32 | 0x203 | 100 ms | ABS active, TCS active, error code |
| STM32 → ESP32 | 0x204 | 100 ms | Steering angle (0.1° units), calibrated flag |
| STM32 → ESP32 | 0x205 | 100 ms | 4× per-wheel traction scale (0–100 %) |
| STM32 → ESP32 | 0x206 | 1000 ms | 5× mapped temperatures (FL/FR/RL/RR/AMB) |
| STM32 → ESP32 | 0x207 | 100 ms | Battery current + voltage (0.01 units) |
| STM32 → ESP32 | 0x103 | On-demand | Command ACK (cmd ID, result, state) |
| STM32 → ESP32 | 0x300 | On-demand | Diagnostic error (code + subsystem) |
| STM32 → ESP32 | 0x301–0x303 | 1000 ms | Service mode bitmasks (fault/enabled/disabled) |
| ESP32 → STM32 | 0x011 | 100 ms | ESP32 heartbeat |
| ESP32 → STM32 | 0x100 | On-demand | Throttle command |
| ESP32 → STM32 | 0x101 | On-demand | Steering command |
| ESP32 → STM32 | 0x102 | On-demand | Mode/gear command |
| ESP32 → STM32 | 0x110 | On-demand | Service command |
| ESP32 → STM32 | 0x208 | ~66 ms | Obstacle distance (mm, zone, health, counter) |
| ESP32 → STM32 | 0x209 | Reserved | Obstacle safety state (accepted but not parsed) |

### Safety Architecture

The STM32 is the sole safety authority. All incoming ESP32 commands pass through `Safety_Validate*()` gates. The state machine enforces which transitions are legal. The independent watchdog (IWDG, ~500 ms) resets the MCU if the main loop stalls. CAN timeout (250 ms) transitions to SAFE. Bus-off detection and recovery is non-blocking with retry limits. Obstacle backstop limiter runs independently of ESP32 logic.

### UI Architecture

All rendering uses partial-redraw: each UI component compares current vs. previous frame values and only redraws changed elements. TFT_eSPI drives an ST7796 480×320 display via SPI at 40 MHz. Frame rate is capped at 20 FPS by `FrameLimiter`. No heap allocation, no `String` class, all fixed-size stack buffers.

---

## 2) COMPLETED FEATURES (SOURCE OF TRUTH)

### Steering

| Feature | Evidence | Files |
|---|---|---|
| PID steering control (P-only, kp=0.09 in count-space) | `Steering_ControlLoop()` with `PID_Compute()` | `Core/Src/motor_control.c` |
| Ackermann geometry (wheel angle computation) | `Ackermann_ComputeWheelAngles()` | `Core/Src/ackermann.c` |
| Ackermann differential torque correction (±15 % max) | `compute_ackermann_differential()` in `Traction_Update()` | `Core/Src/motor_control.c` |
| Automatic steering centering (sweep left/right, inductive sensor) | `SteeringCentering_Step()` state machine | `Core/Src/steering_centering.c` |
| Centering fault handling (abort → DEGRADED L1) | `Centering_Abort()` → `Safety_SetState(SYS_STATE_DEGRADED)` | `Core/Src/steering_centering.c` |
| Steering encoder quadrature decoding (TIM2, 32-bit, 4800 CPR) | `MX_TIM2_Init()` encoder mode | `Core/Src/main.c` |
| Steering deadband (0.5°) | `STEERING_DEADBAND_COUNTS` in `Steering_ControlLoop()` | `Core/Src/motor_control.c` |
| Steering rate limiting (200 °/s max) | `Safety_ValidateSteering()` | `Core/Src/safety_system.c` |
| Steering neutralization (BTS7960 active brake hold) | `Steering_Neutralize()` | `Core/Src/motor_control.c` |
| Steering display (circular gauge on drive screen) | `CarRenderer::drawSteeringGauge()` | `esp32/src/ui/car_renderer.cpp` |
| Degraded-mode steering assist reduction (L1=85%, L2=70%, L3=60%) | `Safety_GetSteeringLimitFactor()` applied in `Steering_ControlLoop()` | `Core/Src/motor_control.c`, `Core/Src/safety_system.c` |

### Motor Control

| Feature | Evidence | Files |
|---|---|---|
| 4× traction motor PWM (TIM1 CH1-4, 20 kHz, 8500 steps) | `Motor_Init()`, `Motor_SetPWM()` | `Core/Src/motor_control.c` |
| 4×4 mode (50/50 axle torque split) | `Traction_Update()` mode4x4 branch | `Core/Src/motor_control.c` |
| 4×2 mode (front wheels only, rear active brake) | `Traction_Update()` else branch, rear `BTS7960_BRAKE_PWM` | `Core/Src/motor_control.c` |
| Tank turn (left reverse, right forward) | `Traction_Update()` axisRotation branch | `Core/Src/motor_control.c` |
| Gear system: P (active hold brake), R, N (coast), D1 (60%), D2 (100%) | `Traction_SetGear()`, `Traction_Update()` gear branches | `Core/Src/motor_control.c` |
| Park hold with current/temp derating | `PARK_HOLD_*` constants, logic in `Traction_Update()` | `Core/Src/motor_control.c` |
| Dynamic braking (proportional to throttle decrease rate) | `DYNBRAKE_*` constants, logic in `Traction_Update()` | `Core/Src/motor_control.c` |
| BTS7960 active brake on zero demand (PWM=100 %, EN=HIGH) | `BTS7960_BRAKE_PWM`, logic in `Traction_Update()` | `Core/Src/motor_control.c` |
| Pedal EMA noise filter (alpha=0.15) | `PEDAL_EMA_ALPHA`, logic in `Traction_SetDemand()` | `Core/Src/motor_control.c` |
| Pedal ramp rate limiter (50 %/s up, 100 %/s down) | `PEDAL_RAMP_*_PCT_S`, logic in `Traction_SetDemand()` | `Core/Src/motor_control.c` |
| Demand anomaly detection (step-rate, frozen pedal) | `MAX_THROTTLE_STEP_PER_10MS`, frozen pedal logic | `Core/Src/motor_control.c` |
| Per-motor emergency temperature cutoff (130 °C, 15 °C hysteresis) | `MOTOR_TEMP_CUTOFF_C`, logic in `Traction_Update()` | `Core/Src/motor_control.c` |
| NaN/Inf float sanitization on all PWM-affecting inputs | `sanitize_float()` calls | `Core/Src/motor_control.c` |
| Emergency stop (all motors off, relays off) | `Traction_EmergencyStop()` | `Core/Src/motor_control.c` |

### Safety

| Feature | Evidence | Files |
|---|---|---|
| State machine: BOOT→STANDBY→ACTIVE⇄DEGRADED→SAFE→ERROR | `Safety_SetState()` with transition guards | `Core/Src/safety_system.c` |
| 3-level degradation (L1=70/85/80%, L2=50/70/60%, L3=40/60/50%) | `DEGRADED_L*_*_PCT`, `Safety_SetDegradedLevel()` | `Core/Inc/safety_system.h`, `Core/Src/safety_system.c` |
| Degraded-mode recovery debounce (500 ms clean hold) | `RECOVERY_HOLD_MS`, logic in `Safety_CheckCANTimeout()` | `Core/Src/safety_system.c` |
| ABS per-wheel pulse modulation (30% reduction, 80 ms cycle, 60/40 duty) | `ABS_Update()` with `abs_pulse_timer[]`, `abs_pulse_phase[]` | `Core/Src/safety_system.c` |
| TCS per-wheel progressive reduction (40% initial, +5%/cycle, 80% max, 25%/s recovery) | `TCS_Update()` with `tcs_reduction[]` | `Core/Src/safety_system.c` |
| Overcurrent protection (25 A, consecutive-error escalation to SAFE after 3) | `Safety_CheckCurrent()` | `Core/Src/safety_system.c` |
| Overtemperature (80 °C warning → DEGRADED, 90 °C critical → SAFE, 5 °C hysteresis) | `Safety_CheckTemperature()` | `Core/Src/safety_system.c` |
| Battery undervoltage (20 V warning → DEGRADED L2, 18 V critical → SAFE, 0.5 V hysteresis) | `Safety_CheckBatteryVoltage()` | `Core/Src/safety_system.c` |
| CAN heartbeat timeout (250 ms → SAFE, auto-recovery on restore) | `Safety_CheckCANTimeout()` | `Core/Src/safety_system.c` |
| CAN bus-off detection and recovery (non-blocking, retry-limited) | `CAN_CheckBusOff()` | `Core/Src/can_handler.c` |
| Sensor plausibility (temperature range, current range, wheel speed range) | `Safety_CheckSensors()` | `Core/Src/safety_system.c` |
| Encoder health (range, jump, frozen-value detection) | `Encoder_CheckHealth()` | `Core/Src/motor_control.c` |
| Obstacle backstop limiter (3-tier: <200mm→SAFE, 200–500→30%, 500–1000→70%) | `Obstacle_Update()` | `Core/Src/safety_system.c` |
| Obstacle stale-data detection (rolling counter, 3-frame freeze) | `Obstacle_ProcessCAN()` stale counter logic | `Core/Src/safety_system.c` |
| Obstacle recovery hysteresis (>500 mm for >1 s) | Recovery logic in `Obstacle_Update()` | `Core/Src/safety_system.c` |
| Non-blocking relay power sequencing (Main 50 ms → Traction 20 ms → Direction) | `Relay_SequencerUpdate()` state machine | `Core/Src/safety_system.c` |
| Command validation gate (throttle clamp, steering rate limit, mode speed gate) | `Safety_ValidateThrottle()`, `Safety_ValidateSteering()`, `Safety_ValidateModeChange()` | `Core/Src/safety_system.c` |
| Fail-safe action (emergency stop + center steering if encoder healthy) | `Safety_FailSafe()` | `Core/Src/safety_system.c` |
| Independent watchdog (~500 ms) | `MX_IWDG_Init()`, `HAL_IWDG_Refresh()` in main loop | `Core/Src/main.c` |
| `Error_Handler()` drives all GPIOC outputs LOW via direct register | `Error_Handler()` | `Core/Src/main.c` |

### CAN Communication

| Feature | Evidence | Files |
|---|---|---|
| FDCAN RX hardware filters (4 filter banks, reject-all default) | `CAN_ConfigureFilters()` | `Core/Src/can_handler.c` |
| Heartbeat TX (0x001, alive counter + state + fault flags + error code) | `CAN_SendHeartbeat()` | `Core/Src/can_handler.c` |
| Status TX: speed, current, temp, safety, steering, traction, temp map, battery | `CAN_SendStatus*()` family | `Core/Src/can_handler.c` |
| Command ACK TX (0x103, result + system state) | `CAN_SendCommandAck()` | `Core/Src/can_handler.c` |
| Service status TX (fault/enabled/disabled bitmasks, 0x301–0x303) | `CAN_SendServiceStatus()` | `Core/Src/can_handler.c` |
| Encoder diagnostic TX (0x300, tag 0x10, raw count + delta, 1 Hz) | `CAN_SendDiagnosticEncoder()` | `Core/Src/can_handler.c` |
| RX message processing (throttle, steering, mode/gear, service, obstacle) | `CAN_ProcessMessages()` switch-case | `Core/Src/can_handler.c` |
| Bus-off statistics tracking | `can_stats.busoff_count` | `Core/Src/can_handler.c` |
| ESP32 CAN RX decoding (all STM32 status IDs) | `can_rx::poll()` | `esp32/src/can_rx.cpp` |
| ESP32 heartbeat TX (0x011, every 100 ms) | Logic in `loop()` | `esp32/src/main.cpp` |

### Display / UI

| Feature | Evidence | Files |
|---|---|---|
| 5-screen state machine (Boot/Standby/Drive/Safe/Error) | `ScreenManager::update()` | `esp32/src/screen_manager.cpp` |
| Drive screen with speed, torque%, temps, steering, battery, pedal, gear | `DriveScreen::draw()` | `esp32/src/screens/drive_screen.cpp` |
| Partial-redraw architecture (prev vs. cur comparison, no full-screen clear) | All `draw()` methods compare `prev_*` values | `esp32/src/screens/*.cpp`, `esp32/src/ui/*.cpp` |
| Battery indicator (18 V → 0 %, 25.2 V → 100 %, color gradient) | `BatteryIndicator::draw()` | `esp32/src/ui/battery_indicator.cpp` |
| Car renderer (body outline, per-wheel torque/temp, steering gauge) | `CarRenderer::draw()` | `esp32/src/ui/car_renderer.cpp` |
| Obstacle distance display (X.XX m + proximity bar + color coding) | `ObstacleSensor::draw()` | `esp32/src/ui/obstacle_sensor.cpp` |
| Gear selector (P/R/N/D1/D2 with active highlight) | `GearDisplay::draw()` | `esp32/src/ui/gear_display.cpp` |
| Mode icons (4×4, 4×2, 360° with active/inactive state) | `ModeIcons::draw()` | `esp32/src/ui/mode_icons.cpp` |
| Pedal bar (0–100 % fill + color gradient) | `PedalBar::draw()` | `esp32/src/ui/pedal_bar.cpp` |
| Frame limiter (20 FPS cap) | `FrameLimiter::shouldDraw()` | `esp32/src/ui/frame_limiter.h` |
| Debug overlay (long-press toggle, semi-transparent stats) | `DebugOverlay` | `esp32/src/ui/debug_overlay.cpp` |

### Sensors

| Feature | Evidence | Files |
|---|---|---|
| 4× wheel speed via EXTI pulse counting + software debounce (1 ms) | `Wheel_IRQDebounced()`, `Wheel_ComputeSpeed()` | `Core/Src/sensor_manager.c` |
| Steering center inductive sensor (PB5/EXTI5) | `SteeringCenter_IRQHandler()`, `SteeringCenter_Detected()` | `Core/Src/sensor_manager.c` |
| Pedal ADC (12-bit, single conversion, PA3/ADC1_IN4) | `Pedal_Update()` | `Core/Src/sensor_manager.c` |
| 6× INA226 current/voltage via TCA9548A I2C multiplexer | `Current_ReadAll()`, `TCA9548A_SelectChannel()`, `INA226_ReadReg()` | `Core/Src/sensor_manager.c` |
| Per-channel shunt resistance (1 mΩ motor, 0.5 mΩ battery) | `INA226_SHUNT_MOHM_*` constants in channel selection logic | `Core/Src/sensor_manager.c` |
| 5× DS18B20 temperature via OneWire (bit-bang, ROM search, CRC-8) | `OW_SearchAll()`, `OW_ReadTemperature()`, `Temperature_ReadAll()` | `Core/Src/sensor_manager.c` |
| I2C bus recovery (NXP AN10216, SCL cycling, 16 toggles) | `I2C_BusRecovery()` | `Core/Src/sensor_manager.c` |
| I2C failure escalation (3 fails → recovery, 2 recoveries → SAFE) | `i2c_fail_count`, `i2c_recovery_attempts` logic | `Core/Src/sensor_manager.c` |
| E6B2-CWZ6C encoder read-only interface (hardware integrated, not used for control) | `Encoder_GetRawCount()`, `Encoder_GetDelta()`, `Encoder_Reset()` | `Core/Src/encoder_reader.c` |
| Encoder raw count CAN diagnostic (0x300, 1 Hz, tag 0x10) | `Encoder_SendDiagnostic()` → `CAN_SendDiagnosticEncoder()` | `Core/Src/encoder_reader.c`, `Core/Src/can_handler.c` |

### Audio

NOT IMPLEMENTED — no audio hardware or software exists in the codebase.

### Lighting

NOT IMPLEMENTED — no lighting hardware or software exists in the codebase.

### Service Mode

| Feature | Evidence | Files |
|---|---|---|
| Module registry (24 modules, CRITICAL vs. NON-CRITICAL classification) | `module_class[]` array | `Core/Src/service_mode.c` |
| Critical module protection (CAN timeout, emergency stop, watchdog, relay cannot be disabled) | `ServiceMode_DisableModule()` rejects CRITICAL | `Core/Src/service_mode.c` |
| Per-module enable/disable | `ServiceMode_EnableModule()`, `ServiceMode_DisableModule()` | `Core/Src/service_mode.c` |
| Per-module fault tracking (NONE/WARNING/ERROR/DISABLED) | `ServiceMode_SetFault()`, `ServiceMode_GetFault()` | `Core/Src/service_mode.c` |
| Factory restore (re-enable all, clear manual-disable faults) | `ServiceMode_FactoryRestore()` | `Core/Src/service_mode.c` |
| CAN bitmask accessors (enabled/disabled/fault masks for ESP32) | `ServiceMode_GetEnabledMask()`, `ServiceMode_GetFaultMask()`, `ServiceMode_GetDisabledMask()` | `Core/Src/service_mode.c` |
| Boot validation checklist (6 checks: temp, current, encoder, battery, safety error, CAN) | `BootValidation_Run()` | `Core/Src/boot_validation.c` |

### Runtime Monitoring (ESP32)

| Feature | Evidence | Files |
|---|---|---|
| Frame timing ring buffer (120 samples, moving average, min/max) | `RuntimeMonitor` static methods | `esp32/src/ui/runtime_monitor.cpp` |
| Per-phase timing (CAN RX, UI update, render) with >4 ms flagging | Phase timing logic | `esp32/src/ui/runtime_monitor.cpp` |
| Per-zone redraw counting | Zone tracking logic | `esp32/src/ui/runtime_monitor.cpp` |
| Serial logging (every 5 s, compact format) | Logging logic | `esp32/src/ui/runtime_monitor.cpp` |

---

## 3) KNOWN LIMITATIONS

| Limitation | Evidence | File |
|---|---|---|
| **Steering PID is P-only** (kp=0.09, ki=0.0, kd=0.0) — no integral or derivative terms | `steering_pid = {0.09f, 0.0f, 0.0f, ...}` | `Core/Src/motor_control.c` line 199 |
| **Pedal is single-channel ADC** — no redundant sensor or cross-check | Only `hadc1` on `ADC_CHANNEL_4` | `Core/Src/main.c`, `Core/Src/sensor_manager.c` |
| **OneWire bit-bang timing is approximate** — busy-wait loop calibrated for 170 MHz | `OW_DelayUs()` uses NOP loop: `us * 42` | `Core/Src/sensor_manager.c` line 341 |
| **DS18B20 ROM search runs only once at init** — hot-plug sensors not detected | `OW_SearchAll()` called only from `Sensor_Init()` | `Core/Src/sensor_manager.c` line 619 |
| **Fallback single-sensor read when no ROMs discovered** — `Temperature_ReadAll()` reads only `temperatures[0]` via Skip ROM | Fallback branch in `Temperature_ReadAll()` | `Core/Src/sensor_manager.c` lines 561–573 |
| **Drive screen gear display is not CAN-driven** — mode flags (4×4/360°) in DriveScreen are not populated from CAN data | Gear and mode flags set locally, not from `VehicleData` CAN fields | `esp32/src/screens/drive_screen.cpp` |
| **Hardcoded vehicle physics constants** — wheelbase (0.95 m), track width (0.70 m), wheel circumference (1.1 m), max steer (54°) are compile-time `#define` | `vehicle_physics.h` | `Core/Inc/vehicle_physics.h` |
| **Hardcoded INA226 shunt resistances** — 1 mΩ motor, 0.5 mΩ battery are compile-time constants | `INA226_SHUNT_MOHM_*` | `Core/Inc/main.h` |
| **No calibration persistence** — steering centering, encoder zero, sensor offsets are not saved to flash/EEPROM | No flash write or EEPROM API calls anywhere | All STM32 source files |
| **CAN bus-off recovery limited to 5 retries then stops** — `CAN_BUSOFF_MAX_RETRIES` | `busoff_retry_count >= CAN_BUSOFF_MAX_RETRIES` in `CAN_CheckBusOff()` | `Core/Src/can_handler.c` |
| **Obstacle message 0x209 accepted but not parsed** — reserved for future ESP32↔STM32 coordination | `case CAN_ID_OBSTACLE_SAFETY: break;` | `Core/Src/can_handler.c` line 586 |
| **Encoder Z-index pulse not used** — PB4/EXTI4 not initialized; only A/B quadrature channels used | Comment block + no EXTI4 in `MX_GPIO_Init()` | `Core/Src/motor_control.c` lines 236–245, `Core/Src/main.c` |
| **No sensor fusion** — wheel speed, current, and temperature data are used independently | No cross-sensor correlation or Kalman filtering | All STM32 source files |
| **ADC pedal conversion is blocking** — `HAL_ADC_PollForConversion()` with 10 ms timeout | `Pedal_Update()` blocking poll | `Core/Src/sensor_manager.c` line 118 |
| **Float-only arithmetic** — no fixed-point fallback; dependent on Cortex-M4 FPU | All computations use `float` | All STM32 source files |
| **Emergency stop is non-recoverable** — `emergency_stopped` flag set, system enters ERROR | `Safety_EmergencyStop()` sets `SYS_STATE_ERROR` | `Core/Src/safety_system.c` |
| **I2C bus recovery uses busy-wait delays** — ~160 µs total, but blocking | NOP loops in `I2C_BusRecovery()` | `Core/Src/sensor_manager.c` lines 177–203 |
| **ESP32 obstacle source not yet implemented** — STM32 accepts obstacle data (0x208) but ESP32 source code does not contain obstacle sensor driver or ultrasonic sensor reading | No obstacle sensor reading code in `esp32/src/` | `esp32/src/` directory |
| **Service mode state is RAM-only** — module enable/disable settings lost on power cycle | `module_enabled[]` is static array, no NVM storage | `Core/Src/service_mode.c` |
| **Encoder reader module is observation-only** — `encoder_reader.c` provides raw count access and CAN diagnostics but is not connected to any control, odometry, speed, traction, braking, or steering logic | `Encoder_GetRawCount()`, `Encoder_GetDelta()`, `Encoder_SendDiagnostic()` | `Core/Src/encoder_reader.c` |

---

## 4) PENDING FEATURES (ENGINEERING BACKLOG)

| # | Description | Subsystem | Dependencies | Blocking Modules |
|---|---|---|---|---|
| 1 | **ESP32 obstacle sensor driver** — STM32 accepts obstacle CAN data (0x208) and processes it, but ESP32 has no ultrasonic/ToF sensor reading code to generate this data | Sensors / Safety | Ultrasonic or ToF sensor hardware, ESP32 GPIO/I2C driver | `Obstacle_ProcessCAN()`, `Obstacle_Update()` in `safety_system.c` |
| 2 | **CAN ID 0x209 parsing** — filter and case exist for `CAN_ID_OBSTACLE_SAFETY` but body is empty (`break;` only) | CAN / Safety | Obstacle sensor driver (#1) | `CAN_ProcessMessages()` in `can_handler.c` |
| 3 | **Steering PID tuning (I and D terms)** — PID structure supports ki/kd but both are 0.0; code path exists in `PID_Compute()` | Steering | Hardware testing with actual steering load | `steering_pid` in `motor_control.c` |
| 4 | **Calibration persistence** — encoder zero and sensor offsets recomputed every power cycle; no NVM/flash write path exists | Steering / Sensors | STM32 flash or EEPROM driver | `Steering_Init()`, `SteeringCentering_Complete()` |
| 5 | **Service mode persistence** — module enable/disable states lost on reboot; no NVM path exists | Service Mode | STM32 flash or EEPROM driver | `ServiceMode_Init()` |
| 6 | **Redundant pedal sensor** — single ADC channel with no cross-check; code structure accepts only one value | Sensors / Safety | Second ADC channel or hall sensor hardware | `Pedal_Update()` in `sensor_manager.c` |
| 7 | **ESP32 mode/gear CAN feedback to DriveScreen** — DriveScreen does not read mode/gear from `VehicleData`; UI state is locally tracked | Display / UI | CAN data population in `can_rx.cpp` for mode/gear state | `DriveScreen` in `drive_screen.cpp` |
| 8 | **Hot-plug DS18B20 detection** — ROM search runs only at init; adding/removing sensors at runtime is not detected | Sensors | Periodic `OW_SearchAll()` or change-detection mechanism | `Sensor_Init()` in `sensor_manager.c` |

---

## 5) IMPLEMENTATION PHASES (ORDERED — MANDATORY)

### PHASE 1 — Stability Foundation

**Goals:**
- Validate all existing safety checks under real hardware conditions
- Confirm CAN communication reliability between STM32 and ESP32
- Confirm steering centering completes reliably on hardware
- Confirm boot validation checklist passes on all sensor configurations
- Confirm watchdog timing is appropriate for actual loop execution time

**What must NOT be touched yet:**
- PID tuning parameters (kp, ki, kd)
- ABS/TCS thresholds and timing
- Obstacle distance thresholds
- Any UI layout or rendering logic
- Gear power scaling percentages

**Exit criteria:**
- STM32 transitions BOOT → STANDBY → ACTIVE on real hardware with all 6 boot validation checks passing
- CAN heartbeat loss correctly triggers SAFE within 250 ms
- Steering centering completes within 10 s timeout on physical steering rack
- IWDG does not trigger during normal operation (all loop tiers complete within timing budgets)
- I2C bus recovery successfully restores communication after injected SDA hold-low

---

### PHASE 2 — Control Reliability

**Goals:**
- Validate traction control pipeline end-to-end (pedal ADC → EMA → ramp → demand → PWM)
- Validate ABS pulse modulation produces measurable wheel speed recovery under simulated slip
- Validate TCS progressive reduction prevents wheel spin under simulated low-grip
- Validate dynamic braking activates proportionally on throttle release
- Validate park hold brake maintains vehicle stationary with current/temp derating
- Validate gear changes are correctly speed-gated

**What must NOT be touched yet:**
- Ackermann differential torque percentages
- Per-motor emergency temperature cutoff thresholds (130 °C / 115 °C)
- Battery undervoltage thresholds
- ESP32 UI layout or rendering
- Service mode module classification

**Exit criteria:**
- Pedal ramp limiter measurably constrains acceleration to ≤50 %/s
- ABS reduces per-wheel PWM by 30 % during ON phase and restores to 100 % during OFF phase
- TCS reduces per-wheel PWM progressively and recovers at 25 %/s when slip clears
- Dynamic braking produces measurable opposing torque when throttle is released at speed
- Gear change from D1 to R is rejected when average wheel speed > 1 km/h

---

### PHASE 3 — Feedback & Sensors

**Goals:**
- Add ESP32 obstacle sensor driver to populate CAN ID 0x208
- Implement CAN ID 0x209 parsing on STM32
- Evaluate and implement redundant pedal sensor path
- Evaluate PID I-term and D-term for steering
- Add periodic DS18B20 ROM search for hot-plug detection

**What must NOT be touched yet:**
- ESP32 UI screen layout
- Service mode module classification or factory restore logic
- Safety state machine transitions
- CAN heartbeat format or timing

**Exit criteria:**
- ESP32 sends valid obstacle distance frames (0x208) with incrementing rolling counter
- STM32 obstacle backstop limiter correctly applies 3-tier torque scaling from live sensor data
- Steering PID overshoot is ≤ 5 % of setpoint under step input (if I/D terms added)
- DS18B20 hot-plug: adding a sensor mid-operation is detected within 10 s

---

### PHASE 4 — Driver Interaction

**Goals:**
- Wire ESP32 CAN mode/gear state into DriveScreen UI
- Add calibration persistence (steering encoder zero, sensor offsets) to STM32 flash
- Add service mode persistence to STM32 flash
- Improve ESP32 ACK feedback to driver for mode/gear changes

**What must NOT be touched yet:**
- Safety state machine transitions or thresholds
- ABS/TCS algorithms
- CAN protocol message formats (add new IDs only, do not modify existing)

**Exit criteria:**
- DriveScreen shows current gear (P/R/N/D1/D2) received from STM32 via CAN
- Steering encoder zero is preserved across power cycles (verified by skipping centering when already calibrated)
- Service mode enable/disable settings survive reboot
- Mode change ACK is visually indicated on DriveScreen within 200 ms

---

### PHASE 5 — Experience Features

**Goals:**
- Implement audio feedback (if hardware added)
- Implement lighting control (if hardware added)
- Add sensor fusion (e.g., wheel speed + current for improved traction estimation)
- Optimize OneWire timing (DMA-based UART or hardware timer)
- Convert blocking ADC pedal read to DMA or interrupt-driven

**What must NOT be touched yet:**
- Core safety state machine
- CAN protocol backwards compatibility

**Exit criteria:**
- Audio/lighting subsystems (if added) have corresponding service mode modules and CAN telemetry
- Sensor fusion produces demonstrably more accurate speed or grip estimation than individual sensors
- ADC pedal read is non-blocking (DMA or interrupt, no `HAL_ADC_PollForConversion`)

---

## 6) RULES FOR CONTRIBUTORS (MANDATORY)

### A PR is invalid if:

1. **It adds behavior without updating this document** — any new function, module, CAN message, safety check, UI element, or sensor reading must be reflected in sections 1 and 2.

2. **It changes architecture without reflecting it here** — any modification to the state machine, CAN protocol, data flow, module responsibilities, or safety thresholds must update section 1 and, if applicable, section 3.

3. **It implements a feature from a future phase** — Phase N+1 work is blocked until all Phase N exit criteria are objectively met and documented. Phase ordering in section 5 is the official execution sequence.

4. **It removes a safety check without safety justification** — no safety-related code (anything in `safety_system.c`, `boot_validation.c`, `Safety_*` functions, `Encoder_CheckHealth`, watchdog, relay sequencing, CAN timeout) may be removed or weakened without explicit technical justification in the PR description and a corresponding update to section 3 (Known Limitations).

5. **It introduces hardcoded constants without documenting them** — any new `#define` or magic number that affects actuator behavior, safety thresholds, or timing must be listed with its rationale, either in code comments or in section 3.

6. **It modifies CAN protocol without updating section 1** — adding, removing, or changing CAN message IDs, payload formats, or timing must be reflected in the CAN data flow table.

---

## 7) DOCUMENT MAINTENANCE PROTOCOL (MANDATORY)

This section defines the enforcement rules that make `docs/PROJECT_MASTER_STATUS.md` a mandatory update gate for every Pull Request in this repository.

### 7.1 — Evaluation requirement

Every PR **MUST** evaluate whether it changes any of the following:

- **Architecture** (section 1) — module responsibilities, data flows, peripheral assignments, CAN topology
- **Completed Features** (section 2) — new executable logic, new subsystem entries, modified behavior
- **Known Limitations** (section 3) — new hardcoded constants, new placeholder logic, removed workarounds
- **Pending Features** (section 4) — items resolved, new backlog entries implied by code changes
- **Phase status** (section 5) — exit criteria met, phase transitions, scope changes

### 7.2 — Mandatory update rule

If any of the categories listed in 7.1 changed as a result of the PR, the PR **MUST** update this document in the same commit set. The update must be included before the PR is marked as ready for review.

### 7.3 — Behavioral changes require document updates

A PR that introduces new behavior (new functions, new modules, new CAN messages, new safety checks, new UI elements, new sensor readings, modified thresholds, or changed state transitions) but does **not** update this document is **invalid** and must be rejected by reviewers.

### 7.4 — Phase ordering enforcement

A PR **cannot** implement a feature from a future phase unless the phase order itself is explicitly modified and justified in the PR description. Phase ordering in section 5 is the official execution sequence. Skipping phases requires updating section 5 with the rationale.

### 7.5 — Reality over plans

This document always reflects **REALITY** — what is provably implemented in code — never plans, intentions, or aspirations. If a feature is listed in section 2 (Completed Features), executable logic for it **must** exist in the repository. If it does not, the entry must be moved to section 4 (Pending Features) or removed.

### 7.6 — Refactor exemption with explicit declaration

Refactors that do **not** change observable behavior (e.g., code style, internal renaming, build system cleanup, comment updates) are exempt from updating this document. However, the PR description **must** explicitly state:

> "No PROJECT_MASTER_STATUS.md changes required — this PR does not alter architecture, features, limitations, backlog, or phase status."

If this declaration is absent from a refactor PR, reviewers must request it before approving.

### 7.7 — Reviewer enforcement obligation

Reviewers **must** reject a PR if this protocol is not followed. Specifically, reviewers must verify:

1. The PR author evaluated whether sections 1–5 are affected.
2. If affected, the corresponding sections have been updated in the same commit set.
3. If not affected, the PR description contains the explicit exemption declaration from rule 7.6.
4. No section 2 entry exists without corresponding executable code in the repository.
5. No phase skip occurred without justification in section 5.
