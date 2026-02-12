# FIRMWARE MATURITY ROADMAP

**Comprehensive Firmware Comparison & Development Roadmap**

| Field | Value |
|-------|-------|
| **Document Version** | 1.0 |
| **Date** | 2026-02-12 |
| **Current Repository** | STM32-Control-Coche-Marcos (STM32G474RE + ESP32-S3 HMI) |
| **Reference Repository** | FULL-FIRMWARE-Coche-Marcos (ESP32-S3 monolithic, v2.17.1 PHASE 14) |
| **Scope** | Analysis & Documentation only — no code changes |

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Global Architecture Comparison](#2-global-architecture-comparison)
3. [Feature-by-Feature Master Checklist](#3-feature-by-feature-master-checklist)
4. [Safety Gap Analysis](#4-safety-gap-analysis)
5. [Phased Development Roadmap](#5-phased-development-roadmap)
6. [Architecture Preservation Check](#6-architecture-preservation-check)
7. [Risk Classification](#7-risk-classification)
8. [Final Verdict](#8-final-verdict)

---

## 1. Executive Summary

The **STM32-Control-Coche-Marcos** repository implements the safety-critical motor control layer of a 4-wheel electric vehicle, running on an STM32G474RE microcontroller at 170 MHz. It operates as the **safety authority** in a split architecture where the ESP32-S3 serves exclusively as the HMI (Human-Machine Interface), communicating over CAN bus at 500 kbps.

The **FULL-FIRMWARE-Coche-Marcos** reference repository is a monolithic ESP32-S3-based firmware (v2.17.1, PHASE 14) that integrates all subsystems — motor control, safety, sensors, HMI, audio, lighting, obstacle detection, and AI-based regenerative braking — into a single microcontroller.

### Key Findings

- **Core motor control, safety state machine, ABS/TCS, CAN protocol, and sensor management** are fully implemented and functional in the current STM32 firmware.
- **Regenerative braking, obstacle detection/avoidance, AI regen logic, advanced limp mode, power management sequencing, and HMI graphics** are not yet implemented — these existed only in the monolithic ESP32 reference firmware.
- **The split architecture (STM32 + ESP32) is fundamentally sound** and superior to the monolithic approach for safety-critical applications. The STM32G474RE provides deterministic real-time control that the ESP32 cannot guarantee.
- **No core design flaws were detected.** The current architecture is scalable and supports incremental feature addition without refactoring.
- **Overall maturity: STABLE** — the firmware provides a safe, functional vehicle control layer with room for feature expansion.

---

## 2. Global Architecture Comparison

### 2.1 Architecture Overview

| Aspect | Current (STM32 Split) | Reference (ESP32 Monolithic) |
|--------|----------------------|------------------------------|
| **MCU** | STM32G474RE (170 MHz Cortex-M4F) | ESP32-S3 N16R8 (240 MHz dual-core) |
| **Architecture** | Split: STM32 (control) + ESP32 (HMI) | Monolithic: single ESP32 handles everything |
| **RTOS** | Bare-metal super-loop (multi-rate) | FreeRTOS with multiple tasks |
| **Communication** | CAN 500 kbps (frozen contract v1.0) | Internal function calls |
| **Safety Model** | Hardware-enforced separation | Software-only separation |
| **Determinism** | Guaranteed (bare-metal, no preemption) | Best-effort (FreeRTOS scheduling) |
| **PWM Generation** | Hardware TIM1/TIM8 @ 20 kHz | PCA9685 I2C external PWM (later migrated) |
| **Watchdog** | IWDG 500 ms (hardware) | Software watchdog |

### 2.2 Subsystem Comparison Table

| Subsystem | Status | Notes |
|-----------|--------|-------|
| **System State Machine** | FULL | BOOT → STANDBY → ACTIVE ⇄ DEGRADED → SAFE → ERROR. Complete with fault escalation and transition rules. |
| **Safety System** | FULL | Multi-layer protection: overcurrent, overtemperature, CAN timeout, encoder fault, relay sequencing. Consecutive error counting with escalation thresholds. |
| **ABS (Anti-Lock Braking)** | FULL | Per-wheel slip detection (15% threshold), per-wheel scale modulation, global fallback for all-wheel lockup, minimum speed gate (10 km/h). |
| **TCS (Traction Control)** | FULL | Per-wheel slip detection (15% threshold), progressive reduction (40% initial → 80% max), recovery rate 25%/s, minimum speed gate (3 km/h), global fallback. |
| **Traction Control Logic** | FULL | Per-wheel PWM via TIM1 channels, gear-based power scaling (D1: 60%, D2: 100%, R: 60%), EMA pedal filter (α=0.15), ramp rate limiting (50%/s up, 100%/s down). |
| **Ackermann Steering** | FULL | Geometric computation of FL/FR wheel angles from road angle, ±54° per-wheel clamp, correct sign convention (positive = left turn). Separate module (ackermann.c). |
| **Gear Logic (D1/D2/N/R/P)** | FULL | 5-gear system via CAN command (0x102). Park includes active brake hold with current/temperature derating. Gear changes restricted below 1 km/h. |
| **Dynamic Braking** | FULL | Proportional to throttle release rate (scale factor 0.5), max 60%, disabled below 3 km/h, disabled during ABS/SAFE/ERROR. Ramp-down at 80%/s. |
| **Regenerative Braking** | NONE | Not implemented. No negative PWM logic, no H-bridge back-EMF handling, no bidirectional current management. Confirmed absent by line-by-line audit (REGEN_BRAKING_AUDIT.md). |
| **Current Limiting** | FULL | Per-motor 25A max via INA226 sensors (6 channels through TCA9548A multiplexer). Overcurrent triggers DEGRADED state; consecutive errors (≥3) escalate to SAFE. Park hold PWM derated above 15A/20A. |
| **Temperature Protection** | FULL | 5× DS18B20 OneWire sensors with CRC-8 validation. Warning at 80°C, critical at 90°C. Park hold PWM derated above 70°C/85°C. Temperature triggers state escalation. |
| **Battery Protection** | PARTIAL | Battery bus current/voltage monitored via INA226 (channel 4, 0.5 mΩ shunt). CAN message 0x207 reports values. No SOC estimation, no undervoltage cutoff, no charge protection logic. |
| **Encoder Handling** | FULL | E6B2-CWZ6C quadrature encoder via TIM2 (4800 CPR). Range check (±987 counts = ±74°), jump detection (>100 counts/cycle), frozen detection (>200 ms with motor active >10%). |
| **Wheel Speed Processing** | FULL | 4× LJ12A3 inductive sensors via EXTI interrupts, 1 ms debounce, speed = (pulses/CPR) × circumference × 3.6. 6 pulses/rev, 1.1 m circumference. |
| **CAN Protocol** | FULL | Frozen contract v1.0. 500 kbps, 11-bit IDs. Hardware RX white-list filters. 12 TX message types (0x001, 0x200–0x207, 0x301–0x303). 4 RX message types (0x011, 0x100–0x102, 0x110). TX/RX statistics tracking. |
| **HMI Integration Model** | FULL | ESP32-S3 connected via CAN. Screen states (Boot/Standby/Drive/Safe/Error) driven by STM32 system state. Stub screen implementations ready for TFT graphics library. CAN contract frozen. |
| **Service Mode** | FULL | 25 modules classified as CRITICAL (4) or NON-CRITICAL (21). Per-module fault tracking (NONE/WARNING/ERROR/DISABLED). CAN commands (0x110) for enable/disable/factory-restore. Status broadcast (0x301–0x303). |
| **Degraded / Limp Mode** | PARTIAL | DEGRADED state exists with 40% power limit. Basic concept implemented. Missing: granular power/steering/speed limiting per degradation level, drive-home prioritization logic, diagnostic struct as in reference LimpMode. |
| **Fault Reporting** | FULL | Error codes via CAN (0x203 safety status, 0x300–0x303 diagnostics/service). Per-module fault bitmasks. Heartbeat includes fault flags and error code. |
| **Steering Calibration** | FULL | Automatic centering via inductive sensor (PB5). Sweep left/right at low PWM (10%), stall detection (300 ms), total timeout (10 s), range limit (6000 counts). State machine: IDLE → SWEEP_LEFT → SWEEP_RIGHT → DONE/FAULT. |
| **Obstacle Detection** | NONE | Not implemented. Reference has TOFSense-M S LiDAR sensor (UART), proximity levels (SAFE/CAUTION/WARNING/CRITICAL), distance zones, fail-safe behavior. |
| **Obstacle Safety / Avoidance** | NONE | Not implemented. Reference has parking assist, collision avoidance, emergency stop, speed reduction factor, blind spot (reserved), adaptive cruise coordination. |
| **AI Regen Logic** | NONE | Not implemented. Reference has lookup-table-based AI model with speed/acceleration/SOC/temperature/slope inputs, energy recovery tracking, confidence scoring. |
| **Energy Recovery Tracking** | NONE | Not implemented. No Wh counters, no regen cycle counting. |
| **SOC Estimation** | NONE | Not implemented. Battery voltage/current monitored but no state-of-charge algorithm. |
| **Power Management Sequencing** | PARTIAL | Relay power sequencing (Main → Traction → Direction with 50 ms delays) is implemented for power-up/power-down. Missing: ignition key detection, power hold relay, shutdown audio sequencing, state machine (OFF → POWER_HOLD → CENTERING → AUX_POWER → FULL_POWER → SHUTDOWN stages). |
| **Hidden Engineering Menu** | NONE | Not implemented on ESP32 HMI side. Reference has secret code entry (8989), pedal/encoder calibration, module enable/disable, factory restore, error log viewer. |
| **LED Lighting System** | NONE | Not implemented. Reference has WS2812B controller (28 front + 16 rear LEDs), LED control menus, state-dependent lighting. |
| **Audio System** | NONE | Not implemented. Reference has DFPlayer Mini integration, alert sounds, audio queue, shutdown audio. |
| **Touch Display** | NONE | ESP32 screen classes are stubs. Reference has ST7796S TFT 480×320, XPT2046 touch, touch calibration, full HUD compositor, gauges, icons, dirty-rect rendering. |
| **I2C Recovery** | NONE | Not implemented. Reference has I2C bus recovery mechanism (clock cycling, bus reset). Current firmware uses I2C for INA226/TCA9548A but has no recovery on bus lockup. |
| **EEPROM / Config Persistence** | NONE | Not implemented. Reference has EEPROM persistence with checksum validation, config manager, storage abstraction. Current firmware uses hardcoded constants only. |
| **Advanced Diagnostics** | PARTIAL | CAN diagnostic messages (0x300–0x303) exist. Missing: runtime self-test, memory stress test, boot sequence validation, functional tests as in reference. |
| **MCP23017 GPIO Expander** | INTENTIONALLY REMOVED | Not needed. STM32G474RE has sufficient GPIO pins (LQFP64, 51 I/O). Reference used MCP23017 I2C expander due to ESP32-S3 pin limitations. |
| **PCA9685 PWM Driver** | INTENTIONALLY REMOVED | Not needed. STM32 uses native hardware TIM1/TIM8 PWM at 20 kHz. Reference initially used PCA9685 I2C PWM driver (250 Hz), later migrated away. |
| **WiFi / OTA** | INTENTIONALLY REMOVED | Removed in reference v2.11.0 for security (standalone-only). Not applicable to STM32 firmware. CAN is the sole communication channel. |
| **RTOS Multi-task** | INTENTIONALLY REMOVED | STM32 uses bare-metal super-loop with multi-rate timing (10/50/100/1000 ms). Provides deterministic execution without RTOS overhead. Appropriate for safety-critical control. |
| **TFT Display (on control MCU)** | INTENTIONALLY REMOVED | Display is handled by ESP32 HMI, not the STM32 control MCU. Correct architectural separation — display rendering should not compete with motor control timing. |

---

## 3. Feature-by-Feature Master Checklist

### Legend

| Symbol | Meaning |
|--------|---------|
| [✓] | Fully implemented and verified |
| [~] | Implemented but could be improved |
| [ ] | Not implemented |
| [-] | Intentionally excluded (with reason) |

### 3.1 Motor Control & Traction

- [✓] Per-wheel PWM generation (TIM1 channels, 20 kHz)
- [✓] Steering motor PWM (TIM8_CH3, PID control)
- [✓] Gear-based power scaling (D1: 60%, D2: 100%, R: 60%)
- [✓] Pedal signal conditioning (EMA filter α=0.15, ramp limiter)
- [✓] Dynamic braking (throttle release proportional, max 60%)
- [✓] Park hold with current/temperature derating
- [✓] Ackermann geometry (separate module, ±54° clamp)
- [✓] 4×4 / 4×2 mode support (via CAN command 0x102)
- [✓] Tank turn mode support
- [✓] Motor enable/disable signals (GPIOC)
- [✓] Direction control signals (GPIOC)
- [ ] Regenerative braking (no negative PWM, no back-EMF handling)
- [ ] Adaptive cruise control (reference: adaptive_cruise.cpp)
- [~] Virtual differential (Ackermann geometry computed, but not dynamically adjusted based on speed/slip — reference traction.cpp has more advanced logic)

### 3.2 Safety Systems

- [✓] System state machine (BOOT → STANDBY → ACTIVE ⇄ DEGRADED → SAFE → ERROR)
- [✓] ABS per-wheel modulation (15% slip threshold, 10 km/h min)
- [✓] TCS per-wheel modulation (15% slip, progressive 40%→80% reduction)
- [✓] ABS/TCS global fallback (all-wheel lockup/spin)
- [✓] Overcurrent protection (25A per motor, fault escalation)
- [✓] Overtemperature protection (80°C warn, 90°C critical)
- [✓] CAN heartbeat timeout (250 ms)
- [✓] Steering encoder fault detection (range, jump, frozen)
- [✓] Relay power sequencing (Main → Traction → Direction)
- [✓] IWDG hardware watchdog (500 ms)
- [✓] Command validation gates (throttle 0–100%, steering ±54°, rate limit 200°/s)
- [✓] Gear change speed gate (below 1 km/h)
- [✓] Mode change speed gate (below 1 km/h)
- [✓] Consecutive error counting with escalation (≥3 → DEGRADED→SAFE)
- [~] Degraded/limp mode (basic 40% power limit exists; lacks granular per-subsystem limiting as in reference limp_mode.cpp with NORMAL/DEGRADED/LIMP/CRITICAL states)
- [ ] Obstacle collision avoidance (reference: obstacle_safety.cpp — parking assist, emergency stop, speed reduction)
- [ ] Battery undervoltage cutoff (reference: limp_mode.cpp checks batteryUndervoltage)
- [ ] Sensor plausibility cross-validation (reference: SensorManagerEnhanced.cpp)
- [ ] Redundant safety checks (reference: SafetyManagerEnhanced.cpp)

### 3.3 Sensor Management

- [✓] 4× wheel speed sensors (LJ12A3 inductive, EXTI interrupts, 1 ms debounce)
- [✓] Steering encoder (E6B2-CWZ6C, TIM2 quadrature, 4800 CPR)
- [✓] Steering center inductive sensor (PB5, EXTI5)
- [✓] Pedal ADC (PA3, ADC1_IN4, 12-bit)
- [✓] 6× INA226 current sensors (I2C via TCA9548A multiplexer)
- [✓] 5× DS18B20 temperature sensors (OneWire bit-bang, CRC-8 validation)
- [✓] Battery bus current/voltage (INA226 channel 4)
- [ ] Obstacle detection sensors (reference: TOFSense-M S LiDAR via UART)
- [ ] I2C bus recovery mechanism (reference: i2c_recovery.cpp — clock cycling, bus reset)
- [~] Sensor data filtering (pedal has EMA; wheel speed and current use raw values — reference has dedicated filters.cpp module with multiple filter types)

### 3.4 CAN Communication

- [✓] FDCAN1 at 500 kbps (classic CAN 2.0A mode)
- [✓] Hardware RX white-list filters (3 filter banks)
- [✓] Heartbeat TX (0x001, 100 ms) with counter, state, fault flags
- [✓] Heartbeat RX (0x011) with timeout monitoring
- [✓] Throttle command RX (0x100) with safety validation
- [✓] Steering command RX (0x101) with safety validation
- [✓] Mode/gear command RX (0x102) with gear validation
- [✓] Service command RX (0x110) for module enable/disable
- [✓] Speed status TX (0x200, 4× wheel speeds)
- [✓] Current status TX (0x201, 4× motor currents)
- [✓] Temperature status TX (0x202, 5× sensors)
- [✓] Safety status TX (0x203, ABS/TCS active, error code)
- [✓] Steering status TX (0x204, angle + calibrated flag)
- [✓] Traction status TX (0x205, 4× per-wheel scale)
- [✓] Temperature map TX (0x206, FL/FR/RL/RR/Ambient)
- [✓] Battery status TX (0x207, current + voltage)
- [✓] Service status TX (0x301–0x303, fault/enabled/disabled masks)
- [✓] TX/RX statistics tracking
- [~] CAN error handling (basic — could add CAN bus-off recovery, error frame counting)

### 3.5 HMI (ESP32-S3)

- [✓] CAN RX decoder (12 message types parsed)
- [✓] VehicleData passive container (11 telemetry channels)
- [✓] Screen state machine (Boot/Standby/Drive/Safe/Error)
- [✓] Heartbeat TX (0x011, 100 ms)
- [✓] CAN ID contract (can_ids.h, frozen v1.0)
- [ ] TFT display rendering (screen classes are stubs — no TFT library integrated)
- [ ] Touch input handling (no touch library)
- [ ] Gauges / graphical HUD (reference: gauges.cpp, hud.cpp, hud_compositor.cpp, icons.cpp)
- [ ] Hidden engineering menu (reference: menu_hidden.cpp — secret code 8989)
- [ ] Pedal/encoder calibration UI (reference: menu_encoder_calibration.cpp)
- [ ] Sensor config menu (reference: menu_sensor_config.cpp)
- [ ] Power config menu (reference: menu_power_config.cpp)
- [ ] LED control menu (reference: menu_led_control.cpp)
- [ ] Obstacle display overlay (reference: obstacle_display.cpp)
- [ ] Limp mode diagnostics display (reference: hud_limp_diagnostics.cpp, hud_limp_indicator.cpp)
- [ ] Wheel status display (reference: wheels_display.cpp)

### 3.6 System Infrastructure

- [✓] Multi-rate super-loop (10/50/100/1000 ms tiers)
- [✓] Clock configuration (HSI → PLL → 170 MHz SYSCLK)
- [✓] Peripheral initialization (FDCAN, I2C, TIM, ADC, IWDG)
- [✓] Startup assembly (startup_stm32g474retx.s)
- [✓] Linker script (STM32G474RETX_FLASH.ld)
- [✓] Makefile build system
- [ ] Config persistence / EEPROM (reference: eeprom_persistence.cpp, config_storage.cpp, storage.cpp)
- [ ] Structured logging (reference: logger.cpp)
- [ ] Boot guard / pre-flight validation (reference: boot_guard.cpp)
- [-] WiFi/OTA (intentionally excluded — security; CAN only)
- [-] RTOS (intentionally excluded — bare-metal for determinism)
- [-] MCP23017 GPIO expander (intentionally excluded — STM32 has sufficient GPIO)
- [-] PCA9685 PWM driver (intentionally excluded — native TIM PWM)

### 3.7 Advanced Features (Reference Only)

- [ ] Regen AI logic (reference: regen_ai.cpp — lookup-table model, feature extraction, confidence scoring)
- [ ] Energy recovery tracking (reference: RegenAI::State.energyRecovered in Wh)
- [ ] SOC estimation (reference: RegenAI::Features.batterySOC)
- [ ] Obstacle avoidance logic (reference: obstacle_safety.cpp — 5-zone distance system, emergency braking)
- [ ] Obstacle detection (reference: obstacle_detection.cpp — TOFSense-M S LiDAR, UART protocol parser)
- [ ] Hidden engineering menu (reference: menu_hidden.cpp — 45 KB, comprehensive diagnostic tool)
- [ ] Module enable/disable system via HMI (reference: menu_sensor_config.cpp — touch-based toggling)
- [ ] Advanced diagnostics (reference: boot_sequence_test.h, functional_tests.h, hardware_failure_tests.h, memory_stress_test.h)
- [ ] Sensor plausibility layers (reference: SensorManagerEnhanced.cpp — cross-validation)
- [ ] Redundant safety checks (reference: SafetyManagerEnhanced.cpp)
- [ ] Startup / shutdown sequencing (reference: power_mgmt.cpp — 10-state machine from OFF to SHUTDOWN_FINAL)
- [ ] Emergency behaviors beyond state machine (reference: obstacle_safety.cpp — triggerEmergencyStop/resetEmergencyStop)
- [ ] AI/lookup-table logic (reference: regen_ai.cpp — predictRegenPower with multi-feature input)
- [ ] Thermal management extensions (reference: temperature.cpp — per-zone mapping, trend analysis)
- [ ] Advanced traction features (reference: traction.cpp — 27 KB, extensive differential and torque vectoring)
- [ ] Audio alerts (reference: alerts.cpp, dfplayer.cpp, queue.cpp)
- [ ] LED lighting control (reference: led_controller.cpp — WS2812B 28+16 LEDs)
- [ ] Touch calibration system (reference: touch_calibration.cpp)
- [ ] Telemetry logging (reference: telemetry.cpp)
- [ ] I2C recovery mechanism (reference: i2c_recovery.cpp)

---

## 4. Safety Gap Analysis

This section identifies safety mechanisms present in the reference firmware that are missing, simplified, or reduced in robustness in the current STM32 implementation. Hardware-level gaps are excluded; only software-level safety logic is evaluated.

### 4.1 Missing Safety Mechanisms

| Safety Feature | Reference Implementation | Current Status | Impact |
|---------------|-------------------------|----------------|--------|
| **Battery undervoltage cutoff** | limp_mode.cpp checks `batteryUndervoltage` flag, reduces power progressively | Not implemented — voltage is read (INA226 ch4) but no cutoff logic exists | Could allow motor operation below safe battery voltage, risking deep discharge or controller brownout |
| **Obstacle collision avoidance** | obstacle_safety.cpp: 5-zone distance system, emergency braking, speed reduction factor (0.0–1.0) | Not implemented — no distance sensors, no collision logic | Vehicle has no forward/rear obstacle awareness. Acceptable if operated in controlled environment only |
| **I2C bus recovery** | i2c_recovery.cpp: clock cycling (9+ pulses on SCL), bus reset, automatic retry | Not implemented — I2C failure on TCA9548A or INA226 would leave sensors unreadable | A stuck I2C bus would prevent current/temperature readings. Could mask overcurrent/overtemperature conditions |
| **Sensor cross-validation** | SensorManagerEnhanced.cpp: compares sensor readings against each other for plausibility | Not implemented — each sensor read independently, no cross-checks | A single faulty sensor could provide wildly incorrect data without detection (e.g., temperature sensor reading −55°C) |
| **Boot guard / pre-flight** | boot_guard.cpp: validates hardware state before enabling control | Not implemented — system transitions BOOT → STANDBY based on timing only | Could allow operation with misconfigured or failed peripherals |
| **Granular limp mode** | limp_mode.cpp: 4-level system (NORMAL/DEGRADED/LIMP/CRITICAL) with separate power, steering, and speed multipliers | Simplified — single DEGRADED state with flat 40% power limit | Does not differentiate between minor sensor warning and major system failure. Cannot apply asymmetric limits (e.g., reduce speed but maintain steering) |
| **Emergency stop via obstacle** | obstacle_safety.cpp: triggerEmergencyStop() with immediate motor cut + relay open | Not implemented — emergency stop only via state machine transition to SAFE/ERROR | No distance-based emergency braking capability |

### 4.2 Simplified Safety Mechanisms

| Safety Feature | Reference Complexity | Current Simplification | Risk |
|---------------|---------------------|----------------------|------|
| **Temperature trend analysis** | temperature.cpp: per-zone trend tracking, predictive warnings | Current: threshold-only (80°C warn, 90°C crit) — no rate-of-change detection | Cannot predict thermal runaway before threshold is crossed |
| **Traction control progression** | traction.cpp (27 KB): torque vectoring, differential speed computation, load-dependent scaling | Current: basic per-wheel slip percentage with fixed reduction curve | Adequate for low-speed operation; may need improvement for higher speeds or wet surfaces |
| **Fault escalation timing** | SafetyManagerEnhanced.cpp: time-windowed error counting, severity-weighted escalation | Current: simple consecutive counter (≥3 → escalate), overcurrent recovery (1 s clean) | Could escalate too aggressively on transient faults, or too slowly on intermittent real faults |

### 4.3 Reduced Robustness Areas

| Area | Reference Robustness | Current Level | Assessment |
|------|---------------------|---------------|------------|
| **CAN bus error handling** | Robust error frame counting, bus-off detection and recovery | Basic — heartbeat timeout only, no bus-off recovery procedure | In CAN bus fault conditions (EMI, wiring issues), system would transition to SAFE via timeout rather than attempting recovery |
| **Watchdog coverage** | IWDG + task-level health monitoring (FreeRTOS) | IWDG only (hardware, 500 ms) | Adequate for single-loop architecture. A stuck ISR or infinite loop in sensor read would still trigger IWDG |
| **Power sequencing safety** | power_mgmt.cpp: 10-state machine, ignition detection, power hold, graceful shutdown with audio | Current: 3-relay sequencing with 50 ms delays | Functional for power-up/down. Missing graceful shutdown — abrupt power loss could leave relays in inconsistent state |

---

## 5. Phased Development Roadmap

### PHASE 1 — Stability Consolidation

**Goal:** Harden existing safety mechanisms and improve sensor robustness without adding new features.

**Priority:** Immediate — these items strengthen the existing foundation.

**Why first:** These changes reduce risk within the current operational envelope before expanding capabilities. They fix potential failure modes in existing code paths.

| Item | Description | Effort |
|------|-------------|--------|
| 1.1 | **I2C recovery mechanism** — Implement clock cycling (9+ SCL pulses) and bus reset on I2C failure. Protects INA226 and TCA9548A communication. | Medium |
| 1.2 | **Battery undervoltage protection** — Add voltage threshold check on INA226 ch4 bus voltage. Below threshold → DEGRADED (reduced power), below critical → SAFE (actuators off). | Low |
| 1.3 | **CAN bus-off recovery** — Monitor FDCAN error state register. On bus-off, attempt automatic recovery after configurable delay. Log bus-off events. | Low |
| 1.4 | **Sensor plausibility checks** — Cross-validate temperature readings (reject if any sensor deviates >30°C from median). Validate wheel speed consistency (if 3 wheels agree and 1 disagrees, flag the outlier). | Medium |
| 1.5 | **DS18B20 fault tolerance** — If a temperature sensor fails CRC or returns out-of-range value, mark it invalid via service_mode and continue with remaining sensors instead of potentially using stale data. | Low |

### PHASE 2 — Safety Extensions

**Goal:** Add safety-critical features that the reference firmware implements and that protect the vehicle occupant.

**Priority:** High — these features directly improve occupant safety.

**Why second:** Requires the stable sensor foundation from Phase 1. These features add protection layers that do not exist today.

| Item | Description | Effort |
|------|-------------|--------|
| 2.1 | **Granular limp mode** — Extend DEGRADED state into a multi-level system (DEGRADED_MINOR / DEGRADED_MAJOR / LIMP) with separate power, steering, and speed multipliers. Maintain drive-home philosophy. | Medium |
| 2.2 | **Boot validation sequence** — Before transitioning BOOT → STANDBY, verify: I2C bus accessible, INA226 responds on all channels, DS18B20 search finds expected count, TIM1/TIM8 PWM outputs confirmed, encoder counter advancing. Report failures via CAN. | Medium |
| 2.3 | **Temperature rate-of-change monitoring** — Track temperature delta per second. If rate exceeds threshold (e.g., >5°C/s), trigger early warning before absolute threshold is reached. | Low |
| 2.4 | **Fault escalation with time windows** — Replace simple consecutive counter with time-windowed error counting (e.g., 5 errors within 10 seconds = escalate, but 3 errors spread over 60 seconds = clear). Prevents false escalation from transient noise. | Medium |
| 2.5 | **Graceful power-down sequence** — On transition to SAFE/ERROR, implement timed relay shutdown: disable traction motors → wait 100 ms → disable steering → wait 100 ms → disable main relay. Prevents inductive kickback. | Low |

### PHASE 3 — Smart Features

**Goal:** Add intelligent vehicle features that improve driving experience and efficiency.

**Priority:** Medium — these are functional improvements, not safety-critical.

**Why third:** These features build on the stable and safe platform from Phases 1–2. They add capability without compromising existing safety.

| Item | Description | Effort |
|------|-------------|--------|
| 3.1 | **Regenerative braking (basic)** — Implement braking via H-bridge reversal on BTS7960 drivers. Apply proportional regen based on throttle release rate (reuse dynamic braking infrastructure). Limit regen to safe current levels (e.g., 10A per motor). Add regen active flag to CAN 0x203. | High |
| 3.2 | **Energy recovery tracking** — Track total energy recovered (Wh) using current × voltage × time integration during regen cycles. Report via new CAN message or extend 0x207. | Low |
| 3.3 | **Config persistence (STM32 Flash)** — Store calibration values (steering center offset, pedal min/max, enabled modules) in STM32 Flash user pages. Load on boot, save on service command. Add CRC-32 for data integrity. | Medium |
| 3.4 | **ESP32 HMI graphics** — Integrate TFT graphics library (TFT_eSPI or LVGL) into ESP32 firmware. Implement drive screen with speed, gear indicator, battery bar. Implement error screen with fault codes. | High |
| 3.5 | **Structured logging** — Add serial debug output with severity levels (DEBUG/INFO/WARN/ERROR) and module tags. Conditionally compiled (DEBUG build only, zero overhead in release). | Low |

### PHASE 4 — Advanced Optimization

**Goal:** Refine existing features with more sophisticated algorithms and add advanced diagnostic capabilities.

**Priority:** Low-Medium — these optimize beyond functional requirements.

**Why fourth:** Requires the regen and logging infrastructure from Phase 3. These are refinement tasks that improve quality but are not blocking.

| Item | Description | Effort |
|------|-------------|--------|
| 4.1 | **Advanced traction control** — Add load-dependent scaling (reduce traction on rear wheels during uphill based on current imbalance). Add surface-adaptive slip thresholds (detect low-grip conditions from sustained slip events). | High |
| 4.2 | **SOC estimation** — Implement coulomb counting using battery current sensor. Track energy consumed and recovered. Estimate remaining capacity. Report via CAN. | Medium |
| 4.3 | **ESP32 engineering menu** — Implement hidden menu on ESP32 (access via CAN service command or long-press pattern). Show per-wheel telemetry, per-sensor health, module enable/disable, factory restore. | Medium |
| 4.4 | **Runtime self-test** — Periodically verify sensor health during STANDBY: pulse each motor briefly (<5 ms) and verify current sensor response. Verify encoder counts change during steering centering. Log results. | Medium |
| 4.5 | **CAN message CRC** — Add CRC-8 to critical CAN messages (throttle, steering, mode commands) for transmission error detection beyond CAN's built-in CRC-15. | Low |

### PHASE 5 — Optional AI Enhancements

**Goal:** Add AI-inspired features that optimize vehicle behavior using data-driven approaches.

**Priority:** Low — these are enhancement features, not safety or functional requirements.

**Why last:** These features require the complete sensor, regen, and diagnostic infrastructure from Phases 1–4. They provide marginal improvement over well-tuned conventional algorithms.

| Item | Description | Effort |
|------|-------------|--------|
| 5.1 | **AI-based regen optimization** — Implement lookup-table model for optimal regen power based on speed, battery voltage, temperature, and estimated slope. Use offline-generated tables (not runtime ML). | Medium |
| 5.2 | **Predictive thermal management** — Use temperature trend data to predict time-to-overheat. Proactively reduce power before thermal limits are reached. | Medium |
| 5.3 | **Adaptive pedal mapping** — Adjust pedal response curve based on driving pattern (detect aggressive vs. gentle driving). Store preferences per session. | Low |
| 5.4 | **Obstacle detection integration** — If ultrasonic or LiDAR sensor is added to hardware, implement distance reading via UART or I2C on STM32. Add proximity-based speed limiting. Integrate with safety state machine. | High |
| 5.5 | **Telemetry data logging** — Log timestamped telemetry snapshots to STM32 Flash or SD card (if SPI available). Enable post-drive analysis. | Medium |

---

## 6. Architecture Preservation Check

### 6.1 Does Any Current Feature Require Refactoring?

**No.** All current modules (motor_control.c, safety_system.c, sensor_manager.c, can_handler.c, steering_centering.c, service_mode.c, ackermann.c) are well-structured with clear responsibilities. The multi-rate super-loop architecture is appropriate for the STM32's bare-metal environment. No module has grown beyond maintainable size. The CAN protocol contract is frozen and correctly implemented on both sides.

### 6.2 Is Any Core Design Flaw Detected?

**No.** The architecture follows sound embedded systems design:
- Safety state machine has clear, unambiguous transitions
- Fault escalation is monotonic (never auto-recovers to a better state without explicit reset)
- CAN communication uses hardware-filtered white-list (rejects unknown IDs)
- PWM generation uses hardware timers (not software-toggled GPIO)
- Watchdog is hardware-based (IWDG, cannot be disabled by software after start)
- Relay sequencing prevents inrush current

### 6.3 Is the Current Architecture Scalable?

**Yes.** The architecture supports incremental growth:
- New sensor types can be added to sensor_manager.c without modifying other modules
- New CAN message types can be added to can_handler.c (TX and RX handlers are independent)
- New safety checks can be added to safety_system.c's update function
- Service mode already supports up to 32 modules (currently using 25)
- The multi-rate loop can accommodate new tasks at any tier (10/50/100/1000 ms)
- The ESP32 CAN decoder (can_rx.cpp) handles unknown IDs gracefully (ignores them)

### 6.4 Would Adding an Obstacle Safety Layer Be Clean or Invasive?

**Clean, with caveats.**
- A new obstacle detection module can be added as `obstacle_sensor.c` (UART-based sensor reading)
- Distance thresholds can be checked in the 50 ms tier (sensor read) and 10 ms tier (safety response)
- Speed reduction can be applied via the existing `wheel_scale[]` array in safety_system.c
- Emergency stop can use the existing `Safety_TransitionState(STATE_SAFE)` mechanism
- **Caveat:** If the obstacle sensor uses UART, a UART peripheral must be configured (currently unused on STM32G474RE — USART1/2/3 are available). This is a peripheral configuration change, not an architectural one.
- **Caveat:** CAN messages for obstacle data would need new IDs (must be added to the frozen CAN contract — requires coordinated update on both STM32 and ESP32).

### 6.5 Would Future Regen Implementation Require Structural Changes?

**Minor changes only, no structural refactoring.**
- The BTS7960 H-bridge drivers already support bidirectional current flow. Regen requires reversing the PWM polarity while monitoring current direction.
- `motor_control.c` already has the `Motor_SetWheelPWM()` function infrastructure. Regen would add a complementary `Motor_SetRegenPWM()` or extend the existing function with a direction parameter.
- `safety_system.c` would need an additional gate: disable regen when battery voltage is too high or temperature exceeds regen-safe threshold.
- `can_handler.c` would need a regen status field in an existing message (e.g., extend 0x203) or a new message ID.
- **No existing module needs to be refactored.** The changes are additive.

---

## 7. Risk Classification

### Risk Legend

| Level | Definition |
|-------|-----------|
| **LOW** | Comfort / UX feature — no safety impact |
| **MEDIUM** | Safety improvement — adds protection against specific failure modes |
| **HIGH** | Safety-critical missing protection — could lead to hazardous condition |

### Risk Classification Table

| Missing Feature | Risk | Justification |
|----------------|------|---------------|
| Regenerative braking | LOW | Functional feature for energy recovery. Vehicle operates safely without it (dynamic braking provides deceleration). |
| AI regen optimization | LOW | Performance optimization. Basic regen (if implemented) provides most of the benefit. |
| Energy recovery tracking | LOW | Informational / UX feature. No safety impact. |
| SOC estimation | LOW | Informational. Battery undervoltage cutoff (Phase 1) provides the safety aspect independently. |
| Hidden engineering menu | LOW | Diagnostic convenience. Service mode already accessible via CAN commands. |
| LED lighting system | LOW | Cosmetic / UX feature. Not related to vehicle control safety. |
| Audio alerts system | LOW | UX feature. Visual warnings via HMI provide equivalent notification. |
| Touch display graphics | LOW | UX feature. CAN telemetry is available regardless of display rendering. |
| Config persistence | LOW | Convenience. Hardcoded defaults are safe. Mainly affects recalibration time after power cycle. |
| Structured logging | LOW | Development convenience. Not user-facing. |
| Adaptive pedal mapping | LOW | Comfort feature. Fixed mapping is functional. |
| Telemetry data logging | LOW | Post-drive analysis tool. No real-time safety impact. |
| CAN message CRC-8 | MEDIUM | Improves command integrity beyond CAN's built-in CRC-15. Unlikely to matter on short bus (2 nodes) but good practice. |
| Advanced traction control | MEDIUM | Improves grip on low-traction surfaces. Current ABS/TCS provides basic protection. |
| Temperature rate-of-change | MEDIUM | Enables predictive thermal warning before absolute threshold. Current threshold-based protection still functional. |
| Fault escalation timing | MEDIUM | Reduces false escalation from transient noise. Current simple counter is conservative (safe but potentially annoying). |
| ESP32 engineering menu | MEDIUM | Improves diagnostic capability. Service mode exists via CAN but is harder to use without GUI. |
| Runtime self-test | MEDIUM | Adds confidence in sensor health. Current boot-time checks and continuous monitoring partially cover this. |
| Boot validation sequence | MEDIUM | Prevents operation with failed peripherals. Current architecture transitions to STANDBY on timing only. IWDG provides partial protection. |
| Graceful power-down | MEDIUM | Prevents relay state inconsistency. Current reverse-order shutdown is functional but could be more robust. |
| Obstacle detection | MEDIUM | Adds spatial awareness. Vehicle can operate safely without it in controlled environments. Becomes HIGH in public/shared spaces. |
| Granular limp mode | MEDIUM | Improves drive-home capability. Current flat 40% limit is safe but suboptimal — could be too aggressive for minor faults or insufficient for major ones. |
| Sensor plausibility checks | MEDIUM | Detects faulty sensor data before it affects control decisions. Current per-sensor fault detection covers most cases. |
| I2C bus recovery | HIGH | A locked I2C bus silently disables all current and temperature sensing (6× INA226 + TCA9548A multiplexer). Without these readings, overcurrent and overtemperature protection are blind. This is the highest-priority missing safety feature. |
| Battery undervoltage cutoff | HIGH | Deep battery discharge can damage cells and cause controller brownout. Voltage is monitored but no cutoff logic exists. Operating motors on a depleted battery risks uncontrolled shutdown. |
| Sensor cross-validation | MEDIUM | Important for detecting stuck or drifting sensors, but each sensor has independent fault detection. Becomes HIGH if safety-critical decisions rely on single sensor readings. |

---

## 8. Final Verdict

### Overall Maturity Assessment

| Category | Rating | Justification |
|----------|--------|---------------|
| **Motor Control** | ADVANCED | Complete per-wheel PWM with Ackermann geometry, gear logic, dynamic braking, pedal conditioning, and park hold. |
| **Safety Systems** | STABLE | Comprehensive state machine, ABS/TCS, overcurrent/overtemperature protection, watchdog, relay sequencing. Two HIGH-risk gaps (I2C recovery, battery cutoff) prevent ADVANCED rating. |
| **Sensor Management** | STABLE | All physical sensors correctly interfaced with hardware-appropriate protocols (EXTI, quadrature, I2C, OneWire, ADC). Missing I2C recovery and cross-validation. |
| **CAN Communication** | ADVANCED | Frozen contract fully implemented, hardware-filtered, well-documented. Both STM32 and ESP32 sides aligned. |
| **HMI** | EARLY | Architecture defined, CAN decoding functional, screen state machine implemented. Display rendering is stub-only. |
| **System Infrastructure** | STABLE | Clean build system, correct clock/peripheral configuration, multi-rate loop. Missing config persistence and structured logging. |
| **Advanced Features** | EARLY | None of the reference's AI, obstacle, regen, audio, or lighting features are implemented. Intentional — these are development targets, not regressions. |

### Overall Rating: **STABLE**

The firmware provides a **safe, functional, and well-architected** vehicle control layer. Core motor control and safety systems are production-quality for a controlled environment. The split architecture (STM32 + ESP32 over CAN) is superior to the monolithic reference design for safety-critical applications.

**Two actions are recommended before considering the firmware production-ready:**
1. Implement I2C bus recovery (Phase 1.1) — eliminates the highest-risk safety gap
2. Implement battery undervoltage cutoff (Phase 1.2) — prevents deep discharge hazard

With these two items addressed, the safety subsystem would achieve ADVANCED maturity. Full PRODUCTION-READY status requires completing Phases 1 and 2 of the roadmap.

---

*Document generated: 2026-02-12*
*This is an analysis and documentation deliverable. No code was modified.*
