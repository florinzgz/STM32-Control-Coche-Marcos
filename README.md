# STM32G474RE Control Car - Marcos

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware Topology](#2-hardware-topology)
3. [Firmware Architecture](#3-firmware-architecture)
4. [Safety Architecture](#4-safety-architecture)
5. [Scheduler / Task Timing](#5-scheduler--task-timing)
6. [Motor Control Pipeline](#6-motor-control-pipeline)
7. [Steering System](#7-steering-system)
8. [ABS / TCS Implementation](#8-abs--tcs-implementation)
9. [Dynamic Braking](#9-dynamic-braking)
10. [Obstacle Safety System](#10-obstacle-safety-system)
11. [CAN Protocol Overview](#11-can-protocol-overview)
12. [Wheel Speed Acquisition](#12-wheel-speed-acquisition)
13. [Encoder Handling](#13-encoder-handling)
14. [Current Sensing — INA226](#14-current-sensing--ina226)
15. [Temperature Monitoring — DS18B20](#15-temperature-monitoring--ds18b20)
16. [Battery Monitoring](#16-battery-monitoring)
17. [Safety States](#17-safety-states)
18. [Emergency Stop Behavior](#18-emergency-stop-behavior)
19. [PWM Generation Architecture](#19-pwm-generation-architecture)
20. [BTS7960 (IBT-2) Driver Handling](#20-bts7960-ibt-2-driver-handling)
21. [Touchscreen / Display System](#21-touchscreen--display-system)
22. [Audio / DFPlayer Mini Integration](#22-audio--dfplayer-mini-integration)
23. [GPIO Expander — MCP23017 (Shifter)](#23-gpio-expander--mcp23017-shifter)
24. [Persistent Storage (Flash / NVS)](#24-persistent-storage-flash--nvs)
25. [Service Mode](#25-service-mode)
26. [Error Log](#26-error-log)
27. [Boot Sequence](#27-boot-sequence)
28. [STM32 GPIO / Pin Map](#28-stm32-gpio--pin-map)
29. [ESP32-S3 GPIO / Pin Map](#29-esp32-s3-gpio--pin-map)
30. [Folder Structure](#30-folder-structure)
31. [Build Instructions](#31-build-instructions)
32. [Feature Status Matrix](#32-feature-status-matrix)
33. [Known Limitations](#33-known-limitations)
34. [Safety Disclaimers](#34-safety-disclaimers)
35. [License](#35-license)
36. [Author](#36-author)

---

## 1. Project Overview

**STM32G474RE Control Car – Marcos** is a complete embedded firmware system for an electric child-scale vehicle with 4-wheel independent traction, closed-loop steering, multi-tier safety systems, and a full-colour touchscreen HMI.

The system is split into two independent microcontroller subsystems that communicate over CAN bus:

| Node | MCU | Role |
|------|-----|------|
| Safety authority | STM32G474RE (NUCLEO-G474RE) | Motor control, safety enforcement, sensor acquisition, relay sequencing |
| HMI | ESP32-S3-DevKitC-1-N16R8 | Touchscreen display, user input, audio, obstacle sensor forwarding |

**Authority model:** The ESP32 is an intent sender only. Every command it sends is validated, clamped, rate-limited, and accepted or rejected by the STM32. The ESP32 never directly controls any actuator.

---

## 2. Hardware Topology

```
┌─────────────────────────────────────────────────────────────────────┐
│                    STM32G474RE (NUCLEO-G474RE)                       │
│  TIM1 20kHz ─── RPWM/LPWM FL, FR       TIM8 20kHz ─── RPWM/LPWM RL,RR │
│  TIM3 20kHz ─── RPWM/LPWM STEER        TIM2 ─── Quadrature encoder  │
│  ADC1 ─── Pedal (PA3)                  I2C1 ─── TCA9548A → INA226×6 │
│  FDCAN1 ─── CAN 500kbps (PA11/PA12)    PB0 ─── OneWire DS18B20×5    │
│  GPIO ─── EN×5, RELAY×4, EXTI×6                                     │
└──────────┬────────────────────────────────────────┬─────────────────┘
           │ PWM/EN/RELAY                            │ CAN 500 kbps
    ┌──────▼────────────────┐                ┌───────▼──────────────────┐
    │   5× BTS7960 (IBT-2)  │                │   ESP32-S3-DevKitC-1     │
    │   FL, FR, RL, RR, ST  │                │   N16R8 (16MB/8MB PSRAM) │
    └──────┬────────────────┘                └───────┬──────────────────┘
    4× traction motors                               │
    1× steering motor                         ┌──────┴──────────────────┐
                                              │  ST7796 480×320 TFT     │
    ┌─────────────────────┐                   │  XPT2046 touch          │
    │  Sensors            │                   │  DFPlayer Mini (audio)  │
    │  6× INA226 (I2C)    │                   │  MCP23017 (gear shifter)│
    │  5× DS18B20 (1W)    │                   │  TF-Mini Plus LiDAR     │
    │  4× EL817 wheel spd │                   │  WS2812B LED strips ×2  │
    │  E6B2-CWZ6C encoder │                   │  TJA1051T/3 CAN xcvr    │
    │  LJ12A3 steer center│                   └─────────────────────────┘
    │  TJA1051T/3 CAN xcvr│
    └─────────────────────┘
```

### Bill of Key Components

| Component | Qty | Usage |
|-----------|-----|-------|
| STM32G474RE NUCLEO-G474RE | 1 | Main safety controller |
| ESP32-S3-DevKitC-1-N16R8 | 1 | HMI (16 MB Flash, 8 MB Octal PSRAM) |
| BTS7960 H-bridge (IBT-2) | 5 | Motor drivers FL/FR/RL/RR + Steering |
| INA226 power monitor | 6 | Motor current (×4) + battery (×1) + steering (×1) |
| TCA9548A I2C multiplexer | 1 | Expands I2C bus to 6 INA226 devices |
| DS18B20 temperature sensor | 5 | Motor + ambient temperatures (OneWire) |
| E6B2-CWZ6C rotary encoder | 1 | 1200 PPR / 4800 counts per rev, steering feedback |
| 6N137 optocoupler | 2 | Galvanic isolation on encoder A/B/Z lines |
| EL817 optocoupler modules | 2 | Wheel speed isolation (×4 ch) + ignition sense (×1 ch) |
| LJ12A3 inductive sensor | 1 | Steering mechanical center detection |
| Benewake TF-Mini Plus | 1 | Forward obstacle LiDAR (100–12 000 mm, 100 Hz UART) |
| TJA1051T/3 CAN transceiver | 1 | CAN PHY (STM32 side) |
| TJA1051T/3 CAN transceiver | 2 | CAN PHY (STM32 + ESP32 sides) |
| ST7796 TFT 480×320 | 1 | Main display (SPI 40 MHz, HSPI/SPI3) |
| XPT2046 touch controller | 1 | Resistive touch (SPI 2.5 MHz, shared MISO) |
| DFPlayer Mini | 1 | MP3 audio playback (UART 9600 bps, 68 tracks) |
| MCP23017 I2C GPIO expander | 1 | Physical gear selector (P/R/N/D1/D2) |
| WS2812B LED strip | 2 | Front (28 LEDs, GPIO 47) + Rear (16 LEDs, GPIO 48) |
| SRD-05VDC relay module (4-ch) | 1 | Traction + direction + audio relay control |
| SRD-05VDC relay module (2-ch) | 1 | Front/rear LED power relays |
| 120 Ω CAN terminator | 2 | One at each CAN bus end |

---

## 3. Firmware Architecture

### STM32 Modules

| File | Purpose |
|------|---------|
| `main.c` | Entry point, peripheral init, multi-tier main loop |
| `motor_control.c/h` | Traction (4 motors), steering PID, Ackermann, dynamic braking |
| `safety_system.c/h` | ABS, TCS, state machine (7 states), error codes, relay sequencer, obstacle |
| `can_handler.c/h` | FDCAN1 Tx/Rx, protocol encoding/decoding, bus-off recovery |
| `sensor_manager.c/h` | Wheel speed (EXTI/DWT), pedal ADC, DS18B20, INA226 |
| `steering_centering.c/h` | Startup sweep + LJ12A3 center sensor, centering state machine |
| `steering_cal_store.c/h` | Flash page 126 persistence (magic + CRC32), boot-time validation |
| `ackermann.c/h` | Pure geometry: inner/outer wheel angles from road angle |
| `eps_params.c/h` | Electric power steering torque-assist parameters |
| `encoder_reader.c/h` | TIM2 quadrature counter read + Z-index drift detection |
| `service_mode.c/h` | 25-module enable/disable, fault classification, factory restore |
| `error_log.c/h` | Flash page 125 ring buffer (250 entries, CRC32, post-mortem) |
| `sensor_map_store.c/h` | Flash page 125 (shared section): DS18B20 physical→role mapping |
| `boot_validation.c/h` | Pre-ACTIVE gate: sensor plausibility, battery, encoder health |
| `loop_diag.c/h` | Peak-hold 100 Hz task duration (DWT cycles → µs → 100 µs units) |
| `math_safety.c/h` | float-safe helpers (`sanitize_float`, clamped int conversions) |
| `build_sanity_checks.h` | Compile-time static asserts on critical constants |

### ESP32 Modules

| File | Purpose |
|------|---------|
| `main.cpp` | FreeRTOS task setup, orchestration (render task on Core 1) |
| `can_rx.cpp/h` | CAN Rx decoder for all STM32 status frames |
| `vehicle_data.cpp/h` | Live telemetry store shared between CAN Rx and screens |
| `screen_manager.cpp/h` | Screen state machine (boot → standby → drive → safe/error) |
| `screens/drive_screen.*` | Main driving display (speed, gear, ABS/TCS, battery, steering) |
| `screens/engineering_screen.*` | Hidden menu (entry PIN 8989): diagnostics, relay override, service |
| `screens/touch_calibration_screen.*` | Interactive XPT2046 calibration wizard |
| `audio_manager.cpp/h` | DFPlayer Mini non-blocking driver (priority queue, 68 tracks) |
| `relay_audio.cpp/h` | Speaker relay state machine (pop/click prevention) |
| `power_manager.cpp/h` | Ignition key detection and orderly shutdown sequence |
| `touch_calibration.cpp/h` | NVS persistence for XPT2046 calibration (magic + CRC32) |
| `touch_handler.cpp/h` | TFT_eSPI touch integration with calibration fallback |
| `shifter_input.cpp/h` | MCP23017 gear selector polling |
| `traction_switch.cpp/h` | 2WD/4WD toggle switch (GPIO 15) |
| `sensors/obstacle_sensor.*` | TF-Mini Plus UART driver (9-byte frames, 115200 bps) |
| `config_store.cpp/h` | NVS `hmi_cfg` namespace for HMI settings and fault log |
| `led_controller.cpp/h` | FastLED WS2812B strip control |

---

## 4. Safety Architecture

The STM32 is the **sole safety authority**. No ESP32 action can directly cause an unsafe actuator state. Every motor command passes through:

```
ESP32 command (CAN) → Safety_IsCommandAllowed() → Safety_ValidateThrottle()
    → Traction_SetDemand() → Traction_Update() → Motor_SetSignedPWM_*()
```

Safety checks run at 100 Hz in the main loop:

| Check | Function | Action on Fault |
|-------|----------|-----------------|
| Overcurrent (>25 A/motor) | `Safety_CheckCurrent()` | `SAFE` state, relays off |
| Overtemperature (>80 °C warn, >90 °C crit) | `Safety_CheckTemperature()` | `DEGRADED` → `SAFE` |
| CAN heartbeat timeout (>250 ms) | `Safety_CheckCANTimeout()` | `LIMP_HOME` |
| Steering CAN timeout | `Safety_CheckSteeringTimeout()` | Steering fault flag |
| Sensor plausibility | `Safety_CheckSensors()` | `DEGRADED` |
| Encoder health | `Safety_CheckEncoder()` | `DEGRADED` |
| Battery undervoltage (<20 V warn, <18 V crit) | `Safety_CheckBatteryVoltage()` | `DEGRADED` → `SAFE` |
| Battery overvoltage (>30 V warn, >35 V crit) | `Safety_CheckBatteryOvervoltage()` | `DEGRADED` → `SAFE` |
| Relay health (INA226 no-current under demand) | `Safety_CheckRelayHealth()` | `SAFETY_ERROR_RELAY_OPEN` |
| FDCAN bus-off | `CAN_CheckBusOff()` | `DEGRADED` → `ERROR` |
| Obstacle proximity | `Obstacle_Update()` | Torque scale reduction |

**Hardware-level safety (independent of firmware):**
- TIM1/TIM8 `BREAK2` input armed to Cortex-M4 LOCKUP signal — any CPU fault immediately zeroes all PWM outputs via hardware without software intervention.
- IWDG watchdog: ~500 ms timeout. All tiers must execute within the budget or the MCU resets.
- EN pins (×5): driven LOW at the top of `MX_GPIO_Init()` via atomic `GPIOC->BSRR` write before GPIO mode is configured, preventing transient motor activation on warm reset or watchdog reset.

---

## 5. Scheduler / Task Timing

The main loop is a cooperative bare-metal scheduler using `HAL_GetTick()` timestamps. No RTOS is used on the STM32.

```
Power-on
  │
  ├─ MX_GPIO_Init()
  ├─ MX_FDCAN1_Init() + CAN_Init()   ← Early: starts ACKing ESP32 frames immediately
  ├─ Boot LED blinks (×3, 2.3 s)
  ├─ MX_ADC1/I2C1/TIM1/TIM2/TIM3/TIM8/IWDG_Init()
  ├─ Motor_Init(), Traction_Init(), Steering_Init(), Sensor_Init(), Safety_Init()
  ├─ SteeringCal_Init() → optional flash-restore
  ├─ SensorMapStore_Init()
  ├─ Safety_SetState(STANDBY)
  └─ Main loop ──────────────────────────────────────────────────
       │
       ├── every 10 ms (100 Hz):  Safety + Steering PID + Traction
       │     Wheel_UpdateSpeeds(), ABS_Update(), TCS_Update()
       │     Safety_Check*(), Obstacle_Update()
       │     Relay_SequencerUpdate(), SteeringCentering_Step()
       │     Steering_ControlLoop(), Traction_Update()
       │     LoopDiag_RecordTaskUs()
       │
       ├── every 50 ms (20 Hz):   Sensors + Pedal + LIMP_HOME arming
       │     Pedal_Update(), Current_ReadAll()
       │     Temperature_StartConversion(), Temperature_ReadAll()
       │     Startup inhibit update, LIMP_HOME pedal arm update
       │     Traction_SetDemand()
       │
       ├── every 100 ms (10 Hz):  CAN telemetry
       │     Safety_CheckBatteryVoltage/Overvoltage()
       │     CAN_SendHeartbeat/Speed/Current/Safety/Steering/Traction/Battery()
       │
       └── every 1000 ms (1 Hz):  Slow telemetry + housekeeping
             CAN_SendStatusTemp/TempMap/ServiceStatus/DebounceDiag()
             CAN_SendErrorLogHeader/StatusLights()
             Encoder_SendDiagnostic(), Temperature_PeriodicRescan()
             CAN_UpdateFrameRate()
```

**Task timing budget:** The 100 Hz task peak duration is measured with the Cortex-M4 DWT cycle counter and transmitted to the ESP32 in `STATUS_SAFETY` byte 5 (units: 100 µs, cap 255 = 25.5 ms). Budget is 10 ms; readings approaching 8 ms indicate scheduling stress.

---

## 6. Motor Control Pipeline

### Traction Subsystem

```
Pedal input (0–100%)
    │
    ▼
Safety_ValidateThrottle()  ← clamp, rate-limit, anomaly detect
    │
    ▼
Traction_SetDemand()
    │
    ▼
Traction_Update():
    ├── Gear scale: D1 = 70%, D2 = 100%, Reverse = 60%
    ├── EMA filter (α = 0.15, –3 dB ≈ 0.5 Hz at 20 Hz)
    ├── Ramp limiter: up 50 %/s, down 100 %/s
    ├── Demand anomaly detection: step-rate, range, frozen-pedal
    ├── Dead-zone compensation: jump to 8% min when drive active
    ├── Dynamic braking: proportional opposing torque on throttle decrease
    ├── Safety_GetPowerLimitFactor() (per DEGRADED level: L1=70%, L2=50%, L3=40%)
    ├── Safety_GetTractionCapFactor() (per DEGRADED level: L1=80%, L2=60%, L3=50%)
    ├── obstacle_scale (from Obstacle_Update: 0.0–1.0)
    ├── ABS/TCS wheel_scale[4] (per-wheel 0.0–1.0)
    │
    └── Motor_SetSignedPWM_FL/FR/RL/RR()
            └── RPWM/LPWM via TIM1/TIM8 CCR registers
```

**4×4 / 4×2 mode:** In 4×2 mode only the front motors (FL, FR) are driven; rear motors (RL, RR) are coasted (EN=LOW). In 4×4 all four motors track the demand with torque-vectoring via Ackermann geometry.

**Axis rotation (tank turn):** When enabled, FL/RL drive forward at demand% while FR/RR drive backward at demand%. Steering output is zeroed. Enabled/disabled via CAN CMD_MODE bit 1.

**Gear positions (CAN CMD_MODE byte 1):**

| Code | Gear | Power | Notes |
|------|------|-------|-------|
| 0 | PARK | 0% | Hold brake on all wheels |
| 1 | REVERSE | 60% | All wheels backward |
| 2 | NEUTRAL | 0% | Coast / hold brake |
| 3 | FORWARD D1 | 70% | Normal forward |
| 4 | FORWARD D2 | 100% | Full power forward |

### LIMP_HOME Traction

When CAN is lost (>250 ms without ESP32 heartbeat), the system enters `LIMP_HOME`. The local pedal (PA3 ADC) drives traction directly with:
- 20% hard torque cap (`LIMP_HOME_TORQUE_LIMIT_FACTOR = 0.20`)
- 5 km/h speed cap
- 10 %/s ramp rate
- No torque vectoring (all wheels track same demand)
- 300 ms pedal-at-rest arming latch before torque allowed (prevents creep from ADC noise)

---

## 7. Steering System

### Closed-Loop PID Control

Steering uses a PID loop running at 100 Hz. The feedback is the E6B2-CWZ6C quadrature encoder (1200 PPR, 4800 counts/rev) decoded by TIM2 in quadrature mode. The encoder is mounted on the steering column (not road wheel); raw encoder angle is divided by the steering gear ratio (6.48) to compute road-wheel degrees.

**Resolution:** 360° / (4800 counts × gear ratio 6.48) ≈ 0.012° per encoder count at road wheel.

**Angular range:** ±54° road wheel (±MAX_STEER_DEG). Encoder range: ±4800 × 6.48 / (360/54) = ±4663 counts full travel.

### Automatic Centering at Startup

```
BOOT/STANDBY state
    │
    ▼
SteeringCentering_Step() (100 Hz call):
    ├── SWEEP_LEFT: drive at ~10% PWM left until center sensor (PB5) triggers
    ├── SWEEP_RIGHT: if timeout exceeded, reverse direction
    ├── Center sensor FALLING edge on PB5 → stop motor, zero TIM2 counter → DONE
    ├── Fault: stall detected (no encoder delta) or timeout → CENTERING_FAULT
    │            → SAFETY_ERROR_CENTERING, blocks transition to ACTIVE
    └── DONE: Steering_SetCalibrated() → system may enter ACTIVE
```

**Persistent calibration (Flash page 126):** If the stored center value matches the current encoder count within ±100 counts **and** the physical center sensor is active, `SteeringCentering_MarkRestoredFromFlash()` is called, skipping the physical sweep.

### Ackermann Geometry

`Ackermann_ComputeWheelAngles(road_angle, &fl_deg, &fr_deg)` computes differential inner/outer wheel angles to achieve a common turn centre:

```
R       = WHEELBASE_M / tan(|road_angle|)     (0.95 m)
inner   = atan(L / (R - T/2))                 track width = 0.70 m
outer   = atan(L / (R + T/2))
```

Both outputs are clamped to ±54°. In 4×4 mode, the traction split between FL and FR is adjusted based on these computed angles.

---

## 8. ABS / TCS Implementation

Both ABS and TCS run at 100 Hz and operate on per-wheel torque scale factors (`wheel_scale[4]`, range 0.0–1.0).

**ABS — Anti-lock Braking:**
- Detects wheel slip during braking: wheel speed drops significantly below vehicle speed
- Threshold: 15% slip ratio
- Action: reduces `wheel_scale[i]` for the slipping wheel
- Resets when slip is resolved
- `FAULT_ABS_ACTIVE` (bit 5) is set in the CAN heartbeat fault flags
- ABS activity suppresses dynamic braking for that wheel

**TCS — Traction Control:**
- Detects wheel spin during acceleration: wheel speed significantly exceeds vehicle speed
- Threshold: 15% slip ratio
- Action: reduces `wheel_scale[i]` for the spinning wheel
- `FAULT_TCS_ACTIVE` (bit 6) is set in the CAN heartbeat fault flags

Both are individually disableable via Service Mode (`MODULE_ABS = 21`, `MODULE_TCS = 22`).

---

## 9. Dynamic Braking

**What it is:** When the driver releases the throttle rapidly, `motor_control.c` detects the negative demand rate and applies an opposing torque by reversing the RPWM/LPWM direction momentarily. This creates electromagnetic braking by opposing the motor's back-EMF.

**What it is NOT:** This is **not** regenerative braking. The kinetic energy is dissipated as heat in the motor windings. The battery is not charged. There is no energy recovery path in this design.

**Implementation:**
```
demand_rate = (demand - prev_demand) / dt      (%/s)
if demand_rate < 0:
    target_brake = |demand_rate| × DYNBRAKE_FACTOR (0.5), max DYNBRAKE_MAX_PCT (60%)
    effective_demand = -dynbrake_pct  ← opposing direction
```

**BTS7960 brake modes (physical states):**

| Mode | EN | PWM | Motor state |
|------|----|-----|-------------|
| `MOTOR_MODE_COAST` | LOW | 0 | Hi-Z, free-rolling |
| `MOTOR_MODE_DRIVE` | HIGH | >0 | Active torque |
| `MOTOR_MODE_BRAKE` | HIGH | 0 | Short-circuit brake (passive, default) |

A compile-time option `BRAKE_ACTIVE_FALLBACK` (default=0) enables active braking (small LPWM duty during brake mode). A runtime API `Motor_SetBrakeActiveOverride(pwm_ticks)` allows field tuning without recompiling.

**Conditions that suppress dynamic braking:**
- ABS active on any wheel
- Speed below minimum (`DYNBRAKE_MIN_SPEED_KMH`)
- State is BOOT, STANDBY, SAFE, or ERROR
- LIMP_HOME: allowed but capped at 20% of normal

---

## 10. Obstacle Safety System

**Architecture:** Dual-layer. The ESP32 reads the TF-Mini Plus LiDAR sensor at 100 Hz and forwards distance data over CAN (0x208, 0x209). The STM32 is the **primary safety authority** — it runs its own obstacle state machine independent of ESP32 liveness.

**STM32 obstacle state machine (100 Hz):**

```
OBS_STATE_NO_SENSOR  → (first valid CAN frame) → OBS_STATE_NORMAL
OBS_STATE_NORMAL     → (obstacle in range, confirmed) → OBS_STATE_CONFIRMING → OBS_STATE_ACTIVE
OBS_STATE_ACTIVE     → (obstacle receding, confirmed) → OBS_STATE_CLEARING → OBS_STATE_NORMAL
OBS_STATE_ACTIVE     → (data implausible) → OBS_STATE_SENSOR_FAULT
```

**Torque reduction (obstacle_scale):**

| Distance | Zone | obstacle_scale |
|----------|------|---------------|
| < 500 mm | 4 — Emergency | 0.0 (forward blocked) |
| 500–1 000 mm | 3 — Critical | 0.3 |
| 1 000–1 500 mm | 2 — Warning | 0.7 |
| 1 500–2 000 mm | 1 — Caution | 0.85 |
| ≥ 2 000 mm | 0 — Normal | 1.0 |

Obstacle scale is applied uniformly to all wheels in `Traction_Update()`.

**CAN timeout safety:** If no obstacle frame is received for >500 ms, `obstacle_scale` is set to 1.0 (no restriction). CAN loss does not trigger an obstacle-emergency. LIMP_HOME speed cap (5 km/h) provides the safety net when CAN is lost.

**What is NOT implemented:** Adaptive cruise control (maintaining a set following distance automatically). This would require an additional control loop in the traction pipeline. It is listed as a future work item in `docs/FIRMWARE_MATURITY_ROADMAP.md`.

---

## 11. CAN Protocol Overview

- **Standard:** CAN 2.0A (classic, 11-bit IDs)
- **Bitrate:** 500 kbps
- **Peripheral:** STM32 FDCAN1 (PA11 RX, PA12 TX, AF9). ESP32 uses ESP32-TWAI-CAN library.
- **Transceivers:** TJA1051T/3 on both nodes (STM32 and ESP32), powered at 3.3V in current installation
- **Termination:** 120 Ω at each end
- **Contract:** `docs/CAN_CONTRACT_FINAL.md` rev 1.4 (2026-05-01) — frozen, versioned

### Message Table

| ID | Dir | DLC | Rate | Description |
|----|-----|-----|------|-------------|
| 0x001 | STM32→ESP32 | 6 | 100 ms | HEARTBEAT_STM32: counter, state, faultFlags, errorCode, statusFlags, relayStatus |
| 0x011 | ESP32→STM32 | 1 | 100 ms | HEARTBEAT_ESP32: rolling counter (frozen counter → CAN timeout) |
| 0x100 | ESP32→STM32 | 1 | 50 ms | CMD_THROTTLE: throttle 0–100% |
| 0x101 | ESP32→STM32 | 2 | 50 ms | CMD_STEERING: angle (signed int16, ×10 = degrees) |
| 0x102 | ESP32→STM32 | 2 | on-demand | CMD_MODE: byte0=mode flags (bit0=4×4, bit1=tank), byte1=gear |
| 0x103 | STM32→ESP32 | 3 | on-demand | CMD_ACK: cmdId, result (0=OK,1=rejected,2=invalid,3=blocked), state |
| 0x110 | ESP32→STM32 | 2 | on-demand | SERVICE_CMD: byte0=action, byte1=moduleId/mask |
| 0x112 | ESP32→STM32 | 5 | on-demand | CMD_SENSOR_MAP_TEMP: DS18B20 physIdx→role mapping |
| 0x120 | ESP32→STM32 | 2 | on-demand | CMD_LED: byte0=front relay, byte1=rear relay |
| 0x130 | ESP32→STM32 | 0–1 | on-demand | CMD_SYSTEM_SHUTDOWN: pre-power-cut safe-state request |
| 0x200 | STM32→ESP32 | 8 | 100 ms | STATUS_SPEED: FL/FR/RL/RR wheel speeds (×10, km/h, uint16 LE) |
| 0x201 | STM32→ESP32 | 8 | 100 ms | STATUS_CURRENT: FL/FR/RL/RR motor current (×100, A, uint16 LE) |
| 0x202 | STM32→ESP32 | 5 | 1000 ms | STATUS_TEMP: 5 temperatures (int8, °C) |
| 0x203 | STM32→ESP32 | 6 | 100 ms | STATUS_SAFETY: abs, tcs, error_code, state, rx_errors, loop_peak_100us |
| 0x204 | STM32→ESP32 | 3 | 100 ms | STATUS_STEERING: angle (int16 ×10), calibrated flag |
| 0x205 | STM32→ESP32 | 4 | 100 ms | STATUS_TRACTION: per-wheel ABS/TCS scale (×100, 0–100%) |
| 0x206 | STM32→ESP32 | 5 | 1000 ms | STATUS_TEMP_MAP: mapped temps by role (FL/FR/RL/RR/Ambient) |
| 0x207 | STM32→ESP32 | 4 | 100 ms | STATUS_BATTERY: 24 V bus current (int16 ×100, A) + voltage (uint16 ×100, V) |
| 0x208 | ESP32→STM32 | 5 | 66 ms | OBSTACLE_DISTANCE: mm (uint16 LE), zone (0–4), health, rolling counter |
| 0x209 | ESP32→STM32 | 4 | 100 ms | OBSTACLE_SAFETY: zone, sensor status, stuck flag, reserved |
| 0x20A | STM32→ESP32 | 2 | 1000 ms | STATUS_LIGHTS: front relay state, rear relay state |
| 0x300 | Both | 2 | on-demand | DIAG_ERROR: error_code, subsystem (0=global,1=motor,2=sensor,3=CAN) |
| 0x301 | STM32→ESP32 | 4 | 1000 ms | SERVICE_FAULTS: module fault bitmask |
| 0x302 | STM32→ESP32 | 4 | 1000 ms | SERVICE_ENABLED: module enabled bitmask |
| 0x303 | STM32→ESP32 | 4 | 1000 ms | SERVICE_DISABLED: module disabled bitmask |
| 0x304 | STM32→ESP32 | 8 | on-demand | ERROR_LOG_ENTRY: timestamp, error_code, subsystem, state, flags, uptime |
| 0x305 | STM32→ESP32 | 8 | 1000 ms | ERROR_LOG_HEADER: entry count + total events |
| 0x306 | STM32→ESP32 | 8 | 1000 ms | DIAG_DEBOUNCE: 4× wheel DWT-filtered edge-reject counts (uint16 LE) |
| 0x307 | STM32→ESP32 | 4 | 1000 ms | DIAG_DEBOUNCE_STEER: steering center DWT-filtered rejects (uint32 LE) |

**Relay status byte** (HEARTBEAT_STM32 byte 5):

| Bit | Meaning |
|-----|---------|
| 0 | Reserved (always 0) |
| 1 | TRACTION relay (PC11) — 1 = commanded ON |
| 2 | DIRECTION relay (PC12) — 1 = commanded ON |
| 7 | Relay sequence complete flag |

**CAN Fault Flags** (HEARTBEAT_STM32 byte 2):

| Bit | Flag |
|-----|------|
| 0 | CAN_TIMEOUT |
| 1 | TEMP_OVERLOAD |
| 2 | CURRENT_OVERLOAD |
| 3 | ENCODER_ERROR |
| 4 | WHEEL_SENSOR |
| 5 | ABS_ACTIVE |
| 6 | TCS_ACTIVE |
| 7 | CENTERING |

**Safety error codes** (STATUS_SAFETY byte 2):

| Code | Error |
|------|-------|
| 0 | NONE |
| 1 | OVERCURRENT |
| 2 | OVERTEMP |
| 3 | CAN_TIMEOUT |
| 4 | SENSOR_FAULT |
| 5 | MOTOR_STALL (reserved, not implemented) |
| 6 | EMERGENCY_STOP |
| 7 | WATCHDOG |
| 8 | CENTERING |
| 9 | BATTERY_UV_WARNING |
| 10 | BATTERY_UV_CRITICAL |
| 11 | I2C_FAILURE |
| 12 | OBSTACLE |
| 13 | CAN_BUSOFF |
| 14 | BATTERY_OV_WARNING |
| 15 | BATTERY_OV_CRITICAL |
| 16 | RELAY_OPEN |

---

## 12. Wheel Speed Acquisition

Four wheel speed sensors (Hall effect, active-low output, isolated through EL817 NPN optocoupler modules) connect to STM32 EXTI pins.

| Wheel | Pin | EXTI | Edge | Pull |
|-------|-----|------|------|------|
| FL | PA0 | EXTI0 | RISING | PULLUP |
| FR | PA1 | EXTI1 | RISING | PULLUP |
| RL | PA2 | EXTI2 | RISING | PULLUP |
| RR | PB15 | EXTI15 | RISING | PULLUP |

**Two-stage EMI debounce:**
1. **DWT pre-filter (200 µs):** Uses the Cortex-M4 DWT cycle counter in the ISR. Pulses separated by less than 200 µs are silently rejected. At 25 km/h (max speed) the pulse period is ~26 ms — 200 µs is 0.77% of the period, so no valid pulses are lost.
2. **HAL_GetTick filter (1 ms):** Secondary coarser filter as a backup.

**Wheel speed computation (`Wheel_UpdateSpeeds()`, called at 100 Hz):**
- Accumulates pulse count and timing between calls
- Speed (km/h) = (pulse_count / WHEEL_PULSES_REV) × WHEEL_CIRCUM_M × (1/dt) × 3.6
- `WHEEL_PULSES_REV = 6` (6 bolts/hub), `WHEEL_CIRCUM_MM = 1100 mm`
- Output clamped to 40 km/h maximum (`WHEEL_SPEED_CLAMP_KMH`)
- Stale detection: if no pulse for >500 ms, speed forced to 0

Rejected pulse counts (EMI noise) are transmitted to the ESP32 engineering screen via CAN 0x306/0x307.

---

## 13. Encoder Handling

The E6B2-CWZ6C encoder (1200 PPR) is mounted on the steering column and galvanically isolated via dual 6N137 high-speed optocouplers (A and B channels). The Z index pulse also passes through a 6N137.

| Signal | Pin | Peripheral | Mode |
|--------|-----|-----------|------|
| ENC_A | PA15 | TIM2_CH1 | Quadrature (AF1) |
| ENC_B | PB3 | TIM2_CH2 | Quadrature (AF1) |
| ENC_Z | PB4 | EXTI4 | FALLING edge, PULLUP |

**TIM2 quadrature mode:** Counts A×B edges: 1200 PPR × 4 = 4800 counts/revolution, resolution 0.075°/count at the encoder shaft.

**Z-channel (index pulse):** Not used for centering. Used exclusively for inter-revolution drift detection (`encoder_reader.c`). At each Z pulse, the expected count change is ±4800. A deviation >threshold flags encoder slippage. Debounce: minimum 200 µs between accepted Z pulses (DWT).

**Steering angle:** `Steering_GetCurrentAngle()` = `TIM2->CNT` / `ENCODER_CPR` × 360 / `STEERING_GEAR_RATIO` = count / 4800 × 360 / 6.48 (degrees, road-wheel).

---

## 14. Current Sensing — INA226

Six INA226 power monitors connected to I2C1 (PB6/PB7) through a TCA9548A I2C multiplexer (address 0x70). The INA226 base address on each multiplexer channel is 0x40.

| Channel | Role | Shunt | Full scale |
|---------|------|-------|-----------|
| 0 | Motor FL | 1.5 mΩ | 50 A / 75 mV |
| 1 | Motor FR | 1.5 mΩ | 50 A / 75 mV |
| 2 | Motor RL | 1.5 mΩ | 50 A / 75 mV |
| 3 | Motor RR | 1.5 mΩ | 50 A / 75 mV |
| 4 | Battery bus 24 V | 0.75 mΩ | 100 A / 75 mV |
| 5 | Steering motor | 1.5 mΩ | 50 A / 75 mV |

Channel 4 (battery) is placed **before** the main traction relay, so `Voltage_GetBus(4)` returns battery voltage regardless of relay state. Motor channels allow bidirectional current measurement (back-EMF / dynamic braking).

**I2C recovery:** On I2C SDA stuck-low, the firmware cycles up to 9 SCL pulses (NXP AN10216 procedure) to unlock the bus.

---

## 15. Temperature Monitoring — DS18B20

Five DS18B20 OneWire temperature sensors on PB0 (open-drain, pull-up, high-speed). All five sensors share one bus. ROM enumeration occurs at boot and periodically re-scans for hot-plug changes.

**Sensor roles** (configurable via CAN CMD_SENSOR_MAP_TEMP 0x112 from engineering menu):
- FL motor, FR motor, RL motor, RR motor, Ambient

**Thresholds:**
- Warning: 80 °C → `FAULT_TEMP_OVERLOAD` bit, `DEGRADED` state
- Critical: 90 °C → `SAFETY_ERROR_OVERTEMP`, `SAFE` state

Diagnostic flags: stale detection (frozen sensor), topology change (sensor added/removed), CRC validity. Transmitted to ESP32 via `STATUS_TEMP` (0x202) and `STATUS_TEMP_MAP` (0x206).

---

## 16. Battery Monitoring

Battery voltage is measured by INA226 channel 4 (placed before the traction relay) and transmitted in `STATUS_BATTERY` (0x207).

| Threshold | Action |
|-----------|--------|
| < 20.0 V | `SAFETY_ERROR_BATTERY_UV_WARNING` → `DEGRADED` |
| < 18.0 V | `SAFETY_ERROR_BATTERY_UV_CRITICAL` → `SAFE` |
| > 30.0 V | `SAFETY_ERROR_BATTERY_OV_WARNING` → `DEGRADED` |
| > 35.0 V | `SAFETY_ERROR_BATTERY_OV_CRITICAL` → `SAFE` |

Recovery hysteresis prevents state flapping. Battery overvoltage has no auto-recovery from SAFE — the operator must manually investigate the source (charger malfunction, dynamic braking back-EMF, regulator failure) before clearing.

---

## 17. Safety States

```
             BOOT
               │
               ▼
           STANDBY ───────────────────────────────────────────────┐
             │  │                                                  │
    HB+cal+boot │ HB timeout (CAN lost)                           │
    valid+no err│                                                  ▼
             │  └──────────────────────────────────────────► LIMP_HOME
             ▼                                                     │
           ACTIVE ◄────────────────────────────── fault cleared+HB restored
             │  │
    non-crit │  │ CAN timeout (comm loss)
    fault    │  └──────────────────────────────────────────► LIMP_HOME
             ▼                                                     │
          DEGRADED ◄──── fault cleared + 500 ms debounce ─────────┘
             │  │
    crit or  │  │ CAN timeout
    ≥3 errors│  └──────────────────────────────────────────► LIMP_HOME
             ▼
           SAFE ──── fault cleared + HB restored ──────────► ACTIVE
             │
             │ unrecoverable fault
             ▼
           ERROR  (terminal — physical reset required)
```

**State behaviours:**

| State | CAN commands | Traction | Steering | Relays |
|-------|-------------|---------|---------|--------|
| BOOT | No | Inhibited | Centering sweep | OFF |
| STANDBY | No | Inhibited | Centering sweep | OFF |
| ACTIVE | Yes | Full | Full | ON |
| DEGRADED | Yes (limited) | L1–L3 limits | L1–L3 limit | ON |
| LIMP_HOME | Ignored | Local pedal, 20% cap | Operational | ON (stay) |
| SAFE | No | Inhibited | Neutralised | OFF |
| ERROR | No | Inhibited | Neutralised | OFF |

**Granular DEGRADED levels:**

| Level | Power | Steering | Traction cap | Triggered by |
|-------|-------|---------|-------------|-------------|
| L1 | 70% | 85% | 80% | Sensor fault, minor anomaly |
| L2 | 50% | 70% | 60% | Thermal warning, single overcurrent |
| L3 | 40% | 60% | 50% | Persistent multiple faults |

---

## 18. Emergency Stop Behavior

`Safety_EmergencyStop()` / `Traction_EmergencyStop()`:
1. Immediately sets all motor PWM to 0 (RPWM=0, LPWM=0)
2. Sets all EN pins LOW (motors coast)
3. Calls `Relay_PowerDown()` (both traction and direction relays open)
4. Transitions to `SYS_STATE_SAFE` or `SYS_STATE_ERROR`

This function is idempotent. It is also triggered via CAN `CMD_SYSTEM_SHUTDOWN` (0x130) for a coordinated pre-power-cut sequence: ESP32 sends the frame when the ignition key is turned off, giving the STM32 time to neutralise before the hardware delay relay cuts mains power.

---

## 19. PWM Generation Architecture

All motor PWM is generated by hardware timers in center-aligned mode (mode 3):

| Timer | Frequency | Channels | Motors |
|-------|-----------|---------|--------|
| TIM1 (advanced) | 20 kHz | CH1 (PA8), CH2 (PA9), CH3 (PA10), CH4 (PC3) | FL (RPWM/LPWM), FR (RPWM/LPWM) |
| TIM8 (advanced) | 20 kHz | CH1 (PC6), CH2 (PC7), CH3 (PC8), CH4 (PC9) | RL (RPWM/LPWM), RR (RPWM/LPWM) |
| TIM3 (general) | 20 kHz | CH1 (PA6), CH2 (PA7) | Steering (RPWM/LPWM) |

**ARR = 4249** → 170 MHz / (2 × 4250) = 20 kHz. Resolution: 4250 steps (≈ 12.1 bit).

**TIM1/TIM8 BREAK2:** Armed to the Cortex-M4 LOCKUP signal. If the CPU enters LOCKUP (infinite fault loop), hardware immediately forces all TIM1/TIM8 outputs to their break state (PWM = 0), preventing motor runaway without any software involvement.

**RPWM/LPWM mutual exclusion:** `Motor_SetSignedPWM_*()` always writes the inactive channel to 0 before setting the active channel. RPWM and LPWM are never simultaneously non-zero.

---

## 20. BTS7960 (IBT-2) Driver Handling

Each IBT-2 module is driven with 5 signals: RPWM, LPWM, R\_EN (tied to L\_EN = EN), GND, VCC.

**⚠ VCC must be 3.3 V, NOT 5 V.** The IBT-2 includes a 74HC244 buffer whose V\_IH(min) = 0.7 × VCC. At 5 V, V\_IH(min) = 3.5 V, which exceeds the STM32's 3.3 V GPIO level. At 3.3 V, V\_IH(min) = 2.31 V — safely below 3.3 V. See `project_config.h` §A for full rationale.

**EN pin safety:** All five EN pins (PC0, PC1, PC2, PC4, PC5) are forced LOW via atomic `GPIOC->BSRR` at the start of `MX_GPIO_Init()`, before GPIO mode is set. This prevents transient motor activation on warm reset or watchdog reset.

**BTS7960 current sense (R\_IS / L\_IS):** These analog outputs are **not connected** to the STM32 ADC. Current monitoring is provided entirely by the INA226 sensors on I2C.

---

## 21. Touchscreen / Display System

**Display:** ST7796 480×320 landscape, HSPI (SPI3\_HOST), 40 MHz

**Touch controller:** XPT2046 resistive touch, SPI 2.5 MHz, shared MISO bus (GPIO 12)

**Library:** TFT\_eSPI v2.5.43 (Bodmer). Configuration injected via `include/User_Setup.h` force-included through PlatformIO `build_flags`.

### Touch Calibration

A full three-layer system is implemented:

1. **Compile-time fallback** (`User_Setup.h` `TOUCH_CALIBRATION` macro):
   - `{ 320, 3470, 230, 3280, 7 }` — averaged values from production panel measurements
   - Used when NVS has no valid calibration (first boot or after factory reset)
   - Accurate enough to tap wizard buttons

2. **Interactive wizard** (`touch_calibration_screen.cpp/h`):
   - Full guided screen to capture XPT2046 calibration using `TFT_eSPI::calibrateTouch()`
   - Accessible from the engineering menu (PIN 8989)
   - Also runs automatically on first boot if no NVS calibration exists

3. **NVS persistent storage** (`touch_calibration.cpp/h`):
   - Namespace: `touch_cal`, key: `data` (20-byte blob)
   - Format: `{ magic 0x54434C31, xMin, xMax, yMin, yMax, rotation, _pad, _pad2, crc32 }`
   - Validation: magic + CRC32/ISO-HDLC + range checks (xMax/yMax ≥ 1000, offsets ≤ 4095, flags ≤ 7)
   - `first_done` NVS flag prevents wizard from re-launching on every boot after first calibration
   - `factoryReset()` erases both data blob and `first_done` flag to re-arm the wizard

### Screen State Machine

| Screen | Entry Condition |
|--------|----------------|
| `boot_screen` | Startup, waiting for CAN |
| `pin_screen` | Entering engineering menu (PIN 8989) |
| `standby_screen` | State = STANDBY |
| `drive_screen` | State = ACTIVE or DEGRADED |
| `safe_screen` | State = SAFE |
| `error_screen` | State = ERROR |
| `engineering_screen` | PIN accepted |
| `touch_calibration_screen` | From engineering menu |

---

## 22. Audio / DFPlayer Mini Integration

- **Module:** DFPlayer Mini (UART 9600 bps)
- **Pins:** GPIO 43 (ESP32 TX → DFPlayer RX), GPIO 44 (DFPlayer TX → ESP32 RX)
- **SD card:** Numbered MP3 files 0001.mp3–0068.mp3
- **Speaker relay:** GPIO 11 (active LOW) — 150 ms delay on, 150 ms delay off, prevents pop/click

**Priority queue (single-slot with preemption):**

| Priority | Events |
|----------|--------|
| LOW (0) | Gear clicks, info beeps, mode changes |
| MEDIUM (1) | Obstacle warnings, battery alerts, temperature warnings |
| HIGH (2) | Errors, emergency, welcome, farewell |

Higher-priority sounds preempt currently playing lower-priority sounds. Per-sound cooldown: 4 000 ms (bypassed for HIGH priority). All `audio::play()` calls are non-blocking.

**Key events triggered over CAN:**
- System startup (WELCOME), shutdown (FAREWELL)
- Gear change: PARK, REVERSE, NEUTRAL, D1, D2
- ABS active/cleared, TCS active/cleared
- Obstacle detected (OBSTACLE\_WARN)
- Battery low/critical (BATTERY\_LOW, BATTERY\_CRITICAL)
- Safety error (EMERGENCY, ERROR\_GENERAL)
- 4×4 / 4×2 traction mode change
- LED lights on/off

---

## 23. GPIO Expander — MCP23017 (Shifter)

The physical gear selector (P/R/N/D1/D2) uses an MCP23017 I2C GPIO expander.

- **I2C address:** 0x20 (default)
- **Bus:** ESP32 I2C on GPIO 8 (SDA), GPIO 9 (SCL)
- **Port:** Port A, active-low with internal pull-ups (one bit per gear position)
- **Polling:** 50 ms interval (`shifter_input.cpp`)
- **Gear enum:** PARK=0, REVERSE=1, NEUTRAL=2, FORWARD=3, FORWARD\_D2=4 — matches STM32 `GearPosition_t`
- **Safe default:** NEUTRAL when MCP23017 does not respond on I2C
- **CAN:** Gear value transmitted in CMD_MODE (0x102) byte 1 on change

---

## 24. Persistent Storage (Flash / NVS)

### STM32 Flash Layout

| Flash page | Address | Size | Content |
|-----------|---------|------|---------|
| Page 125 | 0x0807C000 | 4 KB | Error log ring buffer (250 × 16-byte entries + CRC32 header) |
| Page 126 | 0x0807E000 | 4 KB | Steering calibration (magic + encoder center + CRC32) |

Both use magic numbers and CRC32 integrity checks. Flash is erased and reformatted if magic or CRC fails at boot.

**Steering calibration (Flash page 126):**
- `SteeringCal_Save(encoder_count_at_center)` — called once centering completes
- `SteeringCal_ValidateAtBoot()` — checks: valid CRC, |current_encoder – stored_center| ≤ 100 counts, AND physical center sensor is active
- On success: centering sweep is skipped at boot (fast startup)
- Safety invariant: flash alone never authorises ACTIVE — the center sensor must also agree

**DS18B20 sensor map (Flash page 125, shared section):**
- Stores user-assigned physical index → role mapping (FL/FR/RL/RR/Ambient)
- Configured via engineering menu → CAN CMD_SENSOR_MAP_TEMP (0x112)
- Identity mapping used as fallback if no valid map is stored

### ESP32 NVS Namespaces

| Namespace | Content |
|-----------|---------|
| `touch_cal` | XPT2046 touch calibration (magic + CRC32 + first\_done flag) |
| `hmi_cfg` | HMI settings, fault log, config |

---

## 25. Service Mode

25 modules with two classifications:

**Critical (4 modules — cannot be disabled):**
`MODULE_CAN_TIMEOUT`, `MODULE_EMERGENCY_STOP`, `MODULE_WATCHDOG`, `MODULE_RELAY_TRAC`

**Non-critical (21 modules — individually disableable via engineering menu):**
`MODULE_TEMP_SENSOR_{0–4}`, `MODULE_CURRENT_SENSOR_{0–5}`, `MODULE_WHEEL_SPEED_{FL/FR/RL/RR}`, `MODULE_STEER_CENTER`, `MODULE_STEER_ENCODER`, `MODULE_ABS`, `MODULE_TCS`, `MODULE_ACKERMANN`, `MODULE_OBSTACLE_DETECT`

**Fault states:** NONE / WARNING / ERROR / DISABLED. Faults are always recorded even when a module is disabled.

**Service commands** (CAN SERVICE_CMD 0x110):

| Action byte | Effect |
|-------------|--------|
| 0x00 | Disable module |
| 0x01 | Enable module |
| 0xE0 | Relay override (diagnostic, STANDBY only) |
| 0xF0 | Reset steering PID to factory defaults |
| 0xF1 | Reset wheel sensor calibration |
| 0xF2 | Reset INA226 shunt values |
| 0xF3 | Reset traction motor force parameters |
| 0xF4 | Reset steering motor force parameters |
| 0xFE | Clear error log |
| 0xFF | Factory restore (re-enable all modules) |

---

## 26. Error Log

Flash page 125 ring buffer. Survives power cycles, brownouts, and watchdog resets.

- **Capacity:** 250 entries
- **Entry size:** 16 bytes: `{timestamp_ms, error_code, subsystem, system_state, fault_flags, reset_cause, i2c_fail_count, reserved[2], uptime_sec}`
- **Monotonic counter:** `total_events` is never reset even after `ErrorLog_Clear()`
- **CAN retrieval:** `ERROR_LOG_HEADER` (0x305) → count; `ERROR_LOG_ENTRY` (0x304) → individual entries (on-demand, fetched from engineering screen)

---

## 27. Boot Sequence

```
1. HAL_Init(), SystemClock_Config() → 170 MHz PLL
2. Boot_ReadResetCause() ← must run before IWDG clears flags
3. MX_GPIO_Init() ← EN pins forced LOW first (atomic BSRR)
4. MX_FDCAN1_Init() + CAN_Init() ← early: STM32 ACKs ESP32 CAN frames within ~50 ms
   (boot_phase = 1)
5. Boot LED blinks ×3 on LD2 (PA5) — ~2.3 s — FDCAN running, ACKing
6. MX_ADC1/I2C1/TIM1/TIM2/TIM3/TIM8/IWDG_Init()
   (boot_phase = 2)
7. Motor_Init(), Traction_Init(), Steering_Init(), Sensor_Init()
8. Safety_Init(), ServiceMode_Init()
9. ErrorLog_Init(), ErrorLog_SetResetCause()
   (boot_phase = 3)
10. Post-init CAN status LED: 1 long blink (CAN OK) or 5 rapid blinks (CAN FAIL)
11. SteeringCal_Init() → SteeringCal_ValidateAtBoot() → optional flash-restore
12. SensorMapStore_Init()
13. Safety_SetState(STANDBY)
    (boot_phase = 4)
14. Drain CAN FIFO (discard boot-time messages), reset overflow counter
15. Main loop entered
    (boot_phase = 5)
```

**Startup inhibit:** Torque is blocked from boot until the pedal is held below 3% for 400 ms continuously. Independent of state machine.

**LIMP_HOME pedal arming:** On entering LIMP_HOME, an additional 300 ms pedal-at-rest latch must be satisfied before torque is allowed (prevents ADC noise from causing unwanted motion when CAN is first lost).

---

## 28. STM32 GPIO / Pin Map

> **Source of truth:** `Core/Inc/project_config.h`

### Motor PWM Outputs

| GPIO | Timer | Signal | Description |
|------|-------|--------|-------------|
| PA8 | TIM1_CH1 | RPWM_FL | Front Left forward |
| PA9 | TIM1_CH2 | LPWM_FL | Front Left reverse |
| PA10 | TIM1_CH3 | RPWM_FR | Front Right forward |
| PC3 | TIM1_CH4 | LPWM_FR | Front Right reverse |
| PC6 | TIM8_CH1 | RPWM_RL | Rear Left forward |
| PC7 | TIM8_CH2 | LPWM_RL | Rear Left reverse |
| PC8 | TIM8_CH3 | RPWM_RR | Rear Right forward |
| PC9 | TIM8_CH4 | LPWM_RR | Rear Right reverse |
| PA6 | TIM3_CH1 | RPWM_STEER | Steering forward |
| PA7 | TIM3_CH2 | LPWM_STEER | Steering reverse |

### Motor Enable Signals

| GPIO | Signal | Notes |
|------|--------|-------|
| PC5 | EN_FL | Push-pull out, active HIGH |
| PC0 | EN_FR | Push-pull out, active HIGH |
| PC1 | EN_RL | Push-pull out, active HIGH |
| PC2 | EN_RR | Push-pull out, active HIGH (moved from PC13 — conflicts with USER button B1) |
| PC4 | EN_STEER | Push-pull out, active HIGH |
| PC10 | (unused) | INPUT_PULLDOWN — free GPIO, not wired |

> All EN pins forced LOW via `GPIOC->BSRR` at the start of `MX_GPIO_Init()` before mode configuration.

### Power Relays

| GPIO | Signal | Load |
|------|--------|------|
| PC11 | RELAY_TRAC | 24 V → 4× BTS7960 traction motors |
| PC12 | RELAY_DIR | 12 V → steering BTS7960 |
| PB10 | RELAY_LED_FRONT | 5 V → front WS2812B strip (28 LEDs) |
| PB11 | RELAY_LED_REAR | 5 V → rear WS2812B strip (16 LEDs) |

> Only two power relays: PC11 and PC12. PC10 is free and unused.

### Wheel Speed Sensors (EXTI — Rising edge, Pull-up)

| GPIO | EXTI | Wheel |
|------|------|-------|
| PA0 | EXTI0 | Front Left |
| PA1 | EXTI1 | Front Right |
| PA2 | EXTI2 | Rear Left |
| PB15 | EXTI15 | Rear Right |

### Steering Encoder (TIM2 Quadrature + Z Index)

| GPIO | Peripheral | Signal | Notes |
|------|-----------|--------|-------|
| PA15 | TIM2_CH1 (AF1) | ENC_A | Via 6N137 |
| PB3 | TIM2_CH2 (AF1) | ENC_B | Via 6N137 |
| PB4 | EXTI4 (Falling, Pull-up) | ENC_Z | Index pulse, via 6N137 |

### Steering Center Sensor

| GPIO | Peripheral | Signal | Notes |
|------|-----------|--------|-------|
| PB5 | EXTI5 (Falling, Pull-up) | STEER_CENTER | LJ12A3 inductive, detects mechanical center screw |

### I2C (INA226 via TCA9548A)

| GPIO | Peripheral | Signal |
|------|-----------|--------|
| PB6 | I2C1_SCL | I2C clock |
| PB7 | I2C1_SDA | I2C data |

### OneWire (DS18B20)

| GPIO | Mode | Signal |
|------|------|--------|
| PB0 | OUTPUT_OD, Pull-up, High-speed | OneWire bus |

### CAN Bus (FDCAN1)

| GPIO | Peripheral | Signal |
|------|-----------|--------|
| PA11 | FDCAN1_RX (AF9) | CAN receive |
| PA12 | FDCAN1_TX (AF9) | CAN transmit |

### ADC

| GPIO | Peripheral | Signal | Notes |
|------|-----------|--------|-------|
| PA3 | ADC1_IN4 | Pedal accelerator | Via 10kΩ/6.8kΩ divider from 5 V Hall sensor |

### Indicators

| GPIO | Signal | Notes |
|------|--------|-------|
| PA5 | LD2 (on-board green LED) | Boot/CAN status heartbeat |
| PB14 | LED_DIAG (external) | Freed from TIM15, diagnostic LED |

### SWD Debug

| GPIO | Signal |
|------|--------|
| PA13 | SWDIO |
| PA14 | SWDCLK |

---

## 29. ESP32-S3 GPIO / Pin Map

> **Source of truth:** `esp32/include/User_Setup.h`, `esp32/src/audio_manager.h`, `esp32/src/power_manager.h`, `esp32/src/shifter_input.h`, `esp32/src/sensors/obstacle_sensor.h`, `esp32/platformio.ini`

### TFT Display (ST7796, HSPI/SPI3)

| GPIO | Signal | Notes |
|------|--------|-------|
| 13 | TFT_MOSI | SPI MOSI |
| 12 | TFT_MISO | SPI MISO (shared with XPT2046) |
| 14 | TFT_SCLK | SPI clock |
| 10 | TFT_CS | Display chip select |
| 39 | TFT_DC | Data/command |
| 38 | TFT_RST | Display reset |
| 42 | TFT_BL | Backlight (HIGH=on) |

### Touch Controller (XPT2046)

| GPIO | Signal | Notes |
|------|--------|-------|
| 21 | TOUCH_CS | XPT2046 chip select |

### CAN Bus (TJA1051T/3, ESP32-TWAI)

| GPIO | Signal |
|------|--------|
| 4 | CAN_TX |
| 5 | CAN_RX |

### Audio (DFPlayer Mini)

| GPIO | Signal | Notes |
|------|--------|-------|
| 43 | DFPlayer_TX | ESP32 TX → DFPlayer RX |
| 44 | DFPlayer_RX | DFPlayer TX → ESP32 RX |
| 11 | Audio relay | Active LOW, prevents pop/click |

### Gear Selector (MCP23017)

| GPIO | Signal | Notes |
|------|--------|-------|
| 8 | I2C_SDA | MCP23017 addr 0x20 |
| 9 | I2C_SCL | MCP23017 addr 0x20 |

### Obstacle Sensor (TF-Mini Plus, UART1)

| GPIO | Signal | Notes |
|------|--------|-------|
| 18 | UART1_RX | TF-Mini Plus TX → ESP32 RX; 115 200 bps, no TX needed |

### WS2812B LED Strips (FastLED)

| GPIO | Signal | Notes |
|------|--------|-------|
| 47 | LED_FRONT | Front strip, 28 LEDs |
| 48 | LED_REAR | Rear strip, 16 LEDs |

### Power Management (EL817 optocoupler)

| GPIO | Signal | Notes |
|------|--------|-------|
| 40 | Ignition sense | INPUT_PULLUP + external 10kΩ to 3.3V; LOW=key ON (NPN active-low) |
| 41 | Power hold | HIGH = keep ESP32 alive during shutdown sequence |

### Traction Switch

| GPIO | Signal | Notes |
|------|--------|-------|
| 15 | 2WD/4WD toggle | Physical switch, drives CMD_MODE flag |

### Reserved / Forbidden Pins

| GPIO Range | Reason |
|-----------|--------|
| 26–32 | QSPI Flash bus — boot failure risk if used |
| 33–37 | Octal PSRAM bus (N16R8) — memory crash risk if used |
| 22–25 | Do not exist in ESP32-S3 SoC |
| 0, 3 | Strapping pins — safe as GPIO but care required at boot |
| 45, 46 | Strapping pins — GPIO 45 controls VDD\_SPI voltage |

---

## 30. Folder Structure

```
STM32-Control-Coche-Marcos/
├── Core/
│   ├── Inc/                         STM32 headers
│   │   ├── project_config.h         ← Pin map, constants (source of truth)
│   │   ├── vehicle_physics.h        ← Wheelbase, track width, gear ratios
│   │   ├── motor_control.h
│   │   ├── safety_system.h
│   │   ├── can_handler.h
│   │   ├── sensor_manager.h
│   │   ├── steering_centering.h
│   │   ├── steering_cal_store.h
│   │   ├── ackermann.h
│   │   ├── encoder_reader.h
│   │   ├── service_mode.h
│   │   ├── error_log.h
│   │   ├── boot_validation.h
│   │   ├── loop_diag.h
│   │   ├── math_safety.h
│   │   ├── eps_params.h
│   │   └── sensor_map_store.h
│   └── Src/                         STM32 implementation
│       ├── main.c                   ← Entry point + multi-tier scheduler
│       ├── motor_control.c
│       ├── safety_system.c
│       ├── can_handler.c
│       ├── sensor_manager.c
│       ├── steering_centering.c
│       ├── steering_cal_store.c
│       ├── ackermann.c
│       ├── encoder_reader.c
│       ├── service_mode.c
│       ├── error_log.c
│       ├── boot_validation.c
│       ├── loop_diag.c
│       ├── math_safety.c
│       ├── eps_params.c
│       ├── sensor_map_store.c
│       ├── test_*.c                 ← Host-side unit tests (excluded from firmware build)
│       └── stm32g4xx_*.c            ← HAL support files
├── Drivers/                         STM32 HAL + CMSIS (generated by CubeMX)
├── esp32/
│   ├── platformio.ini               ← PlatformIO: esp32s3, Arduino, C++17, 16 MB flash
│   ├── include/
│   │   ├── can_ids.h                ← CAN contract (frozen, rev 1.4)
│   │   └── User_Setup.h             ← TFT_eSPI config (force-included)
│   └── src/
│       ├── main.cpp
│       ├── screens/                 boot/standby/drive/safe/error/engineering/pin/touch_cal
│       ├── ui/                      Rendering components
│       ├── hmi/                     HMI logic
│       ├── sensors/                 obstacle_sensor (TF-Mini Plus)
│       ├── can/                     CAN Rx decoder
│       ├── audio_manager.*          DFPlayer Mini
│       ├── relay_audio.*            Speaker relay state machine
│       ├── power_manager.*          Ignition key + shutdown
│       ├── touch_calibration.*      NVS XPT2046 persistence
│       ├── touch_handler.*          TFT_eSPI touch integration
│       ├── shifter_input.*          MCP23017 gear selector
│       ├── traction_switch.*        2WD/4WD toggle
│       ├── vehicle_data.*           Live telemetry store
│       └── config_store.*           NVS HMI settings
├── docs/                            100+ technical reference documents
│   ├── CAN_CONTRACT_FINAL.md        ← Authoritative CAN protocol (rev 1.4)
│   ├── HARDWARE_SPECIFICATION.md
│   ├── HARDWARE_WIRING_MANUAL.md
│   ├── SAFETY_ARCHITECTURE.md
│   ├── MOTOR_CONTROL.md
│   ├── TOUCH_CALIBRATION_SYSTEM.md
│   └── ...
├── STM32-Control-Coche-Marcos.ioc   CubeMX project (FDCAN1, TIM1/2/3/8, I2C1, ADC1)
├── Makefile
├── CHANGELOG.md
└── README.md
```

---

## 31. Build Instructions

### STM32 Firmware (STM32CubeIDE)

**Prerequisites:** STM32CubeIDE ≥ 1.14 with STM32G4 support pack, `arm-none-eabi-gcc` 12+.

**First-time setup (HAL drivers must be generated):**
1. Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
2. Open the project: `STM32-Control-Coche-Marcos.ioc`
3. **Project → Generate Code** (downloads HAL to `Drivers/`)
4. Import the project into STM32CubeIDE workspace

**Build:**
```bash
# In STM32CubeIDE
Project → Build Project (Ctrl+B)
Run → Debug (F11) to flash via ST-LINK

# Command line (after CubeMX code generation)
make clean && make all
# Output: Debug/STM32-Control-Coche-Marcos.elf
```

**⚠ CubeMX re-generation warning:** `project_config.h` is NOT managed by CubeMX and will not be overwritten. However, verify after regeneration:
- `PB0` must remain `GPIO_MODE_OUTPUT_OD` (OneWire requires open-drain)
- `PA11`/`PA12` must remain `FDCAN1_RX`/`FDCAN1_TX` (AF9)
- `PC2` must be `EN_RR` output (not re-assigned)

**Unit tests (host-side):** `Core/Src/test_*.c` files contain standalone C tests. They are excluded from the firmware build via `.cproject` `sourceEntries`. Compile separately with GCC on host:
```bash
gcc -o test_motor Core/Src/test_motor_control.c -lm && ./test_motor
```

### ESP32-S3 HMI Firmware (PlatformIO)

**Prerequisites:** PlatformIO Core ≥ 6.1 (VS Code extension or CLI), `espressif32` platform.

```bash
cd esp32/

# Install dependencies and build
pio run

# Flash to ESP32-S3 (USB-CDC, hold BOOT then press RESET if needed)
pio run -t upload

# Serial monitor
pio device monitor --baud 115200
```

**Libraries (auto-installed by PlatformIO):**

| Library | Version | Use |
|---------|---------|-----|
| `handmade0octopus/ESP32-TWAI-CAN` | ^1.0.1 | CAN bus driver |
| `bodmer/TFT_eSPI` | ^2.5.43 | TFT display + XPT2046 touch |
| `fastled/FastLED` | 3.9.12 | WS2812B LED strips |
| DFRobot_DFPlayerMini | (bundled/inline) | DFPlayer Mini |
| Arduino Preferences | (Arduino framework) | NVS storage |

---

## 32. Feature Status Matrix

| Feature | Status | Notes |
|---------|--------|-------|
| 4-wheel independent traction (4×2 / 4×4) | ✅ Implemented | `motor_control.c`, 20 kHz PWM, full per-wheel control |
| Axis rotation (tank turn) | ✅ Implemented | FL/RL forward, FR/RR reverse; steering zeroed |
| Gear selector (P/R/N/D1/D2) | ✅ Implemented | MCP23017 → CAN → STM32 |
| Closed-loop steering PID | ✅ Implemented | E6B2-CWZ6C encoder, TIM2 quadrature |
| Ackermann steering geometry | ✅ Implemented | `ackermann.c`, pure geometry |
| Automatic steering centering | ✅ Implemented | LJ12A3 sensor sweep, BOOT/STANDBY only |
| Persistent steering calibration | ✅ Implemented | Flash page 126, magic + CRC32 |
| ABS (anti-lock braking) | ✅ Implemented | 15% slip threshold, per-wheel scale |
| TCS (traction control) | ✅ Implemented | 15% spin threshold, per-wheel scale |
| Dynamic braking (H-bridge) | ✅ Implemented | Rate-proportional opposing torque, heat dissipation |
| Obstacle torque reduction (5 zones) | ✅ Implemented | TF-Mini Plus → ESP32 CAN → STM32 state machine |
| Battery undervoltage monitoring | ✅ Implemented | 20 V / 18 V thresholds |
| Battery overvoltage monitoring | ✅ Implemented | 30 V / 35 V thresholds |
| Overcurrent protection (INA226) | ✅ Implemented | 25 A per motor, 50 A sensor range |
| Overtemperature protection (DS18B20) | ✅ Implemented | 80 °C warn, 90 °C critical |
| 7-state safety state machine | ✅ Implemented | BOOT/STANDBY/ACTIVE/DEGRADED/SAFE/ERROR/LIMP_HOME |
| Granular DEGRADED levels (L1/L2/L3) | ✅ Implemented | Per-level power/steering/traction caps |
| LIMP_HOME mode (local pedal, no CAN) | ✅ Implemented | 20% torque, 5 km/h cap |
| IWDG watchdog | ✅ Implemented | ~500 ms, resets MCU on firmware hang |
| TIM1/TIM8 BREAK2 hardware kill | ✅ Implemented | Armed to Cortex-M4 LOCKUP |
| Startup movement inhibit | ✅ Implemented | 400 ms pedal-at-rest required after boot |
| CAN bus-off detection + recovery | ✅ Implemented | 10 retry attempts, escalates to ERROR |
| Service mode (25 modules) | ✅ Implemented | 4 critical + 21 non-critical, CAN interface |
| Persistent error log (Flash) | ✅ Implemented | 250 entries, ring buffer, CRC32 |
| DS18B20 persistent role mapping | ✅ Implemented | Flash, editable from engineering menu |
| I2C bus recovery (SDA stuck-low) | ✅ Implemented | NXP AN10216 clock cycling |
| Loop timing diagnostics (DWT) | ✅ Implemented | Peak-hold in STATUS_SAFETY byte 5 |
| DWT EMI debounce counters | ✅ Implemented | CAN 0x306/0x307, engineering screen |
| Touchscreen display (ST7796) | ✅ Implemented | 480×320, 40 MHz SPI |
| XPT2046 touch calibration wizard | ✅ Implemented | `touch_calibration_screen.cpp` |
| XPT2046 NVS persistent calibration | ✅ Implemented | Namespace `touch_cal`, CRC32 |
| Compile-time touch calibration fallback | ✅ Implemented | `TOUCH_CALIBRATION` in `User_Setup.h` |
| DFPlayer Mini audio (68 tracks) | ✅ Implemented | Priority queue, CAN-triggered events |
| Speaker relay (pop prevention) | ✅ Implemented | 150 ms timing, 7 s watchdog |
| Ignition key + shutdown sequence | ✅ Implemented | `power_manager.cpp`, GPIO 40/41 |
| WS2812B LED strips | ✅ Implemented | FastLED, GPIO 47 (front) / 48 (rear) |
| Engineering menu (PIN 8989) | ✅ Implemented | Relay override, service, diagnostics |
| Boot validation pre-ACTIVE gate | ✅ Implemented | `boot_validation.c` |
| Relay health check (INA226) | ✅ Implemented | `SAFETY_ERROR_RELAY_OPEN` |
| CMD_ACK acknowledgment | ✅ Implemented | CAN 0x103, after CMD_MODE/SERVICE_CMD |
| **Regenerative energy recovery to battery** | ❌ Not implemented | Dynamic braking dissipates energy as heat. No energy path to battery. BTS7960 topology does not support regeneration. |
| **AI-based regen optimization** | ❌ Not implemented | No neural/ML logic anywhere in the codebase |
| **Adaptive cruise control** | ❌ Not implemented | Referenced in `docs/FIRMWARE_MATURITY_ROADMAP.md` as future work. Obstacle scale adjusts torque but does not maintain a set following distance. |

---

## 33. Known Limitations

1. **No regenerative braking:** The BTS7960 (IBT-2) H-bridge topology used here does not support returning energy to the battery. Dynamic braking dissipates kinetic energy as heat in the motor windings.

2. **INA226 response latency:** I2C read cycle is ~2 ms at 400 kHz. The BTS7960 analog current sense (R\_IS/L\_IS) is not connected; sub-microsecond hardware overcurrent detection is not available. The 25 A overcurrent threshold is checked at 20 Hz (50 ms rate) via INA226.

3. **Wheel speed at low speed:** EXTI pulse counting becomes unreliable below ~0.4 km/h (WHEEL\_STALE\_TIMEOUT\_MS = 500 ms, 6 pulses/revolution). ABS/TCS are ineffective at very low speeds.

4. **Single CAN bus, no redundancy:** There is one CAN bus. Loss of the bus causes LIMP\_HOME, not full stop. No secondary communication channel.

5. **Flash wear:** Steering calibration (Flash page 126) is written once per calibration cycle. The STM32G474RE Flash endurance is 10 000 cycles — not a concern for normal use.

6. **No real-time clock:** The error log uses `HAL_GetTick()` milliseconds since last reset, not wall-clock time. Post-mortem diagnosis requires knowing the uptime at the time of the fault.

7. **Obstacle sensor field-of-view:** The TF-Mini Plus has a ~2° circular FOV — it detects single-point obstacles directly ahead. Wide or low obstacles (kerbs, ramps) may not be detected.

---

## 34. Safety Disclaimers

> ⚠️ **This is an experimental vehicle control system for a child-scale electric vehicle. It is NOT certified for road use, NOT certified to any automotive safety standard (ISO 26262, IEC 61508, or similar), and NOT suitable for use in any application where human safety depends on its correct operation.**

- The firmware implements multiple fail-safe mechanisms (IWDG, BREAK2, relay sequencer, overcurrent/overtemperature protection, startup inhibit, LIMP_HOME) but these have not been independently verified or safety-qualified.
- Always ensure the vehicle is operated in a safe, enclosed area with adult supervision when children are riding.
- The 24 V electrical system carries sufficient energy to cause injury. Follow all wiring safety guidelines in `docs/HARDWARE_WIRING_MANUAL.md` and use appropriate fusing.
- Verify relay health (INA226 current detection) and BTS7960 driver functionality before each use session.
- Do not modify the safety state machine thresholds or disable critical modules (MODULE_CAN_TIMEOUT, MODULE_EMERGENCY_STOP, MODULE_WATCHDOG, MODULE_RELAY_TRAC) in production.

---

## 35. License

MIT License — see [`LICENSE`](LICENSE) for full text.

Copyright © 2026 florinzgz — STM32-Control-Coche-Marcos

---

## 36. Author

**Ovidiu Florin Salca** ([@florinzgz](https://github.com/florinzgz))
