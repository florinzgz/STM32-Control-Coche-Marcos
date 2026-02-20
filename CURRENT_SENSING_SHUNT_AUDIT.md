# INA226 Current Sensing Shunt — Hardware–Firmware Consistency Audit

> **Audit scope:** Reverse-engineer the intended physical wiring of INA226
> current-sensing shunts from firmware source code only.
> No code modifications. No redesign proposals.

---

## SECTION 1 — Sensor Table

All facts below are extracted directly from source code.

### 1.1 INA226 instance summary

| Channel | Name (from code) | I2C Address | TCA9548A Channel | Shunt R (mΩ) | Max Sensor Rating | Used By Module(s) | Physical Meaning | Control Critical |
|---------|-------------------|-------------|-------------------|---------------|-------------------|--------------------|------------------|------------------|
| 0 | `MODULE_CURRENT_SENSOR_0` (FL) | 0x40 | 0 | 1.0 | 50 A | safety_system, motor_control, boot_validation, can_handler, service_mode | Front-Left motor current | No (non-critical, disableable) |
| 1 | `MODULE_CURRENT_SENSOR_1` (FR) | 0x40 | 1 | 1.0 | 50 A | safety_system, motor_control, boot_validation, can_handler, service_mode | Front-Right motor current | No (non-critical, disableable) |
| 2 | `MODULE_CURRENT_SENSOR_2` (RL) | 0x40 | 2 | 1.0 | 50 A | safety_system, motor_control, boot_validation, can_handler, service_mode | Rear-Left motor current | No (non-critical, disableable) |
| 3 | `MODULE_CURRENT_SENSOR_3` (RR) | 0x40 | 3 | 1.0 | 50 A | safety_system, motor_control, boot_validation, can_handler, service_mode | Rear-Right motor current | No (non-critical, disableable) |
| 4 | `MODULE_CURRENT_SENSOR_4` (battery) | 0x40 | 4 | 0.5 | 100 A | safety_system (battery UV), can_handler (battery telemetry), boot_validation, service_mode | 24 V battery bus total current | No (non-critical, disableable) |
| 5 | `MODULE_CURRENT_SENSOR_5` (steer) | 0x40 | 5 | 1.0 | 50 A | safety_system (overcurrent check), service_mode | Steering motor current | No (non-critical, disableable) |

### 1.2 Key constants extracted from code

| Constant | Value | Source File | Purpose |
|----------|-------|-------------|---------|
| `I2C_ADDR_TCA9548A` | 0x70 | `main.h:84` | TCA9548A I2C multiplexer base address |
| `I2C_ADDR_INA226` | 0x40 | `main.h:85` | INA226 address (same for all; multiplexer selects) |
| `NUM_INA226` | 6 | `main.h:86` | Total INA226 count |
| `INA226_SHUNT_MOHM_MOTOR` | 1 mΩ | `main.h:96` | Shunt resistance for channels 0–3, 5 (50 A sensors) |
| `INA226_SHUNT_MOHM_BATTERY` | 0.5 mΩ | `main.h:97` | Shunt resistance for channel 4 (100 A sensor) |
| `INA226_CHANNEL_BATTERY` | 4 | `main.h:98` | TCA9548A channel index for battery sensor |
| `INA226_SHUNT_LSB_UV` | 2.5 µV | `sensor_manager.c:372` | INA226 shunt voltage register LSB |
| `INA226_BUS_LSB_MV` | 1.25 mV | `sensor_manager.c:373` | INA226 bus voltage register LSB |
| `MAX_CURRENT_A` | 25.0 A | `safety_system.c:24` | Overcurrent threshold (any channel) |
| `SENSOR_CURRENT_MAX_A` | 50.0 A | `safety_system.c:61` | Sensor plausibility ceiling |
| `BATTERY_UV_WARNING_V` | 20.0 V | `safety_system.c:45` | Battery undervoltage warning |
| `BATTERY_UV_CRITICAL_V` | 18.0 V | `safety_system.c:46` | Battery undervoltage critical |

### 1.3 Per-sensor usage detail

#### Channels 0–3 (FL, FR, RL, RR motor current)

- **Initialization:** `Sensor_Init()` zeroes `current_amps[]` and `voltage_bus[]` for all 6 channels (`sensor_manager.c:770–773`).
- **Reading:** `Current_ReadAll()` iterates `i = 0..5`, selects TCA9548A channel `i`, reads shunt voltage register, converts: `I = (shunt_raw × 2.5 µV) / R_shunt / 1000` → amperes (`sensor_manager.c:405–448`).
- **Overcurrent safety:** `Safety_CheckCurrent()` checks all 6 channels against `MAX_CURRENT_A` (25 A). Overcurrent → `SAFETY_ERROR_OVERCURRENT` → `SYS_STATE_DEGRADED` (first event) or `SYS_STATE_SAFE` (≥3 consecutive) (`safety_system.c:837–887`).
- **Sensor plausibility:** `Safety_CheckSensors()` checks all 6 channels: values < −1.0 A or > 50.0 A raise `MODULE_FAULT_ERROR` → `SYS_STATE_DEGRADED` (`safety_system.c:1044–1057`).
- **Boot validation:** `check_current_plausible()` validates channels 0–3 only (not battery/steering) against −1.0 A to 50.0 A (`boot_validation.c:69–79`).
- **CAN telemetry:** `CAN_SendStatusCurrent()` transmits channels 0–3 as FL/FR/RL/RR in CAN ID 0x201, scaled ×100 (0.01 A units) (`main.c:245–249`).
- **Motor control feedback:** `motor_control.c` reads `Current_GetAmps(i)` for `i = 0..3` into `traction_state.wheels[i].currentA` for park-hold derating, dynamic braking current limit, and traction telemetry (`motor_control.c:733, 783, 815, 1319`).
- **Service mode:** Classified as `MODULE_CLASS_NON_CRITICAL` — individually disableable by user via CAN command. When disabled, overcurrent/plausibility checks skip that sensor (`service_mode.c:42–48`).

#### Channel 4 (Battery 24 V bus)

- **Reading:** Same `Current_ReadAll()` loop, but uses `INA226_SHUNT_MOHM_BATTERY` (0.5 mΩ) instead of 1.0 mΩ (`sensor_manager.c:423–424`).
- **Battery voltage monitoring:** `Safety_CheckBatteryVoltage()` reads `Voltage_GetBus(INA226_CHANNEL_BATTERY)` (bus voltage register). < 20.0 V → DEGRADED, < 18.0 V or 0.0 V → SAFE (`safety_system.c:1181–1221`).
- **CAN telemetry:** `CAN_SendStatusBattery()` transmits battery current and voltage in CAN ID 0x207 (0.01 A and 0.01 V units) (`can_handler.c:327–343`).
- **Boot validation:** `check_battery_ok()` reads `Voltage_GetBus(4)` and requires ≥ 20.0 V (`boot_validation.c:93–97`).
- **Overcurrent safety:** Included in `Safety_CheckCurrent()` loop (iterates all `NUM_INA226` = 6) — battery overcurrent above 25 A also triggers DEGRADED/SAFE escalation.
- **Service mode:** `MODULE_CURRENT_SENSOR_4` = NON_CRITICAL, disableable.

#### Channel 5 (Steering motor)

- **Reading:** Same `Current_ReadAll()` loop with 1.0 mΩ shunt resistance.
- **Overcurrent safety:** Included in `Safety_CheckCurrent()` loop — steering motor overcurrent triggers same DEGRADED/SAFE escalation.
- **Sensor plausibility:** Included in `Safety_CheckSensors()` loop.
- **CAN telemetry:** NOT transmitted in CAN ID 0x201 (which only sends 4 wheels). Not transmitted in any dedicated CAN message. Only visible through service mode fault bitmask (CAN 0x301).
- **Service mode:** `MODULE_CURRENT_SENSOR_5` = NON_CRITICAL, disableable.

---

## SECTION 2 — Physical Placement Deduction

### 2.1 Channels 0–3 (per-wheel motor shunts)

**Firmware math requires each shunt to measure the current flowing through one individual motor.**

Evidence:
- `Current_GetAmps(i)` for `i = 0..3` is stored in `traction_state.wheels[i].currentA` — a per-wheel data structure (`motor_control.c:783, 815, 1319`).
- `CAN_SendStatusCurrent()` transmits four separate values labeled FL/FR/RL/RR (`can_handler.c:208–222`; `main.c:245–249`).
- Park-hold derating in `motor_control.c` reads `Current_GetAmps(i)` for each motor individually and applies per-motor current/temperature limits (`motor_control.c:730–737`).
- `Safety_CheckCurrent()` iterates per channel and raises per-sensor faults via `ServiceMode_SetFault(MODULE_CURRENT_SENSOR_0 + i, ...)` (`safety_system.c:843–848`).
- Each INA226 sits behind a separate TCA9548A channel (0..3), and all share the same I2C address 0x40 — physically separate devices, one per motor.

**Placement:** Each shunt **MUST** be placed in series with its respective motor's power supply line — between the BTS7960 H-bridge driver output and the motor winding (or between the battery and the BTS7960 input, as long as the shunt measures only that single motor's current).

**Upstream vs. downstream of driver:** The firmware does not distinguish between high-side and low-side sensing. The INA226 supports both. However, the code comment says "50A sensors with 1 mΩ shunt" (`main.h:93–95`), and the bus voltage register is also read for each channel (`sensor_manager.c:430–431`). The bus voltage on the motor channels is not used in any control or safety decision — only `current_amps[i]` is consumed. The shunt could be on either side of the BTS7960 driver, but the most common and practical placement is **between the BTS7960 driver output and the motor** (low-side or high-side of the driver, measuring motor current).

### 2.2 Channel 4 (battery bus)

**Firmware math requires this shunt to measure total system current draw from the 24 V battery.**

Evidence:
- `INA226_CHANNEL_BATTERY` = 4, with 0.5 mΩ shunt (100 A rating) — sized for the entire system load, not a single motor (`main.h:97–98`).
- `Voltage_GetBus(INA226_CHANNEL_BATTERY)` is used for battery undervoltage detection: the firmware expects this to read the actual battery terminal voltage or bus rail voltage (`safety_system.c:1183`).
- `CAN_SendStatusBattery()` labels it "24V main battery" and transmits both current and voltage (`can_handler.c:317–343`; `vehicle_data.h:117`).
- The 100 A rating (vs. 50 A per motor) and the 0.5 mΩ shunt (half the motor shunt resistance) confirm this is sized for aggregate system current.

**Placement:** The shunt **MUST** be in series with the main battery positive line — between the battery positive terminal and the main power distribution bus (after the main relay `PIN_RELAY_MAIN`). The INA226 bus voltage pin must be connected to the 24 V bus so that `Voltage_GetBus()` returns the actual supply rail voltage.

### 2.3 Channel 5 (steering motor)

**Firmware math requires this shunt to measure the steering motor's current independently.**

Evidence:
- `MODULE_CURRENT_SENSOR_5` is labeled "(steer)" in `service_mode.h:66`.
- The comment in `main.h:95` says "Channel 5 (steering motor): 50A sensor with 1 mΩ shunt".
- It is included in the `Safety_CheckCurrent()` overcurrent loop and `Safety_CheckSensors()` plausibility loop alongside all other channels.
- It is NOT transmitted in the per-wheel current CAN message (0x201) — confirming it is a separate subsystem.

**Placement:** The shunt **MUST** be in series with the steering motor power line — between the BTS7960 H-bridge driver (controlled via TIM8_CH3 / PC8 PWM and PC4 direction / PC9 enable) and the steering motor, or between the steering relay (`PIN_RELAY_DIR`) and the steering BTS7960 input.

---

## SECTION 3 — Safety Dependency Analysis

### 3.1 What depends on shunt placement

The firmware's safety system uses current readings in three independent safety paths:

1. **Overcurrent protection** (`Safety_CheckCurrent()`, 10 ms loop): compares each `Current_GetAmps(i)` against 25 A threshold. Triggers DEGRADED → SAFE escalation.
2. **Sensor plausibility** (`Safety_CheckSensors()`, 10 ms loop): flags readings < −1.0 A or > 50.0 A as sensor faults.
3. **Battery undervoltage** (`Safety_CheckBatteryVoltage()`, 100 ms loop): reads `Voltage_GetBus(4)` for bus voltage monitoring.

### 3.2 Scenario: Shunt placed BEFORE motor driver (between battery/relay and BTS7960 input)

- **Motor channels 0–3:** The INA226 would measure the current entering the BTS7960 driver. The BTS7960 is a full H-bridge; input current approximately equals output current (minus driver losses). The firmware's `Safety_CheckCurrent()` compares against `MAX_CURRENT_A` = 25 A — this threshold is designed for motor current. Placing the shunt before the driver would still measure approximately the correct motor current, so:
  - **Overcurrent detection:** Would still function correctly (current into driver ≈ current through motor).
  - **Park-hold derating:** Would still read correct current (motor current flows through driver from battery side).
  - **Minor inaccuracy:** During PWM off-time, the BTS7960 recirculates motor current internally; the shunt before the driver would see pulsed current (average equals motor current at duty cycle), while a shunt after the driver would see continuous motor current. The INA226's 1.1 ms default conversion time averages this, so practical difference is small.

- **Battery channel 4:** This channel is already expected to be before all motor drivers (at the battery). Placing it "before the driver" is its intended position.

- **What would break:** Nothing catastrophic. Minor measurement noise during dynamic braking due to recirculation current paths.

### 3.3 Scenario: Shunt placed AFTER motor driver (between BTS7960 output and motor)

- **Motor channels 0–3:** This is the most natural placement. The INA226 measures actual motor winding current including regenerative/flyback current. The firmware's threshold of 25 A and plausibility range of −1.0 A to 50.0 A account for small negative readings: "A small negative reading (> −1 A) is expected due to INA226 offset and inductive motor flyback during deceleration" (`safety_system.c:1045–1046`).
  - **Overcurrent detection:** Works correctly — measures actual motor current.
  - **Park-hold derating:** Works correctly — `Current_GetAmps(i)` returns actual motor current.
  - **Dynamic braking:** Works correctly — motor_control.c limits dynamic braking based on motor current.

- **Battery channel 4:** Placing the battery shunt after a motor driver would only measure one motor's current instead of total system current. **This would break:**
  - `Safety_CheckBatteryVoltage()` — bus voltage would read motor terminal voltage, not battery voltage.
  - `CAN_SendStatusBattery()` — reported current/voltage would not represent battery state.
  - `check_battery_ok()` — boot validation would see motor voltage, not battery voltage.
  - **Conclusion: Battery shunt MUST NOT be placed after any motor driver.**

- **Steering channel 5:** Placing it after the steering BTS7960 works identically to the motor channels — measures actual steering motor current.

### 3.4 Scenario: Shunt placed AT battery (main bus only, no per-motor shunts)

If only a single shunt were placed at the battery instead of per-motor shunts:

- **Per-wheel current telemetry** (`CAN_SendStatusCurrent()` on 0x201) would report the same value for all four channels (or zero for unmeasured channels). The ESP32 HMI displays per-wheel current — this data would be meaningless.
- **Park-hold per-motor derating** (`motor_control.c:730–737`) iterates each motor's current individually to find the worst case. With a single battery shunt, per-motor derating would not function — all motors would derate simultaneously based on aggregate current.
- **Service mode per-sensor fault tracking** (`Safety_CheckCurrent()` sets faults per `MODULE_CURRENT_SENSOR_0 + i`) would lose per-motor diagnostic granularity.
- **Overcurrent detection** would still work at a system level (aggregate current > 25 A is still dangerous), but could not identify which motor is overloaded.
- **Battery undervoltage** would work correctly (this is the intended placement for channel 4).
- **Steering current** would not be measured at all.
- **Boot validation** (`check_current_plausible()` checks channels 0–3 individually) would receive aggregate current on one channel and zero on the other three — likely triggering plausibility faults.

**Conclusion: A single battery-only shunt would break per-wheel current telemetry, per-motor derating, per-sensor fault diagnostics, and boot validation plausibility checks. The firmware architecturally requires 6 individual shunts.**

---

## SECTION 4 — Confidence Level

### 4.1 What is explicitly defined in code

| Fact | Confidence | Evidence |
|------|------------|----------|
| 6 INA226 sensors exist | **Explicit** | `#define NUM_INA226 6` (`main.h:86`) |
| All share I2C address 0x40 | **Explicit** | `#define I2C_ADDR_INA226 0x40` (`main.h:85`) |
| TCA9548A at 0x70 multiplexes them | **Explicit** | `#define I2C_ADDR_TCA9548A 0x70` (`main.h:84`); `TCA9548A_SelectChannel(i)` (`sensor_manager.c:378`) |
| Channels 0–3 = motor wheels (FL/FR/RL/RR) | **Explicit** | Comments in `main.h:93–94`, `service_mode.h:62–65`, CAN telemetry mapping in `main.c:245–249` |
| Channel 4 = battery 24 V | **Explicit** | `#define INA226_CHANNEL_BATTERY 4` (`main.h:98`), comment `main.h:94`, `service_mode.h:65` |
| Channel 5 = steering motor | **Explicit** | Comment `main.h:95`, `service_mode.h:66` |
| Shunt resistances: 1 mΩ (motor/steer), 0.5 mΩ (battery) | **Explicit** | `main.h:96–97` |
| Current sensors are NON_CRITICAL (disableable) | **Explicit** | `service_mode.c:42–48` |
| Overcurrent threshold = 25 A | **Explicit** | `#define MAX_CURRENT_A 25.0f` (`safety_system.c:24`) |
| Battery UV thresholds: 20.0 V / 18.0 V | **Explicit** | `safety_system.c:45–46` |

### 4.2 What is inferred mathematically

| Inference | Confidence | Reasoning |
|-----------|------------|-----------|
| Per-motor shunt placement (not shared) | **High** — inferred from per-index usage: `Current_GetAmps(i)` stored in `traction_state.wheels[i].currentA`, per-sensor faults, per-wheel CAN telemetry | Code treats each index as an independent motor's current |
| Battery shunt is at system bus (not per-motor) | **High** — inferred from 100 A rating (vs. 50 A per motor), `Voltage_GetBus()` used for battery voltage, and "24V main battery" labeling | Different shunt resistance + dedicated voltage monitoring |
| Shunts measure driver current, not torque | **High** — firmware never converts current to torque (no motor torque constant `Kt` or `torque_from_current()` anywhere in code). Current is used only for overcurrent protection and telemetry | No torque computation exists in the codebase |
| No per-wheel torque sensing architecture | **High** — ABS/TCS use `wheel_scale[]` (speed-based slip detection), not current-based torque estimation. `wheel_scale` is set by speed comparison algorithms, never by current values | Traction control is speed-based, not current-based |

### 4.3 Architecture intent deduction

From the code logic, the system assumes:

- **A) Per-wheel torque sensing:** ❌ **No.** The firmware does not compute torque from current. There is no motor torque constant (`Kt`) or `current_to_torque()` function anywhere. ABS/TCS operate on wheel speed slip ratios, not motor torque.

- **B) Driver current monitoring:** ✅ **Yes.** Channels 0–3 and 5 measure current through individual BTS7960 motor drivers. This is used for overcurrent protection (25 A threshold), sensor plausibility (−1 A to 50 A range), per-motor park-hold derating, and telemetry. The purpose is **driver protection**, not torque estimation.

- **C) Total power monitoring:** ✅ **Partial.** Channel 4 monitors total battery bus current and voltage. The firmware uses voltage for undervoltage protection and current for overcurrent detection and telemetry. Power (W) is never explicitly computed (`V × I` calculation does not exist in the code), so this is current/voltage monitoring rather than power monitoring.

- **D) Battery BMS-style monitoring:** ❌ **No.** There is no State of Charge (SoC), cell balancing, or coulomb counting. The battery channel provides simple undervoltage protection (threshold-based) and current telemetry. This is a basic voltage watchdog, not a Battery Management System.

**Summary: The architecture is "per-driver overcurrent protection + battery undervoltage watchdog + telemetry". Current is never used for closed-loop motor control or torque estimation.**
