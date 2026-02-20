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

**Placement:** Each shunt **MUST** be placed in series with its respective motor's power supply line — **between the relay output and the BTS7960 H-bridge driver input** (i.e. before the driver, on the power supply side). This ensures the INA226 measures only that single motor's current.

**Before the driver (not after):** The INA226 shunts go between the relay/power distribution and the BTS7960 input (B+ pin). Placing them before the driver means:
- The INA226 measures the current entering each BTS7960, which equals the motor current (minus small driver losses).
- The INA226's `Voltage_GetBus()` reads the supply rail voltage upstream of the driver — a meaningful reference for diagnostics.
- During PWM off-time the BTS7960 recirculates motor current internally; the shunt before the driver sees the average DC supply current, which the INA226's 1.1 ms conversion time integrates naturally.
- The firmware's plausibility range of −1.0 A to 50.0 A (`safety_system.c:1045–1046`) accounts for small negative transients due to inductive flyback reaching the supply rail.

### 2.2 Channel 4 (battery bus)

**Firmware math requires this shunt to measure total system current draw from the 24 V battery.**

Evidence:
- `INA226_CHANNEL_BATTERY` = 4, with 0.5 mΩ shunt (100 A rating) — sized for the entire system load, not a single motor (`main.h:97–98`).
- `Voltage_GetBus(INA226_CHANNEL_BATTERY)` is used for battery undervoltage detection: the firmware expects this to read the actual battery terminal voltage or bus rail voltage (`safety_system.c:1183`).
- `CAN_SendStatusBattery()` labels it "24V main battery" and transmits both current and voltage (`can_handler.c:317–343`; `vehicle_data.h:117`).
- The 100 A rating (vs. 50 A per motor) and the 0.5 mΩ shunt (half the motor shunt resistance) confirm this is sized for aggregate system current.

**Placement:** The shunt **MUST** be in series with the main battery positive line — **between the battery positive terminal and the main relay `PIN_RELAY_MAIN`** (i.e. before the relay, directly at the battery). This is critical because:
- `Safety_CheckBatteryVoltage()` must read battery voltage **at all times**, including when the relay is open (system in STANDBY, SAFE, or ERROR state).
- If the shunt were placed after the relay, opening the relay would disconnect the INA226 from the battery, causing `Voltage_GetBus()` to return 0.0 V, which the firmware treats as a sensor failure / critical undervoltage (`safety_system.c:1185–1190`).
- The ESP32 HMI displays battery voltage via CAN ID 0x207 — this must remain visible even when the system is powered down (relays off).
- The INA226 bus voltage pin reads the actual battery terminal voltage, providing an always-available voltage reference regardless of relay state.

### 2.3 Channel 5 (steering motor)

**Firmware math requires this shunt to measure the steering motor's current independently.**

Evidence:
- `MODULE_CURRENT_SENSOR_5` is labeled "(steer)" in `service_mode.h:66`.
- The comment in `main.h:95` says "Channel 5 (steering motor): 50A sensor with 1 mΩ shunt".
- It is included in the `Safety_CheckCurrent()` overcurrent loop and `Safety_CheckSensors()` plausibility loop alongside all other channels.
- It is NOT transmitted in the per-wheel current CAN message (0x201) — confirming it is a separate subsystem.

**Placement:** The shunt **MUST** be in series with the steering motor power line — **between the steering relay (`PIN_RELAY_DIR`) and the steering BTS7960 input** (i.e. before the driver, on the power supply side). This is consistent with the motor channel placement: shunts go between the relay output and the BTS7960 B+ input pin.

---

## SECTION 3 — Safety Dependency Analysis

### 3.1 What depends on shunt placement

The firmware's safety system uses current readings in three independent safety paths:

1. **Overcurrent protection** (`Safety_CheckCurrent()`, 10 ms loop): compares each `Current_GetAmps(i)` against 25 A threshold. Triggers DEGRADED → SAFE escalation.
2. **Sensor plausibility** (`Safety_CheckSensors()`, 10 ms loop): flags readings < −1.0 A or > 50.0 A as sensor faults.
3. **Battery undervoltage** (`Safety_CheckBatteryVoltage()`, 100 ms loop): reads `Voltage_GetBus(4)` for bus voltage monitoring.

### 3.2 Correct placement: Shunts BEFORE drivers, battery shunt BEFORE relay

This is the intended placement per the firmware architecture:

- **Motor channels 0–3:** INA226 shunts between relay output and BTS7960 input (B+ pin). The INA226 measures current entering each driver. BTS7960 input current approximately equals motor current (minus small driver losses). The firmware's `Safety_CheckCurrent()` compares against `MAX_CURRENT_A` = 25 A — this works correctly with before-driver placement since input current tracks motor current.
  - **Overcurrent detection:** Functions correctly (current into driver ≈ current through motor).
  - **Park-hold derating:** Reads correct current (motor current flows through driver from battery side).
  - **PWM averaging:** During PWM off-time, the BTS7960 recirculates motor current internally; the shunt before the driver sees pulsed current, but the INA226's 1.1 ms default conversion time averages this naturally.

- **Battery channel 4:** INA226 shunt between battery positive terminal and main relay input. The INA226 reads battery voltage at all times, even when the relay is open. This is critical for:
  - `Safety_CheckBatteryVoltage()` — always reads true battery voltage.
  - `CAN_SendStatusBattery()` — ESP32 HMI can always display battery voltage.
  - `check_battery_ok()` — boot validation reads true battery voltage.

- **What would break:** Nothing. This is the correct topology.

### 3.3 Scenario: Battery shunt placed AFTER the main relay (incorrect)

- **When relay is closed:** Works normally — battery voltage and current are readable.
- **When relay is open (SAFE, ERROR, STANDBY, or power-down):** The INA226 is disconnected from the battery. `Voltage_GetBus()` returns 0.0 V, which the firmware interprets as a sensor failure / critical undervoltage (`safety_system.c:1185–1190`), triggering `SAFETY_ERROR_BATTERY_UV_CRITICAL` and `SYS_STATE_SAFE`. **This creates a false-positive fault loop.**
- **ESP32 HMI:** Cannot display battery voltage when the relay is off — the user sees 0.0 V instead of the actual battery state.
- **Conclusion: Battery shunt MUST be placed BEFORE the relay.**

### 3.4 Scenario: Motor shunts placed AFTER motor driver (between BTS7960 output and motor)

- **Motor channels 0–3:** The INA226 would measure actual motor winding current including regenerative/flyback current. The firmware's threshold of 25 A and plausibility range of −1.0 A to 50.0 A account for small negative readings: "A small negative reading (> −1 A) is expected due to INA226 offset and inductive motor flyback during deceleration" (`safety_system.c:1045–1046`).
  - **Overcurrent detection:** Would work, but sees regenerative current spikes that don't represent supply-side danger.
  - **Park-hold derating:** Would work, but reads motor winding current instead of supply current.
  - **Dynamic braking:** Would work — motor_control.c limits dynamic braking based on motor current.

- **Battery channel 4:** Placing the battery shunt after a motor driver would only measure one motor's current instead of total system current. **This would break** all battery monitoring (see 3.3 above).

- **Steering channel 5:** After-driver placement would function but is inconsistent with the before-driver topology used for motor channels.

### 3.5 Scenario: Shunt placed AT battery (main bus only, no per-motor shunts)

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
| Battery shunt BEFORE main relay (not after) | **High** — `Safety_CheckBatteryVoltage()` treats 0.0 V as sensor failure → SAFE state. If shunt were after the relay, opening the relay would always trigger a false critical UV fault. The firmware expects voltage to be always readable. | `safety_system.c:1185–1190`: 0 V → `SAFETY_ERROR_BATTERY_UV_CRITICAL` |
| Motor/steering shunts BEFORE BTS7960 drivers | **High** — firmware does not need to distinguish between supply-side and motor-side current (no torque computation). Placing shunts before drivers measures supply current entering each driver, which equals motor current minus small driver losses. | Consistent with overcurrent protection intent (protecting the driver/wiring) |
| Shunts measure driver current, not torque | **High** — firmware never converts current to torque (no motor torque constant `Kt` or `torque_from_current()` anywhere in code). Current is used only for overcurrent protection and telemetry | No torque computation exists in the codebase |
| No per-wheel torque sensing architecture | **High** — ABS/TCS use `wheel_scale[]` (speed-based slip detection), not current-based torque estimation. `wheel_scale` is set by speed comparison algorithms, never by current values | Traction control is speed-based, not current-based |

### 4.3 Architecture intent deduction

From the code logic, the system assumes:

- **A) Per-wheel torque sensing:** ❌ **No.** The firmware does not compute torque from current. There is no motor torque constant (`Kt`) or `current_to_torque()` function anywhere. ABS/TCS operate on wheel speed slip ratios, not motor torque.

- **B) Driver current monitoring:** ✅ **Yes.** Channels 0–3 and 5 measure current through individual BTS7960 motor drivers. This is used for overcurrent protection (25 A threshold), sensor plausibility (−1 A to 50 A range), per-motor park-hold derating, and telemetry. The purpose is **driver protection**, not torque estimation.

- **C) Total power monitoring:** ✅ **Partial.** Channel 4 monitors total battery bus current and voltage. The firmware uses voltage for undervoltage protection and current for overcurrent detection and telemetry. Power (W) is never explicitly computed (`V × I` calculation does not exist in the code), so this is current/voltage monitoring rather than power monitoring.

- **D) Battery BMS-style monitoring:** ❌ **No.** There is no State of Charge (SoC), cell balancing, or coulomb counting. The battery channel provides simple undervoltage protection (threshold-based) and current telemetry. This is a basic voltage watchdog, not a Battery Management System.

**Summary: The architecture is "per-driver overcurrent protection + battery undervoltage watchdog + telemetry". Current is never used for closed-loop motor control or torque estimation.**
