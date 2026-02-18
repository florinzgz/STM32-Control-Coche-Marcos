# BTS7960 / IBT-2 Motor Driver Wiring & Usage Audit

> **Scope**: Extract-only audit of the current firmware behavior.
> **Source files examined**: `Core/Inc/main.h`, `Core/Src/main.c`,
> `Core/Src/stm32g4xx_hal_msp.c`, `Core/Inc/motor_control.h`,
> `Core/Src/motor_control.c`, `Core/Inc/safety_system.h`.

---

## 1. Per-Motor Pin Mapping

The firmware uses **one MCU PWM output + one GPIO direction + one GPIO enable** per BTS7960/IBT-2 module.
There is **no dual-PWM** (separate RPWM/LPWM) control; instead, the single PWM signal is routed to the BTS7960 module and the external direction logic (on the IBT-2 carrier board or external wiring) selects which half-bridge receives the PWM.

### 1.1 Traction Motors (TIM1 — 20 kHz, center-aligned, ARR = 4249)

| Signal    | Motor FL         | Motor FR         | Motor RL         | Motor RR         |
|-----------|------------------|------------------|------------------|------------------|
| **PWM**   | PA8 — TIM1_CH1   | PA9 — TIM1_CH2   | PA10 — TIM1_CH3  | PA11 — TIM1_CH4  |
| **DIR**   | PC0              | PC1              | PC2              | PC3              |
| **EN**    | PC5              | PC6              | PC7              | PC13             |
| **R_IS / L_IS** | Not connected to MCU — current sensing is done externally via INA226 on I2C (TCA9548A ch 0–3) | ← same | ← same | ← same |

### 1.2 Steering Motor (TIM8 — 20 kHz, center-aligned, ARR = 4249)

| Signal    | Motor Steering   |
|-----------|------------------|
| **PWM**   | PC8 — TIM8_CH3   |
| **DIR**   | PC4              |
| **EN**    | PC9              |
| **R_IS / L_IS** | Not connected to MCU — current sensing via INA226 on I2C (TCA9548A ch 5) |

### 1.3 Current Sense Pins (R_IS / L_IS)

The BTS7960 analog current-sense outputs (R_IS, L_IS) are **not wired to any MCU ADC input**.
All motor current measurement is performed by **external INA226 high-side current-sense ICs** connected via the I2C1 bus through a TCA9548A 8-channel multiplexer:

| TCA9548A Channel | INA226 Measures      | Shunt Resistor |
|------------------|----------------------|----------------|
| 0                | Motor FL current     | 1 mΩ (50 A)   |
| 1                | Motor FR current     | 1 mΩ (50 A)   |
| 2                | Motor RL current     | 1 mΩ (50 A)   |
| 3                | Motor RR current     | 1 mΩ (50 A)   |
| 4                | Battery 24 V bus     | 0.5 mΩ (100 A)|
| 5                | Steering motor       | 1 mΩ (50 A)   |

---

## 2. Direction Implementation in Software

### 2.1 Scheme: DIR + Single PWM (not dual-PWM)

The firmware does **not** generate separate RPWM and LPWM signals.
Each motor is controlled by three independent signals:

| MCU Output | BTS7960 / IBT-2 Connection | Function |
|------------|----------------------------|----------|
| **PWM pin** (TIMx_CHy) | Connected to the PWM input of whichever half-bridge is selected by DIR | Duty cycle 0–4249 (0–100 %) |
| **DIR pin** (GPIOx)     | Directly drives the direction-select input on the IBT-2 module | `GPIO_PIN_SET` (HIGH) = forward; `GPIO_PIN_RESET` (LOW) = reverse |
| **EN pin** (GPIOx)      | Directly drives a combined R_EN + L_EN line (both tied together on the IBT-2 module) | HIGH = H-bridge enabled; LOW = H-bridge disabled (coast) |

**Source code** (`motor_control.c`):

```c
static void Motor_SetPWM(Motor_t *motor, uint16_t pwm) {
    if (pwm > PWM_PERIOD) pwm = PWM_PERIOD;
    __HAL_TIM_SET_COMPARE(motor->timer, motor->channel, pwm);
}

static void Motor_SetDirection(Motor_t *motor, int8_t direction) {
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin,
                      (direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Motor_Enable(Motor_t *motor, uint8_t enable) {
    HAL_GPIO_WritePin(motor->en_port, motor->en_pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
```

The `Motor_t` struct maps each motor's timer/channel, direction port/pin, and enable port/pin:

```c
motor_fl.timer = &htim1;  motor_fl.channel = TIM_CHANNEL_1;
motor_fl.dir_port = GPIOC; motor_fl.dir_pin = PIN_DIR_FL;   // PC0
motor_fl.en_port  = GPIOC; motor_fl.en_pin  = PIN_EN_FL;    // PC5
// (similar for FR, RL, RR, Steering)
```

### 2.2 How Direction Works per Gear

| Gear               | `effective_demand ≥ 0` → `dir` | `effective_demand < 0` → `dir` | Notes |
|--------------------|--------------------------------|--------------------------------|-------|
| FORWARD (D1)       | `+1` (forward)                 | `−1` (reverse)                 | Max 60 % power |
| FORWARD_D2         | `+1` (forward)                 | `−1` (reverse)                 | Max 100 % power |
| REVERSE            | `−1` (reversed)                | `+1` (reversed)                | Max 60 % power; dir is inverted from the base direction |
| PARK               | `+1` (hold brake uses forward) | N/A (no throttle accepted)     | Active hold brake only |
| NEUTRAL            | N/A (motors disabled)          | N/A                            | Coast — no PWM, EN=LOW |

In tank-turn (axis rotation) mode, left-side motors (FL, RL) receive `−dir` while right-side motors (FR, RR) receive `+dir`.

### 2.3 Braking Mode Behavior

The firmware implements **three distinct braking modes**, all using the BTS7960 H-bridge:

#### A. BTS7960 Active Brake (short-brake / hold)

**Constant**: `BTS7960_BRAKE_PWM = PWM_PERIOD = 4249` (100 % duty)

The firmware notes that Chinese BTS7960 modules have a design defect: when EN=HIGH and PWM=0, the motor **floats** (no braking). To achieve active braking, the firmware drives PWM to **100 % duty** with EN=HIGH. This shorts the motor terminals through the H-bridge, producing electromagnetic braking torque.

**Used in**:
- Steering deadband hold (`Steering_ControlLoop` when error < 0.5°)
- Steering neutralize (`Steering_Neutralize`)
- Traction zero-demand hold (when `|effective_demand| ≤ 0.5 %`)
- 4×2 mode rear wheels (rear motors are braked while front motors drive)

#### B. Dynamic Braking (deceleration)

When the driver lifts off the throttle rapidly (throttle rate decreasing), the firmware generates a proportional braking effort:

```
brake_pct = |throttle_rate| × DYNBRAKE_FACTOR (0.5)
```

Clamped to `DYNBRAKE_MAX_PCT` (60 %). The brake is applied by setting `effective_demand = −dynbrake_pct` which reverses the motor direction relative to travel, creating an opposing torque. Energy is dissipated as heat in motor windings — **no regenerative charging**.

Dynamic braking is **disabled** when:
- Average wheel speed < 3 km/h
- System state is SAFE, ERROR, BOOT, or STANDBY
- ABS is active on any wheel (`wheel_scale[i] < 1.0`)
- Gear is NEUTRAL or PARK

#### C. Park Hold Brake

In GEAR_PARK, all four motors receive a controlled low-duty active brake:

- Default hold duty: 30 % (`PARK_HOLD_PWM_PCT`)
- Direction set to forward; braking comes from low-duty driving with EN=HIGH
- Current derating: braking reduced above 15 A, disabled above 20 A
- Temperature derating: braking reduced above 70 °C, disabled above 85 °C
- Overridden (released) when system enters SAFE or ERROR state

---

## 3. Electrical Behavior in Special States

### 3.1 Throttle = 0 (Zero Demand)

**Condition**: `|effective_demand| ≤ TRACTION_ZERO_DEMAND_PCT` (0.5 %)

**What happens**:

| Signal | Value | Electrical Effect |
|--------|-------|-------------------|
| PWM    | `BTS7960_BRAKE_PWM` = 4249 (100 % duty) | Both FETs in the active half-bridge are driven ON |
| DIR    | Unchanged (last direction) | Irrelevant when PWM=100 % — both FETs conduct |
| EN     | HIGH | H-bridge is enabled |

**Result**: Motor terminals are shorted through the H-bridge. The motor acts as a generator loaded by its own winding resistance, producing **electromagnetic braking torque** that holds the vehicle stationary. No energy is returned to the battery.

**Exception — Neutral gear**: PWM=0, EN=LOW → motor floats freely (coast mode).

### 3.2 Brake / Hold Active (Park Gear)

**Condition**: `current_gear == GEAR_PARK`

**What happens**:

| Signal | Value | Electrical Effect |
|--------|-------|-------------------|
| PWM    | `hold_pct × PWM_PERIOD / 100` (default ≈ 1275 for 30 %) | Partial duty — pulsed motor drive |
| DIR    | HIGH (forward) | One half-bridge is pulsed |
| EN     | HIGH (if hold_pwm > 0), LOW (if derating forced hold to 0) | H-bridge enabled unless thermally derated |

**Result**: The motor receives a low-duty forward drive signal. This creates a modest holding torque that resists vehicle motion (simulated parking lock). Current and temperature are monitored; braking is progressively reduced or fully disabled to prevent motor overheating during extended hold.

**Safety override**: If the system enters SAFE or ERROR state while in Park, the hold brake is **released** (PWM=0, EN=LOW → coast) to allow the safety system full authority.

### 3.3 Emergency Stop

**Trigger**: `Traction_EmergencyStop()` called from safety system

**What happens**:

| Signal | Value | Electrical Effect |
|--------|-------|-------------------|
| PWM    | 0 | No drive signal |
| EN     | LOW (all 5 motors) | H-bridge **disabled** — FETs turned off |

**Sequence** (code order):
1. All five motors' EN pins driven LOW (coast mode)
2. All five motors' PWM set to 0
3. `traction_state.demandPct` forced to 0
4. Pedal filter state reset (`pedal_ema`, `pedal_ramped`, `pedal_filter_init`)
5. Dynamic braking state reset

**Result**: All H-bridges are **completely de-energized**. Motors enter coast mode (no braking, no drive). The firmware deliberately uses coast (EN=LOW) instead of active brake (PWM=100 %) because in an emergency the fault condition may involve the power stage itself (overcurrent, short circuit), and keeping FETs conducting would be unsafe.

**Additionally**: The `Error_Handler()` function uses direct register writes to force all GPIOC enable and relay pins LOW, providing a hardware-level backup shutdown independent of the HAL layer.

### 3.4 Error_Handler (Hard Fault / Unrecoverable)

```c
void Error_Handler(void) {
    __disable_irq();
    GPIOC->BSRR = (uint32_t)(PIN_EN_FL | PIN_EN_FR | PIN_EN_RL | PIN_EN_RR | PIN_EN_STEER
                  | PIN_RELAY_MAIN | PIN_RELAY_TRAC | PIN_RELAY_DIR) << 16U;
    while (1) { }
}
```

All motor enable pins and all relay control pins are driven LOW via direct register access (bypassing HAL). This cuts both the H-bridge logic enable AND the relay-supplied motor power buses. The system halts in an infinite loop with interrupts disabled.

---

## 4. Summary: BTS7960 Control Topology

```
                    ┌─────────────────────────────────────────┐
                    │           BTS7960 / IBT-2 Module        │
                    │                                         │
  MCU PWM pin ──────┤─► PWM input ──┬── RPWM (when DIR=1)    │
                    │               └── LPWM (when DIR=0)    │──── Motor
  MCU DIR pin ──────┤─► Direction select                      │
  MCU EN  pin ──────┤─► R_EN + L_EN (tied together)           │
                    │                                         │
                    │   R_IS ──── (not connected to MCU)      │
                    │   L_IS ──── (not connected to MCU)      │
                    └─────────────────────────────────────────┘
                                          │
                         Current measured by external INA226
                         via I2C (TCA9548A multiplexer)
```

**Key observations**:
1. The firmware uses a **DIR + single PWM** scheme, not dual-PWM (separate RPWM/LPWM from the MCU).
2. R_EN and L_EN are driven by a **single** MCU GPIO pin per motor (assumed tied together on the IBT-2 board).
3. The BTS7960's built-in current-sense outputs (R_IS, L_IS) are **not used**; external INA226 ICs provide current measurement.
4. Active braking uses **PWM=100 %** (not PWM=0) due to Chinese BTS7960 module behavior where PWM=0+EN=HIGH results in motor float rather than brake.
5. Emergency stop uses **EN=LOW** (coast/disconnect) to fully de-energize the H-bridge power stage.
