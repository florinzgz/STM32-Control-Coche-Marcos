# BTS7960 / IBT-2 Motor Driver Wiring & Usage Audit

> **Scope**: Extract-only audit of the current firmware behavior.
> **Source files examined**: `Core/Inc/main.h`, `Core/Src/main.c`,
> `Core/Src/stm32g4xx_hal_msp.c`, `Core/Inc/motor_control.h`,
> `Core/Src/motor_control.c`, `Core/Inc/safety_system.h`.
> **Last updated**: 2026-03-21 (aligned with RPWM/LPWM dual-PWM architecture)

---

## 1. Per-Motor Pin Mapping

The firmware uses **dual-PWM** (separate RPWM/LPWM) control — each BTS7960 half-bridge
receives its own independent PWM signal.  Direction is encoded as the **choice of active
channel**: RPWM active = forward, LPWM active = reverse.  Both channels of each motor
share the **same timer** so both CCR preload registers transfer at the same UEV
(update event), guaranteeing **zero overlap** between RPWM and LPWM.

The single write path `Motor_SetSigned()` enforces that RPWM and LPWM are **never
simultaneously non-zero**.  Direction-change dead-state enforcement inserts a zero-torque
intermediate state when reversing.

### 1.1 Traction Motors — Front (TIM1, advanced, 20 kHz center-aligned, ARR = 4249)

| Signal          | Motor FL                    | Motor FR                      |
|-----------------|-----------------------------|-------------------------------|
| **RPWM**        | PA8  — TIM1_CH1             | PA10 — TIM1_CH3               |
| **LPWM**        | PA9  — TIM1_CH2             | PC3  — TIM1_CH4 (AF2)        |
| **R_EN / L_EN** | PC5  — GPIO output (active) | PC0  — GPIO output (active)   |
| **R_IS / L_IS** | Not connected to MCU — current sensing via INA226 I2C (ch 0) | ← same (ch 1) |
| **Protection**  | TIM1 BREAK2 → Cortex LOCKUP | ← same                        |

### 1.2 Traction Motors — Rear (TIM8, advanced, 20 kHz center-aligned, ARR = 4249)

| Signal          | Motor RL                    | Motor RR                      |
|-----------------|-----------------------------|-------------------------------|
| **RPWM**        | PC6  — TIM8_CH1             | PC8  — TIM8_CH3               |
| **LPWM**        | PC7  — TIM8_CH2             | PC9  — TIM8_CH4               |
| **R_EN / L_EN** | PC1  — GPIO output (active) | PC13 — GPIO output (active)   |
| **R_IS / L_IS** | Not connected to MCU — current sensing via INA226 I2C (ch 2) | ← same (ch 3) |
| **Protection**  | TIM8 BREAK2 → Cortex LOCKUP | ← same                        |

### 1.3 Steering Motor (TIM3, general-purpose, 20 kHz center-aligned, ARR = 4249)

| Signal          | Motor Steering              |
|-----------------|-----------------------------|
| **RPWM**        | PA6  — TIM3_CH1             |
| **LPWM**        | PA7  — TIM3_CH2             |
| **R_EN / L_EN** | PC4  — GPIO output (active) |
| **R_IS / L_IS** | Not connected to MCU — current sensing via INA226 I2C (ch 5) |
| **Protection**  | Software only (TIM3 has no BREAK input; fault handlers zero CCR1/CCR2 directly) |

### 1.4 Current Sense (R_IS / L_IS — NOT USED)

The BTS7960 analog current-sense outputs (R_IS, L_IS) are **not wired to any MCU ADC input**.
All motor current measurement is performed by **external INA226 high-side current-sense ICs** connected via the I2C1 bus through a TCA9548A 8-channel multiplexer:

| TCA9548A Channel | INA226 Measures      | Shunt Resistor |
|------------------|----------------------|----------------|
| 0                | Motor FL current     | 1.5 mΩ (50 A) |
| 1                | Motor FR current     | 1.5 mΩ (50 A) |
| 2                | Motor RL current     | 1.5 mΩ (50 A) |
| 3                | Motor RR current     | 1.5 mΩ (50 A) |
| 4                | Battery 24 V bus     | 0.75 mΩ (100 A)|
| 5                | Steering motor       | 1.5 mΩ (50 A) |
| 6–7              | *Unused / unassigned in firmware* | —   |

### 1.5 Enable Pin Strategy

| Motor  | EN Pin  | Management                                                         |
|--------|---------|--------------------------------------------------------------------|
| FL     | PC5     | GPIO output — asserted (HIGH) when duty > 0, deasserted (LOW) when stopped |
| FR     | PC0     | GPIO output — asserted (HIGH) when duty > 0, deasserted (LOW) when stopped |
| RL     | PC1     | GPIO output — asserted (HIGH) when duty > 0, deasserted (LOW) when stopped |
| RR     | PC13    | GPIO output — asserted (HIGH) when duty > 0, deasserted (LOW) when stopped |
| STEER  | PC4     | GPIO output — asserted (HIGH) when duty > 0, deasserted (LOW) when stopped |

**Boot default**: PC0, PC1, PC4, PC5 and PC13 are initialized as GPIO_MODE_OUTPUT_PP in `MX_GPIO_Init()`.
HAL defaults these to GPIO_PIN_RESET (LOW = H-bridge disabled) — **safe on power-up**.

### 1.6 Freed Direction Pins (Legacy)

The original firmware used DIR + single PWM.  After the migration to RPWM/LPWM
dual-PWM control, the direction pins are freed and no longer driven:

| Pin | Former Function | Current Status          |
|-----|-----------------|-------------------------|
| PC0 | DIR_FL          | Freed — leave unconnected or GPIO_OUT LOW |
| PC1 | DIR_FR          | Freed — leave unconnected or GPIO_OUT LOW |
| PC2 | DIR_RL          | Freed — leave unconnected or GPIO_OUT LOW |
| PC3 | DIR_RR          | Freed — leave unconnected or GPIO_OUT LOW |
| PC4 | DIR_STEER       | Freed — leave unconnected or GPIO_OUT LOW |

---

## 2. Direction Implementation in Software

### 2.1 Scheme: Dual-PWM (RPWM / LPWM)

Each motor is controlled by **three signals** — two PWM channels and one enable:

| MCU Output       | BTS7960 Connection | Function                           |
|------------------|--------------------|------------------------------------|
| **RPWM** (TIMx_CHa) | BTS7960 RPWM input | Forward half-bridge PWM (0–4249)   |
| **LPWM** (TIMx_CHb) | BTS7960 LPWM input | Reverse half-bridge PWM (0–4249)   |
| **EN** (GPIO or tied) | BTS7960 R_EN + L_EN | H-bridge enable — HIGH active     |

**Source code** (`motor_control.c` — `Motor_SetSigned()`):

```c
// Forward: RPWM = duty, LPWM = 0
// Reverse: RPWM = 0, LPWM = duty
// Stop:    RPWM = 0, LPWM = 0

if (signed_pwm > 0) {
    __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);  // clear first
    __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, duty);
} else if (signed_pwm < 0) {
    __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);  // clear first
    __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, duty);
} else {
    __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);
    __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);
}
```

The `Motor_t` struct maps each motor's RPWM/LPWM timer channels and enable GPIO:

```c
motor_fl.rpwm_timer   = &htim1;  motor_fl.rpwm_channel = TIM_CHANNEL_1;  // PA8
motor_fl.lpwm_timer   = &htim1;  motor_fl.lpwm_channel = TIM_CHANNEL_2;  // PA9
motor_fl.en_port      = GPIOC;   motor_fl.en_pin       = PIN_EN_FL;      // PC5
```

### 2.2 How Direction Works per Gear

| Gear               | `effective_demand ≥ 0` → direction | `effective_demand < 0` → direction | Notes |
|--------------------|------------------------------------|------------------------------------|-------|
| FORWARD (D1)       | +1 (RPWM active)                  | −1 (LPWM active)                  | Max 60 % power |
| FORWARD_D2         | +1 (RPWM active)                  | −1 (LPWM active)                  | Max 100 % power |
| REVERSE            | −1 (inverted: LPWM active)        | +1 (inverted: RPWM active)        | Max 60 % power |
| PARK               | +1 (hold brake forward)           | N/A (no throttle accepted)         | Active hold brake |
| NEUTRAL            | N/A (both channels = 0)           | N/A                                | Coast — RPWM=0, LPWM=0 |

In tank-turn (axis rotation) mode, left-side motors (FL, RL) receive `−dir` while right-side motors (FR, RR) receive `+dir`.

### 2.3 Braking Mode Behavior

The firmware implements **three distinct braking modes**, all using the BTS7960 H-bridge:

#### A. BTS7960 Passive Brake (both PWM = 0)

**Constant**: `BTS7960_BRAKE_PWM = 0` (both RPWM and LPWM at 0)

With RPWM/LPWM direct control, passive brake is achieved by setting both channels to 0.
The BTS7960 internal logic then holds both motor terminals at the same potential, producing
electromagnetic braking.

**Used in**:
- Traction zero-demand hold (TRAC_PHASE_BRAKE state)
- 4×2 mode rear wheels (rear motors are braked while front motors drive)

#### B. Dynamic Braking (deceleration)

When the driver lifts off the throttle rapidly (throttle rate decreasing), the firmware generates a proportional braking effort:

```
brake_pct = |throttle_rate| × DYNBRAKE_FACTOR (0.5)
```

Clamped to `DYNBRAKE_MAX_PCT` (60 %). The brake is applied by setting `effective_demand = −dynbrake_pct` which activates the opposing LPWM channel relative to travel direction, creating opposing torque. Energy is dissipated as heat in motor windings — **no regenerative charging**.

Dynamic braking is **disabled** when:
- Average wheel speed < 3 km/h
- System state is SAFE, ERROR, BOOT, or STANDBY
- ABS is active on any wheel (`wheel_scale[i] < 1.0`)
- Gear is NEUTRAL or PARK

#### C. Park Hold Brake

In GEAR_PARK, all four motors receive a controlled low-duty active signal via RPWM:

- Default hold duty: 30 % (`PARK_HOLD_PWM_PCT`)
- Current derating: braking reduced above 15 A, disabled above 20 A
- Temperature derating: braking reduced above 70 °C, disabled above 85 °C
- Overridden (released) when system enters SAFE or ERROR state

---

## 3. Electrical Behavior in Special States

### 3.1 Throttle = 0 (Zero Demand — TRAC_PHASE_BRAKE)

| Signal | Value | Electrical Effect |
|--------|-------|-------------------|
| RPWM   | 0 (CCR = 0)   | Right half-bridge inactive |
| LPWM   | 0 (CCR = 0)   | Left half-bridge inactive  |
| EN     | HIGH (where GPIO-controlled) | H-bridge enabled    |

**Result**: Both RPWM=0 and LPWM=0 with EN=HIGH produces passive electromagnetic braking.

**Exception — Neutral gear**: RPWM=0, LPWM=0, EN not driven LOW explicitly but motor receives no torque → coast-like behavior.

**Exception — Coast phase**: When transitioning from DRIVE to low-speed, EN is deasserted (LOW) for motors with GPIO EN (FL, RR), providing true coast (free-rolling).

### 3.2 Brake / Hold Active (Park Gear)

| Signal | Value | Electrical Effect |
|--------|-------|-------------------|
| RPWM   | `hold_pct × PWM_PERIOD / 100` (default ≈ 1275 for 30 %) | Forward half-bridge pulsed |
| LPWM   | 0   | Reverse half-bridge inactive |
| EN     | HIGH (if hold_pwm > 0), LOW (if derating forced hold to 0) | H-bridge enabled unless thermally derated |

**Result**: The motor receives a low-duty forward drive signal via RPWM, creating a modest holding torque.

**Safety override**: If the system enters SAFE or ERROR state while in Park, the hold brake is **released** (RPWM=0, LPWM=0, EN=LOW → coast) to allow the safety system full authority.

### 3.3 Emergency Stop

**Trigger**: `Traction_EmergencyStop()` called from safety system

| Signal | Value | Electrical Effect |
|--------|-------|-------------------|
| RPWM   | 0 | No forward drive signal |
| LPWM   | 0 | No reverse drive signal |
| EN     | LOW (all 5 motors where GPIO-controlled) | H-bridge **disabled** — FETs turned off |

**Sequence** (code order):
1. All five motors: `Motor_SetSigned(motor, 0)` → RPWM=0, LPWM=0, EN=LOW
2. `traction_state.demandPct` forced to 0
3. Pedal filter state reset (`pedal_ema`, `pedal_ramped`, `pedal_filter_init`)
4. Dynamic braking state reset
5. Demand anomaly detection state reset
6. Smooth driving state reset (phase → BRAKE, jerk limiter → 0)

**Result**: All H-bridges are **completely de-energized**. Motors enter coast mode. The firmware deliberately uses coast (EN=LOW) instead of active brake because in an emergency the fault condition may involve the power stage itself (overcurrent, short circuit), and keeping FETs conducting would be unsafe.

### 3.4 Error_Handler (Hard Fault / Unrecoverable)

```c
void Error_Handler(void) {
    __disable_irq();
    TIM1->BDTR &= ~TIM_BDTR_MOE;   // Disable TIM1 outputs (FL, FR)
    TIM8->BDTR &= ~TIM_BDTR_MOE;   // Disable TIM8 outputs (RL, RR)
    TIM3->CCR1  = 0U;               // RPWM_STEER → 0
    TIM3->CCR2  = 0U;               // LPWM_STEER → 0
    GPIOC->BSRR = (uint32_t)(PIN_EN_FL | PIN_EN_RR
                  | PIN_RELAY_MAIN | PIN_RELAY_TRAC | PIN_RELAY_DIR) << 16U;
    GPIOB->BSRR = (uint32_t)(PIN_RELAY_LED | PIN_RELAY_LED_REAR) << 16U;
    while (1) { }
}
```

All motor PWM outputs are disabled (MOE cleared on TIM1/TIM8, CCRs zeroed on TIM3) and all relay/enable GPIO pins are driven LOW via direct register access (bypassing HAL). This cuts both the H-bridge PWM outputs AND the relay-supplied motor power buses. The system halts in an infinite loop with interrupts disabled.

---

## 4. Summary: BTS7960 Control Topology

```
                    ┌─────────────────────────────────────────┐
                    │           BTS7960 / IBT-2 Module        │
                    │                                         │
  MCU RPWM pin ────┤─► RPWM input ──── Right half-bridge     │
                    │                                         │──── Motor
  MCU LPWM pin ────┤─► LPWM input ──── Left  half-bridge     │
                    │                                         │
  MCU EN pin ──────┤─► R_EN + L_EN (tied or GPIO-controlled) │
  (or tied to 3.3V)│                                         │
                    │   R_IS ──── (not connected to MCU)      │
                    │   L_IS ──── (not connected to MCU)      │
                    │                                         │
                    │   VCC  ──── 3.3 V (from STM32 rail)    │
                    │   GND  ──── Common with STM32           │
                    └─────────────────────────────────────────┘
                                          │
                         Current measured by external INA226
                         via I2C (TCA9548A multiplexer)
```

**Key observations**:
1. The firmware uses **dual-PWM** (separate RPWM/LPWM from the MCU) — NOT DIR + single PWM.
2. Direction is encoded as the choice of active PWM channel: RPWM active = forward, LPWM active = reverse.
3. R_EN and L_EN are tied together on the IBT-2 module; only FL (PC5) and RR (PC13) have GPIO-controlled EN. The other three motors have EN permanently tied to 3.3 V.
4. The BTS7960's built-in current-sense outputs (R_IS, L_IS) are **not used**; external INA226 ICs provide current measurement.
5. `Motor_SetSigned()` is the **sole write path** to hardware timer CCR registers — guarantees RPWM and LPWM are never simultaneously non-zero.
6. OCPreload is enabled on all PWM channels; CCR updates take effect only at the next timer UEV (50 µs at 20 kHz).
7. TIM1/TIM8 have BREAK2 linked to Cortex-M4 LOCKUP signal for hardware-level output disable.
8. Emergency stop uses **RPWM=0, LPWM=0, EN=LOW** (coast/disconnect) to fully de-energize the H-bridge.
