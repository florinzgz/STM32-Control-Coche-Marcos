# Hardware Validation Procedure — Phase 1: Stability Foundation

> **Purpose:** Step-by-step physical test plan for real vehicle hardware.
> Validates every safety-critical subsystem that exists in firmware.
> Based strictly on the codebase and the exit criteria defined in `docs/PROJECT_MASTER_STATUS.md`.
>
> **Rules:**
> - No firmware modifications allowed
> - No debugging code to be added
> - Every test has a PASS/FAIL observable result
> - Only tests what exists in code today

---

## 1) TEST ENVIRONMENT SETUP

### Required Hardware Connected

| Component | Connection | Notes |
|-----------|-----------|-------|
| STM32G474RE Nucleo/custom board | USB (ST-Link) for serial monitor | Powers MCU via USB or external 24 V supply |
| ESP32-S3 with ST7796 480×320 TFT display | SPI (display), CAN bus to STM32 | USB serial for ESP32 log output |
| CAN bus wiring | STM32 PB8 (FDCAN1_RX) / PB9 (FDCAN1_TX) ↔ ESP32 GPIO4/GPIO5 | 500 kbps, 120 Ω termination resistors at both ends |
| CAN transceiver modules (×2) | One per MCU (e.g., SN65HVD230 or MCP2551) | Required for physical CAN signaling |
| 4× traction motors with H-bridge drivers | TIM1 CH1–CH4 PWM outputs | Wheels must be off the ground or on a stand |
| Steering motor with H-bridge driver | TIM8 CH3 PWM output | Connected to physical steering rack |
| E6B2-CWZ6C rotary encoder (4800 CPR) | TIM2 CH1 (PA0) / CH2 (PA1) quadrature inputs | Mechanically coupled to steering column |
| Inductive center sensor | PB5 (EXTI5) | Mounted at steering center position |
| 6× INA226 current sensors via TCA9548A I2C mux | I2C1 (PB6 SDA / PB7 SCL) | 4 motor channels + 1 battery + 1 spare |
| 5× DS18B20 temperature sensors | OneWire bus on PB0 | Placed on motors and ambient |
| Pedal potentiometer | PA3 (ADC1_IN4) | 0–3.3 V analog input |
| 4× wheel speed sensors (hall/reed) | EXTI pins (PA4, PA6, PA7, PB1) | One per wheel, pulse output |
| 3× relays (Main, Traction, Direction) | GPIO outputs on GPIOC | Control power to motor drivers |
| 24 V vehicle battery (or bench supply) | Power input | Provide adjustable voltage for undervoltage tests |

### Power Conditions

- Vehicle battery fully charged (≥ 24 V nominal) or bench power supply set to 24.0 V
- USB cables connected for serial monitoring on both STM32 and ESP32
- All motor drivers powered but wheels elevated off the ground (vehicle on jack stands)

### Required Tools

| Tool | Purpose |
|------|---------|
| Serial terminal (115200 baud) on STM32 ST-Link VCP | Monitor reset cause, boot log |
| Serial terminal (115200 baud) on ESP32 USB | Monitor ESP32 reset reason, runtime stats |
| CAN bus monitor / analyzer (e.g., PCAN-View, candump, or logic analyzer) | Capture and verify CAN frames (IDs, timing, payloads) |
| Multimeter | Verify supply voltages, relay states |
| Stopwatch or timer | Measure timeouts (centering, CAN loss, watchdog) |
| Oscilloscope (optional) | Verify PWM signals, encoder waveforms |

### Safe Vehicle Positioning

- **MANDATORY:** Vehicle must be on jack stands with all four wheels off the ground
- Steering rack must be free to move its full range without obstruction
- No persons or objects in the path of any moving wheel or steering linkage
- Emergency power disconnect switch accessible within arm's reach of the tester
- Fire extinguisher available near the test area

---

## 2) BOOT VALIDATION

### Test 2.1 — Reset Cause Reporting (STM32)

**Background:** `Boot_ReadResetCause()` reads `RCC->CSR` flags at boot before IWDG initialization. The result is stored and accessible via `Boot_GetResetCause()`. Flags: `RESET_CAUSE_POWERON` (bit 0), `RESET_CAUSE_SOFTWARE` (bit 1), `RESET_CAUSE_IWDG` (bit 2), `RESET_CAUSE_WWDG` (bit 3), `RESET_CAUSE_BROWNOUT` (bit 4), `RESET_CAUSE_PIN` (bit 5).

**Procedure:**

1. Power cycle the STM32 board (disconnect and reconnect power).
2. Observe serial output immediately after reset.
3. Verify the reset cause indicates power-on reset (bit 0 set).
4. Press the hardware NRST button on the Nucleo board.
5. Verify the reset cause indicates pin reset (bit 5 set).

| Condition | Expected Reset Cause | PASS/FAIL |
|-----------|---------------------|-----------|
| Cold power-on | `RESET_CAUSE_POWERON` (bit 0) reported | ☐ |
| NRST button press | `RESET_CAUSE_PIN` (bit 5) reported | ☐ |

### Test 2.2 — Reset Cause Reporting (ESP32)

**Background:** ESP32 logs reset reason to serial at boot using `esp_reset_reason()`.

**Procedure:**

1. Power cycle the ESP32.
2. Open serial monitor at 115200 baud.
3. Observe the boot log line: `[HMI] Reset reason: PowerOn` (or similar).
4. Trigger a software reset (if available) and verify the reason changes accordingly.

| Condition | Expected Serial Output | PASS/FAIL |
|-----------|----------------------|-----------|
| Cold power-on | `[HMI] Reset reason: PowerOn` | ☐ |

### Test 2.3 — Watchdog Behavior

**Background:** IWDG is configured with prescaler 32, reload 4095, giving ~4.1 s timeout. `HAL_IWDG_Refresh()` is called every main loop iteration. If the main loop stalls, the watchdog resets the MCU.

**Procedure:**

1. Power on the STM32 and confirm it reaches the main loop (CAN heartbeat 0x001 appears on the CAN bus).
2. Verify heartbeat messages arrive continuously at ~100 ms intervals.
3. Confirm no watchdog reset occurs during normal operation over a 60-second observation period:
   - Heartbeat alive counter (byte 0 of CAN ID 0x001) increments monotonically without restarting from 0 unexpectedly.
4. Verify that after a watchdog reset (observable if the system enters a fault loop), the reset cause reports `RESET_CAUSE_IWDG` (bit 2).

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Normal operation (60 s) | Heartbeat alive counter increments continuously, no unexpected reset to 0 | ☐ |
| Watchdog reset (if observed) | Reset cause includes `RESET_CAUSE_IWDG` flag | ☐ |

### Test 2.4 — FDCAN Peripheral Failure Does Not Brick System

**Background:** `MX_FDCAN1_Init()` is patched to return gracefully on failure, setting `fdcan_init_ok = false`. `CAN_Init()` skips activation when FDCAN is not initialized.

**Procedure:**

1. Disconnect the CAN transceiver from the STM32 (remove wiring from PB8/PB9).
2. Power on the STM32.
3. Observe via serial or SWD debugger that the system reaches the main loop.
4. Verify the watchdog does not reset the system (no repeated resets in serial log).
5. Verify the system stays in STANDBY state (no transition to ACTIVE since CAN heartbeat from ESP32 cannot arrive).
6. Reconnect the CAN transceiver and power cycle — verify normal operation resumes.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| CAN transceiver disconnected at boot | System reaches main loop, no reset loop, stays in STANDBY | ☐ |
| CAN transceiver reconnected + power cycle | Normal CAN heartbeat resumes, system can reach ACTIVE | ☐ |

### Test 2.5 — I2C Peripheral Failure Does Not Brick System

**Background:** `MX_I2C1_Init()` is patched to return gracefully on failure, setting `i2c_init_ok = false`. Sensors read 0 and boot validation stays failed.

**Procedure:**

1. Disconnect all I2C devices (INA226 sensors, TCA9548A mux) from the I2C1 bus (PB6/PB7).
2. Power on the STM32.
3. Observe via serial or SWD debugger that the system reaches the main loop.
4. Verify the watchdog does not reset the system.
5. Verify the system stays in STANDBY (boot validation fails due to current sensor checks reading 0).

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| All I2C devices disconnected at boot | System reaches main loop, no reset loop, stays in STANDBY | ☐ |

### Test 2.6 — System Always Reaches Main Loop

**Background:** After all init functions, the system must always reach `while(1)` and begin executing the main loop tiers (10 ms / 50 ms / 100 ms / 1000 ms).

**Procedure:**

1. With all hardware connected normally, power on the system.
2. Verify CAN heartbeat 0x001 appears on the CAN bus within 2 seconds of power-on.
3. Verify the heartbeat alive counter (byte 0) begins incrementing.
4. Verify the system state (byte 1) is either BOOT (0) briefly, then STANDBY (1).

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Normal power-on | CAN heartbeat 0x001 appears within 2 s, alive counter incrementing, state = STANDBY | ☐ |

---

## 3) CAN COMMUNICATION VALIDATION

### Test 3.1 — Heartbeat Behavior

**Background:** STM32 sends heartbeat on CAN ID 0x001 every 100 ms. Payload: byte 0 = alive counter, byte 1 = system state, byte 2 = fault flags, byte 3 = error code. ESP32 sends heartbeat on CAN ID 0x011 every 100 ms. Payload: byte 0 = alive counter.

**Procedure:**

1. Connect CAN analyzer to the bus.
2. Power on both STM32 and ESP32.
3. Capture CAN traffic for 10 seconds.
4. Verify STM32 heartbeat (0x001):
   - Arrives every 100 ms (±10 ms tolerance).
   - Alive counter (byte 0) increments by 1 each message and wraps at 255→0.
   - System state (byte 1) reflects current state (0=BOOT, 1=STANDBY, 2=ACTIVE, etc.).
   - Fault flags (byte 2) are 0x00 during normal operation.
5. Verify ESP32 heartbeat (0x011):
   - Arrives every 100 ms (±10 ms tolerance).
   - Alive counter (byte 0) increments by 1 each message.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| STM32 heartbeat 0x001 timing | Messages arrive every 100 ms ±10 ms | ☐ |
| STM32 heartbeat alive counter | Byte 0 increments monotonically, wraps 255→0 | ☐ |
| STM32 heartbeat state field | Byte 1 matches expected system state | ☐ |
| STM32 heartbeat fault flags (normal) | Byte 2 = 0x00 | ☐ |
| ESP32 heartbeat 0x011 timing | Messages arrive every 100 ms ±10 ms | ☐ |
| ESP32 heartbeat alive counter | Byte 0 increments monotonically | ☐ |

### Test 3.2 — Loss of ESP32 Communication Handling

**Background:** `Safety_CheckCANTimeout()` monitors ESP32 heartbeat. If no heartbeat is received within 250 ms, the system transitions to SAFE state. CAN heartbeat 0x001 byte 2 will show `CAN_TIMEOUT` fault flag.

**Procedure:**

1. Start with both systems running and communicating (system in ACTIVE or STANDBY).
2. Record the current system state from CAN heartbeat 0x001 byte 1.
3. Disconnect the ESP32 CAN bus wiring (remove one CAN wire to break communication).
4. Start a stopwatch at the moment of disconnection.
5. Monitor CAN heartbeat 0x001 from STM32:
   - Within 250 ms, byte 1 (system state) must change to SAFE (4).
   - Byte 2 (fault flags) must show CAN_TIMEOUT flag set.
6. Reconnect the ESP32 CAN wiring.
7. Verify the system recovers from SAFE state once ESP32 heartbeat resumes (recovery debounce: 500 ms clean heartbeat reception).

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| ESP32 disconnected | System transitions to SAFE (state=4) within 250 ms | ☐ |
| CAN timeout flag | Heartbeat byte 2 shows CAN_TIMEOUT fault flag set | ☐ |
| ESP32 reconnected | System recovers from SAFE after 500 ms of continuous heartbeat | ☐ |

### Test 3.3 — STM32 Safe Fallback Behavior

**Background:** When the STM32 enters SAFE state, `Safety_FailSafe()` is called: all motor PWM is set to 0, H-bridges are disabled via `Traction_EmergencyStop()`, and steering is centered if the encoder is healthy.

**Procedure:**

1. Bring the system to ACTIVE state (all boot validation checks passing).
2. Apply a small throttle command via ESP32.
3. Verify motors are spinning (observe wheel rotation or motor current on CAN 0x201).
4. Disconnect ESP32 CAN wiring to trigger CAN timeout.
5. Verify within 250 ms:
   - All motor PWM outputs go to 0 (wheels stop spinning).
   - CAN heartbeat state changes to SAFE (4).
   - Motor current readings on CAN 0x201 drop to 0.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| CAN timeout triggers SAFE | Motors stop, PWM = 0, state = SAFE within 250 ms | ☐ |
| Motor current drops | CAN 0x201 shows all motor currents near 0 | ☐ |

### Test 3.4 — Message Timing Expectations

**Background:** CAN messages are transmitted at defined intervals from the main loop tiers.

**Procedure:**

1. With both systems running in ACTIVE state, capture CAN traffic for 30 seconds.
2. Measure the interval between consecutive messages for each CAN ID.

| CAN ID | Expected Interval | Content | PASS/FAIL |
|--------|-------------------|---------|-----------|
| 0x001 (Heartbeat) | 100 ms | State, faults, error code | ☐ |
| 0x200 (Wheel speeds) | 100 ms | 4× speed values | ☐ |
| 0x201 (Motor currents) | 100 ms | 4× current values | ☐ |
| 0x202 (Temperatures) | 1000 ms | 5× temperature values | ☐ |
| 0x203 (Safety status) | 100 ms | ABS, TCS, error code | ☐ |
| 0x204 (Steering) | 100 ms | Angle, calibrated flag | ☐ |
| 0x205 (Traction scale) | 100 ms | 4× wheel reduction % | ☐ |
| 0x206 (Temp map) | 1000 ms | 5× mapped temperatures | ☐ |
| 0x207 (Battery) | 100 ms | Current, voltage | ☐ |
| 0x011 (ESP32 heartbeat) | 100 ms | Alive counter | ☐ |

---

## 4) STEERING VALIDATION

### Test 4.1 — Encoder Counting Correctness

**Background:** TIM2 is configured in quadrature encoder mode for the E6B2-CWZ6C (4800 counts per revolution). Encoder count is a 32-bit value. CAN diagnostic frame 0x300 (tag 0x10) transmits raw count and delta every 1 second.

**Procedure:**

1. Power on the system with the encoder connected.
2. Monitor CAN diagnostic frame 0x300 for encoder data (1 Hz, tag byte = 0x10).
3. Manually rotate the steering shaft one full revolution clockwise.
4. Verify the encoder count changes by approximately 4800 counts.
5. Rotate one full revolution counterclockwise.
6. Verify the count returns to approximately the starting value.
7. Verify the delta field (count difference since last report) reflects the movement direction and magnitude.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| One full CW revolution | Encoder count increases by ~4800 counts (CAN 0x300) | ☐ |
| One full CCW revolution | Encoder count decreases by ~4800 counts (CAN 0x300) | ☐ |
| Delta field accuracy | Delta reflects direction and magnitude of movement | ☐ |

### Test 4.2 — Centering Procedure

**Background:** `SteeringCentering_Step()` runs a non-blocking state machine: sweep left at 10 % PWM (850/8499), if center sensor (PB5 inductive) not found, sweep right. When the inductive sensor triggers (EXTI5), the encoder is zeroed and steering is calibrated. Total timeout: 10 seconds. Stall timeout: 300 ms.

**Procedure:**

1. Before power-on, manually position the steering away from center (at least 20° off).
2. Power on the STM32 with all hardware connected.
3. Observe the steering motor begin sweeping (first to the left).
4. Verify the inductive center sensor triggers (steering stops at center position).
5. Measure the total centering time from power-on to completion.
6. Monitor CAN 0x204 byte 2 (calibrated flag):
   - Before centering: calibrated = 0.
   - After centering: calibrated = 1.
7. Verify centering completes within the 10-second timeout.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Sweep starts automatically at boot | Steering motor moves (left first) at low PWM | ☐ |
| Center sensor detection | Steering stops at center position, motor PWM goes to 0 | ☐ |
| Calibrated flag on CAN | CAN 0x204 byte 2 changes from 0 to 1 | ☐ |
| Centering completes within 10 s | Total centering time < 10 seconds | ☐ |

### Test 4.3 — Centering Fault Handling

**Procedure:**

1. Disconnect the inductive center sensor from PB5.
2. Power on the STM32.
3. Observe the steering motor sweep left, then right.
4. Verify that after both sweeps fail to find center (or after 10 s timeout), the centering state machine enters FAULT.
5. Verify the system enters DEGRADED state (L1) with a CENTERING error.
6. Verify the steering is neutralized (PWM = 0).

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Center sensor missing | Centering times out or both sweeps fail → FAULT | ☐ |
| System enters DEGRADED | CAN heartbeat byte 1 = DEGRADED (3) | ☐ |
| Steering neutralized | Steering motor PWM = 0 | ☐ |

### Test 4.4 — Range Protection

**Background:** Steering has a maximum range of ±74° (±987 encoder counts at 4800 CPR). `Safety_ValidateSteering()` clamps commands to this range. Rate limiting enforces a maximum of 200 °/s.

**Procedure:**

1. With the system in ACTIVE state and steering calibrated:
2. Send a steering command via ESP32 to the maximum angle (+74°).
3. Verify the steering moves to the limit and stops.
4. Send a command beyond the limit (e.g., +90°).
5. Verify the command is clamped — steering does not exceed ±74°.
6. Monitor CAN 0x204 steering angle and verify it never exceeds ±74° (±740 in 0.1° units).

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Command to +74° | Steering reaches limit, CAN 0x204 shows ≤740 (0.1° units) | ☐ |
| Command beyond ±74° | Steering does not exceed ±74°, value clamped | ☐ |

### Test 4.5 — Encoder Fault Detection (Jump)

**Background:** `Encoder_CheckHealth()` detects jumps > 100 counts per 10 ms cycle. A fault is latched and triggers DEGRADED L1.

**Procedure:**

1. With the system in ACTIVE state:
2. Rapidly disconnect and reconnect one encoder signal wire (A or B channel) momentarily to simulate an encoder jump.
3. Observe CAN heartbeat 0x001:
   - System state changes to DEGRADED (3).
   - Fault flags show encoder fault.
4. Verify steering is disabled (motor PWM = 0).

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Encoder signal interrupted briefly | System transitions to DEGRADED, steering disabled | ☐ |

### Test 4.6 — Encoder Fault Detection (Frozen)

**Background:** If the steering motor is active (> 10 % PWM) but the encoder count does not change for 200 ms, a frozen-encoder fault is declared.

**Procedure:**

1. With the system in ACTIVE state:
2. Send a steering command to move the steering.
3. While the motor is driving, physically block the steering (hold it firmly) so the encoder count remains constant while PWM > 10 %.
4. After approximately 200 ms, observe:
   - System transitions to DEGRADED.
   - Steering motor PWM goes to 0.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Steering physically blocked while motor driving | System enters DEGRADED after ~200 ms, steering disabled | ☐ |

### Test 4.7 — Encoder Fault Detection (Out-of-Range)

**Background:** Encoder count beyond ±6000 counts from center is flagged as out-of-range during centering. If the encoder reaches a range fault during normal operation, the steering is disabled.

**Procedure:**

1. During the centering procedure, manually force the steering to an extreme position beyond normal range.
2. Verify the centering procedure aborts with a range FAULT (exceeding ±6000 count guard).
3. Verify the system enters DEGRADED.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Encoder count exceeds ±6000 during centering | Centering aborts with FAULT, system DEGRADED | ☐ |

---

## 5) MOTOR & TRACTION SAFETY

### Test 5.1 — No Unintended Movement at Boot

**Background:** At boot, the system is in BOOT state, then STANDBY. Motors must not drive until the system reaches ACTIVE state. `Motor_Init()` starts PWM timers but sets all duty cycles to 0. Relays are not energized until `Relay_PowerUp()` is called during STANDBY→ACTIVE transition.

**Procedure:**

1. Position all wheels where rotation is visible.
2. Power on the system.
3. Observe all four wheels for the first 10 seconds after power-on.
4. Verify no wheel moves during BOOT or STANDBY states.
5. Verify relays remain de-energized (verify with multimeter: no voltage on motor driver power inputs).
6. Monitor CAN 0x200 (wheel speeds) and 0x201 (motor currents) — all values must be 0.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| No wheel movement during BOOT | All wheels stationary, CAN 0x200 all zeros | ☐ |
| No wheel movement during STANDBY | All wheels stationary, CAN 0x201 all zeros | ☐ |
| Relays de-energized | No voltage on motor driver power inputs | ☐ |

### Test 5.2 — Relay Power Sequencing

**Background:** `Relay_SequencerUpdate()` powers relays in order: Main relay ON → 50 ms wait → Traction relay ON → 20 ms wait → Direction relay ON. This sequence runs when transitioning to ACTIVE.

**Procedure:**

1. Bring the system to STANDBY (all boot validation checks passing).
2. Trigger transition to ACTIVE (ESP32 heartbeat present, centering complete, boot validation passed).
3. Monitor relay GPIO outputs (GPIOC) with a multimeter or oscilloscope:
   - Main relay energizes first.
   - After 50 ms, Traction relay energizes.
   - After an additional 20 ms, Direction relay energizes.
4. Verify all three relays are energized in sequence.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Main relay first | Main relay GPIO goes HIGH first | ☐ |
| Traction relay after 50 ms | Traction relay GPIO goes HIGH ~50 ms after Main | ☐ |
| Direction relay after 20 ms | Direction relay GPIO goes HIGH ~20 ms after Traction | ☐ |

### Test 5.3 — Brake and Neutral Behavior

**Background:** In gear N (neutral), motor PWM = 0, wheels coast freely. In gear P (park), active hold brake is applied at 30 % PWM with current/temperature derating.

**Procedure:**

1. With the system in ACTIVE state:
2. Set gear to N via ESP32.
3. Verify all motor PWM = 0 and wheels spin freely when pushed by hand.
4. Set gear to P via ESP32.
5. Verify wheels resist rotation (active brake hold applied).
6. Monitor CAN 0x201 (motor currents) — expect non-zero current in P (park hold), zero in N.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Gear N: coast | Motor PWM = 0, wheels spin freely when pushed | ☐ |
| Gear P: park hold | Wheels resist rotation, CAN 0x201 shows holding current | ☐ |

### Test 5.4 — Safe Reaction to Invalid Inputs

**Background:** `Safety_ValidateThrottle()` clamps throttle to [0, 100]. `Safety_ValidateSteering()` enforces rate limit (200 °/s) and range (±74°). `Safety_ValidateModeChange()` rejects gear changes when average wheel speed > 1 km/h. `Traction_SetDemand()` detects anomalous step rates and frozen pedal values.

**Procedure:**

1. With the system in ACTIVE, gear D1:
2. Send a throttle command of 150 % (out of range) via CAN — verify it is clamped to 100 %.
3. Send a negative throttle command (e.g., -10 %) via CAN — verify it is clamped to 0 %.
4. With wheels spinning at > 1 km/h, send a gear change command to R (reverse).
5. Verify the gear change is rejected (gear remains D1).
6. Monitor that no unexpected motor behavior occurs during any invalid input.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Throttle > 100 % | Clamped to 100 %, no motor runaway | ☐ |
| Throttle < 0 % | Clamped to 0 %, motor stops | ☐ |
| Gear D1→R at speed > 1 km/h | Gear change rejected, stays D1 | ☐ |

### Test 5.5 — Emergency Stop

**Background:** `Traction_EmergencyStop()` sets all motor PWM to 0, disables H-bridges, and triggers relay power-down. Called when system enters ERROR state.

**Procedure:**

1. With the system in ACTIVE and motors running at moderate throttle:
2. Trigger a condition that causes ERROR state (e.g., induce overcurrent on 3+ consecutive cycles by momentarily stalling a motor).
3. Verify all motors stop immediately.
4. Verify relays de-energize.
5. Verify CAN heartbeat state = ERROR (5).
6. Verify the system does not recover without a manual power cycle.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Emergency stop triggered | All motors stop, relays off, state = ERROR | ☐ |
| Non-recoverable | System stays in ERROR until power cycle | ☐ |

---

## 6) SENSOR FAILSAFE BEHAVIOR

### Test 6.1 — Missing Temperature Sensors

**Background:** `OW_SearchAll()` runs at init and discovers DS18B20 sensors. If no sensors are found, `Temperature_ReadAll()` uses fallback single-sensor read (Skip ROM). Temperatures read 0.0 °C. Boot validation checks for plausible temperatures (-40 °C to +125 °C, not all 0.0 °C). All-zero temps cause boot validation to fail.

**Procedure:**

1. Disconnect all 5 DS18B20 temperature sensors from the OneWire bus (PB0).
2. Power on the system.
3. Verify the system reaches the main loop (CAN heartbeat appears).
4. Verify the system stays in STANDBY — boot validation fails because all temperatures are 0.0 °C.
5. Monitor CAN 0x202 (temperatures) — all values should be 0.
6. Verify no watchdog reset occurs.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| All temp sensors missing | System stays in STANDBY, CAN 0x202 all zeros | ☐ |
| No reset loop | Heartbeat continues, alive counter increments | ☐ |

### Test 6.2 — Missing Current Sensors (I2C Failure)

**Background:** If I2C devices are not responding, `Current_ReadAll()` experiences timeouts. After 3 consecutive I2C failures, `I2C_BusRecovery()` is invoked. After 2 recovery attempts fail, the system transitions to SAFE. Current values read 0.

**Procedure:**

1. Disconnect the TCA9548A I2C multiplexer (or all INA226 sensors) from the I2C bus.
2. Power on the system.
3. Verify the system reaches the main loop (CAN heartbeat appears).
4. Verify the system stays in STANDBY — boot validation fails because current readings are invalid.
5. Monitor CAN 0x201 (motor currents) — values should be 0.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| I2C sensors missing at boot | System stays in STANDBY, CAN 0x201 all zeros | ☐ |
| No reset loop | Heartbeat continues, alive counter increments | ☐ |

### Test 6.3 — I2C Bus Recovery After Injected Failure

**Background:** `I2C_BusRecovery()` implements NXP AN10216 SCL clock cycling: DeInit I2C → toggle SCL 16× (5 µs low, 5 µs high) → generate STOP condition → re-Init I2C. Triggered after 3 consecutive I2C failures.

**Procedure:**

1. Start with the system running normally in ACTIVE state (all sensors responding).
2. Momentarily hold SDA low (connect PB6 to GND through a 100 Ω resistor for approximately 500 ms, then release).
3. This should cause I2C failures, triggering the bus recovery mechanism.
4. After release, verify I2C communication resumes:
   - CAN 0x201 shows valid current readings again.
   - CAN 0x207 shows valid battery voltage again.
5. The system should recover to its previous state (ACTIVE or DEGRADED depending on the recovery attempt count).

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| SDA held low then released | I2C failures detected (current readings drop to 0 temporarily) | ☐ |
| Bus recovery executes | I2C communication resumes, valid readings return on CAN 0x201/0x207 | ☐ |
| System recovers | System returns to ACTIVE or DEGRADED (not stuck in SAFE) | ☐ |

### Test 6.4 — Missing Encoder

**Background:** If the encoder is disconnected, `Encoder_CheckHealth()` detects a frozen count. The system enters DEGRADED with steering disabled.

**Procedure:**

1. Disconnect the encoder signals (TIM2 CH1/CH2 on PA0/PA1) before power-on.
2. Power on the system.
3. Verify the system reaches the main loop.
4. Verify the centering procedure fails (encoder not responding → FAULT).
5. Verify the system enters DEGRADED state.
6. Verify steering motor PWM = 0 (steering disabled).

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Encoder disconnected at boot | Centering fails, system DEGRADED, steering disabled | ☐ |
| No reset loop | Heartbeat continues, alive counter increments | ☐ |

### Test 6.5 — Stuck Pedal Detection

**Background:** `Traction_SetDemand()` includes a frozen-pedal detector. If the pedal value remains unchanged for an extended period while the vehicle is in motion, the system flags an anomaly.

**Procedure:**

1. With the system in ACTIVE, gear D1:
2. Apply throttle via the pedal potentiometer and hold it at a fixed position while the vehicle is in motion.
3. Observe whether the system flags a frozen-pedal anomaly after the detection period.
4. Verify the system response: demand may be reduced or an anomaly flag set.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Pedal held constant at non-zero while moving | Frozen-pedal anomaly flagged or demand clamped | ☐ |

### Test 6.6 — Wheel Speed Sensor Missing

**Background:** Wheel speed sensors use EXTI pulse counting with 1 ms debounce. If a sensor is disconnected, `Wheel_ComputeSpeed()` returns 0 for that wheel.

**Procedure:**

1. Disconnect one wheel speed sensor (e.g., front-left on PA4).
2. Power on the system and bring it to ACTIVE.
3. Apply throttle — observe all four wheels spinning.
4. Monitor CAN 0x200 (wheel speeds):
   - The disconnected wheel should report 0 km/h.
   - Other wheels report non-zero speed values.
5. Verify ABS/TCS behavior: with one wheel reading 0 and others reading non-zero, the system may flag a slip condition (expected behavior).

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| One wheel speed sensor disconnected | That wheel reads 0 km/h on CAN 0x200, others show speed | ☐ |
| System remains operational | No reset, system stays in ACTIVE or DEGRADED | ☐ |

---

## 7) DISPLAY CONSISTENCY

### Test 7.1 — Boot Screen Reflects CAN Link Status

**Background:** The ESP32 boot screen shows "CAN: LINKED" (green) when heartbeat is received within 500 ms, or "CAN: WAITING..." (red) when not.

**Procedure:**

1. Power on the ESP32 without the STM32 connected.
2. Verify the display shows: "COCHE MARCOS", "HMI v1.0", and **"CAN: WAITING..."** in red.
3. Now power on the STM32 (connect CAN bus).
4. Within 1 second, verify the display changes to **"CAN: LINKED"** in green.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| STM32 not connected | Display shows "CAN: WAITING..." in red | ☐ |
| STM32 connected | Display changes to "CAN: LINKED" in green | ☐ |

### Test 7.2 — Screen Transitions Follow STM32 State

**Background:** The ESP32 `ScreenManager` transitions screens based on `systemState` from CAN heartbeat 0x001 byte 1: BOOT(0)→Boot screen, STANDBY(1)→Standby screen, ACTIVE(2)/DEGRADED(3)→Drive screen, SAFE(4)→Safe screen, ERROR(5)→Error screen.

**Procedure:**

1. Power on both systems with all hardware connected.
2. Observe display transitions:
   - **Boot screen** appears first (system in BOOT/STANDBY).
   - **Standby screen** appears showing temperatures and fault flags when state = STANDBY.
   - When all boot validation checks pass and the system transitions to ACTIVE, the **Drive screen** appears showing speed, torque, steering, battery, pedal, gear.
3. Disconnect ESP32 CAN wiring to trigger SAFE:
   - Within 250 ms, the **Safe screen** must appear with amber "SAFE MODE" banner, "Actuators inhibited" text, and fault flags displayed.
4. Reconnect CAN and verify transition back from Safe screen once system recovers.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| BOOT/STANDBY → Boot/Standby screen | Correct screen displayed | ☐ |
| ACTIVE → Drive screen | Drive screen with live telemetry | ☐ |
| SAFE → Safe screen | Amber "SAFE MODE" banner, fault flags shown | ☐ |
| Recovery from SAFE | Screen returns to appropriate state | ☐ |

### Test 7.3 — Fault Flags Displayed on Safe Screen

**Background:** The Safe screen displays fault flags as hex value and error code. Fault flags byte from heartbeat 0x001 byte 2 is shown.

**Procedure:**

1. Trigger a CAN timeout fault (disconnect ESP32 CAN briefly, then observe SAFE screen via alternate means — or record the state and reconnect to see the Safe screen).
2. On the Safe screen, verify:
   - Fault flags hex value is non-zero and corresponds to the triggered fault (CAN_TIMEOUT flag).
   - Error code is displayed ("Code: X").
   - "Actuators inhibited" text is shown.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| CAN timeout fault displayed | Fault flags hex shows CAN_TIMEOUT bit, error code matches | ☐ |

### Test 7.4 — Error Screen on Critical Fault

**Background:** The Error screen shows red background, "SYSTEM ERROR", "Manual reset required", fault flags, error code, and diagnostic information.

**Procedure:**

1. Trigger an ERROR state condition (e.g., emergency stop via repeated overcurrent).
2. Verify the Error screen appears:
   - Red background.
   - "SYSTEM ERROR" header.
   - "Manual reset required" instruction.
   - Fault flags and error code displayed.

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| ERROR state → Error screen | Red background, "SYSTEM ERROR", fault details shown | ☐ |

### Test 7.5 — Drive Screen Shows Live Telemetry

**Background:** The Drive screen shows speed, torque %, temperatures, steering gauge, battery indicator, pedal bar, and gear display — all from CAN telemetry.

**Procedure:**

1. With the system in ACTIVE on the Drive screen:
2. Apply throttle — verify pedal bar fills proportionally.
3. Turn steering — verify steering gauge moves correspondingly.
4. Observe battery indicator — verify it shows a reasonable percentage (24 V ≈ ~80–100 %).
5. Verify temperature values update (may take up to 1 s since temperatures are sent at 1000 ms intervals).

| Condition | Observable Evidence | PASS/FAIL |
|-----------|-------------------|-----------|
| Pedal bar responds to throttle | Bar fills/empties proportionally to pedal input | ☐ |
| Steering gauge responds | Gauge moves with physical steering position | ☐ |
| Battery indicator shows valid % | Percentage corresponds to measured voltage | ☐ |
| Temperature values update | Values change when temperature changes (up to 1 s delay) | ☐ |

---

## 8) PHASE 1 EXIT CRITERIA CHECKLIST

All items below must PASS to consider Phase 1 — Stability Foundation completed. These criteria are taken directly from `docs/PROJECT_MASTER_STATUS.md` section 5, Phase 1 exit criteria, and validated by the tests in this document.

| # | Exit Criterion | Validated By | PASS/FAIL |
|---|---------------|-------------|-----------|
| 1 | **STM32 transitions BOOT → STANDBY → ACTIVE on real hardware with all 6 boot validation checks passing** | Test 2.6 (main loop reached), Test 5.1 (no movement at boot), Test 5.2 (relay sequencing). Boot validation checks: temperature plausible, current plausible, encoder healthy, battery ≥ 20 V, no safety error, CAN not bus-off. | ☐ |
| 2 | **CAN heartbeat loss correctly triggers SAFE within 250 ms** | Test 3.2 (ESP32 disconnection → SAFE within 250 ms), Test 3.3 (motor stop on CAN loss) | ☐ |
| 3 | **Steering centering completes within 10 s timeout on physical steering rack** | Test 4.2 (centering procedure completes < 10 s, calibrated flag set) | ☐ |
| 4 | **IWDG does not trigger during normal operation (all loop tiers complete within timing budgets)** | Test 2.3 (60 s normal operation, no watchdog reset, alive counter monotonic) | ☐ |
| 5 | **I2C bus recovery successfully restores communication after injected SDA hold-low** | Test 6.3 (SDA held low → bus recovery → valid readings resume) | ☐ |

### Additional Stability Validation (must all pass)

| # | Stability Check | Validated By | PASS/FAIL |
|---|----------------|-------------|-----------|
| 6 | **FDCAN init failure does not brick the system** | Test 2.4 | ☐ |
| 7 | **I2C init failure does not brick the system** | Test 2.5 | ☐ |
| 8 | **System always reaches main loop** | Test 2.6 | ☐ |
| 9 | **Reset cause is correctly reported** | Test 2.1, Test 2.2 | ☐ |
| 10 | **CAN message timing meets specification** | Test 3.4 | ☐ |
| 11 | **Steering encoder counts correctly** | Test 4.1 | ☐ |
| 12 | **Steering range protection works** | Test 4.4 | ☐ |
| 13 | **Encoder fault detection works (jump, frozen, out-of-range)** | Test 4.5, Test 4.6, Test 4.7 | ☐ |
| 14 | **No unintended motor movement at boot** | Test 5.1 | ☐ |
| 15 | **Safe reaction to invalid inputs** | Test 5.4 | ☐ |
| 16 | **Missing sensors do not cause reset loops** | Test 6.1, Test 6.2, Test 6.4 | ☐ |
| 17 | **ESP32 display reflects STM32 state changes** | Test 7.1, Test 7.2, Test 7.3 | ☐ |

---

**Sign-off:**

| Role | Name | Date | Signature |
|------|------|------|-----------|
| Tester | | | |
| Reviewer | | | |
| Project Lead | | | |
