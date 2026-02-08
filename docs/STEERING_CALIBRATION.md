# Steering Calibration Model

## Chosen Model: Option A — Relative Logical Calibration

### How It Works

1. On power-up, `Steering_Init()` zeroes the TIM2 encoder counter at the
   current mechanical position and sets `steering_calibrated = 1`.
2. From that point on, every position is **relative** to the power-up
   position.  Encoder count 0 = wherever the steering was when the system
   started.
3. `Steering_SetAngle()` and `Steering_ControlLoop()` reject all commands
   until `steering_calibrated` is set, ensuring the motor never moves
   without a valid reference.

### Why It Is Safe

- The motor is physically prevented from moving before calibration:
  both `Steering_SetAngle()` and `Steering_ControlLoop()` return
  immediately (with the motor neutralised) when `steering_calibrated == 0`.
- If the encoder develops a fault at any time, `Steering_ControlLoop()`
  calls `Steering_Neutralize()` and the fault is latched — only a full
  system reset can clear it.
- No blind recentring or homing sequence is attempted.  The system never
  drives the motor toward an assumed position.

### What the System Does NOT Do (Limitations)

| Limitation | Explanation |
|---|---|
| **No absolute position reference** | There is no Z-index, limit switch, or hall sensor.  The encoder provides relative counts only. |
| **Power-cycle assumption** | The system assumes the steering has not been moved mechanically while powered off.  If it was, the "zero" will be wrong by that offset. |
| **No auto-centring** | The system never attempts to find centre on its own.  The operator must ensure the steering is centred before powering on. |
| **No persistent calibration** | The encoder zero is not stored in flash or EEPROM.  Every power cycle resets the reference. |

### Confirmation

- No limits, PID gains, safety logic, or CAN protocol were modified.
- No new hardware is assumed (no limit switches, brakes, or
  electromagnets).
- All changes are confined to `motor_control.c` / `motor_control.h`
  and this documentation file.

### Reference

Base firmware: <https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos>
(`steering_motor.cpp` — relative encoder, no Z-index usage).
