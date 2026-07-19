# Field fixes: steering homing, battery profile and straight 4x4

This change set addresses faults observed on the physical vehicle after PR #434.

## Steering homing

- Every power cycle still requires a physical PB5 centre confirmation.
- Search authority is 100% PWM to overcome the installed RS390 gearbox friction.
- Current protection is not removed: CH5 + encoder motion cut PWM before reversing.
- LEFT end-stop detection always commands zero PWM, waits 100 ms, then starts RIGHT.
- The generic EPS overcurrent supervisor only accepts a CH5 sample when normal EPS PWM, PC4 and PC12 are active in the same cycle. It ignores retained current samples after the output has stopped and leaves homing end-stop handling to the homing guard.
- PB5 centre completion performs CENTERING -> NONE -> EPS without intentionally opening PC12.

## Traction

- The base controller still calculates demand, ramps, limits, ABS/TCS and Ackermann.
- The calculation runs on shadow timer/GPIO registers and only the resolved result is written to physical BTS7960 outputs.
- 4x2 drives the installed rear axle RL/RR.
- In 4x4, when the steering angle is inside the 2-degree Ackermann deadband and no wheel limiter is active, all four physical outputs are explicitly written with identical PWM and direction.
- 0x20C remains the authoritative per-wheel final PWM telemetry. If 0x20C is equal but only FR+RL rotate, the remaining fault is downstream hardware, driver enable, wiring or motor power.

## Battery

Installed pack display range:

- 16.00 V = 0%
- 28.50 V = 100%

Protection defaults:

- Warning: 18.00 V
- Derate: 17.00 V
- Cutoff: 16.00 V
- Recovery: 17.00 V
- Filter: 500 ms

Battery limits may be saved in STANDBY or in a verified stationary ACTIVE/DEGRADED service condition: P/N, pedal below 3%, all wheels below 0.5 km/h and final PWM zero. DEGRADED service saving is accepted only when the active cause is battery undervoltage warning.
