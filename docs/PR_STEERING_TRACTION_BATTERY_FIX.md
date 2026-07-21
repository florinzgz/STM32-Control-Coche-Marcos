# Field fixes: steering homing, battery profile and straight 4x4

This change set addresses faults observed on the physical vehicle after PR #434.

## Steering homing

- Every power cycle still requires a physical PB5 centre confirmation.
- Search authority is 100% PWM to overcome the installed RS390 gearbox friction.
- Current protection is not removed: CH5 plus encoder no-motion evidence remove PWM before reversing.
- End-stop pressure requires two genuinely fresh CH5 samples while the encoder has remained stationary for at least 100 ms; a single current spike while the mechanism is moving is not accepted as an end stop.
- LEFT end-stop detection always commands zero PWM, waits 100 ms, then starts RIGHT.
- The generic EPS overcurrent supervisor accepts the initial overcurrent sample only when normal EPS PWM, PC4 and PC12 are active in the same cycle. Homing current remains owned by the homing guard.
- After the overcurrent FSM itself isolates EPS, it may accept only a genuinely new CH5 sample to confirm that current disappeared or, when the residual threshold is calibrated, prove a persistent electrical hazard. An unrelated encoder, calibration, parameter or CH5 fault does not open this post-isolation overcurrent path.
- Retained samples are rejected by sample-sequence checks, and current-polarity diagnosis is ignored when no direction is intentionally commanded.
- PB5 centre completion is blocked when the A/B encoder is faulted and otherwise performs CENTERING -> NONE -> EPS without intentionally opening PC12.
- During the 100 ms reversal dead-time, requested-PWM telemetry reports zero, matching the physical output.

## Traction

- The base controller still calculates demand, ramps, limits, ABS/TCS and Ackermann.
- The calculation runs on shadow timer/GPIO registers and only the resolved result is written to physical BTS7960 outputs.
- Shadow timer CCR values, rather than zeroed legacy telemetry, preserve the bounded Neutral ramp at the physical output layer.
- 4x2 drives the installed rear axle RL/RR. The final RL/RR commands are capped again by their physical per-wheel safety scales, so rear temperature, ABS or TCS protection cannot be bypassed by the logical FL/FR calculation.
- In 4x4, when the steering angle is inside the 2-degree Ackermann deadband, every wheel is in DRIVE, all directions agree and no wheel limiter is active, the four physical outputs are equalised to the lowest resolved PWM. Equalisation can reduce stronger channels but can never raise a limited channel.
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

A valid raw battery sample below 16.00 V triggers SAFE immediately; the 500 ms filter is used for non-critical warning/derate and stable recovery behavior, never to delay the critical cutoff.

Battery limits may be saved in STANDBY or in a verified stationary ACTIVE/DEGRADED service condition: P/N, finite pedal below 3%, all finite wheel speeds strictly below 0.5 km/h, final resolved traction PWM zero and active-brake override disabled. DEGRADED service saving is accepted only when the active cause is battery undervoltage warning.
