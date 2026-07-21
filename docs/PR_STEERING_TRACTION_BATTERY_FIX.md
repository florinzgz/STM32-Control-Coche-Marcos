# Field fixes: steering homing, battery profile and drivetrain output

This change set addresses faults observed on the physical vehicle after PR #434.

## Steering homing

- Every power cycle requires a physical PB5 centre confirmation.
- The active-low PB5 level is always evaluated while homing runs, even when the optional steering-centre service module is disabled. This also covers booting while already positioned over the centre target, where no new EXTI edge is guaranteed.
- Search authority is 100% PWM to overcome the installed RS390 gearbox friction.
- The normal end-stop path requires encoder no-motion for at least 100 ms plus two genuinely fresh CH5 samples at or above 8 A.
- The hard-current path requires two consecutive fresh CH5 samples at or above 20 A and removes PWM even if vibration or encoder noise still changes the count. A single current spike is never sufficient.
- LEFT end-stop detection always commands zero PWM, waits at least 100 ms, then starts RIGHT.
- The generic EPS overcurrent supervisor accepts an initial sample only when normal EPS PWM, PC4 and PC12 are active in the same cycle. Homing current remains owned by the homing guard.
- After the overcurrent FSM itself isolates EPS, it may accept only a genuinely new CH5 sample to confirm that current disappeared or, when the residual threshold is calibrated, prove a persistent electrical hazard. An unrelated encoder, calibration, parameter or CH5 fault does not open this post-isolation path.
- Retained samples are rejected by sample-sequence checks, and current-polarity diagnosis is ignored when no direction is intentionally commanded.
- PB5 completion is blocked when the A/B encoder is faulted and otherwise performs CENTERING -> NONE -> EPS without intentionally opening PC12.
- During the 100 ms reversal dead-time, requested-PWM telemetry reports zero, matching the physical output.
- The encoder-health baseline is refreshed once after successful physical centring, preventing the deliberate TIM2 zero from being misclassified as an impossible encoder jump.

## Traction

- The base controller still calculates demand, ramps, limits, ABS/TCS and Ackermann.
- The effective safety wrapper uses `Traction_IsWheelDriven()` so ABS/TCS treat RL/RR as the driven axle in 4x2 and all four wheels as driven in 4x4/tank mode.
- The calculation runs on shadow timer/GPIO registers and only the resolved result is written to physical BTS7960 outputs.
- Shadow timer CCR values, rather than zeroed legacy telemetry, preserve the bounded Neutral ramp at the physical output layer.
- 4x2 drives the installed rear axle RL/RR and leaves FL/FR in coast. Final RL/RR commands are capped again by their physical per-wheel safety scales.
- A future incompatible left/right 4x2 mode result fails closed to coast rather than being routed asymmetrically.
- In 4x4, equalisation is allowed only when the steering angle and all wheel scales are finite, the angle is inside the 2-degree Ackermann deadband, every wheel is in DRIVE, all directions agree and every scale is unity.
- Straight 4x4 equalisation uses the lowest resolved PWM. It may reduce stronger channels but can never raise a limited channel.
- `0x20C` remains the authoritative final PWM telemetry per physical wheel. If `0x20C` is equal but only FR+RL rotate, the remaining fault is downstream hardware, driver enable, wiring or motor power.

## Battery

Installed pack display range:

- 16.00 V = 0%
- 28.50 V = 100%

Protection defaults:

- Informational warning profile: 18.00 V
- Derate/DEGRADED limit: 17.00 V
- SAFE cutoff: 16.00 V
- SAFE recovery: 17.00 V
- Filter: 500 ms

A valid raw battery sample below Cutoff triggers SAFE immediately. The 500 ms filter is used for non-critical derate and stable recovery behaviour, never to delay the critical cutoff.

Stored values must satisfy `Cutoff < Limit <= Warning`; Recovery is an independent SAFE hysteresis threshold and must remain above Cutoff.

Battery limits may be saved in STANDBY or in a verified stationary ACTIVE/DEGRADED service condition: P/N, finite pedal below 3%, all finite wheel speeds strictly below 0.5 km/h, exact final traction PWM of zero ticks and active-brake override disabled. DEGRADED service saving is accepted only when the active cause is battery undervoltage warning.

## Validation contract

GitHub Actions is the source of truth for the current commit and must complete all of the following before physical testing:

- STM32 ARM build with warnings treated as errors.
- ESP32 PlatformIO build.
- STM32 and ESP32 host tests.
- Effective homing-wrapper integration test.
- Static analysis, integrity checks and CodeQL.

Physical validation remains mandatory before the PR is marked ready or merged:

- [ ] Vehicle immobilised or all driven wheels lifted.
- [ ] LEFT end stop -> PWM zero -> at least 100 ms -> RIGHT.
- [ ] Two fresh hard-current CH5 samples remove PWM while encoder counts still move.
- [ ] PB5 centre -> encoder zero -> EPS owner restored and PC12 remains available.
- [ ] 4x2 moves RL/RR only; FL/FR remain in coast.
- [ ] 4x4 straight reports equal FL/FR/RL/RR in `0x20C` and no diagonal command.
- [ ] Individual RL/RR limiter reduces the corresponding physical rear wheel.
- [ ] Battery Cutoff forces immediate SAFE; stationary P/N save accepts the requested profile.
