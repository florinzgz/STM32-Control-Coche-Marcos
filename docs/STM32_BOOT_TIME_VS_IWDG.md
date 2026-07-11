# STM32 Boot Time vs. IWDG Window

> **STATUS: PENDING PHYSICAL MEASUREMENT.** The IWDG window below is derived from the
> firmware configuration (real, traced to source). The **boot-time budget and margin are an
> ENGINEERING ESTIMATE, not a measured value** — the "Boot time (IWDG-start → first refresh)"
> figures in §4 MUST be captured on real hardware before any IWDG-margin claim is treated as
> confirmed. Already-emitted boot instrumentation: `0x312 DIAG_BOOT_RESET` (uptime + RCC
> reset-cause bitmask, 1 Hz) and the `boot_phase` stage markers (`main.c`, SWD-readable). The
> boot-to-first-refresh **duration** itself is captured on the bench via the Method A / B
> procedure in §4 and is **not** yet a firmware-emitted telemetry value.

**Scope:** Item 6 of the instrumentation-first PR. Documents the real, firmware-derived
independent-watchdog (IWDG) window and the boot-time budget that must fit inside it, plus a
hardware measurement procedure. **No firmware behaviour is changed by this document** — it records
what the current code already does and how to verify it on the bench.

All figures below are traced to source; nothing is invented. Where a value can only be obtained by
measuring on real hardware, a blank cell is provided for the operator to fill in — it is **not**
pre-populated with a guessed number.

---

## 1. IWDG configuration (as built)

Source: `Core/Src/main.c` `MX_IWDG_Init()` (≈ line 1470).

| Field | Value | Source |
|-------|-------|--------|
| Instance | `IWDG` | `main.c:1472` |
| Clock | LSI (≈ 32 kHz), internal | RM0440 |
| Prescaler | `IWDG_PRESCALER_32` → LSI / 32 = **1 kHz** tick | `main.c:1473` |
| Reload | `4095` counts | `main.c:1480` |
| Window | `IWDG_WINDOW_DISABLE` (no lower bound) | `main.c:1481` |

**Nominal timeout:** `4095 counts × (1 / 1000 Hz)` = **4.095 s**.

**Real-world range:** the LSI is rated ±5 % over temperature and supply (RM0440), so the effective
timeout is approximately **3.9 – 4.3 s**. All boot-time budgeting below is sized against the
**worst-case (shortest) 3.9 s** bound. The comment at `main.c:1474-1479` records the same reasoning.

Because the window is **disabled**, there is no *minimum* time before a refresh is allowed — the
firmware may refresh as early and as often as it likes. Only the *maximum* interval between refreshes
matters.

---

## 2. Where the watchdog starts and is first kicked

| Event | Location | Notes |
|-------|----------|-------|
| `MX_IWDG_Init()` — watchdog **starts counting** | `main.c:262` | Once started, IWDG cannot be stopped in software. |
| First `HAL_IWDG_Refresh(&hiwdg)` | `main.c:866` (end of the first main-loop iteration) | Refreshed **every** loop iteration thereafter. |

The critical interval is therefore **from `main.c:262` to the first execution of `main.c:866`** — the
remaining single-shot boot initialisation plus the post-init CAN-status LED animation. Everything that
runs *before* `MX_IWDG_Init()` (early FDCAN bring-up and the first three boot blinks, `main.c:247-254`)
adds to total power-on-to-drivable time but does **not** count against the watchdog window.

---

## 3. Boot-time budget between IWDG start and first refresh

The only **blocking** delays between `MX_IWDG_Init()` (262) and the first refresh (866) are the
LED status-blink `HAL_Delay()` calls in `main.c`. Verified by inspection: the module initialisers
(`Motor_Init`, `Traction_Init`, `Steering_Init`, `Sensor_Init`, `Safety_Init`, `ServiceMode_Init`,
`SteeringCentering_Init`, `ErrorLog_Init`) and the persistent-store loaders (`SteeringCal_Init`,
`SensorMapStore_Init`, `PedalCal_Init`, `GearLimitsStore_Init`, …) contain **no** `HAL_Delay()` calls —
they perform register writes and flash reads that complete in well under a millisecond each.

| Delay | Source line | CAN-OK path | CAN-FAIL path |
|-------|-------------|-------------|---------------|
| Visual separator after boot blinks | `main.c:286` | 500 ms | 500 ms |
| CAN-OK long blink (ON) | `main.c:291` | 600 ms | — |
| CAN-OK gap | `main.c:293` | 300 ms | — |
| CAN-FAIL 5× rapid blink (ON+OFF) | `main.c:298,300` | — | 5 × (150 + 150) = 1500 ms |
| **Explicit blocking total** | | **≈ 1.40 s** | **≈ 2.00 s** |

Additional, non-blocking contributions (each ≪ 10 ms on a healthy bus, no fixed delay):
- Flash-store reads (steering-cal, sensor-map, pedal-cal, gear-limits, drive-tuning, battery-limits).
- Module register initialisation.
- **I²C sensor probing** in `Sensor_Init` uses `HAL_I2C_*` calls with **bounded HAL timeouts**. On a
  healthy bus these return in a few ms; a stuck/locked bus can extend this up to the HAL timeout and is
  separately surfaced as **Error Code 11 / `SAFETY_ERROR_I2C_FAILURE`** (see `SENSOR_INTERFACE.md`).
  This is the single most likely contributor to boot-time variance and is the first thing to check if a
  boot ever approaches the window.

**Worst-case computed budget (CAN-FAIL path): ≈ 2.0 s used vs. ≈ 3.9 s minimum window → ≈ 1.9 s margin
(≈ 49 % of the window unused).** The `main.c:284-285` comment already asserts this "≤ 2 s … acceptable
because IWDG timeout is ~4 s".

---

## 4. Hardware measurement procedure (fill in on the bench)

The computed budget above is derived from source, but the **real** boot time must be measured on
hardware because LSI frequency, flash access, and I²C bus health all vary per unit. Two equivalent
methods:

**Method A — timestamp in firmware (temporary instrumentation).**
1. Capture `uint32_t t0 = HAL_GetTick();` immediately after `MX_IWDG_Init();` (`main.c:263`).
2. On the **first** main-loop iteration, before `HAL_IWDG_Refresh`, log `HAL_GetTick() - t0` over CAN
   (e.g. a temporary diagnostic frame) or SWO/UART.
3. Repeat 10×, cold and warm, CAN connected and disconnected. Record min/typ/max.

**Method B — GPIO toggle + logic analyzer (no code path dependency).**
1. Set a spare GPIO high in `MX_IWDG_Init()` and low at the first `HAL_IWDG_Refresh`.
2. Measure the high pulse width with a logic analyzer / scope. This also captures any I²C-timeout
   stretching that a `HAL_GetTick` sample between the two points would still see.

Record results here:

| Condition | Boot time (IWDG-start → first refresh) | Margin vs. 3.9 s | Pass (< 3.9 s)? |
|-----------|----------------------------------------|------------------|-----------------|
| Cold boot, CAN connected | _____ ms | _____ ms | ☐ |
| Cold boot, CAN disconnected | _____ ms | _____ ms | ☐ |
| Warm reboot, CAN connected | _____ ms | _____ ms | ☐ |
| Boot with I²C mux unpowered (Error 11 path) | _____ ms | _____ ms | ☐ |

**Acceptance:** every measured boot must complete the IWDG-start → first-refresh interval in well under
the worst-case **3.9 s** window, with a comfortable margin (target ≥ 1 s). If any condition approaches
the window, the corrective action is to move an earlier `HAL_IWDG_Refresh` into the boot animation or
shorten the LED-blink delays — **not** to lengthen the reload (which would slow real fault detection).

---

## 5. Cross-references

- `docs/BOOT_SEQUENCE_ANALYSIS.md` — ESP32-side (display) boot sequence.
- `docs/SENSOR_INTERFACE.md` — I²C mux / Error Code 11 details.
- `Core/Src/main.c` — `MX_IWDG_Init()` and the main-loop `HAL_IWDG_Refresh`.
