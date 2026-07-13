# P2 — Display Supervisor & Real TFT Recovery

Audit Problem 2. This documents the vertical slice that connects the pure
`DisplaySupervisor` decision core to the real ESP32 render task, the
differentiated fault taxonomy, the Core-0 recovery choreography, the
"PANTALLA RECUPERADA" banner, and the 20 MHz vs 40 MHz bench procedure.

> Status: **NOT hardware-validated.** Detection + diagnosis are wired in
> firmware. The hardware recovery *execution* (TFT re-init on Core 0) and the
> 20/40 MHz decision are **pending on-bench validation** and must not be
> declared resolved until verified on the real car.

## Modules (all pure, host-tested)

| File | Role | Host test |
|------|------|-----------|
| `esp32/src/display_supervisor.h` | Detection + retry/backoff FSM | `test_display_supervisor.cpp` |
| `esp32/src/display_recovery.h`   | Core-0 recovery step order + banner formatter | `test_display_recovery.cpp` |

Both are header-only, allocation-free and perform **no** hardware access, so
the whole policy is provable off-target under `-Wall -Wextra -Werror`.

## 1. Render heartbeat (requirement 1)

`renderHeartbeatMs` (in `esp32/src/main.cpp`) is published by the Core-0
render task **only at the end of a fully completed frame** — i.e. after:

1. `screenManager.update()`,
2. all TFT drawing operations,
3. the centralized touch read (`tft.getTouch`), and
4. the implicit release of the shared SPI transaction.

A hung render or a stuck SPI transaction therefore freezes the timestamp,
which the Core-1 supervisor observes as staleness. The heartbeat never
advances on a partially rendered frame.

## 2. Differentiated faults (requirement 2, 3)

`display::Fault` distinguishes: `RENDER_TIMEOUT`, `TFT_STATUS_LOST`,
`TFT_RESET_PROBABLE`, `SPI_TIMEOUT`, `LOW_MEMORY`, `ESP32_RESET`,
`READBACK_UNSUPPORTED`.

Because the ST7796 panel **cannot be read back reliably** with the current
wiring, `loop()` feeds `StatusRead::UNSUPPORTED`. A blank screen is thus
reported as a *probable* fault — the code never asserts
"pantalla blanca confirmada" without a trustworthy readback.

## 3. Recovery choreography (requirements 4, 5) — Core 0 only

`display_recovery.h::recoverySequence()` defines the mandated, ordered steps
(host-asserted in `test_display_recovery.cpp`):

```
STOP_RENDER_TOUCH → BLOCK_TFT_ACCESS → TFT_CS_HIGH → TOUCH_CS_HIGH →
CLOSE_SPI → PULSE_GPIO38 → TFT_INIT → SET_ROTATION_1 → RESTORE_TOUCH_CAL →
RESTORE_BACKLIGHT → INVALIDATE_CACHES → FORCE_FULL_REDRAW → VERIFY → RESUME
```

The isolation steps (both chip-selects HIGH, SPI closed, reset pulsed) are
proven to precede `TFT_INIT`; `VERIFY` precedes `RESUME`. Retries are bounded
with linear backoff and a persistent counter in the supervisor
(`max_attempts`, `backoff_base_ms`) — no infinite loop, and TFT is **never**
re-initialized from Core 1.

> The physical execution of these steps (real `tft.init()`, GPIO38 pulse,
> etc.) must run with exclusive bus ownership on the Core-0 render task. It is
> intentionally **not fired blindly** from firmware in this slice; it is wired
> as a documented, host-tested policy pending on-bench validation.

## 4. Banner (requirement 6)

`formatRecoveryBanner()` renders exactly:

```
PANTALLA RECUPERADA
causa: <...>
duracion: <ms> ms
intentos: <n>
render continuo: si|no
reboot ESP32: si|no
heap: <bytes>
recuperaciones totales: <n>
accion recomendada: <...>
```

## 5. 20 MHz vs 40 MHz bench procedure (requirement 7)

The default is **not** changed. To compare on the bench:

1. Baseline run at the current `SPI_FREQUENCY 40000000`
   (`esp32/include/User_Setup.h`). Log `[DISPLAY]` lines and the supervisor
   `recoveryCount()` over a fixed drive cycle (e.g. 30 min with motors/PWM/
   relays active to reproduce the noise environment).
2. Change **only** `#define SPI_FREQUENCY 20000000`, rebuild, repeat the same
   cycle.
3. Compare fault counts by cause (`RENDER_TIMEOUT`/`SPI_TIMEOUT`/
   `TFT_STATUS_LOST`). Record results in
   `docs/DISPLAY_STABILITY_VALIDATION.md`.

Do **not** lower the frequency permanently unless the 20 MHz run shows a
materially lower fault rate; a frequency change must be justified with this
telemetry, not assumed.

## 6. Pending physical checklist

- [ ] Reproduce the white-screen fault on the bench and confirm the captured
      `Fault` cause matches reality.
- [ ] Execute the Core-0 recovery choreography against real hardware and
      confirm the panel recovers without an ESP32 reboot.
- [ ] Run the 20 vs 40 MHz comparison and record the decision.
- [ ] Validate physical mitigations (TFT_RST pull-up, decoupling caps, short
      leads, ground, noise separation) per `User_Setup.h`.
