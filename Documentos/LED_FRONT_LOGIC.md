# LED Front Logic — Hardening Documentation

## 1. Zone Definitions

The front WS2812B strip has **28 LEDs** divided into three logical zones:

```
FRONT VIEW (looking at the front of the vehicle):

  DATA IN →  [0] [1] [2] [3] [4] [5] ... [22] [23] [24] [25] [26] [27]
             ├──── LEFT ────┤  ├──── CENTRE ────┤  ├───── RIGHT ─────┤
             5 LEDs              18 LEDs              5 LEDs
```

| Zone   | LEDs    | Count | Purpose                              |
|--------|---------|-------|--------------------------------------|
| LEFT   | [0–4]   | 5     | Left turn-signal indicator (amber)   |
| CENTRE | [5–22]  | 18    | KITT scanner / throttle / alerts      |
| RIGHT  | [23–27] | 5     | Right turn-signal indicator (amber)   |

> **LED_STRIP_REVERSED=1**: Swaps LEFT ↔ RIGHT zones (compile-time only).
> Centre zone indices are unchanged in both orientations.

---

## 2. Hysteresis Logic

Turn signals are derived from the steering angle in `main.cpp`.
Three independent anti-jitter layers prevent flicker:

### Layer 1 — Angle Hysteresis (5° dead band)

| Transition     | Threshold (0.1° raw) | Degrees |
|----------------|---------------------|---------|
| LEFT ON        | angle < -150        | < -15°  |
| LEFT OFF       | angle > -100        | > -10°  |
| RIGHT ON       | angle > +150        | > +15°  |
| RIGHT OFF      | angle < +100        | < +10°  |

When the angle is between the ON and OFF thresholds (the hysteresis band),
the previous state is maintained — no toggling occurs.

### Layer 2 — Time Persistence Filter (100 ms)

A new turn state must persist for ≥100 ms (2 frames at 20 Hz) before
being applied. This prevents micro-oscillation from PWM-coupled steering
noise or brief threshold crossings.

### Layer 3 — LED Controller Persistent State

Inside `updateFrontTurnSignals()`, static `turnLeftActive` / `turnRightActive`
flags provide frame-stable tracking of the active turn state. These derive
from `currentTurnSignal` (already debounced by layers 1–2) and ensure no
residual visual toggling within the LED rendering pipeline.

---

## 3. Overlay Priority (KITT vs Turn Signals)

Rendering executes in strict order each frame:

```
1. updateFrontLEDs()        → KITT / base effect on FULL 28-LED strip
2. updateFrontTurnSignals() → overlays side zones [0–4] and [23–27]
```

### Overlay Behavior

| Turn Signal | blinkState | Side Zones [0–4], [23–27] | Centre [5–22]  |
|-------------|------------|---------------------------|-----------------|
| OFF         | —          | KITT (no overlay)         | KITT            |
| LEFT/RIGHT  | ON (true)  | AMBER (override)          | KITT            |
| LEFT/RIGHT  | OFF (false)| KITT visible (no write)   | KITT            |
| HAZARD      | ON (true)  | AMBER both sides          | KITT            |
| HAZARD      | OFF (false)| KITT visible (no write)   | KITT            |

**Key design decision**: During blink-OFF, the overlay does **NOT** write
BLACK to the side zones. This allows the underlying KITT fade to remain
visible, eliminating the hard cut-to-black flicker that the previous
`AMBER : BLACK` logic caused.

---

## 4. SAFE MODE Override

When the system enters SAFE or ERROR state (`main.cpp`):

1. `main.cpp` sets `TurnSignal::HAZARD` (immediate, bypasses all debounce)
2. `main.cpp` sets `FrontMode::KITT_IDLE` (red KITT scanner on centre)
3. In `updateFrontTurnSignals()`:
   - `hazardOverride = true` (derived from HAZARD signal)
   - `turnLeftActive = true`, `turnRightActive = true`
   - Both side zones blink amber at 500 ms cadence

**Rear strip simultaneously**:
- Side zones [0–2] and [13–15]: amber HAZARD blink
- Centre [3–12]: BRAKE_EMERGENCY (red flash)

This produces the 4-corner hazard indicator pattern:

```
SAFE/ERROR visual layout:
  Front: [▓▓▓▓▓ AMBER] [KITT RED ×18] [AMBER ▓▓▓▓▓]
  Rear:  [▓▓▓ AMBER]   [BRAKE ×10]    [AMBER ▓▓▓]
```

SAFE MODE override is:
- Independent of steering angle (hazardOverride forces both sides)
- Does NOT affect normal mode behavior
- Resets cleanly when system exits SAFE/ERROR state

---

## 5. Timing Diagram (Conceptual)

```
Time →   0ms    500ms   1000ms  1500ms  2000ms
         ├──────┼───────┼───────┼───────┤

blink:   ON     OFF     ON      OFF     ON

TURN SIGNAL ACTIVE:
Side:    AMBER   (KITT)  AMBER   (KITT)  AMBER
Centre:  ──────── KITT scanner continuous ────────

NO TURN SIGNAL:
Full:    ──────── KITT scanner full 28 LEDs ──────

SAFE MODE (HAZARD):
Side L:  AMBER   (KITT)  AMBER   (KITT)  AMBER
Side R:  AMBER   (KITT)  AMBER   (KITT)  AMBER
Centre:  ──────── KITT RED scanner ───────────────
```

- Blink half-period: 500 ms (TURN_SIGNAL_BLINK_MS)
- Frame rate: 20 Hz (UPDATE_RATE_MS = 50 ms)
- KITT scanner: ~1400 ms full sweep (28 LEDs × 50 ms/step)

---

## 6. Files Modified

| File | Changes |
|------|---------|
| `esp32/src/led_controller.cpp` | Hardened `updateFrontTurnSignals()`: persistent state, blink-ON-only overlay, hazard override |
| `Documentos/LED_FRONT_LOGIC.md` | This documentation file |

No changes to: `led_controller.h`, `main.cpp`, rear LED logic, public API, timing architecture.
