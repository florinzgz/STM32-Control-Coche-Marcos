# Hardware Modifications — NUCLEO-G474RE (MB1367C)

This document lists the **physical hardware modifications and wiring
clarifications** required on the NUCLEO-G474RE board to match the
firmware pin mapping defined in `Core/Inc/project_config.h`.

These items do **not** change any firmware. They document hardware-only
actions that must be completed before connecting the real vehicle
harness for the first time.

---

## 1. SB17 — MUST BE OPEN (PC13 / EN_RR)

### Problem

The firmware uses **PC13** as `PIN_EN_RR` (enable output for the rear-right
BTS7960 driver, see `project_config.h:116`).

On the NUCLEO-G474RE (MB1367C), PC13 is **also routed by default to the
on-board USER button B1** through solder bridge **SB17** (see ST user
manual UM2505, §6.4 *Push-buttons*).

If SB17 is left in its factory configuration:

- Pressing B1 shorts PC13 to GND through the button.
- When firmware drives PC13 HIGH (EN_RR active), the push-pull output
  collides with the button-to-GND path → electrical contention and
  an involuntary disable of the rear-right motor on every press.
- Any accidental contact with B1 during operation would drop EN_RR and
  put the RR wheel in coast mode without any software fault being
  raised.

### Required modification

**Desolder the SB17 solder bridge** on the NUCLEO-G474RE board.

> **SB17 must be OPEN (desoldered) to use PC13 as EN_RR.
> This disconnects USER button B1 from PC13 and prevents unintended
> motor disable.**

After desoldering:

- B1 becomes functionally disconnected (acceptable — B1 is not used
  by this firmware).
- PC13 behaves as a normal GPIO output and can safely drive the
  BTS7960 EN input.

### Verification

1. Power the Nucleo via ST-LINK only (no motor power).
2. Command `PIN_EN_RR` HIGH from the engineering menu (relay override
   path) or via SWD by writing `GPIOC->BSRR = (1U << 13)`.
3. Measure PC13 with a multimeter — must read ≈ 3.3 V stable.
4. Press B1 — PC13 must **stay** at ≈ 3.3 V (SB17 is open).
5. If PC13 drops to 0 V on press, SB17 is still closed → repeat the
   desolder step.

### Firmware reference

- `Core/Inc/project_config.h`: `#define PIN_EN_RR  GPIO_PIN_13`
- Init-safety comment block in the same file documents that all EN
  pins are forced LOW via `GPIOC->BSRR` atomic write before the GPIO
  mode is configured, so no transient activation can occur at reset.

---

## 2. Encoder wiring — A / B / Z all required (PA15 / PB3 / PB4)

### Problem

Early versions of the wiring diagram only listed the index channel
(`ENC_Z` on PB4). The firmware quadrature decoder running on
**TIM2** needs all three encoder channels to function.

### Required wiring

| Encoder signal | STM32 pin | Peripheral | `project_config.h` macro |
|---|---|---|---|
| Channel **A** (white) | **PA15** | TIM2_CH1 (AF1) | `PIN_ENC_A` |
| Channel **B** (green) | **PB3**  | TIM2_CH2 (AF1) | `PIN_ENC_B` |
| Channel **Z** (yellow, index) | **PB4** | GPIO / EXTI4 | `PIN_ENC_Z` |

All three signals must be level-shifted from the 5 V open-collector
output of the E6B2-CWZ6C encoder to 3.3 V before reaching the MCU
(BSS138 level shifter or resistive divider — see
`HARDWARE_WIRING_MANUAL.md` §12).

### Verification

- `grep -n "PIN_ENC_A\|PIN_ENC_B\|PIN_ENC_Z" Core/Inc/project_config.h`
  must show PA15, PB3, PB4 respectively.
- With the encoder powered and turned by hand, the firmware
  `Steering_GetCurrentAngle()` must return a monotonically changing
  value. If only one direction advances, A/B are swapped. If the
  count is stuck, one of A or B is missing or saturated at 5 V.

### Firmware reference

- `Core/Inc/project_config.h`:
  - `#define PIN_ENC_A  GPIO_PIN_15  /* PA15 - TIM2_CH1 */`
  - `#define PIN_ENC_B  GPIO_PIN_3   /* PB3  - TIM2_CH2 */`
  - `#define PIN_ENC_Z  GPIO_PIN_4   /* PB4  - EXTI4 index */`

---

## 3. Header location for EN_FR and EN_RL — CN7 (Morpho), **not** CN9

### Problem

Some external wiring drafts labelled the enable signals of the front-right
and rear-left motors as "CN9 / Arduino header".
On the NUCLEO-G474RE (MB1367C) those pins are **not** exposed on the
Arduino-format CN9 header — they are only available on the
**Morpho CN7** header.

### Correct header mapping

| Signal  | STM32 pin | Nucleo connector | Morpho CN7 pin |
|---------|-----------|------------------|----------------|
| EN_FR   | **PC0**   | **CN7** (Morpho) | pin 38         |
| EN_RL   | **PC1**   | **CN7** (Morpho) | pin 36         |

> ⚠ **CN9 → CN7**: any previous reference to "CN9" for PC0 (`EN_FR`)
> or PC1 (`EN_RL`) is incorrect. The physical wires must land on
> **CN7 (Morpho)**. Verify continuity with a multimeter between the
> Morpho pin and the BTS7960 `R_EN`+`L_EN` input before powering the
> drivers.

### Verification

- Check UM2505 §6.11 *Morpho connectors* — CN7 pin 38 = PC0, CN7 pin 36 = PC1.
- `grep -n "PIN_EN_FR\|PIN_EN_RL" Core/Inc/project_config.h` must show
  `GPIO_PIN_0` (PC0) and `GPIO_PIN_1` (PC1) respectively.

### Firmware reference

- `Core/Inc/project_config.h`:
  - `#define PIN_EN_FR  GPIO_PIN_0   /* PC0 — was DIR_FL */`
  - `#define PIN_EN_RL  GPIO_PIN_1   /* PC1 — was DIR_FR */`

---

## Summary checklist (before first power-up with real motors)

- [ ] SB17 desoldered on NUCLEO-G474RE (PC13/EN_RR isolated from B1).
- [ ] Encoder A wired from encoder white wire → level shifter → **PA15**.
- [ ] Encoder B wired from encoder green wire → level shifter → **PB3**.
- [ ] Encoder Z wired from encoder yellow wire → level shifter → **PB4**.
- [ ] EN_FR cable lands on **CN7 pin 38 (PC0)** — not CN9.
- [ ] EN_RL cable lands on **CN7 pin 36 (PC1)** — not CN9.
- [ ] All EN lines read ≈ 0 V immediately after reset (safety latch).

Once all six items are checked, the board is electrically aligned with
the firmware and safe to connect to the traction and direction relays.
