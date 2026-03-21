# 🔍 BTS7960 (IBT-2) Power & Logic Validation — Official PDF Analysis

**Repository:** `florinzgz/STM32-Control-Coche-Marcos`
**Date:** 2026-03-21
**Reference:** [Handsontec BTS7960 Motor Driver Module PDF](https://www.handsontec.com/dataspecs/module/BTS7960%20Motor%20Driver.pdf)
**Cross-reference:** Infineon BTS7960 IC datasheet, 74HC244 datasheets (Nexperia, ON Semiconductor, TI)

---

## 1. ⚡ LOGIC SUPPLY (VCC PIN)

### Should VCC be powered with 3.3V? → **NO** (per official documentation)

### Should VCC be powered with 5V? → **YES** (per official documentation)

### Recommended voltage according to documentation:

The Handsontec IBT-2 module documentation specifies:
- **Pin 7 (VCC): +5V** — logic supply from microcontroller

This 5V powers the on-board **74HC244 octal buffer IC** that isolates and buffers
the microcontroller's control signals before they reach the BTS7960 driver ICs.

### What happens if VCC is powered with 3.3V instead of 5V?

The 74HC244 has a minimum operating VCC of **2.0V**, so it will operate at 3.3V.
However, the input threshold changes:

| VCC Supply | 74HC244 V_IH(min) | 3.3V Input Status |
|------------|-------------------|-------------------|
| **5.0V** | 3.5V (0.7 × 5.0V) | ❌ **BELOW THRESHOLD** — 3.3V < 3.5V |
| **4.5V** | 3.15V (0.7 × 4.5V) | ⚠️ **MARGINAL** — 3.3V ≈ 3.15V |
| **3.3V** | 2.31V (0.7 × 3.3V) | ✅ **ABOVE THRESHOLD** — 3.3V > 2.31V |

**Practical effect**: At VCC=3.3V, the 74HC244 operates correctly with lower thresholds,
and 3.3V input signals are reliably recognized as HIGH. This is the approach used in
this design.

---

## 2. 🔌 LOGIC INPUT LEVELS (RPWM, LPWM, EN)

### Are control inputs compatible with 3.3V signals? → **YES** (with conditions)

### Are control inputs compatible with 5V signals? → **YES**

### Does compatibility depend on external logic chips? → **YES**

The IBT-2 module includes a **74HC244 octal buffer** between the input header pins
and the BTS7960 IC inputs. All control signals (RPWM, LPWM, R_EN, L_EN) pass through
this buffer. Compatibility depends on the relationship between VCC and input signal level:

| Scenario | VCC | Signal Level | 74HC244 V_IH(min) | Compatible? |
|----------|-----|-------------|-------------------|-------------|
| Arduino (5V logic, 5V VCC) | 5V | 5V | 3.5V | ✅ YES |
| STM32 (3.3V logic, 5V VCC) | 5V | 3.3V | 3.5V | ❌ **NO** — marginal |
| STM32 (3.3V logic, 3.3V VCC) | 3.3V | 3.3V | 2.31V | ✅ **YES** |
| ESP32 (3.3V logic, 5V VCC) | 5V | 3.3V | 3.5V | ❌ **NO** — marginal |

**Key finding**: The Handsontec PDF states "3.3V~5V control input levels." This claim is
based on typical (not worst-case) behavior. In practice, most 74HC244 chips will recognize
3.3V as HIGH at VCC=5V because the typical switching threshold is ~2.5V. However, this is
**NOT guaranteed** by the 74HC datasheet worst-case specifications (V_IH = 0.7 × VCC).

---

## 3. ⚠️ REAL-WORLD CHINESE IBT-2 MODULES

### Do most IBT-2 modules include extra logic chips? → **YES**

The official Handsontec IBT-2 documentation confirms a **74HC244 octal buffer IC** is
standard on the module. This is consistent across most Chinese IBT-2 module implementations.
The 74HC244 serves as:
- Signal buffer / line driver between MCU and BTS7960 ICs
- Protection for MCU pins against noise and back-EMF transients

### Does this affect 3.3V compatibility? → **YES** (when VCC = 5V)

At VCC=5V, the 74HC244 V_IH(min) = 3.5V. A 3.3V input signal from an STM32 is
0.2V below the guaranteed minimum HIGH threshold. While it works in practice on
most chips, this violates the worst-case datasheet specification.

### Is using STM32 (3.3V) directly SAFE? → **CONDITIONAL**

**Condition**: Safe **ONLY IF** IBT-2 module VCC is powered from 3.3V (not 5V).

| Configuration | Status |
|---------------|--------|
| STM32 3.3V signals + IBT-2 VCC = 5V | ⚠️ **NOT GUARANTEED** — works in practice, violates 74HC spec |
| STM32 3.3V signals + IBT-2 VCC = 3.3V | ✅ **SAFE** — 3.3V > V_IH(min) = 2.31V |
| STM32 3.3V signals + BSS138 level shifters + IBT-2 VCC = 5V | ✅ **SAFE** — 5V signals to 74HC244 |

**This design uses VCC = 3.3V** → Safe configuration.

---

## 4. 🔋 FINAL ELECTRICAL REQUIREMENTS

| Parameter | Value | Source |
|-----------|-------|--------|
| **Logic supply (VCC)** | **5V** (per official docs) or **3.3V** (safe with 3.3V MCU signals) | Handsontec PDF pin description |
| **Motor supply (B+/B−)** | **6V to 27V** DC | BTS7960 VS operating range |
| **Logic HIGH threshold** | **3.5V** at VCC=5V (74HC244); **2.31V** at VCC=3.3V (74HC244); **2.0V** (BTS7960 IC direct) | 74HC datasheet + BTS7960 datasheet |
| **Max PWM frequency** | **25 kHz** | BTS7960 datasheet |
| **Max continuous current** | **43A** per half-bridge (with thermal protection) | BTS7960 datasheet |
| **Common GND** | **Required** — MCU and IBT-2 must share ground reference | Standard practice for logic-level signal transfer |

### This system's actual values:

| Parameter | Value | Status |
|-----------|-------|--------|
| Logic VCC | **3.3V** from STM32 regulator | ✅ Safe (74HC244 V_IH = 2.31V at 3.3V VCC) |
| Motor supply | **24V** nominal (6S Li-ion: 18.0V–25.2V) | ✅ Within 6V–27V range |
| PWM frequency | **20 kHz** center-aligned | ✅ Within 25 kHz max |
| Signal voltage | **3.3V** from STM32G474RE GPIO | ✅ Above 2.31V V_IH at 3.3V VCC |
| Common GND | Star-ground topology | ✅ Verified |

---

## 5. 🧠 FINAL DECISION

### ⚠️ 3.3V MAY WORK BUT NOT GUARANTEED (at VCC = 5V)

### ✅ USE 3.3V DIRECTLY (at VCC = 3.3V) — **This is the configuration used**

**Technical justification (3 lines):**

1. The official Handsontec IBT-2 documentation recommends VCC = 5V, but the on-board 74HC244 buffer operates correctly at VCC = 3.3V (min VCC = 2.0V).
2. At VCC = 3.3V, the 74HC244 V_IH(min) = 2.31V, and the STM32's 3.3V GPIO output is reliably above this threshold — no level shifter required.
3. This design correctly powers IBT-2 VCC from the STM32 3.3V rail, ensuring full signal compatibility without additional components.

---

## Summary Decision Matrix

| Question | Answer |
|----------|--------|
| Official VCC recommendation | **5V** |
| Can VCC be 3.3V? | **YES** (74HC244 min VCC = 2.0V) |
| 3.3V signals safe at VCC=5V? | **NO** — not guaranteed (violates 74HC worst-case V_IH) |
| 3.3V signals safe at VCC=3.3V? | **YES** — guaranteed (3.3V > 2.31V V_IH(min)) |
| Level shifter needed? | **NO** — if VCC = 3.3V |
| Level shifter needed? | **YES** — if VCC = 5V with 3.3V MCU |
| This design's configuration | **VCC = 3.3V, signals = 3.3V → SAFE** |

---

## References

1. [Handsontec BTS7960 Motor Driver Module PDF](https://www.handsontec.com/dataspecs/module/BTS7960%20Motor%20Driver.pdf)
2. [Infineon BTS7960 IC Datasheet](https://www.infineon.com/assets/row/public/documents/10/57/infineon-bts7960-ds-en.pdf)
3. [Nexperia 74HC244 Datasheet](https://assets.nexperia.com/documents/data-sheet/74HC_HCT244.pdf) — V_IH = 0.7 × VCC
4. [ON Semiconductor MM74HC244 Datasheet](https://www.onsemi.com/download/data-sheet/pdf/mm74hc244-d.pdf)
5. `Core/Inc/main.h` — BTS7960 logic level compatibility comment (lines 22–29)
6. `docs/BTS7960_VALIDATION_DECISION.md` — Full 6-section validation report
7. `docs/BTS7960_MOTOR_DRIVER_AUDIT.md` — Per-motor pin mapping and control topology
