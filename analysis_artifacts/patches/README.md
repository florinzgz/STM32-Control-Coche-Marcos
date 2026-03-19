# Proposed Documentation Patches

> **Generated:** 2026-03-19  
> **Purpose:** Fix critical discrepancies between firmware and existing documentation.  
> **Status:** Pending human review before applying.

---

## Patch 1: docs/PINOUT_DEFINITIVO.md — Obsolete motor control scheme

**Problem:** PINOUT_DEFINITIVO.md documents a DIR+PWM motor control architecture
that was replaced with RPWM/LPWM dual-PWM in the current firmware (main.h:21-37).
The document also has incorrect wheel sensor pins (PB5/PB10/PB11/PB12 instead of
PA0/PA1/PA2/PB15), a non-existent pin (PD2), and wrong resistor/shunt values.

**Severity:** CRITICAL — a technician following this document would wire motors incorrectly.

**Recommendation:** Mark PINOUT_DEFINITIVO.md as DEPRECATED and direct users to
`docs/LISTADO_PINES_COMPLETO.md` + `analysis_artifacts/wiring_manual.md` as the
authoritative sources. Alternatively, rewrite PINOUT_DEFINITIVO.md completely.

**Specific errors (20 items):** See `analysis_artifacts/findings_hardware.csv` for full list.

---

## Patch 2: docs/HARDWARE_SPECIFICATION.md — Wrong resistor and shunt values

**Problem:** Lines 272-275 specify INA226 shunt as 2 mΩ (all channels).
Firmware (main.h:130-131) specifies 1 mΩ (motors) and 0.5 mΩ (battery).
Lines 342-350 specify pedal divider as 1 kΩ / 2 kΩ.
Firmware (main.h:101) specifies 10 kΩ / 6.8 kΩ.

**Severity:** HIGH — wrong shunt values cause incorrect current readings;
wrong divider values could expose ADC to overvoltage.

**Suggested changes:**
- Replace "0.002Ω (2mΩ)" with "1 mΩ (motors CH0-3,CH5) / 0.5 mΩ (battery CH4)"
- Replace "R1=1kΩ, R2=2kΩ" with "R1=10 kΩ, R2=6.8 kΩ"

---

## Patch 3: docs/PINOUT.md — Wrong relay pin assignments

**Problem:** PINOUT.md section 2 (lines ~307-310) has relay assignments
shifted by one pin and references PD2 which does not exist on LQFP64.

**Severity:** HIGH — relay wiring errors could short 24V to wrong circuit.

**Suggested changes:**
- RELAY_MAIN: PC10 (not PC11)
- RELAY_TRAC: PC11 (not PC12)  
- RELAY_DIR: PC12 (not PD2)

---

## Note

No code changes are proposed. All discrepancies are in documentation files only.
The firmware source (`Core/Inc/main.h`, `esp32/include/*.h`, `esp32/src/*.h`)
is considered the authoritative source of truth and is correct.
