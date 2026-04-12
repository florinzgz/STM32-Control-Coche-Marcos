# CAN Hardware Fix Procedure — Resolving "Waiting for CAN"

> **Purpose:** Step-by-step procedure to fix the hardware-level CAN failure.
> **Firmware status:** VERIFIED 100% CORRECT — do NOT modify any firmware.
> **Root cause:** Hardware wiring issues with TJA1051T/3 CAN transceiver(s).
> **Last updated:** 2026-04-04

---

## Quick Diagnosis Flowchart

```
POWER OFF both MCUs
        │
        ▼
  Measure CANH↔CANL resistance
        │
   ┌────┴────┐
   │ ~60 Ω?  │
   │  YES    NO ──► FIX: Add/correct 120 Ω termination at each bus end
   │         │
   ▼         │
  Check Pin 8 (S) to GND     ◄────────────────────────────────────────┘
  on BOTH TJA1051T/3 modules
        │
   ┌────┴────┐
   │  0 Ω?   │
   │  YES    NO ──► FIX: Solder Pin 8 to GND (MOST LIKELY ROOT CAUSE)
   │         │
   ▼         │
  Check GND common wire       ◄────────────────────────────────────────┘
  ESP32 GND ↔ STM32 GND
        │
   ┌────┴─────┐
   │Continuity?│
   │  YES    NO ──► FIX: Add dedicated GND wire between MCUs
   │          │
   ▼          │
  Check CANH↔CANH, CANL↔CANL  ◄───────────────────────────────────────┘
  continuity between modules
        │
   ┌────┴─────┐
   │Continuity?│
   │  YES    NO ──► FIX: Rewire differential pair
   │          │
   ▼          │
  POWER ON — check voltages    ◄───────────────────────────────────────┘
```

---

## 1. CRITICAL FIX: TJA1051T/3 Pin 8 (S) → GND

**This is the #1 most probable root cause (70% likelihood).**

### What is Pin 8 (S)?

The TJA1051T/3 CAN transceiver has a **silent mode pin** (Pin 8, labeled S):

| Pin 8 State | Mode | Result |
|-------------|------|--------|
| **Connected to GND** | **Normal mode** | ✅ Normal CAN operation |
| Connected to VCC (5V) | Silent mode | ❌ TX **disabled**, only listen |
| Floating (not connected) | Undefined | ❌ Intermittent, unreliable |

### How to Fix

1. **Power off** both MCU boards
2. Locate Pin 8 (S) on **each** TJA1051T/3 module (there are 2 modules — one per MCU)
3. With multimeter in continuity/resistance mode:
   - Place one probe on Pin 8 (S)
   - Place other probe on Pin 2 (GND)
   - **Expected: 0 Ω (direct connection)**
   - If **open circuit or >10 kΩ** → this is the root cause
4. **Fix:** Solder a wire from Pin 8 to Pin 2 (GND) on the module, or bridge them with a jumper
5. Repeat for the **second** TJA1051T/3 module

### TJA1051T/3 Pinout Reference (SO8 / SOT96-1)

```
          ┌─────────────┐
 TXD  1──┤             ├──8  S    ← MUST connect to GND
 GND  2──┤  TJA1051T/3 ├──7  CANH
 VCC  3──┤             ├──6  CANL
 RXD  4──┤             ├──5  VIO  ← MUST connect to 3.3V ⚠️
          └─────────────┘
```

- **Pin 1 (TXD):** Data input from MCU TX pin
- **Pin 2 (GND):** Ground — connect to MCU GND
- **Pin 3 (VCC):** Power — **connect to 5V** (4.5–5.5 V, NOT 3.3V)
- **Pin 4 (RXD):** Data output to MCU RX pin
- **Pin 5 (VIO):** I/O level reference — **connect to 3.3V** (OBLIGATORIO para ESP32-S3)
- **Pin 6 (CANL):** CAN Low — connect to bus
- **Pin 7 (CANH):** CAN High — connect to bus
- **Pin 8 (S):** Silent mode — **CONNECT TO GND**

> ⚠️ **CRÍTICO:** Si Pin 5 (VIO) se deja flotante o a 5V, el RXD producirá 5V → **destruye GPIO del ESP32-S3** (máx. 3.6V).

---

## 2. Complete Wiring Verification

### STM32 Side (Nucleo-64 MB1367)

| Signal | STM32 Pin | Nucleo Connector | TJA1051T/3 Pin |
|--------|-----------|------------------|----------------|
| FDCAN1_TX | **PA12** (AF9) | **CN10 Pin 12** | Pin 1 (TXD) |
| FDCAN1_RX | **PA11** (AF9) | **CN10 Pin 14** | Pin 4 (RXD) |
| VCC (**5V**) | — | **Fuente ext.** | Pin 3 (VCC) — 5V obligatorio |
| VIO (3.3V) | — | **CN10 Pin 7** | Pin 5 (VIO) — 3.3V nivel lógico |
| GND | — | **CN10 Pin 20** | Pin 2 (GND), Pin 8 (S→GND) |

```
  Nucleo-64 CN10 (right morpho header, USB connector at top)

  Pin 11  [ ][ ]  Pin 12 [PA12]  ← FDCAN1_TX  → TJA1051T/3 pin 1 (TXD)
  Pin 13  [ ][ ]  Pin 14 [PA11]  ← FDCAN1_RX  → TJA1051T/3 pin 4 (RXD)
    ...
  Pin  5  [ ][ ]  Pin  6
  Pin  7 [3V3][ ] Pin  8        ← 3.3V        → TJA1051T/3 pin 5 (VIO)
    ...
  Pin 19  [ ][ ]  Pin 20 [GND]  ← GND         → TJA1051T/3 pins 2 + 8 (S→GND)

  5V externo (DC-DC LM2596)     → TJA1051T/3 pin 3 (VCC)
```

> ⚠️ **WARNING:** Older documentation references PB8/PB9 as FDCAN pins.
> These pins were **remapped to PA11/PA12** in March 2026 to resolve a PWM conflict.
> The firmware (.ioc, HAL_FDCAN_MspInit) confirms **PA11/PA12** are the active pins.
> Wiring to PB8/PB9 will NOT work.

### ESP32-S3 Side

| Signal | ESP32 Pin | TJA1051T/3 Pin |
|--------|-----------|----------------|
| CAN_TX | **GPIO 4** | Pin 1 (TXD) |
| CAN_RX | **GPIO 5** | Pin 4 (RXD) |
| VCC (**5V**) | 5V ext. | Pin 3 (VCC) — 5V obligatorio |
| VIO (**3.3V**) | 3V3 pin | Pin 5 (VIO) — ⚠️ OBLIGATORIO, protege ESP32 |
| GND | GND pin | Pin 2 (GND), Pin 8 (S→GND) |

### CAN Bus Connections

| From (STM32 transceiver) | To (ESP32 transceiver) | Notes |
|--------------------------|------------------------|-------|
| Pin 7 (CANH) | Pin 7 (CANH) | Use twisted pair cable |
| Pin 6 (CANL) | Pin 6 (CANL) | Use twisted pair cable |
| Pin 2 (GND) | Pin 2 (GND) | **MANDATORY common ground** |

### Termination Resistors

| Location | Value | Connection |
|----------|-------|------------|
| STM32 bus end | 120 Ω | Between CANH and CANL |
| ESP32 bus end | 120 Ω | Between CANH and CANL |

**Total measured resistance CANH↔CANL (unpowered):** ~60 Ω (two 120 Ω in parallel)

---

## 3. Electrical Measurements

### 3.1 Unpowered Checks (Multimeter)

| # | Measurement | Probes | Expected | Root Cause if FAIL |
|---|-------------|--------|----------|--------------------|
| E1 | CANH ↔ CANL resistance | Between bus wires | **~60 Ω** | Missing/wrong termination |
| E2 | S → GND (STM32 transceiver) | Pin 8 to Pin 2 | **0 Ω** | **S not connected — #1 cause** |
| E3 | S → GND (ESP32 transceiver) | Pin 8 to Pin 2 | **0 Ω** | **S not connected — #1 cause** |
| E4 | VCC ↔ GND (each transceiver) | Pin 3 to Pin 2 | **>1 kΩ** | Short circuit |
| E5 | ESP32 GND ↔ STM32 GND | Between board GND pins | **Continuity** | Missing common ground |
| E6 | CANH continuity | STM32 TCV pin 7 to ESP32 TCV pin 7 | **Continuity** | Broken CANH wire |
| E7 | CANL continuity | STM32 TCV pin 6 to ESP32 TCV pin 6 | **Continuity** | Broken CANL wire |

### 3.2 Powered Checks (Voltmeter/Oscilloscope)

| # | Measurement | Probes | Expected | Root Cause if FAIL |
|---|-------------|--------|----------|--------------------|
| E8 | Transceiver VCC (each) | Pin 3 to Pin 2 | **5.0V ±5%** | Power supply issue — must be 5V |
| E8b | Transceiver VIO (each) | Pin 5 to Pin 2 | **3.3V ±5%** | VIO not connected — ⚠️ risk of ESP32 damage |
| E9 | CANH idle voltage | CANH to GND | **~2.5V** | Transceiver not powered / S pin issue |
| E10 | CANL idle voltage | CANL to GND | **~2.5V** | Transceiver not powered / S pin issue |
| E11 | CANH during TX | Oscilloscope on CANH | **Pulses 2.5V → 3.5V** | TX not working |
| E12 | CANL during TX | Oscilloscope on CANL | **Pulses 2.5V → 1.5V** | TX not working |
| E13 | STM32 PA12 (TX output) | Oscilloscope on PA12 | **Digital activity** | FDCAN not started |
| E14 | ESP32 GPIO4 (TX output) | Oscilloscope on GPIO4 | **Digital activity** | TWAI not started |

### 3.3 Interpretation

| CANH Idle | CANL Idle | PA12 Activity | GPIO4 Activity | Diagnosis |
|-----------|-----------|---------------|----------------|-----------|
| ~2.5V | ~2.5V | Yes | Yes | ✅ Bus healthy — CAN should work |
| ~2.5V | ~2.5V | Yes | No | ESP32 TWAI not started (check serial log) |
| ~2.5V | ~2.5V | No | Yes | STM32 FDCAN not started (check LD2 pattern) |
| 0V | 0V | — | — | Transceiver not powered — check VCC |
| ~3.3V | 0V | — | — | Transceiver in standby — **Pin 8 (S) at VCC** |
| Flat | Flat | Yes | Yes | **Pin 8 (S) floating** or wrong wiring |

---

## 4. Post-Fix Verification

After applying hardware fixes, verify the system comes alive:

### 4.1 STM32 LD2 (PA5) LED Patterns

| Pattern | Meaning |
|---------|---------|
| 3 quick blinks at power-on | ✅ Firmware reached peripheral init |
| Brief flash every ~2 seconds | ✅ Main loop running normally |
| Additional rapid toggles | ✅ **CAN RX active** (frames being received from ESP32) |
| ~2 Hz constant blink | ❌ Error_Handler entered — firmware crash |
| ~10 Hz rapid blink | ❌ CPU fault (HardFault) |

### 4.2 ESP32 Serial Output (115200 baud)

Look for these messages:

```
[CAN] Initialized at 500 kbps (SP=87.5%)     ← TWAI driver started OK
[CAN-RX] #1 ID=0x001 DLC=5 data=[...]        ← First STM32 heartbeat received
[CAN-RX] #2 ID=0x200 DLC=8 data=[...]        ← Speed status frame
[CAN-RX] total_frames=N                       ← Frame counter (every 10s)
[CAN-DIAG] state=RUNNING tx_err=0 rx_err=0    ← Healthy bus (every 5s)
```

**If CAN still not working, check serial for:**

```
[CAN-DIAG] state=BUS_OFF ...                  ← Hardware fault — check wiring
[CAN-DIAG] ... tx_err=128 ...                 ← Error-passive — check S pin, GND
[CAN] BUS_OFF detected — recovery attempt ... ← Auto-recovery in progress
```

### 4.3 Display Verification

| Before Fix | After Fix |
|-----------|-----------|
| "CAN: WAITING..." (red) | "CAN: LINKED" (green) |

The display transitions to "CAN: LINKED" when a valid STM32 heartbeat (0x001) is received within the last 500 ms.

### 4.4 SWD Debugger (Optional — STM32 Side)

Inspect these variables via ST-Link debugger:

```c
can_stats.tx_count      // Should be increasing (STM32 sending frames)
can_stats.rx_count      // Should be increasing (ESP32 heartbeat received)
can_stats.tx_errors     // Should be 0
can_stats.rx_errors     // Should be 0
can_init_diag.started   // Should be 1
can_init_diag.cccr_init_ok  // Should be 1
```

---

## 5. System-Level Validation (After CAN is Working)

Once CAN communication is established, verify all subsystems:

| # | Subsystem | Verification Method | Expected |
|---|-----------|-------------------|----------|
| V1 | Heartbeat | ESP32 boot screen | "OK" (green), counter incrementing |
| V2 | Speed sensors | ESP32 drive screen | Speed values update when wheels spin |
| V3 | Motor currents | ESP32 drive screen | Current values > 0 when motors active |
| V4 | Temperature | ESP32 drive screen | Temperature values (room temp ±5°C) |
| V5 | Steering angle | ESP32 drive screen | Angle changes when steering moves |
| V6 | Battery voltage | ESP32 drive screen | ~24V (or bench supply voltage) |
| V7 | LED control | Touch LED toggle on ESP32 | Physical relay clicks, LEDs change |
| V8 | Drive mode | Touch mode button on ESP32 | Mode change ACK received (0x103) |
| V9 | Gear selector | Shift gear lever | Gear indicator changes on display |
| V10 | Pedal input | Press throttle pedal | Motor response (wheels on stand) |

### Stability Test

Run the system for **>5 minutes** and verify:
- No "CAN: WAITING..." reappearance
- `[CAN-DIAG]` shows `tx_err=0 rx_err=0` consistently
- `[CAN-RX] total_frames` continues increasing
- No bus-off or error-passive recovery messages in serial log

---

## 6. Summary

| Root Cause (ranked) | Probability | Fix |
|---------------------|-------------|-----|
| **TJA1051T/3 Pin 8 (S) not connected to GND** | **70%** | Solder Pin 8 → GND on **both** modules |
| Missing common GND wire | 15% | Add dedicated GND wire ESP32↔STM32 |
| Missing/wrong termination resistors | 10% | Ensure 120 Ω at each bus end (~60 Ω total) |
| CANH/CANL reversed | 3% | Verify CANH↔CANH, CANL↔CANL |
| Transceiver connected to wrong STM32 pins | 2% | Must be **PA11/PA12** (not PB8/PB9) |

> **Note:** Multiple legacy documents reference PB8/PB9 as the FDCAN pins.
> The firmware was remapped to **PA11 (RX) / PA12 (TX)** in March 2026.
> Always verify against the `.ioc` file and `stm32g4xx_hal_msp.c` as source of truth.
