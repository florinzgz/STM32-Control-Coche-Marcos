# CAN Communication Diagnostic Report

**Date:** 2026-03-13
**System:** STM32G474RE (Nucleo-64) ↔ ESP32-S3 HMI
**Symptom:** ESP32 HMI boot screen shows "CAN: WAITING..." — no CAN heartbeat received from STM32

---

## 1. STM32 CAN Configuration

| Parameter | Value | Source |
|-----------|-------|--------|
| **Peripheral** | FDCAN1 | `.ioc` (Mcu.IP1=FDCAN1), `main.c:538` |
| **TX pin** | PB9 (AF9) | `.ioc` (PB9.Signal=FDCAN1_TX), `main.h:57` |
| **RX pin** | PB8 (AF9) | `.ioc` (PB8.Signal=FDCAN1_RX), `main.h:58` |
| **Mode** | FDCAN_MODE_NORMAL (active TX/RX) | `main.c:540` |
| **Frame format** | FDCAN_FRAME_CLASSIC (CAN 2.0) | `main.c:539` |
| **Clock source** | PCLK1 (APB1) | `.ioc` (RCC.FDCANCLKSelection=RCC_FDCANCLKSOURCE_PCLK1) |
| **Clock frequency** | 170 MHz | `.ioc` (RCC.FDCANFreq_Value=170000000) |
| **Prescaler** | 17 | `main.c:542` |
| **TimeSeg1** | 14 TQ | `main.c:544` |
| **TimeSeg2** | 5 TQ | `main.c:545` |
| **SyncJumpWidth** | 1 TQ | `main.c:543` |
| **Calculated bitrate** | 170 MHz ÷ (17 × (1 + 14 + 5)) = **500 000 bps** ✅ | — |
| **Sample point** | (1 + 14) / 20 = **75%** | — |
| **Auto retransmission** | ENABLE | `main.c:546` |
| **HAL_FDCAN_Start()** | Called in `CAN_Init()` at `can_handler.c:195` | ✅ Confirmed |
| **RX filters** | 4-filter whitelist (0x011, 0x100–0x102, 0x110–0x120, 0x208–0x209) | `can_handler.c:107–155` |
| **Interrupt** | FDCAN1_IT0_IRQn, priority 1 | `stm32g4xx_hal_msp.c:24–25` |

### Periodic CAN transmissions from STM32

| Rate | Messages |
|------|----------|
| 100 ms | Heartbeat (0x001), Speed (0x200), Current (0x201), Safety (0x203), Steering (0x204), Traction (0x205), Battery (0x207) |
| 1000 ms | Temperature (0x202), TempMap (0x206), **Lights / STATUS_LIGHTS (0x20A)**, ServiceStatus (0x301–0x305), ErrorLog |

### Function responsible for 0x20A (STATUS_LIGHTS)

```
CAN_SendStatusLights()  →  can_handler.c:547–554
```

Called every 1000 ms from the main loop (`main.c:396`).

---

## 2. ESP32-S3 CAN Configuration

| Parameter | Value | Source |
|-----------|-------|--------|
| **CAN controller** | TWAI (via ESP32-TWAI-CAN library v1.0.1) | `platformio.ini:91` |
| **TX pin** | GPIO 4 | `main.cpp:85` |
| **RX pin** | GPIO 5 | `main.cpp:86` |
| **Bitrate** | 500 kbps (`ESP32Can.convertSpeed(500)`) | `main.cpp:350` |
| **RX queue** | 5 frames | `main.cpp:348` |
| **TX queue** | 5 frames | `main.cpp:349` |
| **Mode** | Normal (library default, no explicit TWAI_MODE_LISTEN_ONLY) | — |
| **Driver start** | `ESP32Can.begin()` in `setup()` | `main.cpp:350–354` |

### Expected CAN message IDs (from `can_ids.h`)

All 25 CAN IDs are defined in `esp32/include/can_ids.h` and match the STM32 definitions in `Core/Inc/can_handler.h`:

- Heartbeat STM32→ESP32: **0x001** (DLC=5, 100 ms)
- Heartbeat ESP32→STM32: **0x011** (DLC=1, 100 ms)
- STATUS_LIGHTS: **0x20A** (DLC=2, 1000 ms)

### CAN RX processing

```
can_rx::poll(vehicleData)  →  esp32/src/can_rx.cpp
```

Called in `loop()`. Non-blocking — reads all available frames and decodes them into the `VehicleData` store.

---

## 3. CAN Message Flow After Boot

### STM32 boot sequence (main.c)

1. `HAL_Init()` → system clock → `MX_FDCAN1_Init()` → other peripherals
2. `CAN_Init()` → configures filters → `HAL_FDCAN_Start()` → peripheral active
3. Main loop starts: **heartbeat 0x001 transmitted every 100 ms immediately**
4. Status messages (0x200–0x20A) begin transmitting on their respective intervals

**Conclusion:** The STM32 transmits CAN frames starting ~100 ms after boot. ✅

### ESP32 boot sequence (main.cpp)

1. Serial, PSRAM, TFT display initialization
2. `ESP32Can.begin()` → TWAI driver started
3. `loop()` starts: `can_rx::poll()` checks for incoming frames
4. Boot screen displays **"CAN: WAITING..."** until a heartbeat timestamp < 500 ms old is received

**Conclusion:** The ESP32 waits passively for the STM32 heartbeat. The "CAN: WAITING..." text clears only when heartbeat 0x001 is received and decoded.

### Which board sends first?

**STM32 sends first** (heartbeat 0x001 at 100 ms intervals). The ESP32 also sends heartbeat 0x011 at 100 ms, but the boot screen only monitors reception of the STM32 heartbeat.

---

## 4. Transceiver Verification

| Parameter | Value | Source |
|-----------|-------|--------|
| **Transceiver IC** | TJA1051T/3 (NXP, 3.3V logic-level variant) | `docs/ESP32_STM32_CAN_CONNECTION.md` |
| **Quantity** | 2 (one per board) | — |
| **VCC** | 5V (required by TJA1051) | Pin 3 on each module |
| **S/SLNT pin** | Must be connected to GND (normal mode) | Pin 8 on each module |

### Required wiring

| STM32 side | | TJA1051 #1 | | Bus | | TJA1051 #2 | | ESP32 side |
|---|---|---|---|---|---|---|---|---|
| PB9 (TX) | → | Pin 1 (TXD) | | | | Pin 1 (TXD) | ← | GPIO 4 (TX) |
| PB8 (RX) | ← | Pin 4 (RXD) | | | | Pin 4 (RXD) | → | GPIO 5 (RX) |
| 5V | → | Pin 3 (VCC) | | | | Pin 3 (VCC) | ← | 5V |
| GND | → | Pin 2 (GND) | | | | Pin 2 (GND) | ← | GND |
| — | — | Pin 8 (S) → GND | | | | Pin 8 (S) → GND | — | — |
| — | — | Pin 6 (CANH) | ↔ | CANH bus | ↔ | Pin 6 (CANH) | — | — |
| — | — | Pin 7 (CANL) | ↔ | CANL bus | ↔ | Pin 7 (CANL) | — | — |

### Critical hardware checks

- [ ] TJA1051 pin 8 (S/SLNT) connected to GND on **both** modules
- [ ] VCC = 5V on **both** modules (3.3V will not work for TJA1051)
- [ ] CANH connects to CANH (not crossed with CANL)
- [ ] CANL connects to CANL
- [ ] Common GND between both boards and both transceivers
- [ ] If using standard TJA1051 (not /3 variant): 5V→3.3V level shifter on RXD→ESP32

---

## 5. Termination Resistors

CAN 2.0 requires **120 Ω termination** at each physical end of the bus.

| Scenario | Expected bus impedance | Behavior |
|----------|----------------------|----------|
| 0 resistors | ∞ (open) | **Bus will NOT work** — reflections corrupt signals |
| 1 resistor (120 Ω) | 120 Ω | May work on short buses (<30 cm), but unreliable |
| **2 resistors (120 Ω each)** | **60 Ω (parallel)** | **Correct configuration** ✅ |

### How to verify

Disconnect power, measure resistance between CANH and CANL with a multimeter:
- **~60 Ω** → 2 termination resistors present ✅
- **~120 Ω** → Only 1 termination resistor present ⚠️
- **Open / very high** → No termination ❌

### Module jumpers

Many TJA1051 breakout modules include a solder jumper or DIP switch for the built-in 120 Ω resistor. **Verify that the termination jumper is ENABLED on both modules.**

---

## 6. Pin Mapping Verification

### STM32 pins — .ioc vs firmware vs wiring documentation

| Pin | .ioc file | main.h | HAL MSP (AF) | Wiring doc | Match |
|-----|-----------|--------|--------------|------------|-------|
| PB8 | FDCAN1_RX | PIN_CAN_RX = GPIO_PIN_8 | AF9_FDCAN1 | FDCAN1_RX → TJA1051 RXD | ✅ |
| PB9 | FDCAN1_TX | PIN_CAN_TX = GPIO_PIN_9 | AF9_FDCAN1 | FDCAN1_TX → TJA1051 TXD | ✅ |

### ESP32 pins — firmware vs wiring documentation

| Pin | main.cpp | Wiring doc | Match |
|-----|----------|------------|-------|
| GPIO 4 | CAN_TX_PIN = 4 | CAN_TX → TJA1051 TXD | ✅ |
| GPIO 5 | CAN_RX_PIN = 5 | CAN_RX → TJA1051 RXD | ✅ |

### Nucleo-64 board connector location (CN7)

| Function | STM32 pin | CN7 position | Arduino label | Notes |
|----------|-----------|--------------|---------------|-------|
| FDCAN1_RX | PB8 | Pin 3 | D15 | ⚠️ Shared with BOOT0 via JP7 — JP7 must be GND |
| FDCAN1_TX | PB9 | Pin 5 | D14 | Shared with I2C1_SDA (I2C must use different pins) |
| GND | — | Pin 7 | — | — |
| 5V | — | Pin 8 | — | — |

**⚠️ BOOT0 conflict:** On the Nucleo-64 (MB1367), PB8 shares a PCB trace with BOOT0 via solder bridge JP7. If JP7 is set to VDD instead of GND, PB8 cannot function as FDCAN1_RX. **Factory default is GND (correct).**

---

## 7. Clock Configuration

| Parameter | Value | Verification |
|-----------|-------|-------------|
| System clock (HCLK) | 170 MHz | `.ioc` RCC configuration |
| APB1 clock (PCLK1) | 170 MHz | `.ioc` RCC.APB1Freq_Value |
| FDCAN clock source | PCLK1 | `.ioc` RCC.FDCANCLKSelection=RCC_FDCANCLKSOURCE_PCLK1 |
| FDCAN clock enabled | Yes | `stm32g4xx_hal_msp.c:14` — `__HAL_RCC_FDCAN_CLK_ENABLE()` |
| GPIO-B clock enabled | Yes | `stm32g4xx_hal_msp.c:15` — `__HAL_RCC_GPIOB_CLK_ENABLE()` |

### Bit timing verification

```
Bitrate = FDCAN_CLK / (Prescaler × (1 + TimeSeg1 + TimeSeg2))
        = 170 000 000 / (17 × (1 + 14 + 5))
        = 170 000 000 / (17 × 20)
        = 170 000 000 / 340
        = 500 000 bps  ✅
```

Sample point = (1 + TimeSeg1) / (1 + TimeSeg1 + TimeSeg2) = 15/20 = **75%**

This is within the recommended range (75–87.5%) for CAN 2.0. ✅

---

## 8. Diagnostic Summary

### CAN Configuration — STM32

| Item | Value |
|------|-------|
| Peripheral | FDCAN1 |
| TX pin | PB9 (CN7 pin 5) |
| RX pin | PB8 (CN7 pin 3) |
| Bitrate | 500 kbps |
| Mode | Normal (active TX/RX) |
| HAL_FDCAN_Start() | ✅ Called |
| First TX after boot | ~100 ms (heartbeat 0x001) |

### CAN Configuration — ESP32

| Item | Value |
|------|-------|
| TX pin | GPIO 4 |
| RX pin | GPIO 5 |
| Driver | TWAI (ESP32-TWAI-CAN library v1.0.1) |
| Bitrate | 500 kbps |
| Mode | Normal (default) |

### Bus Hardware

| Item | Value |
|------|-------|
| Transceiver type | TJA1051T/3 (NXP, 3.3V I/O logic, **VCC = 5V required**) × 2 |
| Termination | 2 × 120 Ω required (one at each bus end) |
| Bus standard | CAN 2.0A (11-bit ID, classic frames) |
| ~~VCC issue~~ | ~~Confirmed: transceivers powered at 3.3V~~ → **FIXED: VCC now 5V** ✅ |
| ⚠️ **Termination** | **No 120 Ω termination resistors installed — bus will not work** |

---

## Previous Root Cause (RESOLVED)

### ~~TJA1051T/3 powered at 3.3V instead of 5V~~ — FIXED ✅

> El usuario ha confirmado que los transceptores TJA1051T/3 ahora están
> alimentados correctamente a **5 V**, compartiendo masa (GND) entre la
> STM32G4 y la ESP32-S3. La pantalla sigue mostrando "CAN: WAITING...".

---

## Current Root Cause Analysis

### Firmware analysis result: **No software defects detected**

Both the STM32 and ESP32 firmware are correctly configured:
- Matching bitrate (500 kbps) on both sides
- Correct CAN mode (NORMAL) on both sides
- `HAL_FDCAN_Start()` is called and the peripheral is active
- The STM32 begins transmitting heartbeat 0x001 immediately after boot
- The ESP32 CAN driver is initialized and polling for frames
- All CAN message IDs are consistent between both firmwares

### Primary cause: Missing 120 Ω termination resistors

> ⚠️ **CAUSA RAÍZ MÁS PROBABLE:** No hay resistencias de terminación de
> 120 Ω conectadas en el bus CAN. El usuario ha confirmado: *"No hay
> terminadores extra conectados todavía."*
>
> **Sin terminación, el bus CAN no puede funcionar.** Las reflexiones de
> señal en los extremos abiertos corrompen la señal diferencial CANH/CANL,
> causando errores de bit continuos. Ambos controladores (FDCAN de la STM32
> y TWAI de la ESP32) detectan estos errores y nunca completan la recepción
> de un frame.

#### Why termination is mandatory (not optional)

CAN 2.0 (ISO 11898-2) uses differential signaling on a linear bus topology.
The bus behaves as a **transmission line** with a characteristic impedance
of ~120 Ω. Without matched termination at both ends:

| Effect | Consequence |
|--------|-------------|
| Signal reflections at open ends | Corrupt bit sampling — receiver sees wrong bit value |
| Dominant-to-recessive transition ringing | CRC errors and stuff-bit violations |
| Voltage levels outside spec | CANH/CANL levels fail to cross the ~0.9 V differential threshold |
| Transmitter error counter increments | After 128 TX errors → **bus-off** (transceiver stops transmitting) |

Even on very short buses (< 10 cm), missing termination causes unreliable
or completely failed communication at 500 kbps.

#### Fix: Install two 120 Ω termination resistors

Place **one 120 Ω resistor between CANH and CANL** at each physical end of the bus:

```
ESP32 side (TJA1051 #1):                STM32 side (TJA1051 #2):

  CANH [pin 7] ──┐                        ┌── CANH [pin 7]
               [120Ω]   ← twisted pair →  [120Ω]
  CANL [pin 6] ──┘                        └── CANL [pin 6]

Result: measured resistance CANH–CANL = ~60 Ω (two 120 Ω in parallel) ✅
```

**Resistor spec:** 120 Ω ±5%, 1/4 W or higher, carbon film or metal film.

**Module jumpers:** Many TJA1051 breakout boards have a **solder jumper** or
**DIP switch** for a built-in 120 Ω resistor. Check both modules — the
jumper may be **disabled by default**. If enabled on both, no external
resistors are needed.

#### Verification after installing terminators

1. **Power off** both systems
2. Measure resistance between CANH and CANL with a multimeter:
   - **~60 Ω** → two terminators present ✅
   - **~120 Ω** → only one terminator (install the second) ⚠️
   - **> 1 kΩ or open** → no terminators (install both) ❌
3. **Power on** both systems and check the ESP32 serial monitor

### Secondary causes to verify

If CAN still does not work after installing termination, check these causes
**in order** (most likely first):

#### 1. TJA1051 S/SLNT (silent mode) pin not grounded

The TJA1051 pin 8 (labeled S, SLNT, or STB depending on the module) **must**
be connected to GND for normal operation. If left floating or pulled high,
the transceiver enters silent mode: it can still receive (weakly), but
**cannot transmit** — the bus sees no frames and both sides remain in error.

| Pin 8 level | Mode | TX | RX |
|-------------|------|----|----|
| **LOW (GND)** | **Normal** ✅ | ✅ Active | ✅ Active |
| HIGH (VCC) | Silent | ❌ Disabled | ✅ Active |
| Floating | Undefined | ⚠️ Erratic | ⚠️ Erratic |

**Action:** Verify pin 8 is connected to GND on **both** TJA1051 modules.
Use a multimeter in continuity mode to confirm the connection from pin 8
to the GND rail.

#### 2. CANH/CANL crossed between transceivers

If CANH on one transceiver connects to CANL on the other (and vice versa),
the differential signal is inverted. The transceivers cannot decode the
inverted polarity and treat every bit as an error.

**Action:** Trace the physical wiring to ensure CANH↔CANH and CANL↔CANL.
Use the color coding on twisted pair cables (e.g., orange = CANH,
orange/white = CANL) and verify at both ends.

#### 3. No common GND between ESP32 and STM32 systems

CAN is differential, but the transceivers need a common ground reference
for the voltage thresholds to work correctly. Without a shared GND, the
common-mode voltage can drift outside the ±2 V tolerance of the TJA1051.

**Action:** Verify that both systems share a dedicated GND wire (not just
through the CAN cable — a separate GND wire between the boards is required).

#### 4. BOOT0 jumper (JP7) on Nucleo-64 in wrong position

PB8 (FDCAN1_RX) shares a PCB trace with BOOT0 via JP7. If JP7 is set to
VDD, PB8 is held high and FDCAN1 RX is non-functional.

**Action:** Verify JP7 is in the GND position (factory default). The user
has confirmed this is correct ✅.

#### 5. 100 nF decoupling capacitors missing

Each TJA1051 should have a 100 nF ceramic capacitor between VCC (pin 3)
and GND (pin 2), placed as close to the IC as possible. Without decoupling,
transient noise on VCC during bus transitions can cause sporadic bit errors.

**Action:** Install 100 nF (0.1 µF) ceramic capacitors on both modules.

---

## Firmware Diagnostics

### ESP32 TWAI bus status logging

The firmware now includes periodic TWAI diagnostic output on the ESP32
serial monitor (every 5 seconds). Connect USB to the ESP32 and open a
serial terminal at 115200 baud. Look for lines like:

```
[CAN-DIAG] state=RUNNING tx_err=0 rx_err=0 tx_fail=0 rx_miss=0 arb_lost=0 bus_err=0
```

#### Interpreting the diagnostic output

| Field | Meaning | Healthy value | Problem indication |
|-------|---------|---------------|-------------------|
| `state` | TWAI driver state | `RUNNING` | `BUS_OFF` = too many errors; `RECOVERING` = attempting recovery |
| `tx_err` | TX error counter (0–255) | 0 | > 96 = error warning; ≥ 128 = bus-off |
| `rx_err` | RX error counter (0–255) | 0 | > 96 = error warning; rising count = signal quality issue |
| `tx_fail` | Failed TX attempts | 0 | > 0 = no ACK received (other node not responding) |
| `rx_miss` | Missed RX frames (queue full) | 0 | > 0 = increase RX queue size |
| `arb_lost` | Arbitration lost count | 0 (only one TX node) | > 0 = normal if multiple TX nodes |
| `bus_err` | Bus error count | 0 | > 0 = physical layer errors (wiring, termination, noise) |

#### Common diagnostic patterns

| Serial output pattern | Likely cause | Fix |
|----------------------|--------------|-----|
| `state=BUS_OFF tx_err=128+` | No termination resistors | Install 2× 120 Ω |
| `state=RUNNING tx_err=0 rx_err=0` but no heartbeat | STM32 not transmitting or TJA1051 silent pin HIGH | Check STM32 side, pin 8 to GND |
| `tx_fail` incrementing | No ACK from any node (STM32 not receiving) | Check STM32 RX wiring, termination |
| `bus_err` incrementing rapidly | Wiring fault, crossed CANH/CANL, or noise | Re-check physical connections |
| `state=STOPPED` | `ESP32Can.begin()` failed | Check serial for `[CAN] Initialization FAILED` |

---

## Recommended Debugging Steps

### Step 1 — Serial monitor (both boards)

Connect USB to both boards and verify:

- **ESP32:** Look for `[CAN] Initialized at 500 kbps` (success) vs
  `[CAN] Initialization FAILED` (TWAI driver problem).
  Then check the periodic `[CAN-DIAG]` lines for error counters.
- **STM32:** The STM32 does not have serial debug output for CAN by default,
  but the onboard LED behavior and safety state transitions can indicate CAN
  status. If the STM32 has a SWV/ITM trace output, `can_stats.tx_errors` and
  `can_stats.busoff_count` can be inspected.

### Step 2 — Multimeter check (power off)

1. Disconnect power from both systems
2. Measure resistance between CANH and CANL:
   - **~60 Ω** → both terminators present ✅
   - **~120 Ω** → only one terminator ⚠️
   - **Open** → no terminators ❌ ← **this is the current state**
3. Check continuity:
   - TJA1051 #1 pin 8 (S) to GND → should beep ✅
   - TJA1051 #2 pin 8 (S) to GND → should beep ✅
   - TJA1051 #1 VCC to 5V rail → should beep ✅
   - TJA1051 #2 VCC to 5V rail → should beep ✅

### Step 3 — Voltage levels (power on, firmware running)

| Measurement | Expected | If wrong |
|-------------|----------|----------|
| TJA1051 #1 VCC (pin 3) | 4.75–5.25 V | Check 5V supply |
| TJA1051 #2 VCC (pin 3) | 4.75–5.25 V | Check 5V supply |
| CANH idle voltage | ~2.5 V | If 0 V or 5 V → bus error, check GND/VCC |
| CANL idle voltage | ~2.5 V | If 0 V or 5 V → bus error, check GND/VCC |
| CANH − CANL idle | ~0 V | If ≠ 0 V → a node is stuck transmitting |

### Step 4 — Oscilloscope (if available)

Probe CANH and CANL with an oscilloscope:
- **With termination:** You should see bursts of differential signaling
  at ~10 Hz (100 ms heartbeat). Dominant bit: CANH ~3.5 V, CANL ~1.5 V.
  Recessive bit: both ~2.5 V.
- **Without termination:** You will see ringing and reflections on every
  transition — this confirms the termination issue.

### Step 5 — Loopback test (isolate each node)

#### STM32 loopback

Temporarily change `FDCAN_MODE_NORMAL` to `FDCAN_MODE_EXTERNAL_LOOPBACK`
in `MX_FDCAN1_Init()` (`Core/Src/main.c` line 540). This makes the FDCAN
peripheral transmit a frame and receive its own frame via the transceiver.
If the heartbeat counter increments in the FDCAN RX FIFO, the STM32 side
(MCU + transceiver + termination) is working.

#### ESP32 loopback

Temporarily change the TWAI mode to `TWAI_MODE_NO_ACK` (self-test mode) in
the `ESP32Can.begin()` call. In this mode, the ESP32 transmits without
requiring an ACK from another node. If the `[CAN-DIAG]` output shows
`tx_err=0` and `tx_fail=0`, the ESP32 side is working.

### Step 6 — CAN analyzer (if available)

Connect a CAN bus analyzer (PCAN-USB, CANable, USBtin, etc.) **in parallel**
to the bus (CANH, CANL, GND). Set it to 500 kbps. If frames with IDs 0x001
and 0x011 appear, both nodes are transmitting correctly.

---

## Summary of Required Actions

| Priority | Action | Status |
|----------|--------|--------|
| 🔴 **Critical** | Install 120 Ω termination between CANH and CANL at **both** ends | ❌ Not done |
| 🟡 **Verify** | TJA1051 pin 8 (S/SLNT) connected to GND on **both** modules | ⬜ Unconfirmed |
| 🟡 **Verify** | 100 nF decoupling capacitor on VCC-GND of **both** TJA1051 | ⬜ Unconfirmed |
| ✅ **Done** | TJA1051 VCC = 5V on both modules | ✅ Confirmed |
| ✅ **Done** | Common GND between ESP32 and STM32 | ✅ Confirmed |
| ✅ **Done** | CANH↔CANH, CANL↔CANL (not crossed) | ✅ Confirmed |
| ✅ **Done** | JP7 (BOOT0) in GND position | ✅ Confirmed |
| ✅ **Done** | Firmware bitrate 500 kbps on both sides | ✅ Verified in code |

---

## Files Examined

| File | Purpose |
|------|---------|
| `STM32-Control-Coche-Marcos.ioc` | CubeMX project — FDCAN1 pin/clock/timing config |
| `Core/Src/main.c` | STM32 main — MX_FDCAN1_Init(), main loop CAN calls |
| `Core/Inc/main.h` | Pin definitions (PIN_CAN_TX, PIN_CAN_RX) |
| `Core/Src/can_handler.c` | CAN_Init(), CAN_SendStatusLights(), CAN_ProcessMessages() |
| `Core/Inc/can_handler.h` | CAN message ID definitions, public API |
| `Core/Src/stm32g4xx_hal_msp.c` | HAL_FDCAN_MspInit() — GPIO/clock/interrupt config |
| `esp32/src/main.cpp` | ESP32 setup() — ESP32Can.begin(), heartbeat TX, TWAI diagnostics |
| `esp32/src/can_rx.cpp` | ESP32 CAN RX decoder |
| `esp32/include/can_ids.h` | ESP32 CAN ID definitions (frozen contract) |
| `esp32/src/screens/boot_screen.cpp` | "CAN: WAITING..." display logic |
| `esp32/platformio.ini` | ESP32-TWAI-CAN library dependency |
| `docs/ESP32_STM32_CAN_CONNECTION.md` | Physical wiring reference |
| `docs/VALIDACION_CONEXION_FISICA_CAN.md` | Physical validation checklist |
