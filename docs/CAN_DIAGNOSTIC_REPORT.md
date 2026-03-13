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
| Transceiver type | TJA1051T/3 (NXP, 3.3V logic) × 2 |
| Termination | 2 × 120 Ω required (one at each bus end) |
| Bus standard | CAN 2.0A (11-bit ID, classic frames) |

---

## Most Likely Cause of the Issue

### Firmware analysis result: **No software defects detected**

Both the STM32 and ESP32 firmware are correctly configured:
- Matching bitrate (500 kbps) on both sides
- Correct CAN mode (NORMAL) on both sides
- `HAL_FDCAN_Start()` is called and the peripheral is active
- The STM32 begins transmitting heartbeat 0x001 immediately after boot
- The ESP32 CAN driver is initialized and polling for frames
- All CAN message IDs are consistent between both firmwares

### Root cause is hardware-related

Since the firmware configuration is correct, the **"CAN: WAITING..."** symptom indicates that CAN frames are not reaching the ESP32 at the physical layer. The most probable hardware causes, in order of likelihood:

#### 1. Missing 120 Ω termination resistors (MOST LIKELY)

CAN requires two 120 Ω termination resistors — one at each end of the bus. Without proper termination, signal reflections corrupt the differential signal, causing bit errors and preventing frame reception. Many TJA1051 breakout modules ship with the termination jumper **disabled by default**.

**Action:** Verify the termination jumper is enabled on both TJA1051 modules. Measure resistance between CANH and CANL (should be ~60 Ω with both terminators in place).

#### 2. TJA1051 S/SLNT (standby/silent) pin not grounded

The TJA1051 pin 8 (S or SLNT) must be connected to GND for normal operation. If left floating or pulled high, the transceiver enters standby mode and will not transmit or receive.

**Action:** Verify pin 8 on both TJA1051 modules is connected to GND.

#### 3. CANH/CANL crossed between transceivers

If CANH on one transceiver connects to CANL on the other (and vice versa), the differential signal is inverted and no frames will be decoded.

**Action:** Trace the physical wiring to ensure CANH↔CANH and CANL↔CANL.

#### 4. BOOT0 jumper (JP7) on Nucleo-64 in wrong position

PB8 (FDCAN1_RX) shares a PCB trace with BOOT0 via JP7. If JP7 is set to VDD, PB8 is held high and cannot function as CAN RX.

**Action:** Verify JP7 is in the GND position (factory default).

#### 5. Insufficient or missing 5V supply to transceivers

The TJA1051 requires 5V VCC. If powered from 3.3V, the transceiver may not operate correctly.

**Action:** Measure VCC on pin 3 of both TJA1051 modules — must be 4.75–5.25V.

---

## Recommended Debugging Steps

1. **Serial monitor check:** Connect USB to both boards and verify:
   - STM32: no error messages (CAN init is silent but non-fatal)
   - ESP32: look for `[CAN] Initialized at 500 kbps` vs `[CAN] Initialization FAILED`

2. **Multimeter check:** Measure CANH-to-CANL resistance (expect ~60 Ω)

3. **Oscilloscope check:** Probe CANH/CANL on the STM32 transceiver output — should show differential signaling at 500 kbps

4. **Loopback test (STM32):** Temporarily change `FDCAN_MODE_NORMAL` to `FDCAN_MODE_EXTERNAL_LOOPBACK` in `MX_FDCAN1_Init()` to test the transceiver independently

5. **Loopback test (ESP32):** Use the TWAI self-test mode to verify the ESP32 CAN peripheral operates correctly

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
| `esp32/src/main.cpp` | ESP32 setup() — ESP32Can.begin(), heartbeat TX |
| `esp32/src/can_rx.cpp` | ESP32 CAN RX decoder |
| `esp32/include/can_ids.h` | ESP32 CAN ID definitions (frozen contract) |
| `esp32/src/screens/boot_screen.cpp` | "CAN: WAITING..." display logic |
| `esp32/platformio.ini` | ESP32-TWAI-CAN library dependency |
| `docs/ESP32_STM32_CAN_CONNECTION.md` | Physical wiring reference |
