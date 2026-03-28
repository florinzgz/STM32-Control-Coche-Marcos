# CAN Communication Diagnostic Report

**Date:** 2026-03-27 (rev 2 — full rewrite)
**System:** STM32G474RE (Nucleo-64) ↔ ESP32-S3 HMI
**Purpose:** Verify and diagnose CAN communication before any firmware or
pin modification.  Analyse GDB register dump captured during debug session.

> **Regla:** Este informe es solo lectura.  No se modifica firmware, pines,
> IDs, filtros ni protocolo antes de completar todas las verificaciones
> descritas aquí.

---

## 1. STM32 FDCAN1 — Current Configuration

| Parameter | Value | Source |
|-----------|-------|--------|
| **Peripheral** | FDCAN1 | `.ioc` (Mcu.IP1=FDCAN1), `main.c` |
| **TX pin** | **PA12** (AF9) | `.ioc`, `project_config.h:214`, `stm32g4xx_hal_msp.c:56` |
| **RX pin** | **PA11** (AF9) | `.ioc`, `project_config.h:215`, `stm32g4xx_hal_msp.c:56` |
| **Mode** | FDCAN_MODE_NORMAL | `main.c MX_FDCAN1_Init()` |
| **Frame format** | FDCAN_FRAME_CLASSIC (CAN 2.0A) | `main.c MX_FDCAN1_Init()` |
| **Clock source** | **PCLK1** (170 MHz) | `SystemClock_Config()`, `.ioc RCC.FDCANCLKSelection` |
| **Prescaler (BRP)** | 10 | `main.c` — `NominalPrescaler = 10` |
| **TimeSeg1** | 29 TQ | `main.c` — `NominalTimeSeg1 = 29` |
| **TimeSeg2** | 4 TQ | `main.c` — `NominalTimeSeg2 = 4` |
| **SyncJumpWidth** | 4 TQ | `main.c` — `NominalSyncJumpWidth = 4` |
| **Bit time** | 1 + 29 + 4 = **34 TQ** | — |
| **Bitrate** | 170 MHz / (10 × 34) = **500 kbps** ✅ | — |
| **Sample point** | (1 + 29) / 34 = **88.2 %** | CiA 301 recommends 87.5 % ±2 % ✅ |
| **SJW tolerance** | ±4/34 = ±11.8 % oscillator | HSI ±1 % → >10× margin ✅ |
| **Auto retransmission** | ENABLE | `main.c` |
| **Transmit pause** | ENABLE | 2 TQ idle between consecutive TX |
| **Init failure** | **Non-fatal** (`fdcan_init_ok = false`, no `Error_Handler`) | `main.c:684` |
| **Interrupt lines** | FDCAN1_IT0 + FDCAN1_IT1, priority 1 | `stm32g4xx_hal_msp.c:63-66` |

### Periodic CAN transmissions from STM32

| Rate | Messages |
|------|----------|
| 100 ms | Heartbeat (0x001), Speed (0x200), Current (0x201), Safety (0x203), Steering (0x204), Traction (0x205), Battery (0x207) |
| 1000 ms | Temperature (0x202), TempMap (0x206), Lights (0x20A), ServiceStatus (0x301–0x305), ErrorLog |
| 500 ms | Test frame (0x123) via `CAN_TestTransmit()` |

---

## 2. ESP32-S3 TWAI — Current Configuration

| Parameter | Value | Source |
|-----------|-------|--------|
| **CAN controller** | TWAI (IDF native — manual `twai_driver_install`) | `main.cpp:349–390` |
| **TX pin** | GPIO 4 | `main.cpp:86` — `CAN_TX_PIN = 4` |
| **RX pin** | GPIO 5 | `main.cpp:87` — `CAN_RX_PIN = 5` |
| **Mode** | TWAI_MODE_NORMAL | `main.cpp:372` |
| **Prescaler (BRP)** | 10 | `main.cpp:380` |
| **TimeSeg1** | 13 TQ | `main.cpp:381` |
| **TimeSeg2** | 2 TQ | `main.cpp:382` |
| **SyncJumpWidth** | 2 TQ | `main.cpp:383` |
| **Bit time** | 1 + 13 + 2 = **16 TQ** | — |
| **Bitrate** | 80 MHz / (10 × 16) = **500 kbps** ✅ | APB clock = 80 MHz |
| **Sample point** | (1 + 13) / 16 = **87.5 %** | CiA 301 recommended ✅ |
| **SJW tolerance** | ±2/16 = ±12.5 % oscillator | Crystal osc → negligible drift ✅ |
| **RX queue** | 5 frames | `main.cpp:375` |
| **TX queue** | 5 frames | `main.cpp:376` |
| **Filter** | Accept ALL (TWAI_FILTER_CONFIG_ACCEPT_ALL) | `main.cpp:386` |
| **Triple sampling** | false | `main.cpp:384` |
| **Init check** | `twai_driver_install() && twai_start()` → ESP_OK | `main.cpp:388–392` |

### GPIO 4/5 safety

| Check | Result |
|-------|--------|
| GPIO4 bootstrapping? | **No** — ESP32-S3 bootstrap pins are GPIO0, GPIO3, GPIO45, GPIO46. GPIO4 is safe ✅ |
| GPIO5 bootstrapping? | **No** — safe ✅ |
| Floating voltage? | Driven by TWAI peripheral when driver is installed. No pull-up/down needed for CAN ✅ |

### ESP32 heartbeat and diagnostics

| Feature | Details |
|---------|---------|
| **Heartbeat TX** | ID 0x011, DLC 1, 100 ms interval (`main.cpp:1020–1031`) |
| **Heartbeat RX** | ID 0x001 from STM32 decoded in `can_rx::poll()` |
| **TWAI diagnostics** | `[CAN-DIAG]` on serial every 5 s (`main.cpp:1039–1061`) |

---

## 3. Bit Timing Compatibility Analysis

| Parameter | ESP32 TWAI | STM32 FDCAN | Delta |
|-----------|-----------|-----------|-------|
| **APB/PCLK** | 80 MHz | 170 MHz | — |
| **BRP** | 10 | 10 | — |
| **TQ frequency** | 8 MHz | 17 MHz | — |
| **TQ period** | 125.00 ns | 58.82 ns | — |
| **Sync + Seg1 + Seg2** | 1 + 13 + 2 = 16 TQ | 1 + 29 + 4 = 34 TQ | — |
| **Bit period** | 16 × 125 ns = **2.000 µs** | 34 × 58.82 ns = **2.000 µs** | **0.00 µs** ✅ |
| **Bitrate** | **500.000 kbps** | **500.000 kbps** | **0.000 kbps** ✅ |
| **Sample point** | **87.5 %** | **88.2 %** | **0.7 %** ✅ |
| **SJW** | 2 TQ (±12.5 %) | 4 TQ (±11.8 %) | — |

### Verdict: **COMPATIBLE** ✅

The 0.7% sample point difference is well within the CiA 301 ±2% window.
Both SJW values provide >10× margin over the actual oscillator tolerances
(ESP32 crystal ≈0%, STM32 HSI ±1%).  No timing changes required.

---

## 4. STM32 Initialization Sequence Verification

### Boot flow (main.c)

```
HAL_Init()                    ← SysTick, HAL_MspInit (enables PWR + SYSCFG clocks)
 ↓
Boot_ReadResetCause()         ← Read RCC_CSR reset flags
 ↓
SystemClock_Config()          ← HSI → PLL (170 MHz), FDCAN clock → PCLK1
 ↓
MX_GPIO_Init()                ← 3× LED startup blinks
 ↓
MX_FDCAN1_Init()              ← HAL_FDCAN_Init → HAL_FDCAN_MspInit callback:
                                 __HAL_RCC_FDCAN_CLK_ENABLE()
                                 __HAL_RCC_FDCAN_FORCE_RESET()
                                 __HAL_RCC_FDCAN_RELEASE_RESET()
                                 GPIOA PA11/PA12 AF9
                                 NVIC: IT0 + IT1 (priority 1)
                               Sets fdcan_init_ok = true on success
 ↓
CAN_Init()                    ← Checks fdcan_init_ok, then:
                                 CAN_ConfigureFilters()
                                 HAL_FDCAN_ActivateNotification(RX_FIFO0)
                                 HAL_FDCAN_Start()
                                 Records each step in can_init_diag
 ↓
Main loop                     ← CAN_SendHeartbeat() every 100 ms
                                 CAN_CheckBusOff()  every 10 ms
```

### CAN_InitDiag_t — reading via SWD

The global `can_init_diag` struct records every FDCAN init step.
Read it through the SWD debugger after boot completes:

```gdb
p can_init_diag
```

| Field | OK value | Meaning if ≠ 0 |
|-------|----------|----------------|
| `hal_init` | 0 | `HAL_FDCAN_Init()` failed (clock / CCCR timeout) |
| `filter_global` | 0 | `HAL_FDCAN_ConfigGlobalFilter()` failed |
| `notify` | 0 | `HAL_FDCAN_ActivateNotification()` failed |
| `start` | 0 | `HAL_FDCAN_Start()` failed |
| `started` | 1 | 0 = FDCAN never reached active state |

### CAN_Diag_t — reading runtime errors via SWD

The global `can_diag` struct is updated every 10 ms by `CAN_CheckBusOff()`:

```gdb
p can_diag
```

| Field | Healthy | Problem |
|-------|---------|---------|
| `last_error_code` | 0 | 1=Stuff 2=Form **3=ACK** 4=Bit1 5=Bit0 6=CRC |
| `tec` | 0 | >96 = warning, ≥128 = bus-off |
| `rec` | 0 | >96 = warning |
| `error_passive` | 0 | 1 = Error Passive state |
| `warning` | 0 | 1 = error warning threshold |
| `bus_off` | 0 | 1 = Bus-Off (fatal — will attempt recovery) |

### Filter configuration

```c
// Non-matching standard IDs → accepted into RX FIFO0
HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
    FDCAN_ACCEPT_IN_RX_FIFO0,    // ← non-matching std IDs: ACCEPT
    FDCAN_REJECT,                 // ← non-matching ext IDs: REJECT
    FDCAN_REJECT_REMOTE,          // ← remote std:  REJECT
    FDCAN_REJECT_REMOTE);         // ← remote ext:  REJECT
```

Five specific filters (0–4) for known IDs, plus the global accept-all
for any unknown standard ID → **no legitimate frame will be rejected** ✅.

---

## 5. GDB Register Dump Analysis

### Raw debug output (from problem statement)

```
p/x RCC->CCIPR               → No symbol "RCC" in current context.
p/x RCC->APB1ENR1             → No symbol "RCC" in current context.
p/x *(uint32_t*)0x40021000    → $2 = 0x500          ← RCC_CR
p/x *(uint32_t*)0x40021088    → $3 = 0x0            ← RCC_CCIPR
p/x *(uint32_t*)0x40021058    → $4 = 0x400          ← RCC_APB1ENR1
p/x hfdcan1.Instance->CCCR    → $5 = 0x8007add      ← FDCAN1 CCCR
```

### Register interpretation

#### RCC_CR (0x4002 1000) = 0x0000 0500

```
Bit 24 PLLON  = 0  → PLL is OFF
Bit 16 HSEON  = 0  → HSE is OFF
Bit 10 HSIRDY = 1  → HSI16 oscillator ready
Bit  8 HSION  = 1  → HSI16 oscillator enabled
```

**Interpretation:** The system is running on HSI16 (16 MHz).  PLL has not
been configured, so SYSCLK = 16 MHz, not 170 MHz.  HSI16 is the default
clock source after reset on STM32G4 (this family has no MSI oscillator).
The PLL was never enabled — either `SystemClock_Config()` is still in
progress or it failed and called `Error_Handler()`.

#### RCC_CCIPR (0x4002 1088) = 0x0000 0000

```
Bits [25:24] FDCANSEL = 00  → FDCAN clock source = HSE (reset default)
```

**Interpretation:** The `HAL_RCCEx_PeriphCLKConfig()` call at the end of
`SystemClock_Config()` never executed.  FDCANSEL defaults to HSE (00), but
HSE is not enabled (HSEON = 0 in RCC_CR above).  **The FDCAN peripheral
has no kernel clock.**

#### RCC_APB1ENR1 (0x4002 1058) = 0x0000 0400

```
Bit 28 PWREN   = 0  → PWR peripheral clock NOT enabled
Bit 25 FDCANEN = 0  → FDCAN peripheral clock NOT enabled
Bit 10 RTCAPBEN = 1 → RTC APB clock enabled (reset default)
```

**Interpretation:** This is the reset-default value.  `HAL_MspInit()` (which
enables PWR via `__HAL_RCC_PWR_CLK_ENABLE()`) has not yet taken effect in
this register snapshot, OR the MCU was reset after `HAL_MspInit()` ran and
we are seeing the fresh-reset default again.  **The FDCAN APB bus clock is
not enabled**, so reading FDCAN registers returns unpredictable bus data.

#### FDCAN1 CCCR (via hfdcan1.Instance) = 0x8007 ADD

```
Expected healthy CCCR after HAL_FDCAN_Start():
  INIT = 0, CCE = 0  → peripheral running, bus active

Observed: 0x8007ADD — this is NOT a valid CCCR value.
  Normal CCCR bit field covers bits [15:0]; bit 31 should be reserved.
```

**Interpretation:** The FDCAN peripheral clock is not enabled
(FDCANEN = 0 in APB1ENR1).  Reading registers of an un-clocked peripheral
returns **bus garbage** — the ARM bus returns whatever data happens to be on
the bus.  This value has no diagnostic meaning.

### Root cause summary

```
         reset
           ↓
     HAL_Init()            ← May or may not have completed
           ↓
     SystemClock_Config()  ← HSI enabled OK, but PLL NOT configured
           ↓
     Error_Handler()?      ← If HAL_PWREx_ControlVoltageScaling or
                              HAL_RCC_OscConfig returned HAL_ERROR
           ↓               ← OR debugger halted MCU mid-boot
     FDCAN never init'd    ← No clock, no MspInit, CCCR = garbage
```

**Two possible explanations:**

1. **Debugger halted during early boot:** The SWD debugger halted the CPU
   between `HAL_RCC_OscConfig()` enabling HSI and the PLL lock.  In this
   case, resuming execution will complete SystemClock_Config and FDCAN init
   normally.  **Test:** Set a breakpoint at the main loop (after all
   `MX_*_Init()` calls) and let the MCU run to it.

2. **SystemClock_Config() failed:** `HAL_PWREx_ControlVoltageScaling()` or
   `HAL_RCC_OscConfig()` returned `HAL_ERROR`, causing `Error_Handler()` to
   be called.  The MCU is now stuck in an infinite blink loop.  PLL was never
   configured.  **Test:** Check LD2 (PA5) — if it blinks at ~2 Hz (not the
   brief flash every 2 s), the system is in `Error_Handler()`.

---

## 6. SWD Step-by-Step Debugging Procedure

### Step 1 — Verify boot completed

```gdb
# Breakpoint after all init, before main loop
b main.c:200
continue
```

If the breakpoint is never reached, the MCU is stuck in `Error_Handler()`.
Check the LED pattern (section 5 above).

### Step 2 — Verify clock configuration

```gdb
# RCC_CR: expect PLLON=1, PLLRDY=1, HSION=1
p/x *(uint32_t*)0x40021000
# Healthy value: 0x03000500 or higher (bits 24+25 set = PLL on + ready)

# RCC_CCIPR: expect FDCANSEL = 10 (PCLK1)
p/x *(uint32_t*)0x40021088
# Healthy value: bits [25:24] = 10 → value has 0x02000000 set

# RCC_APB1ENR1: expect FDCANEN=1 (bit 25), PWREN=1 (bit 28)
p/x *(uint32_t*)0x40021058
# Healthy value: bits 25 + 28 set → at least 0x12000400
```

### Step 3 — Verify FDCAN init result

```gdb
# Was HAL_FDCAN_Init successful?
p fdcan_init_ok
# Expected: true (1)

# What step failed (if any)?
p can_init_diag
# Expected: {hal_init = 0, filter_global = 0, notify = 0, start = 0, started = 1}
```

### Step 4 — Verify FDCAN is running

```gdb
# CCCR register: INIT bit must be 0
p/x hfdcan1.Instance->CCCR
# Healthy: INIT(bit0) = 0, CCE(bit1) = 0 → value like 0x0000_0000 or 0x0000_0040

# Protocol Status Register
p/x hfdcan1.Instance->PSR
# Healthy: LEC(bits 2:0) = 7 (no change), ACT(bits 4:3) = 00 (error active)
#          → typical idle value: 0x0000_0707

# Error Counter Register
p/x hfdcan1.Instance->ECR
# Healthy: TEC(bits 7:0) = 0, REC(bits 14:8) = 0 → value = 0x0000_0000
```

### Step 5 — Read runtime diagnostic struct

```gdb
p can_diag
# Expected healthy:
#   last_error_code = 0    (no error)
#   tec = 0                (no TX errors)
#   rec = 0                (no RX errors)
#   error_passive = 0
#   warning = 0
#   bus_off = 0

# Common problem patterns:
#   last_error_code = 3 (ACK) → no other node acknowledging (ESP32 off / wiring)
#   tec >= 128              → bus-off (physical layer failure)
#   bus_off = 1             → recovery in progress
```

### Step 6 — Read Message RAM and FIFO status

```gdb
# RX FIFO 0 Status (RXF0S): how many messages waiting
p/x hfdcan1.Instance->RXF0S
# Bits [3:0] = fill level (0 = empty)

# TX FIFO Queue Status (TXFQS): free TX slots
p/x hfdcan1.Instance->TXFQS
# Bits [2:0] = free level (should be 3 if not transmitting)

# TX Event FIFO Status
p/x hfdcan1.Instance->TXEFS
```

### Step 7 — Transmit a test frame

In GDB, call the built-in test function:

```gdb
call CAN_TestTransmit()
```

Then immediately check:

```gdb
# PSR: Did the frame get an ACK?
p/x hfdcan1.Instance->PSR
# If LEC = 3 (ACK error) → no receiver on bus

# ECR: Did TEC increment?
p/x hfdcan1.Instance->ECR
```

---

## 7. ESP32 TWAI Debugging

### Serial monitor (115200 baud)

Connect USB to the ESP32 and look for:

```
[CAN] Initialized at 500 kbps (SP=87.5%)    ← success
```

or:

```
[CAN] Initialization FAILED                 ← driver install failed
```

### TWAI diagnostic output (every 5 seconds)

```
[CAN-DIAG] state=RUNNING tx_err=0 rx_err=0 tx_fail=0 rx_miss=0 arb_lost=0 bus_err=0
```

| Field | Healthy | Problem |
|-------|---------|---------|
| `state` | `RUNNING` | `BUS_OFF` = too many errors; `STOPPED` = init failed |
| `tx_err` | 0 | >96 = warning; ≥128 = bus-off |
| `rx_err` | 0 | >96 = warning; rising = signal quality |
| `tx_fail` | 0 | >0 = **no ACK** from STM32 (most common symptom) |
| `rx_miss` | 0 | >0 = RX queue too small |
| `arb_lost` | 0 | Normal with multiple TX nodes |
| `bus_err` | 0 | >0 = physical layer errors |

### Common diagnostic patterns

| Pattern | Likely cause |
|---------|-------------|
| `state=BUS_OFF tx_err=128+` | No termination resistors or wiring fault |
| `state=RUNNING tx_err=0 rx_err=0` but no heartbeat | STM32 not transmitting (check `can_init_diag.started`) |
| `tx_fail` incrementing | No ACK — STM32 FDCAN not started or transceiver silent |
| `bus_err` rising rapidly | Crossed CANH/CANL, missing GND, noise |
| `state=STOPPED` | `twai_driver_install()` or `twai_start()` returned error |

### Heartbeat verification

The ESP32 sends heartbeat 0x011 every 100 ms with an incrementing counter
in byte 0.  If the STM32 is receiving correctly, `can_stats.rx_count`
(visible via SWD) should increment continuously.

---

## 8. Transceiver and Physical Layer

| Parameter | Value | Source |
|-----------|-------|--------|
| **Transceiver IC** | SN65HVD230 (TI, 3.3V logic I/O, 3.3V bus) | `docs/ESP32_STM32_CAN_CONNECTION.md` |
| **Quantity** | 2 (one per board) | — |
| **VCC** | 3.3V | Pin 3 on each module |
| **Rs/SLNT pin** | Must be GND (high-speed mode) | Pin 8 on each module |

### Wiring diagram

```
STM32 side                    SN65HVD230 #1      Bus          SN65HVD230 #2                 ESP32 side
─────────────────────────────────────────────────────────────────────────────────────────────────────────
PA12 (TX, AF9) ──→ Pin 1 D                                   Pin 1 D   ←── GPIO 4 (TX)
PA11 (RX, AF9) ←── Pin 4 R                                   Pin 4 R   ──→ GPIO 5 (RX)
3.3V           ──→ Pin 3 VCC                                  Pin 3 VCC ←── 3.3V
GND            ──→ Pin 2 GND                                  Pin 2 GND ←── GND
                   Pin 8 Rs → GND                             Pin 8 Rs → GND
                   Pin 7 CANH ←──── twisted pair ────→ Pin 7 CANH
                   Pin 6 CANL ←──── twisted pair ────→ Pin 6 CANL
                        ┊                                    ┊
                     [120 Ω]                              [120 Ω]
                    (CANH–CANL)                          (CANH–CANL)
```

### Termination

CAN 2.0 (ISO 11898-2) requires **120 Ω** at each physical end of the bus.

| Measured CANH–CANL | Status |
|--------------------|--------|
| **~60 Ω** | 2 terminators ✅ |
| ~120 Ω | Only 1 ⚠️ |
| Open / >1 kΩ | None ❌ — **bus will NOT work** |

### Hardware checklist

- [ ] SN65HVD230 pin 8 (Rs/SLNT) to GND on **both** modules
- [ ] VCC = 3.3V on **both** modules
- [ ] CANH ↔ CANH, CANL ↔ CANL (not crossed)
- [ ] Common GND wire between boards
- [ ] 120 Ω termination at **both** ends
- [ ] 100 nF decoupling caps on VCC–GND of each SN65HVD230

---

## 9. Diagnostic Verdict

### Firmware configuration: **CORRECT — no changes needed** ✅

| Check | STM32 | ESP32 | Match? |
|-------|-------|-------|--------|
| Bitrate | 500 kbps | 500 kbps | ✅ |
| Sample point | 88.2 % | 87.5 % | ✅ (Δ 0.7 %) |
| SJW margin | ±11.8 % | ±12.5 % | ✅ |
| CAN mode | NORMAL | NORMAL | ✅ |
| Frame format | Classic 2.0A | Classic 2.0A | ✅ |
| Message IDs | can_handler.h | can_ids.h | ✅ (all 25+ IDs match) |
| Heartbeat period | 100 ms | 100 ms | ✅ |
| Global filter | Accept FIFO0 | Accept ALL | ✅ |
| FDCAN reset in MspInit | FORCE+RELEASE | N/A | ✅ |
| Non-fatal CAN init | Yes (fdcan_init_ok) | Yes (ESP_OK check) | ✅ |

### GDB register snapshot: **System not fully booted**

The register values from the problem statement show the MCU was halted
before `SystemClock_Config()` completed (PLL OFF, FDCAN clock not enabled,
CCIPR at reset default).  **This does not indicate a firmware bug** — it
indicates the debugger halted the CPU during early boot, OR
`SystemClock_Config()` failed and the system is in `Error_Handler()`.

**To distinguish:**

| Observation | Meaning |
|-------------|---------|
| LD2 blinks ~2 Hz continuously | `Error_Handler()` — clock config failed |
| LD2 does 3 quick blinks then brief flash every ~2 s | Normal boot — FDCAN should be working |
| LD2 blinks rapidly (~10 Hz) | Fault handler (HardFault, etc.) |
| No LD2 activity | MCU not executing user code (check BOOT0 / flash) |

### Recommendations

| # | Action | When |
|---|--------|------|
| 1 | Set breakpoint after `CAN_Init()`, let MCU run to it | Immediate |
| 2 | Read `can_init_diag` — all fields should be 0/1 | At breakpoint |
| 3 | Read `can_diag` after 1 second of running | After resuming |
| 4 | Verify termination (60 Ω CANH–CANL) | Before bus test |
| 5 | Check `[CAN-DIAG]` on ESP32 serial monitor | In parallel |
| 6 | If `last_error_code = 3` (ACK) on both sides | Wiring / transceiver issue |
| 7 | If everything reads healthy but no communication | Try `CAN_TestTransmit()` from GDB |

---

## 10. Advanced Debug Options

### Read FDCAN Message RAM registers

```gdb
# Standard filter element 0 (first configured filter)
p/x *(uint32_t*)(0x4000A400)

# RX FIFO 0 status
p/x hfdcan1.Instance->RXF0S
#   Bits [6:4] = get index, [3:0] = fill level, bit 24 = full, bit 25 = lost

# TX FIFO/Queue status
p/x hfdcan1.Instance->TXFQS
#   Bits [4:0] = free level, bits [20:16] = put index

# TX Buffer Request Pending
p/x hfdcan1.Instance->TXBRP

# Interrupt Register (check which interrupts fired)
p/x hfdcan1.Instance->IR
```

### Force a test frame from GDB

```gdb
call CAN_TestTransmit()

# Then check:
p/x hfdcan1.Instance->PSR       # LEC = 3 means ACK error (no receiver)
p/x hfdcan1.Instance->ECR       # TEC increment = TX error counted
p can_diag                       # Updated on next CAN_CheckBusOff() cycle
```

### ESP32 test commands

From the ESP32 serial console or code:

```cpp
// Send heartbeat manually
CanFrame f = {};
f.identifier = 0x011;
f.data_length_code = 1;
f.data[0] = 0xAA;
ESP32Can.writeFrame(f);

// Send throttle command
f.identifier = 0x100;  // CMD_THROTTLE
f.data_length_code = 1;
f.data[0] = 0;         // 0% throttle (safe)
ESP32Can.writeFrame(f);

// Send mode command
f.identifier = 0x102;  // CMD_MODE
f.data_length_code = 2;
f.data[0] = 0x00;      // No mode flags
f.data[1] = 0x02;      // Neutral gear
ESP32Can.writeFrame(f);
```

### STM32 internal loopback test

To test the STM32 FDCAN without any bus hardware, temporarily set
`CAN_LOOPBACK_TEST` to 1 (compile-time define in `main.c:39`).
This switches `MX_FDCAN1_Init()` to `FDCAN_MODE_INTERNAL_LOOPBACK`
and `CAN_ConfigureFilters()` to accept all IDs (0x000–0x7FF).

In loopback mode, TX frames are routed back to RX internally — no
transceiver or bus needed.  Verify that `can_stats.tx_count` and
`can_stats.rx_count` both increment.

---

## Files Examined

| File | Purpose |
|------|---------|
| `Core/Src/main.c` | `SystemClock_Config()`, `MX_FDCAN1_Init()`, main loop |
| `Core/Inc/main.h` | HAL handle externs |
| `Core/Inc/project_config.h` | PIN_CAN_TX (PA12), PIN_CAN_RX (PA11) |
| `Core/Src/stm32g4xx_hal_msp.c` | `HAL_FDCAN_MspInit()` — clock, reset, GPIO, NVIC |
| `Core/Src/can_handler.c` | `CAN_Init()`, filters, heartbeat, bus-off recovery, diagnostics |
| `Core/Inc/can_handler.h` | CAN IDs, `CAN_Diag_t`, `CAN_InitDiag_t` |
| `esp32/src/main.cpp` | TWAI manual init, heartbeat TX, `[CAN-DIAG]` output |
| `esp32/src/can_rx.cpp` | CAN RX frame decoder |
| `esp32/include/can_ids.h` | CAN protocol contract (IDs, enums, timing) |
| `STM32-Control-Coche-Marcos.ioc` | CubeMX config (FDCAN1, RCC, pins) |
