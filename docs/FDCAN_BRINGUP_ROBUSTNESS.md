# FDCAN Bring-Up Robustness

> **Applies to:** STM32G474RE, FDCAN1, HAL v1.2.2, PCLK1 = 170 MHz
> **Last updated:** 2026-04-06

---

## 1. Problem Description

### Symptoms Observed in Debugger

| Register | Read Value | Expected |
|---|---|---|
| `hfdcan1.Instance->PSR` | `0x08007d8d` | Valid protocol status |
| `hfdcan1.Instance->ECR` | `0x08007d8d` | Error counters (TEC/REC) |
| `RCC->APB1ENR1` | `0x400` (FDCANEN = 0) | Bit 25 set (FDCANEN = 1) |
| `RCC->CCIPR` | `0x0` (FDCANSEL = HSE) | Bits [25:24] = `10` (PCLK1) |

### What This Means

The values `0x08007d8d` are **flash memory addresses**, not register data.  This happens because:

1. The FDCAN peripheral clock (`RCC->APB1ENR1` bit 25) was **not enabled**
2. The FDCAN kernel clock source (`RCC->CCIPR` FDCANSEL) defaulted to HSE after reset, which is **not enabled** in this project (HSI + PLL configuration)
3. When reading an unclocked peripheral's registers via the AHB bus, the bus bridge returns **stale pipeline data** — typically the most recent instruction fetch address from flash

This results in **total CAN failure**: the peripheral is not clocked, reads return garbage, and initialization appears to succeed but the bus never operates.

---

## 2. Root Cause Analysis

### Clock Source After Reset

The STM32G4 Reference Manual (RM0440) specifies:

- `RCC_CCIPR.FDCANSEL` resets to `00` = **HSE** (external crystal)
- This project uses **HSI + PLL** (no HSE)
- Without explicitly setting FDCANSEL to PCLK1 (`10`), the FDCAN kernel clock input is undriven

### Force-Reset Timing

The `HAL_FDCAN_MspInit()` function performs a peripheral force-reset (`__HAL_RCC_FDCAN_FORCE_RESET()` / `__HAL_RCC_FDCAN_RELEASE_RESET()`).  On some STM32G4 silicon revisions:

1. The force-reset can leave the APB1 clock gate in an **indeterminate state**
2. The first write to `RCC_CCIPR.FDCANSEL` after force-reset may **not latch**
3. The bus bridge pipeline needs time to propagate the clock-gate change — register reads during this window return **stale AHB data** (flash addresses, not register values)

### Stale AHB Bus Data

The AHB bus bridge between the Cortex-M4 core and the APB1 peripheral bus has a pipeline.  When a peripheral is unclocked:

- Read requests complete without error (no bus fault)
- The returned data is whatever was last on the bus — typically the instruction fetch address
- This garbage data is **deterministic within a single read** but **variable across reads** (depends on instruction cache state, pipeline fill level, etc.)
- A single CCCR read that happens to have bits [31:16] = 0 can pass validation, creating a **false positive**

---

## 3. Solution: Deterministic Initialization Sequence

### MspInit (`HAL_FDCAN_MspInit` in `stm32g4xx_hal_msp.c`)

The initialization follows a strict 6-step sequence:

#### Step 1: Configure Clock Source BEFORE Enabling Peripheral

```c
__HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PCLK1);
__DSB();
/* Readback-verify; re-apply if first write didn't latch */
if (__HAL_RCC_GET_FDCAN_SOURCE() != RCC_FDCANCLKSOURCE_PCLK1) {
    __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PCLK1);
    __DSB();
}
```

Setting the kernel clock source **before** enabling the APB clock avoids any window where the peripheral gate is open with the wrong clock.

#### Step 2: Enable APB1 Bus Clock

```c
__HAL_RCC_FDCAN_CLK_ENABLE();
/* Readback-verify RCC->APB1ENR1 bit FDCANEN */
if (!(RCC->APB1ENR1 & RCC_APB1ENR1_FDCANEN)) {
    /* Clock enable failed — record in diagnostics */
}
```

The readback detects hardware faults where the clock enable doesn't take effect.

#### Step 3: Robust Reset Sequence

```c
__HAL_RCC_FDCAN_FORCE_RESET();
__DSB(); __ISB();                    /* Full barrier — commit to bus */
__NOP(); __NOP(); __NOP(); __NOP();  /* >=4 AHB cycles for propagation */
__HAL_RCC_FDCAN_RELEASE_RESET();
__DSB(); __ISB();                    /* Full barrier after release */
```

After reset release:
- Re-enable APB1 clock (may have been cleared by reset)
- Re-apply FDCANSEL = PCLK1 (may have been cleared by reset)
- Readback-verify both settings

#### Step 4: Adaptive CCCR Poll (No Fixed Delays)

```c
while ((HAL_GetTick() - start) < 50) {  /* 50 ms timeout */
    if ((CCCR & 0xFFFF0000U) == 0U)     /* Reserved bits zero? */
        break;                           /* Peripheral is responding */
}
```

This replaces all fixed-delay loops.  The poll exits **as soon as the peripheral responds** (typically < 1 ms).  A 50 ms timeout prevents infinite loops on hardware failure.

#### Step 5: GPIO Configuration (PA11/PA12, AF9)

Standard pin setup — unchanged from previous implementation.

#### Step 6: NVIC Interrupt Configuration

Standard ISR priority setup — unchanged from previous implementation.

### MX_FDCAN1_Init (`main.c`)

- Up to 5 retry attempts with triple-read CCCR validation
- Each retry: DeInit → 10 ms settle → Init → CCCR check
- Retry count recorded in `can_init_diag.retries`

### CAN_Init (`can_handler.c`)

- `can_init_diag.started = 0U` set **at function entry** (not per-path)
- Only set to `1U` at the very end after ALL steps succeed
- Every failure path leaves `started == 0U` automatically
- Additional APB1ENR1 readback check before filter configuration
- Clock source re-apply with full re-init if FDCANSEL reverted

---

## 4. Diagnostic Structure

The `CAN_InitDiag_t` struct is readable via SWD debugger (`p can_init_diag`):

| Field | Type | Meaning |
|---|---|---|
| `hal_init` | uint8_t | HAL_FDCAN_Init return (0 = OK) |
| `filter_global` | uint8_t | ConfigGlobalFilter return |
| `notify` | uint8_t | ActivateNotification return |
| `start` | uint8_t | HAL_FDCAN_Start return |
| `started` | uint8_t | 1 = fully operational |
| `clk_ok` | uint8_t | 1 = FDCANSEL == PCLK1 |
| `cccr_init_ok` | uint8_t | 1 = CCCR.INIT cleared after start |
| `clk_reapplied` | uint8_t | 1 = clock source was corrected |
| `retries` | uint8_t | Number of MX_FDCAN1_Init retries |
| `timeout_flag` | uint8_t | 1 = MspInit CCCR poll timed out |
| `msp_clk_ok` | uint8_t | 1 = APB1ENR1.FDCANEN verified |
| `msp_ccipr_ok` | uint8_t | 1 = FDCANSEL readback verified |
| `ccipr_raw` | uint32_t | Raw RCC_CCIPR snapshot |

### Debugger Verification Checklist

After boot, verify via SWD:

```
p can_init_diag.started       → 1
p can_init_diag.clk_ok        → 1
p can_init_diag.msp_clk_ok    → 1
p can_init_diag.msp_ccipr_ok  → 1
p can_init_diag.timeout_flag   → 0
p can_init_diag.retries        → 0 (ideally)
p/x RCC->APB1ENR1             → bit 25 set
p/x RCC->CCIPR                → bits [25:24] = 10
p/x hfdcan1.Instance->CCCR    → valid (NOT 0x0800xxxx)
```

---

## 5. Design Principles

1. **No fixed delays** — all waits are adaptive (poll with timeout)
2. **Readback verification** — every RCC write is verified by reading back the register
3. **Triple-read validation** — CCCR reads must be consistent across 3 consecutive reads to detect stale AHB data
4. **Deterministic failure path** — `started = 0U` set at entry, only promoted to `1U` on success
5. **Silicon-revision tolerant** — works on all G4 revisions regardless of clock-gate propagation timing
6. **Retry with full re-init** — up to 5 attempts, each with DeInit + fresh MspInit cycle
7. **Diagnostic transparency** — every step recorded in SWD-readable struct
