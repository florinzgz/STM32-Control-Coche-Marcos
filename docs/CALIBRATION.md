# Persistent Calibration

Operator-facing reference for the STM32-side calibration stores currently
held in **flash bank 1 pages 124–127** (4 KB each).  Pages are reserved by
`STM32G474RETX_FLASH.ld` and protected from the linker (`FLASH` region
limited to 496 KB, leaving 16 KB exclusively for NVM).

| Page | Address      | Slot                          | Module                           |
|-----:|:-------------|:------------------------------|:---------------------------------|
| 124  | `0x0807C000` | **Pedal endpoint calibration**| `Core/Src/pedal_cal_store.c`     |
| 125  | `0x0807D000` | DS18B20 sensor-to-position map| `Core/Src/sensor_map_store.c`    |
| 126  | `0x0807E000` | Steering centring             | `Core/Src/steering_cal_store.c`  |
| 127  | `0x0807F000` | EPS parameters                | `Core/Src/eps_params.c`          |

All four stores follow the same on-flash layout (magic + payload + CRC32 +
`valid_flag` byte) and the same write recipe — unlock → erase page →
program by double-word → lock — without disabling IRQs and **without**
introducing any RTOS primitive.

---

## Pedal calibration (`pedal_cal_store.c`)

### Purpose

Allows the workshop to record the raw ADC readings corresponding to the
*released* and *fully-pressed* positions of the accelerator pedal, so the
[`Pedal_RawToPercent()`](../Core/Src/sensor_manager.c) mapping uses the
actual hardware endpoints instead of the compile-time defaults
**150 / 2413**.

### On-flash layout (page 124 — 16 B used out of 4 KB)

| Offset | Size | Field          | Type     | Notes                                       |
|-------:|-----:|:---------------|:---------|:--------------------------------------------|
| 0      | 4 B  | `magic`        | uint32_t | `0x50434C31` (`"PCL1"`)                     |
| 4      | 2 B  | `adc_min`      | uint16_t | Released-pedal endpoint (raw ADC counts)    |
| 6      | 2 B  | `adc_max`      | uint16_t | Pressed-pedal endpoint (raw ADC counts)     |
| 8      | 1 B  | `validity_flag`| uint8_t  | `0xA5` when slot is committed               |
| 9      | 3 B  | `reserved`     | uint8[3] | Padding, written as `0x00`                  |
| 12     | 4 B  | `checksum`     | uint32_t | CRC32 (poly `0xEDB88320`) over first 12 B   |

### Hard validators

A pair is rejected (and **never** written or applied) unless:

- `adc_min ≥ 50`
- `adc_max ≤ 2600`
- `adc_max > adc_min`
- `adc_max − adc_min ≥ 800`

### Fallback

If page 124 contains an invalid magic, CRC mismatch, or an out-of-range
pair, `PedalCal_Init()` silently leaves the runtime endpoints at the
compile-time defaults `150 / 2413`.  Boot is **never** blocked; the pedal
is **never** rendered unusable.

### CAN sub-protocol (`SERVICE_CMD 0x110` with byte 0 = `0xF5`)

| Sub-opcode (byte 1) | Name              | Behaviour                                                            |
|---------------------|-------------------|----------------------------------------------------------------------|
| `0x01`              | `CAPTURE_MIN`     | 8-sample 50 ms stability check → store as pending MIN if dispersion ≤ 8 |
| `0x02`              | `CAPTURE_MAX`     | Identical to CAPTURE_MIN for the pressed endpoint                    |
| `0x03`              | `SAVE`            | Validate pending pair, persist to page 124, apply immediately        |
| `0x04`              | `RESET_DEFAULTS`  | Persist compile-time defaults (150 / 2413) and apply                 |
| `0x05`              | `QUERY`           | Trigger a 1 s burst of `0x308` telemetry frames (10 frames @ 10 Hz)  |

All sub-opcodes acknowledge through the existing `CMD_ACK (0x103)`
channel — **CMD_ACK DLC is preserved at 3 bytes** (no contract change).

### Safety gates

All sub-opcodes **except `QUERY`** require, atomically before sampling:

- `Safety_GetState() == SYS_STATE_STANDBY`
- `Startup_IsInhibited() == true`
- `Pedal_GetPercent() < 3.0f`
- `Pedal_IsPlausible() == true`
- All four wheel speeds `< 0.3 km/h`

Failure → `ACK_BLOCKED_BY_SAFETY`.

### Telemetry — `DIAG_PEDAL_CAL (0x308)`

On-demand only — emitted **only** during the 1 s burst that follows a
QUERY.  Nodes that ignore 0x308 are unaffected; the bus sees zero
overhead when the calibration screen is closed.

The 8-byte payload alternates between two variants (PENDING vs STORED,
selected by flags bit 6) so the operator gets both endpoint pairs at
~5 Hz each.  See `CAN_PROTOCOL.md § 0x308` for the exact byte layout.

### Procedure (Engineering screen → PEDAL CALIBRATION)

1. Power on the vehicle with the gear lever in **Neutral** and the
   pedal **released** (the safety gate enforces this).
2. Open the hidden engineering menu (`8989`) → **PEDAL CALIBRATION**.
3. Verify the **Safety gate: OK** indicator is green.
4. With pedal fully released, tap **CAPTURE MIN** → pending MIN appears.
5. Press the pedal to its mechanical end-stop and **hold steady** while
   tapping **CAPTURE MAX** → pending MAX appears.
6. The **Validation** field turns green when the pair satisfies the
   hard validators above.  The **SAVE** button turns green only when
   both validation and the safety gate are OK.
7. Release the pedal again (so the gate is satisfied at SAVE time) and
   tap **SAVE** — the new endpoints are written to flash page 124 and
   applied immediately.
8. Reboot to confirm persistence — both `Stored MIN` and `Stored MAX`
   should reflect the values you saved.

To restore the factory endpoints at any time, tap **RESET DEFAULTS**;
this re-persists 150 / 2413 and applies them immediately.

### Backward compatibility

- ESP32 firmware that predates this feature continues to operate normally — the new CAN messages `0x308` and the new `SERVICE_CMD` sub-opcodes are simply ignored.
- Any CAN bus node that does not recognise `0x308` discards the frame as part of standard CAN filtering — no flooding, no errors.
- The compile-time defaults remain unchanged at **150 / 2413**, so a board flashed without ever running the wizard behaves bit-identically to the previous firmware.
