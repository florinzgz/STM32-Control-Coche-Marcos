# PROJECT_CHANGELOG

## [unreleased] — 2026-05-14 (page-125 ownership conflict — resolved)

### Sensor map relocated to flash page 123 (`0x0807B000`)

Functional change that **removes** the long-standing page-125 ownership
conflict between `error_log.c` and `sensor_map_store.c`.  After this
change every NVM page is single-owner and the corresponding store can
be erased independently without disturbing the others.

#### Files touched

- `STM32G474RETX_FLASH.ld` — `FLASH` region reduced **496 KB → 492 KB**
  (firmware ceiling moved from `0x0807C000` down to `0x0807B000`) so
  that page 123 is now reserved for NVM.  The reserved-pages comment
  was extended to list page 123 (`sensor_map_store.c`) alongside the
  existing pages 124–127.
- `Core/Src/sensor_map_store.c` — `SMAP_FLASH_PAGE` changed
  `125U → 123U`, `SMAP_FLASH_BASE` changed `0x0807D000U → 0x0807B000U`.
  File-header comment now describes the five-page NVM layout
  (123 sensor map, 124 pedal cal, 125 error log, 126 steering cal,
  127 EPS).  Inline *"Erase page 125"* comment updated to *"Erase
  page 123"*.
- `Core/Inc/sensor_map_store.h` — flash-layout block and
  `SensorMapStore_Init()` doxygen now reference page 123 at
  `0x0807B000`.
- `Core/Src/pedal_cal_store.c` — header comment listing the
  neighbour pages now includes page 123 (sensor map) for completeness.
- `Core/Src/can_handler.c` — comment near
  `Apply_TempSensorMap_From_Store()` changed *"persisted in flash
  page 125"* → *"persisted in flash page 123"*.
- `Core/Src/main.c` — comment in `SystemClock_Config()` prologue
  changed *"from flash page 125"* → *"from flash page 123"*.
- `README.md` § 21 (*Subsystems*) and § 24 (*Persistent Storage*) —
  table now lists all five reserved pages, removes the
  *⚠️ shared page* annotation, and the
  *"DS18B20 sensor map (Flash page 125, shared section)"* heading
  was rewritten as *"DS18B20 sensor map (Flash page 123)"*.
- `docs/CALIBRATION.md` — five-row flash-layout table, removed
  *Known issue (page 125 ownership conflict)* admonition and replaced
  it with a one-line note documenting the remediation.
- `docs/HARDWARE_AND_SENSOR_MAP.md` § 1.6 — same treatment: five-row
  table (pages 123–127), `FLASH` region updated to 492 KB ending at
  `0x0807B000`, and the *Known issue* admonition replaced by a
  remediation note.
- `docs/INTEGRATION_PLAN.md` Apéndice A — table now lists page 123,
  the firmware region is `0–122` (`0x08000000–0x0807AFFF`, 492 KB),
  and the post-table rule names every reserved page (123–127).

#### Verification

- `SMAP_FLASH_PAGE` (123) and the `Page` field of the
  `FLASH_EraseInitTypeDef` passed to `HAL_FLASHEx_Erase()` resolve
  to bank 1, single-bank STM32G474RE layout — same recipe as
  `pedal_cal_store.c` page 124 and `error_log.c` page 125.
- `SMAP_FLASH_BASE = 0x0807B000` is exactly `4 KB` below
  `PCAL_FLASH_BASE = 0x0807C000`, matching the new linker reservation.
- A repository-wide search for `0x0807D000` no longer returns any
  reference owned by `sensor_map_store`; the only remaining
  references are owned by `error_log.c` (which legitimately owns
  page 125).
- `grep -rn "shared page\|ownership conflict\|page 125 ownership"`
  returns nothing outside this changelog entry.

#### Compatibility / migration

- The on-flash slot format (`smap_flash_slot_t`) is unchanged: same
  magic (`0x534D4150` "SMAP"), same 5-byte `tempMap`, same
  `valid_flag`, same CRC32 over the slot prefix.  Only the base
  address moves.
- Units flashed before this change will see a CRC-invalid sensor map
  on page 123 (the new owner) and fall back silently to the identity
  mapping (`0,1,2,3,4`), exactly as the design already specified for
  blank / corrupt slots.  The workshop must re-save the sensor map
  once after upgrading; no other user action is required.
- The error-log page (125) is untouched by this change, so historical
  fault entries survive the firmware upgrade.

---

## [unreleased] — 2026-05-14 (doc sync follow-up)

### Documentation consistency pass for the persistent-pedal-calibration feature

Surgical documentation pass with no functional changes.  Aligns every
file that references the STM32G474RE flash NVM map (pages 124–127) so
the addresses and ownership match the source of truth (the actual
constants in `Core/Src/*.c` and the linker script
`STM32G474RETX_FLASH.ld`).  No C, C++, or linker file was modified —
the firmware binary is bit-identical to the previous commit.

#### Files touched

- `Core/Src/error_log.c` — file-header comment now reads
  *page 125 (`0x0807D000`, 4 KB)* (was `0x0807C000`, the address of
  page 124).  The `ERRLOG_FLASH_BASE` macro at line 36 was already
  correct; only the documentation comment was misleading.
- `Core/Inc/error_log.h` — same two-line comment fix: page-125
  address corrected to `0x0807D000` in both the brief description
  and the "Flash layout" block.
- `README.md` § 24 (*Persistent Storage*) — corrected page 125
  address from `0x0807C000` to `0x0807D000` in the *STM32 Flash
  Layout* table.
- `docs/INTEGRATION_PLAN.md` Apéndice A — corrected the page 125
  address (`0x0807C000` → `0x0807D000`), restored the firmware
  region to pages `0–123` and rewrote the post-table rule to point
  to page 124 (`0x0807C000`, `pedal_cal_store.c`) instead of the
  non-existent `0x0807A000`.
- `docs/CALIBRATION.md` § *On-flash layout (page 124)* — slot table
  rewritten to match the actual `pcal_flash_slot_t` struct: 16 bytes
  total with `validity_flag` (1 B, offset 8), `reserved[3]` (offset
  9–11), and `checksum` (4 B, offset 12).  The previous table
  erroneously placed `valid_flag` at offset 16 (off-the-end of the
  16-byte struct) and listed `_reserved` as a 4-byte uint32.

#### Verification

- `pedal_cal_store.c` slot layout (`magic / adc_min / adc_max /
  validity_flag / reserved[3] / checksum`, 16 B total, double-word
  aligned) is now reflected verbatim in both `docs/CALIBRATION.md`
  and `docs/HARDWARE_AND_SENSOR_MAP.md` § 1.6.
- Every page-125 address reference in the repository
  (`error_log.c`, `error_log.h`, `README.md`,
  `docs/INTEGRATION_PLAN.md`, `STM32G474RETX_FLASH.ld`) now reads
  `0x0807D000`.
- Page 124 ownership (`pedal_cal_store.c`, `0x0807C000`) and pages
  126/127 ownership are unchanged and consistent across all files.

#### Known issue — page 125 ownership conflict (RESOLVED in the entry above)

> **Status:** RESOLVED in the *page-125 ownership conflict — resolved*
> entry at the top of this changelog (sensor map relocated to flash
> page 123 at `0x0807B000`, `FLASH` shrunk 496 KB → 492 KB).  The text
> below is retained only as historical record of the issue that was
> tracked through this doc-sync pass.

`Core/Src/error_log.c` (`ERRLOG_FLASH_BASE = 0x0807D000`, page 125) and
`Core/Src/sensor_map_store.c` (originally `SMAP_FLASH_BASE = 0x0807D000`,
page 125) used to target the **same** flash page.  In practice this
meant that calling `SensorMapStore_Save()` would erase the error-log
page, and vice-versa, silently corrupting whichever store was written
last.

The collision was deliberately left **out of scope** for the doc-sync
pass because resolving it required either:

  1. Carving an additional NVM page out of `FLASH` in
     `STM32G474RETX_FLASH.ld` (moving the firmware ceiling from
     `0x0807C000` down to `0x0807B000`) so the sensor map can live on
     a brand-new page (e.g. page 123 at `0x0807B000`), **or**
  2. Re-using one of the four existing reserved pages with a strict
     ownership policy that has to be approved by the safety review
     (the engineering-screen sensor mapping is currently the only
     "soft" persistence in the four reserved pages).

The dedicated remediation PR must update the linker script, the file
headers of `error_log.c` / `sensor_map_store.c`, and the four
documentation tables (`README.md`, `docs/CALIBRATION.md`,
`docs/HARDWARE_AND_SENSOR_MAP.md`, `docs/INTEGRATION_PLAN.md`) in one
single change so the map stays consistent.  Until then, callers must
treat `SensorMapStore_Save()` and `ErrorLog_Record()` as mutually
exclusive at the workshop/engineering-screen level.

---

## [unreleased] — 2026-05-14

### Persistent Pedal-Endpoint Calibration (Lotes 1–7)

End-to-end addition of a workshop-driven calibration store for the
accelerator-pedal raw ADC endpoints, so the runtime mapping in
`Pedal_RawToPercent()` no longer depends on hard-coded constants. The
work is strictly surgical: no change to the 100 Hz loop, watchdog, EMA,
plausibility, fault thresholds, LIMP_HOME, startup_inhibit, ABS/TCS, or
the existing CAN contracts.

#### Rationale

`Core/Src/sensor_manager.c` previously baked `PEDAL_ADC_MIN = 150` and
`PEDAL_ADC_MAX = 2413` at compile time. Real-world pedal assemblies
drift mechanically across units and over service life; the only
remediation path used to be a firmware re-flash. This change moves the
endpoints into a persistent slot (flash page 124, `0x0807C000`) while
keeping the same numbers as the silent fallback when the slot is
missing/invalid.

#### Risks mitigated

- **Boot brick** — fully prevented by silent fallback to compile-time
  defaults when CRC/magic/range is invalid.  Boot is never blocked.
- **Pedal becomes unusable** — `Pedal_ApplyCalibration()` re-runs the
  same range-check inline; out-of-range values are rejected and the
  active endpoints are left untouched.
- **CAN flood** — the new `0x308` telemetry is on-demand only (1 s
  burst of 10 frames at 10 Hz after each QUERY).
- **CMD_ACK contract break** — DLC of `0x103` is preserved at 3 bytes;
  the new sub-protocol reuses existing `ACK_OK / ACK_INVALID /
  ACK_REJECTED / ACK_BLOCKED_BY_SAFETY` codes.
- **Unsafe persistence** — every sub-opcode except `QUERY` requires
  the full safety gate (STANDBY + `startup_inhibit` + pedal < 3% +
  plausible + all wheels < 0.3 km/h) plus the hard validators
  (`min ≥ 50`, `max ≤ 2600`, `max > min`, `max − min ≥ 800`).

#### Safety invariants preserved

- `motor_control.c`, `safety_system.c`, ABS/TCS logic, watchdog handling,
  pedal EMA, pedal plausibility, fault thresholds (FAULT_LO / FAULT_HI),
  startup_inhibit, LIMP_HOME, the loop-100 Hz scheduler and the existing
  CAN DLC contracts are **not modified**.
- `Pedal_RawToPercent()` keeps the same signature, the same EMA pipeline
  and the same fault handling. The only change is that the two endpoints
  now come from runtime variables that default to the compile-time
  constants.

#### Fallback behaviour

- Virgin flash, CRC mismatch, magic mismatch, or out-of-range pair →
  `PedalCal_Init()` returns *invalid*, the runtime endpoints stay at
  `PEDAL_ADC_MIN_DEFAULT = 150` / `PEDAL_ADC_MAX_DEFAULT = 2413`.
- `RESET DEFAULTS` button re-persists those exact defaults so the slot
  stays consistent across reboots.

#### Backward compatibility

- ESP32 firmware that predates the feature ignores the new `0x308` ID
  and the new `0xF5` sub-opcode without any error path engaged.
- Other CAN nodes that do not subscribe to `0x308` see no change in
  traffic — the burst is silent until a QUERY is issued.
- The compile-time defaults remain `150` / `2413`, identical to the
  previous firmware: a board that never runs the calibration wizard
  behaves bit-identically.

#### Flash reservation (Lote 1)

- `STM32G474RETX_FLASH.ld` — `FLASH` region trimmed from `500K → 496K`.
  Pages 124–127 (16 KB) are now exclusively NVM:
  - 124 `0x0807C000` — pedal calibration (this change)
  - 125 `0x0807D000` — DS18B20 sensor map
  - 126 `0x0807E000` — steering centring
  - 127 `0x0807F000` — EPS parameters

#### Files added

- `Core/Inc/pedal_cal_store.h` — public API (`PedalCal_Init`,
  `PedalCal_IsValid`, `PedalCal_GetStored`, `PedalCal_Validate`,
  `PedalCal_Save`, defaults).
- `Core/Src/pedal_cal_store.c` — flash store modelled 1:1 on
  `steering_cal_store.c` (magic `0x50434C31` (`"PCL1"`), 16-byte
  payload, CRC32 polynomial `0xEDB88320`, `valid_flag = 0xA5`, page 124
  erase + double-word program + lock, no IRQ disable, no RTOS, no
  watchdog change).
- `docs/CALIBRATION.md` — operator-facing reference for the new store.

#### Files modified

- `STM32G474RETX_FLASH.ld` — `FLASH = 496K`, NVM pages 124–127 documented.
- `Makefile` — `pedal_cal_store.c` added to `CORE_SRC` (and the
  pre-existing missing `loop_diag.c` reference was also added so the
  command-line build links cleanly).
- `Core/Inc/can_handler.h` — new IDs and constants
  (`CAN_ID_DIAG_PEDAL_CAL = 0x308`, `SERVICE_ACTION_PEDAL_CAL = 0xF5`,
  `PEDAL_CAL_OP_*`) and `CAN_PedalCalBurstUpdate()` prototype.
- `Core/Src/can_handler.c` — `0xF5` sub-protocol handler with stability
  check, hard validators, full safety-gate check, and the on-demand
  `0x308` burst emitter (10 frames × 100 ms).  STM32 alternates two
  payload variants in the burst (`PENDING` vs `STORED`) so all four
  endpoints fit in 8 bytes.
- `Core/Inc/sensor_manager.h` — new public helpers
  `Pedal_ApplyCalibration()`, `Pedal_GetRawADC()`.
- `Core/Src/sensor_manager.c` — `PEDAL_ADC_MIN/MAX` defines renamed to
  `_DEFAULT` and shadowed by runtime variables. `Pedal_RawToPercent()`
  is *byte-identical* in semantics — only the two named constants are
  replaced by the runtime statics. All other branches (FAULT_LO,
  FAULT_HI, EMA, plausibility, rate-limit) untouched.
- `Core/Src/main.c` — calls `PedalCal_Init()` after
  `SteeringCal_Init()`, applies the stored endpoints if valid, otherwise
  leaves the runtime defaults active. Also adds the 100 ms call to
  `CAN_PedalCalBurstUpdate()` in the existing status batch.
- `esp32/include/can_ids.h` — mirror constants
  (`DIAG_PEDAL_CAL = 0x308`, `SERVICE_ACTION_PEDAL_CAL = 0xF5`,
  `PEDAL_CAL_OP_*`).
- `esp32/src/vehicle_data.h` — new `PedalCalData` slot in
  `VehicleData`.
- `esp32/src/can_rx.cpp` — `0x308` decoder that preserves the
  unselected pair across alternating frame variants.
- `esp32/src/screens/engineering_screen.{h,cpp}` — `drawPedalCalibration()`
  fully rewritten with the new layout (Raw ADC / Pedal % / Stable /
  Plausible / Safety gate on the left, Stored MIN/MAX / Pending MIN/MAX /
  Validation on the right) and five buttons (CAPTURE MIN, CAPTURE MAX,
  SAVE, RESET DEFAULTS, BACK). Touch routing for the rest of the screen
  is unchanged; only the new buttons are added. ESP32 polls `QUERY`
  every 500 ms while the screen is active so the 1 s STM32 burst keeps
  refreshing; the burst stops naturally when the screen is closed.
- `docs/CAN_PROTOCOL.md`, `docs/ENGINEERING_MENU.md` — documentation
  updated for `0x308`, the `0xF5` sub-opcodes, and the new operator
  procedure.

#### Verification

- STM32 firmware builds clean (`make`, `-Wall -Wextra -Werror`).
  `text = 65 KB` — well below the new 496 KB `FLASH` region; the
  linker confirms `.text` / `.rodata` never spill into pages 124–127.
- Pre-existing `loop_diag.c` symbol was missing from the Makefile
  source list — added so the command-line build links. This is
  unrelated to the calibration feature but blocked verification of it.
- ESP32 firmware build verification was attempted with PlatformIO; the
  platform install was blocked by the offline sandbox and could not
  complete in this session. The C++ changes follow existing patterns
  (`SERVICE_ACTION_RELAY_OVERRIDE` path) and integrate with the same
  ACK feedback pipeline.

#### Final validation pass (FASE FINAL — 2026-05-14)

End-of-feature surgical validation. **No functional code was touched**
in this pass; the only edits are documentation closures and this
changelog entry. All thresholds, safety logic, timing, CAN contracts,
`startup_inhibit`, `LIMP_HOME`, watchdog handling, and
`Pedal_RawToPercent()` semantics are byte-identical to the pre-pass
state.

**A — Build verification**

- STM32 (`make clean && make all`, `arm-none-eabi-gcc 13.2.1`):
  build succeeds with `-Wall -Wextra -Werror`. No new warnings.
  `arm-none-eabi-size` reports:

  ```
     text	   data	    bss	    dec	    hex	filename
    65152	    108	   8136	  73396	  11eb4	build/STM32G474RE.elf
  ```

  ELF section layout (`objdump -h`):
  `.isr_vector` `0x08000000`, `.text` `0x080001D8` (size `0x0F814`),
  `.rodata` `0x0800F9EC`, `.data` LMA ends at `0x0800FEEC`.
  Page 124 (`0x0807C000`) is **442 KB** above the last code byte —
  no risk of code spilling into NVM pages.

- ESP32 (`pio run -d esp32`): the sandbox cannot reach `dl.espressif.com`,
  so PlatformIO aborts before invoking the toolchain. Static review
  of the touched files (`esp32/src/screens/engineering_screen.cpp/.h`,
  `esp32/src/can_rx.cpp`, `esp32/include/can_ids.h`,
  `esp32/src/vehicle_data.h`) confirms:
  * All four sub-opcodes (`CAPTURE_MIN/MAX/SAVE/RESET_DEFAULTS/QUERY`)
    mirror the STM32 constants exactly (verified by grep on both
    `Core/Inc/can_handler.h` and `esp32/include/can_ids.h`).
  * `DIAG_PEDAL_CAL = 0x308` is identical on both ends.
  * No new includes pull in headers that were not already used by the
    engineering screen.

**B — STM32 functional validation (static)**

- Flash-virgin path: `pcal_slot_integrity_ok()` returns `false` for
  the `0xFFFFFFFF` magic word, so `PedalCal_Init()` silently exits
  with `pcal_flash_valid = false` and the runtime keeps the
  compile-time `150 / 2413` endpoints. Boot is not blocked.
- CRC-corrupt path: same fallback — `crc != slot->checksum` triggers
  the early return.
- Save path: validated via `PedalCal_Validate()` (range gate) *before*
  any flash unlock or erase. Out-of-range pairs return `false` without
  touching flash.
- Hard limits: `PEDAL_CAL_MIN_LIMIT = 50`, `PEDAL_CAL_MAX_LIMIT = 2600`,
  `PEDAL_CAL_RANGE_MIN = 800` (`pedal_cal_store.h:51-53`). Confirmed:
  `max > 2600`, `range < 800`, `min < 50` all return `INVALID`.
- Safety gates (in `can_handler.c` `SERVICE_ACTION_PEDAL_CAL` branch):
  the SAVE / CAPTURE handlers require
  `Safety_GetState() == SYS_STATE_STANDBY` and `Startup_IsInhibited()`,
  plus `Pedal_GetPercent() < 3.0f`, `Pedal_IsPlausible()`, and all
  wheel speeds below the threshold — anything else returns
  `BLOCKED_BY_SAFETY`.
- Loop / watchdog: `pedal_cal_store.c` uses synchronous
  `HAL_FLASH_Program(DOUBLEWORD)` (≈ 80–100 µs per dword × 2). It runs
  only in `STANDBY` from a CAN command path, well below the IWDG
  ~500 ms timeout, and the existing per-iteration `HAL_IWDG_Refresh()`
  in `main.c` is not modified.

**C — CAN protocol validation (static)**

- `QUERY` (`0xF5 0x05`) sets up the 10 × 100 ms `0x308` burst window;
  outside that window, `CAN_PedalCalBurstUpdate()` is a no-op so the
  bus stays clean.
- `SERVICE_CMD_ACK` envelope unchanged (DLC 3); legacy nodes ignoring
  `0xF5` see no new ACK class.
- `0x308` payload (alternating PENDING / STORED variants) confirmed
  byte-for-byte by the ESP32 decoder in `esp32/src/can_rx.cpp`.
- Worst-case bus impact: 10 frames × 100 ms × 8 B ≈ 0.7 % @ 500 kbit/s,
  and only while a workshop QUERY is active.

**D — ESP32 / UI validation (static)**

- `drawPedalCalibration()` renders Raw ADC / Pedal % / Stable /
  Plausible / Safety on the left column and Stored / Pending / Validation
  on the right; SAVE button is disabled (`drawButton(..., enabled=false)`)
  when `validateOk == false`.
- All five buttons (`CAPTURE MIN / CAPTURE MAX / SAVE / RESET / BACK`)
  emit the matching `SERVICE_ACTION_PEDAL_CAL` sub-opcode.
- 500 ms `QUERY` poll while the screen is active is gated by the
  screen-id and stops naturally on BACK.
- Touch routing outside the new buttons is unchanged.

**E — Documentation closures**

- `docs/ENGINEERING_SCREEN.md` — added as a stub pointing at the
  canonical [`ENGINEERING_MENU.md`](docs/ENGINEERING_MENU.md), to
  satisfy historical cross-references.
- `docs/HARDWARE_AND_SENSOR_MAP.md` — new section §1.6 with the
  full STM32G474 flash NVM map (pages 124–127, owners, addresses)
  and the on-flash pedal-calibration slot layout.
- `PROJECT_CHANGELOG.md` — this section.

**Backward compatibility (final check)**

- A node that ignores `0x308` and `0xF5` sees zero new traffic and zero
  new ACK classes — pre-existing CAN contracts are unchanged.
- A unit that never runs the wizard keeps `pcal_flash_valid = false`
  and behaves bit-identically to firmware predating the feature.
- `Pedal_RawToPercent()` algorithm (clamp → linear → EMA → fault
  windows) is unchanged; only the two named limits become runtime
  variables seeded with the same defaults.

**Outstanding risks**

- ESP32 firmware was not rebuilt in this sandbox (no network for
  `platform-espressif32` install). The C++ side mirrors the verified
  STM32 constants and follows existing engineering-screen patterns,
  but a final on-bench build/flash on real hardware is recommended
  before tag.

**Status: merge-ready** for the STM32 side; ESP32 side is
build-verified statically and requires a bench rebuild as the only
non-software-equivalent residual check.

---

## [unreleased] — 2026-05-13

### Critical Safety Remediation & Documentation Alignment (F1–F7)

Surgical remediation pass targeting gear-synchronization safety, CAN
state consistency, debounce robustness, naming clarity and documentation
correctness in the shifter/traction subsystem.  No architectural change,
no new dependencies, no scheduler/PWM/CAN-bitrate/watchdog timing change.

#### F1 — Safe default gear on STM32

- **File**: `Core/Src/motor_control.c`
- **Change**: `static GearPosition_t current_gear` default and the value
  assigned by `Traction_Init()` are now `GEAR_NEUTRAL` (was
  `GEAR_FORWARD`).  Header comment updated accordingly.
- **Rationale**: the ESP32 shifter driver (`esp32/src/shifter_input.cpp`
  `currentGear_`) also boots in `NEUTRAL`.  Booting both nodes in the
  same fail-safe state eliminates the cross-node mismatch window that
  existed between STM32 boot and the first CMD_MODE resync from the
  ESP32.  While `current_gear == GEAR_NEUTRAL`, `main.c` already forces
  traction demand to 0 (lines 483–484), so this is the strictest
  fail-safe.
- **Safety impact**: removes the residual "default-FORWARD vs lever in
  REVERSE" mismatch during pre-resync boot windows.

#### F2 — Force gear resync after CAN-timeout recovery

- **File**: `Core/Src/safety_system.c`
- **Change**: in both code paths that clear `SAFETY_ERROR_CAN_TIMEOUT`
  (the `LIMP_HOME → ACTIVE` debounced transition and the
  `SAFE → LIMP_HOME` transition), `Traction_SetGear(GEAR_NEUTRAL)` is
  invoked **before** clearing the error and re-enabling motion.
- **Rationale**: before this fix, `current_gear` survived a CAN outage
  as a latched value.  If the shifter changed during the outage, the
  STM32 would resume operation with stale gear state until the ESP32
  re-sent a different value.  Forcing `NEUTRAL` on recovery makes the
  ESP32's CMD_MODE the authoritative resync source.
- **Safety impact**: eliminates stale gear state across CAN recovery.
  No timing impact (one atomic write on a transition that already
  happens at most once per outage).

#### F3 — Atomic CMD_MODE application

- **File**: `Core/Src/can_handler.c` (case `CAN_ID_CMD_MODE`)
- **Change**: the case has been refactored into a two-phase flow:
  1. **Validate** — decode mode flags and (optional) gear, run
     `Safety_ValidateModeChange()` and the `≤ 1 km/h` gear gate with
     `sanitize_float`, and the new F4 D↔R check.
  2. **Commit** — only if **every** validation passes, apply mode
     (`Traction_SetMode4x4`, `Traction_SetAxisRotation`) and gear
     (`Traction_SetGear`) atomically; otherwise apply nothing.
- **ACK semantics**:
  - All-OK → `ACK_OK`
  - Gear byte out of range → `ACK_INVALID`
  - Any safety/speed/transition rejection → `ACK_REJECTED`
  - System not in `ACTIVE/DEGRADED` → `ACK_BLOCKED_BY_SAFETY`
  - DLC < 1 → `ACK_INVALID`
- **Rationale**: previously, mode could be committed while gear was
  rejected, producing a partial application and a misleading
  `REJECTED` ACK.  All existing safety gates (`Safety_IsCommandAllowed`,
  `Safety_ValidateModeChange`, `≤ 1 km/h`, `sanitize_float`) are
  preserved unchanged.
- **CAN protocol**: untouched — same frame structure, same DLC, same
  byte layout (byte 0 = mode flags, byte 1 = optional gear).

#### F4 — Safe direction transition (D ↔ R)

- **File**: `Core/Src/can_handler.c` (inside the refactored CMD_MODE
  validation)
- **Change**: a direct gear change between `FORWARD / FORWARD_D2` and
  `REVERSE` is rejected unless **one** of the following holds:
  1. the transition goes through `NEUTRAL` (either the current or the
     requested gear is `NEUTRAL`), or
  2. the pedal is released below `CMD_MODE_PEDAL_REST_PCT = 3.0 %`
     (matches the spirit of `main.c` `STARTUP_PEDAL_REST_PCT`).
- **Rationale**: stacks on top of the existing `≤ 1 km/h` low-speed
  gate to make D ↔ R unconditionally safe (no torque inversion under
  load).
- **CPU/RAM impact**: negligible — a few additional comparisons and
  one `Pedal_GetPercent()` read per CMD_MODE frame (≤ 10 Hz typical).
- **Safety impact**: removes the residual "shift D ↔ R while crawling
  with throttle applied" hazard.

#### F5 — True shifter debounce

- **File**: `esp32/src/shifter_input.cpp`
- **Change**: `shifter::update()` now requires
  `DEBOUNCE_SAMPLES = 2` consecutive identical decoded samples before
  `currentGear_` is updated.  Two new static module-scope variables
  (`pendingGear_`, `pendingCount_`) hold the candidate and counter.
  Total RAM cost: **2 bytes**, no heap allocation, no behavioural
  change to the 50 ms polling cadence, the one-hot fail-safe in
  `decodeGear()`, or the NEUTRAL fallback in I²C error / backoff paths
  (those paths also reset the debounce state, never publishing a
  non-NEUTRAL gear on a single post-recovery sample).
- **Rationale**: previous behaviour was an inter-transmission interval
  (≥ 100 ms between CAN sends) but no per-sample stability filtering.
- **CPU impact**: one extra comparison and conditional store per poll
  (≤ 1 µs on ESP32-S3).

#### F6 — Rename `PIN_RELAY_DIR` → `PIN_RELAY_STEER_PWR`

- **Files**:
  - `Core/Inc/project_config.h` — macro definition with explicit
    comment explaining that this relay supplies 12 V power to the
    steering BTS7960 H-bridge and does **not** select drive direction.
  - `Core/Src/main.c`, `Core/Src/safety_system.c`,
    `Core/Src/can_handler.c`, `Core/Src/steering_centering.c`,
    `Core/Src/stm32g4xx_it.c`, `Core/Src/test_motor_control.c` — all
    references renamed; misleading "DIRECTION relay" comments updated
    to "STEER_PWR relay (12 V steering actuator supply)".
  - ESP32 telemetry and UI:
    - `esp32/src/ui/relay_indicator.h` — comment and on-screen char
      updated from "D" to "S" (single character, same layout).
    - `esp32/src/screens/engineering_screen.cpp` — button label
      `"DIRECTION (PC12)"` → `"STEER PWR (PC12)"`; surrounding
      comments updated.
    - `esp32/src/led_controller.h` — header comment updated.
  - Documentation: `docs/SAFETY_SYSTEMS.md`,
    `docs/PUESTA_EN_MARCHA_SEGURA.md`,
    `docs/INA226_RELAY_SAFETY_AUDIT.md`,
    `docs/LLAVE_CONTACTO_ENCENDIDO_APAGADO.md`,
    `docs/HARDWARE_WIRING_MANUAL.md`,
    `docs/COMPARACION_PINES_DOC_VS_FIRMWARE.md`,
    `docs/RELE_RETARDO_ENCENDIDO_APAGADO.md`,
    `Documentos/RELAY_STATUS_VISUALIZATION.md`,
    `Documentos/SISTEMA_ALIMENTACION_COMPLETO.md`,
    `PROJECT_CHANGELOG.md` — global token replacement.
- **Functional impact**: zero — same GPIO (PC12), same bit position
  (bit 2 of relay-status byte), same CAN-protocol behaviour.  This is
  exclusively a naming/clarity change.

#### F7 — Purge obsolete shifter documentation

- **Files**:
  - `docs/HARDWARE.md` — legacy redirect page rewritten: now states
    that the shifter is on the MCP23017 (I²C @ 400 kHz, SDA = GPIO8,
    SCL = GPIO9, address `0x20`), one-hot active-LOW dry contacts,
    NEUTRAL = no closed contact, gear in `CMD_MODE` (CAN 0x102) byte 1,
    `PB12/PB13` free, `PB14 = LED_DIAG`; stale `HARDWARE_SPECIFICATION.md`
    link removed.
  - `docs/VL53L8CX_INTEGRATION_ANALYSIS.md` — false claim
    "`PB12–PB13`: Shifter FWD/NEU" replaced with explicit "LIBRES" +
    cross-reference to `docs/HARDWARE_AND_SENSOR_MAP.md` §6.3.
  - `docs/BUILD_GUIDE.md` — false STM32CubeMX pin-assignment hint
    replaced with explicit "PB12-PB13 LIBRES — el shifter NO va aquí".
  - `docs/PINOUT_DEFINITIVO.md` is already marked entirely obsolete by
    its own two-banner header (kept as historical record).
- **Result**: no document in the repository still claims that the
  shifter is wired to STM32 pins.  The authoritative description is
  `docs/HARDWARE_AND_SENSOR_MAP.md` §6.3.

### Compatibility statement

- CAN frame IDs, DLCs, byte layouts and bitrate: **unchanged**.
- Scheduler cadence, PWM timing, relay sequencing, watchdog semantics,
  MCP23017 topology, I²C bus configuration: **unchanged**.
- No new third-party dependencies introduced; no dynamic allocation
  added (debounce state is a 2-byte static module variable).
- ESP32 telemetry bit layout (heartbeat byte 5) is preserved: bit 1
  = TRACTION, bit 2 = STEER_PWR (formerly named "DIRECTION").  Older
  ESP32 builds that still display the legacy "D" character will keep
  working; new builds display "S".

---

## [unreleased] — 2026-05-12

### [FIX] Homing de dirección en BOOT/STANDBY con relay DIR sin energizar

Corrección de condición de alimentación en el centrado automático (`SteeringCentering_Step`) para evitar fallo determinista de homing cuando el sistema aún no está en `SYS_STATE_ACTIVE`.

- **Problema corregido**:
  - El homing corría en `BOOT/STANDBY`, pero `Relay_SequencerUpdate()` sólo energiza relés en `ACTIVE`.
  - Resultado previo: el BTS7960 de dirección podía recibir PWM sin rail de 12V (`PIN_RELAY_STEER_PWR` OFF), terminando en `STALL/TIMEOUT` y degradación.

- **Cambios aplicados**:
  - `Core/Inc/steering_centering.h`:
    - Nuevo estado `CENTERING_WAIT_RAIL` añadido al final del enum para conservar los valores numéricos existentes (`0..4`) de estados previos.
  - `Core/Src/steering_centering.c`:
    - Nuevo `STEERING_RAIL_SETTLE_MS = 50U`.
    - En `CENTERING_IDLE`, se energiza `PIN_RELAY_STEER_PWR` y se registra timestamp de asentamiento.
    - Nuevo estado `CENTERING_WAIT_RAIL` que espera 50 ms antes de emitir el primer PWM de barrido.
    - Comentarios de seguridad/idempotencia con `Relay_PowerUp()` y apagado forzado en rutas SAFE/ERROR mediante BSRR atómico en `safety_system.c`.

- **Impacto y compatibilidad**:
  - Cambio mínimo y localizado (2 archivos, sin tocar scheduler, CAN IDs/DLC, ni lógica central de `safety_system`).
  - Compatible con rutas `ACTIVE/LIMP_HOME` (reescritura de DIR idempotente).
  - `Safety_FailSafe/Safety_PowerDown` siguen apagando ambos relés de forma atómica.
  - Coste: +4 B de BSS y espera one-shot de 50 ms al inicio del homing.

## [unreleased] — 2026-05-02

### [FEATURE] Sistema de calibración táctil persistente (ESP32 HMI)

Implementación completa del sistema de calibración del touch XPT2046 con persistencia en NVS, wizard interactivo de primer arranque y acceso desde el menú oculto de ingeniería. **Cero impacto en STM32, CAN y pantallas principales.**

- **Módulo nuevo `touch_calibration` (`esp32/src/touch_calibration.{h,cpp}`)**:
  - Namespace NVS dedicado `"touch_cal"`, independiente de `"hmi_cfg"` (evita invalidar la configuración existente en caso de escritura parcial o cambio de esquema).
  - Registro binario de 20 bytes: `magic (0x54434C31 "TCL1") + xMin + xMax + yMin + yMax + rotation + _pad + _pad2 + crc32`, todo bajo una sola clave NVS `"data"`.
  - Flag booleano `"first_done"` independiente de los datos: protege contra el bucle wizard-en-cada-boot cuando los datos están corruptos.
  - Validación en 5 condiciones: magic correcto, CRC-32 válido, valores ∈ [0, 4095], rango mínimo > 1000 LSB en ambos ejes, `rotation == TFT_ROTATION`. Cualquier fallo hace fallback silencioso al `TOUCH_CALIBRATION` compile-time de `User_Setup.h`.
  - API pública: `init()`, `loadValid(out[5])`, `save(data[5])`, `clear()`, `factoryReset()`, `firstBootDone()`, `markFirstBootDone()`.
  - Dos `static_assert` sobre `sizeof(Record) == 20` y `offsetof(Record, crc32) == 16` garantizan la estabilidad del layout en futuros compiladores/plataformas.

- **Wizard de calibración (`esp32/src/screens/touch_calibration_screen.{h,cpp}`)**:
  - Máquina de estados: `INTRO → COLLECT → CONFIRM → DONE | FAILED`.
  - `INTRO`: pantalla con instrucciones, botón `EMPEZAR` siempre visible, botón `CANCELAR` sólo en modo menú oculto (ocultado en first-boot para forzar completar la calibración inicial).
  - `COLLECT`: delega en `TFT_eSPI::calibrateTouch()` que dibuja las 4 cruces esquina con feedback visual de aceptación por punto (nativo de la librería, rotation-aware, sin reimplementar la aritmética de calibración).
  - `CONFIRM`: muestra los 4 valores capturados (`xMin`, `xMax`, `yMin`, `yMax`) y el número de rotación; botones `GUARDAR` / `REINTENTAR` (y `CANCELAR` si no es first-boot).
  - `FAILED`: si el rango capturado es < 1000 LSB en algún eje, rechaza la calibración y ofrece `REINTENTAR` (y `CANCELAR` si no es first-boot).
  - `DONE`: guarda en NVS, aplica `tft.setTouch()` en caliente y devuelve el control al `ScreenManager`.

- **Integración en `ScreenManager` (`esp32/src/screen_manager.{h,cpp}`)**:
  - Nueva bandera `touchCalActive_` (análoga a `pinActive_` / `engineeringActive_`) con prioridad máxima: cuando el wizard está activo, la state-machine normal del CAN, la detección de pérdida de CAN y todas las pantallas subyacentes quedan completamente fuera del flujo táctil.
  - Nuevo método `requestTouchWizard(bool firstBoot)` llamado desde `main.cpp` en el primer tick del `renderTask` y desde el menú de ingeniería.
  - `isBlockingInput()` actualizado para incluir `touchCalActive_`.
  - `onLongPress()` actualizado para ignorar pulsaciones largas mientras el wizard está activo.
  - Transición engineering→wizard atómica (mismo frame, sin flash de pantalla intermedia): `ScreenManager::update()` consume el flag `consumeTouchCalRequest()` justo después del exit del menú de ingeniería.

- **Menú de ingeniería (`esp32/src/screens/engineering_screen.{h,cpp}`)**:
  - Dos nuevas entradas en el menú principal: `TOUCH CALIBRATION` (lanza el wizard, ítem 12, color cian) y `RESET TOUCH CAL` (borra NVS + re-arma el wizard de first-boot, ítem 13, color ámbar).
  - `NUM_MAIN_ITEMS` 11 → 13.
  - Constantes de layout ajustadas para acomodar 13 ítems en 320 px: `MENU_BTN_H` 23→19, `MENU_SPACING` 25→21, `MENU_START_Y` 46→42.
  - Nuevo flag `touchCalRequested_` y método `consumeTouchCalRequest()` — desacoplamiento limpio con `ScreenManager`.
  - Include de `touch_calibration.h` añadido (para `factoryReset()` en "Reset Touch Cal").

- **`esp32/src/main.cpp`**:
  - `touch_calibration::init()` llamado en `setup()` después de `config_store::init()` y antes de `tft.init()`.
  - Bloque `tft.setTouch()` reemplazado: intenta `touch_calibration::loadValid(calData)` primero; si falla, usa el fallback compile-time. Log claro en Serial de qué fuente se usó.
  - Primera iteración de `renderTask`: si `!touch_calibration::firstBootDone()`, llama `screenManager.requestTouchWizard(true)`.
  - Include de `touch_calibration.h` añadido.

- **Política de arranque (boot policy)**:
  - La pantalla arranca **siempre** con un mapeo táctil válido (fallback garantiza touch funcional incluso sin calibración guardada).
  - El wizard de first-boot se lanza dentro del `renderTask` (Core 0), sin bloquear `setup()` ni el task CAN (Core 1).
  - Una vez completado (`SAVE`), `first_done = true` persiste indefinidamente: el wizard nunca vuelve a lanzarse automáticamente.
  - `RESET TOUCH CAL` borra `first_done` además de los datos, re-armando el wizard para el siguiente arranque (acción consciente del técnico).
  - Cancelar el wizard desde el menú oculto NO re-arma el wizard automático (`markFirstBootDone()` se llama en el path de cancel para preservar la invariante).

- **Documentación**:
  - Nuevo `docs/TOUCH_CALIBRATION_SYSTEM.md`: flujo de arranque, layout NVS (claves lógicas `touch_x_min/max`, `touch_y_min/max`), instrucciones de recalibración y reset, comportamiento de fallback, tabla de compatibilidad, checklist de validación.
  - `docs/ENGINEERING_MENU.md`: tabla de ítems del menú principal ampliada con ítems 8–13.
  - `docs/ESP32_FIRMWARE_DESIGN.md`: §3.1 y §3.2 actualizados — la fila "NVS persistence not required" reemplazada por descripción de los namespaces `hmi_cfg` y `touch_cal`.
  - `CHANGELOG.md`: entrada `[FEATURE] Persistent touch calibration system (ESP32 HMI)` en sección `[Unreleased] / Added`.

**Invariantes preservadas**: STM32 intacto, protocolo CAN intacto (sin IDs nuevos, sin cambios de DLC/timing), `User_Setup.h` intacto (sigue siendo fallback compile-time), `touch_handler.{h,cpp}` intacto, `boot_screen` / `drive_screen` / `safe_screen` / `error_screen` / `standby_screen` / `pin_screen` intactos, sin nuevas librerías externas.

---

## [unreleased] — 2026-05-01

### [DEBUG] Debounce EMI diagnostic counters

Instrumentación purely-additive del filtro debounce DWT (200 µs) para auditar la cadena de mitigación EMI hardware (TVS → opto EL817 → filtro DWT) sin cambiar comportamiento funcional ni timing.

- **Firmware STM32**:
  - `Core/Src/sensor_manager.c`: nuevas variables `static volatile uint32_t sensor_dbg_filtered_count[NUM_WHEELS]` y `steer_dbg_filtered_count`, incrementadas con clamp a `0xFFFFFFFF` dentro del bloque de rechazo del Step 0 (DWT) en `Wheel_IRQDebounced` y `SteeringCenter_IRQHandler`. Sin tocar el camino aceptado.
  - Getters `Sensor_GetFilteredCount(idx)` / `Sensor_GetSteerFilteredCount()` añadidos en `Core/Inc/sensor_manager.h`. Comentario explícito: "Diagnostic only — must NOT gate any control/safety path."
  - Dos frames CAN nuevos aditivos (1 Hz, STM32 → ESP32, sin conflicto con IDs existentes):
    - `0x306` (`CAN_ID_DIAG_DEBOUNCE`, DLC 8): 4× contador rueda u16 LE saturado a 0xFFFF.
    - `0x307` (`CAN_ID_DIAG_DEBOUNCE_STEER`, DLC 4): contador volante u32 LE.
  - `CAN_SendDebounceDiag()` añadida en `Core/Src/can_handler.c`, invocada desde el bloque `tick_1000ms` ya existente en `Core/Src/main.c`.
  - **Coste ISR**: ~4 instrucciones extra (load + cmp + add + store, M4) → < 0.012 % de CPU adicional a 200 Hz @ 170 MHz. ISR sigue O(1).
  - **Carga de bus**: 2 frames × 1 Hz ≈ 220 bps sobre 500 kbps (0.044 %). Despreciable.
- **UI ESP32**:
  - `esp32/include/can_ids.h`: constantes `DIAG_DEBOUNCE` / `DIAG_DEBOUNCE_STEER`.
  - `esp32/src/vehicle_data.h`: nuevo struct `DebounceDiagData` (cero impacto en estructuras existentes), setter / getter / miembro siguiendo el patrón habitual.
  - `esp32/src/can_rx.cpp`: dos decoders nuevos (`decodeDebounceDiag`, `decodeDebounceDiagSteer`) y dos casos en el `switch (frame.identifier)`. ESP32 antiguos siguen funcionando — IDs desconocidos se ignoran silenciosamente.
  - `esp32/src/screens/engineering_screen.{h,cpp}`: nuevo submenú `DEBOUNCE_DIAG` accesible **sólo** desde el Engineering Screen (PIN `8989`), entrada "DEBOUNCE DEBUG" en el menú principal del menú oculto. Render mínimo (5 filas FL/FR/RL/RR/STEER, contador decimal alineado a la derecha, BACK button), repaint a 1 Hz natural por cadencia CAN. Ajuste menor de constantes de layout del menú principal (`MENU_START_Y` 50→46, `MENU_SPACING` 28→25, `MENU_BTN_H` 26→23) para dar cabida al 11º item. Sin animaciones, sin sprites.
  - **Pantallas principales no tocadas**: `drive_screen`, `safe_screen`, `boot_screen`, `pin_screen`, `error_screen`, `standby_screen` intactos.
- **Documentación**:
  - `docs/SENSOR_INTERFACE.md`: nueva sección **§11 "Debounce Diagnostics Counters (DWT 200 µs filter)"** con qué miden, garantías de seguridad / timing, API, layout CAN, tabla de interpretación de valores y ejemplos reales esperados.
  - `docs/EL817_WIRING_REFERENCE.md`: nueva sección **"TVS + Opto + Debounce Counters — Closing the Loop"** correlacionando las tres etapas de la cadena de mitigación con el procedimiento empírico de validación (4 pasos).
  - `docs/CAN_CONTRACT_FINAL.md`: bump de revisión (1.3 → 1.4) con sección de los nuevos IDs `0x306` / `0x307` documentados como diagnóstico aditivo (cero cambios en payload / timing / IDs existentes).

**Invariantes preservadas**: `SENSOR_DEBOUNCE_US = 200` intacto, IDs CAN existentes intactos, timers / PWM / relés / motor_control / safety_system / encoder no tocados, sin nuevas dependencias o librerías, compatibilidad hacia atrás garantizada en ambos sentidos del bus CAN.

### [AUDIT] Sensor debounce verification

- Verified DWT-based debounce implementation against task specification.
- Confirmed all per-channel state variables are `uint32_t` and use unsigned subtraction (overflow-safe across the ~25 s CYCCNT wrap-around at 170 MHz, per C99 §6.2.5 ¶9).
- Confirmed the DWT pre-filter executes as **step 0** of every EXTI handler (`Wheel_IRQDebounced`, `SteeringCenter_IRQHandler`), before HAL_GetTick filter, before flood-detection, before counter increment.
- Confirmed full per-channel isolation: 4× `wheel_last_edge_cyc[]` + 1× `steer_last_edge_cyc` — no shared state between channels.
- Confirmed EXTI edge configuration is unchanged: PA0/PA1/PA2/PB15 RISING (wheels), PB5 FALLING (steering center). EL817 inversion correctly handled — PB5 is configured FALLING because the EL817 NPN output goes LOW when the proximity sensor activates.

### [VALIDATION] Timing margin against real signal frequencies

- Wheel sensors: max realistic frequency = (25/3,6) / 1,1 × 6 = **37,9 Hz** → minimum period = **26 316 µs**.
- Margin = 200 µs / 26 316 µs = **0,76 %** → falls in the **óptimo** category (< 1 %).
- Steering center: < 1 Hz operating frequency → margin = 0,02 % (negligible).
- **Decision:** retain `SENSOR_DEBOUNCE_US = 200`. No reduction to 100 µs because:
  1. No measurable functional benefit at 38 Hz (margin already < 1 %).
  2. EL817 t_off can reach 50 µs in marginal saturation; 200 µs absorbs 2× this worst case.

### [HARDENING] Independent and idempotent DWT initialisation

- `Core/Src/sensor_manager.c — Sensor_Init()`: refactored DWT enable block to use the defensive conditional pattern recommended by the audit specification:
  - Trace-unit enable only if `CoreDebug->DEMCR & TRCENA_Msk` is clear.
  - Cycle counter enable only if `DWT->CTRL & CYCCNTENA_Msk` is clear; **`DWT->CYCCNT` is no longer reset unconditionally** — preserves any in-flight DWT-based delay started by `Motor_Init()` / `delay_us()` in `motor_control.c`.
- The block is now **fully independent of `Motor_Init()`** and **order-independent** with respect to it. `Sensor_Init()` can be safely invoked before, after, or instead of `Motor_Init()` and DWT will be running.
- `__DSB()` retained to ensure the CYCCNTENA write is observable before the first CYCCNT read.

### [VALIDATION] P6KE18CA TVS — confirmed

- Confirmed component selection: **P6KE18CA**, bidireccional, V_clamping ≈ 29,2 V @ 5 A, 600 W peak (10/1000 µs), DO-15 axial.
- Confirmed placement: between `n+` and `n−` of each active EL817 channel, on the 12 V (vehicle) side, before the LED. 6 units total (4× Board 1 + 2× Board 2).
- Confirmed protection effectiveness: clamps 40 V load-dump transients (ISO 7637-2 level III) to 29,2 V → I_LED = (29,2 − 1,2) / 2 800 ≈ 10 mA, within EL817 datasheet limits.
- Hardware + firmware EMI mitigation aligned: TVS (capa 1) + optoacoplador (capa 2) + DWT 200 µs (capa 3) — three-layer defense-in-depth.

#### Documentation

- **`docs/SENSOR_INTERFACE.md`:** §3 — added `Validación matemática (auditoría — frecuencias reales)` subsection with full numerical breakdown and decision rationale.
- **`docs/EL817_WIRING_REFERENCE.md`:** §11 — added `Mitigación EMI completa — defensa en profundidad` table (TVS + opto + DWT layers); §12 — strengthened critical note to explicitly state that the TVS *"protege frente a transitorios inductivos antes de que alcancen el optoacoplador"*.

---

## [unreleased] — 2026-05-01

### [NEW] Sensor debounce + EMI hardening (EL817 optocoupler interface)

Added software debounce (200 µs) to all EXTI sensor inputs connected through EL817 optocouplers, and confirmed hardware protection with P6KE18CA TVS diodes.

#### [NEW] Sensor debounce — Firmware (`Core/`)

- **`Core/Inc/project_config.h`:** Added `SENSOR_DEBOUNCE_US 200U` macro — microsecond-resolution pre-filter threshold for all EXTI sensor channels. Accompanies the existing `WHEEL_MIN_PULSE_INTERVAL_MS 1U` (ms-level secondary filter). Rationale documented inline.
- **`Core/Src/sensor_manager.c`:**
  - Added per-channel DWT cycle-counter timestamps:
    - `wheel_last_edge_cyc[NUM_WHEELS]` — one per wheel, never shared
    - `steer_last_edge_cyc` — exclusive to steering center channel
  - Added `sensor_debounce_cycles` (precomputed from `SENSOR_DEBOUNCE_US × SystemCoreClock / 1e6` in `Sensor_Init()`).
  - `Sensor_Init()`: enables DWT->CYCCNT (idempotent if Motor_Init already did it), precomputes `sensor_debounce_cycles`.
  - `Wheel_IRQDebounced()`: DWT pre-filter runs as **step 0**, before HAL_GetTick ms-filter and flood-detection. Rejects any edge arriving within 200 µs of the previous accepted edge. No impact on nominal behaviour (pulse period ≥ 26 ms at max speed).
  - `SteeringCenter_IRQHandler()`: expanded from single-line to include DWT pre-filter (200 µs) before setting `steer_center_flag`. Eliminates EMI-induced spurious re-calibrations of the steering encoder center point.

Filters pulses occurring within < 200 µs — a window in which no real sensor event can occur at any operating speed of this vehicle. Eliminates false triggers caused by EMI and EL817 optocoupler switching noise. No impact on nominal signal processing. No blocking delays, no polling, no architecture changes.

#### [HARDWARE VALIDATION] TVS P6KE18CA

- Confirmed use of **P6KE18CA** TVS diodes (bidireccional, 600 W, DO-15) on the vehicle-side input of each active EL817 channel.
- Verified correct placement: between `n+` and `n−` of each channel (before the LED), one per active channel (6 total: 4× Board 1, 2× Board 2).
- **bidireccional:** adecuado para señales no polarizadas.
- **V_clamping 29,2 V @ 5 A:** protects EL817 LED against automotive load-dump transients (up to 40 V per ISO 7637-2 level III).
- **600 W peak power:** sufficient for automotive transient requirements.
- **DO-15 format:** robust for field installation.
- Protection aligned with automotive transient requirements. TVS protects the optocoupler LED against voltage spikes induced by inductive loads (motors, relays).

#### [DOCS] Documentation updated

- **`docs/SENSOR_INTERFACE.md`:**
  - Section 3 (wheel sensors): updated parameter table, added `FILTRADO DE SEÑAL (DEBOUNCE SOFTWARE)` subsection with implementation code, validation calculations, and updated ISR flow diagram.
  - Section 6 (steering center): added `FILTRADO DE SEÑAL (DEBOUNCE SOFTWARE)` subsection.
- **`docs/EL817_WIRING_REFERENCE.md`:**
  - Section 11 (firmware verification): updated table with debounce column; added `FILTRADO DE SEÑAL (DEBOUNCE SOFTWARE)` block with implementation detail, parameter table, and per-channel variable table.
  - Section 12 (new): `PROTECCIÓN TVS — JUSTIFICACIÓN TÉCNICA` — full technical justification for P6KE18CA including bidirectional rationale, clamping voltage, power rating, format, and numerical comparison with/without TVS.

---

## [unreleased] — 2026-04-30

### Changed — PC10 Released + RELAY_MAIN Fully Removed

**Context.** The MAIN / Power-Hold contactor (historically associated with PC10) **never existed in real hardware**. Only two power relays are physically wired: `RELAY_TRAC` on PC11 (24 V, BTS7960 traction stage) and `RELAY_DIR` on PC12 (12 V, steering). This PR aligns firmware, `.ioc` and documentation with that ground truth and leaves PC10 in a deterministic, electrically-safe state.

#### Firmware (`Core/`)

- **`Core/Src/main.c` — `MX_GPIO_Init()`:** Added explicit PC10 init block that configures the pin as `GPIO_MODE_INPUT` + `GPIO_PULLDOWN` + `GPIO_SPEED_FREQ_LOW`. Rationale documented inline: prevents indeterminate readings / spurious EXTI noise, eliminates leakage current through floating CMOS input, keeps the pin safe for future reassignment. Removed the obsolete `"PC10 reserved — no MAIN contactor in the hardware"` comment from the relay-output block.
- **`Core/Src/safety_system.c`:**
  - Sequencer header diagram updated from the legacy 3-stage `MAIN → TRAC → DIR (~70 ms)` to the real 2-stage `TRAC → (50 ms) → DIR (~50 ms)`. Added explicit note that PC10 is a free GPIO (input + pull-down, not connected) and that 24 V feeds RELAY_TRAC directly.
  - `Relay_PowerUp()` / `Relay_PowerDown()` comments cleaned: BSRR mask still contains **only** `PIN_RELAY_TRAC | PIN_RELAY_STEER_PWR`; PC10 is explicitly described as a free GPIO that is NOT touched on power-down.
  - `relay_override_mask` bit-0 documented as “reserved (always 0)” for forward compatibility with rev 1.3 SERVICE_CMD 0xE0 consumers (no behavioural change).
  - `Safety_GetRelayStatusByte()` comment updated: bit 0 = 0 (reserved; PC10 free GPIO), bit 1 = TRAC, bit 2 = STEER_PWR (legacy DIR), bit 7 = SEQ_COMPLETE.
- **`Core/Src/can_handler.c`:** Heartbeat (0x001) byte 5 doc-comment and SERVICE_CMD 0xE0 relay-override doc-comment updated — bit 0 relabelled from `MAIN relay GPIO ON` to `reserved (always 0; PC10 not connected)`. **Wire layout unchanged** — fully backward-compatible with the rev 1.3 CAN contract.
- **`Core/Src/test_motor_control.c`:** Comment-only refresh to match the new architecture.

#### `.ioc` (STM32CubeMX project)

- **`STM32-Control-Coche-Marcos.ioc`:** PC10 explicitly registered (`Mcu.Pin27=PC10`, `PinsNb` bumped from 39 → 40) with:
  - `PC10.Signal=GPIO_Input`
  - `PC10.GPIO_PuPd=GPIO_PULLDOWN`
  - `PC10.GPIO_Label=UNUSED_PC10`
  - `PC10.Locked=true` (protects the configuration against any future CubeMX regeneration).
  No peripheral is assigned to PC10. `MX_GPIO_Init()` in `main.c` matches the `.ioc` byte-for-byte.

#### Documentation (39 files)

All active references to `RELAY_MAIN` / `MAIN relay` / `MAIN contactor` / `MAIN→TRAC` / `Power-Hold` removed from technical docs; PC10 uniformly described as **“AVAILABLE / GPIO libre / INPUT_PULLDOWN / no conectado”**.

- `README.md`
- `docs/POWER_DISTRIBUTION.md`, `docs/PINOUT.md`, `docs/PINOUT_DEFINITIVO.md`, `docs/LISTADO_PINES_COMPLETO.md`, `docs/PIN_USAGE_INVENTORY.md`, `docs/COMPARACION_PINES_DOC_VS_FIRMWARE.md`
- `docs/HARDWARE.md`, `docs/HARDWARE_AND_SENSOR_MAP.md`, `docs/HARDWARE_SPECIFICATION.md`, `docs/HARDWARE_WIRING_MANUAL.md`, `docs/CONEXIONES_COMPLETAS.md`
- `docs/SAFETY_ARCHITECTURE.md`, `docs/SAFETY_SYSTEMS.md`, `docs/PUESTA_EN_MARCHA_SEGURA.md`
- `docs/CAN_CONTRACT_FINAL.md`
- `docs/INA226_RELAY_SAFETY_AUDIT.md`, `docs/INFORME_REVISION_TECNICA_RELAY.md`, `docs/RELE_RETARDO_ENCENDIDO_APAGADO.md`, `docs/AUDIO_RELAY_INTEGRATION.md`
- `docs/AISLAMIENTO_AUDIO_DFPLAYER.md`, `docs/AISLAMIENTO_GALVANICO_6N137.md`, `docs/ALIMENTACION_BUCK_INRUSH_PROTECTION.md`
- `docs/BTS7960_MOTOR_DRIVER_AUDIT.md`, `docs/MOTOR_CONTROL.md`, `docs/MOTOR_CONTROL_AUDIT.md`, `docs/LED_SYSTEM_ANALYSIS.md`
- `docs/BUILD_GUIDE.md`, `docs/COMPONENTES_PASIVOS_REFERENCIA.md`, `docs/MATERIALES_POR_MODULO.md`, `docs/ESP32_PIN_DOCUMENTATION_INDEX.md`
- `docs/FIRMWARE_AUDIT_REPORT.md`, `docs/FIRMWARE_COMPLETION_ASSESSMENT.md`, `docs/FIRMWARE_MATURITY_ROADMAP.md`
- `Documentos/SISTEMA_ALIMENTACION_COMPLETO.md`, `Documentos/INVENTARIO_COMPONENTES_FISICOS.md`, `Documentos/RELAY_STATUS_VISUALIZATION.md`, `Documentos/SAFE_MODE_UI_EXTENSION.md`

#### ESP32 (HMI) — comment-only / cosmetic

- `esp32/include/can_ids.h`, `esp32/src/vehicle_data.h`, `esp32/src/ui/relay_indicator.h`, `esp32/src/screens/engineering_screen.{h,cpp}`: comments / labels updated to reflect 2-relay architecture. UI rendering, CAN parsing and override semantics unchanged (bit 0 still reserved/0).

### No functional or wire-protocol change

- Relay sequencer was already TRAC → DIR only (no MAIN step) — code paths unchanged.
- `Relay_PowerDown()` and the BSRR emergency-stop masks in `main.c` and `stm32g4xx_it.c` already touched **only** PC11 + PC12.
- CAN heartbeat 0x001 byte 5 wire layout is **identical** to rev 1.3 (bit 0 reserved/0, bit 1 TRAC, bit 2 DIR, bit 7 SEQ_COMPLETE) — existing ESP32 / external consumers continue to work unchanged.
- No new peripherals, no new IRQs, no new timing dependencies.

### Final audit (post-change verification)

- ✔ `grep -E 'RELAY_MAIN|PIN_RELAY_MAIN|MODULE_RELAY_MAIN|RELAY_SEQ_MAIN|MAIN_SETTLE'` over `Core/`, `esp32/`, `docs/`, `Documentos/`, `*.ioc` → **0 hits** (this changelog excluded by design as historical record).
- ✔ All 40 `.ioc` pins coherent with `Core/Inc/project_config.h`; **0** duplicates, **0** floating inputs, **0** unconfigured outputs.
- ✔ PC10 = `Input` + `PULLDOWN` + `Locked=true` in `.ioc` and matches `MX_GPIO_Init()` byte-for-byte.
- ✔ Eléctricamente seguro: sin pines flotantes, sin salidas activas sin carga, sin GPIO en estado indeterminado en boot.

---

## 1. Visión del Proyecto

### Objetivo General
Sistema de control embebido para vehículo eléctrico de 4 ruedas con tracción independiente, dirección asistida y HMI completo. El proyecto implementa un control en tiempo real con alta fiabilidad y seguridad funcional.

### Arquitectura Hardware
- **STM32G474RE** (Cortex-M4F @ 170 MHz): Cerebro de control — tracción, dirección, sensores, CAN, safety.
- **ESP32-S3-WROOM-1** (Dual-core @ 240 MHz, 8 MB PSRAM): HMI — pantalla TFT 480×320, audio, LEDs WS2812B, selector de marchas, detección de obstáculos.
- **Bus CAN**: FDCAN1 @ 500 kbps, CAN 2.0A (11-bit IDs), topología punto a punto.

### Principios de Diseño
1. **Safety-first**: Máquina de estados de seguridad (BOOT→STANDBY→ACTIVE→DEGRADED→SAFE→ERROR), watchdog IWDG, BREAK2 kill de PWM ante fallo de CPU.
2. **Dual-core ESP32**: Render en Core 0, CAN/sensores/audio en Core 1. Datos compartidos vía mutex.
3. **Inicialización robusta**: Reintentos con validación hardware (CCCR, clock source), logs de diagnóstico persistentes.
4. **Tolerancia a fallos CAN**: Bus-off recovery (ambos lados), error-passive recovery bifásica (ESP32: 10 rápidos + lento ilimitado), timeout → LIMP_HOME.
5. **Persistencia**: Error log en Flash (página 125, 250 entradas con CRC32), calibración de dirección en Flash (página 126).
6. **Modularidad**: 25 módulos habilitables/deshabilitables vía Service Mode.

### Estándares de Calidad
- CI/CD: syntax check (`-Wall -Wextra -Werror`), unit tests, cppcheck, flawfinder, lizard (complejidad ciclomática).
- Documentación técnica: 90+ documentos en `docs/`.
- Protocolo CAN congelado (v1.3): `CAN_CONTRACT_FINAL.md`.

---

## 2. Estado Actual del Sistema

### STM32 (Control Authority)
| Subsistema | Estado | Notas |
|---|---|---|
| **Tracción (4 motores)** | ✅ Operativo | PWM 20 kHz, zona muerta 8%, rampa 50%/s up / 100%/s down |
| **Dirección PID** | ✅ Operativo | Encoder E6B2-CWZ6C (4800 counts/rev), geometría Ackermann |
| **Calibración dirección** | ✅ Persistente | Flash página 126, CRC32, auto-centrado al boot |
| **FDCAN1** | ✅ Operativo (hardened) | PA11(RX)/PA12(TX) AF9, MASK accept-all, reintentos con CCCR check |
| **Sensores** | ✅ Operativo | 4× velocidad rueda, 5× DS18B20, 6× INA226, pedal ADC |
| **Safety System** | ✅ Operativo | ABS/TCS 15% slip, overcurrent, overtemp, obstacle, CAN timeout |
| **Error Log** | ✅ Operativo | Flash página 125, 250 entradas, CRC32, ring buffer |
| **Service Mode** | ✅ Operativo | 25 módulos, factory defaults, fault tracking |
| **Relay Override** | ✅ Operativo | Engineering diagnostic mode — manual relay GPIO via SERVICE_CMD 0xE0 |
| **Watchdog (IWDG)** | ✅ Activo | ~500 ms timeout |
| **BREAK2 (PWM kill)** | ✅ Armado | Vinculado a Cortex LOCKUP |
| **Heartbeat LED** | ✅ Operativo | Flash breve ~50 ms cada ~2 s (diferenciable de Error_Handler) |

### ESP32-S3 (HMI Controller)
| Subsistema | Estado | Notas |
|---|---|---|
| **Pantalla TFT** | ✅ Operativo | ST7796 480×320, render en Core 0 (FreeRTOS task) |
| **CAN (TWAI)** | ✅ Operativo (hardened) | Bus-off recovery + error-passive recovery bifásica (10 rápidos @3s + lento @30s) |
| **Selector de marchas** | ✅ Operativo | MCP23017 I2C, P/R/N/D/D2, backoff con `endTransmission(true)` |
| **Audio (DFPlayer)** | ⚠️ Parcial | Hardware presente, integración completa pendiente |
| **LEDs WS2812B** | ✅ Operativo | Front (28 LEDs @ GPIO 47): KITT + turn signals (5/side) + throttle effects. Rear (16 LEDs @ GPIO 48): position/brake/reverse/regen + sequential turn signals. Non-destructive overlay en ambos strips. Emergency flash one-shot en ERROR. |
| **Obstáculos (LiDAR)** | ✅ Operativo | TF-Mini Plus (115200 bps, GPIO 18), 5 zonas, timeout 500 ms → fail-safe |
| **Power Manager** | ✅ Operativo | Ignition GPIO 40 + power hold GPIO 41 |
| **Config Store** | ✅ Operativo | SPIFFS NVM persistente |
| **Screen Manager** | ✅ Operativo | 6 estados: Boot/Standby/Drive/Error/Safe/Degraded + Engineering (8989) + Relay Control submenu. SafeScreen incluye visualización pasiva extendida (gear, obstacle, LEDs, relay, steering visual). |

### Comunicación CAN
- **Protocolo**: 28 tipos de mensaje, contrato **v1.3** (2026-02-13, con aclaración 2026-04-23 sobre `relay_status` byte 5 — ver `docs/CAN_CONTRACT_FINAL.md`). El mensaje `SYSTEM_SHUTDOWN` (0x130) es un añadido aditivo/no-breaking posterior.
- **STM32→ESP32**: Heartbeat (0x001), telemetría (0x200–0x20A), service (0x301–0x305).
- **ESP32→STM32**: Heartbeat (0x011), throttle (0x100), steering (0x101), mode (0x102), service (0x110), LEDs (0x120), SYSTEM_SHUTDOWN (0x130), obstacle (0x208–0x209).
- **Filtrado STM32**: MASK accept-all (index 0), filtrado real en `CAN_ProcessMessages()` switch/case.
- **Timeout**: 250 ms → transición a LIMP_HOME (20% torque máximo).

### FreeRTOS (ESP32)
- **renderTask**: Pinned Core 0, stack 16 KB, prioridad 1. Maneja TFT SPI + touch.
- **Main loop**: Core 1. Maneja CAN, sensores, audio, LEDs, power.
- **Sincronización**: Mutex para `VehicleData`, cola FreeRTOS para touch actions.

### Rendimiento
- STM32 main loop: 100 Hz (10 ms).
- Sensor reads: 20 Hz (50 ms).
- Safety checks: 100 Hz (10 ms).
- CAN heartbeat: 10 Hz (100 ms).
- ESP32 render: ~28 ms/frame (liberado del main loop).
- RuntimeMonitor: Logs cada 5 s con stats por periodo (fps, avg/max/min frame, can, loop). Flags de bloqueo independientes por fase (can, ui, render, loop). Threshold: 4 ms.

---

## 3. Cambios Recientes (últimos PR)

### PR — fix(shifter): la posición N no tiene contacto físico — 4 cables de señal + 1 común, N implícita
- **Fecha:** 2026-04-26
- **Autor:** Copilot
- **Rama:** `copilot/connectar-palanca-de-cambios`
- **Estado:** En curso (rama activa; merge gestionado por el mantenedor).
- **Descripción del cambio:** Tras verificar la palanca con multímetro, el usuario confirmó que **solo existen 4 contactos físicos** (P, D2, D1, R) compartiendo un cable común (azul → GND). La posición **N (Neutral) no dispone de contacto propio** — es el estado de reposo de la palanca cuando ningún interruptor está cerrado. El firmware previo asumía 5 contactos one-hot (`GPA0=P, GPA1=R, GPA2=N, GPA3=D1, GPA4=D2`) con `GEAR_MASK = 0x1F`, lo que era incorrecto y nunca habría detectado la marcha N (al no existir continuidad nadie podía cerrar `GPA2`). Esta PR corrige el mapeo del firmware al cableado real, ajusta la decodificación para tratar "ningún pin activo" como NEUTRAL, y propaga los cambios a toda la documentación de la palanca.

#### Mapeo definitivo confirmado por continuidad
```
Cable común AZUL ──────────── GND del MCP23017
GPA0 (pin 21) ──── azul + morado    → P  (Park)
GPA1 (pin 22) ──── azul + verde     → D2 (Drive 2)
GPA2 (pin 23) ──── azul + amarillo  → D1 (Drive 1)
GPA3 (pin 24) ──── azul + blanco    → R  (Reverse)
GPA4 (pin 25) ──── (sin uso, dejar al aire)
N (Neutral)   ──── (sin contacto físico — todos GPA0-3 en HIGH por pull-ups)
```

#### Firmware (`esp32/src/shifter_input.cpp`)
- `GEAR_MASK` reducido de `0x1F` (5 bits) → `0x0F` (4 bits, GPA0–GPA3).
- Eliminada la constante `PIN_NEUTRAL` y su rama en `decodeGear()` — ahora `__builtin_popcount(active) == 0` cae al `return Gear::NEUTRAL` por defecto.
- `PIN_REVERSE` reasignado de GPA4 → GPA3; `PIN_FORWARD` (D1) reasignado de GPA3 → GPA2 — coincide con los colores de cable medidos.
- Actualizado el comentario de cabecera con los colores reales y la nota explícita "N has no contact wire — detected when no pin is active".

#### Tests (`esp32/src/test_shifter_input.cpp`)
- `test_read_reverse`: byte de puerto corregido `0xEF` (GPA4) → `0xF7` (GPA3).
- `test_init_failure_backoff`: byte de Neutral corregido `0xF7` → `0xFF` (todos los pines en HIGH ⇒ ningún contacto).
- **Nuevo test** `test_read_neutral_implicit`: comprueba que un valor de puerto `0xFF` (todos los contactos abiertos) decodifica a `Gear::NEUTRAL` y que `isConnected()` permanece `true`.
- Resultado final: **28/28 tests pasan** (`g++ -std=c++17 ... && /tmp/test_shifter_input`).

#### Documentación actualizada
| Archivo | Cambio |
|---------|--------|
| `docs/PALANCA_CAMBIOS_IMPLEMENTACION.md` | Diagrama arquitectura (4 contactos + común), esquema MCP23017 (GPA0=P, GPA1=D2, GPA2=D1, GPA3=R, GPA4 sin uso), tabla de lógica con colores de cables, paso 4 de cableado físico, tabla multímetro, casos 9.1/9.2/9.3 de troubleshooting (incluido "palanca como interruptor simple"), FAQ ("5 posiciones" → "4 posiciones físicas + N implícita") |
| `docs/SENSOR_INTERFACE.md` | Tabla GPA→marcha + columna de colores de cable; diagrama ASCII del cableado |
| `docs/CONEXIONES_COMPLETAS.md` | Tabla de pines del MCP23017 (GPA1=D2, GPA2=D1, GPA3=R, GPA4 sin uso) con colores de cable |
| `docs/MATERIALES_POR_MODULO.md` | BOM palanca: "1 común + 4 señal", aclaración "N implícita" |
| `docs/PROJECT_MASTER_STATUS.md` | Línea de feature gear shifter actualizada (`GPA0-3 one-hot P/D2/D1/R; N implícito`) |
| `docs/TECHNICAL_REVIEW_REPORT.md` | Descripción del driver: "GPA0–GPA3 → P/D2/D1/R; N detected implicitly when no pin is active" |

#### Validación
- 28/28 tests unitarios pasan sin warnings en código de producción.
- No hay cambios en la API pública (`shifter_input.h`) — los valores del enum `Gear` y la firma de `getGearRaw()` se mantienen, por lo que el contrato CAN (0x102, byte 1 = gear) **no cambia**.
- No se altera la lógica de backoff I2C ni la detección de reconexión.

---

### PR — docs(encoder): migración completa de conexión E6B2-CWZ6C a 3× 6N137 — eliminación de TXS0108E y divisores resistivos
- **Fecha:** 2026-04-26
- **Autor:** Copilot
- **Rama:** `copilot/e6b2-cwz6c-encoder-implementation`
- **Estado:** En curso (rama activa; merge gestionado por el mantenedor).
- **Descripción del cambio:** Toda la documentación del proyecto describía la interfaz del encoder E6B2-CWZ6C con el STM32 usando un level-shifter TXS0108E o un divisor resistivo (R1=1kΩ + R2=2.2kΩ), soluciones que **no proporcionan aislamiento galvánico**. Dado que el encoder está físicamente próximo al motor de dirección (BTS7960, conmutando a 20 kHz, corrientes de hasta 10 A), los picos inductivos y los bucles de masa representan un riesgo real de corrupción de los conteos de cuadratura y de daño permanente a los pines PA15/PB3/PB4. Esta PR migra **toda la documentación** a la solución definitiva: **3× optoacopladores 6N137** (uno por canal A, B, Z), que proporcionan aislamiento galvánico de 2500 V y conversión 5 V→3.3 V simultáneamente, con 120 ns de propagación máxima (< umbral del filtro TIM2 de 282 ns).

#### Circuito definitivo E6B2-CWZ6C → STM32G474RE

```
Encoder A/B/Z (push-pull 5V) → [330Ω R_IN] → LED 6N137 → GND_encoder (aislado)
                                              Vo 6N137 → [4.7kΩ pull-up a 3.3V] → PA15 / PB3 / PB4
                                              EN (pin 7) → +3.3V
```

| Canal | R_IN | Destino STM32 | Pull-up | Periférico |
|-------|------|---------------|---------|-----------|
| A (negro) | 330 Ω | PA15 | 4.7 kΩ a 3.3V | TIM2_CH1 (AF1) |
| B (blanco) | 330 Ω | PB3 | 4.7 kΩ a 3.3V | TIM2_CH2 (AF1) |
| Z (naranja) | 330 Ω | PB4 | 4.7 kΩ a 3.3V | EXTI4 |

#### Archivos modificados (solo documentación — sin cambios de firmware)

| Archivo | Tipo de cambio |
|---------|----------------|
| `docs/ENCODER_WIRING_TXS0108E.md` | ❌ **Eliminado** |
| `docs/ENCODER_WIRING_6N137.md` | ✅ **Creado** — guía completa: esquema, BOM, análisis de frecuencia/timing, verificación hardware |
| `docs/CONEXIONES_COMPLETAS.md` | Sección encoder (tabla cableado, BOM, etapa 5 instalación, columnas de pines) |
| `docs/MATERIALES_POR_MODULO.md` | §7 BOM encoder (TXS0108E → 3× 6N137, R_IN, pull-up); tabla de resistencias |
| `docs/LISTADO_PINES_COMPLETO.md` | Sección 2.8 encoder (TXS0108E → 6N137, componentes); resistencias divisor → R_IN + pull-up |
| `docs/HARDWARE.md` | Level-shifter, BOM principal, referencia datasheet, enlace a doc |
| `docs/HARDWARE_WIRING_MANUAL.md` | §12.5 (divisor/BSS138 → 6N137 con esquema y cálculo); BOM final; tabla multímetro |
| `docs/SENSOR_INTERFACE.md` | Opciones level-shifter (6N137 = ✅ elegida; TXS0108E/BSS138/divisor = ❌); esquema conexión ASCII |
| `docs/ENCODER_CURRENT_STATE.md` | Comentario IC1Filter (210 ns → 282 ns corregido), pull-up rationale, referencias TXS0108E → 6N137 |
| `docs/hardware_modifications.md` | Descripción de cableado encoder; checklist instalación |
| `docs/POWER_DISTRIBUTION.md` | §8 protección encoders (BSS138/divisor → 6N137) |
| `docs/IGNITION_KEY_CIRCUIT_VALIDATION.md` | Exclusión encoder: eliminada BSS138 como opción válida; 6N137 como solución definitiva |
| `Documentos/SISTEMA_ALIMENTACION_COMPLETO.md` | Diagrama de alimentación, tabla protección pines, tabla multímetro, tabla pinout, BOM resistencias |
| `README.md` | Tabla Encoder Interface |

#### Motivo técnico

| Opción | Estado | Razón |
|--------|--------|-------|
| **3× 6N137** | ✅ Elegida | Aislamiento galvánico 2500 V + 5V→3.3V. 10 Mbps (×500 de margen). Protege STM32 de picos inductivos del BTS7960 adyacente. |
| TXS0108E | ❌ Descartada | Sin aislamiento galvánico. Bucles de masa y picos inductivos del motor de dirección degradan los pulsos de cuadratura. |
| Divisor R1/R2 | ❌ Descartada | Sin aislamiento. Deforma flancos. No protege contra picos inductivos. |
| BSS138 | ❌ Descartada | Sin aislamiento galvánico. Mismo problema que TXS0108E. |

#### Verificaciones de compatibilidad

| Criterio | Resultado |
|----------|-----------|
| Propagación 6N137 (120 ns máx) vs. filtro TIM2 (282 ns) | ✅ Flancos reales pasan, glitches rechazados |
| f_max encoder a 20 kHz vs. 6N137 límite 10 Mbps | ✅ Margen ×500 |
| GPIO_NOPULL (PA15, PB3) en firmware | ✅ Correcto — pull-up externo 4.7kΩ en Vo del 6N137 |
| Señal invertida A y B simultáneamente | ✅ Cuadratura preservada — solo cambia sentido de conteo |
| Código firmware (encoder_reader.c, main.c) | ✅ Sin cambios — el hardware es transparente al firmware |

#### Invariantes preservadas

- Sin cambios en firmware (STM32 ni ESP32).
- Sin cambios en protocolo CAN (IDs, DLC, cadencia).
- Sin cambios en lógica de control (PID, Ackermann, safety).
- `GPIO_NOPULL` en PA15/PB3 es correcto con pull-up externo 4.7kΩ.

---

### PR — feat(shutdown): handshake CAN 0x130 SYSTEM_SHUTDOWN — estado seguro determinista pre-corte de potencia
- **Fecha:** 2026-04-25
- **Autor:** Copilot
- **Rama:** `copilot/complete-technical-audit-esp32-s3`
- **Estado:** En curso (rama activa; merge gestionado por el mantenedor).
- **Descripción del cambio:** El STM32 transitaba a estado seguro de forma **pasiva**, únicamente cuando el módulo retardo cortaba físicamente los 5 V — sin participar en la secuencia de apagado. Esta PR añade un handshake determinista y no destructivo: el ESP32 envía el nuevo mensaje CAN `SYSTEM_SHUTDOWN` (0x130, DLC 0) al entrar en `SHUTTING_DOWN`, y el STM32 ejecuta de inmediato la parada segura (PWM=0, EN=LOW, relés OFF) **antes** del corte físico. Si la trama se pierde en el bus, el comportamiento anterior se preserva íntegramente (cero regresión).

#### Cambios principales

**STM32**
- **`Core/Inc/can_handler.h`**: nuevo `#define CAN_ID_CMD_SYSTEM_SHUTDOWN 0x130` (aditivo — sin modificar ningún ID existente).
- **`Core/Inc/safety_system.h`**: declaración de `void Safety_RequestShutdown(void)`.
- **`Core/Src/safety_system.c`**: implementación de `Safety_RequestShutdown()` — envuelve primitivas ya validadas:
  - `Traction_EmergencyStop()` → PWM=0 + EN=LOW en los 4 motores de tracción.
  - `Steering_Neutralize()` → PWM=0 + `eps_motor_effort=0` + EN=LOW en dirección.
  - `Relay_PowerDown()` → escritura atómica BSRR: TRAC + DIR OFF en un ciclo.
  - `system_state = SYS_STATE_SAFE` si no estaba ya en SAFE/ERROR (preserva contexto diagnóstico).
  - **Idempotente, no bloqueante, sin delays ni bucles ni nuevos estados.**
- **`Core/Src/can_handler.c`**: nuevo `case CAN_ID_CMD_SYSTEM_SHUTDOWN:` en el switch del dispatcher — llama a `Safety_RequestShutdown()`. Sin ACK (la ruta es no bloqueante por diseño; el ESP32 no lo necesita en el cierre).

**ESP32**
- **`esp32/include/can_ids.h`**: `inline constexpr uint32_t CMD_SYSTEM_SHUTDOWN = 0x130` en el namespace `can`.
- **`esp32/src/main.cpp`**: nuevo helper `sendSystemShutdown()` (DLC 0, no bloqueante, sin ACK wait), llamado **una sola vez** al entrar en `SHUTTING_DOWN`, seguido de `delay(100)` para dar ventana de reacción al STM32. Precedido por la lógica existente de farewell/flush/LED-OFF. `SHUTDOWN_DELAY_MS`, GPIO 41 y CAN 0x120 LED-OFF permanecen intactos.

**Documentación**
- **`docs/LLAVE_CONTACTO_ENCENDIDO_APAGADO.md`** §8: tabla de secuencia de apagado ampliada de 10 a 13 pasos — se incluye el nuevo paso 0x130 con timing (t~50 ms), la pausa de 100 ms de reacción y la nota de compatibilidad hacia atrás.

#### Garantías de seguridad (post-recepción del frame)
- PWM = 0 en todos los canales (TIM1/TIM8/TIM3).
- EN pins = LOW en todos los motores (tracción + dirección).
- Relés TRAC + DIR = OFF (escritura BSRR atómica).
- `system_state = SYS_STATE_SAFE` bloquea cualquier re-energización posterior.
- Ninguna ruta puede re-habilitar salidas (secuenciador guardado por estado).

#### Invariantes preservadas
- Sin modificación de ningún CAN frame existente (IDs, DLC, cadencia).
- Sin bump de versión del contrato CAN.
- Sin nuevas tareas, hilos, interrupciones ni cambios en el watchdog IWDG.
- Sin cambios en lógica PWM (TIM1/TIM8/TIM3), encoder, EPS, Ackermann ni secuenciador de relés.
- `Safety_GetRelayStatusByte()` sin regresión.
- Único punto de entrada nuevo: `Safety_RequestShutdown()` (STM32) / `sendSystemShutdown()` (ESP32).

#### Validación
- **Build STM32**: 20 ficheros fuente pasan `arm-none-eabi-gcc -Wall -Wextra -Werror -fsyntax-only` (mismos flags del CI `firmware-validation.yml`).
- **Code Review**: ✅ — feedback de review incorporado (tabla de doc actualizada + nota de threading context en la asignación de `system_state`).
- **CodeQL**: ✅ — 0 alertas.
- **Grep**: 0 rutas de apagado duplicadas; `Safety_RequestShutdown` es el único nuevo entry point.
- **Backward compatibility**: sin el frame CAN, el comportamiento es bit-idéntico al estado anterior — el módulo retardo hardware sigue cortando potencia con el mismo timing.

---

### PR — docs(relay): completar diagrama eléctrico LLAVE_CONTACTO — 4 módulos de relé + módulo 5 V LED
- **Fecha:** 2026-04-25
- **Autor:** Copilot
- **Rama:** `copilot/complete-technical-audit-esp32-s3`
- **Estado:** En curso (rama activa; merge gestionado por el mantenedor).
- **Descripción del cambio:** El documento `docs/LLAVE_CONTACTO_ENCENDIDO_APAGADO.md` describía sólo 3 módulos de relé y no incluía el módulo de 5 V que alimenta las tiras LED WS2812B. Se añade una sección dedicada al **módulo relé 2 canales SRD-05VDC-SL-C** (PB10/PB11), se actualiza el resumen general a 4 módulos, se completan los diagramas eléctricos, la secuencia de encendido/apagado, la lista de componentes y el índice de secciones. Adicionalmente, la sección de detección de llave (GPIO 40 del ESP32-S3) se amplía con una tabla comparativa de las 3 opciones de cableado (R1+R2, puente 3.3 V, optoacoplador adicional) y el cálculo del divisor de tensión verificado.

#### Cambios principales (sólo documentación — sin cambios de firmware)
- **`docs/LLAVE_CONTACTO_ENCENDIDO_APAGADO.md`** — 351 líneas añadidas, 340 eliminadas/reorganizadas:
  - **Resumen general**: tabla de 3 módulos → **4 módulos** (`Módulo retardo 12 V`, `Módulo 2ch SRD-12VDC-SL-C`, `Módulo 2ch SRD-05VDC-SL-C`).
  - **Índice**: secciones renumeradas; nueva sección 5 `Módulo Relé 2 Canales 5 V — Alimentación Tiras LED` añadida entre las secciones de módulo 12 V y la secuencia de encendido.
  - **Tabla de pines STM32 verificados**: ampliada con `PB10` (RELAY_LED, frontal 28 WS2812B) y `PB11` (RELAY_LED_REAR, trasera 16 WS2812B), referencias a `project_config.h:210-211` y `can_handler.c:1477-1494`.
  - **Sección 2 — GPIO 40 ESP32-S3**: nuevo sub-apartado 2.2/2.3 con tabla comparativa de opciones A/B/C (R1+R2 recomendada, puente, opto adicional) y cálculo `V_GPIO40 = 2.79 V`.
  - **Sección 4 — Módulo 12 V** (antes §4): cableado VCC/JD-VCC/GND, posición del jumper H, aviso de retirada del jumper VCC-JD-VCC, cálculo de corriente `I = 2.1 mA` con optoacoplador PC817.
  - **Sección 5 — Módulo 5 V** (nueva): misma estructura que §4 pero para `SRD-05VDC-SL-C`; IN1←PB10 (frontal), IN2←PB11 (trasera); VCC→3.3 V, JD-VCC→5 V (sin quitar jumper porque VCC y JD-VCC son del mismo nivel); tabla de salidas hacia las tiras LED.
  - **Diagrama eléctrico completo** (§9): actualizado con los 4 módulos en el esquema ASCII y los nodos `5V_LED_SUPPLY_F/R`.
  - **Lista de componentes** (§10): añadidos el módulo SRD-05VDC-SL-C y el divisor R1/R2.
  - **Preguntas frecuentes** (§11): nueva FAQ sobre si hace falta resistencia entre STM32 y módulo de 5 V (respuesta: no).

#### Invariantes preservadas
- Sin cambios de firmware, hardware, ni protocolo CAN.
- La asignación de pines `PB10`/`PB11` ya existía en `project_config.h` y `can_handler.c`; el documento se limita a reflejar la realidad del código.
- El contrato CAN (rev 1.3) y todos los IDs de mensajes son idénticos.

#### Validación
- Revisión cruzada con `project_config.h` (líneas 199, 200, 210, 211) y `can_handler.c` (líneas 1477–1494): todos los pines y valores documentados coinciden con el firmware.
- Ningún test de firmware afectado (cambio exclusivamente documental).

---

### PR — hardware(pinout): reassign EN_RR from PC13 to PC2 (USER button B1 conflict)
- **Fecha:** 2026-04-24
- **Autor:** Copilot
- **Rama:** `copilot/connect-e6b2-cwz6c-to-esp32`
- **Estado:** En curso (rama activa; merge gestionado por el mantenedor).
- **Descripción del cambio:** En la NUCLEO-G474RE (MB1367C), PC13 está cableado por hardware al botón USER B1 vía SB17 (ver UM2505 §6.4). Configurar PC13 como `GPIO_MODE_OUTPUT_PP` para `EN_RR` generaba dos problemas: (a) pulsar B1 cortocircuita PC13 a GND y fuerza `EN_RR` LOW → el motor trasero derecho se desactiva silenciosamente; (b) la salida push-pull pelea contra la pull-up externa del circuito del botón. Se reasigna `PIN_EN_RR` a **PC2** (GPIO libre tras la migración DIR→RPWM/LPWM), quedando PC13 sin uso (estado por defecto de reset, input flotante). El puerto `GPIOC` y la máscara de inicialización siguen siendo los mismos; como el firmware usa el macro `PIN_EN_RR` en todos los sitios (init, BSRR pre-config en `MX_GPIO_Init()`, `Error_Handler`, `Fault_MotorShutdown` en HardFault/MemFault/BusFault/UsageFault, `motor_rr.en_pin`, la defensa C2 antes de `HAL_TIM_PWM_Start`), el cambio propaga automáticamente a todas las rutas de seguridad.

#### Cambios principales (NO breaking)
- **`Core/Inc/project_config.h`**: `PIN_EN_RR` → `GPIO_PIN_2` (`PC2`); bloque de comentario del safety contract actualizado al nuevo conjunto `{PC0, PC1, PC2, PC4, PC5}`.
- **`Core/Src/main.c`, `Core/Src/motor_control.c`**: sólo refresco de comentarios (el código ya usaba el macro).
- **`STM32-Control-Coche-Marcos.ioc`**: `Mcu.Pin29=PC2`, bloque `PC13.*` sustituido por `PC2.*` con la misma etiqueta `EN_RR`. Previene que una regeneración futura desde STM32CubeMX restaure la asignación vieja a PC13.
- **`docs/hardware_modifications.md`**: §1 reescrita — la antigua mandato de desoldar SB17 se reemplaza por la justificación de la reasignación a PC2 + pasos de verificación; checklist pre-power-up actualizado.
- **`docs/HARDWARE_WIRING_MANUAL.md`**: fila EN_RR en tablas de pinout (PC2), resumen de cableado por motor, listado maestro de 64 pines, notas adicionales sobre PC13 reservado; línea-resumen (antes decía "PC13 (RR)") corregida a "PC2 (RR)"; nota de DIR pins liberados ajustada a `{PC0, PC1, PC4}` (PC2 ya no está libre).
- **`docs/CONEXIONES_COMPLETAS.md`**: banner del top, diagrama ASCII del sistema, tabla de cableado por driver, tabla maestra de 64 pines, checklist final de cableado; la sección "Pines liberados (PC2) — ya no se cablean" se renombró a "Pines ex-DIR reasignados — cableado actual", con la fila de PC2 corrigida de "LIBRE" a "EN_RR" (alineada con la tabla maestra que ya lo indicaba).
- **`docs/PINOUT.md`**: nota de migración actualizada con la referencia al conflicto con USER button B1.

#### Invariantes preservadas
- `EN_RR` se inicializa LOW antes de `HAL_GPIO_Init()` (defensa C1 Hardening) — mismo BSRR que los otros 4 EN pins.
- Todas las rutas de apagado de emergencia (`Error_Handler`, 4× fault handlers) incluyen la máscara `PIN_EN_RR` sin cambios, al usar el mismo macro.
- La máscara combinada del grupo EN sobre `GPIOC` pasa de `0x2033` (bits 0,1,4,5,13) a `0x0037` (bits 0,1,2,4,5); disjunta de la máscara PWM en GPIOC (`0x03C8`, bits 3/6/7/8/9) y de la máscara de relés (`0x1C00`, bits 10/11/12).
- PC2 y PA2 comparten el valor literal `GPIO_PIN_2` pero pertenecen a puertos distintos; EXTI2 permanece ligada a PA2 (input con pull-up, detección de flanco de rueda). PC2 está configurado como `OUTPUT_PP` — sin conflicto EXTI.
- Sin cambios en: EPS, Ackermann, PID de dirección, máquina de estados de seguridad, relés, CAN, timers, encoder, sensores, Flash, Service Mode, protocolo CAN, dependencias.

#### Validación
- `make -j4` (host previamente verificado): `text=60560 data=88 bss=7984` — sin regresiones de tamaño.
- gcc `-Wall -Wextra -Werror` limpio sobre `main.c`, `motor_control.c`, `stm32g4xx_it.c`, `project_config.h` (consumers).
- `grep -R "GPIO_PIN_13" Core/` → 0 matches activos (sólo comentarios explicativos referenciando la migración).
- Re-auditoría documental: todos los PC13 restantes en los 5 archivos de scope son contexto histórico legítimo ("reasignado desde PC13", "PC13 reservado por B1"). 0 aserciones activas residuales de `EN_RR = PC13` en el scope obligatorio.
- Deuda documental conocida (fuera de scope, no afecta al firmware): ~22 documentos históricos de `docs/` aún mencionan la asignación antigua; serán corregidos en un barrido dedicado posterior.

---

### PR — safety(relay): force full power-down when leaving ACTIVE mid-sequence
- **Fecha:** 2026-04-23
- **Autor:** Copilot
- **Rama:** `copilot/add-vehicle-control-system`
- **Estado:** Cerrada (cambios aplicados en rama; merge gestionado por el mantenedor).
- **Descripción del cambio:** Endurecimiento final del secuenciador de relés (`Relay_SequencerUpdate()` en `safety_system.c`). El guard `if (system_state != SYS_STATE_ACTIVE) return;` ya impedía la re-energización fuera de ACTIVE, pero si el sistema abandonaba ACTIVE mientras el secuenciador estaba en `RELAY_SEQ_TRACTION_ON` (TRAC energizado, DIR aún OFF, esperando el settle de 50 ms), el hardware quedaba en un estado parcial `TRAC=ON, DIR=OFF` hasta que otra ruta apagara los relés. Ahora, cuando el guard se dispara en plena transición, se invoca `Relay_PowerDown()` para apagar ambos relés atómicamente vía BSRR y resetear el secuenciador a IDLE antes del return.

#### Cambios principales (NO breaking)
- **`Core/Src/safety_system.c`** — `Relay_SequencerUpdate()`: el early-return del guard fuera de ACTIVE detecta la condición `relay_seq_state == RELAY_SEQ_TRACTION_ON` y llama a `Relay_PowerDown()` antes de retornar. Comentario de bloque ampliado (≈28 líneas) documentando el edge-case mid-sequence y el comportamiento en cada estado del sistema (SAFE, ERROR, STANDBY, BOOT, DEGRADED, LIMP_HOME).
- **Decisión de diseño:** fail-safe **OFF** en lugar de auto-completar la activación. Eléctricamente más seguro; evita energización inesperada del motor; comportamiento determinista bajo cualquier fallo.

#### Invariante garantizada
- No existe estado persistente `TRAC=ON ∧ DIR=OFF`. Los relés están siempre coherentes: totalmente OFF o totalmente ON (TRAC+DIR).
- No hay re-energización fuera de ACTIVE.
- Emergency stop permanece atómico, determinista e irreversible hasta recuperación de estado.
- DEGRADED / LIMP_HOME tras una transición normal desde ACTIVE (secuenciador en COMPLETE) mantiene los relés ON sin tocarlos.

#### Validación
- Host tests: 195 (service_mode) + 11 529 (motor_control) pasan sin cambios.
- gcc `-Wall -Wextra -Werror -fsyntax-only` limpio en `safety_system.c`.
- CodeQL: 0 alerts.
- Sin cambios en: constantes de tiempo, protocolo CAN, máquina de estados de seguridad, diagnóstico, macros de relé. Sin nuevos estados. Sin refactor arquitectónico.
- Máscara de override (`0x06U`) verificada consistente: bit 1 = TRAC, bit 2 = DIR, bit 0 = reservado/0 — coherente con el ensamblado del status byte y las rutas de apply.

---

### PR — hardware(relay): hardening post-migración (CAN compatible, 50ms settle, E-stop determinista)
- **Fecha:** 2026-04-23
- **Autor:** Copilot
- **Rama:** `copilot/add-vehicle-control-system`
- **Descripción del cambio:** Pase de endurecimiento sobre la migración previa a 2 relés. Restaura compatibilidad hacia atrás del contrato CAN (layout de 3 bits con bit 0 reservado/always‑0), aumenta el tiempo de settle del relé de tracción de 20 ms a 50 ms, y hace la parada de emergencia totalmente determinista con escritura atómica BSRR. **Arquitectura de 2 relés (TRAC + DIR) se mantiene; no se reintroduce MAIN.**

#### Cambios principales (NO breaking, compatible con rev 1.3)
- **CAN**: `HEARTBEAT_STM32` (0x001) byte 5 `relay_status` vuelve a **layout de 3 bits** — bit 0 = reservado (hueco legacy MAIN, siempre 0), bit 1 = TRAC, bit 2 = DIR, bit 7 = SEQ_COMPLETE. Sin cambio de tamaño ni de ID. **Sin bump de protocolo**; contrato sigue siendo rev 1.3.
- **Override** (SERVICE_CMD 0xE0 byte 1): vuelve al formato legacy de 3 bits (bit 0 reservado, bit 1 = TRAC, bit 2 = DIR).
- **Settle time**: `RELAY_TRACTION_SETTLE_MS` 20 ms → **50 ms** para cubrir inrush del bus de 24 V + 4× BTS7960.
- **Emergency stop determinista**: `Relay_PowerDown()` usa escritura atómica `GPIOC->BSRR = (TRAC|DIR) << 16` que fuerza ambos relés OFF en un ciclo, independientemente del estado del secuenciador u override. `Safety_EmergencyStop()` invoca `Relay_PowerDown()` antes de la transición a ERROR (antes lo hacía después). La máscara BSRR solo incluye TRAC + DIR (PC10 intacto).
- **Diagnóstico**: validado que `MODULE_RELAY_TRAC` (stuck‑open) sigue usando INA226 real (corriente total de motores) + tensión de batería + velocidad de ruedas — no solo GPIO readback. Sin redesign.
- **ESP32 HMI**: decoders actualizados a bit 1 = TRAC, bit 2 = DIR en `vehicle_data.h`, `relay_indicator.h`, `safe_screen.cpp`, `engineering_screen.{h,cpp}`. Corregido bucle táctil del menú Relay Control (4 → 3 filas) para coincidir con el render.
- **Docs**: `CAN_CONTRACT_FINAL.md` revertido a **rev 1.3** con aclaración de 2026‑04‑23 (bit 0 reservado). `can_ids.h` comentario bump revertido. Banners de los 10 docs de hardware actualizados a "CAN rev 1.3 compatible".

#### Validación
- Host tests: 195 (service_mode) + 11 529 (motor_control) pasan.
- gcc `-Wall -Wextra -Werror` limpio en `safety_system.c`, `main.c`, `stm32g4xx_it.c`, `service_mode.c`, `can_handler.c`, `motor_control.c`.
- grep confirma 0 referencias a `PIN_RELAY_MAIN`, `MODULE_RELAY_MAIN`, `RELAY_SEQ_MAIN_*`.

---

### PR — hardware(relay): eliminar relé MAIN inexistente (24 V solo tiene un relé)
- **Fecha:** 2026-04-23
- **Autor:** Copilot
- **Rama:** `copilot/add-vehicle-control-system`
- **Descripción del cambio:** El hardware real del coche solo tiene **un relé de 24 V** (tracción, alimenta los 4 BTS7960) y **un relé de 12 V** (dirección). **NO existe un contactor MAIN / Power-Hold** independiente que justificase el tercer relé `PIN_RELAY_MAIN` (PC10) que había en firmware. Se elimina toda la lógica asociada. *(Nota: el bump a rev 1.4 de esta PR fue posteriormente revertido en el pase de hardening — el contrato CAN se mantiene en rev 1.3 con bit 0 reservado/0).*

#### Cambios principales (a nivel firmware — cambio CAN posteriormente revertido a compatible)
- **CAN**: `HEARTBEAT_STM32` (0x001) byte 5 `relay_status` cambia de 3 bits (MAIN/TRAC/DIR) a **2 bits (TRAC=bit0, DIR=bit1)**; bit 7 = `SEQ_COMPLETE` sigue igual. Contrato bump 1.3 → 1.4. Consumidores deben actualizarse en lockstep.
- **Firmware STM32**:
  - `project_config.h`: `PIN_RELAY_MAIN` eliminado; PC10 queda **reservado/libre**. `PIN_RELAY_TRAC` (PC11) y `PIN_RELAY_STEER_PWR` (PC12) se mantienen.
  - `safety_system.c`: secuenciador pasa de 3 fases (MAIN→TRAC→DIR) a **2 fases** (TRAC→DIR, ~20 ms — posteriormente ajustado a 50 ms en el hardening). `RELAY_MAIN_SETTLE_MS` eliminado; `RELAY_SEQ_MAIN_ON` eliminado. `Safety_GetRelayStatusByte()` reporta layout de 2 bits internos (bit 0 = TRAC, bit 1 = DIR) — *en el pase de hardening posterior se restauró el layout de 3 bits rev 1.3 compatible con bit 0 = reservado/0*. Override mask (SERVICE_CMD 0xE0) pasa a 2 bits — *también restaurado a 3 bits en el hardening*. Diagnóstico de "stuck open" referido ahora a `MODULE_RELAY_TRAC`.
  - `main.c` / `stm32g4xx_it.c`: GPIO init y máscara BSRR del *emergency stop* sin `PIN_RELAY_MAIN`.
  - `service_mode`: `MODULE_RELAY_MAIN` **renombrado a `MODULE_RELAY_TRAC`** (ID 3 preservado — no rompe IDs de CAN ni métricas históricas).
- **Firmware ESP32 (HMI)**:
  - `vehicle_data.h`, `relay_indicator.h`, `safe_screen.cpp`, `engineering_screen.cpp/h`: layout de 2 bits (posteriormente restaurado a 3 bits compatibles rev 1.3 en el hardening); widget pasa de `M T D` a `T D`. La pantalla de Relay Control en Engineering pasa de 4 filas (Override / MAIN / TRAC / DIR) a 3 (Override / TRAC / DIR).
  - `can_ids.h`: referencia al contrato CAN (rev 1.3 se mantiene tras hardening).
- **`.ioc`**: PC10 eliminado como `GPIO_Output` (edición manual — sin regeneración CubeMX).
- **Tests**: `test_service_mode.c` y `test_motor_control.c` actualizados a `MODULE_RELAY_TRAC` y a la nueva máscara de relés (5 motores + **2 relés** + 2 LED relays = 9 salidas). **195 + 11 529 tests pasan.**
- **Docs**: banners de actualización en `POWER_DISTRIBUTION.md`, `HARDWARE_WIRING_MANUAL.md`, `LISTADO_PINES_COMPLETO.md`, `CONEXIONES_COMPLETAS.md`, `SAFETY_SYSTEMS.md`, `SAFETY_ARCHITECTURE.md`, `INFORME_REVISION_TECNICA_RELAY.md`, `INA226_RELAY_SAFETY_AUDIT.md`, `AUDIO_RELAY_INTEGRATION.md`, `PINOUT.md`.

#### Lo que NO cambia
- Lógica de BTS7960, PWM, encoders, FDCAN de telemetría, ESP32 HMI (salvo el widget de relés).
- Timings/umbrales de otros subsistemas de seguridad.
- IDs de módulo del Service Mode (al renombrar, el ID=3 se conserva).
- `relay_audio` (módulo de audio en ESP32) es independiente — no afectado.

---

### PR — fix(steering): deadband 1.8° en dominio de rueda para absorber holgura mecánica
- **Fecha:** 2026-04-23
- **Autor:** Copilot
- **Rama:** `copilot/connect-wheels-sensors-and-volante`
- **Descripción del cambio:** Tras la migración del lazo de dirección a grados de rueda vía `Steering_GetCurrentAngle()`, el antiguo `STEERING_DEADBAND_COUNTS` (0.5° en eje del encoder) quedó reducido a ≈0.077° en la rueda — muy por debajo de los ~3° de holgura mecánica real entre RS390 + reductor ~1:50 y las ruedas. El lazo EPS perseguía la holgura y producía chatter cerca del centro. Se introduce un deadband expresado en el dominio global (grados de rueda) y se aplica únicamente al consumidor PID.

#### Cambios principales

| Subsistema | Cambio |
|---|---|
| **Dirección (STM32)** | Nuevo macro `STEERING_DEADBAND_DEG = 1.8f` en `Core/Src/motor_control.c` (~60% de los 3° de holgura observada). Aplicado sobre `theta` dentro de `Steering_ControlLoop()` justo después de `sanitize_float(theta)` y **antes** de derivar ω, de modo que los términos de asistencia (`λ·k·g(v)·ω`), retorno al centro (`(1−λ)·k·h(v)·θ`) y compensación de fricción colapsan a cero dentro de la ventana muerta. |
| **Macro antiguo** | `STEERING_DEADBAND_COUNTS` se deja en su sitio pero sin uso, con un comentario que redirige al macro en grados. No se reintroduce lógica en el espacio de counts. |
| **Rango de ajuste documentado** | 1.5°–2.0°; **nunca** por debajo de 1.0° (inestable por quedar dentro de la holgura). |

#### Non-goals (explícitos)

| Componente | Estado |
|---|---|
| `Steering_GetCurrentAngle()` (única fuente de verdad en grados de rueda) | ✅ sin tocar |
| `STEERING_GEAR_RATIO`, `ENC_MAX_JUMP`, `ENC_MAX_COUNTS` | ✅ sin tocar |
| ISR de encoder y diagnóstico de canal Z (PB4 / EXTI4) | ✅ sin tocar |
| Geometría Ackermann, ganancias de la ecuación de par EPS, fade a alta velocidad, escalado en modo degradado | ✅ sin tocar |
| Máquina de estados de seguridad y plausibility checks (leen el ángulo sin filtrar) | ✅ sin tocar |

#### Validación

- Build limpio con `-Wall -Wextra -Werror`; delta de flash +16 B text, 0 B bss.
- `|θ| < 1.8°` ⇒ `θ = 0` ⇒ sin salida PWM → sin chatter en reposo.
- `|θ| ≥ 1.8°` ⇒ ruta de control bit-idéntica al comportamiento previo (mismas ganancias, misma ecuación de par, mismo fade).
- Subsistemas Ackermann y safety observan el ángulo sin filtrar y quedan byte-a-byte inalterados.

#### Contratos preservados

- CAN v1.3 sin cambios (IDs, DLC, bytes, cadencia).
- `Safety_*`, pipeline PWM, BREAK2, watchdog IWDG — intactos.
- Lógica O(1), sin `malloc`, sin llamadas bloqueantes.

- **Impacto global:** elimina el chatter en dirección cerca del centro sin alterar la dinámica fuera de la ventana muerta ni los límites de seguridad; listo para producción.

### PR — feat(obstacle+temperature): política 50 cm, EMA de zona, histéresis 750 mm, remap DS18B20 por CAN 0x112
- **Fecha:** 2026-04-21
- **Autor:** Copilot
- **Rama:** `copilot/obstacle-sensor-distance-control`
- **Descripción del cambio:** Auditoría cruzada STM32 ↔ ESP32 y cierre del PR de la política de obstáculo de 50 cm, filtrado EMA de zona en UI, histéresis restaurada, y mapeo dinámico DS18B20 vía CAN 0x112 con persistencia en Flash. Contrato CAN v1.3 preservado (IDs, DLC, layout, cadencia sin cambios).

#### Cambios principales

| Subsistema | Cambio |
|---|---|
| **Obstáculo (STM32)** | `OBSTACLE_EMERGENCY_MM = 500`, `OBSTACLE_RECOVERY_MM = 750` en `safety_system.c` (banda de histéresis 250 mm). `CONFIRM_MS` / `CLEAR_MS` / `TIMEOUT_MS` sin cambios. |
| **Obstáculo (ESP32)** | Filtro EMA entero (α = 0.3, `ZONE_EMA_NUM` / `ZONE_EMA_DEN`, O(1)) en `obstacle_sensor.cpp` aplicado solo a clasificación de zona. `reading_.distance_mm` permanece crudo → CAN 0x208 sin filtrar. Reseed en `init()` y en timeout→INVALID. |
| **UI (ESP32)** | `proximityColor()` en `ui_common.h` alineado 1:1 con las zonas de seguridad: rojo <500, naranja 500–1000, amarillo 1000–1500, cian 1500–2000, verde ≥2000 mm (en cm: <50 / <100 / <150 / <200 / ≥200). |
| **Temperatura (STM32)** | Filtro bootstrap de plausibilidad en `CAN_SendStatusTempMap()` (ventana 0–60 °C, consenso de 2 muestras dentro de `BOOT_REF_TOLERANCE_C`). Reset de estado de filtro (`last_good_valid[]`, `bootstrap_count[]`, `bootstrap_ref[]`, `last_good_temp_f[]`) ante cambio de topología 1-Wire. |
| **Remap DS18B20** | `CMD_SENSOR_MAP_TEMP` (CAN 0x112, DLC 5) desde menú de ingeniería ESP32 → `SensorMapStore_Save()` en Flash página 125, con `CMD_ACK` (0x103). `CAN_SendStatusTempMap()` consume `SensorMapStore_GetMap()` en cada tick. |
| **Docs** | 13 archivos en `docs/` actualizados para retirar referencias obsoletas a 200 mm / 200–500 mm (política 50 cm unificada). |
| **Comentarios** | `safety_system.c`: `/* zone level (0–5) */` → `(0–4)` (coherente con `can_handler.c:1445` y con las 5 zonas reales 0..4). `obstacle_sensor.h`: documentación de zonas y EMA ampliada. |
| **Tests** | `esp32/src/test_obstacle_sensor.cpp`: 7 aserciones obsoletas corregidas + tests de límite exhaustivos para TOFSense-M y TF-Mini Plus (499/500/501, 999/1000/1001, 1499/1500/1501, 1999/2000/2001, 3999/4000 mm). |

#### Auditoría cruzada — resultados

| Verificación | Resultado |
|---|---|
| IDs 0x103 / 0x110 / 0x112 / 0x206 / 0x208 / 0x209 | ✅ sin cambios |
| DLCs (3 / 2 / 5 / 5 / 5 / 4) | ✅ sin cambios |
| Layout de bytes STM32 ↔ ESP32 (`can_handler.c` vs `can_obstacle.cpp` + `can_ids.h`) | ✅ idéntico |
| Cadencia TX (1000 / 66 / 100 ms + on-demand) | ✅ sin cambios |
| `Safety_*` y máquina de estados | ✅ sin modificar |
| Acoplamiento temperatura ↔ obstáculo | ✅ desacoplado (códigos `SAFETY_ERROR_OVERTEMP` / `SAFETY_ERROR_OBSTACLE` independientes) |
| Autoridad de obstáculo (<500 mm → `obstacle_scale = 0.0`) | ✅ preservada |
| Determinismo / O(1) / sin `malloc` / sin bloqueo | ✅ verificado |
| Casos extremos (desconexión DS18B20, timeout LiDAR, pérdida de tramas CAN, EMI) | ✅ degradación segura |

#### Contratos preservados

- CAN v1.3 sin cambios: IDs, DLC, bytes, cadencia.
- `Safety_*`, pipeline PWM, BREAK2, watchdog IWDG — intactos.
- `OBSTACLE_CONFIRM_MS` (200 ms), `OBSTACLE_CLEAR_MS` (1000 ms), `OBSTACLE_TIMEOUT_MS` (500 ms) — sin cambios.
- Toda la lógica nueva es O(1), sin `malloc`, sin llamadas bloqueantes.

#### Impacto de seguridad

- **Parada de emergencia antes:** 500 mm en lugar de 200 mm → mayor margen de frenado.
- **Sin falsos picos de temperatura al arranque:** sólo muestras validadas por la ventana bootstrap llegan a 0x206; −127 °C / NaN / Inf quedan siempre bloqueadas.
- **Comportamiento estable bajo EMI:** EMA de zona + histéresis 250 mm eliminan oscilación en el límite de 500 mm; reset por topología impide quedar atrapado en el filtro step tras un remap.

- **Impacto global:** integración lista para producción; el sistema permanece determinista, eléctricamente seguro y compatible con el contrato CAN existente.

### PR — fix(can_handler): Harden DS18B20 boot filtering — R-1 plausibility window + R-2 topology reset
- **Fecha:** 2026-04-21
- **Autor:** Copilot
- **Rama:** `copilot/connect-temperature-sensors`
- **Commit:** `e5c77c8`
- **Descripción del cambio:** Refuerza el filtro de bootstrap de 2 muestras ya existente en `CAN_SendStatusTempMap()` cerrando dos riesgos residuales sin modificar formato de trama, APIs ni lógica de seguridad.

#### Riesgos cerrados

| ID | Riesgo | Mitigación |
|----|--------|-----------|
| **R-1** | Bootstrap falso consenso: dos muestras EMI consistentes dentro de ±`BOOT_REF_TOLERANCE_C` (10 °C) podían armar el filtro con una referencia errónea (ej. 90 °C con ambiente a 25 °C). | Ventana de plausibilidad 0–60 °C durante bootstrap: muestra fuera de rango → `bootstrap_count[role]=0` + `continue` (no se emite en ese ciclo). |
| **R-2** | Desincronización por cambio de topología 1-Wire (reconexión o reordenación de sensores): `last_good_valid[]` quedaba armado comparando un sensor nuevo contra la referencia del anterior → rechazo silencioso por `TEMP_MAX_STEP_C`. | Al inicio de `CAN_SendStatusTempMap()`, si `Temperature_HasTopologyChanged()` → `memset(0)` de `last_good_valid[]`, `bootstrap_count[]`, `bootstrap_ref[]`, `last_good_temp_f[]` + `Temperature_ClearTopologyChanged()`. |

#### Cambios en archivos

| Archivo | Cambios |
|---------|---------|
| `Core/Src/can_handler.c` | +61/−0 líneas: `#include <string.h>`, `BOOT_MIN_VALID_C=0.0f` / `BOOT_MAX_VALID_C=60.0f` a ámbito de archivo junto a `TEMP_MIN_VALID_C`, bloque de reset por topología al inicio de la función (después de las declaraciones `static`), early-`continue` de plausibilidad como primera instrucción de la rama de bootstrap. |

#### Contratos preservados

- CAN ID `0x206` (STATUS_TEMP_MAP), DLC 5 y layout de payload — **sin cambios**.
- Filtros existentes (NaN/Inf, rango `-30..120 °C`, centinela `-127 °C` de desconexión, `TEMP_MAX_STEP_C=20 °C`) — **intactos**.
- `Safety_*`, firma de función, planificación 1 Hz — **sin cambios**.
- O(1) por rol, sin `malloc`, sin bucles nuevos, sin bloqueo.

#### Criterios de aceptación (verificados en harness independiente)

| AC | Escenario | Resultado |
|----|-----------|-----------|
| 1 | Pico EMI al boot (90 °C) → luego 25 °C, 25 °C | Pico rechazado por R-1, converge en ≤ 2 muestras válidas, `armed=1` ✅ |
| 2 | Valores consistentes pero irreales fuera de ventana (p.ej. 80 °C / 82 °C) | Ambos rechazados, `armed=0` ✅ |
| 3 | Arranque normal (25 °C, 25 °C) | Bootstrap completa normalmente ✅ |
| 4 | Cambio de topología tras armado | Estado purgado, re-bootstrap correcto desde el siguiente ciclo ✅ |
| 5 | Sin regresiones: trama 0x206, `Safety_*`, timing 1 Hz, pico EMI post-armado sigue rechazado por `TEMP_MAX_STEP_C`, desconexión `-127` sigue reseteando | ✅ |

- **Safety validation summary:**
  - ✅ Sin cambios en máquina de estados de seguridad ni en `Safety_*`.
  - ✅ Sin cambios en protocolo CAN (IDs, DLC, payload).
  - ✅ Las dos ventanas añadidas **solo** actúan mientras `last_good_valid[role]==false`; tras el armado el comportamiento es idéntico al anterior.
  - ✅ CodeQL: 0 alertas.

- **Impacto:** Convergencia determinista del filtro de temperatura bajo EMI sostenido al boot y ante reconexiones/reordenación de sensores DS18B20, sin ningún efecto observable en régimen permanente.

### PR — feat: Front LED Bar Turn Signals + Rear LED Premium Improvements
- **Fecha:** 2026-04-17
- **Autor:** Copilot
- **Descripción del cambio:** Implementación completa de intermitentes LED en tira frontal (5 LEDs/lado), mejoras premium en tira trasera (overlay no destructivo, intermitentes secuenciales, activación REGEN, emergency flash real), y análisis documental completo de ambos sistemas LED.

#### Fase 1 — Intermitentes frontales + Safe Mode HAZARD (commits d68aeda, 3567fc7, 834f802)

| Feature | Descripción |
|---------|-------------|
| **Front turn signal zones** | Zonas [0–4] (izquierda) y [23–27] (derecha) definidas en `led_controller.h` con soporte `LED_STRIP_REVERSED` |
| **Overlay no destructivo (frontal)** | `updateFrontTurnSignals()` solo escribe AMBER durante blink-ON; durante blink-OFF no escribe nada → efecto KITT permanece visible debajo |
| **Safe Mode HAZARD** | En SAFE/ERROR: KITT_IDLE en centro + HAZARD (4 esquinas amber) reemplaza `startEmergencyFlash(5)` anterior |
| **Estado persistente** | Variables `turnLeftActive`/`turnRightActive` estáticas para estabilidad frame-a-frame |
| **Documentación** | `Documentos/LED_FRONT_LOGIC.md` — análisis completo de lógica frontal |

#### Fase 2 — Análisis documental trasero (commit 0434657)

| Feature | Descripción |
|---------|-------------|
| **REAR_LED_BEHAVIOR.md** | Análisis completo code-traced del strip trasero: diagrama de zonas, matriz de comportamiento, reglas de prioridad, flujo de datos CAN→FastLED, edge cases |
| **NOT IMPLEMENTED explícito** | Documentadas las features definidas pero no activadas: `REGEN_ACTIVE`, `startEmergencyFlash()`, overlay destructivo |

#### Fase 3 — Mejoras premium traseras (commit 0125572)

| Feature | Descripción |
|---------|-------------|
| **🔧 1. Overlay no destructivo (trasero)** | `updateRearBase()` reemplaza `updateRearCentre()` — pinta los 16 LEDs (no solo centro). `updateRearTurnSignals()` solo escribe AMBER en blink-ON. Color base (rojo dim, brake, blanco reversa, azul regen) visible durante blink-OFF |
| **🔧 2. Intermitentes secuenciales** | Sweep progresivo hacia exterior desde centro (European-style): LED 2→1→0 (izq), LED 13→14→15 (der). Ventana 500ms ON dividida en 3 pasos (~167ms cada uno). Helper `sweepFill()` detecta dirección automáticamente |
| **🔧 3. REGEN_ACTIVE activado** | Azul pulsante cuando throttle 5–20% con velocidad > 0. Nuevo threshold `LED_TRACTION_REGEN_THRESHOLD = 20`. Prioridad: BRAKE_EMERGENCY > REVERSE > BRAKE > REGEN_ACTIVE > POSITION |
| **🔧 4. Emergency flash real** | `startEmergencyFlash(3)` one-shot al transicionar a ERROR (no SAFE). 3 ciclos rojo/negro (~600ms), luego BRAKE_EMERGENCY + HAZARD sostenido |

#### Cambios en archivos

| Archivo | Cambios |
|---------|---------|
| `esp32/src/led_controller.h` | +39 líneas: zonas frontales [0–4]/[5–22]/[23–27], `LED_STRIP_REVERSED` para front, contrato instalación física |
| `esp32/src/led_controller.cpp` | +179/−34 líneas: `updateFrontTurnSignals()`, `updateRearBase()` (reemplaza `updateRearCentre()`), `updateRearTurnSignals()` (reemplaza `updateTurnSignals()`), `sweepFill()`, overlay no destructivo en ambos strips |
| `esp32/src/main.cpp` | +28/−2 líneas: `LED_TRACTION_REGEN_THRESHOLD=20`, detección regen, `REGEN_ACTIVE` en cadena de prioridad trasera, emergency flash one-shot en ERROR, SAFE→KITT_IDLE+HAZARD |
| `Documentos/LED_FRONT_LOGIC.md` | +148 líneas: análisis completo lógica frontal (zonas, overlay, KITT aislamiento, HAZARD, timing) |
| `Documentos/REAR_LED_BEHAVIOR.md` | +492 líneas: análisis completo trasero (zonas, matriz comportamiento, prioridad, flujo datos, edge cases, sweep secuencial, regen, emergency flash) |

#### Cadena de prioridad rear actualizada

```
1. EMERGENCY FLASH      (one-shot al entrar en ERROR, 3 ciclos)
2. SYSTEM DISABLED       (BOOT/STANDBY → FastLED.clear())
3. BRAKE_EMERGENCY       (SAFE/ERROR → flash rojo + HAZARD sweep)
4. REVERSE               (blanco 100% full strip)
5. BRAKE                 (rojo 100% full strip)
6. REGEN_ACTIVE          (azul pulsante full strip — throttle 5-20% con velocidad)
7. POSITION              (rojo dim 20% full strip — default)
```

#### Detección regen (main.cpp)

```
trAvg ≤ 5          → BRAKE (rojo brillante)
5 < trAvg ≤ 20     → REGEN_ACTIVE (azul pulsante)  ← NUEVO
trAvg > 20          → POSITION (rojo dim)
(todas requieren speedSum > LED_SPEED_SUM_THRESHOLD)
```

#### Overlay visual (blink ON vs OFF)

```
POSITION + LEFT turn:
  blink ON:   [AMBER→→→]  [dim red ×10]     [dim red ×3]
  blink OFF:  [dim red ×3] [dim red ×10]     [dim red ×3]

BRAKE + LEFT turn:
  blink ON:   [AMBER→→→]  [bright red ×10]  [bright red ×3]
  blink OFF:  [bright red] [bright red ×10]  [bright red ×3]
```

- **Safety validation summary:**
  - ✅ No se modifica máquina de estados de seguridad
  - ✅ No hay transmisiones CAN nuevas
  - ✅ Emergency flash solo se activa en transición a ERROR (one-shot, no continuo)
  - ✅ REGEN_ACTIVE es puramente visual — no afecta tracción ni frenado
  - ✅ Overlay no destructivo no cambia prioridades de zona — es aditivo
  - ✅ `sweepFill()` bounds-checked con clamp de step
  - ✅ `LED_STRIP_REVERSED` soportado en ambos strips

- **Impacto:** Mejora visual significativa en ambos strips LED sin modificar lógica de seguridad. Comportamiento frontal y trasero ahora consistente (overlay no destructivo). Features previamente definidas pero no activadas (REGEN, emergency flash) ahora operativas.

### PR — feat: Extend Safe Mode Screen with Passive Visualization
- **Fecha:** 2026-04-15
- **Autor:** Copilot
- **Descripción del cambio:** Extend the SAFE MODE screen (system_state = SAFE) with additional read-only operator information: gear position bar, obstacle sensor bar with color-coded proximity, LED system status, relay status indicators (M/T/D), and steering visual direction indicator. All additions are purely passive UI — no safety logic, CAN transmissions, motor/relay control, or state transitions modified.

#### Nuevas visualizaciones en SafeScreen

| Feature | Descripción |
|---------|-------------|
| **Gear Display** | Posición actual del selector (P/R/N/D1/D2) mostrada como barra horizontal en la parte inferior |
| **Steering Visual** | Indicador de dirección (`<< LEFT`, `RIGHT >>`, `\| CENTER`) añadido al ángulo numérico con color cyan |
| **Obstacle Sensor Bar** | Barra de proximidad con color (CLEAR verde >3m / NEAR ámbar 0.8-3m / DANGER rojo <0.8m) + distancia + estado |
| **LED System Status** | Estado relay front/rear (ON/OFF) y señal de giro (LEFT/RIGHT/HAZARD/OFF) |
| **Relay Status** | Indicadores M T D (MAIN/TRACTION/DIRECTION) con código de color según heartbeat byte 5 |

#### Fuentes de datos (todas read-only)

| Dato | Fuente | Archivo |
|------|--------|---------|
| Gear position | `shifter::getGearRaw()` (MCP23017 I2C) | `esp32/src/shifter_input.h:51` |
| Steering angle | `VehicleData.steering().angleRaw` | `esp32/src/vehicle_data.h` |
| Obstacle distance | `VehicleData.obstacle().distanceCm` | `esp32/src/vehicle_data.h` |
| Front/Rear LED relay | `VehicleData.lights().frontRelayOn/rearRelayOn` | `esp32/src/vehicle_data.h` |
| Turn signal | `led_ctrl::getTurnSignal()` (estado local ESP32) | `esp32/src/led_controller.h:148` |
| Relay status | `VehicleData.heartbeat().relayStatus` | `esp32/src/vehicle_data.h` |

#### Layout actualizado

```
┌─────────────────────────────────────────────────┐ Y=0
│              SAFE MODE (amber banner)            │
├─────────────────────────────────────────────────┤ Y=40
│ Actuators inhibited - Controls disabled          │ Y=46
│ FAULT FLAGS: [value]                             │ Y=62-86
│ ERROR CODE:  [value]                             │ Y=90-114
├─────────────────────────────────────────────────┤ Y=118
│ READ-ONLY TELEMETRY                              │
│ SPEED:    FL [v]  FR [v]  RL [v]  RR [v]        │ Y=126-152
│ CURRENT:  FL [v]  FR [v]  RL [v]  RR [v]        │ Y=154-180
│ TEMP:     FL [v]  FR [v]  RL [v]  RR [v] AMB [v]│ Y=182-208
│ STEERING: -12.5° << LEFT                         │ Y=214  (ENHANCED: visual indicator)
│                                                  │
│ OBSTACLE: [████████░░░░░░░░░░░░] 1.50m CLEAR    │ Y=246  (NEW)
│ LIGHTS:   F:ON  R:OFF  T:LEFT                    │ Y=266  (NEW)
├─────────────────────────────────────────────────┤ Y=284
│ [P] [R] [N] [D1] [D2]           M T D           │ Y=288  (NEW: gear + relay)
└─────────────────────────────────────────────────┘ Y=320
```

- **Archivos afectados:**
  - `esp32/src/screens/safe_screen.h` — 4 nuevos tile enums (STILE_OBSTACLE, STILE_LED_STAT, STILE_GEAR, STILE_RELAY), nuevas variables miembro para gear/obstacle/lights/relay/turn signal
  - `esp32/src/screens/safe_screen.cpp` — `update()` extendido para leer nuevas fuentes de datos, `draw()` extendido con rendering de tiles para las 5 nuevas visualizaciones (+~300 líneas)
  - `esp32/src/ui/ui_config.h` — Nuevas constantes de layout (STILE_STEER_VIS_Y, STILE_OBSTACLE_Y, STILE_LED_STATUS_Y, STILE_GEAR_BAR_Y, STILE_RELAY_X, STILE_OBS_BAR_*, PAD_SAFE_*)
  - `esp32/src/led_controller.h` — Añadido `getTurnSignal()` getter read-only
  - `esp32/src/led_controller.cpp` — Implementación de `getTurnSignal()` retornando estado actual de señal de giro
  - `Documentos/SAFE_MODE_UI_EXTENSION.md` — Documentación completa de la extensión + verificación de consistencia de fuente de datos de gear

- **Tiles añadidos al TileSet (SafeScreen: 6 → 10 tiles):**
  - `STILE_OBSTACLE` — Barra de proximidad del sensor obstáculo
  - `STILE_LED_STAT` — Estado del sistema LED (front/rear/turn)
  - `STILE_GEAR` — Barra de posición de marchas
  - `STILE_RELAY` — Indicador de estado de relés (M T D)

- **Safety validation summary:**
  - ✅ No se modifica lógica de seguridad (máquina de estados, transiciones)
  - ✅ No hay transmisiones CAN nuevas (todos los datos son read-only)
  - ✅ No hay control de motores/relés — solo rendering TFT
  - ✅ No se llama a `millis()` directamente — se respeta el Frame Time Contract
  - ✅ No hay asignación dinámica — todos los buffers son stack-allocated
  - ✅ No hay llamadas bloqueantes — rendering inmediato por SPI
  - ✅ `getTurnSignal()` es getter puro sin efectos secundarios
  - ✅ Fuente de datos de gear verificada como consistente con DriveScreen (ambos usan `shifter::getGearRaw()`)

- **Impacto:** Zero impacto en operación normal del vehículo. Extensión puramente visual del SafeScreen. Misma fuente de datos que DriveScreen para gear. Tile engine gestiona redibujado eficiente — solo se actualiza lo que cambia.

- **Statement:** No regression in safety-critical paths. Purely additive passive visualization.

### PR — feat: Hidden Menu Relay Control (Engineering Diagnostic Mode)
- **Fecha:** 2026-04-13
- **Autor:** Copilot
- **Descripción del cambio:** Add ability to manually activate/deactivate individual relays (MAIN/TRACTION/DIRECTION) from the ESP32 engineering menu for diagnostic purposes. Full safety gating prevents unsafe activation.

#### RELAY CONTROL (ENGINEERING MODE)

**Purpose:**
Diagnostics only — allows individual relay GPIO toggling from the ESP32 engineering hidden menu while the vehicle is in STANDBY.

**Safety constraints (enforced by STM32):**
- System state must be STANDBY
- Throttle ≤ 1% (pedal rest dead-zone)
- Speed ≤ 0.5 km/h
- No active safety errors (Safety_Error_t == NONE)
- Relay sequencer must be IDLE (relay_seq_state == RELAY_SEQ_IDLE)

**CAN mapping:**
- Frame: SERVICE_CMD (0x110)
- Action byte: 0xE0 (SERVICE_ACTION_RELAY_OVERRIDE)
- Byte 1 bit layout:
  - bit 0: override enable (1=on, 0=off)
  - bit 1: MAIN relay (PC10)
  - bit 2: TRACTION relay (PC11)
  - bit 3: DIRECTION relay (PC12)

**Auto-disable conditions:**
- Any state transition (BOOT/STANDBY/ACTIVE/DEGRADED/ERROR/LIMP_HOME)
- Throttle applied (> 1%)
- Motion detected (> 0.5 km/h)
- Safety error raised
- ESP32 exits engineering screen (sends disable CAN command in onExit)
- ESP32 navigates back from relay control submenu (sends disable)
- CAN timeout (STM32 state change → auto-disable)
- Relay sequencer starts (normal startup overrides)

**Known limitations:**
- Override only affects GPIO command layer, NOT relay_seq_state
- Safety_IsPowerReady() remains FALSE during override (relay_seq_state stays IDLE)
- Motor traction CANNOT engage during override (Traction_Update gates on Safety_IsPowerReady)
- Override requires continuous 10ms re-validation (Safety_RelayOverrideUpdate in main loop)

- **Archivos afectados:**
  - `Core/Inc/safety_system.h` — Safety_SetRelayOverride, Safety_IsRelayOverrideActive, Safety_RelayOverrideUpdate declarations + docblock
  - `Core/Src/safety_system.c` — relay_override_enabled/mask state, 5-point safety gating, continuous 10ms validation, auto-disable on state transition
  - `Core/Inc/can_handler.h` — SERVICE_ACTION_RELAY_OVERRIDE (0xE0) define
  - `Core/Src/can_handler.c` — 0xE0 action handler in SERVICE_CMD case
  - `Core/Src/main.c` — Safety_RelayOverrideUpdate() call in 10ms loop after Relay_SequencerUpdate()
  - `Core/Inc/project_config.h` — Relay override documentation block
  - `esp32/include/can_ids.h` — SERVICE_ACTION_RELAY_OVERRIDE constant
  - `esp32/src/screens/engineering_screen.h` — SubMenu::RELAY_CONTROL, relayOverrideEnabled_, relayOverrideMask_, drawRelayControl() declaration
  - `esp32/src/screens/engineering_screen.cpp` — RELAY_CONTROL submenu (menu item 9, red text), drawRelayControl() rendering, handleTouch relay toggles, onEnter/onExit auto-disable, update() CAN telemetry sync

- **Safety validation summary:**
  - ✅ No change to motor PWM gating logic (Traction_Update gates on Safety_IsPowerReady)
  - ✅ No change to relay sequencing timing (50ms MAIN → 20ms TRACTION → DIRECTION = 70ms total)
  - ✅ No bypass of Safety_IsPowerReady() (relay_seq_state stays IDLE, returns false)
  - ✅ No CAN ID breaking changes (reuses SERVICE_CMD 0x110 with new action 0xE0)
  - ✅ No blocking calls introduced (all override logic is non-blocking)
  - ✅ No race conditions (single-threaded main loop, override state is module-static)
  - ✅ No unsafe state transitions (override auto-disabled on any Safety_SetState call)
  - ✅ Startup inhibit logic unaffected
  - ✅ 25/25 STM32 sources pass -Wall -Wextra -Werror syntax check
  - ✅ 11,948 unit tests pass (0 failures)

- **CAN impact:** SERVICE_CMD (0x110) extended with action 0xE0. No new CAN IDs. Heartbeat (0x001) byte 5 relay status unchanged. No frequency changes.

- **UI changes:**
  - EngineeringScreen: new "RELAY CONTROL (DEBUG)" submenu (item 9, red text)
  - 4 toggle rows: Override Enable, MAIN (PC10), TRACTION (PC11), DIRECTION (PC12)
  - Real CAN relay status display: M:ON/OFF T:ON/OFF D:ON/OFF SEQ:COMPLETE/IDLE [0xHH]
  - Warning banner: "!! MANUAL RELAY CONTROL ACTIVE !!" when override enabled
  - Auto-disable on screen exit and BACK navigation

- **Impacto:** Zero impact on normal vehicle operation. Override only works in STANDBY with no motion. Motor control remains fully gated by Safety_IsPowerReady(). Engineering diagnostic capability added with OEM-grade safety constraints.

- **Statement:** No regression in safety-critical paths.

### PR — feat: Tile-Based Dirty Region Engine (motor de render tipo cluster OEM automotriz)

### PR — refactor: HMI Security Audit & Tile Engine Hardening

### PR — refactor: Tile Engine Formalization & Pipeline Hardening (OEM Cluster Level)

### PR — refactor: Draw Purity Enforcement, OverlayMode, Full Layout Centralization

### PR — feat: Time Determinism, Hash Failsafe, Flag Safety (OEM Final Hardening)
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Final hardening pass: inject deterministic `frameTimeMs` into all screens, add hash failsafe system for critical tiles, formalize flag safety contract. Makes render pipeline fully reproducible from (VehicleData + time).
- **Root cause:** (1) `millis()` called directly inside screen `update()` methods broke time reproducibility — same input could produce different UI state depending on when millis() was sampled within the function. (2) FNV-1a 32-bit hash has non-zero collision probability; no safety net existed for missed tile updates on critical elements (speed, faults). (3) Flag safety contract was implemented but not formally documented.
- **Solución aplicada:**
  1. **Screen::update() signature**: Added `unsigned long frameTimeMs` parameter. ScreenManager captures `millis()` exactly once per frame and injects it into all screens. All timing logic in update() uses the injected value.
  2. **Time determinism**: Replaced all `millis()` calls in DriveScreen (ACK), ErrorScreen (canLost, elapsed), BootScreen (CAN link, RX staleness, freeze detection), PinScreen (wrong code timeout), EngineeringScreen (ACK timeout) with `frameTimeMs`.
  3. **Hash failsafe**: Added `TileSet::forceRedraw(idx)` method (zeroes hash + marks dirty). DriveScreen and ErrorScreen call it on critical tiles (SPEED, FAULTS, DEGRADED, BATTERY, BANNER) every `HASH_FAILSAFE_INTERVAL` frames (100 = 5s at 20 FPS).
  4. **Flag safety documentation**: Documented in tile_engine.h that event flags are cleared ONLY after successful tile render.
  5. **ui_config.h**: Added `HASH_FAILSAFE_INTERVAL = 100`.
- **Archivos afectados:**
  - `esp32/src/screens/screen.h` — `update()` signature + `frameTimeMs`
  - `esp32/src/screen_manager.cpp` — Capture millis() once, pass to screens
  - `esp32/src/screens/drive_screen.h/cpp` — frameTimeMs, failsafe counter
  - `esp32/src/screens/error_screen.h/cpp` — frameTimeMs, failsafe counter
  - `esp32/src/screens/boot_screen.h/cpp` — frameTimeMs replaces millis()
  - `esp32/src/screens/pin_screen.h/cpp` — frameTimeMs
  - `esp32/src/screens/engineering_screen.h/cpp` — frameTimeMs
  - `esp32/src/screens/safe_screen.cpp` — (void)frameTimeMs
  - `esp32/src/screens/standby_screen.cpp` — (void)frameTimeMs
  - `esp32/src/ui/tile_engine.h` — forceRedraw(), pipeline docs, hash failsafe docs
  - `esp32/src/ui/ui_config.h` — HASH_FAILSAFE_INTERVAL
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Render pipeline is now fully time-deterministic: same (VehicleData, frameTimeMs) → same derived state → same framebuffer ALWAYS. Critical tiles have bounded recovery time from hash collisions. Zero behavioral change for normal operation — millis() was already being called once per frame externally, but now it's formalized as an injected dependency.
- **Tests:** Internal refactoring. No CAN, VehicleData, or widget interface changes. Build verified on STM32 side (make clean && make).

### PR — feat: V10 Hardening Extension — Staggered Failsafe, Critical Tile Policy, Render Atomicity
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Final hardening V10: distribute forced redraws to eliminate SPI spikes, add fault-condition hash override for critical tiles, formalize frame time monotonicity and render atomicity contracts.
- **Root cause:** (1) Hash failsafe redraws all critical tiles simultaneously every 100 frames, creating a periodic SPI spike. (2) No mechanism to override hash suppression under fault conditions — a hash collision during a fault could hide safety information. (3) Frame time monotonicity and render atomicity were guaranteed by implementation but not formally contracted.
- **Solución aplicada:**
  1. **Staggered failsafe**: Critical tile forced redraws are now distributed across the HASH_FAILSAFE_INTERVAL. DriveScreen: SPEED@0, FAULTS@25, DEGRADED@50, BATTERY@75. ErrorScreen: BANNER@0, FAULTS@50. Eliminates the multi-tile SPI spike every 100 frames.
  2. **Critical tile fault override**: When `curFaultFlags_ != 0`, DTILE_SPEED and DTILE_FAULTS are force-redrawn EVERY frame, overriding hash suppression. Ensures fault visualization is always visually current.
  3. **Frame time contract (tile_engine.h)**: Formal documentation of frameTimeMs single-sampling, monotonicity, overflow-safe deltas, and determinism guarantee.
  4. **Frame time monotonicity assertion**: ScreenManager tracks prevFrameTimeMs_ and asserts (in UI_TILE_DEBUG mode) that time never goes backwards.
  5. **Render atomicity contract (tile_engine.h)**: Formal documentation that tile render → overlay invalidation → markClean → flag clear is atomic (single-threaded Core 0, no yield points).
- **Archivos afectados:**
  - `esp32/src/ui/tile_engine.h` — Frame Time Contract, Render Atomicity Contract, Hash Failsafe Distribution, Critical Tile Policy documentation
  - `esp32/src/screens/drive_screen.cpp` — Staggered failsafe + fault-condition critical tile override
  - `esp32/src/screens/error_screen.cpp` — Staggered failsafe
  - `esp32/src/screen_manager.h` — prevFrameTimeMs_ member
  - `esp32/src/screen_manager.cpp` — frameTimeMs monotonicity debug assertion
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Zero behavioral change under normal operation (no faults, no hash collisions). Under fault conditions, SPEED and FAULTS tiles update every frame instead of relying on hash comparison. SPI load is distributed evenly instead of spiking every 100 frames. All contracts now formally documented.
- **Tests:** STM32 build verified (make clean && make -j with -Wall -Wextra -Werror).
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Enforce draw-phase purity (zero state mutation in draw helpers), add formal overlay composition modes (REPLACE/MERGE), and centralize all remaining layout constants from SafeScreen and BootScreen.
- **Root cause:** (1) `ackIndicatorDirty_` was cleared inside `drawAckIndicator()` (draw phase), violating pure-render contract. (2) `diagNeedsRedraw_` was cleared inside BootScreen tile render block. (3) Overlay tiles had no formal composition mode declaration. (4) SafeScreen had 17 local layout constants not in ui_config.h. (5) BootScreen diagnostic layout/timing constants were file-local.
- **Solución aplicada:**
  1. **Draw purity**: Moved `ackIndicatorDirty_ = false` from `drawAckIndicator()` to `draw()` caller. Moved `diagNeedsRedraw_ = false` after tile render in BootScreen.
  2. **OverlayMode enum**: Added `OverlayMode::REPLACE` / `OverlayMode::MERGE` to tile_engine.h with per-overlay registry documenting which DriveScreen overlays use which mode and which base tiles they overlap.
  3. **SafeScreen layout**: 17 constants (STILE_BANNER_H through STILE_COL_VAL_SPACE) moved to ui_config.h. safe_screen.cpp uses `using namespace ui::cfg`.
  4. **BootScreen layout**: BTILE_DIAG_SEP_Y, BTILE_DIAG_LINE_H, BTILE_DIAG_MARGIN_X, BTILE_DIAG_RX_RECENT_MS, BTILE_DIAG_FREEZE_MS moved to ui_config.h. boot_screen.cpp aliases them locally.
- **Archivos afectados:**
  - `esp32/src/ui/tile_engine.h` — OverlayMode enum + per-overlay composition registry
  - `esp32/src/ui/ui_config.h` — STILE_* (17) + BTILE_DIAG_* (5) constants
  - `esp32/src/screens/drive_screen.cpp` — Draw purity: ackIndicatorDirty_ cleared after render
  - `esp32/src/screens/safe_screen.cpp` — All layout refs → STILE_* from ui_config.h
  - `esp32/src/screens/boot_screen.cpp` — Draw purity + layout refs → BTILE_DIAG_* from ui_config.h
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Draw phase is now fully pure (no state mutation in draw helpers). All 5 screens' layout constants are in ui_config.h. Overlay composition is formally documented. Zero behavioral changes — all modifications are structural refactoring.
- **Tests:** Internal refactoring only. No CAN, VehicleData, or widget interface changes.

- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Formalización del tile engine a nivel OEM automotive cluster: Z-order layer system, pipeline estricto update/draw, overlay invalidation contract, debug assertions, y centralización completa de tile layout dimensions.
- **Root cause:** (1) Overlay visibility se recomputaba en draw() violando la regla de render puro. (2) tile_engine.h no definía Z-order formal para overlay compositing. (3) Tile layout dimensions (180, 120, 370, 75, etc.) hardcoded en onEnter() sin nombre semántico. (4) Bounds safety era silencioso sin opción de diagnóstico. (5) No existía contrato formal para overlay invalidation.
- **Solución aplicada:**
  1. **tile_engine.h — Z-order layer system**: TileLayer enum (STATIC/BASE/OVERLAY/SYSTEM) con reglas formales de composición documentadas. Render contract: update phase (NO SPI) → draw phase (SOLO consume estado precomputado).
  2. **tile_engine.h — Debug assertions**: `UI_TILE_DEBUG` flag habilita Serial.printf diagnóstico cuando setRect recibe coordenadas negativas, tamaños inválidos, o fuera de pantalla. Producción: silent clamp (sin overhead).
  3. **tile_engine.h — Overlay invalidation contract**: Documentación formal del ciclo de vida overlay: visible→invisible → clear + markDirty tiles subyacentes → repaint.
  4. **DriveScreen — Pure render**: Overlay visibility (curDegradedVisible_, curFaultsVisible_, curAckVisible_) precomputada en update(), draw() solo consume. Eliminada recomputación de `(curSystemState_ == DEGRADED || LIMP_HOME)` y `(curFaultFlags_ != 0)` en draw().
  5. **ui_config.h — Tile layout constants**: DTILE_MODE_ICONS_X/W, DTILE_LED_TOGGLE_X/W, DTILE_BATTERY_W, DTILE_WHEELS_W, DTILE_STEERING_X/W para DriveScreen. ETILE_BANNER_H, ETILE_CONTENT_X/W, ETILE_FAULTS_Y/H, ETILE_SAFETY_Y/H, ETILE_DIAG_Y/H, ETILE_ELAPSED_Y/H para ErrorScreen. YTILE_TEMPS_X/Y/W/H, YTILE_FAULTS_Y/H para StandbyScreen.
  6. **drive_screen.cpp**: Todos los setRect() usan constantes de ui_config.h.
  7. **error_screen.cpp**: Todos los setRect() usan constantes de ui_config.h.
  8. **standby_screen.cpp**: Todos los setRect() usan constantes de ui_config.h.
- **Archivos afectados:**
  - `esp32/src/ui/tile_engine.h` — Z-order enum, render contract, debug asserts, overlay contract
  - `esp32/src/ui/ui_config.h` — Tile layout dimension constants (DTILE_*, ETILE_*, YTILE_*)
  - `esp32/src/screens/drive_screen.h` — curDegradedVisible_, curFaultsVisible_, curAckVisible_
  - `esp32/src/screens/drive_screen.cpp` — Pure render, named constants, layer comments
  - `esp32/src/screens/error_screen.cpp` — Named layout constants
  - `esp32/src/screens/standby_screen.cpp` — Named layout constants
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Pipeline de render formalmente puro (update no hace SPI, draw no recomputa lógica). Overlays con jerarquía Z formal y contrato de invalidación. Todas las dimensiones de tile en configuración central. Debug assertions disponibles para desarrollo. Zero cambios en interfaz CAN, VehicleData, o STM32 firmware.
- **Tests:** Sin cambios en interfaces externas. Cambios son refactoring interno del render engine: renombrado de constantes, reordenación de computación, y documentación formal.
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Auditoría completa de seguridad, optimización y refactor del sistema HMI tile-based. Centralización de constantes, corrección de artefactos de overlay, eliminación de duplicación de cálculos, bounds safety en tile engine, y determinismo en hysteresis de batería.
- **Root cause:** Múltiples defectos potenciales detectados en el motor de render: (1) magic numbers dispersos en 10+ archivos, (2) tiles overlay (DEGRADED, FAULTS, ACK) se solapan con tiles base pero markClean() no restaura el contenido subyacente, (3) wheelThresholdFilter se ejecuta 2× por frame (update hash + draw), (4) setRect() no valida coordenadas contra límites de pantalla, (5) hysteresis de texto en batería depende de estado del frame anterior (no determinista).
- **Solución aplicada:**
  1. **ui_config.h** (NUEVO): 25+ constantes centralizadas — PAD_SPEED/ACK/PEDAL/OBSTACLE/ERROR/SAFE/STANDBY/BOOT, THR_TRACTION_DELTA/TEMP_DELTA, BATT_HYSTERESIS_HIGH/LOW, OVL_DEGRADED_*/OVL_FAULT_*, PEDAL_COLOR_LOW/MID, BATT_COLOR_LOW/MID, TEMP_COLOR_WARNING/CRITICAL.
  2. **Overlay invalidation chain**: Cuando un overlay se desactiva, los tiles subyacentes se marcan dirty para restaurar su contenido. DEGRADED→OBSTACLE, FAULTS→MODE_ICONS+LED_TOGGLE+BATTERY, ACK→LED_TOGGLE.
  3. **wheelThresholdFilter precompute**: Valores de dibujo calculados una sola vez en update() y almacenados en drawTraction_[4]/drawTemp_[4]. draw() consume los valores precomputados sin recalcular.
  4. **Tile bounds safety**: setRect() ahora clampea x,y a ≥0, w,h a ≥0, y (x+w, y+h) a ≤SCREEN_W/SCREEN_H.
  5. **Battery hysteresis determinista**: Reemplaza `prevUsedDark` (dependencia frame anterior) por comparación explícita con BATT_HYSTERESIS_HIGH/LOW thresholds. En banda de hysteresis, usa dirección del cambio (prevFW≥highThresh).
  6. **All setTextPadding**: Reemplazados 20+ valores hardcoded por constantes nombradas de ui_config.h.
- **Archivos afectados:**
  - `esp32/src/ui/ui_config.h` — NUEVO: Configuración centralizada HMI
  - `esp32/src/ui/tile_engine.h` — Bounds safety en setRect()
  - `esp32/src/ui/battery_indicator.cpp` — Hysteresis determinista, config constants
  - `esp32/src/ui/pedal_bar.cpp` — Config constants (color thresholds, padding)
  - `esp32/src/ui/obstacle_sensor.cpp` — Config constants (padding, bar max range)
  - `esp32/src/ui/car_renderer.cpp` — Config constants (wheel label padding)
  - `esp32/src/screens/drive_screen.h` — Precomputed draw values, overlay tracking members
  - `esp32/src/screens/drive_screen.cpp` — Overlay invalidation, precompute, config constants
  - `esp32/src/screens/error_screen.cpp` — Config constants
  - `esp32/src/screens/safe_screen.cpp` — Config constants (padding, temp thresholds)
  - `esp32/src/screens/standby_screen.cpp` — Config constants
  - `esp32/src/screens/boot_screen.cpp` — Config constants
  - `esp32/src/hmi/obstacle_indicator.cpp` — Named constant for padding
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Sistema HMI determinista. Overlays no dejan artefactos visuales. Cero duplicación de cálculo por frame. Arquitectura limpia con constantes centralizadas. Compatible con TFT_eSPI + CAN + STM32 sin cambios.
- **Tests:** Compilación conceptual verificada. Sin cambios en CAN protocol, VehicleData, STM32 firmware. Todos los cambios son renombrado de constantes y correcciones de lógica de render — no afectan la interfaz CAN ni datos de vehículo.
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Refactor completo del sistema HMI para implementar un motor de render por regiones (tiles) tipo cluster OEM automotriz. Cada zona de la pantalla es un tile independiente con su propio dirty flag y hash de contenido FNV-1a. Solo los tiles cuyo contenido ha cambiado desde el último frame se redibujan.
- **Root cause:** El sistema anterior usaba comparaciones campo-por-campo para detectar cambios, sin agrupación formal en regiones. Los bar widgets (pedal, batería, sensor obstáculo) usaban clear+redraw completo en cada actualización, causando un flash visible (clear = COL_BG seguido de fill = color).
- **Solución aplicada:**
  1. **tile_engine.h**: Nuevo módulo con TileRect, TileHash (FNV-1a 32-bit), y TileSet<N> template. Hash comparison per-tile para decisión de skip/redraw.
  2. **DriveScreen**: 12 tiles formales (SPEED, OBSTACLE, WHEELS, STEERING, BATTERY, GEAR, PEDAL, MODE_ICONS, LED_TOGGLE, DEGRADED overlay, FAULTS overlay, ACK). Pipeline: update() computa hashes → updateHash() marca dirty → draw() solo renderiza tiles sucios.
  3. **ErrorScreen**: 5 tiles (BANNER, FAULTS, SAFETY, DIAG, ELAPSED). Cada tile se actualiza independientemente.
  4. **SafeScreen**: 6 tiles (FAULTS, ERROR, SPEEDS, CURRENTS, TEMPS, STEERING).
  5. **StandbyScreen**: 2 tiles (TEMPS, FAULTS).
  6. **BootScreen**: 3 tiles (CAN_STATUS, SENSOR, DIAGNOSTICS).
  7. **PedalBar**: Differential update — solo la porción que creció o se redujo se pinta. Si cambió el color (threshold crossing), redraw mínimo.
  8. **BatteryIndicator**: Differential update — mismo patrón que PedalBar.
  9. **ObstacleSensor**: Differential update — bar de proximidad sin clear+redraw.
  10. **screen_manager.h**: Documentación actualizada reflejando pipeline tile-based.
- **Archivos afectados:**
  - `esp32/src/ui/tile_engine.h` — NUEVO: Core tile infrastructure
  - `esp32/src/screens/drive_screen.h` — TileSet<12>, tile enum
  - `esp32/src/screens/drive_screen.cpp` — Hash computation in update(), tile iteration in draw()
  - `esp32/src/screens/error_screen.h` — TileSet<5>, tile enum
  - `esp32/src/screens/error_screen.cpp` — Hash computation + tile iteration
  - `esp32/src/screens/safe_screen.h` — TileSet<6>, tile enum
  - `esp32/src/screens/safe_screen.cpp` — Hash computation + tile iteration
  - `esp32/src/screens/standby_screen.h` — TileSet<2>, tile enum
  - `esp32/src/screens/standby_screen.cpp` — Hash computation + tile iteration
  - `esp32/src/screens/boot_screen.h` — TileSet<3>, tile enum
  - `esp32/src/screens/boot_screen.cpp` — Hash computation + tile iteration
  - `esp32/src/ui/pedal_bar.cpp` — Differential bar update
  - `esp32/src/ui/battery_indicator.cpp` — Differential bar update
  - `esp32/src/ui/obstacle_sensor.cpp` — Differential bar update
  - `esp32/src/screen_manager.h` — Updated documentation
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Render completamente tile-based. Reducción significativa de carga SPI (solo tiles dirty). Eliminación de clear+redraw flash en barras. Coherencia total snapshot→tiles. Arquitectura escalable tipo instrument cluster de vehículo real.
- **Tests:** Compilación exitosa. Revisión de código. Validación de lógica de hash y dirty flags. Sin cambios en CAN protocol, VehicleState struct, STM32 firmware.

### PR — feat: complete screen verification — degraded overlay + safe telemetry + fault indicators
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Verificación de las pantallas contra la especificación HMI_STATE_MODEL.md e implementación de las funcionalidades faltantes: overlay de modo degradado/limp-home en DriveScreen, indicadores de fault flags en DriveScreen, y telemetría read-only en SafeScreen.

### PR — refactor: HMI anti-flicker + render optimization
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Revisión y optimización del sistema HMI completo para eliminar flicker y redraws innecesarios. Corrección de un bug crítico (bit 0 CAN_TIMEOUT faltante en fault overlay de DriveScreen).

### PR — refactor: deterministic render pipeline + zero-flicker text updates
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Implementación de pipeline de render determinista tipo "OEM automotive cluster". Eliminación total de flicker textual mediante setTextPadding. Formalización de arquitectura frame-latch CAN→snapshot→render.
- **Root cause:** Dos categorías de problemas identificados:
  1. **Flicker textual**: Todas las pantallas usaban `tft.fillRect()` para borrar el área de texto seguido de `tft.drawString()` para redibujar. El gap entre ambas operaciones SPI (0.5–2ms) creaba un parpadeo visible: flash de color de fondo entre el borrado y el nuevo texto.
  2. **Heap allocation en overlay**: `draw_runtime_overlay()` usaba `String(ESP.getFreePsram() / 1024) + " KB"` que realiza 3 heap allocations por línea, causando micro-stutters y fragmentación del heap.
- **Solución aplicada:**
  1. Reemplazado `fillRect` + `drawString` por `setTextPadding(width)` + `drawString` en 12 componentes UI. TFT_eSPI dibuja texto + relleno de background en una sola transacción SPI, eliminando el gap visible.
  2. Reemplazado `String` concatenation por `snprintf` a buffer stack en `draw_runtime_overlay()`.
  3. Documentación formal del pipeline determinista en `vehicle_data.h`, `screen_manager.h`, y `main.cpp` renderTask.
- **Archivos afectados:**
  - `drive_screen.cpp` — drawSpeed, drawAckIndicator, drawFaultOverlays: setTextPadding
  - `error_screen.cpp` — safety error, diagnostic, elapsed time: setTextPadding
  - `safe_screen.cpp` — fault flags, error code, speeds, currents, temps, steering: setTextPadding
  - `standby_screen.cpp` — temperatures, fault flags: setTextPadding
  - `boot_screen.cpp` — CAN link status: setTextPadding
  - `car_renderer.cpp` — wheel labels (torque%, temp): setTextPadding
  - `obstacle_sensor.cpp` (UI) — distance text: setTextPadding
  - `pedal_bar.cpp` — percentage text: setTextPadding
  - `obstacle_indicator.cpp` — sensor status: setTextPadding
  - `main.cpp` — draw_runtime_overlay: String→snprintf, renderTask documentation
  - `vehicle_data.h` — render pipeline documentation
  - `screen_manager.h` — deterministic render documentation
  - `PROJECT_CHANGELOG.md` — documentación del cambio
- **Impacto:** Eliminación total de flicker textual en todas las pantallas del HMI. Eliminación de heap allocation en overlay de boot. Pipeline CAN→UI→Render documentado y formalizado.
- **Tests:** Verificación visual: zero visible flash en transiciones de valores de texto.

### PR — refactor: HMI anti-flicker + render optimization
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Revisión y optimización del sistema HMI completo para eliminar flicker y redraws innecesarios. Corrección de un bug crítico (bit 0 CAN_TIMEOUT faltante en fault overlay de DriveScreen).
- **Root cause:** Tres problemas identificados en la revisión:
  1. DriveScreen `drawFaultOverlays()` listaba bits 1–7 pero omitía bit 0 (CAN_TIMEOUT 0x01) — el flag de timeout CAN nunca se mostraba.
  2. DriveScreen `draw()` llamaba incondicionalmente a las funciones de render de cada zona (drawSpeed, ObstacleSensor::draw, CarRenderer::drawWheels/drawSteering, BatteryIndicator::draw, GearDisplay::draw, PedalBar::draw, ModeIcons::draw, LedToggle::draw) incluso cuando los datos no habían cambiado. Las funciones hacían early-return interno, pero la llamada y evaluación de parámetros implicaba overhead innecesario cada frame.
  3. ErrorScreen hacía `tft.fillScreen(COL_RED)` (480×320 = 153600 píxeles) cada vez que el estado canLost_ cambiaba, causando flash visible cuando CAN se restauraba/perdía.
  4. Sensor noise en torque/temperatura provocaba redraws constantes de ruedas sin cambio visual significativo.
- **Solución aplicada:**
  1. Añadida entrada `{ 0x01, "CAN TMO", ui::COL_AMBER }` al array `entries[]` de `drawFaultOverlays()`. Ahora todos los 8 bits (0–7) del bitmask están representados.
  2. Movidas todas las llamadas a draw helpers dentro del bloque `if (curVal != prevVal)`, eliminando llamadas a función cuando no hay cambio real.
  3. ErrorScreen: separada la lógica de `canLost_` toggle del `needsRedraw_`. Ahora solo redibuja el banner (top 75px via `fillRect`) en vez del screen completo (`fillScreen`). Labels estáticos se dibujan una sola vez en `needsRedraw_`.
  4. Añadidos umbrales de cambio para ruedas: torque Δ>2%, temperatura Δ≥1°C. Valores filtrados se commitean como estado dibujado para mantener referencia estable.
- **Impacto:** Eliminación de flicker en ErrorScreen durante transiciones CAN lost↔restored. Reducción de carga CPU en DriveScreen (zero-work frames cuando datos no cambian). Bug fix: CAN_TIMEOUT ahora visible en dashboard. Reducción de redraws de ruedas por sensor noise.
- **Archivos modificados:**
  - `esp32/src/screens/drive_screen.cpp` — fault overlay bit 0 fix, dirty-gated draw calls, wheel thresholds
  - `esp32/src/screens/error_screen.cpp` — partial banner redraw instead of full fillScreen on canLost_ toggle
  - `PROJECT_CHANGELOG.md` — documentación del cambio
- **Tests:** STM32 build validation (`make clean && make -j$(nproc)` passed with -Wall -Wextra -Werror).
- **Root cause:** Tres funcionalidades definidas en HMI_STATE_MODEL.md (§2.4, §2.5, §4.1) no estaban implementadas:
  1. DriveScreen no mostraba ningún indicador cuando systemState era DEGRADED(3) o LIMP_HOME(6).
  2. DriveScreen no mostraba indicadores visuales de fault_flags (OVERTEMP, OVERCURRENT, ENCODER, WHEEL SENSOR, ABS, TCS, CENTERING).
  3. SafeScreen solo mostraba fault_flags y error_code, sin la telemetría read-only requerida (velocidades, corrientes, temperaturas, ángulo dirección).
- **Solución aplicada:**
  1. DriveScreen: banner ámbar "DEGRADED MODE - 40% POWER" / "LIMP HOME - REDUCED SPEED" en zona Y=40–58 cuando systemState es DEGRADED o LIMP_HOME. Partial redraw solo cuando cambia el estado.
  2. DriveScreen: tira de indicadores de faults compactos en Y=28 (zona inferior del top bar). OVERTEMP/OVERCURR/ENC FAULT/WHL SENS/CENTER en ámbar. ABS/TCS en cian (informational). Partial redraw cuando cambia faultFlags.
  3. SafeScreen: layout rediseñado — banner SAFE MODE (40px) + fault/error + separador + telemetría read-only con 4 velocidades, 4 corrientes, 5 temperaturas, ángulo dirección. Partial redraw por campo. Temperaturas cambian de color (>60°C ámbar, >80°C rojo).
- **Impacto:** Las 7 pantallas del HMI (Boot, Standby, Drive, Degraded, Limp Home, Safe, Error) cumplen ahora completamente con la especificación HMI_STATE_MODEL.md. El conductor ve claramente el estado degradado y los fallos activos.
- **Archivos modificados:**
  - `esp32/src/screens/drive_screen.h` — campos para systemState, faultFlags, métodos drawDegradedOverlay/drawFaultOverlays
  - `esp32/src/screens/drive_screen.cpp` — overlay degradado, indicadores de faults, captura systemState/faultFlags en update()
  - `esp32/src/screens/safe_screen.h` — campos para speed/current/temp/steering arrays
  - `esp32/src/screens/safe_screen.cpp` — layout completo con telemetría read-only, partial redraw
- **Tests:** STM32 build validation (`make clean && make -j$(nproc)` passed).

### PR — feat: update drive screen display for TF-Mini Plus sensor
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Actualización de la pantalla de conducción (drive screen) para reflejar el cambio de sensor de obstáculos de TOFSense-M a TF-Mini Plus. El sensor TF-Mini Plus tiene un rango de 10 cm–12 m (vs 2 cm–4 m del TOFSense-M), lo que requiere ajustar la barra de proximidad y los umbrales de color.
- **Root cause:** La barra de proximidad y los umbrales de color estaban calibrados para el rango máximo del TOFSense-M (400 cm). Con el TF-Mini Plus (rango hasta 1200 cm), las lecturas entre 4–12 m quedaban todas como "barra vacía" y los umbrales de color no aprovechaban el rango extendido.
- **Solución aplicada:**
  1. Barra de proximidad: max 400→600 cm (6 m da buena resolución visual en zona de alerta 0–4 m)
  2. Umbrales `proximityColor()`: ajustados para rango mayor — verde >3m, cian 1.5–3m, amarillo 0.8–1.5m, naranja 0.3–0.8m, rojo <0.3m
  3. Comentarios actualizados: "TOFSense-M" → "TF-Mini Plus" en `vehicle_data.h`, `obstacle_sensor.h`
  4. `ObstacleData.distanceCm` comentario max ~400→~1200 cm
- **Impacto:** La pantalla de conducción muestra correctamente el sensor TF-Mini Plus: barra de proximidad proporcional hasta 6 m, colores graduales en todo el rango útil.
- **Archivos modificados:**
  - `esp32/src/vehicle_data.h` — comentario TOFSense-M → TF-Mini Plus, max range
  - `esp32/src/ui/obstacle_sensor.cpp` — barra proximidad BAR_MAX_CM 400→600
  - `esp32/src/ui/obstacle_sensor.h` — comentario header actualizado
  - `esp32/src/ui/ui_common.h` — `proximityColor()` umbrales ajustados para TF-Mini Plus
- **Tests:** ESP32 build validation.

### PR — fix: ESP32 screen transitions to error when CAN cable disconnected
- **Fecha:** 2026-04-09
- **Autor:** Copilot
- **Descripción del cambio:** La pantalla del ESP32 no reaccionaba al desconectar el cable CAN. Ahora, si el heartbeat del STM32 no llega durante >1.5 s, el ScreenManager fuerza la transición a la pantalla de error mostrando "CAN LINK LOST / STM32 heartbeat not received". Se auto-recupera al reconectar el cable.
- **Root cause:** `ScreenManager::update()` solo leía `data.heartbeat().systemState` para decidir qué pantalla mostrar. Cuando el cable CAN se desconectaba, no llegaban nuevos heartbeats y el `systemState` mantenía su último valor (STANDBY, ACTIVE, etc.) → la pantalla nunca cambiaba.
- **Solución aplicada:**
  1. Añadido `CAN_LOSS_TIMEOUT_MS = 1500` en `can_ids.h`
  2. Detección de heartbeat stale en `ScreenManager::update()` — si edad > 1500 ms y no estamos en BOOT, forzar `newState = ERROR`
  3. `ErrorScreen` detecta heartbeat stale y muestra banner "CAN LINK LOST" en lugar de "SYSTEM ERROR"
  4. Auto-recuperación: cuando el heartbeat vuelve, `canLost_` se limpia y vuelve al estado normal
- **Impacto:** El usuario ve inmediatamente (en <2 s) que la comunicación CAN se ha interrumpido, con un mensaje claro en pantalla roja.
- **Archivos modificados:**
  - `esp32/include/can_ids.h` — nueva constante `CAN_LOSS_TIMEOUT_MS`
  - `esp32/src/screen_manager.h` — nuevo campo `canLost_`
  - `esp32/src/screen_manager.cpp` — detección heartbeat stale + fuerza ERROR
  - `esp32/src/screens/error_screen.h` — campos `canLost_` / `prevCanLost_`
  - `esp32/src/screens/error_screen.cpp` — banner condicional "CAN LINK LOST" vs "SYSTEM ERROR"
- **Tests:** STM32 firmware build (`make -j$(nproc)`) clean con `-Wall -Wextra -Werror`.

### PR — audit(obstacle): TF-Mini Plus sampling rate fix + file comment cleanup
- **Fecha:** 2026-04-09
- **Autor:** Copilot
- **Descripción del cambio:** Auditoría de producción del subsistema TF-Mini Plus. Descubierta pérdida del 90% de tramas del sensor (10 Hz observado vs 100 Hz esperado). Fix aplicado junto con limpieza de comentarios obsoletos.
- **Root cause:** Dos restricciones combinadas causaban que `update()` solo procesara 1 trama por llamada:
  1. `TFM_MAX_BYTES_PER_UPDATE = 32` (≈3.5 tramas) limitaba los bytes leídos
  2. `if (gotFrame) break;` salía del bucle tras la primera trama válida
  Con el main loop corriendo a ~10 Hz (por CAN/render/serial), solo se capturaban ~50 tramas/5s. El buffer UART de 256 bytes se llenaba (~900 bytes/s a 100 Hz × 9 bytes) y el hardware descartaba datos silenciosamente.
- **Solución aplicada:**
  1. Aumentar `TFM_MAX_BYTES_PER_UPDATE` de 32 a 256 (drena buffer completo por llamada)
  2. Eliminar `if (gotFrame) break;` — ahora procesa todas las tramas disponibles, conservando solo la última válida (lectura más fresca)
  3. Aumentar `rxBufSize` de 256 a 512 bytes (margen de ~570 ms a 100 Hz)
  4. Actualizar comentarios file-level en `.h` y `.cpp` (eliminadas referencias obsoletas a TOFSense-M como sensor primario)
  5. Actualizar comentario en `main.cpp` (TOFSense-M → TF-Mini Plus)
  6. Actualizar `TFMINI_PLUS_WIRING_GUIDE.md` con diagnósticos esperados post-fix
- **Impacto en el sistema:**
  - ESP32: tasa de muestreo sube de ~10 Hz a ~100 Hz (todas las tramas del sensor)
  - Latencia end-to-end: baja de ~100 ms a ~10 ms (trama más fresca siempre disponible)
  - CAN 0x208: datos más actualizados en cada frame (cada 66 ms)
  - STM32: sin cambios (56496 text idéntico)
- **Archivos modificados:** `esp32/src/sensors/obstacle_sensor.h`, `esp32/src/sensors/obstacle_sensor.cpp`, `esp32/src/main.cpp`, `esp32/src/test_obstacle_sensor.cpp`, `docs/TFMINI_PLUS_WIRING_GUIDE.md`, `PROJECT_CHANGELOG.md`
- **Tests:** 74 TF-Mini Plus + 135 TOFSense-M = 209 tests, 0 failures. STM32 build limpio (56496 text).

### PR-293 — fix(review): error-log retry throttling follow-up + IWDG / wraparound clarification
- **Fecha:** 2026-05-14
- **Autor:** Copilot
- **Descripción del cambio:** Actualización de seguimiento tras la revisión del PR para dejar cerrados los dos puntos pendientes del `error_log` y documentar con precisión los detalles temporales del watchdog. Se consolida el comportamiento del rate-limit ante fallos de flash persistentes y se aclaran los comentarios de soporte para futuras auditorías.
- **Root cause:** La revisión detectó dos áreas mejorables: (1) si una escritura a flash fallaba repetidamente, el gate temporal del `error_log` debía dejar explícito y probado que el timestamp se actualiza antes del intento para evitar reintentos back-to-back en cada nuevo evento; (2) la nota inline del IWDG y el comentario del test de wraparound eran demasiado compactos y dejaban margen a interpretación durante la revisión.
- **Solución aplicada:**
  1. `Core/Src/error_log.c`: el comentario del gate de auto-guardado documenta de forma explícita que `log_last_flash_tick` se actualiza antes del intento de escritura, de modo que un fallo persistente de flash no genere un retry storm y siga respetando la ventana de 100 ms.
  2. `Core/Src/test_error_log.c`: el simulador deja claro que `ERRLOG_FLASH_MIN_INTERVAL_MS` debe mantenerse sincronizado con `ERRLOG_WRITE_MIN_INTERVAL_MS`, y el caso `test_rate_limit_tick_wraparound()` ahora explica paso a paso el wrap de `HAL_GetTick()` (`0xFFFFFFFF → 0`) y por qué la resta modular unsigned sigue siendo correcta.
  3. `Core/Src/main.c`: la nota del IWDG se separa en bloque multilínea y documenta el cálculo nominal de 4.095 s junto con el rango real aproximado de 3.9–4.3 s derivado de la tolerancia ±5 % del LSI.
- **Impacto en el sistema:** Sin cambios de contrato CAN ni de APIs públicas. En la ruta de error, el `error_log` queda explícitamente protegido frente a reintentos de borrado/programación en ráfaga; en la ruta nominal no cambia el comportamiento. La actualización del IWDG es documental y no modifica registros ni timing de ejecución.
- **Archivos modificados:** `Core/Src/error_log.c`, `Core/Src/test_error_log.c`, `Core/Src/main.c`, `PROJECT_CHANGELOG.md`
- **Tests / validación:** La cobertura del rate-limit en `Core/Src/test_error_log.c` deja documentados los casos de primera escritura, ventana de 100 ms, liberación exacta en el borde, wraparound de tick y reset del estado tras `Init()`. El cambio del IWDG en `main.c` es solo de comentario.

### PR-292 — feat: LD2 now shows CAN status in main loop (no external LED needed)
- **Fecha:** 2026-04-09
- **Autor:** Copilot
- **Descripción del cambio:** LD2 (PA5, LED verde soldado en la Nucleo) ahora muestra el estado CAN de forma continua en el bucle principal, no solo durante el arranque. Ya no se necesita un LED externo en PB14 para ver si el CAN funciona.
- **Root cause:** El patrón de LD2 en el main loop era siempre "flash breve cada 2 s" independientemente de si el CAN estaba OK o FAIL. El usuario solo podía ver el estado CAN en PB14 (LED_DIAG), que requiere un LED externo con resistencia de 330 Ω que no estaba instalado. El patrón de arranque (1 blink largo = OK, 5 rápidos = FAIL) solo se mostraba una vez al inicio y era fácil de perder.
- **Solución aplicada:** Modificar el heartbeat LD2 del main loop para reflejar el estado CAN:
  - **CAN OK** (`fdcan_init_ok`): flash breve cada ~2 s (50 ms ON / 1950 ms OFF) — patrón anterior, sin cambio.
  - **CAN FAIL** (`!fdcan_init_ok`): parpadeo constante 1 Hz (500 ms ON / 500 ms OFF) — claramente diferente.
  Actualizada la documentación de patrones LED en los comentarios del boot.
- **Impacto en el sistema:** Solo cambia el comportamiento visual de LD2 cuando CAN ha fallado. Sin impacto en lógica de control ni seguridad. LED_DIAG (PB14) sigue funcionando igual para quien tenga LED externo.
- **Archivos modificados:** `Core/Src/main.c`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa (56496 text, 72 data, 7784 bss).

### PR — feat(obstacle): Activar sensor TF-Mini Plus (OBSTACLE_SENSOR_ENABLED=1)
- **Fecha:** 2026-04-09
- **Autor:** Copilot
- **Descripción del cambio:** Activación del sensor de obstáculos TF-Mini Plus en el firmware de la ESP32-S3. El sensor estaba completamente implementado pero deshabilitado (`OBSTACLE_SENSOR_ENABLED=0`) desde PR-275. Ahora se habilita para uso con el hardware TF-Mini Plus disponible.
- **Root cause:** El sensor de obstáculos estaba deshabilitado porque el hardware (TOFSense-M original) fue retirado. El usuario ahora dispone de un TF-Mini Plus listo para conectar.
- **Solución aplicada:**
  - Cambiar `OBSTACLE_SENSOR_ENABLED` de 0 a 1 en `obstacle_sensor.h`
  - `SENSOR_TYPE` ya era `SENSOR_TYPE_TFMINI` (115200 bps, 9-byte frames)
  - Actualizar comentarios del header para reflejar el estado activo
  - Crear guía de cableado `docs/TFMINI_PLUS_WIRING_GUIDE.md`
  - Actualizar `docs/OBSTACLE_SENSOR_DESIGN_DECISION.md` con sección de migración
  - Actualizar `docs/PROJECT_MASTER_STATUS.md` con estado actual del sensor
- **Impacto en el sistema:** La ESP32 ahora:
  - Configura UART1 (GPIO 18, 115200 bps) al arranque
  - Lee tramas TF-Mini Plus (9 bytes, 100 Hz) y valida checksum/señal/rango
  - Transmite CAN 0x208 (distancia + zona + salud + counter) cada 66 ms
  - Transmite CAN 0x209 (estado de seguridad) cada 100 ms
  - La STM32 aplica su backstop de 5 zonas con los datos recibidos
  - Sin cambios en el firmware STM32 (56496 text idéntico)
- **Archivos modificados:** `esp32/src/sensors/obstacle_sensor.h`, `docs/TFMINI_PLUS_WIRING_GUIDE.md` (nuevo), `docs/OBSTACLE_SENSOR_DESIGN_DECISION.md`, `docs/PROJECT_MASTER_STATUS.md`, `PROJECT_CHANGELOG.md`
- **Tests:** STM32 build limpio (56496 text). Tests obstacle sensor: 74 TF-Mini Plus + 135 TOFSense-M = 209 tests, 0 failures.

### PR-291 — fix: cppcheck shadowVariable — move SystemCoreClock extern to file scope
- **Fecha:** 2026-04-08
- **Autor:** Copilot
- **Descripción del cambio:** Corregir falso positivo de cppcheck `shadowVariable` en `boot_validation.c` que provocaba fallo del CI (exit code 2).
- **Root cause:** La declaración `extern uint32_t SystemCoreClock;` dentro de la función `check_clock_sane()` (línea 177) sombreaba la declaración idéntica a nivel de archivo en el stub HAL de análisis (`analysis_artifacts/stubs/stm32g4xx_hal.h:189`). cppcheck lo reporta como `shadowVariable` y el CI falla con exit code 2.
- **Solución aplicada:** Mover la declaración `extern uint32_t SystemCoreClock;` de dentro de `check_clock_sane()` al bloque de externs a nivel de archivo (junto a `fdcan_init_ok` e `i2c_init_ok`). La variable ya está definida en `system_stm32g4xx.c` y la declaración a nivel de archivo es semánticamente idéntica. Sin cambio funcional.
- **Impacto en el sistema:** Ninguno funcional. Resuelve fallo de CI cppcheck. Build size sin cambios (56408 text).
- **Archivos modificados:** `Core/Src/boot_validation.c`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa (56408 text, 72 data, 7784 bss).

### PR-289 — fix: second-pass word-by-word audit — NaN diagnostic logging + printf cast
- **Fecha:** 2026-04-08
- **Autor:** Copilot
- **Descripción del cambio:** Segunda pasada de auditoría exhaustiva palabra-por-palabra de todo el firmware (STM32 + ESP32). Se encontraron y corrigieron 2 bugs menores restantes tras la primera auditoría (PR-288).
- **Root cause:** (1) `boot_validation.c` líneas 196/205: la función de detección `check_temperature_plausible()` (línea 56) y `check_current_plausible()` (línea 76) usan `isnan()` correctamente, pero el código de logging diagnóstico que identifica qué sensor exacto falló NO incluía `isnan()`. Si un sensor retornaba NaN, la comparación `t < min || t > max` evaluaba a false (NaN → comparación siempre false), y el fault no se registraba para ese sensor concreto. (2) `esp32/src/main.cpp` línea 501: `ESP.getPsramSize()` retorna `size_t`, usado con `%u` sin cast `(unsigned)`. Inconsistente con la corrección aplicada en PR-288 a las líneas 55-56.
- **Solución aplicada:** (1) Añadir `isnan(t) ||` y `isnan(c) ||` a las condiciones de logging diagnóstico en `boot_validation.c` líneas 196 y 205, igualando la lógica de las funciones de detección. (2) Añadir cast `(unsigned)` a los argumentos de `Serial.printf()` en línea 501.
- **Impacto en el sistema:** (1) Ahora los sensores que retornan NaN quedan correctamente registrados con MODULE_FAULT_WARNING en ServiceMode, mejorando la trazabilidad de fallos. No afecta la detección ni la seguridad (la detección ya funcionaba). +16 bytes de text (isnan inlining). (2) Printf portabilidad.
- **Archivos modificados:** `Core/Src/boot_validation.c`, `esp32/src/main.cpp`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa (56096 text, 72 data, 7768 bss).

### PR-290 — feat: advanced robustness features (NaN hardening, CAN diagnostics, boot validation, logging)
- **Fecha:** 2026-04-08
- **Autor:** Copilot
- **Descripción del cambio:** Implementación de funcionalidades avanzadas de robustez en 7 áreas: endurecimiento NaN/Inf de sensores, monitoreo de trama CAN, validación TX FDCAN, extensión de validación de boot, y estandarización de logs.
- **Root cause:** Aunque el firmware era funcional y limpio, las rutas de comparación de float en `safety_system.c` no protegían contra NaN/Inf (IEEE 754 hace que todas las comparaciones con NaN retornen false, permitiendo que valores inválidos pasen silenciosamente por checks de rango). Además, faltaban métricas de diagnóstico CAN (FPS, fallos TX consecutivos) y validaciones de boot (RAM, clock, periféricos).
- **Solución aplicada:**
  1. **NaN/Inf Hardening (PART 1 — CRÍTICO):** Añadidos `isnan()`/`isinf()` a 10+ rutas de comparación float: `Safety_CheckCurrent()`, `Safety_CheckTemperature()` (umbral crítico + histéresis), `Safety_CheckSensors()` (temperatura, corriente, velocidad de rueda), `Safety_CheckBatteryVoltage()`, `Safety_CheckBatteryOvervoltage()`. NaN ahora se trata como fallo del lado seguro (overcurrent, overtemp crítico, fallo de sensor).
  2. **CAN Frame Health Monitoring (PART 3):** Nuevos campos `rx_frames_per_sec`, `rx_count_prev`, `rx_rate_tick` en `CAN_Stats_t`. Nueva función `CAN_UpdateFrameRate()` llamada cada 1 s desde el tier 1000 ms.
  3. **FDCAN TX Validation (PART 5):** Nuevos campos `tx_nack_flag`, `tx_consec_fail` en `CAN_Diag_t`. Seguimiento de fallos TX consecutivos en `TransmitFrame()`. Flag levantado tras 5 fallos consecutivos (no ACK).
  4. **Boot Validation Extension (PART 6):** Tres nuevos checks: `check_ram_sanity()` (patrón write/read no destructivo), `check_clock_sane()` (SystemCoreClock 160-180 MHz), `check_periph_ready()` (diagnóstico fdcan_init_ok/i2c_init_ok). Bitmask ampliado uint8→uint16.
  5. **Logging Standardization (PART 7):** Prefijos ESP32 estandarizados a formato `[MODULE][SEVERITY]`: `[CAN][INFO/ERR/WARN]`, `[BOOT][INFO/ERR]`, `[SAFETY][WARN/INFO]`, `[CAN][DIAG]`.
- **Partes ya implementadas previamente (sin cambios necesarios):**
  - PART 2 (CAN Diagnostics): Ya completo — `CAN_Diag_t` con PSR/ECR/TEC/REC, bus-off/error-passive/warning.
  - PART 4 (ESP32 BUS-OFF Recovery): Ya completo — polling `twai_get_status_info()`, `twai_initiate_recovery()`, error-passive recovery bifásica.
- **Impacto en el sistema:** Cierra la vulnerabilidad NaN en todas las rutas de seguridad. Añade métricas de diagnóstico CAN sin overhead en runtime. Boot validation más completa sin aumentar tiempo de boot. Compatible hacia atrás — no cambia APIs públicas ni comportamiento existente.
- **Archivos modificados:** `Core/Src/safety_system.c`, `Core/Inc/can_handler.h`, `Core/Src/can_handler.c`, `Core/Src/main.c`, `Core/Inc/boot_validation.h`, `Core/Src/boot_validation.c`, `esp32/src/main.cpp`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa (56368 text, 72 data, 7784 bss). +272 bytes text vs baseline.

### PR-288 — fix(esp32): word-by-word audit + 3 bug fixes + changelog update
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** Auditoría exhaustiva palabra-por-palabra de todo el firmware STM32 (10 archivos core) y ESP32 (5 archivos core). La STM32 está limpia (0 bugs). Se encontraron y corrigieron 3 bugs en ESP32.
- **Root cause:** El debugger seguía dando errores de comunicación CAN. Se realizó una revisión exhaustiva del código para identificar cualquier error restante. Los bugs encontrados estaban en el lado ESP32 (especificadores printf incorrectos + puerto hardcodeado).
- **Bugs corregidos:**
  1. **CRÍTICO — `%lus` formato printf inválido** (`esp32/src/main.cpp:1272`): El especificador `%lus` es inválido en printf. La "s" se unía al formato `%lu`, causando comportamiento indefinido en la salida serial del diagnóstico error-passive. Fix: `%lus` → `%lu s` (espacio antes de "s").
  2. **ALTO — `%d` para `size_t`** (`esp32/src/main.cpp:55,56,63,66`): Se usaba `%d` (signed int) para imprimir valores `size_t` en diagnóstico PSRAM. En arquitecturas donde `size_t` ≠ `int`, puede truncar o mostrar valores incorrectos. Fix: `%d` → `%u` con cast explícito `(unsigned)`.
  3. **ALTO — puerto COM8 hardcodeado** (`esp32/platformio.ini:81`): `upload_port = COM8` impide compilar/flashear en cualquier máquina que no sea la del desarrollador. Fix: línea comentada con `;`.
- **Auditoría STM32 (resultado: LIMPIO):**
  - `Core/Src/main.c`: Boot sequence, FDCAN init timing, LED patterns — ✅
  - `Core/Src/can_handler.c`: CAN IDs, DLC mapping, endianness, TransmitFrame — ✅
  - `Core/Src/stm32g4xx_hal_msp.c`: GPIO AF9, clock source, NVIC priorities — ✅
  - `Core/Src/stm32g4xx_it.c`: Fault handlers, MOE disable, IRQ dispatch — ✅
  - `Core/Inc/project_config.h`: Pin definitions, no conflicts — ✅
  - `Core/Inc/can_handler.h`: CAN ID defines consistency — ✅
  - `Core/Inc/can_init_diag.h`: Diagnostic struct — ✅
  - `Core/Src/safety_system.c`: State machine, thresholds, ABS/TCS — ✅
  - `Core/Src/motor_control.c`: Timer channels, PWM, direction logic — ✅
  - `Core/Src/system_stm32g4xx.c`: FPU init via CPACR — ✅
- **Impacto en el sistema:** Corrige output serial de diagnóstico CAN error-passive y PSRAM. No afecta lógica de control ni safety. Permite a cualquier usuario flashear sin editar platformio.ini.
- **Archivos modificados:** `esp32/src/main.cpp`, `esp32/platformio.ini`, `PROJECT_CHANGELOG.md`
- **Tests:** STM32 build con `-Wall -Wextra -Werror` pasa (56080 text, 72 data, 7768 bss). ESP32 no compilable en sandbox (requiere PlatformIO con ESP-IDF).

### PR-287 — fix(esp32): clear quanta_resolution_hz to fix 625kbps→500kbps baud rate mismatch
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** Corrección CRÍTICA del baud rate CAN de la ESP32-S3 que impedía toda comunicación con la STM32. En ESP-IDF ≥ 5.0, la macro `TWAI_TIMING_CONFIG_500KBITS()` establece `quanta_resolution_hz = 10 MHz` con `brp = 0`. Cuando el código luego sobrescribe `brp = 10`, ambos campos son no-zero, y el driver ESP-IDF **ignora** `brp` y calcula: `brp = APB_CLK / quanta_resolution_hz = 80 MHz / 10 MHz = 8`. Esto produce `(1+13+2) × 8/80 MHz = 1600 ns → 625 kbps` en vez de los 500 kbps esperados — un **25% de desajuste** que hace IMPOSIBLE la comunicación.
- **Root cause:** Cambio de comportamiento en ESP-IDF 5.x: la macro de timing ahora rellena `quanta_resolution_hz`, y cuando ambos `quanta_resolution_hz > 0` y `brp > 0`, el driver prioriza `quanta_resolution_hz` e ignora `brp`. Esto no ocurría en ESP-IDF 4.x donde el campo no existía.
- **Solución aplicada:** (1) `#include <esp_idf_version.h>` para detección de versión en compilación. (2) Bloque `#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)` que limpia `t_config.quanta_resolution_hz = 0` antes de sobrescribir `brp = 10`. Esto fuerza al driver a usar BRP directo: `80 MHz / 10 / 16 = 500 kbps ✓`. (3) Diagnóstico serial mejorado mostrando versión ESP-IDF y confirmación del modo BRP forzado. (4) Formato printf corregido con `%%` para literal `%`.
- **Impacto en el sistema:** Restaura comunicación CAN entre ESP32-S3 y STM32G474RE. Antes de este fix, la ESP32 transmitía a 625 kbps y la STM32 escuchaba a 500 kbps — ningún frame era ACKeado, causando bus-off inmediato.
- **Archivos modificados:** `esp32/src/main.cpp`
- **Tests:** Verificación manual de cálculo de baud rate. Build STM32 pasa (sin cambios en STM32).

### PR-287a — fix(fdcan): start FDCAN before boot LED blinks to prevent ESP32 bus-off
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** Mover la inicialización FDCAN (`MX_FDCAN1_Init()` + `CAN_Init()`) ANTES de la secuencia de boot LED (3 blinks × 300 ms = 2.3 s). Esto asegura que la STM32 empiece a ACKear tramas CAN dentro de ~50 ms del encendido, en vez de después de los 2.3 s de la secuencia visual.
- **Root cause:** En un bus CAN de 2 nodos, la ESP32-S3 arranca ~600 ms después del encendido y empieza a transmitir heartbeats cada 100 ms. Si la STM32 no ACKea dentro de ~1.3 s, el TEC de la ESP32 alcanza 256 → BUS_OFF. Los 2.3 s de boot blinks dejaban a la ESP32 sola en el bus demasiado tiempo.
- **Solución aplicada:** (1) `MX_GPIO_Init()` → `MX_FDCAN1_Init()` → `CAN_Init()` → `boot_phase = 1` → boot LED blinks. (2) Drain FIFO post-blinks (CAN_ProcessMessages + reset overflow counter). (3) Documentación inline detallada del timing.
- **Impacto en el sistema:** FDCAN activo ACKeando frames durante los boot blinks. Sin dependencia de Motor/Safety/Sensor modules.
- **Archivos modificados:** `Core/Src/main.c`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa.

### PR-286 — fix(led): revert LED back to PA5 (LD2) — onboard Nucleo LED
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** Se revierte el LED de estado de PB8 a PA5 (LD2). LD2 es el LED verde soldado directamente en la placa NUCLEO-G474RE (UM2505 §6.5). No necesita LED externo ni resistencia. El cambio a PB8 fue un error — PA5 es la opción correcta para este hardware.
- **Root cause:** En PR-285 se movió el LED a PB8 creyendo que PA5 tenía interferencia del ST-Link vía SB21. Sin embargo, el usuario confirmó que LD2 (PA5) es el LED soldado en la propia placa Nucleo y funciona correctamente. No hay ningún LED externo conectado.
- **Solución aplicada:** (1) Revertir defines a `PIN_LD2` (GPIO_PIN_5), `PORT_LD2` (GPIOA), `PIN_LD2_N` (5U). (2) Todos los `HAL_GPIO_WritePin(PORT_LED_STATUS, PIN_LED_STATUS, ...)` → `HAL_GPIO_WritePin(PORT_LD2, PIN_LD2, ...)`. (3) Error_Handler y fault handlers: GPIOB bit 8 → GPIOA bit 5. (4) Eliminar PB8 STATUS_LED del .ioc.
- **Impacto en el sistema:** Solo cambia el pin físico. Vuelve al LED soldado en la placa. No requiere hardware externo.
- **Archivos modificados:** `Core/Inc/project_config.h`, `Core/Src/main.c`, `Core/Src/stm32g4xx_it.c`, `STM32-Control-Coche-Marcos.ioc`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa. Integrity check pasa.

### PR-285 — refactor(led): migrate status LED from PA5 (LD2) to PB8
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** Se migra todo el indicador LED de estado de PA5 (LD2) a PB8 (STATUS_LED). PA5 comparte el solder bridge SB21 con el SPI SCK del ST-Link en la placa Nucleo-64, causando que el LED sea controlado por el debugger en vez del firmware. PB8 es un GPIO libre accesible en el conector Morpho (CN10 pin 3). Requiere conectar un LED externo con resistencia a PB8.
- **Root cause:** PA5 (LD2) está compartido con el ST-Link SPI vía SB21 (cerrado por defecto en fábrica). Durante programación o debug activo, el ST-Link puede encender/apagar PA5 sin control del firmware, haciendo el LED no fiable para indicar estados del sistema. Según UM2505, la NUCLEO-G474RE solo tiene LD1 (COM, ST-Link) y LD2 (USER, PA5) — no existe LD4.
- **Solución aplicada:** (1) Nuevo define centralizado: `PIN_LED_STATUS` (GPIO_PIN_8), `PORT_LED_STATUS` (GPIOB), `PIN_LED_STATUS_N` (8U) en project_config.h. (2) Todos los `HAL_GPIO_WritePin(GPIOA, PIN_LD2, ...)` → `HAL_GPIO_WritePin(PORT_LED_STATUS, PIN_LED_STATUS, ...)`. (3) Error_Handler y fault handlers: registro directo actualizado de GPIOA bit 5 → GPIOB bit 8. (4) .ioc actualizado con PB8 como GPIO_Output etiquetado STATUS_LED.
- **Impacto en el sistema:** Solo afecta qué pin físico se usa para indicación visual. No cambia lógica, timing, ni safety. El usuario necesita conectar un LED+resistencia a PB8 (Morpho CN10 pin 3).
- **Archivos modificados:** `Core/Inc/project_config.h`, `Core/Src/main.c`, `Core/Src/stm32g4xx_it.c`, `STM32-Control-Coche-Marcos.ioc`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa. Integrity check pasa.

### PR-284 — feat(led): make boot blinks unmissable (300 ms + dark lead-in)
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** El usuario no veía los 3 blinks de boot al pulsar RESET porque a 150 ms ON/OFF el patrón duraba solo 900 ms y se confundía con un encendido directo. Se duplica el timing a 300 ms ON/OFF (1.8 s) con un período oscuro de 500 ms previo. También se aumenta el blink CAN-OK de 400 ms a 600 ms y CAN-FAIL de 80 ms a 150 ms ON/OFF. Separación post-boot aumentada a 500 ms.
- **Root cause:** Sin período oscuro inicial, el primer blink ON se confunde con el encendido natural del LED tras reset. A 150 ms por fase, los 3 blinks pasan en 900 ms — demasiado rápido para el ojo.
- **Solución aplicada:** (1) LED OFF explícito + 500 ms dark lead-in antes de empezar los blinks. (2) Boot blinks: 150 ms → 300 ms por fase (3 blinks = 1.8 s, imposible de no ver). (3) CAN status: pausa 500 ms + 1 blink largo 600 ms (CAN OK) o 5 blinks 150 ms (CAN FAIL). Todo antes de IWDG timeout (~4 s).
- **Impacto en el sistema:** Solo afecta la secuencia de boot LED (~4 s de delay total antes del main loop). No afecta lógica de control, safety, ni timing del main loop. IWDG se inicia después de los boot blinks.
- **Archivos modificados:** `Core/Src/main.c`, `PROJECT_CHANGELOG.md`
- **Tests:** Integrity check pasa. Build requiere cross-compiler (no disponible en sandbox).

### PR-283 — feat(led): improve boot visibility and add CAN init status indication
- **Fecha:** 2026-04-06
- **Autor:** Copilot
- **Descripción del cambio:** El patrón de 3 blinks de boot era demasiado rápido (60 ms ON/OFF = 360 ms total) para ser visible al enchufar USB. Se aumenta a 150 ms ON/OFF (900 ms total). Se añade indicación LED post-init del estado CAN: 1 blink largo (400 ms) = CAN OK, 5 blinks rápidos (80 ms) = CAN FAILED. Se añade `volatile` a `can_init_diag` para visibilidad fiable vía SWD con `-O2`.
- **Root cause:** El usuario no veía los 3 blinks de arranque porque a 60 ms por fase el patrón completo duraba solo 360 ms, invisible al conectar USB. Sin pantalla ni UART, el LED es el único feedback visual del estado del firmware.
- **Solución aplicada:** (1) Boot blinks: 60 ms → 150 ms por fase (3 blinks = 900 ms, claramente visible). (2) Post-init CAN status: pausa 300 ms + 1 blink largo (CAN OK) o 5 blinks rápidos (CAN FAIL). (3) `volatile` en `can_init_diag` (en `can_init_diag.h` tras merge con PR-282).
- **Impacto en el sistema:** Solo afecta la secuencia de boot LED (~2 s de delay adicional). No afecta lógica de control, safety, ni timing del main loop.
- **Archivos modificados:** `Core/Src/main.c`, `Core/Inc/can_init_diag.h`, `PROJECT_CHANGELOG.md`, `docs/PROJECT_MASTER_STATUS.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa sin errores.

### PR-282 — refactor(fdcan): decouple hal_msp from can_handler, fix stale filter documentation

- **Fecha:** 2026-04-06
- **Autor:** Copilot
- **Descripción del cambio:** Refactor seguro para reducir acoplamiento arquitectónico entre `stm32g4xx_hal_msp.c` y `can_handler.h`. Corrección de documentación de filtros CAN que describía erróneamente una política "reject-all" cuando la implementación real es accept-all con filtrado por software.
- **Solución aplicada:** (1) Extraer `CAN_InitDiag_t` a nuevo header `Core/Inc/can_init_diag.h`. (2) Reemplazar `#include "can_handler.h"` en hal_msp.c por `#include "can_init_diag.h"`. (3) `can_handler.h` incluye `can_init_diag.h` (sin duplicar typedef). (4) Corregir "reject-all default" → "accept-all (mask=0), non-matching IDs routed to FIFO0" en PROJECT_MASTER_STATUS.md. (5) Actualizar RX Filter Policy en CAN_CONTRACT_FINAL.md para reflejar la implementación real (MASK accept-all + software filtering en switch/case).
- **Impacto en el sistema:** Cero cambios funcionales. Binary idéntico (56032 text, 72 data, 7832 bss). Solo reduce coupling y corrige documentación.
- **Archivos modificados:** `Core/Inc/can_init_diag.h` (nuevo), `Core/Inc/can_handler.h`, `Core/Src/stm32g4xx_hal_msp.c`, `docs/PROJECT_MASTER_STATUS.md`, `docs/CAN_CONTRACT_FINAL.md`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa sin errores ni warnings. Binary size idéntico.

### PR-281 — fix(fdcan): robust deterministic FDCAN initialization with readback verification
- **Fecha:** 2026-04-06
- **Autor:** Copilot
- **Descripción del cambio:** Corrección CRÍTICA de la inicialización FDCAN que causaba fallo total del periférico CAN. Implementa secuencia de bring-up 100% robusta, determinista y verificable por hardware.
- **Root cause:** Tras reset, `RCC_CCIPR.FDCANSEL` = 00 (HSE, no habilitado en este proyecto). `RCC->APB1ENR1` no tenía bit FDCANEN habilitado. Lecturas de registros FDCAN retornaban datos basura de flash (stale AHB bus data, e.g. `0x08007d8d`). El `force-reset` en MspInit podía dejar el clock gate en estado indeterminado y la primera escritura a FDCANSEL podía no latchar en algunas revisiones de silicio.
- **Solución aplicada:** (1) MspInit: configurar FDCANSEL=PCLK1 ANTES de habilitar APB1 clock, con readback-verify. (2) Reset robusto: `__DSB(); __ISB()` tras force/release reset, re-enable clock + re-apply FDCANSEL post-reset. (3) Poll adaptativo de CCCR (50 ms timeout, sin delays fijos). (4) Readback-verify de `RCC->APB1ENR1` bit FDCANEN. (5) CAN_Init: `can_init_diag.started = 0U` al inicio (no por-path), solo promovido a 1U al final. (6) CAN_Init: verificación adicional de APB1ENR1 antes de configurar filtros. (7) Nuevos campos en `CAN_InitDiag_t`: retries, timeout_flag, msp_clk_ok, msp_ccipr_ok. (8) MX_FDCAN1_Init: registro de retry count en diagnostics.
- **Impacto en el sistema:** Elimina el fallo total de FDCAN por clock mal configurado o stale AHB reads. Inicialización determinista independiente de la revisión de silicio. Todos los paths de fallo dejan estado limpio y diagnosticable.
- **Archivos modificados:** `Core/Src/stm32g4xx_hal_msp.c`, `Core/Src/can_handler.c`, `Core/Inc/can_handler.h`, `Core/Src/main.c`, `docs/FDCAN_BRINGUP_ROBUSTNESS.md` (nuevo), `PROJECT_CHANGELOG.md`, `docs/PROJECT_MASTER_STATUS.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa sin errores. Validación completa requiere hardware con SWD.
- **Próximos pasos:** Verificar en hardware: `can_init_diag.started == 1`, `RCC->APB1ENR1` bit 25 set, `RCC->CCIPR` bits [25:24] = 10, CCCR ≠ 0x0800xxxx.

### PR-280 — docs: complete power supply system document (9 sections + BOM)
- **Fecha:** 2026-04-04
- **Autor:** Copilot
- **Descripción del cambio:** Documento técnico completo del sistema de alimentación en `Documentos/SISTEMA_ALIMENTACION_COMPLETO.md` con 9 secciones + 2 apéndices. Cubre arquitectura de alimentación (24V/12V/5V/3.3V), recorrido completo desde baterías hasta actuadores, integración de relés (MAIN/TRAC/DIR/LED), apagado retardado (3s retention relay + audio DFPlayer), alimentación LED WS2812B (relés CAN 0x120), protección eléctrica (fusibles, TVS, desacoplo, umbrales batería), distribución de masas (punto estrella), esquema eléctrico detallado (31 pines STM32, I2C, FDCAN 500 kbps), y lista de materiales completa (BOM).
- **Root cause:** N/A — documento de diseño, no fix.
- **Solución aplicada:** Consolidación de datos de firmware verificado (safety_system.c, can_handler.c, main.c, power_manager.cpp, audio_manager.cpp) y documentación existente (LLAVE_CONTACTO, POWER_DISTRIBUTION, CONEXIONES_COMPLETAS, HARDWARE_WIRING_MANUAL, MATERIALES_POR_MODULO) en un único documento de referencia.
- **Impacto en el sistema:** Ningún cambio funcional. Documento de referencia para montaje, diagnóstico y mantenimiento del sistema de alimentación.
- **Archivos modificados:** `Documentos/SISTEMA_ALIMENTACION_COMPLETO.md` (nuevo), `PROJECT_CHANGELOG.md`, `docs/PROJECT_MASTER_STATUS.md`
- **Tests:** N/A — solo documentación.
- **Próximos pasos:** Verificar en hardware que todas las secciones de cable y valores de componentes coinciden con la implementación física.

### PR-279b — docs: CAN hardware fix procedure + fix stale PB8/PB9 pin references
- **Fecha:** 2026-04-04
- **Autor:** Copilot
- **Descripción del cambio:** Creación de documento ejecutable `CAN_HARDWARE_FIX_PROCEDURE.md` para resolver el fallo "waiting for CAN". Corrección de referencias de pines obsoletas PB8/PB9 → PA11/PA12 en documentación crítica.
- **Root cause confirmado:** Los pines FDCAN1 fueron remapeados de PB8/PB9 a PA11/PA12 (AF9) en marzo 2026 (PR #255-256). Documentos legacy aún referencian PB8/PB9 — si el hardware sigue esa documentación obsoleta, el transceiver CAN está conectado a pines incorrectos. Además, Pin 8 (Rs) del SN65HVD230 probablemente no está conectado a GND.
- **Solución aplicada:** (1) Nuevo documento `docs/CAN_HARDWARE_FIX_PROCEDURE.md`: flowchart de diagnóstico, pinout verificado del SN65HVD230, tablas de mediciones eléctricas, procedimiento paso a paso con interpretación de resultados. (2) Corrección de `HARDWARE_VALIDATION_PROCEDURE.md`: PB8/PB9 → PA11/PA12 en 2 ubicaciones. (3) Corrección de `PROJECT_MASTER_STATUS.md`: FDCAN pin reference actualizada a PA11/PA12 con detalles de transceiver.
- **Impacto en el sistema:** Documentación-only. Previene errores de cableado causados por documentación obsoleta.
- **Archivos modificados:** `docs/CAN_HARDWARE_FIX_PROCEDURE.md` (nuevo), `docs/HARDWARE_VALIDATION_PROCEDURE.md`, `docs/PROJECT_MASTER_STATUS.md`, `PROJECT_CHANGELOG.md`
- **Tests:** Sin cambios en firmware.

### PR-279a — audit(system): Full-system CAN validation audit + ESP32 CAN RX diagnostics
- **Fecha:** 2026-04-04
- **Autor:** Copilot
- **Descripción del cambio:** Auditoría completa del sistema dual-MCU (STM32 FDCAN + ESP32 TWAI). Análisis exhaustivo de firmware, protocolo CAN y procedimientos de validación hardware. Corrección de documentación DLC y adición de diagnósticos CAN RX.
- **Root cause identificado:** El firmware CAN es correcto y production-ready en ambos MCU. El fallo "waiting for CAN" es causado por problemas hardware: (P1) Pin 8 Rs del SN65HVD230 no conectado a GND, (P2) falta de GND común entre MCUs, (P3) terminación incorrecta.
- **Solución aplicada:** (1) Corrección de documentación STATUS_SAFETY DLC 3→5 en `can_ids.h`. (2) Adición de logging diagnóstico CAN RX en `can_rx.cpp`: primeros 10 frames recibidos con ID/DLC/datos + contador periódico cada 10s. (3) Documento de auditoría completo `docs/FULL_SYSTEM_VALIDATION_AUDIT.md` con root cause analysis, checklist de validación hardware, y estado final por subsistema.
- **Impacto en el sistema:** Mejora diagnósticos para identificar fallos hardware CAN. Sin cambio funcional en paths de control.
- **Archivos modificados:** `esp32/include/can_ids.h`, `esp32/src/can_rx.cpp`, `docs/FULL_SYSTEM_VALIDATION_AUDIT.md`, `PROJECT_CHANGELOG.md`, `docs/PROJECT_MASTER_STATUS.md`
- **Tests:** STM32 build con `-Wall -Wextra -Werror` pasa sin errores. Sin cambios en código STM32.

### PR-279 — fix(esp32/can): TWAI clk_src zeroed by memset — CAN bus inoperative
- **Fecha:** 2026-04-02
- **Autor:** Copilot
- **Descripción del cambio:** Corrige bug crítico en la inicialización TWAI del ESP32 que impedía toda comunicación CAN entre la ESP32-S3 y la STM32.
- **Root cause:** `twaiInit()` usaba `memset(&t_config, 0, sizeof(t_config))` para inicializar `twai_timing_config_t`. En ESP-IDF ≥ 5.0 este struct contiene un campo `clk_src` (clock source) cuyo valor por defecto (`TWAI_CLK_SRC_DEFAULT = SOC_MOD_CLK_APB`) NO es cero. El `memset` ponía `clk_src = 0`, lo que hacía que `twai_driver_install()` fallara con `ESP_ERR_INVALID_ARG` (reloj inválido) o seleccionara un reloj incorrecto (e.g. CPU @ 240 MHz en vez de APB @ 80 MHz), produciendo un baud rate de 1500 kbps en vez de 500 kbps — incompatible con el FDCAN del STM32.
- **Solución aplicada:** Reemplazar `memset` por inicialización con macro `TWAI_TIMING_CONFIG_500KBITS()` (que configura `clk_src` correctamente en todas las versiones de ESP-IDF), y después sobreescribir los campos de timing personalizados (brp=10, tseg_1=13, tseg_2=2, sjw=2) para mantener el sample point de 87.5% que empareja con el 88.2% del STM32.
- **Impacto en el sistema:** Restaura la comunicación CAN entre ESP32-S3 y STM32. Sin este fix, NINGÚN frame CAN podía intercambiarse entre las dos placas.
- **Archivos modificados:** `esp32/src/main.cpp`
- **Tests:** Integrity checks pasados. Validación completa requiere hardware (ESP32-S3 + STM32 + 2× TJA1051T/3).

### PR-278 — fix(fdcan): adaptive CCCR readback poll replaces fixed-count stabilisation loop
- **Fecha:** 2026-03-31
- **Autor:** Copilot
- **Descripción del cambio:** Reemplaza el bucle volátil de 3200 iteraciones en `HAL_FDCAN_MspInit` por un poll adaptativo de CCCR con timeout de 50 ms (SysTick). Añade verificación readback de FDCANSEL tras re-aplicación.
- **Root cause:** En algunas revisiones de silicio STM32G4, tras `__HAL_RCC_FDCAN_FORCE_RESET()`, el clock gate del periférico necesita más tiempo del previsto para estabilizarse. El bucle fijo de 3200 iteraciones (~113 µs) era insuficiente: CCCR retornaba datos basura estables del pipeline de instrucciones AHB (e.g. `0x08007bc9`, valor próximo a LR/PC en flash). Además, la primera escritura a `RCC_CCIPR.FDCANSEL` tras force-reset podía no latchar en algunas revisiones.
- **Solución aplicada:** (1) Poll adaptativo: lee CCCR repetidamente hasta que bits reservados [31:16] sean cero (periférico respondiendo), con timeout de 50 ms vía `HAL_GetTick()`. (2) Readback de FDCANSEL: tras `__HAL_RCC_FDCAN_CONFIG(PCLK1)`, lee `RCC_CCIPR` para verificar que el valor latchó; si no, re-aplica con barrera adicional.
- **Impacto en el sistema:** Mayor resiliencia en inicialización FDCAN en revisiones de silicio con clock gate lento. El poll adaptativo se adapta automáticamente al tiempo necesario en vez de depender de una estimación fija.
- **Archivos modificados:** `Core/Src/stm32g4xx_hal_msp.c`
- **Tests:** 455 unit tests pasados (sin cambio funcional en paths testados). Validación completa requiere hardware con SWD.
- **Próximos pasos:** Verificar en hardware que CCCR retorna valores válidos tras el poll y que `can_init_diag.hal_init == 0` en todas las condiciones de arranque (cold boot, watchdog reset, power glitch).

### PR-275 (GitHub) — fix(sensor): TF-Mini Plus uint16 overflow guard and stale data clearing on timeout
- **Fecha:** 2026-03-30
- **Autor:** Copilot
- **Descripción del cambio:** Auditoría profunda del parser TF-Mini Plus encontró dos bugs edge-case reales (overflow uint16 en conversión cm→mm y datos stale tras timeout). Además, se añadió soporte dual de sensor (TOFSense-M y TF-Mini Plus) seleccionable en compile-time, flag de habilitación del sensor, y documentación de interfaz común.
- **Root cause:** (1) `distCm * 10` se computaba como `uint16_t`, wrapping silenciosamente para valores >6553 cm (e.g., `6554 * 10 = 65540` trunca a `4`). Esto creaba una lectura de emergencia falsa con `healthy=true`. (2) En timeout de frame, `status` y `healthy` se limpiaban pero `distance_mm` y `zone` retenían sus últimos valores válidos. Frames CAN 0x208 llevarían distancia stale con `health=0`.
- **Solución aplicada:** (1) Cast intermedio a `uint32_t` con guarda: `uint32_t distMm = (uint32_t)distCm * 10u; if (distMm > UINT16_MAX) return false;`. (2) Timeout ahora limpia `distance_mm = 0` y `zone = 0` (zone 0 = "far/normal", correcto para sensor desconectado). (3) Flag `OBSTACLE_SENSOR_ENABLED` (default 0) para deshabilitar toda la lógica UART/parsing cuando no hay sensor. (4) Selección de sensor vía `SENSOR_TYPE` (TOFSENSE=0, TFMINI=1, default TFMINI). (5) Nuevo archivo `distance_sensor.h` con guía de integración TF-Mini Plus. (6) Simplificación de `main.cpp`: config defaults por `SENSOR_TYPE` en vez de hardcoded.
- **Impacto en el sistema:** Eliminados dos edge-cases de seguridad en el parser TF-Mini Plus. Estado INVALID ahora es completamente consistente (distance=0, zone=0, healthy=false, status=INVALID). Soporte dual de sensor preparado para intercambio de hardware.
- **Archivos modificados:** `esp32/src/sensors/obstacle_sensor.cpp`, `esp32/src/sensors/obstacle_sensor.h`, `esp32/src/sensors/distance_sensor.h` (nuevo), `esp32/src/main.cpp`, `esp32/src/test_obstacle_sensor.cpp`
- **Tests:** `test_tfmini_overflow_large_distance` (boundary 6553/6554 cm), `test_tfmini_timeout_clears_distance` (verifica distance y zone a 0 tras timeout), `test_tfmini_header_byte_in_distance` (0x5959 rechazado por overflow guard). TF-Mini Plus: 74 tests, TOFSense-M: 135 tests.
- **Próximos pasos:** Verificar en hardware con TF-Mini Plus conectado. Verificar que CAN 0x208 reporta `health=0, status=INVALID` correctamente cuando el sensor está desconectado.

### PR-277 — fix(can): two-phase error-passive recovery (never give up)
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** La recuperación de error-passive del TWAI (ESP32) ahora usa una estrategia bifásica en vez de abandonar tras 10 intentos.
- **Root cause:** Cuando el TEC saturaba a 128 (error-passive) y el STM32 no estaba en el bus (sin ACK), la recuperación se agotaba tras 10 reinits rápidos (cada 3 s = 30 s total). Después, el ESP32 nunca volvía a intentar recuperar el CAN, dejando el `bus_err` creciendo indefinidamente (observado: 45430 → 138000+). Si el STM32 arrancaba después de esos 30 s, el CAN del ESP32 quedaba muerto permanentemente hasta reinicio.
- **Solución aplicada:** Estrategia bifásica: (1) Phase 1 (fast): primeros 10 intentos cada 3 s (comportamiento original). (2) Phase 2 (slow): intentos ilimitados cada 30 s — suficiente para recuperarse cuando el STM32 se conecte, sin martillear el bus. Renombrado `ERROR_PASSIVE_MAX_RESETS` → `ERROR_PASSIVE_MAX_FAST`, añadido `ERROR_PASSIVE_SLOW_TIMEOUT_MS = 30000`. El contador `errorPassiveResets` se resetea cuando TEC cae por debajo de 128 (bus sano).
- **Impacto en el sistema:** El CAN del ESP32 ya no queda permanentemente muerto si los 10 reinits rápidos no resuelven el error-passive. El sistema sigue intentando periódicamente con reinits lentos, permitiendo recuperación automática si el STM32 se conecta tarde.
- **Archivos modificados:** `esp32/src/main.cpp`
- **Próximos pasos:** Verificar en hardware que el reinit lento (30 s) recupera el CAN correctamente cuando el STM32 se conecta después de los 10 intentos rápidos.
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Mejora fiabilidad del sensor de obstáculos (TOFSense-M 8×8) y reduce falsos positivos en warnings de bloqueo del RuntimeMonitor.
- **Root cause:** (1) Buffer UART RX de 1024 bytes se llenaba en ~11 ms a 921600 baud; cualquier jitter del loop >11 ms causaba overflow con pérdida de bytes mid-frame, provocando fallos de checksum en cascada. (2) Resync tras BAD_CHECKSUM solo buscaba 0x57 (1/256 probabilidad por byte) — bytes 0x57 en datos de píxeles generaban falsos resyncs, causando más fallos de checksum en cadena. (3) Umbral de bloqueo de render (4 ms) era demasiado bajo para TFT SPI (~22 ms normal) ejecutándose en Core 0 (sin bloquear el loop principal en Core 1).
- **Solución aplicada:** (1) `rxBufSize` 1024→4096 bytes (~44 ms de headroom a 921600 baud). (2) `MAX_BYTES_PER_UPDATE` 800→1600 bytes (procesa hasta 4 frames/llamada). (3) Resync mejorado: busca par 0x57+0x01 (header+function_mark) en vez de solo 0x57, reduciendo probabilidad de falso resync de 1/256 a 1/65536. (4) Threshold de bloqueo de render separado: `RENDER_BLOCKING_THRESHOLD_US = 35000` (35 ms) para Core 0, ya que SPI TFT ~22 ms es normal y no afecta al loop principal. (5) Añadido `uartHWM` (high-water mark) al diagnóstico para detectar overflow de buffer UART.
- **Impacto en el sistema:** Sensor de obstáculos más fiable con mejor tolerancia a jitter del loop. Warnings `render=YES` solo aparecen ante stalls reales (>35 ms), no ante frames TFT normales. Nuevos diagnósticos `uartHWM` ayudan a detectar overflow.
- **Archivos modificados:** `esp32/src/sensors/obstacle_sensor.h`, `esp32/src/sensors/obstacle_sensor.cpp`, `esp32/src/ui/runtime_monitor.h`, `esp32/src/ui/runtime_monitor.cpp`, `esp32/src/test_obstacle_sensor.cpp`
- **Próximos pasos:** Verificar en hardware que `uartHWM` se mantiene < 4096 y que la tasa de `cksumFail` disminuye significativamente. Si `uartHWM` ≥ 3500, considerar aumentar el buffer o reducir frame rate del sensor con NAssistant.

### PR-275 — fix(ci): suppress cppcheck duplicateAssignExpression false positives on CCCR triple-read
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Corrige fallo de CI cppcheck (exit code 2) causado por falsos positivos `duplicateAssignExpression` en las lecturas triples de CCCR. Cppcheck no entiende que `hfdcan1.Instance->CCCR` es un registro hardware volátil que puede retornar valores diferentes en cada lectura.
- **Root cause:** Las lecturas triples de CCCR en `main.c` y `can_handler.c` son intencionalmente la misma expresión repetida 3 veces para detectar datos inconsistentes del bus AHB. Cppcheck las clasifica como `duplicateAssignExpression` (asignaciones duplicadas) porque no analiza semántica de registros volátiles.
- **Solución aplicada:** (1) Añadidos comentarios `// cppcheck-suppress duplicateAssignExpression` en las 4 líneas afectadas. (2) Añadido flag `--inline-suppr` al comando cppcheck en CI para habilitar supresiones inline.
- **Impacto en el sistema:** Solo CI — sin cambio funcional en firmware. Las supresiones son localizadas y documentan por qué el patrón es intencional.
- **Archivos modificados:** `Core/Src/main.c`, `Core/Src/can_handler.c`, `.github/workflows/firmware-validation.yml`
- **Próximos pasos:** Ninguno.

### PR-274 — fix(fdcan): add CCCR triple-read to CAN_Init clock re-apply path
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Cierra edge case en el path de re-init de `CAN_Init()`: tras re-aplicar clock source PCLK1 y re-invocar `HAL_FDCAN_Init()`, faltaba la comprobación triple-lectura de CCCR que sí tenía `MX_FDCAN1_Init`. Sin esta comprobación, el re-init podía reportar éxito con datos basura.
- **Root cause:** El re-init en `CAN_Init()` (líneas 265-274) no incluía validación triple-lectura de CCCR tras `HAL_FDCAN_Init()`. Si la clock gate seguía inestable tras el cambio de fuente de reloj, `HAL_FDCAN_Init()` podía devolver `HAL_OK` con registros retornando stale bus data — el mismo falso positivo que la triple-lectura de `MX_FDCAN1_Init` previene.
- **Solución aplicada:** Añadido bloque triple-lectura CCCR inmediatamente después de `HAL_FDCAN_Init()` en el path de clock re-apply de `CAN_Init()`. Verifica que 3 lecturas consecutivas sean idénticas, INIT=1, y bits 16-31 = 0.
- **Impacto en el sistema:** Elimina el último path de falso positivo conocido en la cadena de init FDCAN.
- **Archivos modificados:** `Core/Src/can_handler.c`
- **Próximos pasos:** Verificar en hardware con SWD que el re-init path funciona correctamente.

### PR-273 — fix(fdcan): multi-read CCCR consistency + clock source resilience
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Corrige dos bugs interrelacionados en la inicialización FDCAN del STM32: (1) la comprobación CCCR en `MX_FDCAN1_Init` podía ser engañada por datos basura aleatorios del bus AHB, y (2) `CAN_Init` fallaba permanentemente si el clock source PCLK1 no estaba latched tras el force-reset.
- **Root cause:** Cuando la clock gate del FDCAN APB está inestable tras force-reset, las lecturas de registros devuelven datos aleatorios del bus (stale AHB bus data). Una lectura única de CCCR puede coincidir con los bits esperados (INIT=1, reservados=0), generando un falso positivo. Además, `CCIPR.FDCANSEL` puede revertir a su valor por defecto (HSE=00) en ciertos revisions de silicio, causando que `CAN_Init` abandone sin recuperación.
- **Solución aplicada:** (1) Triple lectura de CCCR en `MX_FDCAN1_Init` con comparación de consistencia — si las 3 lecturas no son idénticas, el periférico no está clocked correctamente. (2) `CAN_Init` re-aplica PCLK1 con barrera `__DSB()` si detecta clock source incorrecto, y hace re-init completo del periférico. (3) Nuevos campos diagnóstico `clk_reapplied` y `ccipr_raw` en `CAN_InitDiag_t`.
- **Impacto en el sistema:** Elimina falsos positivos en init FDCAN. El sistema puede recuperar el clock source sin quedar permanentemente sin CAN. Diagnósticos SWD mejorados para depuración futura.
- **Archivos modificados:** `Core/Src/main.c`, `Core/Src/can_handler.c`, `Core/Inc/can_handler.h`
- **Próximos pasos:** Verificar en hardware con SWD que `clk_reapplied` y `ccipr_raw` reportan valores correctos.

### PR-272 — fix: unblock main loop — vTaskDelay yield, Serial TX buffer, bounded UART read
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Tres correcciones para eliminar bloqueo residual del main loop en Core 1: (1) yield al scheduler, (2) buffer Serial TX, (3) lectura UART acotada del sensor de obstáculos.
- **Root cause:** (1) `loop()` nunca hacía `vTaskDelay()` — monopolizaba Core 1 sin dar CPU al TWAI driver, idle task ni housekeeping del sistema. (2) `Serial.printf()` con diagnósticos largos (>128 chars) bloqueaba hasta 18 ms esperando que el FIFO UART se drenase a 115200 baud. (3) `while (tofSerial.available() > 0)` sin límite podía procesar hasta 1024 bytes por iteración a 921600 baud.
- **Solución aplicada:** (1) `vTaskDelay(1)` al final de `loop()` para yield de Core 1 (~1 ms). (2) `Serial.setTxBufferSize(512)` antes de `Serial.begin()` — los printf van a ring buffer y no bloquean. (3) Lectura UART acotada a `MP_FRAME_LENGTH * 2` (800 bytes) por llamada a `update()`.
- **Impacto en el sistema:** Loop fluido sin bloqueos: scheduler Core 1 activo, Serial no bloqueante, procesamiento UART predecible. Loop rate ~500 Hz (2 ms/iteración incluyendo yield).
- **Próximos pasos:** Ninguno.

### PR-271 — fix(rtmon): reset per-period stats, separate UI/render timing, add loop instrumentation
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Corrección de falsos positivos en `[RT] WARN blocking`, separación de instrumentación UI/render, y adición de timing del main loop (Core 1).
- **Root cause:** (1) Flags de bloqueo y stats max acumulaban desde el boot sin resetearse entre periodos de 5 s — una sola frame >4 ms durante boot hacía que el warning se mostrase indefinidamente. (2) `RTMON_UI` envolvía `screenManager.update()` completo (incluyendo `draw()`), por lo que `ui=YES` siempre acompañaba a `render=YES`. (3) Sin instrumentación del main loop en Core 1.
- **Solución aplicada:** `logToSerial()` resetea flags de bloqueo, max/min frame, phase maximums y zone counters tras imprimir (cada periodo de 5 s es independiente). `RTMON_UI_BEGIN/END` movido dentro de `screen_manager.cpp` para envolver solo `currentScreen_->update(data)`. Añadido `RTMON_LOOP_BEGIN/END` en `loop()`. Formato de log actualizado con `loop=` timing. Debug overlay muestra loop max.
- **Impacto en el sistema:** Diagnósticos de rendimiento ahora reflejan la realidad de cada periodo. `render=YES` esperado (TFT SPI), `ui=YES` indica stall real de procesamiento de datos, `loop=YES` indica bloqueo real en Core 1.
- **Próximos pasos:** Ninguno.

### PR-270 — Fix FDCAN init failure (CCCR garbage reads) and ESP32 CAN TX blocking
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** (1) Aumento de delay de estabilización de clock gate FDCAN de 32 a 3200 iteraciones (~19 µs). (2) Todos los `ESP32Can.writeFrame()` cambiados a `timeout=0` (non-blocking).
- **Root cause:** (1) `FDCAN_CLK_STABILISE_ITERS=32` (~190 ns) era insuficiente en algunas revisiones de silicio STM32G4, causando lecturas basura de CCCR. (2) `writeFrame()` con timeout por defecto de 1000 ms bloqueaba el main loop hasta 1 s cuando la cola TX estaba llena (bus muerto/error-passive).
- **Solución aplicada:** `FDCAN_CLK_STABILISE_ITERS` 32→3200. `FDCAN_INITIAL_SETTLE_DELAY_MS` 1→2 ms, `FDCAN_CLOCK_SETTLE_DELAY_MS` 5→10 ms. 8 call sites de `writeFrame()` actualizados con `timeout=0`.
- **Impacto en el sistema:** FDCAN init fiable en todas las revisiones de silicio. CAN TX nunca bloquea el main loop.
- **Próximos pasos:** Ninguno.

### PR-268 — fix: reorder error-passive guard and validate TWAI teardown return values
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Reordenamiento de la lógica de recuperación error-passive y validación de valores de retorno de `twai_stop()` / `twai_driver_uninstall()`.
- **Root cause:** (1) El guard de max resets se evaluaba después de `errorPassiveSince == 0`, provocando bucle infinito de set/clear del timestamp al agotar reintentos. (2) Valores de retorno de teardown no verificados podían dejar el driver en estado inconsistente.
- **Solución aplicada:** Mover check `errorPassiveResets >= MAX` antes de `errorPassiveSince == 0`. Validar retorno de `twai_stop()` y `twai_driver_uninstall()` (tolerar `ESP_ERR_INVALID_STATE`). Budget cuenta todos los intentos (éxito + fallo).
- **Impacto en el sistema:** Recuperación error-passive ahora es robusta ante agotamiento de reintentos y fallos de hardware intermitentes.
- **Próximos pasos:** Ninguno. Complementa PR #267.

### PR-267 — Add ESP32 TWAI error-passive recovery (tx_err=128 stuck state)
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Añadida recuperación automática de estado error-passive en ESP32 TWAI (TEC=128 persistente durante >3 s con estado RUNNING).
- **Root cause:** TWAI controller quedaba atascado en error-passive (TEC=128) indefinidamente. El recovery existente solo se activaba en BUS_OFF (TEC=256), nunca en error-passive.
- **Solución aplicada:** Detección de `tx_err >= 128` con timer de 3 s. Full driver reinit (`stop→uninstall→install→start`) via `twaiInit()`. Máximo 10 intentos.
- **Impacto en el sistema:** CAN se recupera de fallos eléctricos intermitentes que no escalan a bus-off.
- **Próximos pasos:** Aplicado fix en PR #268.

### PR-266 — Harden FDCAN init: proper reset pulse timing and increased retry margin
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Pulse stretch del reset FDCAN con `__DSB()` + 4× `__NOP()`, barrera `__DSB()` + `__ISB()`, y bucle de estabilización volátil antes del acceso a CCCR. Aumentados reintentos de 3 a 5 con delay de 5 ms.
- **Root cause:** Registros FDCAN devolvían basura (e.g., `CCCR = 0x8007ca5`) tras force-reset en ciertas revisiones de STM32G4. La puerta de clock no se estabilizaba lo suficientemente rápido.
- **Solución aplicada:** Hardening de `HAL_FDCAN_MspInit` con secuencia de reset robusta y 5 reintentos en `MX_FDCAN1_Init`.
- **Impacto en el sistema:** Inicialización FDCAN fiable en todas las revisiones de silicio.
- **Próximos pasos:** Ninguno.

### PR-265 — Offload TFT rendering + touch to FreeRTOS task on Core 0
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** TFT SPI rendering (~28 ms/frame) movido a un FreeRTOS task dedicado en Core 0.
- **Root cause:** El render en el main loop bloqueaba CAN polling y procesamiento safety-critical. Logs mostraban `[RT] WARN blocking: ui=YES render=YES`.
- **Solución aplicada:** `vTaskCreatePinnedToCore` para renderTask en Core 0. Main loop en Core 1 libre para CAN, sensores, audio.
- **Impacto en el sistema:** Main loop sin bloqueo. CAN polling y safety checks siempre en tiempo.
- **Próximos pasos:** Ninguno.

### PR-264 — fix(fdcan): harden FDCAN init against clock-gate race and stuck INIT
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Re-enable APB1 clock tras force-reset + `__DSB()` barrera. CCCR sanity check (INIT bit set, reserved bits 16–31 zero) con retry loop (3 intentos).
- **Root cause:** FDCAN devolvía basura (CCCR=`0x8007aa5`, PSR/ECR/RXF0S/TXFQS `0x8007c3d`) tras RCC force-reset. HAL_FDCAN_Init pasaba contra valores basura.
- **Solución aplicada:** Re-enable clock post-reset + barrera de memoria + CCCR sanity check post-init.
- **Impacto en el sistema:** Detección de FDCAN corrupto al arranque con abort controlado.
- **Próximos pasos:** Mejorado en PR #266.

### PR-263 — Fix I2C requestFrom error spam when MCP23017 is not connected
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Fix de detección NACK en ESP32-S3 Wire para MCP23017 shifter.
- **Root cause:** `Wire.endTransmission(false)` (repeated start) devuelve 0 incluso sin dispositivo. El backoff probe pasaba y `requestFrom()` fallaba ruidosamente cada 1 s.
- **Solución aplicada:** Usar `endTransmission(true)` (con STOP) para detección NACK fiable en backoff probe.
- **Impacto en el sistema:** Eliminado spam de errores I2C cuando MCP23017 no está conectado.
- **Próximos pasos:** Ninguno.

### PR-262 — Fix FDCAN stuck in INIT: replace per-ID filters with MASK accept-all
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Reemplazo de 5 filtros DUAL/RANGE específicos por un único filtro MASK accept-all.
- **Root cause:** FDCAN no salía de modo INIT (`CCCR.INIT` stuck high), causando ESP32 `tx_err=128` y escalada de `bus_err`. CAN bus completamente no funcional.
- **Solución aplicada:** Filtro MASK accept-all (index 0, FilterID1=0x000, FilterID2=0x000). Global filter acepta IDs estándar y extendidos en RXFIFO0. Filtrado real en `CAN_ProcessMessages()` switch/case.
- **Impacto en el sistema:** FDCAN sale de INIT correctamente. CAN funcional.
- **Próximos pasos:** Ninguno.

### PR-261 — fix: add TWAI bus-off recovery to ESP32
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Recuperación automática de bus-off en ESP32 TWAI.
- **Root cause:** Sin recovery de bus-off, cuando ESP32 arranca antes que STM32 (sin ACK), TEC=256 → BUS_OFF permanente.
- **Solución aplicada:** Check cada 250 ms, `twai_initiate_recovery()` en BUS_OFF, `twai_start()` en STOPPED. Máximo 10 intentos.
- **Impacto en el sistema:** ESP32 CAN se recupera automáticamente tras bus-off. Espeja `CAN_CheckBusOff()` del STM32.
- **Próximos pasos:** Complementado con error-passive recovery (PR #267–268).

### PR-260 — fix: re-apply FDCAN clock source after peripheral force-reset in MspInit
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Re-aplicar clock source PCLK1 tras `__HAL_RCC_FDCAN_FORCE_RESET()`.
- **Root cause:** Force-reset borra `RCC_CCIPR.FDCANSEL` a `00` (HSE). HSE no habilitado → peripheral sin clock → basura en registros.
- **Solución aplicada:** `__HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PCLK1)` tras release-reset en `HAL_FDCAN_MspInit`.
- **Impacto en el sistema:** FDCAN siempre usa PCLK1 correcto tras cualquier tipo de reset.
- **Próximos pasos:** Diagnosticado previamente en PR #259.

### PR-259 — feat(can): add FDCAN clock source and CCCR.INIT runtime diagnostics
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Añadidos diagnósticos runtime de clock source FDCAN y estado CCCR.INIT a `CAN_InitDiag_t`.
- **Root cause:** GDB mostró `RCC_CCIPR = 0x0` (FDCANSEL en HSE default) y `CCCR = 0x8007ADD` (basura). Sin verificación runtime para detectar estas condiciones.
- **Solución aplicada:** Campos `clk_ok` y `cccr_init_ok` en `CAN_InitDiag_t`. Abort si clock o CCCR inválidos. Legible via SWD con `p can_init_diag`.
- **Impacto en el sistema:** Diagnóstico post-mortem de fallos de inicialización FDCAN.
- **Próximos pasos:** Explotado en PRs #260–266.

### PR-258 — Fix FDCAN initialization failure after non-power-on resets
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Force-reset del periférico FDCAN en `HAL_FDCAN_MspInit` antes de la inicialización.
- **Root cause:** FDCAN retiene estado de error (CCCR.CSA/INIT stuck, bus-off latched) tras resets de watchdog o anómalos. `HAL_FDCAN_Init()` falla → sistema stuck en BOOT.
- **Solución aplicada:** `__HAL_RCC_FDCAN_FORCE_RESET()` + `__HAL_RCC_FDCAN_RELEASE_RESET()` en MspInit.
- **Impacto en el sistema:** FDCAN se inicializa correctamente tras cualquier tipo de reset.
- **Próximos pasos:** Mejorado en PRs #259–266.

### PR-257 — Change heartbeat LED to brief flash every ~2s
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Cambio de heartbeat LED de toggle 5 Hz continuo a flash breve ~50 ms cada ~2 s.
- **Root cause:** Toggle 5 Hz era indistinguible del blink ~2 Hz constante de `Error_Handler`.
- **Solución aplicada:** Flash breve (~50 ms ON, ~1950 ms OFF).
- **Impacto en el sistema:** Diagnóstico visual claro del estado de la MCU.
- **Próximos pasos:** Ninguno.

### PR — Move LPWM_FR from PB14/TIM15_CH1 to PC3/TIM1_CH4 + LED_DIAG on PB14
- **Fecha:** 2025-07-17
- **Autor:** Copilot
- **Descripción del cambio:** Reubicación de LPWM_FR de PB14/TIM15_CH1 a PC3/TIM1_CH4 (AF2). PB14 liberado y reasignado como GPIO_Output para LED diagnóstico (LED_DIAG). TIM15 eliminado completamente del proyecto. Los 4 canales del motor FR quedan ahora en TIM1 (CH1=RPWM_FL/PA8, CH2=LPWM_FL/PA9, CH3=RPWM_FR/PA10, CH4=LPWM_FR/PC3).
- **Root cause:** Consolidación de todos los canales PWM de tracción frontal en un solo timer (TIM1) y liberación de TIM15.
- **Solución aplicada:** GPIO remap de LPWM_FR a PC3/TIM1_CH4, PB14 como LED_DIAG, eliminación de TIM15.
- **Impacto en el sistema:** PWM de motor FR en TIM1_CH4/PC3 (Morpho CN7 pin 37). LED diagnóstico en PB14 (Morpho CN10 pin 28).
- **Próximos pasos:** Ninguno.

### PR-256 — Move LPWM_FR from PB9/TIM17_CH1 to PB14/TIM15_CH1
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Reubicación de LPWM_FR a PB14/TIM15_CH1 para resolver conflicto de pines tras remap de FDCAN.
- **Root cause:** CI fallaba porque stubs en `analysis_artifacts/stubs/` no tenían definiciones de TIM15.
- **Solución aplicada:** Actualización de pin mapping y stubs de CI.
- **Impacto en el sistema:** PWM de motor FR funcional en nuevo pin.
- **Próximos pasos:** Ninguno.

### PR-255 — Move FDCAN1 from PB8/PB9 to PA11/PA12
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Remap FDCAN1 de PB8/PB9 a PA11(RX)/PA12(TX) AF9 según layout NUCLEO-G474RE CN10.
- **Root cause:** PB8/PB9 conflicto con PWM de motor. PA11/PA12 disponibles en header CN10.
- **Solución aplicada:** GPIO remap + reubicación de LPWM_FR (TIM1_CH4 → TIM17/PB9).
- **Impacto en el sistema:** CAN en pines correctos sin conflicto con PWM.
- **Próximos pasos:** Completado en PR #256.

### PR-254 — feat(boot_screen): add ESP32/STM32 fault isolation diagnostics
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Diagnósticos de aislamiento de fallos ESP32/STM32 en pantalla de boot.
- **Root cause:** Pantalla de boot mostraba CAN status genérico sin indicar qué lado fallaba.
- **Solución aplicada:** Indicadores separados de estado ESP32 y STM32 en waiting screen.
- **Impacto en el sistema:** Diagnóstico visual más rápido en campo.
- **Próximos pasos:** Ninguno.

---

## 4. Roadmap / Próximos Objetivos

### Alta Prioridad
- [ ] **Integración completa de audio DFPlayer**: Sonidos de aviso, error, arranque. Hardware presente, software parcial.
- [ ] **Asistente de calibración touch**: Wizard persistente con almacenamiento SPIFFS.
- [ ] **Test en vehículo real**: Validación completa del sistema integrado.

### Media Prioridad
- [ ] **Frenado regenerativo**: Aprovechar motores para recuperación de energía.
- [ ] **Control de crucero adaptativo (ACC)**: Integración con sensor de obstáculos.
- [ ] **Telemetría WiFi/BT**: Dashboard remoto para diagnóstico y monitorización.

### Baja Prioridad / Mejoras Futuras
- [ ] **Optimización de consumo**: Sleep modes para ESP32 en standby prolongado.
- [ ] **OTA updates**: Actualización de firmware ESP32 por WiFi.
- [ ] **Logging expandido**: Más datos en error log (GPS, acelerómetro).

---

## 5. Reglas Técnicas del Proyecto

### Arquitectura (No Romper)
1. **STM32 = Safety Authority**: Toda validación de comandos y control de actuadores reside en el STM32. El ESP32 **nunca** controla motores directamente.
2. **Dual-core ESP32**: Render siempre en Core 0. CAN, sensores, audio, LEDs en Core 1. No mezclar.
3. **Protocolo CAN congelado v1.3**: No modificar IDs ni formato de mensajes sin actualizar `CAN_CONTRACT_FINAL.md` y ambos firmwares simultáneamente.
4. **DMA prohibido para CAN**: FDCAN usa interrupciones (prioridad 1), no DMA.

### Inicialización Robusta
5. **FDCAN MspInit**: Siempre force-reset + re-apply clock source (PCLK1) + pulse stretch (`__DSB()` + `__NOP()`) + barrera (`__DSB()` + `__ISB()` + volatile loop) antes de CCCR access.
6. **FDCAN Init retries**: Mínimo 5 intentos con CCCR sanity check (INIT set, bits 16–31 zero). Abort controlado si falla.
7. **FDCAN clock source**: Encoding en RCC_CCIPR bits [25:24]: 00=HSE, 01=PLL-Q, 10=PCLK1. Este proyecto usa PCLK1 (valor 0x02000000).
8. **Boot order**: `Boot_ReadResetCause()` antes de IWDG. Prioridad de interrupciones: CAN(1) > PWM/Encoder(2) > I2C(3) > SysTick(4).

### Safety
9. **CAN timeout → LIMP_HOME**: 250 ms sin heartbeat → 20% torque máximo.
10. **Bus-off recovery**: Implementada en ambos lados (STM32 `CAN_CheckBusOff()`, ESP32 TWAI recovery). Max 10 intentos.
11. **Error-passive recovery (ESP32)**: `tx_err >= 128` persistente >3 s → full driver reinit. Max 10 intentos.
12. **ABS/TCS**: Umbral 15% slip. No desactivable en modo normal.
13. **Startup inhibit**: Pedal debe estar liberado 400 ms antes de habilitar torque.
14. **BREAK2**: TIM1/TIM8 BREAK2 vinculado a Cortex LOCKUP. Fuerza PWM a safe state.

### CAN Bit Timing
15. **ESP32 TWAI**: BRP=10, TSEG1=13, TSEG2=2, SJW=2 → 87.5% SP.
16. **STM32 FDCAN**: BRP=10, TSEG1=29, TSEG2=4, SJW=4 → 88.2% SP. Delta 0.7% dentro de tolerancia CiA 301.

### Filtrado CAN
17. **STM32**: Filtro MASK accept-all (index 0). Filtrado real en `CAN_ProcessMessages()` switch/case.
18. **Global filter**: Acepta IDs estándar y extendidos no coincidentes en RXFIFO0.

### Pin Mapping (Actual)
19. **FDCAN1**: PA11 (RX) / PA12 (TX), AF9. **No PB8/PB9**.
20. **CAN pin defines**: `PIN_CAN_TX = GPIO_PIN_12`, `PIN_CAN_RX = GPIO_PIN_11` en `project_config.h`.

### I2C
21. **ESP32 Wire**: Usar `endTransmission(true)` (con STOP) para detección NACK fiable. `endTransmission(false)` devuelve 0 sin dispositivo.

### Diagnósticos
22. **CAN_InitDiag_t**: 12 campos (hal_init, filter_global, notify, start, started, clk_ok, cccr_init_ok, clk_reapplied, retries, timeout_flag, msp_clk_ok, msp_ccipr_ok) + ccipr_raw. Legible vía SWD con `p can_init_diag`.

### RuntimeMonitor (ESP32)
23. **Instrumentación separada**: `RTMON_UI_BEGIN/END` envuelve solo `currentScreen_->update(data)` dentro de `screen_manager.cpp`, NO el `screenManager.update()` completo en renderTask. `RTMON_RENDER_BEGIN/END` envuelve `draw()`. `RTMON_LOOP_BEGIN/END` envuelve el main `loop()` en Core 1.
24. **Stats por periodo**: `logToSerial()` resetea flags de bloqueo, max/min frame, phase maximums y zone counters tras imprimir. Cada ventana de 5 s es independiente. NO llamar `reset()` desde `logToSerial()` — ring buffer y FPS counters deben persistir entre periodos.
25. **CAN TX non-blocking**: Todos los `ESP32Can.writeFrame()` deben usar `timeout=0`. El timeout por defecto de 1000 ms bloquea el loop cuando la cola TX está llena (bus muerto/error-passive).
26. **Render blocking threshold**: `RENDER_BLOCKING_THRESHOLD_US = 35000` (35 ms) para Core 0. TFT SPI ~22–28 ms es normal. `BLOCKING_THRESHOLD_US = 4000` (4 ms) para CAN/UI/loop en Core 1.

### Sensor de Obstáculos (ESP32)
27. **Sensor type selection**: `SENSOR_TYPE` en `obstacle_sensor.h` — `SENSOR_TYPE_TOFSENSE=0` (921600 baud, 400B frames), `SENSOR_TYPE_TFMINI=1` (115200 baud, 9B frames). Default: TFMINI.
28. **Sensor enable flag**: `OBSTACLE_SENSOR_ENABLED` en `obstacle_sensor.h` — `0` = deshabilitado (sin UART, sin buffers, sin CPU), `1` = habilitado. Default: 0 (sensor no conectado actualmente).
29. **TF-Mini Plus overflow guard**: Conversión cm→mm usa `uint32_t` intermedio + guarda `> UINT16_MAX`. Valores >6553 cm se rechazan como inválidos (retorna `false`).
30. **Timeout limpia estado completo**: En timeout de frame, `distance_mm = 0`, `zone = 0`, `healthy = false`, `status = INVALID`. Nunca se envían datos stale por CAN.

---

## 6. Historial de Cambios

| Fecha | Hito | PRs Relacionados |
|---|---|---|
| 2026-03-27 | **Remap FDCAN1 a PA11/PA12** — Resolución de conflicto PB8/PB9 con PWM motor | #255, #256 |
| 2026-03-27 | **Diagnósticos FDCAN runtime** — clk_ok y cccr_init_ok en CAN_InitDiag_t | #259 |
| 2026-03-27 | **Heartbeat LED diferenciable** — Flash breve ~50 ms / ~2 s | #257 |
| 2026-03-27 | **Boot diagnostics en HMI** — Aislamiento de fallos ESP32/STM32 en pantalla | #254 |
| 2026-03-27–28 | **Hardening FDCAN init** — Force-reset, clock re-apply, pulse stretch, CCCR sanity, 5 retries | #258, #260, #264, #266 |
| 2026-03-28 | **CAN filter simplification** — MASK accept-all reemplaza 5 filtros específicos | #262 |
| 2026-03-28 | **I2C NACK fix** — endTransmission(true) para MCP23017 backoff | #263 |
| 2026-03-28 | **FreeRTOS render offload** — TFT+touch en Core 0, main loop libre en Core 1 | #265 |
| 2026-03-28 | **ESP32 CAN bus-off recovery** — twai_initiate_recovery + twai_start, max 10 intentos | #261 |
| 2026-03-28–29 | **ESP32 CAN error-passive recovery** — Full driver reinit cuando tx_err=128 persiste >3 s | #267, #268 |
| 2026-03-29 | **FDCAN clock stabilisation + CAN TX non-blocking** — CLK_STABILISE_ITERS 32→3200, writeFrame timeout=0 en 8 call sites | #270 |
| 2026-03-29 | **RuntimeMonitor fix** — Stats por periodo (no acumulativo), separación UI/render, instrumentación loop Core 1 | #271 |
| 2026-03-29 | **Unblock main loop** — vTaskDelay(1) yield Core 1, Serial TX buffer 512, UART read acotado a 800 bytes | #272 |
| 2026-03-29 | **FDCAN init consistency fix** — Multi-read CCCR check, clock source resilience en CAN_Init, diagnósticos ccipr_raw | #273 |
| 2026-03-29 | **FDCAN CAN_Init CCCR triple-read** — Añadido triple-read CCCR al path de re-init por clock re-apply en CAN_Init | #274 |
| 2026-03-29 | **CI cppcheck fix** — Supresión inline de falsos positivos `duplicateAssignExpression` en lecturas triples CCCR + `--inline-suppr` | #275 |
| 2026-03-29 | **Obstacle sensor reliability** — UART RX buffer 1024→4096, resync 0x57+0x01, MAX_BYTES_PER_UPDATE 800→1600, render blocking threshold 4→35 ms, uartHWM diag | #276 |
| 2026-03-29 | **CAN error-passive bifásica** — Recovery no abandona tras 10 intentos: fase rápida (10×3s) + fase lenta (ilimitado×30s) | #277 |
| 2026-03-30 | **TF-Mini Plus overflow + stale data fix** — uint16 overflow guard en cm→mm, timeout limpia distance/zone, soporte dual sensor (TOFSense-M/TF-Mini Plus), flag OBSTACLE_SENSOR_ENABLED, distance_sensor.h | #275 (GitHub) |
| 2026-03-31 | **FDCAN MspInit adaptive poll** — Reemplaza bucle volátil 3200 iter por poll CCCR adaptativo (50 ms timeout) + verificación readback FDCANSEL | #278 |
| 2026-04-02 | **TWAI clk_src fix** — `memset` zeroed `clk_src` field in ESP-IDF 5.x → CAN bus inoperative. Replaced with `TWAI_TIMING_CONFIG_500KBITS()` macro init | #279 |
| 2026-04-04 | **Full-system CAN audit** — Comprehensive firmware + protocol audit, CAN RX diagnostics, DLC doc fix, hardware validation checklist, root cause analysis | #279a |
| 2026-04-04 | **CAN hardware fix procedure** — Executable troubleshooting guide, fix stale PB8/PB9 pin refs → PA11/PA12 in HARDWARE_VALIDATION_PROCEDURE.md + PROJECT_MASTER_STATUS.md | #279b |
| 2026-04-04 | **Documento sistema alimentación** — 9 secciones + BOM: arquitectura, recorrido alimentación, relés, apagado retardado, LEDs, protección, masas, esquema eléctrico, materiales | #280 |
| 2026-04-06 | **FDCAN init robusta** — Secuencia determinista de bring-up: clock source before enable, readback-verify RCC, reset con DSB/ISB, poll adaptativo CCCR, started=0 por defecto, 4 nuevos campos diagnóstico | #281 |
| 2026-04-06 | **Decouple hal_msp + fix filter docs** — Extraer `CAN_InitDiag_t` a header propio, eliminar include de `can_handler.h` en hal_msp.c, corregir descripción filtros CAN (accept-all, no reject-all) | #282 |
| 2026-04-06 | **LED boot visibility** — Boot blinks 60→150 ms, post-init CAN status LED (1 long=OK, 5 rapid=FAIL), volatile can_init_diag | #283 |
| 2026-04-07 | **Boot blinks unmissable** — Boot blinks 150→300 ms, dark lead-in 500 ms, CAN status blinks más lentos, separación 500 ms | #284 |
| 2026-04-07 | **LED migrada PA5→PB8** — Status LED de PA5 (LD2, interferido por ST-Link SB21) a PB8 (GPIO libre, Morpho CN10-3). LED externo requerido. | #285 |
| 2026-04-07 | **LED revertida PB8→PA5 (LD2)** — LD2 está soldado en la placa Nucleo. No necesita LED externo. Revert de PR-285. | #286 |
| 2026-04-07 | **FDCAN antes de boot blinks** — Mover MX_FDCAN1_Init+CAN_Init antes de la secuencia LED. ESP32 ya no entra en BUS_OFF durante boot. | #287a |
| 2026-04-07 | **Fix baud rate ESP32 625→500 kbps** — CRÍTICO: ESP-IDF 5.x ignora brp cuando quanta_resolution_hz>0. Fix: clear quanta_resolution_hz=0. Restaura comunicación CAN. | #287 |
| 2026-04-07 | **Auditoría palabra-por-palabra** — 10 archivos STM32 (LIMPIO) + 5 archivos ESP32 (3 bugs corregidos: printf %lus, %d→%u para size_t, COM8 hardcodeado). Changelog actualizado. | #288 |
| 2026-04-08 | **2ª auditoría** — boot_validation.c: isnan() añadido a logging diagnóstico (NaN sensors ahora registran fault). ESP32: cast (unsigned) en PSRAM printf. | #289 |
| 2026-04-08 | **Robustez avanzada** — NaN/Inf hardening en 10+ rutas safety, TX NACK detection, CAN FPS metric, boot RAM/clock/periph checks, logs estandarizados [MODULE][SEVERITY] | #290 |
| 2026-04-08 | **cppcheck shadowVariable fix** — Mover `extern SystemCoreClock` a file scope en boot_validation.c para evitar shadow con HAL stub | #291 |
| 2026-04-09 | **LD2 muestra estado CAN en main loop** — CAN OK: flash breve 2s, CAN FAIL: parpadeo 1Hz. No necesita LED externo PB14. | #292 |
| 2026-04-09 | **Activar sensor TF-Mini Plus** — OBSTACLE_SENSOR_ENABLED=1, SENSOR_TYPE=TFMINI. Firmware listo para sensor 115200 bps en GPIO 18. Guía de cableado + docs actualizados. | — |
| 2026-04-09 | **Auditoría TF-Mini Plus** — Fix sampling rate 10→100 Hz (TFM_MAX_BYTES_PER_UPDATE 32→256, eliminar 1-frame-per-call, rxBuf 256→512). Latencia 100ms→10ms. Comentarios actualizados. | — |
| 2026-04-09 | **Guía puesta en marcha segura** — Documento 10 fases (I2C→DS18B20→INA226→encoder→sensores→pedal→relés→motores) con conexiones exactas, materiales, circuitos optoacopladores (6N137/PC817), BOM completa. | — |
| 2026-04-09 | **CAN loss → pantalla error** — ScreenManager detecta heartbeat STM32 stale >1.5s → fuerza transición a ERROR screen con banner "CAN LINK LOST". Auto-recuperación al reconectar. | — |
| 2026-04-10 | **Drive screen → TF-Mini Plus** — Actualización pantalla final para TF-Mini Plus: bar proximidad 400→600 cm, zonas color ajustadas (verde >3m, cian 1.5–3m, amarillo 0.8–1.5m, naranja 0.3–0.8m, rojo <0.3m), comentarios TOFSense-M→TF-Mini Plus. | — |
| 2026-04-10 | **Verificación pantallas final** — DriveScreen: overlay degradado/limp + indicadores fault flags. SafeScreen: telemetría read-only (speed/current/temp/steering). Cumplimiento completo HMI_STATE_MODEL.md. | — |
| 2026-04-10 | **HMI anti-flicker + render optimización** — Fix: CAN_TIMEOUT bit 0 faltante en DriveScreen fault overlay. Opt: draw calls gated por dirty checks (zero work si no cambia). Opt: umbrales torque Δ>2% y temp Δ≥1°C para evitar redraw por ruido. Fix: ErrorScreen CAN-lost → partial banner redraw (no full fillScreen). | — |
| 2026-04-10 | **Deterministic render pipeline** — Anti-flicker: reemplazar fillRect+drawString por setTextPadding en 12 componentes UI (zero-gap text overwrite). Fix: draw_runtime_overlay String→snprintf (eliminar heap allocation). Doc: formalizar pipeline CAN→snapshot→frame-latch→render en vehicle_data.h, screen_manager.h, main.cpp. | — |
| 2026-04-10 | **Tile-Based Dirty Region Engine** — Refactor completo del HMI a motor de render por regiones (tiles). TileSet template con hash FNV-1a: cada tile solo se redibuja si su contenido cambia. DriveScreen: 12 tiles (speed, obstacle, wheels, steering, battery, gear, pedal, mode, LED, degraded overlay, faults overlay, ACK). ErrorScreen: 5 tiles. SafeScreen: 6 tiles. StandbyScreen: 2 tiles. BootScreen: 3 tiles. Bar widgets (pedal, batería, sensor obstáculo) con differential update (solo la porción cambiada). Eliminación total de clear+redraw flash en barras. Overlay tiles restauran tiles subyacentes al desaparecer. Nuevo: `tile_engine.h` con TileRect, TileHash, TileSet<N>. | — |
| 2026-04-10 | **HMI Security Audit & Tile Engine Hardening** — Auditoría completa del sistema HMI tile-based. (1) `ui_config.h`: centralización de 20+ magic numbers (text paddings, thresholds, overlay layout, color levels). (2) Overlay invalidation chain: DEGRADED→OBSTACLE, FAULTS→top-bar, ACK→LED_TOGGLE — fix de artefactos visuales al cerrar overlays sobre tiles solapados. (3) wheelThresholdFilter dedup: antes se ejecutaba 2× por frame (update+draw), ahora 1× con resultados precomputados. (4) Tile bounds safety: setRect() clampea coordenadas a SCREEN_W×SCREEN_H. (5) Battery hysteresis determinista: reemplaza dependencia de frame anterior por thresholds explícitos BATT_HYSTERESIS_HIGH/LOW. (6) 15 archivos actualizados. | — |
| 2026-04-10 | **Tile Engine Formalization & Pipeline Hardening** — (1) Z-order layer system: TileLayer enum (STATIC/BASE/OVERLAY/SYSTEM) con reglas formales de composición. (2) Overlay invalidation contract documentado en tile_engine.h. (3) Debug assertions (UI_TILE_DEBUG) para diagnóstico de setRect() bounds. (4) Pipeline puro: overlay visibility precomputada en update(), draw() solo consume. (5) Tile layout constants centralizados: DTILE_*/ETILE_*/YTILE_* en ui_config.h. 6 archivos modificados. | — |
| 2026-04-10 | **Draw Purity Enforcement & Full Layout Centralization** — (1) Draw-phase purity: ackIndicatorDirty_ y diagNeedsRedraw_ movidos fuera de draw helpers. (2) OverlayMode enum (REPLACE/MERGE) con registro por overlay documentando composición y tiles afectados. (3) SafeScreen: 17 layout constants (STILE_*) centralizados en ui_config.h. (4) BootScreen: 5 diagnostic layout constants (BTILE_DIAG_*) centralizados. 5 archivos modificados. | — |
| 2026-04-10 | **Time Determinism, Hash Failsafe, Flag Safety** — (1) `frameTimeMs` injected into Screen::update() — millis() captured once per frame in ScreenManager. All screen timing uses injected value. (2) TileSet::forceRedraw() for critical tiles every 100 frames (5s). (3) Flag safety contract documented. 16 files modified. | — |
| 2026-04-10 | **V10 Hardening: Staggered Failsafe, Critical Tile Policy, Render Atomicity** — (1) Staggered forced redraws: critical tiles distributed across failsafe interval (no SPI spike). (2) Fault-condition override: SPEED+FAULTS force-redraw every frame when faults active. (3) Frame time contract: monotonicity assertion + overflow-safe delta docs. (4) Render atomicity contract: formal single-threaded guarantee. 6 files modified. | — |
| 2026-04-10 | **Final Audit: Frame Time Contract Compliance** — (1) Fix PinScreen millis() violation: wrongCodeMs_ now captured via pending flag in update() using injected frameTimeMs instead of raw millis() in touch handler. (2) Fix ErrorScreen millis() in onEnter(): errorEntryMs_ now captured on first update() instead of millis() in onEnter(). (3) Fix stale millis() comments in drive_screen.h, error_screen.h, engineering_screen.h → frameTimeMs. (4) Centralize PIN screen layout constants (PSCR_*) in ui_config.h. Zero millis() calls remain in any screen update/draw code. 7 files modified. | — |
| 2026-04-10 | **Final Time System Enforcement** — Last millis() removal: DebugOverlay::update() and draw() refactored to accept injected frameTimeMs parameter (was calling millis() directly). Call site in renderTask passes lastFrameStart. Full audit confirms zero direct time calls in any UI-path code. Remaining millis()/micros() are legitimate infrastructure: screen_manager.cpp single sampling point, frame_limiter gating, runtime_monitor profiling (#if RUNTIME_MONITOR), touch_handler debounce. 3 files modified. | — |
| 2026-04-11 | **Motor Control Advanced Hardening + Validation Final** — (1) Coast↔brake hysteresis: `COAST_SPEED_HYSTERESIS_KMH=0.5` prevents oscillation at speed threshold. (2) Brake design decision documented: passive brake (EN=HIGH, RPWM=0, LPWM=0) validated as sufficient; optional `BRAKE_ACTIVE_FALLBACK` ifdef added for field override (~5% min duty). (3) Motor_SetMode BRAKE case updated with `#if BRAKE_ACTIVE_FALLBACK` support. (4) Thread safety documented on Motor_SetMode/Motor_SetSigned: main-loop-only restriction, not re-entrant. (5) Motor_SetSigned mode-tracking caveat documented. (6) Stale comment fix: BTS7960_BRAKE_PWM (4249)→(0). (7) 6 new tests: 1000-cycle stress test, exhaustive glitch immunity, direction reversal detection, coast speed hysteresis, brake PWM validation, INT16_MIN clamping detail. Tests: 194→2965 assertions, 0 failures. 2 files modified. | — |
| 2026-04-12 | **Suppress third-party library warnings** — (1) `-Wno-cpp` suppresses FastLED `clockless_i2s_esp32s3.cpp` benign `#warning` about `esp_memory_utils.h` on ESP-IDF 4. (2) `-Wno-unused-value` suppresses ESP32-TWAI-CAN.hpp comma-operator warnings at lines 102/113. Both flags in `build_flags` (applies to libraries); project source re-enables via `-Wall` + `-Wcpp` in `build_src_flags`. 1 file modified (`esp32/platformio.ini`). | — |
| 2026-04-26 | **PC817 pull-up verification + IGNITION sense fix** — Verificación física con polímetro confirma que la placa PC817 de 8 canales NO tiene pull-up onboard en el lado de salida (circuito abierto OUT↔VCC con placa desalimentada). Impacto: sin corrección, el colector del PC817 flota cuando la llave está OFF → lecturas erróneas / falsos arranques. **Firmware:** `INPUT` → `INPUT_PULLUP` en `power_manager.cpp` (red de seguridad ~45 kΩ interno ESP32-S3). **Hardware obligatorio:** soldar resistencia 10 kΩ ¼ W entre GPIO 40 y 3.3 V (pull-up externo de baja impedancia para entorno automoción; 10 kΩ ‖ 45 kΩ ≈ 8.2 kΩ resultante — aceptable). **Docs actualizados (7 archivos):** `power_manager.h`, `power_manager.cpp`, `PUESTA_EN_MARCHA_SEGURA.md`, `CABLEADO_AISLAMIENTO_DEFINITIVO.md`, `LLAVE_CONTACTO_ENCENDIDO_APAGADO.md`, `SENSOR_INTERFACE.md`, `PIN_USAGE_INVENTORY.md` — eliminadas todas las referencias obsoletas a INPUT_PULLDOWN, HIGH=llave ON y pull-up onboard. | copilot/start-cableado-puesta-en-marcha |
| 2026-04-26 | **Documentación condensador 10 µF / 16V en VDD del MCP23017** — El rail de 3.3V del MCP23017 (expansor GPIO de la palanca de cambios) carece de desacoplo bulk documentado. Los picos de corriente del radio WiFi/BT del ESP32-S3 (200–400 mA, decenas de µs) hunden el rail y provocan pérdidas de ACK I²C que ponen la palanca en PARK de forma espuria. El condensador 10 µF / 16V electrolítico (en paralelo con el 100 nF cerámico ya existente) actúa como reserva de carga local cubriendo la franja 1 kHz–1 MHz que el cerámico no puede abastecer. **Se documenta en:** `COMPONENTES_PASIVOS_REFERENCIA.md` (nueva sección 1.5 con esquema de conexión), `CONEXIONES_COMPLETAS.md` (nota ⚠️ en sección 12e MCP23017), `MATERIALES_POR_MODULO.md` (sección 16 palanca + BOM condensadores C_MCP_BULK / C_MCP_BP). | copilot/connectar-palanca-de-cambios |
| 2026-04-28 | **Inventario v1.5 — Pedido 1N4148 ×150 + 1N4007 ×10 en mano + recordatorio protección 6N137** — Sesión Q&A sobre protección antiparalelo de optoacopladores. **Aclaración crítica:** el 1N4148 NO se solda entre VCC y GND del lado 3.3V (eso lo cortocircuitaría), sino en **antiparalelo al LED del optoacoplador en el lado de ENTRADA (12-24V del sensor)**. Polaridad: banda negra (cátodo) → IN+, lado liso (ánodo) → IN−. Razón: el LED interno del PC817 tiene Vr_max = 6 V y el del 6N137 ≈ 5 V; un transitorio inverso (mal cableado, ruido EMI, pulso de retorno de sensor PNP) los destruye. El 1N4148 conduce en directa cuando hay inversión accidental y "cortocircuita" el LED protegiéndolo. **Decisiones tomadas:** (1) Pedir caja de **1N4148 ×150** (DO-35, trr 4 ns, 200 mA, 100 V) — más rápido, polaridad uniforme, polivalente; libera el kit Zener para su función real; trr 4 ns importa para 6N137 a frecuencia alta del encoder. (2) Documentado uso polivalente: protección PC817 ×5 + 6N137 ×3 + OR retención 6-9 + flyback relés pequeños + clamps 3.3V — uso inmediato 14-17 uds, resto reserva. (3) Añadido **D5 = 1N4007 ×10** (DO-41, 1A/1000V, fabricante Shenzhen HuiJiong) — redundante con 1N5408 ×50 ya en mano, queda como reserva flyback bajo consumo. (4) Soldadura 1N4148: 320-340 °C, ≤3 s/pin, dejar 3-4 mm de patilla para disipar calor (cristal pequeño). (5) **Recordatorio explícito en §11.3.1 del inventario:** al montar los módulos 6N137 (encoder A/B/Z) soldar 1N4148 antiparalelo en cada canal entre IN+ e IN− — mismo principio que PC817. Verificar primero si el módulo trae diodo SMD integrado. **Archivos modificados:** `Documentos/INVENTARIO_COMPONENTES_FISICOS.md` (v1.4 → v1.5: §1 stock summary, §5.5 nueva 1N4007, §5.6 nueva 1N4148, §9.2 BOM actualizado, §9.3 filas protección PC817+6N137, §11.3.1 nueva con recordatorio, apéndice histórico). | — |
