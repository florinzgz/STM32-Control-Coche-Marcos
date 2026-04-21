# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project loosely follows [Semantic Versioning](https://semver.org/).

A richer, Spanish-language narrative history of every PR is maintained in
[`PROJECT_CHANGELOG.md`](PROJECT_CHANGELOG.md). This file is the concise,
machine-readable rollup used as the production-readiness release log.

---

## [Unreleased]

Cross-system refinement pass on the temperature and obstacle subsystems.
CAN contract v1.3 preserved — no CAN ID, DLC, byte layout, or timing change.

### Added

- **DS18B20 persistent physIdx→role mapping** with Flash persistence on
  STM32 (`Core/Src/sensor_map_store.c`) and UI-driven assignment on ESP32
  engineering screen. Consumed by `CAN_SendStatusTempMap()` so the
  reported `STATUS_TEMP_MAP` (0x206) bytes always follow the saved
  topology (FL/FR/RL/RR/AMB) regardless of 1-Wire enumeration order.
- **`CMD_SENSOR_MAP_TEMP` (CAN 0x112, DLC 5, ESP32→STM32, on-demand)** for
  live remapping of the 5 DS18B20 roles from the engineering menu, with
  `CMD_ACK` (0x103) response after safety validation. Persisted by
  `SensorMapStore_Save()` in Flash page 125.
- **Temperature diagnostics** in `Core/Src/temperature.c`: topology-change
  detection (`Temperature_HasTopologyChanged()` /
  `Temperature_ClearTopologyChanged()`), per-sensor stale counters, and
  explicit disconnect sentinel handling (−127 °C).
- **Obstacle zone EMA filter** in `esp32/src/sensors/obstacle_sensor.cpp`
  (integer α = 0.3 via named constants `ZONE_EMA_NUM` / `ZONE_EMA_DEN`,
  O(1), no malloc). Applied **only** to zone classification; raw
  `measuredMm` is preserved in `reading_.distance_mm` and hence in CAN
  `OBSTACLE_DISTANCE` (0x208), so the STM32 independent backstop keeps
  seeing unfiltered distance. Filter reseeds on `init()` and on
  timeout→INVALID transitions.
- **Exhaustive zone-boundary regression tests**
  (`esp32/src/test_obstacle_sensor.cpp`) covering 499/500/501,
  999/1000/1001, 1499/1500/1501, 1999/2000/2001, 3999/4000 mm on both
  the TOFSense-M and TF-Mini Plus code paths.

### Changed

- **Obstacle safety thresholds updated to the 50 cm minimum-distance
  policy**. `OBSTACLE_EMERGENCY_MM` = 500 mm (previously 200 mm):
  `safety_system.c::Obstacle_Update()` and the ESP32
  `distanceToZone()` map now use the same 500 / 1000 / 1500 / 2000 mm
  boundaries, and the STM32 applies `obstacle_scale` ∈
  {0.0, 0.3, 0.7, 0.85, 0.95, 1.0}.
- **UI color mapping aligned 1:1 with safety zones** in
  `esp32/src/ui/ui_common.h::proximityColor()`: red < 500 mm,
  orange 500–1000, yellow 1000–1500, cyan 1500–2000, green ≥ 2000.
  Eliminates the previous UI/safety mismatch in the 200–500 mm band.
- **Obstacle spatial hysteresis restored**. `OBSTACLE_RECOVERY_MM`
  raised from 500 → 750 mm, re-establishing a 250 mm band above
  `OBSTACLE_EMERGENCY_MM`. `OBSTACLE_CONFIRM_MS` (200 ms) and
  `OBSTACLE_CLEAR_MS` (1000 ms) unchanged.
- **Bootstrap plausibility filter added to temperature CAN TX**
  (`CAN_SendStatusTempMap()`): two-sample consensus within
  `BOOT_REF_TOLERANCE_C` and a 0–60 °C bootstrap window
  (`BOOT_MIN_VALID_C` / `BOOT_MAX_VALID_C`) before a role's
  `last_good_valid` is armed. Post-armed behaviour
  (`TEMP_MAX_STEP_C` step filter, −127 °C reset) is unchanged.
- **Documentation normalised to the 50 cm policy** in 13 files under
  `docs/`: `OBSTACLE_SYSTEM_ARCHITECTURE.md`, `SAFETY_ARCHITECTURE.md`,
  `SENSOR_INTERFACE.md`, `CAN_CONTRACT_FINAL.md`,
  `FAIL_OPERATIONAL_MIGRATION_AUDIT.md`,
  `ESP32_HMI_ARCHITECTURE_REBUILD.md`, `PROJECT_MASTER_STATUS.md`,
  `FIRMWARE_MATURITY_ROADMAP.md`, `TECHNICAL_REVIEW_REPORT.md`,
  `TFMINI_PLUS_WIRING_GUIDE.md`, `OBSTACLE_SENSOR_DESIGN_DECISION.md`,
  `OBSTACLE_HARD_CUTOFF_ANALYSIS.md`, `FIRMWARE_COMPLETION_ASSESSMENT.md`.

### Fixed

- **Boot-time EMI lock in temperature filtering**: R-1 plausibility
  window and R-2 topology-reset logic in `CAN_SendStatusTempMap()`
  prevent a false bootstrap consensus on two EMI-corrupted samples and
  purge stale `last_good_valid[]` state after a 1-Wire topology change,
  so remapping a sensor no longer causes silent `TEMP_MAX_STEP_C`
  rejection of the new reading.
- **Obstacle boundary instability near 500 mm**: zone flipping under
  TF-Mini Plus cm-scale jitter is suppressed by the EMA filter in
  combination with the restored 500→750 mm hysteresis band.
- **Documentation inconsistencies** — legacy `< 200 mm` / `200–500 mm`
  obstacle threshold references removed from current-behaviour docs.
- **Stale zone comment** in `Core/Src/safety_system.c`: `/* Last zone
  (0–5) */` → `/* Last zone (0–4) */`; expanded zone-mapping comments
  in `esp32/src/sensors/obstacle_sensor.h` (file header and
  `Reading::zone`) with the explicit 500/1000/1500/2000 mm tiers.

### Safety

- **Earlier obstacle intervention.** Emergency stop now triggers at
  500 mm instead of 200 mm, giving a larger braking margin while
  preserving the same `Safety_*` state machine and the 200 ms
  `OBSTACLE_CONFIRM_MS` temporal confirmation.
- **No false temperature spikes at boot.** The bootstrap filter
  guarantees that a role's CAN payload is either a sample that passed
  the plausibility window and step-filter, or the sensor's
  "disconnected" sentinel — −127 °C and NaN/Inf readings can no longer
  reach `STATUS_TEMP_MAP` (0x206) as apparent valid temperatures.
- **Stabilised sensor behaviour under EMI.** Obstacle zone EMA
  (α = 0.3) plus recovery hysteresis (250 mm band), and temperature
  bootstrap plausibility plus topology-change reset, together
  eliminate the two edge-case oscillation paths identified during the
  audit without altering any safety decision.

### Contracts preserved

- CAN IDs unchanged: `CMD_ACK` (0x103), `SERVICE_CMD` (0x110),
  `CMD_SENSOR_MAP_TEMP` (0x112), `STATUS_TEMP_MAP` (0x206),
  `OBSTACLE_DISTANCE` (0x208), `OBSTACLE_SAFETY` (0x209).
- DLCs unchanged: 0x103 = 3, 0x110 = 2, 0x112 = 5, 0x206 = 5,
  0x208 = 5, 0x209 = 4.
- Byte layouts unchanged on both STM32 (`Core/Src/can_handler.c`) and
  ESP32 (`esp32/src/can/can_obstacle.cpp`, `esp32/include/can_ids.h`).
- TX cadence unchanged: 0x206 @ 1000 ms, 0x208 @ 66 ms,
  0x209 @ 100 ms; 0x103 / 0x110 / 0x112 remain on-demand.
- `Safety_*` state machine and motor/PWM pipeline untouched.
- All new logic O(1), no `malloc`, no blocking calls.

---
