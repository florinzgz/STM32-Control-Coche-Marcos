# Static Analysis Suppressions

Date baseline created: 2026-07-10

## Rules

- No blanket `*:all` suppressions.
- Every suppression must stay tied to a specific tool finding and a concrete rationale.
- Generated/external code may be excluded or suppressed by file path, but owned firmware logic must stay visible.
- Any suppression that stops matching should be removed.

## Current Suppressions

| Tool | Scope | Finding | Reason | Classification |
| --- | --- | --- | --- | --- |
| cppcheck | `Drivers/CMSIS/Include/cmsis_gcc.h` | `comparePointers` | CMSIS macro internals are external vendor code and out of repo scope. | FALSE_POSITIVE / external |
| cppcheck | `Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_ll_adc.h` | `constParameterPointer` | HAL LL headers are vendor code; style suggestions there are not actionable in this repo. | FALSE_POSITIVE / external |
| cppcheck | `Core/Src/can_handler.c:450` | `knownConditionTrueFalse` | The code intentionally performs three hardware register reads to reject stale FDCAN bus data; cppcheck collapses the volatile reads. | FALSE_POSITIVE |
| cppcheck | `Core/Src/can_handler.c:1365` | `knownConditionTrueFalse` | `NUM_WHEELS` is 4 by design; the second bound documents the CAN packing contract explicitly. | FALSE_POSITIVE |
| cppcheck | `Core/Src/sensor_manager.c:1925` | `knownConditionTrueFalse` | The `i < 5U` bound documents the five exported stale bits even when `NUM_DS18B20` is lower. | FALSE_POSITIVE |
| cppcheck | `Core/Src/motor_control.c:360` | `unusedStructMember` | `Motor_t::power` is retained intentionally for ABI/layout compatibility, as documented in the source comment. | FALSE_POSITIVE |
| cppcheck | `Core/Src/steering_cal_store.c:66` | `unusedStructMember` | `reserved[3]` preserves the on-flash v1 layout and checksum compatibility. | FALSE_POSITIVE |

## Current Non-Suppressed Report-Only Findings

These remain visible for manual triage because they may deserve future cleanup but are not confirmed bugs today:

- STM32 / cppcheck
  - `Core/Src/can_handler.c:258` `constParameterPointer`
  - `Core/Src/sensor_manager.c:1835` `funcArgNamesDifferent`
- ESP32 / cppcheck
  - `esp32/src/config_store.cpp:380` defensive clamp marked as tautological
  - `esp32/src/main.cpp:1137` duplicate-condition warning needs manual review in full context
  - `esp32/src/remote_control.cpp:288,308` state-machine flow warning under static assumptions
- Lizard legacy hotspots
  - `Core/Src/motor_control.c`
  - `Core/Src/safety_system.c`
  - `Core/Src/can_handler.c`
  - `Core/Src/main.c`

## Review Process

1. Re-run `./scripts/run_static_analysis.sh`.
2. Confirm the finding is a false positive, external-code issue, or intentionally defensive logic.
3. Add the narrowest possible suppression.
4. Document the change here with scope and rationale.
