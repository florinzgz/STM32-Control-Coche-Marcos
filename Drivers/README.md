# Drivers — HAL Version Notice

## HAL Version

The HAL drivers in this directory are pinned to **STM32G4xx HAL v1.2.2**.

## .ioc Firmware Package Reference

The CubeMX project file (`.ioc`) references **STM32Cube FW_G4 V1.6.2**.

This version mismatch is intentional and must be preserved.

## Rules

- **DO NOT** regenerate Drivers/ automatically via CubeMX.
- **DO NOT** accept CubeMX migration prompts that would upgrade the HAL.
- Any HAL update must be performed **manually** after full regression testing.
- The `.ioc` settings `AskForMigrate=false` and `DeletePrevious=false` are set
  to protect against accidental overwrites.

## Note on SysTick Priority

SysTick interrupt priority is set to 4, which is lower than control interrupts
(EXTI and TIM at priority 2, FDCAN at priority 1). This is intentional to avoid
preempting time-critical motor control paths. Tuning may be considered if HAL
tick jitter is observed in control loops.
