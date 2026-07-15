# Deterministic CAN heartbeat — ESP32 → STM32 (`0x011`)

## Problem corrected

The STM32 safety watchdog requires a fresh, advancing ESP32 heartbeat within
250 ms.  The original producer lives near the end of the large Core-1 Arduino
`loop()` and uses a non-blocking enqueue.  A long iteration, synchronous NVS
work, LEDs/audio, or a temporarily full TWAI TX queue can therefore create a
false `CAN TIMEOUT` while the transceivers and both controllers remain alive.

The safety timeout, bitrate, pins, CAN identifier and payload remain unchanged.

## Current implementation

`CanHeartbeat` is a dedicated FreeRTOS task pinned to Core 1 at priority 5.  It
starts after TWAI initialization through `can_obstacle::init()` and sends
`0x011` every 100 ms independently of:

- Arduino `loop()`;
- TFT/render/touch;
- obstacle sensor state;
- audio and WS2812B updates;
- normal telemetry;
- configuration/NVS scheduling.

The task uses an absolute `vTaskDelayUntil()` cadence.  Each due frame is sent
with `twai_transmit()` and a bounded 5 ms queue wait.  A failed enqueue is
retried after 10 ms by the same periodic task.  There is no infinite retry loop.
The rolling counter advances **only** after TWAI accepts the frame.

The existing loop producer is retained as a temporary compatibility fallback
on this branch.  The deterministic task starts its counter at `0x80`, while the
legacy producer starts at `0x00`; both counters advance after successful normal
operation, so adjacent frames remain different and the STM32 frozen-counter
protection is not defeated.  The extra traffic is negligible at 500 kbit/s.
After a full PlatformIO build and physical validation, the legacy block can be
removed in a small cleanup commit, leaving the dedicated task as the sole
producer.

## Controller-state behaviour

The heartbeat task never fights the existing BUS-OFF/error-passive recovery:

- `RUNNING`: normal periodic transmit;
- `STOPPED`, `BUS_OFF`, `RECOVERING`: no transmit attempt;
- when TWAI returns to `RUNNING`, the absolute scheduler resumes automatically;
- failed transmission time never updates the last-success timestamp.

## Diagnostics

The host-tested policy records:

- attempts, successes and failures;
- bounded retries;
- queue-full observations;
- drops reported by other CAN producers;
- skipped ticks while TWAI is not running;
- last successful heartbeat time;
- current and maximum successful-heartbeat gap;
- warning events above 150 ms;
- maximum observed Arduino-loop gap.

Loop liveness is diagnostic only.  It does not gate heartbeat transmission.

## Offline validation without GitHub Actions

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Iesp32/src \
  esp32/src/test_can_heartbeat_guard.cpp \
  -o /tmp/test_can_heartbeat_guard
/tmp/test_can_heartbeat_guard
```

Expected result:

```text
can_heartbeat_guard: 37 checks, 0 failures
```

The test covers absolute 100 ms cadence, loop blocking independence, failed
queue insertion, 10 ms retry, no false success timestamp, controller outage,
warning-gap accounting, external TX-drop diagnostics and `millis()` rollover.

## Physical validation

Vehicle immobilized, wheels raised and emergency stop accessible:

1. Flash the ESP32 firmware from this branch.
2. Clear the DTC log.
3. Leave the complete system powered for at least 15 minutes.
4. Confirm no new `CAN TIMEOUT` appears around the previous ~60 s cadence.
5. Confirm STM32 remains out of `LIMP_HOME` while ESP32, LEDs, audio and TFT run.
6. Record TWAI `tx_err`, `tx_failed_count`, BUS-OFF count and serial boot reason.
7. Repeat while changing screens, running LEDs/audio and saving one setting.

A green host test proves the scheduling policy.  Final resolution still requires
an ESP32 PlatformIO build and the physical test above; it must not be claimed
from GitHub checks that never started.
