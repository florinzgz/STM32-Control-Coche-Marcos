# Deterministic CAN heartbeat — ESP32 → STM32 (`0x011`)

## Problem corrected

The STM32 safety watchdog requires a fresh, advancing ESP32 heartbeat within
250 ms.  The original producer lived near the end of the large Core-1 Arduino
`loop()` and used a non-blocking enqueue.  A long iteration, synchronous NVS
work, LEDs/audio, or a temporarily full TWAI TX queue could therefore create a
false `CAN TIMEOUT` while the transceivers and both controllers remained alive
(observed physically as a `CAN TIMEOUT` 15–20 s after boot that recovered to
`ACTIVE`).

The safety timeout, bitrate, pins, CAN identifier and payload remain unchanged.
The STM32 250 ms watchdog is **not** relaxed to hide the fault.

## Current implementation — single producer

There is now exactly **one** `0x011` producer: the dedicated `CanHeartbeat`
FreeRTOS task.  The legacy `loop()` producer has been removed.

`CanHeartbeat` is pinned to Core 1 at priority 6.  It starts after TWAI
initialization through `can_obstacle::init()` and transmits `0x011`
**unconditionally every 100 ms** — never as a mere backup — independently of:

- Arduino `loop()`;
- TFT/render/touch;
- obstacle sensor state;
- audio and WS2812B updates;
- normal telemetry;
- configuration/NVS scheduling.

The task uses an absolute `vTaskDelayUntil()` cadence.  Each slot sends one
frame with `twai_transmit()` and a bounded 5 ms queue wait.  A rejected enqueue
is retried **once, 10 ms later**, re-sending the same counter value; there is no
unbounded retry loop.

A **single** rolling `uint8_t` counter is owned exclusively by this task.  It
advances **only** after `twai_transmit()` accepts the frame, so a retry never
skips a sequence value and `uint8_t` wraps naturally at 255 → 0.

## Controller-state behaviour

Diagnostics observe the controller before each transmit (queue depth, TWAI
state, `tx_failed_count`, error counters).  The task does not fight the existing
BUS-OFF/error-passive recovery: while the controller is not `RUNNING` the
`twai_transmit()` call simply fails and is accounted as a rejection; the counter
does not advance and the last-accepted timestamp is untouched.  When TWAI
returns to `RUNNING` the absolute scheduler resumes automatically.

## Diagnostics

The task logs a coherent snapshot (throttled to ~1 s) containing:

- last accepted send time;
- current and maximum accepted-to-accepted gap;
- `tx_failed_count` (hardware) and software rejections;
- bounded-retry episodes;
- TWAI TX queue depth;
- TWAI controller state;
- BUS_OFF / error-passive flags;
- reset reason (`esp_reset_reason()`).

`can_heartbeat::diag()` exposes the same snapshot for engineering telemetry.

## Offline validation without GitHub Actions

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Iesp32/src \
  esp32/src/test_can_heartbeat_guard.cpp \
  -o /tmp/test_can_heartbeat_guard
/tmp/test_can_heartbeat_guard
```

The host test covers: unconditional 100 ms cadence with the application loop
blocked; independence from TFT/audio/LED/NVS work; congested queue with full and
retry-rescued recovery; no accepted-to-accepted gap above 250 ms; a single
producer and single counter that advances only on accept; correct `uint8_t`
wrap; the diagnostics snapshot fields; and `millis()` rollover.

## Physical validation

Vehicle immobilized, wheels raised and emergency stop accessible:

1. Flash the ESP32 firmware from this branch.
2. Clear the DTC log.
3. Leave the complete system powered for at least 30 minutes.
4. Confirm **zero** new `CAN TIMEOUT` and zero accidental `LIMP_HOME` entries.
5. Confirm STM32 remains out of `LIMP_HOME` while ESP32, LEDs, audio and TFT run.
6. Record TWAI `tx_err`, `tx_failed_count`, BUS-OFF count and serial boot reason.
7. Repeat while changing screens, running LEDs/audio and saving one setting.

A green host test proves the scheduling and counter policy.  Final resolution
still requires an ESP32 PlatformIO build and the physical test above; it must not
be claimed from GitHub checks that never started.
