# CAN heartbeat guard — ESP32 → STM32 (0x011)

## Problem

The ESP32 safety heartbeat is normally emitted from the large Core-1 Arduino
`loop()`.  The STM32 watchdog requires a fresh, advancing 0x011 counter within
250 ms.  A synchronous NVS write, a long application iteration, or a temporarily
full TWAI TX queue can therefore create a false CAN timeout even while the bus
and both controllers remain alive.

The existing producer remains the normal source of the 100 ms heartbeat.  This
change adds a silent failover guard rather than changing the frozen CAN contract
or increasing the STM32 timeout.

## Behaviour

`CanHeartbeatGuard` is a high-priority FreeRTOS task pinned to Core 1. It is
started after TWAI initialization through `can_obstacle::init()`.

The main loop calls `can_heartbeat::notifyLoopAlive()` before any obstacle-sensor
early return. During normal operation the guard sends nothing, so no duplicate
heartbeat traffic is added.

The guard injects a backup 0x011 only when:

- the main loop has not reached its liveness kick for 120 ms; or
- TWAI TX congestion/drop evidence was observed and the queue has recovered.

This leaves at least 130 ms of margin before the STM32 250 ms watchdog expires.
The backup transmission uses a bounded 5 ms enqueue wait and one immediate
10 ms retry. The rolling counter advances only after TWAI accepts the frame.
Further retries remain rate-limited; no infinite loop is possible.

## Diagnostics

The guard records:

- loop-stall events;
- TX-congestion events;
- backup attempts/successes/failures;
- retries;
- maximum observed Core-1 loop gap;
- last backup success/failure timestamps.

A compact serial line is printed every 10 seconds only after a fault-related
event has occurred:

```text
[CAN][HB-GUARD] stall=1 congestion=0 ok=2 fail=0 retry=0 maxLoopGap=327 ms
```

## Safety properties

- No pin, CAN ID, bitrate, DLC, or STM32 timeout is changed.
- The guard is silent while the normal loop is healthy.
- No dynamic allocation occurs in the periodic path.
- TX retries are bounded.
- `millis()` wrap-around is handled by unsigned subtraction.
- The implementation does not mask BUS-OFF; existing TWAI recovery remains the
  authority for controller recovery.

## Validation

Host policy test:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Iesp32/src \
  esp32/src/test_can_heartbeat_guard.cpp \
  -o /tmp/test_can_heartbeat_guard
/tmp/test_can_heartbeat_guard
```

The test covers healthy-loop silence, a simulated NVS/main-loop stall, bounded
rate, TX queue congestion, failed enqueue/retry, and `millis()` rollover.

## Physical validation

With the vehicle immobilized and wheels raised:

1. Clear the DTC history.
2. Leave the system powered for at least 10 minutes.
3. Confirm no new `CAN TIMEOUT` DTC appears.
4. Watch Serial for `HB-GUARD` events.
5. If events occur, record `maxLoopGap`, `ok`, `fail`, and TWAI counters.
6. Confirm the STM32 remains out of LIMP_HOME while the guard reports successful
   backup heartbeats.
