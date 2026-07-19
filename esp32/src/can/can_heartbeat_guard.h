#ifndef CAN_HEARTBEAT_GUARD_H
#define CAN_HEARTBEAT_GUARD_H

#include <cstdint>
#include "can_heartbeat_guard_policy.h"

namespace can_heartbeat {

/*
 * Start the single ESP32 -> STM32 heartbeat (0x011) producer task.
 *
 * The task is pinned to Core 1 and transmits 0x011 unconditionally every
 * 100 ms using vTaskDelayUntil() — it does NOT depend on Arduino loop(),
 * obstacle data, TFT, audio, LEDs or NVS work.  It owns the single rolling
 * counter and performs one bounded 10 ms retry when the driver rejects a
 * frame.  Safe to call repeatedly; only the first call creates the task.
 */
bool init();

/* Coherent diagnostic snapshot for serial/engineering telemetry. */
HeartbeatDiag diag();

}  // namespace can_heartbeat

#endif  // CAN_HEARTBEAT_GUARD_H
