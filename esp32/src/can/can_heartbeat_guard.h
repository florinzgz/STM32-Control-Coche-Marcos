#ifndef CAN_HEARTBEAT_GUARD_H
#define CAN_HEARTBEAT_GUARD_H

#include <cstdint>
#include "can_heartbeat_guard_policy.h"

namespace can_heartbeat {

/*
 * Start the ESP32 -> STM32 heartbeat failover task.
 *
 * The existing main-loop heartbeat remains the normal producer.  This task is
 * pinned to Core 1 and stays silent while loop() is healthy; it only injects a
 * backup 0x011 after a measured loop stall or TX congestion/drop event.
 * Safe to call repeatedly; only the first successful call creates the task.
 */
bool init();

/* Main-loop liveness kick.  Call once per normal loop iteration before any
 * potentially slow NVS/LED/audio work. */
void notifyLoopAlive(uint32_t nowMs);

/* Report a TX drop observed by another CAN producer. */
void notifyTxDrop();

/* Coherent diagnostic snapshot for serial/engineering telemetry. */
GuardStats stats();

}  // namespace can_heartbeat

#endif  // CAN_HEARTBEAT_GUARD_H
