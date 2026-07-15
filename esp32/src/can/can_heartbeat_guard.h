#ifndef CAN_HEARTBEAT_GUARD_H
#define CAN_HEARTBEAT_GUARD_H

#include <cstdint>
#include "can_heartbeat_guard_policy.h"

namespace can_heartbeat {

// Starts the high-priority CAN heartbeat failover task. Safe to call more than
// once; only the first call creates the task. Call after TWAI has started.
bool init();

// Main-loop liveness kick. The guard stays silent while the normal heartbeat
// producer is healthy, avoiding duplicate frames during normal operation.
void notifyLoopAlive(uint32_t nowMs);

// Report a non-heartbeat CAN TX drop so the guard can inject a priority
// heartbeat as soon as the TWAI TX queue has room again.
void notifyTxDrop();

// Read a coherent diagnostic snapshot for engineering telemetry / serial logs.
GuardStats stats();

}  // namespace can_heartbeat

#endif  // CAN_HEARTBEAT_GUARD_H
