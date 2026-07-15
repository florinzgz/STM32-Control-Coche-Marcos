#ifndef CAN_HEARTBEAT_GUARD_H
#define CAN_HEARTBEAT_GUARD_H

#include <cstdint>
#include "can_heartbeat_guard_policy.h"

namespace can_heartbeat {

/*
 * Start the dedicated ESP32 -> STM32 heartbeat producer.
 *
 * The task is pinned to Core 1, uses vTaskDelayUntil(), and is independent from
 * Arduino loop(), TFT rendering, audio, LEDs and normal telemetry.  Safe to call
 * repeatedly; only the first successful call creates the task.
 */
bool init();

/* Optional loop-liveness instrumentation.  It no longer controls whether a
 * heartbeat is sent; a blocked loop therefore cannot suppress 0x011. */
void notifyLoopAlive(uint32_t nowMs);

/* Report a drop observed by another CAN producer.  Diagnostic only: the
 * heartbeat producer already checks its own transmit result and retries. */
void notifyTxDrop();

/* Coherent diagnostic snapshot for serial/engineering telemetry. */
GuardStats stats();

/* Age of the last heartbeat accepted by the TWAI driver. */
uint32_t currentSuccessGapMs(uint32_t nowMs);

}  // namespace can_heartbeat

#endif  // CAN_HEARTBEAT_GUARD_H
