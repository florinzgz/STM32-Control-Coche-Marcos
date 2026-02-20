// =============================================================================
// ESP32-S3 — CAN Obstacle TX Module
//
// Transmits CAN frame 0x208 (OBSTACLE_DISTANCE) at 66 ms interval,
// exactly matching the frozen CAN contract (rev 1.3 §3.4).
//
// Payload (DLC 5):
//   Bytes 0-1: distance_mm (uint16 LE)
//   Byte  2:   zone (0–3)
//   Byte  3:   sensor_health (0=unhealthy, 1=healthy)
//   Byte  4:   rolling_counter (0–255, increments each frame)
//
// Failsafe: if sensor unavailable, stops sending frames entirely.
// STM32 naturally enters its 500 ms CAN timeout path.
//
// Reference: docs/CAN_CONTRACT_FINAL.md rev 1.3
//            Core/Src/safety_system.c — Obstacle_ProcessCAN()
// =============================================================================

#ifndef CAN_OBSTACLE_H
#define CAN_OBSTACLE_H

#include <cstdint>

namespace can_obstacle {

/// Initialize CAN TX module.  Call once from setup() after CAN bus init.
void init();

/// Update and transmit 0x208 frame if interval elapsed.
/// Call from loop() every iteration.
void update();

} // namespace can_obstacle

#endif // CAN_OBSTACLE_H
