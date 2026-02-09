// =============================================================================
// ESP32-S3 HMI — CAN RX Module
//
// Receives CAN frames from STM32 and decodes them into vehicle_data.
// Only messages defined in CAN_CONTRACT_FINAL.md are handled.
// Unknown CAN IDs are silently ignored.
//
// Reference: docs/CAN_CONTRACT_FINAL.md rev 1.0
// =============================================================================

#ifndef CAN_RX_H
#define CAN_RX_H

#include "vehicle_data.h"

namespace can_rx {

/// Poll the CAN RX queue and decode any available frames.
/// Call this from loop(). Non-blocking — returns immediately if no frames.
void poll(vehicle::VehicleData& data);

} // namespace can_rx

#endif // CAN_RX_H
