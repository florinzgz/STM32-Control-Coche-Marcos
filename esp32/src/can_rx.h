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

/// Per-ID 0x309 RX counters (audit questions E/F).  Permanent counters that
/// let the HMI tell "no 0x309 frame ever arrived" apart from "0x309 frames
/// arrive but are rejected for an invalid DLC".
uint32_t rx0x309Count();      ///< total 0x309 frames seen on the bus (any DLC)
uint32_t dropped0x309Dlc();   ///< 0x309 frames rejected for DLC < 5
uint8_t  last0x309Dlc();      ///< DLC of the most recent 0x309 frame

/// Per-ID 0x30B RX counters (I2C service scan report).
uint32_t rx0x30BCount();      ///< total 0x30B frames seen on the bus (any DLC)
uint32_t dropped0x30BDlc();   ///< 0x30B frames rejected for DLC < 5
uint8_t  last0x30BDlc();      ///< DLC of the most recent 0x30B frame

/// Per-ID 0x30C RX counter (FDCAN diagnostic dump).
uint32_t rx0x30CCount();      ///< total 0x30C frames seen on the bus (any DLC)

} // namespace can_rx

#endif // CAN_RX_H
