#ifndef CAN_RX_POLICY_H
#define CAN_RX_POLICY_H

#include "can_handler.h"
#include "rc_arbiter.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Pure protocol firewall used before any received frame can refresh liveness
 * or reach an actuator/service handler.  Production accepts only CAN 2.0A
 * standard data frames with a known ID and contract-valid DLC. */
static inline bool CanRxPolicy_DlcValid(uint32_t id, uint8_t dlc)
{
    switch (id) {
    case CAN_ID_HEARTBEAT_ESP32:      return dlc == 1U;
    case CAN_ID_CMD_THROTTLE:         return dlc == 1U;
    case CAN_ID_CMD_STEERING:         return dlc == 2U;
    case CAN_ID_CMD_MODE:             return dlc == 2U;
    case CAN_ID_CMD_RC_OVERRIDE:      return dlc == RC_OVERRIDE_DLC;
    case CAN_ID_SERVICE_CMD:          return dlc >= 2U && dlc <= 8U;
    case CAN_ID_OBSTACLE_DISTANCE:    return dlc == 5U;
    case CAN_ID_OBSTACLE_SAFETY:      return dlc == 4U;
    case CAN_ID_CMD_LED:              return dlc == 2U;
    case CAN_ID_CMD_SYSTEM_SHUTDOWN:  return dlc <= 1U;
    case CAN_ID_CMD_SENSOR_MAP_TEMP:  return dlc == 5U;
    default:                          return false;
    }
}

static inline bool CanRxPolicy_Accept(uint32_t id, uint32_t id_type,
                                      uint32_t frame_type, uint8_t dlc)
{
    return id_type == FDCAN_STANDARD_ID &&
           frame_type == FDCAN_DATA_FRAME &&
           CanRxPolicy_DlcValid(id, dlc);
}

#endif
