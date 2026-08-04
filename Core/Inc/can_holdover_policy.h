#ifndef CAN_HOLDOVER_POLICY_H
#define CAN_HOLDOVER_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_HOLDOVER_RECOVERY_STABLE_MS 500U
#define CAN_HOLDOVER_MAX_MS             750U

static inline bool CanHoldover_Expired(uint32_t started_ms, uint32_t now_ms)
{
    return (uint32_t)(now_ms - started_ms) >= CAN_HOLDOVER_MAX_MS;
}

#endif
