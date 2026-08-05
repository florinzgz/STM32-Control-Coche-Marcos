#ifndef SERVICE_HOLD_POLICY_H
#define SERVICE_HOLD_POLICY_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define SERVICE_HOLD_RELEASE_STABLE_MS 500U

typedef struct {
    bool armed;
    uint32_t safe_since_ms;
} ServiceHoldPolicy;

static inline void ServiceHoldPolicy_Reset(ServiceHoldPolicy *p)
{
    if (p == NULL) return;
    p->armed = false;
    p->safe_since_ms = 0U;
}

static inline bool ServiceHoldPolicy_Update(ServiceHoldPolicy *p,
                                             bool safe_now,
                                             uint32_t now_ms)
{
    if (p == NULL) return false;
    if (!safe_now) {
        ServiceHoldPolicy_Reset(p);
        return false;
    }
    if (!p->armed) {
        p->armed = true;
        p->safe_since_ms = now_ms;
        return false;
    }
    if ((uint32_t)(now_ms - p->safe_since_ms) <
        SERVICE_HOLD_RELEASE_STABLE_MS) {
        return false;
    }
    ServiceHoldPolicy_Reset(p);
    return true;
}

#endif
