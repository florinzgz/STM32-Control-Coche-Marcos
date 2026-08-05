#ifdef HOST_TEST
#include "can_holdover_policy.h"
#include "service_hold_policy.h"
#include <stdio.h>
static int n, f;
#define T(x) do { ++n; if (!(x)) { ++f; printf("FAIL %d: %s\n", __LINE__, #x); } } while (0)
int main(void) {
    T(!CanHoldover_Expired(1000U, 1749U));
    T(CanHoldover_Expired(1000U, 1750U));
    T(CanHoldover_Expired(0xFFFFFF00U, 0x000001EEU));
    ServiceHoldPolicy p = {0};
    T(!ServiceHoldPolicy_Update(&p, true, 100U));
    T(!ServiceHoldPolicy_Update(&p, true, 599U));
    T(ServiceHoldPolicy_Update(&p, true, 600U));
    T(!ServiceHoldPolicy_Update(&p, true, 700U));
    T(!ServiceHoldPolicy_Update(&p, false, 800U));
    T(!ServiceHoldPolicy_Update(&p, true, 900U));
    T(!ServiceHoldPolicy_Update(&p, false, 1399U));
    printf("test_holdover_service_policy: %d run, %d failed\n", n, f);
    return f != 0;
}
#endif
