#ifdef HOST_TEST
#include "can_rx_policy.h"
#include <stdio.h>
static int n, f;
#define T(x) do { ++n; if (!(x)) { ++f; printf("FAIL %d: %s\n", __LINE__, #x); } } while (0)
int main(void) {
    T(CanRxPolicy_Accept(CAN_ID_HEARTBEAT_ESP32, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, 1));
    T(!CanRxPolicy_Accept(CAN_ID_HEARTBEAT_ESP32, FDCAN_EXTENDED_ID, FDCAN_DATA_FRAME, 1));
    T(!CanRxPolicy_Accept(CAN_ID_CMD_THROTTLE, FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, 1));
    T(!CanRxPolicy_Accept(0x7AA, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, 1));
    T(CanRxPolicy_DlcValid(CAN_ID_CMD_MODE, 2));
    T(!CanRxPolicy_DlcValid(CAN_ID_CMD_MODE, 1));
    T(CanRxPolicy_DlcValid(CAN_ID_SERVICE_CMD, 2));
    T(CanRxPolicy_DlcValid(CAN_ID_SERVICE_CMD, 8));
    T(!CanRxPolicy_DlcValid(CAN_ID_SERVICE_CMD, 1));
    T(CanRxPolicy_DlcValid(CAN_ID_CMD_SYSTEM_SHUTDOWN, 0));
    T(CanRxPolicy_DlcValid(CAN_ID_CMD_SYSTEM_SHUTDOWN, 1));
    T(!CanRxPolicy_DlcValid(CAN_ID_CMD_SYSTEM_SHUTDOWN, 2));
    printf("test_can_rx_policy: %d run, %d failed\n", n, f);
    return f != 0;
}
#endif
