#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="${TMPDIR:-/tmp}/can-heartbeat-validation"
POLICY_EXE="$TMP/test_can_heartbeat_guard"

rm -rf "$TMP"
mkdir -p "$TMP/stubs/driver" "$TMP/stubs/freertos" "$TMP/stubs/can"

printf '[CAN-HB] compiling host policy test...\n'
g++ -std=c++17 -Wall -Wextra -Werror \
  -I"$ROOT/esp32/src" \
  "$ROOT/esp32/src/test_can_heartbeat_guard.cpp" \
  -o "$POLICY_EXE"

printf '[CAN-HB] running host policy test...\n'
"$POLICY_EXE"

printf '[CAN-HB] preparing strict production-source stubs...\n'
cat > "$TMP/stubs/Arduino.h" <<'EOF'
#pragma once
#include <cstdint>
struct SerialStub { void println(const char*) {} };
extern SerialStub Serial;
inline uint32_t millis() { return 0U; }
EOF

cat > "$TMP/stubs/driver/twai.h" <<'EOF'
#pragma once
#include <cstdint>
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_ERR_TIMEOUT = 1;
enum twai_state_t {
    TWAI_STATE_STOPPED,
    TWAI_STATE_RUNNING,
    TWAI_STATE_BUS_OFF,
    TWAI_STATE_RECOVERING
};
struct twai_status_info_t {
    twai_state_t state = TWAI_STATE_RUNNING;
    uint32_t tx_failed_count = 0U;
};
struct twai_message_t {
    uint32_t identifier = 0U;
    uint32_t extd = 0U;
    uint32_t rtr = 0U;
    uint8_t data_length_code = 0U;
    uint8_t data[8] = {};
};
inline esp_err_t twai_get_status_info(twai_status_info_t*) { return ESP_OK; }
inline esp_err_t twai_transmit(const twai_message_t*, uint32_t) { return ESP_OK; }
EOF

cat > "$TMP/stubs/freertos/FreeRTOS.h" <<'EOF'
#pragma once
#include <cstdint>
using TickType_t = uint32_t;
using UBaseType_t = unsigned;
using BaseType_t = int;
using TaskHandle_t = void*;
struct portMUX_TYPE {};
constexpr BaseType_t pdPASS = 1;
#define portMUX_INITIALIZER_UNLOCKED portMUX_TYPE{}
inline TickType_t pdMS_TO_TICKS(uint32_t ms) { return ms; }
inline void taskENTER_CRITICAL(portMUX_TYPE*) {}
inline void taskEXIT_CRITICAL(portMUX_TYPE*) {}
EOF

cat > "$TMP/stubs/freertos/task.h" <<'EOF'
#pragma once
#include "FreeRTOS.h"
inline TickType_t xTaskGetTickCount() { return 0U; }
inline void vTaskDelayUntil(TickType_t*, TickType_t) {}
inline BaseType_t xTaskCreatePinnedToCore(
    void (*)(void*), const char*, uint32_t, void*, UBaseType_t,
    TaskHandle_t*, BaseType_t) { return pdPASS; }
EOF

cat > "$TMP/stubs/can_ids.h" <<'EOF'
#pragma once
#include <cstdint>
namespace can {
inline constexpr uint32_t HEARTBEAT_ESP32 = 0x011U;
inline constexpr uint32_t HEARTBEAT_INTERVAL_MS = 100U;
inline constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 250U;
}
EOF

cat > "$TMP/stubs/serial_stub.cpp" <<'EOF'
#include "Arduino.h"
SerialStub Serial;
EOF

printf '[CAN-HB] compiling production heartbeat source with -Werror...\n'
g++ -std=c++17 -Wall -Wextra -Werror \
  -I"$TMP/stubs" \
  -I"$ROOT/esp32/src" \
  -I"$ROOT/esp32/src/can" \
  -c "$ROOT/esp32/src/can/can_heartbeat_guard.cpp" \
  -o "$TMP/can_heartbeat_guard.o"

g++ -std=c++17 -Wall -Wextra -Werror \
  -I"$TMP/stubs" \
  -c "$TMP/stubs/serial_stub.cpp" \
  -o "$TMP/serial_stub.o"

printf '[CAN-HB] checking production invariants...\n'
grep -q 'vTaskDelayUntil' "$ROOT/esp32/src/can/can_heartbeat_guard.cpp"
grep -q 'twai_transmit' "$ROOT/esp32/src/can/can_heartbeat_guard.cpp"
grep -q 'if (sent)' "$ROOT/esp32/src/can/can_heartbeat_guard.cpp"
grep -q 'HEARTBEAT_INTERVAL_MS < can::HEARTBEAT_TIMEOUT_MS' \
  "$ROOT/esp32/src/can/can_heartbeat_guard.cpp"
grep -q 'can_heartbeat::init' "$ROOT/esp32/src/can/can_obstacle.cpp"

printf '[CAN-HB] PASS: policy tests, strict production compile and integration invariants validated.\n'
