#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${TMPDIR:-/tmp}/test_can_heartbeat_guard"

printf '[CAN-HB] compiling host policy test...\n'
g++ -std=c++17 -Wall -Wextra -Werror \
  -I"$ROOT/esp32/src" \
  "$ROOT/esp32/src/test_can_heartbeat_guard.cpp" \
  -o "$OUT"

printf '[CAN-HB] running host policy test...\n'
"$OUT"

printf '[CAN-HB] checking production invariants...\n'
grep -q 'vTaskDelayUntil' "$ROOT/esp32/src/can/can_heartbeat_guard.cpp"
grep -q 'twai_transmit' "$ROOT/esp32/src/can/can_heartbeat_guard.cpp"
grep -q 'if (sent)' "$ROOT/esp32/src/can/can_heartbeat_guard.cpp"
grep -q 'HEARTBEAT_INTERVAL_MS < can::HEARTBEAT_TIMEOUT_MS' \
  "$ROOT/esp32/src/can/can_heartbeat_guard.cpp"

printf '[CAN-HB] PASS: deterministic scheduling, bounded retry and rollover policy validated.\n'
