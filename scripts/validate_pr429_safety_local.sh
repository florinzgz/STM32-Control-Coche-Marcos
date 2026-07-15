#!/usr/bin/env bash
# Local verification for PR #429 safety corrections.
# Usage:
#   ./scripts/validate_pr429_safety_local.sh            # full host + firmware builds
#   ./scripts/validate_pr429_safety_local.sh --host-only

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-full}"
TMP="${TMPDIR:-/tmp}/pr429-safety-validation"

if [[ "$MODE" != "full" && "$MODE" != "--host-only" ]]; then
  echo "Usage: $0 [--host-only]" >&2
  exit 2
fi

rm -rf "$TMP"
mkdir -p "$TMP"
cd "$ROOT"

printf '\n[PR429] Checking production source selection...\n'
grep -q 'sensor_manager_patched.c' Makefile
grep -q 'safety_system_patched.c' Makefile
grep -q 'steering_centering_patched.c' Makefile
grep -q 'steering_centering_diag_patched.c' Makefile
! grep -Eq '^  \$\(CORE_SRC\)/(sensor_manager|safety_system|steering_centering|steering_centering_diag)\.c' Makefile

printf '\n[PR429] INA226 CH5 classifier tests...\n'
gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  Core/Src/test_ina226_channel_diag.c Core/Src/ina226_channel_diag.c \
  -lm -o "$TMP/test_ina226_channel_diag"
"$TMP/test_ina226_channel_diag"

gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  Core/Src/test_ina226_ch5_frame.c Core/Src/ina226_channel_diag.c \
  -lm -o "$TMP/test_ina226_ch5_frame"
"$TMP/test_ina226_ch5_frame"

g++ -std=c++17 -Wall -Wextra -Werror \
  -Iesp32/src -Iesp32/include -ICore/Inc \
  esp32/src/test_ina226_ch5_view.cpp Core/Src/ina226_channel_diag.c \
  -o "$TMP/test_ina226_ch5_view"
"$TMP/test_ina226_ch5_view"

printf '\n[PR429] Relay/current evidence tests...\n'
gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  Core/Src/test_relay_health_diag.c Core/Src/relay_health_diag.c \
  -lm -o "$TMP/test_relay_health_diag"
"$TMP/test_relay_health_diag"

gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  Core/Src/test_relay_degraded.c Core/Src/motion_inhibit.c \
  -lm -o "$TMP/test_relay_degraded"
"$TMP/test_relay_degraded"

g++ -std=c++17 -Wall -Wextra -Werror \
  -Iesp32/src -Iesp32/include -ICore/Inc \
  esp32/src/test_relay_health_view.cpp Core/Src/relay_health_diag.c \
  -o "$TMP/test_relay_health_view"
"$TMP/test_relay_health_view"

printf '\n[PR429] Steering diagnostic view and heartbeat tests...\n'
g++ -std=c++17 -Wall -Wextra -Werror \
  -Iesp32/src -Iesp32/include \
  esp32/src/test_steering_diag_view.cpp \
  -o "$TMP/test_steering_diag_view"
"$TMP/test_steering_diag_view"

chmod +x scripts/validate_can_heartbeat_local.sh
./scripts/validate_can_heartbeat_local.sh

if [[ "$MODE" == "--host-only" ]]; then
  printf '\n[PR429] PASS: host-only safety validation completed.\n'
  exit 0
fi

printf '\n[PR429] Building STM32 firmware...\n'
command -v arm-none-eabi-gcc >/dev/null || {
  echo "ERROR: arm-none-eabi-gcc is required for full validation." >&2
  exit 1
}
make clean
make -j2

printf '\n[PR429] Building ESP32 firmware...\n'
if command -v pio >/dev/null 2>&1; then
  (cd esp32 && pio run -e esp32s3)
elif command -v platformio >/dev/null 2>&1; then
  (cd esp32 && platformio run -e esp32s3)
else
  echo "ERROR: PlatformIO (pio) is required for full validation." >&2
  exit 1
fi

printf '\n[PR429] Running repository integrity checks...\n'
./scripts/check_integrity.sh

printf '\n[PR429] PASS: host tests, STM32 build, ESP32 build and integrity completed.\n'
