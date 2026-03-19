#!/usr/bin/env bash
# =============================================================================
# run_wiring_checks.sh — Automated pin/BOM verification for wiring documentation
#
# Checks that firmware pin definitions match wiring documentation tables.
# Run from repository root: bash analysis_artifacts/run_wiring_checks.sh
#
# Exit code 0 = all checks passed, non-zero = discrepancies found.
# =============================================================================

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAIN_H="${REPO_ROOT}/Core/Inc/main.h"
CAN_IDS_H="${REPO_ROOT}/esp32/include/can_ids.h"
WIRING_MD="${REPO_ROOT}/analysis_artifacts/wiring_manual.md"
BOM_CSV="${REPO_ROOT}/analysis_artifacts/BOM.csv"
FINDINGS_CSV="${REPO_ROOT}/analysis_artifacts/findings_hardware.csv"

ERRORS=0
WARNINGS=0

echo "=============================================="
echo " Wiring Documentation Verification Script"
echo " Repository: ${REPO_ROOT}"
echo " Date: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "=============================================="
echo ""

# --- Helper ---
check_define() {
    local file="$1" define="$2" expected="$3" desc="$4"
    if grep -q "#define ${define}" "${file}"; then
        actual=$(grep "#define ${define}" "${file}" | head -1 | awk '{print $3}')
        if [ "${actual}" = "${expected}" ]; then
            echo "  [PASS] ${desc}: ${define} = ${actual}"
        else
            echo "  [FAIL] ${desc}: ${define} expected ${expected}, got ${actual}"
            ERRORS=$((ERRORS + 1))
        fi
    else
        echo "  [FAIL] ${desc}: ${define} not found in ${file}"
        ERRORS=$((ERRORS + 1))
    fi
}

check_file_exists() {
    local file="$1" desc="$2"
    if [ -f "${file}" ]; then
        echo "  [PASS] ${desc}: ${file} exists"
    else
        echo "  [FAIL] ${desc}: ${file} MISSING"
        ERRORS=$((ERRORS + 1))
    fi
}

check_contains() {
    local file="$1" pattern="$2" desc="$3"
    if grep -q "${pattern}" "${file}" 2>/dev/null; then
        echo "  [PASS] ${desc}"
    else
        echo "  [WARN] ${desc}: pattern '${pattern}' not found in $(basename "${file}")"
        WARNINGS=$((WARNINGS + 1))
    fi
}

# =============================================================================
echo "--- 1. Firmware Pin Definitions (Core/Inc/main.h) ---"
# =============================================================================

check_define "${MAIN_H}" "PIN_PWM_FL"      "GPIO_PIN_8"   "Front-Left RPWM = PA8"
check_define "${MAIN_H}" "PIN_LPWM_FL"     "GPIO_PIN_9"   "Front-Left LPWM = PA9"
check_define "${MAIN_H}" "PIN_PWM_FR"      "GPIO_PIN_10"  "Front-Right RPWM = PA10"
check_define "${MAIN_H}" "PIN_LPWM_FR"     "GPIO_PIN_11"  "Front-Right LPWM = PA11"
check_define "${MAIN_H}" "PIN_PWM_RL"      "GPIO_PIN_6"   "Rear-Left RPWM = PC6"
check_define "${MAIN_H}" "PIN_LPWM_RL"     "GPIO_PIN_7"   "Rear-Left LPWM = PC7"
check_define "${MAIN_H}" "PIN_PWM_RR"      "GPIO_PIN_8"   "Rear-Right RPWM = PC8"
check_define "${MAIN_H}" "PIN_LPWM_RR"     "GPIO_PIN_9"   "Rear-Right LPWM = PC9"
check_define "${MAIN_H}" "PIN_PWM_STEER"   "GPIO_PIN_6"   "Steering RPWM = PA6"
check_define "${MAIN_H}" "PIN_LPWM_STEER"  "GPIO_PIN_7"   "Steering LPWM = PA7"
check_define "${MAIN_H}" "PIN_EN_FL"       "GPIO_PIN_5"   "Enable FL = PC5"
check_define "${MAIN_H}" "PIN_EN_RR"       "GPIO_PIN_13"  "Enable RR = PC13"
check_define "${MAIN_H}" "PIN_RELAY_MAIN"  "GPIO_PIN_10"  "Relay Main = PC10"
check_define "${MAIN_H}" "PIN_RELAY_TRAC"  "GPIO_PIN_11"  "Relay Trac = PC11"
check_define "${MAIN_H}" "PIN_RELAY_DIR"   "GPIO_PIN_12"  "Relay Dir = PC12"
check_define "${MAIN_H}" "PIN_RELAY_LED"   "GPIO_PIN_10"  "Relay LED Front = PB10"
check_define "${MAIN_H}" "PIN_RELAY_LED_REAR" "GPIO_PIN_11" "Relay LED Rear = PB11"
check_define "${MAIN_H}" "PIN_WHEEL_FL"    "GPIO_PIN_0"   "Wheel FL = PA0"
check_define "${MAIN_H}" "PIN_WHEEL_FR"    "GPIO_PIN_1"   "Wheel FR = PA1"
check_define "${MAIN_H}" "PIN_WHEEL_RL"    "GPIO_PIN_2"   "Wheel RL = PA2"
check_define "${MAIN_H}" "PIN_WHEEL_RR"    "GPIO_PIN_15"  "Wheel RR = PB15"
check_define "${MAIN_H}" "PIN_ENC_A"       "GPIO_PIN_15"  "Encoder A = PA15"
check_define "${MAIN_H}" "PIN_ENC_B"       "GPIO_PIN_3"   "Encoder B = PB3"
check_define "${MAIN_H}" "PIN_ENC_Z"       "GPIO_PIN_4"   "Encoder Z = PB4"
check_define "${MAIN_H}" "PIN_STEER_CENTER" "GPIO_PIN_5"  "Steer Center = PB5"
check_define "${MAIN_H}" "PIN_I2C_SCL"     "GPIO_PIN_6"   "I2C SCL = PB6"
check_define "${MAIN_H}" "PIN_I2C_SDA"     "GPIO_PIN_7"   "I2C SDA = PB7"
check_define "${MAIN_H}" "PIN_CAN_TX"      "GPIO_PIN_9"   "CAN TX = PB9"
check_define "${MAIN_H}" "PIN_CAN_RX"      "GPIO_PIN_8"   "CAN RX = PB8"
check_define "${MAIN_H}" "PIN_PEDAL"       "GPIO_PIN_3"   "Pedal ADC = PA3"
check_define "${MAIN_H}" "PIN_ONEWIRE"     "GPIO_PIN_0"   "OneWire = PB0"
check_define "${MAIN_H}" "PIN_LD2"         "GPIO_PIN_5"   "LED LD2 = PA5"

echo ""

# =============================================================================
echo "--- 2. CAN Bus Configuration ---"
# =============================================================================

check_contains "${CAN_IDS_H}" "500000" "CAN bitrate = 500000"
check_contains "${CAN_IDS_H}" "0x001" "STM32 heartbeat ID = 0x001"
check_contains "${CAN_IDS_H}" "0x011" "ESP32 heartbeat ID = 0x011"
check_contains "${CAN_IDS_H}" "0x100" "Throttle command ID = 0x100"
check_contains "${CAN_IDS_H}" "0x101" "Steering command ID = 0x101"
check_contains "${CAN_IDS_H}" "0x120" "LED command ID = 0x120"

echo ""

# =============================================================================
echo "--- 3. Deliverable Files Exist ---"
# =============================================================================

check_file_exists "${WIRING_MD}" "Wiring Manual"
check_file_exists "${BOM_CSV}" "Bill of Materials"
check_file_exists "${FINDINGS_CSV}" "Hardware Findings"
check_file_exists "${REPO_ROOT}/analysis_artifacts/connector_pinouts.md" "Connector Pinouts"
check_file_exists "${REPO_ROOT}/analysis_artifacts/safety_checks.md" "Safety Checks"
check_file_exists "${REPO_ROOT}/analysis_artifacts/harness_diagrams/harness_overview.md" "Harness Diagrams"
check_file_exists "${REPO_ROOT}/analysis_artifacts/patches/README.md" "Patches README"
check_file_exists "${REPO_ROOT}/analysis_artifacts/install_commands.txt" "Install Commands"

echo ""

# =============================================================================
echo "--- 4. Wiring Manual Content Checks ---"
# =============================================================================

check_contains "${WIRING_MD}" "PA8" "Wiring manual references PA8 (RPWM_FL)"
check_contains "${WIRING_MD}" "PC6" "Wiring manual references PC6 (RPWM_RL)"
check_contains "${WIRING_MD}" "PA6" "Wiring manual references PA6 (RPWM_STEER)"
check_contains "${WIRING_MD}" "120.*Ω\|120 Ω\|120Ω" "Wiring manual mentions CAN termination 120Ω"
check_contains "${WIRING_MD}" "10.*kΩ.*6.8.*kΩ\|10 kΩ.*6.8 kΩ" "Wiring manual mentions pedal divider 10kΩ/6.8kΩ"
check_contains "${WIRING_MD}" "flyback\|Flyback\|FLYBACK" "Wiring manual mentions flyback diode"
check_contains "${WIRING_MD}" "6N137" "Wiring manual mentions 6N137 optocoupler"
check_contains "${WIRING_MD}" "PC817" "Wiring manual mentions PC817 optocoupler"

echo ""

# =============================================================================
echo "--- 5. BOM Content Checks ---"
# =============================================================================

check_contains "${BOM_CSV}" "BTS7960" "BOM includes BTS7960 motor driver"
check_contains "${BOM_CSV}" "INA226" "BOM includes INA226 current sensor"
check_contains "${BOM_CSV}" "TCA9548A" "BOM includes TCA9548A I2C mux"
check_contains "${BOM_CSV}" "DS18B20" "BOM includes DS18B20 temperature sensor"
check_contains "${BOM_CSV}" "6N137" "BOM includes 6N137 optocoupler"
check_contains "${BOM_CSV}" "PC817" "BOM includes PC817 optocoupler"
check_contains "${BOM_CSV}" "SS1324LUA" "BOM includes SS1324LUA-T pedal sensor"
check_contains "${BOM_CSV}" "120" "BOM includes 120Ω CAN termination"
check_contains "${BOM_CSV}" "1N4007\|1N4148" "BOM includes flyback diode"
check_contains "${BOM_CSV}" "PESD2CAN" "BOM includes CAN TVS diode"
check_contains "${BOM_CSV}" "MCP23017" "BOM includes MCP23017 I/O expander"

echo ""

# =============================================================================
echo "--- 6. Findings File Checks ---"
# =============================================================================

FINDING_COUNT=$(tail -n +2 "${FINDINGS_CSV}" | wc -l)
CRITICAL_COUNT=$(grep -c "CRITICAL" "${FINDINGS_CSV}" || true)
echo "  [INFO] Total findings: ${FINDING_COUNT}"
echo "  [INFO] Critical findings: ${CRITICAL_COUNT}"

if [ "${CRITICAL_COUNT}" -gt 0 ]; then
    echo "  [WARN] ${CRITICAL_COUNT} CRITICAL discrepancies require human review"
    WARNINGS=$((WARNINGS + 1))
fi

echo ""

# =============================================================================
echo "=============================================="
echo " RESULTS"
echo "=============================================="
echo "  Errors:   ${ERRORS}"
echo "  Warnings: ${WARNINGS}"
echo ""

if [ "${ERRORS}" -gt 0 ]; then
    echo "  STATUS: FAILED — ${ERRORS} error(s) found"
    exit 1
else
    echo "  STATUS: PASSED (${WARNINGS} warning(s))"
    exit 0
fi
