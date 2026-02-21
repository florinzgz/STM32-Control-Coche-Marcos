// =============================================================================
// ESP32-S3 — CAN Command TX Module (implementation)
//
// Implements the HMI Control Contract for continuous, analog-equivalent
// driving control from the ESP32 touchscreen.
//
// Key behaviors:
//   - 20 ms periodic TX for throttle and steering (even with no touch)
//   - 150 ms monotonic throttle ramp-down on release
//   - ACTIVE gate: 2 consecutive ACTIVE heartbeats before enabling control
//   - Mode/gear sent on-demand only
//
// Reference: docs/CAN_CONTRACT_FINAL.md rev 1.3
// =============================================================================

#include "can_tx.h"
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include "can_ids.h"

namespace can_tx {

// -------------------------------------------------------------------------
// Control Contract §2 — Throttle ramp-down parameters
// -------------------------------------------------------------------------
static constexpr uint32_t RAMP_DOWN_MS    = 150;   // Total ramp-down duration
static constexpr uint32_t RAMP_STEP_MS    = 20;    // Step interval = TX period

// -------------------------------------------------------------------------
// Control Contract §3 — ACTIVE gate
// -------------------------------------------------------------------------
static constexpr uint8_t  ACTIVE_GATE_CYCLES = 2;  // Consecutive ACTIVE heartbeats

// -------------------------------------------------------------------------
// Module state
// -------------------------------------------------------------------------
static bool     initialized_      = false;

// ACTIVE gate state
static uint8_t  activeCount_      = 0;      // Consecutive ACTIVE heartbeats seen
static bool     controlEnabled_   = false;   // True once gate passes

// Throttle state
static uint8_t  targetThrottle_   = 0;       // Desired throttle from touch
static uint8_t  outputThrottle_   = 0;       // Actual value being transmitted (after ramp)

// Steering state
static int16_t  steeringRaw_      = 0;       // Desired steering angle (0.1° units)

// Mode/gear state (on-demand)
static uint8_t  modeFlags_        = 0;
static uint8_t  gear_             = 2;       // Default: N (neutral)
static bool     modeDirty_        = false;   // True when mode/gear needs sending

// Timing
static unsigned long lastThrottleTxMs_ = 0;
static unsigned long lastSteeringTxMs_ = 0;

// -------------------------------------------------------------------------
// Internal helpers
// -------------------------------------------------------------------------

/// Send CMD_THROTTLE (0x100) — DLC 1, byte0 = pct
static void sendThrottle(uint8_t pct) {
    CanFrame frame = {};
    frame.identifier       = can::CMD_THROTTLE;
    frame.extd             = 0;
    frame.data_length_code = 1;
    frame.data[0]          = pct;
    ESP32Can.writeFrame(frame);
}

/// Send CMD_STEERING (0x101) — DLC 2, bytes 0-1 = angle (int16 LE)
static void sendSteering(int16_t angleRaw) {
    CanFrame frame = {};
    frame.identifier       = can::CMD_STEERING;
    frame.extd             = 0;
    frame.data_length_code = 2;
    frame.data[0]          = static_cast<uint8_t>(angleRaw & 0xFF);
    frame.data[1]          = static_cast<uint8_t>((angleRaw >> 8) & 0xFF);
    ESP32Can.writeFrame(frame);
}

/// Send CMD_MODE (0x102) — DLC 2, byte0 = flags, byte1 = gear
static void sendMode(uint8_t flags, uint8_t gearVal) {
    CanFrame frame = {};
    frame.identifier       = can::CMD_MODE;
    frame.extd             = 0;
    frame.data_length_code = 2;
    frame.data[0]          = flags;
    frame.data[1]          = gearVal;
    ESP32Can.writeFrame(frame);
}

/// Apply throttle ramp-down (§2).
/// When targetThrottle_ drops, outputThrottle_ ramps down monotonically
/// over RAMP_DOWN_MS instead of cutting instantly.
static void applyThrottleRamp() {
    if (outputThrottle_ <= targetThrottle_) {
        // Target is equal or higher — follow immediately (no ramp-up delay)
        outputThrottle_ = targetThrottle_;
        return;
    }

    // Ramp down: compute step size per 20ms tick to reach 0 in RAMP_DOWN_MS
    // steps = RAMP_DOWN_MS / RAMP_STEP_MS = 150 / 20 ≈ 7 steps
    // step_size = outputThrottle_at_release / steps
    // But we use a fixed linear ramp from current to target:
    uint8_t diff = outputThrottle_ - targetThrottle_;
    uint8_t steps = RAMP_DOWN_MS / RAMP_STEP_MS;  // 7
    if (steps == 0) steps = 1;
    uint8_t step = (diff + steps - 1) / steps;    // Ceiling division
    if (step == 0) step = 1;

    if (outputThrottle_ > targetThrottle_ + step) {
        outputThrottle_ -= step;
    } else {
        outputThrottle_ = targetThrottle_;
    }
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init() {
    targetThrottle_   = 0;
    outputThrottle_   = 0;
    steeringRaw_      = 0;
    modeFlags_        = 0;
    gear_             = 2;  // N
    modeDirty_        = false;
    activeCount_      = 0;
    controlEnabled_   = false;
    lastThrottleTxMs_ = 0;
    lastSteeringTxMs_ = 0;
    initialized_      = true;
    Serial.println("[CAN_TX] Command TX initialized (Control Contract active)");
}

void update(can::SystemState systemState) {
    if (!initialized_) return;

    unsigned long now = millis();

    // ---- §3 ACTIVE gate ----
    // Track consecutive ACTIVE heartbeats.  Any non-ACTIVE state resets.
    if (systemState == can::SystemState::ACTIVE ||
        systemState == can::SystemState::DEGRADED) {
        if (activeCount_ < ACTIVE_GATE_CYCLES) {
            activeCount_++;
        }
        if (activeCount_ >= ACTIVE_GATE_CYCLES) {
            if (!controlEnabled_) {
                Serial.println("[CAN_TX] ACTIVE gate passed — control enabled");
            }
            controlEnabled_ = true;
        }
    } else {
        // Not ACTIVE/DEGRADED — reset gate and disable control
        if (controlEnabled_) {
            Serial.println("[CAN_TX] Left ACTIVE — control disabled");
        }
        activeCount_    = 0;
        controlEnabled_ = false;
        // Force outputs to zero while disabled
        targetThrottle_ = 0;
        outputThrottle_ = 0;
        steeringRaw_    = 0;
    }

    // If control is not enabled, do not transmit any command frames (§3)
    if (!controlEnabled_) return;

    // ---- §1 Continuous throttle TX every 20 ms ----
    if (now - lastThrottleTxMs_ >= can::CMD_THROTTLE_RATE_MS) {
        lastThrottleTxMs_ = now;

        // §2 Apply ramp-down before sending
        applyThrottleRamp();
        sendThrottle(outputThrottle_);
    }

    // ---- §1 Continuous steering TX every 20 ms ----
    if (now - lastSteeringTxMs_ >= can::CMD_STEERING_RATE_MS) {
        lastSteeringTxMs_ = now;
        sendSteering(steeringRaw_);
    }

    // ---- §4 Mode/gear on-demand only ----
    if (modeDirty_) {
        modeDirty_ = false;
        sendMode(modeFlags_, gear_);
    }
}

void setThrottle(uint8_t pct) {
    if (pct > 100) pct = 100;
    targetThrottle_ = pct;
}

void setSteering(int16_t angleRaw) {
    steeringRaw_ = angleRaw;
}

void setModeAndGear(uint8_t flags, uint8_t gear) {
    if (flags != modeFlags_ || gear != gear_) {
        modeFlags_ = flags;
        gear_      = gear;
        modeDirty_ = true;
    }
}

bool isControlEnabled() {
    return controlEnabled_;
}

uint8_t currentThrottle() {
    return outputThrottle_;
}

int16_t currentSteering() {
    return steeringRaw_;
}

} // namespace can_tx
