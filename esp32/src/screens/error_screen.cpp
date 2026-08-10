// =============================================================================
// ESP32-S3 HMI — Error Screen Implementation (Tile-Based Dirty Region Engine)
//
// Shows full human-readable fault flag names, safety error descriptions,
// diagnostic subsystem names, and elapsed time since the error occurred.
//
// Tile map:
//   ETILE_BANNER   [  0,  0, 480, 75]  — title banner (SYSTEM ERROR / CAN LINK LOST)
//   ETILE_FAULTS   [ 10, 80, 460, 83]  — fault flags hex + individual names
//   ETILE_SAFETY   [ 10,170, 460, 40]  — safety error code + name
//   ETILE_DIAG     [ 10,220, 460, 40]  — diagnostic error + subsystem
//   ETILE_ELAPSED  [ 10,270, 460, 30]  — elapsed time since error
//
// Each tile is only redrawn when its content hash changes.
// =============================================================================

#include "error_screen.h"
#include "ui/ui_common.h"
#include "ui/ui_config.h"
#include "ui/render_trace.h"
#include "ui/sensor_fault_hint.h"
#include "can_ids.h"
#include <TFT_eSPI.h>
#include <Arduino.h>
#include <cstdio>

extern TFT_eSPI tft;

// ---- Fault flag bit names (bit 0..7 of HEARTBEAT_STM32 faultFlags) ----
static const char* const FAULT_NAMES[8] = {
    "CAN_TIMEOUT",     // Bit 0
    "TEMP_OVERLOAD",   // Bit 1
    "CURR_OVERLOAD",   // Bit 2
    "ENCODER_ERR",     // Bit 3
    "WHEEL_SENSOR",    // Bit 4
    "ABS_ACTIVE",      // Bit 5
    "TCS_ACTIVE",      // Bit 6
    "CENTERING"        // Bit 7
};

// ---- Safety error code → human-readable name ----
const char* ErrorScreen::safetyErrorName(uint8_t code) {
    switch (code) {
        case 0:  return "NONE";
        case 1:  return "OVERCURRENT";
        case 2:  return "OVERTEMP";
        case 3:  return "CAN TIMEOUT";
        case 4:  return "SENSOR FAULT";
        case 5:  return "MOTOR STALL";
        case 6:  return "EMERGENCY STOP";
        case 7:  return "WATCHDOG";
        case 8:  return "CENTERING FAIL";
        case 9:  return "BATTERY UV WARN";
        case 10: return "BATTERY UV CRIT";
        case 11: return "I2C FAILURE";
        case 12: return "OBSTACLE";
        case 13: return "CAN BUS-OFF";
        case 14: return "BATTERY OV WARN";
        case 15: return "BATTERY OV CRIT";
        case 16: return "RELAY OPEN";
        /* SVCDIAG_DTC_CODE_ENTER/_EXIT (can_handler.c) — SERVICE_DIAG
         * self-test session ENTER/EXIT log events, not real safety faults;
         * deliberately outside the 0-16 Safety_Error_t range.             */
        case 0x40: return "SVC-DIAG ENTER";
        case 0x41: return "SVC-DIAG EXIT";
        default: return "UNKNOWN";
    }
}

// ---- Diagnostic subsystem → name ----
const char* ErrorScreen::diagSubsystemName(uint8_t sub) {
    switch (sub) {
        case 0: return "GLOBAL";
        case 1: return "MOTOR";
        case 2: return "SENSOR";
        case 3: return "CAN_BUS";
        case 4: return "SVC_DIAG";
        default: return "UNKNOWN";
    }
}

// ---- Wheel-diagnostic reason (0x313) → human-readable name ----
static const char* wheelReasonName(uint8_t reason) {
    switch (reason) {
        case can::WHEEL_DIAG_REASON_OK:              return "OK";
        case can::WHEEL_DIAG_REASON_NO_PULSE:        return "NO PULSE";
        case can::WHEEL_DIAG_REASON_STUCK_HIGH:      return "STUCK HIGH";
        case can::WHEEL_DIAG_REASON_STUCK_LOW:       return "STUCK LOW";
        case can::WHEEL_DIAG_REASON_MISMATCH:        return "MISMATCH";
        case can::WHEEL_DIAG_REASON_IMPOSSIBLE_RATE: return "BAD RATE";
        case can::WHEEL_DIAG_REASON_MANUAL_MOVEMENT: return "MANUAL";
        case can::WHEEL_DIAG_REASON_DISABLED_STATE:  return "DISABLED";
        default:                                     return "UNKNOWN";
    }
}

void ErrorScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("error");
    needsRedraw_ = true;
    canLost_     = false;
    canRxActive_ = false;
    prevCanLost_ = false;
    prevCanRxActive_ = false;
    faultFlags_  = 0;
    prevFaultFlags_ = 0xFF;
    errorCode_   = 0;
    prevErrorCode_ = 0xFF;
    diagCode_    = 0;
    prevDiagCode_ = 0xFF;
    diagSubsystem_ = 0;
    errorEntryMs_  = 0;               // captured on first update() (frame time contract)
    prevElapsedSec_ = 0xFFFFFFFF;
    prevDiagSubsystem_ = 0xFF;
    failsafeFrameCount_ = 0;

    // Initialize tile regions (dimensions from ui_config.h)
    tiles_.setRect(ETILE_BANNER,  0,   0,
                   ui::SCREEN_W, ui::cfg::ETILE_BANNER_H);
    tiles_.setRect(ETILE_FAULTS,  ui::cfg::ETILE_CONTENT_X, ui::cfg::ETILE_FAULTS_Y,
                   ui::cfg::ETILE_CONTENT_W, ui::cfg::ETILE_FAULTS_H);
    tiles_.setRect(ETILE_SAFETY,  ui::cfg::ETILE_CONTENT_X, ui::cfg::ETILE_SAFETY_Y,
                   ui::cfg::ETILE_CONTENT_W, ui::cfg::ETILE_SAFETY_H);
    tiles_.setRect(ETILE_DIAG,    ui::cfg::ETILE_CONTENT_X, ui::cfg::ETILE_DIAG_Y,
                   ui::cfg::ETILE_CONTENT_W, ui::cfg::ETILE_DIAG_H);
    tiles_.setRect(ETILE_ELAPSED, ui::cfg::ETILE_CONTENT_X, ui::cfg::ETILE_ELAPSED_Y,
                   ui::cfg::ETILE_CONTENT_W, ui::cfg::ETILE_ELAPSED_H);
    tiles_.invalidateAll();
}

void ErrorScreen::onExit() {}

void ErrorScreen::update(const vehicle::VehicleData& data, unsigned long frameTimeMs) {
    // Capture error entry time on first update() — ensures we use the injected
    // frameTimeMs instead of a raw millis() call (frame time contract §4).
    if (errorEntryMs_ == 0) {
        errorEntryMs_ = frameTimeMs;
    }

    // Detect CAN communication loss: heartbeat was once received but is
    // now older than CAN_LOSS_TIMEOUT_MS.
    unsigned long hbTs = data.heartbeat().timestampMs;
    canLost_ = (hbTs > 0) &&
               ((frameTimeMs - hbTs) > can::CAN_LOSS_TIMEOUT_MS);

    // Detect whether OTHER CAN frames (not heartbeat) have arrived recently.
    // If so, the physical bus is alive and only the heartbeat is missing —
    // the banner should say "STM32 HB LOST" rather than "CAN LINK LOST".
    if (canLost_) {
        const unsigned long kRecentMs = can::CAN_LOSS_TIMEOUT_MS;
        const unsigned long i2cTs   = data.i2cDiag().timestampMs;
        const unsigned long speedTs = data.speed().timestampMs;
        const unsigned long safetyTs = data.safety().timestampMs;
        canRxActive_ =
            (i2cTs > 0   && (frameTimeMs - i2cTs)   < kRecentMs) ||
            (speedTs > 0 && (frameTimeMs - speedTs)  < kRecentMs) ||
            (safetyTs > 0 && (frameTimeMs - safetyTs) < kRecentMs);
    } else {
        canRxActive_ = false;
    }

    faultFlags_    = data.heartbeat().faultFlags;
    errorCode_     = data.safety().errorCode;
    diagCode_      = data.diag().errorCode;
    diagSubsystem_ = data.diag().subsystem;

    // Cache the per-wheel fault-reason diagnostic (0x313) for the WHEEL_SENSOR
    // hint.  Treat a frame older than 2 s (or never seen) as no data.
    {
        const auto& wd = data.wheelSensorDiag();
        wheelDiagValid_ = wd.valid &&
                          ((frameTimeMs - wd.timestampMs) <= 2000UL);
        for (uint8_t i = 0; i < 5; ++i) wheelReason_[i] = wd.reason[i];
        wheelFaultMask_ = wd.faultMask;
    }

    // Cache the pedal (0x20B) and service-fault (0x301/0x303) diagnostic
    // inputs used by the SENSOR_FAULT (code 4) cause hint.  Same 2 s
    // freshness convention as the wheel-diag hint above.
    {
        const auto& pd = data.pedal();
        pedalExtended_      = pd.extended;
        pedalPlausible_     = pd.plausible;
        pedalContradictory_ = pd.contradictory;
        pedalFresh_         = (pd.timestampMs > 0) &&
                               ((frameTimeMs - pd.timestampMs) <= 2000UL);

        const auto& sd = data.service();
        serviceFaultMask_    = sd.faultMask;
        serviceDisabledMask_ = sd.disabledMask;
        serviceFresh_        = (sd.faultTimestampMs > 0) &&
                                ((frameTimeMs - sd.faultTimestampMs) <= 2000UL);
    }

    // ---- Compute tile hashes ----
    // Feed canLost_ into the fault/safety/diag hashes so the "(STALE)" marker
    // appears/clears correctly: on CAN loss these values are the last frame
    // received before the link dropped and may be mutually inconsistent
    // (e.g. faultFlags=0 "No active faults" yet a cached Safety Code 4).
    {
        ui::TileHash bh = ui::tileHashVal(canLost_);
        bh = ui::tileHashFeed(bh, canRxActive_ ? 1u : 0u);
        tiles_.updateHash(ETILE_BANNER, bh);
    }
    {
        ui::TileHash fh = ui::tileHashVal(faultFlags_);
        fh = ui::tileHashFeed(fh, canLost_ ? 1u : 0u);
        // Fold the wheel-diag hint into the fault tile so it refreshes when the
        // failing wheel/reason changes even while faultFlags_ stays 0x10.
        fh = ui::tileHashFeed(fh, wheelDiagValid_ ? 1u : 0u);
        fh = ui::tileHashFeed(fh, wheelFaultMask_);
        for (uint8_t i = 0; i < 5; ++i) fh = ui::tileHashFeed(fh, wheelReason_[i]);
        // Fold the SENSOR_FAULT cause-hint inputs so the tile refreshes when
        // the pedal/service diagnostic changes even though errorCode_ and
        // faultFlags_ stay constant (e.g. the culprit INA226 channel changes).
        fh = ui::tileHashFeed(fh, errorCode_);
        fh = ui::tileHashFeed(fh, pedalExtended_ ? 1u : 0u);
        fh = ui::tileHashFeed(fh, pedalPlausible_ ? 1u : 0u);
        fh = ui::tileHashFeed(fh, pedalContradictory_ ? 1u : 0u);
        fh = ui::tileHashFeed(fh, pedalFresh_ ? 1u : 0u);
        fh = ui::tileHashFeed(fh, serviceFaultMask_);
        fh = ui::tileHashFeed(fh, serviceDisabledMask_);
        fh = ui::tileHashFeed(fh, serviceFresh_ ? 1u : 0u);
        tiles_.updateHash(ETILE_FAULTS, fh);
    }
    {
        ui::TileHash sh = ui::tileHashVal(errorCode_);
        sh = ui::tileHashFeed(sh, canLost_ ? 1u : 0u);
        tiles_.updateHash(ETILE_SAFETY, sh);
    }
    {
        ui::TileHash dh = ui::tileHashVal(diagCode_);
        dh = ui::tileHashFeed(dh, diagSubsystem_);
        dh = ui::tileHashFeed(dh, canLost_ ? 1u : 0u);
        tiles_.updateHash(ETILE_DIAG, dh);
    }
    // Elapsed time — computed once here, used in both hash and draw()
    elapsedSec_ = (frameTimeMs - errorEntryMs_) / 1000;
    tiles_.updateHash(ETILE_ELAPSED, ui::tileHashVal(elapsedSec_));

    // ---- Hash failsafe: staggered forced redraw of critical tiles (V10) ----
    ++failsafeFrameCount_;
    if (failsafeFrameCount_ >= ui::cfg::HASH_FAILSAFE_INTERVAL) {
        failsafeFrameCount_ = 0;
    }
    {
        constexpr uint16_t STAGGER = ui::cfg::HASH_FAILSAFE_INTERVAL / 2;
        if (failsafeFrameCount_ == 0)          tiles_.forceRedraw(ETILE_BANNER);
        if (failsafeFrameCount_ == STAGGER)    tiles_.forceRedraw(ETILE_FAULTS);
    }
}

void ErrorScreen::draw() {
    if (needsRedraw_) {
        needsRedraw_ = false;

        // ---- STATIC LAYER ----
        RTRACE_SET_LAYER(0);
        tft.fillScreen(ui::COL_RED);
        RTRACE_FILL_SCREEN(ui::COL_RED);

        RTRACE_SET_LAYER(1);

        // Section labels (left-aligned) — drawn once
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(1);
        tft.drawString("FAULT FLAGS:", 10, 80);
        RTRACE_TEXT(10, 80, "FAULT FLAGS:",
                    ui::COL_WHITE, ui::COL_RED, 1, TL_DATUM);
        tft.drawString("SAFETY ERROR:", 10, 170);
        RTRACE_TEXT(10, 170, "SAFETY ERROR:",
                    ui::COL_WHITE, ui::COL_RED, 1, TL_DATUM);
        tft.drawString("DIAGNOSTIC:", 10, 220);
        RTRACE_TEXT(10, 220, "DIAGNOSTIC:",
                    ui::COL_WHITE, ui::COL_RED, 1, TL_DATUM);
        tft.drawString("ELAPSED:", 10, 270);
        RTRACE_TEXT(10, 270, "ELAPSED:",
                    ui::COL_WHITE, ui::COL_RED, 1, TL_DATUM);

        // Force redraw of all dynamic tiles
        prevFaultFlags_ = faultFlags_ + 1;
        prevErrorCode_  = errorCode_ + 1;
        prevDiagCode_   = diagCode_ + 1;
        prevElapsedSec_ = 0xFFFFFFFF;

        tiles_.markAllDirty();
    }

    RTRACE_SET_LAYER(2);

    // ---- TILE: Banner (CAN LOST / SYSTEM ERROR) ----
    if (tiles_.isDirty(ETILE_BANNER)) {
        prevCanLost_     = canLost_;
        prevCanRxActive_ = canRxActive_;

        // Clear only the banner area (top 75 px), not the entire screen
        tft.fillRect(0, 0, ui::SCREEN_W, 75, ui::COL_RED);
        RTRACE_FILL_RECT(0, 0, ui::SCREEN_W, 75, ui::COL_RED);

        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);

        if (canLost_) {
            // Differentiate "heartbeat lost but bus alive" from "bus down":
            // if other CAN frames are arriving recently, the physical bus is OK
            // and only the STM32 heartbeat (0x001) is missing.
            if (canRxActive_) {
                tft.drawString("STM32 HB LOST", ui::SCREEN_W / 2, 30);
                RTRACE_TEXT(ui::SCREEN_W / 2, 30, "STM32 HB LOST",
                            ui::COL_WHITE, ui::COL_RED, 3, MC_DATUM);

                tft.setTextSize(1);
                tft.drawString("CAN RX ACTIVE — STM32 heartbeat not received", ui::SCREEN_W / 2, 60);
                RTRACE_TEXT(ui::SCREEN_W / 2, 60, "CAN RX ACTIVE — STM32 heartbeat not received",
                            ui::COL_WHITE, ui::COL_RED, 1, MC_DATUM);
            } else {
                tft.drawString("CAN LINK LOST", ui::SCREEN_W / 2, 30);
                RTRACE_TEXT(ui::SCREEN_W / 2, 30, "CAN LINK LOST",
                            ui::COL_WHITE, ui::COL_RED, 3, MC_DATUM);

                tft.setTextSize(1);
                tft.drawString("STM32 heartbeat not received", ui::SCREEN_W / 2, 60);
                RTRACE_TEXT(ui::SCREEN_W / 2, 60, "STM32 heartbeat not received",
                            ui::COL_WHITE, ui::COL_RED, 1, MC_DATUM);
            }
        } else {
            tft.drawString("SYSTEM ERROR", ui::SCREEN_W / 2, 30);
            RTRACE_TEXT(ui::SCREEN_W / 2, 30, "SYSTEM ERROR",
                        ui::COL_WHITE, ui::COL_RED, 3, MC_DATUM);

            tft.setTextSize(1);
            tft.drawString("Manual reset required", ui::SCREEN_W / 2, 60);
            RTRACE_TEXT(ui::SCREEN_W / 2, 60, "Manual reset required",
                        ui::COL_WHITE, ui::COL_RED, 1, MC_DATUM);
        }
        tft.setTextDatum(TL_DATUM);
        tiles_.markClean(ETILE_BANNER);
    }

    // ---- TILE: Fault flags ----
    if (tiles_.isDirty(ETILE_FAULTS)) {
        prevFaultFlags_ = faultFlags_;

        // Clear the fault flags display area (multi-line layout requires fillRect)
        tft.fillRect(10, 93, 460, 70, ui::COL_RED);
        RTRACE_FILL_RECT(10, 93, 460, 70, ui::COL_RED);

        char buf[64];
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(TL_DATUM);

        // Hex code — padded to avoid remnants
        snprintf(buf, sizeof(buf), "0x%02X", faultFlags_);
        tft.setTextPadding(ui::cfg::PAD_ERROR_HEX);
        tft.drawString(buf, 130, 80);
        RTRACE_TEXT(130, 80, buf, ui::COL_WHITE, ui::COL_RED, 2, TL_DATUM);
        tft.setTextPadding(0);

        // List active fault flag names (text size 1, two columns)
        tft.setTextSize(1);
        if (faultFlags_ == 0) {
            tft.setTextColor(ui::COL_GREEN, ui::COL_RED);
            tft.drawString(canLost_ ? "No active faults (STALE)" : "No active faults", 20, 98);
        } else {
            tft.setTextColor(ui::COL_YELLOW, ui::COL_RED);
            int16_t flagY = 98;
            int16_t flagX = 20;
            for (uint8_t bit = 0; bit < 8; ++bit) {
                if (faultFlags_ & (1U << bit)) {
                    snprintf(buf, sizeof(buf), "[%u] %s", bit, FAULT_NAMES[bit]);
                    tft.drawString(buf, flagX, flagY);
                    flagY += 12;
                    if (flagY > 150) {
                        flagY = 98;
                        flagX = 250;
                    }
                }
            }
            // CAN down → these flags are a frozen snapshot, not live faults.
            if (canLost_) {
                tft.setTextColor(ui::COL_AMBER, ui::COL_RED);
                tft.drawString("(STALE — last seen before CAN loss)", 20, 150);
            }
        }

        // ---- WHEEL_SENSOR (0x10) hint: name the failing wheel + reason ----
        // Uses the 0x313 diagnostic to turn an unlabelled "SENSOR FAULT" into
        // e.g. "Wheel: FR MISMATCH".  Falls back silently to the old behaviour
        // when no 0x313 data is available (backward-compatible).
        if ((faultFlags_ & 0x10U) && wheelDiagValid_) {
            static const char* const kWheel[5] = { "FL", "FR", "RL", "RR", "ST" };
            int8_t pick = -1;
            // Prefer a channel with a latched fault bit; else the first channel
            // reporting an escalating fault reason (NO_PULSE..IMPOSSIBLE_RATE).
            for (uint8_t i = 0; i < 4 && pick < 0; ++i) {
                if (wheelFaultMask_ & (1U << i)) pick = (int8_t)i;
            }
            for (uint8_t i = 0; i < 5 && pick < 0; ++i) {
                const uint8_t r = wheelReason_[i];
                if (r >= can::WHEEL_DIAG_REASON_NO_PULSE &&
                    r <= can::WHEEL_DIAG_REASON_IMPOSSIBLE_RATE) pick = (int8_t)i;
            }
            char hbuf[40];
            if (pick >= 0) {
                snprintf(hbuf, sizeof(hbuf), "Wheel: %s %s",
                         kWheel[pick], wheelReasonName(wheelReason_[pick]));
            } else {
                // 0x10 latched but every current reason is non-fault (e.g. a
                // historic DTC): tell the operator it is clearable, not live.
                snprintf(hbuf, sizeof(hbuf), "Wheel: clear DTC / reboot");
            }
            tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
            tft.drawString(hbuf, 250, 150);
        } else if (errorCode_ == ui::SAFETY_ERROR_SENSOR_FAULT_CODE) {
            // ---- SENSOR_FAULT (code 4) cause hint: pedal / INA226 / temp /
            // wheel (fallback) ----
            // Reuses this same hint slot (mutually exclusive with the
            // WHEEL_SENSOR hint above) to name which underlying sensor raised
            // the generic SENSOR FAULT.  See esp32/src/ui/sensor_fault_hint.h
            // for the priority/format rules; falls back silently (no text) if
            // no specific cause can be identified from the cached CAN data.
            char hbuf[40];
            ui::SensorFaultCause cause = ui::buildSensorFaultHint(
                errorCode_,
                pedalExtended_, pedalPlausible_, pedalContradictory_, pedalFresh_,
                serviceFaultMask_, serviceDisabledMask_, serviceFresh_,
                hbuf, sizeof(hbuf));
            if (cause != ui::SensorFaultCause::NONE) {
                tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
                tft.drawString(hbuf, 250, 150);
            }
        }
        tiles_.markClean(ETILE_FAULTS);
    }

    // ---- TILE: Safety error ----
    if (tiles_.isDirty(ETILE_SAFETY)) {
        prevErrorCode_ = errorCode_;

        char buf[64];
        snprintf(buf, sizeof(buf), canLost_ ? "Code %u: %s (STALE)" : "Code %u: %s", // flawfinder: ignore
                 errorCode_, safetyErrorName(errorCode_));

        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(ui::cfg::PAD_ERROR_FULL);
        tft.drawString(buf, 10, 186);
        RTRACE_TEXT(10, 186, buf, ui::COL_WHITE, ui::COL_RED, 2, TL_DATUM);
        tft.setTextPadding(0);
        tiles_.markClean(ETILE_SAFETY);
    }

    // ---- TILE: Diagnostic ----
    if (tiles_.isDirty(ETILE_DIAG)) {
        prevDiagCode_ = diagCode_;
        prevDiagSubsystem_ = diagSubsystem_;

        char buf[64];
        snprintf(buf, sizeof(buf), canLost_ ? "Error %u  Subsystem: %s (STALE)" // flawfinder: ignore
                                             : "Error %u  Subsystem: %s",
                 diagCode_, diagSubsystemName(diagSubsystem_));

        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(ui::cfg::PAD_ERROR_FULL);
        tft.drawString(buf, 10, 236);
        RTRACE_TEXT(10, 236, buf, ui::COL_WHITE, ui::COL_RED, 2, TL_DATUM);
        tft.setTextPadding(0);
        tiles_.markClean(ETILE_DIAG);
    }

    // ---- TILE: Elapsed time ----
    if (tiles_.isDirty(ETILE_ELAPSED)) {
        prevElapsedSec_ = elapsedSec_;

        char buf[48];
        uint32_t mins = elapsedSec_ / 60;
        uint32_t secs = elapsedSec_ % 60;
        if (mins > 0) {
            snprintf(buf, sizeof(buf), "%lum %02lus in error state",
                     (unsigned long)mins, (unsigned long)secs);
        } else {
            snprintf(buf, sizeof(buf), "%lus in error state",
                     (unsigned long)secs);
        }

        tft.setTextColor(ui::COL_AMBER, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(ui::cfg::PAD_ERROR_FULL);
        tft.drawString(buf, 10, 286);
        RTRACE_TEXT(10, 286, buf, ui::COL_AMBER, ui::COL_RED, 2, TL_DATUM);
        tft.setTextPadding(0);
        tiles_.markClean(ETILE_ELAPSED);
    }

    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    RTRACE_DUMP_IF_PENDING();
}
