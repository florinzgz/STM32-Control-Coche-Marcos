// =============================================================================
// ESP32-S3 HMI — Error Screen Implementation
//
// Shows full human-readable fault flag names, safety error descriptions,
// diagnostic subsystem names, and elapsed time since the error occurred.
// =============================================================================

#include "error_screen.h"
#include "ui/ui_common.h"
#include "ui/render_trace.h"
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
        default: return "UNKNOWN";
    }
}

void ErrorScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("error");
    needsRedraw_ = true;
    faultFlags_  = 0;
    prevFaultFlags_ = 0xFF;
    errorCode_   = 0;
    prevErrorCode_ = 0xFF;
    diagCode_    = 0;
    prevDiagCode_ = 0xFF;
    diagSubsystem_ = 0;
    errorEntryMs_  = millis();
    prevElapsedSec_ = 0xFFFFFFFF;
}

void ErrorScreen::onExit() {}

void ErrorScreen::update(const vehicle::VehicleData& data) {
    faultFlags_    = data.heartbeat().faultFlags;
    errorCode_     = data.safety().errorCode;
    diagCode_      = data.diag().errorCode;
    diagSubsystem_ = data.diag().subsystem;
}

void ErrorScreen::draw() {
    if (needsRedraw_) {
        needsRedraw_ = false;

        RTRACE_SET_LAYER(0);
        tft.fillScreen(ui::COL_RED);
        RTRACE_FILL_SCREEN(ui::COL_RED);

        RTRACE_SET_LAYER(1);

        // SYSTEM ERROR banner
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("SYSTEM ERROR", ui::SCREEN_W / 2, 30);
        RTRACE_TEXT(ui::SCREEN_W / 2, 30, "SYSTEM ERROR",
                    ui::COL_WHITE, ui::COL_RED, 3, MC_DATUM);

        // Manual reset instruction
        tft.setTextSize(1);
        tft.drawString("Manual reset required", ui::SCREEN_W / 2, 60);
        RTRACE_TEXT(ui::SCREEN_W / 2, 60, "Manual reset required",
                    ui::COL_WHITE, ui::COL_RED, 1, MC_DATUM);

        // Section labels (left-aligned)
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

        // Force redraw of all dynamic values
        prevFaultFlags_ = faultFlags_ + 1;
        prevErrorCode_  = errorCode_ + 1;
        prevDiagCode_   = diagCode_ + 1;
        prevElapsedSec_ = 0xFFFFFFFF;
    }

    RTRACE_SET_LAYER(2);

    // ---- Fault flags: hex code + individual flag names ----
    if (faultFlags_ != prevFaultFlags_) {
        prevFaultFlags_ = faultFlags_;

        // Clear the fault flags display area (full width)
        tft.fillRect(10, 93, 460, 70, ui::COL_RED);
        RTRACE_FILL_RECT(10, 93, 460, 70, ui::COL_RED);

        char buf[64];
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(TL_DATUM);

        // Hex code
        snprintf(buf, sizeof(buf), "0x%02X", faultFlags_);
        tft.drawString(buf, 130, 80);
        RTRACE_TEXT(130, 80, buf, ui::COL_WHITE, ui::COL_RED, 2, TL_DATUM);

        // List active fault flag names (text size 1, two columns)
        tft.setTextSize(1);
        if (faultFlags_ == 0) {
            tft.setTextColor(ui::COL_GREEN, ui::COL_RED);
            tft.drawString("No active faults", 20, 98);
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
        }
    }

    // ---- Safety error: code + human-readable name ----
    if (errorCode_ != prevErrorCode_) {
        prevErrorCode_ = errorCode_;

        tft.fillRect(10, 183, 460, 30, ui::COL_RED);
        RTRACE_FILL_RECT(10, 183, 460, 30, ui::COL_RED);

        char buf[64];
        snprintf(buf, sizeof(buf), "Code %u: %s", errorCode_,
                 safetyErrorName(errorCode_));

        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(TL_DATUM);
        tft.drawString(buf, 10, 186);
        RTRACE_TEXT(10, 186, buf, ui::COL_WHITE, ui::COL_RED, 2, TL_DATUM);
    }

    // ---- Diagnostic: error code + subsystem name ----
    if (diagCode_ != prevDiagCode_) {
        prevDiagCode_ = diagCode_;

        tft.fillRect(10, 233, 460, 30, ui::COL_RED);
        RTRACE_FILL_RECT(10, 233, 460, 30, ui::COL_RED);

        char buf[64];
        snprintf(buf, sizeof(buf), "Error %u  Subsystem: %s",
                 diagCode_, diagSubsystemName(diagSubsystem_));

        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(TL_DATUM);
        tft.drawString(buf, 10, 236);
        RTRACE_TEXT(10, 236, buf, ui::COL_WHITE, ui::COL_RED, 2, TL_DATUM);
    }

    // ---- Elapsed time (permanence) since error ----
    uint32_t elapsedSec = (millis() - errorEntryMs_) / 1000;
    if (elapsedSec != prevElapsedSec_) {
        prevElapsedSec_ = elapsedSec;

        tft.fillRect(10, 283, 460, 30, ui::COL_RED);
        RTRACE_FILL_RECT(10, 283, 460, 30, ui::COL_RED);

        char buf[48];
        uint32_t mins = elapsedSec / 60;
        uint32_t secs = elapsedSec % 60;
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
        tft.drawString(buf, 10, 286);
        RTRACE_TEXT(10, 286, buf, ui::COL_AMBER, ui::COL_RED, 2, TL_DATUM);
    }

    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    RTRACE_DUMP_IF_PENDING();
}
