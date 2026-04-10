// =============================================================================
// ESP32-S3 HMI — Safe Screen Implementation (Tile-Based Dirty Region Engine)
//
// Displays SAFE MODE banner, fault flags, error code, and read-only
// telemetry: wheel speeds, motor currents, temperatures, steering angle.
// Each data group is a tile — only redrawn when its hash changes.
//
// Reference: docs/HMI_STATE_MODEL.md §2.5
// =============================================================================

#include "safe_screen.h"
#include "ui/ui_common.h"
#include "ui/ui_config.h"
#include "ui/render_trace.h"
#include <TFT_eSPI.h>
#include <cstdio>

extern TFT_eSPI tft;

// Layout constants — centralized in ui_config.h (namespace ui::cfg::STILE_*)
using namespace ui::cfg;

void SafeScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("safe");
    needsRedraw_ = true;
    faultFlags_  = 0;
    prevFaultFlags_ = 0xFF;
    errorCode_   = 0;
    prevErrorCode_ = 0xFF;

    wheelSpeed_.fill(0);
    prevWheelSpeed_.fill(0xFFFF);
    motorCurrent_.fill(0);
    prevMotorCurrent_.fill(0xFFFF);
    temps_.fill(0);
    prevTemps_.fill(0x7F);
    steeringAngle_     = 0;
    prevSteeringAngle_ = 0x7FFF;

    // Initialize tile regions
    tiles_.setRect(STILE_FAULTS,   STILE_COL_LABEL_X, STILE_FAULT_VALUE_Y, 460, 12);
    tiles_.setRect(STILE_ERROR,    STILE_COL_LABEL_X, STILE_ERR_VALUE_Y,   460, 12);
    tiles_.setRect(STILE_SPEEDS,   STILE_COL_VAL_START, STILE_SPEED_VAL_Y, 400, 14);
    tiles_.setRect(STILE_CURRENTS, STILE_COL_VAL_START, STILE_CURR_VAL_Y,  400, 14);
    tiles_.setRect(STILE_TEMPS,    STILE_COL_VAL_START, STILE_TEMP_VAL_Y,  400, 14);
    tiles_.setRect(STILE_STEERING, STILE_COL_VAL_START, STILE_STEER_Y,     140, 14);
    tiles_.invalidateAll();
}

void SafeScreen::onExit() {}

void SafeScreen::update(const vehicle::VehicleData& data) {
    faultFlags_ = data.heartbeat().faultFlags;
    errorCode_  = data.safety().errorCode;

    for (uint8_t i = 0; i < 4; ++i) {
        wheelSpeed_[i]   = data.speed().raw[i];
        motorCurrent_[i] = data.current().raw[i];
    }
    for (uint8_t i = 0; i < 5; ++i) {
        temps_[i] = data.temp().temps[i];
    }
    steeringAngle_ = data.steering().angleRaw;

    // ---- Compute tile hashes ----
    tiles_.updateHash(STILE_FAULTS, ui::tileHashVal(faultFlags_));
    tiles_.updateHash(STILE_ERROR,  ui::tileHashVal(errorCode_));
    tiles_.updateHash(STILE_SPEEDS, ui::tileHash(wheelSpeed_.data(),
                      wheelSpeed_.size() * sizeof(uint16_t)));
    tiles_.updateHash(STILE_CURRENTS, ui::tileHash(motorCurrent_.data(),
                      motorCurrent_.size() * sizeof(uint16_t)));
    tiles_.updateHash(STILE_TEMPS, ui::tileHash(temps_.data(),
                      temps_.size() * sizeof(int8_t)));
    tiles_.updateHash(STILE_STEERING, ui::tileHashVal(steeringAngle_));
}

void SafeScreen::draw() {
    if (needsRedraw_) {
        needsRedraw_ = false;

        RTRACE_SET_LAYER(0);
        tft.fillScreen(ui::COL_BG);
        RTRACE_FILL_SCREEN(ui::COL_BG);

        RTRACE_SET_LAYER(1);

        // Full-width SAFE MODE banner
        tft.fillRect(0, 0, ui::SCREEN_W, STILE_BANNER_H, ui::COL_AMBER);
        RTRACE_FILL_RECT(0, 0, ui::SCREEN_W, STILE_BANNER_H, ui::COL_AMBER);
        tft.setTextColor(ui::COL_BLACK, ui::COL_AMBER);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("SAFE MODE", ui::SCREEN_W / 2, STILE_BANNER_H / 2);
        RTRACE_TEXT(ui::SCREEN_W / 2, STILE_BANNER_H / 2, "SAFE MODE",
                    ui::COL_BLACK, ui::COL_AMBER, 3, MC_DATUM);

        // Explanation
        tft.setTextSize(1);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString("Actuators inhibited - Controls disabled", ui::SCREEN_W / 2, STILE_INFO_Y);
        RTRACE_TEXT(ui::SCREEN_W / 2, STILE_INFO_Y, "Actuators inhibited - Controls disabled",
                    ui::COL_WHITE, ui::COL_BG, 1, MC_DATUM);

        // Fault / error labels
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("FAULT FLAGS:", STILE_COL_LABEL_X, STILE_FAULT_LABEL_Y);
        tft.drawString("ERROR CODE:", STILE_COL_LABEL_X, STILE_ERR_LABEL_Y);

        // Telemetry separator
        tft.drawFastHLine(40, STILE_TELEM_SEP_Y, ui::SCREEN_W - 80, ui::COL_DARK_GRAY);

        // Telemetry labels — wheel positions
        static const char* wheelLabels[4] = { "FL", "FR", "RL", "RR" };
        tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        tft.drawString("READ-ONLY TELEMETRY", STILE_COL_LABEL_X, STILE_TELEM_SEP_Y + 2);

        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("SPEED:", STILE_COL_LABEL_X, STILE_SPEED_LABEL_Y);
        tft.drawString("CURRENT:", STILE_COL_LABEL_X, STILE_CURR_LABEL_Y);
        tft.drawString("TEMP:", STILE_COL_LABEL_X, STILE_TEMP_LABEL_Y);
        tft.drawString("STEERING:", STILE_COL_LABEL_X, STILE_STEER_Y);

        for (uint8_t i = 0; i < 4; ++i) {
            int16_t x = STILE_COL_VAL_START + i * STILE_COL_VAL_SPACE;
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            tft.drawString(wheelLabels[i], x, STILE_SPEED_LABEL_Y);
            tft.drawString(wheelLabels[i], x, STILE_CURR_LABEL_Y);
        }

        // Temp sensor labels (5 sensors)
        static const char* tempLabels[5] = { "FL", "FR", "RL", "RR", "AMB" };
        for (uint8_t i = 0; i < 5; ++i) {
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            tft.drawString(tempLabels[i], STILE_COL_VAL_START + i * 76, STILE_TEMP_LABEL_Y);
        }

        // Force all dynamic values to redraw
        prevFaultFlags_ = faultFlags_ + 1;
        prevErrorCode_  = errorCode_ + 1;
        prevWheelSpeed_.fill(0xFFFF);
        prevMotorCurrent_.fill(0xFFFF);
        prevTemps_.fill(0x7F);
        prevSteeringAngle_ = steeringAngle_ + 1;

        tiles_.markAllDirty();
    }

    RTRACE_SET_LAYER(2);

    // ---- TILE: Fault flags ----
    if (tiles_.isDirty(STILE_FAULTS)) {
        prevFaultFlags_ = faultFlags_;

        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
        tft.setTextPadding(ui::cfg::PAD_SAFE_FULL);

        if (faultFlags_ == 0) {
            tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
            tft.drawString("NO FAULTS", STILE_COL_LABEL_X, STILE_FAULT_VALUE_Y);
        } else {
            char buf[ui::FMT_BUF_MED];
            snprintf(buf, sizeof(buf), "0x%02X", faultFlags_);
            tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
            tft.drawString(buf, STILE_COL_LABEL_X, STILE_FAULT_VALUE_Y);
        }
        tft.setTextPadding(0);
        tiles_.markClean(STILE_FAULTS);
    }

    // ---- TILE: Error code ----
    if (tiles_.isDirty(STILE_ERROR)) {
        prevErrorCode_ = errorCode_;

        char buf[ui::FMT_BUF_MED];
        snprintf(buf, sizeof(buf), "Code: %u", errorCode_);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(ui::cfg::PAD_SAFE_FULL);
        tft.drawString(buf, STILE_COL_LABEL_X, STILE_ERR_VALUE_Y);
        tft.setTextPadding(0);
        tiles_.markClean(STILE_ERROR);
    }

    // ---- TILE: Wheel speeds ----
    if (tiles_.isDirty(STILE_SPEEDS)) {
        for (uint8_t i = 0; i < 4; ++i) {
            if (wheelSpeed_[i] != prevWheelSpeed_[i]) {
                prevWheelSpeed_[i] = wheelSpeed_[i];
                int16_t x = STILE_COL_VAL_START + 18 + i * STILE_COL_VAL_SPACE;
                char buf[ui::FMT_BUF_SMALL];
                snprintf(buf, sizeof(buf), "%u.%u km/h",
                         wheelSpeed_[i] / 10, wheelSpeed_[i] % 10);
                tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
                tft.setTextSize(1);
                tft.setTextDatum(TL_DATUM);
                tft.setTextPadding(ui::cfg::PAD_SAFE_WHEEL_VAL);
                tft.drawString(buf, x, STILE_SPEED_VAL_Y);
                tft.setTextPadding(0);
            }
        }
        tiles_.markClean(STILE_SPEEDS);
    }

    // ---- TILE: Motor currents ----
    if (tiles_.isDirty(STILE_CURRENTS)) {
        for (uint8_t i = 0; i < 4; ++i) {
            if (motorCurrent_[i] != prevMotorCurrent_[i]) {
                prevMotorCurrent_[i] = motorCurrent_[i];
                int16_t x = STILE_COL_VAL_START + 18 + i * STILE_COL_VAL_SPACE;
                char buf[ui::FMT_BUF_SMALL];
                snprintf(buf, sizeof(buf), "%u.%02uA",
                         motorCurrent_[i] / 100, motorCurrent_[i] % 100);
                tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
                tft.setTextSize(1);
                tft.setTextDatum(TL_DATUM);
                tft.setTextPadding(ui::cfg::PAD_SAFE_WHEEL_VAL);
                tft.drawString(buf, x, STILE_CURR_VAL_Y);
                tft.setTextPadding(0);
            }
        }
        tiles_.markClean(STILE_CURRENTS);
    }

    // ---- TILE: Temperatures ----
    if (tiles_.isDirty(STILE_TEMPS)) {
        for (uint8_t i = 0; i < 5; ++i) {
            if (temps_[i] != prevTemps_[i]) {
                prevTemps_[i] = temps_[i];
                int16_t x = STILE_COL_VAL_START + 18 + i * 76;
                char buf[ui::FMT_BUF_SMALL];
                snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", temps_[i]);
                uint16_t col = (temps_[i] >= ui::cfg::TEMP_COLOR_CRITICAL) ? ui::COL_RED :
                               (temps_[i] >= ui::cfg::TEMP_COLOR_WARNING) ? ui::COL_AMBER : ui::COL_WHITE;
                tft.setTextColor(col, ui::COL_BG);
                tft.setTextSize(1);
                tft.setTextDatum(TL_DATUM);
                tft.setTextPadding(ui::cfg::PAD_SAFE_TEMP);
                tft.drawString(buf, x, STILE_TEMP_VAL_Y);
                tft.setTextPadding(0);
            }
        }
        tiles_.markClean(STILE_TEMPS);
    }

    // ---- TILE: Steering angle ----
    if (tiles_.isDirty(STILE_STEERING)) {
        prevSteeringAngle_ = steeringAngle_;
        char buf[ui::FMT_BUF_MED];
        int16_t deg = steeringAngle_ / 10;
        int16_t frac = steeringAngle_ % 10;
        if (frac < 0) frac = -frac;
        snprintf(buf, sizeof(buf), "%d.%d\xC2\xB0", deg, frac);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(ui::cfg::PAD_SAFE_STEERING);
        tft.drawString(buf, STILE_COL_VAL_START, STILE_STEER_Y);
        tft.setTextPadding(0);
        tiles_.markClean(STILE_STEERING);
    }

    RTRACE_DUMP_IF_PENDING();
}
