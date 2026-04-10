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
#include "ui/render_trace.h"
#include <TFT_eSPI.h>
#include <cstdio>

extern TFT_eSPI tft;

// Layout constants for safe screen zones
static constexpr int16_t BANNER_H       = 40;
static constexpr int16_t INFO_Y         = 46;
static constexpr int16_t FAULT_LABEL_Y  = 62;
static constexpr int16_t FAULT_VALUE_Y  = 74;
static constexpr int16_t ERR_LABEL_Y    = 90;
static constexpr int16_t ERR_VALUE_Y    = 102;
static constexpr int16_t TELEM_SEP_Y    = 118;
static constexpr int16_t SPEED_LABEL_Y  = 126;
static constexpr int16_t SPEED_VAL_Y    = 138;
static constexpr int16_t CURR_LABEL_Y   = 154;
static constexpr int16_t CURR_VAL_Y     = 166;
static constexpr int16_t TEMP_LABEL_Y   = 182;
static constexpr int16_t TEMP_VAL_Y     = 194;
static constexpr int16_t STEER_Y        = 214;
static constexpr int16_t COL_LABEL_X    = 10;
static constexpr int16_t COL_VAL_START  = 80;
static constexpr int16_t COL_VAL_SPACE  = 100;

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
    tiles_.setRect(STILE_FAULTS,   COL_LABEL_X, FAULT_VALUE_Y, 460, 12);
    tiles_.setRect(STILE_ERROR,    COL_LABEL_X, ERR_VALUE_Y,   460, 12);
    tiles_.setRect(STILE_SPEEDS,   COL_VAL_START, SPEED_VAL_Y, 400, 14);
    tiles_.setRect(STILE_CURRENTS, COL_VAL_START, CURR_VAL_Y,  400, 14);
    tiles_.setRect(STILE_TEMPS,    COL_VAL_START, TEMP_VAL_Y,  400, 14);
    tiles_.setRect(STILE_STEERING, COL_VAL_START, STEER_Y,     140, 14);
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
        tft.fillRect(0, 0, ui::SCREEN_W, BANNER_H, ui::COL_AMBER);
        RTRACE_FILL_RECT(0, 0, ui::SCREEN_W, BANNER_H, ui::COL_AMBER);
        tft.setTextColor(ui::COL_BLACK, ui::COL_AMBER);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("SAFE MODE", ui::SCREEN_W / 2, BANNER_H / 2);
        RTRACE_TEXT(ui::SCREEN_W / 2, BANNER_H / 2, "SAFE MODE",
                    ui::COL_BLACK, ui::COL_AMBER, 3, MC_DATUM);

        // Explanation
        tft.setTextSize(1);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString("Actuators inhibited - Controls disabled", ui::SCREEN_W / 2, INFO_Y);
        RTRACE_TEXT(ui::SCREEN_W / 2, INFO_Y, "Actuators inhibited - Controls disabled",
                    ui::COL_WHITE, ui::COL_BG, 1, MC_DATUM);

        // Fault / error labels
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("FAULT FLAGS:", COL_LABEL_X, FAULT_LABEL_Y);
        tft.drawString("ERROR CODE:", COL_LABEL_X, ERR_LABEL_Y);

        // Telemetry separator
        tft.drawFastHLine(40, TELEM_SEP_Y, ui::SCREEN_W - 80, ui::COL_DARK_GRAY);

        // Telemetry labels — wheel positions
        static const char* wheelLabels[4] = { "FL", "FR", "RL", "RR" };
        tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        tft.drawString("READ-ONLY TELEMETRY", COL_LABEL_X, TELEM_SEP_Y + 2);

        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("SPEED:", COL_LABEL_X, SPEED_LABEL_Y);
        tft.drawString("CURRENT:", COL_LABEL_X, CURR_LABEL_Y);
        tft.drawString("TEMP:", COL_LABEL_X, TEMP_LABEL_Y);
        tft.drawString("STEERING:", COL_LABEL_X, STEER_Y);

        for (uint8_t i = 0; i < 4; ++i) {
            int16_t x = COL_VAL_START + i * COL_VAL_SPACE;
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            tft.drawString(wheelLabels[i], x, SPEED_LABEL_Y);
            tft.drawString(wheelLabels[i], x, CURR_LABEL_Y);
        }

        // Temp sensor labels (5 sensors)
        static const char* tempLabels[5] = { "FL", "FR", "RL", "RR", "AMB" };
        for (uint8_t i = 0; i < 5; ++i) {
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            tft.drawString(tempLabels[i], COL_VAL_START + i * 76, TEMP_LABEL_Y);
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
        tft.setTextPadding(460);

        if (faultFlags_ == 0) {
            tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
            tft.drawString("NO FAULTS", COL_LABEL_X, FAULT_VALUE_Y);
        } else {
            char buf[ui::FMT_BUF_MED];
            snprintf(buf, sizeof(buf), "0x%02X", faultFlags_);
            tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
            tft.drawString(buf, COL_LABEL_X, FAULT_VALUE_Y);
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
        tft.setTextPadding(460);
        tft.drawString(buf, COL_LABEL_X, ERR_VALUE_Y);
        tft.setTextPadding(0);
        tiles_.markClean(STILE_ERROR);
    }

    // ---- TILE: Wheel speeds ----
    if (tiles_.isDirty(STILE_SPEEDS)) {
        for (uint8_t i = 0; i < 4; ++i) {
            if (wheelSpeed_[i] != prevWheelSpeed_[i]) {
                prevWheelSpeed_[i] = wheelSpeed_[i];
                int16_t x = COL_VAL_START + 18 + i * COL_VAL_SPACE;
                char buf[ui::FMT_BUF_SMALL];
                snprintf(buf, sizeof(buf), "%u.%u km/h",
                         wheelSpeed_[i] / 10, wheelSpeed_[i] % 10);
                tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
                tft.setTextSize(1);
                tft.setTextDatum(TL_DATUM);
                tft.setTextPadding(78);
                tft.drawString(buf, x, SPEED_VAL_Y);
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
                int16_t x = COL_VAL_START + 18 + i * COL_VAL_SPACE;
                char buf[ui::FMT_BUF_SMALL];
                snprintf(buf, sizeof(buf), "%u.%02uA",
                         motorCurrent_[i] / 100, motorCurrent_[i] % 100);
                tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
                tft.setTextSize(1);
                tft.setTextDatum(TL_DATUM);
                tft.setTextPadding(78);
                tft.drawString(buf, x, CURR_VAL_Y);
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
                int16_t x = COL_VAL_START + 18 + i * 76;
                char buf[ui::FMT_BUF_SMALL];
                snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", temps_[i]);
                uint16_t col = (temps_[i] >= 80) ? ui::COL_RED :
                               (temps_[i] >= 60) ? ui::COL_AMBER : ui::COL_WHITE;
                tft.setTextColor(col, ui::COL_BG);
                tft.setTextSize(1);
                tft.setTextDatum(TL_DATUM);
                tft.setTextPadding(52);
                tft.drawString(buf, x, TEMP_VAL_Y);
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
        tft.setTextPadding(140);
        tft.drawString(buf, COL_VAL_START, STEER_Y);
        tft.setTextPadding(0);
        tiles_.markClean(STILE_STEERING);
    }

    RTRACE_DUMP_IF_PENDING();
}
