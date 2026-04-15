// =============================================================================
// ESP32-S3 HMI — Safe Screen Implementation (Tile-Based Dirty Region Engine)
//
// Displays SAFE MODE banner, fault flags, error code, and read-only
// telemetry: wheel speeds, motor currents, temperatures, steering angle.
// Extended passive visualization: gear bar, obstacle bar, LED status, relay.
// Each data group is a tile — only redrawn when its hash changes.
//
// SAFETY NOTE: This screen is PURELY PASSIVE VISUALIZATION.
// No safety logic, state transitions, CAN transmissions, motor/relay
// control, or any functional behavior is modified.
//
// Reference: docs/HMI_STATE_MODEL.md §2.5
// =============================================================================

#include "safe_screen.h"
#include "ui/ui_common.h"
#include "ui/ui_config.h"
#include "ui/render_trace.h"
#include "ui/relay_indicator.h"
#include "shifter_input.h"
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

    // Extended visualization — force initial draw
    curGear_  = ui::Gear::N;
    prevGear_ = ui::Gear::P;
    obstacleCm_     = 0;
    prevObstacleCm_ = 0xFFFF;
    frontLedOn_     = false;
    rearLedOn_      = false;
    turnSignal_     = led_ctrl::TurnSignal::OFF;
    prevFrontLedOn_ = true;
    prevRearLedOn_  = true;
    prevTurnSignal_ = led_ctrl::TurnSignal::LEFT;
    relayStatus_     = 0;
    prevRelayStatus_ = 0xFF;

    // Initialize tile regions — existing tiles
    tiles_.setRect(STILE_FAULTS,   STILE_COL_LABEL_X, STILE_FAULT_VALUE_Y, 460, 12);
    tiles_.setRect(STILE_ERROR,    STILE_COL_LABEL_X, STILE_ERR_VALUE_Y,   460, 12);
    tiles_.setRect(STILE_SPEEDS,   STILE_COL_VAL_START, STILE_SPEED_VAL_Y, 400, 14);
    tiles_.setRect(STILE_CURRENTS, STILE_COL_VAL_START, STILE_CURR_VAL_Y,  400, 14);
    tiles_.setRect(STILE_TEMPS,    STILE_COL_VAL_START, STILE_TEMP_VAL_Y,  400, 14);
    tiles_.setRect(STILE_STEERING, STILE_COL_VAL_START, STILE_STEER_Y,     400, 14);

    // New extended tiles
    tiles_.setRect(STILE_OBSTACLE, STILE_COL_LABEL_X, STILE_OBSTACLE_Y, 460, 18);
    tiles_.setRect(STILE_LED_STAT, STILE_COL_LABEL_X, STILE_LED_STATUS_Y, 420, 14);
    tiles_.setRect(STILE_GEAR,     0, STILE_GEAR_BAR_Y, 430, 30);
    tiles_.setRect(STILE_RELAY,    STILE_RELAY_X, STILE_GEAR_BAR_Y, 50, 30);

    tiles_.invalidateAll();
}

void SafeScreen::onExit() {}

void SafeScreen::update(const vehicle::VehicleData& data, unsigned long frameTimeMs) {
    (void)frameTimeMs;  // Not used in SafeScreen — no timing-dependent logic
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

    // ---- Extended data reads (passive, read-only) ----

    // Gear — read from physical shifter (same logic as DriveScreen)
    {
        uint8_t raw = shifter::getGearRaw();
        switch (raw) {
            case 0: curGear_ = ui::Gear::P;  break;
            case 1: curGear_ = ui::Gear::R;  break;
            case 2: curGear_ = ui::Gear::N;  break;
            case 3: curGear_ = ui::Gear::D1; break;
            case 4: curGear_ = ui::Gear::D2; break;
            default: curGear_ = ui::Gear::N; break;
        }
    }

    // Obstacle sensor
    obstacleCm_ = data.obstacle().distanceCm;

    // LED relay states from STM32
    frontLedOn_ = data.lights().frontRelayOn;
    rearLedOn_  = data.lights().rearRelayOn;

    // Turn signal state (local ESP32 state, read-only)
    turnSignal_ = led_ctrl::getTurnSignal();

    // Relay status (heartbeat byte 5)
    relayStatus_ = data.heartbeat().relayStatus;

    // ---- Compute tile hashes ----
    tiles_.updateHash(STILE_FAULTS, ui::tileHashVal(faultFlags_));
    tiles_.updateHash(STILE_ERROR,  ui::tileHashVal(errorCode_));
    tiles_.updateHash(STILE_SPEEDS, ui::tileHash(wheelSpeed_.data(),
                      wheelSpeed_.size() * sizeof(uint16_t)));
    tiles_.updateHash(STILE_CURRENTS, ui::tileHash(motorCurrent_.data(),
                      motorCurrent_.size() * sizeof(uint16_t)));
    tiles_.updateHash(STILE_TEMPS, ui::tileHash(temps_.data(),
                      temps_.size() * sizeof(int8_t)));

    // Steering hash now includes the direction indicator
    {
        ui::TileHash sh = ui::tileHashVal(steeringAngle_);
        tiles_.updateHash(STILE_STEERING, sh);
    }

    // Extended tile hashes
    tiles_.updateHash(STILE_OBSTACLE, ui::tileHashVal(obstacleCm_));
    {
        ui::TileHash lh = ui::tileHashVal(frontLedOn_);
        lh = ui::tileHashFeed(lh, rearLedOn_);
        lh = ui::tileHashFeed(lh, turnSignal_);
        tiles_.updateHash(STILE_LED_STAT, lh);
    }
    tiles_.updateHash(STILE_GEAR, ui::tileHashVal(curGear_));
    tiles_.updateHash(STILE_RELAY, ui::tileHashVal(relayStatus_));
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

        // ---- Extended static labels ----

        // Obstacle label
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("OBSTACLE:", STILE_COL_LABEL_X, STILE_OBSTACLE_Y);
        // Obstacle bar outline
        tft.drawRect(STILE_OBS_BAR_X, STILE_OBSTACLE_Y + 6, STILE_OBS_BAR_W, STILE_OBS_BAR_H, ui::COL_GRAY);

        // LED status label
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("LIGHTS:", STILE_COL_LABEL_X, STILE_LED_STATUS_Y);

        // Gear bar separator
        tft.drawFastHLine(40, STILE_GEAR_BAR_Y - 4, ui::SCREEN_W - 80, ui::COL_DARK_GRAY);

        // Force all dynamic values to redraw
        prevFaultFlags_ = faultFlags_ + 1;
        prevErrorCode_  = errorCode_ + 1;
        prevWheelSpeed_.fill(0xFFFF);
        prevMotorCurrent_.fill(0xFFFF);
        prevTemps_.fill(0x7F);
        prevSteeringAngle_ = steeringAngle_ + 1;
        prevGear_ = (curGear_ == ui::Gear::P) ? ui::Gear::N : ui::Gear::P;
        prevObstacleCm_ = obstacleCm_ + 1;
        prevFrontLedOn_ = !frontLedOn_;
        prevRearLedOn_  = !rearLedOn_;
        prevTurnSignal_ = (turnSignal_ == led_ctrl::TurnSignal::OFF)
                          ? led_ctrl::TurnSignal::LEFT : led_ctrl::TurnSignal::OFF;
        prevRelayStatus_ = relayStatus_ + 1;

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

    // ---- TILE: Steering angle + visual direction indicator ----
    if (tiles_.isDirty(STILE_STEERING)) {
        prevSteeringAngle_ = steeringAngle_;
        char buf[ui::FMT_BUF_MED];
        int16_t deg = steeringAngle_ / 10;
        int16_t frac = steeringAngle_ % 10;
        if (frac < 0) frac = -frac;

        // Direction indicator: < for left, > for right, | for center
        const char* dirIndicator;
        if (steeringAngle_ < -150) {
            dirIndicator = "<< LEFT ";
        } else if (steeringAngle_ > 150) {
            dirIndicator = " RIGHT >>";
        } else {
            dirIndicator = " | CENTER";
        }

        snprintf(buf, sizeof(buf), "%d.%d\xC2\xB0%s", deg, frac, dirIndicator);

        // Color code direction
        uint16_t col = ui::COL_WHITE;
        if (steeringAngle_ < -150 || steeringAngle_ > 150) {
            col = ui::COL_CYAN;
        }

        tft.setTextColor(col, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(ui::cfg::PAD_SAFE_STEER_VIS);
        tft.drawString(buf, STILE_COL_VAL_START, STILE_STEER_Y);
        tft.setTextPadding(0);
        tiles_.markClean(STILE_STEERING);
    }

    // ---- TILE: Obstacle sensor bar ----
    if (tiles_.isDirty(STILE_OBSTACLE)) {
        prevObstacleCm_ = obstacleCm_;

        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        if (obstacleCm_ == 0) {
            // No data
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            tft.setTextPadding(ui::cfg::PAD_SAFE_OBSTACLE_TEXT);
            tft.drawString("NO DATA", STILE_OBS_BAR_X + STILE_OBS_BAR_W + 10, STILE_OBSTACLE_Y);
            tft.setTextPadding(0);

            // Clear bar fill
            tft.fillRect(STILE_OBS_BAR_X + 1, STILE_OBSTACLE_Y + 7,
                         STILE_OBS_BAR_W - 2, STILE_OBS_BAR_H - 2, ui::COL_BG);
        } else {
            // Determine state and color using existing proximity thresholds
            const char* stateText;
            uint16_t barCol;
            if (obstacleCm_ > 300) {
                stateText = "CLEAR";
                barCol = ui::COL_GREEN;
            } else if (obstacleCm_ > 80) {
                stateText = "NEAR";
                barCol = ui::COL_AMBER;
            } else {
                stateText = "DANGER";
                barCol = ui::COL_RED;
            }

            // Distance text + state
            char distBuf[ui::FMT_BUF_MED];
            snprintf(distBuf, sizeof(distBuf), "%u.%02um %s",
                     obstacleCm_ / 100, obstacleCm_ % 100, stateText);
            tft.setTextColor(barCol, ui::COL_BG);
            tft.setTextPadding(ui::cfg::PAD_SAFE_OBSTACLE_TEXT);
            tft.drawString(distBuf, STILE_OBS_BAR_X + STILE_OBS_BAR_W + 10, STILE_OBSTACLE_Y);
            tft.setTextPadding(0);

            // Fill bar proportionally (closer = more filled)
            int16_t barInner = STILE_OBS_BAR_W - 2;
            uint16_t maxCm = ui::cfg::OBSTACLE_BAR_MAX_CM;
            uint16_t clamped = (obstacleCm_ > maxCm) ? maxCm : obstacleCm_;
            int16_t fillW = static_cast<int16_t>(
                (static_cast<int32_t>(maxCm - clamped) * barInner) / maxCm);

            // Clear entire bar inner then fill
            tft.fillRect(STILE_OBS_BAR_X + 1, STILE_OBSTACLE_Y + 7,
                         barInner, STILE_OBS_BAR_H - 2, ui::COL_BG);
            if (fillW > 0) {
                tft.fillRect(STILE_OBS_BAR_X + 1, STILE_OBSTACLE_Y + 7,
                             fillW, STILE_OBS_BAR_H - 2, barCol);
            }
        }
        tiles_.markClean(STILE_OBSTACLE);
    }

    // ---- TILE: LED system status ----
    if (tiles_.isDirty(STILE_LED_STAT)) {
        prevFrontLedOn_ = frontLedOn_;
        prevRearLedOn_  = rearLedOn_;
        prevTurnSignal_ = turnSignal_;

        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        // Determine turn signal label
        const char* turnStr;
        switch (turnSignal_) {
            case led_ctrl::TurnSignal::LEFT:   turnStr = "LEFT";   break;
            case led_ctrl::TurnSignal::RIGHT:  turnStr = "RIGHT";  break;
            case led_ctrl::TurnSignal::HAZARD: turnStr = "HAZARD"; break;
            default:                           turnStr = "OFF";    break;
        }

        // Draw with color-coded segments
        int16_t x = STILE_COL_VAL_START;

        // Front LED
        tft.setTextColor(frontLedOn_ ? ui::COL_GREEN : ui::COL_GRAY, ui::COL_BG);
        tft.setTextPadding(42);
        tft.drawString(frontLedOn_ ? "F:ON" : "F:OFF", x, STILE_LED_STATUS_Y);
        tft.setTextPadding(0);

        // Rear LED
        tft.setTextColor(rearLedOn_ ? ui::COL_GREEN : ui::COL_GRAY, ui::COL_BG);
        tft.setTextPadding(42);
        tft.drawString(rearLedOn_ ? "R:ON" : "R:OFF", x + 60, STILE_LED_STATUS_Y);
        tft.setTextPadding(0);

        // Turn signal
        uint16_t turnCol = (turnSignal_ != led_ctrl::TurnSignal::OFF)
                           ? ui::COL_AMBER : ui::COL_GRAY;
        char turnBuf[ui::FMT_BUF_SMALL];
        snprintf(turnBuf, sizeof(turnBuf), "T:%s", turnStr);
        tft.setTextColor(turnCol, ui::COL_BG);
        tft.setTextPadding(80);
        tft.drawString(turnBuf, x + 120, STILE_LED_STATUS_Y);
        tft.setTextPadding(0);

        tiles_.markClean(STILE_LED_STAT);
    }

    // ---- TILE: Gear bar (reuses GearDisplay widget style) ----
    if (tiles_.isDirty(STILE_GEAR)) {
        // Draw all gear labels with active gear highlighted
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);

        static const char* gearLabels[ui::NUM_GEARS] = { "P", "R", "N", "D1", "D2" };
        for (uint8_t i = 0; i < ui::NUM_GEARS; ++i) {
            int16_t gx = ui::GEAR_START_X + i * ui::GEAR_SPACING;
            int16_t gy = STILE_GEAR_BAR_Y;
            int16_t cx = gx + ui::GEAR_LABEL_W / 2;
            int16_t cy = gy + ui::GEAR_LABEL_H / 2;

            bool isActive = (i == static_cast<uint8_t>(curGear_));

            if (isActive) {
                tft.fillRect(gx, gy, ui::GEAR_LABEL_W, ui::GEAR_LABEL_H, ui::COL_GREEN);
                tft.drawRect(gx, gy, ui::GEAR_LABEL_W, ui::GEAR_LABEL_H, ui::COL_WHITE);
                tft.setTextColor(ui::COL_BLACK, ui::COL_GREEN);
            } else {
                tft.fillRect(gx, gy, ui::GEAR_LABEL_W, ui::GEAR_LABEL_H, ui::COL_BG);
                tft.drawRect(gx, gy, ui::GEAR_LABEL_W, ui::GEAR_LABEL_H, ui::COL_GRAY);
                tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            }
            tft.drawString(gearLabels[i], cx, cy);
        }

        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);

        prevGear_ = curGear_;
        tiles_.markClean(STILE_GEAR);
    }

    // ---- TILE: Relay status indicator ----
    if (tiles_.isDirty(STILE_RELAY)) {
        // Reuse RelayIndicator widget with SafeScreen Y position
        const bool seqComplete = (relayStatus_ & 0x80U) != 0;
        const bool mainOn      = (relayStatus_ & 0x01U) != 0;
        const bool tracOn      = (relayStatus_ & 0x02U) != 0;
        const bool dirOn       = (relayStatus_ & 0x04U) != 0;

        // Clear background
        tft.fillRect(STILE_RELAY_X, STILE_GEAR_BAR_Y,
                     50, ui::GEAR_LABEL_H, ui::COL_BG);

        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        const int16_t textY = STILE_GEAR_BAR_Y + (ui::GEAR_LABEL_H - 8) / 2;

        auto relayCol = [](bool on, bool seq) -> uint16_t {
            if (!on) return ui::COL_GRAY;
            return seq ? ui::COL_GREEN : ui::COL_AMBER;
        };

        tft.setTextColor(relayCol(mainOn, seqComplete), ui::COL_BG);
        tft.drawChar('M', STILE_RELAY_X, textY);

        tft.setTextColor(relayCol(tracOn, seqComplete), ui::COL_BG);
        tft.drawChar('T', STILE_RELAY_X + 14, textY);

        tft.setTextColor(relayCol(dirOn, seqComplete), ui::COL_BG);
        tft.drawChar('D', STILE_RELAY_X + 28, textY);

        prevRelayStatus_ = relayStatus_;
        tiles_.markClean(STILE_RELAY);
    }

    RTRACE_DUMP_IF_PENDING();
}
