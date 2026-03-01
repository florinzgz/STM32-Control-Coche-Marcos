// =============================================================================
// ESP32-S3 HMI — Engineering Screen Implementation
//
// Hidden engineering menu with submenus for diagnostics and calibration.
// Accessed via secret code "8989" (handled by screen_manager).
// =============================================================================

#include "engineering_screen.h"
#include "ui/ui_common.h"
#include "ui/render_trace.h"
#include "can_ids.h"
#include "config_store.h"
#include <TFT_eSPI.h>
#include <ESP32-TWAI-CAN.hpp>
#include <cstdio>
#include <cstring>

extern TFT_eSPI tft;

// ---- Menu button layout ----
static constexpr int16_t MENU_X       = 40;
static constexpr int16_t MENU_W       = 400;
static constexpr int16_t MENU_BTN_H   = 36;
static constexpr int16_t MENU_START_Y = 55;
static constexpr int16_t MENU_SPACING = 42;

static constexpr int     NUM_MAIN_ITEMS = 7;
static const char* const mainLabels[NUM_MAIN_ITEMS] = {
    "FAULT VIEWER",
    "MODULE ENABLE/DISABLE",
    "PEDAL CALIBRATION",
    "ENCODER CALIBRATION",
    "INA226 SENSOR MAPPING",
    "TEMP SENSOR MAPPING",
    "FACTORY RESTORE"
};

// ---- Back / Save buttons ----
static constexpr int16_t BACK_X = 10;
static constexpr int16_t BACK_Y = 280;
static constexpr int16_t BACK_W = 80;
static constexpr int16_t BACK_H = 30;

static constexpr int16_t SAVE_X = 390;
static constexpr int16_t SAVE_Y = 280;
static constexpr int16_t SAVE_W = 80;
static constexpr int16_t SAVE_H = 30;

// ---- Sensor mapping row layout ----
static constexpr int16_t MAP_ROW_X   = 10;
static constexpr int16_t MAP_ROW_W   = 460;
static constexpr int16_t MAP_ROW_H   = 28;
static constexpr int16_t MAP_ROW_Y0  = 55;
static constexpr int16_t MAP_ROW_SPC = 32;

// INA226 position labels (channel index → position name)
static const char* const INA_CHAN_NAMES[config_store::NUM_INA226_CH] = {
    "Ch0", "Ch1", "Ch2", "Ch3", "Ch4", "Ch5"
};
static const char* const INA_POS_NAMES[config_store::NUM_INA226_CH] = {
    "FL Motor", "FR Motor", "RL Motor", "RR Motor", "Battery", "Steering"
};

// DS18B20 position labels (sensor index → position name)
static const char* const TEMP_IDX_NAMES[config_store::NUM_TEMP_SENS] = {
    "Sens0", "Sens1", "Sens2", "Sens3", "Sens4"
};
static const char* const TEMP_POS_NAMES[config_store::NUM_TEMP_SENS] = {
    "FL Wheel", "FR Wheel", "RL Wheel", "RR Wheel", "Ambient"
};

// ---- Module control layout ----
static constexpr int16_t MOD_ROW_X   = 10;
static constexpr int16_t MOD_ROW_W   = 460;
static constexpr int16_t MOD_ROW_H   = 22;
static constexpr int16_t MOD_ROW_Y0  = 48;
static constexpr int16_t MOD_ROW_SPC = 24;
static constexpr uint8_t MOD_ROWS_PER_PAGE = 9;
static constexpr uint8_t MOD_TOTAL_COUNT   = 25;

// First non-critical module ID (modules 0–3 are CRITICAL and cannot be toggled)
static constexpr uint8_t FIRST_NON_CRITICAL = 4;

// NEXT/PREV button layout
static constexpr int16_t PAGE_BTN_X  = 200;
static constexpr int16_t PAGE_BTN_Y  = 280;
static constexpr int16_t PAGE_BTN_W  = 80;
static constexpr int16_t PAGE_BTN_H  = 30;

// Module names matching ModuleID_t in service_mode.h
static const char* const MODULE_NAMES[MOD_TOTAL_COUNT] = {
    "CAN Timeout",        // 0  CRITICAL
    "Emergency Stop",     // 1  CRITICAL
    "Watchdog",           // 2  CRITICAL
    "Main Relay",         // 3  CRITICAL
    "Temp Sensor 0",      // 4
    "Temp Sensor 1",      // 5
    "Temp Sensor 2",      // 6
    "Temp Sensor 3",      // 7
    "Temp Sensor 4",      // 8
    "Current Sens 0 (FL)", // 9
    "Current Sens 1 (FR)", // 10
    "Current Sens 2 (RL)", // 11
    "Current Sens 3 (RR)", // 12
    "Current Sens 4 (Bat)",// 13
    "Current Sens 5 (Str)",// 14
    "Wheel Speed FL",      // 15
    "Wheel Speed FR",      // 16
    "Wheel Speed RL",      // 17
    "Wheel Speed RR",      // 18
    "Steer Center",        // 19
    "Steer Encoder",       // 20
    "ABS",                 // 21
    "TCS",                 // 22
    "Ackermann",           // 23
    "Obstacle Detect"      // 24
};

// -------------------------------------------------------------------------
// Lifecycle
// -------------------------------------------------------------------------
void EngineeringScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("engineering");
    needsRedraw_ = true;
    exitRequested_ = false;
    currentMenu_ = SubMenu::MAIN;
    inaEditRow_  = 0;
    tempEditRow_ = 0;
    moduleCtrlPage_ = 0;
    prevFaultBits_    = 0xFFFFFFFF;
    prevEnabledBits_  = 0xFFFFFFFF;
    prevDisabledBits_ = 0xFFFFFFFF;
    pedalDataChanged_   = false;
    encoderDataChanged_ = false;

    // Load current sensor mappings from config into working copies
    const auto& cfg = config_store::get();
    memcpy(inaMap_,  cfg.ina226Map,      config_store::NUM_INA226_CH);
    memcpy(tempMap_, cfg.tempSensorMap,  config_store::NUM_TEMP_SENS);
}

void EngineeringScreen::onExit() {}

void EngineeringScreen::update(const vehicle::VehicleData& data) {
    // Cache service mode data for fault viewer / module control
    faultBits_    = data.service().faultMask;
    enabledBits_  = data.service().enabledMask;
    disabledBits_ = data.service().disabledMask;

    // Cache pedal calibration telemetry
    if (currentMenu_ == SubMenu::PEDAL_CAL) {
        bool changed = false;
        for (uint8_t i = 0; i < 4; ++i) {
            if (wheelSpeed_[i] != data.speed().raw[i])     changed = true;
            if (motorCurrent_[i] != data.current().raw[i]) changed = true;
            if (tractionScale_[i] != data.traction().scale[i]) changed = true;
            wheelSpeed_[i]    = data.speed().raw[i];
            motorCurrent_[i]  = data.current().raw[i];
            tractionScale_[i] = data.traction().scale[i];
        }
        if (batteryVoltage_ != data.battery().voltageRaw) changed = true;
        if (batteryCurrent_ != data.battery().currentRaw) changed = true;
        if (absActive_ != data.safety().absActive)        changed = true;
        if (tcsActive_ != data.safety().tcsActive)        changed = true;
        batteryVoltage_ = data.battery().voltageRaw;
        batteryCurrent_ = data.battery().currentRaw;
        absActive_      = data.safety().absActive;
        tcsActive_      = data.safety().tcsActive;
        pedalDataChanged_ = changed;
    }

    // Cache encoder calibration telemetry
    if (currentMenu_ == SubMenu::ENCODER_CAL) {
        bool changed = false;
        if (steeringAngle_ != data.steering().angleRaw)    changed = true;
        if (steeringCal_ != data.steering().calibrated)    changed = true;
        steeringAngle_ = data.steering().angleRaw;
        steeringCal_   = data.steering().calibrated;
        encoderDataChanged_ = changed;
    }
}

// -------------------------------------------------------------------------
// Draw
// -------------------------------------------------------------------------
void EngineeringScreen::draw() {
    if (needsRedraw_) {
        needsRedraw_ = false;
        tft.fillScreen(ui::COL_BG);

        switch (currentMenu_) {
            case SubMenu::MAIN:           drawMainMenu();            break;
            case SubMenu::FAULT_VIEWER:   drawFaultViewer();         break;
            case SubMenu::MODULE_CONTROL: drawModuleControl();       break;
            case SubMenu::PEDAL_CAL:      drawPedalCalibration();    break;
            case SubMenu::ENCODER_CAL:    drawEncoderCalibration();  break;
            case SubMenu::SENSOR_MAP_INA: drawSensorMapIna();        break;
            case SubMenu::SENSOR_MAP_TEMP: drawSensorMapTemp();      break;
        }
    }

    // Partial redraw for fault viewer
    if (currentMenu_ == SubMenu::FAULT_VIEWER) {
        if (faultBits_ != prevFaultBits_ ||
            enabledBits_ != prevEnabledBits_ ||
            disabledBits_ != prevDisabledBits_) {
            prevFaultBits_    = faultBits_;
            prevEnabledBits_  = enabledBits_;
            prevDisabledBits_ = disabledBits_;

            char buf[ui::FMT_BUF_LARGE];

            RTRACE_SET_LAYER(2);

            tft.fillRect(200, 100, 240, 16, ui::COL_BG);
            RTRACE_FILL_RECT(200, 100, 240, 16, ui::COL_BG);
            snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)faultBits_);
            tft.setTextColor(faultBits_ ? ui::COL_RED : ui::COL_GREEN, ui::COL_BG);
            tft.setTextSize(1);
            tft.drawString(buf, 200, 100);
            RTRACE_TEXT(200, 100, buf,
                        faultBits_ ? ui::COL_RED : ui::COL_GREEN, ui::COL_BG,
                        1, TL_DATUM);

            tft.fillRect(200, 130, 240, 16, ui::COL_BG);
            RTRACE_FILL_RECT(200, 130, 240, 16, ui::COL_BG);
            snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)enabledBits_);
            tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
            tft.drawString(buf, 200, 130);
            RTRACE_TEXT(200, 130, buf,
                        ui::COL_GREEN, ui::COL_BG, 1, TL_DATUM);

            tft.fillRect(200, 160, 240, 16, ui::COL_BG);
            RTRACE_FILL_RECT(200, 160, 240, 16, ui::COL_BG);
            snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)disabledBits_);
            tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
            tft.drawString(buf, 200, 160);
            RTRACE_TEXT(200, 160, buf,
                        ui::COL_AMBER, ui::COL_BG, 1, TL_DATUM);
        }
    }

    // Partial redraw for pedal calibration (live telemetry values)
    if (currentMenu_ == SubMenu::PEDAL_CAL && pedalDataChanged_) {
        pedalDataChanged_ = false;

        char buf[ui::FMT_BUF_LARGE];
        static const char* const wheelNames[4] = {"FL", "FR", "RL", "RR"};

        RTRACE_SET_LAYER(2);
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        for (uint8_t i = 0; i < 4; ++i) {
            int16_t rowY = 75 + i * 22;

            // Speed value
            tft.fillRect(130, rowY, 80, 16, ui::COL_BG);
            snprintf(buf, sizeof(buf), "%u.%u km/h",
                     wheelSpeed_[i] / 10, wheelSpeed_[i] % 10);
            tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
            tft.drawString(buf, 130, rowY);

            // Current value
            tft.fillRect(250, rowY, 80, 16, ui::COL_BG);
            snprintf(buf, sizeof(buf), "%u.%02u A",
                     motorCurrent_[i] / 100, motorCurrent_[i] % 100);
            // Current color: green < 15A, yellow 15–25A, red > 25A (BTS7960 safe limits)
            uint16_t curCol = (motorCurrent_[i] > 2500) ? ui::COL_RED :
                              (motorCurrent_[i] > 1500) ? ui::COL_YELLOW : ui::COL_GREEN;
            tft.setTextColor(curCol, ui::COL_BG);
            tft.drawString(buf, 250, rowY);

            // Traction scale
            tft.fillRect(370, rowY, 60, 16, ui::COL_BG);
            snprintf(buf, sizeof(buf), "%u%%", tractionScale_[i]);
            tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
            tft.drawString(buf, 370, rowY);
        }

        // Battery row
        int16_t batY = 170;
        tft.fillRect(130, batY, 160, 16, ui::COL_BG);
        snprintf(buf, sizeof(buf), "%u.%02u V  %u.%02u A",
                 batteryVoltage_ / 100, batteryVoltage_ % 100,
                 batteryCurrent_ / 100, batteryCurrent_ % 100);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString(buf, 130, batY);

        // ABS/TCS row
        int16_t safeY = 192;
        tft.fillRect(130, safeY, 200, 16, ui::COL_BG);
        snprintf(buf, sizeof(buf), "ABS:%s  TCS:%s",
                 absActive_ ? "ON" : "off", tcsActive_ ? "ON" : "off");
        tft.setTextColor(
            (absActive_ || tcsActive_) ? ui::COL_AMBER : ui::COL_GREEN,
            ui::COL_BG);
        tft.drawString(buf, 130, safeY);
    }

    // Partial redraw for encoder calibration (live steering angle)
    if (currentMenu_ == SubMenu::ENCODER_CAL && encoderDataChanged_) {
        encoderDataChanged_ = false;

        char buf[ui::FMT_BUF_LARGE];

        RTRACE_SET_LAYER(2);
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        // Steering angle
        tft.fillRect(180, 80, 160, 16, ui::COL_BG);
        int16_t absAngle = steeringAngle_ < 0 ? -steeringAngle_ : steeringAngle_;
        snprintf(buf, sizeof(buf), "%s%d.%d deg",
                 steeringAngle_ < 0 ? "-" : "+",
                 absAngle / 10, absAngle % 10);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString(buf, 180, 80);

        // Calibration status
        tft.fillRect(180, 100, 160, 16, ui::COL_BG);
        if (steeringCal_) {
            tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
            tft.drawString("CALIBRATED", 180, 100);
        } else {
            tft.setTextColor(ui::COL_RED, ui::COL_BG);
            tft.drawString("NOT CALIBRATED", 180, 100);
        }

        // Visual angle bar (centered gauge)
        int16_t gaugeY = 140;
        int16_t gaugeW = 300;
        int16_t gaugeH = 20;
        int16_t gaugeX = (ui::SCREEN_W - gaugeW) / 2;

        tft.fillRect(gaugeX, gaugeY, gaugeW, gaugeH, ui::COL_DARK_GRAY);
        // Center line
        int16_t centerX = gaugeX + gaugeW / 2;
        tft.drawFastVLine(centerX, gaugeY, gaugeH, ui::COL_GRAY);

        // Angle indicator (clamp to ±45°, scale to bar width)
        int16_t clampedAngle = steeringAngle_;
        if (clampedAngle > 450)  clampedAngle = 450;
        if (clampedAngle < -450) clampedAngle = -450;
        int16_t indicatorX = centerX + (int32_t)clampedAngle * gaugeW / 900;
        tft.fillRect(indicatorX - 3, gaugeY + 2, 7, gaugeH - 4, ui::COL_AMBER);
    }

    RTRACE_DUMP_IF_PENDING();
}

// -------------------------------------------------------------------------
// Touch handling
// -------------------------------------------------------------------------
bool EngineeringScreen::handleTouch(int16_t x, int16_t y) {
    // Back button (submenus → main) or Exit button (main → normal screens)
    if (x >= BACK_X && x <= BACK_X + BACK_W &&
        y >= BACK_Y && y <= BACK_Y + BACK_H) {
        if (currentMenu_ != SubMenu::MAIN) {
            currentMenu_ = SubMenu::MAIN;
            needsRedraw_ = true;
        } else {
            exitRequested_ = true;
        }
        return true;
    }

    // Save button (sensor mapping submenus only)
    if ((currentMenu_ == SubMenu::SENSOR_MAP_INA ||
         currentMenu_ == SubMenu::SENSOR_MAP_TEMP) &&
        x >= SAVE_X && x <= SAVE_X + SAVE_W &&
        y >= SAVE_Y && y <= SAVE_Y + SAVE_H) {
        if (currentMenu_ == SubMenu::SENSOR_MAP_INA) {
            config_store::setIna226Map(inaMap_);
            Serial.println("[ENG] INA226 map saved");
        } else {
            config_store::setTempSensorMap(tempMap_);
            Serial.println("[ENG] Temp sensor map saved");
        }
        config_store::flush();
        // Return to main
        currentMenu_ = SubMenu::MAIN;
        needsRedraw_ = true;
        return true;
    }

    // Main menu item selection
    if (currentMenu_ == SubMenu::MAIN) {
        if (x >= MENU_X && x <= MENU_X + MENU_W) {
            for (int i = 0; i < NUM_MAIN_ITEMS; ++i) {
                int16_t btnY = MENU_START_Y + i * MENU_SPACING;
                if (y >= btnY && y <= btnY + MENU_BTN_H) {
                    switch (i) {
                        case 0: currentMenu_ = SubMenu::FAULT_VIEWER;   break;
                        case 1:
                            moduleCtrlPage_ = 0;
                            currentMenu_ = SubMenu::MODULE_CONTROL;
                            break;
                        case 2: currentMenu_ = SubMenu::PEDAL_CAL;      break;
                        case 3: currentMenu_ = SubMenu::ENCODER_CAL;    break;
                        case 4:
                            // Load fresh copy before entering INA mapping
                            memcpy(inaMap_, config_store::get().ina226Map,
                                   config_store::NUM_INA226_CH);
                            inaEditRow_ = 0;
                            currentMenu_ = SubMenu::SENSOR_MAP_INA;
                            break;
                        case 5:
                            // Load fresh copy before entering temp mapping
                            memcpy(tempMap_, config_store::get().tempSensorMap,
                                   config_store::NUM_TEMP_SENS);
                            tempEditRow_ = 0;
                            currentMenu_ = SubMenu::SENSOR_MAP_TEMP;
                            break;
                        case 6: {
                            // Factory restore — send SERVICE_CMD 0x110
                            CanFrame frame = {};
                            frame.identifier       = can::SERVICE_CMD;
                            frame.extd             = 0;
                            frame.data_length_code = 2;
                            frame.data[0]          = can::SERVICE_ACTION_FACTORY_RESTORE;
                            frame.data[1]          = 0;
                            ESP32Can.writeFrame(frame);
                            Serial.println("[ENG] Factory restore sent");
                            break;
                        }
                    }
                    needsRedraw_ = true;
                    return true;
                }
            }
        }
        return false;
    }

    // Module control: tap a row to toggle enable/disable, or page button
    if (currentMenu_ == SubMenu::MODULE_CONTROL) {
        // PAGE button (cycle through pages)
        if (x >= PAGE_BTN_X && x <= PAGE_BTN_X + PAGE_BTN_W &&
            y >= PAGE_BTN_Y && y <= PAGE_BTN_Y + PAGE_BTN_H) {
            moduleCtrlPage_ = (moduleCtrlPage_ + 1) % MODULE_CTRL_PAGES;
            needsRedraw_ = true;
            return true;
        }
        // Module rows
        uint8_t startIdx = moduleCtrlPage_ * MOD_ROWS_PER_PAGE;
        uint8_t endIdx   = startIdx + MOD_ROWS_PER_PAGE;
        if (endIdx > MOD_TOTAL_COUNT) endIdx = MOD_TOTAL_COUNT;

        for (uint8_t i = startIdx; i < endIdx; ++i) {
            uint8_t rowOff = i - startIdx;
            int16_t rowY = MOD_ROW_Y0 + rowOff * MOD_ROW_SPC;
            if (x >= MOD_ROW_X && x <= MOD_ROW_X + MOD_ROW_W &&
                y >= rowY && y <= rowY + MOD_ROW_H) {
                // Only toggle non-critical modules (IDs >= FIRST_NON_CRITICAL)
                if (i >= FIRST_NON_CRITICAL) {
                    bool currentlyEnabled = (enabledBits_ >> i) & 1U;
                    uint8_t action = currentlyEnabled
                        ? can::SERVICE_ACTION_DISABLE
                        : can::SERVICE_ACTION_ENABLE;
                    CanFrame frame = {};
                    frame.identifier       = can::SERVICE_CMD;
                    frame.extd             = 0;
                    frame.data_length_code = 2;
                    frame.data[0]          = action;
                    frame.data[1]          = i;  // module ID
                    ESP32Can.writeFrame(frame);
                    Serial.printf("[ENG] Module %u %s\n", i,
                                  currentlyEnabled ? "DISABLE" : "ENABLE");
                    needsRedraw_ = true;
                }
                return true;
            }
        }
        return false;
    }

    // INA226 mapping: tap a row to cycle its position assignment
    if (currentMenu_ == SubMenu::SENSOR_MAP_INA) {
        for (uint8_t i = 0; i < config_store::NUM_INA226_CH; ++i) {
            int16_t rowY = MAP_ROW_Y0 + i * MAP_ROW_SPC;
            if (x >= MAP_ROW_X && x <= MAP_ROW_X + MAP_ROW_W &&
                y >= rowY && y <= rowY + MAP_ROW_H) {
                // Cycle position: increment mod NUM_INA226_CH
                inaMap_[i] = (inaMap_[i] + 1) % config_store::NUM_INA226_CH;
                needsRedraw_ = true;
                return true;
            }
        }
        return false;
    }

    // Temp sensor mapping: tap a row to cycle its position assignment
    if (currentMenu_ == SubMenu::SENSOR_MAP_TEMP) {
        for (uint8_t i = 0; i < config_store::NUM_TEMP_SENS; ++i) {
            int16_t rowY = MAP_ROW_Y0 + i * MAP_ROW_SPC;
            if (x >= MAP_ROW_X && x <= MAP_ROW_X + MAP_ROW_W &&
                y >= rowY && y <= rowY + MAP_ROW_H) {
                // Cycle position: increment mod NUM_TEMP_SENS
                tempMap_[i] = (tempMap_[i] + 1) % config_store::NUM_TEMP_SENS;
                needsRedraw_ = true;
                return true;
            }
        }
        return false;
    }

    return false;
}

// -------------------------------------------------------------------------
// Draw helpers
// -------------------------------------------------------------------------
void EngineeringScreen::drawMainMenu() {
    RTRACE_BEGIN_SCREEN("eng_main");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("ENGINEERING", ui::SCREEN_W / 2, 22);
    RTRACE_TEXT(ui::SCREEN_W / 2, 22, "ENGINEERING",
                ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);
    tft.setTextDatum(TL_DATUM);

    // Menu buttons
    for (int i = 0; i < NUM_MAIN_ITEMS; ++i) {
        int16_t btnY = MENU_START_Y + i * MENU_SPACING;
        uint16_t bgCol = (i == 6) ? ui::COL_RED : ui::COL_DARK_GRAY;
        uint16_t txtCol = ui::COL_WHITE;

        tft.fillRect(MENU_X, btnY, MENU_W, MENU_BTN_H, bgCol);
        RTRACE_FILL_RECT(MENU_X, btnY, MENU_W, MENU_BTN_H, bgCol);
        tft.drawRect(MENU_X, btnY, MENU_W, MENU_BTN_H, ui::COL_GRAY);
        RTRACE_DRAW_RECT(MENU_X, btnY, MENU_W, MENU_BTN_H, ui::COL_GRAY);

        tft.setTextColor(txtCol, bgCol);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(mainLabels[i], MENU_X + MENU_W / 2,
                        btnY + MENU_BTN_H / 2);
        RTRACE_TEXT(MENU_X + MENU_W / 2, btnY + MENU_BTN_H / 2, mainLabels[i],
                    txtCol, bgCol, 1, MC_DATUM);
    }
    tft.setTextDatum(TL_DATUM);

    // EXIT button (bottom-left — returns to normal screens)
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    RTRACE_FILL_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_AMBER);
    RTRACE_DRAW_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_AMBER);
    tft.setTextColor(ui::COL_AMBER, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("EXIT", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    RTRACE_TEXT(BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2, "EXIT",
                ui::COL_AMBER, ui::COL_DARK_GRAY, 1, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
}

void EngineeringScreen::drawFaultViewer() {
    RTRACE_BEGIN_SCREEN("eng_fault_viewer");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("FAULT VIEWER", ui::SCREEN_W / 2, 25);
    RTRACE_TEXT(ui::SCREEN_W / 2, 25, "FAULT VIEWER",
                ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    // Labels
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Fault Bits:", 40, 100);
    RTRACE_TEXT(40, 100, "Fault Bits:", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("Enabled Bits:", 40, 130);
    RTRACE_TEXT(40, 130, "Enabled Bits:", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("Disabled Bits:", 40, 160);
    RTRACE_TEXT(40, 160, "Disabled Bits:", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    // Force redraw of values
    prevFaultBits_ = faultBits_ + 1;

    // Back button
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    RTRACE_FILL_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    RTRACE_DRAW_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    RTRACE_TEXT(BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2, "BACK",
                ui::COL_WHITE, ui::COL_DARK_GRAY, 1, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
}

void EngineeringScreen::drawModuleControl() {
    RTRACE_BEGIN_SCREEN("eng_module_ctrl");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    char hdrBuf[ui::FMT_BUF_LARGE];
    snprintf(hdrBuf, sizeof(hdrBuf), "MODULE CONTROL (%u/%u)",
             moduleCtrlPage_ + 1, MODULE_CTRL_PAGES);
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(hdrBuf, ui::SCREEN_W / 2, 15);
    RTRACE_TEXT(ui::SCREEN_W / 2, 15, hdrBuf,
                ui::COL_AMBER, ui::COL_BG, 1, MC_DATUM);

    // Column headers
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("ID", MOD_ROW_X + 4, 32);
    tft.drawString("Module", MOD_ROW_X + 30, 32);
    tft.drawString("Status (tap to toggle)", MOD_ROW_X + 280, 32);

    // Module rows for current page
    uint8_t startIdx = moduleCtrlPage_ * MOD_ROWS_PER_PAGE;
    uint8_t endIdx   = startIdx + MOD_ROWS_PER_PAGE;
    if (endIdx > MOD_TOTAL_COUNT) endIdx = MOD_TOTAL_COUNT;

    char buf[ui::FMT_BUF_LARGE];

    for (uint8_t i = startIdx; i < endIdx; ++i) {
        uint8_t rowOff = i - startIdx;
        int16_t rowY = MOD_ROW_Y0 + rowOff * MOD_ROW_SPC;

        bool isCritical = (i < FIRST_NON_CRITICAL);
        bool isEnabled  = (enabledBits_ >> i) & 1U;
        bool isFaulted  = (faultBits_  >> i) & 1U;
        bool isDisabled = (disabledBits_ >> i) & 1U;

        // Row background
        uint16_t bgCol = isCritical ? ui::COL_DARK_GRAY : ui::COL_BG;
        tft.fillRect(MOD_ROW_X, rowY, MOD_ROW_W, MOD_ROW_H, bgCol);
        tft.drawRect(MOD_ROW_X, rowY, MOD_ROW_W, MOD_ROW_H, ui::COL_GRAY);

        // Module ID
        snprintf(buf, sizeof(buf), "%2u", i);
        tft.setTextColor(ui::COL_GRAY, bgCol);
        tft.setTextDatum(ML_DATUM);
        tft.drawString(buf, MOD_ROW_X + 4, rowY + MOD_ROW_H / 2);

        // Module name
        tft.setTextColor(ui::COL_WHITE, bgCol);
        tft.drawString(MODULE_NAMES[i], MOD_ROW_X + 30, rowY + MOD_ROW_H / 2);

        // Status indicator
        const char* statusText;
        uint16_t statusCol;
        if (isCritical) {
            statusText = "CRITICAL";
            statusCol  = ui::COL_AMBER;
        } else if (isFaulted && isDisabled) {
            statusText = "FAULT+DIS";
            statusCol  = ui::COL_ORANGE;
        } else if (isFaulted) {
            statusText = "FAULT";
            statusCol  = ui::COL_RED;
        } else if (isDisabled) {
            statusText = "DISABLED";
            statusCol  = ui::COL_GRAY;
        } else if (isEnabled) {
            statusText = "ENABLED";
            statusCol  = ui::COL_GREEN;
        } else {
            statusText = "OFF";
            statusCol  = ui::COL_GRAY;
        }

        // Status dot
        int16_t dotX = MOD_ROW_X + 280;
        tft.fillCircle(dotX, rowY + MOD_ROW_H / 2, 4, statusCol);

        // Status text
        tft.setTextColor(statusCol, bgCol);
        tft.drawString(statusText, dotX + 10, rowY + MOD_ROW_H / 2);
    }

    tft.setTextDatum(TL_DATUM);

    // Back button
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    RTRACE_FILL_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    RTRACE_DRAW_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    RTRACE_TEXT(BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2, "BACK",
                ui::COL_WHITE, ui::COL_DARK_GRAY, 1, MC_DATUM);

    // PAGE button
    tft.fillRect(PAGE_BTN_X, PAGE_BTN_Y, PAGE_BTN_W, PAGE_BTN_H, ui::COL_DARK_GRAY);
    RTRACE_FILL_RECT(PAGE_BTN_X, PAGE_BTN_Y, PAGE_BTN_W, PAGE_BTN_H, ui::COL_DARK_GRAY);
    tft.drawRect(PAGE_BTN_X, PAGE_BTN_Y, PAGE_BTN_W, PAGE_BTN_H, ui::COL_CYAN);
    RTRACE_DRAW_RECT(PAGE_BTN_X, PAGE_BTN_Y, PAGE_BTN_W, PAGE_BTN_H, ui::COL_CYAN);
    tft.setTextColor(ui::COL_CYAN, ui::COL_DARK_GRAY);
    tft.drawString("PAGE >>", PAGE_BTN_X + PAGE_BTN_W / 2, PAGE_BTN_Y + PAGE_BTN_H / 2);
    RTRACE_TEXT(PAGE_BTN_X + PAGE_BTN_W / 2, PAGE_BTN_Y + PAGE_BTN_H / 2, "PAGE >>",
                ui::COL_CYAN, ui::COL_DARK_GRAY, 1, MC_DATUM);

    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// drawPedalCalibration — Live pedal response verification
//
// Shows real-time wheel speeds, motor currents, traction scale, battery
// data, and ABS/TCS status.  Updated via partial redraw in draw().
// -------------------------------------------------------------------------
void EngineeringScreen::drawPedalCalibration() {
    RTRACE_BEGIN_SCREEN("eng_pedal_cal");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("PEDAL CALIBRATION", ui::SCREEN_W / 2, 15);
    RTRACE_TEXT(ui::SCREEN_W / 2, 15, "PEDAL CALIBRATION",
                ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    // Column headers
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Wheel", 40, 55);
    RTRACE_TEXT(40, 55, "Wheel", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("Speed", 130, 55);
    RTRACE_TEXT(130, 55, "Speed", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("Current", 250, 55);
    RTRACE_TEXT(250, 55, "Current", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("Traction", 370, 55);
    RTRACE_TEXT(370, 55, "Traction", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    // Wheel labels (static)
    static const char* const wheelNames[4] = {"FL", "FR", "RL", "RR"};
    for (uint8_t i = 0; i < 4; ++i) {
        int16_t rowY = 75 + i * 22;
        tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        tft.drawString(wheelNames[i], 40, rowY);
        RTRACE_TEXT(40, rowY, wheelNames[i], ui::COL_CYAN, ui::COL_BG, 1, TL_DATUM);
    }

    // Battery label
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Battery:", 40, 170);
    RTRACE_TEXT(40, 170, "Battery:", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    // Safety label
    tft.drawString("Safety:", 40, 192);
    RTRACE_TEXT(40, 192, "Safety:", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    // Instruction text
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Operate pedal to verify motor response.", 40, 230);
    RTRACE_TEXT(40, 230, "Operate pedal to verify motor response.",
                ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("Values update in real-time from CAN telemetry.", 40, 248);
    RTRACE_TEXT(40, 248, "Values update in real-time from CAN telemetry.",
                ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    // Force initial partial redraw of values
    pedalDataChanged_ = true;

    // Back button
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    RTRACE_FILL_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    RTRACE_DRAW_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    RTRACE_TEXT(BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2, "BACK",
                ui::COL_WHITE, ui::COL_DARK_GRAY, 1, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// drawEncoderCalibration — Live steering encoder verification
//
// Shows real-time steering angle, calibration status, and a visual
// angle gauge bar.  Updated via partial redraw in draw().
// -------------------------------------------------------------------------
void EngineeringScreen::drawEncoderCalibration() {
    RTRACE_BEGIN_SCREEN("eng_encoder_cal");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("ENCODER CALIBRATION", ui::SCREEN_W / 2, 15);
    RTRACE_TEXT(ui::SCREEN_W / 2, 15, "ENCODER CALIBRATION",
                ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    // Static labels
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Steering Angle:", 40, 80);
    RTRACE_TEXT(40, 80, "Steering Angle:", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("Calibration:", 40, 100);
    RTRACE_TEXT(40, 100, "Calibration:", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    // Gauge border
    int16_t gaugeY = 140;
    int16_t gaugeW = 300;
    int16_t gaugeH = 20;
    int16_t gaugeX = (ui::SCREEN_W - gaugeW) / 2;
    tft.drawRect(gaugeX - 1, gaugeY - 1, gaugeW + 2, gaugeH + 2, ui::COL_GRAY);
    RTRACE_DRAW_RECT(gaugeX - 1, gaugeY - 1, gaugeW + 2, gaugeH + 2, ui::COL_GRAY);

    // Scale labels
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("-45", gaugeX, gaugeY + gaugeH + 12);
    tft.drawString("0", gaugeX + gaugeW / 2, gaugeY + gaugeH + 12);
    tft.drawString("+45", gaugeX + gaugeW, gaugeY + gaugeH + 12);
    RTRACE_TEXT(gaugeX, gaugeY + gaugeH + 12, "-45", ui::COL_GRAY, ui::COL_BG, 1, MC_DATUM);
    RTRACE_TEXT(gaugeX + gaugeW / 2, gaugeY + gaugeH + 12, "0", ui::COL_GRAY, ui::COL_BG, 1, MC_DATUM);
    RTRACE_TEXT(gaugeX + gaugeW, gaugeY + gaugeH + 12, "+45", ui::COL_GRAY, ui::COL_BG, 1, MC_DATUM);
    tft.setTextDatum(TL_DATUM);

    // Instructions
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Turn steering to verify encoder response.", 40, 200);
    RTRACE_TEXT(40, 200, "Turn steering to verify encoder response.",
                ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("Gauge range: -45 to +45 degrees.", 40, 218);
    RTRACE_TEXT(40, 218, "Gauge range: -45 to +45 degrees.",
                ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("Center line = 0 degrees (straight ahead).", 40, 236);
    RTRACE_TEXT(40, 236, "Center line = 0 degrees (straight ahead).",
                ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    // Force initial partial redraw of values
    encoderDataChanged_ = true;

    // Back button
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    RTRACE_FILL_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    RTRACE_DRAW_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    RTRACE_TEXT(BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2, "BACK",
                ui::COL_WHITE, ui::COL_DARK_GRAY, 1, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// drawSensorMapIna — INA226 channel-to-position mapping editor
//
// Displays a table: each row = one TCA9548A channel (0-5).
// Column A = channel name (fixed), Column B = assigned position (editable).
// Tap a row to cycle the position through all 6 options.
// Tap SAVE to persist. Tap BACK to discard changes.
// -------------------------------------------------------------------------
void EngineeringScreen::drawSensorMapIna() {
    RTRACE_BEGIN_SCREEN("eng_ina_map");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("INA226 CHANNEL MAPPING", ui::SCREEN_W / 2, 15);
    RTRACE_TEXT(ui::SCREEN_W / 2, 15, "INA226 CHANNEL MAPPING",
                ui::COL_AMBER, ui::COL_BG, 1, MC_DATUM);

    // Column headers
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("Channel", MAP_ROW_X + 4, 30);
    tft.drawString("Assigned Position  (tap to change)", MAP_ROW_X + 90, 30);
    RTRACE_TEXT(MAP_ROW_X + 4, 30, "Channel", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    char buf[ui::FMT_BUF_LARGE];

    for (uint8_t i = 0; i < config_store::NUM_INA226_CH; ++i) {
        int16_t rowY = MAP_ROW_Y0 + i * MAP_ROW_SPC;

        // Row background
        tft.fillRect(MAP_ROW_X, rowY, MAP_ROW_W, MAP_ROW_H, ui::COL_DARK_GRAY);
        tft.drawRect(MAP_ROW_X, rowY, MAP_ROW_W, MAP_ROW_H, ui::COL_GRAY);

        // Channel name (left column)
        tft.setTextColor(ui::COL_CYAN, ui::COL_DARK_GRAY);
        tft.setTextDatum(ML_DATUM);
        tft.drawString(INA_CHAN_NAMES[i], MAP_ROW_X + 4, rowY + MAP_ROW_H / 2);

        // Assigned position (right column)
        uint8_t posIdx = inaMap_[i];
        if (posIdx >= config_store::NUM_INA226_CH) posIdx = 0;
        snprintf(buf, sizeof(buf), "→ %s", INA_POS_NAMES[posIdx]);
        tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
        tft.drawString(buf, MAP_ROW_X + 90, rowY + MAP_ROW_H / 2);
    }

    tft.setTextDatum(TL_DATUM);

    // Back button
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    RTRACE_FILL_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    RTRACE_DRAW_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    RTRACE_TEXT(BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2, "BACK",
                ui::COL_WHITE, ui::COL_DARK_GRAY, 1, MC_DATUM);

    // Save button
    tft.fillRect(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_GREEN);
    RTRACE_FILL_RECT(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_GREEN);
    tft.drawRect(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_WHITE);
    RTRACE_DRAW_RECT(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_WHITE);
    tft.setTextColor(ui::COL_BLACK, ui::COL_GREEN);
    tft.drawString("SAVE", SAVE_X + SAVE_W / 2, SAVE_Y + SAVE_H / 2);
    RTRACE_TEXT(SAVE_X + SAVE_W / 2, SAVE_Y + SAVE_H / 2, "SAVE",
                ui::COL_BLACK, ui::COL_GREEN, 1, MC_DATUM);

    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// drawSensorMapTemp — DS18B20 sensor-to-position mapping editor
//
// Displays a table: each row = one discovered DS18B20 index (0-4).
// Column A = sensor index (fixed), Column B = assigned position (editable).
// Tap a row to cycle the position through all 5 options.
// Tap SAVE to persist. Tap BACK to discard changes.
// -------------------------------------------------------------------------
void EngineeringScreen::drawSensorMapTemp() {
    RTRACE_BEGIN_SCREEN("eng_temp_map");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("DS18B20 TEMP SENSOR MAPPING", ui::SCREEN_W / 2, 15);
    RTRACE_TEXT(ui::SCREEN_W / 2, 15, "DS18B20 TEMP SENSOR MAPPING",
                ui::COL_AMBER, ui::COL_BG, 1, MC_DATUM);

    // Column headers
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("Sensor", MAP_ROW_X + 4, 30);
    tft.drawString("Assigned Position  (tap to change)", MAP_ROW_X + 90, 30);
    RTRACE_TEXT(MAP_ROW_X + 4, 30, "Sensor", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    char buf[ui::FMT_BUF_LARGE];

    for (uint8_t i = 0; i < config_store::NUM_TEMP_SENS; ++i) {
        int16_t rowY = MAP_ROW_Y0 + i * MAP_ROW_SPC;

        // Row background
        tft.fillRect(MAP_ROW_X, rowY, MAP_ROW_W, MAP_ROW_H, ui::COL_DARK_GRAY);
        tft.drawRect(MAP_ROW_X, rowY, MAP_ROW_W, MAP_ROW_H, ui::COL_GRAY);

        // Sensor index (left column)
        tft.setTextColor(ui::COL_CYAN, ui::COL_DARK_GRAY);
        tft.setTextDatum(ML_DATUM);
        tft.drawString(TEMP_IDX_NAMES[i], MAP_ROW_X + 4, rowY + MAP_ROW_H / 2);

        // Assigned position (right column)
        uint8_t posIdx = tempMap_[i];
        if (posIdx >= config_store::NUM_TEMP_SENS) posIdx = 0;
        snprintf(buf, sizeof(buf), "→ %s", TEMP_POS_NAMES[posIdx]);
        tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
        tft.drawString(buf, MAP_ROW_X + 90, rowY + MAP_ROW_H / 2);
    }

    tft.setTextDatum(TL_DATUM);

    // Back button
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    RTRACE_FILL_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    RTRACE_DRAW_RECT(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    RTRACE_TEXT(BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2, "BACK",
                ui::COL_WHITE, ui::COL_DARK_GRAY, 1, MC_DATUM);

    // Save button
    tft.fillRect(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_GREEN);
    RTRACE_FILL_RECT(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_GREEN);
    tft.drawRect(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_WHITE);
    RTRACE_DRAW_RECT(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_WHITE);
    tft.setTextColor(ui::COL_BLACK, ui::COL_GREEN);
    tft.drawString("SAVE", SAVE_X + SAVE_W / 2, SAVE_Y + SAVE_H / 2);
    RTRACE_TEXT(SAVE_X + SAVE_W / 2, SAVE_Y + SAVE_H / 2, "SAVE",
                ui::COL_BLACK, ui::COL_GREEN, 1, MC_DATUM);

    tft.setTextDatum(TL_DATUM);
}
