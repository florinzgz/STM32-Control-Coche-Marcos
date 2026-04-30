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
static constexpr int16_t MENU_BTN_H   = 26;
static constexpr int16_t MENU_START_Y = 50;
static constexpr int16_t MENU_SPACING = 28;

static constexpr int     NUM_MAIN_ITEMS = 10;
static const char* const mainLabels[NUM_MAIN_ITEMS] = {
    "FAULT VIEWER",
    "MODULE ENABLE/DISABLE",
    "PEDAL CALIBRATION",
    "ENCODER CALIBRATION",
    "INA226 SENSOR MAPPING",
    "TEMP SENSOR MAPPING",
    "FACTORY DEFAULTS",
    "DTC ERROR LOG",
    "MAINTENANCE",
    "RELAY CONTROL (DEBUG)"
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

// Display helper literals — UTF-8 degree symbol and arrow for sensor map rows.
// TFT_eSPI's built-in font renders byte 0x1A as a right-pointing arrow glyph;
// this is a font-specific glyph, NOT the ASCII control character SUB.
static constexpr const char TEMP_DEGREE_STR[] = "\xC2\xB0" "C";   // "°C" in UTF-8
static constexpr const char TEMP_ARROW_STR[]  = "\x1A ";           // TFT_eSPI glyph 0x1A = right arrow

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

// ACK feedback timeout (ms) — how long the status message is shown
static constexpr uint32_t ACK_FEEDBACK_TIMEOUT_MS = 2000;

// NEXT/PREV button layout
static constexpr int16_t PAGE_BTN_X  = 200;
static constexpr int16_t PAGE_BTN_Y  = 280;
static constexpr int16_t PAGE_BTN_W  = 80;
static constexpr int16_t PAGE_BTN_H  = 30;

// ---- Factory Defaults submenu ----
static constexpr int     NUM_FACTORY_ITEMS = 6;
static const char* const factoryLabels[NUM_FACTORY_ITEMS] = {
    "RESET STEERING PID",
    "RESET WHEEL SENSORS",
    "RESET INA226 / SHUNTS",
    "RESET TRACTION MOTOR FORCE",
    "RESET STEERING MOTOR FORCE",
    "RESET ALL (FACTORY RESTORE)"
};
// CAN service action codes matching each factory default option
static const uint8_t factoryActions[NUM_FACTORY_ITEMS] = {
    can::SERVICE_ACTION_RESET_STEERING_PID,
    can::SERVICE_ACTION_RESET_WHEEL_SENSORS,
    can::SERVICE_ACTION_RESET_INA226_SHUNTS,
    can::SERVICE_ACTION_RESET_TRACTION_FORCE,
    can::SERVICE_ACTION_RESET_STEERING_FORCE,
    can::SERVICE_ACTION_FACTORY_RESTORE
};

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
    clearLogPending_ = false;   // §5: reset confirmation state on screen enter
    inaEditRow_  = 0;
    tempEditRow_ = 0;
    moduleCtrlPage_ = 0;
    prevFaultBits_    = 0xFFFFFFFF;
    prevEnabledBits_  = 0xFFFFFFFF;
    prevDisabledBits_ = 0xFFFFFFFF;
    pedalDataChanged_   = false;
    encoderDataChanged_ = false;
    lastAckResult_      = 0;
    lastAckMs_          = 0;
    lastAckTracked_     = 0;

    // Load current sensor mappings from config into working copies
    const auto& cfg = config_store::get();
    memcpy(inaMap_,  cfg.ina226Map,      config_store::NUM_INA226_CH);
    memcpy(tempMap_, cfg.tempSensorMap,  config_store::NUM_TEMP_SENS);

    // Reset relay override UI state on screen entry.
    // Send a disable command to ensure STM32 state is in sync in case
    // the override was left active from a previous session.
    relayOverrideEnabled_ = false;
    relayOverrideMask_    = 0;
    relayOverrideChanged_ = false;
    {
        CanFrame frame = {};
        frame.identifier       = can::SERVICE_CMD;
        frame.extd             = 0;
        frame.data_length_code = 2;
        frame.data[0]          = can::SERVICE_ACTION_RELAY_OVERRIDE;
        frame.data[1]          = 0x00;  // disable override
        ESP32Can.writeFrame(frame, 0);
    }
}

void EngineeringScreen::onExit() {
    // Auto-disable relay override when leaving engineering screen.
    // Sends CAN command to STM32 to turn off override + all relay GPIOs.
    if (relayOverrideEnabled_) {
        CanFrame frame = {};
        frame.identifier       = can::SERVICE_CMD;
        frame.extd             = 0;
        frame.data_length_code = 2;
        frame.data[0]          = can::SERVICE_ACTION_RELAY_OVERRIDE;
        frame.data[1]          = 0x00;  // enable=0, mask=0 → disable all
        ESP32Can.writeFrame(frame, 0);
        relayOverrideEnabled_ = false;
        relayOverrideMask_    = 0;
        Serial.println("[ENG] Relay override auto-disabled on exit");
    }
}

void EngineeringScreen::update(const vehicle::VehicleData& data, unsigned long frameTimeMs) {
    // Cache service mode data for fault viewer / module control
    faultBits_    = data.service().faultMask;
    enabledBits_  = data.service().enabledMask;
    disabledBits_ = data.service().disabledMask;

    // Relay status (shown on main menu header)
    relayStatus_ = data.heartbeat().relayStatus;

    // Relay override: sync UI state with real CAN telemetry.
    // If the system state has moved away from STANDBY, the STM32 has
    // auto-disabled the override — reflect this in the UI.
    if (relayOverrideEnabled_) {
        if (data.heartbeat().systemState != can::SystemState::STANDBY) {
            relayOverrideEnabled_ = false;
            relayOverrideMask_    = 0;
            relayOverrideChanged_ = true;
        }
    }
    // Detect relay status changes while in relay control submenu.
    // Compare against the raw relayStatus_ value to catch any change.
    if (currentMenu_ == SubMenu::RELAY_CONTROL) {
        static uint8_t prevRelayCtrlStatus = 0xFF;
        if (relayStatus_ != prevRelayCtrlStatus) {
            prevRelayCtrlStatus = relayStatus_;
            relayOverrideChanged_ = true;
        }
    }

    // Track SERVICE_CMD ACK responses (cmdIdLow = 0x10)
    if (currentMenu_ == SubMenu::MODULE_CONTROL) {
        const auto& ad = data.ack();
        if (ad.cmdIdLow == 0x10 && ad.timestampMs != lastAckTracked_) {
            lastAckTracked_ = ad.timestampMs;
            if (ad.result == can::AckResult::OK) {
                lastAckResult_ = 1;
            } else if (ad.result == can::AckResult::REJECTED) {
                lastAckResult_ = 2;
            } else if (ad.result == can::AckResult::BLOCKED_BY_SAFETY) {
                lastAckResult_ = 3;
            } else {
                lastAckResult_ = 2;  // treat INVALID as rejected
            }
            lastAckMs_ = frameTimeMs;
            needsRedraw_ = true;
        }
        // Expire feedback message after 2 seconds
        if (lastAckResult_ != 0 && (frameTimeMs - lastAckMs_) > ACK_FEEDBACK_TIMEOUT_MS) {
            lastAckResult_ = 0;
            needsRedraw_ = true;
        }
    }

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

    // Cache raw physical-index temperatures for the temp sensor mapping editor.
    // data.temp() is populated from STATUS_TEMP (0x202) which sends sensors in
    // discovery order — exactly the physical indices shown in the mapping table.
    if (currentMenu_ == SubMenu::SENSOR_MAP_TEMP) {
        bool changed = false;
        for (uint8_t i = 0; i < config_store::NUM_TEMP_SENS; ++i) {
            int8_t t = data.temp().temps[i];
            if (rawTempC_[i] != t) {
                rawTempC_[i] = t;
                changed = true;
            }
        }
        if (changed) needsRedraw_ = true;
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
            case SubMenu::MAIN:             drawMainMenu();            break;
            case SubMenu::FAULT_VIEWER:     drawFaultViewer();         break;
            case SubMenu::MODULE_CONTROL:   drawModuleControl();       break;
            case SubMenu::PEDAL_CAL:        drawPedalCalibration();    break;
            case SubMenu::ENCODER_CAL:      drawEncoderCalibration();  break;
            case SubMenu::SENSOR_MAP_INA:   drawSensorMapIna();        break;
            case SubMenu::SENSOR_MAP_TEMP:  drawSensorMapTemp();       break;
            case SubMenu::FACTORY_DEFAULTS: drawFactoryDefaults();     break;
            case SubMenu::DTC_LOG_VIEWER:   drawDtcLogViewer();        break;
            case SubMenu::MAINTENANCE:      drawMaintenance();         break;
            case SubMenu::RELAY_CONTROL:    drawRelayControl();        break;
        }
    }

    // Partial redraw for main menu relay status panel
    if (currentMenu_ == SubMenu::MAIN && relayStatus_ != prevRelayStatus_) {
        const int16_t relX = 120;
        const int16_t relY = BACK_Y;
        // 3-bit wire layout (backward-compatible): bit0 reserved, bit1=TRAC, bit2=DIR.
        const bool seqComplete = (relayStatus_ & 0x80U) != 0;
        const bool tracOn   = (relayStatus_ & 0x02U) != 0;
        const bool dirOn    = (relayStatus_ & 0x04U) != 0;

        char buf[40];

        // Clear and redraw relay values
        tft.fillRect(relX, relY + 12, 250, 22, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        snprintf(buf, sizeof(buf), "T:%s D:%s",
                 tracOn ? "ON " : "OFF",
                 dirOn  ? "ON " : "OFF");
        uint16_t col = seqComplete ? ui::COL_GREEN : ui::COL_AMBER;
        tft.setTextColor(col, ui::COL_BG);
        tft.drawString(buf, relX, relY + 12);

        snprintf(buf, sizeof(buf), "SEQ:%s  [0x%02X]",
                 seqComplete ? "COMPLETE   " : "IN PROGRESS",
                 relayStatus_);
        tft.setTextColor(seqComplete ? ui::COL_GREEN : ui::COL_AMBER, ui::COL_BG);
        tft.drawString(buf, relX, relY + 22);

        prevRelayStatus_ = relayStatus_;
    }

    // Partial redraw for relay control submenu (CAN telemetry update)
    if (currentMenu_ == SubMenu::RELAY_CONTROL && relayOverrideChanged_) {
        relayOverrideChanged_ = false;
        needsRedraw_ = true;  // full redraw — small screen, simpler than partial
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
                 (unsigned)(batteryVoltage_ / 100), (unsigned)(batteryVoltage_ % 100),
                 (unsigned)(batteryCurrent_ / 100), (unsigned)(batteryCurrent_ % 100));
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
            // Auto-disable relay override when leaving relay control submenu
            if (currentMenu_ == SubMenu::RELAY_CONTROL && relayOverrideEnabled_) {
                CanFrame frame = {};
                frame.identifier       = can::SERVICE_CMD;
                frame.extd             = 0;
                frame.data_length_code = 2;
                frame.data[0]          = can::SERVICE_ACTION_RELAY_OVERRIDE;
                frame.data[1]          = 0x00;
                ESP32Can.writeFrame(frame, 0);
                relayOverrideEnabled_ = false;
                relayOverrideMask_    = 0;
                Serial.println("[ENG] Relay override disabled on BACK");
            }
            clearLogPending_ = false;  // reset confirmation state on navigation (§4.1)
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
            /* Send the mapping to STM32 via CAN (0x112) so it can apply it
             * to STATUS_TEMP_MAP (0x206) frames immediately without reboot.
             * The STM32 will also persist it to flash page 125.             */
            {
                CanFrame frame = {};
                frame.identifier       = can::CMD_SENSOR_MAP_TEMP;
                frame.extd             = 0;
                frame.data_length_code = config_store::NUM_TEMP_SENS;
                for (uint8_t i = 0; i < config_store::NUM_TEMP_SENS; ++i) {
                    frame.data[i] = tempMap_[i];
                }
                ESP32Can.writeFrame(frame, 0);
                Serial.println("[ENG] Temp sensor map sent to STM32 via CAN 0x112");
            }
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
                        case 6:
                            // Open Factory Defaults submenu
                            currentMenu_ = SubMenu::FACTORY_DEFAULTS;
                            break;
                        case 7:
                            // Open DTC Error Log viewer
                            currentMenu_ = SubMenu::DTC_LOG_VIEWER;
                            break;
                        case 8:
                            // Open Maintenance status/reset
                            currentMenu_ = SubMenu::MAINTENANCE;
                            break;
                        case 9:
                            // Open Relay Control (debug)
                            currentMenu_ = SubMenu::RELAY_CONTROL;
                            break;
                        default:
                            break;
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
                    ESP32Can.writeFrame(frame, 0);  // Non-blocking
                    Serial.printf("[ENG] Module %u %s\n", i,
                                  currentlyEnabled ? "DISABLE" : "ENABLE");
                    needsRedraw_ = true;
                }
                return true;
            }
        }
        return false;
    }

    // Factory defaults submenu: tap a row to send the reset command
    if (currentMenu_ == SubMenu::FACTORY_DEFAULTS) {
        if (x >= MENU_X && x <= MENU_X + MENU_W) {
            for (int i = 0; i < NUM_FACTORY_ITEMS; ++i) {
                int16_t btnY = MENU_START_Y + i * MENU_SPACING;
                if (y >= btnY && y <= btnY + MENU_BTN_H) {
                    CanFrame frame = {};
                    frame.identifier       = can::SERVICE_CMD;
                    frame.extd             = 0;
                    frame.data_length_code = 2;
                    frame.data[0]          = factoryActions[i];
                    frame.data[1]          = 0;
                    ESP32Can.writeFrame(frame, 0);  // Non-blocking
                    Serial.printf("[ENG] Factory reset cmd 0x%02X sent\n",
                                  factoryActions[i]);
                    needsRedraw_ = true;
                    return true;
                }
            }
        }
        return false;
    }

    // DTC Log Viewer: CLEAR LOG button (bottom-right) — requires confirmation (§4.1)
    if (currentMenu_ == SubMenu::DTC_LOG_VIEWER) {
        if (x >= SAVE_X && x <= SAVE_X + SAVE_W &&
            y >= SAVE_Y && y <= SAVE_Y + SAVE_H) {
            if (clearLogPending_) {
                // Second tap — confirmed; clear the log
                config_store::clearFaultLog();
                clearLogPending_ = false;
            } else {
                // First tap — enter confirmation state
                clearLogPending_ = true;
            }
            needsRedraw_ = true;
            return true;
        }
        // Any other touch cancels confirmation
        if (clearLogPending_) {
            clearLogPending_ = false;
            needsRedraw_ = true;
        }
        return false;
    }

    // Maintenance: RESET COUNTER and ACK buttons
    if (currentMenu_ == SubMenu::MAINTENANCE) {
        // RESET COUNTER button (bottom-right, where SAVE usually is)
        if (x >= SAVE_X && x <= SAVE_X + SAVE_W &&
            y >= SAVE_Y && y <= SAVE_Y + SAVE_H) {
            config_store::resetMaintenanceCounter();
            config_store::flush();
            needsRedraw_ = true;
            Serial.println("[ENG] Maintenance counter reset");
            return true;
        }
        // ACK REMINDER button (center-right, above RESET)
        if (x >= 200 && x <= 400 && y >= 200 && y <= 230) {
            config_store::acknowledgeMaintenance();
            config_store::flush();
            needsRedraw_ = true;
            Serial.println("[ENG] Maintenance acknowledged");
            return true;
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

    // Relay control: toggle override enable / individual relays
    if (currentMenu_ == SubMenu::RELAY_CONTROL) {
        // Relay control button layout (Y positions match drawRelayControl)
        static constexpr int16_t RC_ROW_Y0 = 80;
        static constexpr int16_t RC_ROW_SPC = 36;
        static constexpr int16_t RC_ROW_H = 30;

        for (int i = 0; i < 3; ++i) {
            int16_t rowY = RC_ROW_Y0 + i * RC_ROW_SPC;
            if (x >= MENU_X && x <= MENU_X + MENU_W &&
                y >= rowY && y <= rowY + RC_ROW_H) {
                if (i == 0) {
                    // Toggle override enable/disable
                    relayOverrideEnabled_ = !relayOverrideEnabled_;
                    if (!relayOverrideEnabled_) {
                        relayOverrideMask_ = 0;
                    }
                } else {
                    // Toggle individual relay bit (only when override enabled)
                    if (relayOverrideEnabled_) {
                        uint8_t bit = (uint8_t)(1U << (i - 1));
                        relayOverrideMask_ ^= bit;
                    }
                }
                // Send CAN command
                {
                    CanFrame frame = {};
                    frame.identifier       = can::SERVICE_CMD;
                    frame.extd             = 0;
                    frame.data_length_code = 2;
                    frame.data[0]          = can::SERVICE_ACTION_RELAY_OVERRIDE;
                    uint8_t ctl = 0;
                    if (relayOverrideEnabled_) ctl |= 0x01U;
                    ctl |= (uint8_t)((relayOverrideMask_ & 0x07U) << 1);
                    frame.data[1] = ctl;
                    ESP32Can.writeFrame(frame, 0);
                    Serial.printf("[ENG] Relay override: en=%d mask=0x%02X\n",
                                  relayOverrideEnabled_ ? 1 : 0, relayOverrideMask_);
                }
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
        uint16_t bgCol = ui::COL_DARK_GRAY;
        uint16_t txtCol = (i == 6) ? ui::COL_AMBER :       // Factory Defaults
                          (i == 7) ? ui::COL_CYAN :         // DTC Error Log
                          (i == 8) ? ui::COL_GREEN :        // Maintenance
                          (i == 9) ? ui::COL_RED :          // Relay Control (debug)
                          ui::COL_WHITE;

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

    // ---- Relay Status Panel (right side of bottom row) ----
    // Shows detailed relay GPIO command state from STM32 heartbeat byte 5.
    {
        const int16_t relX = 120;
        const int16_t relY = BACK_Y;

        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        tft.drawString("RELAY STATUS", relX, relY);

        const bool seqComplete = (relayStatus_ & 0x80U) != 0;
        const bool tracOn   = (relayStatus_ & 0x02U) != 0;
        const bool dirOn    = (relayStatus_ & 0x04U) != 0;

        // Row 1: individual relays (3-bit wire layout, bit0 reserved)
        char buf[40];
        snprintf(buf, sizeof(buf), "T:%s D:%s",
                 tracOn ? "ON " : "OFF",
                 dirOn  ? "ON " : "OFF");
        uint16_t col = seqComplete ? ui::COL_GREEN : ui::COL_AMBER;
        tft.setTextColor(col, ui::COL_BG);
        tft.drawString(buf, relX, relY + 12);

        // Row 2: sequence status + raw hex
        snprintf(buf, sizeof(buf), "SEQ:%s  [0x%02X]",
                 seqComplete ? "COMPLETE   " : "IN PROGRESS",
                 relayStatus_);
        tft.setTextColor(seqComplete ? ui::COL_GREEN : ui::COL_AMBER, ui::COL_BG);
        tft.drawString(buf, relX, relY + 22);

        prevRelayStatus_ = relayStatus_;
    }
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
             (unsigned)(moduleCtrlPage_ + 1), (unsigned)MODULE_CTRL_PAGES);
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

    // ACK feedback status bar (shown briefly after toggle attempt)
    if (lastAckResult_ != 0) {
        const char* msg;
        uint16_t msgCol;
        switch (lastAckResult_) {
            case 1:  msg = "OK";                          msgCol = ui::COL_GREEN;  break;
            case 2:  msg = "REJECTED (critical module)";  msgCol = ui::COL_RED;    break;
            case 3:  msg = "BLOCKED (vehicle in motion or unsafe state)";
                     msgCol = ui::COL_RED;    break;
            default: msg = "ERROR";                       msgCol = ui::COL_RED;    break;
        }
        tft.fillRect(100, 265, 280, 14, ui::COL_BG);
        tft.setTextColor(msgCol, ui::COL_BG);
        tft.drawString(msg, ui::SCREEN_W / 2, 270);
    }

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
// Column A = sensor index + live °C reading (from STATUS_TEMP 0x202, physical order)
// Column B = assigned position (editable, tap to cycle through options).
// Tap SAVE to persist and send to STM32. Tap BACK to discard changes.
//
// The live temperature helps identify which sensor is which physically:
// touch a sensor to warm it up and watch which row changes.
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
    tft.drawString("Sensor  Temp", MAP_ROW_X + 4, 30);
    tft.drawString("Assigned Position  (tap to change)", MAP_ROW_X + 130, 30);
    RTRACE_TEXT(MAP_ROW_X + 4, 30, "Sensor  Temp", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    char buf[ui::FMT_BUF_LARGE];

    for (uint8_t i = 0; i < config_store::NUM_TEMP_SENS; ++i) {
        int16_t rowY = MAP_ROW_Y0 + i * MAP_ROW_SPC;

        // Row background
        tft.fillRect(MAP_ROW_X, rowY, MAP_ROW_W, MAP_ROW_H, ui::COL_DARK_GRAY);
        tft.drawRect(MAP_ROW_X, rowY, MAP_ROW_W, MAP_ROW_H, ui::COL_GRAY);

        // Sensor index + live raw temperature (left column)
        tft.setTextDatum(ML_DATUM);
        snprintf(buf, sizeof(buf), "%s %3d%s", TEMP_IDX_NAMES[i], (int)rawTempC_[i], TEMP_DEGREE_STR);
        tft.setTextColor(ui::COL_CYAN, ui::COL_DARK_GRAY);
        tft.drawString(buf, MAP_ROW_X + 4, rowY + MAP_ROW_H / 2);

        // Assigned position (right column)
        uint8_t posIdx = tempMap_[i];
        if (posIdx >= config_store::NUM_TEMP_SENS) posIdx = 0;
        snprintf(buf, sizeof(buf), "%s%s", TEMP_ARROW_STR, TEMP_POS_NAMES[posIdx]);
        tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
        tft.drawString(buf, MAP_ROW_X + 130, rowY + MAP_ROW_H / 2);
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
// drawFactoryDefaults — Individual factory-default reset options
//
// Provides granular calibration resets:
//   - Steering PID calibration
//   - Wheel speed sensor calibration
//   - INA226 / shunt calibration
//   - Traction motor force limits
//   - Steering motor force limits
//   - Full factory restore (all of the above)
// -------------------------------------------------------------------------
void EngineeringScreen::drawFactoryDefaults() {
    RTRACE_BEGIN_SCREEN("eng_factory_defaults");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("FACTORY DEFAULTS", ui::SCREEN_W / 2, 22);
    RTRACE_TEXT(ui::SCREEN_W / 2, 22, "FACTORY DEFAULTS",
                ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);
    tft.setTextDatum(TL_DATUM);

    // Menu buttons
    for (int i = 0; i < NUM_FACTORY_ITEMS; ++i) {
        int16_t btnY = MENU_START_Y + i * MENU_SPACING;
        // Last item (RESET ALL) is red, others are dark gray
        uint16_t bgCol = (i == NUM_FACTORY_ITEMS - 1)
                         ? ui::COL_RED : ui::COL_DARK_GRAY;
        uint16_t txtCol = ui::COL_WHITE;

        tft.fillRect(MENU_X, btnY, MENU_W, MENU_BTN_H, bgCol);
        RTRACE_FILL_RECT(MENU_X, btnY, MENU_W, MENU_BTN_H, bgCol);
        tft.drawRect(MENU_X, btnY, MENU_W, MENU_BTN_H, ui::COL_GRAY);
        RTRACE_DRAW_RECT(MENU_X, btnY, MENU_W, MENU_BTN_H, ui::COL_GRAY);

        tft.setTextColor(txtCol, bgCol);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(factoryLabels[i], MENU_X + MENU_W / 2,
                        btnY + MENU_BTN_H / 2);
        RTRACE_TEXT(MENU_X + MENU_W / 2, btnY + MENU_BTN_H / 2,
                    factoryLabels[i], txtCol, bgCol, 1, MC_DATUM);
    }
    tft.setTextDatum(TL_DATUM);

    // Warning text
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(1);
    tft.drawString("Tap an option to reset that calibration to defaults.",
                    20, MENU_START_Y + NUM_FACTORY_ITEMS * MENU_SPACING + 8);
    tft.drawString("Vehicle must be stationary. Reboot may be required.",
                    20, MENU_START_Y + NUM_FACTORY_ITEMS * MENU_SPACING + 22);

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
// drawDtcLogViewer — Persistent DTC Fault Log Viewer
//
// Shows last N fault events stored in NVS with human-readable error code
// descriptions, fault flags, subsystem, and system state at the time.
// CLEAR LOG button erases all entries.
// -------------------------------------------------------------------------

// Safety error code → short name (matches can_ids.h SafetyError)
static const char* dtcErrorName(uint8_t code) {
    switch (code) {
        case 0:  return "NONE";
        case 1:  return "OVERCURRENT";
        case 2:  return "OVERTEMP";
        case 3:  return "CAN TIMEOUT";
        case 4:  return "SENSOR FAULT";
        case 5:  return "MOTOR STALL";
        case 6:  return "E-STOP";
        case 7:  return "WATCHDOG";
        case 8:  return "CENTERING";
        case 9:  return "BATT UV WARN";
        case 10: return "BATT UV CRIT";
        case 11: return "I2C FAIL";
        case 12: return "OBSTACLE";
        case 13: return "CAN BUS-OFF";
        case 14: return "BATT OV WARN";
        case 15: return "BATT OV CRIT";
        case 16: return "RELAY OPEN";
        default: return "UNKNOWN";
    }
}

// System state → short name
static const char* dtcStateName(uint8_t st) {
    switch (st) {
        case 0: return "BOOT";
        case 1: return "STANDBY";
        case 2: return "ACTIVE";
        case 3: return "DEGRADED";
        case 4: return "SAFE";
        case 5: return "ERROR";
        case 6: return "LIMP";
        default: return "?";
    }
}

void EngineeringScreen::drawDtcLogViewer() {
    RTRACE_BEGIN_SCREEN("eng_dtc_log");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("DTC FAULT LOG", ui::SCREEN_W / 2, 12);
    RTRACE_TEXT(ui::SCREEN_W / 2, 12, "DTC FAULT LOG",
                ui::COL_AMBER, ui::COL_BG, 1, MC_DATUM);
    tft.setTextDatum(TL_DATUM);

    uint8_t count = config_store::getFaultLogCount();

    if (count == 0) {
        tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
        tft.drawString("No faults recorded.", 40, 50);
        RTRACE_TEXT(40, 50, "No faults recorded.",
                    ui::COL_GREEN, ui::COL_BG, 1, TL_DATUM);
    } else {
        // Column headers
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("#", 10, 28);
        tft.drawString("Uptime", 30, 28);
        tft.drawString("Error", 110, 28);
        tft.drawString("Flags", 230, 28);
        tft.drawString("Sub", 300, 28);
        tft.drawString("State", 350, 28);

        // Show up to 12 most recent entries (screen space limited)
        uint8_t maxShow = (count < 12) ? count : 12;
        // Show most recent first: iterate from newest (count-1) to oldest
        for (uint8_t i = 0; i < maxShow; ++i) {
            uint8_t idx = count - 1 - i;  // newest first
            config_store::FaultLogEntry fte;
            if (!config_store::getFaultLogEntry(idx, fte)) continue;

            int16_t rowY = 40 + i * 18;
            char buf[80];

            // Entry number
            snprintf(buf, sizeof(buf), "%u", idx + 1);
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            tft.drawString(buf, 10, rowY);

            // Uptime (hh:mm:ss for clarity §4.2)
            uint32_t sec = fte.uptimeMs / 1000;
            uint32_t hrs = sec / 3600;
            uint32_t mns = (sec % 3600) / 60;
            uint32_t scs = sec % 60;
            if (hrs > 0) {
                snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu",
                         (unsigned long)hrs, (unsigned long)mns, (unsigned long)scs);
            } else {
                snprintf(buf, sizeof(buf), "%lu:%02lu",
                         (unsigned long)mns, (unsigned long)scs);
            }
            tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
            tft.drawString(buf, 30, rowY);

            // Error code + name
            snprintf(buf, sizeof(buf), "%u:%s", fte.errorCode,
                     dtcErrorName(fte.errorCode));
            uint16_t errCol = (fte.errorCode > 0) ? ui::COL_RED : ui::COL_GREEN;
            tft.setTextColor(errCol, ui::COL_BG);
            tft.drawString(buf, 110, rowY);

            // Fault flags hex
            snprintf(buf, sizeof(buf), "0x%02X", fte.faultFlags);
            tft.setTextColor(fte.faultFlags ? ui::COL_AMBER : ui::COL_GREEN, ui::COL_BG);
            tft.drawString(buf, 230, rowY);

            // Subsystem
            static const char* const subNames[] = {"GLB","MOT","SEN","CAN"};
            const char* subName = (fte.subsystem < 4) ? subNames[fte.subsystem] : "?";
            tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
            tft.drawString(subName, 300, rowY);

            // System state
            tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
            tft.drawString(dtcStateName(fte.systemState), 350, rowY);
        }

        // Entry count summary
        char countBuf[32];
        snprintf(countBuf, sizeof(countBuf), "Total: %u/%u entries",
                 count, config_store::FAULT_LOG_MAX_ENTRIES);
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString(countBuf, 10, 260);
    }

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

    // CLEAR LOG button (red, bottom-right) — shows "CONFIRM?" after first tap (§4.1)
    if (count > 0) {
        uint16_t btnCol = clearLogPending_ ? ui::COL_AMBER : ui::COL_RED;
        const char* btnLabel = clearLogPending_ ? "CONFIRM?" : "CLEAR";
        tft.fillRect(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, btnCol);
        RTRACE_FILL_RECT(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, btnCol);
        tft.drawRect(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_WHITE);
        RTRACE_DRAW_RECT(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_WHITE);
        tft.setTextColor(clearLogPending_ ? ui::COL_BLACK : ui::COL_WHITE, btnCol);
        tft.drawString(btnLabel, SAVE_X + SAVE_W / 2, SAVE_Y + SAVE_H / 2);
        RTRACE_TEXT(SAVE_X + SAVE_W / 2, SAVE_Y + SAVE_H / 2, btnLabel,
                    clearLogPending_ ? ui::COL_BLACK : ui::COL_WHITE, btnCol, 1, MC_DATUM);
    }
    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// drawMaintenance — Maintenance counter status and reset
//
// Shows cumulative runtime, maintenance threshold, and whether service
// is due.  Provides RESET COUNTER and ACK REMINDER buttons.
// -------------------------------------------------------------------------
void EngineeringScreen::drawMaintenance() {
    RTRACE_BEGIN_SCREEN("eng_maintenance");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("MAINTENANCE", ui::SCREEN_W / 2, 22);
    RTRACE_TEXT(ui::SCREEN_W / 2, 22, "MAINTENANCE",
                ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    const auto& cfg = config_store::get();
    char buf[64];

    // Runtime
    uint32_t totalSec = cfg.runtimeSeconds;
    uint32_t hours = totalSec / 3600;
    uint32_t mins  = (totalSec % 3600) / 60;
    snprintf(buf, sizeof(buf), "Runtime: %lu h %lu min", (unsigned long)hours, (unsigned long)mins);
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.drawString(buf, 40, 60);

    // Threshold
    snprintf(buf, sizeof(buf), "Service interval: %lu hours",
             (unsigned long)cfg.maintIntervalHours);
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString(buf, 40, 80);

    // Progress bar
    uint32_t thresholdSec = cfg.maintIntervalHours * 3600UL;
    uint8_t pct = (thresholdSec > 0) ?
        static_cast<uint8_t>((totalSec * 100UL) / thresholdSec) : 100;
    if (pct > 100) pct = 100;

    int16_t barX = 40, barY = 100, barW = 300, barH = 18;
    tft.drawRect(barX, barY, barW, barH, ui::COL_GRAY);
    uint16_t fillCol = (pct >= 100) ? ui::COL_RED :
                       (pct >= 80)  ? ui::COL_AMBER : ui::COL_GREEN;
    int16_t fillW = static_cast<int16_t>((int32_t)barW * pct / 100);
    if (fillW > 2) {
        tft.fillRect(barX + 1, barY + 1, fillW - 2, barH - 2, fillCol);
    }
    snprintf(buf, sizeof(buf), "%u%%", pct);
    tft.setTextColor(ui::COL_WHITE, fillCol);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(buf, barX + barW / 2, barY + barH / 2);
    tft.setTextDatum(TL_DATUM);

    // Status message
    bool due = config_store::isMaintenanceDue();
    if (due) {
        tft.setTextColor(ui::COL_RED, ui::COL_BG);
        tft.setTextSize(2);
        tft.drawString("MANTENIMIENTO PENDIENTE", 40, 140);
        tft.setTextSize(1);

        if (cfg.maintAcknowledged) {
            tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
            tft.drawString("(Acknowledged - reminder silenced)", 40, 165);
        } else {
            // ACK REMINDER button
            tft.fillRect(200, 200, 200, 30, ui::COL_AMBER);
            tft.drawRect(200, 200, 200, 30, ui::COL_WHITE);
            tft.setTextColor(ui::COL_BLACK, ui::COL_AMBER);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("ACEPTAR MANTENIMIENTO", 300, 215);
            tft.setTextDatum(TL_DATUM);
        }
    } else {
        tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
        tft.setTextSize(2);
        tft.drawString("OK - NO SERVICE DUE", 40, 140);
        tft.setTextSize(1);
    }

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

    // RESET COUNTER button (bottom-right)
    tft.fillRect(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_RED);
    RTRACE_FILL_RECT(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_RED);
    tft.drawRect(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_WHITE);
    RTRACE_DRAW_RECT(SAVE_X, SAVE_Y, SAVE_W, SAVE_H, ui::COL_WHITE);
    tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
    tft.drawString("RESET", SAVE_X + SAVE_W / 2, SAVE_Y + SAVE_H / 2);
    RTRACE_TEXT(SAVE_X + SAVE_W / 2, SAVE_Y + SAVE_H / 2, "RESET",
                ui::COL_WHITE, ui::COL_RED, 1, MC_DATUM);

    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// RELAY CONTROL (Engineering Diagnostic Mode)
//
// Manual relay GPIO override for diagnostic purposes.
// Safety constraints enforced on STM32 side — the UI only reflects state.
// -------------------------------------------------------------------------
void EngineeringScreen::drawRelayControl() {
    RTRACE_BEGIN_SCREEN("eng_relay_ctrl");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("RELAY CONTROL", ui::SCREEN_W / 2, 22);
    tft.setTextDatum(TL_DATUM);

    // Safety warning when override is active
    if (relayOverrideEnabled_) {
        tft.setTextColor(ui::COL_RED, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("!! MANUAL RELAY CONTROL ACTIVE !!", ui::SCREEN_W / 2, 48);
        tft.setTextDatum(TL_DATUM);
    } else {
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Override disabled (STANDBY only)", ui::SCREEN_W / 2, 48);
        tft.setTextDatum(TL_DATUM);
    }

    // Button rows: Override Enable, TRACTION, DIRECTION (CAN rev 1.3 compatible (2026-04-23 clarification))
    static constexpr int16_t RC_ROW_Y0 = 80;
    static constexpr int16_t RC_ROW_SPC = 36;
    static constexpr int16_t RC_ROW_H = 30;

    static const char* const rowLabels[3] = {
        "Override Enable",
        "TRACTION  (PC11)",
        "DIRECTION (PC12)"
    };

    // Real relay state from CAN heartbeat byte 5 (3-bit wire layout).
    // bit 0 = reserved (always 0), bit 1 = TRAC, bit 2 = DIR.
    const bool realTrac = (relayStatus_ & 0x02U) != 0;
    const bool realDir  = (relayStatus_ & 0x04U) != 0;
    const bool realState[3] = {
        relayOverrideEnabled_,
        realTrac,
        realDir
    };

    tft.setTextSize(1);
    for (int i = 0; i < 3; ++i) {
        int16_t rowY = RC_ROW_Y0 + i * RC_ROW_SPC;

        // Button background
        uint16_t bgCol = ui::COL_DARK_GRAY;
        uint16_t borderCol = ui::COL_GRAY;
        if (i == 0) {
            // Override enable toggle
            bgCol = relayOverrideEnabled_ ? ui::COL_RED : ui::COL_DARK_GRAY;
            borderCol = relayOverrideEnabled_ ? ui::COL_WHITE : ui::COL_GRAY;
        } else if (relayOverrideEnabled_) {
            bgCol = (relayOverrideMask_ & (1U << (i - 1)))
                    ? ui::COL_GREEN : ui::COL_DARK_GRAY;
            borderCol = ui::COL_WHITE;
        }

        tft.fillRect(MENU_X, rowY, MENU_W, RC_ROW_H, bgCol);
        tft.drawRect(MENU_X, rowY, MENU_W, RC_ROW_H, borderCol);

        // Label (left)
        tft.setTextColor(ui::COL_WHITE, bgCol);
        tft.setTextDatum(ML_DATUM);
        tft.drawString(rowLabels[i], MENU_X + 8, rowY + RC_ROW_H / 2);

        // Status (right)
        const char* statusStr = realState[i] ? "ON " : "OFF";
        uint16_t statusCol = realState[i] ? ui::COL_GREEN : ui::COL_GRAY;
        if (i == 0) {
            statusStr = relayOverrideEnabled_ ? "ENABLED " : "DISABLED";
            statusCol = relayOverrideEnabled_ ? ui::COL_RED : ui::COL_GRAY;
        }
        tft.setTextColor(statusCol, bgCol);
        tft.setTextDatum(MR_DATUM);
        tft.drawString(statusStr, MENU_X + MENU_W - 8, rowY + RC_ROW_H / 2);
    }

    // Relay status from CAN (real GPIO state)
    {
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
        const int16_t infoY = RC_ROW_Y0 + 3 * RC_ROW_SPC + 10;

        tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        tft.drawString("CAN RELAY STATUS (real)", MENU_X, infoY);

        char buf[48];
        snprintf(buf, sizeof(buf), "T:%s D:%s  SEQ:%s  [0x%02X]",
                 realTrac ? "ON " : "OFF",
                 realDir  ? "ON " : "OFF",
                 (relayStatus_ & 0x80U) ? "COMPLETE" : "IDLE    ",
                 relayStatus_);

        uint16_t relCol = (relayStatus_ & 0x80U) ? ui::COL_GREEN : ui::COL_AMBER;
        tft.setTextColor(relCol, ui::COL_BG);
        tft.drawString(buf, MENU_X, infoY + 14);
    }

    // BACK button (bottom-left)
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_AMBER);
    tft.setTextColor(ui::COL_AMBER, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    tft.setTextDatum(TL_DATUM);
}
