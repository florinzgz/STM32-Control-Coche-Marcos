// =============================================================================
// ESP32-S3 HMI — Engineering Screen Implementation
//
// Hidden engineering menu with submenus for diagnostics and calibration.
// Accessed via secret code "8989" (handled by screen_manager).
// =============================================================================

#include "engineering_screen.h"
#include "ui/ui_common.h"
#include "can_ids.h"
#include <TFT_eSPI.h>
#include <ESP32-TWAI-CAN.hpp>
#include <cstdio>
#include <cstring>

extern TFT_eSPI tft;

// ---- Menu button layout ----
static constexpr int16_t MENU_X       = 40;
static constexpr int16_t MENU_W       = 400;
static constexpr int16_t MENU_BTN_H   = 40;
static constexpr int16_t MENU_START_Y = 60;
static constexpr int16_t MENU_SPACING = 50;

static constexpr int     NUM_MAIN_ITEMS = 5;
static const char* const mainLabels[NUM_MAIN_ITEMS] = {
    "FAULT VIEWER",
    "MODULE ENABLE/DISABLE",
    "PEDAL CALIBRATION",
    "ENCODER CALIBRATION",
    "FACTORY RESTORE"
};

// ---- Back button ----
static constexpr int16_t BACK_X = 10;
static constexpr int16_t BACK_Y = 280;
static constexpr int16_t BACK_W = 80;
static constexpr int16_t BACK_H = 30;

// -------------------------------------------------------------------------
// Lifecycle
// -------------------------------------------------------------------------
void EngineeringScreen::onEnter() {
    needsRedraw_ = true;
    currentMenu_ = SubMenu::MAIN;
    prevFaultBits_    = 0xFFFFFFFF;
    prevEnabledBits_  = 0xFFFFFFFF;
    prevDisabledBits_ = 0xFFFFFFFF;
}

void EngineeringScreen::onExit() {}

void EngineeringScreen::update(const vehicle::VehicleData& data) {
    // Cache service mode data for fault viewer
    faultBits_    = data.service().faultMask;
    enabledBits_  = data.service().enabledMask;
    disabledBits_ = data.service().disabledMask;
}

// -------------------------------------------------------------------------
// Draw
// -------------------------------------------------------------------------
void EngineeringScreen::draw() {
    if (needsRedraw_) {
        needsRedraw_ = false;
        tft.fillScreen(ui::COL_BG);

        switch (currentMenu_) {
            case SubMenu::MAIN:           drawMainMenu();          break;
            case SubMenu::FAULT_VIEWER:   drawFaultViewer();       break;
            case SubMenu::MODULE_CONTROL: drawModuleControl();     break;
            case SubMenu::PEDAL_CAL:      drawCalibration("PEDAL CALIBRATION"); break;
            case SubMenu::ENCODER_CAL:    drawCalibration("ENCODER CALIBRATION"); break;
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

            tft.fillRect(200, 100, 240, 16, ui::COL_BG);
            snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)faultBits_);
            tft.setTextColor(faultBits_ ? ui::COL_RED : ui::COL_GREEN, ui::COL_BG);
            tft.setTextSize(1);
            tft.drawString(buf, 200, 100);

            tft.fillRect(200, 130, 240, 16, ui::COL_BG);
            snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)enabledBits_);
            tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
            tft.drawString(buf, 200, 130);

            tft.fillRect(200, 160, 240, 16, ui::COL_BG);
            snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)disabledBits_);
            tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
            tft.drawString(buf, 200, 160);
        }
    }
}

// -------------------------------------------------------------------------
// Touch handling
// -------------------------------------------------------------------------
bool EngineeringScreen::handleTouch(int16_t x, int16_t y) {
    // Back button (all submenus)
    if (currentMenu_ != SubMenu::MAIN) {
        if (x >= BACK_X && x <= BACK_X + BACK_W &&
            y >= BACK_Y && y <= BACK_Y + BACK_H) {
            currentMenu_ = SubMenu::MAIN;
            needsRedraw_ = true;
            return true;
        }
    }

    // Main menu item selection
    if (currentMenu_ == SubMenu::MAIN) {
        if (x >= MENU_X && x <= MENU_X + MENU_W) {
            for (int i = 0; i < NUM_MAIN_ITEMS; ++i) {
                int16_t btnY = MENU_START_Y + i * MENU_SPACING;
                if (y >= btnY && y <= btnY + MENU_BTN_H) {
                    switch (i) {
                        case 0: currentMenu_ = SubMenu::FAULT_VIEWER;   break;
                        case 1: currentMenu_ = SubMenu::MODULE_CONTROL; break;
                        case 2: currentMenu_ = SubMenu::PEDAL_CAL;      break;
                        case 3: currentMenu_ = SubMenu::ENCODER_CAL;    break;
                        case 4: {
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
    }

    return false;
}

// -------------------------------------------------------------------------
// Draw helpers
// -------------------------------------------------------------------------
void EngineeringScreen::drawMainMenu() {
    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("ENGINEERING", ui::SCREEN_W / 2, 25);
    tft.setTextDatum(TL_DATUM);

    // Menu buttons
    for (int i = 0; i < NUM_MAIN_ITEMS; ++i) {
        int16_t btnY = MENU_START_Y + i * MENU_SPACING;
        uint16_t bgCol = (i == 4) ? ui::COL_RED : ui::COL_DARK_GRAY;
        uint16_t txtCol = ui::COL_WHITE;

        tft.fillRect(MENU_X, btnY, MENU_W, MENU_BTN_H, bgCol);
        tft.drawRect(MENU_X, btnY, MENU_W, MENU_BTN_H, ui::COL_GRAY);

        tft.setTextColor(txtCol, bgCol);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(mainLabels[i], MENU_X + MENU_W / 2,
                        btnY + MENU_BTN_H / 2);
    }
    tft.setTextDatum(TL_DATUM);
}

void EngineeringScreen::drawFaultViewer() {
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("FAULT VIEWER", ui::SCREEN_W / 2, 25);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    // Labels
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Fault Bits:", 40, 100);
    tft.drawString("Enabled Bits:", 40, 130);
    tft.drawString("Disabled Bits:", 40, 160);

    // Force redraw of values
    prevFaultBits_ = faultBits_ + 1;

    // Back button
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    tft.setTextDatum(TL_DATUM);
}

void EngineeringScreen::drawModuleControl() {
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("MODULE CONTROL", ui::SCREEN_W / 2, 25);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Use SERVICE_CMD 0x110 to enable/disable modules.", 40, 80);
    tft.drawString("Module IDs: 0-24 (see service_mode.h)", 40, 100);

    // Back button
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    tft.setTextDatum(TL_DATUM);
}

void EngineeringScreen::drawCalibration(const char* title) {
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(title, ui::SCREEN_W / 2, 25);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Calibration interface placeholder.", 40, 80);
    tft.drawString("Connect engineering tool for full calibration.", 40, 100);

    // Back button
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    tft.setTextDatum(TL_DATUM);
}
