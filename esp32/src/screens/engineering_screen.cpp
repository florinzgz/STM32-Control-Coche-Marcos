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
#include "can_rx.h"
#include "config_store.h"
#include "display_backlight.h"
#include "touch_calibration.h"
#include "ui/debug_overlay.h"
#include <TFT_eSPI.h>
#include <ESP32-TWAI-CAN.hpp>
#include <cstdio>
#include <cstring>
#include <cmath>

extern TFT_eSPI tft;

// ---- Submenu list-button layout (FACTORY_DEFAULTS, RELAY_CONTROL) ----
// MENU_BTN_H/MENU_SPACING/MENU_START_Y/MENU_X/MENU_W lay out the simple list
// rows still used by the FACTORY_DEFAULTS and RELAY_CONTROL submenus.  The
// main Engineering menu no longer uses these — it was redesigned (FASE 2) into
// a paged grid of large touch tiles (see TILE_* constants below).
static constexpr int16_t MENU_X       = 40;
static constexpr int16_t MENU_W       = 400;
static constexpr int16_t MENU_BTN_H   = 16;
static constexpr int16_t MENU_START_Y = 40;
static constexpr int16_t MENU_SPACING = 17;

// Number of main-menu functions.  Full canonical names are kept in tileLabel*
// (abbreviated, two-line) below.
static constexpr int     NUM_MAIN_ITEMS = 17;

// ---- FASE 2 — Professional tile layout for the main menu --------------------
// Functions are presented as large touch tiles across two pages
// (page 1 = items 0..8, page 2 = items 9..16).  Tiles are 148x72 px with a
// 10 px separation — well above the 44 px minimum touch target, usable with
// gloves.  The dispatch logic is unchanged: a tile maps back to its original
// item index and runs the exact same code path as the old list row.
static constexpr int16_t TILE_W      = 148;
static constexpr int16_t TILE_H      = 72;
static constexpr int16_t TILE_GAP    = 10;
static constexpr int16_t TILE_COL0_X = 8;     // left margin: (480-3*148-2*10)/2
static constexpr int16_t TILE_ROW0_Y = 36;    // below the title bar
static constexpr int     TILE_COLS   = 3;

static constexpr uint8_t MAIN_PAGE_COUNT = 2;
static constexpr int     PAGE1_ITEM_COUNT = 9;   // items 0..8

// Two-line short labels (rendered at text size 2 for legibility from the
// driver's seat / with gloves — never size 1 for these tile captions).
static const char* const tileLabel1[NUM_MAIN_ITEMS] = {
    "FAULT", "MODULE", "PEDAL", "ENCODER", "INA226",
    "TEMP", "FACTORY", "DTC", "MAINT.",
    "RELAY", "INA226", "CAN", "TOUCH", "RESET", "MCP23017", "GEAR", "BRIGHT"
};
static const char* const tileLabel2[NUM_MAIN_ITEMS] = {
    "VIEWER", "EN/DIS", "CAL", "CAL", "MAP",
    "MAP", "DEFAULT", "LOG", "",
    "CTRL", "LIVE", "DIAG", "CAL", "TOUCH CAL", "SHIFTER", "LIMITS", "DISPLAY"
};

// Category accent colour per function (FASE 2 colour coding):
//   Diagnóstico = cian, Calibración = verde, Configuración = azul,
//   Mantenimiento = amarillo, Acciones destructivas = rojo/ámbar.
// Captions are always rendered in white (size 2) so legibility never depends
// on the accent hue; the accent only tints the icon and the tile border.
static const uint16_t tileColor[NUM_MAIN_ITEMS] = {
    ui::COL_CYAN,    // 0  Fault Viewer        — diagnostic
    ui::COL_BLUE,    // 1  Module En/Dis       — configuration
    ui::COL_GREEN,   // 2  Pedal Calibration   — calibration
    ui::COL_GREEN,   // 3  Encoder Calibration — calibration
    ui::COL_BLUE,    // 4  INA226 Mapping      — configuration
    ui::COL_BLUE,    // 5  Temp Mapping        — configuration
    ui::COL_AMBER,   // 6  Factory Defaults    — destructive
    ui::COL_CYAN,    // 7  DTC Error Log       — diagnostic
    ui::COL_YELLOW,  // 8  Maintenance         — maintenance
    ui::COL_RED,     // 9  Relay Control       — destructive / debug
    ui::COL_CYAN,    // 10 INA226 Live Diag    — diagnostic
    ui::COL_CYAN,    // 11 Debounce/CAN Diag   — diagnostic
    ui::COL_GREEN,   // 12 Touch Calibration   — calibration
    ui::COL_AMBER,   // 13 Reset Touch Cal     — destructive
    ui::COL_CYAN,    // 14 MCP23017 Live       — diagnostic
    ui::COL_BLUE,    // 15 Gear Power Limits   — configuration
    ui::COL_BLUE     // 16 Display Brightness  — configuration
};

// Bottom navigation bar for the main menu (FASE 2): PAGE 1 / PAGE 2 / EXIT.
// Always visible on both pages.  Submenus keep the global BACK button below.
static constexpr int16_t NAV_Y    = 276;
static constexpr int16_t NAV_H    = 36;
static constexpr int16_t NAVP1_X  = 8;
static constexpr int16_t NAVP1_W  = 132;
static constexpr int16_t NAVP2_X  = 150;
static constexpr int16_t NAVP2_W  = 132;
static constexpr int16_t NAVEX_X  = 360;
static constexpr int16_t NAVEX_W  = 112;

// Compact relay-status read-out in the header right corner (FASE 2 keeps the
// existing relay GPIO state visible on both menu pages without a dedicated
// list row).  Shared by drawMainMenu() and the partial-redraw path in draw().
static constexpr int16_t RELAY_RDX = 300;
static constexpr int16_t RELAY_RDY = 3;

// Mirror Safe Screen 0x309 staleness threshold.
static constexpr unsigned long ENG_I2C_DIAG_STALE_MS = 2000;

// ---- Back / Save buttons ----
static constexpr int16_t BACK_X = 10;
static constexpr int16_t BACK_Y = 280;
static constexpr int16_t BACK_W = 80;
static constexpr int16_t BACK_H = 30;

// Steering calibration gauge + read-out panel geometry (FASE 3.5).
static constexpr int16_t ECAL_GAUGE_CX = 132;
static constexpr int16_t ECAL_GAUGE_CY = 178;
static constexpr int16_t ECAL_GAUGE_R  = 96;
static constexpr int16_t ECAL_PANEL_X  = 270;
static constexpr int16_t ECAL_Z_BTN_Y  = 280;
static constexpr int16_t ECAL_Z_BTN_H  = 30;
static constexpr int16_t ECAL_Z_QUERY_X = 100;
static constexpr int16_t ECAL_Z_QUERY_W = 90;
static constexpr int16_t ECAL_Z_CAL_X   = 200;
static constexpr int16_t ECAL_Z_CAL_W   = 120;
static constexpr int16_t ECAL_Z_CLEAR_X = 330;
static constexpr int16_t ECAL_Z_CLEAR_W = 140;

static constexpr int16_t SAVE_X = 390;
static constexpr int16_t SAVE_Y = 280;
static constexpr int16_t SAVE_W = 80;
static constexpr int16_t SAVE_H = 30;

// ---- DEBUG OVERLAY toggle button (DEBOUNCE / CAN DIAG submenu only) ----
// Bottom centre, between BACK (x=10..90) and RUN I2C SCAN (x=320..470).
// This is the ONLY way to show the runtime performance overlay — it is no
// longer bound to the global long-press gesture.
#if RUNTIME_MONITOR
static constexpr int16_t DBGOVL_X = 110;
static constexpr int16_t DBGOVL_Y = 280;
static constexpr int16_t DBGOVL_W = 150;
static constexpr int16_t DBGOVL_H = 30;
#endif

// ---- Pedal calibration submenu button layout (480×320) ----
// Two rows of buttons centred under the live telemetry block.  BACK reuses
// the global BACK_X/Y above; the four action buttons are below the
// instructional row to maximise touch separation.
static constexpr int16_t PED_BTN_Y       = 232;
static constexpr int16_t PED_BTN_H       = 36;
static constexpr int16_t PED_BTN_W       = 110;
static constexpr int16_t PED_BTN_CAPMIN_X  = 10;
static constexpr int16_t PED_BTN_CAPMAX_X  = 125;
static constexpr int16_t PED_BTN_SAVE_X    = 245;
static constexpr int16_t PED_BTN_RESET_X   = 360;

// ---- Gear power-limits editor layout (GEAR_LIMITS submenu) ----
// Three editable rows (D2/D1/R), each with a −5% / value / +5% control, plus
// a bottom action row (SAVE / RESTORE DEF.) and the global BACK button.
// The same screen hosts two paged groups — POWER LIMIT % and ACCEL
// RESPONSE % — toggled by the PAGE button (top-right).
static constexpr int16_t GL_ROW0_Y    = 70;    // first gear row top
static constexpr int16_t GL_ROW_H     = 44;    // row pitch
static constexpr int16_t GL_ROW_BTN_H = 36;
static constexpr int16_t GL_MINUS_X   = 230;   // −5% button x
static constexpr int16_t GL_PLUS_X    = 400;   // +5% button x
static constexpr int16_t GL_STEP_W    = 60;    // step button width
static constexpr int16_t GL_BTN_Y     = 232;   // bottom action row y
static constexpr int16_t GL_BTN_H     = 36;
static constexpr int16_t GL_BTN_W     = 150;
static constexpr int16_t GL_SAVE_X    = 175;   // SAVE x
static constexpr int16_t GL_RESTORE_X = 330;   // RESTORE DEF. x
static constexpr int16_t GL_STEP_PCT  = 5;     // ±5 % per tap
// PAGE toggle (POWER <-> RESPONSE) — top-right, clear of the centred title.
static constexpr int16_t GL_PAGE_X    = 385;
static constexpr int16_t GL_PAGE_Y    = 4;
static constexpr int16_t GL_PAGE_W    = 90;
static constexpr int16_t GL_PAGE_H    = 26;

// ---- TFT brightness submenu layout (BRIGHTNESS) ----
static constexpr int16_t BRI_BOX_X    = 70;
static constexpr int16_t BRI_BOX_Y    = 78;
static constexpr int16_t BRI_BOX_W    = 340;
static constexpr int16_t BRI_BOX_H    = 126;
static constexpr int16_t BRI_MINUS_X  = 120;
static constexpr int16_t BRI_PLUS_X   = 300;
static constexpr int16_t BRI_BTN_Y    = 228;
static constexpr int16_t BRI_BTN_W    = 60;
static constexpr int16_t BRI_BTN_H    = 36;

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
    mainMenuPage_ = 0;          // FASE 2: always open on page 1
    clearLogPending_ = false;   // §5: reset confirmation state on screen enter
    factoryPendingIdx_ = -1;    // FASE 2 §1: clear factory confirm on screen enter
    modulePendingId_   = -1;    // FASE 2 §1: clear module confirm on screen enter
    relayStandbyMsg_   = false; // FASE 2 §2: clear standby notice on screen enter
    inaEditRow_  = 0;
    tempEditRow_ = 0;
    moduleCtrlPage_ = 0;
    prevFaultBits_    = 0xFFFFFFFF;
    prevEnabledBits_  = 0xFFFFFFFF;
    prevDisabledBits_ = 0xFFFFFFFF;
    pedalDataChanged_   = false;
    encoderDataChanged_ = false;
    steerZDataChanged_  = false;
    steerZClearPending_ = false;
    lastAckResult_      = 0;
    lastAckMs_          = 0;
    lastAckTracked_     = 0;

    // Load current sensor mappings from config into working copies
    const auto& cfg = config_store::get();
    memcpy(inaMap_,  cfg.ina226Map,      config_store::NUM_INA226_CH);
    memcpy(tempMap_, cfg.tempSensorMap,  config_store::NUM_TEMP_SENS);
    brightnessEdit_  = display_backlight::current();
    brightnessDirty_ = false;

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
    lastFrameTimeMs_ = frameTimeMs;   // cached for touch-handler timestamping
    // Cache service mode data for fault viewer / module control
    faultBits_    = data.service().faultMask;
    enabledBits_  = data.service().enabledMask;
    disabledBits_ = data.service().disabledMask;

    // Relay status (shown on main menu header)
    relayStatus_ = data.heartbeat().relayStatus;

    // Cache the system state so the relay-control page can show why override
    // is locked (override is STANDBY-only by safety design on the STM32).
    sysStateRaw_ = static_cast<uint8_t>(data.heartbeat().systemState);

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

        // Module Enable/Disable confirmation window (FASE 2 §1).  Latched intent
        // from the touch handler is stamped here so the timeout uses the injected
        // frameTimeMs (deterministic, no millis() in the UI path).
        if (modulePendingArm_) {
            modulePendingArm_ = false;
            modulePendingMs_  = frameTimeMs;
        }
        if (modulePendingId_ >= 0 &&
            (frameTimeMs - modulePendingMs_) >= MODULE_CONFIRM_TIMEOUT_MS) {
            modulePendingId_ = -1;
            needsRedraw_ = true;
        }
    }

    // Relay control STANDBY notice window (FASE 2 §2).  Latched intent from the
    // touch handler is stamped here; the notice auto-clears after the timeout.
    if (currentMenu_ == SubMenu::RELAY_CONTROL) {
        if (relayStandbyMsgArm_) {
            relayStandbyMsgArm_ = false;
            relayStandbyMsgMs_  = frameTimeMs;
        }
        if (relayStandbyMsg_ &&
            (frameTimeMs - relayStandbyMsgMs_) >= RELAY_STANDBY_MSG_MS) {
            relayStandbyMsg_ = false;
            needsRedraw_ = true;
        }
    }

    // Factory Defaults confirmation window (FASE 2 §1).  Latched intent from the
    // touch handler is stamped here so the timeout uses the injected frameTimeMs
    // (deterministic, no millis() in the UI path).
    if (currentMenu_ == SubMenu::FACTORY_DEFAULTS) {
        if (factoryPendingArm_) {
            factoryPendingArm_ = false;
            factoryPendingMs_  = frameTimeMs;
        }
        // Auto-cancel a pending confirmation that was not confirmed in time.
        if (factoryPendingIdx_ >= 0 &&
            (frameTimeMs - factoryPendingMs_) >= FACTORY_CONFIRM_TIMEOUT_MS) {
            factoryPendingIdx_ = -1;
            needsRedraw_ = true;
        }
    }

    // Cache pedal calibration telemetry (0x308 + live ADC stability ring)
    if (currentMenu_ == SubMenu::PEDAL_CAL) {
        const vehicle::PedalCalData& pc = data.pedalCal();
        bool changed = false;
        if (pc.timestampMs != 0 && pc.timestampMs != pedalCalLastTs_) {
            pedalCalLastTs_ = pc.timestampMs;
            if (pedalCalFlags_      != pc.flags)        changed = true;
            if (pedalCalRawAdc_     != pc.rawAdc)       changed = true;
            if (pedalCalStoredMin_  != pc.storedMin)    changed = true;
            if (pedalCalStoredMax_  != pc.storedMax)    changed = true;
            if (pedalCalPendingMin_ != pc.pendingMin)   changed = true;
            if (pedalCalPendingMax_ != pc.pendingMax)   changed = true;
            if (pedalCalPercent_    != pc.pedalPercent) changed = true;
            pedalCalFlags_      = pc.flags;
            pedalCalRawAdc_     = pc.rawAdc;
            pedalCalStoredMin_  = pc.storedMin;
            pedalCalStoredMax_  = pc.storedMax;
            pedalCalPendingMin_ = pc.pendingMin;
            pedalCalPendingMax_ = pc.pendingMax;
            pedalCalPercent_    = pc.pedalPercent;
            // Push rawAdc into the stability ring.
            pedalStabRing_[pedalStabHead_] = pc.rawAdc;
            pedalStabHead_ = (pedalStabHead_ + 1U) % PEDAL_STAB_N;
            if (pedalStabCount_ < PEDAL_STAB_N) pedalStabCount_++;
        }
        // Periodic QUERY (every ~500 ms) so the STM32 keeps emitting the
        // 1 s burst while the operator stays on the calibration screen.
        // Backward-compatibility: nodes that ignore 0x308 / 0xF5 are
        // unaffected.  Stops immediately on exit (BACK).
        constexpr unsigned long PEDAL_QUERY_PERIOD_MS = 500;
        if ((frameTimeMs - pedalCalLastQueryMs_) >= PEDAL_QUERY_PERIOD_MS) {
            sendPedalCalOp(can::PEDAL_CAL_OP_QUERY);
            pedalCalLastQueryMs_ = frameTimeMs;
        }
        pedalDataChanged_ = changed;
    }

    // Cache gear power-limits telemetry (0x30D) + track SAVE/RESET ACK
    // (cmdIdLow = 0x10).  This submenu owns its own ACK state machine so it
    // does not collide with MODULE_CONTROL feedback.
    if (currentMenu_ == SubMenu::GEAR_LIMITS) {
        const vehicle::GearLimitsData& gl = data.gearLimits();
        bool changed = false;
        if (gl.timestampMs != 0 && gl.timestampMs != gearLimitsLastTs_) {
            gearLimitsLastTs_ = gl.timestampMs;
            if (gearActiveD2_ != gl.activeD2 ||
                gearActiveD1_ != gl.activeD1 ||
                gearActiveR_  != gl.activeR  ||
                gearActiveRespD2_ != gl.activeRespD2 ||
                gearActiveRespD1_ != gl.activeRespD1 ||
                gearActiveRespR_  != gl.activeRespR) {
                changed = true;
            }
            gearActiveD2_ = gl.activeD2;
            gearActiveD1_ = gl.activeD1;
            gearActiveR_  = gl.activeR;
            gearActiveRespD2_ = gl.activeRespD2;
            gearActiveRespD1_ = gl.activeRespD1;
            gearActiveRespR_  = gl.activeRespR;
            // Until the user starts editing, keep the edit buffer tracking the
            // live applied values so the screen shows reality on entry.
            if (!gearLimitsEditActive_) {
                gearEditD2_ = gl.activeD2;
                gearEditD1_ = gl.activeD1;
                gearEditR_  = gl.activeR;
                gearEditRespD2_ = gl.activeRespD2;
                gearEditRespD1_ = gl.activeRespD1;
                gearEditRespR_  = gl.activeRespR;
            }
        }

        // SERVICE_CMD ACK (0x10) — only meaningful while awaiting a SAVE/RESET.
        const auto& ad = data.ack();
        if (gearLimitsSaveWait_ && ad.cmdIdLow == 0x10 &&
            ad.timestampMs != lastAckTracked_) {
            lastAckTracked_ = ad.timestampMs;
            if (ad.result == can::AckResult::OK) {
                gearLimitsAck_        = GearAck::SAVED;
                gearLimitsEditActive_ = false;   // committed → follow live again
            } else if (ad.result == can::AckResult::BLOCKED_BY_SAFETY) {
                gearLimitsAck_ = GearAck::REJECTED;
            } else if (ad.result == can::AckResult::INVALID) {
                gearLimitsAck_ = GearAck::INVALID;
            } else {
                gearLimitsAck_ = GearAck::REJECTED;
            }
            gearLimitsSaveWait_ = false;
            gearLimitsAckMs_    = frameTimeMs;
            changed             = true;
        }
        // SAVE/RESET timeout — no ACK within the window.
        if (gearLimitsSaveWait_ &&
            (frameTimeMs - gearLimitsSaveSentMs_) > GEAR_SAVE_TIMEOUT_MS) {
            gearLimitsSaveWait_ = false;
            gearLimitsAck_      = GearAck::TIMEOUT;
            gearLimitsAckMs_    = frameTimeMs;
            changed             = true;
        }
        // Auto-clear the status banner.
        if (gearLimitsAck_ != GearAck::NONE &&
            (frameTimeMs - gearLimitsAckMs_) > GEAR_ACK_CLEAR_MS) {
            gearLimitsAck_ = GearAck::NONE;
            changed        = true;
        }
        // Auto-cancel an armed RESTORE confirmation that timed out.
        if (gearLimitsRestoreArm_ &&
            (frameTimeMs - gearLimitsRestoreArmMs_) > GEAR_RESTORE_CONFIRM_MS) {
            gearLimitsRestoreArm_ = false;
            changed               = true;
        }
        // Periodic QUERY (every ~500 ms) so the STM32 keeps emitting the burst
        // while the operator stays on the screen.  pedalCalLastQueryMs_ style.
        constexpr unsigned long GEAR_QUERY_PERIOD_MS = 500;
        if ((frameTimeMs - gearLimitsLastQueryMs_) >= GEAR_QUERY_PERIOD_MS) {
            sendGearLimitOp(can::GEAR_LIMIT_OP_QUERY, 0);
            gearLimitsLastQueryMs_ = frameTimeMs;
        }
        if (changed) { gearLimitsChanged_ = true; needsRedraw_ = true; }
    }

    // Cache encoder calibration telemetry
    if (currentMenu_ == SubMenu::ENCODER_CAL) {
        bool changed = false;
        if (steeringAngle_ != data.steering().angleRaw)    changed = true;
        if (steeringCal_ != data.steering().calibrated)    changed = true;
        steeringAngle_ = data.steering().angleRaw;
        steeringCal_   = data.steering().calibrated;
        encoderDataChanged_ = changed;

        const vehicle::SteeringZData& sz = data.steeringZ();
        bool zChanged = false;
        if (steeringZ_.flags       != sz.flags)       zChanged = true;
        if (steeringZ_.status      != sz.status)      zChanged = true;
        if (steeringZ_.pb5Live     != sz.pb5Live)     zChanged = true;
        if (steeringZ_.zValid      != sz.zValid)      zChanged = true;
        if (steeringZ_.zSlip       != sz.zSlip)       zChanged = true;
        if (steeringZ_.zPulseCount != sz.zPulseCount) zChanged = true;
        if (steeringZ_.zLastPos    != sz.zLastPos)    zChanged = true;
        if (steeringZ_.zOffset     != sz.zOffset)     zChanged = true;
        if (steeringZ_.zLastError  != sz.zLastError)  zChanged = true;
        if (steeringZ_.zTolerance  != sz.zTolerance)  zChanged = true;
        steeringZ_ = sz;
        steerZDataChanged_ = zChanged;

        if (steerZClearPending_ &&
            (frameTimeMs - steerZClearPendingMs_) >= STEER_Z_CLEAR_CONFIRM_MS) {
            steerZClearPending_ = false;
            needsRedraw_ = true;
        }

        constexpr unsigned long STEER_Z_QUERY_PERIOD_MS = 500;
        if ((frameTimeMs - steerZLastQueryMs_) >= STEER_Z_QUERY_PERIOD_MS) {
            sendSteerZOp(can::STEER_Z_OP_QUERY);
            steerZLastQueryMs_ = frameTimeMs;
        }
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

    // Cache INA226/current telemetry for the live diagnostic submenu.
    if (currentMenu_ == SubMenu::INA226_LIVE_DIAG) {
        bool changed = false;

        const auto& cur = data.current();
        for (uint8_t i = 0; i < 4; ++i) {
            const uint16_t raw = cur.raw[i];
            if (inaLiveMotorCurrentRaw_[i] != raw) changed = true;
            inaLiveMotorCurrentRaw_[i] = raw;
        }

        const auto& bat = data.battery();
        if (inaLiveBtCurrentRaw_ != bat.currentRaw) changed = true;
        if (inaLiveBtVoltageRaw_ != bat.voltageRaw) changed = true;
        inaLiveBtCurrentRaw_ = bat.currentRaw;
        inaLiveBtVoltageRaw_ = bat.voltageRaw;

        const auto& id = data.i2cDiag();
        unsigned long ageMs = 0;
        if (id.valid && frameTimeMs >= id.timestampMs) {
            ageMs = frameTimeMs - id.timestampMs;
        }
        const bool stale = id.valid && (ageMs > ENG_I2C_DIAG_STALE_MS);
        const unsigned long ageSec = ageMs / 1000U;

        if (inaLiveMuxPresent_    != id.muxPresent)     changed = true;
        if (inaLiveOkMask_        != id.inaOkMask)      changed = true;
        if (inaLiveExpectedMask_  != id.inaExpectedMask) changed = true;
        if (inaLiveFailCount_     != id.failCount)      changed = true;
        if (inaLiveRecoveryCount_ != id.recoveryCount)  changed = true;
        if (inaLiveValid_         != id.valid)          changed = true;
        if (inaLiveStale_         != stale)             changed = true;
        if (inaLiveLastAgeSec_    != ageSec)            changed = true;

        inaLiveMuxPresent_    = id.muxPresent;
        inaLiveOkMask_        = id.inaOkMask;
        inaLiveExpectedMask_  = id.inaExpectedMask;
        inaLiveFailCount_     = id.failCount;
        inaLiveRecoveryCount_ = id.recoveryCount;
        inaLiveValid_         = id.valid;
        inaLiveStale_         = stale;
        inaLiveAgeMs_         = ageMs;
        inaLiveLastAgeSec_    = ageSec;

        inaLiveDataChanged_ = changed;
    }

    // Cache debounce DWT EMI counters (1 Hz update via 0x306/0x307)
    if (currentMenu_ == SubMenu::DEBOUNCE_DIAG) {
        const auto& dd = data.debounceDiag();
        if (dd.timestampMs != debounceLastTs_) {
            debounceLastTs_ = dd.timestampMs;
            for (uint8_t i = 0; i < 4; ++i) {
                debounceWheelFiltered_[i] = dd.wheelFiltered[i];
            }
            debounceSteerFiltered_ = dd.steerFiltered;
            debounceDataChanged_   = true;
        }

        // Cache CAN/0x309 delivery meta (0x30A, 1 Hz), latest I2C scan (0x30B)
        // and FDCAN dump (0x30C), plus the can_rx per-ID 0x309 counters.
        const auto& cm = data.canMeta();
        const uint32_t rx   = can_rx::rx0x309Count();
        const uint32_t drop = can_rx::dropped0x309Dlc();
        const uint8_t  dlc  = can_rx::last0x309Dlc();
        if (cm.timestampMs != canDiagLastTs_ ||
            rx != rx0x309Count_ || drop != drop0x309Dlc_ || dlc != last0x309Dlc_) {
            canDiagLastTs_  = cm.timestampMs;
            canMeta_        = cm;
            i2cScan_        = data.i2cScan();
            fdcanDiag_      = data.fdcanDiag();
            rx0x309Count_   = rx;
            drop0x309Dlc_   = drop;
            last0x309Dlc_   = dlc;
            canDiagChanged_ = true;
        }

        // ---- RUN I2C SCAN feedback state machine ----
        // Latched intent from the touch handler is processed here so all
        // timestamps use the injected frameTimeMs (deterministic, no millis()).
        if (scanFbArm_) {
            scanFbArm_ = false;
            scanFbMs_  = frameTimeMs;     // stamp for banner auto-clear
        }
        if (scanArmReply_) {
            scanArmReply_      = false;
            scanAwaitingReply_ = true;
            scanSentMs_        = frameTimeMs;
            scanBaseI2cTs_     = data.i2cScan().timestampMs;
            scanBaseFdcanTs_   = data.fdcanDiag().timestampMs;
        }
        if (scanAwaitingReply_) {
            const auto& sc = data.i2cScan();
            const auto& fd = data.fdcanDiag();
            const bool gotI2c   = sc.valid && sc.timestampMs != scanBaseI2cTs_;
            const bool gotFdcan = fd.valid && fd.timestampMs != scanBaseFdcanTs_;
            if (gotI2c || gotFdcan) {
                // Fresh 0x30B/0x30C reply: drop the transient banner so the
                // live result lines below speak for themselves.
                scanAwaitingReply_ = false;
                scanFb_            = ScanFb::NONE;
                scanFbChanged_     = true;
            } else if (frameTimeMs - scanSentMs_ >= SCAN_TIMEOUT_MS) {
                scanAwaitingReply_ = false;
                scanFb_            = ScanFb::TIMEOUT;
                scanFbMs_          = frameTimeMs;
                scanFbChanged_     = true;
            }
        }
        // Auto-clear a settled banner (FAILED/TIMEOUT held a few seconds) so the
        // line does not linger forever.  SENT persists while awaiting a reply.
        if (!scanAwaitingReply_ && scanFb_ != ScanFb::NONE &&
            (frameTimeMs - scanFbMs_) >= SCAN_FB_CLEAR_MS) {
            scanFb_        = ScanFb::NONE;
            scanFbChanged_ = true;
        }
    }

    // Cache MCP23017 shifter live diagnostic (read-only snapshot — NO I2C here)
    if (currentMenu_ == SubMenu::MCP23017_LIVE) {
        shifter::Diag d = shifter::getDiag();
        unsigned long ageMs = 0;
        if (d.lastValidMs != 0 && frameTimeMs >= d.lastValidMs) {
            ageMs = frameTimeMs - d.lastValidMs;
        }
        // Coarse signature: repaint on any counter/state change or once a
        // second as the age ticks (keeps the TFT writes bounded).
        uint32_t sig = static_cast<uint32_t>(d.gpioRaw)
                     | (static_cast<uint32_t>(d.gearDecoded) << 8)
                     | (static_cast<uint32_t>(d.errorCount)  << 12)
                     | (static_cast<uint32_t>(d.connected ? 1u : 0u) << 20)
                     | (static_cast<uint32_t>(d.recoveryCount & 0x7FF) << 21);
        // Fold in the extended instrumentation so the panel repaints when the
        // GPIOB byte, valid/invalid counters, last pattern or reject reason
        // change (problem statement §1.F/§1.G).
        sig ^= (static_cast<uint32_t>(d.gpiobRaw) << 3)
             ^ (static_cast<uint32_t>(d.lastValidPattern) << 11)
             ^ (static_cast<uint32_t>(d.validReads) << 1)
             ^ (static_cast<uint32_t>(d.invalidReads) << 5)
             ^ (static_cast<uint32_t>(static_cast<uint8_t>(d.rejectReason)) << 17);
        sig ^= static_cast<uint32_t>(ageMs / 1000U);
        if (sig != mcpDiagSig_) {
            mcpDiagSig_     = sig;
            mcpDiag_        = d;
            mcpAgeMs_       = ageMs;
            mcpDataChanged_ = true;
        }
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
            case SubMenu::INA226_LIVE_DIAG: drawInaLiveDiag();         break;
            case SubMenu::DEBOUNCE_DIAG:    drawDebounceDiag();        break;
            case SubMenu::MCP23017_LIVE:    drawMcpLiveDiag();         break;
            case SubMenu::GEAR_LIMITS:      drawGearLimits();          break;
            case SubMenu::BRIGHTNESS:       drawBrightness();          break;
        }
    }

    // Partial redraw for main menu relay status read-out (header right corner)
    if (currentMenu_ == SubMenu::MAIN && relayStatus_ != prevRelayStatus_) {
        const int16_t relX = RELAY_RDX;
        const int16_t relY = RELAY_RDY;
        // 3-bit wire layout (backward-compatible): bit0 reserved, bit1=TRAC, bit2=DIR.
        const bool seqComplete = (relayStatus_ & 0x80U) != 0;
        const bool tracOn   = (relayStatus_ & 0x02U) != 0;
        const bool dirOn    = (relayStatus_ & 0x04U) != 0;

        char buf[40];

        // Clear and redraw relay values (two lines below the static label)
        tft.fillRect(relX, relY + 11, 180, 18, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        snprintf(buf, sizeof(buf), "T:%s D:%s",
                 tracOn ? "ON " : "OFF",
                 dirOn  ? "ON " : "OFF");
        uint16_t col = seqComplete ? ui::COL_GREEN : ui::COL_AMBER;
        tft.setTextColor(col, ui::COL_BG);
        tft.drawString(buf, relX, relY + 11);

        snprintf(buf, sizeof(buf), "SEQ:%s [0x%02X]",
                 seqComplete ? "OK " : "...",
                 relayStatus_);
        tft.setTextColor(seqComplete ? ui::COL_GREEN : ui::COL_AMBER, ui::COL_BG);
        tft.drawString(buf, relX, relY + 21);

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

            // ---- Headline status banner ----------------------------------
            // The clearest possible read: if no fault bit is set, say so in
            // plain language and green; otherwise warn in red.  This removes
            // the ambiguity of staring at a raw hex mask.
            tft.fillRect(0, 60, ui::SCREEN_W, 18, ui::COL_BG);
            RTRACE_FILL_RECT(0, 60, ui::SCREEN_W, 18, ui::COL_BG);
            const char* statusTxt = faultBits_ ? "ACTIVE FAULTS PRESENT" : "NO ACTIVE FAULTS";
            uint16_t statusCol = faultBits_ ? ui::COL_RED : ui::COL_GREEN;
            tft.setTextColor(statusCol, ui::COL_BG);
            tft.setTextSize(1);
            tft.setTextDatum(MC_DATUM);
            tft.drawString(statusTxt, ui::SCREEN_W / 2, 68);
            RTRACE_TEXT(ui::SCREEN_W / 2, 68, statusTxt,
                        statusCol, ui::COL_BG, 1, MC_DATUM);
            tft.setTextDatum(TL_DATUM);

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

    // Partial redraw for pedal calibration (live 0x308 telemetry values)
    if (currentMenu_ == SubMenu::PEDAL_CAL && pedalDataChanged_) {
        pedalDataChanged_ = false;

        char buf[ui::FMT_BUF_LARGE];

        RTRACE_SET_LAYER(2);
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        // Local stability check — last PEDAL_STAB_N rawAdc samples within
        // PEDAL_STAB_LOCAL_TOL counts of each other.  This is a UI hint
        // only; the actual stability gate runs server-side on STM32
        // synchronously during CAPTURE with the same tolerance.
        static constexpr uint16_t PEDAL_STAB_LOCAL_TOL = 8;
        bool stable = false;
        if (pedalStabCount_ >= PEDAL_STAB_N) {
            uint16_t mn = pedalStabRing_[0], mx = pedalStabRing_[0];
            for (uint8_t k = 1; k < PEDAL_STAB_N; ++k) {
                if (pedalStabRing_[k] < mn) mn = pedalStabRing_[k];
                if (pedalStabRing_[k] > mx) mx = pedalStabRing_[k];
            }
            stable = ((uint32_t)(mx - mn) <= PEDAL_STAB_LOCAL_TOL);
        }

        const bool plausible   = (pedalCalFlags_ & 0x20U) != 0;
        const bool safety_ok   = (pedalCalFlags_ & 0x10U) != 0;
        const bool stored_ok   = (pedalCalFlags_ & 0x08U) != 0;
        const bool valid_pair  = (pedalCalFlags_ & 0x04U) != 0;
        const bool have_min    = (pedalCalFlags_ & 0x01U) != 0;
        const bool have_max    = (pedalCalFlags_ & 0x02U) != 0;

        // ---- Left column (live) ----
        // Raw ADC
        tft.fillRect(110, 55, 130, 16, ui::COL_BG);
        snprintf(buf, sizeof(buf), "%u", (unsigned)pedalCalRawAdc_);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString(buf, 110, 55);

        // Pedal %
        tft.fillRect(110, 80, 130, 16, ui::COL_BG);
        snprintf(buf, sizeof(buf), "%u%%", (unsigned)pedalCalPercent_);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString(buf, 110, 80);

        // Stable yes/no
        tft.fillRect(110, 105, 130, 16, ui::COL_BG);
        tft.setTextColor(stable ? ui::COL_GREEN : ui::COL_AMBER, ui::COL_BG);
        tft.drawString(stable ? "YES" : "no", 110, 105);

        // Plausible
        tft.fillRect(110, 130, 130, 16, ui::COL_BG);
        tft.setTextColor(plausible ? ui::COL_GREEN : ui::COL_RED, ui::COL_BG);
        tft.drawString(plausible ? "YES" : "NO", 110, 130);

        // Safety gate
        tft.fillRect(110, 155, 130, 16, ui::COL_BG);
        tft.setTextColor(safety_ok ? ui::COL_GREEN : ui::COL_RED, ui::COL_BG);
        tft.drawString(safety_ok ? "OK" : "BLOCKED", 110, 155);

        // ---- Right column (stored + pending + validation) ----
        // Stored MIN
        tft.fillRect(360, 55, 110, 16, ui::COL_BG);
        if (stored_ok) {
            snprintf(buf, sizeof(buf), "%u", (unsigned)pedalCalStoredMin_);
            tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        } else {
            snprintf(buf, sizeof(buf), "default");
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        }
        tft.drawString(buf, 360, 55);

        // Stored MAX
        tft.fillRect(360, 80, 110, 16, ui::COL_BG);
        if (stored_ok) {
            snprintf(buf, sizeof(buf), "%u", (unsigned)pedalCalStoredMax_);
            tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        } else {
            snprintf(buf, sizeof(buf), "default");
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        }
        tft.drawString(buf, 360, 80);

        // Pending MIN
        tft.fillRect(360, 105, 110, 16, ui::COL_BG);
        if (have_min) {
            snprintf(buf, sizeof(buf), "%u", (unsigned)pedalCalPendingMin_);
            tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        } else {
            snprintf(buf, sizeof(buf), "--");
            tft.setTextColor(ui::COL_DARK_GRAY, ui::COL_BG);
        }
        tft.drawString(buf, 360, 105);

        // Pending MAX
        tft.fillRect(360, 130, 110, 16, ui::COL_BG);
        if (have_max) {
            snprintf(buf, sizeof(buf), "%u", (unsigned)pedalCalPendingMax_);
            tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        } else {
            snprintf(buf, sizeof(buf), "--");
            tft.setTextColor(ui::COL_DARK_GRAY, ui::COL_BG);
        }
        tft.drawString(buf, 360, 130);

        // Validation
        tft.fillRect(360, 155, 110, 16, ui::COL_BG);
        const char* v_text;
        uint16_t    v_col;
        if (!have_min || !have_max) {
            v_text = "incomplete";
            v_col  = ui::COL_GRAY;
        } else if (valid_pair) {
            v_text = "OK";
            v_col  = ui::COL_GREEN;
        } else {
            v_text = "OUT OF RANGE";
            v_col  = ui::COL_RED;
        }
        tft.setTextColor(v_col, ui::COL_BG);
        tft.drawString(v_text, 360, 155);

        // ---- SAVE button colour reflects validation + safety gate ----
        const bool save_enabled = valid_pair && safety_ok;
        uint16_t   save_fill    = save_enabled ? ui::COL_GREEN : ui::COL_DARK_GRAY;
        tft.fillRect(PED_BTN_SAVE_X, PED_BTN_Y, PED_BTN_W, PED_BTN_H, save_fill);
        tft.drawRect(PED_BTN_SAVE_X, PED_BTN_Y, PED_BTN_W, PED_BTN_H, ui::COL_GRAY);
        tft.setTextColor(ui::COL_WHITE, save_fill);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("SAVE",
                       PED_BTN_SAVE_X + PED_BTN_W / 2,
                       PED_BTN_Y + PED_BTN_H / 2);
        tft.setTextDatum(TL_DATUM);
    }

    // Partial redraw for encoder calibration (live steering angle gauge + Z diagnostic)
    if (currentMenu_ == SubMenu::ENCODER_CAL &&
        (encoderDataChanged_ || steerZDataChanged_)) {
        encoderDataChanged_ = false;
        steerZDataChanged_ = false;

        char buf[ui::FMT_BUF_LARGE];
        const int16_t cx = ECAL_GAUGE_CX;
        const int16_t cy = ECAL_GAUGE_CY;
        const int16_t R  = ECAL_GAUGE_R;
        const float   d2r = 3.14159265f / 180.0f;

        // Real road-wheel angle (0.1°, clamped to firmware envelope ±54°).
        int16_t wheelTenth = ui::clampRoadWheelTenths(steeringAngle_);
        // Reconstructed steering-wheel (column) angle, clamped for display.
        int16_t colDeg = ui::steeringWheelDeg(wheelTenth);
        if (colDeg >  350) colDeg =  350;
        if (colDeg < -350) colDeg = -350;

        // Turn-intensity colour shared across the gauge + read-out.
        int16_t absWheelDeg = (wheelTenth < 0 ? -wheelTenth : wheelTenth) / 10;
        uint16_t turnCol = (absWheelDeg <= 3)  ? ui::COL_GREEN
                         : (absWheelDeg <= 30) ? ui::COL_AMBER
                                               : ui::COL_ORANGE;

        RTRACE_SET_LAYER(2);

        // ---- Re-clear the gauge interior (erases old needle/arc/value) ----
        int16_t rIn = R - 6;
        tft.fillCircle(cx, cy, rIn, ui::COL_DIAL_FACE);
        RTRACE_FILL_CIRCLE(cx, cy, rIn, ui::COL_DIAL_FACE);

        // Redraw the static ticks/labels the clear just erased.
        drawEncoderGaugeTicks();

        // ---- Progressive arc from top, proportional to the column angle ----
        float visEnd = static_cast<float>(colDeg) * 150.0f / 350.0f;
        int16_t step = (visEnd < 0) ? -3 : 3;
        int16_t rA1 = R - 10, rA2 = R - 6;
        for (int16_t v = 0; (step > 0) ? (v <= static_cast<int16_t>(visEnd))
                                       : (v >= static_cast<int16_t>(visEnd)); v += step) {
            float a = (static_cast<float>(v) - 90.0f) * d2r;
            int16_t x1 = cx + static_cast<int16_t>(cosf(a) * rA1);
            int16_t y1 = cy + static_cast<int16_t>(sinf(a) * rA1);
            int16_t x2 = cx + static_cast<int16_t>(cosf(a) * rA2);
            int16_t y2 = cy + static_cast<int16_t>(sinf(a) * rA2);
            tft.drawLine(x1, y1, x2, y2, turnCol);
        }
        RTRACE_LINE(cx, cy, cx, cy - rA2, turnCol);

        // ---- Needle (direction arrow) toward the column angle ----
        float na  = (visEnd - 90.0f) * d2r;
        float ca = cosf(na), sa = sinf(na);
        int16_t nx = cx + static_cast<int16_t>(ca * (R - 22));
        int16_t ny = cy + static_cast<int16_t>(sa * (R - 22));
        // Perpendicular base for a tapered needle.
        int16_t px = static_cast<int16_t>(-sa * 5.0f);
        int16_t py = static_cast<int16_t>( ca * 5.0f);
        tft.fillTriangle(cx + px, cy + py, cx - px, cy - py, nx, ny, turnCol);
        RTRACE_LINE(cx, cy, nx, ny, turnCol);

        // Hub.
        tft.fillCircle(cx, cy, 9, ui::COL_RIM);
        tft.fillCircle(cx, cy, 6, ui::COL_DIAL_FACE);
        tft.fillCircle(cx, cy, 2, turnCol);
        RTRACE_FILL_CIRCLE(cx, cy, 9, ui::COL_RIM);

        // ---- Centre value inside the gauge (+128°) ----
        snprintf(buf, sizeof(buf), "%+d\xC2\xB0", colDeg);
        tft.setTextDatum(MC_DATUM);
        tft.setTextSize(2);
        tft.setTextColor(turnCol, ui::COL_DIAL_FACE);
        tft.drawString(buf, cx, cy + R - 26);
        RTRACE_TEXT(cx, cy + R - 26, buf, turnCol, ui::COL_DIAL_FACE, 2, MC_DATUM);

        // ---- Right panel: STEERING WHEEL big value ----
        tft.setTextDatum(TL_DATUM);
        tft.fillRect(ECAL_PANEL_X, 82, 200, 30, ui::COL_BG);
        snprintf(buf, sizeof(buf), "%+d\xC2\xB0", colDeg);
        tft.setTextColor(turnCol, ui::COL_BG);
        tft.setTextSize(3);
        tft.drawString(buf, ECAL_PANEL_X, 84);
        RTRACE_TEXT(ECAL_PANEL_X, 84, buf, turnCol, ui::COL_BG, 3, TL_DATUM);

        // ---- Right panel: WHEEL ANGLE real value ----
        int16_t aw = (wheelTenth < 0 ? -wheelTenth : wheelTenth);
        tft.fillRect(ECAL_PANEL_X, 154, 200, 26, ui::COL_BG);
        snprintf(buf, sizeof(buf), "%s%d.%d\xC2\xB0",
                 wheelTenth < 0 ? "-" : "+", aw / 10, aw % 10);
        tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        tft.setTextSize(2);
        tft.drawString(buf, ECAL_PANEL_X, 156);
        RTRACE_TEXT(ECAL_PANEL_X, 156, buf, ui::COL_CYAN, ui::COL_BG, 2, TL_DATUM);

        // ---- Existing encoder calibration state (compact, above Z block) ----
        tft.fillRect(ECAL_PANEL_X, 190, 205, 12, ui::COL_BG);
        RTRACE_FILL_RECT(ECAL_PANEL_X, 190, 205, 12, ui::COL_BG);
        snprintf(buf, sizeof(buf), "ENC CAL: %s", steeringCal_ ? "CALIBRATED" : "NOT CAL");
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
        tft.setTextColor(steeringCal_ ? ui::COL_GREEN : ui::COL_RED, ui::COL_BG);
        tft.drawString(buf, ECAL_PANEL_X, 190);
        RTRACE_TEXT(ECAL_PANEL_X, 190, buf,
                    steeringCal_ ? ui::COL_GREEN : ui::COL_RED,
                    ui::COL_BG, 1, TL_DATUM);

        // ---- Steering Z diagnostic text block ----
        const int16_t zx = ECAL_PANEL_X;
        const int16_t zy = 204;
        const int16_t zlh = 12;
        tft.fillRect(zx, zy, 205, 72, ui::COL_BG);
        RTRACE_FILL_RECT(zx, zy, 205, 72, ui::COL_BG);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);

        tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
        tft.drawString("STEERING Z CENTER", zx, zy);
        RTRACE_TEXT(zx, zy, "STEERING Z CENTER",
                    ui::COL_AMBER, ui::COL_BG, 1, TL_DATUM);

        const char* zStatus = "NOT CALIBRATED";
        uint16_t zStatusCol = ui::COL_GRAY;
        switch (steeringZ_.status) {
            case 1: zStatus = "OK";              zStatusCol = ui::COL_GREEN; break;
            case 2: zStatus = "Z NOT SEEN";      zStatusCol = ui::COL_AMBER; break;
            case 3: zStatus = "Z OUT OF WINDOW"; zStatusCol = ui::COL_RED;   break;
            case 4: zStatus = "MECH OFFSET";     zStatusCol = ui::COL_RED;   break;
            case 0: default: break;
        }

        snprintf(buf, sizeof(buf), "PB5: %s", steeringZ_.pb5Live ? "ACTIVE" : "--");
        tft.setTextColor(steeringZ_.pb5Live ? ui::COL_GREEN : ui::COL_GRAY, ui::COL_BG);
        tft.drawString(buf, zx, zy + zlh);
        RTRACE_TEXT(zx, zy + zlh, buf,
                    steeringZ_.pb5Live ? ui::COL_GREEN : ui::COL_GRAY,
                    ui::COL_BG, 1, TL_DATUM);

        snprintf(buf, sizeof(buf), "Z STATUS: %s", zStatus);
        tft.setTextColor(zStatusCol, ui::COL_BG);
        tft.drawString(buf, zx, zy + 2 * zlh);
        RTRACE_TEXT(zx, zy + 2 * zlh, buf, zStatusCol, ui::COL_BG, 1, TL_DATUM);

        snprintf(buf, sizeof(buf), "Z PULSES: %u", (unsigned)steeringZ_.zPulseCount);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString(buf, zx, zy + 3 * zlh);
        RTRACE_TEXT(zx, zy + 3 * zlh, buf, ui::COL_WHITE, ui::COL_BG, 1, TL_DATUM);

        snprintf(buf, sizeof(buf), "Z POS: %d  OFF: %d",
                 (int)steeringZ_.zLastPos, (int)steeringZ_.zOffset);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString(buf, zx, zy + 4 * zlh);
        RTRACE_TEXT(zx, zy + 4 * zlh, buf, ui::COL_WHITE, ui::COL_BG, 1, TL_DATUM);

        snprintf(buf, sizeof(buf), "Z CAL: %s  SLIP: %s  TOL: %u",
                 steeringZ_.zValid ? "YES" : "NO",
                 steeringZ_.zSlip ? "YES" : "NO",
                 (unsigned)steeringZ_.zTolerance);
        uint16_t zCalCol = steeringZ_.zSlip ? ui::COL_RED
                          : (steeringZ_.zValid ? ui::COL_GREEN : ui::COL_GRAY);
        tft.setTextColor(zCalCol, ui::COL_BG);
        tft.drawString(buf, zx, zy + 5 * zlh);
        RTRACE_TEXT(zx, zy + 5 * zlh, buf, zCalCol, ui::COL_BG, 1, TL_DATUM);
    }

    // Partial redraw for INA226/current live diagnostic values.
    if (currentMenu_ == SubMenu::INA226_LIVE_DIAG && inaLiveDataChanged_) {
        inaLiveDataChanged_ = false;

        auto fmtA = [](char* out, size_t n, uint16_t raw) {
            snprintf(out, n, "%u.%u A",
                     (unsigned)(raw / 100U),
                     (unsigned)((raw / 10U) % 10U));
        };
        auto fmtV = [](char* out, size_t n, uint16_t raw) {
            snprintf(out, n, "%u.%u V",
                     (unsigned)(raw / 100U),
                     (unsigned)((raw / 10U) % 10U));
        };

        char buf[ui::FMT_BUF_LARGE];
        const int16_t x = 10;
        const int16_t y0 = 44;
        const int16_t lh = 16;

        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);
        tft.fillRect(0, y0, ui::SCREEN_W, BACK_Y - y0 - 4, ui::COL_BG);

        static const char* const chNames[4] = {"CH0 FL", "CH1 FR", "CH2 RL", "CH3 RR"};
        for (uint8_t i = 0; i < 4; ++i) {
            char aBuf[20];
            fmtA(aBuf, sizeof(aBuf), inaLiveMotorCurrentRaw_[i]);
            snprintf(buf, sizeof(buf), "%s: %s", chNames[i], aBuf);
            tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
            tft.drawString(buf, x, y0 + i * lh);
        }

        char btABuf[20];
        char btVBuf[20];
        fmtA(btABuf, sizeof(btABuf), inaLiveBtCurrentRaw_);
        fmtV(btVBuf, sizeof(btVBuf), inaLiveBtVoltageRaw_);
        tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        char btLineBuf[52]; // "CH4 BT: "(8) + btABuf(≤19) + "   "(3) + btVBuf(≤19) + NUL = 50
        snprintf(btLineBuf, sizeof(btLineBuf), "CH4 BT: %s   %s", btABuf, btVBuf);
        tft.drawString(btLineBuf, x, y0 + 4 * lh);

        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("CH5 ST: --.- A (n/d)", x, y0 + 5 * lh);
        // CH5 steering current is not present on CAN today (0x201/0x207/0x309 only).
        // Could be added in the future by extending 0x201 or adding a new ID.

        const int16_t bx = 10;
        const int16_t by = y0 + 6 * lh + 4;
        tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
        tft.drawString("BATTERY INA DIAG", bx, by);

        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        snprintf(buf, sizeof(buf), "BT volts = %s", btVBuf);
        tft.drawString(buf, bx, by + lh);
        snprintf(buf, sizeof(buf), "BT amps  = %s", btABuf);
        tft.drawString(buf, bx, by + 2 * lh);

        if (!inaLiveValid_) {
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            tft.drawString("BT INA   = N/D", bx, by + 3 * lh);
            tft.drawString("BT expected = N/D", bx, by + 4 * lh);
        } else {
            const bool btOk = (inaLiveOkMask_ & (1U << 4)) != 0;
            const bool btExpected = (inaLiveExpectedMask_ & (1U << 4)) != 0;
            tft.setTextColor(btOk ? ui::COL_GREEN : ui::COL_RED, ui::COL_BG);
            snprintf(buf, sizeof(buf), "BT INA   = %s", btOk ? "OK" : "FAIL");
            tft.drawString(buf, bx, by + 3 * lh);

            tft.setTextColor(btExpected ? ui::COL_GREEN : ui::COL_AMBER, ui::COL_BG);
            snprintf(buf, sizeof(buf), "BT expected = %s", btExpected ? "YES" : "NO");
            tft.drawString(buf, bx, by + 4 * lh);
        }

        // Battery INA RAW context (problem statement §2.C/§2.E).
        // The ESP32 only receives the final current over CAN (0.01 A units); the
        // raw INA226 shunt/bus registers live on the STM32 and are not on the
        // bus.  These two constants MIRROR the STM32 firmware values in
        // Core/Inc/project_config.h (INA226_SHUNT_MOHM_BATTERY = 0.75 mΩ) and
        // sensor_manager.c (INA226_SHUNT_LSB_UV = 2.5 µV).  With a 0.75 mΩ shunt
        // the current resolution is 2.5 µV / 0.75 mΩ ≈ 3.33 A per LSB, so any
        // battery current below ~1.6 A rounds to 0 — this is why CH4 reads 0.0 A
        // at rest and is sensing evidence, not a fault.
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("Shunt(BAT)=0.75mOhm  Res~3.33A/LSB", bx, by + 5 * lh);
        snprintf(buf, sizeof(buf), "BT raw(CAN)=%u (0.01A units)",
                 (unsigned)inaLiveBtCurrentRaw_);
        tft.drawString(buf, bx, by + 6 * lh);

        const int16_t gx = 250;
        const int16_t gy = by;
        tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        tft.drawString("I2C TOPOLOGY", gx, gy);

        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        snprintf(buf, sizeof(buf), "INA OK: 0x%02X", (unsigned)inaLiveOkMask_);
        tft.drawString(buf, gx, gy + lh);
        snprintf(buf, sizeof(buf), "EXPECTED: 0x%02X", (unsigned)inaLiveExpectedMask_);
        tft.drawString(buf, gx, gy + 2 * lh);

        snprintf(buf, sizeof(buf), "MUX: %s", inaLiveMuxPresent_ ? "PRESENT" : "MISSING");
        tft.setTextColor(inaLiveMuxPresent_ ? ui::COL_GREEN : ui::COL_RED, ui::COL_BG);
        tft.drawString(buf, gx, gy + 3 * lh);

        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        snprintf(buf, sizeof(buf), "fail:%u rec:%u",
                 (unsigned)inaLiveFailCount_, (unsigned)inaLiveRecoveryCount_);
        tft.drawString(buf, gx, gy + 4 * lh);

        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        if (!inaLiveValid_) {
            tft.drawString("0x309: NO DATA", gx, gy + 5 * lh);
        } else if (inaLiveStale_) {
            snprintf(buf, sizeof(buf), "0x309 STALE %lus", inaLiveAgeMs_ / 1000U);
            tft.drawString(buf, gx, gy + 5 * lh);
        } else {
            snprintf(buf, sizeof(buf), "0x309 age %lus", inaLiveAgeMs_ / 1000U);
            tft.drawString(buf, gx, gy + 5 * lh);
        }

        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("Legend: rest~0A OK | rest~100A suspect shunt/sense", 10, 260);
        tft.drawString("Volts OK + amps wrong => current path/shunt issue", 10, 272);
    }

    // Partial redraw for debounce DWT EMI counters (1 Hz)
    if (currentMenu_ == SubMenu::DEBOUNCE_DIAG && debounceDataChanged_) {
        debounceDataChanged_ = false;

        char buf[ui::FMT_BUF_LARGE];
        const int16_t valX = 320;
        const int16_t rowY0 = 62;
        const int16_t rowH  = 22;

        RTRACE_SET_LAYER(2);
        tft.setTextSize(2);
        tft.setTextDatum(TR_DATUM);

        static const char* const labels[5] = { "FL", "FR", "RL", "RR", "STEER" };
        for (uint8_t i = 0; i < 5; ++i) {
            const int16_t y = rowY0 + i * rowH;
            uint32_t v = (i < 4) ? (uint32_t)debounceWheelFiltered_[i]
                                 : debounceSteerFiltered_;

            // Saturated u16 rendered as "65535+" so users notice the cap.
            // Steer (full u32) shown as decimal up to 9 digits.
            bool wheelSaturated = (i < 4) && (v == 0xFFFFU);

            // Clear value cell
            tft.fillRect(valX - 140, y, 140, 22, ui::COL_BG);

            uint16_t color = ui::COL_GREEN;
            if (v == 0) {
                color = ui::COL_GRAY;
            } else if (wheelSaturated) {
                color = ui::COL_RED;
            } else if (v >= 100) {
                color = ui::COL_AMBER;
            }
            tft.setTextColor(color, ui::COL_BG);

            if (wheelSaturated) {
                snprintf(buf, sizeof(buf), "65535+");
            } else {
                snprintf(buf, sizeof(buf), "%lu", (unsigned long)v);
            }
            tft.drawString(buf, valX, y);
            RTRACE_TEXT(valX, y, buf, color, ui::COL_BG, 2, TR_DATUM);

            // Label (drawn once is fine but repaint cheap; helps detect first frame)
            tft.setTextDatum(TL_DATUM);
            tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
            tft.fillRect(40, y, 100, 22, ui::COL_BG);
            tft.drawString(labels[i], 40, y);
            RTRACE_TEXT(40, y, labels[i], ui::COL_WHITE, ui::COL_BG, 2, TL_DATUM);
            tft.setTextDatum(TR_DATUM);
        }

        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
    }

    // Partial redraw for the CAN/0x309 delivery diagnostic block (1 Hz).
    if (currentMenu_ == SubMenu::DEBOUNCE_DIAG && canDiagChanged_) {
        canDiagChanged_ = false;

        char buf[80];
        const int16_t x   = 10;
        const int16_t y0  = 194;
        const int16_t lh  = 12;   // line height (size-1 font)

        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        // L1: per-ID 0x309 RX counters (audit E/F).
        tft.fillRect(x, y0, ui::SCREEN_W - 2 * x, lh, ui::COL_BG);
        tft.setTextColor(rx0x309Count_ ? ui::COL_GREEN : ui::COL_RED, ui::COL_BG);
        snprintf(buf, sizeof(buf), "0x309 rx=%lu lastDLC=%u dlcDrop=%lu",
                 (unsigned long)rx0x309Count_, (unsigned)last0x309Dlc_,
                 (unsigned long)drop0x309Dlc_);
        tft.drawString(buf, x, y0);

        // L2: STM32 meta counters (audit A–D) from 0x30A.
        tft.fillRect(x, y0 + lh, ui::SCREEN_W - 2 * x, lh, ui::COL_BG);
        if (canMeta_.valid) {
            tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
            snprintf(buf, sizeof(buf),
                     "calls=%u tick=%u txok=%u txerr=%u fifo=%u init=%u",
                     (unsigned)canMeta_.diag309CallCount,
                     (unsigned)canMeta_.tick1000msCount,
                     (unsigned)canMeta_.diag309TxOk,
                     (unsigned)canMeta_.diag309TxErr,
                     (unsigned)canMeta_.txFifoFullDrops,
                     (unsigned)(canMeta_.fdcanInitOk ? 1 : 0));
        } else {
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            snprintf(buf, sizeof(buf), "0x30A meta: no data");
        }
        tft.drawString(buf, x, y0 + lh);

        // L3: latest I2C service-scan result from 0x30B.
        tft.fillRect(x, y0 + 2 * lh, ui::SCREEN_W - 2 * x, lh, ui::COL_BG);
        if (i2cScan_.valid) {
            tft.setTextColor(i2cScan_.muxPresent ? ui::COL_GREEN : ui::COL_AMBER,
                             ui::COL_BG);
            snprintf(buf, sizeof(buf),
                     "scan mux=%u ina=0x%02X sda=%u scl=%u rec=%u/%u",
                     (unsigned)(i2cScan_.muxPresent ? 1 : 0),
                     (unsigned)i2cScan_.inaPresentMask,
                     (unsigned)(i2cScan_.sdaIdleHigh ? 1 : 0),
                     (unsigned)(i2cScan_.sclIdleHigh ? 1 : 0),
                     (unsigned)i2cScan_.recoveryAttempts,
                     (unsigned)i2cScan_.failCount);
        } else {
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            snprintf(buf, sizeof(buf), "scan: tap RUN I2C SCAN");
        }
        tft.drawString(buf, x, y0 + 2 * lh);

        // L4: FDCAN error counters from 0x30C.
        tft.fillRect(x, y0 + 3 * lh, ui::SCREEN_W - 2 * x, lh, ui::COL_BG);
        if (fdcanDiag_.valid) {
            tft.setTextColor((fdcanDiag_.busOff || fdcanDiag_.errorPassive)
                                 ? ui::COL_RED : ui::COL_WHITE, ui::COL_BG);
            snprintf(buf, sizeof(buf),
                     "fdcan tec=%u rec=%u lec=%u %s%s",
                     (unsigned)fdcanDiag_.tec, (unsigned)fdcanDiag_.rec,
                     (unsigned)fdcanDiag_.lastErrorCode,
                     fdcanDiag_.busOff ? "BUSOFF " : "",
                     fdcanDiag_.errorPassive ? "EPASS" : "");
        } else {
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            snprintf(buf, sizeof(buf), "fdcan: tap RUN I2C SCAN");
        }
        tft.drawString(buf, x, y0 + 3 * lh);
    }

    // Partial redraw for the RUN I2C SCAN status banner (immediate feedback).
    if (currentMenu_ == SubMenu::DEBOUNCE_DIAG && scanFbChanged_) {
        scanFbChanged_ = false;
        const int16_t bx = 10;
        const int16_t by = 250;
        tft.fillRect(bx, by, ui::SCREEN_W - 2 * bx, 18, ui::COL_BG);
        const char* msg = nullptr;
        uint16_t    col = ui::COL_GRAY;
        switch (scanFb_) {
            case ScanFb::SENT:    msg = "SCAN CMD SENT";   col = ui::COL_CYAN;  break;
            case ScanFb::FAILED:  msg = "SCAN CMD FAILED"; col = ui::COL_RED;   break;
            case ScanFb::TIMEOUT: msg = "SCAN TIMEOUT";    col = ui::COL_AMBER; break;
            case ScanFb::NONE:
            default:              break;
        }
        if (msg) {
            tft.setTextSize(2);
            tft.setTextDatum(TL_DATUM);
            tft.setTextColor(col, ui::COL_BG);
            tft.drawString(msg, bx, by);
            tft.setTextSize(1);
        }
    }

    // Partial redraw for MCP23017 shifter live diagnostic (~1 Hz / on change).
    if (currentMenu_ == SubMenu::MCP23017_LIVE && mcpDataChanged_) {
        mcpDataChanged_ = false;

        const int16_t x  = 10;
        const int16_t y0 = 44;
        const int16_t lh = 18;
        char buf[64];   // wider than FMT_BUF_LARGE: several diag lines exceed 32 chars

        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);
        tft.fillRect(0, y0, ui::SCREEN_W, BACK_Y - y0 - 4, ui::COL_BG);

        const shifter::Diag& d = mcpDiag_;

        // Address + connection state
        snprintf(buf, sizeof(buf), "Addr: 0x%02X   Init: %s   Online: %s",
                 (unsigned)d.i2cAddr,
                 d.initialized ? "YES" : "NO",
                 d.connected   ? "YES" : "NO");
        tft.setTextColor(d.connected ? ui::COL_GREEN : ui::COL_RED, ui::COL_BG);
        tft.drawString(buf, x, y0);

        // Config registers
        snprintf(buf, sizeof(buf),
                 "IODIRA: 0x%02X (exp 0x0F)   GPPUA: 0x%02X (exp 0x0F)",
                 (unsigned)d.iodirA, (unsigned)d.gppuA);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString(buf, x, y0 + lh);

        // Raw GPIOA + per-pin state (active-low pull-ups: 0xFF = all open / error)
        snprintf(buf, sizeof(buf), "GPIOA raw: 0x%02X", (unsigned)d.gpioRaw);
        tft.setTextColor(d.gpioRaw == 0xFF ? ui::COL_AMBER : ui::COL_CYAN,
                         ui::COL_BG);
        tft.drawString(buf, x, y0 + 2 * lh);

        // Per-pin contact state.  When GPIOA is in error (0xFF) fall back to the
        // last valid pattern so the technician still sees the real wiring.
        // Active-low: bit=0 → contact CLOSED (common→GND); bit=1 → OPEN (pulled up).
        const uint8_t pat = (d.gpioRaw != 0xFF) ? d.gpioRaw : d.lastValidPattern;
        snprintf(buf, sizeof(buf),
                 "Pins: P(0)=%c D2(1)=%c D1(2)=%c R(3)=%c",
                 (pat & 0x01) ? 'H' : 'L',
                 (pat & 0x02) ? 'H' : 'L',
                 (pat & 0x04) ? 'H' : 'L',
                 (pat & 0x08) ? 'H' : 'L');
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString(buf, x, y0 + 3 * lh);

        // Raw GPIOB (unused by the lever — shown for completeness / wiring sanity)
        snprintf(buf, sizeof(buf), "GPIOB raw: 0x%02X", (unsigned)d.gpiobRaw);
        tft.setTextColor(d.gpiobRaw == 0xFF ? ui::COL_AMBER : ui::COL_CYAN,
                         ui::COL_BG);
        tft.drawString(buf, x, y0 + 4 * lh);

        // Decoded gear
        static const char* const gearNames[5] =
            { "PARK", "REVERSE", "NEUTRAL", "FORWARD(D1)", "FORWARD(D2)" };
        const char* gn = (d.gearDecoded < 5) ? gearNames[d.gearDecoded] : "?";
        snprintf(buf, sizeof(buf), "Gear decoded: %u (%s)",
                 (unsigned)d.gearDecoded, gn);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString(buf, x, y0 + 5 * lh);

        // Exact reason the last GPIOA read was rejected (problem statement §1.F/§1.G)
        static const char* const rejNames[4] =
            { "NONE (read OK)", "ADDR_NACK (reg ptr)", "NO_DATA (requestFrom)",
              "BACKOFF (absent)" };
        const uint8_t rj = static_cast<uint8_t>(d.rejectReason);
        const char* rjs = (rj < 4) ? rejNames[rj] : "?";
        snprintf(buf, sizeof(buf), "Last reject: %s", rjs);
        tft.setTextColor(d.rejectReason == shifter::RejectReason::NONE
                             ? ui::COL_GREEN : ui::COL_RED, ui::COL_BG);
        tft.drawString(buf, x, y0 + 6 * lh);

        // Valid / invalid read counters + last valid pattern
        snprintf(buf, sizeof(buf), "Reads OK: %u   BAD: %u   LastPat: 0x%02X",
                 (unsigned)d.validReads, (unsigned)d.invalidReads,
                 (unsigned)d.lastValidPattern);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString(buf, x, y0 + 7 * lh);

        // I2C error + bus-recovery counters
        snprintf(buf, sizeof(buf), "I2C errors: %u   Bus recoveries: %u",
                 (unsigned)d.errorCount, (unsigned)d.recoveryCount);
        tft.setTextColor(d.errorCount ? ui::COL_AMBER : ui::COL_GREEN,
                         ui::COL_BG);
        tft.drawString(buf, x, y0 + 8 * lh);

        // Age of last valid read
        if (d.lastValidMs == 0) {
            snprintf(buf, sizeof(buf), "Last valid: never");
            tft.setTextColor(ui::COL_RED, ui::COL_BG);
        } else {
            snprintf(buf, sizeof(buf), "Last valid: %lu ms ago",
                     (unsigned long)mcpAgeMs_);
            tft.setTextColor(mcpAgeMs_ > 1000UL ? ui::COL_AMBER : ui::COL_GREEN,
                             ui::COL_BG);
        }
        tft.drawString(buf, x, y0 + 9 * lh);
    }

    RTRACE_DUMP_IF_PENDING();
}
// -------------------------------------------------------------------------
// Touch handling
// -------------------------------------------------------------------------
bool EngineeringScreen::handleTouch(int16_t x, int16_t y) {
    // Back button (submenus → main).  On the MAIN menu the same screen corner
    // is occupied by the PAGE 1 nav tile, so EXIT/paging is handled in the
    // MAIN block below; this BACK handler only applies inside submenus.
    if (currentMenu_ != SubMenu::MAIN &&
        x >= BACK_X && x <= BACK_X + BACK_W &&
        y >= BACK_Y && y <= BACK_Y + BACK_H) {
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
        factoryPendingIdx_ = -1;   // cancel any pending factory confirm (FASE 2 §1)
        modulePendingId_   = -1;   // cancel any pending module confirm (FASE 2 §1)
        relayStandbyMsg_   = false;// clear relay STANDBY notice (FASE 2 §2)
        gearLimitsRestoreArm_ = false; // BACK discards gear-limit edits (FASE 2)
        steerZClearPending_ = false;   // cancel steering-Z clear confirmation
        gearLimitsEditActive_ = false;
        gearLimitsSaveWait_   = false;
        if (currentMenu_ == SubMenu::BRIGHTNESS && brightnessDirty_) {
            config_store::flush();
            brightnessDirty_ = false;
        }
        currentMenu_ = SubMenu::MAIN;
        needsRedraw_ = true;
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

    // ---- Main menu (FASE 2 tile layout) -------------------------------
    if (currentMenu_ == SubMenu::MAIN) {
        // Bottom navigation bar: PAGE 1 / PAGE 2 / EXIT.
        if (y >= NAV_Y && y <= NAV_Y + NAV_H) {
            if (x >= NAVP1_X && x <= NAVP1_X + NAVP1_W) {
                if (mainMenuPage_ != 0) { mainMenuPage_ = 0; needsRedraw_ = true; }
                return true;
            }
            if (x >= NAVP2_X && x <= NAVP2_X + NAVP2_W) {
                if (mainMenuPage_ != 1) { mainMenuPage_ = 1; needsRedraw_ = true; }
                return true;
            }
            if (x >= NAVEX_X && x <= NAVEX_X + NAVEX_W) {
                exitRequested_ = true;
                return true;
            }
        }

        // Tile hit-test → resolve the original item index `i`, then run the
        // EXACT same dispatch as the legacy list (logic unchanged).
        const int startItem = mainMenuPage_ * PAGE1_ITEM_COUNT;
        const int endItem   = (mainMenuPage_ == 0) ? PAGE1_ITEM_COUNT
                                                   : NUM_MAIN_ITEMS;
        for (int i = startItem; i < endItem; ++i) {
            const int idx = i - startItem;
            const int col = idx % TILE_COLS;
            const int row = idx / TILE_COLS;
            const int16_t tx = TILE_COL0_X + col * (TILE_W + TILE_GAP);
            const int16_t ty = TILE_ROW0_Y + row * (TILE_H + TILE_GAP);
            if (x >= tx && x <= tx + TILE_W && y >= ty && y <= ty + TILE_H) {
                switch (i) {
                        case 0: currentMenu_ = SubMenu::FAULT_VIEWER;   break;
                        case 1:
                            moduleCtrlPage_ = 0;
                            currentMenu_ = SubMenu::MODULE_CONTROL;
                            break;
                        case 2:
                            // Reset pedal-cal local state so the screen
                            // shows clean defaults until the first 0x308
                            // burst frame arrives.
                            pedalCalLastTs_      = 0;
                            pedalCalLastQueryMs_ = 0;
                            pedalCalFlags_       = 0;
                            pedalStabCount_      = 0;
                            pedalStabHead_       = 0;
                            currentMenu_ = SubMenu::PEDAL_CAL;
                            break;
                        case 3:
                            steerZLastQueryMs_ = 0;
                            steerZClearPending_ = false;
                            steerZDataChanged_ = true;
                            currentMenu_ = SubMenu::ENCODER_CAL;
                            break;
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
                        case 10:
                            // Open INA226/current live diagnostic viewer
                            inaLiveDataChanged_ = true;    // force first paint
                            currentMenu_ = SubMenu::INA226_LIVE_DIAG;
                            break;
                        case 11:
                            // Open Debounce DWT EMI counter viewer
                            debounceDataChanged_ = true;   // force first paint
                            currentMenu_ = SubMenu::DEBOUNCE_DIAG;
                            break;
                        case 12:
                            // Launch persistent touch calibration wizard.
                            // We cannot open the wizard from inside this
                            // screen (it does not own the ScreenManager
                            // flags); ScreenManager polls
                            // consumeTouchCalRequest() each frame and
                            // launches the wizard accordingly.  We exit
                            // the engineering screen so the wizard fully
                            // takes over the display.
                            touchCalRequested_ = true;
                            exitRequested_     = true;
                            break;
                        case 13:
                            // Reset persistent touch calibration: erase
                            // the NVS data AND clear the first_done flag
                            // so the next reboot will re-launch the wizard
                            // automatically.  This is the dedicated reset
                            // path documented in TOUCH_CALIBRATION_SYSTEM.md.
                            touch_calibration::factoryReset();
                            Serial.println(
                                "[ENG] Touch calibration NVS erased; "
                                "wizard will re-arm on next boot");
                            break;
                        case 14:
                            // Open MCP23017 (shifter) live I2C diagnostic
                            mcpDiagSig_     = 0xFFFFFFFFu;  // force first cache
                            mcpDataChanged_ = true;         // force first paint
                            currentMenu_ = SubMenu::MCP23017_LIVE;
                            break;
                        case 15:
                            // Open Gear Power Limits editor.  Reset local
                            // edit state; update() will emit the first QUERY
                            // (pedalCalLastQueryMs_=0 pattern) so the screen
                            // shows the STM32's live values.
                            gearLimitsEditActive_  = false;
                            gearLimitsSaveWait_    = false;
                            gearLimitsAck_         = GearAck::NONE;
                            gearLimitsLastTs_      = 0;
                            gearLimitsLastQueryMs_ = 0;
                            gearLimitsRestoreArm_  = false;
                            gearLimitsShowResponse_ = false;  // start on POWER page
                            gearLimitsChanged_     = true;   // force first paint
                            currentMenu_ = SubMenu::GEAR_LIMITS;
                            break;
                        case 16:
                            brightnessEdit_  = display_backlight::current();
                            brightnessDirty_ = false;
                            currentMenu_ = SubMenu::BRIGHTNESS;
                            break;
                        default:
                            break;
                    }
                    needsRedraw_ = true;
                    return true;
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
            modulePendingId_ = -1;  // changing page cancels a pending confirm
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
                // Only toggle non-critical modules (IDs >= FIRST_NON_CRITICAL).
                // Critical modules 0–3 are never armed nor sent (FASE 2 §1).
                if (i >= FIRST_NON_CRITICAL) {
                    if (modulePendingId_ == (int8_t)i) {
                        // Second tap on the same module — confirmed; send toggle.
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
                        modulePendingId_ = -1;
                    } else {
                        // First tap (or a different row) — arm confirmation, do
                        // NOT send yet.  Latch the time stamp in update().
                        modulePendingId_  = (int8_t)i;
                        modulePendingArm_ = true;
                        Serial.printf("[ENG] Module %u toggle armed (confirm)\n", i);
                    }
                    needsRedraw_ = true;
                }
                return true;
            }
        }
        // Any touch outside the rows / page button cancels a pending confirm.
        if (modulePendingId_ >= 0) {
            modulePendingId_ = -1;
            needsRedraw_ = true;
        }
        return false;
    }

    // Pedal calibration submenu: 4 action buttons + back (back handled above).
    if (currentMenu_ == SubMenu::PEDAL_CAL) {
        if (y >= PED_BTN_Y && y <= PED_BTN_Y + PED_BTN_H) {
            // CAPTURE MIN
            if (x >= PED_BTN_CAPMIN_X && x <= PED_BTN_CAPMIN_X + PED_BTN_W) {
                sendPedalCalOp(can::PEDAL_CAL_OP_CAPTURE_MIN);
                Serial.println("[ENG] Pedal CAPTURE MIN");
                return true;
            }
            // CAPTURE MAX
            if (x >= PED_BTN_CAPMAX_X && x <= PED_BTN_CAPMAX_X + PED_BTN_W) {
                sendPedalCalOp(can::PEDAL_CAL_OP_CAPTURE_MAX);
                Serial.println("[ENG] Pedal CAPTURE MAX");
                return true;
            }
            // SAVE — only forward when the local view believes validation
            // passes (the STM32 re-validates anyway, but blocking the wire
            // when we know it would be rejected reduces user-visible
            // ACK_REJECTED churn).  When the local view shows invalid the
            // tap is silently ignored.
            if (x >= PED_BTN_SAVE_X && x <= PED_BTN_SAVE_X + PED_BTN_W) {
                const bool valid_pair = (pedalCalFlags_ & 0x04U) != 0;
                const bool safety_ok  = (pedalCalFlags_ & 0x10U) != 0;
                if (valid_pair && safety_ok) {
                    sendPedalCalOp(can::PEDAL_CAL_OP_SAVE);
                    Serial.println("[ENG] Pedal SAVE");
                } else {
                    Serial.println("[ENG] Pedal SAVE blocked (local validation)");
                }
                return true;
            }
            // RESET DEFAULTS
            if (x >= PED_BTN_RESET_X && x <= PED_BTN_RESET_X + PED_BTN_W) {
                sendPedalCalOp(can::PEDAL_CAL_OP_RESET_DEFAULTS);
                Serial.println("[ENG] Pedal RESET DEFAULTS");
                return true;
            }
        }
        return false;
    }

    // Encoder calibration submenu: steering-Z diagnostic action buttons.
    if (currentMenu_ == SubMenu::ENCODER_CAL) {
        if (y >= ECAL_Z_BTN_Y && y <= ECAL_Z_BTN_Y + ECAL_Z_BTN_H) {
            if (x >= ECAL_Z_QUERY_X && x <= ECAL_Z_QUERY_X + ECAL_Z_QUERY_W) {
                sendSteerZOp(can::STEER_Z_OP_QUERY);
                steerZClearPending_ = false;
                needsRedraw_ = true;
                Serial.println("[ENG] Steering Z QUERY");
                return true;
            }
            if (x >= ECAL_Z_CAL_X && x <= ECAL_Z_CAL_X + ECAL_Z_CAL_W) {
                sendSteerZOp(can::STEER_Z_OP_CALIBRATE);
                steerZClearPending_ = false;
                needsRedraw_ = true;
                Serial.println("[ENG] Steering Z CALIBRATE");
                return true;
            }
            if (x >= ECAL_Z_CLEAR_X && x <= ECAL_Z_CLEAR_X + ECAL_Z_CLEAR_W) {
                if (steerZClearPending_) {
                    steerZClearPending_ = false;
                    sendSteerZOp(can::STEER_Z_OP_CLEAR);
                    Serial.println("[ENG] Steering Z CLEAR");
                } else {
                    steerZClearPending_ = true;
                    steerZClearPendingMs_ = lastFrameTimeMs_;
                    Serial.println("[ENG] Steering Z CLEAR armed (confirm)");
                }
                needsRedraw_ = true;
                return true;
            }
        }
        if (steerZClearPending_) {
            steerZClearPending_ = false;
            needsRedraw_ = true;
        }
        return false;
    }

    // Gear power-limits editor: per-row −5%/+5% steppers + SAVE / RESTORE.
    // Edits stay local (pending) until SAVE; BACK (handled above) discards.
    if (currentMenu_ == SubMenu::GEAR_LIMITS) {
        // Helper: clamp an edited value to its per-gear range.
        auto clampGear = [](int v, uint8_t lo, uint8_t hi) -> uint8_t {
            if (v < (int)lo) v = lo;
            if (v > (int)hi) v = hi;
            return (uint8_t)v;
        };
        const bool showResp = gearLimitsShowResponse_;
        // PAGE toggle (top-right) — switch POWER <-> RESPONSE group.  Does NOT
        // discard pending edits; both groups are committed together on SAVE.
        if (x >= GL_PAGE_X && x <= GL_PAGE_X + GL_PAGE_W &&
            y >= GL_PAGE_Y && y <= GL_PAGE_Y + GL_PAGE_H) {
            gearLimitsShowResponse_ = !gearLimitsShowResponse_;
            gearLimitsRestoreArm_   = false;
            gearLimitsChanged_      = true;
            needsRedraw_            = true;
            return true;
        }
        // Three editable rows: D2 (0), D1 (1), R (2).
        for (int row = 0; row < 3; ++row) {
            const int16_t ry = GL_ROW0_Y + row * GL_ROW_H;
            if (y < ry || y > ry + GL_ROW_BTN_H) continue;
            const bool minus = (x >= GL_MINUS_X && x <= GL_MINUS_X + GL_STEP_W);
            const bool plus  = (x >= GL_PLUS_X  && x <= GL_PLUS_X  + GL_STEP_W);
            if (!minus && !plus) continue;
            const int delta = plus ? GL_STEP_PCT : -GL_STEP_PCT;
            gearLimitsEditActive_ = true;   // stop tracking live values
            gearLimitsRestoreArm_ = false;  // any edit cancels a RESTORE arm
            if (!showResp) {
                if (row == 0) {
                    gearEditD2_ = clampGear((int)gearEditD2_ + delta,
                                            can::GEAR_LIMIT_D2_MIN_PCT,
                                            can::GEAR_LIMIT_D2_MAX_PCT);
                } else if (row == 1) {
                    gearEditD1_ = clampGear((int)gearEditD1_ + delta,
                                            can::GEAR_LIMIT_D1_MIN_PCT,
                                            can::GEAR_LIMIT_D1_MAX_PCT);
                } else {
                    gearEditR_ = clampGear((int)gearEditR_ + delta,
                                           can::GEAR_LIMIT_R_MIN_PCT,
                                           can::GEAR_LIMIT_R_MAX_PCT);
                }
            } else {
                if (row == 0) {
                    gearEditRespD2_ = clampGear((int)gearEditRespD2_ + delta,
                                                can::GEAR_RESPONSE_D2_MIN_PCT,
                                                can::GEAR_RESPONSE_D2_MAX_PCT);
                } else if (row == 1) {
                    gearEditRespD1_ = clampGear((int)gearEditRespD1_ + delta,
                                                can::GEAR_RESPONSE_D1_MIN_PCT,
                                                can::GEAR_RESPONSE_D1_MAX_PCT);
                } else {
                    gearEditRespR_ = clampGear((int)gearEditRespR_ + delta,
                                               can::GEAR_RESPONSE_R_MIN_PCT,
                                               can::GEAR_RESPONSE_R_MAX_PCT);
                }
            }
            gearLimitsChanged_ = true;
            needsRedraw_       = true;
            return true;
        }
        // Bottom action row: SAVE / RESTORE DEFAULTS.
        if (y >= GL_BTN_Y && y <= GL_BTN_Y + GL_BTN_H) {
            // SAVE — push ALL six pending values (power + response), then
            // commit.  The STM32 re-validates both groups and gates on
            // STANDBY; the single 0x10 ACK drives the banner.
            if (x >= GL_SAVE_X && x <= GL_SAVE_X + GL_BTN_W) {
                sendGearLimitOp(can::GEAR_LIMIT_OP_SET_D2, gearEditD2_);
                sendGearLimitOp(can::GEAR_LIMIT_OP_SET_D1, gearEditD1_);
                sendGearLimitOp(can::GEAR_LIMIT_OP_SET_R,  gearEditR_);
                sendGearLimitOp(can::GEAR_LIMIT_OP_SET_D2_RESPONSE, gearEditRespD2_);
                sendGearLimitOp(can::GEAR_LIMIT_OP_SET_D1_RESPONSE, gearEditRespD1_);
                sendGearLimitOp(can::GEAR_LIMIT_OP_SET_R_RESPONSE,  gearEditRespR_);
                sendGearLimitOp(can::GEAR_LIMIT_OP_SAVE,   0);
                gearLimitsSaveWait_   = true;
                gearLimitsSaveSentMs_ = lastFrameTimeMs_;
                gearLimitsAck_        = GearAck::NONE;
                gearLimitsRestoreArm_ = false;
                Serial.println("[ENG] Gear limits SAVE");
                needsRedraw_ = true;
                return true;
            }
            // RESTORE DEFAULTS — destructive, requires a double-confirm tap.
            if (x >= GL_RESTORE_X && x <= GL_RESTORE_X + GL_BTN_W) {
                if (gearLimitsRestoreArm_) {
                    gearLimitsRestoreArm_ = false;
                    sendGearLimitOp(can::GEAR_LIMIT_OP_RESET_DEFAULTS, 0);
                    gearLimitsSaveWait_   = true;   // RESET also ACKs as 0x10
                    gearLimitsSaveSentMs_ = lastFrameTimeMs_;
                    gearLimitsAck_        = GearAck::NONE;
                    gearLimitsEditActive_ = false;
                    Serial.println("[ENG] Gear limits RESTORE DEFAULTS");
                } else {
                    gearLimitsRestoreArm_   = true;
                    gearLimitsRestoreArmMs_ = lastFrameTimeMs_;
                    Serial.println("[ENG] Gear RESTORE armed (confirm)");
                }
                needsRedraw_ = true;
                return true;
            }
        }
        // Any other touch cancels a pending RESTORE confirmation.
        if (gearLimitsRestoreArm_) {
            gearLimitsRestoreArm_ = false;
            needsRedraw_ = true;
        }
        return false;
    }

    // TFT backlight brightness editor: immediate PWM update, persisted in NVS.
    if (currentMenu_ == SubMenu::BRIGHTNESS) {
        if (y >= BRI_BTN_Y && y <= BRI_BTN_Y + BRI_BTN_H) {
            const bool minus = (x >= BRI_MINUS_X && x <= BRI_MINUS_X + BRI_BTN_W);
            const bool plus  = (x >= BRI_PLUS_X  && x <= BRI_PLUS_X  + BRI_BTN_W);
            if (minus || plus) {
                int next = brightnessEdit_;
                next += plus ? (int)display_backlight::BRIGHTNESS_STEP
                             : -(int)display_backlight::BRIGHTNESS_STEP;
                brightnessEdit_ = display_backlight::apply((uint8_t)next);
                config_store::setBrightness(brightnessEdit_);
                brightnessDirty_ = true;
                needsRedraw_ = true;
                return true;
            }
        }
        return false;
    }

    // Factory defaults submenu: tap a row to arm confirmation; tap the SAME
    // row again (within the time window) to actually send the reset command.
    // This double-tap guard protects destructive actions, especially the
    // 0xFF FACTORY_RESTORE / calibration-erase options (FASE 2 §1).
    if (currentMenu_ == SubMenu::FACTORY_DEFAULTS) {
        if (x >= MENU_X && x <= MENU_X + MENU_W) {
            for (int i = 0; i < NUM_FACTORY_ITEMS; ++i) {
                int16_t btnY = MENU_START_Y + i * MENU_SPACING;
                if (y >= btnY && y <= btnY + MENU_BTN_H) {
                    if (factoryPendingIdx_ == i) {
                        // Second tap on the same row — confirmed; send command.
                        CanFrame frame = {};
                        frame.identifier       = can::SERVICE_CMD;
                        frame.extd             = 0;
                        frame.data_length_code = 2;
                        frame.data[0]          = factoryActions[i];
                        frame.data[1]          = 0;
                        ESP32Can.writeFrame(frame, 0);  // Non-blocking
                        Serial.printf("[ENG] Factory reset cmd 0x%02X sent\n",
                                      factoryActions[i]);
                        factoryPendingIdx_ = -1;
                    } else {
                        // First tap (or a different row) — arm confirmation, do
                        // NOT send yet.  Latch the time stamp in update().
                        factoryPendingIdx_ = (int8_t)i;
                        factoryPendingArm_ = true;
                        Serial.printf("[ENG] Factory reset cmd 0x%02X armed (confirm)\n",
                                      factoryActions[i]);
                    }
                    needsRedraw_ = true;
                    return true;
                }
            }
        }
        // Any touch outside the option rows cancels a pending confirmation.
        if (factoryPendingIdx_ >= 0) {
            factoryPendingIdx_ = -1;
            needsRedraw_ = true;
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
                // FASE 2 §2: relay override is STANDBY-only by safety design on
                // the STM32.  Before mutating the local UI state, verify the
                // cached system state is STANDBY.  Outside STANDBY we refuse to
                // change the visual state and flash an "ONLY IN STANDBY" notice
                // instead, so the buttons never lie about the relay state.
                const bool inStandby =
                    (sysStateRaw_ == static_cast<uint8_t>(can::SystemState::STANDBY));
                if (!inStandby) {
                    relayStandbyMsg_    = true;
                    relayStandbyMsgArm_ = true;
                    needsRedraw_ = true;
                    Serial.println("[ENG] Relay override blocked: not in STANDBY");
                    return true;
                }
                if (i == 0) {
                    // Toggle override enable/disable
                    relayOverrideEnabled_ = !relayOverrideEnabled_;
                    if (!relayOverrideEnabled_) {
                        relayOverrideMask_ = 0;
                    }
                } else {
                    // Toggle individual relay bit (only when override enabled).
                    // Wire layout: bit1=TRACTION(PC11), bit2=STEER_PWR(PC12)
                    // (legacy name "DIRECTION relay" — steering actuator
                    //  power supply; does NOT select drive direction).
                    // i=1→TRAC: 1U<<1=0x02; i=2→STEER_PWR: 1U<<2=0x04.
                    // After the << 1 shift in the CAN encoding below, STM32
                    // decode (mask = (ctl>>1)&0x07, then &0x06) sees the
                    // correct bit-1/bit-2 relay assignment.
                    if (relayOverrideEnabled_) {
                        uint8_t bit = (uint8_t)(1U << i);
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

    // DEBOUNCE / CAN DIAG submenu — single action: RUN I2C SCAN button, which
    // emits SERVICE_CMD 0xF6 so the STM32 runs an active I2C probe and replies
    // with 0x30B (scan) + 0x30C (FDCAN).  Read-only otherwise.
    if (currentMenu_ == SubMenu::DEBOUNCE_DIAG) {
        const int16_t bx = SAVE_X - 70;
        const int16_t bw = BACK_W + 70;
        if (x >= bx && x <= bx + bw && y >= BACK_Y && y <= BACK_Y + BACK_H) {
            sendI2cServiceScan();
            return true;
        }
#if RUNTIME_MONITOR
        // DEBUG OVERLAY toggle button
        if (x >= DBGOVL_X && x <= DBGOVL_X + DBGOVL_W &&
            y >= DBGOVL_Y && y <= DBGOVL_Y + DBGOVL_H) {
            RTMON_OVERLAY_TOGGLE();
            needsRedraw_ = true;   // repaint button label (ON/OFF)
            Serial.printf("[ENG] Debug overlay %s\n",
                          RTMON_OVERLAY_VISIBLE() ? "ON" : "OFF");
            return true;
        }
#endif
        return false;
    }

    return false;
}

// -------------------------------------------------------------------------
// sendI2cServiceScan — emit a SERVICE_CMD (0x110) with byte0 = 0xF6
// (SERVICE_ACTION_I2C_SERVICE).  The STM32 runs an active I2C topology probe
// (mux/INA presence, SDA/SCL idle levels, optional bus recovery) and replies
// with 0x30B (scan report) + 0x30C (FDCAN dump) plus a CMD_ACK.  Read-only
// diagnostic — does not change any STM32 control or safety state.
// -------------------------------------------------------------------------
void EngineeringScreen::sendI2cServiceScan() {
    CanFrame frame = {};
    frame.identifier       = can::SERVICE_CMD;
    frame.extd             = 0;
    frame.data_length_code = 1;
    frame.data[0]          = can::SERVICE_ACTION_I2C_SERVICE;
    const bool ok = ESP32Can.writeFrame(frame, 0);  // Non-blocking

    // Immediate visual confirmation: the previous implementation transmitted
    // the frame silently, so a missing/blank STM32 reply made the button look
    // dead.  Latch the result here and defer all timing to update() (frame-time
    // contract: no millis() in the UI path).
    scanFb_        = ok ? ScanFb::SENT : ScanFb::FAILED;
    scanFbChanged_ = true;     // repaint the banner next draw()
    scanFbArm_     = true;     // update() stamps scanFbMs_ for auto-clear
    scanArmReply_  = ok;       // update() arms the 2 s reply watchdog
    Serial.printf("[ENG] RUN I2C SCAN tap -> 0xF6 tx %s\n", ok ? "OK" : "FAILED");
}

// -------------------------------------------------------------------------
// Draw helpers
// -------------------------------------------------------------------------
void EngineeringScreen::drawMainMenu() {
    RTRACE_BEGIN_SCREEN("eng_main");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // ---- Header --------------------------------------------------------
    // Title (left), page indicator (centre), compact relay read-out (right).
    tft.setTextSize(2);
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("ENGINEERING", 8, 8);
    RTRACE_TEXT(8, 8, "ENGINEERING", ui::COL_AMBER, ui::COL_BG, 2, TL_DATUM);

    char pgBuf[16];
    snprintf(pgBuf, sizeof(pgBuf), "PAGE %u/%u",
             (unsigned)(mainMenuPage_ + 1), (unsigned)MAIN_PAGE_COUNT);
    tft.setTextSize(2);
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(pgBuf, 232, 14);
    RTRACE_TEXT(232, 14, pgBuf, ui::COL_WHITE, ui::COL_BG, 2, MC_DATUM);
    tft.setTextDatum(TL_DATUM);

    // Divider under the header.
    tft.drawFastHLine(0, 32, ui::SCREEN_W, ui::COL_DARK_GRAY);

    // ---- Tiles ---------------------------------------------------------
    const int startItem = mainMenuPage_ * PAGE1_ITEM_COUNT;
    const int endItem   = (mainMenuPage_ == 0) ? PAGE1_ITEM_COUNT
                                               : NUM_MAIN_ITEMS;
    for (int i = startItem; i < endItem; ++i) {
        const int idx = i - startItem;        // 0-based index within the page
        const int col = idx % TILE_COLS;
        const int row = idx / TILE_COLS;
        const int16_t tx = TILE_COL0_X + col * (TILE_W + TILE_GAP);
        const int16_t ty = TILE_ROW0_Y + row * (TILE_H + TILE_GAP);
        const uint16_t accent = tileColor[i];

        // Card background + 2 px accent border.
        tft.fillRoundRect(tx, ty, TILE_W, TILE_H, 6, ui::COL_DARK_GRAY);
        RTRACE_FILL_RECT(tx, ty, TILE_W, TILE_H, ui::COL_DARK_GRAY);
        tft.drawRoundRect(tx, ty, TILE_W, TILE_H, 6, accent);
        tft.drawRoundRect(tx + 1, ty + 1, TILE_W - 2, TILE_H - 2, 5, accent);
        RTRACE_DRAW_RECT(tx, ty, TILE_W, TILE_H, accent);

        // Category icon (top portion of the tile).
        drawTileIcon((uint8_t)i, tx + TILE_W / 2, ty + 22, accent);

        // Two-line caption (text size 2 — never size 1 for tile captions).
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        const bool twoLine = (tileLabel2[i][0] != '\0');
        tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
        if (twoLine) {
            tft.drawString(tileLabel1[i], tx + TILE_W / 2, ty + 46);
            RTRACE_TEXT(tx + TILE_W / 2, ty + 46, tileLabel1[i],
                        ui::COL_WHITE, ui::COL_DARK_GRAY, 2, MC_DATUM);
            tft.drawString(tileLabel2[i], tx + TILE_W / 2, ty + 62);
            RTRACE_TEXT(tx + TILE_W / 2, ty + 62, tileLabel2[i],
                        ui::COL_WHITE, ui::COL_DARK_GRAY, 2, MC_DATUM);
        } else {
            tft.drawString(tileLabel1[i], tx + TILE_W / 2, ty + 54);
            RTRACE_TEXT(tx + TILE_W / 2, ty + 54, tileLabel1[i],
                        ui::COL_WHITE, ui::COL_DARK_GRAY, 2, MC_DATUM);
        }
    }
    tft.setTextDatum(TL_DATUM);

    // ---- Bottom navigation bar (PAGE 1 / PAGE 2 / EXIT) ----------------
    auto drawNavBtn = [&](int16_t bx, int16_t bw, const char* label,
                          bool active, uint16_t accent) {
        const uint16_t fill = active ? accent : ui::COL_DARK_GRAY;
        const uint16_t txt  = active ? ui::COL_BLACK : accent;
        tft.fillRoundRect(bx, NAV_Y, bw, NAV_H, 5, fill);
        RTRACE_FILL_RECT(bx, NAV_Y, bw, NAV_H, fill);
        tft.drawRoundRect(bx, NAV_Y, bw, NAV_H, 5, accent);
        RTRACE_DRAW_RECT(bx, NAV_Y, bw, NAV_H, accent);
        tft.setTextSize(2);
        tft.setTextColor(txt, fill);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(label, bx + bw / 2, NAV_Y + NAV_H / 2);
        RTRACE_TEXT(bx + bw / 2, NAV_Y + NAV_H / 2, label, txt, fill, 2, MC_DATUM);
    };
    drawNavBtn(NAVP1_X, NAVP1_W, "PAGE 1", mainMenuPage_ == 0, ui::COL_CYAN);
    drawNavBtn(NAVP2_X, NAVP2_W, "PAGE 2", mainMenuPage_ == 1, ui::COL_CYAN);
    drawNavBtn(NAVEX_X, NAVEX_W, "EXIT",   false,              ui::COL_AMBER);
    tft.setTextDatum(TL_DATUM);

    // ---- Compact relay status read-out (header right) ------------------
    {
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        tft.drawString("RELAY STATUS", RELAY_RDX, RELAY_RDY);

        const bool seqComplete = (relayStatus_ & 0x80U) != 0;
        const bool tracOn   = (relayStatus_ & 0x02U) != 0;
        const bool dirOn    = (relayStatus_ & 0x04U) != 0;

        char buf[40];
        snprintf(buf, sizeof(buf), "T:%s D:%s",
                 tracOn ? "ON " : "OFF",
                 dirOn  ? "ON " : "OFF");
        uint16_t col = seqComplete ? ui::COL_GREEN : ui::COL_AMBER;
        tft.setTextColor(col, ui::COL_BG);
        tft.drawString(buf, RELAY_RDX, RELAY_RDY + 11);

        snprintf(buf, sizeof(buf), "SEQ:%s [0x%02X]",
                 seqComplete ? "OK " : "...",
                 relayStatus_);
        tft.setTextColor(seqComplete ? ui::COL_GREEN : ui::COL_AMBER, ui::COL_BG);
        tft.drawString(buf, RELAY_RDX, RELAY_RDY + 21);

        prevRelayStatus_ = relayStatus_;
    }
}

// ---------------------------------------------------------------------------
// Procedural category icons (FASE 2).  Drawn with TFT_eSPI primitives only —
// no PROGMEM bitmap arrays, no heap, zero RAM cost (code lives in flash).
// Each icon is centred on (cx,cy) and fits within roughly a 30x26 px box so it
// sits in the upper third of a tile above the two-line caption.
// ---------------------------------------------------------------------------
void EngineeringScreen::drawTileIcon(uint8_t item, int16_t cx, int16_t cy,
                                     uint16_t col) {
    switch (item) {
        case 0:   // Fault Viewer — warning triangle with "!"
            tft.drawTriangle(cx, cy - 12, cx - 13, cy + 10, cx + 13, cy + 10, col);
            tft.drawTriangle(cx, cy - 10, cx - 11, cy + 9,  cx + 11, cy + 9,  col);
            tft.drawFastVLine(cx, cy - 4, 7, col);
            tft.fillRect(cx - 1, cy + 5, 2, 2, col);
            break;

        case 1:   // Module Enable/Disable — ON/OFF toggle switch
            tft.drawRoundRect(cx - 14, cy - 7, 28, 14, 7, col);
            tft.fillCircle(cx + 7, cy, 5, col);
            break;

        case 2:   // Pedal Calibration — accelerator pedal
            tft.fillTriangle(cx - 9, cy + 10, cx + 11, cy - 8, cx + 11, cy + 2, col);
            tft.drawFastVLine(cx - 9, cy + 2, 8, col);
            tft.drawFastHLine(cx - 12, cy + 10, 6, col);
            break;

        case 3:   // Encoder Calibration — steering wheel
            tft.drawCircle(cx, cy, 12, col);
            tft.drawCircle(cx, cy, 11, col);
            tft.fillCircle(cx, cy, 3, col);
            tft.drawFastVLine(cx, cy, 9, col);
            tft.drawLine(cx, cy, cx - 8, cy - 6, col);
            tft.drawLine(cx, cy, cx + 8, cy - 6, col);
            break;

        case 4:   // INA226 Mapping — ammeter ("A" gauge)
        case 10:  // INA226 Live Diag — same ammeter glyph
            tft.drawCircle(cx, cy, 12, col);
            tft.drawLine(cx, cy, cx + 7, cy - 7, col);   // needle
            tft.fillCircle(cx, cy, 2, col);
            tft.drawFastHLine(cx - 3, cy + 7, 6, col);   // base mark
            break;

        case 5:   // Temp Mapping — thermometer
            tft.drawFastVLine(cx - 2, cy - 11, 14, col);
            tft.drawFastVLine(cx + 2, cy - 11, 14, col);
            tft.drawFastHLine(cx - 2, cy - 11, 5, col);
            tft.fillCircle(cx, cy + 7, 5, col);
            tft.fillRect(cx - 1, cy - 2, 2, 9, col);
            break;

        case 6:   // Factory Defaults — circular reset arrow
            tft.drawCircle(cx, cy, 11, col);
            tft.drawCircle(cx, cy, 10, col);
            tft.fillRect(cx - 1, cy - 13, 3, 6, ui::COL_DARK_GRAY);  // gap
            tft.fillTriangle(cx + 2, cy - 13, cx + 9, cy - 11,
                             cx + 2, cy - 7, col);                   // arrowhead
            break;

        case 7:   // DTC Error Log — clipboard
            tft.drawRoundRect(cx - 9, cy - 11, 18, 23, 2, col);
            tft.fillRect(cx - 4, cy - 13, 8, 4, col);    // clip
            tft.drawFastHLine(cx - 5, cy - 3, 10, col);
            tft.drawFastHLine(cx - 5, cy + 2, 10, col);
            tft.drawFastHLine(cx - 5, cy + 7, 6,  col);
            break;

        case 8:   // Maintenance — wrench
            tft.drawLine(cx - 9, cy + 9, cx + 5, cy - 5, col);
            tft.drawLine(cx - 8, cy + 10, cx + 6, cy - 4, col);
            tft.drawCircle(cx + 7, cy - 7, 5, col);
            tft.fillCircle(cx - 8, cy + 9, 2, col);
            break;

        case 9:   // Relay Control — relay (coil box + contact arm)
            tft.drawRect(cx - 12, cy - 8, 14, 16, col);
            for (int16_t yy = cy - 6; yy <= cy + 6; yy += 3)
                tft.drawFastHLine(cx - 12, yy, 14, col);
            tft.drawLine(cx + 2, cy - 6, cx + 12, cy - 10, col);  // armature
            tft.fillCircle(cx + 12, cy + 6, 2, col);              // contact
            break;

        case 11:  // Debounce / CAN Diag — network nodes + links
            tft.fillCircle(cx, cy - 9, 3, col);
            tft.fillCircle(cx - 11, cy + 8, 3, col);
            tft.fillCircle(cx + 11, cy + 8, 3, col);
            tft.drawLine(cx, cy - 9, cx - 11, cy + 8, col);
            tft.drawLine(cx, cy - 9, cx + 11, cy + 8, col);
            tft.drawLine(cx - 11, cy + 8, cx + 11, cy + 8, col);
            break;

        case 12:  // Touch Calibration — target / diana
        case 13:  // Reset Touch Cal — target with reset arrow
            tft.drawCircle(cx, cy, 11, col);
            tft.drawCircle(cx, cy, 6, col);
            tft.fillCircle(cx, cy, 2, col);
            tft.drawFastHLine(cx - 14, cy, 5, col);
            tft.drawFastHLine(cx + 9, cy, 5, col);
            tft.drawFastVLine(cx, cy - 14, 5, col);
            tft.drawFastVLine(cx, cy + 9, 5, col);
            if (item == 13) {  // small reset arrowhead (top-right)
                tft.fillTriangle(cx + 9, cy - 9, cx + 14, cy - 11,
                                 cx + 12, cy - 5, col);
            }
            break;

        case 14:  // MCP23017 Live — IC chip with pins
            tft.drawRect(cx - 8, cy - 8, 16, 16, col);
            tft.fillCircle(cx - 4, cy - 4, 1, col);      // pin-1 dot
            for (int16_t yy = cy - 5; yy <= cy + 5; yy += 5) {
                tft.drawFastHLine(cx - 12, yy, 4, col);  // left pins
                tft.drawFastHLine(cx + 8,  yy, 4, col);  // right pins
            }
            break;

        case 15:  // Gear Power Limits — gear/cog wheel
            tft.drawCircle(cx, cy, 10, col);
            tft.drawCircle(cx, cy, 4, col);
            // four cog teeth (N/S/E/W)
            tft.fillRect(cx - 2, cy - 13, 4, 4, col);
            tft.fillRect(cx - 2, cy + 9,  4, 4, col);
            tft.fillRect(cx - 13, cy - 2, 4, 4, col);
            tft.fillRect(cx + 9,  cy - 2, 4, 4, col);
            break;

        case 16:  // Display Brightness — sun icon
            tft.drawCircle(cx, cy, 8, col);
            tft.fillCircle(cx, cy, 3, col);
            tft.drawFastVLine(cx, cy - 14, 4, col);
            tft.drawFastVLine(cx, cy + 10, 4, col);
            tft.drawFastHLine(cx - 14, cy, 4, col);
            tft.drawFastHLine(cx + 10, cy, 4, col);
            tft.drawLine(cx - 10, cy - 10, cx - 7, cy - 7, col);
            tft.drawLine(cx + 10, cy - 10, cx + 7, cy - 7, col);
            tft.drawLine(cx - 10, cy + 10, cx - 7, cy + 7, col);
            tft.drawLine(cx + 10, cy + 10, cx + 7, cy + 7, col);
            break;

        default:
            tft.drawCircle(cx, cy, 10, col);
            break;
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

    // Headline status — filled in by the dynamic value pass below.  Drawn here
    // only as a reserved row so the static layout is complete.

    // Labels — spelled out so the reader can tell real faults from the
    // informational module enable/disable bitmaps.
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Active faults (mask):", 40, 100);
    RTRACE_TEXT(40, 100, "Active faults (mask):", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("Enabled modules:", 40, 130);
    RTRACE_TEXT(40, 130, "Enabled modules:", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("Disabled modules:", 40, 160);
    RTRACE_TEXT(40, 160, "Disabled modules:", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    // Legend clarifying that only the first row represents real active faults.
    tft.setTextColor(ui::COL_DARK_GRAY, ui::COL_BG);
    tft.drawString("Active faults = real errors now. Enabled/Disabled = module map (info).",
                   40, 190);
    RTRACE_TEXT(40, 190,
                "Active faults = real errors now. Enabled/Disabled = module map (info).",
                ui::COL_DARK_GRAY, ui::COL_BG, 1, TL_DATUM);

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
        const bool pendingConfirm = (modulePendingId_ == (int8_t)i);
        if (pendingConfirm) {
            // Armed for confirmation — prompt the user to tap again (FASE 2 §1).
            statusText = "CONFIRM? TAP";
            statusCol  = ui::COL_AMBER;
        } else if (isCritical) {
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
    } else if (modulePendingId_ >= 0) {
        // Confirmation prompt for a pending module toggle (FASE 2 §1).
        tft.fillRect(60, 265, 360, 14, ui::COL_BG);
        tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
        tft.drawString("Tap the same module again to confirm",
                       ui::SCREEN_W / 2, 270);
    }

    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// drawPedalCalibration — Persistent pedal endpoint calibration
//
// Operator UI for capturing the ADC counts corresponding to the released
// (MIN) and fully-pressed (MAX) accelerator positions and persisting them
// to STM32 flash page 124.  All operator actions are routed through the
// SERVICE_CMD 0x110 / 0xF5 sub-protocol and are gated server-side by the
// STM32 safety state machine (STANDBY + startup_inhibit + pedal<3% +
// plausible + wheels<0.3 km/h).  See docs/CALIBRATION.md.
//
// Live values are driven from the on-demand 0x308 burst that this screen
// requests (QUERY) every ~500 ms while active.  The burst stops 1 s after
// the screen is left → no CAN flooding when the screen is not in use.
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

    // Static labels — left column (live), right column (stored / pending)
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Raw ADC:",    20, 55);
    tft.drawString("Pedal %:",    20, 80);
    tft.drawString("Stable:",     20, 105);
    tft.drawString("Plausible:",  20, 130);
    tft.drawString("Safety gate:",20, 155);

    tft.drawString("Stored MIN:", 260, 55);
    tft.drawString("Stored MAX:", 260, 80);
    tft.drawString("Pending MIN:",260, 105);
    tft.drawString("Pending MAX:",260, 130);
    tft.drawString("Validation:", 260, 155);

    // Hint line
    tft.setTextColor(ui::COL_DARK_GRAY, ui::COL_BG);
    tft.drawString("Release pedal → CAPTURE MIN. Press fully → CAPTURE MAX. Then SAVE.",
                   10, 195);

    // Force initial partial redraw of value cells
    pedalDataChanged_ = true;

    // ---- Action buttons ----
    auto drawBtn = [&](int16_t x, const char* label, uint16_t fill) {
        tft.fillRect(x, PED_BTN_Y, PED_BTN_W, PED_BTN_H, fill);
        RTRACE_FILL_RECT(x, PED_BTN_Y, PED_BTN_W, PED_BTN_H, fill);
        tft.drawRect(x, PED_BTN_Y, PED_BTN_W, PED_BTN_H, ui::COL_GRAY);
        RTRACE_DRAW_RECT(x, PED_BTN_Y, PED_BTN_W, PED_BTN_H, ui::COL_GRAY);
        tft.setTextColor(ui::COL_WHITE, fill);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(label, x + PED_BTN_W / 2, PED_BTN_Y + PED_BTN_H / 2);
        RTRACE_TEXT(x + PED_BTN_W / 2, PED_BTN_Y + PED_BTN_H / 2, label,
                    ui::COL_WHITE, fill, 1, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
    };
    drawBtn(PED_BTN_CAPMIN_X, "CAPTURE MIN", ui::COL_DARK_GRAY);
    drawBtn(PED_BTN_CAPMAX_X, "CAPTURE MAX", ui::COL_DARK_GRAY);
    // SAVE button colour reflects validation status on partial-redraw pass.
    drawBtn(PED_BTN_SAVE_X,   "SAVE",        ui::COL_DARK_GRAY);
    drawBtn(PED_BTN_RESET_X,  "RESET DEF.",  ui::COL_DARK_GRAY);

    // Back button (engineering convention — top-left)
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
// Premium circular gauge (FASE 3.5): a STEERING WHEEL ±350° dial with needle
// + progressive arc, a right-hand read-out (reconstructed wheel angle and the
// real ±54° control angle) and a CALIBRATED / NOT CALIBRATED badge.
// Presentation only — calibration logic / flash storage are untouched.
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
    tft.drawString("STEERING CALIBRATION", ui::SCREEN_W / 2, 15);
    RTRACE_TEXT(ui::SCREEN_W / 2, 15, "STEERING CALIBRATION",
                ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    // ---- Premium circular gauge (STEERING WHEEL ±350°) ----
    const int16_t cx = ECAL_GAUGE_CX;
    const int16_t cy = ECAL_GAUGE_CY;
    const int16_t R  = ECAL_GAUGE_R;

    // Recessed face + metallic outer ring.
    tft.fillCircle(cx, cy, R, ui::COL_DIAL_FACE);
    RTRACE_FILL_CIRCLE(cx, cy, R, ui::COL_DIAL_FACE);
    tft.drawCircle(cx, cy, R,     ui::COL_RIM);
    tft.drawCircle(cx, cy, R - 1, ui::COL_RIM);
    tft.drawCircle(cx, cy, R - 2, ui::COL_DIAL_RING);
    RTRACE_CIRCLE(cx, cy, R, ui::COL_RIM);

    // Major ticks every 87.5° of column travel across the ±150° visual arc,
    // with end + centre labels.  Top (12 o'clock) = 0°.  Shared with the
    // partial redraw so the interior clear never leaves the ticks erased.
    drawEncoderGaugeTicks();
    tft.setTextDatum(TL_DATUM);

    // ---- Right-hand read-out panel (static captions) ----
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("STEERING WHEEL", ECAL_PANEL_X, 64);
    RTRACE_TEXT(ECAL_PANEL_X, 64, "STEERING WHEEL", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("(reconstructed  +-350)", ECAL_PANEL_X, 116);
    RTRACE_TEXT(ECAL_PANEL_X, 116, "(reconstructed  +-350)",
                ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("WHEEL ANGLE", ECAL_PANEL_X, 138);
    RTRACE_TEXT(ECAL_PANEL_X, 138, "WHEEL ANGLE", ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
    tft.drawString("(real control  +-54)", ECAL_PANEL_X, 182);
    RTRACE_TEXT(ECAL_PANEL_X, 182, "(real control  +-54)",
                ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);

    // Steering Z diagnostic section.  Values are painted by the partial redraw
    // path below so live CAN updates refresh without redrawing the whole page.
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("STEERING Z CENTER", ECAL_PANEL_X, 204);
    RTRACE_TEXT(ECAL_PANEL_X, 204, "STEERING Z CENTER",
                ui::COL_AMBER, ui::COL_BG, 1, TL_DATUM);

    // Force initial partial redraw of values
    encoderDataChanged_ = true;
    steerZDataChanged_ = true;

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

    auto zBtn = [](int16_t bx, int16_t bw, const char* lbl, uint16_t fill) {
        tft.fillRect(bx, ECAL_Z_BTN_Y, bw, ECAL_Z_BTN_H, fill);
        RTRACE_FILL_RECT(bx, ECAL_Z_BTN_Y, bw, ECAL_Z_BTN_H, fill);
        tft.drawRect(bx, ECAL_Z_BTN_Y, bw, ECAL_Z_BTN_H, ui::COL_GRAY);
        RTRACE_DRAW_RECT(bx, ECAL_Z_BTN_Y, bw, ECAL_Z_BTN_H, ui::COL_GRAY);
        tft.setTextColor(ui::COL_WHITE, fill);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(lbl, bx + bw / 2, ECAL_Z_BTN_Y + ECAL_Z_BTN_H / 2);
        RTRACE_TEXT(bx + bw / 2, ECAL_Z_BTN_Y + ECAL_Z_BTN_H / 2, lbl,
                    ui::COL_WHITE, fill, 1, MC_DATUM);
    };
    zBtn(ECAL_Z_QUERY_X, ECAL_Z_QUERY_W, "QUERY Z", ui::COL_DARK_GRAY);
    zBtn(ECAL_Z_CAL_X, ECAL_Z_CAL_W, "CALIBRATE Z", ui::COL_DARK_GRAY);
    zBtn(ECAL_Z_CLEAR_X, ECAL_Z_CLEAR_W,
         steerZClearPending_ ? "CONFIRM CLEAR" : "CLEAR Z",
         steerZClearPending_ ? ui::COL_RED : ui::COL_DARK_GRAY);
    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// drawEncoderGaugeTicks — major/minor tick marks + scale labels.
//
// Shared by the static draw and the live partial redraw: the partial redraw
// clears the gauge interior (radius R-6), which would otherwise erase the
// ticks (whose inner ends reach R-14 / R-9) and the centre/end labels.  Call
// this after clearing so the ticks always stay visible.
// -------------------------------------------------------------------------
void EngineeringScreen::drawEncoderGaugeTicks() {
    const int16_t cx = ECAL_GAUGE_CX;
    const int16_t cy = ECAL_GAUGE_CY;
    const int16_t R  = ECAL_GAUGE_R;
    const float   d2r = 3.14159265f / 180.0f;

    for (int t = -4; t <= 4; ++t) {
        float colDeg = t * 87.5f;
        float vis    = colDeg * 150.0f / 350.0f;     // ±150° visual sweep
        float a      = (vis - 90.0f) * d2r;
        bool  major  = (t == -4 || t == 0 || t == 4);
        int16_t r1   = R - (major ? 14 : 9);
        int16_t x1 = cx + static_cast<int16_t>(cosf(a) * r1);
        int16_t y1 = cy + static_cast<int16_t>(sinf(a) * r1);
        int16_t x2 = cx + static_cast<int16_t>(cosf(a) * (R - 4));
        int16_t y2 = cy + static_cast<int16_t>(sinf(a) * (R - 4));
        tft.drawLine(x1, y1, x2, y2, major ? ui::COL_WHITE : ui::COL_DIAL_RING);
        RTRACE_LINE(x1, y1, x2, y2, major ? ui::COL_WHITE : ui::COL_DIAL_RING);
    }
    // Scale end labels.
    tft.setTextSize(1);
    tft.setTextColor(ui::COL_GRAY, ui::COL_DIAL_FACE);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("0", cx, cy - R + 16);
    tft.drawString("-350", cx - R + 22, cy + 20);
    tft.drawString("+350", cx + R - 22, cy + 20);
    RTRACE_TEXT(cx, cy - R + 16, "0", ui::COL_GRAY, ui::COL_DIAL_FACE, 1, MC_DATUM);
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
        const bool pending = (factoryPendingIdx_ == i);
        // Last item (RESET ALL) is red, others are dark gray. A row awaiting
        // confirmation is highlighted amber so the prompt stands out.
        uint16_t bgCol = pending ? ui::COL_AMBER
                       : (i == NUM_FACTORY_ITEMS - 1) ? ui::COL_RED
                                                      : ui::COL_DARK_GRAY;
        uint16_t txtCol = pending ? ui::COL_BLACK : ui::COL_WHITE;

        tft.fillRect(MENU_X, btnY, MENU_W, MENU_BTN_H, bgCol);
        RTRACE_FILL_RECT(MENU_X, btnY, MENU_W, MENU_BTN_H, bgCol);
        tft.drawRect(MENU_X, btnY, MENU_W, MENU_BTN_H, ui::COL_GRAY);
        RTRACE_DRAW_RECT(MENU_X, btnY, MENU_W, MENU_BTN_H, ui::COL_GRAY);

        // While a row is armed, replace its label with the confirmation prompt.
        const char* label = pending ? "CONFIRMAR? PULSA OTRA VEZ" : factoryLabels[i];

        tft.setTextColor(txtCol, bgCol);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(label, MENU_X + MENU_W / 2,
                        btnY + MENU_BTN_H / 2);
        RTRACE_TEXT(MENU_X + MENU_W / 2, btnY + MENU_BTN_H / 2,
                    label, txtCol, bgCol, 1, MC_DATUM);
    }
    tft.setTextDatum(TL_DATUM);

    // Warning text
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(1);
    if (factoryPendingIdx_ >= 0) {
        tft.drawString("CONFIRMAR RESTAURAR? Pulsa la misma opcion otra vez.",
                        20, MENU_START_Y + NUM_FACTORY_ITEMS * MENU_SPACING + 8);
        tft.drawString("Cualquier otro toque cancela la operacion.",
                        20, MENU_START_Y + NUM_FACTORY_ITEMS * MENU_SPACING + 22);
    } else {
        tft.drawString("Tap an option, then tap again to confirm the reset.",
                        20, MENU_START_Y + NUM_FACTORY_ITEMS * MENU_SPACING + 8);
        tft.drawString("Vehicle must be stationary. Reboot may be required.",
                        20, MENU_START_Y + NUM_FACTORY_ITEMS * MENU_SPACING + 22);
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

    // Safety warning when override is active; otherwise explain WHY it is
    // locked.  Relay override is STANDBY-only by safety design on the STM32 —
    // this message makes that explicit and shows the current system state so
    // the user understands the buttons are intentionally inert, not broken.
    if (relayOverrideEnabled_) {
        tft.setTextColor(ui::COL_RED, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("!! MANUAL RELAY CONTROL ACTIVE !!", ui::SCREEN_W / 2, 48);
        tft.setTextDatum(TL_DATUM);
    } else {
        const bool inStandby = (sysStateRaw_ == static_cast<uint8_t>(can::SystemState::STANDBY));
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);

        tft.setTextColor(inStandby ? ui::COL_GRAY : ui::COL_AMBER, ui::COL_BG);
        tft.drawString("Relay override only available in STANDBY",
                       ui::SCREEN_W / 2, 44);

        char stBuf[40];
        snprintf(stBuf, sizeof(stBuf), "Current state: %s", dtcStateName(sysStateRaw_));
        tft.setTextColor(inStandby ? ui::COL_GREEN : ui::COL_RED, ui::COL_BG);
        tft.drawString(stBuf, ui::SCREEN_W / 2, 56);

        tft.setTextDatum(TL_DATUM);
    }

    // Transient "ONLY IN STANDBY" notice — shown after the user taps a relay
    // button while the system is not in STANDBY (FASE 2 §2).  The local UI
    // state was intentionally left unchanged; this explains why.
    if (relayStandbyMsg_) {
        tft.setTextColor(ui::COL_RED, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("ONLY IN STANDBY", ui::SCREEN_W / 2, 68);
        tft.setTextDatum(TL_DATUM);
    }
    // compatible — legacy "DIRECTION" relay = steering actuator power)
    static constexpr int16_t RC_ROW_Y0 = 80;
    static constexpr int16_t RC_ROW_SPC = 36;
    static constexpr int16_t RC_ROW_H = 30;

    static const char* const rowLabels[3] = {
        "Override Enable",
        "TRACTION  (PC11)",
        "STEER PWR (PC12)"
    };

    // Real relay state from CAN heartbeat byte 5 (3-bit wire layout).
    // bit 0 = reserved (always 0), bit 1 = TRAC, bit 2 = STEER_PWR.
    const bool realTrac     = (relayStatus_ & 0x02U) != 0;
    const bool realSteerPwr = (relayStatus_ & 0x04U) != 0;  /* PC12 STEER_PWR (legacy DIR) */
    const bool realState[3] = {
        relayOverrideEnabled_,
        realTrac,
        realSteerPwr
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
            // Wire layout: bit1=TRAC (i=1), bit2=DIR (i=2) — use 1U<<i, not 1U<<(i-1).
            bgCol = (relayOverrideMask_ & (1U << i))
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
        snprintf(buf, sizeof(buf), "T:%s S:%s  SEQ:%s  [0x%02X]",
                 realTrac     ? "ON " : "OFF",
                 realSteerPwr ? "ON " : "OFF",
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

// =========================================================================
// INA226 live diagnostic viewer (INA226_LIVE_DIAG submenu)
//
// Read-only HMI page for live currents + battery INA status from existing CAN
// frames: 0x201 (CH0..CH3 currents), 0x207 (CH4 battery current/voltage),
// 0x309 (MUX + INA masks + fail/recovery counters).
// =========================================================================
void EngineeringScreen::drawInaLiveDiag() {
    RTRACE_BEGIN_SCREEN("eng_ina_live");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("INA226 LIVE DIAG", ui::SCREEN_W / 2, 16);

    tft.setTextSize(1);
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Read-only: 0x201 + 0x207 + 0x309 telemetry", ui::SCREEN_W / 2, 34);
    tft.setTextDatum(TL_DATUM);

    // Dynamic lines are repainted by the partial-redraw branch in draw().
    inaLiveDataChanged_ = true;

    // BACK button (bottom-left)
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_AMBER);
    tft.setTextColor(ui::COL_AMBER, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    tft.setTextDatum(TL_DATUM);
}

// =========================================================================
// Debounce DWT EMI counters viewer (DEBOUNCE_DIAG submenu)
//
// Reads the per-channel filtered counts populated by 0x306/0x307 frames
// into VehicleData.debounceDiag().  Pure read-only display — no commands
// transmitted, no state changes anywhere in the system.
//
// Layout: title, 5 rows (FL/FR/RL/RR/STEER) with right-aligned decimal
// counter, BACK button.  Values are repainted by the partial-redraw branch
// in draw() at the natural 1 Hz CAN cadence.
// =========================================================================
void EngineeringScreen::drawDebounceDiag() {
    RTRACE_BEGIN_SCREEN("eng_debounce");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("DEBOUNCE / CAN DIAG", ui::SCREEN_W / 2, 16);
    RTRACE_TEXT(ui::SCREEN_W / 2, 16, "DEBOUNCE / CAN DIAG",
                ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);

    // Sub-header
    tft.setTextSize(1);
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("DWT 200us pre-filter rejected pulses",
                   ui::SCREEN_W / 2, 36);

    // Column titles (CHANNEL — COUNT)
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
    tft.drawString("CHANNEL", 40, 48);
    tft.setTextDatum(TR_DATUM);
    tft.drawString("FILTERED COUNT", 320, 48);
    tft.setTextDatum(TL_DATUM);

    // Row baseline.  Real values are painted by the partial-redraw branch
    // in draw() as soon as debounceDataChanged_ is set; here we render
    // a "---" placeholder so the screen never looks blank on entry.
    tft.setTextSize(2);
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    static const char* const labels[5] = { "FL", "FR", "RL", "RR", "STEER" };
    const int16_t rowY0 = 62;
    const int16_t rowH  = 22;
    for (uint8_t i = 0; i < 5; ++i) {
        const int16_t y = rowY0 + i * rowH;
        tft.drawString(labels[i], 40, y);
        tft.setTextDatum(TR_DATUM);
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("---", 320, y);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.setTextDatum(TL_DATUM);
    }
    tft.setTextSize(1);

    // ---- CAN / 0x309 delivery diagnostic section (audit A–J) ----
    // Static labels; live values painted by the canDiagChanged_ partial
    // redraw branch in draw().  Header line acts as a visual separator.
    tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
    tft.drawString("CAN 0x309 DELIVERY (0x30A/0x30B/0x30C)", 10, 180);

    // RUN I2C SCAN button (emits SERVICE_CMD 0xF6 → 0x30B + 0x30C).
    tft.fillRect(SAVE_X - 70, BACK_Y, BACK_W + 70, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(SAVE_X - 70, BACK_Y, BACK_W + 70, BACK_H, ui::COL_CYAN);
    tft.setTextColor(ui::COL_CYAN, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("RUN I2C SCAN", SAVE_X - 70 + (BACK_W + 70) / 2,
                   BACK_Y + BACK_H / 2);
    tft.setTextDatum(TL_DATUM);

    // Force a first paint of the CAN diag values on entry.
    canDiagChanged_ = true;
    // Repaint the RUN I2C SCAN status banner too (blank unless a scan is mid-
    // flight when the user re-enters the page).
    scanFbChanged_ = true;

#if RUNTIME_MONITOR
    // DEBUG OVERLAY toggle button — the only entry point for the runtime
    // performance overlay (FPS / frame timing).  Shows current state so the
    // user knows whether it is armed.
    {
        const bool ovlOn = RTMON_OVERLAY_VISIBLE();
        const uint16_t bg = ovlOn ? ui::COL_GREEN : ui::COL_DARK_GRAY;
        const uint16_t fg = ovlOn ? ui::COL_BLACK : ui::COL_CYAN;
        tft.fillRect(DBGOVL_X, DBGOVL_Y, DBGOVL_W, DBGOVL_H, bg);
        tft.drawRect(DBGOVL_X, DBGOVL_Y, DBGOVL_W, DBGOVL_H, ui::COL_CYAN);
        tft.setTextColor(fg, bg);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(ovlOn ? "DEBUG OVERLAY: ON" : "DEBUG OVERLAY: OFF",
                       DBGOVL_X + DBGOVL_W / 2, DBGOVL_Y + DBGOVL_H / 2);
        tft.setTextDatum(TL_DATUM);
    }
#endif

    // BACK button (bottom-left)
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_AMBER);
    tft.setTextColor(ui::COL_AMBER, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// sendPedalCalOp — emit a SERVICE_CMD (0x110) with byte0 = 0xF5
// (SERVICE_ACTION_PEDAL_CAL) and byte1 = sub-opcode.  The STM32 replies
// with a CMD_ACK (0x103, byte0 = 0x10) which is picked up by the
// existing ackData() pipeline and shown in the standard ack feedback
// banner.  Safety gates and validation are enforced server-side.
// -------------------------------------------------------------------------
void EngineeringScreen::sendPedalCalOp(uint8_t op) {
    CanFrame frame = {};
    frame.identifier       = can::SERVICE_CMD;
    frame.extd             = 0;
    frame.data_length_code = 2;
    frame.data[0]          = can::SERVICE_ACTION_PEDAL_CAL;
    frame.data[1]          = op;
    ESP32Can.writeFrame(frame, 0);
}

// -------------------------------------------------------------------------
// sendSteerZOp — emit a SERVICE_CMD (0x110) with byte0 = 0xF8
// (SERVICE_ACTION_STEERING_Z) and byte1 = sub-opcode.  Diagnostic-only HMI;
// STM32 validates PB5/Z state and owns all calibration/clear effects.
// -------------------------------------------------------------------------
void EngineeringScreen::sendSteerZOp(uint8_t op) {
    CanFrame frame = {};
    frame.identifier       = can::SERVICE_CMD;
    frame.extd             = 0;
    frame.data_length_code = 2;
    frame.data[0]          = can::SERVICE_ACTION_STEERING_Z;
    frame.data[1]          = op;
    ESP32Can.writeFrame(frame, 0);
}

// =========================================================================
// GEAR LIMITS editor (GEAR_LIMITS submenu)
//
// Lets the operator view and adjust the per-gear traction power limits the
// STM32 applies in Traction_Update() (D2 / D1 / R, as a percentage of the
// pedal demand).  The screen is purely an HMI: values are staged locally
// and only sent on SAVE.  The STM32 owns range validation, the STANDBY
// safety gate and flash persistence (gear_limits_store.c); the 0x30D burst
// reports the live + pending values back here, and the SERVICE_CMD ACK
// (0x103, byte0 = 0x10) drives the SAVED / REJECTED / INVALID banner.
//
// Defaults & ranges mirror can::GEAR_LIMIT_* (D2 30–100, D1 20–100, R 10–60).
// =========================================================================
void EngineeringScreen::drawGearLimits() {
    RTRACE_BEGIN_SCREEN("eng_gear_limits");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("GEAR LIMITS", ui::SCREEN_W / 2, 15);
    RTRACE_TEXT(ui::SCREEN_W / 2, 15, "GEAR LIMITS",
                ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    const bool showResp = gearLimitsShowResponse_;

    // PAGE toggle button (top-right) — switches POWER <-> RESPONSE group.
    {
        tft.fillRect(GL_PAGE_X, GL_PAGE_Y, GL_PAGE_W, GL_PAGE_H, ui::COL_DARK_GRAY);
        RTRACE_FILL_RECT(GL_PAGE_X, GL_PAGE_Y, GL_PAGE_W, GL_PAGE_H, ui::COL_DARK_GRAY);
        tft.drawRect(GL_PAGE_X, GL_PAGE_Y, GL_PAGE_W, GL_PAGE_H, ui::COL_GRAY);
        RTRACE_DRAW_RECT(GL_PAGE_X, GL_PAGE_Y, GL_PAGE_W, GL_PAGE_H, ui::COL_GRAY);
        const char* pageLbl = showResp ? "POWER >" : "RESPONSE >";
        tft.setTextColor(ui::COL_CYAN, ui::COL_DARK_GRAY);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(pageLbl, GL_PAGE_X + GL_PAGE_W / 2, GL_PAGE_Y + GL_PAGE_H / 2);
        RTRACE_TEXT(GL_PAGE_X + GL_PAGE_W / 2, GL_PAGE_Y + GL_PAGE_H / 2, pageLbl,
                    ui::COL_CYAN, ui::COL_DARK_GRAY, 1, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
    }

    // Group title (which set is being edited)
    tft.setTextColor(showResp ? ui::COL_CYAN : ui::COL_GREEN, ui::COL_BG);
    tft.setTextDatum(ML_DATUM);
    tft.drawString(showResp ? "ACCEL RESPONSE %" : "POWER LIMIT %", 20, 40);
    tft.setTextDatum(TL_DATUM);

    // Sub-header / column legend
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Gear", 20, 52);
    tft.drawString("ACTIVE", 95, 52);
    tft.drawString("EDIT",  170, 52);

    struct Row { const char* name; uint8_t active; uint8_t edit;
                 uint8_t lo; uint8_t hi; };
    const Row powerRows[3] = {
        { "D2", gearActiveD2_, gearEditD2_,
          can::GEAR_LIMIT_D2_MIN_PCT, can::GEAR_LIMIT_D2_MAX_PCT },
        { "D1", gearActiveD1_, gearEditD1_,
          can::GEAR_LIMIT_D1_MIN_PCT, can::GEAR_LIMIT_D1_MAX_PCT },
        { "R",  gearActiveR_,  gearEditR_,
          can::GEAR_LIMIT_R_MIN_PCT,  can::GEAR_LIMIT_R_MAX_PCT },
    };
    const Row responseRows[3] = {
        { "D2", gearActiveRespD2_, gearEditRespD2_,
          can::GEAR_RESPONSE_D2_MIN_PCT, can::GEAR_RESPONSE_D2_MAX_PCT },
        { "D1", gearActiveRespD1_, gearEditRespD1_,
          can::GEAR_RESPONSE_D1_MIN_PCT, can::GEAR_RESPONSE_D1_MAX_PCT },
        { "R",  gearActiveRespR_,  gearEditRespR_,
          can::GEAR_RESPONSE_R_MIN_PCT,  can::GEAR_RESPONSE_R_MAX_PCT },
    };
    const Row* rows = showResp ? responseRows : powerRows;

    char buf[24];
    for (int i = 0; i < 3; ++i) {
        const int16_t ry = GL_ROW0_Y + i * GL_ROW_H;
        const int16_t ty = ry + GL_ROW_BTN_H / 2;

        // Gear label
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.setTextSize(2);
        tft.setTextDatum(ML_DATUM);
        tft.drawString(rows[i].name, 20, ty);
        tft.setTextSize(1);

        // Active value (applied now)
        snprintf(buf, sizeof(buf), "%u%%", (unsigned)rows[i].active);
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.setTextDatum(ML_DATUM);
        tft.drawString(buf, 95, ty);

        // Edited (pending) value — highlight if it differs from active
        const bool diff = (rows[i].edit != rows[i].active);
        snprintf(buf, sizeof(buf), "%u%%", (unsigned)rows[i].edit);
        tft.setTextColor(diff ? ui::COL_AMBER : ui::COL_WHITE, ui::COL_BG);
        tft.setTextSize(2);
        tft.drawString(buf, 165, ty);
        tft.setTextSize(1);

        // −5% / +5% step buttons
        auto stepBtn = [&](int16_t bx, const char* lbl) {
            tft.fillRect(bx, ry, GL_STEP_W, GL_ROW_BTN_H, ui::COL_DARK_GRAY);
            RTRACE_FILL_RECT(bx, ry, GL_STEP_W, GL_ROW_BTN_H, ui::COL_DARK_GRAY);
            tft.drawRect(bx, ry, GL_STEP_W, GL_ROW_BTN_H, ui::COL_GRAY);
            RTRACE_DRAW_RECT(bx, ry, GL_STEP_W, GL_ROW_BTN_H, ui::COL_GRAY);
            tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
            tft.setTextDatum(MC_DATUM);
            tft.setTextSize(2);
            tft.drawString(lbl, bx + GL_STEP_W / 2, ry + GL_ROW_BTN_H / 2);
            tft.setTextSize(1);
            RTRACE_TEXT(bx + GL_STEP_W / 2, ry + GL_ROW_BTN_H / 2, lbl,
                        ui::COL_WHITE, ui::COL_DARK_GRAY, 2, MC_DATUM);
        };
        stepBtn(GL_MINUS_X, "-");
        stepBtn(GL_PLUS_X,  "+");

        // Range hint between the steppers
        snprintf(buf, sizeof(buf), "%u-%u", (unsigned)rows[i].lo,
                 (unsigned)rows[i].hi);
        tft.setTextColor(ui::COL_DARK_GRAY, ui::COL_BG);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(buf, (GL_MINUS_X + GL_STEP_W + GL_PLUS_X) / 2 + 5,
                       ry + GL_ROW_BTN_H / 2);
    }
    tft.setTextDatum(TL_DATUM);

    // ---- Status banner (SAVED / REJECTED / INVALID / TIMEOUT) ----
    const char* statusTxt = nullptr;
    uint16_t    statusCol = ui::COL_GRAY;
    switch (gearLimitsAck_) {
        case GearAck::SAVED:    statusTxt = "SAVED";    statusCol = ui::COL_GREEN;  break;
        case GearAck::REJECTED: statusTxt = "REJECTED (need STANDBY)"; statusCol = ui::COL_RED; break;
        case GearAck::INVALID:  statusTxt = "INVALID";  statusCol = ui::COL_RED;    break;
        case GearAck::TIMEOUT:  statusTxt = "TIMEOUT";  statusCol = ui::COL_YELLOW; break;
        case GearAck::NONE:     default: break;
    }
    if (gearLimitsSaveWait_) { statusTxt = "SAVING..."; statusCol = ui::COL_CYAN; }
    if (statusTxt) {
        tft.setTextColor(statusCol, ui::COL_BG);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(statusTxt, ui::SCREEN_W / 2, 210);
        RTRACE_TEXT(ui::SCREEN_W / 2, 210, statusTxt, statusCol, ui::COL_BG,
                    1, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
    }

    // ---- Action buttons: SAVE / RESTORE DEFAULTS ----
    auto actBtn = [&](int16_t bx, const char* lbl, uint16_t fill) {
        tft.fillRect(bx, GL_BTN_Y, GL_BTN_W, GL_BTN_H, fill);
        RTRACE_FILL_RECT(bx, GL_BTN_Y, GL_BTN_W, GL_BTN_H, fill);
        tft.drawRect(bx, GL_BTN_Y, GL_BTN_W, GL_BTN_H, ui::COL_GRAY);
        RTRACE_DRAW_RECT(bx, GL_BTN_Y, GL_BTN_W, GL_BTN_H, ui::COL_GRAY);
        tft.setTextColor(ui::COL_WHITE, fill);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(lbl, bx + GL_BTN_W / 2, GL_BTN_Y + GL_BTN_H / 2);
        RTRACE_TEXT(bx + GL_BTN_W / 2, GL_BTN_Y + GL_BTN_H / 2, lbl,
                    ui::COL_WHITE, fill, 1, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
    };
    actBtn(GL_SAVE_X, "SAVE", ui::COL_DARK_GRAY);
    actBtn(GL_RESTORE_X,
           gearLimitsRestoreArm_ ? "CONFIRM RESTORE" : "RESTORE DEF.",
           gearLimitsRestoreArm_ ? ui::COL_RED : ui::COL_DARK_GRAY);

    // Back button (engineering convention — top-left)
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

    gearLimitsChanged_ = false;
}

void EngineeringScreen::drawBrightness() {
    RTRACE_BEGIN_SCREEN("eng_brightness");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("DISPLAY BRIGHTNESS", ui::SCREEN_W / 2, 20);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);

    tft.fillRoundRect(BRI_BOX_X, BRI_BOX_Y, BRI_BOX_W, BRI_BOX_H, 8, ui::COL_DARK_GRAY);
    tft.drawRoundRect(BRI_BOX_X, BRI_BOX_Y, BRI_BOX_W, BRI_BOX_H, 8, ui::COL_BLUE);

    char buf[32];
    const uint8_t pct = display_backlight::toPercent(brightnessEdit_);
    snprintf(buf, sizeof(buf), "%u%%", (unsigned)pct);
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextSize(4);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(buf, ui::SCREEN_W / 2, BRI_BOX_Y + 46);

    tft.setTextSize(2);
    snprintf(buf, sizeof(buf), "PWM %u/255", (unsigned)brightnessEdit_);
    tft.drawString(buf, ui::SCREEN_W / 2, BRI_BOX_Y + 96);
    tft.setTextSize(1);

    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    snprintf(buf, sizeof(buf), "RANGE %u%%-%u%%",
             (unsigned)display_backlight::toPercent(display_backlight::BRIGHTNESS_MIN),
             (unsigned)display_backlight::toPercent(display_backlight::BRIGHTNESS_MAX));
    tft.drawString(buf, ui::SCREEN_W / 2, 212);

    auto drawStepButton = [&](int16_t x0, const char* label) {
        tft.fillRect(x0, BRI_BTN_Y, BRI_BTN_W, BRI_BTN_H, ui::COL_DARK_GRAY);
        tft.drawRect(x0, BRI_BTN_Y, BRI_BTN_W, BRI_BTN_H, ui::COL_CYAN);
        tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
        tft.setTextSize(2);
        tft.drawString(label, x0 + BRI_BTN_W / 2, BRI_BTN_Y + BRI_BTN_H / 2);
        tft.setTextSize(1);
    };
    drawStepButton(BRI_MINUS_X, "-");
    drawStepButton(BRI_PLUS_X, "+");

    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_GRAY);
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// sendGearLimitOp — emit a SERVICE_CMD (0x110) with byte0 = 0xF7
// (SERVICE_ACTION_GEAR_LIMITS), byte1 = sub-opcode, byte2 = percent value.
// SET_* ops stage a pending value on the STM32; SAVE validates + persists +
// applies; RESET_DEFAULTS restores factory limits; QUERY triggers the 0x30D
// telemetry burst.  The STM32 replies with CMD_ACK (0x103, byte0 = 0x10).
// Safety gating (STANDBY) and range validation are enforced server-side.
// -------------------------------------------------------------------------
void EngineeringScreen::sendGearLimitOp(uint8_t op, uint8_t value) {
    CanFrame frame = {};
    frame.identifier       = can::SERVICE_CMD;
    frame.extd             = 0;
    frame.data_length_code = 3;
    frame.data[0]          = can::SERVICE_ACTION_GEAR_LIMITS;
    frame.data[1]          = op;
    frame.data[2]          = value;
    ESP32Can.writeFrame(frame, 0);
}

// =========================================================================
// MCP23017 LIVE (shifter) diagnostic viewer (MCP23017_LIVE submenu)
//
// Reads shifter::getDiag() — a cached, read-only snapshot maintained by the
// shifter poller on the main loop (Core 1).  This method performs NO I2C of
// its own, so the Core-0 render path never touches the shared Wire bus,
// avoiding any cross-core race with the shifter poller.
//
// Shows: I2C address + online state, IODIRA/GPPUA config registers, raw
// GPIOA/GPIOB bytes, per-pin contact state, decoded gear, the exact reason
// the last read was rejected, valid/invalid read counters + last valid
// pattern, I2C error + bus-recovery counters, and the age of the last valid
// read.  Dynamic lines are repainted by the partial-redraw branch in draw().
// Pure read-only display — no commands, no state changes.
// =========================================================================
void EngineeringScreen::drawMcpLiveDiag() {
    RTRACE_BEGIN_SCREEN("eng_mcp_live");
    RTRACE_SET_LAYER(0);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Header
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("MCP23017 LIVE", ui::SCREEN_W / 2, 16);
    RTRACE_TEXT(ui::SCREEN_W / 2, 16, "MCP23017 LIVE",
                ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);

    // Sub-header
    tft.setTextSize(1);
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString("Read-only ESP32 shifter I2C snapshot",
                   ui::SCREEN_W / 2, 34);
    tft.setTextDatum(TL_DATUM);

    // Dynamic lines are repainted by the partial-redraw branch in draw().
    mcpDataChanged_ = true;

    // BACK button (bottom-left)
    tft.fillRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_DARK_GRAY);
    tft.drawRect(BACK_X, BACK_Y, BACK_W, BACK_H, ui::COL_AMBER);
    tft.setTextColor(ui::COL_AMBER, ui::COL_DARK_GRAY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    tft.setTextDatum(TL_DATUM);
}
