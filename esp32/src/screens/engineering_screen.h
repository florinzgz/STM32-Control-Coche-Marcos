// =============================================================================
// ESP32-S3 HMI — Engineering Screen
//
// Hidden engineering menu, accessed by entering code "8989" on the touch
// screen.  Provides submenus for:
//   1. Fault viewer (SERVICE_FAULTS/ENABLED/DISABLED 0x301-0x303)
//   2. Module enable/disable (via SERVICE_CMD 0x110)
//   3. Pedal calibration (live telemetry verification)
//   4. Encoder calibration (live steering encoder verification)
//   5. INA226 sensor mapping — assign channels to vehicle positions
//   6. DS18B20 temp sensor mapping — assign sensors to positions
//   7. Factory defaults — individual reset options for PID, wheel sensors,
//      INA226/shunts, traction motor force, steering motor force,
//      or full factory restore (SERVICE_CMD 0x110, actions 0xF0-0xFF)
//
// No heap allocation.  All format buffers are fixed-size stack arrays.
//
// Reference: docs/SERVICE_MODE.md, docs/ENGINEERING_MENU.md
// =============================================================================

#ifndef ENGINEERING_SCREEN_H
#define ENGINEERING_SCREEN_H

#include "screen.h"
#include "config_store.h"
#include <cstdint>

class EngineeringScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data, unsigned long frameTimeMs) override;
    void draw()    override;

    /// Process a touch tap event.  Returns true if consumed.
    bool handleTouch(int16_t x, int16_t y);

    /// Returns true if the user requested to exit engineering mode.
    bool exitRequested() const { return exitRequested_; }

    /// Clear the exit-requested flag (called by ScreenManager after handling).
    void clearExitRequest() { exitRequested_ = false; }

private:
    enum class SubMenu : uint8_t {
        MAIN = 0,
        FAULT_VIEWER,
        MODULE_CONTROL,
        PEDAL_CAL,
        ENCODER_CAL,
        SENSOR_MAP_INA,    // INA226 channel-to-position mapping
        SENSOR_MAP_TEMP,   // DS18B20 sensor-to-position mapping
        FACTORY_DEFAULTS,  // Individual factory-default reset options
        DTC_LOG_VIEWER,    // Persistent DTC fault log viewer
        MAINTENANCE,       // Maintenance counter reset + status
        RELAY_CONTROL      // Manual relay override (engineering diagnostic)
    };

    void drawMainMenu();
    void drawFaultViewer();
    void drawModuleControl();
    void drawPedalCalibration();
    void drawEncoderCalibration();
    void drawSensorMapIna();
    void drawSensorMapTemp();
    void drawFactoryDefaults();
    void drawDtcLogViewer();
    void drawMaintenance();
    void drawRelayControl();

    bool        needsRedraw_ = true;
    bool        exitRequested_ = false;
    SubMenu     currentMenu_ = SubMenu::MAIN;

    // Cached data for fault viewer
    uint32_t    faultBits_   = 0;
    uint32_t    enabledBits_ = 0;
    uint32_t    disabledBits_ = 0;
    uint32_t    prevFaultBits_ = 0xFFFFFFFF;
    uint32_t    prevEnabledBits_ = 0xFFFFFFFF;
    uint32_t    prevDisabledBits_ = 0xFFFFFFFF;

    // Cached data for pedal calibration (live telemetry)
    uint16_t    wheelSpeed_[4]   = {};     // 0.1 km/h units
    uint16_t    motorCurrent_[4] = {};     // 0.01 A units
    uint8_t     tractionScale_[4] = {};    // 0–100 %
    uint16_t    batteryVoltage_  = 0;      // 0.01 V units
    uint16_t    batteryCurrent_  = 0;      // 0.01 A units
    uint8_t     absActive_       = 0;
    uint8_t     tcsActive_       = 0;
    bool        pedalDataChanged_ = false;

    // Cached data for encoder calibration (live steering)
    int16_t     steeringAngle_   = 0;      // 0.1° units
    uint8_t     steeringCal_     = 0;      // 0 = uncalibrated, 1 = calibrated
    bool        encoderDataChanged_ = false;

    // Module control page state
    uint8_t     moduleCtrlPage_  = 0;
    static constexpr uint8_t MODULE_CTRL_PAGES = 3;

    // Module control ACK feedback — brief status message after toggle
    uint8_t     lastAckResult_   = 0;      // 0=none, 1=OK, 2=REJECTED, 3=BLOCKED
    uint32_t    lastAckMs_       = 0;      // frameTimeMs when ACK was received
    uint32_t    lastAckTracked_  = 0;      // last ack timestamp we processed

    // Sensor mapping edit state
    // Selected row (0=none selected, 1..N = row index 1-based)
    uint8_t     inaEditRow_  = 0;
    uint8_t     tempEditRow_ = 0;
    // Working copies of mapping arrays (edited in-place, saved on "SAVE")
    uint8_t     inaMap_[config_store::NUM_INA226_CH]  = {0,1,2,3,4,5};
    uint8_t     tempMap_[config_store::NUM_TEMP_SENS] = {0,1,2,3,4};
    // Live raw temperatures by physical sensor index (from STATUS_TEMP 0x202)
    // Used in the temp mapping editor so the user can identify sensors by
    // touching them (warmth increases the displayed reading for that index).
    int8_t      rawTempC_[config_store::NUM_TEMP_SENS] = {};

    // DTC log CLEAR confirmation state (§4.1 — prevent accidental clear)
    bool        clearLogPending_ = false;  // true after first tap on CLEAR; awaiting confirm

    // Relay status for main menu header display
    uint8_t     relayStatus_     = 0;      // heartbeat byte 5 (bit0=T, bit1=D, bit7=SEQ; CAN rev 1.4)
    uint8_t     prevRelayStatus_ = 0xFF;   // force initial draw

    // Relay override (engineering diagnostic mode)
    bool        relayOverrideEnabled_ = false;   // local UI toggle state
    uint8_t     relayOverrideMask_    = 0;       // bit0=MAIN, bit1=TRAC, bit2=DIR
    bool        relayOverrideChanged_ = false;   // true when real CAN state changed
};

#endif // ENGINEERING_SCREEN_H
