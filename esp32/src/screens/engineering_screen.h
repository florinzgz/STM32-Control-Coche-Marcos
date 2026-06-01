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

    /// Returns true (and clears the flag) if the user selected the
    /// "TOUCH CALIBRATION" menu entry on this frame.  ScreenManager polls
    /// this and launches the wizard accordingly.
    bool consumeTouchCalRequest() {
        bool v = touchCalRequested_;
        touchCalRequested_ = false;
        return v;
    }

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
        RELAY_CONTROL,     // Manual relay override (engineering diagnostic)
        DEBOUNCE_DIAG      // DWT-debounce EMI filtered counters viewer
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
    void drawDebounceDiag();

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

    // Cached data for pedal calibration (live telemetry verification and
    // persistent endpoint calibration UI — 0x308 telemetry burst).
    // Legacy fields below preserved for backward compatibility with the
    // old verification panel but no longer rendered on the new UI.
    uint16_t    wheelSpeed_[4]   = {};     // 0.1 km/h units
    uint16_t    motorCurrent_[4] = {};     // 0.01 A units
    uint8_t     tractionScale_[4] = {};    // 0–100 %
    uint16_t    batteryVoltage_  = 0;      // 0.01 V units
    uint16_t    batteryCurrent_  = 0;      // 0.01 A units
    uint8_t     absActive_       = 0;
    uint8_t     tcsActive_       = 0;
    bool        pedalDataChanged_ = false;

    // Pedal endpoint calibration state (mirrors latest 0x308 frame).
    uint8_t       pedalCalFlags_      = 0;
    uint16_t      pedalCalRawAdc_     = 0;
    uint16_t      pedalCalStoredMin_  = 0;
    uint16_t      pedalCalStoredMax_  = 0;
    uint16_t      pedalCalPendingMin_ = 0;
    uint16_t      pedalCalPendingMax_ = 0;
    uint8_t       pedalCalPercent_    = 0;
    unsigned long pedalCalLastTs_     = 0;       // last accepted 0x308 timestamp
    unsigned long pedalCalLastQueryMs_ = 0;      // last QUERY tx (ms)
    // Local stability ring: last 8 rawAdc samples; UI shows "stable" when
    // (max-min) within tolerance.  Independent from the STM32-side stability
    // check that runs synchronously during CAPTURE.
    static constexpr uint8_t PEDAL_STAB_N = 8;
    uint16_t      pedalStabRing_[PEDAL_STAB_N] = {};
    uint8_t       pedalStabCount_   = 0;
    uint8_t       pedalStabHead_    = 0;
    // Helper: send a SERVICE_CMD (0x110) with byte0=0xF5 (PEDAL_CAL) + sub-opcode.
    void sendPedalCalOp(uint8_t op);

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
    uint8_t     relayStatus_     = 0;      // heartbeat byte 5 (bit0=reserved, bit1=T, bit2=D, bit7=SEQ)
    uint8_t     prevRelayStatus_ = 0xFF;   // force initial draw

    // Relay override (engineering diagnostic mode)
    bool        relayOverrideEnabled_ = false;   // local UI toggle state
    uint8_t     relayOverrideMask_    = 0;       // bit0=reserved, bit1=TRAC, bit2=DIR
    bool        relayOverrideChanged_ = false;   // true when real CAN state changed

    // Debounce DWT EMI counters cache (DEBOUNCE_DIAG submenu)
    uint16_t      debounceWheelFiltered_[4] = {};
    uint32_t      debounceSteerFiltered_    = 0;
    unsigned long debounceLastTs_           = 0;       // last timestamp consumed
    bool          debounceDataChanged_      = false;

    // CAN/0x309 delivery + I2C-scan diagnostics cache (DEBOUNCE_DIAG page).
    // Populated from 0x30A (meta), 0x30B (I2C scan), 0x30C (FDCAN) and the
    // can_rx per-ID 0x309 counters.  Read-only display; the only action is a
    // "RUN I2C SCAN" button that emits SERVICE_CMD 0xF6.
    vehicle::CanMetaData   canMeta_{};
    vehicle::I2cScanData   i2cScan_{};
    vehicle::FdcanDiagData fdcanDiag_{};
    uint32_t      rx0x309Count_   = 0;
    uint32_t      drop0x309Dlc_   = 0;
    uint8_t       last0x309Dlc_   = 0;
    unsigned long canDiagLastTs_  = 0;     // last 0x30A timestamp consumed
    bool          canDiagChanged_ = false;
    // Send SERVICE_CMD 0x110 byte0=0xF6 to trigger the STM32 I2C service scan.
    void sendI2cServiceScan();

    // Touch-calibration menu entry — when the user taps "TOUCH CALIBRATION"
    // we cannot launch the wizard from inside EngineeringScreen (it does
    // not own the ScreenManager flags).  Instead we set this flag and the
    // ScreenManager polls it via consumeTouchCalRequest() each frame.
    bool          touchCalRequested_ = false;
};

#endif // ENGINEERING_SCREEN_H
