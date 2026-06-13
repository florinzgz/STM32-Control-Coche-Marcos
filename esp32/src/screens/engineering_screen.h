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
#include "can_ids.h"
#include "../shifter_input.h"
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
        INA226_LIVE_DIAG,  // Live INA226/current diagnostic viewer
        DEBOUNCE_DIAG,     // DWT-debounce EMI filtered counters viewer
        MCP23017_LIVE,     // ESP32-local MCP23017 shifter I2C live diagnostic
        GEAR_LIMITS,       // Configurable per-gear traction power limits (D2/D1/R)
        BRIGHTNESS,        // TFT backlight brightness control (local PWM)
        EPS_TUNING,        // EPS steering assist parameter fine-tuning
        STEER_DIAG         // Live steering diagnostic (encoder, PWM, state)
    };

    void drawMainMenu();
    // Draw a procedural (PROGMEM-free, zero-RAM) category icon for main-menu
    // tile `item` (0..NUM_MAIN_ITEMS-1) centred at (cx,cy) in colour `col`.
    void drawTileIcon(uint8_t item, int16_t cx, int16_t cy, uint16_t col);
    void drawFaultViewer();
    void drawModuleControl();
    void drawPedalCalibration();
    void drawEncoderCalibration();
    void drawEncoderGaugeTicks();   // static ticks + scale labels (shared)
    void drawSensorMapIna();
    void drawSensorMapTemp();
    void drawFactoryDefaults();
    void drawDtcLogViewer();
    void drawMaintenance();
    void drawRelayControl();
    void drawInaLiveDiag();
    void drawDebounceDiag();
    void drawMcpLiveDiag();
    void drawGearLimits();
    void drawBrightness();
    void drawEpsTuning();
    void drawSteerDiag();

    bool        needsRedraw_ = true;
    bool        exitRequested_ = false;
    SubMenu     currentMenu_ = SubMenu::MAIN;

    // Main-menu paging (FASE 2 tile redesign).  Functions are split across two
    // pages of large touch tiles: page 0 = items 0..8 (9 tiles), page 1 =
    // items 9..16 (8 tiles).  The dispatch logic is unchanged — a
    // tile simply maps back to its original item index.
    uint8_t     mainMenuPage_ = 0;

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
    vehicle::SteeringZData steeringZ_{};   // diagnostic-only PB5/Z snapshot
    bool        steerZDataChanged_ = false;
    unsigned long steerZLastQueryMs_ = 0;
    bool        steerZClearPending_ = false;
    unsigned long steerZClearPendingMs_ = 0;
    static constexpr unsigned long STEER_Z_CLEAR_CONFIRM_MS = 5000;
    void sendSteerZOp(uint8_t op);

    // Module control page state
    uint8_t     moduleCtrlPage_  = 0;
    static constexpr uint8_t MODULE_CTRL_PAGES = 3;

    // Module control ACK feedback — brief status message after toggle
    uint8_t     lastAckResult_   = 0;      // 0=none, 1=OK, 2=REJECTED, 3=BLOCKED
    uint32_t    lastAckMs_       = 0;      // frameTimeMs when ACK was received
    uint32_t    lastAckTracked_  = 0;      // last ack timestamp we processed

    // Module Enable/Disable double-tap confirmation (FASE 2 §1 — prevent
    // accidental enable/disable of non-critical modules).  The first tap on a
    // non-critical row arms the confirmation; the toggle command is only sent
    // when the SAME module is tapped again within MODULE_CONFIRM_TIMEOUT_MS.
    // Any other touch, page change, screen entry or BACK cancels.  Critical
    // modules (0–3) are never toggleable and never arm a confirmation.  Time
    // transitions happen in update() (frame-time contract: no millis() in the
    // touch/UI path); the touch handler only latches intent via modulePendingArm_.
    int8_t        modulePendingId_  = -1;     // module ID awaiting confirm, -1 = none
    bool          modulePendingArm_ = false;  // stamp modulePendingMs_ next update()
    unsigned long modulePendingMs_  = 0;      // frameTimeMs when armed
    static constexpr unsigned long MODULE_CONFIRM_TIMEOUT_MS = 5000;

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

    // Factory Defaults double-tap confirmation (FASE 2 §1 — prevent accidental
    // destructive resets, especially 0xFF FACTORY_RESTORE).  The first tap on a
    // row arms the confirmation; the command is only sent when the SAME row is
    // tapped again within FACTORY_CONFIRM_TIMEOUT_MS.  Any other touch, screen
    // entry or BACK cancels.  Time transitions happen in update() (frame-time
    // contract: no millis() in the touch/UI path); the touch handler only
    // latches intent via factoryPendingArm_.
    int8_t        factoryPendingIdx_ = -1;     // row awaiting confirm, -1 = none
    bool          factoryPendingArm_ = false;  // stamp factoryPendingMs_ next update()
    unsigned long factoryPendingMs_  = 0;      // frameTimeMs when armed
    static constexpr unsigned long FACTORY_CONFIRM_TIMEOUT_MS = 5000;

    // Relay status for main menu header display
    uint8_t     relayStatus_     = 0;      // heartbeat byte 5 (bit0=reserved, bit1=T, bit2=D, bit7=SEQ)
    uint8_t     prevRelayStatus_ = 0xFF;   // force initial draw
    uint8_t     sysStateRaw_     = 0;      // cached heartbeat system state (for relay-override gating message)

    // Relay override (engineering diagnostic mode)
    bool        relayOverrideEnabled_ = false;   // local UI toggle state
    uint8_t     relayOverrideMask_    = 0;       // bit0=reserved, bit1=TRAC, bit2=DIR
    bool        relayOverrideChanged_ = false;   // true when real CAN state changed

    // Relay control STANDBY gate (FASE 2 §2 — local UI must not change unless
    // the STM32 is in STANDBY, mirroring the firmware's STANDBY-only override).
    // When the user taps a relay button outside STANDBY we refuse to change the
    // local visual state and flash an "ONLY IN STANDBY" notice instead.  Time
    // transitions happen in update() (frame-time contract); the touch handler
    // only latches intent via relayStandbyMsgArm_.
    bool          relayStandbyMsg_    = false;  // true while the notice is shown
    bool          relayStandbyMsgArm_ = false;  // stamp relayStandbyMsgMs_ next update()
    unsigned long relayStandbyMsgMs_  = 0;      // frameTimeMs when the notice armed
    static constexpr unsigned long RELAY_STANDBY_MSG_MS = 2500;

    // Debounce DWT EMI counters cache (DEBOUNCE_DIAG submenu)
    uint16_t      debounceWheelFiltered_[4] = {};
    uint32_t      debounceSteerFiltered_    = 0;
    unsigned long debounceLastTs_           = 0;       // last timestamp consumed
    bool          debounceDataChanged_      = false;

    // INA226 live diagnostic cache (INA226_LIVE_DIAG submenu)
    uint16_t      inaLiveMotorCurrentRaw_[4] = {};  // CH0..CH3, 0.01 A units
    uint16_t      inaLiveBtCurrentRaw_       = 0;   // CH4 (battery), 0.01 A
    uint16_t      inaLiveBtVoltageRaw_       = 0;   // CH4 (battery), 0.01 V
    bool          inaLiveMuxPresent_         = false;
    uint8_t       inaLiveOkMask_             = 0;
    uint8_t       inaLiveExpectedMask_       = 0x3F;
    uint8_t       inaLiveFailCount_          = 0;
    uint8_t       inaLiveRecoveryCount_      = 0;
    bool          inaLiveValid_              = false;
    bool          inaLiveStale_              = false;
    unsigned long inaLiveAgeMs_              = 0;
    unsigned long inaLiveLastAgeSec_         = 0;
    bool          inaLiveDataChanged_        = false;

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

    // RUN I2C SCAN button feedback.  The command path itself was already
    // correctly wired (0x110/0xF6 → STM32 → 0x30B/0x30C), but the button gave
    // no visual confirmation, so it looked unresponsive.  These fields drive an
    // immediate "SCAN CMD SENT/FAILED" banner plus a 2 s "SCAN TIMEOUT" if no
    // 0x30B/0x30C reply arrives.  Pure HMI feedback — no protocol change.
    // All time-based transitions happen in update() (frame-time contract: no
    // direct millis() in the UI path); the touch handler only latches intent.
    enum class ScanFb : uint8_t { NONE = 0, SENT, FAILED, TIMEOUT };
    ScanFb        scanFb_            = ScanFb::NONE;
    bool          scanFbChanged_     = false;  // repaint just the status banner
    bool          scanFbArm_         = false;  // stamp scanFbMs_ next update()
    bool          scanArmReply_      = false;  // start reply watchdog next update()
    bool          scanAwaitingReply_ = false;  // waiting for 0x30B/0x30C
    unsigned long scanSentMs_        = 0;       // frameTimeMs at command tx
    unsigned long scanFbMs_          = 0;       // frameTimeMs when banner set
    unsigned long scanBaseI2cTs_     = 0;       // i2cScan ts at send (detect reply)
    unsigned long scanBaseFdcanTs_   = 0;       // fdcanDiag ts at send
    static constexpr unsigned long SCAN_TIMEOUT_MS  = 2000;
    static constexpr unsigned long SCAN_FB_CLEAR_MS = 4000;

    // Touch-calibration menu entry — when the user taps "TOUCH CALIBRATION"
    // we cannot launch the wizard from inside EngineeringScreen (it does
    // not own the ScreenManager flags).  Instead we set this flag and the
    // ScreenManager polls it via consumeTouchCalRequest() each frame.
    bool          touchCalRequested_ = false;

    // MCP23017 LIVE (shifter) diagnostic cache.  Populated in update() from
    // shifter::getDiag() (a cached, read-only snapshot — performs NO I2C of
    // its own, so the Core-0 render path never touches the shared Wire bus).
    shifter::Diag mcpDiag_{};
    unsigned long mcpAgeMs_        = 0;     // age of last valid GPIOA read
    bool          mcpDataChanged_  = false; // repaint trigger
    // Coarse change-detection signature so the page repaints ~1 Hz / on change
    // without thrashing the TFT.  Combines counters + raw + age (seconds).
    uint32_t      mcpDiagSig_      = 0xFFFFFFFFu;

    // ---- Gear power-limits editor (GEAR_LIMITS submenu, 0x30D telemetry) ----
    // Active values = limits currently applied by the STM32 (from 0x30D).
    // Edit values   = local pending edits (only after the user taps +/-).
    // Nothing is sent to the STM32 until SAVE; BACK without SAVE discards.
    enum class GearAck : uint8_t { NONE = 0, SAVED, REJECTED, INVALID, TIMEOUT };
    uint8_t       gearActiveD2_      = can::GEAR_LIMIT_D2_DEFAULT_PCT;
    uint8_t       gearActiveD1_      = can::GEAR_LIMIT_D1_DEFAULT_PCT;
    uint8_t       gearActiveR_       = can::GEAR_LIMIT_R_DEFAULT_PCT;
    uint8_t       gearEditD2_        = can::GEAR_LIMIT_D2_DEFAULT_PCT;
    uint8_t       gearEditD1_        = can::GEAR_LIMIT_D1_DEFAULT_PCT;
    uint8_t       gearEditR_         = can::GEAR_LIMIT_R_DEFAULT_PCT;
    // Accel-response profile (v2) — second editable group on the same screen.
    uint8_t       gearActiveRespD2_  = can::GEAR_RESPONSE_D2_DEFAULT_PCT;
    uint8_t       gearActiveRespD1_  = can::GEAR_RESPONSE_D1_DEFAULT_PCT;
    uint8_t       gearActiveRespR_   = can::GEAR_RESPONSE_R_DEFAULT_PCT;
    uint8_t       gearEditRespD2_    = can::GEAR_RESPONSE_D2_DEFAULT_PCT;
    uint8_t       gearEditRespD1_    = can::GEAR_RESPONSE_D1_DEFAULT_PCT;
    uint8_t       gearEditRespR_     = can::GEAR_RESPONSE_R_DEFAULT_PCT;
    // Which group is shown: false = POWER page, true = RESPONSE page.
    bool          gearLimitsShowResponse_ = false;
    bool          gearLimitsEditActive_ = false;   // true once the user edits
    bool          gearLimitsChanged_    = false;   // repaint trigger
    bool          gearLimitsRestoreArm_ = false;   // RESTORE double-confirm armed
    bool          gearLimitsSaveWait_   = false;   // awaiting ACK after SAVE
    GearAck       gearLimitsAck_        = GearAck::NONE;
    unsigned long gearLimitsAckMs_      = 0;       // frameTimeMs when ACK shown
    unsigned long gearLimitsSaveSentMs_ = 0;       // frameTimeMs at SAVE tx
    unsigned long gearLimitsLastTs_     = 0;       // last consumed 0x30D ts
    unsigned long gearLimitsLastQueryMs_ = 0;      // last QUERY tx (ms)
    unsigned long gearLimitsRestoreArmMs_ = 0;     // frameTimeMs RESTORE armed
    unsigned long lastFrameTimeMs_ = 0;            // cached frameTimeMs (update)
    static constexpr unsigned long GEAR_SAVE_TIMEOUT_MS    = 2000;
    static constexpr unsigned long GEAR_ACK_CLEAR_MS       = 3000;
    static constexpr unsigned long GEAR_RESTORE_CONFIRM_MS = 5000;
    // Emit SERVICE_CMD 0x110 byte0=0xF7 (GEAR_LIMITS) + sub-opcode + value.
    void sendGearLimitOp(uint8_t op, uint8_t value);

    // ---- TFT brightness editor (BRIGHTNESS submenu) ----
    uint8_t       brightnessEdit_    = 100;
    bool          brightnessDirty_   = false;

    // ---- EPS Steering Tuning editor (EPS_TUNING submenu) ----
    // Working edit copy of every eps_params_t field; initialised from the
    // most-recent 0x30F telemetry on enter.  Nothing is sent until SAVE.
    // A SET_PARAM command IS sent immediately when the user changes a value
    // (real-time effect), and SAVE persists to flash when in STANDBY.
    enum class EpsAck : uint8_t { NONE = 0, SAVED, REJECTED, INVALID, TIMEOUT };
    static constexpr uint8_t EPS_PAGES = 3;   // page0: gains A/B, page1: speed/mech, page2: read-back
    uint8_t       epsPage_            = 0;     // currently visible page (0..EPS_PAGES-1)
    bool          epsEditActive_      = false; // true once the user touches a +/- button
    bool          epsDataChanged_     = false; // repaint trigger
    EpsAck        epsAck_             = EpsAck::NONE;
    unsigned long epsAckMs_           = 0;
    unsigned long epsSaveSentMs_      = 0;
    bool          epsSaveWait_        = false;
    bool          epsRestoreArm_      = false;
    unsigned long epsRestoreArmMs_    = 0;
    unsigned long epsLastTs_          = 0;    // last consumed 0x30F ts
    unsigned long epsLastQueryMs_     = 0;    // last QUERY tx
    static constexpr unsigned long EPS_SAVE_TIMEOUT_MS    = 2000;
    static constexpr unsigned long EPS_ACK_CLEAR_MS       = 3000;
    static constexpr unsigned long EPS_RESTORE_CONFIRM_MS = 5000;
    static constexpr unsigned long EPS_QUERY_INTERVAL_MS  = 500;
    // Local edit values (floats, 12 params matching EPS_PARAM_* order)
    float epsEdit_[12] = {
        0.45f, 0.30f, 0.10f, 0.05f, 3.0f, 8.0f,
        18.0f, 35.0f, 1.8f,  60.0f, 5.883f, 0.0f
    };
    // Helper: send SERVICE_CMD 0x110 byte0=0xF9 (EPS_PARAMS) + sub-opcode [+ payload].
    void sendEpsParamOp(uint8_t op, uint8_t paramId, float value);
    void sendEpsQuery();

    // ---- STEER_DIAG live telemetry display ----
    bool          steerDiagChanged_   = false;
    unsigned long steerDiagLastTs_    = 0;
    unsigned long steerDiagQueryMs_   = 0;
};

#endif // ENGINEERING_SCREEN_H
