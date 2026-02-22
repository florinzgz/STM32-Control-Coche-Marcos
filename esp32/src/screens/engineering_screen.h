// =============================================================================
// ESP32-S3 HMI — Engineering Screen
//
// Hidden engineering menu, accessed by entering code "8989" on the touch
// screen.  Provides submenus for:
//   1. Pedal calibration
//   2. Encoder calibration
//   3. Module enable/disable (via SERVICE_CMD 0x110)
//   4. Factory restore (via SERVICE_CMD 0x110, action 0xFF)
//   5. Fault viewer (SERVICE_FAULTS/ENABLED/DISABLED 0x301-0x303)
//
// No heap allocation.  All format buffers are fixed-size stack arrays.
//
// Reference: docs/SERVICE_MODE.md
// =============================================================================

#ifndef ENGINEERING_SCREEN_H
#define ENGINEERING_SCREEN_H

#include "screen.h"
#include <cstdint>

class EngineeringScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;

    /// Process a touch tap event.  Returns true if consumed.
    bool handleTouch(int16_t x, int16_t y);

private:
    enum class SubMenu : uint8_t {
        MAIN = 0,
        FAULT_VIEWER,
        MODULE_CONTROL,
        PEDAL_CAL,
        ENCODER_CAL
    };

    void drawMainMenu();
    void drawFaultViewer();
    void drawModuleControl();
    void drawCalibration(const char* title);

    bool        needsRedraw_ = true;
    SubMenu     currentMenu_ = SubMenu::MAIN;

    // Cached data for fault viewer
    uint32_t    faultBits_   = 0;
    uint32_t    enabledBits_ = 0;
    uint32_t    disabledBits_ = 0;
    uint32_t    prevFaultBits_ = 0xFFFFFFFF;
    uint32_t    prevEnabledBits_ = 0xFFFFFFFF;
    uint32_t    prevDisabledBits_ = 0xFFFFFFFF;
};

#endif // ENGINEERING_SCREEN_H
