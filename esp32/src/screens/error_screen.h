// =============================================================================
// ESP32-S3 HMI — Error Screen
//
// Shown when system_state = ERROR (5).
// Red full-screen overlay with "SYSTEM ERROR" or "CAN LINK LOST" banner.
// Displays human-readable fault flags, safety error name, diagnostic
// subsystem name, and elapsed time since the error occurred.
//
// TILE-BASED DIRTY REGION ENGINE:
//   ETILE_BANNER    — top banner (SYSTEM ERROR / CAN LINK LOST)
//   ETILE_FAULTS    — fault flags hex + individual flag names
//   ETILE_SAFETY    — safety error code + name
//   ETILE_DIAG      — diagnostic error + subsystem
//   ETILE_ELAPSED   — elapsed time since error entry
//
// Reference: docs/HMI_STATE_MODEL.md §2.6
// =============================================================================

#ifndef ERROR_SCREEN_H
#define ERROR_SCREEN_H

#include "screen.h"
#include "ui/tile_engine.h"
#include <cstdint>

/// Tile indices for ErrorScreen
enum ErrorTile : uint8_t {
    ETILE_BANNER = 0,
    ETILE_FAULTS,
    ETILE_SAFETY,
    ETILE_DIAG,
    ETILE_ELAPSED,
    ETILE_COUNT
};

class ErrorScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;

private:
    static const char* safetyErrorName(uint8_t code);
    static const char* diagSubsystemName(uint8_t sub);

    ui::TileSet<ETILE_COUNT> tiles_;

    bool     needsRedraw_    = true;
    bool     canLost_        = false;   // CAN communication lost (ESP32-detected)
    bool     prevCanLost_    = false;   // previous value for redraw detection
    uint8_t  faultFlags_     = 0;
    uint8_t  prevFaultFlags_ = 0xFF;
    uint8_t  errorCode_      = 0;
    uint8_t  prevErrorCode_  = 0xFF;
    uint8_t  diagCode_       = 0;
    uint8_t  prevDiagCode_   = 0xFF;
    uint8_t  diagSubsystem_  = 0;
    uint8_t  prevDiagSubsystem_ = 0xFF;
    uint32_t errorEntryMs_   = 0;      // millis() when error screen entered
    uint32_t prevElapsedSec_ = 0xFFFFFFFF;
};

#endif // ERROR_SCREEN_H
