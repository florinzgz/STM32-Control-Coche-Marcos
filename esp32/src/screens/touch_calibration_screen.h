// =============================================================================
// ESP32-S3 HMI — Touch Calibration Wizard Screen
//
// 5-state wizard that captures XPT2046 touch calibration and persists it to
// NVS via the `touch_calibration` module.
//
// State machine:
//   INTRO     → Instructions + START / CANCEL (CANCEL hidden on first boot)
//   COLLECT   → 4 corner crosses with per-corner visual feedback.  This step
//               delegates to TFT_eSPI::calibrateTouch(), which draws each
//               cross and waits for a stable touch — exactly matching the
//               "Pantalla 2–5" requirement.  It is rotation-aware and emits
//               the standard 5-element calData array.
//   CONFIRM   → Shows the 4 captured raw values and offers SAVE / RETRY
//               (and CANCEL when not first-boot).
//   DONE      → Persisted to NVS, applied to TFT_eSPI in caller, exits.
//   FAILED    → Error message (capture failed validation), offers RETRY /
//               CANCEL (CANCEL hidden on first boot).
//
// First-boot mode (`firstBoot_ == true`):
//   CANCEL is suppressed in INTRO/CONFIRM/FAILED so the user MUST complete
//   the calibration once before reaching the normal UI.  Once SAVE succeeds
//   the `first_done` NVS flag is set and the wizard never re-launches
//   automatically again (per the problem statement).
//
// CAN / STM32 safety:
//   This screen issues NO CAN frames, touches NO can_rx state, and has no
//   timing dependency on the heartbeat.  It is fully isolated on the ESP32
//   render task (Core 0).
//
// Reference: docs/TOUCH_CALIBRATION_SYSTEM.md
// =============================================================================

#ifndef TOUCH_CALIBRATION_SCREEN_H
#define TOUCH_CALIBRATION_SCREEN_H

#include "screen.h"
#include "touch_calibration.h"
#include <cstdint>

class TouchCalibrationScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data, unsigned long frameTimeMs) override;
    void draw()    override;

    /// Configure the wizard for first-boot mode (no CANCEL, must complete)
    /// or engineering-menu mode (CANCEL allowed).  Must be called immediately
    /// before onEnter().
    void setFirstBoot(bool firstBoot) { firstBoot_ = firstBoot; }

    /// Process a touch tap on INTRO / CONFIRM / FAILED screens.
    /// Returns true if consumed.
    bool handleTouch(int16_t x, int16_t y);

    /// True after the wizard has produced valid calibration data and the
    /// caller (`ScreenManager`) should apply it via `tft.setTouch()` and
    /// transition out.  Cleared by `clearResultFlags()`.
    bool isSaved() const { return saved_; }

    /// True when the user has chosen to leave the wizard without saving
    /// (only reachable when not first-boot).  Cleared by `clearResultFlags()`.
    bool isCancelled() const { return cancelled_; }

    /// Get the calibration data (5 elements, TFT_eSPI calData layout) that
    /// was captured and saved to NVS.  Valid only when `isSaved()` is true.
    const uint16_t* result() const { return calData_; }

    /// Clear `saved_` and `cancelled_` flags (called by ScreenManager after
    /// transitioning out).  Resets internal wizard state to INTRO.
    void clearResultFlags();

private:
    enum class State : uint8_t {
        INTRO,
        COLLECT,    // delegates to TFT_eSPI::calibrateTouch() for visual feedback
        CONFIRM,
        DONE,
        FAILED
    };

    void drawIntro();
    void drawConfirm();
    void drawFailed();
    void runCollection();   // performs the blocking calibrateTouch() call

    State    state_         = State::INTRO;
    bool     firstBoot_     = false;
    bool     needsRedraw_   = true;
    bool     saved_         = false;
    bool     cancelled_     = false;

    // Calibration data captured by COLLECT (TFT_eSPI calData layout:
    // { xMin, xMax, yMin, yMax, rotation }).  Zeroed on entry.
    uint16_t calData_[touch_calibration::CAL_DATA_LEN] = {0, 0, 0, 0, 0};
};

#endif // TOUCH_CALIBRATION_SCREEN_H
