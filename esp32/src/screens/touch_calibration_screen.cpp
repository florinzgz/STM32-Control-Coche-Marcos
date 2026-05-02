// =============================================================================
// ESP32-S3 HMI — Touch Calibration Wizard Screen — Implementation
// =============================================================================

#include "touch_calibration_screen.h"
#include "ui/ui_common.h"
#include <TFT_eSPI.h>
#include <Arduino.h>
#include <cstdio>
#include <cstring>

extern TFT_eSPI tft;

// ---- Layout constants (480 × 320 landscape) ------------------------------
namespace {

constexpr int16_t TITLE_Y    = 22;
constexpr int16_t BODY_Y     = 70;
constexpr int16_t LINE_GAP   = 22;

constexpr int16_t BTN_W      = 130;
constexpr int16_t BTN_H      = 44;
constexpr int16_t BTN_Y      = 240;

// 1-, 2- or 3-button row layouts (screen-centred):
constexpr int16_t BTN1_X     = (ui::SCREEN_W - BTN_W) / 2;
constexpr int16_t BTN2_LX    = ui::SCREEN_W / 2 - BTN_W - 10;
constexpr int16_t BTN2_RX    = ui::SCREEN_W / 2 + 10;
constexpr int16_t BTN3_LX    = ui::SCREEN_W / 6 - BTN_W / 2 + 30;
constexpr int16_t BTN3_CX    = (ui::SCREEN_W - BTN_W) / 2;
constexpr int16_t BTN3_RX    = (5 * ui::SCREEN_W) / 6 - BTN_W / 2 - 30;

// Touch calibration cross size (pixels)
constexpr uint8_t CROSS_SIZE = 15;

// ---- Helper: filled button with border + centred label ------------------
static void drawButton(int16_t x, int16_t y, int16_t w, int16_t h,
                       const char* label, uint16_t bgCol, uint16_t fgCol) {
    tft.fillRect(x, y, w, h, bgCol);
    tft.drawRect(x, y, w, h, ui::COL_GRAY);
    tft.setTextColor(fgCol, bgCol);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(label, x + w / 2, y + h / 2);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

// ---- Helper: hit-test a rectangular button -----------------------------
static inline bool hit(int16_t tx, int16_t ty,
                       int16_t bx, int16_t by, int16_t bw, int16_t bh) {
    return (tx >= bx) && (tx <= bx + bw) && (ty >= by) && (ty <= by + bh);
}

} // anonymous namespace

// =========================================================================
// Lifecycle
// =========================================================================
void TouchCalibrationScreen::onEnter() {
    state_       = State::INTRO;
    needsRedraw_ = true;
    saved_       = false;
    cancelled_   = false;
    memset(calData_, 0, sizeof(calData_));
    Serial.printf("[TOUCH_CAL] Wizard entered (firstBoot=%d)\n",
                  static_cast<int>(firstBoot_));
}

void TouchCalibrationScreen::onExit() {
    // Nothing to release — calibrateTouch() does not allocate.
}

void TouchCalibrationScreen::clearResultFlags() {
    saved_     = false;
    cancelled_ = false;
    state_     = State::INTRO;
}

// =========================================================================
// Update — one tick per frame.  Most states are static; COLLECT is driven
// by handleTouch() below (transition INTRO → COLLECT runs the blocking
// calibrateTouch() call once and then transitions to CONFIRM/FAILED).
// =========================================================================
void TouchCalibrationScreen::update(const vehicle::VehicleData& /*data*/,
                                     unsigned long /*frameTimeMs*/) {
    // No periodic logic — the wizard is purely event-driven via handleTouch().
}

// =========================================================================
// Touch handling — buttons on INTRO / CONFIRM / FAILED screens
// =========================================================================
bool TouchCalibrationScreen::handleTouch(int16_t x, int16_t y) {
    switch (state_) {

    case State::INTRO: {
        // START button (always present)
        const int16_t startX = firstBoot_ ? BTN1_X : BTN2_LX;
        if (hit(x, y, startX, BTN_Y, BTN_W, BTN_H)) {
            // Run blocking corner collection, then transition based on result.
            runCollection();
            return true;
        }
        // CANCEL button (only when not first-boot)
        if (!firstBoot_ && hit(x, y, BTN2_RX, BTN_Y, BTN_W, BTN_H)) {
            cancelled_ = true;
            Serial.println("[TOUCH_CAL] Wizard cancelled at INTRO");
            return true;
        }
        return false;
    }

    case State::CONFIRM: {
        // SAVE
        const bool showCancel = !firstBoot_;
        const int16_t saveX  = showCancel ? BTN3_LX : BTN2_LX;
        const int16_t retryX = showCancel ? BTN3_CX : BTN2_RX;
        if (hit(x, y, saveX, BTN_Y, BTN_W, BTN_H)) {
            if (touch_calibration::save(calData_)) {
                saved_       = true;
                state_       = State::DONE;
                needsRedraw_ = true;
                Serial.println("[TOUCH_CAL] Calibration SAVED");
            } else {
                state_       = State::FAILED;
                needsRedraw_ = true;
            }
            return true;
        }
        // RETRY
        if (hit(x, y, retryX, BTN_Y, BTN_W, BTN_H)) {
            state_       = State::INTRO;
            needsRedraw_ = true;
            return true;
        }
        // CANCEL (only when not first-boot)
        if (showCancel && hit(x, y, BTN3_RX, BTN_Y, BTN_W, BTN_H)) {
            cancelled_ = true;
            Serial.println("[TOUCH_CAL] Wizard cancelled at CONFIRM");
            return true;
        }
        return false;
    }

    case State::FAILED: {
        // RETRY (always)
        const int16_t retryX = firstBoot_ ? BTN1_X : BTN2_LX;
        if (hit(x, y, retryX, BTN_Y, BTN_W, BTN_H)) {
            state_       = State::INTRO;
            needsRedraw_ = true;
            return true;
        }
        // CANCEL (only when not first-boot)
        if (!firstBoot_ && hit(x, y, BTN2_RX, BTN_Y, BTN_W, BTN_H)) {
            cancelled_ = true;
            Serial.println("[TOUCH_CAL] Wizard cancelled after FAILED");
            return true;
        }
        return false;
    }

    case State::COLLECT:
    case State::DONE:
    default:
        return false;
    }
}

// =========================================================================
// Draw — repaints once per state transition (touch-driven, low frequency)
// =========================================================================
void TouchCalibrationScreen::draw() {
    if (!needsRedraw_) return;
    needsRedraw_ = false;

    switch (state_) {
        case State::INTRO:   drawIntro();   break;
        case State::CONFIRM: drawConfirm(); break;
        case State::FAILED:  drawFailed();  break;
        // COLLECT is fully painted by tft.calibrateTouch() itself; DONE
        // is transient (caller transitions out immediately after save).
        case State::COLLECT:
        case State::DONE:
        default: break;
    }
}

void TouchCalibrationScreen::drawIntro() {
    tft.fillScreen(ui::COL_BG);

    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("CALIBRACION TACTIL", ui::SCREEN_W / 2, TITLE_Y);
    tft.setTextDatum(TL_DATUM);

    tft.setTextSize(1);
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Toca con precision las cruces que apareceran",
                   ui::SCREEN_W / 2, BODY_Y);
    tft.drawString("en cada esquina de la pantalla.",
                   ui::SCREEN_W / 2, BODY_Y + LINE_GAP);
    tft.drawString("Manten el dedo firme sobre cada cruz hasta",
                   ui::SCREEN_W / 2, BODY_Y + 3 * LINE_GAP);
    tft.drawString("que la cruz se confirme y aparezca la siguiente.",
                   ui::SCREEN_W / 2, BODY_Y + 4 * LINE_GAP);

    if (firstBoot_) {
        tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
        tft.drawString("Calibracion inicial obligatoria",
                       ui::SCREEN_W / 2, BODY_Y + 6 * LINE_GAP);
    }
    tft.setTextDatum(TL_DATUM);

    // Buttons
    if (firstBoot_) {
        drawButton(BTN1_X, BTN_Y, BTN_W, BTN_H, "EMPEZAR",
                   ui::COL_DARK_GRAY, ui::COL_GREEN);
    } else {
        drawButton(BTN2_LX, BTN_Y, BTN_W, BTN_H, "EMPEZAR",
                   ui::COL_DARK_GRAY, ui::COL_GREEN);
        drawButton(BTN2_RX, BTN_Y, BTN_W, BTN_H, "CANCELAR",
                   ui::COL_DARK_GRAY, ui::COL_RED);
    }
}

void TouchCalibrationScreen::drawConfirm() {
    tft.fillScreen(ui::COL_BG);

    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("CONFIRMAR CALIBRACION", ui::SCREEN_W / 2, TITLE_Y);
    tft.setTextDatum(TL_DATUM);

    tft.setTextSize(1);
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Valores capturados (formato TFT_eSPI calData):",
                   ui::SCREEN_W / 2, BODY_Y);

    char buf[64];
    snprintf(buf, sizeof(buf), "x_min=%u   x_max=%u",
             calData_[0], calData_[1]);
    tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
    tft.drawString(buf, ui::SCREEN_W / 2, BODY_Y + 2 * LINE_GAP);
    snprintf(buf, sizeof(buf), "y_min=%u   y_max=%u",
             calData_[2], calData_[3]);
    tft.drawString(buf, ui::SCREEN_W / 2, BODY_Y + 3 * LINE_GAP);
    snprintf(buf, sizeof(buf), "rotation=%u", calData_[4]);
    tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
    tft.drawString(buf, ui::SCREEN_W / 2, BODY_Y + 4 * LINE_GAP);

    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.drawString("Pulsa GUARDAR para persistir, REINTENTAR para recalibrar",
                   ui::SCREEN_W / 2, BODY_Y + 6 * LINE_GAP);
    tft.setTextDatum(TL_DATUM);

    if (firstBoot_) {
        drawButton(BTN2_LX, BTN_Y, BTN_W, BTN_H, "GUARDAR",
                   ui::COL_DARK_GRAY, ui::COL_GREEN);
        drawButton(BTN2_RX, BTN_Y, BTN_W, BTN_H, "REINTENTAR",
                   ui::COL_DARK_GRAY, ui::COL_AMBER);
    } else {
        drawButton(BTN3_LX, BTN_Y, BTN_W, BTN_H, "GUARDAR",
                   ui::COL_DARK_GRAY, ui::COL_GREEN);
        drawButton(BTN3_CX, BTN_Y, BTN_W, BTN_H, "REINTENTAR",
                   ui::COL_DARK_GRAY, ui::COL_AMBER);
        drawButton(BTN3_RX, BTN_Y, BTN_W, BTN_H, "CANCELAR",
                   ui::COL_DARK_GRAY, ui::COL_RED);
    }
}

void TouchCalibrationScreen::drawFailed() {
    tft.fillScreen(ui::COL_BG);

    tft.setTextColor(ui::COL_RED, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("CALIBRACION INVALIDA", ui::SCREEN_W / 2, TITLE_Y);

    tft.setTextSize(1);
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.drawString("Los valores capturados estan fuera de rango.",
                   ui::SCREEN_W / 2, BODY_Y);
    tft.drawString("Asegurate de tocar con precision cada esquina",
                   ui::SCREEN_W / 2, BODY_Y + LINE_GAP);
    tft.drawString("y vuelve a intentarlo.",
                   ui::SCREEN_W / 2, BODY_Y + 2 * LINE_GAP);
    tft.setTextDatum(TL_DATUM);

    if (firstBoot_) {
        drawButton(BTN1_X, BTN_Y, BTN_W, BTN_H, "REINTENTAR",
                   ui::COL_DARK_GRAY, ui::COL_AMBER);
    } else {
        drawButton(BTN2_LX, BTN_Y, BTN_W, BTN_H, "REINTENTAR",
                   ui::COL_DARK_GRAY, ui::COL_AMBER);
        drawButton(BTN2_RX, BTN_Y, BTN_W, BTN_H, "CANCELAR",
                   ui::COL_DARK_GRAY, ui::COL_RED);
    }
}

// =========================================================================
// Corner collection — delegates to TFT_eSPI::calibrateTouch().
//
// calibrateTouch() draws each corner cross with on-screen feedback (cross
// turns from foreground to background colour upon successful sample) and
// returns the standard 5-element calData array, which is exactly what
// TFT_eSPI::setTouch() consumes.  This satisfies the "4 puntos esquinas
// táctiles + confirmación visual de cada punto" requirement without
// reimplementing the rotation-aware fitting maths from scratch.
//
// The call blocks the render task (Core 0) for the duration of the user's
// taps.  Core 1 (CAN poll, safety, sensors, audio, LEDs) is unaffected.
// =========================================================================
void TouchCalibrationScreen::runCollection() {
    state_       = State::COLLECT;
    needsRedraw_ = false;

    tft.fillScreen(ui::COL_BG);

    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("TOCA LAS 4 ESQUINAS", ui::SCREEN_W / 2, TITLE_Y);
    tft.setTextSize(1);
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.drawString("Manten el dedo firme sobre cada cruz",
                   ui::SCREEN_W / 2, TITLE_Y + 30);
    tft.setTextDatum(TL_DATUM);

    // Capture: TFT_eSPI draws crosses at TL, BL, TR, BR with visual feedback.
    uint16_t cal[touch_calibration::CAL_DATA_LEN] = {0};
    tft.calibrateTouch(cal, ui::COL_WHITE, ui::COL_BG, CROSS_SIZE);

    Serial.printf("[TOUCH_CAL] Captured: { %u, %u, %u, %u, %u }\n",
                  cal[0], cal[1], cal[2], cal[3], cal[4]);

    // Sanity check the captured data before letting the user save it.
    // We reuse the validation logic in touch_calibration::save() but only
    // run it once we reach SAVE — here we apply a minimal range check so
    // a totally degenerate capture jumps directly to FAILED.
    const uint16_t xRange = (cal[1] > cal[0]) ? (cal[1] - cal[0]) : 0;
    const uint16_t yRange = (cal[3] > cal[2]) ? (cal[3] - cal[2]) : 0;
    if (xRange < touch_calibration::MIN_RANGE ||
        yRange < touch_calibration::MIN_RANGE) {
        Serial.println("[TOUCH_CAL] Capture range too small — FAILED");
        state_       = State::FAILED;
        needsRedraw_ = true;
        return;
    }

    memcpy(calData_, cal, sizeof(calData_));
    state_       = State::CONFIRM;
    needsRedraw_ = true;
}
