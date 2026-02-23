// =============================================================================
// ESP32-S3 HMI — PIN Entry Screen Implementation
// =============================================================================

#include "pin_screen.h"
#include "ui/ui_common.h"
#include "ui/render_trace.h"
#include <TFT_eSPI.h>
#include <Arduino.h>
#include <cstdio>
#include <cstring>

extern TFT_eSPI tft;

// ---- Out-of-line definition of constexpr array -----
constexpr uint8_t PinScreen::CORRECT_PIN[PinScreen::PIN_LEN];

// ---- Layout constants -----------------------------------------------
// Keypad buttons: 90×50 px, 3 columns centred on 480 px wide screen
static constexpr int16_t BTN_W      = 90;
static constexpr int16_t BTN_H      = 50;
static constexpr int16_t BTN_GAP    = 15;   // horizontal gap between columns
static constexpr int16_t BTN_VGAP   = 10;   // vertical gap between rows
// 3 cols × 90 + 2 × 15 = 300; start X = (480 - 300) / 2 = 90
static constexpr int16_t COL_X[3]   = {90, 195, 300};
// 4 rows starting at Y=80, row pitch = 50+10 = 60
static constexpr int16_t ROW_Y[4]   = {80, 140, 200, 260};

// Key mapping [row][col]: 0-9 digits, KEY_DELETE, KEY_CANCEL
static constexpr uint8_t KEY_MAP[4][3] = {
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9},
    {PinScreen::KEY_DELETE, 0, PinScreen::KEY_CANCEL}
};

// PIN dot display
static constexpr int16_t DOT_W      = 28;
static constexpr int16_t DOT_H      = 28;
static constexpr int16_t DOT_GAP    = 12;
// 4 dots × 28 + 3 × 12 = 148; start X = (480 - 148) / 2 = 166
static constexpr int16_t DOT_START_X = (ui::SCREEN_W - (4 * DOT_W + 3 * DOT_GAP)) / 2;
static constexpr int16_t DOT_Y      = 38;

// -------------------------------------------------------------------------
// Lifecycle
// -------------------------------------------------------------------------
void PinScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("pin");
    reset();
}

void PinScreen::onExit() {}

void PinScreen::update(const vehicle::VehicleData& /*data*/) {
    // Expire "wrong code" message after timeout
    if (wrongCode_ && (millis() - wrongCodeMs_) >= WRONG_CODE_DISPLAY_MS) {
        wrongCode_   = false;
        needsRedraw_ = true;
    }
}

void PinScreen::reset() {
    memset(digits_, 0, sizeof(digits_));
    len_         = 0;
    confirmed_   = false;
    cancelled_   = false;
    wrongCode_   = false;
    wrongCodeMs_ = 0;
    needsRedraw_ = true;
}

// -------------------------------------------------------------------------
// Draw
// -------------------------------------------------------------------------
void PinScreen::draw() {
    if (!needsRedraw_) return;
    needsRedraw_ = false;

    RTRACE_SET_LAYER(0);
    tft.fillScreen(ui::COL_BG);
    RTRACE_FILL_SCREEN(ui::COL_BG);
    RTRACE_SET_LAYER(1);

    // Title
    tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("INGENIERIA", ui::SCREEN_W / 2, 16);
    RTRACE_TEXT(ui::SCREEN_W / 2, 16, "INGENIERIA",
                ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);
    tft.setTextDatum(TL_DATUM);

    // Subtitle — prompt or wrong-code message
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    if (wrongCode_) {
        tft.setTextColor(ui::COL_RED, ui::COL_BG);
        tft.drawString("Codigo incorrecto", ui::SCREEN_W / 2, 32);  // NOLINT accent intentionally omitted (ASCII-safe TFT font)
        RTRACE_TEXT(ui::SCREEN_W / 2, 32, "Codigo incorrecto",
                    ui::COL_RED, ui::COL_BG, 1, MC_DATUM);
    } else {
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("Introduce el codigo", ui::SCREEN_W / 2, 32);  // NOLINT accent intentionally omitted
        RTRACE_TEXT(ui::SCREEN_W / 2, 32, "Introduce el codigo",
                    ui::COL_GRAY, ui::COL_BG, 1, MC_DATUM);
    }
    tft.setTextDatum(TL_DATUM);

    // PIN dots
    drawDots();

    // Keypad
    char numBuf[4];
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 3; ++col) {
            uint8_t     key   = KEY_MAP[row][col];
            int16_t     bx    = COL_X[col];
            int16_t     by    = ROW_Y[row];
            const char* label;
            uint16_t    bgCol = ui::COL_DARK_GRAY;
            uint16_t    fgCol = ui::COL_WHITE;

            if (key == KEY_DELETE) {
                label = "<";
                bgCol = ui::COL_GRAY;
                fgCol = ui::COL_BLACK;
            } else if (key == KEY_CANCEL) {
                label = "X";
                bgCol = ui::COL_RED;
                fgCol = ui::COL_WHITE;
            } else {
                snprintf(numBuf, sizeof(numBuf), "%u", static_cast<unsigned>(key));
                label = numBuf;
            }
            drawButton(bx, by, BTN_W, BTN_H, label, bgCol, fgCol);
        }
    }

    RTRACE_DUMP_IF_PENDING();
}

void PinScreen::drawDots() {
    for (int i = 0; i < PIN_LEN; ++i) {
        int16_t dx = DOT_START_X + i * (DOT_W + DOT_GAP);
        int16_t dy = DOT_Y;
        if (i < static_cast<int>(len_)) {
            tft.fillRect(dx, dy, DOT_W, DOT_H, ui::COL_AMBER);
            RTRACE_FILL_RECT(dx, dy, DOT_W, DOT_H, ui::COL_AMBER);
        } else {
            tft.fillRect(dx, dy, DOT_W, DOT_H, ui::COL_BG);
            RTRACE_FILL_RECT(dx, dy, DOT_W, DOT_H, ui::COL_BG);
            tft.drawRect(dx, dy, DOT_W, DOT_H, ui::COL_GRAY);
            RTRACE_DRAW_RECT(dx, dy, DOT_W, DOT_H, ui::COL_GRAY);
        }
    }
}

void PinScreen::drawButton(int16_t x, int16_t y, int16_t w, int16_t h,
                            const char* label, uint16_t bgCol, uint16_t fgCol) {
    tft.fillRect(x, y, w, h, bgCol);
    RTRACE_FILL_RECT(x, y, w, h, bgCol);
    tft.drawRect(x, y, w, h, ui::COL_GRAY);
    RTRACE_DRAW_RECT(x, y, w, h, ui::COL_GRAY);
    tft.setTextColor(fgCol, bgCol);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(label, x + w / 2, y + h / 2);
    RTRACE_TEXT(x + w / 2, y + h / 2, label, fgCol, bgCol, 2, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

// -------------------------------------------------------------------------
// Touch handling
// -------------------------------------------------------------------------
bool PinScreen::handleTouch(int16_t x, int16_t y) {
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 3; ++col) {
            int16_t bx = COL_X[col];
            int16_t by = ROW_Y[row];
            if (x >= bx && x <= bx + BTN_W &&
                y >= by && y <= by + BTN_H) {
                uint8_t key = KEY_MAP[row][col];
                if (key == KEY_DELETE) {
                    handleDelete();
                } else if (key == KEY_CANCEL) {
                    handleCancel();
                } else {
                    handleDigit(key);
                }
                return true;
            }
        }
    }
    return false;
}

void PinScreen::handleDigit(uint8_t d) {
    if (len_ >= PIN_LEN) return;
    digits_[len_++] = d;
    needsRedraw_    = true;
    if (len_ == PIN_LEN) {
        checkPin();
    }
}

void PinScreen::handleDelete() {
    if (len_ > 0) {
        --len_;
        wrongCode_   = false;
        needsRedraw_ = true;
    }
}

void PinScreen::handleCancel() {
    cancelled_ = true;
}

void PinScreen::checkPin() {
    for (uint8_t i = 0; i < PIN_LEN; ++i) {
        if (digits_[i] != CORRECT_PIN[i]) {
            // Wrong — show message, clear entry for retry
            wrongCode_   = true;
            wrongCodeMs_ = millis();
            len_         = 0;
            memset(digits_, 0, sizeof(digits_));
            needsRedraw_ = true;
            Serial.println("[PIN] Wrong code");
            return;
        }
    }
    confirmed_ = true;
    Serial.println("[PIN] Code accepted");
}
