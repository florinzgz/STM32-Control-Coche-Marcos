// =============================================================================
// ESP32-S3 HMI — PIN Entry Screen
//
// Numeric keypad shown after a 3-second long press on the battery icon.
// User enters a 4-digit PIN; auto-confirms on the fourth digit.
// Correct PIN (8989) → transitions to EngineeringScreen.
// Wrong PIN → shows brief error, clears entry for retry.
// Cancel (✕ key) → returns to the normal screen.
//
// No heap allocation.  All buffers are fixed-size stack arrays.
//
// Layout (480×320 landscape):
//   Title       Y=8   "INGENIERIA" centred, amber
//   Subtitle    Y=32  prompt / error text
//   PIN dots    Y=38  4 squares (filled=entered, outline=empty)
//   Keypad      Y=80  3×4 grid: [1][2][3] / [4][5][6] / [7][8][9] / [←][0][✕]
// =============================================================================

#ifndef PIN_SCREEN_H
#define PIN_SCREEN_H

#include "screen.h"
#include <cstdint>

class PinScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;

    /// Process a touch tap.  Returns true if consumed.
    bool handleTouch(int16_t x, int16_t y);

    /// True once the correct 4-digit PIN has been entered.
    bool isConfirmed() const { return confirmed_; }

    /// True when the user pressed the cancel (✕) key.
    bool isCancelled() const { return cancelled_; }

    /// Reset all state (call before reusing after confirm/cancel).
    void reset();

    // ---- PIN length and correct code --------------------------------
    static constexpr uint8_t PIN_LEN       = 4;
    static constexpr uint8_t CORRECT_PIN[PIN_LEN] = {8, 9, 8, 9};

    // Special key values stored in the key map (public for file-scope KEY_MAP)
    static constexpr uint8_t KEY_DELETE = 10;
    static constexpr uint8_t KEY_CANCEL = 11;

private:

    uint8_t  digits_[PIN_LEN] = {};
    uint8_t  len_         = 0;
    bool     confirmed_   = false;
    bool     cancelled_   = false;
    bool     needsRedraw_ = true;
    bool     wrongCode_   = false;    // briefly show "Codigo incorrecto"
    uint32_t wrongCodeMs_ = 0;

    static constexpr uint32_t WRONG_CODE_DISPLAY_MS = 1000;

    void handleDigit(uint8_t d);
    void handleDelete();
    void handleCancel();
    void checkPin();

    void drawDots();
    static void drawButton(int16_t x, int16_t y, int16_t w, int16_t h,
                           const char* label, uint16_t bgCol, uint16_t fgCol);
};

#endif // PIN_SCREEN_H
