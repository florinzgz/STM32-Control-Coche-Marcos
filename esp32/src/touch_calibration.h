// =============================================================================
// ESP32-S3 HMI — Touch Calibration Persistence (NVS)
//
// Persists XPT2046 touch-screen calibration data (5-element TFT_eSPI calData
// array) in a dedicated NVS namespace `touch_cal`, separated from the main
// `hmi_cfg` config to avoid corrupting unrelated configuration on partial
// writes or schema changes.
//
// Storage layout (single NVS blob `data` (20 bytes), plus boolean flag
// `first_done`):
//   - magic     uint32   0x54434C31 ("TCL1")
//   - xMin      uint16   touch_x_offset (XPT2046 raw minimum X, 0..4095)
//   - xMax      uint16   touch_x_range  (XPT2046 raw span = max_X - min_X)
//   - yMin      uint16   touch_y_offset (XPT2046 raw minimum Y, 0..4095)
//   - yMax      uint16   touch_y_range  (XPT2046 raw span = max_Y - min_Y)
//   - rotation  uint8    TFT_eSPI axis-flags bitmask (bit0=rotate, bit1=inv_x, bit2=inv_y; range 0-7)
//   - _pad      uint8    structure padding (zeroed)
//   - _pad2     uint16   structure padding (zeroed)
//   - crc32     uint32   CRC-32/ISO-HDLC over preceding fields
//
// The blob is loaded into the `uint16_t calData[5]` format expected by
// TFT_eSPI::setTouch(), namely { x_offset, x_range, y_offset, y_range, flags }.
//
// Validation (all must hold to accept stored calibration):
//   1. magic == 0x54434C31
//   2. crc32 matches recomputed value
//   3. x_range >= MIN_RANGE  &&  y_range >= MIN_RANGE
//   4. x_offset and y_offset in [0, 4095]
//   5. flags in [0, 7] (valid 3-bit bitmask)
//
// Reference: docs/TOUCH_CALIBRATION_SYSTEM.md
// =============================================================================

#ifndef TOUCH_CALIBRATION_H
#define TOUCH_CALIBRATION_H

#include <cstdint>

namespace touch_calibration {

// Number of elements in the TFT_eSPI calData array.
inline constexpr uint8_t CAL_DATA_LEN = 5;

// Minimum acceptable XPT2046 raw range between min/max for either axis.
// Values significantly smaller than this indicate a degenerate or corrupt
// calibration (e.g. user tapped only one corner, or two opposing corners
// reported nearly identical raw coordinates).
inline constexpr uint16_t MIN_RANGE   = 1000;

/// Initialize the NVS namespace.  Call once from setup() BEFORE loadValid().
void init();

/// Load and validate persisted calibration into `out` (5 elements, in the
/// TFT_eSPI calData order { x_offset, x_range, y_offset, y_range, flags }).
/// Returns true if a fully valid calibration was found, false otherwise.
/// On false, `out` is left untouched.
bool loadValid(uint16_t out[CAL_DATA_LEN]);

/// Persist a calibration to NVS.  Computes magic + CRC32.
/// Also sets the `first_done` flag so the boot-time wizard is suppressed
/// on subsequent boots.  Returns true on success.
bool save(const uint16_t data[CAL_DATA_LEN]);

/// Erase the calibration data blob.  The `first_done` flag is left
/// untouched — explicit recalibration requires entering the wizard from
/// the engineering menu (the wizard is NOT relaunched automatically).
void clear();

/// Erase BOTH the data blob and the `first_done` flag.  Used only by the
/// dedicated "Reset Touch Cal" engineering menu entry, which intentionally
/// re-arms the first-boot wizard for the next reboot.
void factoryReset();

/// Returns true once the wizard has been completed at least once on this
/// device.  Driven by the `first_done` NVS key (independent of whether
/// the calibration data is currently valid — protects against a corruption
/// loop relaunching the wizard on every boot).
bool firstBootDone();

/// Mark the first-boot wizard as completed.  Called automatically by
/// save() but also exposed so that `factoryReset()` callers can clearly
/// re-arm the flag if needed.
void markFirstBootDone();

} // namespace touch_calibration

#endif // TOUCH_CALIBRATION_H
