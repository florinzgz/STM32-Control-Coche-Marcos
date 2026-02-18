/**
  ****************************************************************************
  * @file    math_safety.h
  * @brief   Unified numeric safety utilities for float→integer conversion,
  *          NaN/Inf handling, and clamping.
  *
  *          Every float-to-integer cast and telemetry scaling in the
  *          firmware should use these helpers to guarantee deterministic
  *          behaviour across toolchains (no undefined wrap-around, no
  *          propagation of NaN/Inf).
  ****************************************************************************
  */

#ifndef MATH_SAFETY_H
#define MATH_SAFETY_H

#include <stdint.h>
#include <math.h>

/**
 * @brief  Sanitize a float value: return @p def if @p v is NaN or Inf.
 * @param  v    Input value.
 * @param  def  Safe default returned when @p v is not finite.
 * @retval Sanitized float.
 */
float sanitize_float(float v, float def);

/**
 * @brief  Convert a float to uint16_t with saturation.
 *         NaN / Inf / negative → 0.  Values > 65535 → 65535.
 * @param  v  Input value.
 * @retval Clamped uint16_t.
 */
uint16_t float_to_u16_clamped(float v);

/**
 * @brief  Multiply a float by @p scale, then convert to uint16_t with
 *         saturation.  NaN / Inf / negative → 0.
 * @param  v      Input value.
 * @param  scale  Multiplicative scale factor applied before conversion.
 * @retval Clamped uint16_t.
 */
uint16_t float_scaled_to_u16(float v, float scale);

/**
 * @brief  Convert a float to uint8_t, clamped to [0, @p max].
 *         NaN / Inf / negative → 0.
 * @param  v    Input value.
 * @param  max  Upper bound (inclusive), clamped to 255 internally.
 * @retval Clamped uint8_t.
 */
uint8_t float_to_u8_clamped(float v, float max);

/**
 * @brief  Clamp a float to [@p lo, @p hi].
 *         NaN / Inf → @p lo (safe side).
 * @param  v   Input value.
 * @param  lo  Lower bound.
 * @param  hi  Upper bound.
 * @retval Clamped float.
 */
float clampf(float v, float lo, float hi);

#endif /* MATH_SAFETY_H */
