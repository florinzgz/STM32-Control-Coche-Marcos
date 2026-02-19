/**
  ****************************************************************************
  * @file    math_safety.c
  * @brief   Unified numeric safety utilities — canonical implementations.
  *
  *          Rules enforced by every helper:
  *            – NaN or Inf  → return the safe default / 0
  *            – Negative    → clamp to 0 (unless explicitly allowed)
  *            – No unsigned wrap-around from unchecked float casts
  ****************************************************************************
  */

#include "math_safety.h"

/* ------------------------------------------------------------------ */

float sanitize_float(float v, float def)
{
    if (isnan(v) || isinf(v)) {
        return def;
    }
    return v;
}

/* ------------------------------------------------------------------ */

uint16_t float_to_u16_clamped(float v)
{
    if (isnan(v) || isinf(v) || v < 0.0f) {
        return 0U;
    }
    if (v > 65535.0f) {
        return 65535U;
    }
    return (uint16_t)v;
}

/* ------------------------------------------------------------------ */

uint16_t float_scaled_to_u16(float v, float scale)
{
    if (isnan(v) || isinf(v)) {
        return 0U;
    }
    if (isnan(scale) || isinf(scale)) {
        return 0U;
    }
    float product = v * scale;
    if (isnan(product) || isinf(product)) {
        return 0U;
    }
    if (product < 0.0f) {
        return 0U;
    }
    if (product > 65535.0f) {
        return 65535U;
    }
    return (uint16_t)product;
}

/* ------------------------------------------------------------------ */

uint8_t float_to_u8_clamped(float v, float max)
{
    /* Sanitize max: NaN/Inf → 0, then clamp to [0, 255] */
    if (isnan(max) || isinf(max)) {
        max = 0.0f;
    }
    if (max > 255.0f) {
        max = 255.0f;
    }
    if (max < 0.0f) {
        max = 0.0f;
    }
    /* Sanitize v: NaN/Inf → 0, then clamp to [0, max] */
    if (isnan(v) || isinf(v)) {
        v = 0.0f;
    }
    if (v < 0.0f) {
        v = 0.0f;
    }
    if (v > max) {
        v = max;
    }
    return (uint8_t)v;
}

/* ------------------------------------------------------------------ */

float clampf(float v, float lo, float hi)
{
    /* Sanitize bounds: if lo or hi are not finite, fall back to 0 */
    if (isnan(lo) || isinf(lo)) {
        lo = 0.0f;
    }
    if (isnan(hi) || isinf(hi)) {
        hi = 0.0f;
    }
    /* Guarantee lo <= hi: swap if inverted */
    if (lo > hi) {
        float tmp = lo;
        lo = hi;
        hi = tmp;
    }
    if (isnan(v) || isinf(v)) {
        return lo;
    }
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}
