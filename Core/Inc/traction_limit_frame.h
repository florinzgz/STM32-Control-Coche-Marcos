/**
  ****************************************************************************
  * @file    traction_limit_frame.h
  * @brief   Pure pack/unpack contract for 0x31A traction-limit diagnostics.
  *
  * Wire layout (STM32 -> ESP32, DLC 4, 1000 ms):
  *   b0 obstacle_scale_pct  (0..100)
  *   b1 traction_cap_pct    (0..100)
  *   b2 brake_release_pct   (0..100)
  *   b3 obstacle_state      (ObstacleState_t wire value 0..5)
  ****************************************************************************
  */
#ifndef TRACTION_LIMIT_FRAME_H
#define TRACTION_LIMIT_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define TRACTION_LIMIT_FRAME_DLC 4U

typedef struct {
    uint8_t obstacle_scale_pct;
    uint8_t traction_cap_pct;
    uint8_t brake_release_pct;
    uint8_t obstacle_state;
} TractionLimitFrame;

static inline uint8_t TractionLimitFrame_SatPct(float value)
{
    if (!isfinite(value) || value <= 0.0f) return 0U;
    if (value >= 100.0f) return 100U;
    return (uint8_t)(value + 0.5f);
}

static inline void TractionLimitFrame_Pack(float obstacle_scale,
                                           float traction_cap,
                                           float brake_release_pct,
                                           uint8_t obstacle_state,
                                           uint8_t out[TRACTION_LIMIT_FRAME_DLC])
{
    if (out == NULL) return;
    out[0] = TractionLimitFrame_SatPct(obstacle_scale * 100.0f);
    out[1] = TractionLimitFrame_SatPct(traction_cap * 100.0f);
    out[2] = TractionLimitFrame_SatPct(brake_release_pct);
    out[3] = obstacle_state;
}

static inline void TractionLimitFrame_Unpack(
    const uint8_t in[TRACTION_LIMIT_FRAME_DLC], TractionLimitFrame *out)
{
    if (in == NULL || out == NULL) return;
    out->obstacle_scale_pct = in[0];
    out->traction_cap_pct = in[1];
    out->brake_release_pct = in[2];
    out->obstacle_state = in[3];
}

#ifdef __cplusplus
}
#endif
#endif /* TRACTION_LIMIT_FRAME_H */
