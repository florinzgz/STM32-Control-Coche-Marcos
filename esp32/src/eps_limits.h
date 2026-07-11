/**
 ****************************************************************************
 * @file    eps_limits.h
 * @brief   Authoritative EPS parameter limit contract (single source of
 *          truth for the HMI editor).
 *
 *          One contract, enforced in two places:
 *            - HMI editor ranges  : engineering_screen.cpp drawEpsTuning()
 *              kRows[] reference eps::LIMITS[...] (this file).
 *            - STM32 server-side  : Core/Src/eps_params.c eps_limits[] MUST
 *              hold the identical [min, max] for every parameter — a raw CAN
 *              EPS_PARAM_OP_SET_PARAM frame is validated against it, so the
 *              server must never accept a value the HMI would not allow
 *              (no raw-CAN frame wider than the HMI-safe range).
 *
 *          The two tables are cross-checked, per parameter, by:
 *            - esp32/src/test_eps_limits_contract.cpp (HMI vs. contract), and
 *            - Core/Src/test_eps_params.c              (STM32 vs. contract).
 *          Both mirror the numeric values below; any drift fails CI.
 *
 *          Rationale for the specific bounds (previously the two sides
 *          disagreed):
 *            - ASSIST/CENTER strength max is 1.0 (the HMI previously allowed
 *              2.0 — an over-assist range the STM32 already rejected).
 *            - FRICTION/COAST/MIN_DRIVE/ASSISTvSPD/RETURNvSPD/DEADBAND/
 *              SLEW were WIDER on the STM32 than the HMI; the server is
 *              narrowed to the HMI-safe range so a raw CAN frame cannot
 *              exceed it.
 *            - The two speed divisors keep a tiny positive minimum
 *              (MIN_POS) on BOTH sides: a zero divisor is a control hazard
 *              (1/(1+v/X) with X=0), so exactly 0.0 must stay rejectable.
 *
 *          Pure header — no Arduino/ESP-IDF dependency — so it links into the
 *          host contract tests unchanged.  Index order MUST match
 *          EPS_PARAM_* in can_ids.h / eps_param_id_t in eps_params.h.
 ****************************************************************************
 */
#ifndef EPS_LIMITS_H
#define EPS_LIMITS_H

namespace eps {

struct Limit {
    float min;   // inclusive lower bound
    float max;   // inclusive upper bound
};

// Tiny positive lower bound for parameters that must be strictly > 0
// (speed divisors).  Mirrors EPS_MIN_POS in Core/Src/eps_params.c.
constexpr float MIN_POS = 1e-6f;

// Authoritative [min, max] per parameter, indexed by EPS_PARAM_* (0..11).
constexpr Limit LIMITS[12] = {
    /* [0]  ASSIST_STRENGTH   */ { 0.0f,     1.0f   },
    /* [1]  CENTER_STRENGTH   */ { 0.0f,     1.0f   },
    /* [2]  DAMPING           */ { 0.0f,     1.0f   },
    /* [3]  FRICTION_COMP     */ { 0.0f,     0.5f   },
    /* [4]  COAST_BAND_PCT    */ { 0.0f,    20.0f   },
    /* [5]  MIN_DRIVE_PCT     */ { 1.0f,    50.0f   },
    /* [6]  ASSIST_VS_SPEED   */ { MIN_POS, 100.0f  },  // divisor > 0
    /* [7]  RETURN_VS_SPEED   */ { MIN_POS, 100.0f  },  // divisor > 0
    /* [8]  DEADBAND_DEG      */ { 0.1f,    10.0f   },
    /* [9]  MAX_PWM_PCT       */ { 5.0f,   100.0f   },
    /* [10] SLEW_RATE_PCT     */ { 0.1f,    20.0f   },
    /* [11] CENTER_OFFSET_DEG */ { -10.0f,  10.0f   },
};

constexpr int LIMIT_COUNT = sizeof(LIMITS) / sizeof(LIMITS[0]);

}  // namespace eps

#endif  // EPS_LIMITS_H
