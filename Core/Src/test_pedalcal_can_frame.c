/**
  ****************************************************************************
  * @file    test_pedalcal_can_frame.c
  * @brief   Integration test for the productive PedalCalSession <-> CAN seam
  *          (audit Problem 5 / Problem 6).
  *
  *          Where test_pedal_cal_session.c exercises the FSM in isolation, this
  *          test drives the REAL pedal_cal_session.c through the HMI command
  *          flow (BEGIN -> CAPTURE MIN -> CAPTURE MAX pressed -> SAVE, plus a
  *          movement abort) and encodes the 0x319 DIAG_PEDAL_CAL_SESSION frame
  *          EXACTLY as can_handler.c::pedalcal_send_session_status() does, then
  *          decodes it the way esp32/src/can_rx.cpp::decodePedalCalSession()
  *          does.  This locks the integration contract:
  *            - STM32 PedalCalState enum values == ESP32 wire constants;
  *            - reason bitmask crosses CAN unchanged;
  *            - captured MIN/MAX endpoints survive encode/decode;
  *            - flags (active/have_min/have_max/completed/aborted) are correct;
  *            - CAPTURE MAX only latches with the pedal objectively pressed;
  *            - SAVE only after release + readback verify.
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -Wall -Wextra -Werror \
  *                Core/Src/test_pedalcal_can_frame.c \
  *                Core/Src/pedal_cal_session.c -lm \
  *                -o /tmp/test_pedalcal_can_frame
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <string.h>

#include "pedal_cal_session.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- ESP32-side wire constants (mirror esp32/include/can_ids.h) ---------- */
enum {
    W_IDLE = 0, W_ENTERING = 1, W_WAIT_RELEASED = 2, W_CAPTURING_MIN = 3,
    W_WAIT_FULL_PRESS = 4, W_CAPTURING_MAX = 5, W_WAIT_RELEASE_FOR_SAVE = 6,
    W_READY_TO_SAVE = 7, W_SAVING = 8, W_COMPLETED = 9, W_ABORTED = 10
};

/* ---- Fake persistence backing store (readback verified) ------------------ */
static uint16_t g_store_min, g_store_max;
static bool     g_persist_ok  = true;
static bool     g_readback_ok = true;

static bool fake_validate(uint16_t mn, uint16_t mx)
{
    return (mx > mn) && ((uint32_t)(mx - mn) >= 800U) && (mx <= 4094U);
}
static bool fake_persist(uint16_t mn, uint16_t mx)
{
    if (!g_persist_ok) return false;
    g_store_min = mn; g_store_max = mx;
    return true;
}
static void fake_apply(uint16_t mn, uint16_t mx) { (void)mn; (void)mx; }
static bool fake_readback(uint16_t *mn, uint16_t *mx)
{
    if (!g_readback_ok) return false;
    *mn = g_store_min; *mx = g_store_max;
    return true;
}
static PedalCalSessionHooks make_hooks(void)
{
    PedalCalSessionHooks h;
    h.validate = fake_validate;
    h.persist  = fake_persist;
    h.apply    = fake_apply;
    h.readback = fake_readback;
    return h;
}

static PedalCalConds base_conds(uint32_t t)
{
    PedalCalConds c;
    memset(&c, 0, sizeof(c));
    c.now_ms               = t;
    c.in_standby           = true;
    c.gear_park_or_neutral = true;
    c.wheels_moving        = false;
    c.pedal_plausible      = true;
    c.pedal_released       = true;
    c.pedal_pressed_full   = false;
    c.pedal_raw            = 40;
    c.critical_error       = false;
    c.safe_state           = false;
    c.emergency            = false;
    c.can_loss             = false;
    c.traction_inhibited   = true;
    c.traction_locked      = true;
    return c;
}

static void run_ticks(PedalCalSession *s, PedalCalConds *c, int n,
                      uint16_t raw, uint32_t dt)
{
    for (int i = 0; i < n; i++) {
        c->pedal_raw = raw;
        PedalCalSession_Update(s, c);
        c->now_ms += dt;
    }
}

/* ---- 0x319 encode: byte-for-byte copy of can_handler.c builder ----------- */
struct Frame319 { uint8_t d[8]; };
static struct Frame319 encode_0x319(const PedalCalSession *s, bool entry_ok_now)
{
    struct Frame319 f;
    uint8_t flags = 0U;
    if (PedalCalSession_Active(s))       flags |= 0x01U;
    if (s->have_min)                     flags |= 0x02U;
    if (s->have_max)                     flags |= 0x04U;
    if (s->state == PEDAL_CAL_COMPLETED) flags |= 0x08U;
    if (s->state == PEDAL_CAL_ABORTED)   flags |= 0x10U;
    if (entry_ok_now)                    flags |= 0x20U;
    /* audit P5: OPERATOR cancel and LOCK_LOST live above the 16-bit reason
     * word, so they are surfaced to the HMI via dedicated flag bits. */
    if (s->reason & PEDAL_CAL_ABORT_OPERATOR)  flags |= 0x40U;
    if (s->reason & PEDAL_CAL_ABORT_LOCK_LOST) flags |= 0x80U;

    f.d[0] = (uint8_t)s->state;
    f.d[1] = flags;
    f.d[2] = (uint8_t)(s->reason & 0xFFU);
    f.d[3] = (uint8_t)((s->reason >> 8) & 0xFFU);
    f.d[4] = (uint8_t)(s->adc_min & 0xFFU);
    f.d[5] = (uint8_t)((s->adc_min >> 8) & 0xFFU);
    f.d[6] = (uint8_t)(s->adc_max & 0xFFU);
    f.d[7] = (uint8_t)((s->adc_max >> 8) & 0xFFU);
    return f;
}

/* ---- ESP32 decode: mirror of can_rx.cpp decodePedalCalSession() ---------- */
struct Decoded { uint8_t state, flags; uint16_t reason, adcMin, adcMax; };
static struct Decoded decode_0x319(const struct Frame319 *f)
{
    struct Decoded d;
    d.state  = f->d[0];
    d.flags  = f->d[1];
    d.reason = (uint16_t)(f->d[2] | (f->d[3] << 8));
    d.adcMin = (uint16_t)(f->d[4] | (f->d[5] << 8));
    d.adcMax = (uint16_t)(f->d[6] | (f->d[7] << 8));
    return d;
}

/* The STM32 enum must equal the ESP32 wire constant, byte for byte. */
static void test_enum_contract(void)
{
    CHECK((uint8_t)PEDAL_CAL_IDLE                == W_IDLE);
    CHECK((uint8_t)PEDAL_CAL_ENTERING            == W_ENTERING);
    CHECK((uint8_t)PEDAL_CAL_WAIT_RELEASED       == W_WAIT_RELEASED);
    CHECK((uint8_t)PEDAL_CAL_CAPTURING_MIN       == W_CAPTURING_MIN);
    CHECK((uint8_t)PEDAL_CAL_WAIT_FULL_PRESS     == W_WAIT_FULL_PRESS);
    CHECK((uint8_t)PEDAL_CAL_CAPTURING_MAX       == W_CAPTURING_MAX);
    CHECK((uint8_t)PEDAL_CAL_WAIT_RELEASE_FOR_SAVE == W_WAIT_RELEASE_FOR_SAVE);
    CHECK((uint8_t)PEDAL_CAL_READY_TO_SAVE       == W_READY_TO_SAVE);
    CHECK((uint8_t)PEDAL_CAL_SAVING              == W_SAVING);
    CHECK((uint8_t)PEDAL_CAL_COMPLETED           == W_COMPLETED);
    CHECK((uint8_t)PEDAL_CAL_ABORTED             == W_ABORTED);
}

/* Full HMI flow, encoding + decoding 0x319 at each milestone. */
static void test_full_flow_frames(void)
{
    g_persist_ok = true; g_readback_ok = true;

    PedalCalSession s;
    PedalCalSessionHooks h = make_hooks();
    PedalCalSession_Init(&s, NULL, &h);

    PedalCalConds c = base_conds(1000);

    /* BEGIN (HMI CAPTURE MIN button, opcode 0x01). */
    CHECK(PedalCalSession_Begin(&s, &c) == true);
    PedalCalSession_Update(&s, &c); c.now_ms += 50;   /* ENTERING->WAIT_RELEASED */
    PedalCalSession_Update(&s, &c); c.now_ms += 50;   /* ->CAPTURING_MIN */
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_CAPTURING_MIN);

    /* Frame while capturing MIN: active + entry-ok, no endpoints yet. */
    {
        struct Frame319 f = encode_0x319(&s, true);
        struct Decoded d = decode_0x319(&f);
        CHECK(d.state == W_CAPTURING_MIN);
        CHECK((d.flags & 0x01U) != 0);   /* active */
        CHECK((d.flags & 0x20U) != 0);   /* entry ok */
        CHECK((d.flags & 0x02U) == 0);   /* no min yet */
        CHECK(d.reason == 0);
    }

    /* 8 stable released samples -> MIN latched. */
    run_ticks(&s, &c, 8, 42, 50);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_WAIT_FULL_PRESS);
    CHECK(s.have_min);

    /* CAPTURE MAX must NOT latch just because it was requested: the FSM stays
     * in WAIT_FULL_PRESS until the operator ARMS the capture. */
    run_ticks(&s, &c, 4, 42, 50);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_WAIT_FULL_PRESS);

    /* Arm + press pedal -> CAPTURING_MAX (raw-based, button-armed). */
    c.pedal_released = false; c.pedal_pressed_full = true;
    PedalCalSession_ArmCaptureMax(&s);
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_CAPTURING_MAX);
    run_ticks(&s, &c, 8, 4000, 50);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_WAIT_RELEASE_FOR_SAVE);
    CHECK(s.have_max);

    /* Release -> READY_TO_SAVE; frame carries both endpoints. */
    c.pedal_released = true; c.pedal_pressed_full = false;
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_READY_TO_SAVE);
    {
        struct Frame319 f = encode_0x319(&s, true);
        struct Decoded d = decode_0x319(&f);
        CHECK(d.state == W_READY_TO_SAVE);
        CHECK((d.flags & 0x02U) != 0);   /* have_min */
        CHECK((d.flags & 0x04U) != 0);   /* have_max */
        CHECK(d.adcMin == s.adc_min);
        CHECK(d.adcMax == s.adc_max);
        CHECK(d.adcMin < d.adcMax);
    }

    /* SAVE (HMI opcode 0x03) -> COMPLETED, readback verified. */
    PedalCalSession_RequestSave(&s, &c);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_COMPLETED);
    {
        struct Frame319 f = encode_0x319(&s, true);
        struct Decoded d = decode_0x319(&f);
        CHECK(d.state == W_COMPLETED);
        CHECK((d.flags & 0x08U) != 0);   /* completed */
        CHECK((d.flags & 0x01U) == 0);   /* not active */
        CHECK(d.reason == 0);            /* PEDAL_CAL_SESS_OK */
        CHECK(d.adcMin == g_store_min && d.adcMax == g_store_max);
    }
}

/* Movement mid-session -> ABORTED, reason ABORT_MOVEMENT crosses CAN. */
static void test_movement_abort_frame(void)
{
    PedalCalSession s;
    PedalCalSessionHooks h = make_hooks();
    PedalCalSession_Init(&s, NULL, &h);

    PedalCalConds c = base_conds(1000);
    CHECK(PedalCalSession_Begin(&s, &c) == true);
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_CAPTURING_MIN);

    /* Wheels start rolling -> immediate safe abort. */
    c.wheels_moving = true;
    PedalCalSession_Update(&s, &c);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_ABORTED);

    struct Frame319 f = encode_0x319(&s, false);
    struct Decoded d = decode_0x319(&f);
    CHECK(d.state == W_ABORTED);
    CHECK((d.flags & 0x10U) != 0);       /* aborted */
    CHECK((d.reason & 0x0200U) != 0);    /* PEDCAL_SESS_ABORT_MOVEMENT */
}

/* Readback mismatch on SAVE -> ABORTED with FAIL_READBACK on the wire. */
static void test_save_readback_fail_frame(void)
{
    g_persist_ok = true; g_readback_ok = false;   /* readback fails */

    PedalCalSession s;
    PedalCalSessionHooks h = make_hooks();
    PedalCalSession_Init(&s, NULL, &h);

    PedalCalConds c = base_conds(1000);
    CHECK(PedalCalSession_Begin(&s, &c) == true);
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    run_ticks(&s, &c, 8, 42, 50);                 /* MIN */
    c.pedal_released = false; c.pedal_pressed_full = true;
    PedalCalSession_ArmCaptureMax(&s);
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    run_ticks(&s, &c, 8, 4000, 50);               /* MAX */
    c.pedal_released = true; c.pedal_pressed_full = false;
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_READY_TO_SAVE);

    PedalCalSession_RequestSave(&s, &c);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_ABORTED);

    struct Frame319 f = encode_0x319(&s, true);
    struct Decoded d = decode_0x319(&f);
    CHECK(d.state == W_ABORTED);
    CHECK((d.reason & 0x8000U) != 0);    /* PEDCAL_SESS_FAIL_READBACK */

    g_readback_ok = true;
}

/* Operator cancel and lock-loss ride on byte-1 flag bits 6/7, NOT as a
 * 16-bit reason word (they live above bit 15) and NOT as an emergency. */
static void test_operator_and_lock_flag_bits(void)
{
    /* Operator cancel from WAIT_FULL_PRESS. */
    {
        PedalCalSession s;
        PedalCalSession_Init(&s, NULL, NULL);
        PedalCalConds c = base_conds(1000);
        CHECK(PedalCalSession_Begin(&s, &c) == true);
        PedalCalSession_Update(&s, &c); c.now_ms += 50;
        PedalCalSession_Update(&s, &c); c.now_ms += 50;
        run_ticks(&s, &c, 8, 42, 50);
        CHECK(PedalCalSession_State(&s) == PEDAL_CAL_WAIT_FULL_PRESS);
        PedalCalSession_Abort(&s, PEDAL_CAL_ABORT_OPERATOR);

        struct Frame319 f = encode_0x319(&s, false);
        struct Decoded d = decode_0x319(&f);
        CHECK(d.state == W_ABORTED);
        CHECK((d.flags & 0x40U) != 0);          /* operator cancel bit  */
        CHECK((d.flags & 0x80U) == 0);          /* not lock-lost        */
        CHECK((d.reason & 0x0004U) == 0);       /* not EMERGENCY (bit 2) */
    }

    /* Lock lost mid-session. */
    {
        PedalCalSession s;
        PedalCalSession_Init(&s, NULL, NULL);
        PedalCalConds c = base_conds(1000);
        CHECK(PedalCalSession_Begin(&s, &c) == true);
        PedalCalSession_Update(&s, &c); c.now_ms += 50;
        c.traction_locked = false;
        PedalCalSession_Update(&s, &c);
        CHECK(PedalCalSession_State(&s) == PEDAL_CAL_ABORTED);

        struct Frame319 f = encode_0x319(&s, false);
        struct Decoded d = decode_0x319(&f);
        CHECK(d.state == W_ABORTED);
        CHECK((d.flags & 0x80U) != 0);          /* lock-lost bit        */
        CHECK((d.flags & 0x40U) == 0);          /* not operator cancel  */
    }
}

int main(void)
{
    test_enum_contract();
    test_full_flow_frames();
    test_movement_abort_frame();
    test_save_readback_fail_frame();
    test_operator_and_lock_flag_bits();
    printf("pedalcal_can_frame: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#else
int main(void) { return 0; }
#endif /* HOST_TEST */
