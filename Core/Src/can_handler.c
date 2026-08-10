/**
  ****************************************************************************
  * @file    can_handler.c
  * @brief   CAN communication implementation for ESP32-STM32 link
  *
  *          The STM32 is the safety authority on the CAN bus:
  *            – RX filters accept only valid ESP32 message IDs
  *            – All received commands pass through Safety_Validate*()
  *            – Heartbeat includes system state and fault flags
  ****************************************************************************
  */

#include "can_handler.h"
#include "main.h"
#include "motor_control.h"
#include "motion_inhibit.h"
#include "safety_system.h"
#include "sensor_manager.h"
#include "service_mode.h"
#include "math_safety.h"
#include "error_log.h"
#include "busoff_recovery.h"
#include "sensor_map_store.h"
#include "pedal_cal_store.h"
#include "pedal_cal_session.h"
#include "gear_limits_store.h"
#include "drive_tuning_store.h"
#include "battery_limits_store.h"
#include "steering_cal_store.h"
#include "steering_z.h"
#include "steering_centering.h"
#include "steering_centering_frame.h"
#include "relay_health_frame.h"
#include "ina226_ch5_frame.h"
#include "traction_limit_frame.h"
#include "status_safety_frame.h"
#include "steering_service_store.h"
#include "service_diag_session.h"
#include "service_diag_frame.h"
#include "wheel_equality_test.h"
#include "wheel_equality_frame.h"
#include "encoder_reader.h"
#include "rc_arbiter.h"
#include "can_rx_policy.h"
#include "service_hold_policy.h"
#include <math.h>
#include <string.h>

/* ---- Defensive declarations ----
 * These symbols are normally provided by main.h (included via
 * can_handler.h).  Re-declared here so the build succeeds even if
 * CubeMX regenerates main.h without the custom content.           */
#ifndef PIN_RELAY_LED
#define PIN_RELAY_LED       GPIO_PIN_10  /* PB10 — front LED strip relay */
#endif
#ifndef PIN_RELAY_LED_REAR
#define PIN_RELAY_LED_REAR  GPIO_PIN_11  /* PB11 — rear  LED strip relay */
#endif
#ifndef INA226_CHANNEL_BATTERY
#define INA226_CHANNEL_BATTERY  4        /* TCA9548A channel index       */
#endif
extern bool Startup_IsInhibited(void);

/* Safe-default for speed when NaN/Inf detected — ensures gear change is rejected */
#define SANITIZE_SPEED_DEFAULT  99.0f

/* Global variables */
extern FDCAN_HandleTypeDef hfdcan1;
CAN_Stats_t    can_stats     = {0};
CAN_Diag_t     can_diag      = {0};
CAN_TxMeta_t   can_txmeta    = {0};
/* volatile: written once at init, read only via SWD debugger — volatile
 * ensures -O2 never optimises away the stores (same rationale as
 * g_CAN_RxData / g_CAN_RxHeader / g_CAN_TxData below).                */
volatile CAN_InitDiag_t can_init_diag = {0};

/* Debug-visible global CAN buffers — volatile so the debugger always
 * reads them from RAM, not from an optimised-out register.            */
volatile uint8_t              g_CAN_RxData[8] = {0};
volatile FDCAN_RxHeaderTypeDef g_CAN_RxHeader = {0};
volatile uint8_t              g_CAN_TxData[8] = {0};

/* Internal state */
static uint32_t last_tx_heartbeat = 0;
static uint8_t  heartbeat_counter = 0;

/* Heartbeat status_flags bitmasks (byte 4 of 0x001) */
#define STATUS_FLAG_STARTUP_INHIBIT  0x01U   /* Bit 0 */
#define STATUS_FLAG_MODE_4X4         0x02U   /* Bit 1 */
#define STATUS_FLAG_TANK_TURN        0x04U   /* Bit 2 */
#define STATUS_FLAG_TEMP_COUNT_SHIFT 3       /* Bits 3-5: DS18B20 count */
#define STATUS_FLAG_TEMP_COUNT_MASK  0x07U   /* 3-bit mask (0-7) */

/* LED relay states — front (PB10) and rear (PB11) — default OFF */
static bool led_relay_front = false;
static bool led_relay_rear  = false;

/* ---- DS18B20 sanity-filter thresholds (STATUS_TEMP_MAP) ----
 * Practical operating window used by CAN_SendStatusTempMap() to reject
 * glitched readings (disconnect sentinel -127 °C, NaN/Inf, CRC errors).
 * Tighter than the datasheet -55..+125 °C so that any implausible in-
 * vehicle reading is also filtered.                                   */
#define TEMP_MIN_VALID_C   (-30.0f)
#define TEMP_MAX_VALID_C   (120.0f)

/* ---- Boot-time plausibility window (R-1) ----
 * Narrower than the steady-state sanity window above.  Used ONLY during
 * the 2-sample bootstrap in CAN_SendStatusTempMap() to reject EMI-corrupted
 * samples that happen to be in-range-but-unrealistic at startup (e.g. a
 * pair of coherent 90 °C readings while the cabin is at 25 °C).  Chosen
 * to cover every realistic ambient / warm-bench condition (0..60 °C) but
 * exclude values that a motor winding would only reach under sustained
 * load — which cannot occur in the first few seconds after boot, before
 * any drive command has been executed.  Has NO effect after the role is
 * armed; the broader TEMP_MIN_VALID_C / TEMP_MAX_VALID_C window continues
 * to gate steady-state samples exactly as before.                       */
#define BOOT_MIN_VALID_C   (0.0f)
#define BOOT_MAX_VALID_C   (60.0f)

/* SAFETY FIX: ESP32 heartbeat alive-counter freeze detection.
 * Track the rolling counter sent in byte 0 of the ESP32 heartbeat (0x011).
 * A "zombie" ESP32 (main task frozen but timer ISR still firing) sends the
 * same counter value indefinitely.  If the counter does not advance for
 * HEARTBEAT_COUNTER_FREEZE_COUNT consecutive frames the liveness watchdog
 * is NOT updated, causing a normal CAN timeout → LIMP_HOME transition.
 * Recovery: when the counter resumes changing, the freeze is cleared.      */
#define HEARTBEAT_COUNTER_FREEZE_COUNT  5U  /* 5 × 100 ms = 500 ms frozen  */
static uint8_t  esp32_hb_last_counter    = 0xFFU; /* Init to impossible value */
static uint8_t  esp32_hb_same_count      = 0;     /* Consecutive same-counter */

/* Bus-off recovery state (non-blocking, timestamp-based) */
static uint8_t  busoff_active       = 0;    /* 1 = bus-off episode active (recovering or awaiting confirmation) */
static uint32_t busoff_last_attempt = 0;    /* Timestamp of last recovery attempt         */
static uint8_t  busoff_retry_count  = 0;    /* Number of recovery attempts since bus-off  */
/* Heartbeat-confirmation window: recovery is only declared once valid
 * ESP32 heartbeats are sustained over CAN_BUSOFF_RECOVERY_WINDOW_MS
 * (see busoff_recovery.h).  During this "probation" phase busoff_active
 * stays 1 (fault still latched) but no further re-init is attempted.    */
static BusOffRecoveryWindow_t busoff_window = { false, false, 0U };

/* SAFETY FIX: Global CAN watchdog — track last received CAN message (any ID).
 * Unlike last_can_rx_time (heartbeat only), this tracks ANY received frame.
 * If the CAN bus dies completely (no frames at all), this timestamp goes stale.
 * Checked by CAN_IsGlobalSilent() to detect total bus silence.               */
static uint32_t last_any_rx_tick = 0;
/* CAN global silence threshold (ms).  If no CAN frames of any kind arrive
 * within this window, the bus is considered dead.  Set wider than heartbeat
 * timeout (250 ms) because non-heartbeat frames may arrive at different rates.
 * 1000 ms = 1 second of complete silence.                                     */
#define CAN_GLOBAL_SILENCE_MS  1000U

/* Consecutive TX failure threshold before raising diagnostic flag.
 * 5 consecutive failures (500 ms of failed heartbeats at 100 ms rate)
 * strongly indicates the bus has no peer to ACK frames.                */
#define CAN_TX_NACK_THRESHOLD  5U

/* Saturating increment for uint32_t counters.
 * Prevents wrap-around to 0 after 4 billion events, which would
 * clear the saturated diagnostic byte in CAN_SendStatusSafety.   */
static inline void sat_inc_u32(uint32_t *counter) {
    if (*counter < UINT32_MAX) (*counter)++;
}

/* ---- Software TX queue (additive burst-absorber) -------------------------
 * The STM32G4 FDCAN hardware TX FIFO holds only 3 elements.  Both the 1 Hz
 * diagnostic block and the 100 ms status block in main.c enqueue many frames
 * in a single (microsecond-fast) main-loop pass, far quicker than the bus can
 * drain them (~250 us/frame at 500 kbit/s).  The previous implementation
 * dropped every frame that arrived once the 3-slot FIFO was full, which
 * reliably killed the diagnostic frames emitted last in the burst (0x309,
 * 0x30A) and the on-demand scan replies (0x30B/0x30C/ACK) that collided with
 * a full FIFO — observed as "0x309 rx=0", "0x30A no data" and "SCAN TIMEOUT".
 *
 * Instead, every frame is appended to this software ring buffer and drained
 * one-by-one into the hardware FIFO by CAN_TxPump() as slots free up (called
 * on every TransmitFrame() and once per main-loop iteration).  Frames are
 * therefore never lost to a momentarily-full hardware FIFO; they are merely
 * transmitted a few milliseconds later, well inside their 100 ms / 1 s window.
 * A frame is only dropped (and counted in tx_fifo_full_drops) on a true
 * software-queue overflow, which cannot happen at the current frame rates.
 *
 * Additive + report-only: frame IDs, DLCs and send order are preserved, the
 * main loop never blocks, and no control/safety path depends on this queue.
 * Single producer + single consumer, both in main-loop context (no ISR ever
 * calls TransmitFrame), so no locking is required.                          */
#define CAN_TXQ_SIZE 32U   /* power of two; absorbs the worst 100ms+1Hz burst */

typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
} CanTxItem_t;

static CanTxItem_t can_txq[CAN_TXQ_SIZE];
static uint8_t      can_txq_head;   /* next slot to write */
static uint8_t      can_txq_tail;   /* next slot to read  */

/* Push a frame onto the hardware TX FIFO.  Returns the HAL status. */
static HAL_StatusTypeDef CAN_TxHwSend(uint32_t msg_id, const uint8_t *payload, uint8_t len) {
    FDCAN_TxHeaderTypeDef tx_hdr = {0};

    /* Map byte count to FDCAN DLC code */
    uint32_t dlc_code;
    switch (len) {
        case 0:  dlc_code = FDCAN_DLC_BYTES_0; break;
        case 1:  dlc_code = FDCAN_DLC_BYTES_1; break;
        case 2:  dlc_code = FDCAN_DLC_BYTES_2; break;
        case 3:  dlc_code = FDCAN_DLC_BYTES_3; break;
        case 4:  dlc_code = FDCAN_DLC_BYTES_4; break;
        case 5:  dlc_code = FDCAN_DLC_BYTES_5; break;
        case 6:  dlc_code = FDCAN_DLC_BYTES_6; break;
        case 7:  dlc_code = FDCAN_DLC_BYTES_7; break;
        case 8:  dlc_code = FDCAN_DLC_BYTES_8; break;
        default: dlc_code = FDCAN_DLC_BYTES_8; break; /* Clamp to max */
    }

    tx_hdr.Identifier = msg_id;
    tx_hdr.IdType = FDCAN_STANDARD_ID;
    tx_hdr.TxFrameType = FDCAN_DATA_FRAME;
    tx_hdr.DataLength = dlc_code;
    tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
    tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
    tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_hdr.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_hdr, (uint8_t *)payload);
}

/**
 * @brief  Drain the software TX queue into the hardware TX FIFO.
 *
 * Non-blocking: moves frames from the ring buffer to the FDCAN FIFO while a
 * hardware slot is free, then returns.  Whatever does not fit stays queued
 * and is retried on the next call.  Must be called frequently — on every
 * TransmitFrame() (opportunistic) and once per main-loop iteration so the
 * queue keeps draining even when no new frames are being produced.
 */
void CAN_TxPump(void) {
    while (can_txq_head != can_txq_tail) {
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
            break;  /* hardware FIFO full — retry on next pass */
        }

        const CanTxItem_t *it = &can_txq[can_txq_tail];
        HAL_StatusTypeDef result = CAN_TxHwSend(it->id, it->data, it->len);

        if (result == HAL_OK) {
            sat_inc_u32(&can_stats.tx_count);
            can_diag.tx_consec_fail = 0;
            can_diag.tx_nack_flag   = 0;
            can_txq_tail = (uint8_t)((can_txq_tail + 1U) & (CAN_TXQ_SIZE - 1U));
        } else {
            /* Transient HAL error (e.g. peripheral not ready): leave the frame
             * queued, count the failure and retry next pass.  Do not spin. */
            sat_inc_u32(&can_stats.tx_errors);
            if (can_diag.tx_consec_fail < 255U)
                can_diag.tx_consec_fail++;
            if (can_diag.tx_consec_fail >= CAN_TX_NACK_THRESHOLD)
                can_diag.tx_nack_flag = 1;
            break;
        }
    }

    /* Refresh the current-occupancy gauge after draining (report-only). The
     * high-water mark is only ever raised in TransmitFrame(); this keeps the
     * instantaneous depth surfaced on 0x312 honest between bursts.           */
    can_txmeta.tx_queue_depth =
        (uint8_t)((can_txq_head - can_txq_tail) & (CAN_TXQ_SIZE - 1U));
}

/* Internal helper to send a CAN frame.
 *
 * Appends the frame to the software TX queue (preserving submission order) and
 * opportunistically pumps the queue into the hardware FIFO.  Returns HAL_OK
 * when the frame was accepted for transmission (queued) and HAL_BUSY only on a
 * genuine software-queue overflow, which is then counted in tx_fifo_full_drops.
 */
static HAL_StatusTypeDef TransmitFrame(uint32_t msg_id, const uint8_t *payload, uint32_t len) {
    uint8_t l = (len > 8U) ? 8U : (uint8_t)len;
    uint8_t next = (uint8_t)((can_txq_head + 1U) & (CAN_TXQ_SIZE - 1U));

    if (next == can_txq_tail) {
        /* Software queue full — true overflow (producer outran the bus for an
         * extended period).  Record the drop so 0x30A "tx_fifo_full_drops"
         * stays meaningful; never silently lose a frame without counting. */
        sat_inc_u32(&can_stats.tx_errors);
        sat_inc_u32(&can_txmeta.tx_fifo_full_drops);
        if (can_diag.tx_consec_fail < 255U)
            can_diag.tx_consec_fail++;
        if (can_diag.tx_consec_fail >= CAN_TX_NACK_THRESHOLD)
            can_diag.tx_nack_flag = 1;
        /* Still try to make room for next time. */
        CAN_TxPump();
        return HAL_BUSY;
    }

    can_txq[can_txq_head].id  = msg_id;
    can_txq[can_txq_head].len = l;
    memcpy(can_txq[can_txq_head].data, payload, l);
    can_txq_head = next;

    /* Observability G: track software TX ring occupancy so a heartbeat-vs-
     * backlog stall would be visible on 0x312.  Depth is measured after the
     * push (before the opportunistic drain below), capturing the worst case.
     * Report-only; never gates control or safety.                          */
    {
        uint8_t depth = (uint8_t)((can_txq_head - can_txq_tail) & (CAN_TXQ_SIZE - 1U));
        can_txmeta.tx_queue_depth = depth;
        if (depth > can_txmeta.tx_queue_depth_max)
            can_txmeta.tx_queue_depth_max = depth;
    }

    /* Opportunistically drain so low-rate single frames go out immediately. */
    CAN_TxPump();
    return HAL_OK;
}

/* ================================================================== */
/*  CAN RX Filter Configuration                                       */
/* ================================================================== */

/**
 * @brief  Configure FDCAN RX filters.
 *
 * When CAN_LOOPBACK_TEST is defined and non-zero, a single range filter
 * accepts ALL standard IDs (0x000–0x7FF) so that self-test / loopback
 * frames are received.  Otherwise the production filters restrict
 * acceptance to known ESP32 message IDs.
 */
static void CAN_ConfigureFilters(void)
{
    FDCAN_FilterTypeDef filter = {0};

#if defined(CAN_LOOPBACK_TEST) && CAN_LOOPBACK_TEST
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0U;
    filter.FilterType   = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000U;
    filter.FilterID2    = 0x7FFU;
    (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
    can_init_diag.filter_global = (uint8_t)HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
#else
    static const uint16_t accepted_ids[] = {
        CAN_ID_HEARTBEAT_ESP32,
        CAN_ID_CMD_THROTTLE,
        CAN_ID_CMD_STEERING,
        CAN_ID_CMD_MODE,
#if RC_OVERRIDE_ENABLED
        CAN_ID_CMD_RC_OVERRIDE,
#endif
        CAN_ID_SERVICE_CMD,
        CAN_ID_OBSTACLE_DISTANCE,
        CAN_ID_OBSTACLE_SAFETY,
        CAN_ID_CMD_LED,
        CAN_ID_CMD_SYSTEM_SHUTDOWN,
        CAN_ID_CMD_SENSOR_MAP_TEMP
    };
    const uint8_t count = (uint8_t)(sizeof(accepted_ids) / sizeof(accepted_ids[0]));
    for (uint8_t i = 0U, fi = 0U; i < count; i = (uint8_t)(i + 2U), ++fi) {
        filter.IdType       = FDCAN_STANDARD_ID;
        filter.FilterIndex  = fi;
        filter.FilterType   = FDCAN_FILTER_DUAL;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1    = accepted_ids[i];
        filter.FilterID2    = (i + 1U < count) ? accepted_ids[i + 1U]
                                               : accepted_ids[i];
        if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
            can_init_diag.filter_global = (uint8_t)HAL_ERROR;
            return;
        }
    }

    /* Reject every non-whitelisted standard ID and every extended ID. */
    can_init_diag.filter_global = (uint8_t)HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
#endif
}

/* ================================================================== */
/*  Public API                                                         */
/* ================================================================== */

void CAN_Init(void) {
    /* Reset statistics */
    can_stats.tx_count = 0;
    can_stats.rx_count = 0;
    can_stats.tx_errors = 0;
    can_stats.rx_errors = 0;
    can_stats.last_heartbeat_esp32 = HAL_GetTick();
    can_stats.busoff_count = 0;
    can_stats.fifo_overflow_count = 0;
    heartbeat_counter = 0;

    /* Reset bus-off recovery state */
    busoff_active       = 0;
    busoff_last_attempt = 0;
    busoff_retry_count  = 0;
    BusOffRecoveryWindow_Reset(&busoff_window);
    last_any_rx_tick    = HAL_GetTick();  /* Global CAN watchdog init */

    /* SAFETY FIX: Reset ESP32 heartbeat counter tracking state */
    esp32_hb_last_counter = 0xFFU;
    esp32_hb_same_count   = 0;

    /* SAFETY FIX: Clear started flag early — will only be set to 1U
     * at the very end if ALL steps succeed.  This ensures every
     * failure path (explicit return or fall-through) leaves
     * can_init_diag.started == 0U.                                    */
    can_init_diag.started = 0U;

    /* Skip hardware activation if FDCAN peripheral init failed.
     * System continues without CAN — Safety_CheckCANTimeout() will
     * detect the missing heartbeat and keep the system in STANDBY.  */
    extern bool fdcan_init_ok;
    if (!fdcan_init_ok) {
        can_init_diag.hal_init = 1U;  /* Record: HAL_FDCAN_Init failed */
        return;
    }
    can_init_diag.hal_init = 0U;      /* HAL_FDCAN_Init succeeded      */

    /* Snapshot RCC_CCIPR for SWD debugging — shows actual FDCANSEL
     * and all other peripheral clock mux bits at CAN_Init() entry.    */
    can_init_diag.ccipr_raw = RCC->CCIPR;

    /* Verify that the FDCAN kernel clock is sourced from PCLK1.
     * After reset FDCANSEL defaults to 00 = HSE (which is not enabled
     * in this project).  SystemClock_Config() must have already set it
     * to PCLK1 via RCC_FDCANCLKSOURCE_PCLK1 (CCIPR bits [25:24] = 10)
     * before this point.  A wrong source means the bit-timing registers
     * produce a different baud rate, causing ACK / stuff errors.
     *
     * Resilience: if the clock source has reverted to its reset default
     * (e.g. due to silicon-specific behaviour during the force-reset
     * sequence in MspInit), re-apply PCLK1 with proper barriers and
     * verify it sticks.  This avoids a permanent CAN-disabled state
     * on silicon revisions where CCIPR.FDCANSEL does not latch
     * reliably during the MspInit reset window.                       */
    can_init_diag.clk_reapplied = 0U;
    if (__HAL_RCC_GET_FDCAN_SOURCE() != RCC_FDCANCLKSOURCE_PCLK1) {
        __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PCLK1);
        __DSB();
        can_init_diag.clk_reapplied = 1U;

        /* If the clock source was wrong, MX_FDCAN1_Init's "success" was
         * likely a false positive (CCCR garbage happened to pass the
         * sanity check while the peripheral was unclocked).  Re-init
         * the FDCAN peripheral now that the clock source is correct.
         * DeInit resets the HAL state to RESET, enabling a fresh
         * MspInit call inside HAL_FDCAN_Init that re-enables the APB
         * clock and stabilises the peripheral.                         */
        HAL_FDCAN_DeInit(&hfdcan1);
        /* 2 ms delay matches FDCAN_INITIAL_SETTLE_DELAY_MS used in
         * MX_FDCAN1_Init — provides margin for the APB1 bus bridge
         * pipeline to fully propagate the CCIPR clock-source change
         * before HAL_FDCAN_Init accesses peripheral registers.        */
        HAL_Delay(2);
        if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
            fdcan_init_ok = false;
            return;  /* Re-init failed — CAN disabled */
        }

        /* Triple-read CCCR sanity check — same as MX_FDCAN1_Init.
         * After clock-source re-apply the peripheral may still return
         * stale AHB bus data.  Three consecutive reads must agree AND
         * show INIT=1 with reserved upper bits zero.                   */
        #define FDCAN_CCCR_RESERVED_MASK  0xFFFF0000U  /* Bits 16-31 reserved */
        {
            // cppcheck-suppress duplicateAssignExpression
            uint32_t c1 = hfdcan1.Instance->CCCR;
            // cppcheck-suppress duplicateAssignExpression
            uint32_t c2 = hfdcan1.Instance->CCCR;
            uint32_t c3 = hfdcan1.Instance->CCCR;
            if (c1 != c2 || c2 != c3 ||
                (c1 & FDCAN_CCCR_INIT) == 0U ||
                (c1 & FDCAN_CCCR_RESERVED_MASK) != 0U) {
                fdcan_init_ok = false;
                return;  /* CCCR still garbage after clock re-apply */
            }
        }
    }
    can_init_diag.clk_ok =
        (__HAL_RCC_GET_FDCAN_SOURCE() == RCC_FDCANCLKSOURCE_PCLK1) ? 1U : 0U;

    if (!can_init_diag.clk_ok) {
        fdcan_init_ok = false;
        return;  /* Wrong kernel clock — CAN baud rate would be wrong */
    }

    /* Verify APB1 clock is still enabled — belt-and-suspenders check.
     * If the clock gate dropped since MspInit, all subsequent register
     * accesses would return garbage.                                   */
    if (!(RCC->APB1ENR1 & RCC_APB1ENR1_FDCANEN)) {
        __HAL_RCC_FDCAN_CLK_ENABLE();
        __DSB();
        if (!(RCC->APB1ENR1 & RCC_APB1ENR1_FDCANEN)) {
            fdcan_init_ok = false;
            return;  /* Clock enable failed — hardware fault */
        }
    }

    /* Configure RX acceptance filters */
    CAN_ConfigureFilters();

    /* If global filter setup failed, FDCAN state may be inconsistent */
    if (can_init_diag.filter_global != HAL_OK) {
        fdcan_init_ok = false;
        return;  /* Non-fatal: CAN disabled, safety timeout will engage */
    }

    /* Enable RX FIFO0 new message notifications */
    HAL_StatusTypeDef rc;
    rc = HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    can_init_diag.notify = (uint8_t)rc;
    if (rc != HAL_OK) {
        fdcan_init_ok = false;
        return;  /* Non-fatal: CAN disabled, safety timeout will engage */
    }

    /* Start FDCAN peripheral */
    rc = HAL_FDCAN_Start(&hfdcan1);
    can_init_diag.start = (uint8_t)rc;
    if (rc != HAL_OK) {
        fdcan_init_ok = false;
        return;  /* Non-fatal: CAN disabled, safety timeout will engage */
    }

    /* Confirm CCCR.INIT is cleared — the peripheral has left
     * initialisation mode and is actively participating on the bus.
     * If INIT is still set, HAL_FDCAN_Start did not complete the
     * handshake (possible stale state after abnormal reset).          */
    can_init_diag.cccr_init_ok =
        ((hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U) ? 1U : 0U;

    if (!can_init_diag.cccr_init_ok) {
        fdcan_init_ok = false;
        return;  /* Peripheral stuck in INIT — bus not operational */
    }

    can_init_diag.started = 1U;
}

/**
 * @brief  Send a CAN test frame (ID = CAN_ID_TEST_FRAME, DLC = 8).
 *
 * Intended to be called periodically (e.g. every 500 ms) from the main
 * loop for CAN bus validation.  In internal-loopback mode
 * (CAN_LOOPBACK_TEST = 1) the frame is echoed back to Rx FIFO0 and
 * triggers HAL_FDCAN_RxFifo0Callback, providing a complete self-test.
 * The payload is also copied to the volatile g_CAN_TxData[] buffer so
 * it can be inspected in the debugger.
 */
void CAN_TestTransmit(void) {
    uint8_t tx_buf[8] = {1, 2, 3, 4, 5, 6, 7, 8};

    for (uint8_t i = 0; i < 8; i++)
        ((volatile uint8_t *)g_CAN_TxData)[i] = tx_buf[i];

    TransmitFrame(CAN_ID_TEST_FRAME, tx_buf, 8);
}

void CAN_SendHeartbeat(void) {
    uint32_t current_time = HAL_GetTick();
    
    /* Send every 100ms */
    if ((current_time - last_tx_heartbeat) >= 100) {
        /* Per CAN protocol doc (0x001):
         *   Byte 0: alive_counter  (uint8, cyclic 0-255, rollover is intentional)
         *   Byte 1: system_state   (uint8, 0=Boot..6=LimpHome)
         *   Byte 2: fault_flags    (bitmask)
         *   Byte 3: error_code     (Safety_Error_t, specific fault ID for HMI)
         *   Byte 4: status_flags   (bitmask)
         *            bit 0: STARTUP_INHIBIT active (Power-On Movement Prevention)
         *            bit 1: 4x4 mode active  (echo of mode applied by STM32)
         *            bit 2: Tank turn active  (echo of mode applied by STM32)
         *            bit 3-5: DS18B20 sensor count (0-5)
         *   Byte 5: relay_status   (bitmask, added for relay visibility)
         *            bit 0: reserved (always 0; PC10 not connected)
         *            bit 1: TRACTION relay GPIO ON
         *            bit 2: STEER_PWR relay GPIO ON (12 V steering actuator supply)
         *            bit 7: relay sequence complete
         *
         * DLC extended from 5 to 6.  ESP32 parsers that check DLC >= 5
         * continue to work — byte 5 is additive, not breaking.          */
        const TractionState_t *ts = Traction_GetState();
        uint8_t status_flags = 0;
        if (Startup_IsInhibited()) status_flags |= STATUS_FLAG_STARTUP_INHIBIT;
        if (ts->mode4x4)          status_flags |= STATUS_FLAG_MODE_4X4;
        if (ts->axisRotation)     status_flags |= STATUS_FLAG_TANK_TURN;
        status_flags |= (uint8_t)((Temperature_GetCount() & STATUS_FLAG_TEMP_COUNT_MASK)
                                  << STATUS_FLAG_TEMP_COUNT_SHIFT);

        uint8_t payload[6];
        payload[0] = heartbeat_counter++;
        payload[1] = (uint8_t)Safety_GetState();
        payload[2] = Safety_GetFaultFlags();
        payload[3] = (uint8_t)Safety_GetError();
        payload[4] = status_flags;
        payload[5] = Safety_GetRelayStatusByte();

        if (TransmitFrame(CAN_ID_HEARTBEAT_STM32, payload, 6) == HAL_OK) {
            sat_inc_u32(&can_txmeta.hb_tx_count);
        } else {
            sat_inc_u32(&can_txmeta.hb_tx_err);
        }
        last_tx_heartbeat = current_time;
    }
}

void CAN_SendStatusSpeed(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr) {
    uint8_t speed_data[8];
    
    /* Pack wheel speeds as little-endian 16-bit values */
    speed_data[0] = (uint8_t)(fl & 0xFF);
    speed_data[1] = (uint8_t)((fl >> 8) & 0xFF);
    speed_data[2] = (uint8_t)(fr & 0xFF);
    speed_data[3] = (uint8_t)((fr >> 8) & 0xFF);
    speed_data[4] = (uint8_t)(rl & 0xFF);
    speed_data[5] = (uint8_t)((rl >> 8) & 0xFF);
    speed_data[6] = (uint8_t)(rr & 0xFF);
    speed_data[7] = (uint8_t)((rr >> 8) & 0xFF);
    
    TransmitFrame(CAN_ID_STATUS_SPEED, speed_data, 8);
}

void CAN_SendStatusCurrent(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr) {
    uint8_t current_data[8];
    
    /* Pack currents as little-endian 16-bit values */
    current_data[0] = (uint8_t)(fl & 0xFF);
    current_data[1] = (uint8_t)((fl >> 8) & 0xFF);
    current_data[2] = (uint8_t)(fr & 0xFF);
    current_data[3] = (uint8_t)((fr >> 8) & 0xFF);
    current_data[4] = (uint8_t)(rl & 0xFF);
    current_data[5] = (uint8_t)((rl >> 8) & 0xFF);
    current_data[6] = (uint8_t)(rr & 0xFF);
    current_data[7] = (uint8_t)((rr >> 8) & 0xFF);
    
    TransmitFrame(CAN_ID_STATUS_CURRENT, current_data, 8);
}

void CAN_SendStatusTemp(int8_t t1, int8_t t2, int8_t t3, int8_t t4, int8_t t5) {
    uint8_t temp_data[5];
    
    /* Pack 5 temperature values as signed bytes */
    temp_data[0] = (uint8_t)t1;
    temp_data[1] = (uint8_t)t2;
    temp_data[2] = (uint8_t)t3;
    temp_data[3] = (uint8_t)t4;
    temp_data[4] = (uint8_t)t5;
    
    TransmitFrame(CAN_ID_STATUS_TEMP, temp_data, 5);
}

void CAN_SendStatusSafety(bool abs, bool tcs, uint8_t error_code,
                          uint8_t loop_peak_100us) {
    uint8_t safety_data[STATUS_SAFETY_DLC];

    /* Layout, DLC and forward-compatibility rules live in ONE place:
     * Core/Inc/status_safety_frame.h (shared with the ESP32 decoder and
     * the host round-trip test), so the wire contract cannot drift.
     *   b0 abs, b1 tcs, b2 error_code, b3 system state,
     *   b4 saturated CAN RX error count,
     *   b5 peak 100 Hz task duration in 100 us units (observational,
     *      reset each TX cycle).
     * Receivers accepting DLC >= 5 stay valid; byte 5 is optional. */
    StatusSafetyFrame_t f;
    f.abs_active      = abs;
    f.tcs_active      = tcs;
    f.error_code      = error_code;
    f.state           = (uint8_t)Safety_GetState();
    f.rx_errors       = (can_stats.rx_errors > 255) ? 255U
                        : (uint8_t)can_stats.rx_errors;
    f.loop_peak_100us = loop_peak_100us;

    uint8_t len = StatusSafetyFrame_Pack(&f, safety_data);

    TransmitFrame(CAN_ID_STATUS_SAFETY, safety_data, len);
}

void CAN_SendStatusSteering(int16_t angle, bool calibrated) {
    uint8_t steering_data[3];
    
    /* Pack angle as signed 16-bit little-endian + calibration flag */
    steering_data[0] = (uint8_t)(angle & 0xFF);
    steering_data[1] = (uint8_t)((angle >> 8) & 0xFF);
    steering_data[2] = calibrated ? 1 : 0;
    
    TransmitFrame(CAN_ID_STATUS_STEERING, steering_data, 3);
}

/**
 * @brief  Send per-wheel ABS/TCS traction LIMIT to ESP32 (0x205).
 *
 * Exposes the ABS/TCS per-wheel scale factor already computed by the
 * safety system (safety_status.wheel_scale[0..3]).  Each value is
 * converted from float 0.0–1.0 to uint8 0–100 (percent).
 *
 * IMPORTANT (AUDIT A): this is the traction PERMITTED by ABS/TCS, NOT the
 * torque/PWM actually applied.  100 = ABS/TCS not limiting that wheel;
 * 0 = wheel fully inhibited.  It reads 100 even when the real duty is far
 * lower.  For the REAL applied effort use 0x20C (CAN_SendStatusWheelEffort).
 * Never present 0x205 as TORQUE, EMPUJE or applied-percentage.
 *
 *   Byte 0: FL ABS/TCS limit %  (0 = fully inhibited, 100 = full power allowed)
 *   Byte 1: FR ABS/TCS limit %
 *   Byte 2: RL ABS/TCS limit %
 *   Byte 3: RR ABS/TCS limit %
 *
 * CAN ID: 0x205   DLC: 4   Rate: 100 ms (10 Hz)
 */
void CAN_SendStatusTraction(void) {
    uint8_t data[4];

    for (uint8_t i = 0; i < 4; i++) {
        float s = sanitize_float(safety_status.wheel_scale[i], 0.0f);
        if (s < 0.0f) s = 0.0f;
        if (s > 1.0f) s = 1.0f;
        data[i] = (uint8_t)(s * 100.0f);
    }

    TransmitFrame(CAN_ID_STATUS_TRACTION, data, 4);
}

/**
 * @brief  Send per-wheel FINAL applied PWM effort to ESP32 (0x20C).
 *
 * AUDIT A: additive frame carrying the REAL per-wheel PWM duty written to
 * each BTS7960 this cycle (Traction_GetWheelFinalPwmPct), i.e. the value
 * that survives pedal → filter → ramp → gear response → D1/D2/R → ACTIVE/
 * DEGRADED/LIMP_HOME → speed/battery/temp/obstacle limits → Ackermann →
 * 4x2/4x4 → ABS → TCS → jerk limiter → brake-release ramp → DRIVE/BRAKE/
 * COAST → final EN state → CCR write.  COAST/BRAKE/disabled-motor → 0.
 *
 * This does NOT change or replace 0x205; it is a separate, additive contract.
 *
 *   Byte 0: FL final PWM 0-100 %
 *   Byte 1: FR final PWM 0-100 %
 *   Byte 2: RL final PWM 0-100 %
 *   Byte 3: RR final PWM 0-100 %
 *
 * CAN ID: 0x20C   DLC: 4   Rate: 100 ms (10 Hz)
 */
void CAN_SendStatusWheelEffort(void) {
    uint8_t data[4];

    for (uint8_t i = 0; i < 4; i++) {
        uint8_t p = Traction_GetWheelFinalPwmPct(i);
        if (p > 100U) p = 100U;
        data[i] = p;
    }

    TransmitFrame(CAN_ID_STATUS_WHEEL_EFFORT, data, 4);
}

/**
 * @brief  Send MOTION_INHIBIT_REASON instrumentation frame (0x315).
 *
 * Emits the bitfield computed by the traction pipeline explaining why the
 * vehicle is (or is not) producing torque, plus the demand/PWM context
 * needed to interpret it.  Instrumentation only — see motion_inhibit.h.
 *
 * CAN ID: 0x315   DLC: 8   Rate: 100 ms (10 Hz)
 */
void CAN_SendMotionInhibit(void) {
    uint8_t data[8];

    uint16_t reason = Traction_GetMotionInhibit();

    /* Clamp signed demands to the int8 range for transport. */
    const TractionState_t *ts = Traction_GetState();
    float demand_f = sanitize_float(ts ? ts->demandPct : 0.0f, 0.0f);
    float eff_f    = sanitize_float(Traction_GetEffectiveDemandPct(), 0.0f);
    if (demand_f >  100.0f) demand_f =  100.0f;
    if (demand_f < -100.0f) demand_f = -100.0f;
    if (eff_f    >  100.0f) eff_f    =  100.0f;
    if (eff_f    < -100.0f) eff_f    = -100.0f;

    uint8_t flags = 0;
    if (Safety_IsPowerReady())        flags |= 0x01;
    if (Obstacle_IsForwardBlocked())  flags |= 0x02;
    /* Relay-sequence phase (bits 2-3): commanded GPIO/sequencer state only —
     * the firmware has NO physical relay-contact feedback.                  */
    uint8_t relay_seq = MOTION_INHIBIT_RELAY_SEQ_IDLE;
    if (Safety_IsPowerReady())               relay_seq = MOTION_INHIBIT_RELAY_SEQ_COMPLETE;
    else if (Relay_IsSequenceInProgress())   relay_seq = MOTION_INHIBIT_RELAY_SEQ_IN_PROGRESS;
    flags |= (uint8_t)((relay_seq & 0x03U) << 2);
    if (Safety_GetState() == SYS_STATE_DEGRADED) {
        flags |= (uint8_t)(((uint8_t)Safety_GetDegradedLevel() & 0x0F) << 4);
    }

    data[0] = (uint8_t)(reason & 0xFF);
    data[1] = (uint8_t)((reason >> 8) & 0xFF);
    data[2] = (uint8_t)Safety_GetState();
    data[3] = (uint8_t)Traction_GetGear();
    data[4] = (uint8_t)(int8_t)demand_f;
    data[5] = (uint8_t)(int8_t)eff_f;
    data[6] = Traction_GetFinalPwmPct();
    data[7] = flags;

    TransmitFrame(CAN_ID_DIAG_MOTION_INHIBIT, data, 8);
}

/* 0x31A — post-demand traction-limit observability, 1 Hz.
 * Complements 0x315: obstacle_forward_blocked only reports a hard forward
 * block, whereas this frame exposes partial obstacle scaling, the degraded
 * traction cap, and the brake-release ramp.  Instrumentation only. */
void CAN_SendTractionLimitDiag(void)
{
    uint8_t data[TRACTION_LIMIT_FRAME_DLC] = {0};
    TractionLimitFrame_Pack(Obstacle_GetScale(),
                            Safety_GetTractionCapFactor(),
                            Traction_GetBrakeReleasePct(),
                            (uint8_t)Obstacle_GetState(),
                            data);
    TransmitFrame(CAN_ID_DIAG_TRACTION_LIMITS, data,
                  TRACTION_LIMIT_FRAME_DLC);
}


/**
 * @brief  Send live Hall pedal position to ESP32 (telemetry only).
 *
 * Publishes the real pedal travel reported by Pedal_GetPercent() so the HMI
 * THROTTLE bar reflects the physical pedal (0 % released … 100 % full) rather
 * than the per-wheel torque/TCS scale (0x205), which is unrelated to pedal
 * position.  This frame is strictly informational: it does NOT feed any
 * control, PID, safety or pedal-logic path on either node.
 *
 *   Byte 0: pedal position %  (0 = released, 100 = full travel)
 *   Byte 1: pedal fault flags  (bit0 plausible, bit1 dual-sample contradiction)
 *   Byte 2-3: raw ADC counts (u16 LE) — lets the HMI show the actual value
 *             behind an out-of-range/implausible pedal fault
 *
 * Bytes 1-3 are additive: legacy receivers that only read byte 0 are
 * unaffected.  This frame remains strictly informational — it does NOT feed
 * any control, PID, safety or pedal-logic path on either node.
 *
 * CAN ID: 0x20B   DLC: 4   Rate: 100 ms (10 Hz)
 */
void CAN_SendStatusPedal(uint8_t pedal_pct) {
    if (pedal_pct > 100) pedal_pct = 100;
    uint8_t data[4];
    data[0] = pedal_pct;

    uint8_t flags = 0U;
    if (Pedal_IsPlausible())     flags |= 0x01U;
    if (Pedal_IsContradictory()) flags |= 0x02U;
    data[1] = flags;

    uint16_t raw = Pedal_GetRawADC();
    data[2] = (uint8_t)(raw & 0xFFU);
    data[3] = (uint8_t)((raw >> 8) & 0xFFU);

    TransmitFrame(CAN_ID_STATUS_PEDAL, data, 4);
}

/**
 * @brief  Send explicit temperature sensor mapping to ESP32.
 *
 * Applies the physIdx→role map stored in flash (via sensor_map_store) so
 * that each byte in the frame corresponds to the labelled motor position:
 *
 *   Byte 0: FL motor temperature  (°C, int8_t)
 *   Byte 1: FR motor temperature  (°C, int8_t)
 *   Byte 2: RL motor temperature  (°C, int8_t)
 *   Byte 3: RR motor temperature  (°C, int8_t)
 *   Byte 4: Ambient temperature   (°C, int8_t)
 *
 * The mapping is set by the user via the engineering menu on the ESP32
 * display (CAN_ID_CMD_SENSOR_MAP_TEMP, 0x112) and persisted in flash page 123.
 * If no valid mapping has been saved, discovery order is used (index 0=FL, etc.)
 *
 * ---- Sanity filter (production hardening) ----
 * DS18B20 readings can briefly become invalid when a sensor is physically
 * disconnected, mid-conversion, or producing a CRC error.  Rather than
 * propagating garbage to the ESP32 HMI, we filter each sample:
 *   - NaN / ±Inf                      → rejected
 *   - ≤ -30 °C or ≥ +120 °C            → rejected (outside practical range;
 *                                        -127 sentinel from OneWire
 *                                        read errors is also covered)
 * Rejected samples are replaced with the last-known-good value for that
 * role (or 0 at cold-boot when nothing has been seen yet), so a brief
 * glitch never shows up as a wild spike on the dashboard.  The filter
 * is strictly informational — it does NOT touch Safety_* or the
 * overheat supervision path, which operate on the raw reading.
 *
 * CAN ID: 0x206   DLC: 5   Rate: 1000 ms (1 Hz)
 */
void CAN_SendStatusTempMap(void) {
    /* Last-known-good °C per role (index = role 0..4).  Cold-start value
     * of 0 is safe — it is the same neutral default the frame already
     * used before this filter was added.                               */
    static int8_t last_good_temp[5] = {0, 0, 0, 0, 0};

    /* Float mirror of last_good_temp[] used ONLY for the TEMP_MAX_STEP_C
     * comparison.  Storing the value in int8 for the CAN payload (which
     * is unchanged) introduces up to ±1 °C of truncation error in the
     * step check; the float mirror removes that bias so the 20 °C
     * threshold is honoured exactly.  The int8 array remains the source
     * of truth for the frame — this mirror is internal only.           */
    static float last_good_temp_f[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    /* Tracks whether `last_good_temp[role]` holds a *validated* sampled
     * value (true) or not (false).  Used by the anti-EMI step-filter
     * (TASK 3) so no sample is rejected against a neutral default, and
     * by the boot-time bootstrap phase below so `last_good_valid` is
     * only raised once two consecutive samples have agreed.            */
    static bool last_good_valid[5] = {false, false, false, false, false};

    /* --------------------------------------------------------------------
     * Boot-time / post-disconnect bootstrap state.
     *
     * Rationale: in a real vehicle the *first* post-boot sample coincides
     * with the worst EMI burst of the whole duty cycle (motors + BTS7960
     * + relays all energising).  A corrupted-but-in-range first sample
     * (e.g. 90 °C when the real reading is 25 °C) would otherwise be
     * accepted unconditionally, arming `last_good_valid`.  Every later
     * real sample would then exceed TEMP_MAX_STEP_C and get rejected
     * forever (stuck until reboot).
     *
     * Fix: require the first *two consecutive* valid samples to agree
     * within ±BOOT_REF_TOLERANCE_C (a single EMI glitch will not pass
     * this gate, because the next undistorted sample lands tens of
     * degrees away and resets the reference) before promoting the role
     * to the steady-state EMI step filter.  While bootstrapping we
     * still emit `current_temp` directly (after the existing sanity
     * tests), so the dashboard sees a live value from the very first
     * valid reading — only the *filter's* memory is withheld until
     * the candidate has been corroborated.  O(1), deterministic, no
     * heap, no blocking.                                                */
    static float   bootstrap_ref[5]   = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static uint8_t bootstrap_count[5] = {0, 0, 0, 0, 0};

    /* Tolerance between the bootstrap reference and the next sample
     * required to accept them as agreeing.  Chosen tighter than
     * TEMP_MAX_STEP_C (20 °C) so that a large, transient EMI glitch
     * cannot slip through the bootstrap as a "plausible step". */
    #define BOOT_REF_TOLERANCE_C  10.0f

    /* Maximum plausible single-frame step (°C).  Temperature physics
     * of the motor phase windings is dominated by thermal mass —
     * even under a short-circuit fault, the reading cannot realistically
     * jump by tens of degrees inside a 1 Hz frame window.  A jump
     * larger than this threshold is therefore attributed to EMI on the
     * 1-Wire line (the DS18B20 CRC catches most of it, but intermittent
     * ground-bounce glitches can produce in-range garbage that
     * nevertheless decodes cleanly).  We reject such samples and keep
     * reporting the last-known-good value for the affected role.      */
    #define TEMP_MAX_STEP_C    20.0f

    /* Sanity thresholds are defined at file scope (TEMP_MIN_VALID_C /
     * TEMP_MAX_VALID_C) so they can be shared with other temperature
     * code paths if needed.                                            */

    /* ----------------------------------------------------------------
     * R-2 — Topology change reset.
     *
     * When the 1-Wire bus changes physically (a DS18B20 reconnects, a
     * new ROM appears, or the enumerated sensor order changes), the
     * armed per-role references above now describe *different* sensors
     * than the ones currently being sampled.  Comparing a fresh reading
     * from a newly-attached motor probe against the previous motor's
     * last_good_temp_f would either (a) wrongly reject a legitimate new
     * sensor via the TEMP_MAX_STEP_C filter, or (b) mask a real fault.
     *
     * `Temperature_HasTopologyChanged()` is a sticky latch raised by
     * `sensor_manager` on any enumeration/ROM-set change; we ack it
     * here so the filter relearns from scratch via the bootstrap
     * phase.  The reset is strictly state-only — no CAN behaviour is
     * altered and the frame is still emitted on this cycle.
     * ---------------------------------------------------------------- */
    if (Temperature_HasTopologyChanged()) {
        memset(last_good_valid,  0, sizeof(last_good_valid));
        memset(bootstrap_count,  0, sizeof(bootstrap_count));
        memset(bootstrap_ref,    0, sizeof(bootstrap_ref));
        memset(last_good_temp_f, 0, sizeof(last_good_temp_f));
        Temperature_ClearTopologyChanged();
    }

    uint8_t data[5] = {0};

    const uint8_t *map = SensorMapStore_GetMap();
    for (uint8_t physIdx = 0; physIdx < 5U; physIdx++) {
        uint8_t role = map[physIdx];
        if (role >= 5U) {
            /* Unset / out-of-range role — shouldn't happen because the
             * RX handler enforces strict mode, but stay defensive.     */
            continue;
        }

        float raw = Temperature_Get(physIdx);

        /* ----------------------------------------------------------------
         * TASK 4 — explicit DS18B20 disconnect sentinel check.
         *
         * A disconnected DS18B20 (pull-up floating high / parasite lost)
         * typically reads as all-ones on the scratchpad, which decodes
         * to exactly -127 °C (0xFC90 / 16 = -127.9375, rounded).  We
         * test this explicitly BEFORE the generic range check so that
         * the log/telemetry layer can later distinguish "disconnected"
         * from "out-of-range glitch" without changing the range constants.
         *
         * The comparison is one-sided (`raw <= -126`): any reading at or
         * below -126 °C is definitionally outside the DS18B20 operating
         * range (-55 °C per datasheet), so it is always a disconnect /
         * bus fault.  This covers both the -127.0 and the -127.9375
         * variants.  The subsequent range test would also catch these,
         * but the named check makes intent obvious.
         * -------------------------------------------------------------- */
        #define DS18B20_DISCONNECT_SENTINEL   (-127.0f)
        bool disconnected = (raw <= (DS18B20_DISCONNECT_SENTINEL + 1.0f));

        /* Accept sample only if it is a finite number inside the window
         * AND not the disconnect sentinel.  isnan / isinf are standard
         * <math.h> checks.                                              */
        bool sample_ok = !disconnected
                      && !isnan(raw) && !isinf(raw)
                      && (raw >= TEMP_MIN_VALID_C)
                      && (raw <= TEMP_MAX_VALID_C);

        /* Snapshot the "hard invalid" verdict (NaN / Inf / out-of-range
         * / -127 °C sentinel) before the step filter can soft-reject a
         * physically plausible but EMI-corrupted sample below.  Only a
         * hard-invalid sample is allowed to tear down the steady-state
         * reference and force a new bootstrap — a single step-filter
         * rejection must NOT demote an armed role, otherwise one EMI
         * spike would cost two extra cycles of unfiltered output.     */
        bool hard_invalid = !sample_ok;

        /* ----------------------------------------------------------------
         * TASK 3 — anti-EMI plausibility filter (steady-state phase).
         *
         * Even in-range samples can be corrupted by electromagnetic
         * interference on the 1-Wire line.  Compare against the last
         * accepted value for this ROLE and reject samples that jump
         * by more than TEMP_MAX_STEP_C between two 1 Hz frames.
         *
         * Applied only when the role has completed the boot-time
         * bootstrap (`last_good_valid`) — during bootstrap the step
         * filter is intentionally bypassed so a corrupted first sample
         * cannot latch the filter memory and lock out every subsequent
         * real reading.  Comparison uses the float mirror of the last
         * good value to avoid the ±1 °C truncation error introduced by
         * the int8 CAN representation.
         *
         * The filter runs *after* the basic sanity tests so that a
         * sensor coming back online with a large but plausible new
         * reading is not mistaken for an EMI glitch; a definitively
         * bad reading (NaN, out-of-range, disconnect) is already
         * rejected by `sample_ok` above.
         * -------------------------------------------------------------- */
        if (sample_ok && last_good_valid[role]) {
            if (fabsf(raw - last_good_temp_f[role]) > TEMP_MAX_STEP_C) {
                sample_ok = false;
            }
        }

        int8_t reported;
        if (hard_invalid) {
            /* Hard-invalid sample (NaN / Inf / out-of-range / -127 °C
             * sentinel). Reset bootstrap and drop any armed steady-
             * state reference so the next time the sensor returns
             * coherent data it has to re-prove itself via the 2-sample
             * bootstrap. This is the only path that resets per spec;
             * a mere step-filter rejection (below) keeps the armed
             * reference intact.                                        */
            last_good_valid[role] = false;
            bootstrap_count[role] = 0;

            /* Fall back to the last valid reading for this ROLE (not
             * physIdx): if the user re-maps sensors the last-good value
             * rightly follows the role, so a freshly-assigned sensor
             * inherits the previous motor's last reading for one
             * frame rather than flashing 0 °C.                         */
            reported = last_good_temp[role];
        } else if (!sample_ok) {
            /* Soft rejection: sample was in range but failed the EMI
             * step filter. Keep the armed reference, just report the
             * last-known-good value for this frame.                   */
            reported = last_good_temp[role];
        } else if (last_good_valid[role]) {
            /* Steady-state: sample passed EMI step filter. Accept it
             * as the new reference for both the CAN payload (int8)
             * and the filter comparator (float).                       */
            reported              = (int8_t)raw;
            last_good_temp[role]  = reported;
            last_good_temp_f[role] = raw;
        } else {
            /* ------------------------------------------------------------
             * Bootstrap phase.
             *
             * The role has no validated reference yet (cold boot, or
             * a previous reset after an invalid sample).  Emit the
             * current raw reading directly so the dashboard is not
             * blanked — but only promote the role to the steady-state
             * filter once two consecutive samples agree within
             * BOOT_REF_TOLERANCE_C.  A single EMI glitch therefore
             * cannot arm the filter with a bogus reference.
             * --------------------------------------------------------- */

            /* R-1 — boot-time plausibility window.
             *
             * The 2-sample consensus filter below protects against a
             * single transient glitch, but a sustained EMI pattern
             * (two back-to-back readings both biased the same way,
             * e.g. 90 °C while the true value is 25 °C) could still
             * sneak past the 10 °C agreement tolerance.  Clamp the
             * physically plausible boot range to 0..60 °C: anything
             * outside this window at cold-start is treated as noise,
             * the bootstrap counter is reset, and the sample is NOT
             * emitted this cycle (data[role] stays at its zeroed
             * default, identical to a cold-boot frame).  This has
             * no effect once the role is armed — steady-state samples
             * still use the broader TEMP_MIN_VALID_C/TEMP_MAX_VALID_C
             * window and the TEMP_MAX_STEP_C step filter.            */
            if (raw < BOOT_MIN_VALID_C || raw > BOOT_MAX_VALID_C) {
                bootstrap_count[role] = 0U;
                continue;
            }

            if (bootstrap_count[role] == 0U) {
                /* First valid sample — candidate reference. */
                bootstrap_ref[role]   = raw;
                bootstrap_count[role] = 1U;
            } else if (fabsf(raw - bootstrap_ref[role]) <= BOOT_REF_TOLERANCE_C) {
                /* Corroborating sample — candidate confirmed. */
                bootstrap_count[role]++;
            } else {
                /* Disagreement — previous reference was likely a glitch.
                 * Start over with the new sample as the candidate. */
                bootstrap_ref[role]   = raw;
                bootstrap_count[role] = 1U;
            }

            reported = (int8_t)raw;

            if (bootstrap_count[role] >= 2U) {
                /* Two consecutive agreeing samples — arm the steady-
                 * state filter.  From the next cycle onwards the
                 * TEMP_MAX_STEP_C check protects the role.          */
                last_good_temp[role]   = reported;
                last_good_temp_f[role] = raw;
                last_good_valid[role]  = true;
                /* bootstrap_count stays; it will be reset on the next
                 * invalid sample.  Leaving it non-zero is harmless —
                 * the bootstrap branch is gated by last_good_valid. */
            }
        }
        data[role] = (uint8_t)reported;
    }

    TransmitFrame(CAN_ID_STATUS_TEMP_MAP, data, 5);
}

/**
 * @brief  Send battery bus current and voltage to ESP32.
 *
 * Reads the INA226 on channel INA226_CHANNEL_BATTERY (100A shunt on
 * 24V bus) and transmits current and voltage so the ESP32 HMI can
 * display battery current in the upper-right corner of the screen.
 *
 *   Byte 0-1: Battery current (0.01 A units, uint16 little-endian)
 *   Byte 2-3: Battery voltage (0.01 V units, uint16 little-endian)
 *
 * CAN ID: 0x207   DLC: 4   Rate: 100 ms (10 Hz)
 */
void CAN_SendStatusBattery(void) {
    uint8_t data[4];

    float amps = sanitize_float(Current_GetAmps(INA226_CHANNEL_BATTERY), 0.0f);
    float volts = sanitize_float(Voltage_GetBus(INA226_CHANNEL_BATTERY), 0.0f);
    if (amps < 0.0f) amps = 0.0f;
    if (volts < 0.0f) volts = 0.0f;
    uint16_t amps_raw = float_to_u16_clamped(amps * 100.0f);
    uint16_t volts_raw = float_to_u16_clamped(volts * 100.0f);

    data[0] = (uint8_t)(amps_raw & 0xFF);
    data[1] = (uint8_t)((amps_raw >> 8) & 0xFF);
    data[2] = (uint8_t)(volts_raw & 0xFF);
    data[3] = (uint8_t)((volts_raw >> 8) & 0xFF);

    TransmitFrame(CAN_ID_STATUS_BATTERY, data, 4);
}

void CAN_SendError(uint8_t error_code, uint8_t subsystem) {
    uint8_t error_data[2];
    
    error_data[0] = error_code;
    error_data[1] = subsystem;
    
    TransmitFrame(CAN_ID_DIAG_ERROR, error_data, 2);
}

/**
 * @brief  Send raw encoder diagnostic data over CAN.
 *
 * Uses CAN_ID_DIAG_ERROR (0x300) with a dedicated subsystem tag
 * (0x10) to distinguish from regular error reports.  Diagnostic
 * only — not part of any control path.
 *
 *   Byte 0:    0x10 (encoder diagnostic tag)
 *   Byte 1:    reserved (0)
 *   Byte 2-5:  raw_count (int32_t, little-endian)
 *   Byte 6-7:  delta     (int16_t, little-endian)
 */
void CAN_SendDiagnosticEncoder(int32_t raw_count, int16_t delta) {
    uint8_t data[8];
    data[0] = 0x10;  /* Encoder diagnostic subsystem tag */
    data[1] = 0;
    data[2] = (uint8_t)( raw_count        & 0xFF);
    data[3] = (uint8_t)((raw_count >>  8) & 0xFF);
    data[4] = (uint8_t)((raw_count >> 16) & 0xFF);
    data[5] = (uint8_t)((raw_count >> 24) & 0xFF);
    data[6] = (uint8_t)( delta       & 0xFF);
    data[7] = (uint8_t)((delta >> 8) & 0xFF);

    TransmitFrame(CAN_ID_DIAG_ERROR, data, 8);
}

/**
 * @brief  Send command acknowledgment to ESP32.
 *
 * Transmits a 3-byte ACK frame after the STM32 has validated
 * and accepted or rejected an ESP32 command.
 *
 *   Byte 0: cmd_id_low — low byte of the original command CAN ID
 *                         (e.g. 0x02 for CMD_MODE 0x102)
 *   Byte 1: result     — CAN_AckResult_t (0=OK, 1=REJECTED, 2=INVALID,
 *                         3=BLOCKED_BY_SAFETY)
 *   Byte 2: system_state — current SystemState_t for context
 *
 * CAN ID: 0x103   DLC: 3   Rate: on-demand (after each command)
 */
void CAN_SendCommandAck(uint8_t cmd_id_low, CAN_AckResult_t result) {
    uint8_t ack_data[3];

    ack_data[0] = cmd_id_low;
    ack_data[1] = (uint8_t)result;
    ack_data[2] = (uint8_t)Safety_GetState();

    TransmitFrame(CAN_ID_CMD_ACK, ack_data, 3);
}

/**
 * @brief  Send service mode status to ESP32.
 *
 * Transmits three frames:
 *   0x301 — fault bitmask   (4 bytes, little-endian uint32)
 *   0x302 — enabled bitmask (4 bytes, little-endian uint32)
 *   0x303 — disabled bitmask(4 bytes, little-endian uint32)
 *
 * ESP32 uses these to populate the service/diagnostic menu.
 */
void CAN_SendServiceStatus(void) {
    uint8_t data[4];
    uint32_t val;

    /* Fault bitmask */
    val = ServiceMode_GetFaultMask();
    data[0] = (uint8_t)(val & 0xFF);
    data[1] = (uint8_t)((val >> 8) & 0xFF);
    data[2] = (uint8_t)((val >> 16) & 0xFF);
    data[3] = (uint8_t)((val >> 24) & 0xFF);
    TransmitFrame(CAN_ID_SERVICE_FAULTS, data, 4);

    /* Enabled bitmask */
    val = ServiceMode_GetEnabledMask();
    data[0] = (uint8_t)(val & 0xFF);
    data[1] = (uint8_t)((val >> 8) & 0xFF);
    data[2] = (uint8_t)((val >> 16) & 0xFF);
    data[3] = (uint8_t)((val >> 24) & 0xFF);
    TransmitFrame(CAN_ID_SERVICE_ENABLED, data, 4);

    /* Disabled bitmask */
    val = ServiceMode_GetDisabledMask();
    data[0] = (uint8_t)(val & 0xFF);
    data[1] = (uint8_t)((val >> 8) & 0xFF);
    data[2] = (uint8_t)((val >> 16) & 0xFF);
    data[3] = (uint8_t)((val >> 24) & 0xFF);
    TransmitFrame(CAN_ID_SERVICE_DISABLED, data, 4);
}

/**
 * @brief  Send DWT-debounce EMI diagnostic counters to ESP32.
 *
 * Three additive frames (1000 ms cadence, low priority):
 *   0x306 — DLC 8: FL/FR/RL/RR REJECTED (filtered) counts, uint16 LE sat 0xFFFF
 *   0x307 — DLC 4: steering-center REJECTED (filtered) count, uint32 LE
 *   0x314 — DLC 8: FL/FR/RL/RR VALID (accepted) pulse counts, uint16 LE sat 0xFFFF
 *
 * The internal counters in sensor_manager.c are 32-bit saturated; these frames
 * truncate the 4 wheel counters to 16 bits for compactness.  Truncation is
 * acceptable because the values are diagnostic only and "saturated" is a
 * meaningful UI state.
 *
 * REJECTED (0x306) are edges dropped by the DWT 200 us pre-filter (bounce/EMI)
 * and do NOT correspond to distance/speed.  VALID (0x314) are the accepted
 * edges (Wheel_GetPulseCount) that DO map to real wheel rotation, so the HMI
 * can display "VALID PULSES | REJECTED PULSES" side by side and confirm the
 * wheels count equally (~6 valid pulses per revolution).
 *
 * Diagnostic only — no control / safety path consumes these.
 */
void CAN_SendDebounceDiag(void) {
    uint8_t  data8[8];
    uint8_t  data4[4];
    uint32_t v;

    /* 0x306 — wheel REJECTED (filtered) counts (FL, FR, RL, RR) */
    for (uint8_t i = 0; i < 4; i++) {
        v = Sensor_GetFilteredCount(i);
        uint16_t v16 = (v > 0xFFFFU) ? 0xFFFFU : (uint16_t)v;
        data8[2 * i]     = (uint8_t)(v16 & 0xFF);
        data8[2 * i + 1] = (uint8_t)((v16 >> 8) & 0xFF);
    }
    TransmitFrame(CAN_ID_DIAG_DEBOUNCE, data8, 8);

    /* 0x307 — steering filtered count (uint32 LE, no truncation) */
    v = Sensor_GetSteerFilteredCount();
    data4[0] = (uint8_t)(v & 0xFF);
    data4[1] = (uint8_t)((v >> 8) & 0xFF);
    data4[2] = (uint8_t)((v >> 16) & 0xFF);
    data4[3] = (uint8_t)((v >> 24) & 0xFF);
    TransmitFrame(CAN_ID_DIAG_DEBOUNCE_STEER, data4, 4);

    /* 0x314 — wheel VALID (accepted) pulse counts (FL, FR, RL, RR).
     * These are the real edges that passed the DWT pre-filter and drive the
     * speed/distance estimate, so the operator can compare per-wheel counts
     * (a full wheel turn ≈ 6 valid pulses) independently of the REJECTED
     * (bounce/EMI) counts above.  Diagnostic only.                         */
    for (uint8_t i = 0; i < 4; i++) {
        v = Wheel_GetPulseCount(i);
        uint16_t v16 = (v > 0xFFFFU) ? 0xFFFFU : (uint16_t)v;
        data8[2 * i]     = (uint8_t)(v16 & 0xFF);
        data8[2 * i + 1] = (uint8_t)((v16 >> 8) & 0xFF);
    }
    TransmitFrame(CAN_ID_DIAG_WHEEL_PULSES, data8, 8);
}

/**
 * @brief  Send I2C topology diagnostic to ESP32 (HMI Safe Mode helper).
 *
 * Lets the operator tell apart a missing TCA9548A multiplexer (0x70) from a
 * missing / dead INA226 (0x40) on a specific channel, and see whether the
 * failures are intermittent (bad pull-up / loose terminal) or permanent
 * (not connected).  Diagnostic only — no control / safety path consumes it.
 *
 * CAN ID: 0x309   DLC: 8   Rate: 1000 ms (1 Hz)
 *   Byte 0: mux_present (0 = TCA9548A 0x70 not acking, 1 = present)
 *   Byte 1: ina_ok_mask (bit i = INA226 acked behind mux channel i;
 *           bit0=FL, bit1=FR, bit2=RL, bit3=RR, bit4=BAT, bit5=STEER)
 *   Byte 2: i2c_fail_count        (failed transactions in the last cycle)
 *   Byte 3: i2c_recovery_attempts (sticky bus-recovery attempt counter)
 *   Byte 4: flags (bit0 = "ever OK" — at least one INA seen healthy since boot)
 *   Byte 5: ina_expected_mask (bit i = channel i's power branch is energised,
 *           so its INA226 SHOULD answer this phase; same bit order as byte1).
 *           Lets the HMI show an unpowered branch as "WAIT PWR" instead of a
 *           red FAIL.  Additive — pre-extension consumers (DLC 5) ignore it.
 *   Byte 6: i2c_last_read_ms (duration of the last Current_ReadAll() in ms,
 *           saturated at 255 → HMI shows "255+ms").  Additive — DLC-6/DLC-5
 *           consumers ignore it.
 *   Byte 7: reserved (0)
 */
void CAN_SendI2CDiag(void) {
    uint8_t data[8];
    data[0] = Sensor_GetMuxPresent() ? 1U : 0U;
    data[1] = Sensor_GetInaOkMask();
    data[2] = Sensor_GetI2cFailCount();
    data[3] = Sensor_GetI2cRecoveryAttempts();
    data[4] = Sensor_GetI2cEverOk() ? 0x01U : 0x00U;
    data[5] = Sensor_GetInaExpectedMask();
    /* Byte 6: duration of the last Current_ReadAll() in ms (saturated at 255).
     * Lets the operator see I2C blocking time directly on the HMI.          */
    data[6] = Sensor_GetI2cLastReadMs();
    data[7] = 0U;   /* reserved */

    /* Observability A/C: count every invocation and the TX outcome so the
     * 0x30A meta-frame can prove whether 0x309 reaches the FDCAN TX FIFO. */
    sat_inc_u32(&can_txmeta.diag309_call_count);
    if (TransmitFrame(CAN_ID_DIAG_I2C, data, 8) == HAL_OK) {
        sat_inc_u32(&can_txmeta.diag309_tx_ok);
    } else {
        sat_inc_u32(&can_txmeta.diag309_tx_err);
    }
}

/**
 * @brief  CAN/0x309 delivery meta-diagnostic (additive, report-only).
 *
 * Surfaces the counters from CAN_TxMeta_t so an operator can tell, from the
 * ESP32 alone, whether the 0x309 diagnostic frame is being generated, queued
 * and accepted by the FDCAN TX FIFO — independent of the I2C bus state.
 * Diagnostic only — no control / safety path consumes it.
 *
 * CAN ID: 0x30A   DLC: 8   Rate: 1000 ms (1 Hz)
 *   Byte 0-1: diag309_call_count  (uint16 LE, saturated)  [A]
 *   Byte 2-3: tick_1000ms_count   (uint16 LE, saturated)  [B]
 *   Byte 4:   diag309_tx_ok       (uint8, saturated)      [C]
 *   Byte 5:   diag309_tx_err      (uint8, saturated)      [C]
 *   Byte 6:   tx_fifo_full_drops  (uint8, saturated)      [D]
 *   Byte 7:   bit0 = fdcan_init_ok; bits 7:1 = hb_tx_err (sat. at 127) [E/F]
 */
void CAN_SendCanMetaDiag(void) {
    extern bool fdcan_init_ok;
    uint8_t data[8];
    uint16_t calls = (can_txmeta.diag309_call_count > 0xFFFFU)
                   ? 0xFFFFU : (uint16_t)can_txmeta.diag309_call_count;
    uint16_t ticks = (can_txmeta.tick_1000ms_count > 0xFFFFU)
                   ? 0xFFFFU : (uint16_t)can_txmeta.tick_1000ms_count;
    data[0] = (uint8_t)(calls & 0xFF);
    data[1] = (uint8_t)((calls >> 8) & 0xFF);
    data[2] = (uint8_t)(ticks & 0xFF);
    data[3] = (uint8_t)((ticks >> 8) & 0xFF);
    data[4] = (can_txmeta.diag309_tx_ok      > 0xFFU) ? 0xFFU : (uint8_t)can_txmeta.diag309_tx_ok;
    data[5] = (can_txmeta.diag309_tx_err     > 0xFFU) ? 0xFFU : (uint8_t)can_txmeta.diag309_tx_err;
    data[6] = (can_txmeta.tx_fifo_full_drops > 0xFFU) ? 0xFFU : (uint8_t)can_txmeta.tx_fifo_full_drops;
    /* bit 0: fdcan_init_ok; bits 7:1: hb_tx_err saturated at 127 */
    {
        uint8_t hb_err_sat = (can_txmeta.hb_tx_err > 0x7FU) ? 0x7FU
                           : (uint8_t)can_txmeta.hb_tx_err;
        data[7] = (fdcan_init_ok ? 0x01U : 0x00U) | (uint8_t)(hb_err_sat << 1);
    }
    TransmitFrame(CAN_ID_DIAG_CAN_META, data, 8);
}

/**
 * @brief  Boot/reset diagnostic (additive, report-only).
 *
 * Surfaces the STM32 uptime and the RCC reset-cause bitmask captured at
 * boot so the HMI can confirm whether recent 8–10 s gaps were caused by
 * IWDG timeouts, brownout events, or software resets — independently of
 * the main safety telemetry.  Diagnostic only; no control path consumes it.
 *
 * CAN ID: 0x312   DLC: 8   Rate: 1000 ms (1 Hz)
 *   Byte 0-3: HAL_GetTick() uptime in ms (uint32 LE, wraps at ~49.7 days)
 *   Byte 4:   reset_cause bitmask — mirrors RESET_CAUSE_* in main.h
 *               bit0 = POWERON   (only PINRSTF set → normal power-up)
 *               bit1 = SOFTWARE  (SFTRSTF)
 *               bit2 = IWDG      (IWDGRSTF)
 *               bit3 = WWDG      (WWDGRSTF)
 *               bit4 = BROWNOUT  (BORRSTF or LPWRRSTF)
 *               bit5 = PIN       (PINRSTF, combined with another flag)
 *   Byte 5:   tx_queue_depth      — software TX ring occupancy now (0..31)
 *   Byte 6:   tx_queue_depth_max  — software TX ring high-water mark (0..31)
 *   Byte 7:   reserved (0)
 */
void CAN_SendBootResetDiag(void) {
    uint32_t uptime = HAL_GetTick();
    uint8_t data[8] = {0};
    data[0] = (uint8_t)( uptime        & 0xFFU);
    data[1] = (uint8_t)((uptime >>  8) & 0xFFU);
    data[2] = (uint8_t)((uptime >> 16) & 0xFFU);
    data[3] = (uint8_t)((uptime >> 24) & 0xFFU);
    data[4] = Boot_GetResetCause();
    /* bytes 5-6: software TX queue depth (now / high-water) — lets the HMI
     * prove the heartbeat (enqueued first each 100 ms cycle) is never sitting
     * behind a TX backlog.  Additive/report-only; byte 7 stays reserved.    */
    data[5] = can_txmeta.tx_queue_depth;
    data[6] = can_txmeta.tx_queue_depth_max;
    /* byte 7 reserved */
    TransmitFrame(CAN_ID_DIAG_BOOT_RESET, data, 8);
}

/**
 * @brief  Per-wheel speed-sensor fault-reason diagnostic (additive, report-only).
 *
 * Surfaces, for every wheel channel, the WheelDiag_t reason code that
 * Safety_CheckSensors() computed, together with the raw GPIO level, the
 * latched service-mode fault mask and a small flags nibble.  Lets the HMI
 * tell the operator exactly WHICH wheel is silent and WHY, so a 0x10
 * SENSOR_FAULT is no longer an unlabelled "guess with the flags" event and
 * an expected hand-spin (MANUAL_MOVEMENT) is visibly distinct from a genuine
 * fault under traction (NO_PULSE / STUCK_* / MISMATCH).
 *
 * CAN ID: 0x313   DLC: 8   Rate: 1000 ms (1 Hz)
 *   Byte 0-3: reason FL/FR/RL/RR (WheelDiag_t wire code, 0-8)
 *   Byte 4:   reason STEER/CENTER (reserved — no diag channel yet, always 0)
 *   Byte 5:   gpio_level_mask (bit0 FL..bit3 RR, bit4 STEER=0)
 *   Byte 6:   active_fault_mask (bit0 FL..bit3 RR, bit4 STEER=0)
 *   Byte 7:   bit0 powertrain_engaged, bit1 manual_movement_detected,
 *             bit2 wheel_fault_debouncing, bit3 wheel_fault_latched,
 *             bits4-7 sequence counter (wraps 0-15)
 *
 * Diagnostic only — no control/safety path consumes this frame.  It reads the
 * already-computed safety diagnostics and never mutates safety state.
 */
void CAN_SendWheelSensorDiag(void) {
    static uint8_t seq = 0U;
    uint8_t data[8] = {0};

    uint8_t gpio_mask   = 0U;
    uint8_t manual_seen = 0U;
    uint8_t debouncing  = 0U;
    for (uint8_t i = 0; i < NUM_WHEELS && i < 4U; i++) {
        WheelDiag_t d_live = Safety_GetWheelDiag(i);
        /* Report the latched culprit reason so a channel that aborted stays
         * identifiable even after its live reason self-heals to OK.        */
        data[i] = Safety_WheelDiagToCanReason(Safety_GetWheelLatchedReason(i));
        if (Wheel_GetGpioLevel(i) == 1U) {
            gpio_mask |= (uint8_t)(1U << i);
        }
        if (d_live == WHEEL_DIAG_MANUAL_MOVEMENT) manual_seen = 1U;
        /* A reason of MISMATCH means "under load, silent, debounce armed but
         * not yet latched" — i.e. the fault is currently debouncing.        */
        if (d_live == WHEEL_DIAG_MISMATCH)        debouncing = 1U;
    }
    /* Byte 4 (STEER/CENTER): reflects the steering-encoder service module so a
     * disabled/unwired encoder (E6B2-CWZ6C) renders as DESHAB. instead of a
     * misleading OK.  Genuine encoder faults surface via FAULT_ENCODER_ERROR. */
    data[4] = Safety_GetSteerDiagReason();

    data[5] = gpio_mask;                       /* bit4 STEER stays 0 */
    data[6] = Safety_GetWheelFaultMask();      /* bit4 STEER stays 0 */

    uint8_t flags = 0U;
    if (Safety_IsPowertrainEngaged())                       flags |= (1U << 0);
    if (manual_seen)                                        flags |= (1U << 1);
    if (debouncing)                                         flags |= (1U << 2);
    if (Safety_GetFaultFlags() & FAULT_WHEEL_SENSOR)        flags |= (1U << 3);
    flags |= (uint8_t)((seq & 0x0FU) << 4);
    seq++;
    data[7] = flags;

    TransmitFrame(CAN_ID_DIAG_WHEEL_SENSOR, data, 8);
}

/**
 * @brief  Steering homing (centering) telemetry — 0x316 (1 Hz).
 *
 * Serialises the classified SteeringCenteringDiag snapshot maintained by
 * steering_centering.c so the ESP32 can render the real cause of a stuck
 * homing sweep ("DIRECCIÓN NO SE MUEVE") instead of a generic "Error 8".
 * Instrumentation only — reads the cached snapshot, drives nothing.
 */
void CAN_SendSteeringCenteringDiag(void) {
    const SteeringCenteringDiag *d = SteeringCentering_GetDiag();
    uint8_t data[8] = {0};

    /* Single source of truth for the 0x316 wire layout — shared with the
     * ESP32 receiver and the host round-trip test. */
    SteerCentering_PackFrame(d, data);

    TransmitFrame(CAN_ID_DIAG_STEERING_CENTERING, data, 8);
}

/**
 * @brief  Relay / current-sense health telemetry — 0x317 (1 Hz).
 *
 * Serialises the evidence-graded RelayHealthDiag snapshot classified by
 * safety_system.c so the ESP32 can show the REAL cause (CURRENT SENSE INVALID
 * vs RELAY OPEN SUSPECTED) and the numbers behind it, instead of a bare
 * "RELAY OPEN".  Instrumentation only — reads the cached snapshot.
 */
void CAN_SendRelayHealthDiag(void) {
    const RelayHealthDiag *d = Safety_GetRelayHealthDiag();
    uint8_t data[8] = {0};

    /* Single source of truth for the 0x317 wire layout — shared with the
     * ESP32 receiver and the host round-trip test. */
    RelayHealth_PackFrame(d, data);

    TransmitFrame(CAN_ID_DIAG_RELAY_HEALTH, data, 8);
}

/**
 * @brief  Steering-INA (CH5) channel diagnostic frame (additive, report-only).
 *
 * Serialises the classified Ina226ChannelDiag snapshot maintained by
 * sensor_manager.c (Sensor_GetChannel5Diag) so the ESP32 can distinguish the
 * real steering-INA state (MISSING / WRONG ADDRESS / CONFIG FAIL / PRESENT NO
 * SHUNT / POLARITY REVERSED / STALE / OK) and show the signed shunt/current
 * behind it — never flattening a reversed (negative) current to zero.  The
 * presence of this frame is what lets the HMI separate "n/d" (no CAN contract)
 * from a genuine MISSING (chip did not ACK).  Instrumentation only.
 */
void CAN_SendIna226Ch5Diag(void) {
    const Ina226ChannelDiag *d = Sensor_GetChannel5Diag();
    uint8_t data[8] = {0};

    /* Single source of truth for the 0x318 wire layout — shared with the
     * ESP32 receiver and the host round-trip test. */
    Ina226Ch5_PackFrame(d, data);

    TransmitFrame(CAN_ID_DIAG_INA_CH5, data, 8);
}

/**
 * @brief  I2C service-mode scan report (additive, on-demand, report-only).
 *
 * Triggered by SERVICE_CMD 0x110 byte0 = SERVICE_ACTION_I2C_SERVICE (0xF6).
 * Runs an active I2C probe (Sensor_RunI2CServiceScan) and reports the result
 * so the operator can localise an I2C-side fault (wrong address, stuck SDA/
 * SCL, dead mux/INA) — covers diagnostic questions G/H/I/J.  Diagnostic only.
 *
 * CAN ID: 0x30B   DLC: 8
 *   Byte 0: bus flags — bit0 SCL idle high, bit1 SDA idle high,
 *           bit2 recovery attempted, bit3 recovery succeeded
 *   Byte 1: mux_present (TCA9548A 0x70 acked)
 *   Byte 2: ina_present_mask (bit0..5 = INA226 0x40 acked behind ch0..5)
 *   Byte 3: i2c_fail_count
 *   Byte 4: i2c_recovery_attempts
 *   Byte 5: terminal scan phase (I2C_SCAN_PHASE_*) — additive, derived from
 *           the fields above so the HMI can name the fault (bus busy / TCA
 *           missing / TCA ack) instead of a bare timeout
 *   Byte 6-7: reserved (0)
 */
void CAN_SendI2CScanReport(void) {
    Sensor_I2cScanResult_t r = Sensor_RunI2CServiceScan();
    uint8_t data[8] = {0};
    data[0] = (uint8_t)((r.scl_idle_high ? 0x01U : 0U)
                      | (r.sda_idle_high ? 0x02U : 0U)
                      | (r.recovery_attempted ? 0x04U : 0U)
                      | (r.recovery_success ? 0x08U : 0U));
    data[1] = r.mux_present ? 1U : 0U;
    data[2] = r.ina_present_mask;
    data[3] = r.fail_count;
    data[4] = r.recovery_attempts;
    /* Byte 5: compact terminal phase.  SDA still low after the recovery
     * attempt means the bus is physically stuck; otherwise the mux ACK
     * (or lack of it) is the decisive condition. */
    if (!r.sda_idle_high) {
        data[5] = I2C_SCAN_PHASE_BUS_BUSY;
    } else if (!r.mux_present) {
        data[5] = I2C_SCAN_PHASE_TCA_MISSING;
    } else {
        data[5] = I2C_SCAN_PHASE_TCA_ACK;
    }
    TransmitFrame(CAN_ID_DIAG_I2C_SCAN, data, 8);
}

/**
 * @brief  FDCAN error-counter dump (additive, on-demand, report-only).
 *
 * Mirrors the can_diag block (TEC/REC/LEC/state) so the operator can confirm
 * the physical CAN bus health from the ESP32 side.  Diagnostic only.
 *
 * CAN ID: 0x30C   DLC: 6
 *   Byte 0: last_error_code (PSR.LEC)
 *   Byte 1: state flags — bit0 error_passive, bit1 bus_off, bit2 warning
 *   Byte 2: tec   Byte 3: rec
 *   Byte 4: tx_nack_flag   Byte 5: tx_consec_fail
 */
void CAN_SendFdcanDiag(void) {
    uint8_t data[6];
    data[0] = can_diag.last_error_code;
    data[1] = (uint8_t)((can_diag.error_passive ? 0x01U : 0U)
                      | (can_diag.bus_off ? 0x02U : 0U)
                      | (can_diag.warning ? 0x04U : 0U));
    data[2] = can_diag.tec;
    data[3] = can_diag.rec;
    data[4] = can_diag.tx_nack_flag;
    data[5] = can_diag.tx_consec_fail;
    TransmitFrame(CAN_ID_DIAG_FDCAN, data, 6);
}

/**
 * @brief  Transmit error log header (entry count + total events).
 *         Sent periodically (1000 ms) so the ESP32 engineering menu
 *         knows how many log entries are available.
 *
 * CAN ID 0x305, DLC 8:
 *   Byte 0-1: entry_count (uint16 LE)
 *   Byte 2-5: total_events (uint32 LE, lifetime counter)
 *   Byte 6-7: reserved (0)
 */
void CAN_SendErrorLogHeader(void) {
    uint8_t data[8] = {0};
    uint16_t count = ErrorLog_GetCount();
    uint32_t total = ErrorLog_GetTotalEvents();
    data[0] = (uint8_t)(count & 0xFF);
    data[1] = (uint8_t)((count >> 8) & 0xFF);
    data[2] = (uint8_t)(total & 0xFF);
    data[3] = (uint8_t)((total >> 8) & 0xFF);
    data[4] = (uint8_t)((total >> 16) & 0xFF);
    data[5] = (uint8_t)((total >> 24) & 0xFF);
    data[6] = 0;
    data[7] = 0;
    TransmitFrame(CAN_ID_ERROR_LOG_HEADER, data, 8);
}

/**
 * @brief  Set front LED power relay state (PB10).
 * @param  on  true = relay ON (front LEDs powered), false = relay OFF
 */
void LED_Relay_Set(bool on) {
    led_relay_front = on;
    HAL_GPIO_WritePin(GPIOB, PIN_RELAY_LED,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Set rear LED power relay state (PB11).
 * @param  on  true = relay ON (rear LEDs powered), false = relay OFF
 */
void LED_Relay_Rear_Set(bool on) {
    led_relay_rear = on;
    HAL_GPIO_WritePin(GPIOB, PIN_RELAY_LED_REAR,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool LED_Relay_Get(void) {
    return led_relay_front;
}

bool LED_Relay_Rear_Get(void) {
    return led_relay_rear;
}

/**
 * @brief  Send LED/light status to ESP32.
 *
 *   Byte 0: led_relay_front (0 = OFF, 1 = ON)
 *   Byte 1: led_relay_rear  (0 = OFF, 1 = ON)
 *
 * CAN ID: 0x20A   DLC: 2   Rate: 1000 ms (1 Hz)
 */
void CAN_SendStatusLights(void) {
    uint8_t data[2];

    data[0] = led_relay_front ? 1 : 0;
    data[1] = led_relay_rear  ? 1 : 0;

    TransmitFrame(CAN_ID_STATUS_LIGHTS, data, 2);
}

/* Helper to extract byte count from FDCAN DLC */
static uint8_t ExtractDLC(uint32_t dlc_code) {
    switch (dlc_code) {
        case FDCAN_DLC_BYTES_0:  return 0;
        case FDCAN_DLC_BYTES_1:  return 1;
        case FDCAN_DLC_BYTES_2:  return 2;
        case FDCAN_DLC_BYTES_3:  return 3;
        case FDCAN_DLC_BYTES_4:  return 4;
        case FDCAN_DLC_BYTES_5:  return 5;
        case FDCAN_DLC_BYTES_6:  return 6;
        case FDCAN_DLC_BYTES_7:  return 7;
        case FDCAN_DLC_BYTES_8:  return 8;
        default: return 0;
    }
}

/* ==================================================================
 *  Pedal calibration (0xF5 sub-protocol + 0x308 / 0x319 telemetry)
 *
 *  The productive calibration is a single guided PedalCalSession FSM
 *  (pedal_cal_session.c) — see the block comment just below.  This CAN
 *  module owns exactly ONE session instance, populates its per-tick
 *  condition snapshot from the live safety / sensor state, drives it,
 *  enforces safe outputs while it runs, and transports its state on CAN.
 *  The underlying pedal pipeline (sensor_manager) is touched only via the
 *  public Pedal_ApplyCalibration() / Pedal_GetRawADC() helpers and the
 *  flash store (pedal_cal_store.c).
 * ================================================================== */

/* ---- 0x308 telemetry burst cadence ---- */
#define PEDALCAL_BURST_FRAMES     10U    /* 10 × 100 ms = 1 s             */
#define PEDALCAL_BURST_PERIOD_MS  100U

/* ---- Pedal-percent thresholds for the guided session (audit P5) ----
 * released : pedal percent strictly below this counts as "released".
 *
 * LEGACY (audit P5 final): PEDALCAL_FULL_PRESS_PCT / pedal_pressed_full are NO
 * LONGER used.  CAPTURE MAX is latched purely from the RAW ADC (8 stable
 * samples, raw > MIN, span >= PEDAL_CAL_RANGE_MIN) so it can never depend on a
 * percent derived from a possibly-wrong old calibration.  The field is kept
 * only for ABI/telemetry compatibility and is populated as a constant false. */
#define PEDALCAL_RELEASED_PCT     3.0f
/* Deprecated legacy threshold retained for documentation only — must NOT be
 * reintroduced as a CAPTURE MAX dependency. */
#define PEDALCAL_FULL_PRESS_PCT_LEGACY 80.0f

/* ---- 0x319 session-status cadence ---- */
#define PEDALCAL_SESS_PERIOD_MS   100U

/* ======================================================================
 *  Single productive calibration session (audit P5)
 *
 *  Exactly ONE PedalCalSession instance (pedal_cal_session.c) owns the
 *  whole accelerator calibration.  No second FSM exists — the historical
 *  PCAL_FSM_* capture machine and the pedalcal_pending_* RAM endpoints
 *  have been removed.  The HMI CAPTURE MIN / CAPTURE MAX / SAVE / ABORT
 *  buttons map onto Begin / (advisory) / RequestSave / Abort.
 *
 *  Persistence, apply and readback are the real flash-store + pedal
 *  pipeline hooks.  CAPTURE MAX is armed by the CAPTURE MAX button
 *  (PedalCalSession_ArmCaptureMax) and then locked purely from the RAW ADC
 *  (8 stable samples, raw > MIN, span >= PEDAL_CAL_RANGE_MIN) — never from a
 *  percent threshold derived from the old calibration; SAVE requires a final
 *  release and verifies the value read back from flash.
 *
 *  While a session is active the caller (CAN_PedalCalCaptureTick) enforces
 *  the real movement lock every tick (Traction_CalibrationLock: demand 0,
 *  traction PWM 0, traction enables LOW) and requires the traction relay
 *  OFF, so the motors physically cannot move; the session self-aborts on
 *  movement, SAFE, ERROR, emergency, CAN loss, or loss of that lock.
 *
 *  Telemetry:
 *    0x308 : legacy pedal-cal diagnostic burst (unchanged wire format),
 *            now sourced from the session's captured endpoints.
 *    0x319 : new session state + reason + endpoints (CAN_ID_DIAG_PEDAL_CAL_SESSION).
 * ====================================================================== */

static PedalCalSession pedalcal_session;
static bool            pedalcal_session_ready = false;
/* Service lock used to calibrate safely from STANDBY, ACTIVE or a clean
 * DEGRADED recovery window.  It never changes the global system state and
 * never masks SAFE/ERROR: it only forces relays OFF and traction COAST while
 * the guided session is active. */
static bool            pedalcal_service_active = false;
static bool            pedalcal_service_pending = false;
static bool            pedalcal_service_hold = false;
static ServiceHoldPolicy pedalcal_hold_policy = {0};

bool CAN_PedalCalServiceActive(void)
{
    return pedalcal_service_pending || pedalcal_service_active ||
           pedalcal_service_hold;
}

bool CAN_PedalCalServiceConfirmed(void)
{
    return pedalcal_service_active;
}

/* ---- 0x308 telemetry burst state ---- */
static uint8_t   pedalcal_burst_left    = 0;
static uint32_t  pedalcal_next_tx_ms    = 0;
static uint8_t   pedalcal_burst_seq     = 0;

/* ---- 0x319 session-status cadence state ---- */
static uint32_t  pedalcal_sess_next_tx_ms = 0;

static void pedalcal_start_burst(void)
{
    pedalcal_burst_left = PEDALCAL_BURST_FRAMES;
    pedalcal_next_tx_ms = HAL_GetTick();
    pedalcal_burst_seq  = 0U;
}

/* ---- PedalCalSession hooks (real flash store + pedal pipeline) ---- */
static bool pedalcal_hook_validate(uint16_t adc_min, uint16_t adc_max)
{
    return PedalCal_Validate(adc_min, adc_max);
}
static bool pedalcal_hook_persist(uint16_t adc_min, uint16_t adc_max)
{
    return PedalCal_Save(adc_min, adc_max);
}
static void pedalcal_hook_apply(uint16_t adc_min, uint16_t adc_max)
{
    Pedal_ApplyCalibration(adc_min, adc_max);
}
static bool pedalcal_hook_readback(uint16_t *adc_min, uint16_t *adc_max)
{
    if (!PedalCal_IsValid()) return false;
    PedalCal_GetStored(adc_min, adc_max);
    return true;
}

/* (Re)initialise the single session with fresh hooks. */
static void pedalcal_session_configure(void)
{
    PedalCalSessionHooks hooks;
    hooks.validate = pedalcal_hook_validate;
    hooks.persist  = pedalcal_hook_persist;
    hooks.apply    = pedalcal_hook_apply;
    hooks.readback = pedalcal_hook_readback;
    PedalCalSession_Init(&pedalcal_session, NULL, &hooks);
}

static void pedalcal_session_lazy_init(void)
{
    if (pedalcal_session_ready) return;
    pedalcal_session_configure();
    pedalcal_session_ready = true;
}

/* Populate the per-tick condition snapshot from live safety / sensor state. */
static void pedalcal_build_conds(PedalCalConds *c)
{
    const SystemState_t  st   = Safety_GetState();
    const GearPosition_t gear = Traction_GetGear();
    float pct = Pedal_GetPercent();
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    c->now_ms               = HAL_GetTick();
    /* A confirmed service lock is the calibration-equivalent of STANDBY:
     * global state remains observable as ACTIVE/DEGRADED, but movement and
     * power relays are physically inhibited by independent 10 ms guards. */
    c->in_standby           = (st == SYS_STATE_STANDBY) ||
                              CAN_PedalCalServiceConfirmed();
    c->gear_park_or_neutral = (gear == GEAR_PARK || gear == GEAR_NEUTRAL);
    c->wheels_moving        = (Wheel_GetSpeed_FL() >= 0.3f ||
                               Wheel_GetSpeed_FR() >= 0.3f ||
                               Wheel_GetSpeed_RL() >= 0.3f ||
                               Wheel_GetSpeed_RR() >= 0.3f);
    c->pedal_plausible      = Pedal_IsPlausible();
    c->pedal_released       = (pct <  PEDALCAL_RELEASED_PCT);
    /* LEGACY (audit P5 final): pedal_pressed_full is no longer consulted by the
     * session — CAPTURE MAX is raw-ADC based.  Kept false to guarantee no
     * dependency on the calibrated percent. */
    c->pedal_pressed_full   = false;
    c->pedal_raw            = Pedal_SampleRawNow();
    c->critical_error       = (st == SYS_STATE_ERROR);
    c->safe_state           = (st == SYS_STATE_SAFE);
    c->emergency            = (Safety_GetError() == SAFETY_ERROR_EMERGENCY_STOP);
    c->can_loss             = ((Safety_GetFaultFlags() & FAULT_CAN_TIMEOUT) != 0U) ||
                              (st == SYS_STATE_LIMP_HOME);
    /* The pedal calibration must run with traction inhibited.  During STANDBY
     * the traction path is already de-energised, so treat STANDBY as the
     * inhibited precondition (motion-inhibit is only populated once ACTIVE). */
    c->traction_inhibited   = (Traction_GetMotionInhibit() != 0U) ||
                              (st == SYS_STATE_STANDBY) ||
                              CAN_PedalCalServiceActive();
    /* audit P5.4: live, read-only verification that the real movement lock is
     * still holding — traction relay de-energised (0x… status bit1 = 0) and
     * the resolved traction PWM at 0.  CAN_PedalCalCaptureTick actively
     * ENFORCES the lock (Traction_CalibrationLock) before each Update; this
     * flag lets the session abort with LOCK_LOST the instant it is lost. */
    c->traction_locked      = ((Safety_GetRelayStatusByte() & (1U << 1)) == 0U) &&
                              (Traction_GetFinalPwmPct() == 0U);
}

/* Read-only decision for requesting the service lock.  DEGRADED is accepted
 * only after its active error has cleared (the state may remain latched during
 * the recovery debounce).  SAFE, ERROR, LIMP_HOME and any active safety error
 * remain hard blocks. */
static bool pedalcal_service_request_allowed(const PedalCalConds *c)
{
    const SystemState_t st = Safety_GetState();
    const bool state_ok = (st == SYS_STATE_STANDBY) ||
                          (st == SYS_STATE_ACTIVE) ||
                          (st == SYS_STATE_DEGRADED);
    return state_ok &&
           !pedalcal_service_pending && !pedalcal_service_active &&
           !pedalcal_service_hold &&
           Safety_GetError() == SAFETY_ERROR_NONE &&
           c->gear_park_or_neutral && !c->wheels_moving &&
           c->pedal_plausible && c->pedal_released &&
           !c->critical_error && !c->safe_state && !c->emergency &&
           !c->can_loss;
}

/* Enter through a PENDING guard so periodic relay/traction tasks cannot
 * re-energise outputs while the synchronous physical checks are in progress.
 * Only the confirmed ACTIVE phase authorizes persistence. */
static bool pedalcal_service_enter(void)
{
    PedalCalConds c;
    pedalcal_build_conds(&c);
    if (!pedalcal_service_request_allowed(&c)) return false;

    pedalcal_service_pending = true;
    Traction_SetAxisRotation(false);
    Steering_Neutralize();
    Relay_PowerDown();
    const bool en_pwm_locked = Traction_CalibrationLock();
    const bool relay_off = ((Safety_GetRelayStatusByte() & (1U << 1)) == 0U);
    if (!en_pwm_locked || !relay_off) {
        pedalcal_service_pending = false;
        pedalcal_service_hold = true;
        ServiceHoldPolicy_Reset(&pedalcal_hold_policy);
        Relay_PowerDown();
        (void)Traction_CalibrationLock();
        return false;
    }

    pedalcal_service_active = true;
    pedalcal_service_pending = false;
    Relay_PowerDown();
    (void)Traction_CalibrationLock();
    return true;
}

/* Never release directly back to a drive-capable state.  Terminal completion,
 * operator abort, runtime abort and failed entry all transition to HOLD.  HOLD
 * keeps relays OFF and all BTS7960 outputs in COAST until P/N, stopped wheels
 * and a plausible released pedal are observed together. */
static void pedalcal_service_exit(void)
{
    pedalcal_service_pending = false;
    pedalcal_service_active = false;
    pedalcal_service_hold = true;
    ServiceHoldPolicy_Reset(&pedalcal_hold_policy);
    Relay_PowerDown();
    (void)Traction_CalibrationLock();
}

static void pedalcal_service_hold_update(void)
{
    if (!pedalcal_service_hold) return;

    Relay_PowerDown();
    const bool output_locked = Traction_CalibrationLock();
    const bool relay_off = ((Safety_GetRelayStatusByte() & (1U << 1)) == 0U);

    PedalCalConds c;
    pedalcal_build_conds(&c);
    const bool safe_rearm = output_locked && relay_off &&
                            c.gear_park_or_neutral && !c.wheels_moving &&
                            c.pedal_plausible && c.pedal_released &&
                            !c.critical_error && !c.safe_state &&
                            !c.emergency && !c.can_loss;
    if (ServiceHoldPolicy_Update(&pedalcal_hold_policy, safe_rearm,
                                 HAL_GetTick())) {
        pedalcal_service_hold = false;
    }
}

/* True when the current live entry guards would permit a session to start.
 *
 * audit fix — this MUST be strictly read-only: it feeds the informative 0x319
 * "entry OK" bit and is reachable from QUERY / telemetry without starting a
 * session.  It therefore uses Traction_IsCalibrationLockConfirmed() (a pure
 * read) instead of Traction_CalibrationLock() (which forces demand 0 / PWM 0 /
 * traction enables LOW).  The real lock is enforced ONLY from CAPTURE_MIN just
 * before Begin and from CAN_PedalCalCaptureTick() during an active session. */
static bool pedalcal_entry_ok(void)
{
    PedalCalConds c;
    pedalcal_build_conds(&c);
    /* While locked, report the real physical lock.  Before BEGIN, report that
     * the request is eligible to enter service lock; do not mutate outputs. */
    if (pedalcal_service_active) {
        c.traction_locked = Traction_IsCalibrationLockConfirmed();
        return c.in_standby && c.gear_park_or_neutral && !c.wheels_moving &&
               c.pedal_plausible && !c.critical_error &&
               c.traction_inhibited && c.traction_locked;
    }
    return pedalcal_service_request_allowed(&c);
}

/* ---- 0x308 live reject-reason (PEDCAL_REJECT_* form, unchanged contract) ----
 * These populate the 0x308 diagnostic frame's reject bitmask so the existing
 * ESP32 0x308 view keeps working.  The SESSION's own reason (PEDAL_CAL_SESS_*)
 * is transported separately on 0x319.                                        */
static uint16_t pedalcal_reject_live_safety(void)
{
    uint16_t bits = 0U;
    PedalCalConds live;
    pedalcal_build_conds(&live);
    if (!pedalcal_service_active &&
        !pedalcal_service_request_allowed(&live)) {
        bits |= PEDCAL_REJECT_NOT_STANDBY;
    }
    /* audit P5.5: PEDAL-NOT-RELEASED is a legitimate gate ONLY while the
     * session expects a released pedal (MIN capture / release-for-save).  In
     * the WAIT_FULL_PRESS / CAPTURING_MAX phases the operator MUST press the
     * pedal, so a pressed pedal there is normal — reporting it as a blocked
     * "Safety gate" on 0x308 would be wrong. */
    {
        const PedalCalState pst = pedalcal_session.state;
        const bool max_phase = (pst == PEDAL_CAL_WAIT_FULL_PRESS) ||
                               (pst == PEDAL_CAL_CAPTURING_MAX);
        if (!max_phase && (Pedal_GetPercent() >= PEDALCAL_RELEASED_PCT)) {
            bits |= PEDCAL_REJECT_PEDAL_NOT_RELEASED;
        }
    }
    if (!Pedal_IsPlausible())                     bits |= PEDCAL_REJECT_PEDAL_NOT_PLAUSIBLE;
    if (Wheel_GetSpeed_FL() >= 0.3f ||
        Wheel_GetSpeed_FR() >= 0.3f ||
        Wheel_GetSpeed_RL() >= 0.3f ||
        Wheel_GetSpeed_RR() >= 0.3f) {
        bits |= PEDCAL_REJECT_WHEELS_MOVING;
    }
    return bits;
}
static uint16_t pedalcal_reject_live_pair(void)
{
    uint16_t bits = 0U;
    if (!pedalcal_session.have_min || !pedalcal_session.have_max) {
        bits |= PEDCAL_REJECT_PENDING_INCOMPLETE;
        return bits;
    }
    const uint16_t mn = pedalcal_session.adc_min;
    const uint16_t mx = pedalcal_session.adc_max;
    if (mx <= mn) {
        bits |= PEDCAL_REJECT_MIN_GT_MAX;
        bits |= PEDCAL_REJECT_RANGE_INVALID;
    }
    if ((mx > mn) && ((uint32_t)(mx - mn) < PEDAL_CAL_RANGE_MIN)) {
        bits |= PEDCAL_REJECT_RANGE_TOO_SMALL;
        bits |= PEDCAL_REJECT_RANGE_INVALID;
    }
    if (mx > PEDAL_CAL_MAX_LIMIT) {
        bits |= PEDCAL_REJECT_MAX_TOO_HIGH;
        bits |= PEDCAL_REJECT_RANGE_INVALID;
    }
    if (!PedalCal_Validate(mn, mx)) {
        bits |= PEDCAL_REJECT_RANGE_INVALID;
    }
    return bits;
}
static uint16_t pedalcal_get_reject_reason(void)
{
    return (uint16_t)(pedalcal_reject_live_safety() | pedalcal_reject_live_pair());
}
static inline bool pedalcal_safety_ok(void)
{
    return (pedalcal_reject_live_safety() == 0U);
}

/* 0x308 pedal-cal diagnostic burst (DLC 8, little-endian).  Wire format is
 * unchanged from the historical contract; the pending endpoints are now the
 * session's captured MIN/MAX.  See the ESP32 decodePedalCal() for the layout:
 *   byte 0   : flags (bit0 pending MIN, bit1 pending MAX, bit2 pair valid,
 *              bit3 stored slot valid, bit4 safety gates satisfied,
 *              bit5 pedal plausible, bit6 pair selector 0=PENDING/1=STORED,
 *              bit7 0=pair frame / 1=diag frame)
 *   pair frame : b1-2 raw ADC, b3-4 MIN, b5-6 MAX, b7 percent
 *   diag frame : b1-2 raw ADC2, b3-4 |raw1-raw2|, b5-6 reject_reason, b7 pct */
static void pedalcal_send_status(void)
{
    uint8_t phase = (uint8_t)(pedalcal_burst_seq % 3U);
    bool send_diag   = (phase == 1U);
    bool send_stored = (phase == 2U);

    uint16_t stored_min = 0, stored_max = 0;
    PedalCal_GetStored(&stored_min, &stored_max);

    const bool     have_min  = pedalcal_session.have_min;
    const bool     have_max  = pedalcal_session.have_max;
    const uint16_t pend_min  = pedalcal_session.adc_min;
    const uint16_t pend_max  = pedalcal_session.adc_max;

    uint8_t  flags = 0;
    if (have_min)  flags |= 0x01U;
    if (have_max)  flags |= 0x02U;
    if (have_min && have_max && PedalCal_Validate(pend_min, pend_max))
        flags |= 0x04U;
    if (PedalCal_IsValid())   flags |= 0x08U;
    if (pedalcal_safety_ok()) flags |= 0x10U;
    if (Pedal_IsPlausible())  flags |= 0x20U;
    if (send_stored)          flags |= 0x40U;
    if (send_diag)            flags |= 0x80U;

    uint16_t raw_adc  = Pedal_GetRawADC();
    uint16_t raw_adc2 = Pedal_GetRawADC2();
    uint16_t diff_raw = Pedal_GetRawADCDiff();
    uint16_t reject_reason = pedalcal_get_reject_reason();
    float    pct_f   = Pedal_GetPercent();
    if (pct_f < 0.0f)   pct_f = 0.0f;
    if (pct_f > 100.0f) pct_f = 100.0f;

    uint16_t mn = send_stored ? stored_min : pend_min;
    uint16_t mx = send_stored ? stored_max : pend_max;

    uint8_t payload[8];
    payload[0] = flags;
    if (send_diag) {
        payload[1] = (uint8_t)(raw_adc2 & 0xFFU);
        payload[2] = (uint8_t)((raw_adc2 >> 8) & 0xFFU);
        payload[3] = (uint8_t)(diff_raw & 0xFFU);
        payload[4] = (uint8_t)((diff_raw >> 8) & 0xFFU);
        payload[5] = (uint8_t)(reject_reason & 0xFFU);
        payload[6] = (uint8_t)((reject_reason >> 8) & 0xFFU);
    } else {
        payload[1] = (uint8_t)(raw_adc & 0xFFU);
        payload[2] = (uint8_t)((raw_adc >> 8) & 0xFFU);
        payload[3] = (uint8_t)(mn & 0xFFU);
        payload[4] = (uint8_t)((mn >> 8) & 0xFFU);
        payload[5] = (uint8_t)(mx & 0xFFU);
        payload[6] = (uint8_t)((mx >> 8) & 0xFFU);
    }
    payload[7] = (uint8_t)pct_f;

    (void)TransmitFrame(CAN_ID_DIAG_PEDAL_CAL, payload, sizeof(payload));
}

/* 0x319 PedalCalSession status (DLC 8, little-endian):
 *   byte 0    : session state (PedalCalState, CAN-stable enum)
 *   byte 1    : flags — bit0 active, bit1 have_min, bit2 have_max,
 *               bit3 completed, bit4 aborted, bit5 entry guards OK now,
 *               bit6 operator-cancel abort, bit7 movement-lock-lost abort
 *   bytes 2-3 : reason bitmask low 16 (PEDAL_CAL_SESS_* / BLOCK_* / ABORT_* /
 *               FAIL_*); the extended OPERATOR/LOCK_LOST causes live above
 *               bit 15 and are surfaced via flag bits 6/7 above
 *   bytes 4-5 : captured MIN adc (u16 LE)
 *   bytes 6-7 : captured MAX adc (u16 LE)                                    */
static void pedalcal_send_session_status(void)
{
    const PedalCalSession *s = &pedalcal_session;
    uint8_t flags = 0U;
    if (PedalCalSession_Active(s))            flags |= 0x01U;
    if (s->have_min)                          flags |= 0x02U;
    if (s->have_max)                          flags |= 0x04U;
    if (s->state == PEDAL_CAL_COMPLETED)      flags |= 0x08U;
    if (s->state == PEDAL_CAL_ABORTED)        flags |= 0x10U;
    if (pedalcal_entry_ok())                  flags |= 0x20U;
    if (s->reason & PEDAL_CAL_ABORT_OPERATOR)  flags |= 0x40U;
    if (s->reason & PEDAL_CAL_ABORT_LOCK_LOST) flags |= 0x80U;

    uint8_t payload[8];
    payload[0] = (uint8_t)s->state;
    payload[1] = flags;
    payload[2] = (uint8_t)(s->reason & 0xFFU);
    payload[3] = (uint8_t)((s->reason >> 8) & 0xFFU);
    payload[4] = (uint8_t)(s->adc_min & 0xFFU);
    payload[5] = (uint8_t)((s->adc_min >> 8) & 0xFFU);
    payload[6] = (uint8_t)(s->adc_max & 0xFFU);
    payload[7] = (uint8_t)((s->adc_max >> 8) & 0xFFU);
    (void)TransmitFrame(CAN_ID_DIAG_PEDAL_CAL_SESSION, payload, sizeof(payload));
}

/* Public: drive the 0x308 + 0x319 burst from the 100 Hz main-loop tick.
 * No effect when no burst is in progress.                            */
void CAN_PedalCalBurstUpdate(void)
{
    if (pedalcal_burst_left == 0U) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - pedalcal_next_tx_ms) < 0) return;
    pedalcal_send_status();
    pedalcal_send_session_status();
    pedalcal_burst_left--;
    pedalcal_burst_seq++;
    pedalcal_next_tx_ms = now + PEDALCAL_BURST_PERIOD_MS;
}

/* Handle a SERVICE_CMD frame with byte 0 == SERVICE_ACTION_PEDAL_CAL.
 * Buttons map onto the single PedalCalSession:
 *   0x01 CAPTURE_MIN    -> Begin the guided session
 *   0x02 CAPTURE_MAX    -> arm the raw-based MAX capture; request a burst
 *   0x03 SAVE           -> RequestSave (validate + persist + apply + verify)
 *   0x04 RESET_DEFAULTS -> restore 50 / 4000 and reset the session
 *   0x05 QUERY          -> request a 1 s telemetry burst
 *   0x06 ABORT          -> operator abort
 * Always replies with one CAN_SendCommandAck(0x10, ...).             */
static void pedalcal_handle_service_cmd(const uint8_t *payload, uint8_t len)
{
    if (len < 2) {
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
    pedalcal_session_lazy_init();
    uint8_t op = payload[1];

    switch (op) {
    case PEDAL_CAL_OP_QUERY:
        pedalcal_start_burst();
        pedalcal_send_session_status();
        CAN_SendCommandAck(0x10, ACK_OK);
        return;

    case PEDAL_CAL_OP_CAPTURE_MIN: {   /* BEGIN the guided session */
        /* A repeated BEGIN must never tear down an already-owned lock. */
        if (PedalCalSession_Active(&pedalcal_session)) {
            pedalcal_start_burst();
            pedalcal_send_session_status();
            CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
            return;
        }
        /* Enter a physical service lock first.  This permits calibration from
         * ACTIVE or a clean DEGRADED recovery window without weakening the
         * global state graph or disabling any safety transition. */
        const bool service_ok = pedalcal_service_enter();
        PedalCalConds c;
        pedalcal_build_conds(&c);
        c.traction_locked = service_ok && Traction_IsCalibrationLockConfirmed();
        const bool begin_ok = PedalCalSession_Begin(&pedalcal_session, &c);
        const bool ok = service_ok && begin_ok;
        /* A rejected BEGIN whose preconditions were never met did not
         * touch the powertrain.  Enter HOLD only after service_enter()
         * actually acquired the physical lock. */
        if (!ok && service_ok) pedalcal_service_exit();
        pedalcal_start_burst();
        pedalcal_send_session_status();
        if (ok) {
            CAN_SendCommandAck(0x10, ACK_OK);
        } else {
            uint32_t r = PedalCalSession_Reason(&pedalcal_session);
            const uint32_t safety_bits =
                (PEDAL_CAL_BLOCK_NOT_STANDBY | PEDAL_CAL_BLOCK_GEAR |
                 PEDAL_CAL_BLOCK_WHEELS_MOVING | PEDAL_CAL_BLOCK_TRACTION_LIVE |
                 PEDAL_CAL_BLOCK_CRITICAL_ERROR | PEDAL_CAL_BLOCK_PEDAL_IMPLAUSIBLE |
                 PEDAL_CAL_BLOCK_LOCK_NOT_CONFIRMED);
            CAN_SendCommandAck(0x10, (r & safety_bits) ? ACK_BLOCKED_BY_SAFETY
                                                       : ACK_REJECTED);
        }
        return;
    }

    case PEDAL_CAL_OP_CAPTURE_MAX:     /* ARM the raw-based MAX capture (audit P5) */
        PedalCalSession_ArmCaptureMax(&pedalcal_session);
        pedalcal_start_burst();
        pedalcal_send_session_status();
        CAN_SendCommandAck(0x10, ACK_OK);
        return;

    case PEDAL_CAL_OP_SAVE: {
        PedalCalConds c;
        pedalcal_build_conds(&c);
        PedalCalSession_RequestSave(&pedalcal_session, &c);
        pedalcal_start_burst();
        pedalcal_send_session_status();
        PedalCalState st = PedalCalSession_State(&pedalcal_session);
        if (st == PEDAL_CAL_COMPLETED) {
            CAN_SendCommandAck(0x10, ACK_OK);
            pedalcal_service_exit();
        } else if (st == PEDAL_CAL_ABORTED) {
            uint32_t r = PedalCalSession_Reason(&pedalcal_session);
            CAN_SendCommandAck(0x10, (r & PEDAL_CAL_FAIL_READBACK)
                                         ? ACK_REJECTED : ACK_INVALID);
            pedalcal_service_exit();
        } else {
            /* Not in READY_TO_SAVE (e.g. pedal not released yet). */
            CAN_SendCommandAck(0x10, ACK_REJECTED);
        }
        return;
    }

    case PEDAL_CAL_OP_ABORT:
        /* Reject silently when there is no active service context: no session
         * was pending, active, or in HOLD, so there is nothing to abort.
         * Calling service_exit() unconditionally would power down relays and
         * enter HOLD even on an actively-driving vehicle.                    */
        if (!pedalcal_service_pending && !pedalcal_service_active &&
                !pedalcal_service_hold) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        /* audit P5.3: the ABORT button is a normal operator cancellation, NOT
         * an emergency — classify it as PEDAL_CAL_ABORT_OPERATOR. */
        PedalCalSession_Abort(&pedalcal_session, PEDAL_CAL_ABORT_OPERATOR);
        pedalcal_start_burst();
        pedalcal_send_session_status();
        pedalcal_service_exit();
        CAN_SendCommandAck(0x10, ACK_OK);
        return;

    case PEDAL_CAL_OP_RESET_DEFAULTS:
        if (!pedalcal_service_enter()) {
            CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
            return;
        }
        if (!PedalCal_Save(PEDAL_CAL_DEFAULT_MIN, PEDAL_CAL_DEFAULT_MAX)) {
            pedalcal_start_burst();
            pedalcal_service_exit();
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        Pedal_ApplyCalibration(PEDAL_CAL_DEFAULT_MIN, PEDAL_CAL_DEFAULT_MAX);
        pedalcal_session_configure();   /* clear any captured pending pair */
        pedalcal_start_burst();
        pedalcal_send_session_status();
        pedalcal_service_exit();
        CAN_SendCommandAck(0x10, ACK_OK);
        return;

    default:
        pedalcal_start_burst();
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
}

/* Productive PedalCalSession driver (audit P5).  Called from the main loop's
 * 50 ms branch immediately after Pedal_Update().  No-op while the session is
 * IDLE / terminal.  While active it:
 *   1. Enforces the real movement lock (Traction_CalibrationLock: demand 0,
 *      traction PWM 0, traction enables LOW) + requires the traction relay
 *      OFF, verified every tick, so the motors physically cannot move.
 *   2. Builds the live PedalCalConds and advances PedalCalSession_Update()
 *      (which self-captures MIN, MAX only once armed, and self-aborts on
 *      movement/SAFE/ERROR/emergency/CAN loss/timeout/lock loss).
 *   3. Publishes 0x319 on every state change (and a 1 s 0x308+0x319 burst),
 *      otherwise rate-limited to ~100 ms.                              */
void CAN_PedalCalCaptureTick(void)
{
    pedalcal_session_lazy_init();
    pedalcal_service_hold_update();
    if (!PedalCalSession_Active(&pedalcal_session)) return;

    /* Enforce the REAL movement lock for the WHOLE session (audit P5.4):
     * demand 0, traction PWM 0, traction enables LOW — verified every tick.
     * The traction relay is also required OFF; combine both into the live
     * traction_locked condition so any loss aborts the session (LOCK_LOST). */
    Relay_PowerDown();
    bool en_pwm_locked = Traction_CalibrationLock();
    bool relay_off     = ((Safety_GetRelayStatusByte() & (1U << 1)) == 0U);

    PedalCalConds c;
    pedalcal_build_conds(&c);
    c.traction_locked = en_pwm_locked && relay_off;
    PedalCalState prev = pedalcal_session.state;
    PedalCalState st   = PedalCalSession_Update(&pedalcal_session, &c);

    uint32_t now = HAL_GetTick();
    if (st != prev) {
        pedalcal_start_burst();          /* 1 s of 0x308+0x319 on any change */
        pedalcal_send_session_status();
        pedalcal_sess_next_tx_ms = now + PEDALCAL_SESS_PERIOD_MS;
    } else if ((int32_t)(now - pedalcal_sess_next_tx_ms) >= 0) {
        pedalcal_send_session_status();
        pedalcal_sess_next_tx_ms = now + PEDALCAL_SESS_PERIOD_MS;
    }

    if (!PedalCalSession_Active(&pedalcal_session)) {
        pedalcal_service_exit();
    }
}


/* ==================================================================
 *  Gear power-limit configuration (0xF7 sub-protocol + 0x30D telemetry)
 *
 *  Lets the ESP32 Engineering menu view and tune the per-gear traction
 *  power limits (D2 / D1 / R) applied in motor_control.c.  SET_* sub-
 *  opcodes stage values in RAM ("pending"); nothing is applied or
 *  persisted until SAVE.  This module owns all protocol/staging state;
 *  it touches the traction controller only via the public
 *  Traction_*GearLimits() helpers and the flash store
 *  (gear_limits_store.c).
 *
 *  Safety invariant (gearlim_edit_allowed()):
 *    - Safety_GetState() == SYS_STATE_STANDBY
 *  Changing a traction power limit is only permitted while parked in
 *  STANDBY; QUERY is exempt (read-only telemetry request).
 *
 *  0x30D telemetry burst (DLC 8): emitted only on demand — after a QUERY
 *  sub-opcode, 10 frames spaced 100 ms apart (1 s total), then silence.
 *  Backward-compatible: nodes that ignore 0x30D see no extra traffic.
 * ================================================================== */

#define GEARLIM_BURST_FRAMES     10U    /* 10 × 100 ms = 1 s             */
#define GEARLIM_BURST_PERIOD_MS  100U

/* ---- Pending limits (RAM-only until SAVE) ----
 * Seeded lazily from the active limits the first time a SET arrives, so
 * a partial edit (e.g. only D1 touched) carries the live values for the
 * untouched gears into SAVE.  Both the power limits and the accel-response
 * profile are staged together and committed atomically on SAVE.         */
static bool      gearlim_pending_seeded = false;
static uint8_t   gearlim_pending_d2     = 0;
static uint8_t   gearlim_pending_d1     = 0;
static uint8_t   gearlim_pending_r      = 0;
static uint8_t   gearlim_pending_d2_resp = 0;
static uint8_t   gearlim_pending_d1_resp = 0;
static uint8_t   gearlim_pending_r_resp  = 0;

/* ---- 0x30D telemetry burst state ---- */
static uint8_t   gearlim_burst_left     = 0;
static uint32_t  gearlim_next_tx_ms     = 0;

/* Gear-profile mutations are deliberately STANDBY-only.  QUERY remains
 * read-only and exempt, but SET/SAVE/RESET must never stage or persist values
 * from ACTIVE/DEGRADED even when the vehicle happens to be stopped. */
static bool gearlim_edit_allowed(void)
{
    return Safety_GetState() == SYS_STATE_STANDBY &&
           Safety_GetError() == SAFETY_ERROR_NONE &&
           !CAN_PedalCalServiceActive();
}

static bool gearlim_save_context_confirmed(void)
{
    return gearlim_edit_allowed() &&
           ((Safety_GetRelayStatusByte() & (1U << 1)) == 0U) &&
           Traction_GetFinalPwmPct() == 0U;
}

static void gearlim_seed_pending_if_needed(void)
{
    if (gearlim_pending_seeded) return;
    Traction_GetGearLimits(&gearlim_pending_d2,
                           &gearlim_pending_d1,
                           &gearlim_pending_r);
    Traction_GetGearResponse(&gearlim_pending_d2_resp,
                             &gearlim_pending_d1_resp,
                             &gearlim_pending_r_resp);
    gearlim_pending_seeded = true;
}

/* Emit one 0x30D frame (DLC 8, little-endian).
 *
 * Two interleaved frame kinds share the 0x30D ID, distinguished by
 * byte0 bit4 (0x10):
 *   bit4 = 0  POWER   frame: bytes 1-3 active power limits, 4-6 pending
 *   bit4 = 1  RESPONSE frame: bytes 1-3 active response %, 4-6 pending
 * The remaining flag bits keep the same meaning in both kinds, so the
 * ESP32 editor can drive its pending/validate UX from either frame and
 * must preserve the "other half" of GearLimitsData when decoding.
 *
 *   byte 0   : flags
 *              bit 0: stored slot valid (GearLimitsStore_IsValid())
 *              bit 1: pending differs from active (unsaved edit, this half)
 *              bit 2: safety gate satisfied (STANDBY)
 *              bit 3: pending set validates OK (this half)
 *              bit 4: frame kind (0 = power, 1 = response)
 *              bits 5-7: reserved (0)
 *   byte 1   : active  D2 value (percent)
 *   byte 2   : active  D1 value (percent)
 *   byte 3   : active  R  value (percent)
 *   byte 4   : pending D2 value (percent)
 *   byte 5   : pending D1 value (percent)
 *   byte 6   : pending R  value (percent)
 *   byte 7   : system state (Safety_GetState())                        */
static void gearlim_send_status_kind(bool response_frame)
{
    uint8_t act_d2, act_d1, act_r;
    uint8_t pend_d2, pend_d1, pend_r;
    bool valid_ok;

    if (response_frame) {
        Traction_GetGearResponse(&act_d2, &act_d1, &act_r);
        pend_d2 = act_d2; pend_d1 = act_d1; pend_r = act_r;
        if (gearlim_pending_seeded) {
            pend_d2 = gearlim_pending_d2_resp;
            pend_d1 = gearlim_pending_d1_resp;
            pend_r  = gearlim_pending_r_resp;
        }
        valid_ok = Traction_ValidateGearResponse(pend_d2, pend_d1, pend_r);
    } else {
        Traction_GetGearLimits(&act_d2, &act_d1, &act_r);
        pend_d2 = act_d2; pend_d1 = act_d1; pend_r = act_r;
        if (gearlim_pending_seeded) {
            pend_d2 = gearlim_pending_d2;
            pend_d1 = gearlim_pending_d1;
            pend_r  = gearlim_pending_r;
        }
        valid_ok = Traction_ValidateGearLimits(pend_d2, pend_d1, pend_r);
    }

    uint8_t flags = 0;
    if (GearLimitsStore_IsValid()) flags |= 0x01U;
    if (gearlim_pending_seeded &&
        (pend_d2 != act_d2 || pend_d1 != act_d1 || pend_r != act_r))
        flags |= 0x02U;
    if (gearlim_edit_allowed()) flags |= 0x04U;
    if (valid_ok) flags |= 0x08U;
    if (response_frame) flags |= 0x10U;

    uint8_t payload[8];
    payload[0] = flags;
    payload[1] = act_d2;
    payload[2] = act_d1;
    payload[3] = act_r;
    payload[4] = pend_d2;
    payload[5] = pend_d1;
    payload[6] = pend_r;
    payload[7] = (uint8_t)Safety_GetState();

    (void)TransmitFrame(CAN_ID_DIAG_GEAR_LIMITS, payload, sizeof(payload));
}

static void gearlim_send_status(void)
{
    /* Send both kinds back-to-back so a single QUERY refreshes the whole
     * editor (power + response) without needing two separate requests.  */
    gearlim_send_status_kind(false);
    gearlim_send_status_kind(true);
}

/* Public: drive the 0x30D burst from the 100 Hz main-loop tick. */
void CAN_GearLimitsBurstUpdate(void)
{
    if (gearlim_burst_left == 0U) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - gearlim_next_tx_ms) < 0) return;
    gearlim_send_status();
    gearlim_burst_left--;
    gearlim_next_tx_ms = now + GEARLIM_BURST_PERIOD_MS;
}

/* Handle a SERVICE_CMD frame with byte 0 == SERVICE_ACTION_GEAR_LIMITS.
 * Always replies with one CAN_SendCommandAck(0x10, ...).               */
static void gearlim_handle_service_cmd(const uint8_t *payload, uint8_t len)
{
    if (len < 2) {
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
    uint8_t op = payload[1];

    /* QUERY is read-only — no safety gate, just a telemetry burst. */
    if (op == GEAR_LIMIT_OP_QUERY) {
        gearlim_burst_left = GEARLIM_BURST_FRAMES;
        gearlim_next_tx_ms = HAL_GetTick();   /* emit on next tick */
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }

    /* All other sub-opcodes require STANDBY. */
    if (!gearlim_edit_allowed()) {
        CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
        return;
    }

    switch (op) {
    case GEAR_LIMIT_OP_SET_D2:
    case GEAR_LIMIT_OP_SET_D1:
    case GEAR_LIMIT_OP_SET_R: {
        if (len < 3) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        uint8_t val = payload[2];
        gearlim_seed_pending_if_needed();
        /* Stage into a temporary so we can validate the WHOLE pending set
         * (cross-field constraints) before committing the single field.  */
        uint8_t d2 = gearlim_pending_d2;
        uint8_t d1 = gearlim_pending_d1;
        uint8_t r  = gearlim_pending_r;
        if      (op == GEAR_LIMIT_OP_SET_D2) d2 = val;
        else if (op == GEAR_LIMIT_OP_SET_D1) d1 = val;
        else                                 r  = val;
        if (!Traction_ValidateGearLimits(d2, d1, r)) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        gearlim_pending_d2 = d2;
        gearlim_pending_d1 = d1;
        gearlim_pending_r  = r;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    case GEAR_LIMIT_OP_SET_D2_RESPONSE:
    case GEAR_LIMIT_OP_SET_D1_RESPONSE:
    case GEAR_LIMIT_OP_SET_R_RESPONSE: {
        if (len < 3) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        uint8_t val = payload[2];
        gearlim_seed_pending_if_needed();
        uint8_t d2 = gearlim_pending_d2_resp;
        uint8_t d1 = gearlim_pending_d1_resp;
        uint8_t r  = gearlim_pending_r_resp;
        if      (op == GEAR_LIMIT_OP_SET_D2_RESPONSE) d2 = val;
        else if (op == GEAR_LIMIT_OP_SET_D1_RESPONSE) d1 = val;
        else                                          r  = val;
        if (!Traction_ValidateGearResponse(d2, d1, r)) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        gearlim_pending_d2_resp = d2;
        gearlim_pending_d1_resp = d1;
        gearlim_pending_r_resp  = r;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    case GEAR_LIMIT_OP_SAVE: {
        if (!gearlim_pending_seeded) {
            /* Nothing staged — nothing to save. */
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        if (!Traction_ValidateGearLimits(gearlim_pending_d2,
                                         gearlim_pending_d1,
                                         gearlim_pending_r) ||
            !Traction_ValidateGearResponse(gearlim_pending_d2_resp,
                                           gearlim_pending_d1_resp,
                                           gearlim_pending_r_resp)) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }

        if (!gearlim_save_context_confirmed()) {
            CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
            return;
        }
        const bool saved = GearLimitsStore_Save(gearlim_pending_d2,
                                                 gearlim_pending_d1,
                                                 gearlim_pending_r,
                                                 gearlim_pending_d2_resp,
                                                 gearlim_pending_d1_resp,
                                                 gearlim_pending_r_resp);
        if (!saved) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        /* Apply only while the same physical lock still owns the vehicle. */
        (void)Traction_SetGearLimits(gearlim_pending_d2,
                                     gearlim_pending_d1,
                                     gearlim_pending_r);
        (void)Traction_SetGearResponse(gearlim_pending_d2_resp,
                                       gearlim_pending_d1_resp,
                                       gearlim_pending_r_resp);
        gearlim_pending_seeded = false;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    case GEAR_LIMIT_OP_RESET_DEFAULTS: {
        if (!gearlim_save_context_confirmed()) {
            CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
            return;
        }
        const bool saved = GearLimitsStore_Save(GEAR_LIMIT_D2_DEFAULT_PCT,
                                                 GEAR_LIMIT_D1_DEFAULT_PCT,
                                                 GEAR_LIMIT_R_DEFAULT_PCT,
                                                 GEAR_RESPONSE_D2_DEFAULT_PCT,
                                                 GEAR_RESPONSE_D1_DEFAULT_PCT,
                                                 GEAR_RESPONSE_R_DEFAULT_PCT);
        if (!saved) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        (void)Traction_SetGearLimits(GEAR_LIMIT_D2_DEFAULT_PCT,
                                     GEAR_LIMIT_D1_DEFAULT_PCT,
                                     GEAR_LIMIT_R_DEFAULT_PCT);
        (void)Traction_SetGearResponse(GEAR_RESPONSE_D2_DEFAULT_PCT,
                                       GEAR_RESPONSE_D1_DEFAULT_PCT,
                                       GEAR_RESPONSE_R_DEFAULT_PCT);
        gearlim_pending_seeded = false;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    default:
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
}

/* ==================================================================
 *  Drive-tuning configuration (0xFA sub-protocol + 0x310 telemetry)
 *
 *  Lets the ESP32 Engineering menu view and tune the traction "feel"
 *  parameters (accel / brake / reverse ramp rates and the creep dead-
 *  zone) applied in motor_control.c.  SET_* sub-opcodes stage values in
 *  RAM ("pending"); nothing is applied or persisted until SAVE.  This
 *  module owns all protocol/staging state and touches the traction
 *  controller only via the public Traction_*DriveTuning() helpers and
 *  the flash store (drive_tuning_store.c).
 *
 *  Safety invariant (drvtune_safety_ok()): Safety_GetState() == STANDBY.
 *  QUERY is exempt (read-only telemetry request).
 *
 *  0x310 telemetry burst (DLC 8): emitted only on demand — after a QUERY,
 *  10 sweeps spaced 100 ms apart (1 s total), each sweep streaming all
 *  DRIVE_TUNE_FIELD_COUNT fields one frame per field.  Backward-compatible:
 *  nodes that ignore 0x310 see no extra traffic.
 *
 *  0x310 frame layout (one field per frame, little-endian):
 *    byte 0 : flags
 *             bit0: stored slot valid (DriveTuningStore_IsValid())
 *             bit1: pending differs from active (unsaved edit)
 *             bit2: safety gate satisfied (STANDBY)
 *             bit3: pending set validates OK
 *             bits4-7: reserved (0)
 *    byte 1 : field id (DRIVE_TUNE_FIELD_*)
 *    byte 2-3: active  value (uint16 LE)
 *    byte 4-5: pending value (uint16 LE)
 *    byte 6 : system state (Safety_GetState())
 *    byte 7 : total field count (DRIVE_TUNE_FIELD_COUNT)
 * ================================================================== */

#define DRVTUNE_BURST_SWEEPS     10U    /* 10 × 100 ms = 1 s             */
#define DRVTUNE_BURST_PERIOD_MS  100U

static bool          drvtune_pending_seeded = false;
static DriveTuning_t drvtune_pending;            /* RAM-only until SAVE  */

static uint8_t       drvtune_burst_left = 0;
static uint32_t      drvtune_next_tx_ms = 0;

static inline bool drvtune_safety_ok(void)
{
    return (Safety_GetState() == SYS_STATE_STANDBY);
}

static void drvtune_seed_pending_if_needed(void)
{
    if (drvtune_pending_seeded) return;
    Traction_GetDriveTuning(&drvtune_pending);
    drvtune_pending_seeded = true;
}

/* Extract the (active, pending) value pair for a telemetry field id. */
static void drvtune_field_values(uint8_t field, const DriveTuning_t *act,
                                 const DriveTuning_t *pend,
                                 uint16_t *act_out, uint16_t *pend_out)
{
    switch (field) {
    case DRIVE_TUNE_FIELD_ACCEL_RAMP:
        *act_out = act->accel_ramp;   *pend_out = pend->accel_ramp;   break;
    case DRIVE_TUNE_FIELD_BRAKE_RAMP:
        *act_out = act->brake_ramp;   *pend_out = pend->brake_ramp;   break;
    case DRIVE_TUNE_FIELD_REVERSE_RAMP:
        *act_out = act->reverse_ramp; *pend_out = pend->reverse_ramp; break;
    case DRIVE_TUNE_FIELD_CREEP_ENABLE:
        *act_out = act->creep_enable; *pend_out = pend->creep_enable; break;
    case DRIVE_TUNE_FIELD_CREEP_POWER:
        *act_out = act->creep_power;  *pend_out = pend->creep_power;  break;
    case DRIVE_TUNE_FIELD_CREEP_DELAY:
        *act_out = act->creep_delay;  *pend_out = pend->creep_delay;  break;
    default:
        *act_out = 0; *pend_out = 0; break;
    }
}

static void drvtune_send_field(uint8_t field)
{
    DriveTuning_t act;
    Traction_GetDriveTuning(&act);

    DriveTuning_t pend = act;
    if (drvtune_pending_seeded) pend = drvtune_pending;

    uint16_t act_v = 0, pend_v = 0;
    drvtune_field_values(field, &act, &pend, &act_v, &pend_v);

    bool valid_ok = Traction_ValidateDriveTuning(&pend);

    uint8_t flags = 0;
    if (DriveTuningStore_IsValid()) flags |= 0x01U;
    if (drvtune_pending_seeded && (act_v != pend_v)) flags |= 0x02U;
    if (drvtune_safety_ok()) flags |= 0x04U;
    if (valid_ok) flags |= 0x08U;

    uint8_t payload[8];
    payload[0] = flags;
    payload[1] = field;
    payload[2] = (uint8_t)(act_v  & 0xFFU);
    payload[3] = (uint8_t)(act_v  >> 8);
    payload[4] = (uint8_t)(pend_v & 0xFFU);
    payload[5] = (uint8_t)(pend_v >> 8);
    payload[6] = (uint8_t)Safety_GetState();
    payload[7] = (uint8_t)DRIVE_TUNE_FIELD_COUNT;

    (void)TransmitFrame(CAN_ID_DIAG_DRIVE_TUNING, payload, sizeof(payload));
}

static void drvtune_send_sweep(void)
{
    for (uint8_t f = 1U; f <= DRIVE_TUNE_FIELD_COUNT; ++f) {
        drvtune_send_field(f);
    }
}

void CAN_DriveTuningBurstUpdate(void)
{
    if (drvtune_burst_left == 0U) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - drvtune_next_tx_ms) < 0) return;
    drvtune_send_sweep();
    drvtune_burst_left--;
    drvtune_next_tx_ms = now + DRVTUNE_BURST_PERIOD_MS;
}

/* Apply a SET sub-opcode to a staged copy, returning false if the field
 * value is out of the uint16 wire range for that field.                */
static void drvtune_apply_set(DriveTuning_t *t, uint8_t op, uint16_t val)
{
    switch (op) {
    case DRIVE_TUNE_OP_SET_ACCEL_RAMP:   t->accel_ramp   = (uint8_t)val; break;
    case DRIVE_TUNE_OP_SET_BRAKE_RAMP:   t->brake_ramp   = (uint8_t)val; break;
    case DRIVE_TUNE_OP_SET_REVERSE_RAMP: t->reverse_ramp = (uint8_t)val; break;
    case DRIVE_TUNE_OP_SET_CREEP_ENABLE: t->creep_enable = (uint8_t)val; break;
    case DRIVE_TUNE_OP_SET_CREEP_POWER:  t->creep_power  = (uint8_t)val; break;
    case DRIVE_TUNE_OP_SET_CREEP_DELAY:  t->creep_delay  = val;          break;
    default: break;
    }
}

/* Handle a SERVICE_CMD frame with byte 0 == SERVICE_ACTION_DRIVE_TUNING.
 * Always replies with one CAN_SendCommandAck(0x10, ...).               */
static void drvtune_handle_service_cmd(const uint8_t *payload, uint8_t len)
{
    if (len < 2) {
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
    uint8_t op = payload[1];

    /* QUERY is read-only — no safety gate, just a telemetry burst. */
    if (op == DRIVE_TUNE_OP_QUERY) {
        drvtune_burst_left = DRVTUNE_BURST_SWEEPS;
        drvtune_next_tx_ms = HAL_GetTick();
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }

    /* All other sub-opcodes require STANDBY. */
    if (!drvtune_safety_ok()) {
        CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
        return;
    }

    switch (op) {
    case DRIVE_TUNE_OP_SET_ACCEL_RAMP:
    case DRIVE_TUNE_OP_SET_BRAKE_RAMP:
    case DRIVE_TUNE_OP_SET_REVERSE_RAMP:
    case DRIVE_TUNE_OP_SET_CREEP_ENABLE:
    case DRIVE_TUNE_OP_SET_CREEP_POWER:
    case DRIVE_TUNE_OP_SET_CREEP_DELAY: {
        if (len < 4) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        uint16_t val = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
        drvtune_seed_pending_if_needed();
        DriveTuning_t cand = drvtune_pending;
        drvtune_apply_set(&cand, op, val);
        if (!Traction_ValidateDriveTuning(&cand)) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        drvtune_pending = cand;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    case DRIVE_TUNE_OP_SAVE: {
        if (!drvtune_pending_seeded) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        if (!Traction_ValidateDriveTuning(&drvtune_pending)) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        if (!DriveTuningStore_Save(&drvtune_pending)) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        /* Apply immediately; STANDBY (gated above) means no live demand. */
        (void)Traction_SetDriveTuning(&drvtune_pending);
        drvtune_pending_seeded = false;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    case DRIVE_TUNE_OP_RESET_DEFAULTS: {
        DriveTuning_t def;
        DriveTuningStore_GetDefaults(&def);
        if (!DriveTuningStore_Save(&def)) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        (void)Traction_SetDriveTuning(&def);
        drvtune_pending_seeded = false;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    default:
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
}

/* ==================================================================
 *  Battery-limit configuration (0xFB sub-protocol + 0x311 telemetry)
 *
 *  Lets the ESP32 Engineering menu view and tune the battery voltage
 *  thresholds (warning / derate / cutoff / recovery) and the optional
 *  voltage filter applied in safety_system.c.  SET_* sub-opcodes stage
 *  values in RAM ("pending"); nothing is applied or persisted until SAVE.
 *  This module touches the safety system only via the public
 *  Safety_*BatteryLimits() helpers and the flash store
 *  (battery_limits_store.c).  It NEVER changes the safety state machine.
 *
 *  Safety invariant (battlim_safety_ok()): Safety_GetState() == STANDBY.
 *  QUERY is exempt (read-only telemetry request).
 *
 *  0x311 telemetry burst (DLC 8): same field-stream layout as 0x310 but
 *  values are centivolts (V×100) or ms.  Frame layout:
 *    byte 0 : flags (bit0 stored-valid, bit1 pending-differs,
 *                    bit2 safety-ok, bit3 pending-valid)
 *    byte 1 : field id (BATT_LIM_FIELD_*)
 *    byte 2-3: active  value (uint16 LE, cV or ms)
 *    byte 4-5: pending value (uint16 LE, cV or ms)
 *    byte 6 : system state (Safety_GetState())
 *    byte 7 : total field count (BATT_LIM_FIELD_COUNT)
 * ================================================================== */

#define BATTLIM_BURST_SWEEPS     10U
#define BATTLIM_BURST_PERIOD_MS  100U

static bool            battlim_pending_seeded = false;
static BatteryLimits_t battlim_pending;

static uint8_t         battlim_burst_left = 0;
static uint32_t        battlim_next_tx_ms = 0;

static inline bool battlim_safety_ok(void)
{
    return BatteryLimitsStore_ServiceWriteAllowed();
}

static void battlim_seed_pending_if_needed(void)
{
    if (battlim_pending_seeded) return;
    Safety_GetBatteryLimits(&battlim_pending);
    battlim_pending_seeded = true;
}

static void battlim_field_values(uint8_t field, const BatteryLimits_t *act,
                                 const BatteryLimits_t *pend,
                                 uint16_t *act_out, uint16_t *pend_out)
{
    switch (field) {
    case BATT_LIM_FIELD_WARNING:
        *act_out = act->warning_cv;  *pend_out = pend->warning_cv;  break;
    case BATT_LIM_FIELD_LIMIT:
        *act_out = act->limit_cv;    *pend_out = pend->limit_cv;    break;
    case BATT_LIM_FIELD_CUTOFF:
        *act_out = act->cutoff_cv;   *pend_out = pend->cutoff_cv;   break;
    case BATT_LIM_FIELD_RECOVERY:
        *act_out = act->recovery_cv; *pend_out = pend->recovery_cv; break;
    case BATT_LIM_FIELD_FILTER:
        *act_out = act->filter_ms;   *pend_out = pend->filter_ms;   break;
    default:
        *act_out = 0; *pend_out = 0; break;
    }
}

static void battlim_send_field(uint8_t field)
{
    BatteryLimits_t act;
    Safety_GetBatteryLimits(&act);

    BatteryLimits_t pend = act;
    if (battlim_pending_seeded) pend = battlim_pending;

    uint16_t act_v = 0, pend_v = 0;
    battlim_field_values(field, &act, &pend, &act_v, &pend_v);

    bool valid_ok = Safety_ValidateBatteryLimits(&pend);

    uint8_t flags = 0;
    if (BatteryLimitsStore_IsValid()) flags |= 0x01U;
    if (battlim_pending_seeded && (act_v != pend_v)) flags |= 0x02U;
    if (battlim_safety_ok()) flags |= 0x04U;
    if (valid_ok) flags |= 0x08U;

    uint8_t payload[8];
    payload[0] = flags;
    payload[1] = field;
    payload[2] = (uint8_t)(act_v  & 0xFFU);
    payload[3] = (uint8_t)(act_v  >> 8);
    payload[4] = (uint8_t)(pend_v & 0xFFU);
    payload[5] = (uint8_t)(pend_v >> 8);
    payload[6] = (uint8_t)Safety_GetState();
    payload[7] = (uint8_t)BATT_LIM_FIELD_COUNT;

    (void)TransmitFrame(CAN_ID_DIAG_BATTERY_LIMITS, payload, sizeof(payload));
}

static void battlim_send_sweep(void)
{
    for (uint8_t f = 1U; f <= BATT_LIM_FIELD_COUNT; ++f) {
        battlim_send_field(f);
    }
}

void CAN_BatteryLimitsBurstUpdate(void)
{
    if (battlim_burst_left == 0U) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - battlim_next_tx_ms) < 0) return;
    battlim_send_sweep();
    battlim_burst_left--;
    battlim_next_tx_ms = now + BATTLIM_BURST_PERIOD_MS;
}

static void battlim_apply_set(BatteryLimits_t *b, uint8_t op, uint16_t val)
{
    switch (op) {
    case BATT_LIM_OP_SET_WARNING:  b->warning_cv  = val; break;
    case BATT_LIM_OP_SET_LIMIT:    b->limit_cv    = val; break;
    case BATT_LIM_OP_SET_CUTOFF:   b->cutoff_cv   = val; break;
    case BATT_LIM_OP_SET_RECOVERY: b->recovery_cv = val; break;
    case BATT_LIM_OP_SET_FILTER:   b->filter_ms   = val; break;
    default: break;
    }
}

/* Handle a SERVICE_CMD frame with byte 0 == SERVICE_ACTION_BATTERY_LIMITS.
 * Always replies with one CAN_SendCommandAck(0x10, ...).               */
static void battlim_handle_service_cmd(const uint8_t *payload, uint8_t len)
{
    if (len < 2) {
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
    uint8_t op = payload[1];

    if (op == BATT_LIM_OP_QUERY) {
        battlim_burst_left = BATTLIM_BURST_SWEEPS;
        battlim_next_tx_ms = HAL_GetTick();
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }

    if (!battlim_safety_ok()) {
        CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
        return;
    }

    switch (op) {
    case BATT_LIM_OP_SET_WARNING:
    case BATT_LIM_OP_SET_LIMIT:
    case BATT_LIM_OP_SET_CUTOFF:
    case BATT_LIM_OP_SET_RECOVERY:
    case BATT_LIM_OP_SET_FILTER: {
        if (len < 4) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        uint16_t val = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
        battlim_seed_pending_if_needed();
        BatteryLimits_t cand = battlim_pending;
        battlim_apply_set(&cand, op, val);
        if (!Safety_ValidateBatteryLimits(&cand)) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        battlim_pending = cand;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    case BATT_LIM_OP_SAVE: {
        if (!battlim_pending_seeded) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        if (!Safety_ValidateBatteryLimits(&battlim_pending)) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        if (!BatteryLimitsStore_Save(&battlim_pending)) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        (void)Safety_SetBatteryLimits(&battlim_pending);
        battlim_pending_seeded = false;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    case BATT_LIM_OP_RESET_DEFAULTS: {
        BatteryLimits_t def;
        BatteryLimitsStore_GetDefaults(&def);
        if (!BatteryLimitsStore_Save(&def)) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        (void)Safety_SetBatteryLimits(&def);
        battlim_pending_seeded = false;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    default:
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
}

/* ==================================================================
 *  Steering PB5 + encoder-Z dual center reference (0xF8 + 0x30E)
 *
 *  Diagnostic + calibration sub-protocol.  PB5 (LJ12A3, EXTI5) remains
 *  the PRIMARY physical/safety center reference; the encoder Z (index)
 *  pulse on PB4 is a SECONDARY precision reference that can NEVER center
 *  on its own.  A Z pulse without PB5 confirmation is NOT a center.
 *
 *  0x30E telemetry frame (DLC 8, little-endian) — diagnostic only:
 *    byte 0 : flags
 *             bit 0-2: combined status (SteeringZStatus_t 0..4)
 *                      0 NOT_CALIBRATED, 1 OK, 2 NOT_SEEN,
 *                      3 OUT_OF_WINDOW, 4 MECHANICAL_OFFSET
 *             bit 3  : PB5 center detected right now (live)
 *             bit 4  : z_center_valid (calibration validated)
 *             bit 5  : encoder Z slip latched
 *             bits 6-7: reserved (0)
 *    byte 1 : Z pulse count since boot (saturated 0..255)
 *    byte 2-3: enc_z_last_pos (int16 LE, clamped) — TIM2 count at last Z
 *    byte 4-5: z_center_offset_counts (int16 LE) — Z↔center offset
 *    byte 6 : enc_z_last_error (int8, clamped) — last inter-pulse error
 *    byte 7 : z_center_tolerance (uint8, counts) — window used
 * ================================================================== */
#define STEERZ_BURST_FRAMES     10U    /* 10 × 100 ms = 1 s             */
#define STEERZ_BURST_PERIOD_MS  100U

static uint8_t  steerz_burst_left = 0;
static uint32_t steerz_next_tx_ms = 0;

static int16_t steerz_clamp_i16(int32_t v)
{
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

static void steerz_send_status(void)
{
    SteeringZStatus_t st = SteeringZ_GetStatus();
    bool pb5_live = SteeringCenter_Detected();

    uint8_t flags = (uint8_t)(st & 0x07U);
    if (pb5_live)            flags |= 0x08U;
    if (SteeringZ_IsValid()) flags |= 0x10U;
    if (Encoder_Z_HasSlipped()) flags |= 0x20U;

    uint32_t pulses = Encoder_Z_GetPulseCount();
    uint8_t  pulses8 = (pulses > 255U) ? 255U : (uint8_t)pulses;

    int16_t last_pos = steerz_clamp_i16(Encoder_Z_GetLastPosition());
    int16_t offset   = steerz_clamp_i16(SteeringZ_GetOffset());

    int32_t err = Encoder_Z_GetLastError();
    if (err >  127) err =  127;
    if (err < -128) err = -128;

    int32_t tol = SteeringCal_GetStoredZTolerance();
    if (tol < 0)   tol = 0;
    if (tol > 255) tol = 255;

    uint8_t payload[8];
    payload[0] = flags;
    payload[1] = pulses8;
    payload[2] = (uint8_t)(last_pos & 0xFFU);
    payload[3] = (uint8_t)((last_pos >> 8) & 0xFFU);
    payload[4] = (uint8_t)(offset & 0xFFU);
    payload[5] = (uint8_t)((offset >> 8) & 0xFFU);
    payload[6] = (uint8_t)((int8_t)err);
    payload[7] = (uint8_t)tol;

    (void)TransmitFrame(CAN_ID_DIAG_STEERING_Z, payload, sizeof(payload));
}

void CAN_SteeringZBurstUpdate(void)
{
    if (steerz_burst_left == 0U) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - steerz_next_tx_ms) < 0) return;
    steerz_send_status();
    steerz_burst_left--;
    steerz_next_tx_ms = now + STEERZ_BURST_PERIOD_MS;
}

/* Handle a SERVICE_CMD frame with byte 0 == SERVICE_ACTION_STEERING_Z.
 * Always replies with one CAN_SendCommandAck(0x10, ...).
 *
 * Safety: CALIBRATE recomputes the Z↔center offset and is only accepted
 * when PB5 currently confirms the physical center AND the vehicle is in
 * BOOT/STANDBY.  Z can never (re)center on its own — the offset is taken
 * relative to the PB5-confirmed center (current TIM2 count).            */
static void steerz_handle_service_cmd(const uint8_t *payload, uint8_t len)
{
    if (len < 2) {
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
    uint8_t op = payload[1];

    /* QUERY is read-only — no safety gate, just a telemetry burst. */
    if (op == STEER_Z_OP_QUERY) {
        steerz_burst_left = STEERZ_BURST_FRAMES;
        steerz_next_tx_ms = HAL_GetTick();
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }

    /* CALIBRATE / CLEAR require BOOT or STANDBY (no live driving). */
    SystemState_t state = Safety_GetState();
    if (state != SYS_STATE_BOOT && state != SYS_STATE_STANDBY) {
        CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
        return;
    }

    switch (op) {
    case STEER_Z_OP_CALIBRATE: {
        /* PB5 must physically confirm center — never trust Z alone. */
        if (HAL_GPIO_ReadPin(GPIOB, PIN_STEER_CENTER) != GPIO_PIN_RESET) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        int32_t center = Encoder_GetRawCount();
        bool    z_seen = (Encoder_Z_GetPulseCount() > 0U);
        SteeringZ_OnCenterConfirmed(center,
                                    Encoder_Z_GetLastPosition(),
                                    z_seen);
        bool ok = SteeringCal_SaveWithZ(center,
                                        SteeringZ_GetOffset(),
                                        SteeringZ_IsValid(),
                                        (int32_t)STEERING_Z_WINDOW_COUNTS);
        /* Refresh telemetry so the HMI sees the result immediately. */
        steerz_burst_left = STEERZ_BURST_FRAMES;
        steerz_next_tx_ms = HAL_GetTick();
        CAN_SendCommandAck(0x10, ok ? ACK_OK : ACK_REJECTED);
        return;
    }
    case STEER_Z_OP_CLEAR: {
        SteeringZ_ClearCalibration();
        /* Persist the PB5 center with the Z reference invalidated. */
        bool ok = SteeringCal_SaveWithZ(SteeringCal_GetStoredCenter(),
                                        0, false, 0);
        steerz_burst_left = STEERZ_BURST_FRAMES;
        steerz_next_tx_ms = HAL_GetTick();
        CAN_SendCommandAck(0x10, ok ? ACK_OK : ACK_REJECTED);
        return;
    }
    default:
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
}

/* ==================================================================
 *  EPS runtime parameter tuning (0xF9 + 0x30F telemetry)
 *
 *  Provides Engineering Menu access to all eps_params_t fields.
 *  SET_PARAM applies live but is gated by EPS_Params_SetAllowed()
 *  (Item A): STANDBY + PARK/NEUTRAL + stationary + ~zero traction
 *  demand + no dangerous fault.  Blocked SETs reply ACK_BLOCKED_BY_SAFETY.
 *  SAVE and RESET require SYS_STATE_STANDBY for flash safety.
 *
 *  0x30F frame layout (DLC 8):
 *    byte 0: (kind << 4) | flags
 *            flags bit0 = flash_valid, bit1 = sys_in_standby
 *    kind 0: assist_strength×1000, center_strength×1000, damping×1000 (int16 LE)
 *    kind 1: friction_comp×1000, coast_band_pct×100, min_drive_pct×100 (int16 LE)
 *    kind 2: assist_vs_speed×10, return_vs_speed×10, deadband_deg×100 (int16 LE)
 *    kind 3: max_pwm_pct×100, slew_rate_pct×100, center_offset_deg×100 (int16 LE)
 *    kind 4: enc_raw(int16), angle_raw(×10, int16), motor_effort(×10, int16),
 *            steer_state(byte: 0=uncal,1=cal,2=enc_fault)
 *    bytes 1-2: param_a int16 LE
 *    bytes 3-4: param_b int16 LE
 *    bytes 5-6: param_c int16 LE
 *    byte 7: steer_state (kind 4) or reserved (kinds 0-3)
 * ================================================================== */
#define EPS_BURST_FRAMES     10U    /* 10 × 100 ms = 1 s     */
#define EPS_BURST_PERIOD_MS  100U
#define EPS_PARAM_KINDS      5U     /* 5 frame kinds per tick */

static uint8_t  eps_burst_left  = 0;
static uint32_t eps_next_tx_ms  = 0;

/* Clamp a float to int16 range */
static int16_t eps_clamp_i16f(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

/* Emit one complete set of 5 × 0x30F frames covering all param kinds. */
static void eps_send_all_kinds(void)
{
    const eps_params_t *p    = EPS_Params_Get();
    SystemState_t      state = Safety_GetState();

    uint8_t flags = 0;
    if (EPS_Params_IsFlashValid())         flags |= 0x01U;
    if (state == SYS_STATE_STANDBY)        flags |= 0x02U;

    uint8_t payload[8];

    /* kind 0: assist_strength×1000, center_strength×1000, damping×1000 */
    {
        int16_t a = eps_clamp_i16f(p->assist_strength * 1000.0f);
        int16_t b = eps_clamp_i16f(p->center_strength * 1000.0f);
        int16_t c = eps_clamp_i16f(p->damping         * 1000.0f);
        payload[0] = (uint8_t)(flags & 0x0FU);
        payload[1] = (uint8_t)(a & 0xFF); payload[2] = (uint8_t)(a >> 8);
        payload[3] = (uint8_t)(b & 0xFF); payload[4] = (uint8_t)(b >> 8);
        payload[5] = (uint8_t)(c & 0xFF); payload[6] = (uint8_t)(c >> 8);
        payload[7] = 0;
        (void)TransmitFrame(CAN_ID_DIAG_EPS_PARAMS, payload, sizeof(payload));
    }
    /* kind 1: friction_comp×1000, coast_band_pct×100, min_drive_pct×100 */
    {
        int16_t a = eps_clamp_i16f(p->friction_comp  * 1000.0f);
        int16_t b = eps_clamp_i16f(p->coast_band_pct *  100.0f);
        int16_t c = eps_clamp_i16f(p->min_drive_pct  *  100.0f);
        payload[0] = (uint8_t)((1U << 4) | (flags & 0x0FU));
        payload[1] = (uint8_t)(a & 0xFF); payload[2] = (uint8_t)(a >> 8);
        payload[3] = (uint8_t)(b & 0xFF); payload[4] = (uint8_t)(b >> 8);
        payload[5] = (uint8_t)(c & 0xFF); payload[6] = (uint8_t)(c >> 8);
        payload[7] = 0;
        (void)TransmitFrame(CAN_ID_DIAG_EPS_PARAMS, payload, sizeof(payload));
    }
    /* kind 2: assist_vs_speed×10, return_vs_speed×10, deadband_deg×100 */
    {
        int16_t a = eps_clamp_i16f(p->assist_vs_speed *  10.0f);
        int16_t b = eps_clamp_i16f(p->return_vs_speed *  10.0f);
        int16_t c = eps_clamp_i16f(p->deadband_deg    * 100.0f);
        payload[0] = (uint8_t)((2U << 4) | (flags & 0x0FU));
        payload[1] = (uint8_t)(a & 0xFF); payload[2] = (uint8_t)(a >> 8);
        payload[3] = (uint8_t)(b & 0xFF); payload[4] = (uint8_t)(b >> 8);
        payload[5] = (uint8_t)(c & 0xFF); payload[6] = (uint8_t)(c >> 8);
        payload[7] = 0;
        (void)TransmitFrame(CAN_ID_DIAG_EPS_PARAMS, payload, sizeof(payload));
    }
    /* kind 3: max_pwm_pct×100, slew_rate_pct×100, center_offset_deg×100 */
    {
        int16_t a = eps_clamp_i16f(p->max_pwm_pct        * 100.0f);
        int16_t b = eps_clamp_i16f(p->slew_rate_pct      * 100.0f);
        int16_t c = eps_clamp_i16f(p->center_offset_deg  * 100.0f);
        payload[0] = (uint8_t)((3U << 4) | (flags & 0x0FU));
        payload[1] = (uint8_t)(a & 0xFF); payload[2] = (uint8_t)(a >> 8);
        payload[3] = (uint8_t)(b & 0xFF); payload[4] = (uint8_t)(b >> 8);
        payload[5] = (uint8_t)(c & 0xFF); payload[6] = (uint8_t)(c >> 8);
        payload[7] = 0;
        (void)TransmitFrame(CAN_ID_DIAG_EPS_PARAMS, payload, sizeof(payload));
    }
    /* kind 4: enc_raw(int16), angle×10(int16), motor_effort×10(int16),
     *         steer_state (0=uncal, 1=cal, 2=enc_fault)             */
    {
        int32_t enc32 = Steering_GetEncoderRaw();
        int16_t enc   = (enc32 >  32767) ? (int16_t) 32767 :
                        (enc32 < -32768) ? (int16_t)-32768 : (int16_t)enc32;
        float   ang   = Steering_GetCurrentAngle();
        int16_t ang16 = eps_clamp_i16f(ang * 10.0f);
        int16_t eff16 = eps_clamp_i16f(Steering_GetMotorEffortPct() * 10.0f);
        uint8_t stst  = Steering_IsCalibrated() ? (Encoder_HasFault() ? 2U : 1U) : 0U;
        payload[0] = (uint8_t)((4U << 4) | (flags & 0x0FU));
        payload[1] = (uint8_t)(enc  & 0xFF); payload[2] = (uint8_t)(enc  >> 8);
        payload[3] = (uint8_t)(ang16 & 0xFF); payload[4] = (uint8_t)(ang16 >> 8);
        payload[5] = (uint8_t)(eff16 & 0xFF); payload[6] = (uint8_t)(eff16 >> 8);
        payload[7] = stst;
        (void)TransmitFrame(CAN_ID_DIAG_EPS_PARAMS, payload, sizeof(payload));
    }
}

void CAN_EPS_ParamsBurstUpdate(void)
{
    if (eps_burst_left == 0U) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - eps_next_tx_ms) < 0) return;
    eps_send_all_kinds();
    eps_burst_left--;
    eps_next_tx_ms = now + EPS_BURST_PERIOD_MS;
}

/* Handle a SERVICE_CMD frame with byte 0 == SERVICE_ACTION_EPS_PARAMS. */
static void eps_handle_service_cmd(const uint8_t *payload, uint8_t len)
{
    if (len < 2) {
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
    uint8_t op = payload[1];

    /* QUERY: read-only, no state gate */
    if (op == EPS_PARAM_OP_QUERY) {
        eps_burst_left = EPS_BURST_FRAMES;
        eps_next_tx_ms = HAL_GetTick();
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }

    /* SET_PARAM: apply live, but only when the SET_PARAM safety gate
     * (Item A) confirms the vehicle is safe & stationary.  A live SET
     * alters steering-assist torque on the next control cycle, so the
     * numeric range check in EPS_Params_Set() alone is not sufficient. */
    if (op == EPS_PARAM_OP_SET_PARAM) {
        if (len < 7) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }

        /* Assemble the live safety/motion snapshot and gate the SET. */
        float ws_fl = fabsf(Wheel_GetSpeed_FL());
        float ws_fr = fabsf(Wheel_GetSpeed_FR());
        float ws_rl = fabsf(Wheel_GetSpeed_RL());
        float ws_rr = fabsf(Wheel_GetSpeed_RR());
        float ws_max = ws_fl;
        if (ws_fr > ws_max) ws_max = ws_fr;
        if (ws_rl > ws_max) ws_max = ws_rl;
        if (ws_rr > ws_max) ws_max = ws_rr;

        const TractionState_t *ts = Traction_GetState();
        eps_setparam_gate_t gate = {
            .state               = (uint8_t)Safety_GetState(),
            .state_standby       = (uint8_t)SYS_STATE_STANDBY,
            .gear                = (uint8_t)Traction_GetGear(),
            .gear_park           = (uint8_t)GEAR_PARK,
            .gear_neutral        = (uint8_t)GEAR_NEUTRAL,
            .max_wheel_speed_kmh = ws_max,
            .demand_pct          = ts ? ts->demandPct : 0.0f,
            .dangerous_fault     = Safety_IsError(),
        };
        if (!EPS_Params_SetAllowed(&gate)) {
            CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
            return;
        }

        uint8_t param_id = payload[2];
        /* Deserialise float from bytes 3-6 (little-endian IEEE 754) */
        float value;
        uint8_t fb[4] = { payload[3], payload[4], payload[5], payload[6] };
        __builtin_memcpy(&value, fb, sizeof(float));
        if (EPS_Params_Set((eps_param_id_t)param_id, value)) {
            /* Trigger a brief burst so the HMI sees the updated value */
            eps_burst_left = 2U;
            eps_next_tx_ms = HAL_GetTick();
            CAN_SendCommandAck(0x10, ACK_OK);
        } else {
            CAN_SendCommandAck(0x10, ACK_INVALID);
        }
        return;
    }

    /* SAVE and RESET require STANDBY */
    SystemState_t state = Safety_GetState();
    if (state != SYS_STATE_STANDBY) {
        CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
        return;
    }

    switch (op) {
    case EPS_PARAM_OP_SAVE:
        if (EPS_Params_Save()) {
            eps_burst_left = EPS_BURST_FRAMES;
            eps_next_tx_ms = HAL_GetTick();
            CAN_SendCommandAck(0x10, ACK_OK);
        } else {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
        }
        break;
    case EPS_PARAM_OP_RESET:
        EPS_Params_ResetDefaults();
        eps_burst_left = EPS_BURST_FRAMES;
        eps_next_tx_ms = HAL_GetTick();
        CAN_SendCommandAck(0x10, ACK_OK);
        break;
    default:
        CAN_SendCommandAck(0x10, ACK_INVALID);
        break;
    }
}

/* ==================================================================
 *  SERVICE_DIAG self-test session (0xFC sub-protocol + 0x31B/0x31C telemetry)
 *  Bloque A, PR #445 Hito 1.
 *
 *  Wires the pure ServiceDiagSession FSM (service_diag_session.c) to the
 *  live safety/sensor state and to CAN, exactly like every other guided
 *  session in this file: this module owns exactly ONE instance, populates
 *  its per-tick ServiceDiagConds snapshot from real getters, drives the
 *  FSM, and transports its state/results on CAN.
 *
 *  IMPORTANT — Hito 1 does NOT drive any actuator.  active_channel /
 *  active_pwm_pct / active_direction are read here ONLY to populate
 *  telemetry (0x31B/0x31C); no PWM register or motor driver is ever written
 *  from this module.  Applying the commanded PWM to the real motor
 *  (BTS7960 etc.) is Bloque B — steering_service_store.h already documents
 *  service_diag_session.c as its "first production consumer... when a value
 *  is about to be applied to the motor", which has not happened yet.
 *  Consequently the current/pulses reported on 0x31C reflect the REAL,
 *  pre-existing sensors, which will read ~0 during a Hito-1 "step" because
 *  nothing is actually being driven; Hito 3 (Bloque B) completes the wiring.
 *
 *  The FSM's OWN guard logic (entry gates, watchdog, timeouts, dead-time,
 *  grounded-wheel-pulse, test-overcurrent, battery/gear/CAN/state aborts) is
 *  fully exercised over real CAN in this milestone even though no PWM is
 *  applied — a deliberate "dry run" of the safety envelope ahead of Bloque B.
 * ================================================================== */

/* 0x31B cadence while the session is active (recommended by spec: 10 Hz). */
#define SVCDIAG_SESS_PERIOD_MS  100U

/* Cross-referenced against the two existing sources of truth so the 60%
 * test-overcurrent threshold (SVCDIAG_TEST_OVERCURRENT_FACTOR) never
 * silently drifts from the REAL per-channel limits:
 *   - MAX_CURRENT_A (safety_system.c, static #define)            = 25.0 A
 *   - STEERING_ACTIVE_OVERCURRENT_MA (steering_supervisor.h) / 1000 = 25.0 A
 * Both currently agree, so a single constant covers every channel rather
 * than pulling in steering_supervisor.h for one float.                    */
#define SVCDIAG_TEST_CURRENT_NORMAL_LIMIT_A  25.0f

/* DTC pseudo-codes for the session ENTER/EXIT log — see the
 * error_log_entry_t field comments in error_log.h.  Deliberately outside
 * the Safety_Error_t range (0-16) so they can never be confused with a real
 * safety fault; this reuses the EXISTING ErrorLog_Record() ring buffer
 * rather than inventing a new log.                                        */
#define SVCDIAG_DTC_CODE_ENTER   0x40U
#define SVCDIAG_DTC_CODE_EXIT    0x41U
#define SVCDIAG_DTC_SUBSYSTEM    4U   /* SERVICE_DIAG — see error_log.h      */

static ServiceDiagSession svcdiag_session;
static bool               svcdiag_ready = false;

/* 1 s debounce for wheels_stationary_1s: timestamp of the last cycle any
 * wheel read >= SVCDIAG_WHEEL_STATIONARY_KMH.  Exactly like pedalcal's
 * wheels_moving, but inverted and timed.  Initialised to "now" at lazy-init
 * so a fresh boot needs a full confirmed second of stillness before Begin()
 * can ever succeed.                                                       */
static uint32_t svcdiag_last_moving_ms = 0;

/* 0x31B periodic-cadence bookkeeping (mirrors pedalcal_sess_next_tx_ms). */
static uint32_t svcdiag_sess_next_tx_ms = 0;

/* Last-observed state/active, tracked across EVERY mutating call (Begin/
 * RequestStep/Abort/Update) — NOT just within a single tick — so a step
 * closed or a session ended by an operator CAN command (which mutates the
 * FSM directly, outside CAN_ServiceDiagTick()) is still detected exactly
 * once, however it happened. */
static ServiceDiagState_t svcdiag_last_state  = SVCDIAG_STATE_IDLE;
static bool               svcdiag_last_active = false;

/* Live sample of the channel actually STEPPING, cached every cycle so the
 * 0x31C result reflects the last real reading taken WHILE the step was
 * still live — by the time a transition out of STEPPING is observed,
 * ServiceDiagSession_Active*() already report NONE/0. */
static ServiceDiagChannel_t svcdiag_last_step_channel = SVCDIAG_CH_NONE;
static uint8_t              svcdiag_last_step_pwm_pct = 0U;
static uint16_t             svcdiag_last_current_ma   = 0U;
static uint16_t             svcdiag_last_pulses_ps     = 0U;

/* Hito 2 (PR #445) wheel-equality self-test object.  Declared here (rather
 * than only where the full CAN wiring is added) so svcdiag_build_conds()'s
 * grounded-wheel-pulse exemption below can see which wheel(s) it is
 * actuating — see WheelEqTest_ActiveWheelMask().  Zero-initialised by the
 * compiler (state==WHEQ_STATE_IDLE==0), which is already the safe default;
 * CAN_WheelEqualityInit() (full wiring) still calls WheelEqTest_Init()
 * explicitly for defensive clarity, matching every sibling module.        */
static WheelEqTest wheeleq_test;

static void svcdiag_lazy_init(void)
{
    if (svcdiag_ready) return;
    ServiceDiagSession_Init(&svcdiag_session, NULL);
    uint32_t now = HAL_GetTick();
    svcdiag_last_moving_ms    = now;
    svcdiag_sess_next_tx_ms   = now;
    svcdiag_last_state        = ServiceDiagSession_State(&svcdiag_session);
    svcdiag_last_active       = false;
    svcdiag_last_step_channel = SVCDIAG_CH_NONE;
    svcdiag_last_step_pwm_pct = 0U;
    svcdiag_last_current_ma   = 0U;
    svcdiag_last_pulses_ps    = 0U;
    svcdiag_ready = true;
}

/* Explicit one-time init, called from main.c's startup section —
 * matches every sibling module's convention (SteeringServiceStore_Init(),
 * ErrorLog_Init(), etc.).  Every other entry point (the CAN command handler,
 * the periodic tick) ALSO calls the same guarded init defensively, so the
 * module is safe regardless of call order; calling it twice is a no-op. */
void CAN_ServiceDiagInit(void)
{
    svcdiag_lazy_init();
}

/* Populate the per-tick condition snapshot from live safety/sensor state.
 * confirm_token_ok is always left false here; only the START handler
 * overrides it from the received payload right before calling Begin(). */
static void svcdiag_build_conds(ServiceDiagConds *c)
{
    memset(c, 0, sizeof(*c));
    const SystemState_t  st   = Safety_GetState();
    const GearPosition_t gear = Traction_GetGear();
    uint32_t now = HAL_GetTick();

    float speeds[4];
    speeds[0] = sanitize_float(Wheel_GetSpeed_FL(), SANITIZE_SPEED_DEFAULT);
    speeds[1] = sanitize_float(Wheel_GetSpeed_FR(), SANITIZE_SPEED_DEFAULT);
    speeds[2] = sanitize_float(Wheel_GetSpeed_RL(), SANITIZE_SPEED_DEFAULT);
    speeds[3] = sanitize_float(Wheel_GetSpeed_RR(), SANITIZE_SPEED_DEFAULT);
    bool any_moving = (fabsf(speeds[0]) >= SVCDIAG_WHEEL_STATIONARY_KMH) ||
                      (fabsf(speeds[1]) >= SVCDIAG_WHEEL_STATIONARY_KMH) ||
                      (fabsf(speeds[2]) >= SVCDIAG_WHEEL_STATIONARY_KMH) ||
                      (fabsf(speeds[3]) >= SVCDIAG_WHEEL_STATIONARY_KMH);
    if (any_moving) svcdiag_last_moving_ms = now;

    BatteryLimits_t lim;
    Safety_GetBatteryLimits(&lim);
    float batt_v = sanitize_float(Voltage_GetBus(INA226_CHANNEL_BATTERY), 0.0f);

    c->now_ms                = now;
    c->boot_state            = (st == SYS_STATE_BOOT);
    c->gear_park_or_neutral  = (gear == GEAR_PARK || gear == GEAR_NEUTRAL);
    c->wheels_stationary_1s  = (uint32_t)(now - svcdiag_last_moving_ms) >= SVCDIAG_STATIONARY_MIN_MS;
    c->confirm_token_ok      = false;
    c->battery_above_cutoff  = batt_v > (lim.cutoff_cv  / 100.0f);
    c->system_state_raw      = (uint8_t)st;
    c->battery_above_warning = batt_v > (lim.warning_cv / 100.0f);
    c->can_rx_age_ms         = Safety_GetCanRxAgeMs();

    /* Grounded-wheel-pulse: any wheel OTHER than the active channel showing
     * movement means the vehicle is not genuinely suspended (or something
     * else is spinning it).  Checked against ALL 4 wheels when the active
     * channel is steering or none (nothing to exempt).
     *
     * Hito 2 (PR #445) extension: the wheel-equality self-test drives its
     * own wheel(s) through this SAME suspended-vehicle session, using a
     * separate FSM (WheelEqTest) with its own wheel_mask — Hito 1's
     * ActiveChannel() alone never sees it (stays SVCDIAG_CH_NONE), so
     * without this the first wheel Hito 2 spins would immediately read
     * back as a "grounded wheel pulse" and abort the session.  The
     * exemption below tracks ONLY the wheel(s) WheelEqTest is actually
     * commanding THIS cycle (never a global permission): a single bit in
     * Fase 1, all four only while Fase 2 is actually running, and zero the
     * instant the test is idle/aborted/blocked/between steps/done — see
     * WheelEqTest_ActiveWheelMask().  Outside any wheel-equality session
     * this mask is always 0, so the guard behaves exactly as before.      */
    ServiceDiagChannel_t active = ServiceDiagSession_ActiveChannel(&svcdiag_session);
    uint8_t wheeleq_exempt_mask = WheelEqTest_ActiveWheelMask(&wheeleq_test);
    bool grounded = false;
    for (uint8_t i = 0; i < 4U; i++) {
        if ((ServiceDiagChannel_t)i == active) continue;
        if ((wheeleq_exempt_mask & (1U << i)) != 0U) continue;
        if (fabsf(speeds[i]) >= SVCDIAG_WHEEL_STATIONARY_KMH) { grounded = true; break; }
    }
    c->grounded_wheel_pulse = grounded;

    bool overcurrent = false;
    if (active <= SVCDIAG_CH_STEERING) {
        uint8_t ina_idx = (active == SVCDIAG_CH_STEERING)
                              ? (uint8_t)INA226_CHANNEL_STEER : (uint8_t)active;
        float amps = Current_GetAmps(ina_idx);
        float thresh = SVCDIAG_TEST_CURRENT_NORMAL_LIMIT_A * SVCDIAG_TEST_OVERCURRENT_FACTOR;
        /* NaN/Inf hardening mirrors Safety_CheckCurrent(): treat an invalid
         * reading as an overcurrent (safe side), never as a silent 0 A.    */
        overcurrent = isnan(amps) || isinf(amps) || fabsf(amps) > thresh;

        /* Cache the live sample for the eventual 0x31C result (see the
         * static block comment above for why this must happen here).     */
        svcdiag_last_step_channel = active;
        svcdiag_last_step_pwm_pct = ServiceDiagSession_ActivePwmPct(&svcdiag_session);
        float safe_amps = (isnan(amps) || isinf(amps)) ? 0.0f : fabsf(amps);
        svcdiag_last_current_ma = ServiceDiagFrame_SatU16((int32_t)(safe_amps * 1000.0f));

        float pulses_ps = 0.0f;
        if (active <= SVCDIAG_CH_RR) {
            float v_ms = fabsf(speeds[(uint8_t)active]) * 1000.0f / 3600.0f;
            pulses_ps = (v_ms / WHEEL_CIRCUMF_M) * (float)WHEEL_PULSES_REV;
        }
        svcdiag_last_pulses_ps = ServiceDiagFrame_SatU16((int32_t)(pulses_ps + 0.5f));
    }
    c->test_overcurrent = overcurrent;

    /* Steering PWM ceiling as a percent of full scale.  V2 audit requires
     * every production consumer applying this value to the motor to use
     * the CLAMPED accessor (see steering_service_store.h) — Hito 1 never
     * applies it to the motor, but consumes the same clamped source so the
     * telemetry ceiling always matches what Bloque B will actually use.   */
    SteeringServiceParams_t ssp;
    SteeringServiceStore_GetEffectiveClamped(&ssp);
    uint32_t pct = ((uint32_t)ssp.search_pwm_counts * 100U) / HOMING_PWM_COUNTS;
    c->steering_pwm_ceiling_pct = (pct > 100U) ? 100U : (uint8_t)pct;
}

/* Publish the 0x31B session-status frame (current FSM state/progress). */
static void svcdiag_send_session_status(void)
{
    uint32_t now = HAL_GetTick();
    ServiceDiagSessionFrame_t f;
    f.state            = (uint8_t)ServiceDiagSession_State(&svcdiag_session);
    f.step_index        = ServiceDiagSession_StepIndex(&svcdiag_session);
    f.active_channel    = (uint8_t)ServiceDiagSession_ActiveChannel(&svcdiag_session);
    f.progress_pct      = ServiceDiagSession_ProgressPct(&svcdiag_session, now);
    f.reason            = (uint8_t)ServiceDiagSession_Reason(&svcdiag_session);
    f.origin_state_raw  = ServiceDiagSession_OriginState(&svcdiag_session);
    f.elapsed_sec        = ServiceDiagSession_ElapsedSec(&svcdiag_session, now);
    f.active_pwm_pct     = ServiceDiagSession_ActivePwmPct(&svcdiag_session);

    uint8_t data[SERVICE_DIAG_SESSION_FRAME_DLC];
    ServiceDiagSessionFrame_Pack(&f, data);
    (void)TransmitFrame(CAN_ID_DIAG_SERVICE_SESSION, data, SERVICE_DIAG_SESSION_FRAME_DLC);
}

/* Publish the 0x31C per-step test-result frame from the cached last-known
 * live sample of the step that just closed (see svcdiag_build_conds()). */
static void svcdiag_send_test_result(void)
{
    ServiceDiagTestResultFrame_t f;
    f.channel        = (uint8_t)svcdiag_last_step_channel;
    f.pwm_step_pct   = svcdiag_last_step_pwm_pct;
    f.current_ma     = svcdiag_last_current_ma;
    f.pulses_per_sec = svcdiag_last_pulses_ps;
    f.verdict        = (uint8_t)ServiceDiagSession_StepVerdict(&svcdiag_session);
    f.step_index     = ServiceDiagSession_StepIndex(&svcdiag_session);

    uint8_t data[SERVICE_DIAG_TEST_RESULT_FRAME_DLC];
    ServiceDiagTestResultFrame_Pack(&f, data);
    (void)TransmitFrame(CAN_ID_DIAG_TEST_RESULT, data, SERVICE_DIAG_TEST_RESULT_FRAME_DLC);
}

/* Single choke point: call after ANY operation that may change the FSM's
 * state (Begin / RequestStep / Abort / Update).  Detects a genuine
 * STEPPING-closed or active-to-inactive transition exactly once, however it
 * happened (operator CAN command OR the periodic tick's Update()), and
 * drives the resulting side effects:
 *   - 0x31C test-result the instant a step closes.
 *   - DTC exit log the instant the session goes inactive, carrying
 *     the origin state and the concrete exit reason (operator/abort/
 *     timeout) — see error_log.h.
 * Returns true if the FSM state changed, so callers can decide whether an
 * immediate 0x31B is warranted versus the periodic cadence.              */
static bool svcdiag_process_transition(void)
{
    ServiceDiagState_t new_state  = ServiceDiagSession_State(&svcdiag_session);
    bool               new_active = ServiceDiagSession_Active(&svcdiag_session);
    bool changed = (new_state != svcdiag_last_state);

    if (svcdiag_last_state == SVCDIAG_STATE_STEPPING && new_state != SVCDIAG_STATE_STEPPING) {
        svcdiag_send_test_result();
    }
    if (svcdiag_last_active && !new_active) {
        ErrorLog_Record(SVCDIAG_DTC_CODE_EXIT, SVCDIAG_DTC_SUBSYSTEM,
                        ServiceDiagSession_OriginState(&svcdiag_session),
                        (uint8_t)ServiceDiagSession_Reason(&svcdiag_session));
    }
    svcdiag_last_state  = new_state;
    svcdiag_last_active = new_active;
    return changed;
}

/* Handle a SERVICE_CMD frame with byte 0 == SERVICE_ACTION_SELF_TEST.
 * Byte 1 = sub-opcode (SVCDIAG_OP_*).  Bloque A entry preconditions are
 * validated INSIDE ServiceDiagSession_Begin() (entry_block()) before START
 * is ever accepted; the concrete reject reason travels on the 0x31B
 * `reason` byte (the ACK itself only carries 4 coarse values).           */
static void svcdiag_handle_service_cmd(const uint8_t *payload, uint8_t len)
{
    if (len < 2) {
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
    svcdiag_lazy_init();
    uint8_t op = payload[1];

    switch (op) {
    case SVCDIAG_OP_QUERY:
        /* Read-only, always-safe: a single immediate 0x31B reply.  Does
         * NOT start the periodic 10 Hz stream, which stays silent whenever
         * the session is not active (svcdiag_sess_next_tx_ms untouched). */
        svcdiag_send_session_status();
        CAN_SendCommandAck(0x10, ACK_OK);
        return;

    case SVCDIAG_OP_START: {
        ServiceDiagConds c;
        svcdiag_build_conds(&c);
        c.confirm_token_ok = (len >= 3) && (payload[2] == SVCDIAG_CONFIRM_TOKEN);
        bool ok = ServiceDiagSession_Begin(&svcdiag_session, &c);
        svcdiag_process_transition();
        svcdiag_send_session_status();
        if (ok) {
            ErrorLog_Record(SVCDIAG_DTC_CODE_ENTER, SVCDIAG_DTC_SUBSYSTEM,
                            ServiceDiagSession_OriginState(&svcdiag_session), 0U);
            CAN_SendCommandAck(0x10, ACK_OK);
        } else {
            ServiceDiagReason_t r = ServiceDiagSession_Reason(&svcdiag_session);
            /* Vehicle-state / already-running blocks are "blocked by
             * safety"; a missing/wrong confirmation token is instead a
             * rejection of the request itself (nothing unsafe about the
             * vehicle — the operator simply did not confirm).            */
            bool safety_block = (r == SVCDIAG_REASON_BOOT_STATE)    ||
                                 (r == SVCDIAG_REASON_GEAR)          ||
                                 (r == SVCDIAG_REASON_WHEELS_MOVING) ||
                                 (r == SVCDIAG_REASON_BATTERY_LOW)   ||
                                 (r == SVCDIAG_REASON_ALREADY_ACTIVE);
            CAN_SendCommandAck(0x10, safety_block ? ACK_BLOCKED_BY_SAFETY
                                                  : ACK_REJECTED);
        }
        return;
    }

    case SVCDIAG_OP_NEXT: {
        /* byte2=channel, byte3=direction, byte4=pwm_pct, byte5-6=plateau_ms LE */
        if (len < 7) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        uint8_t  ch_raw  = payload[2];
        uint8_t  dir_raw = payload[3];
        uint8_t  pwm_pct = payload[4];
        uint16_t plateau = (uint16_t)(payload[5] | ((uint16_t)payload[6] << 8));

        if (ch_raw > (uint8_t)SVCDIAG_CH_STEERING ||
            (dir_raw != (uint8_t)SVCDIAG_DIR_FORWARD &&
             dir_raw != (uint8_t)SVCDIAG_DIR_REVERSE)) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }

        ServiceDiagConds c;
        svcdiag_build_conds(&c);
        bool ok = ServiceDiagSession_RequestStep(&svcdiag_session, &c,
                                                  (ServiceDiagChannel_t)ch_raw,
                                                  (ServiceDiagDirection_t)dir_raw,
                                                  pwm_pct, plateau);
        svcdiag_process_transition();
        svcdiag_send_session_status();
        CAN_SendCommandAck(0x10, ok ? ACK_OK : ACK_REJECTED);
        return;
    }

    case SVCDIAG_OP_ABORT:
        /* Idempotent — a no-op (with no side effect) when no session is
         * running; always ACK_OK, matching the always-safe reset actions
         * (0xFF/0xF0-0xF4) elsewhere in this dispatcher.                  */
        ServiceDiagSession_Abort(&svcdiag_session, SVCDIAG_REASON_OPERATOR);
        svcdiag_process_transition();
        svcdiag_send_session_status();
        CAN_SendCommandAck(0x10, ACK_OK);
        return;

    default:
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
}

/* Drives the ServiceDiagSession FSM (Bloque A).  Call every cycle at
 * <= SVCDIAG_WATCHDOG_MAX_GAP_MS (100 ms) — the main loop's 50 ms tick
 * gives a 2x margin.  No-op while the session is IDLE/ABORTED (matching
 * ServiceDiagSession_Update()'s own contract of only needing to be called
 * "every cycle while state != IDLE").                                     */
void CAN_ServiceDiagTick(void)
{
    svcdiag_lazy_init();
    if (!ServiceDiagSession_Active(&svcdiag_session)) return;

    ServiceDiagConds c;
    svcdiag_build_conds(&c);
    ServiceDiagSession_Update(&svcdiag_session, &c);

    bool changed = svcdiag_process_transition();

    uint32_t now = HAL_GetTick();
    if (changed) {
        svcdiag_send_session_status();
        svcdiag_sess_next_tx_ms = now + SVCDIAG_SESS_PERIOD_MS;
    } else if (ServiceDiagSession_Active(&svcdiag_session) &&
               (int32_t)(now - svcdiag_sess_next_tx_ms) >= 0) {
        svcdiag_send_session_status();
        svcdiag_sess_next_tx_ms = now + SVCDIAG_SESS_PERIOD_MS;
    }
}

void CAN_ProcessMessages(void) {
    FDCAN_RxHeaderTypeDef rx_hdr;
    uint8_t rx_payload[8];
    
    /* Check for FIFO message-lost (overflow) condition.
     * The FDCAN sets this flag when a new message arrives but
     * FIFO0 is full and the newest message is discarded.
     * Detecting this allows field diagnostics of missed frames.
     *
     * SAFETY FIX: FIFO overflow means CAN messages were lost.
     * Lost messages = unknown system state.  Escalate to DEGRADED_L1
     * if overflow count exceeds threshold (sustained bus overload).   */
    if (__HAL_FDCAN_GET_FLAG(&hfdcan1, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST)) {
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
        sat_inc_u32(&can_stats.fifo_overflow_count);

        /* Escalate on repeated overflow (≥ threshold = sustained overload).
         * A single overflow is a transient — bus load spike or delayed poll.
         * Persistent overflow means messages are consistently being lost.  */
        if (can_stats.fifo_overflow_count >= CAN_FIFO_OVERFLOW_DEGRADE_THRESHOLD) {
            Safety_SetDegradedLevel(DEGRADED_L1, DEGRADED_REASON_SENSOR_FAULT);
        }
    }

    /* Process all pending messages in RX FIFO0 */
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_hdr, rx_payload) != HAL_OK) {
            sat_inc_u32(&can_stats.rx_errors);
            continue;
        }

        /* Store received frame in global debug buffers so that the
         * debugger (Live Watch / SWV) can inspect them at any time.
         * Previously done in the ISR callback, but moved here so the
         * message is read exactly once — by the processing loop.     */
        *((volatile FDCAN_RxHeaderTypeDef *)&g_CAN_RxHeader) = rx_hdr;
        for (uint8_t i = 0; i < 8; i++)
            ((volatile uint8_t *)g_CAN_RxData)[i] = rx_payload[i];
        
        uint8_t msg_len = ExtractDLC(rx_hdr.DataLength);
        if (!CanRxPolicy_Accept(rx_hdr.Identifier, rx_hdr.IdType,
                                rx_hdr.RxFrameType, msg_len)) {
            sat_inc_u32(&can_stats.rx_errors);
            continue;
        }

        sat_inc_u32(&can_stats.rx_count);
        last_any_rx_tick = HAL_GetTick();  /* Valid protocol traffic only. */

        /* Parse received messages based on ID.
         *
         * All actuator commands are validated through the safety layer
         * before being applied.  The STM32 enforces physical reality:
         * it may clamp, rate-limit, or reject any ESP32 request.       */
        switch (rx_hdr.Identifier) {
            case CAN_ID_HEARTBEAT_ESP32:
                /* SAFETY FIX: Validate alive counter before accepting as liveness proof.
                 * Only count this heartbeat if the counter has advanced since the last
                 * frame.  A frozen ESP32 (timer ISR alive but main task stuck) will send
                 * the same counter indefinitely; after HEARTBEAT_COUNTER_FREEZE_COUNT
                 * identical frames the CAN timeout is allowed to expire → LIMP_HOME.
                 * When the counter resumes changing the freeze state is cleared.        */
                if (msg_len >= 1) {
                    uint8_t counter = rx_payload[0];
                    if (counter != esp32_hb_last_counter) {
                        /* Counter advanced — healthy heartbeat */
                        esp32_hb_last_counter = counter;
                        esp32_hb_same_count   = 0;
                        can_stats.last_heartbeat_esp32 = HAL_GetTick();
                        Safety_UpdateCANRxTime();
                    } else {
                        /* Counter unchanged — track consecutive repeats.
                         * Increment only while below the threshold so the
                         * counter saturates rather than wrapping.
                         * Update liveness only while still below threshold:
                         *   same_count < FREEZE−1 → increment then still < FREEZE → update
                         *   same_count = FREEZE−1 → increment reaches FREEZE → no update
                         *   same_count ≥ FREEZE   → already frozen, no update            */
                        if (esp32_hb_same_count < HEARTBEAT_COUNTER_FREEZE_COUNT) {
                            esp32_hb_same_count++;
                            if (esp32_hb_same_count < HEARTBEAT_COUNTER_FREEZE_COUNT) {
                                /* Still within tolerance — accept as live */
                                can_stats.last_heartbeat_esp32 = HAL_GetTick();
                                Safety_UpdateCANRxTime();
                            }
                            /* If the increment just reached the threshold: do NOT call
                             * Safety_UpdateCANRxTime().  CAN timeout will expire →
                             * LIMP_HOME as designed.                                    */
                        }
                        /* If already at/above threshold: no update, CAN timeout expires */
                    }
                } else {
                    /* DLC=0 heartbeat — no counter available, cannot
                     * validate liveness.  Reject to prevent bypass of
                     * the freeze detection mechanism.  CAN timeout will
                     * naturally expire → LIMP_HOME as designed.          */
                    sat_inc_u32(&can_stats.rx_errors);
                }
                break;
                
            case CAN_ID_CMD_THROTTLE:
                /* SAFETY FIX: Reject CAN throttle while Power-On Movement Prevention
                 * latch is active.  Startup_IsInhibited() is true from every MCU reset
                 * until the operator releases the pedal to rest for STARTUP_PEDAL_CLEAR_MS.
                 * Without this guard, a CAN throttle arriving in the ACTIVE window before
                 * the latch clears could bypass the inhibit for up to one 50 ms pedal
                 * update cycle.  Safety_ValidateThrottle() additionally enforces the
                 * state-machine gate, so both checks must independently pass.           */
                if (msg_len < 1) {
                    sat_inc_u32(&can_stats.rx_errors);
                    break;
                }
                if (!Startup_IsInhibited()) {
                    float requested_pct = (float)rx_payload[0];
                    /* Reject out-of-range values at CAN ingress.
                     * Valid throttle is 0–100%; values 101–255 from a
                     * corrupt or injected frame are rejected entirely.
                     * Using break (not clamp-to-0) avoids creating a
                     * demand discontinuity that would trigger the
                     * step-rate anomaly detector in Traction_SetDemand
                     * and cause a spurious DEGRADED transition.          */
                    if (requested_pct > 100.0f) {
                        sat_inc_u32(&can_stats.rx_errors);
                        break;
                    }
                    float validated_pct = Safety_ValidateThrottle(requested_pct);
                    Traction_SetDemand(validated_pct);
                }
                break;
                
            case CAN_ID_CMD_STEERING:
                /* 0x101 carries the LOCAL steering demand (driven by the
                 * STM32 from the child's physical steering input via the
                 * ESP32 sensor path).  When the RC override is active
                 * (CAN_ID_CMD_RC_OVERRIDE 0x10A fresh AND override flag
                 * set), this local value must NOT reach Steering_SetAngle
                 * — the arbiter's last validated RC angle takes over.
                 * Routing through RcArbiter_GetSteering() gives a single
                 * point of truth: local in, arbitrated out.              */
                if (msg_len >= 2) {
                    int16_t angle_raw = (int16_t)(rx_payload[0] | (rx_payload[1] << 8));
                    float requested_deg = (float)angle_raw / 10.0f;
                    float arbitrated_deg = RcArbiter_GetSteering(requested_deg, HAL_GetTick());
                    float validated_deg = Safety_ValidateSteering(arbitrated_deg);
                    Steering_SetAngle(validated_deg);
                } else {
                    sat_inc_u32(&can_stats.rx_errors);
                }
                break;

            case CAN_ID_CMD_RC_OVERRIDE:
                /* Bench/local-only images compile the STM32 authority path
                 * out as well as the ESP32 iBUS parser.  RcArbiter_OnFrame()
                 * records the injection as rejected and never refreshes an
                 * authority watchdog when RC_OVERRIDE_ENABLED == 0. */
                RcArbiter_OnFrame(rx_payload, msg_len, HAL_GetTick());
                break;
                
            case CAN_ID_CMD_MODE:
                if (msg_len < 1) {
                    sat_inc_u32(&can_stats.rx_errors);
                    CAN_SendCommandAck(0x02, ACK_INVALID);
                    break;
                }
                if (!Safety_IsCommandAllowed()) {
                    /* §3 (Option A) — close the "boot in the wrong mode" window.
                     * When the command gate is closed only because the vehicle
                     * is still in STANDBY, a dedicated SAFE gate lets a CMD_MODE
                     * update ONLY the LOGICAL 4x4 / tank-turn flags so the STM32
                     * mode tracks the physical selector BEFORE traction is ever
                     * energised.  It applies NO gear and NO motion (the flags
                     * have no effect until the pedal/demand path runs in ACTIVE)
                     * and requires pedal released, zero PWM and safe speed.  Any
                     * other blocked case (SAFE/ERROR/etc.) still replies
                     * BLOCKED_BY_SAFETY. */
                    if (Safety_IsStandbyModeSyncAllowed()) {
                        uint8_t mode_flags = rx_payload[0];
                        Traction_SetMode4x4((mode_flags & 0x01) != 0);
                        Traction_SetAxisRotation((mode_flags & 0x02) != 0);
                        /* Logical mode applied; gear/motion remain blocked. */
                        CAN_SendCommandAck(0x02, ACK_OK);
                    } else {
                        CAN_SendCommandAck(0x02, ACK_BLOCKED_BY_SAFETY);
                    }
                    break;
                }
                {
                    /* ====================================================
                     * F3 — Atomic CMD_MODE validation:
                     *   1) Decode requested mode and (optional) gear.
                     *   2) Run ALL safety validations first.
                     *   3) Commit BOTH mode and gear only if EVERY check
                     *      passes; otherwise commit NOTHING and return a
                     *      proper ACK (REJECTED / BLOCKED_BY_SAFETY /
                     *      INVALID).
                     *   This prevents partial application (mode applied
                     *   while gear rejected) that previously left the
                     *   system in an inconsistent half-committed state.
                     *
                     * F4 — Safe direction transition:
                     *   When switching between FORWARD/FORWARD_D2 and
                     *   REVERSE, require EITHER a transition through
                     *   NEUTRAL (current gear == NEUTRAL or requested ==
                     *   NEUTRAL) OR that the pedal is released below
                     *   CMD_MODE_PEDAL_REST_PCT.  The ≤1 km/h speed gate
                     *   is preserved and stacks with this check.
                     * ==================================================== */

                    /* Pedal-release threshold for D↔R transition (% of full).
                     * Intentionally a local constant — mirrors the value of
                     * main.c STARTUP_PEDAL_REST_PCT (3 %) without creating
                     * a cross-module dependency in this safety-critical TU.
                     * KEEP IN SYNC: if STARTUP_PEDAL_REST_PCT in main.c is
                     * tuned, update this constant too.                     */
                    static const float CMD_MODE_PEDAL_REST_PCT = 3.0f;

                    uint8_t mode_flags = rx_payload[0];
                    bool enable_4x4 = (mode_flags & 0x01) != 0;
                    bool tank_turn  = (mode_flags & 0x02) != 0;

                    /* --- Decode optional gear byte (msg_len >= 2) ------ */
                    bool gear_present = (msg_len >= 2);
                    GearPosition_t requested_gear = Traction_GetGear();
                    bool gear_range_ok = true;
                    if (gear_present) {
                        uint8_t gear_raw = rx_payload[1];
                        if (gear_raw <= (uint8_t)GEAR_FORWARD_D2) {
                            requested_gear = (GearPosition_t)gear_raw;
                        } else {
                            gear_range_ok = false;
                        }
                    }

                    /* --- Validate mode change (low-speed gate) --------- */
                    bool mode_ok = Safety_ValidateModeChange(enable_4x4, tank_turn);

                    /* --- Validate gear change (low-speed + D↔R safety) - */
                    bool gear_ok = gear_range_ok;
                    if (gear_present && gear_ok) {
                        /* Shifting into NEUTRAL is ALWAYS permitted,
                         * regardless of speed.  Coasting (N) is inherently
                         * safe, and a self-induced turn (tank turn / axis
                         * rotation) spins the wheels, which would otherwise
                         * trap the driver in the current gear by tripping the
                         * ≤1 km/h speed gate.  NEUTRAL alone guarantees an
                         * escape path, so it is never blocked.
                         *
                         * PARK is deliberately EXCLUDED from this escape
                         * hatch: GEAR_PARK engages MOTOR_MODE_BRAKE (shorted
                         * motor terminals) in Traction_Update, so allowing
                         * PARK at arbitrary wheel speed would cause abrupt
                         * dynamic braking and a large current spike.  PARK
                         * therefore remains subject to the ≤1 km/h gate.     */
                        bool req_is_safe_gear = (requested_gear == GEAR_NEUTRAL);
                        if (!req_is_safe_gear) {
                            float avg_spd = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                                             Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
                            avg_spd = sanitize_float(avg_spd, SANITIZE_SPEED_DEFAULT);
                            if (avg_spd > 1.0f) {
                                gear_ok = false;        /* preserves ≤1 km/h gate */
                            } else {
                                /* F4: forbid direct D↔R without NEUTRAL or
                                 * pedal-released confirmation.                */
                                GearPosition_t cur = Traction_GetGear();
                                bool cur_is_fwd = (cur == GEAR_FORWARD ||
                                                   cur == GEAR_FORWARD_D2);
                                bool req_is_fwd = (requested_gear == GEAR_FORWARD ||
                                                   requested_gear == GEAR_FORWARD_D2);
                                bool cur_is_rev = (cur == GEAR_REVERSE);
                                bool req_is_rev = (requested_gear == GEAR_REVERSE);
                                bool direction_swap = (cur_is_fwd && req_is_rev) ||
                                                      (cur_is_rev && req_is_fwd);
                                if (direction_swap) {
                                    float pedal = Pedal_GetPercent();
                                    pedal = sanitize_float(pedal, 0.0f);
                                    if (pedal > CMD_MODE_PEDAL_REST_PCT) {
                                        /* Direct D↔R swap requested while the
                                         * pedal is not released — reject.
                                         * (The NEUTRAL path is implicitly
                                         *  handled: direction_swap is false
                                         *  whenever either side is NEUTRAL.) */
                                        gear_ok = false;
                                    }
                                }
                            }
                        }
                    }

                    /* --- Atomic commit (all-or-nothing) ---------------- */
                    /* Exception: shifting into NEUTRAL must always succeed
                     * once the range is valid, even if the mode-change speed
                     * gate rejected the 4x4/tank-turn portion.  Otherwise a
                     * self-induced turn (tank turn) would keep the wheels
                     * spinning >1 km/h and trap the driver in the current
                     * driving gear — coasting is always safe, so the path to
                     * NEUTRAL is never blocked.
                     *
                     * PARK is excluded on purpose: it maps to MOTOR_MODE_BRAKE
                     * (dynamic braking), so applying it while the wheels spin
                     * would brake abruptly.  NEUTRAL alone provides the escape
                     * path; PARK stays subject to the ≤1 km/h gate above.     */
                    bool safe_gear_req = gear_present && gear_ok &&
                                         (requested_gear == GEAR_NEUTRAL);
                    if (mode_ok && gear_ok) {
                        Traction_SetMode4x4(enable_4x4);
                        Traction_SetAxisRotation(tank_turn);
                        if (gear_present) {
                            Traction_SetGear(requested_gear);
                        }
                        CAN_SendCommandAck(0x02, ACK_OK);
                    } else if (safe_gear_req) {
                        /* Apply only the gear (to NEUTRAL); leave the mode
                         * unchanged because its speed gate was not met.   */
                        Traction_SetGear(requested_gear);
                        CAN_SendCommandAck(0x02, ACK_OK);
                    } else if (!gear_range_ok) {
                        CAN_SendCommandAck(0x02, ACK_INVALID);
                    } else {
                        CAN_SendCommandAck(0x02, ACK_REJECTED);
                    }
                }
                break;

            case CAN_ID_SERVICE_CMD:
                /* Service mode commands from ESP32:
                 *   Byte 0: command (0=disable, 1=enable, 0xFF=factory restore)
                 *   Byte 1: module_id (only for disable/enable)
                 *
                 * Safety constraints enforced here (STM32 is safety authority):
                 *   1. Critical modules cannot be disabled (ServiceMode_DisableModule
                 *      rejects them).
                 *   2. Disable commands require ACTIVE or DEGRADED state — rejected
                 *      in SAFE/ERROR/BOOT/STANDBY/LIMP_HOME.
                 *   3. Safety-relevant modules (ABS, TCS, wheel speed, obstacle)
                 *      cannot be disabled while vehicle speed > 0.
                 *   4. Enable and factory-restore are always accepted (restoring
                 *      modules is inherently safe).
                 *
                 * Timing / execution-order notes:
                 *   - CAN_ProcessMessages() runs in the main loop after the 10 ms
                 *     safety checks (ABS/TCS/Safety_Check*) and 50 ms sensor reads.
                 *   - Wheel_GetSpeed_*() calls Wheel_ComputeSpeed() inline, which
                 *     reads ISR-updated pulse counters and HAL_GetTick() at call
                 *     time — speed data is always current, never stale.
                 *   - Safety_IsCommandAllowed() reads system_state, which is a
                 *     single-word write on Cortex-M4 (atomic).  No ISR can modify
                 *     it between check and use because CAN processing is not
                 *     interrupt-driven.
                 *   - No rolling counter or authentication on SERVICE_CMD.
                 *     Physical CAN bus access could inject valid frames.
                 *     Acceptable for closed prototype / educational vehicle.
                 *     SECURITY: For production deployment, SERVICE_CMD would
                 *     require a rolling counter, session token, or challenge-
                 *     response mechanism to prevent replay and injection.     */
                if (msg_len < 1) {
                    sat_inc_u32(&can_stats.rx_errors);
                    CAN_SendCommandAck(0x10, ACK_INVALID);
                    break;
                }
                {
                    uint8_t cmd = rx_payload[0];
                    if (cmd == 0xFF) {
                        /* Factory restore — re-enable all modules (always safe) */
                        ServiceMode_FactoryRestore();
                        CAN_SendCommandAck(0x10, ACK_OK);
                    } else if (cmd == 0xFE) {
                        /* Clear error log.  Gated to STANDBY in
                         * ErrorLog_Clear(); propagate its return
                         * code so the engineering UI distinguishes
                         * "cleared OK" from "rejected: not in
                         * standby / flash error".                  */
                        bool cleared = ErrorLog_Clear();
                        CAN_SendCommandAck(0x10, cleared ? ACK_OK : ACK_REJECTED);
                    } else if (cmd >= 0xF0 && cmd <= 0xF4) {
                        /* Individual factory-default reset commands (always safe).
                         * These reset specific calibration categories to defaults.
                         * The STM32 re-enables the corresponding modules and clears
                         * their faults.  A reboot may be needed for full effect. */
                        switch (cmd) {
                            case 0xF0: /* Reset steering PID calibration */
                                ServiceMode_EnableModule(MODULE_STEER_CENTER);
                                ServiceMode_EnableModule(MODULE_STEER_ENCODER);
                                ServiceMode_ClearFault(MODULE_STEER_CENTER);
                                ServiceMode_ClearFault(MODULE_STEER_ENCODER);
                                break;
                            case 0xF1: /* Reset wheel speed sensors */
                                ServiceMode_EnableModule(MODULE_WHEEL_SPEED_FL);
                                ServiceMode_EnableModule(MODULE_WHEEL_SPEED_FR);
                                ServiceMode_EnableModule(MODULE_WHEEL_SPEED_RL);
                                ServiceMode_EnableModule(MODULE_WHEEL_SPEED_RR);
                                ServiceMode_ClearFault(MODULE_WHEEL_SPEED_FL);
                                ServiceMode_ClearFault(MODULE_WHEEL_SPEED_FR);
                                ServiceMode_ClearFault(MODULE_WHEEL_SPEED_RL);
                                ServiceMode_ClearFault(MODULE_WHEEL_SPEED_RR);
                                break;
                            case 0xF2: /* Reset INA226 / shunt calibration.
                                      * MODULE_CURRENT_SENSOR_0 (9) through
                                      * MODULE_CURRENT_SENSOR_5 (14) are
                                      * consecutive in ModuleID_t enum. */
                                for (unsigned s = MODULE_CURRENT_SENSOR_0;
                                     s <= MODULE_CURRENT_SENSOR_5; s++) {
                                    ServiceMode_EnableModule((ModuleID_t)s);
                                    ServiceMode_ClearFault((ModuleID_t)s);
                                }
                                break;
                            case 0xF3: /* Reset traction motor force */
                                ServiceMode_EnableModule(MODULE_ABS);
                                ServiceMode_EnableModule(MODULE_TCS);
                                ServiceMode_ClearFault(MODULE_ABS);
                                ServiceMode_ClearFault(MODULE_TCS);
                                break;
                            case 0xF4: /* Reset steering motor force */
                                ServiceMode_EnableModule(MODULE_STEER_CENTER);
                                ServiceMode_EnableModule(MODULE_STEER_ENCODER);
                                ServiceMode_EnableModule(MODULE_ACKERMANN);
                                ServiceMode_ClearFault(MODULE_STEER_CENTER);
                                ServiceMode_ClearFault(MODULE_STEER_ENCODER);
                                ServiceMode_ClearFault(MODULE_ACKERMANN);
                                break;
                            default:
                                break;
                        }
                        CAN_SendCommandAck(0x10, ACK_OK);
                    } else if (cmd == SERVICE_ACTION_I2C_SERVICE) {
                        /* ---- I2C SERVICE-MODE SCAN (Level 3 diagnostic) ----
                         * Active probe of the I2C topology: mux/INA presence,
                         * SDA/SCL idle levels and a bus-recovery attempt if
                         * SDA is stuck low.  Reports on 0x30B and also dumps
                         * the FDCAN error counters on 0x30C.  Diagnostic only
                         * — does not gate any control or safety path.
                         *
                         * Emit an immediate "scan started" echo (CMD_ACK with
                         * cmd_id_low = 0xF6) BEFORE running the synchronous
                         * probe.  This lets the HMI distinguish a lost request
                         * (no echo → "TIMEOUT: NO CAN ACK") from a lost reply
                         * (echo but no 0x30B → "TIMEOUT: NO SCAN REPLY").     */
                        CAN_SendCommandAck(SERVICE_ACTION_I2C_SERVICE, ACK_OK);
                        CAN_SendI2CScanReport();
                        CAN_SendFdcanDiag();
                        CAN_SendCommandAck(0x10, ACK_OK);
                    } else if (cmd == SERVICE_ACTION_PEDAL_CAL) {
                        /* ---- PEDAL ENDPOINT CALIBRATION ----
                         * Byte 1 = sub-opcode (0x01..0x05).  See
                         * PEDAL_CAL_OP_* macros in can_handler.h and
                         * docs/CALIBRATION.md.  The handler enforces
                         * safety gates internally and replies on the
                         * standard CMD_ACK (0x103) channel — DLC of
                         * CMD_ACK is preserved at 3 bytes.            */
                        pedalcal_handle_service_cmd(rx_payload, msg_len);
                    } else if (cmd == SERVICE_ACTION_GEAR_LIMITS) {
                        /* ---- GEAR POWER-LIMIT CONFIG ----
                         * Byte 1 = sub-opcode (0x01..0x06).  See
                         * GEAR_LIMIT_OP_* macros in can_handler.h and
                         * docs/ENGINEERING_MENU.md.  The handler enforces
                         * the STANDBY safety gate internally and replies on
                         * the standard CMD_ACK (0x103) channel.            */
                        gearlim_handle_service_cmd(rx_payload, msg_len);
                    } else if (cmd == SERVICE_ACTION_STEERING_Z) {
                        /* ---- PB5 + ENCODER-Z CENTER DIAGNOSTIC ----
                         * Byte 1 = sub-opcode (QUERY/CALIBRATE/CLEAR).
                         * PB5 stays the primary/safety reference; Z is a
                         * secondary precision reference and can never center
                         * on its own.  CALIBRATE requires PB5 to confirm
                         * center and BOOT/STANDBY; the handler enforces the
                         * gates internally and replies on CMD_ACK (0x103). */
                        steerz_handle_service_cmd(rx_payload, msg_len);
                    } else if (cmd == SERVICE_ACTION_EPS_PARAMS) {
                        /* ---- EPS RUNTIME PARAMETER TUNING ----
                         * Byte 1 = sub-opcode.  SET_PARAM applies live but
                         * is gated to STANDBY + PARK/NEUTRAL + stationary +
                         * ~zero demand + no dangerous fault (Item A);
                         * SAVE/RESET are STANDBY-gated.  Replies on
                         * CMD_ACK (0x103); telemetry on 0x30F.            */
                        eps_handle_service_cmd(rx_payload, msg_len);
                    } else if (cmd == SERVICE_ACTION_DRIVE_TUNING) {
                        /* ---- DRIVE-TUNING (RAMP/CREEP) CONFIG ----
                         * Byte 1 = sub-opcode; SET_* carry a uint16 LE in
                         * bytes 2-3.  The handler enforces the STANDBY safety
                         * gate internally and replies on CMD_ACK (0x103);
                         * telemetry on 0x310.                              */
                        drvtune_handle_service_cmd(rx_payload, msg_len);
                    } else if (cmd == SERVICE_ACTION_BATTERY_LIMITS) {
                        /* ---- BATTERY VOLTAGE-LIMIT CONFIG ----
                         * Byte 1 = sub-opcode; SET_* carry a uint16 LE
                         * (centivolts / ms) in bytes 2-3.  The handler
                         * enforces the STANDBY safety gate internally and
                         * never touches the safety state machine.  Replies on
                         * CMD_ACK (0x103); telemetry on 0x311.             */
                        battlim_handle_service_cmd(rx_payload, msg_len);
                    } else if (cmd == SERVICE_ACTION_SELF_TEST) {
                        /* ---- SERVICE_DIAG SELF-TEST SESSION (Bloque A) ----
                         * Byte 1 = sub-opcode (START/NEXT/ABORT/QUERY). The
                         * pure ServiceDiagSession FSM enforces every entry
                         * gate/abort condition internally (BOOT, gear,
                         * stationary wheels, confirm token, battery, one
                         * actuator at a time, watchdog, timeouts); this
                         * handler only supplies the live conditions and
                         * relays the concrete reject reason on 0x31B.
                         * Replies on CMD_ACK (0x103); telemetry on 0x31B
                         * (10 Hz while active, silent otherwise) and 0x31C
                         * (once per step close).  Hito 1: no actuator is
                         * driven yet (see the block comment above
                         * svcdiag_build_conds()).                          */
                        svcdiag_handle_service_cmd(rx_payload, msg_len);
                    } else if (cmd == 0xE0) {
                        /* ---- RELAY OVERRIDE (Engineering Diagnostic Mode) ----
                         * Byte 1: relay control mask
                         *   bit 0: override enable (1=on, 0=off)
                         *   bit 1: reserved (always 0; PC10 not connected)
                         *   bit 2: TRACTION relay
                         *   bit 3: STEER_PWR relay (12 V steering actuator supply;
                         *          legacy name "DIRECTION relay")
                         *
                         * Safety gating is enforced by Safety_SetRelayOverride():
                         *   - System must be in STANDBY
                         *   - Throttle == 0%, Speed == 0 km/h
                         *   - No active safety errors
                         * If any condition fails, the command is silently rejected
                         * and the override is disabled.                           */
                        if (msg_len >= 2) {
                            uint8_t relay_ctl = rx_payload[1];
                            bool enable = (relay_ctl & 0x01U) != 0;
                            uint8_t mask = (relay_ctl >> 1) & 0x07U;
                            Safety_SetRelayOverride(enable, mask);
                            CAN_AckResult_t ack_result = Safety_IsRelayOverrideActive()
                                ? ACK_OK : (enable ? ACK_BLOCKED_BY_SAFETY : ACK_OK);
                            CAN_SendCommandAck(0x10, ack_result);
                        } else {
                            CAN_SendCommandAck(0x10, ACK_INVALID);
                        }
                    } else if (msg_len >= 2) {
                        uint8_t mod_id = rx_payload[1];
                        if (mod_id < MODULE_COUNT) {
                            if (cmd == 0) {
                                /* ---- DISABLE command ---- */

                                /* Gate 1: system state must allow commands.
                                 * Matches the CMD_MODE pattern (line 651). */
                                if (!Safety_IsCommandAllowed()) {
                                    CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
                                    break;
                                }

                                /* Gate 2: safety-relevant modules cannot be
                                 * disabled while the vehicle is in motion.
                                 * ABS, TCS, wheel speed sensors, and obstacle
                                 * detection are needed for safe deceleration. */
                                if (mod_id == MODULE_ABS              ||
                                    mod_id == MODULE_TCS              ||
                                    mod_id == MODULE_WHEEL_SPEED_FL   ||
                                    mod_id == MODULE_WHEEL_SPEED_FR   ||
                                    mod_id == MODULE_WHEEL_SPEED_RL   ||
                                    mod_id == MODULE_WHEEL_SPEED_RR   ||
                                    mod_id == MODULE_OBSTACLE_DETECT) {
                                    float avg_spd = (Wheel_GetSpeed_FL() +
                                                     Wheel_GetSpeed_FR() +
                                                     Wheel_GetSpeed_RL() +
                                                     Wheel_GetSpeed_RR()) / 4.0f;
                                    avg_spd = sanitize_float(avg_spd,
                                                             SANITIZE_SPEED_DEFAULT);
                                    if (avg_spd > 0.5f) {
                                        CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
                                        break;
                                    }
                                }

                                /* Gate 3: critical classification (final guard) */
                                bool ok = ServiceMode_DisableModule((ModuleID_t)mod_id);
                                CAN_SendCommandAck(0x10, ok ? ACK_OK : ACK_REJECTED);
                            } else if (cmd == 1) {
                                /* ENABLE — always accepted (restoring is safe) */
                                ServiceMode_EnableModule((ModuleID_t)mod_id);
                                CAN_SendCommandAck(0x10, ACK_OK);
                            } else {
                                CAN_SendCommandAck(0x10, ACK_INVALID);
                            }
                        } else {
                            CAN_SendCommandAck(0x10, ACK_INVALID);
                        }
                    } else {
                        CAN_SendCommandAck(0x10, ACK_INVALID);
                    }
                }
                break;

            case CAN_ID_OBSTACLE_DISTANCE:
                /* Obstacle distance from ESP32 (0x208):
                 *   Byte 0-1: minimum distance (mm, uint16 LE)
                 *   Byte 2:   zone level (0–4, uint8)
                 *   Byte 3:   sensor health (0=unhealthy, 1=healthy)
                 *   Byte 4:   rolling counter (uint8, 0–255)
                 *
                 * Processed by Obstacle_ProcessCAN() in safety_system.c.
                 * The STM32 applies a simplified backstop limiter
                 * independently of the ESP32's 5-zone logic.            */
                if (msg_len >= 5) {
                    Obstacle_ProcessCAN(rx_payload, msg_len);
                } else {
                    sat_inc_u32(&can_stats.rx_errors);
                }
                break;

            case CAN_ID_OBSTACLE_SAFETY:
                /* Obstacle safety state from ESP32 (0x209):
                 *   Byte 0: zone (0–4)
                 *   Byte 1: sensor status (0=WAITING, 1=INVALID, 2=VALID)
                 *   Byte 2: stuck flag (0=OK, 1=stuck)
                 *   Byte 3: reserved
                 *
                 * Informational: STM32 computes its own obstacle_scale
                 * from raw distance in 0x208.  This frame is used for
                 * cross-validation and diagnostic logging only.             */
                if (msg_len >= 3) {
                    Obstacle_ProcessSafetyCAN(rx_payload, msg_len);
                } else {
                    sat_inc_u32(&can_stats.rx_errors);
                }
                break;

            case CAN_ID_CMD_LED:
                /* LED relay control from ESP32 (0x120):
                 *   Byte 0: front relay (0 = OFF, 1 = ON)
                 *   Byte 1: rear  relay (0 = OFF, 1 = ON)
                 *
                 * Controls the 5V power relays for WS2812B LED strips.
                 * Always accepted (not safety-critical).  ACK confirms
                 * the new state.                                        */
                if (msg_len >= 1) {
                    LED_Relay_Set(rx_payload[0] != 0);
                    if (msg_len >= 2) {
                        LED_Relay_Rear_Set(rx_payload[1] != 0);
                    }
                    CAN_SendCommandAck(CAN_ID_CMD_LED & 0xFF, ACK_OK);
                } else {
                    sat_inc_u32(&can_stats.rx_errors);
                    CAN_SendCommandAck(CAN_ID_CMD_LED & 0xFF, ACK_INVALID);
                }
                break;

            case CAN_ID_CMD_SYSTEM_SHUTDOWN:
                /* Deterministic pre-power-cut safe-state request from ESP32
                 * (0x130).  Sent when the ignition key is turned OFF, before
                 * the external delay relay physically removes power.
                 *
                 * Payload is empty or one byte (ignored).  The frame is
                 * accepted unconditionally — even if msg_len == 0 — because
                 * forcing the safe state is always a non-destructive action
                 * and is idempotent (Safety_RequestShutdown is safe to call
                 * multiple times).
                 *
                 * No ACK is sent: by the time this frame is received the
                 * ESP32 is preparing for power loss and will not consume
                 * the ACK.  Avoiding the ACK keeps the path entirely
                 * non-blocking on the STM32 side.                        */
                Safety_RequestShutdown();
                break;

            case CAN_ID_CMD_SENSOR_MAP_TEMP:
                /* DS18B20 physIdx→role mapping from ESP32 engineering menu.
                 *   DLC:    5 (one byte per physical sensor index 0-4)
                 *   Byte i: role assigned to physical sensor i
                 *            0=FL, 1=FR, 2=RL, 3=RR, 4=Ambient
                 *            0xFF reserved for "unset" (not accepted in
                 *            strict mode — see below).
                 *
                 * ---- STRICT VALIDATION (production hardening) ----
                 * To guarantee that every motor position has an
                 * unambiguous temperature source, we REQUIRE a complete
                 * permutation of roles 0..4: each role must appear
                 * exactly once.  This rejects:
                 *   - payloads shorter than 5 bytes        (DLC check)
                 *   - values outside the 0..4 range        (includes 0xFF)
                 *   - duplicated roles (e.g. two sensors claim FL motor)
                 *   - missing roles (implied by the two above rules)
                 *
                 * The ESP32 engineering UI always sends a full 5-byte
                 * permutation, so strict mode is transparent in normal
                 * operation and only catches genuinely broken payloads.
                 *
                 * ---- ACK SEMANTICS (format UNCHANGED) ----
                 *   ACK_OK       → map validated AND flash write succeeded
                 *   ACK_INVALID  → payload rejected at validation (never
                 *                  reached flash, never touched any state)
                 *   ACK_REJECTED → payload was valid but flash write
                 *                  failed (unlock/erase/program error) OR
                 *                  rate-limited (too many writes).  State
                 *                  unchanged — previous mapping still
                 *                  active.
                 * The frame format is untouched; only the meaning of
                 * each code on this specific command is clarified.      */
                if (msg_len < 5) {
                    sat_inc_u32(&can_stats.rx_errors);
                    CAN_SendCommandAck(CAN_ID_CMD_SENSOR_MAP_TEMP & 0xFF, ACK_INVALID);
                    break;
                }
                {
                    bool    map_valid = true;
                    uint8_t role_seen = 0;  /* bitmask of roles 0..4 seen */
                    for (uint8_t i = 0; i < 5U; i++) {
                        uint8_t role = rx_payload[i];
                        /* Strict mode: 0xFF (unset) is NOT accepted.  The
                         * ESP32 always provides a complete permutation.   */
                        if (role >= 5U) { map_valid = false; break; }
                        uint8_t bit = (uint8_t)(1U << role);
                        if (role_seen & bit) { map_valid = false; break; }
                        role_seen |= bit;
                    }
                    /* All five bits (0b00011111 = 0x1F) must be set — this
                     * proves every role is assigned at least once and,
                     * combined with the duplicate check above, exactly
                     * once.  Redundant with the per-iteration checks but
                     * kept as an explicit, self-documenting invariant.   */
                    if (map_valid && role_seen != 0x1FU) {
                        map_valid = false;
                    }
                    if (!map_valid) {
                        sat_inc_u32(&can_stats.rx_errors);
                        CAN_SendCommandAck(CAN_ID_CMD_SENSOR_MAP_TEMP & 0xFF, ACK_INVALID);
                        break;
                    }
                    /* Valid payload — attempt to persist.  The store
                     * performs its own no-op elision and wear-rate
                     * limiting; on either a true write error or a
                     * rate-limited rejection we report ACK_REJECTED so
                     * the UI can distinguish "bad data" from "busy".   */
                    bool saved = SensorMapStore_Save(rx_payload);
                    CAN_SendCommandAck(CAN_ID_CMD_SENSOR_MAP_TEMP & 0xFF,
                                       saved ? ACK_OK : ACK_REJECTED);
                    /* Post-save consistency:  SensorMapStore_Save()
                     * updated the in-RAM active map BEFORE returning
                     * (both on no-op elision and on successful write),
                     * so the very next CAN_SendStatusTempMap() tick at
                     * the 1 Hz slow-tier will naturally reflect the
                     * new mapping — no extra CAN frame needed.          */
                }
                break;

            default:
                /* Unknown message ID – filtered out by hardware,
                 * should never reach here.                        */
                break;
        }
    }
}

/* ================================================================== */
/*  Bus-Off Detection and Recovery                                     */
/* ================================================================== */

/**
 * @brief  Query whether the CAN bus is currently in bus-off state.
 * @return true if bus-off is active, false otherwise.
 */
bool CAN_IsBusOff(void)
{
    return (busoff_active != 0);
}

/**
 * @brief  Check FDCAN for bus-off condition and attempt non-blocking recovery.
 *
 * Called from the 10 ms safety loop in main.c.  Uses the FDCAN protocol
 * status register to detect bus-off.  When bus-off is detected:
 *   1. Sets SAFETY_ERROR_CAN_BUSOFF
 *   2. Transitions to SYS_STATE_SAFE
 *   3. Attempts non-blocking recovery at CAN_BUSOFF_RETRY_INTERVAL_MS intervals
 *
 * Recovery sequence: Stop → DeInit → Init → ConfigFilters → ActivateNotification → Start
 *
 * If recovery succeeds, the bus-off flag is cleared.  The system will
 * recover from SAFE via the existing Safety_CheckCANTimeout() path once
 * heartbeat messages resume.
 *
 * No blocking delays.  Watchdog continues to be fed by the main loop.
 */
void CAN_CheckBusOff(void)
{
    FDCAN_ProtocolStatusTypeDef psr;
    FDCAN_ErrorCountersTypeDef  ecr;
    uint32_t now = HAL_GetTick();

    if (busoff_active) {
        /* ---- Phase 2: PROBATION — peripheral re-initialised, awaiting
         * sustained heartbeats before declaring recovery. -------------- */
        if (BusOffRecoveryWindow_InProbation(&busoff_window)) {
            /* Watch for a fresh bus-off during probation.  If the bus goes
             * off again the retry counter is deliberately NOT reset, so a
             * persistent fault keeps counting toward CAN_BUSOFF_MAX_RETRIES
             * instead of looping forever. */
            if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &psr) == HAL_OK) {
                can_diag.bus_off = psr.BusOff ? 1U : 0U;
                if (psr.BusOff) {
                    BusOffRecoveryWindow_Reset(&busoff_window);
                    busoff_last_attempt = now;  /* re-enter retry interval */
                    sat_inc_u32(&can_stats.busoff_count);
                    return;
                }
            }

            /* Require valid heartbeats sustained over the confirmation
             * window before clearing the fault.  Being RUNNING is not
             * sufficient. */
            if (BusOffRecoveryWindow_Update(&busoff_window, now,
                                            CAN_IsESP32Alive(),
                                            CAN_BUSOFF_RECOVERY_WINDOW_MS)) {
                busoff_active      = 0;
                busoff_retry_count = 0;
                Safety_ClearError(SAFETY_ERROR_CAN_BUSOFF);
            }
            return;
        }

        /* ---- Phase 1: RECOVERING — attempt peripheral re-init. -------- */
        if ((now - busoff_last_attempt) < CAN_BUSOFF_RETRY_INTERVAL_MS) {
            return;  /* Not yet time for next attempt */
        }

        /* Too many retries — stop attempting, system stays in LIMP/SAFE */
        if (busoff_retry_count >= CAN_BUSOFF_MAX_RETRIES) {
            return;
        }

        /* Attempt recovery: Stop → DeInit → Init → Filters → Notify → Start */
        busoff_last_attempt = now;
        busoff_retry_count++;

        HAL_FDCAN_Stop(&hfdcan1);

        if (HAL_FDCAN_DeInit(&hfdcan1) != HAL_OK) {
            return;  /* DeInit failed — retry next interval */
        }

        if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
            return;  /* Init failed — retry next interval */
        }

        CAN_ConfigureFilters();
        if (can_init_diag.filter_global != HAL_OK) {
            return;  /* Filter/firewall setup failed — retry next interval. */
        }

        if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            return;  /* Notification setup failed — retry next interval */
        }

        if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
            return;  /* Start failed — retry next interval */
        }

        /* Peripheral is RUNNING again, but recovery is NOT yet declared:
         * begin the heartbeat-confirmation probation window.  The fault
         * and retry counter are intentionally left in place until valid
         * heartbeats are sustained (see busoff_recovery.h). */
        BusOffRecoveryWindow_Begin(&busoff_window, now);
        return;
    }

    /* ---- Phase 0: HEALTHY — poll FDCAN protocol status for bus-off ---- */
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &psr) != HAL_OK) {
        return;  /* Cannot read status — skip this cycle */
    }

    /* ---- CAN bus error diagnostics ----
     * Populate can_diag with the full protocol status so a debugger
     * (or a future diagnostic CAN frame) can identify the exact error
     * type.  LastErrorCode distinguishes ACK, bit, stuff, form, CRC.   */
    can_diag.last_error_code = (uint8_t)psr.LastErrorCode;
    can_diag.error_passive   = psr.ErrorPassive ? 1U : 0U;
    can_diag.bus_off         = psr.BusOff       ? 1U : 0U;
    can_diag.warning         = psr.Warning      ? 1U : 0U;
    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &ecr) == HAL_OK) {
        can_diag.tec = (uint8_t)ecr.TxErrorCnt;
        can_diag.rec = (uint8_t)ecr.RxErrorCnt;
    }

    if (psr.BusOff) {
        /* Fresh bus-off — raise fault and enter LIMP_HOME.
         * CAN bus-off is a communication failure, not a hardware
         * danger.  Vehicle remains mobile at walking speed.            */
        busoff_active       = 1;
        busoff_last_attempt = now;
        busoff_retry_count  = 0;
        BusOffRecoveryWindow_Reset(&busoff_window);
        sat_inc_u32(&can_stats.busoff_count);
        Safety_SetError(SAFETY_ERROR_CAN_BUSOFF);
        Safety_SetState(SYS_STATE_LIMP_HOME);
    }
}

bool CAN_IsESP32Alive(void) {
    uint32_t time_since_heartbeat = HAL_GetTick() - can_stats.last_heartbeat_esp32;
    return (time_since_heartbeat < CAN_TIMEOUT_HEARTBEAT_MS);
}

/**
 * @brief  Check if the CAN bus is globally silent (no frames at all).
 *
 * Unlike CAN_IsESP32Alive() which only tracks ESP32 heartbeats, this
 * function detects total bus silence — no messages from any node.
 * A completely silent CAN bus indicates a hardware fault (bad wiring,
 * failed transceiver, bus contention) that the heartbeat timeout alone
 * might not catch quickly if other periodic messages mask the problem.
 *
 * @retval true   No CAN frames received for > CAN_GLOBAL_SILENCE_MS
 * @retval false  At least one frame received recently
 */
bool CAN_IsGlobalSilent(void) {
    return ((HAL_GetTick() - last_any_rx_tick) > CAN_GLOBAL_SILENCE_MS);
}

/**
 * @brief  Compute CAN RX frames-per-second metric.
 *
 * Call once per second from the 1000 ms task tier.  Compares the current
 * rx_count with the snapshot taken at the previous call and stores the
 * delta in can_stats.rx_frames_per_sec.
 *
 * A sudden drop from e.g. 50 fps to 0 indicates the bus has died (even
 * if CAN_IsGlobalSilent() has not yet reached its 1-second threshold).
 * The value is purely diagnostic — no safety action is taken here.
 */
void CAN_UpdateFrameRate(void) {
    uint32_t now = HAL_GetTick();
    /* Guard: avoid division-by-zero if called too soon after init.
     * rx_rate_tick is 0 on first call — initialise and return.        */
    if (can_stats.rx_rate_tick == 0) {
        can_stats.rx_rate_tick  = now;
        can_stats.rx_count_prev = can_stats.rx_count;
        return;
    }
    uint32_t elapsed = now - can_stats.rx_rate_tick;
    if (elapsed == 0) return;  /* Avoid division by zero */
    /* Compute frames received since last snapshot */
    uint32_t delta = can_stats.rx_count - can_stats.rx_count_prev;
    /* Normalise to 1-second rate (handles jitter if called at ±10 ms) */
    can_stats.rx_frames_per_sec = (delta * 1000U) / elapsed;
    /* Save snapshot for next call */
    can_stats.rx_count_prev = can_stats.rx_count;
    can_stats.rx_rate_tick  = now;
}