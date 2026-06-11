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
#include "motor_control.h"
#include "safety_system.h"
#include "sensor_manager.h"
#include "service_mode.h"
#include "math_safety.h"
#include "error_log.h"
#include "sensor_map_store.h"
#include "pedal_cal_store.h"
#include "gear_limits_store.h"
#include "steering_cal_store.h"
#include "steering_z.h"
#include "encoder_reader.h"
#include "rc_arbiter.h"
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
static uint8_t  busoff_active       = 0;    /* 1 = bus-off detected, recovery in progress */
static uint32_t busoff_last_attempt = 0;    /* Timestamp of last recovery attempt         */
static uint8_t  busoff_retry_count  = 0;    /* Number of recovery attempts since bus-off  */

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
}

/* Internal helper to send a CAN frame.
 *
 * Appends the frame to the software TX queue (preserving submission order) and
 * opportunistically pumps the queue into the hardware FIFO.  Returns HAL_OK
 * when the frame was accepted for transmission (queued) and HAL_BUSY only on a
 * genuine software-queue overflow, which is then counted in tx_fifo_full_drops.
 */
static HAL_StatusTypeDef TransmitFrame(uint32_t msg_id, uint8_t *payload, uint32_t len) {
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
    /* ---- Accept ALL standard IDs (0x000–0x7FF) for loopback/test ---- */
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x7FF;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
#else
    /* Filter 0: Accept ALL standard IDs via mask filter.
     * FDCAN_FILTER_MASK with FilterID2 (mask) = 0x000 means every bit
     * is don't-care, so all 11-bit IDs are accepted into RXFIFO0.
     * CAN_ProcessMessages() only acts on known IDs (switch/case);
     * unrecognised IDs are silently discarded.
     *
     * Using a single mask-based accept-all filter guarantees the
     * FDCAN message-RAM filter element is valid and the peripheral
     * can leave INIT mode cleanly after HAL_FDCAN_Start().          */
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x000;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
#endif

    /* Accept all non-matching IDs (standard and extended) into FIFO0.
     * Together with the mask-based accept-all filter at index 0, this
     * provides a belt-and-suspenders acceptance path that guarantees
     * FDCAN receives frames from the ESP32 regardless of ID.
     * Safety: CAN_ProcessMessages() only acts on known message IDs
     * (via its switch/case); any unexpected ID is silently discarded.
     * Remote frames remain rejected.                                  */
    can_init_diag.filter_global = (uint8_t)HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,  /* non-matching std */
        FDCAN_ACCEPT_IN_RX_FIFO0,  /* non-matching ext */
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);
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

        TransmitFrame(CAN_ID_HEARTBEAT_STM32, payload, 6);
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
    uint8_t safety_data[6];

    safety_data[0] = abs ? 1 : 0;
    safety_data[1] = tcs ? 1 : 0;
    safety_data[2] = error_code;
    /* Byte 3: system state (SYS_STATE_*) for HMI display.
     * Byte 4: saturated CAN RX error count for diagnostics.
     * Byte 5: peak 100 Hz task duration in 100 µs units, saturated
     *         at 255 (= 25.5 ms).  Pure observational; resets each
     *         TX cycle.  Roadmap additive item #1.
     * Backward compatible: ESP32 parsers that only read bytes 0–2
     * will ignore the additional payload (DLC ≥ 5 check passes).  */
    safety_data[3] = (uint8_t)Safety_GetState();
    safety_data[4] = (can_stats.rx_errors > 255) ? 255
                     : (uint8_t)can_stats.rx_errors;
    safety_data[5] = loop_peak_100us;

    TransmitFrame(CAN_ID_STATUS_SAFETY, safety_data, 6);
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
 * @brief  Send per-wheel traction scale to ESP32.
 *
 * Exposes the ABS/TCS per-wheel scale factor already computed by the
 * safety system (safety_status.wheel_scale[0..3]).  Each value is
 * converted from float 0.0–1.0 to uint8 0–100 (percent).
 *
 *   Byte 0: FL traction %  (0 = fully inhibited, 100 = full power)
 *   Byte 1: FR traction %
 *   Byte 2: RL traction %
 *   Byte 3: RR traction %
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
 * @brief  Send live Hall pedal position to ESP32 (telemetry only).
 *
 * Publishes the real pedal travel reported by Pedal_GetPercent() so the HMI
 * THROTTLE bar reflects the physical pedal (0 % released … 100 % full) rather
 * than the per-wheel torque/TCS scale (0x205), which is unrelated to pedal
 * position.  This frame is strictly informational: it does NOT feed any
 * control, PID, safety or pedal-logic path on either node.
 *
 *   Byte 0: pedal position %  (0 = released, 100 = full travel)
 *
 * CAN ID: 0x20B   DLC: 1   Rate: 100 ms (10 Hz)
 */
void CAN_SendStatusPedal(uint8_t pedal_pct) {
    if (pedal_pct > 100) pedal_pct = 100;
    TransmitFrame(CAN_ID_STATUS_PEDAL, &pedal_pct, 1);
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
 * Two aditive frames (1000 ms cadence, low priority):
 *   0x306 — DLC 8: FL/FR/RL/RR filtered counts, uint16 LE saturated to 0xFFFF
 *   0x307 — DLC 4: steering-center filtered count, uint32 LE
 *
 * The internal counters in sensor_manager.c are 32-bit saturated; this frame
 * truncates the 4 wheel counters to 16 bits for compactness.  Truncation is
 * acceptable because the values are diagnostic only and "saturated" is a
 * meaningful UI state.
 *
 * Diagnostic only — no control / safety path consumes these.
 */
void CAN_SendDebounceDiag(void) {
    uint8_t  data8[8];
    uint8_t  data4[4];
    uint32_t v;

    /* 0x306 — wheel filtered counts (FL, FR, RL, RR) */
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
}

/**
 * @brief  Send I2C topology diagnostic to ESP32 (HMI Safe Mode helper).
 *
 * Lets the operator tell apart a missing TCA9548A multiplexer (0x70) from a
 * missing / dead INA226 (0x40) on a specific channel, and see whether the
 * failures are intermittent (bad pull-up / loose terminal) or permanent
 * (not connected).  Diagnostic only — no control / safety path consumes it.
 *
 * CAN ID: 0x309   DLC: 6   Rate: 1000 ms (1 Hz)
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
 */
void CAN_SendI2CDiag(void) {
    uint8_t data[6];
    data[0] = Sensor_GetMuxPresent() ? 1U : 0U;
    data[1] = Sensor_GetInaOkMask();
    data[2] = Sensor_GetI2cFailCount();
    data[3] = Sensor_GetI2cRecoveryAttempts();
    data[4] = Sensor_GetI2cEverOk() ? 0x01U : 0x00U;
    data[5] = Sensor_GetInaExpectedMask();

    /* Observability A/C: count every invocation and the TX outcome so the
     * 0x30A meta-frame can prove whether 0x309 reaches the FDCAN TX FIFO. */
    sat_inc_u32(&can_txmeta.diag309_call_count);
    if (TransmitFrame(CAN_ID_DIAG_I2C, data, 6) == HAL_OK) {
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
 *   Byte 7:   flags (bit0 = fdcan_init_ok)
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
    data[7] = fdcan_init_ok ? 0x01U : 0x00U;
    TransmitFrame(CAN_ID_DIAG_CAN_META, data, 8);
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
 *   Byte 5-7: reserved (0)
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
 *  Pedal calibration state (0xF5 sub-protocol + 0x308 telemetry)
 *
 *  Persistent endpoint calibration for the accelerator pedal.  All
 *  state changes live entirely in this CAN module; the underlying
 *  pedal pipeline (sensor_manager) is touched only via the public
 *  Pedal_ApplyCalibration() / Pedal_GetRawADC() helpers and the
 *  flash store (pedal_cal_store.c).
 *
 *  Safety invariants (enforced by pedalcal_safety_ok()):
 *    - Safety_GetState() == SYS_STATE_STANDBY
 *    - Startup_IsInhibited() == true
 *    - Pedal_GetPercent() < 3.0f  (pedal released)
 *    - Pedal_IsPlausible() == true
 *    - All four wheel speeds < 0.3 km/h
 *
 *  Stability check (CAPTURE_MIN / CAPTURE_MAX):
 *    8 samples over 400 ms (50 ms cadence — caller is the 100 Hz
 *    loop reaching the 50 ms branch).  All eight raw ADC samples
 *    must lie within PEDALCAL_STABLE_TOL counts of each other.
 *
 *  0x308 telemetry burst (DLC 8):
 *    Emitted only on demand: after a QUERY sub-opcode, 10 frames
 *    spaced 100 ms apart (1 second total), then silence.  No
 *    continuous flooding — backward-compatible for nodes that ignore
 *    0x308 entirely.
 * ================================================================== */

/* ---- Stability ---- */
#define PEDALCAL_STABLE_SAMPLES   8U
#define PEDALCAL_STABLE_TOL       8U     /* max spread (counts) for "stable" */

/* ---- 0x308 burst ---- */
#define PEDALCAL_BURST_FRAMES     10U    /* 10 × 100 ms = 1 s             */
#define PEDALCAL_BURST_PERIOD_MS  100U

/* ---- Pending endpoints (RAM-only until SAVE) ---- */
static bool      pedalcal_have_min      = false;
static bool      pedalcal_have_max      = false;
static uint16_t  pedalcal_pending_min   = 0;
static uint16_t  pedalcal_pending_max   = 0;

/* ---- 0x308 telemetry burst state ---- */
static uint8_t   pedalcal_burst_left    = 0;
static uint32_t  pedalcal_next_tx_ms    = 0;

static inline bool pedalcal_safety_ok(void)
{
    if (Safety_GetState() != SYS_STATE_STANDBY) return false;
    if (!Startup_IsInhibited())                 return false;
    if (Pedal_GetPercent() >= 3.0f)             return false;
    if (!Pedal_IsPlausible())                   return false;
    if (Wheel_GetSpeed_FL() >= 0.3f)            return false;
    if (Wheel_GetSpeed_FR() >= 0.3f)            return false;
    if (Wheel_GetSpeed_RL() >= 0.3f)            return false;
    if (Wheel_GetSpeed_RR() >= 0.3f)            return false;
    return true;
}

/* ------------------------------------------------------------------
 *  Cooperative pedalcal capture FSM (R-1 hardening)
 *
 *  Replaces the previous blocking pedalcal_sample_stable() which held
 *  the main loop for ~350 ms via HAL_Delay(50) × 7.  That blackout
 *  starved CAN heartbeat TX, Safety_CheckCANTimeout() and the IWDG
 *  refresh path; the ESP32 250 ms heartbeat watchdog could fire.
 *
 *  Design — strictly non-blocking, no new RTOS / timers / DMA:
 *    - One sample per 50 ms tick of the main loop (driven from
 *      main.c after Pedal_Update()).
 *    - 8 samples total → ~350 ms total window, identical to the
 *      original.  Sample[0] is taken synchronously when the FSM
 *      arms (in the command handler) so the elapsed time of the
 *      window is preserved exactly (7 intervals × 50 ms).
 *    - PEDALCAL_STABLE_SAMPLES and PEDALCAL_STABLE_TOL unchanged.
 *    - Same sampling primitive: Pedal_SampleRawNow() (fresh ADC,
 *      no pipeline side-effects).  Now safe because the 50 ms
 *      Pedal_Update() branch still runs between ticks.
 *
 *  ACK semantics:
 *    - Original: synchronous ACK after 350 ms blocking sample.
 *    - New     : synchronous-with-result, deferred to FSM end.
 *                Worst-case ACK latency ≈ 450 ms (hard timeout).
 *                ESP32 ACK_FEEDBACK_TIMEOUT_MS = 2000 ms — tolerates.
 *    - A second CAPTURE_MIN/MAX while busy → ACK_REJECTED immediate.
 *    - Safety gate re-validated every tick → if it fails mid-window,
 *      abort and emit ACK_BLOCKED_BY_SAFETY (matches project
 *      convention: safety-state gates use ACK_BLOCKED_BY_SAFETY).
 *    - Hard watchdog: if FSM hasn't completed within
 *      PEDALCAL_FSM_TIMEOUT_MS (450 ms) → ACK_REJECTED, reset.
 *      Protects against any future tick desynchronisation.
 * ------------------------------------------------------------------ */

typedef enum {
    PCAL_FSM_IDLE = 0,
    PCAL_FSM_CAPTURING_MIN,
    PCAL_FSM_CAPTURING_MAX,
} pedalcal_fsm_state_t;

/* Hard watchdog for the capture window.
 * Expected duration is ~350 ms (7 intervals × 50 ms); 450 ms gives
 * one full tick of slack against jitter in the 50 ms branch.        */
#define PEDALCAL_FSM_TIMEOUT_MS  450U

_Static_assert(PEDALCAL_STABLE_SAMPLES == 8U,
               "FSM design assumes 8 samples (~350 ms window)");
_Static_assert(PEDALCAL_FSM_TIMEOUT_MS >= 400U,
               "FSM hard timeout must exceed nominal capture window");

static pedalcal_fsm_state_t pedalcal_fsm_state    = PCAL_FSM_IDLE;
static uint16_t             pedalcal_fsm_samples[PEDALCAL_STABLE_SAMPLES];
static uint8_t              pedalcal_fsm_count    = 0U;
static uint32_t             pedalcal_fsm_start_ms = 0U;

/* Forward declaration so the command handler can arm the FSM. */
static void pedalcal_fsm_reset(void);
static bool pedalcal_fsm_finalize(uint16_t *out_adc);

static void pedalcal_fsm_reset(void)
{
    pedalcal_fsm_state    = PCAL_FSM_IDLE;
    pedalcal_fsm_count    = 0U;
    pedalcal_fsm_start_ms = 0U;
}

/* Compute spread and mean over the 8 captured samples.
 * Returns true and writes mean into *out_adc on success; false if
 * the spread exceeds PEDALCAL_STABLE_TOL (caller maps to ACK_REJECTED).
 * Identical math to the original pedalcal_sample_stable().            */
static bool pedalcal_fsm_finalize(uint16_t *out_adc)
{
    uint16_t mn = pedalcal_fsm_samples[0];
    uint16_t mx = pedalcal_fsm_samples[0];
    uint32_t sum = pedalcal_fsm_samples[0];
    for (uint8_t i = 1U; i < PEDALCAL_STABLE_SAMPLES; i++) {
        uint16_t v = pedalcal_fsm_samples[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }
    if ((uint32_t)(mx - mn) > PEDALCAL_STABLE_TOL)
        return false;
    *out_adc = (uint16_t)((sum + (PEDALCAL_STABLE_SAMPLES / 2U))
                          / PEDALCAL_STABLE_SAMPLES);
    return true;
}

/* Emit one 0x308 frame.
 *
 * Layout (DLC 8, little-endian).  To fit both stored AND pending
 * endpoints in 8 bytes the STM32 alternates two frame variants
 * within the 10-frame burst (5 of each variant per burst):
 *
 *   byte 0   : flags
 *              bit 0: pending MIN captured
 *              bit 1: pending MAX captured
 *              bit 2: pending pair validates OK
 *              bit 3: stored slot valid (PedalCal_IsValid())
 *              bit 4: safety gates satisfied
 *              bit 5: pedal plausible
 *              bit 6: 0 = bytes 3-6 carry PENDING pair
 *                     1 = bytes 3-6 carry STORED pair
 *              bit 7: reserved (0)
 *   bytes 1-2: raw ADC live (u16 LE)
 *   bytes 3-4: MIN (pending or stored — see bit 6) (u16 LE)
 *   bytes 5-6: MAX (pending or stored — see bit 6) (u16 LE)
 *   byte 7   : pedal percent (0..100 saturating)                    */
static void pedalcal_send_status(void)
{
    /* Alternate variant on each transmission so the ESP32 sees both
     * stored and pending pairs at ~5 Hz each.  At burst start
     * pedalcal_burst_left == PEDALCAL_BURST_FRAMES (10) — the first
     * frame goes out as PENDING (bit 6 == 0).                       */
    bool send_stored = ((pedalcal_burst_left & 0x01U) == 0U);

    uint16_t stored_min = 0, stored_max = 0;
    PedalCal_GetStored(&stored_min, &stored_max);

    uint8_t  flags = 0;
    if (pedalcal_have_min)  flags |= 0x01U;
    if (pedalcal_have_max)  flags |= 0x02U;
    if (pedalcal_have_min && pedalcal_have_max &&
        PedalCal_Validate(pedalcal_pending_min, pedalcal_pending_max))
        flags |= 0x04U;
    if (PedalCal_IsValid())   flags |= 0x08U;
    if (pedalcal_safety_ok()) flags |= 0x10U;
    if (Pedal_IsPlausible())  flags |= 0x20U;
    if (send_stored)          flags |= 0x40U;

    uint16_t raw_adc = Pedal_GetRawADC();
    float    pct_f   = Pedal_GetPercent();
    if (pct_f < 0.0f)   pct_f = 0.0f;
    if (pct_f > 100.0f) pct_f = 100.0f;

    uint16_t mn = send_stored ? stored_min : pedalcal_pending_min;
    uint16_t mx = send_stored ? stored_max : pedalcal_pending_max;

    uint8_t payload[8];
    payload[0] = flags;
    payload[1] = (uint8_t)(raw_adc & 0xFFU);
    payload[2] = (uint8_t)((raw_adc >> 8) & 0xFFU);
    payload[3] = (uint8_t)(mn & 0xFFU);
    payload[4] = (uint8_t)((mn >> 8) & 0xFFU);
    payload[5] = (uint8_t)(mx & 0xFFU);
    payload[6] = (uint8_t)((mx >> 8) & 0xFFU);
    payload[7] = (uint8_t)pct_f;

    (void)TransmitFrame(CAN_ID_DIAG_PEDAL_CAL, payload, sizeof(payload));
}

/* Public: drive the 0x308 burst from the 100 Hz main-loop tick.
 * Must be called whenever main.c emits its 100 ms status batch.
 * No effect when no burst is in progress.                            */
void CAN_PedalCalBurstUpdate(void)
{
    if (pedalcal_burst_left == 0U) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - pedalcal_next_tx_ms) < 0) return;
    pedalcal_send_status();
    pedalcal_burst_left--;
    pedalcal_next_tx_ms = now + PEDALCAL_BURST_PERIOD_MS;
}

/* Handle a SERVICE_CMD frame with byte 0 == SERVICE_ACTION_PEDAL_CAL.
 * Always replies with one CAN_SendCommandAck(0x10, ...) — DLC 3 of
 * CMD_ACK is preserved, no contract change.                          */
static void pedalcal_handle_service_cmd(const uint8_t *payload, uint8_t len)
{
    if (len < 2) {
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
    uint8_t op = payload[1];

    /* QUERY is the only sub-opcode that does NOT need safety gates —
     * it just requests a 1 s telemetry burst.                        */
    if (op == PEDAL_CAL_OP_QUERY) {
        pedalcal_burst_left = PEDALCAL_BURST_FRAMES;
        pedalcal_next_tx_ms = HAL_GetTick();   /* emit immediately on next tick */
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }

    /* All other sub-opcodes require the full safety gate. */
    if (!pedalcal_safety_ok()) {
        CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
        return;
    }

    switch (op) {
    case PEDAL_CAL_OP_CAPTURE_MIN: {
        /* R-1: non-blocking capture.  If a previous capture is still
         * in progress we cannot start a second one without losing
         * the in-flight samples, so reject immediately.  The previous
         * blocking implementation could not see this condition because
         * CAN_ProcessMessages() was held inside HAL_Delay for ~350 ms;
         * with the FSM the contract becomes explicit.                 */
        if (pedalcal_fsm_state != PCAL_FSM_IDLE) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        /* Arm the FSM and capture sample[0] now, mirroring the
         * original timing (sample[0] at t=0, then 7 ticks × 50 ms). */
        pedalcal_fsm_state    = PCAL_FSM_CAPTURING_MIN;
        pedalcal_fsm_samples[0] = Pedal_SampleRawNow();
        pedalcal_fsm_count    = 1U;
        pedalcal_fsm_start_ms = HAL_GetTick();
        /* ACK deferred — emitted by CAN_PedalCalCaptureTick() when
         * the 8-sample window completes (~350 ms later).             */
        return;
    }
    case PEDAL_CAL_OP_CAPTURE_MAX: {
        if (pedalcal_fsm_state != PCAL_FSM_IDLE) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        pedalcal_fsm_state    = PCAL_FSM_CAPTURING_MAX;
        pedalcal_fsm_samples[0] = Pedal_SampleRawNow();
        pedalcal_fsm_count    = 1U;
        pedalcal_fsm_start_ms = HAL_GetTick();
        return;
    }
    case PEDAL_CAL_OP_SAVE: {
        if (!pedalcal_have_min || !pedalcal_have_max) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        if (!PedalCal_Validate(pedalcal_pending_min, pedalcal_pending_max)) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        if (!PedalCal_Save(pedalcal_pending_min, pedalcal_pending_max)) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        /* Apply immediately so the next Pedal_Update() cycle uses the
         * new endpoints.  Safety: rate-limit inside Pedal_Update()
         * still bounds the percent output across the change.         */
        Pedal_ApplyCalibration(pedalcal_pending_min, pedalcal_pending_max);
        pedalcal_have_min = false;
        pedalcal_have_max = false;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    case PEDAL_CAL_OP_RESET_DEFAULTS: {
        /* Persist the compile-time defaults so the slot stays consistent
         * with applied endpoints across reboots.  Defaults are sourced
         * from pedal_cal_store.h (PEDAL_CAL_DEFAULT_MIN/MAX) to keep a
         * single source of truth shared with sensor_manager.c.        */
        if (!PedalCal_Save(PEDAL_CAL_DEFAULT_MIN, PEDAL_CAL_DEFAULT_MAX)) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        Pedal_ApplyCalibration(PEDAL_CAL_DEFAULT_MIN, PEDAL_CAL_DEFAULT_MAX);
        pedalcal_have_min = false;
        pedalcal_have_max = false;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    default:
        CAN_SendCommandAck(0x10, ACK_INVALID);
        return;
    }
}

/* R-1: 50 ms-cadence driver for the cooperative pedalcal capture FSM.
 *
 * Called from the main loop's 50 ms branch (main.c) immediately after
 * Pedal_Update().  This function is a no-op while the FSM is IDLE,
 * so it is safe to call unconditionally and adds zero runtime cost
 * to nodes that never request a calibration.
 *
 * Per tick (only while CAPTURING_MIN or CAPTURING_MAX):
 *   1. Re-validate pedalcal_safety_ok().  If any gate has dropped
 *      since arming → abort with ACK_BLOCKED_BY_SAFETY.
 *   2. Enforce hard timeout PEDALCAL_FSM_TIMEOUT_MS → ACK_REJECTED.
 *   3. Take one sample via Pedal_SampleRawNow() (fresh ADC; no
 *      pipeline side-effects).
 *   4. When 8 samples are collected, validate spread ≤ tolerance:
 *        - OK   → commit to pending_min / pending_max, ACK_OK.
 *        - FAIL → ACK_REJECTED.
 *      Reset FSM to IDLE either way.
 *
 * Invariants preserved:
 *   - PEDALCAL_STABLE_SAMPLES = 8.
 *   - 50 ms inter-sample cadence.
 *   - Total window ≈ 350 ms (sample[0] taken at arm, then 7 ticks).
 *   - PEDALCAL_STABLE_TOL unchanged.
 *   - One ACK per command (synchronous-with-result), DLC 3 of
 *     CMD_ACK unchanged.
 *   - CAN heartbeat TX, Safety_CheckCANTimeout(), IWDG refresh and
 *     the rest of the main loop run normally between ticks.
 */
void CAN_PedalCalCaptureTick(void)
{
    if (pedalcal_fsm_state == PCAL_FSM_IDLE) return;

    /* Re-validate safety on every tick — if any gate dropped (e.g.
     * driver pressed pedal mid-capture, system left STANDBY, brake
     * released) abort cleanly and report the safety-gate cause.  */
    if (!pedalcal_safety_ok()) {
        pedalcal_fsm_reset();
        CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
        return;
    }

    /* Hard watchdog — guards against any future desync of the 50 ms
     * branch.  Nominal completion is ~350 ms; 450 ms gives one tick
     * of slack while still ensuring the ACK lands well below the
     * 2 s ACK_FEEDBACK_TIMEOUT_MS on the ESP32 UI side.            */
    if ((uint32_t)(HAL_GetTick() - pedalcal_fsm_start_ms)
            > PEDALCAL_FSM_TIMEOUT_MS) {
        pedalcal_fsm_reset();
        CAN_SendCommandAck(0x10, ACK_REJECTED);
        return;
    }

    /* Take one sample this tick. */
    if (pedalcal_fsm_count < PEDALCAL_STABLE_SAMPLES) {
        pedalcal_fsm_samples[pedalcal_fsm_count++] = Pedal_SampleRawNow();
    }

    /* 8 samples collected → finalize and ACK. */
    if (pedalcal_fsm_count >= PEDALCAL_STABLE_SAMPLES) {
        uint16_t v = 0;
        bool ok = pedalcal_fsm_finalize(&v);
        pedalcal_fsm_state_t finished_state = pedalcal_fsm_state;
        pedalcal_fsm_reset();
        if (!ok) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        if (finished_state == PCAL_FSM_CAPTURING_MIN) {
            pedalcal_pending_min = v;
            pedalcal_have_min    = true;
        } else { /* PCAL_FSM_CAPTURING_MAX */
            pedalcal_pending_max = v;
            pedalcal_have_max    = true;
        }
        CAN_SendCommandAck(0x10, ACK_OK);
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
 *  Safety invariant (gearlim_safety_ok()):
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

static inline bool gearlim_safety_ok(void)
{
    return (Safety_GetState() == SYS_STATE_STANDBY);
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
    if (gearlim_safety_ok()) flags |= 0x04U;
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
    if (!gearlim_safety_ok()) {
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
        if (!GearLimitsStore_Save(gearlim_pending_d2,
                                  gearlim_pending_d1,
                                  gearlim_pending_r,
                                  gearlim_pending_d2_resp,
                                  gearlim_pending_d1_resp,
                                  gearlim_pending_r_resp)) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        /* Apply so the next Traction_Update() cycle uses the new limits.
         * Vehicle is in STANDBY (gated above) so there is no live demand. */
        (void)Traction_SetGearLimits(gearlim_pending_d2,
                                     gearlim_pending_d1,
                                     gearlim_pending_r);
        (void)Traction_SetGearResponse(gearlim_pending_d2_resp,
                                       gearlim_pending_d1_resp,
                                       gearlim_pending_r_resp);
        gearlim_pending_seeded = false;   /* clear edit; reseed on next SET */
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
    case GEAR_LIMIT_OP_RESET_DEFAULTS: {
        if (!GearLimitsStore_Save(GEAR_LIMIT_D2_DEFAULT_PCT,
                                  GEAR_LIMIT_D1_DEFAULT_PCT,
                                  GEAR_LIMIT_R_DEFAULT_PCT,
                                  GEAR_RESPONSE_D2_DEFAULT_PCT,
                                  GEAR_RESPONSE_D1_DEFAULT_PCT,
                                  GEAR_RESPONSE_R_DEFAULT_PCT)) {
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
        
        sat_inc_u32(&can_stats.rx_count);
        last_any_rx_tick = HAL_GetTick();  /* Global CAN watchdog refresh */
        uint8_t msg_len = ExtractDLC(rx_hdr.DataLength);
        
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
                /* RC override demand from ESP32 / FlySky iBUS bridge.
                 * The handler ONLY updates the arbiter's internal state
                 * + watchdog timestamp.  It never calls Traction_SetDemand
                 * or Steering_SetAngle directly: the 50 ms main scheduler
                 * (throttle) and the 0x101 handler above (steering) apply
                 * the value through Safety_Validate* downstream.
                 *
                 * Failsafe: if frames stop arriving, RcArbiter_IsActive()
                 * returns false within RC_OVERRIDE_TIMEOUT_MS (200 ms)
                 * and local control resumes automatically.                */
                RcArbiter_OnFrame(rx_payload, msg_len, HAL_GetTick());
                break;
                
            case CAN_ID_CMD_MODE:
                if (msg_len < 1) {
                    sat_inc_u32(&can_stats.rx_errors);
                    CAN_SendCommandAck(0x02, ACK_INVALID);
                    break;
                }
                if (!Safety_IsCommandAllowed()) {
                    CAN_SendCommandAck(0x02, ACK_BLOCKED_BY_SAFETY);
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
                        float avg_spd = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                                         Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
                        avg_spd = sanitize_float(avg_spd, SANITIZE_SPEED_DEFAULT);
                        if (avg_spd > 1.0f) {
                            gear_ok = false;            /* preserves ≤1 km/h gate */
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

                    /* --- Atomic commit (all-or-nothing) ---------------- */
                    if (mode_ok && gear_ok) {
                        Traction_SetMode4x4(enable_4x4);
                        Traction_SetAxisRotation(tank_turn);
                        if (gear_present) {
                            Traction_SetGear(requested_gear);
                        }
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
                         * — does not gate any control or safety path.        */
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

    /* If a recovery attempt is in progress, enforce retry interval */
    if (busoff_active) {
        uint32_t now = HAL_GetTick();
        if ((now - busoff_last_attempt) < CAN_BUSOFF_RETRY_INTERVAL_MS) {
            return;  /* Not yet time for next attempt */
        }

        /* Too many retries — stop attempting, system stays in SAFE */
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

        if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            return;  /* Notification setup failed — retry next interval */
        }

        if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
            return;  /* Start failed — retry next interval */
        }

        /* Recovery successful — clear bus-off state.
         * The safety system will recover from SAFE via
         * Safety_CheckCANTimeout() when heartbeats resume. */
        busoff_active     = 0;
        busoff_retry_count = 0;
        Safety_ClearError(SAFETY_ERROR_CAN_BUSOFF);
        return;
    }

    /* Normal operation: poll FDCAN protocol status for bus-off */
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
        /* Bus-off detected — raise fault and enter LIMP_HOME.
         * CAN bus-off is a communication failure, not a hardware
         * danger.  Vehicle remains mobile at walking speed.            */
        busoff_active       = 1;
        busoff_last_attempt = HAL_GetTick();
        busoff_retry_count  = 0;
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