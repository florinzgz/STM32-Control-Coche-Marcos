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
#include <math.h>

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
CAN_InitDiag_t can_init_diag = {0};

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

/* Saturating increment for uint32_t counters.
 * Prevents wrap-around to 0 after 4 billion events, which would
 * clear the saturated diagnostic byte in CAN_SendStatusSafety.   */
static inline void sat_inc_u32(uint32_t *counter) {
    if (*counter < UINT32_MAX) (*counter)++;
}

/* Internal helper to send a CAN frame */
static HAL_StatusTypeDef TransmitFrame(uint32_t msg_id, uint8_t *payload, uint32_t len) {
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
    
    HAL_StatusTypeDef result = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_hdr, payload);
    
    if (result == HAL_OK) {
        sat_inc_u32(&can_stats.tx_count);
    } else {
        sat_inc_u32(&can_stats.tx_errors);
    }
    
    return result;
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

    /* Skip hardware activation if FDCAN peripheral init failed.
     * System continues without CAN — Safety_CheckCANTimeout() will
     * detect the missing heartbeat and keep the system in STANDBY.  */
    extern bool fdcan_init_ok;
    if (!fdcan_init_ok) {
        can_init_diag.hal_init = 1U;  /* Record: HAL_FDCAN_Init failed */
        return;
    }
    can_init_diag.hal_init = 0U;      /* HAL_FDCAN_Init succeeded      */

    /* Verify that the FDCAN kernel clock is sourced from PCLK1.
     * After reset FDCANSEL defaults to 00 = HSE (which is not enabled
     * in this project).  SystemClock_Config() must have already set it
     * to PCLK1 via RCC_FDCANCLKSOURCE_PCLK1 (CCIPR bits [25:24] = 10)
     * before this point.  A wrong source means the bit-timing registers
     * produce a different baud rate, causing ACK / stuff errors.      */
    can_init_diag.clk_ok =
        (__HAL_RCC_GET_FDCAN_SOURCE() == RCC_FDCANCLKSOURCE_PCLK1) ? 1U : 0U;

    if (!can_init_diag.clk_ok) {
        fdcan_init_ok = false;
        return;  /* Wrong kernel clock — CAN baud rate would be wrong */
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
        can_init_diag.started = 0U;
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
         *            bit 3-5: DS18B20 sensor count (0-5)                       */
        const TractionState_t *ts = Traction_GetState();
        uint8_t status_flags = 0;
        if (Startup_IsInhibited()) status_flags |= STATUS_FLAG_STARTUP_INHIBIT;
        if (ts->mode4x4)          status_flags |= STATUS_FLAG_MODE_4X4;
        if (ts->axisRotation)     status_flags |= STATUS_FLAG_TANK_TURN;
        status_flags |= (uint8_t)((Temperature_GetCount() & STATUS_FLAG_TEMP_COUNT_MASK)
                                  << STATUS_FLAG_TEMP_COUNT_SHIFT);

        uint8_t payload[5];
        payload[0] = heartbeat_counter++;
        payload[1] = (uint8_t)Safety_GetState();
        payload[2] = Safety_GetFaultFlags();
        payload[3] = (uint8_t)Safety_GetError();
        payload[4] = status_flags;

        TransmitFrame(CAN_ID_HEARTBEAT_STM32, payload, 5);
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

void CAN_SendStatusSafety(bool abs, bool tcs, uint8_t error_code) {
    uint8_t safety_data[5];
    
    safety_data[0] = abs ? 1 : 0;
    safety_data[1] = tcs ? 1 : 0;
    safety_data[2] = error_code;
    /* Byte 3: system state (SYS_STATE_*) for HMI display.
     * Byte 4: saturated CAN RX error count for diagnostics.
     * Backward compatible: ESP32 parsers that only read bytes 0–2
     * will ignore the additional payload (DLC ≥ 3 check passes).  */
    safety_data[3] = (uint8_t)Safety_GetState();
    safety_data[4] = (can_stats.rx_errors > 255) ? 255
                     : (uint8_t)can_stats.rx_errors;
    
    TransmitFrame(CAN_ID_STATUS_SAFETY, safety_data, 5);
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
 * @brief  Send explicit temperature sensor mapping to ESP32.
 *
 * Uses the same DS18B20 readings already acquired by Temperature_ReadAll().
 * The payload assigns an unambiguous meaning to each byte:
 *
 *   Byte 0: Motor FL temperature (°C, int8_t)
 *   Byte 1: Motor FR temperature (°C, int8_t)
 *   Byte 2: Motor RL temperature (°C, int8_t)
 *   Byte 3: Motor RR temperature (°C, int8_t)
 *   Byte 4: Ambient temperature  (°C, int8_t)
 *
 * Sensor index mapping: 0=FL, 1=FR, 2=RL, 3=RR, 4=Ambient.
 * Values are not filtered or recalculated — raw DS18B20 readings.
 * If a sensor is disabled in Service Mode the value is still reported.
 *
 * CAN ID: 0x206   DLC: 5   Rate: 1000 ms (1 Hz)
 */
void CAN_SendStatusTempMap(void) {
    uint8_t data[5];

    data[0] = (uint8_t)(int8_t)Temperature_Get(0);  /* FL  */
    data[1] = (uint8_t)(int8_t)Temperature_Get(1);  /* FR  */
    data[2] = (uint8_t)(int8_t)Temperature_Get(2);  /* RL  */
    data[3] = (uint8_t)(int8_t)Temperature_Get(3);  /* RR  */
    data[4] = (uint8_t)(int8_t)Temperature_Get(4);  /* AMB */

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
                if (msg_len >= 2) {
                    int16_t angle_raw = (int16_t)(rx_payload[0] | (rx_payload[1] << 8));
                    float requested_deg = (float)angle_raw / 10.0f;
                    float validated_deg = Safety_ValidateSteering(requested_deg);
                    Steering_SetAngle(validated_deg);
                } else {
                    sat_inc_u32(&can_stats.rx_errors);
                }
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
                    uint8_t mode_flags = rx_payload[0];
                    bool enable_4x4 = (mode_flags & 0x01) != 0;
                    bool tank_turn  = (mode_flags & 0x02) != 0;
                    bool mode_ok = false;
                    /* STM32 decides: mode change only allowed at low speed */
                    if (Safety_ValidateModeChange(enable_4x4, tank_turn)) {
                        Traction_SetMode4x4(enable_4x4);
                        Traction_SetAxisRotation(tank_turn);
                        mode_ok = true;
                    }

                    /* Byte 1 (optional): gear position (P/R/N/D).
                     * Backward compatible: if only 1 byte is sent, gear
                     * remains unchanged (defaults to FORWARD on init).
                     * Gear changes are only accepted at very low speed
                     * (same constraint as mode changes).                  */
                    bool gear_ok = true;
                    if (msg_len >= 2) {
                        uint8_t gear_raw = rx_payload[1];
                        if (gear_raw <= (uint8_t)GEAR_FORWARD_D2) {
                            GearPosition_t requested = (GearPosition_t)gear_raw;
                            /* Gear change only allowed near standstill */
                            float avg_spd = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                                             Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
                            avg_spd = sanitize_float(avg_spd, SANITIZE_SPEED_DEFAULT);
                            if (avg_spd <= 1.0f) {
                                Traction_SetGear(requested);
                            } else {
                                gear_ok = false;
                            }
                        } else {
                            gear_ok = false;
                        }
                    }

                    if (mode_ok && gear_ok) {
                        CAN_SendCommandAck(0x02, ACK_OK);
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
                        /* Clear error log (always safe) */
                        ErrorLog_Clear();
                        CAN_SendCommandAck(0x10, ACK_OK);
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