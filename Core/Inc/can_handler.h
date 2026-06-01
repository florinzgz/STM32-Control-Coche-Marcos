/**
  ****************************************************************************
  * @file    can_handler.h
  * @brief   CAN bus handler for ESP32-S3 HMI communication
  *          Protocol: 500 kbps, CAN 2.0A (11-bit IDs)
  ****************************************************************************
  */

#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can_init_diag.h"
#include <stdbool.h>
#include <stdint.h>

/* CAN test frame ID — used by CAN_TestTransmit() and the Rx test filter */
#define CAN_ID_TEST_FRAME   0x123

/* CAN Message IDs (ESP32 ↔ STM32) */
#define CAN_ID_HEARTBEAT_STM32    0x001  // STM32 → ESP32 (100ms, DLC 6: byte5=relay status)
#define CAN_ID_HEARTBEAT_ESP32    0x011  // ESP32 → STM32 (100ms)
#define CAN_ID_CMD_THROTTLE       0x100  // ESP32 → STM32 (50ms)
#define CAN_ID_CMD_STEERING       0x101  // ESP32 → STM32 (50ms)
#define CAN_ID_CMD_MODE           0x102  // ESP32 → STM32 (on-demand)
#define CAN_ID_CMD_RC_OVERRIDE    0x10A  // ESP32 → STM32 (50ms) RC override demand
                                          // DLC 5: byte0=flags (bit0=override_active),
                                          // byte1=throttle 0..100, byte2..3=int16 LE
                                          // steer 1/10°, byte4=seq.  See rc_arbiter.h.
                                          // 200 ms watchdog → failsafe to local pedal.
#define CAN_ID_CMD_LED            0x120  // ESP32 → STM32 (on-demand) LED relay control
#define CAN_ID_CMD_SYSTEM_SHUTDOWN 0x130 // ESP32 → STM32 (on-demand) pre-power-cut safe-state request
                                          // Payload: empty or 1 byte (ignored).
                                          // Idempotent. Reuses existing safety primitives;
                                          // forces PWM=0, EN=LOW, relays OFF, state=SAFE.
                                          // If the frame never arrives, behaviour is unchanged
                                          // (hardware delay relay still cuts power).
#define CAN_ID_STATUS_SPEED       0x200  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_CURRENT     0x201  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_TEMP        0x202  // STM32 → ESP32 (1000ms)
#define CAN_ID_STATUS_SAFETY      0x203  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_STEERING    0x204  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_TRACTION    0x205  // STM32 → ESP32 (100ms) per-wheel traction scale
#define CAN_ID_STATUS_TEMP_MAP    0x206  // STM32 → ESP32 (1000ms) explicit temp sensor map
#define CAN_ID_STATUS_BATTERY     0x207  // STM32 → ESP32 (100ms) battery 24V bus current + voltage
#define CAN_ID_OBSTACLE_DISTANCE  0x208  // ESP32 → STM32 (66ms) obstacle distance + zone + health
#define CAN_ID_OBSTACLE_SAFETY    0x209  // ESP32 → STM32 (100ms) obstacle safety state
#define CAN_ID_STATUS_LIGHTS      0x20A  // STM32 → ESP32 (1000ms) LED relay + light state
#define CAN_ID_DIAG_ERROR         0x300  // Both directions (on-demand)
#define CAN_ID_SERVICE_FAULTS     0x301  // STM32 → ESP32 (1000ms) fault bitmask
#define CAN_ID_SERVICE_ENABLED    0x302  // STM32 → ESP32 (1000ms) enabled bitmask
#define CAN_ID_SERVICE_DISABLED   0x303  // STM32 → ESP32 (1000ms) disabled bitmask
#define CAN_ID_ERROR_LOG_ENTRY    0x304  // STM32 → ESP32 (on-demand) error log entry
#define CAN_ID_ERROR_LOG_HEADER   0x305  // STM32 → ESP32 (1000ms) error log count + total
#define CAN_ID_DIAG_DEBOUNCE      0x306  // STM32 → ESP32 (1000ms) DWT-debounce filtered counts (4× wheel u16 LE)
#define CAN_ID_DIAG_DEBOUNCE_STEER 0x307 // STM32 → ESP32 (1000ms) DWT-debounce filtered count (steer u32 LE)
#define CAN_ID_DIAG_PEDAL_CAL     0x308  // STM32 → ESP32 (on-demand, 10 Hz for 1 s after QUERY) pedal calibration telemetry
#define CAN_ID_DIAG_I2C           0x309  // STM32 → ESP32 (1000ms) I2C topology diag: mux + per-channel INA226 health
#define CAN_ID_DIAG_CAN_META      0x30A  // STM32 → ESP32 (1000ms) CAN/0x309 delivery meta-diagnostic (additive)
#define CAN_ID_DIAG_I2C_SCAN      0x30B  // STM32 → ESP32 (on-demand) I2C service-mode scan report (additive)
#define CAN_ID_DIAG_FDCAN         0x30C  // STM32 → ESP32 (on-demand) FDCAN error-counter dump (additive)
#define CAN_ID_SERVICE_CMD              0x110  // ESP32 → STM32 (on-demand) module control
#define CAN_ID_CMD_SENSOR_MAP_TEMP      0x112  // ESP32 → STM32 (on-demand) DS18B20 physIdx→role map (DLC 5)
#define CAN_ID_CMD_ACK                  0x103  // STM32 → ESP32 (on-demand) command acknowledgment

/* Service command action codes (SERVICE_CMD byte 0) */
#define SERVICE_ACTION_DISABLE             0x00
#define SERVICE_ACTION_ENABLE              0x01
#define SERVICE_ACTION_RELAY_OVERRIDE      0xE0  /* Engineering relay override (byte1=mask) */
#define SERVICE_ACTION_RESET_STEERING_PID  0xF0
#define SERVICE_ACTION_RESET_WHEEL_SENSORS 0xF1
#define SERVICE_ACTION_RESET_INA226_SHUNTS 0xF2
#define SERVICE_ACTION_RESET_TRACTION_FORCE 0xF3
#define SERVICE_ACTION_RESET_STEERING_FORCE 0xF4
#define SERVICE_ACTION_CLEAR_ERROR_LOG     0xFE
#define SERVICE_ACTION_PEDAL_CAL           0xF5  /* Pedal endpoint calibration (byte1 = sub-opcode) */
#define SERVICE_ACTION_I2C_SERVICE         0xF6  /* I2C service-mode scan: probe mux/INA, SDA/SCL levels, recovery */
#define SERVICE_ACTION_FACTORY_RESTORE     0xFF

/* ---- Pedal-calibration sub-opcodes (byte1 when byte0 == 0xF5) ----
 * 0x01 CAPTURE_MIN    Capture current ADC as released endpoint (pending)
 * 0x02 CAPTURE_MAX    Capture current ADC as pressed  endpoint (pending)
 * 0x03 SAVE           Validate pending pair + persist to flash + apply
 * 0x04 RESET_DEFAULTS Erase flash slot + restore 150 / 2413
 * 0x05 QUERY          Request a 1 s burst of 0x308 telemetry at 10 Hz   */
#define PEDAL_CAL_OP_CAPTURE_MIN    0x01U
#define PEDAL_CAL_OP_CAPTURE_MAX    0x02U
#define PEDAL_CAL_OP_SAVE           0x03U
#define PEDAL_CAL_OP_RESET_DEFAULTS 0x04U
#define PEDAL_CAL_OP_QUERY          0x05U

/* Command ACK result codes (uint8_t) */
typedef enum {
    ACK_OK                  = 0,   /* Command accepted and applied           */
    ACK_REJECTED            = 1,   /* Command rejected (speed too high, etc) */
    ACK_INVALID             = 2,   /* Command payload invalid / malformed    */
    ACK_BLOCKED_BY_SAFETY   = 3    /* Command blocked by safety system state */
} CAN_AckResult_t;

/* Timeouts */
#define CAN_TIMEOUT_HEARTBEAT_MS  250    // Heartbeat timeout
#define CAN_TIMEOUT_OBSTACLE_MS   500    // Obstacle data timeout (fail-safe)

/* CAN bus-off recovery configuration */
#define CAN_BUSOFF_RETRY_INTERVAL_MS  500   /* Non-blocking retry interval      */
#define CAN_BUSOFF_MAX_RETRIES        10    /* Max recovery attempts before ERROR */

/* FIFO overflow escalation threshold.
 * After this many cumulative message-lost events, CAN_ProcessMessages()
 * escalates to DEGRADED_L1.  A single overflow is a transient; sustained
 * overflow indicates bus overload or a stuck processing loop.             */
#define CAN_FIFO_OVERFLOW_DEGRADE_THRESHOLD  5U

/* CAN Statistics */
typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t tx_errors;
    uint32_t rx_errors;
    uint32_t last_heartbeat_esp32;
    uint32_t busoff_count;                  /* Total bus-off events detected     */
    uint32_t fifo_overflow_count;           /* FIFO message-lost events          */
    /* ---- Frame health monitoring (Part 3) ---- */
    uint32_t rx_count_prev;                 /* rx_count snapshot at last 1-second tick */
    uint32_t rx_frames_per_sec;             /* Computed: frames received in last 1 s   */
    uint32_t rx_rate_tick;                  /* Timestamp of last FPS computation        */
} CAN_Stats_t;

/* CAN bus error diagnostics — readable via SWD debugger.
 * Updated every 10 ms by CAN_CheckBusOff().                          */
typedef struct {
    uint8_t  last_error_code;   /* FDCAN PSR.LEC: 0=none 1=stuff 2=form
                                 * 3=ack 4=bit1(rec) 5=bit0(dom) 6=CRC */
    uint8_t  error_passive;     /* 1 = Error Passive state              */
    uint8_t  bus_off;           /* 1 = Bus-Off state                    */
    uint8_t  warning;           /* 1 = error warning threshold exceeded */
    uint8_t  tec;               /* Transmit Error Counter (0-255)       */
    uint8_t  rec;               /* Receive Error Counter  (0-255)       */
    /* ---- TX validation (Part 5) ---- */
    uint8_t  tx_nack_flag;      /* 1 = repeated TX failures detected    */
    uint8_t  tx_consec_fail;    /* Consecutive TX failures (0-255)      */
} CAN_Diag_t;

/* ---- CAN/0x309 delivery meta-diagnostic (additive, report-only) ----
 * Answers the observability questions A–D from the 0x309 audit without
 * touching any control or safety path:
 *   A) diag309_call_count   — times CAN_SendI2CDiag() actually ran
 *   B) tick_1000ms_count    — iterations of the main-loop 1 Hz block
 *   C) diag309_tx_ok / err  — TransmitFrame() result for 0x309 specifically
 *   D) tx_fifo_full_drops   — frames dropped because the TX FIFO was full
 * All counters saturate; never wrap to 0.  Surfaced on CAN 0x30A.        */
typedef struct {
    uint32_t diag309_call_count;   /* A: CAN_SendI2CDiag() invocations     */
    uint32_t tick_1000ms_count;    /* B: 1 Hz scheduler-block iterations   */
    uint32_t diag309_tx_ok;        /* C: 0x309 queued to TX FIFO OK        */
    uint32_t diag309_tx_err;       /* C: 0x309 TransmitFrame() != HAL_OK   */
    uint32_t tx_fifo_full_drops;   /* D: any frame dropped (FIFO full)     */
} CAN_TxMeta_t;

/* CAN_InitDiag_t is defined in can_init_diag.h (included above) to allow
 * stm32g4xx_hal_msp.c to record MspInit diagnostics without depending on
 * the full CAN handler API.                                              */

/* Function prototypes */
void CAN_Init(void);
void CAN_TestTransmit(void);
void CAN_SendHeartbeat(void);
void CAN_SendStatusSpeed(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr);
void CAN_SendStatusCurrent(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr);
void CAN_SendStatusTemp(int8_t t1, int8_t t2, int8_t t3, int8_t t4, int8_t t5);
void CAN_SendStatusSafety(bool abs, bool tcs, uint8_t error_code,
                          uint8_t loop_peak_100us);
void CAN_SendStatusSteering(int16_t angle, bool calibrated);
void CAN_SendStatusTraction(void);
void CAN_SendStatusTempMap(void);
void CAN_SendStatusBattery(void);
void CAN_SendStatusLights(void);
void CAN_SendError(uint8_t error_code, uint8_t subsystem);
void CAN_SendDiagnosticEncoder(int32_t raw_count, int16_t delta);
void CAN_SendCommandAck(uint8_t cmd_id_low, CAN_AckResult_t result);
void CAN_SendServiceStatus(void);
void CAN_SendErrorLogHeader(void);
void CAN_SendDebounceDiag(void);    /* 1 Hz DWT-debounce filter EMI counters (0x306 + 0x307) */
void CAN_SendI2CDiag(void);         /* 1 Hz I2C topology diagnostic (0x309): mux + per-channel INA226 */
void CAN_SendCanMetaDiag(void);     /* 1 Hz CAN/0x309 delivery meta-diagnostic (0x30A) */
void CAN_SendI2CScanReport(void);   /* On-demand I2C service-mode scan report (0x30B) */
void CAN_SendFdcanDiag(void);       /* On-demand FDCAN error-counter dump (0x30C) */
void CAN_ProcessMessages(void);
bool CAN_IsESP32Alive(void);
bool CAN_IsGlobalSilent(void);
void CAN_CheckBusOff(void);
bool CAN_IsBusOff(void);
void CAN_UpdateFrameRate(void);     /* Call every ~1 s to compute rx FPS  */

/* Drives the on-demand 0x308 pedal-calibration telemetry burst.
 * Call once per 100 ms tick — the function is a no-op while no
 * burst is in progress, so it is safe to call unconditionally and
 * has zero impact on backward-compatible nodes that ignore 0x308. */
void CAN_PedalCalBurstUpdate(void);

/* Drives the cooperative non-blocking pedalcal capture FSM (R-1).
 * Call once per 50 ms main-loop tick, immediately after Pedal_Update().
 * No-op while the FSM is idle; while a CAPTURE_MIN/MAX is in flight
 * it takes one sample per tick (8 samples total), re-validates
 * safety, enforces a hard 450 ms timeout, and emits the deferred
 * ACK on completion.  Replaces the previous blocking HAL_Delay-based
 * sampler that starved CAN heartbeat TX and Safety_CheckCANTimeout(). */
void CAN_PedalCalCaptureTick(void);

/* LED relay states — front (PB10) and rear (PB11) — toggled via CAN 0x120 */
void LED_Relay_Set(bool on);          /* front relay */
bool LED_Relay_Get(void);             /* front relay state */
void LED_Relay_Rear_Set(bool on);     /* rear relay */
bool LED_Relay_Rear_Get(void);        /* rear relay state */

extern CAN_Stats_t    can_stats;
extern CAN_Diag_t     can_diag;
extern CAN_TxMeta_t   can_txmeta;
extern FDCAN_HandleTypeDef hfdcan1;

/* Debug-visible global CAN buffers (volatile for debugger inspection) */
extern volatile uint8_t               g_CAN_RxData[8];
extern volatile FDCAN_RxHeaderTypeDef  g_CAN_RxHeader;
extern volatile uint8_t               g_CAN_TxData[8];

#ifdef __cplusplus
}
#endif

#endif /* CAN_HANDLER_H */
