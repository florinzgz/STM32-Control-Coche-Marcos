/**
  ****************************************************************************
  * @file    can_init_diag.h
  * @brief   FDCAN initialisation diagnostics — shared type definition.
  *
  *          Extracted from can_handler.h so that stm32g4xx_hal_msp.c can
  *          record MspInit diagnostics without pulling in the full CAN
  *          handler API (function prototypes, message-ID defines, etc.).
  ****************************************************************************
  */

#ifndef CAN_INIT_DIAG_H
#define CAN_INIT_DIAG_H

#include <stdint.h>

/* FDCAN initialisation sequence diagnostics — readable via SWD debugger.
 * Populated by MspInit and CAN_Init(); never modified at runtime after boot.
 * Each field stores the HAL return value (HAL_OK = 0) so the exact
 * step that failed can be identified without printf/UART.
 *
 * Debug console cheat-sheet (copy-paste into GDB / STM32CubeIDE):
 *   print can_init_diag
 *   print boot_phase            // 0=pre-GPIO … 5=main-loop
 *   print fdcan_init_ok
 *   print/x *(uint32_t*)0x40021088   // RCC->CCIPR  (FDCANSEL bits 25:24)
 *   print/x *(uint32_t*)0x40021058   // RCC->APB1ENR1 (FDCANEN bit 25)  */
typedef struct {
    uint8_t  hal_init;           /* HAL_FDCAN_Init          return value */
    uint8_t  filter_global;      /* ConfigGlobalFilter      return value */
    uint8_t  notify;             /* ActivateNotification    return value */
    uint8_t  start;              /* HAL_FDCAN_Start         return value */
    uint8_t  started;            /* 1 = FDCAN fully started, 0 = failed */
    uint8_t  clk_ok;             /* 1 = FDCANSEL == PCLK1, 0 = wrong   */
    uint8_t  cccr_init_ok;       /* 1 = CCCR.INIT cleared after start  */
    uint8_t  clk_reapplied;      /* 1 = PCLK1 was re-applied by CAN_Init */
    uint8_t  retries;            /* Number of full init retries used    */
    uint8_t  timeout_flag;       /* 1 = CCCR poll timed out in MspInit */
    uint8_t  msp_clk_ok;        /* 1 = APB1ENR1.FDCANEN verified in MspInit */
    uint8_t  msp_ccipr_ok;      /* 1 = FDCANSEL==PCLK1 verified in MspInit */
    uint32_t ccipr_raw;          /* Raw RCC_CCIPR snapshot for debugging */
    uint8_t  msp_call_count;     /* Incremented each HAL_FDCAN_MspInit call */
    uint32_t cccr_last;          /* Last raw CCCR reading from retry loop   */
} CAN_InitDiag_t;

extern volatile CAN_InitDiag_t can_init_diag;

#endif /* CAN_INIT_DIAG_H */
