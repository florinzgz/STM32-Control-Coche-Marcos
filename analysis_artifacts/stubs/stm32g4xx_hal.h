/**
  * @file  stm32g4xx_hal.h  (HOST_TEST stub)
  * @brief Minimal HAL type definitions and no-op inline functions
  *        so that production sources (error_log.c, eps_params.c, …)
  *        compile and link on a host PC during unit-test builds.
  *
  *        This header is found via  -Ianalysis_artifacts/stubs  which
  *        the CI workflow already passes to gcc.
  */

#ifndef STM32G4xx_HAL_H
#define STM32G4xx_HAL_H

#include <stdint.h>
#include <stddef.h>

/* ---- Status enum ---- */
typedef enum {
    HAL_OK      = 0x00U,
    HAL_ERROR   = 0x01U,
    HAL_BUSY    = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

/* ---- Opaque peripheral handle types (fields unused in tests) ---- */
typedef struct { uint32_t _dummy; } ADC_HandleTypeDef;
typedef struct { uint32_t _dummy; } FDCAN_HandleTypeDef;
typedef struct { uint32_t _dummy; } I2C_HandleTypeDef;
typedef struct { uint32_t _dummy; } TIM_HandleTypeDef;
typedef struct { uint32_t _dummy; } IWDG_HandleTypeDef;

/* ---- Flash erase descriptor ---- */
typedef struct {
    uint32_t TypeErase;
    uint32_t Banks;
    uint32_t Page;
    uint32_t NbPages;
} FLASH_EraseInitTypeDef;

/* ---- Flash constants ---- */
#define FLASH_TYPEERASE_PAGES        0x00U
#define FLASH_BANK_1                 0x01U
#define FLASH_TYPEPROGRAM_DOUBLEWORD 0x00U

/* ---- Tick ---- */
static inline uint32_t HAL_GetTick(void)
{
    static uint32_t tick = 0;
    return tick += 10;
}

/* ---- Flash operations (no-ops, always succeed) ---- */
static inline HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
static inline HAL_StatusTypeDef HAL_FLASH_Lock(void)   { return HAL_OK; }

static inline HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type,
                                                   uint32_t addr,
                                                   uint64_t data)
{
    (void)type; (void)addr; (void)data;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e,
                                                   uint32_t *page_err)
{
    (void)e;
    if (page_err) *page_err = 0xFFFFFFFFU;
    return HAL_OK;
}

#endif /* STM32G4xx_HAL_H */
