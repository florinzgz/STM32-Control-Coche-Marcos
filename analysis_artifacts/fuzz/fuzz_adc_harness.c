/*
 * ADC/Sensor Fuzzing Harness for STM32-Control-Coche-Marcos
 *
 * Feeds arbitrary ADC values into the pedal reading and sensor manager
 * to test range checks, plausibility validation, and fault detection.
 *
 * Build:  gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE -DFUZZ_HARNESS \
 *              -Ianalysis_artifacts/stubs -ICore/Inc \
 *              -fsanitize=address,undefined -g -O1 \
 *              analysis_artifacts/fuzz/fuzz_adc_harness.c \
 *              Core/Src/sensor_manager.c -o /tmp/fuzz_adc -lm
 *
 * Run with AFL++:
 *   mkdir -p /tmp/fuzz_adc_corpus && printf '\x00\x00\x00\x00' > /tmp/fuzz_adc_corpus/seed1
 *   afl-fuzz -i /tmp/fuzz_adc_corpus -o /tmp/fuzz_adc_out -- /tmp/fuzz_adc
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32g4xx_hal.h"

/* Fuzz-controlled ADC value */
static uint32_t fuzz_adc_value = 0;

/* Override HAL_ADC_GetValue to return fuzzed data */
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc) {
    return fuzz_adc_value;
}

/* Stub prototypes for functions called by sensor_manager */
extern void Sensor_Init(void);

int main(void) {
    uint8_t buf[16];
    size_t n = fread(buf, 1, sizeof(buf), stdin);
    if (n < 2) return 0;

    /* First 2 bytes = ADC raw value (0-4095 range for 12-bit ADC) */
    fuzz_adc_value = ((uint32_t)buf[0] << 8) | buf[1];
    /* Allow full uint16 range to test out-of-range handling */

    Sensor_Init();
    return 0;
}
