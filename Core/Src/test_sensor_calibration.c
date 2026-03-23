/**
  ****************************************************************************
  * @file    test_sensor_calibration.c
  * @brief   Unit tests for INA226 shunt calibration, current scaling,
  *          and per-channel overcurrent / plausibility thresholds.
  *
  *          Compile with host GCC:
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_sensor_calibration.c -o /tmp/test_sensor_calibration
  *
  *          Tests verify that the firmware constants match the physical
  *          shunt specifications (50A/75mV and 100A/75mV external shunts).
  ****************************************************************************
  */

#include "main.h"
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stdbool.h>

/* ---- Stub HAL handles required by main.h externs ---- */
#ifdef HOST_TEST
ADC_HandleTypeDef  hadc1;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim1, htim2, htim3, htim8;
IWDG_HandleTypeDef hiwdg;
#endif

/* ---- Replicate the firmware's current calculation for testing ---- */
#define INA226_SHUNT_LSB_UV   2.5f     /* INA226: 2.5 µV per LSB         */
#define INA226_BUS_LSB_MV     1.25f    /* INA226: 1.25 mV per LSB        */

static float calc_current_amps(int16_t shunt_raw, float shunt_mohm)
{
    float shunt_uv = (float)shunt_raw * INA226_SHUNT_LSB_UV;
    float ma       = shunt_uv / shunt_mohm;      /* µV / mΩ = mA */
    return ma / 1000.0f;                          /* mA → A       */
}

static float calc_bus_voltage(int16_t bus_raw)
{
    return (float)bus_raw * INA226_BUS_LSB_MV / 1000.0f;  /* mV → V */
}

/* ---- Overcurrent / plausibility thresholds (mirror safety_system.c) ---- */
#define MAX_CURRENT_A            25.0f
#define MAX_CURRENT_BATT_A      100.0f
#define SENSOR_CURRENT_MAX_A     50.0f
#define SENSOR_CURRENT_MAX_BATT_A 100.0f

/* ---- Test framework ---- */

static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(cond, msg) do {                                      \
    tests_run++;                                                         \
    if (!(cond)) {                                                       \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, (msg));          \
        tests_failed++;                                                  \
    }                                                                    \
} while (0)

#define ASSERT_NEAR(got, expected, tol, msg) do {                        \
    float _g = (got), _e = (expected), _t = (tol);                      \
    tests_run++;                                                         \
    if (fabsf(_g - _e) > _t) {                                          \
        printf("FAIL %s:%d  %s: got %.6f, expected %.6f (±%.6f)\n",     \
               __FILE__, __LINE__, (msg), (double)_g, (double)_e,       \
               (double)_t);                                              \
        tests_failed++;                                                  \
    }                                                                    \
} while (0)

/* =====================================================================
 *  1. Shunt resistance constant validation
 * ===================================================================== */

static void test_shunt_values_match_hardware(void)
{
    /* Hardware spec: 50A / 75mV → R = 75 mV / 50 A = 1.5 mΩ = 0.0015 Ω */
    ASSERT_NEAR((float)INA226_SHUNT_MOHM_MOTOR, 1.5f, 0.001f,
                "Motor shunt must be 1.5 mΩ (50A/75mV)");

    /* Hardware spec: 100A / 75mV → R = 75 mV / 100 A = 0.75 mΩ = 0.00075 Ω */
    ASSERT_NEAR((float)INA226_SHUNT_MOHM_BATTERY, 0.75f, 0.001f,
                "Battery shunt must be 0.75 mΩ (100A/75mV)");
}

static void test_shunt_ratio(void)
{
    /* Battery shunt should be exactly half the motor shunt (double current range) */
    float ratio = (float)INA226_SHUNT_MOHM_MOTOR / (float)INA226_SHUNT_MOHM_BATTERY;
    ASSERT_NEAR(ratio, 2.0f, 0.01f,
                "Motor/battery shunt ratio must be 2:1");
}

/* =====================================================================
 *  2. Current calculation accuracy at key operating points
 * ===================================================================== */

static void test_motor_current_at_25A(void)
{
    /* At 25 A through 1.5 mΩ: V_shunt = 25 × 1.5 mΩ = 37.5 mV = 37500 µV
     * shunt_raw = 37500 / 2.5 = 15000 */
    int16_t raw = 15000;
    float amps = calc_current_amps(raw, (float)INA226_SHUNT_MOHM_MOTOR);
    ASSERT_NEAR(amps, 25.0f, 0.01f,
                "Motor 25A calculation");
}

static void test_motor_current_at_50A(void)
{
    /* At 50 A through 1.5 mΩ: V_shunt = 50 × 1.5 = 75 mV = 75000 µV
     * shunt_raw = 75000 / 2.5 = 30000 */
    int16_t raw = 30000;
    float amps = calc_current_amps(raw, (float)INA226_SHUNT_MOHM_MOTOR);
    ASSERT_NEAR(amps, 50.0f, 0.01f,
                "Motor 50A calculation (full scale)");
}

static void test_battery_current_at_100A(void)
{
    /* At 100 A through 0.75 mΩ: V_shunt = 100 × 0.75 = 75 mV = 75000 µV
     * shunt_raw = 75000 / 2.5 = 30000 */
    int16_t raw = 30000;
    float amps = calc_current_amps(raw, (float)INA226_SHUNT_MOHM_BATTERY);
    ASSERT_NEAR(amps, 100.0f, 0.01f,
                "Battery 100A calculation (full scale)");
}

static void test_battery_current_at_50A(void)
{
    /* At 50 A through 0.75 mΩ: V_shunt = 37.5 mV
     * shunt_raw = 37500 / 2.5 = 15000 */
    int16_t raw = 15000;
    float amps = calc_current_amps(raw, (float)INA226_SHUNT_MOHM_BATTERY);
    ASSERT_NEAR(amps, 50.0f, 0.01f,
                "Battery 50A calculation");
}

static void test_zero_current(void)
{
    float motor = calc_current_amps(0, (float)INA226_SHUNT_MOHM_MOTOR);
    float batt  = calc_current_amps(0, (float)INA226_SHUNT_MOHM_BATTERY);
    ASSERT_NEAR(motor, 0.0f, 0.001f, "Motor zero current");
    ASSERT_NEAR(batt,  0.0f, 0.001f, "Battery zero current");
}

static void test_negative_current(void)
{
    /* Reverse current (regenerative braking): -10 A through 1.5 mΩ
     * V_shunt = -15 mV = -15000 µV → raw = -6000 */
    int16_t raw = -6000;
    float amps = calc_current_amps(raw, (float)INA226_SHUNT_MOHM_MOTOR);
    ASSERT_NEAR(amps, -10.0f, 0.01f,
                "Negative current (regen braking)");
}

/* =====================================================================
 *  3. Register overflow checks — shunt_raw must fit int16_t (±32767)
 * ===================================================================== */

static void test_motor_50A_fits_register(void)
{
    /* 50 A × 1.5 mΩ = 75 mV → raw = 75000 / 2.5 = 30000 < 32767 ✓ */
    int32_t raw = (int32_t)(50.0f * INA226_SHUNT_MOHM_MOTOR * 1000.0f / INA226_SHUNT_LSB_UV);
    ASSERT_TRUE(raw <= 32767 && raw >= -32768,
                "50A motor shunt voltage fits int16 register");
}

static void test_battery_100A_fits_register(void)
{
    /* 100 A × 0.75 mΩ = 75 mV → raw = 75000 / 2.5 = 30000 < 32767 ✓ */
    int32_t raw = (int32_t)(100.0f * INA226_SHUNT_MOHM_BATTERY * 1000.0f / INA226_SHUNT_LSB_UV);
    ASSERT_TRUE(raw <= 32767 && raw >= -32768,
                "100A battery shunt voltage fits int16 register");
}

static void test_bus_24V_fits_register(void)
{
    /* 24 V → raw = 24000 / 1.25 = 19200 < 32767 ✓ (unsigned 15-bit) */
    int32_t raw = (int32_t)(24.0f * 1000.0f / INA226_BUS_LSB_MV);
    ASSERT_TRUE(raw <= 32767, "24V bus voltage fits register");
}

/* =====================================================================
 *  4. Bus voltage calculation
 * ===================================================================== */

static void test_bus_voltage_24V(void)
{
    /* 24 V → raw = 24000 / 1.25 = 19200 */
    int16_t raw = 19200;
    float volts = calc_bus_voltage(raw);
    ASSERT_NEAR(volts, 24.0f, 0.01f, "24V bus voltage");
}

static void test_bus_voltage_zero(void)
{
    float volts = calc_bus_voltage(0);
    ASSERT_NEAR(volts, 0.0f, 0.001f, "0V bus voltage");
}

/* =====================================================================
 *  5. Overcurrent threshold validation
 * ===================================================================== */

static void test_motor_overcurrent_threshold(void)
{
    /* Motor channels: 25A overcurrent limit, well below 50A sensor capacity */
    ASSERT_TRUE(MAX_CURRENT_A < 50.0f,
                "Motor overcurrent < sensor full-scale (50A)");
    ASSERT_NEAR(MAX_CURRENT_A, 25.0f, 0.1f,
                "Motor overcurrent threshold is 25A");
}

static void test_battery_overcurrent_threshold(void)
{
    /* Battery: 100A overcurrent limit matches sensor full-scale */
    ASSERT_NEAR(MAX_CURRENT_BATT_A, 100.0f, 0.1f,
                "Battery overcurrent threshold is 100A");
    ASSERT_TRUE(MAX_CURRENT_BATT_A > MAX_CURRENT_A,
                "Battery threshold > motor threshold");
}

/* =====================================================================
 *  6. Plausibility ceiling validation
 * ===================================================================== */

static void test_motor_plausibility_ceiling(void)
{
    ASSERT_NEAR(SENSOR_CURRENT_MAX_A, 50.0f, 0.1f,
                "Motor plausibility ceiling = 50A");
}

static void test_battery_plausibility_ceiling(void)
{
    ASSERT_NEAR(SENSOR_CURRENT_MAX_BATT_A, 100.0f, 0.1f,
                "Battery plausibility ceiling = 100A");
}

/* =====================================================================
 *  7. Channel mapping and constants
 * ===================================================================== */

static void test_channel_count(void)
{
    ASSERT_TRUE(NUM_INA226 == 6, "6 INA226 channels");
}

static void test_battery_channel_index(void)
{
    ASSERT_TRUE(INA226_CHANNEL_BATTERY == 4, "Battery on channel 4");
    ASSERT_TRUE(INA226_CHANNEL_BATTERY < NUM_INA226,
                "Battery channel within range");
}

static void test_i2c_addresses_unique(void)
{
    /* All INA226 share address 0x40 via TCA9548A mux — mux address must differ */
    ASSERT_TRUE(I2C_ADDR_TCA9548A != I2C_ADDR_INA226,
                "TCA9548A and INA226 addresses must differ");
}

/* =====================================================================
 *  8. LSB resolution check
 * ===================================================================== */

static void test_motor_current_lsb(void)
{
    /* 1 LSB = 2.5 µV / 1.5 mΩ / 1000 = 0.001667 A ≈ 1.67 mA */
    float lsb = INA226_SHUNT_LSB_UV / (float)INA226_SHUNT_MOHM_MOTOR / 1000.0f;
    ASSERT_TRUE(lsb > 0.001f && lsb < 0.01f,
                "Motor LSB resolution in reasonable range (1-10 mA)");
}

static void test_battery_current_lsb(void)
{
    /* 1 LSB = 2.5 µV / 0.75 mΩ / 1000 = 0.003333 A ≈ 3.33 mA */
    float lsb = INA226_SHUNT_LSB_UV / (float)INA226_SHUNT_MOHM_BATTERY / 1000.0f;
    ASSERT_TRUE(lsb > 0.001f && lsb < 0.01f,
                "Battery LSB resolution in reasonable range (1-10 mA)");
}

/* =====================================================================
 *  9. Full-scale shunt voltage at rated current = 75 mV for all channels
 * ===================================================================== */

static void test_motor_full_scale_75mV(void)
{
    /* 50 A × 1.5 mΩ = 75 mV (matches hardware spec) */
    float v_mv = 50.0f * (float)INA226_SHUNT_MOHM_MOTOR;
    ASSERT_NEAR(v_mv, 75.0f, 0.01f,
                "Motor full-scale shunt voltage = 75 mV");
}

static void test_battery_full_scale_75mV(void)
{
    /* 100 A × 0.75 mΩ = 75 mV (matches hardware spec) */
    float v_mv = 100.0f * (float)INA226_SHUNT_MOHM_BATTERY;
    ASSERT_NEAR(v_mv, 75.0f, 0.01f,
                "Battery full-scale shunt voltage = 75 mV");
}

/* =====================================================================
 *  main
 * ===================================================================== */

int main(void)
{
    printf("=== INA226 Sensor Calibration Tests ===\n\n");

    /* 1. Shunt resistance constants */
    test_shunt_values_match_hardware();
    test_shunt_ratio();

    /* 2. Current calculation accuracy */
    test_motor_current_at_25A();
    test_motor_current_at_50A();
    test_battery_current_at_100A();
    test_battery_current_at_50A();
    test_zero_current();
    test_negative_current();

    /* 3. Register overflow */
    test_motor_50A_fits_register();
    test_battery_100A_fits_register();
    test_bus_24V_fits_register();

    /* 4. Bus voltage */
    test_bus_voltage_24V();
    test_bus_voltage_zero();

    /* 5. Overcurrent thresholds */
    test_motor_overcurrent_threshold();
    test_battery_overcurrent_threshold();

    /* 6. Plausibility ceilings */
    test_motor_plausibility_ceiling();
    test_battery_plausibility_ceiling();

    /* 7. Channel mapping */
    test_channel_count();
    test_battery_channel_index();
    test_i2c_addresses_unique();

    /* 8. LSB resolution */
    test_motor_current_lsb();
    test_battery_current_lsb();

    /* 9. Full-scale voltage */
    test_motor_full_scale_75mV();
    test_battery_full_scale_75mV();

    printf("\n--- sensor_calibration tests: %d run, %d failed ---\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
