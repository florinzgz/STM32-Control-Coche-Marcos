/**
  ******************************************************************************
  * @file    build_sanity_checks.h
  * @brief   Compile-time validation — catches CubeMX regressions early.
  *
  *          PURE preprocessor logic.  No variables, no functions, no memory,
  *          no runtime cost.  Include ONCE from main.c after all other headers.
  *
  *          Fails compilation (#error) only when something is clearly broken.
  *          Uses #warning for non-critical hints (compiler-dependent).
  ******************************************************************************
  */

#ifndef BUILD_SANITY_CHECKS_H
#define BUILD_SANITY_CHECKS_H

/* ========================================================================== */
/*  1) Critical GPIO pin macros (from project_config.h)                       */
/* ========================================================================== */

#ifndef PIN_ONEWIRE
  #error "BUILD CHECK: PIN_ONEWIRE is not defined — DS18B20 OneWire bus will not work"
#endif

#ifndef PIN_WHEEL_FL
  #error "BUILD CHECK: PIN_WHEEL_FL is not defined — front-left wheel speed sensor missing"
#endif

#ifndef PIN_WHEEL_FR
  #error "BUILD CHECK: PIN_WHEEL_FR is not defined — front-right wheel speed sensor missing"
#endif

#ifndef PIN_WHEEL_RL
  #error "BUILD CHECK: PIN_WHEEL_RL is not defined — rear-left wheel speed sensor missing"
#endif

#ifndef PIN_WHEEL_RR
  #error "BUILD CHECK: PIN_WHEEL_RR is not defined — rear-right wheel speed sensor missing"
#endif

/* ========================================================================== */
/*  2) HAL module & device presence (interrupt infrastructure)                */
/* ========================================================================== */
/*  IRQn values (EXTI0_IRQn, FDCAN1_IT0_IRQn, etc.) are enum members in the  */
/*  CMSIS device header, not preprocessor macros — they cannot be tested with */
/*  #ifdef.  Instead we verify that the correct device and HAL modules are    */
/*  enabled, which guarantees the IRQn enum and HAL IRQ handlers exist.       */

#ifndef STM32G474xx
  #error "BUILD CHECK: STM32G474xx device macro not defined — wrong MCU target or missing -DSTM32G474xx"
#endif

/*  HAL module checks only apply when the real stm32g4xx_hal_conf.h has been  */
/*  included (not when building with analysis stubs).                          */
#ifdef STM32G4xx_HAL_CONF_H

  #ifndef HAL_EXTI_MODULE_ENABLED
    #error "BUILD CHECK: HAL_EXTI_MODULE_ENABLED not defined — EXTI interrupts (wheel speed sensors) unavailable"
  #endif

  #ifndef HAL_FDCAN_MODULE_ENABLED
    #error "BUILD CHECK: HAL_FDCAN_MODULE_ENABLED not defined — FDCAN interrupts (CAN bus) unavailable"
  #endif

  #ifndef HAL_TIM_MODULE_ENABLED
    #error "BUILD CHECK: HAL_TIM_MODULE_ENABLED not defined — TIM interrupts (motor PWM) unavailable"
  #endif

#endif /* STM32G4xx_HAL_CONF_H */

/* ========================================================================== */
/*  3) Project configuration enforcement (from project_config.h)              */
/* ========================================================================== */

#ifndef WHEEL_MIN_PULSE_INTERVAL_MS
  #error "BUILD CHECK: WHEEL_MIN_PULSE_INTERVAL_MS not defined — wheel debounce config missing"
#endif

#ifndef WHEEL_MAX_FREQ_HZ
  #error "BUILD CHECK: WHEEL_MAX_FREQ_HZ not defined — wheel ISR flood protection config missing"
#endif

#ifndef WHEEL_STALE_TIMEOUT_MS
  #error "BUILD CHECK: WHEEL_STALE_TIMEOUT_MS not defined — wheel stale detection config missing"
#endif

/* ========================================================================== */
/*  4) Optional soft checks — warnings only, never fail the build             */
/* ========================================================================== */
/*  #warning is a widely supported extension (GCC, Clang, ARMCC) but not      */
/*  part of C11.  Guarded with __GNUC__ to avoid issues on exotic compilers.  */

#if defined(__GNUC__)

  #ifndef PIN_STEER_CENTER
    #warning "BUILD HINT: PIN_STEER_CENTER not defined — steering centre sensor may be unconfigured"
  #endif

  #ifndef PIN_EN_FL
    #warning "BUILD HINT: PIN_EN_FL not defined — front-left motor enable pin may be missing"
  #endif

  #ifndef PIN_EN_RR
    #warning "BUILD HINT: PIN_EN_RR not defined — rear-right motor enable pin may be missing"
  #endif

#endif /* __GNUC__ */

#endif /* BUILD_SANITY_CHECKS_H */
