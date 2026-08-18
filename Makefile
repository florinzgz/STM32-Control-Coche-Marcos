# Makefile for ARM GCC build system for STM32G474RE
 # Drivers/ (HAL + CMSIS) are committed in the repository.
 
 # Define the project name
 PROJECT = STM32G474RE
 
 # Source directories
 CORE_SRC = Core/Src
 CORE_INC = Core/Inc
 
 # Source files
 C_SOURCES = \
   $(CORE_SRC)/main.c \
   $(CORE_SRC)/adc.c \
   $(CORE_SRC)/fdcan.c \
   $(CORE_SRC)/gpio.c \
   $(CORE_SRC)/i2c.c \
   $(CORE_SRC)/iwdg.c \
   $(CORE_SRC)/tim.c \
   $(CORE_SRC)/motor_control_patched.c \
   $(CORE_SRC)/tcs_tuned.c \
   $(CORE_SRC)/can_handler.c \
   $(CORE_SRC)/busoff_recovery.c \
   $(CORE_SRC)/motion_inhibit.c \
   $(CORE_SRC)/relay_health_diag.c \
   $(CORE_SRC)/ina226_channel_diag.c \
   $(CORE_SRC)/rc_arbiter.c \
   $(CORE_SRC)/sensor_manager_patched.c \
   $(CORE_SRC)/pedal_logic.c \
   $(CORE_SRC)/safety_system_patched.c \
   $(CORE_SRC)/service_mode.c \
   $(CORE_SRC)/ackermann.c \
   $(CORE_SRC)/ackermann_diff.c \
   $(CORE_SRC)/steering_centering_patched.c \
   $(CORE_SRC)/steering_centering_diag.c \
   $(CORE_SRC)/steering_eps.c \
   $(CORE_SRC)/steering_output.c \
   $(CORE_SRC)/boot_validation.c \
   $(CORE_SRC)/encoder_reader.c \
   $(CORE_SRC)/eps_params.c \
   $(CORE_SRC)/error_log.c \
   $(CORE_SRC)/steering_cal_store.c \
   $(CORE_SRC)/steering_z.c \
   $(CORE_SRC)/steering_supervisor.c \
   $(CORE_SRC)/steering_supervisor_io.c \
   $(CORE_SRC)/sensor_map_store.c \
   $(CORE_SRC)/pedal_cal_store.c \
   $(CORE_SRC)/pedal_cal_session.c \
   $(CORE_SRC)/gear_limits_store.c \
   $(CORE_SRC)/drive_tuning_store.c \
   $(CORE_SRC)/battery_limits_store.c \
   $(CORE_SRC)/tcs_tuning_store.c \
   $(CORE_SRC)/geometry_store.c \
   $(CORE_SRC)/shunt_store.c \
   $(CORE_SRC)/steering_service_store.c \
   $(CORE_SRC)/wheel_sensor_store.c \
   $(CORE_SRC)/service_diag_session.c \
   $(CORE_SRC)/wheel_equality_test.c \
   $(CORE_SRC)/loop_diag.c \
   $(CORE_SRC)/math_safety.c \
   $(CORE_SRC)/standby_mode_sync_policy.c \
   $(CORE_SRC)/stm32g4xx_it.c \
   $(CORE_SRC)/stm32g4xx_hal_msp.c \
   $(CORE_SRC)/system_stm32g4xx.c \
   $(CORE_SRC)/syscalls.c \
   $(CORE_SRC)/sysmem.c \
+  $(CORE_SRC)/forensic_snapshot.c
 
 # HAL driver sources (STM32G4xx HAL v1.2.2)
 HAL_SRC = Drivers/STM32G4xx_HAL_Driver/Src
 HAL_SOURCES = \
   $(HAL_SRC)/stm32g4xx_hal.c \
   $(HAL_SRC)/stm32g4xx_hal_cortex.c \
   $(HAL_SRC)/stm32g4xx_hal_gpio.c \
   $(HAL_SRC)/stm32g4xx_hal_rcc.c \
   $(HAL_SRC)/stm32g4xx_hal_rcc_ex.c \
   $(HAL_SRC)/stm32g4xx_hal_pwr.c \
   $(HAL_SRC)/stm32g4xx_hal_pwr_ex.c \
   $(HAL_SRC)/stm32g4xx_hal_tim.c \
   $(HAL_SRC)/stm32g4xx_hal_tim_ex.c \
   $(HAL_SRC)/stm32g4xx_hal_fdcan.c \
   $(HAL_SRC)/stm32g4xx_hal_i2c.c \
   $(HAL_SRC)/stm32g4xx_hal_i2c_ex.c \
   $(HAL_SRC)/stm32g4xx_hal_adc.c \
   $(HAL_SRC)/stm32g4xx_hal_adc_ex.c \
   $(HAL_SRC)/stm32g4xx_hal_iwdg.c \
   $(HAL_SRC)/stm32g4xx_hal_dma.c \
   $(HAL_SRC)/stm32g4xx_hal_dma_ex.c \
   $(HAL_SRC)/stm32g4xx_hal_exti.c \
   $(HAL_SRC)/stm32g4xx_hal_flash.c \
   $(HAL_SRC)/stm32g4xx_hal_flash_ex.c
