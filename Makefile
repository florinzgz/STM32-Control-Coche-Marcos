# Makefile for ARM GCC build system for STM32G474RE

# Define the project name
PROJECT = STM32G474RE

# Source directories
CORE_SRC = Core/Src
CORE_INC = Core/Inc

# Source files
C_SOURCES = \
  $(CORE_SRC)/main.c \
  $(CORE_SRC)/motor_control.c \
  $(CORE_SRC)/can_handler.c \
  $(CORE_SRC)/sensor_manager.c \
  $(CORE_SRC)/safety_system.c \
  $(CORE_SRC)/service_mode.c \
  $(CORE_SRC)/ackermann.c \
  $(CORE_SRC)/steering_centering.c \
  $(CORE_SRC)/boot_validation.c \
  $(CORE_SRC)/encoder_reader.c \
  $(CORE_SRC)/stm32g4xx_it.c \
  $(CORE_SRC)/stm32g4xx_hal_msp.c \
  $(CORE_SRC)/system_stm32g4xx.c

# HAL driver sources (added by STM32CubeMX — generate code before building)
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
  $(HAL_SRC)/stm32g4xx_hal_adc.c \
  $(HAL_SRC)/stm32g4xx_hal_adc_ex.c \
  $(HAL_SRC)/stm32g4xx_hal_fdcan.c \
  $(HAL_SRC)/stm32g4xx_hal_i2c.c \
  $(HAL_SRC)/stm32g4xx_hal_i2c_ex.c \
  $(HAL_SRC)/stm32g4xx_hal_iwdg.c

# ASM startup
ASM_SOURCES = startup_stm32g474retx.s

# Include paths
C_INCLUDES = \
  -I$(CORE_INC) \
  -IDrivers/STM32G4xx_HAL_Driver/Inc \
  -IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
  -IDrivers/CMSIS/Include

# Compiler and Linker
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
LD = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SZ = $(PREFIX)size

# CPU flags
CPU = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Compiler flags
CFLAGS = $(CPU) $(C_INCLUDES) -DSTM32G474xx -DUSE_HAL_DRIVER \
         -Wall -fdata-sections -ffunction-sections -g -O2

ASFLAGS = $(CPU) -g

# Linker flags
LDFLAGS = $(CPU) -TSTM32G474RETX_FLASH.ld -Wl,--gc-sections -lm -lc -lnosys

# Build directory
BUILD_DIR = build

# Collect all objects
OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(HAL_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))

vpath %.c $(sort $(dir $(C_SOURCES) $(HAL_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Build targets
all: $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).hex $(BUILD_DIR)/$(PROJECT).bin
	$(SZ) $(BUILD_DIR)/$(PROJECT).elf

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR)/$(PROJECT).elf: $(OBJECTS)
	$(LD) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary -S $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean
