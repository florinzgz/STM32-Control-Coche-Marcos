# Makefile for ARM GCC build system for STM32G474RE

# Define the project name
PROJECT = STM32G474RE

# Source files
SRC = main.c stm32g4xx_hal.c stm32g4xx_it.c \
      peripherals.c drivers.c

# Object files
OBJ = $(SRC:.c=.o)

# Compiler and Linker
CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc

# Compiler flags
CFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
          -g -Wall -O2 -DSTM32G474xx

# Linker flags
LDFLAGS = -Tstm32g474.ld -Wl,--gc-sections

# Build targets
all: $(PROJECT).elf $(PROJECT).bin

$(PROJECT).elf: $(OBJ)
	$(LD) -o $@ $^ $(LDFLAGS)

$(PROJECT).bin: $(PROJECT).elf
	arm-none-eabi-objcopy -O binary $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(PROJECT).elf $(PROJECT).bin
