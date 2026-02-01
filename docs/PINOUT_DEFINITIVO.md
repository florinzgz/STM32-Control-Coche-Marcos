# PINOUT DEFINITIVO - STM32G474RE (LQFP64)

## Especificaciones del Microcontrolador

- **MCU**: STM32G474RE
- **Core**: ARM Cortex-M4F @ 170 MHz
- **Memoria**: 128KB RAM, 512KB Flash
- **Package**: LQFP64 (64 pines)
- **Pines GPIO útiles**: 51 (excluyendo alimentación, tierra, y pines reservados)

## Tabla Completa de Pinout

### Power & Ground Pins
| Pin | Function | Description |
|-----|----------|-------------|
| 1   | VBAT     | Backup battery power (3.3V) |
| 8   | VSSA     | Analog ground |
| 9   | VDDA     | Analog power supply (3.3V) |
| 19  | VSS      | Digital ground |
| 20  | VDD      | Digital power supply (3.3V) |
| 32  | VSS      | Digital ground |
| 33  | VDD      | Digital power supply (3.3V) |
| 48  | VSS      | Digital ground |
| 49  | VDD      | Digital power supply (3.3V) |
| 64  | VSS      | Digital ground |

### Clock & Reset
| Pin | Function | Description |
|-----|----------|-------------|
| 5   | PH0/OSC_IN | External crystal oscillator input (8MHz) |
| 6   | PH1/OSC_OUT | External crystal oscillator output |
| 7   | NRST     | System reset (active low) |

### PWM Outputs - TIM1 (Motores de Tracción)
| Pin | GPIO | Timer    | Function | Description |
|-----|------|----------|----------|-------------|
| 41  | PA8  | TIM1_CH1 | PWM_FL   | Motor Delantero Izquierdo |
| 42  | PA9  | TIM1_CH2 | PWM_FR   | Motor Delantero Derecho |
| 43  | PA10 | TIM1_CH3 | PWM_RL   | Motor Trasero Izquierdo |
| 44  | PA11 | TIM1_CH4 | PWM_RR   | Motor Trasero Derecho |

**Configuración PWM:**
- Frecuencia: 20 kHz
- Resolución: 13-bit (ARR = 8499)
- Duty cycle: 0-8499

### PWM Output - TIM8 (Motor de Dirección)
| Pin | GPIO | Timer    | Function | Description |
|-----|------|----------|----------|-------------|
| 39  | PC8  | TIM8_CH3 | PWM_STEER | Motor de Dirección |

### Direction Control (GPIO Output)
| Pin | GPIO | Function  | Description |
|-----|------|-----------|-------------|
| 15  | PC0  | DIR_FL    | Dirección Motor FL |
| 16  | PC1  | DIR_FR    | Dirección Motor FR |
| 17  | PC2  | DIR_RL    | Dirección Motor RL |
| 18  | PC3  | DIR_RR    | Dirección Motor RR |
| 34  | PC4  | DIR_STEER | Dirección Motor Dirección |

### Enable Signals (GPIO Output)
| Pin | GPIO | Function  | Description |
|-----|------|-----------|-------------|
| 35  | PC5  | EN_FL     | Habilitación Motor FL |
| 36  | PC6  | EN_FR     | Habilitación Motor FR |
| 37  | PC7  | EN_RL     | Habilitación Motor RL |
| 38  | PD2  | EN_RR     | Habilitación Motor RR |
| 40  | PC9  | EN_STEER  | Habilitación Motor Dirección |

### Relay Control (GPIO Output - Active HIGH)
| Pin | GPIO  | Function    | Description |
|-----|-------|-------------|-------------|
| 51  | PC10  | RELAY_MAIN  | Relé Principal (alimentación general) |
| 52  | PC11  | RELAY_TRAC  | Relé Tracción (alimentación motores) |
| 53  | PC12  | RELAY_DIR   | Relé Dirección (alimentación dirección) |

**Especificación Relés:**
- Bobina: 5V DC
- Contactos: 30A @ 12V DC
- Control: A través de optoacopladores HY-M158

### Encoder de Dirección - TIM2 (Quadrature)
| Pin | GPIO | Timer    | Function | Description |
|-----|------|----------|----------|-------------|
| 50  | PA15 | TIM2_CH1 | ENC_A    | Canal A - Encoder E6B2-CWZ6C |
| 55  | PB3  | TIM2_CH2 | ENC_B    | Canal B - Encoder E6B2-CWZ6C |
| 56  | PB4  | GPIO/EXTI4 | ENC_Z  | Pulso índice Z (interrupción) |

**Especificación Encoder:**
- Modelo: E6B2-CWZ6C
- Resolución: 1200 PPR
- Modo Quadrature: 4800 counts/rev (1200 × 4)
- Resolución angular: 0.075°/count (360° / 4800)

### Sensores LJ12A3-4-Z/BX (GPIO Input - EXTI)
| Pin | GPIO  | Function      | Description |
|-----|-------|---------------|-------------|
| 57  | PB5   | WHEEL_FL_SENS | Sensor velocidad rueda FL (6 tornillos) |
| 21  | PB10  | WHEEL_FR_SENS | Sensor velocidad rueda FR (6 tornillos) |
| 22  | PB11  | WHEEL_RL_SENS | Sensor velocidad rueda RL (6 tornillos) |
| 25  | PB12  | WHEEL_RR_SENS | Sensor velocidad rueda RR (6 tornillos) |
| 56  | PB4   | ENC_Z         | Sensor índice encoder (reutilizado arriba) |

**Especificación LJ12A3-4-Z/BX:**
- Tipo: Inductivo NPN NO
- Distancia detección: 4mm
- Tensión: 6-36V DC
- Salida: NPN transistor (activo bajo)
- Configuración: Pullup a 3.3V con resistencia 10kΩ

### Sensores DS18B20 (OneWire - 1 bus compartido)
| Pin | GPIO | Function     | Description |
|-----|------|--------------|-------------|
| 28  | PB0  | ONEWIRE_BUS  | Bus OneWire (5 sensores) |

**Distribución Sensores:**
1. Motor FL - Temperatura
2. Motor FR - Temperatura
3. Motor RL - Temperatura
4. Motor RR - Temperatura
5. Volante/Dirección - Temperatura

**Especificación DS18B20:**
- Rango temperatura: -55°C a +125°C
- Resolución: 0.0625°C (12-bit)
- Protocolo: OneWire
- Resistencia pullup: 4.7kΩ a 3.3V
- ROM único: 64-bit para identificación

### Sensores INA226 (I2C1 via TCA9548A)
| Pin | GPIO | I2C     | Function | Description |
|-----|------|---------|----------|-------------|
| 58  | PB6  | I2C1_SCL | SCL     | Clock I2C |
| 59  | PB7  | I2C1_SDA | SDA     | Datos I2C |

**Configuración I2C:**
- Velocidad: 400 kHz (Fast Mode)
- Multiplexor: TCA9548A (8 canales)
- Dirección TCA9548A: 0x70

**Distribución INA226 (6 sensores):**
| Canal TCA9548A | Sensor | Ubicación | Rango Corriente |
|----------------|--------|-----------|-----------------|
| 0 | INA226_FL | Motor FL | ±20A |
| 1 | INA226_FR | Motor FR | ±20A |
| 2 | INA226_RL | Motor RL | ±20A |
| 3 | INA226_RR | Motor RR | ±20A |
| 4 | INA226_STEER | Motor Dirección | ±10A |
| 5 | INA226_MAIN | Alimentación Principal | ±50A |

**Especificación INA226:**
- Rango tensión: 0-36V
- Resistencia shunt: 0.002Ω (2mΩ)
- Precisión: ±0.1%
- Ganancia ADC: Configurable

### Sensor de Pedal (ADC - Analog Input)
| Pin | GPIO | ADC      | Function | Description |
|-----|------|----------|----------|-------------|
| 13  | PA3  | ADC1_IN4 | PEDAL_POS | Posición pedal acelerador |

**Especificación Pedal:**
- Sensor: A1324LUA-T (Hall Effect Linear)
- Salida: 0.5V - 4.5V
- Divisor tensión: 5V → 3.3V (R1=1kΩ, R2=2kΩ)
- Entrada ADC: 0.33V - 2.97V
- Resolución ADC: 12-bit (0-4095)

**Cálculo Conversión:**
```
Voltage_ADC = (ADC_value / 4095) × 3.3V
Pedal_% = ((Voltage_ADC - 0.33) / (2.97 - 0.33)) × 100%
```

### CAN Bus (FDCAN1)
| Pin | GPIO  | Function   | Description |
|-----|-------|------------|-------------|
| 44  | PA11  | FDCAN1_RX  | Recepción CAN |
| 45  | PA12  | FDCAN1_TX  | Transmisión CAN |

**Configuración CAN:**
- Tipo: FDCAN (CAN-FD compatible)
- Velocidad: 500 kbps
- Terminación: 120Ω
- Transceiver: TJA1050 o equivalente

### UART/USART (Debug/Programming)
| Pin | GPIO  | UART     | Function | Description |
|-----|-------|----------|----------|-------------|
| 46  | PA13  | SWDIO    | Debug/Program | SWD Data |
| 49  | PA14  | SWCLK    | Debug/Program | SWD Clock |
| 42  | PA9   | USART1_TX | UART TX | Debug UART (alternativa) |
| 43  | PA10  | USART1_RX | UART RX | Debug UART (alternativa) |

### GPIO Adicionales (Reservados/Expansión)
| Pin | GPIO  | Function | Description |
|-----|-------|----------|-------------|
| 10  | PA0   | GPIO     | Libre/Expansión |
| 11  | PA1   | GPIO     | Libre/Expansión |
| 12  | PA2   | GPIO     | Libre/Expansión |
| 14  | PA4   | GPIO     | Libre/Expansión |
| 15  | PA5   | GPIO     | Libre/Expansión |
| 16  | PA6   | GPIO     | Libre/Expansión |
| 17  | PA7   | GPIO     | Libre/Expansión |
| 21  | PB0   | OneWire  | (Usado para DS18B20) |
| 22  | PB1   | GPIO     | Libre/Expansión |
| 23  | PB2   | GPIO     | Libre/Expansión |
| 60  | PB8   | GPIO     | Libre/Expansión |
| 61  | PB9   | GPIO     | Libre/Expansión |
| 47  | PB13  | GPIO     | Libre/Expansión |
| 48  | PB14  | GPIO     | Libre/Expansión |
| 49  | PB15  | GPIO     | Libre/Expansión |

## Recomendaciones de Cableado

### Calibre de Cables por Función

| Función | Corriente Max | Calibre AWG | Color Sugerido |
|---------|---------------|-------------|----------------|
| Alimentación principal (12V) | 50A | 10 AWG | Rojo |
| GND principal | 50A | 10 AWG | Negro |
| Motores tracción (individual) | 20A | 14 AWG | Amarillo |
| Motor dirección | 10A | 16 AWG | Azul |
| Señales PWM | 100mA | 24 AWG | Naranja |
| Señales DIR/EN | 50mA | 26 AWG | Verde |
| Sensores (I2C, OneWire) | 10mA | 28 AWG | Blanco |
| CAN Bus | 50mA | 24 AWG | Par trenzado |

### Protección y Filtrado

**Alimentación:**
- Condensador bulk: 1000μF/25V cerca de la entrada de alimentación
- Condensadores cerámicos: 100nF en cada pin VDD del MCU
- Condensador tantalio: 10μF/6.3V en VDDA (alimentación analógica)

**Entradas Digitales:**
- Resistencias pullup/pulldown: 10kΩ
- Condensadores de desacoplo: 100nF
- Diodos de protección: BAV99 o equivalente

**Salidas PWM:**
- Resistencias de limitación: 220Ω en serie
- Optoacopladores: TLP250 o HCPL-3120 para aislamiento

**Bus CAN:**
- Resistencias terminación: 120Ω en ambos extremos
- Protección ESD: TPD2E001 o equivalente

## Notas Importantes

1. **Todos los pines GPIO tienen protección ESD integrada** de ±2kV (HBM)
2. **Corriente máxima por pin**: 25mA
3. **Corriente total VDD**: 150mA recomendado
4. **Tensión absoluta máxima**: -0.3V a 4.0V (nunca exceder)
5. **Temperatura operación**: -40°C a +85°C (rango industrial)

## Referencias

- **Datasheet**: STM32G474RE [DS12589 Rev 5]
- **Reference Manual**: RM0440
- **Programming Manual**: PM0214
- **HAL Library**: STM32Cube_FW_G4_V1.5.0

---

**Documento creado**: 2026-02-01  
**Versión**: 1.0  
**Autor**: Sistema de Control Coche Marcos
