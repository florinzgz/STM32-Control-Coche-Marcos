# 📌 PINOUT Completo - STM32G474RE

> ## ⛔ DOCUMENTO OBSOLETO — NO USAR PARA CABLEADO
>
> **Este documento NO refleja la arquitectura actual del firmware (RPWM/LPWM directo, PR #120).**
> **Actualización CAN rev 1.4 (2026-04-23):** PC10 queda reservado/libre — no existe relé MAIN.
> Contiene asignaciones de pines incorrectas que pueden causar daño al hardware si se siguen.
>
> **Errores críticos en este documento:**
> - Pines de motor incorrectos (usa arquitectura antigua PWM+DIR+EN en vez de RPWM/LPWM)
> - Pines de sensores de rueda incorrectos (PB0/PB1/PB2/PB10 en vez de PA0/PA1/PA2/PB15)
> - Valor ARR incorrecto (8499 en vez de 4249)
> - Valor de shunt INA226 incorrecto (2 mΩ en vez de 1.5 mΩ / 0.75 mΩ)
> - Asignación de canales INA226 incorrecta
> - Divisor del pedal incorrecto (1kΩ+2kΩ en vez de 10kΩ+6.8kΩ)
>
> **Usar en su lugar:**
> - [`docs/CONEXIONES_COMPLETAS.md`](CONEXIONES_COMPLETAS.md) — Guía cable por cable actualizada
> - [`docs/HARDWARE_WIRING_MANUAL.md`](HARDWARE_WIRING_MANUAL.md) — Manual eléctrico completo
> - [`Core/Inc/main.h`](../Core/Inc/main.h) — Definiciones de pines en firmware (fuente de verdad)

**Firmware de Control Vehicular - Asignación de Pines Definitiva**

---

## 📋 Tabla de Contenidos

1. [Resumen Ejecutivo](#-resumen-ejecutivo)
2. [Motores y PWM](#-motores-y-pwm)
3. [Sensores de Rueda](#-sensores-de-rueda)
4. [Encoder de Dirección](#-encoder-de-dirección)
5. [Sensores de Corriente](#-sensores-de-corriente-ina226)
6. [Sensores de Temperatura](#-sensores-de-temperatura-ds18b20)
7. [Pedal y Shifter](#-pedal-y-shifter)
8. [Comunicación CAN](#-comunicación-can)
9. [Relés de Potencia](#-relés-de-potencia)
10. [Tabla Resumen](#-tabla-resumen-completa)

---

## 🎯 Resumen Ejecutivo

### Especificaciones Clave
- **MCU:** STM32G474RE (LQFP64)
- **Total Pines Usados:** 42 de 51 I/O disponibles
- **PWM:** TIM1 (4 canales tracción) + TIM8 (dirección) @ 20 kHz
- **Sensores:**
  - **5 sensores de rueda** (4 ruedas + 1 encoder dirección)
  - **5 sensores de temperatura** DS18B20 (4 motores + 1 ambiente)
  - **6 sensores de corriente** INA226 (4 tracción + 1 dirección + 1 batería)

---

## ⚙️ Motores y PWM

### Control Directo por PWM (SIN PCA9685)

Los motores son controlados **directamente** por los timers del STM32:
- **TIM1** (Advanced Timer) → 4 motores de tracción
- **TIM8** (Advanced Timer) → Motor de dirección
- **Frecuencia:** 20 kHz (period 50 µs)
- **Resolución:** ~13 bits (ARR = 8499)

### 🔴 Motores de Tracción (4× BTS7960)

Cada motor requiere 3 señales: **PWM** (velocidad), **DIR** (dirección), **EN** (habilitación).

| Motor | PWM Pin | Timer/Canal | DIR Pin | EN Pin | Función |
|-------|---------|-------------|---------|--------|---------|
| **FL** (Front Left) | **PA8** | TIM1_CH1 | **PC0** | **PC1** | Rueda delantera izquierda |
| **FR** (Front Right) | **PA9** | TIM1_CH2 | **PC2** | **PC3** | Rueda delantera derecha |
| **RL** (Rear Left) | **PA10** | TIM1_CH3 | **PC4** | **PC5** | Rueda trasera izquierda |
| **FR LPWM** | **PC3** | TIM1_CH4 (AF2) | — | — | LPWM motor delantero derecho |

#### Configuración TIM1
```c
// Configuración TIM1 para PWM @ 20 kHz
TIM1->PSC = 0;              // Prescaler = 1 (170 MHz / 1 = 170 MHz)
TIM1->ARR = 8499;           // Auto-reload: 170 MHz / 8500 = 20 kHz
TIM1->CCR1 = 0;             // Duty cycle motor FL (0-8499)
TIM1->CCR2 = 0;             // Duty cycle motor FR (0-8499)
TIM1->CCR3 = 0;             // Duty cycle motor RL (0-8499)
TIM1->CCR4 = 0;             // Duty cycle motor RR (0-8499)
```

### 🟢 Motor de Dirección (1× BTS7960)

| Motor | PWM Pin | Timer/Canal | DIR Pin | EN Pin | Función |
|-------|---------|-------------|---------|--------|---------|
| **STEER** | **PC8** | TIM8_CH3 | **PC9** | **PC10** | Motor de dirección Ackermann |

#### Configuración TIM8
```c
// Configuración TIM8 para PWM @ 20 kHz
TIM8->PSC = 0;              // Prescaler = 1 (170 MHz)
TIM8->ARR = 8499;           // Auto-reload: 20 kHz
TIM8->CCR3 = 0;             // Duty cycle motor dirección (0-8499)
```

---

## 🎯 Sensores de Rueda

### Especificación: 5 Sensores Totales

Los **5 sensores de rueda** incluyen:
- **4 sensores de rueda** (uno por rueda) → GPIO con EXTI para conteo de pulsos
- **1 encoder de dirección** E6B2-CWZ6C → Modo quadrature en TIM2 (contado como "5to sensor de rueda")

### Sensores de Rueda Simples (4×)

| Sensor | Pin | Modo | EXTI | Función |
|--------|-----|------|------|---------|
| **WHEEL_FL** | **PB0** | GPIO Input (Pull-up) | EXTI0 | Velocidad rueda delantera izquierda |
| **WHEEL_FR** | **PB1** | GPIO Input (Pull-up) | EXTI1 | Velocidad rueda delantera derecha |
| **WHEEL_RL** | **PB2** | GPIO Input (Pull-up) | EXTI2 | Velocidad rueda trasera izquierda |
| **WHEEL_RR** | **PB10** | GPIO Input (Pull-up) | EXTI10 | Velocidad rueda trasera derecha |

**Características:**
- Pulsos por revolución: Variable según sensor (típicamente 1-4 PPR)
- Detección por flanco descendente (activo bajo)
- Resistencia pull-up interna habilitada
- Interrupciones EXTI para conteo preciso

---

## 🔄 Encoder de Dirección

### E6B2-CWZ6C - Encoder Incremental

**Especificaciones:**
- **Resolución:** 1200 PPR (Pulses Per Revolution)
- **Modo Quadrature:** 4800 conteos/revolución (1200 × 4)
- **Resolución angular:** 0.075° por count (360° / 4800)
- **Canales:** A, B (quadrature) + Z (índice/home)
- **Timer:** TIM2 en modo Encoder

| Señal | Pin | Timer | Función |
|-------|-----|-------|---------|
| **ENC_A** | **PA15** | TIM2_CH1 | Canal A (quadrature) |
| **ENC_B** | **PB3** | TIM2_CH2 | Canal B (quadrature) |
| **ENC_Z** | **PB4** | GPIO/EXTI4 | Índice (pulso por vuelta) |

#### Configuración TIM2
```c
// TIM2 en modo Encoder (Quadrature)
TIM2->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; // SMS=011 (Encoder mode 3)
TIM2->ARR = 65535;                             // Contador 16-bit completo
TIM2->CNT = 32768;                             // Centro del rango (offset)
// Lectura: posición = (int16_t)(TIM2->CNT - 32768)
```

**Resolución Angular:**
- 1 conteo = 360° / 1440 = **0.25°** por paso
- Rango de detección: ±180° (±720 conteos desde centro)

---

## 🔋 Sensores de Corriente (INA226)

### Especificación: 6 Sensores INA226

Conectados vía **I²C1** con multiplexor **TCA9548A** (8 canales).

| Sensor | Función | Dirección I²C | Canal TCA9548A |
|--------|---------|---------------|----------------|
| **INA226 #1** | Motor FL (corriente tracción) | 0x40 | CH0 |
| **INA226 #2** | Motor FR (corriente tracción) | 0x41 | CH1 |
| **INA226 #3** | Motor RL (corriente tracción) | 0x44 | CH2 |
| **INA226 #4** | Motor RR (corriente tracción) | 0x45 | CH3 |
| **INA226 #5** | Motor Dirección | 0x48 | CH4 |
| **INA226 #6** | Batería Principal | 0x49 | CH5 |

### Pines I²C

| Señal | Pin | Función |
|-------|-----|---------|
| **I2C_SCL** | **PB6** | I2C1_SCL @ 400 kHz |
| **I2C_SDA** | **PB7** | I2C1_SDA @ 400 kHz |

**Configuración TCA9548A:**
- Dirección I²C: **0x70** (default)
- Selección de canal: Escribir byte con máscara (bit N = canal N)
- Ejemplo: `0x01` activa CH0 (INA226 #1)

**Lectura Secuencial:**
```c
// 1. Seleccionar canal TCA9548A
I2C_Write(0x70, 0x01);  // Activar CH0

// 2. Leer INA226 en canal seleccionado
uint16_t current = INA226_ReadCurrent(0x40);

// 3. Cerrar canal
I2C_Write(0x70, 0x00);
```

---

## 🌡️ Sensores de Temperatura (DS18B20)

### Especificación: 5 Sensores OneWire

Todos los sensores DS18B20 comparten un **único bus OneWire**.

| Sensor | ROM Address (64-bit) | Función |
|--------|-----------------------|---------|
| **TEMP_FL** | 0x28XXXXXXXXXX01 | Temperatura motor FL |
| **TEMP_FR** | 0x28XXXXXXXXXX02 | Temperatura motor FR |
| **TEMP_RL** | 0x28XXXXXXXXXX03 | Temperatura motor RL |
| **TEMP_RR** | 0x28XXXXXXXXXX04 | Temperatura motor RR |
| **TEMP_AMB** | 0x28XXXXXXXXXX05 | Temperatura ambiente |

### Pin OneWire

| Señal | Pin | Modo | Resistencia Pull-Up |
|-------|-----|------|---------------------|
| **TEMP_ONEWIRE** | **PB5** | GPIO Open-Drain | 4.7 kΩ externo a VCC |

**Configuración:**
- Modo: Open-Drain Output con Pull-Up
- Velocidad: Low (máximo 2 MHz para OneWire)
- Resolución: 12-bit (0.0625°C)
- Tiempo de conversión: ~750 ms @ 12-bit

**Lectura con ROM Addressing:**
```c
// 1. Reset bus
OneWire_Reset();

// 2. Enviar comando Match ROM
OneWire_WriteByte(0x55);

// 3. Enviar ROM de 64 bits del sensor
OneWire_WriteBytes(ROM_Address, 8);

// 4. Enviar comando Convert Temperature
OneWire_WriteByte(0x44);

// 5. Esperar conversión (~750 ms)
HAL_Delay(750);

// 6. Leer scratchpad
OneWire_ReadTemperature();
```

---

## 🎮 Pedal y Shifter

### Pedal Analógico Hall

| Señal | Pin | ADC | Rango | Función |
|-------|-----|-----|-------|---------|
| **PEDAL** | **PA0** | ADC1_IN1 | 0-3.3V | Sensor Hall sin contacto |

**Configuración ADC1:**
- Resolución: 12-bit (0-4095)
- Modo: Single-ended
- Trigger: TIM3 @ 200 Hz (5 ms)
- DMA: Circular buffer (100 muestras)
- Filtro: Media móvil de 10 muestras

**Mapeo:**
- 0V (ADC=0) → 0% throttle (reposo)
- 3.3V (ADC=4095) → 100% throttle (máximo)

### Shifter Mecánico (F/N/R)

3 posiciones mutuamente excluyentes: **Forward**, **Neutral**, **Reverse**.

| Posición | Pin | Modo | Estado Activo | Función |
|----------|-----|------|---------------|---------|
| **FWD** | **PB12** | GPIO Input | LOW | Marcha adelante |
| **NEU** | **PB13** | GPIO Input | LOW | Neutral (punto muerto) |
| **REV** | *(eliminado)* | — | — | PB14 reasignado a LED_DIAG (GPIO_Output) |

**Configuración:**
- Pull-up interno habilitado
- Activo en BAJO (cuando se presiona el contacto)
- Solo UNA posición puede estar LOW simultáneamente (hardware garantiza exclusión)

**Lógica de Lectura:**
```c
if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
    shifter_state = SHIFTER_FORWARD;
else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
    shifter_state = SHIFTER_NEUTRAL;
else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
    shifter_state = SHIFTER_REVERSE;
else
    shifter_state = SHIFTER_ERROR; // Nunca debería ocurrir
```

---

## 📡 Comunicación CAN

### FDCAN1 @ 500 kbps

| Señal | Pin | Función Alternativa | Transceptor |
|-------|-----|---------------------|-------------|
| **CAN_TX** | **PA12** | FDCAN1_TX (AF9) | TJA1051T/3 (pin TX) |
| **CAN_RX** | **PA11** | FDCAN1_RX (AF9) | TJA1051T/3 (pin RX) |

**Configuración FDCAN1:**
- Modo: CAN 2.0A (11-bit IDs estándar)
- Bitrate: 500 kbps
- Prescaler: 20 (170 MHz / 20 = 8.5 MHz)
- Time Segment 1: 13 TQ
- Time Segment 2: 3 TQ
- SJW: 1 TQ
- Total: 17 TQ → 500 kbps

**Terminación CAN:**
- Resistencias de 120Ω en **ambos extremos** del bus
- STM32 (nodo 1) + ESP32-S3 (nodo 2)

---

## 🔌 Relés de Potencia

### 3 Relés Fail-Safe (Activo ALTO)

| Relé | Pin | Estado por Defecto | Función |
|------|-----|--------------------|---------|
| **RELAY_MAIN** | **PC11** | LOW (abierto) | Relé principal (power-hold) |
| **RELAY_TRAC** | **PC12** | LOW (abierto) | Alimentación motores tracción |
| **RELAY_DIR** | **PD2** | LOW (abierto) | Alimentación motor dirección |

**Configuración:**
- Modo: GPIO Output Push-Pull
- Estado inicial: LOW (relés abiertos)
- Activación: HIGH cierra el relé (energiza bobina)
- Fail-Safe: En caso de reset/fallo, relés se abren automáticamente

**Secuencia de Encendido:**
```c
// 1. Relé principal (esperar estabilización)
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
HAL_Delay(100);

// 2. Relé tracción
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
HAL_Delay(50);

// 3. Relé dirección
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
```

---

## 📊 Tabla Resumen Completa

### Clasificación por Puerto GPIO

| Puerto | Pin | Función | Periférico | Notas |
|--------|-----|---------|------------|-------|
| **PA0** | 1 | PEDAL | ADC1_IN1 | Pedal Hall analógico |
| **PA8** | 29 | PWM_FL | TIM1_CH1 | Motor tracción FL |
| **PA9** | 30 | PWM_FR | TIM1_CH2 | Motor tracción FR |
| **PA10** | 31 | PWM_RL | TIM1_CH3 | Motor tracción RL |
| **PA11** | 32 | CAN_RX | FDCAN1_RX (AF9) | Recepción CAN |
| **PA12** | 33 | CAN_TX | FDCAN1_TX (AF9) | Transmisión CAN |
| **PC3** | 11 | LPWM_FR | TIM1_CH4 (AF2) | LPWM motor tracción FR |
| **PA15** | 38 | ENC_A | TIM2_CH1 | Encoder dirección A |
| **PB0** | 14 | WHEEL_FL | GPIO/EXTI0 | Sensor rueda FL |
| **PB1** | 15 | WHEEL_FR | GPIO/EXTI1 | Sensor rueda FR |
| **PB2** | 16 | WHEEL_RL | GPIO/EXTI2 | Sensor rueda RL |
| **PB3** | 39 | ENC_B | TIM2_CH2 | Encoder dirección B |
| **PB4** | 40 | ENC_Z | GPIO/EXTI4 | Encoder índice Z |
| **PB5** | 41 | TEMP_ONEWIRE | GPIO | Bus DS18B20 (5 sensores) |
| **PB6** | 42 | I2C_SCL | I2C1_SCL | Sensores INA226 |
| **PB7** | 43 | I2C_SDA | I2C1_SDA | Sensores INA226 |
| **PB8** | 45 | LIBRE | — | GPIO disponible |
| **PB9** | 46 | LIBRE | — | GPIO disponible |
| **PB10** | 21 | WHEEL_RR | GPIO/EXTI10 | Sensor rueda RR |
| **PB12** | 25 | SHIFTER_FWD | GPIO Input | Shifter Forward |
| **PB13** | 26 | SHIFTER_NEU | GPIO Input | Shifter Neutral |
| **PB14** | 27 | LED_DIAG | GPIO Output | LED diagnóstico |
| **PC0** | 8 | DIR_FL | GPIO Output | Dirección motor FL |
| **PC1** | 9 | EN_FL | GPIO Output | Enable motor FL |
| **PC2** | 10 | DIR_FR | GPIO Output | Dirección motor FR |
| **PC3** | 11 | EN_FR | GPIO Output | Enable motor FR |
| **PC4** | 24 | DIR_RL | GPIO Output | Dirección motor RL |
| **PC5** | 25 | EN_RL | GPIO Output | Enable motor RL |
| **PC6** | 37 | DIR_RR | GPIO Output | Dirección motor RR |
| **PC7** | 38 | EN_RR | GPIO Output | Enable motor RR |
| **PC8** | 39 | PWM_STEER | TIM8_CH3 | Motor dirección |
| **PC9** | 40 | DIR_STEER | GPIO Output | Dirección steering |
| **PC10** | 51 | EN_STEER | GPIO Output | Enable steering |
| **PC11** | 52 | RELAY_MAIN | GPIO Output | Relé principal |
| **PC12** | 53 | RELAY_TRAC | GPIO Output | Relé tracción |
| **PD2** | 54 | RELAY_DIR | GPIO Output | Relé dirección |

### Resumen de Periféricos

| Periférico | Pines Usados | Función |
|------------|--------------|---------|
| **TIM1** | 4 canales (PA8, PA9, PA10, PC3) | PWM motores tracción @ 20 kHz |
| **TIM2** | 2 canales (PA15, PB3) | Encoder quadrature dirección |
| **TIM8** | 1 canal (PC8) | PWM motor dirección @ 20 kHz |
| **ADC1** | 1 canal (PA0) | Pedal analógico Hall |
| **I2C1** | SCL/SDA (PB6/PB7) | 6× INA226 vía TCA9548A |
| **FDCAN1** | TX/RX (PA12/PA11) | CAN @ 500 kbps |
| **OneWire** | PB5 | 5× DS18B20 (temperaturas) |
| **GPIO** | 26 pines | Sensores, motores, relés, shifter |

---

## ⚠️ Notas Importantes

### Conflictos de Pines Resueltos
- **PA13/PA14:** Reservados para **SWD** (ST-Link debugging). **NO USAR**.
- **PB3/PB4:** Disponibles porque JTAG está deshabilitado (solo SWD activo).
- **PA15:** Requiere remapeo de JTAG (SWJ_CFG = 0x02) para usar como TIM2_CH1.

### Limitaciones de Hardware
- **TIM1/TIM8:** Canales CH1N-CH4N (complementarios) NO usados (solo CHx positivos).
- **Dead-time insertion:** NO necesario (BTS7960 tiene protección interna).
- **Break input:** NO configurado (protección por software).

### Prioridades de Interrupción
```c
NVIC_SetPriority(FDCAN1_IT0_IRQn, 0);       // CAN: Máxima prioridad
NVIC_SetPriority(TIM2_IRQn, 1);             // Encoder: Alta prioridad
NVIC_SetPriority(EXTI0_IRQn, 2);            // Sensores rueda: Media
NVIC_SetPriority(I2C1_EV_IRQn, 3);          // I2C: Baja prioridad
```

---

## 📖 Referencias

- [STM32G474RE Datasheet](https://www.st.com/resource/en/datasheet/stm32g474re.pdf)
- [RM0440 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [TJA1051T/3 Datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf)
- [E6B2-CWZ6C Encoder](https://www.ia.omron.com/products/family/487/)
- [INA226 Datasheet](https://www.ti.com/lit/ds/symlink/ina226.pdf)
- [DS18B20 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf)

---

**Última actualización:** 2026-02-01  
**Autor:** florinzgz  
**Proyecto:** STM32-Control-Coche-Marcos
