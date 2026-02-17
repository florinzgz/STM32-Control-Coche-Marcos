# ESQUEMA COMPLETO DE CONEXIONES — Guía Cable por Cable

**Fecha:** 2026-02-15
**Propósito:** Referencia de taller para conectar todo el hardware y validar Phase 1
**Fuente:** Extraído directamente del firmware (`main.h`, `main.c`, `motor_control.c`, `sensor_manager.c`, `safety_system.c`, `can_handler.c`, `steering_centering.c`, `platformio.ini`)

---

## RESUMEN DEL SISTEMA

```
  ┌─────────────────────────────────────────────────────────────────┐
  │                     STM32G474RE (Nucleo-64)                      │
  │                        170 MHz, 3.3V                             │
  │                                                                  │
  │  TIM1 (20kHz) ──► 4× PWM motores tracción (PA8/PA9/PA10/PA11)  │
  │  TIM8 (20kHz) ──► 1× PWM motor dirección (PC8)                 │
  │  TIM2 (encoder) ◄── Encoder dirección (PA15/PB3)               │
  │  GPIO out ──► 5× DIR + 5× EN + 3× RELAY                       │
  │  EXTI ◄── 4× velocidad rueda + 1× centrado + 1× encoder Z     │
  │  I2C1 ──► TCA9548A ──► 6× INA226                              │
  │  I2C1 ──► ADS1115 ◄── Pedal acelerador (A0, 5V)               │
  │  OneWire ──► 5× DS18B20 (PB0)                                  │
  │  FDCAN1 ──► TJA1051 ──► CAN Bus ──► TJA1051 ──► ESP32-S3      │
  └─────────────────────────────────────────────────────────────────┘
```

**Total cables del STM32:** 32 GPIO + alimentación + I2C + CAN
**Componentes a conectar:** 5 BTS7960, 1 encoder, 1 ADS1115 + 1 pedal, 4 sensores rueda, 1 sensor centrado, 6 INA226, 1 TCA9548A, 5 DS18B20, 3 relés, 2 TJA1051

---

## 1) MOTORES DE TRACCIÓN — 4× BTS7960 + Motor 24V

Cada motor de tracción tiene **3 cables** del STM32 al BTS7960:

### Motor FL (Delantero Izquierdo)

| Cable | De (STM32) | A (BTS7960 FL) | Función |
|-------|-----------|----------------|---------|
| 1 | **PA8** | RPWM o LPWM | PWM 20 kHz (TIM1_CH1), controla velocidad |
| 2 | **PC0** | IN1 o IN2 | Dirección: HIGH=adelante, LOW=atrás |
| 3 | **PC5** | R_EN + L_EN (unidos) | Enable: HIGH=activado, LOW=apagado |

### Motor FR (Delantero Derecho)

| Cable | De (STM32) | A (BTS7960 FR) | Función |
|-------|-----------|----------------|---------|
| 4 | **PA9** | RPWM o LPWM | PWM 20 kHz (TIM1_CH2) |
| 5 | **PC1** | IN1 o IN2 | Dirección |
| 6 | **PC6** | R_EN + L_EN | Enable |

### Motor RL (Trasero Izquierdo)

| Cable | De (STM32) | A (BTS7960 RL) | Función |
|-------|-----------|----------------|---------|
| 7 | **PA10** | RPWM o LPWM | PWM 20 kHz (TIM1_CH3) |
| 8 | **PC2** | IN1 o IN2 | Dirección |
| 9 | **PC7** | R_EN + L_EN | Enable |

### Motor RR (Trasero Derecho)

| Cable | De (STM32) | A (BTS7960 RR) | Función |
|-------|-----------|----------------|---------|
| 10 | **PA11** | RPWM o LPWM | PWM 20 kHz (TIM1_CH4) |
| 11 | **PC3** | IN1 o IN2 | Dirección |
| 12 | **PC13** | R_EN + L_EN | Enable |

### Conexiones de potencia BTS7960 (por cada driver)

| Cable | De | A (BTS7960) | Notas |
|-------|-----|-----------|-------|
| — | Batería 24V+ | VCC (motor power) | Cable grueso ≥2.5 mm² |
| — | Batería 24V- | GND (motor power) | Cable grueso ≥2.5 mm² |
| — | Motor cable A | OUT1 | Al motor DC |
| — | Motor cable B | OUT2 | Al motor DC |
| — | STM32 3.3V o 5V | VCC (lógica) | Alimentación lógica del driver |
| — | STM32 GND | GND (lógica) | **GND COMÚN OBLIGATORIO** |

> ⚠️ **IMPORTANTE:** El GND del STM32, de los BTS7960, y de la fuente 24V deben estar conectados entre sí (GND común). Sin esto, las señales PWM y DIR no funcionan.

---

## 2) MOTOR DE DIRECCIÓN — 1× BTS7960 + Motor 12V

| Cable | De (STM32) | A (BTS7960 STEER) | Función |
|-------|-----------|-------------------|---------|
| 13 | **PC8** | RPWM o LPWM | PWM 20 kHz (TIM8_CH3) |
| 14 | **PC4** | IN1 o IN2 | Dirección: HIGH=izquierda, LOW=derecha |
| 15 | **PC9** | R_EN + L_EN | Enable: HIGH=activado |

**Alimentación:** Fuente de 12V separada para el motor de dirección.

---

## 3) ENCODER DE DIRECCIÓN — E6B2-CWZ6C (1200 PPR)

| Cable | De (Encoder) | A (STM32) | Función | Notas |
|-------|-------------|-----------|---------|-------|
| 16 | Cable A (blanco) | **PA15** | Cuadratura canal A (TIM2_CH1) | ⚠️ Adaptador 5V→3.3V necesario |
| 17 | Cable B (negro) | **PB3** | Cuadratura canal B (TIM2_CH2) | ⚠️ Adaptador 5V→3.3V necesario |
| 18 | Cable Z (naranja) | **PB4** | Pulso de índice (EXTI4) | 1 pulso por vuelta |
| — | Cable rojo (+) | 5V | Alimentación encoder | NO conectar a 3.3V |
| — | Cable azul (shield/0V) | GND | Masa encoder | GND común |

> ⚠️ **CRÍTICO:** El encoder E6B2 es de salida 5V (open-collector NPN). Las señales A y B necesitan un adaptador de nivel 5V→3.3V (o un divisor de tensión con resistencias) antes de conectar a PA15 y PB3. Conectar 5V directo al STM32 puede dañar los pines.

**Resolución resultante:** 1200 PPR × 4 (cuadratura) = **4800 cuentas por vuelta** = 0.075° por cuenta

---

## 4) SENSOR DE CENTRADO DE DIRECCIÓN — LJ12A3 Inductivo

| Cable | De (Sensor) | A (STM32) | Función | Notas |
|-------|------------|-----------|---------|-------|
| 19 | Cable señal (amarillo/azul) | **PB5** | Detección centro (EXTI5, flanco subida) | Pull-up interno activado |
| — | Cable marrón (+) | 5V–24V DC | Alimentación sensor | Según modelo |
| — | Cable azul/negro (-) | GND | Masa | GND común |

**Función:** Cuando la dirección pasa por el punto central durante el centrado automático al arrancar, este sensor genera un pulso que el STM32 detecta para calibrar la posición cero del encoder.

> ⚠️ Si el sensor es de salida PNP (activo alto a voltaje de alimentación), necesita un divisor de tensión o adaptador antes de PB5.

---

## 5) SENSORES DE VELOCIDAD DE RUEDA — 4× LJ12A3 Inductivo

| Cable | De (Sensor) | A (STM32) | Función |
|-------|------------|-----------|---------|
| 20 | Sensor rueda FL | **PA0** | Velocidad FL (EXTI0, flanco subida, pull-up) |
| 21 | Sensor rueda FR | **PA1** | Velocidad FR (EXTI1, flanco subida, pull-up) |
| 22 | Sensor rueda RL | **PA2** | Velocidad RL (EXTI2, flanco subida, pull-up) |
| 23 | Sensor rueda RR | **PB15** | Velocidad RR (EXTI15, flanco subida, pull-up) |

**Cada sensor también necesita:**
| Cable | De | A | Notas |
|-------|-----|---|-------|
| — | Marrón (+) | 5V–24V DC | Alimentación |
| — | Azul/negro (-) | GND | GND común |

**Especificaciones:**
- 6 pulsos por vuelta (6 pernos/imanes por rueda)
- Circunferencia de rueda: 1.1 m
- Debounce: 1 ms en firmware
- Cálculo: velocidad = (pulsos / 6) × 1.1 × 3.6 km/h

> ⚠️ Mismo aviso que el sensor de centrado: verificar que la señal de salida no supere 3.3V. Los sensores LJ12A3 de alimentación 5V suelen tener salida NPN (open-collector), que es compatible con el pull-up interno del STM32 a 3.3V.

---

## 6) PEDAL ACELERADOR — ADS1115 + Sensor Hall SS1324LUA-T (5V)

El sensor Hall SS1324LUA-T opera a 5V y produce una señal de 0.3V (reposo) a 4.8V (pisado a fondo). **No se puede conectar directamente al STM32** (máximo absoluto GPIO = 3.6V). Se usa un módulo ADS1115 como ADC externo I2C.

### Conexión del Pedal al ADS1115

| Cable | De (Pedal) | A (ADS1115) | Función |
|-------|-----------|-------------|---------|
| 24a | Pin 1 (VCC) | **VDD** | Alimentación 5V del sensor |
| 24b | Pin 2 (GND) | **GND** | GND común |
| 24c | Pin 3 (Señal) | **A0** | Señal analógica 0.3V–4.8V |

### Conexión del ADS1115 al STM32

| Cable | De (ADS1115) | A (STM32/Bus I2C) | Función |
|-------|-------------|-------------------|---------|
| — | VDD | **5V** | Alimentación módulo ADS1115 |
| — | GND | **GND** | GND común |
| 24d | SCL | **PB6** (bus I2C1) | Reloj I2C compartido |
| 24e | SDA | **PB7** (bus I2C1) | Datos I2C compartidos |
| — | ADDR | **GND** | Dirección I2C = 0x48 |

**Especificaciones:**
- ADS1115 es un ADC de 16 bits con I2C (dirección 0x48 con ADDR→GND)
- PGA configurado a ±6.144V (permite leer señales de 0–5V sin problema)
- LSB = 187.5 µV; rango del pedal: 1600–25600 cuentas
- Muestreo cada 50 ms en el loop del STM32
- Filtro EMA (α=0.15) + rampa máxima 50%/s (en motor_control.c)
- Calibración en firmware: PEDAL_ADC_MIN=1600 (~0.3V), PEDAL_ADC_MAX=25600 (~4.8V)

> ⚠️ **IMPORTANTE:** El ADS1115 comparte el bus I2C1 con el TCA9548A/INA226, pero tiene dirección diferente (0x48 vs 0x70/0x40), por lo que NO necesita ir a través del multiplexor.

> ⚠️ **IMPORTANTE:** El bus I2C usa pull-ups a 3.3V (en PB6/PB7). El ADS1115 alimentado a 5V acepta niveles I2C de 3.3V (VIH = 0.7 × VDD = 3.5V, compatible con 3.3V). Los pull-ups de 4.7kΩ ya existentes en el bus son suficientes.

> ⚠️ **PA3 queda libre** — ya no se usa como ADC. Disponible para futuros sensores analógicos si se habilita de nuevo el ADC interno.

---

## 7) BUS I2C — TCA9548A + 6× INA226

### Conexión I2C principal

| Cable | De (STM32) | A (TCA9548A) | Función |
|-------|-----------|-------------|---------|
| 25 | **PB6** | SCL | Reloj I2C (400 kHz, AF4) |
| 26 | **PB7** | SDA | Datos I2C (AF4) |
| — | 3.3V | VCC | Alimentación del multiplexor |
| — | GND | GND | GND común |
| — | GND | A0, A1, A2 | Dirección I2C = **0x70** |

**Resistencias pull-up:** 4.7 kΩ de PB6 a 3.3V + 4.7 kΩ de PB7 a 3.3V
(Necesarias si no están en la placa del TCA9548A)

### Canales del TCA9548A → INA226

| Canal TCA | INA226 # | Mide | Shunt | Dirección I2C (detrás del mux) |
|----------|---------|------|-------|-------------------------------|
| CH0 | INA226 #0 | Corriente Motor FL | 1 mΩ (50A max) | 0x40 |
| CH1 | INA226 #1 | Corriente Motor FR | 1 mΩ (50A max) | 0x40 |
| CH2 | INA226 #2 | Corriente Motor RL | 1 mΩ (50A max) | 0x40 |
| CH3 | INA226 #3 | Corriente Motor RR | 1 mΩ (50A max) | 0x40 |
| CH4 | INA226 #4 | Corriente Motor STEER | 1 mΩ (50A max) | 0x40 |
| CH5 | INA226 #5 | Corriente/Tensión Batería 24V | 0.5 mΩ (100A max) | 0x40 |

**Cada INA226 necesita:**
| Cable | De | A (INA226) | Notas |
|-------|-----|-----------|-------|
| — | TCA9548A CH_SDA | SDA | I2C datos (a través del mux) |
| — | TCA9548A CH_SCL | SCL | I2C reloj (a través del mux) |
| — | 3.3V | VCC | Alimentación sensor |
| — | GND | GND | GND común |
| — | Cable motor (+) | IN+ | ANTES del shunt |
| — | Cable motor (+) | IN- | DESPUÉS del shunt (al motor) |
| — | GND | A0, A1 | Dirección = 0x40 (ambos a GND) |

> **Nota:** Los INA226 se conectan en SERIE con el cable de potencia del motor. La resistencia shunt va en el cable positivo entre la fuente y el motor.

---

## 8) SENSORES DE TEMPERATURA — 5× DS18B20 (OneWire)

| Cable | De (DS18B20) | A (STM32) | Función |
|-------|-------------|-----------|---------|
| 27 | Data (amarillo) de TODOS | **PB0** | Bus OneWire compartido |
| — | VCC (rojo) | 3.3V | Alimentación |
| — | GND (negro) | GND | GND común |

**Resistencia pull-up:** 4.7 kΩ de PB0 a 3.3V (OBLIGATORIA)

**Asignación de sensores (auto-detectados por ROM):**
| Índice | Ubicación | Notas |
|--------|-----------|-------|
| 0 | Motor FL | Pegar/fijar al cuerpo del motor |
| 1 | Motor FR | Pegar/fijar al cuerpo del motor |
| 2 | Motor RL | Pegar/fijar al cuerpo del motor |
| 3 | Motor RR | Pegar/fijar al cuerpo del motor |
| 4 | Ambiente / Dirección | Temperatura general |

> **Nota:** Los DS18B20 se auto-enumeran por dirección ROM. La asignación física puede no coincidir con el orden de detección. Para Phase 1, esto es aceptable; la asignación exacta se verificará con los motores calientes.

---

## 9) CAN BUS — STM32 ↔ ESP32

### Lado STM32 → TJA1051T/3 #1

| Cable | De (STM32) | A (TJA1051 #1) | Función |
|-------|-----------|----------------|---------|
| 28 | **PB9** | TXD (pin 1) | Transmitir datos CAN (FDCAN1_TX, AF9) |
| 29 | **PB8** | RXD (pin 4) | Recibir datos CAN (FDCAN1_RX, AF9) |
| — | 5V | VCC (pin 3) | Alimentación transceiver |
| — | GND | GND (pin 2) | GND común |
| — | GND | S (pin 8) | Modo normal (NO conectar a VCC) |

### Lado ESP32-S3 → TJA1051T/3 #2

| Cable | De (ESP32) | A (TJA1051 #2) | Función |
|-------|-----------|----------------|---------|
| — | **GPIO4** | TXD (pin 1) | Transmitir datos CAN |
| — | **GPIO5** | RXD (pin 4) | Recibir datos CAN |
| — | 5V | VCC (pin 3) | Alimentación transceiver |
| — | GND | GND (pin 2) | GND común |
| — | GND | S (pin 8) | Modo normal |

### Bus CAN entre los dos transceivers

| Cable | De (TJA1051 #1) | A (TJA1051 #2) | Notas |
|-------|-----------------|----------------|-------|
| 30 | CANH (pin 7) | CANH (pin 7) | **Par trenzado** recomendado |
| 31 | CANL (pin 6) | CANL (pin 6) | **Par trenzado** recomendado |

### Terminación CAN

| Componente | Ubicación | Valor |
|-----------|-----------|-------|
| Resistencia R1 | CANH↔CANL en TJA1051 #1 (STM32) | **120 Ω** |
| Resistencia R2 | CANH↔CANL en TJA1051 #2 (ESP32) | **120 Ω** |

```
STM32 Nucleo          TJA1051 #1           Bus CAN           TJA1051 #2          ESP32-S3
┌─────────┐       ┌──────────────┐    ┌─────────────┐    ┌──────────────┐       ┌─────────┐
│ PB9(TX) ─┼──────►│ TXD    CANH ─┼────┼── CANH ─────┼────┼─ CANH    TXD│◄──────┼─ GPIO4  │
│ PB8(RX) ◄┼──────│ RXD    CANL ─┼────┼── CANL ─────┼────┼─ CANL    RXD│──────►│  GPIO5  │
│ 5V ──────┼──────│ VCC         │    │  120Ω      │    │         VCC│──────┼── 5V     │
│ GND ─────┼──────│ GND     S──►GND │    │  120Ω      │    │ GND◄──S GND│──────┼── GND    │
└─────────┘       └──────────────┘    └─────────────┘    └──────────────┘       └─────────┘
```

> ⚠️ **NUNCA** conectar PB8/PB9 directamente a CANH/CANL. El transceiver TJA1051 es OBLIGATORIO. Sin él, el CAN no funciona y puedes dañar los pines.

---

## 10) RELÉS DE POTENCIA — 3× Relé + Optoacoplador

| Cable | De (STM32) | A (Módulo relé) | Función |
|-------|-----------|-----------------|---------|
| 32 | **PC10** | IN (RELAY_MAIN) | Relé principal: alimentación general |
| 33 | **PC11** | IN (RELAY_TRAC) | Relé tracción: alimentación motores 24V |
| 34 | **PC12** | IN (RELAY_DIR) | Relé dirección: alimentación motor 12V |

**Secuencia de encendido (automática en firmware):**
```
Boot → PC10=HIGH (MAIN on) → espera 50ms → PC11=HIGH (TRAC on) → espera 20ms → PC12=HIGH (DIR on)
```

**Apagado (cualquier fallo → SAFE/ERROR):**
```
PC12=LOW → PC11=LOW → PC10=LOW (todo OFF inmediato)
```

**Conexiones de potencia del relé:**
| Cable | De | A | Notas |
|-------|-----|---|-------|
| — | Batería 24V+ | Relé MAIN (COM) | Cable grueso ≥4 mm² |
| — | Relé MAIN (NO) | Relé TRAC (COM) + Relé DIR (COM) | Se bifurca |
| — | Relé TRAC (NO) | BTS7960 tracción VCC (×4) | Alimentación motores 24V |
| — | Relé DIR (NO) | BTS7960 dirección VCC | Alimentación motor 12V (con conversor si aplica) |

> **Nota:** Los relés deben usar módulos con optoacoplador (tipo HY-M158 o similar) para aislar la lógica 3.3V del STM32 de los contactos de potencia. La señal HIGH (3.3V) del STM32 activa el optoacoplador que a su vez activa la bobina del relé.

---

## 11) ESP32-S3 — Conexiones Display (TFT ST7796)

| Cable | De (ESP32) | A (Display TFT) | Función |
|-------|-----------|-----------------|---------|
| — | **GPIO13** | MOSI (SDA) | Datos SPI |
| — | **GPIO14** | SCLK (SCL) | Reloj SPI |
| — | **GPIO15** | CS | Chip Select display |
| — | **GPIO16** | DC (A0) | Data/Command |
| — | **GPIO17** | RST | Reset display |
| — | **GPIO42** | BL | Backlight (HIGH=encendido) |
| — | **GPIO21** | TOUCH_CS | Touch panel chip select |
| — | 3.3V | VCC | Alimentación display |
| — | GND | GND | GND común |

---

## 12) TABLA COMPLETA — TODOS LOS CABLES DEL STM32

| # | Pin STM32 | Puerto | Tipo | Periférico | Conectar a | Notas |
|---|-----------|--------|------|------------|-----------|-------|
| 1 | **PA0** | GPIOA | Input | EXTI0 | Sensor velocidad rueda FL | Pull-up, flanco subida |
| 2 | **PA1** | GPIOA | Input | EXTI1 | Sensor velocidad rueda FR | Pull-up, flanco subida |
| 3 | **PA2** | GPIOA | Input | EXTI2 | Sensor velocidad rueda RL | Pull-up, flanco subida |
| 4 | **PA3** | GPIOA | — | (libre) | — No conectar — | Disponible para futuros sensores |
| 5 | **PA8** | GPIOA | AF6 | TIM1_CH1 | BTS7960 FL → RPWM/LPWM | PWM 20 kHz |
| 6 | **PA9** | GPIOA | AF6 | TIM1_CH2 | BTS7960 FR → RPWM/LPWM | PWM 20 kHz |
| 7 | **PA10** | GPIOA | AF6 | TIM1_CH3 | BTS7960 RL → RPWM/LPWM | PWM 20 kHz |
| 8 | **PA11** | GPIOA | AF6 | TIM1_CH4 | BTS7960 RR → RPWM/LPWM | PWM 20 kHz |
| 9 | **PA15** | GPIOA | AF1 | TIM2_CH1 | Encoder E6B2 canal A | ⚠️ Adaptador 5V→3.3V |
| 10 | **PB0** | GPIOB | Output | Bit-bang | Bus OneWire (5× DS18B20) | Pull-up 4.7kΩ a 3.3V |
| 11 | **PB3** | GPIOB | AF1 | TIM2_CH2 | Encoder E6B2 canal B | ⚠️ Adaptador 5V→3.3V |
| 12 | **PB4** | GPIOB | Input | EXTI4 | Encoder E6B2 índice Z | 1 pulso/vuelta |
| 13 | **PB5** | GPIOB | Input | EXTI5 | Sensor inductivo centrado | Pull-up, flanco subida |
| 14 | **PB6** | GPIOB | AF4 | I2C1_SCL | TCA9548A + ADS1115 | Pull-up 4.7kΩ, 400 kHz |
| 15 | **PB7** | GPIOB | AF4 | I2C1_SDA | TCA9548A + ADS1115 | Pull-up 4.7kΩ, 400 kHz |
| 16 | **PB8** | GPIOB | AF9 | FDCAN1_RX | TJA1051 #1 → RXD | ⚠️ Vía transceiver, NO directo |
| 17 | **PB9** | GPIOB | AF9 | FDCAN1_TX | TJA1051 #1 → TXD | ⚠️ Vía transceiver, NO directo |
| 18 | **PB15** | GPIOB | Input | EXTI15 | Sensor velocidad rueda RR | Pull-up, flanco subida |
| 19 | **PC0** | GPIOC | Output | GPIO | BTS7960 FL → DIR | HIGH/LOW = adelante/atrás |
| 20 | **PC1** | GPIOC | Output | GPIO | BTS7960 FR → DIR | HIGH/LOW |
| 21 | **PC2** | GPIOC | Output | GPIO | BTS7960 RL → DIR | HIGH/LOW |
| 22 | **PC3** | GPIOC | Output | GPIO | BTS7960 RR → DIR | HIGH/LOW |
| 23 | **PC4** | GPIOC | Output | GPIO | BTS7960 STEER → DIR | HIGH/LOW |
| 24 | **PC5** | GPIOC | Output | GPIO | BTS7960 FL → R_EN + L_EN | HIGH = motor habilitado |
| 25 | **PC6** | GPIOC | Output | GPIO | BTS7960 FR → R_EN + L_EN | HIGH = motor habilitado |
| 26 | **PC7** | GPIOC | Output | GPIO | BTS7960 RL → R_EN + L_EN | HIGH = motor habilitado |
| 27 | **PC8** | GPIOC | AF4 | TIM8_CH3 | BTS7960 STEER → RPWM/LPWM | PWM 20 kHz |
| 28 | **PC9** | GPIOC | Output | GPIO | BTS7960 STEER → R_EN + L_EN | HIGH = motor habilitado |
| 29 | **PC10** | GPIOC | Output | GPIO | Módulo relé MAIN | HIGH = ON (vía optoacoplador) |
| 30 | **PC11** | GPIOC | Output | GPIO | Módulo relé TRACCIÓN | HIGH = ON (vía optoacoplador) |
| 31 | **PC12** | GPIOC | Output | GPIO | Módulo relé DIRECCIÓN | HIGH = ON (vía optoacoplador) |
| 32 | **PC13** | GPIOC | Output | GPIO | BTS7960 RR → R_EN + L_EN | HIGH = motor habilitado |

---

## 13) LISTA DE COMPRAS / VERIFICACIÓN

### Componentes necesarios

| Qty | Componente | Uso | Notas |
|-----|-----------|-----|-------|
| 1 | STM32G474RE Nucleo-64 | Controlador principal | Nucleo board incluye regulador |
| 1 | ESP32-S3 DevKitC-1 | HMI / Display | N16R8 recomendado |
| 1 | Display TFT 480×320 | Pantalla | ST7796 driver, SPI |
| 5 | BTS7960 módulo driver | Drivers motores | 4 tracción + 1 dirección |
| 4 | Motor DC 24V | Motores tracción | Brushed DC |
| 1 | Motor DC 12V | Motor dirección | Brushed DC |
| 1 | Encoder E6B2-CWZ6C | Encoder dirección | 1200 PPR, 5V, open-collector |
| 1 | Sensor Hall SS1324LUA-T | Pedal acelerador | Salida 0.3–4.8V (5V supply) |
| 1 | ADS1115 módulo | ADC I2C para pedal | 16-bit, I2C addr 0x48 |
| 5 | Sensor inductivo LJ12A3 | 4× velocidad rueda + 1× centrado | NPN, NO |
| 6 | INA226 módulo | Sensores corriente | Breakout boards |
| 1 | TCA9548A módulo | Multiplexor I2C | Breakout board |
| 5 | DS18B20 | Sensores temperatura | Versión cable (waterproof) |
| 2 | TJA1051T/3 módulo | Transceivers CAN | 3.3V compatible |
| 3 | Módulo relé + optoacoplador | Relés potencia | HY-M158 o similar, 3.3V trigger |
| 2 | Resistencia 120 Ω | Terminación CAN | ¼W mínimo |
| 3 | Resistencia 4.7 kΩ | Pull-ups (I2C + OneWire) | PB6, PB7, PB0 |
| 1 | Adaptador nivel 5V→3.3V | Encoder | 2 canales mín (A, B) |
| 5 | Resistencia shunt | INA226 | 4× 1 mΩ + 1× 0.5 mΩ |
| 1 | Fuente 24V | Tracción | ≥20A capacidad |
| 1 | Fuente 12V | Dirección | ≥5A capacidad |
| 1 | Fuente 5V | Lógica / sensores | ≥2A capacidad |

### Herramientas necesarias para Phase 1

| Herramienta | Uso |
|------------|-----|
| Multímetro | Verificar tensiones, continuidad |
| Osciloscopio (opcional) | Verificar PWM 20 kHz, señales CAN |
| Monitor serie USB | Debug serial STM32 y ESP32 |
| Monitor CAN (PCAN, USBtin) | Verificar mensajes CAN entre MCUs |
| Cables dupont M-F, M-M | Conexiones a Nucleo y breadboard |
| Protoboard / PCB | Montaje de circuitos auxiliares |

---

## 14) AVISOS DE SEGURIDAD PARA PHASE 1

### ⚠️ ANTES DE ENCENDER

1. **Verificar GND común** — STM32, ESP32, BTS7960, fuentes de alimentación, y sensores deben compartir el mismo GND
2. **Verificar tensiones** — PA15/PB3 (encoder) ≤ 3.3V, todos los sensores ≤ 3.3V en su señal. El pedal 5V va al ADS1115, NO directamente al STM32
3. **No conectar motores todavía** — Para Phase 1, se puede probar sin motores conectados (solo verificar señales PWM con osciloscopio o LED)
4. **Conectar CAN con transceivers** — NUNCA conectar PB8/PB9 directo a cables CAN
5. **Poner resistencias pull-up** — I2C (PB6, PB7) y OneWire (PB0) no funcionan sin pull-ups

### ⚠️ SECUENCIA DE ENCENDIDO RECOMENDADA

1. Conectar 5V/3.3V (lógica) → verificar que STM32 y ESP32 arrancan
2. Verificar CAN (heartbeat cada 100 ms entre STM32↔ESP32)
3. Conectar sensores I2C (INA226 via TCA9548A)
4. Conectar sensores OneWire (DS18B20)
5. Conectar sensores de velocidad de rueda
6. Conectar encoder de dirección
7. Conectar pedal (vía ADS1115)
8. **ÚLTIMO:** Conectar alimentación de motores (24V/12V vía relés)

### ⚠️ QUÉ OBSERVAR EN PHASE 1

| Verificación | Cómo comprobar | Resultado esperado |
|-------------|----------------|-------------------|
| STM32 arranca | Monitor serie USB | Boot → Standby → Active (si ESP32 conectado) |
| CAN funciona | Monitor CAN | Heartbeat 0x001 cada 100 ms del STM32 |
| ESP32 responde | Monitor CAN | Heartbeat 0x011 cada 100 ms del ESP32 |
| I2C funciona | Monitor serie STM32 | INA226 detectados (6 canales) |
| Temperatura leída | Monitor serie / CAN | Valores de 0x202 cada 1000 ms |
| Encoder cuenta | Girar dirección manualmente | Valor de encoder cambia en CAN 0x204 |
| Centrado funciona | Observar al boot | Dirección se centra automáticamente (≤10 s) |
| IWDG no dispara | Dejar funcionar 60 s | No hay reset inesperado |
| CAN timeout | Desconectar ESP32 | STM32 pasa a SAFE en ≤250 ms |

---

## REFERENCIAS

- `docs/PINOUT_DEFINITIVO.md` — Tabla de pines detallada
- `docs/HARDWARE_WIRING_MANUAL.md` — Manual eléctrico completo
- `docs/HARDWARE_SPECIFICATION.md` — Especificaciones de componentes
- `docs/ESP32_STM32_CAN_CONNECTION.md` — Conexión CAN detallada
- `docs/HARDWARE_VALIDATION_PROCEDURE.md` — Procedimiento de validación Phase 1
- `Core/Inc/main.h` — Definiciones de pines en firmware
- `esp32/platformio.ini` — Pines ESP32 (CAN + Display)
