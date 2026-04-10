# ESQUEMA COMPLETO DE CONEXIONES — Guía Cable por Cable

**Fecha:** 2026-04-10
**Propósito:** Referencia de taller para conectar todo el hardware y validar Phase 1
**Fuente:** Extraído directamente del firmware (`project_config.h`, `main.c`, `stm32g4xx_hal_msp.c`, `motor_control.c`, `sensor_manager.c`, `safety_system.c`, `can_handler.c`, `steering_centering.c`, `obstacle_sensor.h`, `audio_manager.h`, `led_controller.h`, `shifter_input.h`, `relay_audio.h`)
**Actualizado:** Abril 2026 — CAN movido a PA11/PA12, LPWM_FR movido a PC3, sensor obstáculos cambiado de TOFSense-M a TF-Mini Plus, añadidos: DFPlayer Mini, tiras LED WS2812B, palanca de cambios MCP23017, relé de audio, LED diagnóstico PB14

---

## RESUMEN DEL SISTEMA

```
  ┌──────────────────────────────────────────────────────────────────────────────┐
  │                       STM32G474RE (Nucleo-64)                                 │
  │                          170 MHz, 3.3V                                        │
  │                                                                               │
  │  TIM1 (20kHz) ──► FL RPWM/LPWM (PA8/PA9) + FR RPWM/LPWM (PA10/PC3)        │
  │  TIM8 (20kHz) ──► RL RPWM/LPWM (PC6/PC7) + RR RPWM/LPWM (PC8/PC9)         │
  │  TIM3 (20kHz) ──► STEER RPWM/LPWM (PA6/PA7)                                │
  │  TIM2 (encoder) ◄── Encoder dirección (PA15/PB3)                             │
  │  FDCAN1 ──► PA12 TX / PA11 RX ──► TJA1051 ──► CAN Bus ──► ESP32-S3         │
  │  GPIO out ──► 2× EN GPIO (PC5/PC13) + 3+2 RELAY + EN_FR/RL/STEER→3.3V     │
  │  EXTI ◄── 4× velocidad rueda + 1× centrado + 1× encoder Z                   │
  │  I2C1 ──► TCA9548A ──► 6× INA226                                            │
  │  ADC1 ◄── Divisor ◄── Pedal (dual-sample + plausibilidad software, PA3)     │
  │  OneWire ──► 5× DS18B20 (PB0)                                                │
  │  LED_DIAG (PB14) ──► LED externo + 330Ω (diagnóstico CAN)                   │
  └──────────────────────────────────────────────────────────────────────────────┘

  ┌──────────────────────────────────────────────────────────────────────────────┐
  │                        ESP32-S3 DevKitC-1 (HMI)                              │
  │                          240 MHz, 3.3V                                       │
  │                                                                              │
  │  CAN ──► TJA1051 ──► CAN Bus ──► STM32 (GPIO4 TX, GPIO5 RX)                │
  │  SPI ──► Display TFT ST7796 480×320 (GPIO10/12/13/14/21/38/39/42)           │
  │  UART1 RX (GPIO18) ◄── TF-Mini Plus (sensor obstáculos, 115200 bps)        │
  │  UART2 ──► DFPlayer Mini (audio, GPIO43 TX / GPIO44 RX, 9600 bps)          │
  │  I2C (GPIO8 SDA / GPIO9 SCL) ──► MCP23017 ──► Palanca de cambios           │
  │  GPIO11 ──► Relé audio (active LOW, aísla altavoz)                          │
  │  GPIO47 ──► WS2812B frontal (28 LEDs, datos)                                │
  │  GPIO48 ──► WS2812B trasero (16 LEDs, datos)                                │
  └──────────────────────────────────────────────────────────────────────────────┘
```

**Total cables del STM32:** GPIO + alimentación + I2C + CAN
**Componentes a conectar:** 5 BTS7960, 1 encoder, 1 divisor resistivo + 1 pedal, 4 sensores rueda, 1 sensor centrado, 6 INA226, 1 TCA9548A, 5 DS18B20, 5 relés (3 potencia + 2 LED), 2 TJA1051, 1 LED_DIAG externo, 1 TF-Mini Plus (en ESP32), 1 DFPlayer Mini (en ESP32), 1 MCP23017 + palanca de cambios (en ESP32), 1 relé audio (en ESP32), 2 tiras WS2812B (en ESP32)

> ⚠️ **CAMBIO ARQUITECTURA (PR #120):** Los motores ya NO usan un pin DIR + un solo PWM. Ahora cada motor recibe **RPWM y LPWM directos** desde el mismo timer. **Eliminar** el cableado DIR (PC0–PC4). Los pines EN_FR, EN_RL y EN_STEER ya no son GPIO; conectar R_EN y L_EN del BTS7960 correspondiente directamente a 3.3 V.

---

## 1) MOTORES DE TRACCIÓN — 4× BTS7960 + Motor 24V

Cada motor de tracción tiene **2 cables PWM (RPWM + LPWM)** del STM32 al BTS7960. La dirección se controla haciendo RPWM > 0 / LPWM = 0 (adelante) o RPWM = 0 / LPWM > 0 (atrás). **No hay pin DIR.**

### Motor FL (Delantero Izquierdo) — TIM1 CH1/CH2, BREAK2/LOCKUP armado

| Cable | De (STM32) | A (BTS7960 FL) | Función |
|-------|-----------|----------------|---------|
| 1 | **PA8** (TIM1_CH1, AF6) | RPWM | PWM 20 kHz — adelante |
| 2 | **PA9** (TIM1_CH2, AF6) | LPWM | PWM 20 kHz — atrás |
| 3 | **PC5** (GPIO output) | R_EN + L_EN (unidos) | Enable: HIGH=activado, LOW=apagado |

> ✅ RPWM y LPWM en el **mismo** timer (TIM1). Con OCPreload activo, ambos CCR se actualizan en el mismo UEV → overlap = 0 µs.

### Motor FR (Delantero Derecho) — TIM1 CH3/CH4, BREAK2/LOCKUP armado

| Cable | De (STM32) | A (BTS7960 FR) | Función |
|-------|-----------|----------------|---------|
| 4 | **PA10** (TIM1_CH3, AF6) | RPWM | PWM 20 kHz — adelante |
| 5 | **PC3** (TIM1_CH4, AF2) | LPWM | PWM 20 kHz — atrás |
| — | **3.3V** (directo) | R_EN + L_EN (unidos) | Tied HIGH permanente — siempre habilitado |

> ⚠️ **CAMBIO CRÍTICO:** PA11 ya **NO** es LPWM_FR — ahora PA11 es **FDCAN1_RX** (bus CAN). LPWM_FR se ha movido a **PC3** (TIM1_CH4 vía AF2). Conectar PA11 al BTS7960 FR **destruirá la comunicación CAN**.
> ⚠️ PC6 ya no es EN_FR. PC6 es ahora RPWM_RL (TIM8_CH1). Conectar R_EN y L_EN del BTS7960 FR directamente a 3.3 V.

### Motor RL (Trasero Izquierdo) — TIM8 CH1/CH2, BREAK2/LOCKUP armado

| Cable | De (STM32) | A (BTS7960 RL) | Función |
|-------|-----------|----------------|---------|
| 7 | **PC6** (TIM8_CH1, AF4) | RPWM | PWM 20 kHz — adelante |
| 8 | **PC7** (TIM8_CH2, AF4) | LPWM | PWM 20 kHz — atrás |
| — | **3.3V** (directo) | R_EN + L_EN (unidos) | Tied HIGH permanente — siempre habilitado |

> ⚠️ PC7 ya no es EN_RL. Conectar R_EN y L_EN del BTS7960 RL directamente a 3.3 V.

### Motor RR (Trasero Derecho) — TIM8 CH3/CH4, BREAK2/LOCKUP armado

| Cable | De (STM32) | A (BTS7960 RR) | Función |
|-------|-----------|----------------|---------|
| 10 | **PC8** (TIM8_CH3, AF4) | RPWM | PWM 20 kHz — adelante |
| 11 | **PC9** (TIM8_CH4, AF4) | LPWM | PWM 20 kHz — atrás |
| 12 | **PC13** (GPIO output) | R_EN + L_EN (unidos) | Enable: HIGH=activado, LOW=apagado |

> ✅ RPWM y LPWM en el **mismo** timer (TIM8). Con OCPreload activo, overlap = 0 µs.

### Conexiones de potencia BTS7960 (por cada driver de tracción)

| Cable | De | A (BTS7960) | Notas |
|-------|-----|-----------|-------|
| — | Batería 24V+ (vía relé TRAC + shunt INA226) | B+ (motor power) | Cable grueso ≥2.5 mm² |
| — | Batería 24V- | B- / GND (motor power) | Cable grueso ≥2.5 mm² |
| — | Motor cable A | M+ (OUT1) | Al motor DC |
| — | Motor cable B | M- (OUT2) | Al motor DC |
| — | STM32 3.3V | VCC (lógica) | Alimentación lógica del driver (⚠️ usar 3.3V, NO 5V — ver nota abajo) |
| — | STM32 GND | GND (lógica) | **GND COMÚN OBLIGATORIO** |

### ⚡ Condensadores de protección — OBLIGATORIOS (por cada BTS7960 de tracción)

Instalar lo más cerca posible de cada módulo BTS7960 para suprimir transitorios EMI del PWM a 20 kHz:

| Componente | Valor | Conexión | Función | Sin él puede pasar... |
|-----------|-------|----------|---------|----------------------|
| C_bulk (electrolítico) | **470 µF / 35 V** | Entre **B+** y **GND** del driver | Reserva de energía; absorbe inrush y caídas bruscas de corriente al arrancar el motor | Caída de tensión en el bus 24V, reseteo del STM32 por ruido |
| C_bypass (cerámico X7R) | **100 nF / 50 V** | Entre **B+** y **GND**, lo más cerca posible del IC | Filtro de alta frecuencia (PWM 20 kHz y sus armónicos) | Interferencias en I2C, CAN y ADC del pedal |
| C_logica (cerámico X7R) | **100 nF / 50 V** | Entre **VCC lógico** y **GND lógico** del BTS7960 | Desacoplo local del buffer 74HC244; evita glitches lógicos a 20 kHz | Control intermitente o errático del motor |
| C_motor (cerámico X7R) | **100 nF / 50 V** | Entre terminales **M+** y **M-** del motor (junto al motor) | Snubber EMI; suprime picos de contra-EMF del motor brushed | Ruido RF en I2C/CAN/ADC; posible daño al BTS7960 |

```
         B+  ──────┬──────────────────► BTS7960 B+
                   │                         │
             [470µF/35V]  [100nF/50V]    Motor
              (bulk)       (bypass)     M+ ──┤
                   │           │             │ [100nF/50V]
         GND ──────┴───────────┴             │   (snubber)
                                        M- ──┤
         VCC_logic ──────┬──────────► BTS7960 VCC             │
                   [100nF/50V]          └──────────────────────┘
         GND_logic ──────┘
```

> ⚠️ **IMPORTANTE:** El GND del STM32, de los BTS7960, y de la fuente 24V deben estar conectados entre sí (GND común). Sin esto, las señales RPWM y LPWM no funcionan.
>
> ⚠️ **VCC LÓGICA = 3.3V (NO 5V):** El módulo IBT-2 incluye un buffer 74HC244 cuyo V_IH(min) = 0.7 × VCC. A VCC=5V → V_IH(min)=3.5V, lo que hace que las señales de 3.3V del STM32 estén **por debajo del umbral** y puedan no ser reconocidas. Alimentando el VCC lógico del IBT-2 desde 3.3V → V_IH(min)=2.31V, bien por debajo de 3.3V. Ver `main.h` líneas 28–33.

---

## 2) MOTOR DE DIRECCIÓN — 1× BTS7960 + Motor 12V

### Motor STEER — TIM3 CH1/CH2 (PA6/PA7, AF2)

| Cable | De (STM32) | A (BTS7960 STEER) | Función |
|-------|-----------|-------------------|---------|
| 13 | **PA6** (TIM3_CH1, AF2) | RPWM | PWM 20 kHz — izquierda |
| 14 | **PA7** (TIM3_CH2, AF2) | LPWM | PWM 20 kHz — derecha |
| — | **3.3V** (directo) | R_EN + L_EN (unidos) | Tied HIGH permanente — siempre habilitado |

**Alimentación:** Fuente de 12V separada para el motor de dirección (vía relé DIR + shunt INA226 #5).

> ⚠️ PC8 y PC9 ya no son STEER PWM/EN. Son ahora RPWM_RR y LPWM_RR (TIM8_CH3/CH4).
> ⚠️ PC4 ya no es DIR_STEER. Queda libre; dejar desconectado o como GPIO output LOW.
> ✅ RPWM y LPWM en el **mismo** timer (TIM3). TIM3 no tiene BREAK input; los fault handlers zerean CCR1/CCR2 por software.

### ⚡ Condensadores de protección — OBLIGATORIOS (BTS7960 de dirección)

Instalar lo más cerca posible del módulo BTS7960 de dirección (bus 12V):

| Componente | Valor | Conexión | Función | Sin él puede pasar... |
|-----------|-------|----------|---------|----------------------|
| C_bulk (electrolítico) | **470 µF / 25 V** | Entre **B+** (12V) y **GND** del driver | Reserva de energía; absorbe inrush y oscilaciones al girar la dirección | Oscilación en el bus 12V; comportamiento errático del volante |
| C_bypass (cerámico X7R) | **100 nF / 50 V** | Entre **B+** y **GND**, lo más cerca posible del IC | Filtro de alta frecuencia (PWM 20 kHz) | Interferencias EMI en sensores y CAN |
| C_logica (cerámico X7R) | **100 nF / 50 V** | Entre **VCC lógico** y **GND lógico** del BTS7960 | Desacoplo local del buffer 74HC244 | Control intermitente del motor de dirección |
| C_motor (cerámico X7R) | **100 nF / 50 V** | Entre terminales **M+** y **M-** del motor (junto al motor) | Snubber EMI; suprime picos de contra-EMF del motor de dirección | Ruido RF en encoder (PA15/PB3), afecta calibración de centrado |

```
         B+ (12V) ──┬──────────────────► BTS7960 STEER B+
                    │                           │
             [470µF/25V]  [100nF/50V]        Motor STEER
              (bulk)       (bypass)         M+ ──┤
                    │           │                │ [100nF/50V]
         GND ───────┴───────────┴                │   (snubber)
                                             M- ──┤
         VCC_logic ──────┬──────────► BTS7960 VCC              │
                   [100nF/50V]          └───────────────────────┘
         GND_logic ──────┘
```

> ⚠️ **ESPECIALMENTE IMPORTANTE para el motor de dirección:** El condensador snubber en los terminales del motor (M+/M-) es crítico porque el encoder E6B2-CWZ6C está físicamente cerca del motor. Sin él, la contra-EMF del motor de dirección genera ruido RF que puede corromper los pulsos del encoder y provocar fallos de centrado (`SAFETY_ERROR_CENTERING`).

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

## 6) PEDAL ACELERADOR — ADC interno dual-sample + plausibilidad software

El sensor Hall SS1324LUA-T opera a 5V y produce una señal de 0.3V (reposo) a 4.8V (pisado a fondo). Se usa el **ADC interno del STM32** con plausibilidad por software:

- **Canal ADC (rápido):** ADC interno del STM32 en PA3, a través de un divisor de tensión resistivo (5V → 3.3V). Doble lectura en ~2 µs.
- **Plausibilidad software:** Consistencia dual-sample, filtro EMA (α=0.3), validación de rango, límite de tasa de cambio.

El firmware verifica plausibilidad por software: si las dos muestras ADC consecutivas divergen, el valor sale de rango, o la tasa de cambio excede el límite físico, se activa una falta de plausibilidad que fuerza el acelerador a 0%.

### Canal primario: Divisor de tensión → PA3 (ADC1_IN4)

```
Pedal señal (0.3V–4.8V)
        │
       [R1 = 10 kΩ]
        │
        ├──────► PA3 (ADC1_IN4, STM32)
        │
       [R2 = 6.8 kΩ]
        │
       GND
```

**Cálculo del divisor:**
- Ratio = 6.8 / (10 + 6.8) = **0.4048**
- Pedal 0.3V → 0.3 × 0.4048 = **0.121V** → ADC: ~150 counts (12-bit, 3.3V ref)
- Pedal 4.8V → 4.8 × 0.4048 = **1.943V** → ADC: ~2413 counts
- Máximo posible: 5.0V × 0.4048 = 2.024V → **bien por debajo de 3.3V** ✅

| Cable | De | A | Función |
|-------|-----|---|---------|
| 24f | Pedal Pin 3 (Señal) | R1 (10 kΩ) entrada | Señal 5V del sensor |
| 24g | R1/R2 nodo medio | **PA3** (STM32) | Señal dividida ~0–2V |
| — | R2 otro extremo | **GND** | Referencia |

> ⚠️ **USAR RESISTENCIAS DE PRECISIÓN** — Tolerancia 1% o mejor. Resistencias de 5% pueden introducir error de calibración de hasta ±3% del rango del pedal.

> ⚠️ **UBICAR CERCA DEL STM32** — El divisor debe estar físicamente cerca del pin PA3 para minimizar captación de ruido PWM del motor.

### Plausibilidad por software (sin ADS1115)

> ⚠️ **CAMBIO ARQUITECTURA:** El ADS1115 externo ha sido **eliminado** del subsistema del pedal. La plausibilidad se realiza íntegramente por software usando el ADC interno del STM32, que es ~5000× más rápido y completamente determinístico.

**Especificaciones del canal ADC:**
- **Resolución:** 12-bit, ~1 µs por muestra × 2 muestras = ~2 µs total
- **PEDAL_ADC_MIN:** 150 counts (~0.3V pedal suelto, tras divisor)
- **PEDAL_ADC_MAX:** 2413 counts (~4.8V pedal pisado, tras divisor)
- **PEDAL_ADC_FAULT_LO:** 30 counts (detección circuito abierto)
- **PEDAL_ADC_FAULT_HI:** 2800 counts (detección cortocircuito)
- **PEDAL_SAMPLE_TOLERANCE:** ±30 counts entre muestras consecutivas
- **PEDAL_MAX_RATE_PCT:** 35%/ciclo (máximo cambio por ciclo de 50 ms)
- **Filtro EMA:** α=0.3 (~150 ms settling time)
- Muestreo cada 50 ms en el loop del STM32

> ⚠️ **NOTA:** PA3 ahora se usa como ADC1_IN4 (canal primario del pedal). La señal pasa por el divisor de tensión, por lo que NUNCA supera 2.1V en el pin.

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
| CH0 | INA226 #0 | Corriente Motor FL | 1.5 mΩ (50A/75mV) | 0x40 |
| CH1 | INA226 #1 | Corriente Motor FR | 1.5 mΩ (50A/75mV) | 0x40 |
| CH2 | INA226 #2 | Corriente Motor RL | 1.5 mΩ (50A/75mV) | 0x40 |
| CH3 | INA226 #3 | Corriente Motor RR | 1.5 mΩ (50A/75mV) | 0x40 |
| CH4 | INA226 #4 | Corriente/Tensión Batería 24V | 0.75 mΩ (100A/75mV) | 0x40 |
| CH5 | INA226 #5 | Corriente Motor STEER | 1.5 mΩ (50A/75mV) | 0x40 |

**Cada INA226 necesita:**
| Cable | De | A (INA226) | Notas |
|-------|-----|-----------|-------|
| — | TCA9548A CH_SDA | SDA | I2C datos (a través del mux) |
| — | TCA9548A CH_SCL | SCL | I2C reloj (a través del mux) |
| — | 3.3V | VCC | Alimentación sensor |
| — | GND | GND | GND común |
| — | Cable alimentación (+) | IN+ | ANTES del shunt (lado fuente/relé) |
| — | Cable alimentación (+) | IN- | DESPUÉS del shunt (lado driver BTS7960) |
| — | GND | A0, A1 | Dirección = 0x40 (ambos a GND) |

> **Nota:** Los INA226 de motor (CH0-CH3) y dirección (CH5) se conectan en SERIE con el cable de potencia **ANTES del driver BTS7960** (entre la salida del relé y la entrada B+ del BTS7960). La resistencia shunt va en el cable positivo entre el relé y el driver.
>
> **Nota IMPORTANTE — INA226 batería (CH4):** El INA226 de batería se conecta **ANTES del relé principal** (entre el borne + de la batería y la entrada COM del relé MAIN). Esto permite leer el voltaje de la batería en todo momento, incluso cuando el relé está abierto (sistema apagado). Si el shunt de batería se colocara después del relé, al abrir el relé se leería 0V y el firmware lo interpretaría como fallo crítico.

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
| 28 | **PA12** | TXD (pin 1) | Transmitir datos CAN (FDCAN1_TX, AF9) |
| 29 | **PA11** | RXD (pin 4) | Recibir datos CAN (FDCAN1_RX, AF9) |
| — | 5V | VCC (pin 3) | Alimentación transceiver |
| — | GND | GND (pin 2) | GND común |
| — | GND | S (pin 8) | Modo normal (NO conectar a VCC) |

> ⚠️ **CAMBIO CRÍTICO vs. versiones anteriores del documento:** Los pines CAN del STM32 ya **NO** son PB8/PB9. Son **PA11 (RX)** y **PA12 (TX)** — confirmado en `stm32g4xx_hal_msp.c` y `project_config.h`. Conectar PB8/PB9 al transceiver CAN **no funcionará** y dejará la comunicación CAN completamente inoperativa.
>
> ⚠️ PA11 está en el conector Morpho **CN10 pin 14**. PA12 está en **CN10 pin 12**.

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
│PA12(TX) ─┼──────►│ TXD    CANH ─┼────┼── CANH ─────┼────┼─ CANH    TXD│◄──────┼─ GPIO4  │
│PA11(RX) ◄┼──────│ RXD    CANL ─┼────┼── CANL ─────┼────┼─ CANL    RXD│──────►│  GPIO5  │
│ 5V ──────┼──────│ VCC         │    │  120Ω      │    │         VCC│──────┼── 5V     │
│ GND ─────┼──────│ GND     S──►GND │    │  120Ω      │    │ GND◄──S GND│──────┼── GND    │
└─────────┘       └──────────────┘    └─────────────┘    └──────────────┘       └─────────┘
```

> ⚠️ **NUNCA** conectar PA11/PA12 directamente a CANH/CANL. El transceiver TJA1051 es OBLIGATORIO. Sin él, el CAN no funciona y puedes dañar los pines.

---

## 10) RELÉS DE POTENCIA — 3× Relé + Circuito de protección

### Señales de control (STM32 → módulo relé con optoacoplador)

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

### Circuito de protección del driver de relé (por cada relé)

Los módulos de relé con optoacoplador (HY-M158 o similar) deben incluir:

```
STM32 GPIO ──► Optoacoplador ──► Transistor NPN ──► Bobina relé
                                                         │
                         Diodo flyback (1N4007)          │
                         Cátodo → VCC (5–12V) ◄──────── COM bobina
                         Ánodo  → colector NPN ◄──────── COM bobina (otro terminal)

Condensador snubber (100 nF, 250V) en paralelo con los contactos del relé
(entre COM y NO/NC) para suprimir arcos durante la conmutación.
```

| Componente | Valor | Ubicación | Función |
|-----------|-------|-----------|---------|
| Diodo flyback D1 | **1N4007** (1A, 1000V) | En paralelo con bobina del relé, polaridad inversa | Suprime pico de tensión al desactivar la bobina |
| Condensador snubber C1 | **100 nF / 250 V** (polipropileno) | En paralelo con contactos COM–NO del relé | Amortigua arco eléctrico en conmutación; 250V para margen ante picos inductivos |
| Resistencia serie R_snubber | **100 Ω / 0.5 W** | En serie con C1 (snubber RC) | Limita corriente de descarga del condensador |

> ⚠️ Si los módulos de relé ya incluyen diodo flyback en placa (muchos HY-M158 lo incluyen), verificar antes de añadir uno externo. El condensador snubber en los contactos es adicional y generalmente NO está en el módulo.

### Conexiones de potencia del relé

| Cable | De | A | Notas |
|-------|-----|---|-------|
| — | Batería 24V+ | **INA226 #4 (shunt batería)** | Cable grueso ≥4 mm², shunt ANTES del relé |
| — | INA226 #4 (salida shunt) | Relé MAIN (COM) | Cable grueso ≥4 mm² |
| — | Relé MAIN (NO) | Relé TRAC (COM) + Relé DIR (COM) | Se bifurca |
| — | Relé TRAC (NO) | **INA226 #0-#3 (shunts motor)** → BTS7960 tracción VCC (×4) | Shunts ANTES de los drivers |
| — | Relé DIR (NO) | **INA226 #5 (shunt dirección)** → BTS7960 dirección VCC | Shunt ANTES del driver (con conversor si aplica) |

> **Nota:** Los relés deben usar módulos con optoacoplador (tipo HY-M158 o similar) para aislar la lógica 3.3V del STM32 de los contactos de potencia. La señal HIGH (3.3V) del STM32 activa el optoacoplador que a su vez activa la bobina del relé.

### Condensadores de desacoplo en BTS7960 (por cada driver)

Instalar junto a cada BTS7960 para suprimir transitorios PWM en el bus de alimentación:

| Componente | Valor | Conexión | Función |
|-----------|-------|----------|---------|
| Condensador bulk C_bulk | **470 µF / 35 V** (electrolítico) | Entre B+ y GND del driver | Reserva de energía, elimina caídas bruscas |
| Condensador cerámico C_bypass | **100 nF / 50 V** (X7R/X5R) | Entre B+ y GND, lo más cerca posible del IC | Filtro de alta frecuencia (PWM 20 kHz) |

> ⚠️ Sin estos condensadores, los transitorios de PWM a 20 kHz pueden propagarse al bus 24V y generar interferencias en sensores o resetear el STM32.

---

## 10b) RELÉS LED — 2× Relé para tiras WS2812B

El STM32 controla la alimentación 5V de las tiras LED WS2812B (frontal y trasera) mediante dos relés adicionales. El ESP32 genera la señal de datos para los WS2812B; el STM32 controla el corte de energía por seguridad.

### Señales de control (STM32 → módulo relé con optoacoplador)

| Cable | De (STM32) | A (Módulo relé) | Función |
|-------|-----------|-----------------|---------|
| 35 | **PB10** | IN (RELAY_LED) | Relé LED frontal: alimentación 5V tira WS2812B frontal (28 LEDs) |
| 36 | **PB11** | IN (RELAY_LED_REAR) | Relé LED trasero: alimentación 5V tira WS2812B trasera (16 LEDs) |

**Control por CAN:** El ESP32 envía el comando CAN ID **0x120** para activar/desactivar:
- Byte 0 = relé frontal (PB10): 0x01=ON, 0x00=OFF
- Byte 1 = relé trasero (PB11): 0x01=ON, 0x00=OFF

**Conexiones de potencia:**

| Cable | De | A | Notas |
|-------|-----|---|-------|
| — | Fuente 5V (LED) | Relé LED frontal (COM) | Fuente ≥3A para 28 LEDs |
| — | Relé LED frontal (NO) | Tira WS2812B frontal (VCC) | 5V conmutados |
| — | Fuente 5V (LED) | Relé LED trasero (COM) | Fuente ≥2A para 16 LEDs |
| — | Relé LED trasero (NO) | Tira WS2812B trasera (VCC) | 5V conmutados |

> ⚠️ Los relés LED requieren el mismo circuito de protección (diodo flyback, snubber RC) que los relés de potencia (sección 10). Usar módulos con optoacoplador para aislar la lógica 3.3V.
>
> **Referencia firmware:** `PIN_RELAY_LED` (PB10) y `PIN_RELAY_LED_REAR` (PB11) en `main.h`.

---

## 11) ESP32-S3 — Conexiones Display (TFT ST7796)

| Cable | De (ESP32) | A (Display TFT) | Función |
|-------|-----------|-----------------|---------|
| — | **GPIO13** | MOSI (SDA) | Datos SPI |
| — | **GPIO14** | SCLK (SCL) | Reloj SPI |
| — | **GPIO10** | CS | Chip Select display |
| — | **GPIO39** | DC (A0) | Data/Command |
| — | **GPIO38** | RST | Reset display |
| — | **GPIO42** | BL | Backlight (HIGH=encendido) |
| — | **GPIO12** | SDO (MISO) / T_DO | Datos SPI desde display/touch |
| — | **GPIO21** | TOUCH_CS | Touch panel chip select |
| — | 3.3V | VCC | Alimentación display |
| — | GND | GND | GND común |

---

## 12) ESP32-S3 — Sensor de Obstáculos TF-Mini Plus (UART1)

El sensor **Benewake TF-Mini Plus** es un LiDAR Time-of-Flight de punto único conectado directamente al ESP32-S3 vía UART. Proporciona detección de obstáculos con una lectura de distancia a ~100 Hz.

> ⚠️ **CAMBIO DE SENSOR:** El sensor anterior era el TOFSense-M S (Nooploop) con protocolo de 921600 bps y divisor de tensión obligatorio. Ha sido reemplazado por el **TF-Mini Plus** con 115200 bps y **conexión directa sin divisor** (3.3V TTL nativo).

### Conexión TF-Mini Plus → ESP32-S3

| Cable | De (TF-Mini Plus, 4 hilos) | A (ESP32-S3) | Función |
|-------|---------------------------|-------------|---------|
| — | **Rojo (VCC)** | **5V** (regulador o VBUS) | Alimentación sensor (⚠️ 5V obligatorio, NO 3.3V) |
| — | **Negro (GND)** | **GND** | GND común con ESP32 |
| — | **Blanco (RX)** | **No conectado** | Recepción unidireccional — no se envían comandos al sensor |
| — | **Verde (TX)** | **GPIO18** (UART1 RX) **directo** | Sensor TX → ESP32 RX. ✅ Conexión directa (3.3V TTL nativo) |

### Diagrama de conexión

```
                    TF-Mini Plus (4 hilos)
                   ┌──────────────────────┐
 5V (regulador) ───┤ VCC  (rojo)          │
                   │                      │     ⚠ VCC = 5V obligatorio
 GND ──────────────┤ GND  (negro)         │     ✅ TX = 3.3V TTL nativo
                   │                      │
           n/c ────┤ RX   (blanco)        │     (no se envían comandos)
                   │                      │
                   │ TX   (verde) ────────┼────── ESP32 GPIO18 (UART1 RX)
                   └──────────────────────┘       ✅ Conexión directa
                                                  (sin divisor de tensión)
```

> ✅ **Sin divisor de tensión:** A diferencia del TOFSense-M (que producía 3.5–3.6V), el TF-Mini Plus transmite a **3.3V TTL nativo**, compatible directamente con los GPIO del ESP32-S3. Hay margen de 300 mV respecto al máximo absoluto (3.6V).

### Especificaciones de comunicación

| Parámetro | Valor |
|-----------|-------|
| **Interfaz** | UART (modo activo, transmisión continua) |
| **Baudrate** | **115200 bps** (factory default) |
| **Formato** | 8N1 (8 data bits, sin paridad, 1 stop bit) |
| **Nivel lógico UART** | 3.3V TTL nativo — conexión directa a ESP32 |
| **Frecuencia de datos** | ~100 Hz (una trama cada ~10 ms) |
| **Protocolo** | Trama 9 bytes: `[0x59][0x59][DIST_L][DIST_H][STR_L][STR_H][TEMP_L][TEMP_H][CHK]` |
| **Rango útil** | 100 mm – 12000 mm (10 cm – 12 m) |
| **Consumo** | ~120 mA típico (pico ~140 mA) |
| **Dimensiones** | 35 × 21 × 18 mm |

### Referencia en firmware

- `esp32/src/sensors/obstacle_sensor.h` — Configuración: `rxPin = 18`, `txPin = -1`, `baudRate = 115200`, `SENSOR_TYPE = SENSOR_TYPE_TFMINI`
- `esp32/src/sensors/obstacle_sensor.cpp` — Parser del protocolo TF-Mini Plus, detección de sensor atascado
- `esp32/src/main.cpp` — Inicialización: `obstacle_sensor::init()` en `setup()`
- `docs/TFMINI_PLUS_WIRING_GUIDE.md` — Guía completa de cableado y troubleshooting

> ⚠️ **IMPORTANTE:** El sensor requiere 5V en VCC para funcionar. A 3.3V no arranca o produce lecturas inválidas. ¡Pero la señal TX es 3.3V — NO necesita divisor ni level shifter!
>
> ⚠️ Se recomienda un condensador de desacoplo de **100 nF** entre VCC y GND del sensor, lo más cerca posible del conector, para filtrar ruido de alimentación.

---

## 13) TABLA COMPLETA — TODOS LOS CABLES DEL STM32

> ⚠️ **Tabla actualizada Abril 2026.** Cambios críticos vs. versiones anteriores:
> - **CAN:** PA11 (RX) / PA12 (TX) — ya NO son PB8/PB9
> - **LPWM_FR:** PC3 (TIM1_CH4, AF2) — ya NO es PA11
> - **PC3:** Reutilizado como LPWM_FR (ya no es LIBRE/DIR_RR)
> - **PB14:** LED diagnóstico externo (nuevo)
> - **PA5:** LED LD2 de la Nucleo (heartbeat/estado CAN)

| # | Pin STM32 | Puerto | Tipo | Periférico | Conectar a | Notas |
|---|-----------|--------|------|------------|-----------|-------|
| 1 | **PA0** | GPIOA | Input | EXTI0 | Sensor velocidad rueda FL | Pull-up, flanco subida |
| 2 | **PA1** | GPIOA | Input | EXTI1 | Sensor velocidad rueda FR | Pull-up, flanco subida |
| 3 | **PA2** | GPIOA | Input | EXTI2 | Sensor velocidad rueda RL | Pull-up, flanco subida |
| 4 | **PA3** | GPIOA | Analog | ADC1_IN4 | Pedal (divisor) | Canal primario, señal dividida 0–2V |
| 5 | **PA5** | GPIOA | Output | GPIO | LED LD2 (en Nucleo) | Heartbeat CAN: OK=flash 50ms/2s, FAIL=blink 1Hz |
| 6 | **PA6** | GPIOA | AF2 | TIM3_CH1 | BTS7960 STEER → **RPWM** | PWM 20 kHz — izquierda |
| 7 | **PA7** | GPIOA | AF2 | TIM3_CH2 | BTS7960 STEER → **LPWM** | PWM 20 kHz — derecha |
| 8 | **PA8** | GPIOA | AF6 | TIM1_CH1 | BTS7960 FL → **RPWM** | PWM 20 kHz — adelante |
| 9 | **PA9** | GPIOA | AF6 | TIM1_CH2 | BTS7960 FL → **LPWM** | PWM 20 kHz — atrás |
| 10 | **PA10** | GPIOA | AF6 | TIM1_CH3 | BTS7960 FR → **RPWM** | PWM 20 kHz — adelante |
| 11 | **PA11** | GPIOA | AF9 | **FDCAN1_RX** | TJA1051 #1 → **RXD** | ⚠️ CAN RX — Morpho CN10 pin 14 |
| 12 | **PA12** | GPIOA | AF9 | **FDCAN1_TX** | TJA1051 #1 → **TXD** | ⚠️ CAN TX — Morpho CN10 pin 12 |
| 13 | **PA15** | GPIOA | AF1 | TIM2_CH1 | Encoder E6B2 canal A | ⚠️ Adaptador 5V→3.3V |
| 14 | **PB0** | GPIOB | Output OD | Bit-bang | Bus OneWire (5× DS18B20) | Pull-up 4.7kΩ a 3.3V |
| 15 | **PB3** | GPIOB | AF1 | TIM2_CH2 | Encoder E6B2 canal B | ⚠️ Adaptador 5V→3.3V |
| 16 | **PB4** | GPIOB | Input | GPIO (polled) | Encoder E6B2 índice Z | Pull-up, 1 pulso/vuelta |
| 17 | **PB5** | GPIOB | Input | EXTI5 | Sensor inductivo centrado | Pull-up, flanco subida |
| 18 | **PB6** | GPIOB | AF4 | I2C1_SCL | TCA9548A (INA226) | Pull-up 4.7kΩ, 400 kHz |
| 19 | **PB7** | GPIOB | AF4 | I2C1_SDA | TCA9548A (INA226) | Pull-up 4.7kΩ, 400 kHz |
| 20 | **PB10** | GPIOB | Output | GPIO | Módulo relé LED frontal | HIGH = ON (tira WS2812B 28 LEDs) |
| 21 | **PB11** | GPIOB | Output | GPIO | Módulo relé LED trasero | HIGH = ON (tira WS2812B 16 LEDs) |
| 22 | **PB14** | GPIOB | Output | GPIO | **LED_DIAG** (externo) | LED + 330Ω en Morpho CN10 pin 28. CAN OK=ON, FAIL=OFF |
| 23 | **PB15** | GPIOB | Input | EXTI15 | Sensor velocidad rueda RR | Pull-up, flanco subida |
| 24 | **PC0** | GPIOC | — | **LIBRE** | ~~DIR_FL~~ — desconectado | Pin liberado |
| 25 | **PC1** | GPIOC | — | **LIBRE** | ~~DIR_FR~~ — desconectado | Pin liberado |
| 26 | **PC2** | GPIOC | — | **LIBRE** | ~~DIR_RL~~ — desconectado | Pin liberado |
| 27 | **PC3** | GPIOC | **AF2** | **TIM1_CH4** | BTS7960 FR → **LPWM** | ⚠️ PWM 20 kHz — atrás (ya NO es LIBRE) |
| 28 | **PC4** | GPIOC | — | **LIBRE** | ~~DIR_STEER~~ — desconectado | Pin liberado |
| 29 | **PC5** | GPIOC | Output | GPIO | BTS7960 FL → R_EN + L_EN | HIGH = motor habilitado |
| 30 | **PC6** | GPIOC | AF4 | TIM8_CH1 | BTS7960 RL → **RPWM** | PWM 20 kHz — adelante |
| 31 | **PC7** | GPIOC | AF4 | TIM8_CH2 | BTS7960 RL → **LPWM** | PWM 20 kHz — atrás |
| 32 | **PC8** | GPIOC | AF4 | TIM8_CH3 | BTS7960 RR → **RPWM** | PWM 20 kHz — adelante |
| 33 | **PC9** | GPIOC | AF4 | TIM8_CH4 | BTS7960 RR → **LPWM** | PWM 20 kHz — atrás |
| 34 | **PC10** | GPIOC | Output | GPIO | Módulo relé MAIN | HIGH = ON (vía optoacoplador) |
| 35 | **PC11** | GPIOC | Output | GPIO | Módulo relé TRACCIÓN | HIGH = ON (vía optoacoplador) |
| 36 | **PC12** | GPIOC | Output | GPIO | Módulo relé DIRECCIÓN | HIGH = ON (vía optoacoplador) |
| 37 | **PC13** | GPIOC | Output | GPIO | BTS7960 RR → R_EN + L_EN | HIGH = motor habilitado |

### Conexiones hardware (no GPIO)

| Componente | Desde | Hasta | Valor |
|-----------|-------|-------|-------|
| R_EN + L_EN (BTS7960 FR) | — | **3.3V** | Tied HIGH permanente |
| R_EN + L_EN (BTS7960 RL) | — | **3.3V** | Tied HIGH permanente |
| R_EN + L_EN (BTS7960 STEER) | — | **3.3V** | Tied HIGH permanente |

---

## 14) LISTA DE COMPRAS / VERIFICACIÓN

### Componentes principales

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
| 1 | Resistencia 10 kΩ (1%) | Divisor pedal R1 | Canal primario ADC, ¼W |
| 1 | Resistencia 6.8 kΩ (1%) | Divisor pedal R2 | Canal primario ADC, ¼W |
| 5 | Sensor inductivo LJ12A3 | 4× velocidad rueda + 1× centrado | NPN, NO |
| 6 | INA226 módulo | Sensores corriente | Breakout boards |
| 1 | TCA9548A módulo | Multiplexor I2C | Breakout board |
| 5 | DS18B20 | Sensores temperatura | Versión cable (waterproof) |
| 2 | TJA1051T/3 módulo | Transceivers CAN | 3.3V compatible |
| 1 | TF-Mini Plus (Benewake) | Sensor obstáculos LiDAR punto único | Conectado a ESP32-S3 GPIO18 (UART1), VCC=5V, 115200 bps, conexión directa 3.3V |
| 5 | Módulo relé + optoacoplador | Relés potencia y LED | HY-M158 o similar, 3.3V trigger (3× potencia + 2× LED) |
| 2 | Resistencia 120 Ω | Terminación CAN | ¼W mínimo |
| 3 | Resistencia 4.7 kΩ | Pull-ups (I2C + OneWire) | PB6, PB7, PB0 |
| 1 | Adaptador nivel 5V→3.3V | Encoder A/B | 2 canales mín (PA15, PB3) |
| 6 | Resistencia shunt | INA226 | 5× 1.5 mΩ (50A/75mV, motores+dirección) + 1× 0.75 mΩ (100A/75mV, batería) |
| 1 | Fuente 24V | Tracción | ≥20A capacidad |
| 1 | Fuente 12V | Dirección | ≥5A capacidad |
| 1 | Fuente 5V | Lógica / sensores | ≥2A capacidad |

### Componentes de protección (OBLIGATORIOS — PR #120)

#### Condensadores por motor (5 motores × 4 condensadores = 20 condensadores en total)

| Qty | Componente | Valor | Ubicación | Función |
|-----|-----------|-------|-----------|---------|
| 4 | Condensador bulk (motores tracción) | **470 µF / 35 V** electrolítico | Entre B+ y GND de cada BTS7960 de tracción (FL, FR, RL, RR) | Reserva energía, absorbe inrush al arrancar, filtra transitorios DC |
| 1 | Condensador bulk (dirección) | **470 µF / 25 V** electrolítico | Entre B+ (12V) y GND del BTS7960 STEER | Igual para el bus de 12V del motor de dirección |
| 5 | Condensador bypass potencia | **100 nF / 50 V** X7R cerámico | Entre B+ y GND de cada BTS7960 (todos), lo más cerca del IC | Filtra armónicos del PWM 20 kHz en el bus de potencia |
| 5 | Condensador bypass lógica | **100 nF / 50 V** X7R cerámico | Entre VCC lógico y GND lógico de cada BTS7960 (junto al conector VCC) | Desacoplo local del 74HC244; evita glitches a 20 kHz |
| 5 | Condensador snubber motor | **100 nF / 50 V** X7R cerámico | Entre terminales **M+** y **M-** de cada motor (junto al motor, NO en el BTS7960) | Suprime picos de contra-EMF del motor brushed; protege encoder y señales I2C/CAN |

#### Diodos y snubbers para relés

| Qty | Componente | Valor | Uso | Notas |
|-----|-----------|-------|-----|-------|
| 5 | Diodo flyback bobina relé | **1N4007** (1A, 1000V) | Uno por bobina de relé (MAIN, TRAC, DIR, LED_F, LED_R) | En paralelo con la bobina, polaridad inversa; incluido en muchos módulos HY-M158 — verificar |
| 5 | Condensador snubber contacto relé | **100 nF / 250 V** (polipropileno) | En paralelo con contactos COM–NO de cada relé (5 relés) | Reduce arcos en conmutación; 250V para margen ante picos inductivos |
| 5 | Resistencia snubber contacto relé | **100 Ω / 0.5 W** | En serie con el condensador snubber de contacto (RC snubber) | Limita corriente de descarga del condensador |

#### Protección adicional (opcional pero recomendada)

| Qty | Componente | Valor | Uso | Notas |
|-----|-----------|-------|-----|-------|
| 5 | Diodo Schottky volante libre motor | **SB560** o similar (5A, 60V) | Uno por motor, entre terminales M+ y M- (en paralelo con el motor) | El BTS7960 ya incluye diodos internos; el diodo externo añade protección extra ante contra-EMF prolongada. Usar ≥60V por margen sobre 24V |

> ⚠️ Los diodos flyback en la bobina de relé son críticos. Sin ellos, al desactivar el relé se genera un pico de cientos de voltios que puede destruir el transistor del módulo optoacoplador.
>
> ⚠️ Los condensadores bulk en cada BTS7960 son especialmente importantes con la nueva arquitectura RPWM/LPWM directo, ya que el driver maneja transiciones de dirección con mayor frecuencia.
>
> ⚠️ El condensador snubber en los terminales del motor de dirección (M+/M-) es especialmente crítico porque el encoder E6B2-CWZ6C está físicamente cerca: sin él, la contra-EMF puede corromper los pulsos del encoder y provocar `SAFETY_ERROR_CENTERING`.

### Pines liberados (PC0, PC1, PC2, PC4) — ya no se cablean

> ⚠️ **PC3 ya NO está libre** — ahora es LPWM_FR (TIM1_CH4, AF2). Solo quedan 4 pines libres.

| Pin | Uso anterior | Estado actual |
|-----|-------------|---------------|
| PC0 | DIR_FL (GPIO OUT) | **LIBRE** — dejar desconectado o como GPIO output LOW |
| PC1 | DIR_FR (GPIO OUT) | **LIBRE** — dejar desconectado o como GPIO output LOW |
| PC2 | DIR_RL (GPIO OUT) | **LIBRE** — dejar desconectado o como GPIO output LOW |
| PC3 | DIR_RR (GPIO OUT) | **⚠️ REUTILIZADO:** Ahora es **LPWM_FR** (TIM1_CH4, AF2) — PWM activo, NO desconectar |
| PC4 | DIR_STEER (GPIO OUT) | **LIBRE** — dejar desconectado o como GPIO output LOW |

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

## 15) AVISOS DE SEGURIDAD PARA PHASE 1

### ⚠️ ANTES DE ENCENDER

1. **Verificar GND común** — STM32, ESP32, BTS7960, fuentes de alimentación, y sensores deben compartir el mismo GND
2. **Verificar tensiones** — PA15/PB3 (encoder) ≤ 3.3V, PA3 (pedal divisor) ≤ 2.1V. PA6/PA7/PA8/PA9/PA10/PA11/PC6/PC7/PC8/PC9 son salidas PWM (NO conectar a señales externas). El pedal 5V va al divisor resistivo, NO directamente al STM32
3. **No conectar motores todavía** — Para Phase 1, se puede probar sin motores conectados (solo verificar señales RPWM/LPWM con osciloscopio o LED en PA8/PA9/PA10/PA11/PC6/PC7/PC8/PC9/PA6/PA7)
4. **Conectar CAN con transceivers** — NUNCA conectar PB8/PB9 directo a cables CAN
5. **Poner resistencias pull-up** — I2C (PB6, PB7) y OneWire (PB0) no funcionan sin pull-ups

### ⚠️ SECUENCIA DE ENCENDIDO RECOMENDADA

1. Conectar 5V/3.3V (lógica) → verificar que STM32 y ESP32 arrancan
2. Verificar CAN (heartbeat cada 100 ms entre STM32↔ESP32)
3. Conectar sensores I2C (INA226 via TCA9548A)
4. Conectar sensores OneWire (DS18B20)
5. Conectar sensores de velocidad de rueda
6. Conectar encoder de dirección
7. Conectar pedal (divisor resistivo → PA3)
8. Conectar TOFSense-M S al ESP32-S3 (VCC=5V, GND, TX→GPIO18) → verificar `[OBSTACLE] TOFSense-M initialized` en monitor serie
9. Verificar que EN_FR, EN_RL y EN_STEER (BTS7960 R_EN/L_EN) están cableados a 3.3V
10. **ÚLTIMO:** Conectar alimentación de motores (24V/12V vía relés)

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
| TOFSense-M lee | Monitor serie ESP32 | `[OBSTACLE] TOFSense-M initialized` + estado VALID tras warmup |

---

## REFERENCIAS

- `docs/MATERIALES_POR_MODULO.md` — **Lista completa de materiales por módulo y por conexión (BOM)**
- `docs/PINOUT_DEFINITIVO.md` — Tabla de pines detallada
- `docs/HARDWARE_WIRING_MANUAL.md` — Manual eléctrico completo
- `docs/HARDWARE_SPECIFICATION.md` — Especificaciones de componentes
- `docs/ESP32_STM32_CAN_CONNECTION.md` — Conexión CAN detallada
- `docs/TOFSENSE_M_WIRING_GUIDE.md` — Guía de conexión TOFSense-M S (sensor obstáculos)
- `docs/HARDWARE_VALIDATION_PROCEDURE.md` — Procedimiento de validación Phase 1
- `Core/Inc/project_config.h` — **Fuente de verdad de definiciones de pines en firmware**
- `esp32/platformio.ini` — Pines ESP32 (CAN + Display)
- `esp32/src/sensors/obstacle_sensor.h` — Configuración UART del TOFSense-M (GPIO18, 921600 bps)
