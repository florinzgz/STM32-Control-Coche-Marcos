# ESQUEMA COMPLETO DE CONEXIONES — Guía Cable por Cable

**Fecha:** 2026-02-25
**Propósito:** Referencia de taller para conectar todo el hardware y validar Phase 1
**Fuente:** Extraído directamente del firmware (`main.h`, `main.c`, `motor_control.c`, `sensor_manager.c`, `safety_system.c`, `can_handler.c`, `steering_centering.c`, `platformio.ini`, `obstacle_sensor.h`)
**Actualizado:** PR #120 — arquitectura RPWM/LPWM directo por motor (mismo timer), BREAK2/LOCKUP hardware; añadido sensor TOFSense-M S (ESP32-S3, UART1 GPIO18)

---

## RESUMEN DEL SISTEMA

```
  ┌──────────────────────────────────────────────────────────────────────────────┐
  │                       STM32G474RE (Nucleo-64)                                 │
  │                          170 MHz, 3.3V                                        │
  │                                                                               │
  │  TIM1 (20kHz) ──► FL RPWM/LPWM (PA8/PA9) + FR RPWM/LPWM (PA10/PA11)        │
  │  TIM8 (20kHz) ──► RL RPWM/LPWM (PC6/PC7) + RR RPWM/LPWM (PC8/PC9)         │
  │  TIM3 (20kHz) ──► STEER RPWM/LPWM (PA6/PA7)                                │
  │  TIM2 (encoder) ◄── Encoder dirección (PA15/PB3)                             │
  │  GPIO out ──► 2× EN GPIO (PC5/PC13) + 3× RELAY + EN_FR/RL/STEER→3.3V       │
  │  EXTI ◄── 4× velocidad rueda + 1× centrado + 1× encoder Z                   │
  │  I2C1 ──► TCA9548A ──► 6× INA226                                            │
  │  ADC1 ◄── Divisor ◄── Pedal (dual-sample + plausibilidad software, PA3)         │
  │  OneWire ──► 5× DS18B20 (PB0)                                                │
  │  FDCAN1 ──► TJA1051 ──► CAN Bus ──► TJA1051 ──► ESP32-S3                   │
  └──────────────────────────────────────────────────────────────────────────────┘

  ┌──────────────────────────────────────────────────────────────────────────────┐
  │                        ESP32-S3 DevKitC-1 (HMI)                              │
  │                          240 MHz, 3.3V                                       │
  │                                                                              │
  │  CAN ──► TJA1051 ──► CAN Bus ──► STM32 (GPIO4 TX, GPIO5 RX)                │
  │  SPI ──► Display TFT ST7796 480×320 (GPIO10/12/13/14/21/38/39/42)           │
  │  UART1 RX (GPIO18) ◄── TOFSense-M S (sensor obstáculos, 921600 bps)        │
  │  UART2 ──► DFPlayer Mini (audio)                                             │
  └──────────────────────────────────────────────────────────────────────────────┘
```

**Total cables del STM32:** GPIO + alimentación + I2C + CAN
**Componentes a conectar:** 5 BTS7960, 1 encoder, 1 divisor resistivo + 1 pedal, 4 sensores rueda, 1 sensor centrado, 6 INA226, 1 TCA9548A, 5 DS18B20, 3 relés, 2 TJA1051, 1 TOFSense-M S (en ESP32)

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
| 5 | **PA11** (TIM1_CH4, AF6) | LPWM | PWM 20 kHz — atrás |
| — | **3.3V** (directo) | R_EN + L_EN (unidos) | Tied HIGH permanente — siempre habilitado |

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

### Conexiones de potencia BTS7960 (por cada driver)

| Cable | De | A (BTS7960) | Notas |
|-------|-----|-----------|-------|
| — | Batería 24V+ | VCC (motor power) | Cable grueso ≥2.5 mm² |
| — | Batería 24V- | GND (motor power) | Cable grueso ≥2.5 mm² |
| — | Motor cable A | OUT1 | Al motor DC |
| — | Motor cable B | OUT2 | Al motor DC |
| — | STM32 3.3V o 5V | VCC (lógica) | Alimentación lógica del driver |
| — | STM32 GND | GND (lógica) | **GND COMÚN OBLIGATORIO** |

> ⚠️ **IMPORTANTE:** El GND del STM32, de los BTS7960, y de la fuente 24V deben estar conectados entre sí (GND común). Sin esto, las señales RPWM y LPWM no funcionan.

---

## 2) MOTOR DE DIRECCIÓN — 1× BTS7960 + Motor 12V

### Motor STEER — TIM3 CH1/CH2 (PA6/PA7, AF2)

| Cable | De (STM32) | A (BTS7960 STEER) | Función |
|-------|-----------|-------------------|---------|
| 13 | **PA6** (TIM3_CH1, AF2) | RPWM | PWM 20 kHz — izquierda |
| 14 | **PA7** (TIM3_CH2, AF2) | LPWM | PWM 20 kHz — derecha |
| — | **3.3V** (directo) | R_EN + L_EN (unidos) | Tied HIGH permanente — siempre habilitado |

**Alimentación:** Fuente de 12V separada para el motor de dirección.

> ⚠️ PC8 y PC9 ya no son STEER PWM/EN. Son ahora RPWM_RR y LPWM_RR (TIM8_CH3/CH4).
> ⚠️ PC4 ya no es DIR_STEER. Queda libre; dejar desconectado o como GPIO output LOW.
> ✅ RPWM y LPWM en el **mismo** timer (TIM3). TIM3 no tiene BREAK input; los fault handlers zerean CCR1/CCR2 por software.

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
| CH0 | INA226 #0 | Corriente Motor FL | 1 mΩ (50A max) | 0x40 |
| CH1 | INA226 #1 | Corriente Motor FR | 1 mΩ (50A max) | 0x40 |
| CH2 | INA226 #2 | Corriente Motor RL | 1 mΩ (50A max) | 0x40 |
| CH3 | INA226 #3 | Corriente Motor RR | 1 mΩ (50A max) | 0x40 |
| CH4 | INA226 #4 | Corriente/Tensión Batería 24V | 0.5 mΩ (100A max) | 0x40 |
| CH5 | INA226 #5 | Corriente Motor STEER | 1 mΩ (50A max) | 0x40 |

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

## 12) ESP32-S3 — Sensor de Obstáculos TOFSense-M S (UART1)

El sensor TOFSense-M S (Nooploop) es un LiDAR 8×8 Time-of-Flight conectado directamente al ESP32-S3 vía UART. Proporciona detección de obstáculos con 64 píxeles de distancia a ~10 Hz.

### Conexión TOFSense-M S → ESP32-S3

| Cable | De (TOFSense-M, conector GH1.25) | A (ESP32-S3) | Función |
|-------|----------------------------------|-------------|---------|
| — | **Pin 1 (VCC)** | **5V** (regulador o fuente) | Alimentación sensor (⚠️ 5V obligatorio, NO 3.3V) |
| — | **Pin 2 (GND)** | **GND** | GND común con ESP32 |
| — | **Pin 3 (RX)** | **No conectado** | Recepción unidireccional — no se envían comandos al sensor |
| — | **Pin 4 (TX)** | **GPIO18** (UART1 RX) via divisor | Sensor TX → divisor R1=1kΩ+R2=4.7kΩ → ESP32 RX. ⚠️ Divisor obligatorio (TX medido = 3.5–3.6V) |

### Diagrama de conexión

```
                    TOFSense-M S (GH1.25)
                   ┌──────────────────────┐
 5V (regulador) ───┤ VCC  (pin 1)         │
                   │                      │     ⚠ VCC = 5V obligatorio
 GND ──────────────┤ GND  (pin 2)         │     ⚠ TX medido = 3.5–3.6V
                   │                      │
           n/c ────┤ RX   (pin 3)         │     (no se envían comandos)
                   │                      │
                   │ TX   (pin 4) ────────┼──┐
                   └──────────────────────┘  │  Divisor de tensión
                         │    │              │  OBLIGATORIO
                        100nF (desacoplo)    │
                                        ┌────┘
                                        │
                                   ┌────┴────┐
                                   │  R1=1kΩ │ (serie)
                                   └────┬────┘
                                        │
                                        ├──── ESP32 GPIO18 (UART1 RX)
                                        │
                                   ┌────┴────┐
                                   │ R2=4.7kΩ│ (a GND)
                                   └────┬────┘
                                        │
                                       GND
```

### Especificaciones de comunicación

| Parámetro | Valor |
|-----------|-------|
| **Interfaz** | UART (modo activo, transmisión continua) |
| **Baudrate** | 921600 bps |
| **Formato** | 8N1 (8 data bits, sin paridad, 1 stop bit) |
| **Nivel lógico UART** | 3.5–3.6V medido (nominal 3.3V TTL). Divisor obligatorio: R1=1kΩ + R2=4.7kΩ → ~2.9V en GPIO 18 |
| **Frecuencia de datos** | ~10 Hz (una trama de 400 bytes cada ~100 ms) |
| **Protocolo** | NLink_TOFSense_M_Frame0 (header 0x57, 64 píxeles × 6 bytes) |
| **Rango útil** | 20 mm – 4000 mm |

### Referencia en firmware

- `esp32/src/sensors/obstacle_sensor.h` — Configuración: `rxPin = 18`, `txPin = -1`, `baudRate = 921600`
- `esp32/src/sensors/obstacle_sensor.cpp` — Parser del protocolo NLink, detección de sensor atascado
- `esp32/src/main.cpp` — Inicialización: `obstacle_sensor::init()` en `setup()`
- `docs/TOFSENSE_M_WIRING_GUIDE.md` — Guía completa de cableado y troubleshooting

> ⚠️ **IMPORTANTE:** El sensor requiere 5V en VCC para funcionar. A 3.3V no arranca o produce lecturas inválidas. Las señales UART miden **3.5–3.6 V** (por encima del 3.3V nominal del datasheet). Se requiere un **divisor de tensión obligatorio** (R1=1 kΩ serie + R2=4.7 kΩ a GND) entre sensor TX y GPIO18. **NO conectar directamente** — el ESP32-S3 tiene máx. absoluto de 3.6V en GPIO.

> ⚠️ **IMPORTANTE:** Se recomienda un condensador de desacoplo de **100 nF** entre VCC y GND del sensor, lo más cerca posible del conector GH1.25, para filtrar ruido de alimentación.

---

## 13) TABLA COMPLETA — TODOS LOS CABLES DEL STM32

> ⚠️ Tabla actualizada tras PR #120. Los pines DIR (PC0–PC4) están liberados. PC6/PC7/PC8/PC9 son ahora salidas PWM TIM8. PA6/PA7 son nuevas salidas PWM TIM3 (STEER).

| # | Pin STM32 | Puerto | Tipo | Periférico | Conectar a | Notas |
|---|-----------|--------|------|------------|-----------|-------|
| 1 | **PA0** | GPIOA | Input | EXTI0 | Sensor velocidad rueda FL | Pull-up, flanco subida |
| 2 | **PA1** | GPIOA | Input | EXTI1 | Sensor velocidad rueda FR | Pull-up, flanco subida |
| 3 | **PA2** | GPIOA | Input | EXTI2 | Sensor velocidad rueda RL | Pull-up, flanco subida |
| 4 | **PA3** | GPIOA | Analog | ADC1_IN4 | Pedal (divisor) | Canal primario, señal dividida 0–2V |
| 5 | **PA6** | GPIOA | AF2 | TIM3_CH1 | BTS7960 STEER → **RPWM** | PWM 20 kHz — izquierda |
| 6 | **PA7** | GPIOA | AF2 | TIM3_CH2 | BTS7960 STEER → **LPWM** | PWM 20 kHz — derecha |
| 7 | **PA8** | GPIOA | AF6 | TIM1_CH1 | BTS7960 FL → **RPWM** | PWM 20 kHz — adelante |
| 8 | **PA9** | GPIOA | AF6 | TIM1_CH2 | BTS7960 FL → **LPWM** | PWM 20 kHz — atrás |
| 9 | **PA10** | GPIOA | AF6 | TIM1_CH3 | BTS7960 FR → **RPWM** | PWM 20 kHz — adelante |
| 10 | **PA11** | GPIOA | AF6 | TIM1_CH4 | BTS7960 FR → **LPWM** | PWM 20 kHz — atrás |
| 11 | **PA15** | GPIOA | AF1 | TIM2_CH1 | Encoder E6B2 canal A | ⚠️ Adaptador 5V→3.3V |
| 12 | **PB0** | GPIOB | Output | Bit-bang | Bus OneWire (5× DS18B20) | Pull-up 4.7kΩ a 3.3V |
| 13 | **PB3** | GPIOB | AF1 | TIM2_CH2 | Encoder E6B2 canal B | ⚠️ Adaptador 5V→3.3V |
| 14 | **PB4** | GPIOB | Input | EXTI4 | Encoder E6B2 índice Z | 1 pulso/vuelta |
| 15 | **PB5** | GPIOB | Input | EXTI5 | Sensor inductivo centrado | Pull-up, flanco subida |
| 16 | **PB6** | GPIOB | AF4 | I2C1_SCL | TCA9548A (INA226) | Pull-up 4.7kΩ, 400 kHz |
| 17 | **PB7** | GPIOB | AF4 | I2C1_SDA | TCA9548A (INA226) | Pull-up 4.7kΩ, 400 kHz |
| 18 | **PB8** | GPIOB | AF9 | FDCAN1_RX | TJA1051 #1 → RXD | ⚠️ Vía transceiver, NO directo |
| 19 | **PB9** | GPIOB | AF9 | FDCAN1_TX | TJA1051 #1 → TXD | ⚠️ Vía transceiver, NO directo |
| 20 | **PB15** | GPIOB | Input | EXTI15 | Sensor velocidad rueda RR | Pull-up, flanco subida |
| 21 | **PC0** | GPIOC | — | **LIBRE** | ~~BTS7960 FL DIR~~ — desconectado | Pin liberado (PR #120) |
| 22 | **PC1** | GPIOC | — | **LIBRE** | ~~BTS7960 FR DIR~~ — desconectado | Pin liberado (PR #120) |
| 23 | **PC2** | GPIOC | — | **LIBRE** | ~~BTS7960 RL DIR~~ — desconectado | Pin liberado (PR #120) |
| 24 | **PC3** | GPIOC | — | **LIBRE** | ~~BTS7960 RR DIR~~ — desconectado | Pin liberado (PR #120) |
| 25 | **PC4** | GPIOC | — | **LIBRE** | ~~BTS7960 STEER DIR~~ — desconectado | Pin liberado (PR #120) |
| 26 | **PC5** | GPIOC | Output | GPIO | BTS7960 FL → R_EN + L_EN | HIGH = motor habilitado |
| 27 | **PC6** | GPIOC | AF4 | TIM8_CH1 | BTS7960 RL → **RPWM** | PWM 20 kHz — adelante |
| 28 | **PC7** | GPIOC | AF4 | TIM8_CH2 | BTS7960 RL → **LPWM** | PWM 20 kHz — atrás |
| 29 | **PC8** | GPIOC | AF4 | TIM8_CH3 | BTS7960 RR → **RPWM** | PWM 20 kHz — adelante |
| 30 | **PC9** | GPIOC | AF4 | TIM8_CH4 | BTS7960 RR → **LPWM** | PWM 20 kHz — atrás |
| 31 | **PC10** | GPIOC | Output | GPIO | Módulo relé MAIN | HIGH = ON (vía optoacoplador) |
| 32 | **PC11** | GPIOC | Output | GPIO | Módulo relé TRACCIÓN | HIGH = ON (vía optoacoplador) |
| 33 | **PC12** | GPIOC | Output | GPIO | Módulo relé DIRECCIÓN | HIGH = ON (vía optoacoplador) |
| 34 | **PC13** | GPIOC | Output | GPIO | BTS7960 RR → R_EN + L_EN | HIGH = motor habilitado |

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
| 1 | TOFSense-M S (Nooploop) | Sensor obstáculos LiDAR 8×8 | Conectado a ESP32-S3 GPIO18 (UART1), VCC=5V, conector GH1.25 |
| 3 | Módulo relé + optoacoplador | Relés potencia | HY-M158 o similar, 3.3V trigger |
| 2 | Resistencia 120 Ω | Terminación CAN | ¼W mínimo |
| 3 | Resistencia 4.7 kΩ | Pull-ups (I2C + OneWire) | PB6, PB7, PB0 |
| 1 | Adaptador nivel 5V→3.3V | Encoder A/B | 2 canales mín (PA15, PB3) |
| 5 | Resistencia shunt | INA226 | 4× 1 mΩ + 1× 0.5 mΩ |
| 1 | Fuente 24V | Tracción | ≥20A capacidad |
| 1 | Fuente 12V | Dirección | ≥5A capacidad |
| 1 | Fuente 5V | Lógica / sensores | ≥2A capacidad |

### Componentes de protección (OBLIGATORIOS — PR #120)

| Qty | Componente | Valor | Uso | Notas |
|-----|-----------|-------|-----|-------|
| 3 | Diodo flyback | **1N4007** (1A, 1000V) | Uno por bobina de relé (MAIN, TRAC, DIR) | En paralelo con la bobina, polaridad inversa; incluido en muchos módulos HY-M158 — verificar |
| 3 | Condensador snubber | **100 nF / 250V** (polipropileno) | En paralelo con contactos COM–NO de cada relé | Reduce arcos en conmutación; 250V para margen ante picos inductivos en carga inductiva 24V |
| 3 | Resistencia snubber | **100 Ω / 0.5W** | En serie con condensador snubber (RC snubber) | Limita corriente de descarga del condensador |
| 5 | Condensador bulk | **470 µF / 35V** (electrolítico) | Uno por BTS7960, entre B+ y GND del driver | Reserva de energía, filtra transitorios de carga del motor |
| 10 | Condensador bypass | **100 nF / 50V** (X7R/X5R cerámico) | Dos por BTS7960 (uno en B+/GND, uno en VCC lógico/GND) | Filtra ruido PWM 20 kHz en alimentación |
| 5 | Diodo rueda libre motor | **SB560** Schottky (5A, 60V) o similar | Opcional: uno por motor, entre terminales del motor (cable A–cable B, no en el BTS7960) | El BTS7960 ya incluye diodos internos de protección. Un diodo Schottky externo de 60V ≥3A en los terminales del motor añade protección ante contra-EMF de larga duración. Usar 60V mínimo (margen sobre 24V). |

> ⚠️ Los diodos flyback en la bobina de relé son críticos. Sin ellos, al desactivar el relé se genera un pico de cientos de voltios que puede destruir el transistor del módulo optoacoplador.
>
> ⚠️ Los condensadores bulk en cada BTS7960 son especialmente importantes con la nueva arquitectura RPWM/LPWM directo, ya que el driver maneja transiciones de dirección con mayor frecuencia.

### Pines liberados (PC0–PC4) — ya no se cablean

| Pin | Uso anterior | Estado actual |
|-----|-------------|---------------|
| PC0 | DIR_FL (GPIO OUT) | **LIBRE** — dejar desconectado o como GPIO output LOW |
| PC1 | DIR_FR (GPIO OUT) | **LIBRE** — dejar desconectado o como GPIO output LOW |
| PC2 | DIR_RL (GPIO OUT) | **LIBRE** — dejar desconectado o como GPIO output LOW |
| PC3 | DIR_RR (GPIO OUT) | **LIBRE** — dejar desconectado o como GPIO output LOW |
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

- `docs/PINOUT_DEFINITIVO.md` — Tabla de pines detallada
- `docs/HARDWARE_WIRING_MANUAL.md` — Manual eléctrico completo
- `docs/HARDWARE_SPECIFICATION.md` — Especificaciones de componentes
- `docs/ESP32_STM32_CAN_CONNECTION.md` — Conexión CAN detallada
- `docs/TOFSENSE_M_WIRING_GUIDE.md` — Guía de conexión TOFSense-M S (sensor obstáculos)
- `docs/HARDWARE_VALIDATION_PROCEDURE.md` — Procedimiento de validación Phase 1
- `Core/Inc/main.h` — Definiciones de pines en firmware
- `esp32/platformio.ini` — Pines ESP32 (CAN + Display)
- `esp32/src/sensors/obstacle_sensor.h` — Configuración UART del TOFSense-M (GPIO18, 921600 bps)
