# MANUAL ELÉCTRICO DE CABLEADO — STM32G474RE

**Documento de referencia para taller — Solo conexiones reales**

> **IMPORTANTE**: Todo lo documentado aquí está trazado directamente al código fuente del firmware
> (`main.h`, `main.c`, `motor_control.c`, `sensor_manager.c`, `safety_system.c`,
> `stm32g4xx_hal_msp.c`, `can_handler.c`, `vehicle_physics.h`, `steering_centering.c`).
> No se ha inventado ningún componente, pin ni sensor. Si algo no existe en el firmware,
> se indica explícitamente.

---

## Índice

1. [Visión general del sistema](#1-visión-general-del-sistema)
2. [Alimentación y masas](#2-alimentación-y-masas)
3. [CAN Bus](#3-can-bus-crítico)
4. [Motores de tracción (4×)](#4-motores-de-tracción-4)
5. [Motor de dirección](#5-motor-de-dirección)
6. [Sensores](#6-sensores)
7. [Relés](#7-relés)
8. [Tabla final de pinout](#8-tabla-final-de-pinout)
9. [Cosas que NO existen en el STM32](#9-cosas-que-no-existen-en-el-stm32)

---

## 1. Visión general del sistema

### Rol del STM32G474RE

El STM32G474RE (Nucleo-64) es el **controlador de tiempo real** del vehículo. Ejecuta a 170 MHz
(HSI 16 MHz → PLL ×85 ÷2) y se encarga de:

- Control PWM de los 4 motores de tracción (20 kHz, TIM1)
- Control PWM del motor de dirección (20 kHz, TIM8)
- Lectura del encoder de dirección (TIM2, modo cuadratura)
- Lectura de 4 sensores inductivos de velocidad de rueda (EXTI)
- Lectura de 6 sensores de corriente INA226 (I2C1 vía TCA9548A)
- Lectura de 5 sensores de temperatura DS18B20 (OneWire, PB0)
- Lectura del pedal de acelerador (ADC1, PA3)
- Detección del sensor inductivo de centrado de dirección (EXTI5)
- Control de 3 relés de potencia (MAIN, TRACCIÓN, DIRECCIÓN)
- Comunicación CAN con el ESP32 (FDCAN1, 500 kbps)
- Sistemas de seguridad: ABS, TCS, límites de corriente/temperatura
- Watchdog independiente (IWDG, ~4 s)

### Qué NO controla el STM32 (depende del ESP32)

- Palanca de cambios (F/N/R) — el ESP32 lee el hardware y envía comandos CAN
- Llave de contacto
- Pantalla / HMI
- Audio
- Sensores de obstáculos / ultrasonidos
- Iluminación (luces, intermitentes)
- Interfaz de usuario

### Tensiones del sistema

| Tensión | Uso |
|---------|-----|
| **24 V** | Motores de tracción (4×) — alimentación de los BTS7960 de tracción |
| **12 V** | Motor de dirección — alimentación del BTS7960 de dirección |
| **5 V** | Lógica de los módulos BTS7960 (VCC lógico) |
| **3.3 V** | STM32G474RE, sensores I2C, señales digitales |

> **Nota**: Las tensiones de 24 V y 12 V no aparecen como constantes explícitas en el firmware
> (el MCU solo genera señales PWM/GPIO). La información de tensiones proviene de la
> documentación de hardware del proyecto (`HARDWARE_SPECIFICATION.md`), que describe los
> BTS7960 alimentados a 24 V (tracción) y 12 V (dirección).

---

## 2. Alimentación y masas

### Alimentación del STM32

El STM32G474RE en formato Nucleo-64 se alimenta por:

- **USB** (solo para depuración/programación, no para operación en vehículo)
- **Pin VIN** (7–12 V) a través del regulador de la placa Nucleo
- **Pin 3.3V** directo (si se usa regulador externo)

> **NO DEDUCIBLE SOLO DESDE EL CÓDIGO**: El firmware no especifica cuál de estos métodos se
> usa en el vehículo final. Consultar el diseño eléctrico del chasis.

### Reglas de masa común

1. **TODAS** las masas (GND) del sistema deben estar conectadas en un punto común
2. La masa del STM32 debe unirse a la masa de los BTS7960, sensores y bus CAN
3. Usar cables de masa de sección suficiente para las corrientes de los motores
4. Separar físicamente los retornos de potencia (motores) de los retornos de señal (sensores)
   siempre que sea posible, uniéndolos en un único punto (topología estrella)

### Qué NO conectar directamente al MCU

- **NUNCA** conectar 24 V o 12 V a ningún pin del STM32 — destruirá el chip
- **NUNCA** conectar la salida de un motor directamente al STM32
- **NUNCA** conectar el bus CAN (CANH/CANL) directamente a PB8/PB9 — se necesita transceiver
- Los sensores inductivos (LJ12A3) si operan a más de 3.3 V necesitan adaptación de nivel
- Los sensores DS18B20 operan a 3.3 V y pueden conectarse directamente (con pull-up)

### Protecciones

> **NO DEDUCIBLE SOLO DESDE EL CÓDIGO**: El firmware no define fusibles ni protecciones
> eléctricas explícitas. Los relés de potencia (`PIN_RELAY_MAIN`, `PIN_RELAY_TRAC`,
> `PIN_RELAY_DIR`) actúan como seccionadores controlados por software, pero no sustituyen
> a fusibles físicos. Se recomienda instalar fusibles en las líneas de 24 V y 12 V
> según el diseño eléctrico del chasis.

---

## 3. CAN Bus (CRÍTICO)

### Pines STM32 usados

| Señal | Pin STM32 | Función alternativa | Definición en código |
|-------|-----------|---------------------|----------------------|
| CAN TX | **PB9** | FDCAN1_TX (AF9) | `PIN_CAN_TX` (`main.h:70`) |
| CAN RX | **PB8** | FDCAN1_RX (AF9) | `PIN_CAN_RX` (`main.h:71`) |

Configuración en `stm32g4xx_hal_msp.c`: GPIO modo AF push-pull, velocidad alta, función alternativa AF9.

### Transceiver CAN — OBLIGATORIO

**PB8 y PB9 son salidas lógicas CMOS a 3.3 V. NO son señales CAN.**

Es **obligatorio** un transceiver CAN externo (por ejemplo: MCP2551, SN65HVD230, TJA1050)
entre el STM32 y el bus CAN físico.

### Esquema de conexión

```
STM32G474RE            Transceiver CAN              Bus CAN
┌──────────┐          ┌──────────────┐          ┌───────────┐
│      PB9 ├──────────► TXD      CANH├──────────► CANH      │
│  (CAN TX)│          │              │          │           │
│      PB8 ◄──────────┤ RXD      CANL├──────────► CANL      │
│  (CAN RX)│          │              │          │           │
│     3.3V ├──────────► VCC      GND ├──────────► GND       │
│      GND ├──────────► GND          │          │           │
└──────────┘          └──────────────┘          └───────────┘
```

### Configuración FDCAN

Definida en `main.c` (función `MX_FDCAN1_Init`):

| Parámetro | Valor |
|-----------|-------|
| Modo | Normal (Classic CAN, no FD) |
| Velocidad | **500 kbps** |
| Prescaler | 17 |
| SyncJumpWidth | 1 |
| TimeSeg1 | 14 |
| TimeSeg2 | 5 |
| IDs | 11 bits (CAN 2.0A estándar) |

Cálculo: 170 MHz ÷ 17 ÷ (1 + 14 + 5) = 500 kbps, donde (1 + 14 + 5) = SyncSeg + TimeSeg1 + TimeSeg2 = 20 time quanta por bit.

### Resistencias de terminación (120 Ω)

- El bus CAN **requiere exactamente 2 resistencias de 120 Ω**, una en cada extremo del bus
- Si el STM32 está en un extremo del bus → poner 120 Ω entre CANH y CANL junto al transceiver
- Si el STM32 está en medio del bus → **NO** poner resistencia
- El otro extremo (ESP32) debe tener su propia terminación de 120 Ω

**Si falta terminación**: el bus CAN presentará errores de comunicación, reflexiones de señal,
y el firmware detectará `CAN_TIMEOUT` (250 ms sin heartbeat del ESP32), forzando al sistema
al estado **SAFE** (todos los actuadores desactivados, relés apagados).

### Filtros RX configurados

Definidos en `can_handler.c`:

| Filtro | ID | Descripción |
|--------|----|-------------|
| 0 | 0x011 | Heartbeat del ESP32 |
| 1 | 0x100–0x102 | Comandos (acelerador, dirección, modo) |
| 2 | 0x110 | Comandos de servicio |

Cualquier otro ID CAN es **rechazado** (política de seguridad).

---

## 4. Motores de tracción (4×)

### Tipo de driver

Cada motor de tracción usa un driver **BTS7960** (H-bridge, Infineon). La documentación del
proyecto (`HARDWARE_SPECIFICATION.md`) especifica este modelo. El firmware los controla
genéricamente como H-bridge: un pin PWM + un pin DIR + un pin EN por motor.

### Tensión de alimentación

**24 V** para los 4 motores de tracción.

### Señales por rueda

Cada BTS7960 de tracción recibe 3 señales del STM32:

| Señal | Función | Nivel |
|-------|---------|-------|
| **PWM** | Velocidad del motor (20 kHz, TIM1) | 0–3.3 V → lógica BTS7960 |
| **DIR** | Dirección de giro (GPIO, GPIOC) | HIGH/LOW = adelante/atrás |
| **EN** | Habilitación del driver (GPIO, GPIOC) | HIGH = habilitado |

### Tabla por rueda

| Rueda | PWM (Pin) | Timer/Canal | DIR (Pin) | EN (Pin) | Sensor velocidad | INA226 (índice) | DS18B20 (índice) |
|-------|-----------|-------------|-----------|----------|------------------|-----------------|-------------------|
| **FL** (Frontal izq.) | PA8 | TIM1_CH1 | PC0 | PC5 | PA0 (EXTI0) | 0 | 0 |
| **FR** (Frontal der.) | PA9 | TIM1_CH2 | PC1 | PC6 | PA1 (EXTI1) | 1 | 1 |
| **RL** (Trasera izq.) | PA10 | TIM1_CH3 | PC2 | PC7 | PA2 (EXTI2) | 2 | 2 |
| **RR** (Trasera der.) | PA11 | TIM1_CH4 | PC3 | PC13 | PB15 (EXTI15) | 3 | 3 |

> **Nota sobre índices INA226 y DS18B20**: El firmware usa indexación 0–5 para INA226
> (6 sensores) y 0–4 para DS18B20 (5 sensores). El código de seguridad
> (`safety_system.c`) itera sobre los 4 primeros índices para las ruedas.
> La asignación exacta índice→rueda física depende del orden de conexión en el
> multiplexor TCA9548A (INA226) y del orden de descubrimiento en el bus OneWire (DS18B20).
> **NO DEDUCIBLE SOLO DESDE EL CÓDIGO**: verificar el cableado físico del multiplexor
> y el orden de enumeración ROM de los DS18B20.

### Configuración PWM (TIM1)

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Frecuencia | **20 kHz** | `motor_control.c`: `PWM_FREQUENCY = 20000` |
| Periodo | 8499 | `motor_control.c`: `PWM_PERIOD = 8499` |
| Prescaler | 0 | `main.c`: `MX_TIM1_Init` |
| Reloj timer (APB2) | 170 MHz | 170 MHz ÷ (0+1) ÷ (8499+1) = 20 kHz |
| Canales activos | CH1, CH2, CH3, CH4 | `stm32g4xx_hal_msp.c` |

### Conexión STM32 → BTS7960 (por motor de tracción)

```
STM32 (PA8/9/10/11) ──PWM──► BTS7960 RPWM o LPWM (según DIR)
STM32 (PC0/1/2/3)   ──DIR──► BTS7960 (lógica de selección RPWM/LPWM)
STM32 (PC5/6/7/13)  ──EN───► BTS7960 R_EN + L_EN (ambos habilitados juntos)
                               BTS7960 VCC ← 5V lógica
                               BTS7960 B+ ← 24V potencia
                               BTS7960 B- ← GND potencia
```

> **Nota**: El firmware controla un solo pin PWM por motor y un pin DIR para la dirección.
> La lógica de adaptación entre la señal PWM+DIR del STM32 y las entradas RPWM/LPWM
> del BTS7960 puede requerir circuitería externa o configuración específica del módulo
> BTS7960. **NO DEDUCIBLE SOLO DESDE EL CÓDIGO**: consultar el esquemático del vehículo
> para la conexión exacta PWM+DIR → RPWM/LPWM.

---

## 5. Motor de dirección

### Driver

**BTS7960** (1 unidad dedicada a dirección).

### Tensión

**12 V** para el motor de dirección.

### Pines STM32

| Señal | Pin | Función | Referencia |
|-------|-----|---------|------------|
| PWM | **PC8** | TIM8_CH3 (AF4) | `PIN_PWM_STEER` (`main.h:22`) |
| DIR | **PC4** | GPIO output (GPIOC) | `PIN_DIR_STEER` (`main.h:29`) |
| EN | **PC9** | GPIO output (GPIOC) | `PIN_EN_STEER` (`main.h:36`) |

### Configuración PWM (TIM8)

| Parámetro | Valor |
|-----------|-------|
| Frecuencia | **20 kHz** |
| Periodo | 8499 |
| Prescaler | 0 |
| Canal | CH3 |

### Encoder de dirección (E6B2-CWZ6C)

Definido en `main.h` (líneas 11–15) y leído por TIM2 en modo cuadratura.

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Modelo | E6B2-CWZ6C | `main.h` (comentario) |
| PPR | **1200** | `ENCODER_PPR` (`main.h:14`) |
| CPR (×4 cuadratura) | **4800** | `ENCODER_CPR` (`main.h:15`) |
| Canal A | **PA15** (TIM2_CH1, AF1) | `PIN_ENC_A` (`main.h:50`) |
| Canal B | **PB3** (TIM2_CH2, AF1) | `PIN_ENC_B` (`main.h:51`) |
| Índice (Z) | **PB4** (EXTI4) | `PIN_ENC_Z` (`main.h:52`) |
| Periodo TIM2 | 65535 | `main.c`: `MX_TIM2_Init` |
| Modo conteo | TI12 (ambos canales, ×4) | `main.c` |

### Sensor inductivo de centrado

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Tipo | **LJ12A3** (inductivo) | `main.h` (comentario línea 55) |
| Pin | **PB5** | `PIN_STEER_CENTER` (`main.h:57`) |
| Interrupción | EXTI5 (flanco ascendente, pull-up) | `main.c`: GPIO init |
| Función | Detecta un tornillo en la cremallera en la posición de centro |
| Uso | Calibración automática al arranque (`steering_centering.c`) |

### Calibración de centrado

Al arrancar (estados BOOT/STANDBY), el firmware ejecuta un barrido automático
(`steering_centering.c`):

1. Mueve la dirección a la izquierda (PWM = 850, ~10% del periodo) buscando el pulso del sensor inductivo
2. Si detecta estancamiento (encoder sin cambio durante 300 ms), invierte dirección
3. Si detecta el sensor de centro → para motor, pone encoder a 0, calibración completada
4. Si excede ±6000 cuentas de encoder o 10 segundos → fallo de calibración (`SAFETY_ERROR_CENTERING`)

### Finales de carrera mecánicos

**NO EXISTEN.** El firmware no implementa finales de carrera mecánicos para la dirección.
La protección contra sobrerrecorrido se realiza por software:

- Rango máximo del encoder: ±4933 cuentas (equivale a ±370° de eje de encoder)
- Ángulo máximo de rueda de carretera: ±54° (`MAX_STEER_DEG`, `vehicle_physics.h:20`)
- Detección de estancamiento del encoder (timeout 200 ms en `motor_control.c`)

---

## 6. Sensores

### 6.1 Sensores inductivos de velocidad de rueda (4×)

Tipo: sensores inductivos de proximidad (tipo LJ12A3, 6 tornillos por revolución).

| Rueda | Pin STM32 | Puerto | Interrupción | Flanco | Pull-up | Referencia |
|-------|-----------|--------|--------------|--------|---------|------------|
| FL | **PA0** | GPIOA | EXTI0 | Ascendente | Sí (interno) | `PIN_WHEEL_FL` (`main.h:44`) |
| FR | **PA1** | GPIOA | EXTI1 | Ascendente | Sí (interno) | `PIN_WHEEL_FR` (`main.h:45`) |
| RL | **PA2** | GPIOA | EXTI2 | Ascendente | Sí (interno) | `PIN_WHEEL_RL` (`main.h:46`) |
| RR | **PB15** | GPIOB | EXTI15 | Ascendente | Sí (interno) | `PIN_WHEEL_RR` (`main.h:47`) |

Constantes de cálculo de velocidad (`main.h`, `vehicle_physics.h`):

- Pulsos por revolución: **6** (`WHEEL_PULSES_REV`)
- Circunferencia de rueda: **1.1 m** (`WHEEL_CIRCUMF_M`)

> **Nota sobre tensión de los sensores inductivos**: Si los sensores LJ12A3 operan a una tensión
> superior a 3.3 V (muchos modelos son de 6–36 V DC con salida NPN/PNP), es necesario un
> **divisor de tensión o adaptador de nivel** entre el sensor y el pin EXTI del STM32.
> **NO DEDUCIBLE SOLO DESDE EL CÓDIGO**: verificar el modelo exacto y su tensión de salida.

### 6.2 Encoder de dirección

Ver [sección 5 — Motor de dirección](#encoder-de-dirección-e6b2-cwz6c) para detalle completo.

- Canal A: PA15 (TIM2_CH1)
- Canal B: PB3 (TIM2_CH2)
- Índice Z: PB4 (EXTI4)
- Modo: cuadratura ×4, 4800 CPR

### 6.3 Sensores de temperatura DS18B20 (5×)

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Cantidad | **5** | `NUM_DS18B20` (`main.h:77`) |
| Bus | **OneWire** (bit-bang) | `sensor_manager.c` |
| Pin | **PB0** | `PIN_ONEWIRE` (`main.h:64`) |
| Protocolo | Búsqueda automática de ROM al inicio | `OW_SearchAll()` en `sensor_manager.c` |
| Familia | 0x28 (validación CRC-8) | `sensor_manager.c` |
| Pull-up | **Obligatorio**: resistencia de 4.7 kΩ entre PB0 y 3.3 V | Requisito del protocolo OneWire |

**Cableado de los DS18B20:**

```
         3.3V
          │
         [4.7 kΩ]  ← Pull-up OBLIGATORIO
          │
PB0 ──────┼──── DS18B20 #0 (DQ)
          ├──── DS18B20 #1 (DQ)
          ├──── DS18B20 #2 (DQ)
          ├──── DS18B20 #3 (DQ)
          └──── DS18B20 #4 (DQ)
                  │
                 GND
```

Todos los DS18B20 comparten el mismo bus (PB0). El firmware descubre automáticamente
las direcciones ROM de cada sensor. El orden de índices (0–4) depende del orden de
enumeración del algoritmo de búsqueda OneWire.

> **NO DEDUCIBLE SOLO DESDE EL CÓDIGO**: la asignación física sensor→ubicación
> (FL, FR, RL, RR, ambiente) depende del orden de enumeración ROM, que varía
> según las direcciones de fábrica de cada DS18B20.

### 6.4 Sensores de corriente INA226 (6×)

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Cantidad | **6** | `NUM_INA226` (`main.h:76`) |
| Bus | **I2C1** (400 kHz, Fast Mode) | `main.c`: `MX_I2C1_Init` |
| Pin SCL | **PB6** | `PIN_I2C_SCL` (`main.h:60`) |
| Pin SDA | **PB7** | `PIN_I2C_SDA` (`main.h:61`) |
| Dirección INA226 | **0x40** | `I2C_ADDR_INA226` (`main.h:75`) |
| Multiplexor | **TCA9548A** en dirección **0x70** | `I2C_ADDR_TCA9548A` (`main.h:74`) |
| Resistencia shunt | **1 mΩ** | `INA226_SHUNT_MOHM` (`main.h:82`) |

**Esquema de conexión I2C:**

```
STM32                TCA9548A (0x70)           INA226 (0x40 cada uno)
┌──────┐            ┌──────────────┐
│  PB6 ├──SCL───────┤ SCL      CH0 ├───SCL/SDA──► INA226 #0
│  PB7 ├──SDA───────┤ SDA      CH1 ├───SCL/SDA──► INA226 #1
│      │            │          CH2 ├───SCL/SDA──► INA226 #2
│      │            │          CH3 ├───SCL/SDA──► INA226 #3
│      │            │          CH4 ├───SCL/SDA──► INA226 #4
│      │            │          CH5 ├───SCL/SDA──► INA226 #5
│      │            │     CH6, CH7 │  (no usados)
└──────┘            └──────────────┘
```

Los 6 INA226 tienen todos la **misma dirección I2C (0x40)** porque están aislados
por los canales del multiplexor TCA9548A. El firmware selecciona el canal del TCA9548A
antes de cada lectura.

> **Pull-ups I2C**: La configuración GPIO en `stm32g4xx_hal_msp.c` usa modo
> open-drain con pull-up. Se necesitan resistencias pull-up externas (típicamente
> 4.7 kΩ a 3.3 V) en las líneas SCL y SDA si no están incluidas en los módulos.

**Índices de los INA226:**

| Índice | Canal TCA9548A | Uso probable |
|--------|----------------|--------------|
| 0 | CH0 | Motor FL |
| 1 | CH1 | Motor FR |
| 2 | CH2 | Motor RL |
| 3 | CH3 | Motor RR |
| 4 | CH4 | Motor dirección |
| 5 | CH5 | Bus principal (24 V) u otro |

> **NO DEDUCIBLE SOLO DESDE EL CÓDIGO**: la asignación exacta de cada índice INA226 a un
> motor específico depende del cableado físico del multiplexor. El firmware solo itera
> por índice numérico. Los 4 primeros (0–3) se usan en los bucles de seguridad de ruedas
> y los 6 se leen en el ciclo completo de sensores.

### 6.5 Sensor de pedal (acelerador)

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Tipo | **Analógico** (potenciómetro o sensor Hall) | `sensor_manager.c` |
| Pin | **PA3** | `PIN_PEDAL` (`main.h:67`) |
| Canal ADC | **ADC1_IN4** | `main.c`: `MX_ADC1_Init` |
| Resolución | 12 bits (0–4095) | `main.c` |
| Tiempo muestreo | 47.5 ciclos | `main.c` |
| Disparo | Software (polling, timeout 10 ms) | `sensor_manager.c` |
| Conversión | `porcentaje = (raw × 100) / 4095` | `sensor_manager.c` |
| Filtrado | EMA (α = 0.15) a 20 Hz | `motor_control.c` |
| Rampa | Subida: 50%/s máx. — Bajada: 100%/s máx. | `motor_control.c` |

El pedal debe producir una señal de **0 a 3.3 V** en PA3.

---

## 7. Relés

### Relés controlados por el STM32

| Relé | Pin STM32 | Puerto | Función | Tensión conmutada |
|------|-----------|--------|---------|-------------------|
| **MAIN** (Principal) | **PC10** | GPIOC | Alimentación general del sistema de potencia | NO DEDUCIBLE SOLO DESDE EL CÓDIGO |
| **TRAC** (Tracción) | **PC11** | GPIOC | Alimentación de los drivers de tracción (24 V) | 24 V |
| **DIR** (Dirección) | **PC12** | GPIOC | Alimentación del driver de dirección (12 V) | 12 V |

Definidos en `main.h` (líneas 39–41):
- `PIN_RELAY_MAIN` = GPIO_PIN_10 (PC10)
- `PIN_RELAY_TRAC` = GPIO_PIN_11 (PC11)
- `PIN_RELAY_DIR`  = GPIO_PIN_12 (PC12)

> **Nota**: Los pines GPIO del STM32 (3.3 V, máximo ~20 mA) **no pueden accionar un relé
> directamente**. Se requiere un transistor/MOSFET de conmutación o un módulo de relé
> con opto-aislamiento. **NO DEDUCIBLE SOLO DESDE EL CÓDIGO**: consultar el esquemático
> para el circuito de driver de relé.

### Orden de activación (Power-Up)

Definido en `safety_system.c` (función `Relay_PowerUp`):

```
1. PC10 → HIGH  (RELAY_MAIN ON)
2. Esperar 50 ms                    ← settling de corriente inrush
3. PC11 → HIGH  (RELAY_TRAC ON)
4. Esperar 20 ms                    ← supresión de arco del contactor
5. PC12 → HIGH  (RELAY_DIR ON)
```

### Orden de desactivación (Power-Down)

Definido en `safety_system.c` (función `Relay_PowerDown`):

```
1. PC12 → LOW   (RELAY_DIR OFF)
2. PC11 → LOW   (RELAY_TRAC OFF)
3. PC10 → LOW   (RELAY_MAIN OFF)
```

Orden inverso, sin retardos.

### Comportamiento en estados de error

| Estado del sistema | Relés | Actuadores | Referencia |
|--------------------|-------|------------|------------|
| **ACTIVE** | Todos ON | Operación normal, 100% potencia | `safety_system.c` |
| **DEGRADED** | Todos ON | Potencia limitada al 40%, velocidad al 50% | `DEGRADED_POWER_LIMIT_PCT` / `DEGRADED_SPEED_LIMIT_PCT` |
| **SAFE** | **Todos OFF** | Actuadores inhibidos, FailSafe ejecutado | `Safety_FailSafe()` |
| **ERROR** | **Todos OFF** | Apagado total, requiere reinicio | `Relay_PowerDown()` |

En **SAFE**: el firmware intenta centrar la dirección (si el encoder está sano) antes de
apagar los relés, para que el vehículo ruede en línea recta. Si el encoder está fallado,
simplemente corta PWM y desactiva los drivers.

En **ERROR** (emergencia): apagado inmediato de relés, sin intento de centrado.

---

## 8. Tabla final de pinout

Tabla completa de **todos los pines del STM32G474RE realmente usados** en el firmware:

| # | Pin STM32 | Puerto | Función | Componente | Periférico | Tensión señal | Referencia código |
|---|-----------|--------|---------|------------|------------|---------------|-------------------|
| 1 | **PA0** | GPIOA | Sensor velocidad FL | Inductivo (LJ12A3) | EXTI0 | 3.3 V (con adaptación si necesario) | `PIN_WHEEL_FL` |
| 2 | **PA1** | GPIOA | Sensor velocidad FR | Inductivo (LJ12A3) | EXTI1 | 3.3 V | `PIN_WHEEL_FR` |
| 3 | **PA2** | GPIOA | Sensor velocidad RL | Inductivo (LJ12A3) | EXTI2 | 3.3 V | `PIN_WHEEL_RL` |
| 4 | **PA3** | GPIOA | Pedal acelerador | Potenciómetro/Hall | ADC1_IN4 | 0–3.3 V analógico | `PIN_PEDAL` |
| 5 | **PA8** | GPIOA | PWM motor FL | BTS7960 FL | TIM1_CH1 (AF6) | 3.3 V PWM | `PIN_PWM_FL` |
| 6 | **PA9** | GPIOA | PWM motor FR | BTS7960 FR | TIM1_CH2 (AF6) | 3.3 V PWM | `PIN_PWM_FR` |
| 7 | **PA10** | GPIOA | PWM motor RL | BTS7960 RL | TIM1_CH3 (AF6) | 3.3 V PWM | `PIN_PWM_RL` |
| 8 | **PA11** | GPIOA | PWM motor RR | BTS7960 RR | TIM1_CH4 (AF6) | 3.3 V PWM | `PIN_PWM_RR` |
| 9 | **PA15** | GPIOA | Encoder dirección A | E6B2-CWZ6C CH-A | TIM2_CH1 (AF1) | 3.3 V o 5 V (con adaptación) | `PIN_ENC_A` |
| 10 | **PB0** | GPIOB | Bus OneWire | DS18B20 (×5) | GPIO bit-bang | 3.3 V (pull-up 4.7 kΩ) | `PIN_ONEWIRE` |
| 11 | **PB3** | GPIOB | Encoder dirección B | E6B2-CWZ6C CH-B | TIM2_CH2 (AF1) | 3.3 V o 5 V (con adaptación) | `PIN_ENC_B` |
| 12 | **PB4** | GPIOB | Encoder índice Z | E6B2-CWZ6C CH-Z | EXTI4 | 3.3 V o 5 V (con adaptación) | `PIN_ENC_Z` |
| 13 | **PB5** | GPIOB | Sensor centro dirección | LJ12A3 inductivo | EXTI5 | 3.3 V (con adaptación si necesario) | `PIN_STEER_CENTER` |
| 14 | **PB6** | GPIOB | I2C SCL | TCA9548A + INA226 | I2C1_SCL (AF4) | 3.3 V (open-drain, pull-up ext.) | `PIN_I2C_SCL` |
| 15 | **PB7** | GPIOB | I2C SDA | TCA9548A + INA226 | I2C1_SDA (AF4) | 3.3 V (open-drain, pull-up ext.) | `PIN_I2C_SDA` |
| 16 | **PB8** | GPIOB | CAN RX | Transceiver CAN | FDCAN1_RX (AF9) | 3.3 V lógico (NO conectar a CANH/CANL) | `PIN_CAN_RX` |
| 17 | **PB9** | GPIOB | CAN TX | Transceiver CAN | FDCAN1_TX (AF9) | 3.3 V lógico (NO conectar a CANH/CANL) | `PIN_CAN_TX` |
| 18 | **PB15** | GPIOB | Sensor velocidad RR | Inductivo (LJ12A3) | EXTI15 | 3.3 V (con adaptación si necesario) | `PIN_WHEEL_RR` |
| 19 | **PC0** | GPIOC | Dirección motor FL | BTS7960 FL (DIR) | GPIO Output | 3.3 V digital | `PIN_DIR_FL` |
| 20 | **PC1** | GPIOC | Dirección motor FR | BTS7960 FR (DIR) | GPIO Output | 3.3 V digital | `PIN_DIR_FR` |
| 21 | **PC2** | GPIOC | Dirección motor RL | BTS7960 RL (DIR) | GPIO Output | 3.3 V digital | `PIN_DIR_RL` |
| 22 | **PC3** | GPIOC | Dirección motor RR | BTS7960 RR (DIR) | GPIO Output | 3.3 V digital | `PIN_DIR_RR` |
| 23 | **PC4** | GPIOC | Dirección motor STEER | BTS7960 STEER (DIR) | GPIO Output | 3.3 V digital | `PIN_DIR_STEER` |
| 24 | **PC5** | GPIOC | Enable motor FL | BTS7960 FL (EN) | GPIO Output | 3.3 V digital | `PIN_EN_FL` |
| 25 | **PC6** | GPIOC | Enable motor FR | BTS7960 FR (EN) | GPIO Output | 3.3 V digital | `PIN_EN_FR` |
| 26 | **PC7** | GPIOC | Enable motor RL | BTS7960 RL (EN) | GPIO Output | 3.3 V digital | `PIN_EN_RL` |
| 27 | **PC8** | GPIOC | PWM motor STEER | BTS7960 STEER | TIM8_CH3 (AF4) | 3.3 V PWM | `PIN_PWM_STEER` |
| 28 | **PC9** | GPIOC | Enable motor STEER | BTS7960 STEER (EN) | GPIO Output | 3.3 V digital | `PIN_EN_STEER` |
| 29 | **PC10** | GPIOC | Relé MAIN | Relé principal | GPIO Output | 3.3 V (vía driver) | `PIN_RELAY_MAIN` |
| 30 | **PC11** | GPIOC | Relé TRACCIÓN | Relé tracción (24 V) | GPIO Output | 3.3 V (vía driver) | `PIN_RELAY_TRAC` |
| 31 | **PC12** | GPIOC | Relé DIRECCIÓN | Relé dirección (12 V) | GPIO Output | 3.3 V (vía driver) | `PIN_RELAY_DIR` |
| 32 | **PC13** | GPIOC | Enable motor RR | BTS7960 RR (EN) | GPIO Output | 3.3 V digital | `PIN_EN_RR` |

**Total: 32 pines del STM32 en uso.**

---

## 9. Cosas que NO existen en el STM32

Las siguientes funciones **NO están implementadas** en el firmware del STM32G474RE.
Están gestionadas por el ESP32 o simplemente no existen en el sistema:

| Función | Estado | Explicación |
|---------|--------|-------------|
| **Palanca de cambios (F/N/R)** | **En el ESP32** | El ESP32 lee la palanca física y envía el modo al STM32 via CAN (ID 0x102). No hay pines GPIO en el STM32 para la palanca. Los pines PB12/PB13/PB14 mencionados en documentación anterior **no están inicializados** en `MX_GPIO_Init()`. |
| **Llave de contacto** | **No implementado en STM32** | No existe ningún pin GPIO ni lógica de lectura de llave en el firmware. |
| **Pantalla / Display** | **En el ESP32** | No hay periféricos SPI/paralelo para display en el STM32. La interfaz HMI reside en el ESP32. |
| **Audio / Buzzer** | **No implementado en STM32** | No hay salida DAC, I2S ni PWM para audio. |
| **Sensores de obstáculos / Ultrasonidos** | **En el ESP32 o no implementado** | No hay lecturas de ultrasonidos ni sensores de distancia en el firmware STM32. |
| **Iluminación (luces, intermitentes)** | **En el ESP32 o no implementado** | No hay GPIOs configurados para control de luces en el firmware STM32. |
| **GPS** | **No implementado en STM32** | No hay periférico UART para GPS. |
| **Bluetooth / WiFi** | **En el ESP32** | El STM32G474RE no tiene radio inalámbrica. |

---

## Notas finales

### Prioridades de interrupción (NVIC)

Configuradas en `stm32g4xx_hal_msp.c`:

| Prioridad | Periférico | Función |
|-----------|-----------|---------|
| 1 (más alta) | FDCAN1_IT0 | Recepción CAN |
| 2 | TIM1_UP, TIM2, TIM8_UP | PWM y encoder |
| 2 | EXTI0, EXTI1, EXTI2, EXTI15_10 | Sensores de velocidad de rueda |
| 3 | I2C1_EV, I2C1_ER | Comunicación I2C |

### Watchdog (IWDG)

| Parámetro | Valor |
|-----------|-------|
| Prescaler | 32 |
| Reloj | 32 kHz (LSI) ÷ 32 = 1 kHz |
| Reload | 4095 |
| Timeout | **~4 s** (el comentario en `main.c` dice "~500 ms" pero el cálculo real con prescaler 32, reload 4095 y LSI 32 kHz da ≈4,1 s) |

Si el firmware no refresca el watchdog dentro de este periodo, el MCU se reinicia
automáticamente.

### Resumen de buses de comunicación

| Bus | Pines | Velocidad | Dispositivos |
|-----|-------|-----------|-------------|
| **FDCAN1** | PB8 (RX), PB9 (TX) | 500 kbps | ESP32 (vía transceiver) |
| **I2C1** | PB6 (SCL), PB7 (SDA) | 400 kHz | TCA9548A (0x70) → 6× INA226 (0x40) |
| **OneWire** | PB0 | ~16 kbps (bit-bang) | 5× DS18B20 |
| **ADC1** | PA3 | N/A (polling) | Pedal acelerador |

---

> **Este documento ha sido generado exclusivamente a partir del código fuente del firmware.**
> Todos los pines, direcciones, constantes y comportamientos están trazados a archivos
> específicos del repositorio. Las incertidumbres están marcadas como
> "NO DEDUCIBLE SOLO DESDE EL CÓDIGO".
