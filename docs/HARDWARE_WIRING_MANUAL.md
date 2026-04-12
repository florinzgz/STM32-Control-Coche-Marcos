# MANUAL ELÉCTRICO DE CABLEADO — STM32G474RE

**Documento de referencia para taller — Solo conexiones reales**

> **IMPORTANTE**: Todo lo documentado aquí está trazado directamente al código fuente del firmware
> (`project_config.h`, `main.c`, `motor_control.c`, `sensor_manager.c`, `safety_system.c`,
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
10. [Conexión directa BTS7960 — RPWM/LPWM desde STM32 (sin lógica externa)](#10-conexión-directa-bts7960--rpwmlpwm-desde-stm32-sin-lógica-externa)
11. [Circuito completo de driver de relé con optoacoplador](#11-circuito-completo-de-driver-de-relé-con-optoacoplador)
12. [Protección anti-reinicios y anti-sobretensiones](#12-protección-anti-reinicios-y-anti-sobretensiones)

---

## ⚠️ COMPONENTES DE PROTECCIÓN — OBLIGATORIOS ANTES DE ENCENDER

> **Leer esta sección antes de conectar cualquier cable.**
> Sin estos componentes, el sistema puede sufrir reinicios inexplicables del STM32
> (causados por ruido en la alimentación) o destrucción de pines del MCU
> (causada por sobretensión de sensores o motores).

### Resumen de componentes críticos por zona

| Zona | Componente | Valor | Función | Sin él, puede pasar... |
|------|-----------|-------|---------|------------------------|
| STM32 Nucleo 3.3 V | Condensador cerámico | 100 nF | Desacoplo rápido VDD | Reinicio por pico de ruido PWM |
| STM32 Nucleo 3.3 V | Condensador electrolítico | 10 µF | Desacoplo lento (bulk) | Reinicio por demanda transitoria de corriente |
| Cada BTS7960 lógica VCC | Condensador cerámico | 100 nF | Desacoplo local | Glitch lógico al conmutar 20 kHz |
| Bus 24 V (junto a relés) | Condensador electrolítico | 1 000 µF / 35 V | Absorción inrush | Sobretensión en cierre de relé principal |
| Bus 12 V (dirección) | Condensador electrolítico | 470 µF / 25 V | Absorción inrush | Sobretensión al activar motor dirección |
| Cada bobina de relé | Diodo 1N4007 | 1 A / 1 000 V | Flyback (anti-pico inductivo) | Pico de -100 V destruye GPIO del STM32 |
| Cada terminal de motor | Condensador cerámico | 100 nF / 50 V | Snubber EMI | Ruido RF en señales I2C / CAN / ADC |
| Encoder → STM32 (PA15, PB3) | Divisor resistivo o BSS138 | ver §12 | Limitación 5 V → 3,3 V | Destrucción permanente del pin STM32 |
| Sensor LJ12A3 → STM32 (EXTI) | Optoacoplador PC817 / 6N137 | ver `CABLEADO_AISLAMIENTO_DEFINITIVO.md` | Aislamiento 6–36 V → 3,3 V | Pico inductivo destruye pin STM32 |
| Pedal Hall → PA3 (ADC) | Divisor 10 kΩ + 6,8 kΩ | ver §6.5 | Escalado 5 V → 2 V | Destrucción ADC si supera 3,6 V |
| CAN bus | Resistencia terminación | 120 Ω × 2 | Terminación diferencial | Errores CAN, STM32 entra en SAFE |

---

## 1. Visión general del sistema

### Rol del STM32G474RE

El STM32G474RE (Nucleo-64) es el **controlador de tiempo real** del vehículo. Ejecuta a 170 MHz
(HSI 16 MHz → PLL ×85 ÷2) y se encarga de:

- Control PWM de los motores de tracción FL/FR (20 kHz, TIM1) y RL/RR (20 kHz, TIM8)
- Control PWM del motor de dirección (20 kHz, TIM3)
- Lectura del encoder de dirección (TIM2, modo cuadratura)
- Lectura de 4 sensores inductivos de velocidad de rueda (EXTI)
- Lectura de 6 sensores de corriente INA226 (I2C1 vía TCA9548A)
- Lectura de 5 sensores de temperatura DS18B20 (OneWire, PB0)
- Lectura del pedal de acelerador (ADC1, PA3)
- Detección del sensor inductivo de centrado de dirección (EXTI5)
- Control de 3 relés de potencia (MAIN, TRACCIÓN, DIRECCIÓN)
- Comunicación CAN con el ESP32 (FDCAN1, 500 kbps)
- Sistemas de seguridad: ABS, TCS, límites de corriente/temperatura
- Watchdog independiente (IWDG, ~500 ms)

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
| **3.3 V** | Lógica de los módulos BTS7960 (VCC lógico) — ⚠️ NO usar 5V (ver §10) |
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
- **NUNCA** conectar el bus CAN (CANH/CANL) directamente a PA11/PA12 — se necesita transceiver
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
| CAN TX | **PA12** | FDCAN1_TX (AF9) | `PIN_CAN_TX` (`project_config.h:226`) |
| CAN RX | **PA11** | FDCAN1_RX (AF9) | `PIN_CAN_RX` (`project_config.h:227`) |

Configuración en `stm32g4xx_hal_msp.c`: GPIO modo AF push-pull, velocidad alta, función alternativa AF9.

### Transceiver CAN — OBLIGATORIO

**PA11 y PA12 son salidas lógicas CMOS a 3.3 V. NO son señales CAN.**

Es **obligatorio** un transceiver CAN externo (por ejemplo: MCP2551, SN65HVD230, TJA1050)
entre el STM32 y el bus CAN físico.

### Esquema de conexión

```
STM32G474RE            Transceiver CAN              Bus CAN
┌──────────┐          ┌──────────────┐          ┌───────────┐
│     PA12 ├──────────► TXD      CANH├──────────► CANH      │
│  (CAN TX)│          │              │          │           │
│     PA11 ◄──────────┤ RXD      CANL├──────────► CANL      │
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

Cada BTS7960 de tracción recibe **2 señales PWM + 1 señal Enable** del STM32:

| Señal | Función | Nivel |
|-------|---------|-------|
| **RPWM** | PWM de avance (20 kHz, TIM1/TIM8) | 0–3.3 V → lógica BTS7960 |
| **LPWM** | PWM de retroceso (20 kHz, TIM1/TIM8) | 0–3.3 V → lógica BTS7960 |
| **EN** | Habilitación del driver (GPIO o 3.3V fijo) | HIGH = habilitado |

> **Nota:** La dirección se codifica por elección de canal: RPWM > 0 / LPWM = 0 (avance)
> o RPWM = 0 / LPWM > 0 (retroceso). **No hay pin DIR.** Los pines DIR (PC0–PC4) han sido
> liberados y no deben conectarse.

### Tabla por rueda

| Rueda | RPWM (Pin) | Timer/Canal | LPWM (Pin) | Timer/Canal | DIR (Pin) | EN (Pin) | Sensor velocidad | INA226 (índice) | DS18B20 (índice) |
|-------|------------|-------------|------------|-------------|-----------|----------|------------------|-----------------|-------------------|
| **FL** (Frontal izq.) | PA8 | TIM1_CH1 | PA9 | TIM1_CH2 | *(freed)* | PC5 (GPIO) | PA0 (EXTI0) | 0 | 0 |
| **FR** (Frontal der.) | PA10 | TIM1_CH3 | PC3 | TIM1_CH4 | *(freed → EN)* | PC0 (GPIO) | PA1 (EXTI1) | 1 | 1 |
| **RL** (Trasera izq.) | PC6 | TIM8_CH1 | PC7 | TIM8_CH2 | *(freed → EN)* | PC1 (GPIO) | PA2 (EXTI2) | 2 | 2 |
| **RR** (Trasera der.) | PC8 | TIM8_CH3 | PC9 | TIM8_CH4 | *(freed)* | PC13 (GPIO) | PB15 (EXTI15) | 3 | 3 |

> **Nota**: DIR pins (PC0–PC2, PC4) ya no son controlados por el firmware — la dirección se
> determina por la elección de RPWM vs LPWM. PC3 (ex-DIR_RR) fue reasignado a LPWM_FR (TIM1_CH4).
> Los demás DIR pins deben dejarse desconectados o configurados como GPIO output LOW.
> EN pins: todos los motores (FL, FR, RL, RR, STEER) tienen EN controlado por GPIO:
> PC5 (FL), PC0 (FR), PC1 (RL), PC13 (RR), PC4 (STEER).
> NO conectar R_EN/L_EN a 3.3V fijo — el firmware controla el enable.

> **Nota sobre índices INA226 y DS18B20**: El firmware usa indexación 0–5 para INA226
> (6 sensores) y 0–4 para DS18B20 (5 sensores). El código de seguridad
> (`safety_system.c`) itera sobre los 4 primeros índices para las ruedas.
> La asignación exacta índice→rueda física depende del orden de conexión en el
> multiplexor TCA9548A (INA226) y del orden de descubrimiento en el bus OneWire (DS18B20).
> **NO DEDUCIBLE SOLO DESDE EL CÓDIGO**: verificar el cableado físico del multiplexor
> y el orden de enumeración ROM de los DS18B20.

### Configuración PWM (TIM1 — tracción FL/FR)

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Frecuencia | **20 kHz** | `motor_control.c`: `PWM_PERIOD = 4249` |
| Periodo (ARR) | 4249 | `main.c`: `MX_TIM1_Init` |
| Prescaler | 0 | `main.c`: `MX_TIM1_Init` |
| Modo contador | Center-Aligned | 170 MHz ÷ (2 × 4250) = 20 kHz |
| Canales activos | CH1 (RPWM_FL), CH2 (LPWM_FL), CH3 (RPWM_FR), CH4 (LPWM_FR) | `project_config.h` |
| BREAK2 | Cortex LOCKUP | Hardware PWM kill en fallo CPU |

### Configuración PWM (TIM8 — tracción RL/RR)

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Frecuencia | **20 kHz** | `main.c`: `MX_TIM8_Init` |
| Periodo (ARR) | 4249 | Center-Aligned: 170 MHz ÷ (2 × 4250) = 20 kHz |
| Canales activos | CH1 (RPWM_RL), CH2 (LPWM_RL), CH3 (RPWM_RR), CH4 (LPWM_RR) | `project_config.h` |
| BREAK2 | Cortex LOCKUP | Hardware PWM kill en fallo CPU |

### Conexión STM32 → BTS7960 (por motor de tracción)

El firmware genera **dos señales PWM independientes** por motor: RPWM y LPWM, directamente
desde los timers hardware (TIM1, TIM3, TIM8). **No se necesita circuito de adaptación externo.**

Ver [Sección 10](#10-conexión-directa-bts7960--rpwmlpwm-desde-stm32-sin-lógica-externa) para la tabla completa de asignación de pines.

**Resumen de conexiones directas (sin lógica 74HC):**

```
STM32                                               BTS7960 (motor FL)
──────────────                                      ──────────────────
PA8  (TIM1_CH1 — RPWM_FL) ───────────────────────► RPWM  (avance)
PA9  (TIM1_CH2 — LPWM_FL) ───────────────────────► LPWM  (retroceso)
PC5  (GPIO EN_FL)          ───────────────────────► R_EN ─┐
                                                           ├─ (unir)
                                                    L_EN ─┘
                                                    VCC ← 3.3 V (⚠️ NO 5V — ver nota §10)
                                                    B+  ← 24 V (vía relé TRAC + shunt INA226 1.5 mΩ)
                                                    B-  ← GND potencia
```

> **Motor FR**: PA10 (TIM1_CH3 → RPWM), PC3 (TIM1_CH4 → LPWM, AF2). EN: PC0 (GPIO output).
> **Motor RL**: PC6 (TIM8_CH1 → RPWM), PC7 (TIM8_CH2 → LPWM). EN: PC1 (GPIO output).
> **Motor RR**: PC8 (TIM8_CH3 → RPWM), PC9 (TIM8_CH4 → LPWM). EN: PC13 GPIO.
> **Motor STEER**: PA6 (TIM3_CH1 → RPWM), PA7 (TIM3_CH2 → LPWM). EN: PC4 (GPIO output).

### ⚡ Condensadores obligatorios por BTS7960 de tracción (4× motores)

Instalar **todos los condensadores siguientes** antes de encender el sistema. Sin ellos, el PWM a 20 kHz genera transitorios que pueden resetear el STM32 o dañar sensores.

| Componente | Valor | Ubicación exacta | Función |
|-----------|-------|-----------------|---------|
| C_bulk | **470 µF / 35 V** electrolítico (105°C) | Entre **B+** y **GND** de cada BTS7960 de tracción | Reserva de energía; absorbe inrush al arrancar el motor y picos del bus 24V |
| C_bypass potencia | **100 nF / 50 V** X7R cerámico | Entre **B+** y **GND**, lo más cerca posible del IC BTS7960 | Filtra armónicos del PWM 20 kHz en el bus de potencia |
| C_bypass lógica | **100 nF / 50 V** X7R cerámico | Entre **VCC** y **GND** lógicos del módulo (junto al pin VCC) | Desacoplo del buffer 74HC244; evita glitches de control a 20 kHz |
| C_snubber motor | **100 nF / 50 V** X7R cerámico | Entre **M+** y **M-** del motor (fijar al conector del motor, NO en el BTS7960) | Suprime picos de contra-EMF del motor brushed; filtra ruido EMI en I2C/CAN/ADC |

```
         B+ (24V) ──┬──────────────────► BTS7960 B+
                    │                          │
             [470µF/35V]  [100nF/50V]       Motor
              (bulk)       (bypass)        M+ ──┤
                    │           │               │ [100nF/50V]
         GND ───────┴───────────┴               │  (snubber)
                                            M- ──┤
         VCC_logic ──────┬──────────► VCC               │
                   [100nF/50V]          └────────────────┘
         GND_logic ──────┘
```

**Condensador de desacoplo obligatorio en VCC del BTS7960:**
```
3.3V ────[100 nF cerámico]──── GND   (soldar junto al pin VCC de cada módulo BTS7960)
```

> ⚠️ **VCC LÓGICA = 3.3V (NO 5V):** El módulo IBT-2 incluye un buffer 74HC244 cuyo
> V_IH(min) = 0.7 × VCC. A VCC=5V → V_IH(min)=3.5V, y las señales de 3.3V del STM32
> quedan **por debajo del umbral** → riesgo de control intermitente del motor.
> Con VCC=3.3V → V_IH(min)=2.31V, bien por debajo de 3.3V. Ver `project_config.h` líneas 48–59.

---

## 5. Motor de dirección

### Driver

**BTS7960** (1 unidad dedicada a dirección).

### Tensión

**12 V** para el motor de dirección (vía relé DIR + shunt INA226 #5, 1.5 mΩ).

### Pines STM32

| Señal | Pin | Función | Referencia |
|-------|-----|---------|------------|
| RPWM | **PA6** | TIM3_CH1 (AF2) | `PIN_PWM_STEER` (`project_config.h`) |
| LPWM | **PA7** | TIM3_CH2 (AF2) | `PIN_LPWM_STEER` (`project_config.h`) |

> Los pines DIR (PC4) y EN (PC9) ya **no se usan**. PC9 ha sido reasignado a TIM8_CH4 (LPWM_RR).
> EN: PC4 (GPIO output). NO conectar a 3.3V fijo — el firmware controla el enable.

### ⚡ Condensadores obligatorios — BTS7960 de dirección

> **Especialmente crítico:** El encoder E6B2-CWZ6C está físicamente próximo al motor de
> dirección. Sin el condensador snubber en los terminales del motor, la contra-EMF puede
> corromper los pulsos del encoder → fallo de calibración `SAFETY_ERROR_CENTERING`.

| Componente | Valor | Ubicación exacta | Función |
|-----------|-------|-----------------|---------|
| C_bulk | **470 µF / 25 V** electrolítico (105°C) | Entre **B+** (12V) y **GND** del BTS7960 STEER | Reserva de energía; absorbe inrush y oscilaciones al girar la dirección |
| C_bypass potencia | **100 nF / 50 V** X7R cerámico | Entre **B+** y **GND**, junto al IC BTS7960 STEER | Filtra PWM 20 kHz en el bus de 12V |
| C_bypass lógica | **100 nF / 50 V** X7R cerámico | Entre **VCC** y **GND** lógicos del BTS7960 STEER | Desacoplo del 74HC244 |
| C_snubber motor | **100 nF / 50 V** X7R cerámico | Entre **M+** y **M-** del motor de dirección (junto al motor) | **Crítico:** suprime contra-EMF que deteriora las señales del encoder |

```
         B+ (12V) ──┬──────────────────► BTS7960 STEER B+
                    │                            │
             [470µF/25V]  [100nF/50V]         Motor STEER
              (bulk)       (bypass)          M+ ──┤
                    │           │                 │ [100nF/50V]
         GND ───────┴───────────┴                 │  (snubber)
                                             M- ──┤
         VCC_logic ──────┬──────────► VCC                │
                   [100nF/50V]          └─────────────────┘
         GND_logic ──────┘
```

### Configuración PWM (TIM3)

| Parámetro | Valor |
|-----------|-------|
| Frecuencia | **20 kHz** |
| Periodo (ARR) | 4249 |
| Modo contador | Center-Aligned |
| Prescaler | 0 |
| Canales | CH1 (RPWM_STEER), CH2 (LPWM_STEER) |
| BREAK | No disponible (TIM3 es general-purpose) |
| Protección fallo | Fault handlers escriben CCR1=0, CCR2=0 directamente |

### Encoder de dirección (E6B2-CWZ6C)

Definido en `project_config.h` y leído por TIM2 en modo cuadratura.

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Modelo | E6B2-CWZ6C | comentario en `project_config.h` |
| PPR | **1200** | `ENCODER_PPR` (`project_config.h`) |
| CPR (×4 cuadratura) | **4800** | `ENCODER_CPR` (`project_config.h`) |
| Canal A | **PA15** (TIM2_CH1, AF1) | `PIN_ENC_A` (`project_config.h`) |
| Canal B | **PB3** (TIM2_CH2, AF1) | `PIN_ENC_B` (`project_config.h`) |
| Índice (Z) | **PB4** (EXTI4) | `PIN_ENC_Z` (`project_config.h`) |
| Periodo TIM2 | 65535 | `main.c`: `MX_TIM2_Init` |
| Modo conteo | TI12 (ambos canales, ×4) | `main.c` |

### Sensor inductivo de centrado

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Tipo | **LJ12A3** (inductivo) | `project_config.h` |
| Pin | **PB5** | `PIN_STEER_CENTER` (`project_config.h`) |
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
| FL | **PA0** | GPIOA | EXTI0 | Ascendente | Sí (interno) | `PIN_WHEEL_FL` (`project_config.h:126`) |
| FR | **PA1** | GPIOA | EXTI1 | Ascendente | Sí (interno) | `PIN_WHEEL_FR` (`project_config.h:127`) |
| RL | **PA2** | GPIOA | EXTI2 | Ascendente | Sí (interno) | `PIN_WHEEL_RL` (`project_config.h:128`) |
| RR | **PB15** | GPIOB | EXTI15 | Ascendente | Sí (interno) | `PIN_WHEEL_RR` (`project_config.h:129`) |

Constantes de cálculo de velocidad (`project_config.h`, `vehicle_physics.h`):

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
| Cantidad | **5** | `NUM_DS18B20` (`project_config.h:239`) |
| Bus | **OneWire** (bit-bang) | `sensor_manager.c` |
| Pin | **PB0** | `PIN_ONEWIRE` (`project_config.h:209`) |
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
| Cantidad | **6** | `NUM_INA226` (`project_config.h:238`) |
| Bus | **I2C1** (400 kHz, Fast Mode) | `main.c`: `MX_I2C1_Init` |
| Pin SCL | **PB6** | `PIN_I2C_SCL` (`project_config.h:174`) |
| Pin SDA | **PB7** | `PIN_I2C_SDA` (`project_config.h:175`) |
| Dirección INA226 | **0x40** | `I2C_ADDR_INA226` (`project_config.h:233`) |
| Multiplexor | **TCA9548A** en dirección **0x70** | `I2C_ADDR_TCA9548A` (`project_config.h:232`) |
| Resistencia shunt (motor) | **1.5 mΩ** (50A/75mV) | `INA226_SHUNT_MOHM_MOTOR` (`project_config.h:257`) |
| Resistencia shunt (batería) | **0.75 mΩ** (100A/75mV) | `INA226_SHUNT_MOHM_BATTERY` (`project_config.h:258`) |

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

| Índice | Canal TCA9548A | Uso (por firmware) | Colocación del shunt |
|--------|----------------|---------------------|----------------------|
| 0 | CH0 | Motor FL | ANTES del BTS7960 FL (entre relé TRAC y B+ del driver) |
| 1 | CH1 | Motor FR | ANTES del BTS7960 FR (entre relé TRAC y B+ del driver) |
| 2 | CH2 | Motor RL | ANTES del BTS7960 RL (entre relé TRAC y B+ del driver) |
| 3 | CH3 | Motor RR | ANTES del BTS7960 RR (entre relé TRAC y B+ del driver) |
| 4 | CH4 | Batería 24V (corriente+tensión) | **ANTES del relé MAIN** (entre borne + batería y COM del relé) |
| 5 | CH5 | Motor dirección | ANTES del BTS7960 STEER (entre relé DIR y B+ del driver) |

> **IMPORTANTE — Colocación del shunt de batería (índice 4):** El INA226 de batería debe
> estar **ANTES del relé principal** (entre el borne + de la batería y la entrada del relé
> MAIN). Esto garantiza que `Voltage_GetBus()` pueda leer el voltaje de la batería en todo
> momento, incluso cuando el relé está abierto. Si se colocara después del relé, al abrir
> el relé se leería 0 V y el firmware lo trataría como fallo crítico de subtensión.
>
> **Colocación de los shunts de motor (índices 0–3, 5):** Los INA226 de motor van **ANTES
> de cada driver BTS7960**, en el cable positivo de alimentación entre la salida del relé
> y la entrada B+ del BTS7960. Esto mide la corriente de alimentación que entra al driver,
> que es igual a la corriente del motor (menos pequeñas pérdidas del driver).

> **NO DEDUCIBLE SOLO DESDE EL CÓDIGO**: la asignación exacta de cada índice INA226 a un
> motor específico depende del cableado físico del multiplexor. El firmware solo itera
> por índice numérico. Los 4 primeros (0–3) se usan en los bucles de seguridad de ruedas
> y los 6 se leen en el ciclo completo de sensores.

### 6.5 Sensor de pedal (acelerador)

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Tipo | **Analógico** (potenciómetro o sensor Hall) | `sensor_manager.c` |
| Pin | **PA3** | `PIN_PEDAL` (`project_config.h:221`) |
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
| **LED** (Frontal) | **PB10** | GPIOB | Alimentación 5V tira WS2812B frontal (28 LEDs) | 5 V |
| **LED_REAR** (Trasero) | **PB11** | GPIOB | Alimentación 5V tira WS2812B trasera (16 LEDs) | 5 V |

Definidos en `project_config.h`:
- `PIN_RELAY_MAIN` = GPIO_PIN_10 (PC10)
- `PIN_RELAY_TRAC` = GPIO_PIN_11 (PC11)
- `PIN_RELAY_DIR`  = GPIO_PIN_12 (PC12)
- `PIN_RELAY_LED`  = GPIO_PIN_10 (PB10)
- `PIN_RELAY_LED_REAR` = GPIO_PIN_11 (PB11)

> **Nota:** Los relés LED (PB10/PB11) se controlan mediante el comando CAN ID 0x120
> enviado desde el ESP32. El ESP32 genera la señal de datos WS2812B; el STM32 solo
> controla el corte de alimentación 5V por seguridad.

> **Nota**: Los pines GPIO del STM32 (3.3 V, máximo ~20 mA) **no pueden accionar un relé
> directamente**. Se requiere un transistor/MOSFET de conmutación o un módulo de relé
> con opto-aislamiento. Ver [Sección 11](#11-circuito-completo-de-driver-de-relé-con-optoacoplador)
> para el esquema completo con todos los componentes de protección.

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
| 4 | **PA3** | GPIOA | Pedal acelerador | Hall SS1324LUA-T | ADC1_IN4 | 0–3.3 V analógico (div. 10kΩ/6.8kΩ) | `PIN_PEDAL` |
| 5 | **PA6** | GPIOA | RPWM motor STEER | BTS7960 STEER | TIM3_CH1 (AF2) | 3.3 V PWM | `PIN_PWM_STEER` |
| 6 | **PA7** | GPIOA | LPWM motor STEER | BTS7960 STEER | TIM3_CH2 (AF2) | 3.3 V PWM | `PIN_LPWM_STEER` |
| 7 | **PA8** | GPIOA | RPWM motor FL | BTS7960 FL | TIM1_CH1 (AF6) | 3.3 V PWM | `PIN_PWM_FL` |
| 8 | **PA9** | GPIOA | LPWM motor FL | BTS7960 FL | TIM1_CH2 (AF6) | 3.3 V PWM | `PIN_LPWM_FL` |
| 9 | **PA10** | GPIOA | RPWM motor FR | BTS7960 FR | TIM1_CH3 (AF6) | 3.3 V PWM | `PIN_PWM_FR` |
| 10 | **PC3** | GPIOC | LPWM motor FR | BTS7960 FR | TIM1_CH4 (AF2) | 3.3 V PWM | `PIN_LPWM_FR` |
| 11 | **PA15** | GPIOA | Encoder dirección A | E6B2-CWZ6C CH-A | TIM2_CH1 (AF1) | 3.3 V o 5 V (con adaptación) | `PIN_ENC_A` |
| 12 | **PB0** | GPIOB | Bus OneWire | DS18B20 (×5) | GPIO bit-bang | 3.3 V (pull-up 4.7 kΩ) | `PIN_ONEWIRE` |
| 13 | **PB3** | GPIOB | Encoder dirección B | E6B2-CWZ6C CH-B | TIM2_CH2 (AF1) | 3.3 V o 5 V (con adaptación) | `PIN_ENC_B` |
| 14 | **PB4** | GPIOB | Encoder índice Z | E6B2-CWZ6C CH-Z | EXTI4 | 3.3 V o 5 V (con adaptación) | `PIN_ENC_Z` |
| 15 | **PB5** | GPIOB | Sensor centro dirección | LJ12A3 inductivo | EXTI5 | 3.3 V (con adaptación si necesario) | `PIN_STEER_CENTER` |
| 16 | **PB6** | GPIOB | I2C SCL | TCA9548A + INA226 | I2C1_SCL (AF4) | 3.3 V (open-drain, pull-up ext.) | `PIN_I2C_SCL` |
| 17 | **PB7** | GPIOB | I2C SDA | TCA9548A + INA226 | I2C1_SDA (AF4) | 3.3 V (open-drain, pull-up ext.) | `PIN_I2C_SDA` |
| 18 | **PA11** | GPIOA | CAN RX | Transceiver CAN | FDCAN1_RX (AF9) | 3.3 V lógico (NO conectar a CANH/CANL) | `PIN_CAN_RX` |
| 19 | **PA12** | GPIOA | CAN TX | Transceiver CAN | FDCAN1_TX (AF9) | 3.3 V lógico (NO conectar a CANH/CANL) | `PIN_CAN_TX` |
| 20 | **PB10** | GPIOB | Relé LED frontal | Relé 5V tira WS2812B | GPIO Output | 3.3 V (vía driver) | `PIN_RELAY_LED` |
| 21 | **PB11** | GPIOB | Relé LED trasero | Relé 5V tira WS2812B | GPIO Output | 3.3 V (vía driver) | `PIN_RELAY_LED_REAR` |
| 22 | **PB15** | GPIOB | Sensor velocidad RR | Inductivo (LJ12A3) | EXTI15 | 3.3 V (con adaptación si necesario) | `PIN_WHEEL_RR` |
| 23 | **PC0** | GPIOC | Enable motor FR | BTS7960 FR (EN) | GPIO Output | 3.3 V digital | `PIN_EN_FR` |
| 24 | **PC1** | GPIOC | Enable motor RL | BTS7960 RL (EN) | GPIO Output | 3.3 V digital | `PIN_EN_RL` |
| 25 | **PC4** | GPIOC | Enable motor STEER | BTS7960 STEER (EN) | GPIO Output | 3.3 V digital | `PIN_EN_STEER` |
| 26 | **PC5** | GPIOC | Enable motor FL | BTS7960 FL (EN) | GPIO Output | 3.3 V digital | `PIN_EN_FL` |
| 27 | **PC6** | GPIOC | RPWM motor RL | BTS7960 RL | TIM8_CH1 (AF4) | 3.3 V PWM | `PIN_PWM_RL` |
| 28 | **PC7** | GPIOC | LPWM motor RL | BTS7960 RL | TIM8_CH2 (AF4) | 3.3 V PWM | `PIN_LPWM_RL` |
| 29 | **PC8** | GPIOC | RPWM motor RR | BTS7960 RR | TIM8_CH3 (AF4) | 3.3 V PWM | `PIN_PWM_RR` |
| 30 | **PC9** | GPIOC | LPWM motor RR | BTS7960 RR | TIM8_CH4 (AF4) | 3.3 V PWM | `PIN_LPWM_RR` |
| 31 | **PC10** | GPIOC | Relé MAIN | Relé principal | GPIO Output | 3.3 V (vía driver) | `PIN_RELAY_MAIN` |
| 32 | **PC11** | GPIOC | Relé TRACCIÓN | Relé tracción (24 V) | GPIO Output | 3.3 V (vía driver) | `PIN_RELAY_TRAC` |
| 33 | **PC12** | GPIOC | Relé DIRECCIÓN | Relé dirección (12 V) | GPIO Output | 3.3 V (vía driver) | `PIN_RELAY_DIR` |
| 34 | **PC13** | GPIOC | Enable motor RR | BTS7960 RR (EN) | GPIO Output | 3.3 V digital | `PIN_EN_RR` |

> **Nota:** Los pines de dirección antiguos (PC0, PC1, PC4) han sido reasignados como EN (enable)
> para FR, RL y STEER respectivamente. RPWM/LPWM se generan directamente desde los timers hardware.
> Todos los motores (FL, FR, RL, RR, STEER) tienen EN controlado por GPIO.

**Total: 34 pines del STM32 en uso.**

---

## 9. Cosas que NO existen en el STM32

Las siguientes funciones **NO están implementadas** en el firmware del STM32G474RE.
Están gestionadas por el ESP32 o simplemente no existen en el sistema:

| Función | Estado | Explicación |
|---------|--------|-------------|
| **Palanca de cambios (F/N/R)** | **En el ESP32** | El ESP32 lee la palanca física y envía el modo al STM32 via CAN (ID 0x102). No hay pines GPIO en el STM32 para la palanca. Los pines PB12/PB13 mencionados en documentación anterior **no están inicializados** en `MX_GPIO_Init()`. PB14 ahora es LED_DIAG (GPIO_Output). |
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
| **FDCAN1** | PA11 (RX), PA12 (TX) | 500 kbps | ESP32 (vía transceiver) |
| **I2C1** | PB6 (SCL), PB7 (SDA) | 400 kHz | TCA9548A (0x70) → 6× INA226 (0x40) |
| **OneWire** | PB0 | ~16 kbps (bit-bang) | 5× DS18B20 |
| **ADC1** | PA3 | N/A (polling) | Pedal acelerador |

---

> **Este documento ha sido generado exclusivamente a partir del código fuente del firmware.**
> Todos los pines, direcciones, constantes y comportamientos están trazados a archivos
> específicos del repositorio.

---

## 10. Conexión directa BTS7960 — RPWM/LPWM desde STM32 (sin lógica externa)

> **ACTUALIZACIÓN DE HARDWARE** — Los chips 74HC08 (AND) y 74HC04 (NOT) han sido
> **eliminados completamente**. El firmware genera RPWM y LPWM directamente desde
> timers hardware del STM32G474RE. No se necesita ninguna lógica externa.

### Por qué ya no se necesitan los 74HC

El firmware anterior generaba una sola señal PWM + una señal DIR (GPIO), y los 74HC
convertían esa combinación en RPWM/LPWM. El firmware actualizado usa **dos canales PWM
independientes por motor**, uno para RPWM y otro para LPWM, generados directamente por
los timers TIM1, TIM8 y TIM3. La lógica de dirección es interna al microcontrolador:

```
Motor_SetSigned(motor, +duty)  →  RPWM = duty,  LPWM = 0   (avance)
Motor_SetSigned(motor, -duty)  →  RPWM = 0,     LPWM = duty (retroceso)
Motor_SetSigned(motor,  0)     →  RPWM = 0,     LPWM = 0   (freno pasivo / coast)
```

**Garantía hardware**: RPWM y LPWM nunca son simultáneamente no-cero. La función
`Motor_SetSigned()` limpia el canal inactivo antes de escribir el canal activo. Ambos
CCR tienen OCPreload habilitado, por lo que el cambio real en el pin solo ocurre en el
siguiente período del timer (no hay glitch entre transiciones).

### Tabla de asignación de pines RPWM/LPWM

| Motor     | RPWM (avance)           | LPWM (retroceso)         | EN GPIO         |
|-----------|-------------------------|--------------------------|-----------------|
| FL (front-left)  | PA8 — TIM1_CH1   | PA9 — TIM1_CH2           | PC5 (GPIO out)  |
| FR (front-right) | PA10 — TIM1_CH3  | PC3 — TIM1_CH4 (AF2)    | PC0 (GPIO out)  |
| RL (rear-left)   | PC6 — TIM8_CH1   | PC7 — TIM8_CH2           | PC1 (GPIO out)  |
| RR (rear-right)  | PC8 — TIM8_CH3   | PC9 — TIM8_CH4           | PC13 (GPIO out) |
| STEER            | PA6 — TIM3_CH1   | PA7 — TIM3_CH2           | PC4 (GPIO out)  |

> **Nota EN**: Todos los motores (FL, FR, RL, RR, STEER) tienen EN controlado por GPIO:
> PC5 (FL), PC0 (FR), PC1 (RL), PC13 (RR), PC4 (STEER).
> NO conectar R_EN/L_EN a 3.3V fijo — el firmware controla el enable.

### Conexión directa — motor FL (ejemplo)

```
STM32 PA8  (TIM1_CH1) ─────────────────────────────► BTS7960 FL: RPWM
STM32 PA9  (TIM1_CH2) ─────────────────────────────► BTS7960 FL: LPWM
STM32 PC5  (GPIO EN)  ─────────────────────────────► BTS7960 FL: R_EN  ─┐
                                                                          ├─ (cable corto)
                                                      BTS7960 FL: L_EN  ─┘
                                                      BTS7960 FL: VCC  ◄── 3.3 V (⚠️ NO 5V)
                                                      BTS7960 FL: GND  ◄── GND común
                                                      BTS7960 FL: B+   ◄── 24 V (relé TRAC → shunt INA226 #0)
                                                      BTS7960 FL: B-   ◄── GND potencia
                                                      BTS7960 FL: M+   ──► Motor FL terminal A
                                                      BTS7960 FL: M-   ──► Motor FL terminal B
```

### Conexión directa — motor FR (ejemplo, EN GPIO PC0)

```
STM32 PA10 (TIM1_CH3) ─────────────────────────────► BTS7960 FR: RPWM
STM32 PC3  (TIM1_CH4, AF2) ────────────────────────► BTS7960 FR: LPWM
STM32 PC0  (GPIO EN_FR) ───────────────────────────► BTS7960 FR: R_EN  ─┐
                                                                          ├─ (unir con cable corto)
                                                      BTS7960 FR: L_EN  ─┘
```

> Lo mismo aplica para RL (PC6/PC7), RR (PC8/PC9, con PC13 como EN GPIO) y STEER (PA6/PA7).

### Tabla de funcionamiento RPWM/LPWM

| Estado        | RPWM        | LPWM        | Motor              |
|---------------|-------------|-------------|--------------------|
| Avance 50 %   | 50 % duty   | 0           | Avanza al 50 %     |
| Retroceso 50 %| 0           | 50 % duty   | Retrocede al 50 %  |
| Freno pasivo  | 0           | 0           | Freno (ambas LOW)  |

### Pines PC0–PC4 liberados (DIR, ya no se usan)

| Pin antiguo | Uso anterior  | Estado nuevo             |
|-------------|---------------|--------------------------|
| PC0         | DIR_FL GPIO   | Libre — no conectar      |
| PC1         | DIR_FR GPIO   | Libre — no conectar      |
| PC2         | DIR_RL GPIO   | Libre — no conectar      |
| PC3         | DIR_RR GPIO   | LPWM_FR — TIM1_CH4 (AF2) |
| PC4         | DIR_STEER GPIO| Libre — no conectar      |

### Componentes que se eliminan del BOM

| Componente eliminado              | Cantidad | Motivo |
|-----------------------------------|----------|--------|
| SN74HC08N (quad AND gate, DIP-14) | 3        | PWM/DIR combinados externamente — ya no necesario |
| SN74HC04N (hex NOT gate, DIP-14)  | 1        | Inversión de DIR — ya no necesario |
| Condensadores 100 nF desacoplo    | 4        | Para los CI lógicos eliminados |

> Los condensadores de snubber en los bornes del motor (100 nF entre M+ y M-)
> y los diodos 1N5408 anti-paralelo **se mantienen** — no tienen relación con la
> lógica de control y siguen siendo necesarios para protección anti-EMI.

### Frecuencia y modo PWM (sin cambios)

Todos los timers (TIM1, TIM3, TIM8) operan en modo **Center-Aligned PWM1** a **20 kHz**:
- Prescaler = 0, ARR = 4249, Prescaler = 0
- 170 MHz / (2 × 4250) = **20 kHz**
- Rango de duty: 0–4249 (igual que antes)
- OCPreload habilitado en todos los canales

### Nota sobre los módulos PCA9685 disponibles

Los módulos AZDelivery PCA9685 (I2C, 16 canales, 12 bit) **no son adecuados** para
el control de motores BTS7960 en este proyecto:

- La comunicación I2C a 400 kHz añade **20–40 µs de latencia** por actualización de canal.
  Actualizar 10 canales (5 motores × 2) en cada ciclo de 10 ms consumiría ~400 µs = 4 % del ciclo.
- El PCA9685 tiene su propio oscilador interno de 25 MHz (±1 %); la frecuencia de PWM
  no está sincronizada con los timers del STM32 ni con el ciclo de control.
- Introduce un punto de fallo externo en el bucle de seguridad: si el I2C falla,
  los PWM se "congelan" en su último valor, sin que la state machine de seguridad pueda
  reaccionar inmediatamente.
- Los timers hardware del STM32 tienen latencia **cero** (el CCR se actualiza en
  el siguiente período sin intervención de CPU), son síncronos con el reloj del sistema,
  y están integrados en el mismo dominio de seguridad que el IWDG y la SAFE state machine.

**Conclusión**: usar los timers internos del STM32 es la solución correcta.

---

## 11. Circuito completo de driver de relé con optoacoplador

### Por qué se necesita

Los pines GPIO del STM32 operan a 3,3 V y pueden suministrar máximo 20 mA.
La bobina de un relé mecánico requiere 70–100 mA a 5 V o 12 V.
Además, al desactivar la bobina se genera un pico inductivo de hasta −200 V
que destruiría el GPIO si no hay protección.

### Componentes por relé

| Componente | Valor | Función |
|-----------|-------|---------|
| Resistencia R1 | **330 Ω, 1/4 W** | Limita corriente LED del optoacoplador a ~8 mA |
| Optoacoplador | **PC817** (o equivalente CNY17) | Aísla el GPIO 3,3 V del circuito de la bobina |
| Transistor NPN | **BC547** o **2N2222** | Amplifica la corriente para accionar la bobina |
| Resistencia R2 | **10 kΩ, 1/4 W** | Pull-down en la base del transistor (evita disparo espurio) |
| Diodo flyback | **1N4007** (1 A / 1 000 V) | Elimina el pico inductivo al desactivar el relé |
| Relé mecánico | **SRD-05VDC-SL-C** (5 V, 30 A) | Contactos de potencia |

> Si se usa un **módulo de relé industrial con optoacoplador incorporado** (como HY-M158,
> SRD-05VDC, o los módulos de 1/2/4 canales con entrada "IN" de nivel bajo activo),
> verificar que el módulo sea compatible con señal HIGH a 3,3 V (algunos requieren 5 V
> en la entrada). En ese caso, conectar PC10/PC11/PC12 directamente al pin "IN" del módulo
> y el módulo se encarga del resto. Ver nota de verificación al final de esta sección.

### Esquema completo (circuito discreto, un relé)

```
  STM32 (3,3 V)                    Lado bobina (5 V)        Contactos (24 V / 12 V)
  ─────────────                    ─────────────────        ───────────────────────

  PC10/11/12                5V ──┬──── Bobina relé (+) ──┬──── COM (común)
      │                          │         │              │
     [R1]                     [1N4007]    │              │    (cerrado cuando
    330 Ω                    (flyback)    │              │     relé activo)
      │                   cátodo → 5V    │              │
      │                   ánodo  → ─────►┤ Bobina (-)  NO ──── (normalmente abierto)
      ▼                                  │
   PC817                                 │
  LED (+)──── (ánodo del LED dentro del PC817)           NC ──── (normalmente cerrado)
  LED (-) ─────────────────────────────── GND
      │
  PC817 colector ─── VCC 5V (alimentación relé) ──────── Colector BC547
  PC817 emisor  ─────────────────────────── Base BC547 (vía R2 = 10 kΩ a GND)
                                                             Emisor BC547 ──── GND

```

**Esquema ASCII simplificado y detallado:**

```
    3,3 V STM32
        │
   PC10 (RELAY_MAIN)
        │
       [R1 = 330 Ω]
        │
        ├──── ÁNODO LED (PC817 pin 1)
        │
       GND_STM32 ──── CÁTODO LED (PC817 pin 2)

        PC817 COLECTOR (pin 4) ──── 5V_relé              ← VCC del lado relé
        PC817 EMISOR   (pin 3) ──────────────────────┐  ← salida fototransistor
                                                BASE BC547
                                                         │
                                                    [R2 = 10 kΩ]
                                                         │
                                                        GND

        5V_relé ──────────────────────────────── COLECTOR BC547
                                                         │
                                                   BOBINA relé (−)
                                                         │
                                                   BOBINA relé (+) ───── 5V_relé
                                                         │
                                               ┌── 1N4007 ÁNODO
                                               │
                                    1N4007 CÁTODO ───── 5V_relé
                              (diodo en paralelo con la bobina,
                               polaridad inversa — flyback)
```

> **Nota:** El PC817 funciona en modo seguidor de emisor: el colector (pin 4) va a VCC del
> relé (5V), y el emisor (pin 3) es la salida que controla la base del BC547.
> Cuando el LED se ilumina → fototransistor conduce → emisor sube a ~4.7V → BC547 ON → relé ON.
> Cuando el LED se apaga → fototransistor OFF → emisor es tirado a GND por R2 → BC547 OFF → relé OFF.


**En forma compacta:**

```
STM32 GPIO  ──[330Ω]──► LED(PC817) ──GND
                        │
              5V ─────── COLECTOR(PC817)
                         EMISOR(PC817) ──► BASE(BC547) ──[10kΩ]──GND
                                           COLECTOR(BC547) ──► BOBINA(−)
              5V ──────────────────────── BOBINA(+) ──[1N4007]── BOBINA(−)
              (diodo flyback: cátodo al + de la bobina, ánodo al − de la bobina)
```

### Verificación de módulos de relé industriales (HY-M158, SRD, etc.)

Si se usan módulos prefabricados:

1. **Medir con multímetro** la tensión mínima que activa el relé en el pin "IN".
   - Si activa con 3,3 V: conectar directamente al GPIO del STM32.
   - Si requiere 5 V: interponer un transistor BSS138 (level shifter) para elevar el nivel.

2. **Verificar el diodo flyback**: los módulos industriales suelen incluirlo ya montado.
   Si no está, soldar un 1N4007 entre los terminales de la bobina (ver esquema anterior).

3. **Verificar la lógica de activación**:
   - Módulos con lógica positiva ("active HIGH"): STM32 GPIO HIGH → relé ON. ✅ Compatible con el firmware.
   - Módulos con lógica negativa ("active LOW"): STM32 GPIO HIGH → relé OFF. ❌ Requiere invertir la lógica del firmware o invertir la lógica en el firmware.

### Snubber en los contactos del relé

Los contactos mecánicos del relé generan arcos al abrir/cerrar con cargas inductivas
(motores, cables largos). Para proteger los contactos y reducir interferencias:

```
Contacto COM ──[R = 100 Ω, 1/2 W] ──┬── Contacto NO
                                      │
                              [C = 100 nF / 250 V ac]
                                      │
                                     GND (o directamente entre COM y NO)
```

> Nota: el snubber RC va **entre los terminales del contacto** (COM y NO), no entre COM y GND.

### Resumen de diodos flyback en todo el sistema

| Componente | Diodo | Ubicación |
|-----------|-------|-----------|
| Relé MAIN (bobina 5 V) | 1N4007 | Entre terminales de bobina, cátodo al + |
| Relé TRAC (bobina 5 V) | 1N4007 | Entre terminales de bobina, cátodo al + |
| Relé DIR (bobina 5 V) | 1N4007 | Entre terminales de bobina, cátodo al + |
| BTS7960 VCC lógica (enable signals) | 1N4148 en R_EN/L_EN (opcional) | Serie de protección en señales EN |

---

## 12. Protección anti-reinicios y anti-sobretensiones

### 12.1 Por qué se producen los reinicios del STM32

El STM32G474RE tiene un umbral de **bajo voltaje (BOR)** a ≈2,7 V. Si la tensión de 3,3 V
cae por debajo de este umbral aunque sea 1 µs, el MCU se reinicia. Esto ocurre cuando:

- Los motores conmutan (20 kHz PWM) y generan picos de corriente en la alimentación lógica.
- Los relés cierran (inrush del condensador en el bus de potencia).
- La fuente de 5 V o 3,3 V no tiene suficiente desacoplo local.

**La solución es filtrar agresivamente la alimentación lógica** con condensadores.

### 12.2 Red de desacoplo para la placa Nucleo STM32G474RE

Añadir los siguientes condensadores **externos, soldados en pistas cortas** entre los
pines VDD (3,3 V) y GND de la placa Nucleo o de la protoboard:

```
3,3 V ──┬──[100 nF cerámico X7R]──┬── GND   (respuesta rápida, <1 ns ESL)
         │                         │
         └──[10 µF tántalo / electrolítico]── GND  (bulk, respuesta 1–100 µs)

UBICACIÓN: soldar lo más cerca posible de los pines CN7/CN10 de la Nucleo-64.
Si se usa protoboard, colocar entre los raíles de alimentación adyacentes al MCU.
```

Adicionalmente, en la alimentación 5 V que alimenta a los módulos lógicos (TJA1051, BTS7960 VCC):

```
5 V ──┬──[100 nF cerámico]──┬── GND   (junto a cada CI, una por chip)
       │                     │
       └──[47 µF electrolítico]── GND  (uno en el punto de entrada de la fuente 5 V)
```

### 12.3 Condensadores de bulk en los buses de potencia

Cuando el relé MAIN cierra, el condensador vacío del bus de 24 V genera un inrush
que puede superar 100 A en 10 µs. Esto colapsa momentáneamente la tensión del bus
y, por acoplamiento, puede afectar a la alimentación lógica.

**Solución: condensador de bulk en el bus de potencia:**

```
Relé MAIN NO ──┬──────────────────────────── Bus 24 V (a BTS7960 y relés)
               │
           [1 000 µF / 35 V electrolítico]
               │
              GND potencia (GND común)

UBICACIÓN: lo más cerca posible del relé MAIN, en el bus de 24 V.
NOTA: este condensador también absorbe la energía cinética del motor al frenar
(CEMF regenerativa), reduciendo la sobretensión en el bus.
```

Para el bus de 12 V (motor de dirección):

```
Relé DIR NO ──┬────── Bus 12 V
              │
          [470 µF / 25 V electrolítico]
              │
             GND potencia
```

### 12.4 Ferrita / filtro LC en la alimentación lógica

Para impedir que el ruido de los PWM de 20 kHz se propague por la alimentación:

```
5 V_sucio ──[Ferrita BLM18AG601SN1D o 100 µH]── 5 V_limpio ──► TJA1051
                                                              └── [10 µF] a GND

3.3 V_MCU ────────────────────────────────────── 3.3 V ──► BTS7960 VCC (lógica)
                                                         └── [100 nF] a GND (por módulo)
```

> **Nota:** El TJA1051 requiere 5V. Los módulos BTS7960 (IBT-2) se alimentan desde 3.3V
> (ver §10 para justificación del 74HC244).

> Si no se dispone de ferrita, un fusible reseteable (PTC) de 500 mA en la línea 5 V_limpio
> actúa también como filtro de baja frecuencia y protege contra cortocircuitos.

### 12.5 Protección de pines de entrada contra sobretensión

#### Encoder E6B2-CWZ6C (salida 5 V → pines PA15, PB3, PB4 a 3,3 V)

**⚠️ Conectar 5 V directamente a un pin STM32 lo destruye permanentemente.**

Usar un **divisor resistivo de baja impedancia** para cada canal (A, B, Z):

```
Encoder señal (5 V, OC con pull-up 10 kΩ interno)
    │
   [R1 = 1 kΩ]
    │
    ├─────────────────────────► PA15 (o PB3, PB4)
    │
   [R2 = 2,2 kΩ]
    │
   GND

Tensión en el pin = 5 V × (2,2 / (1 + 2,2)) = 3,44 V   ← dentro del rango 3,3 V ± 10 %
```

> Nota: el resistor R1 (1 kΩ) también protege el pin en caso de que la señal suba por encima
> de VDD por un transitorio. La corriente máxima de clamp del diodo interno del STM32 es 5 mA.
> Con R1 = 1 kΩ y Vtrans = 5 V: I = (5 – 3,3) / 1 kΩ = 1,7 mA < 5 mA ✅

Alternativa más limpia: **BSS138 bidirectional level shifter** (módulo breakout común):

```
3,3V ──────────────────────────────────────── 3,3V_side
                                                    │
                                              [BSS138 MOSFET]
                                                    │
5V ──── Pull-up 10 kΩ ──┬── Encoder señal (5 V)  5V_side
                         │
                    (esto es el nodo LV del BSS138)

PA15 ◄──── LV_side (salida del BSS138)
```

Los módulos de conversión de nivel I2C bidireccionales basados en BSS138 (muy comunes en
Arduino/Nucleo) son exactamente esta topología. Un módulo de 4 canales cubre A, B, Z.

#### Sensor de pedal Hall (5 V → PA3 ADC)

Documentado en §6.5. Usar el divisor 10 kΩ + 6,8 kΩ. El pin PA3 nunca superará 2 V.

#### Sensores inductivos LJ12A3 (6–36 V → pines EXTI STM32)

Documentado en `docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md`. Usar optoacoplador PC817 para
todos los sensores de velocidad de rueda y centrado. **No conectar nunca directamente.**

#### Bus CAN (PA11, PA12 → transceiver TJA1051)

El transceiver CAN actúa como barrera de tensión. Los pines PA11/PA12 solo ven señales
lógicas de 3,3 V generadas por el TJA1051. **No conectar el bus CAN directamente a PA11/PA12.**

### 12.6 Tensiones a verificar con multímetro antes de encender el STM32

Antes de conectar la placa Nucleo, medir con multímetro en modo DC:

| Punto de medición | Tensión esperada | Si está mal |
|-----------------|-----------------|-------------|
| 3,3 V (alimentación Nucleo) | 3,25 – 3,35 V | No encender hasta corregir la fuente |
| 5 V (fuente lógica) | 4,85 – 5,15 V | Ajustar la fuente |
| PA15 con encoder girando (max) | ≤ 3,5 V | Revisar divisor resistivo |
| PB3 con encoder girando (max) | ≤ 3,5 V | Revisar divisor resistivo |
| PA3 con pedal a fondo (max) | ≤ 2,1 V | Revisar divisor 10 kΩ + 6,8 kΩ |
| Salida optoacoplador PA0-PA2, PB15 | 3,3 V en reposo, 0 V al detectar | Revisar PC817 |
| PA11 (CAN RX) con bus activo | 0,5 – 2,5 V oscilante | Normal, señal CAN diferencial convertida |
| GND–GND entre STM32 y BTS7960 | < 0,1 V | Si hay > 0,3 V, revisar cableado de masa |

### 12.7 Secuencia de encendido que evita reinicios

Seguir este orden estrictamente:

```
1. Encender fuente 3,3 V / 5 V (lógica) → esperar 2 s → verificar tensiones con multímetro
2. Conectar STM32 Nucleo a la alimentación → verificar que arranca (LED verde on)
3. Conectar ESP32 → verificar arranque
4. Verificar heartbeat CAN (monitor serie o osciloscopio en CANH/CANL)
5. Conectar sensores I2C (INA226, TCA9548A) → verificar en serial que se detectan
6. Conectar sensores OneWire (DS18B20) → verificar lecturas de temperatura
7. Conectar encoder de dirección → girar manualmente y verificar que el contador cambia
8. Conectar sensores de velocidad de rueda (vía optoacoplador) → girar rueda a mano
9. Conectar pedal y verificar lectura ADC (0–100 %)
   ── HASTA AQUÍ: potencia lógica solamente ──
10. Conectar fuente 24 V SIN conectar motores todavía → verificar tensión con multímetro
11. Conectar fuente 12 V SIN conectar motor dirección
12. Con el firmware en estado STANDBY (ESP32 conectado, sistema en espera),
    dar comando de arranque desde el ESP32 → los relés deben activar en secuencia:
    MAIN (50 ms) → TRAC (20 ms) → DIR
13. Verificar tensión en bus 24 V y 12 V tras cierre de relés
14. Conectar motores de tracción y verificar giro suave al dar acelerador
15. Conectar motor de dirección y verificar centrado automático al arrancar
```

### 12.8 Resumen de condensadores, diodos y resistencias de protección (lista de compra)

| Qty | Componente | Valor | Uso |
|-----|-----------|-------|-----|
| 2 | Condensador cerámico | 100 nF / 16 V X7R | Desacoplo 3,3 V Nucleo (VDD pins) |
| 1 | Condensador tántalo o electrolítico | 10 µF / 10 V | Bulk 3,3 V Nucleo |
| 5 | Condensador cerámico | 100 nF / 50 V | Desacoplo VCC de cada BTS7960 (lógica) |
| 5 | Condensador cerámico | 100 nF / 50 V | Snubber en terminales de motor |
| 1 | Condensador electrolítico | 1 000 µF / 35 V | Bulk bus 24 V |
| 1 | Condensador electrolítico | 470 µF / 25 V | Bulk bus 12 V |
| 1 | Condensador electrolítico | 47 µF / 10 V | Bulk 5 V lógica |
| 5 | Diodo 1N4007 | 1 A / 1 000 V | Flyback bobinas de relé (uno por relé, incluye LED relays) |
| 5 | Diodo 1N5408 | 3 A / 1 000 V | Anti-CEMF en terminales de motor |
| 5 | Resistencia | 330 Ω / 1/4 W | Serie LED optoacoplador relé (si circuito discreto, 5 relés) |
| 5 | Resistencia | 10 kΩ / 1/4 W | Pull-down base transistor relé (si circuito discreto, 5 relés) |
| 5 | Resistencia | 100 Ω / 1/2 W | Snubber RC contactos relé (R del RC, 5 relés) |
| 5 | Condensador cerámico | 100 nF / 250 V ac | Snubber RC contactos relé (C del RC, 5 relés) |
| 3 | Resistencia | 1 kΩ / 1/4 W | R1 divisor encoder (A, B, Z) 5 V→3,3 V |
| 3 | Resistencia | 2,2 kΩ / 1/4 W | R2 divisor encoder (A, B, Z) a GND |

> **Nota:** Los CI SN74HC08N (AND) y SN74HC04N (NOT) han sido **eliminados del BOM**.
> El firmware genera RPWM/LPWM directamente desde los timers. Ver [Sección 10](#10-conexión-directa-bts7960--rpwmlpwm-desde-stm32-sin-lógica-externa).

---

> **Este documento ha sido actualizado con todos los circuitos de protección y los valores
> de componentes necesarios para una instalación segura.**
> Revisar también: `docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md` (optoacopladores de sensores),
> `docs/CONEXIONES_COMPLETAS.md` (guía cable por cable completa).
