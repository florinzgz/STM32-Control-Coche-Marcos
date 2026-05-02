# HARDWARE SPECIFICATION - Sistema de Control STM32G474RE

## Arquitectura General del Sistema

```
┌─────────────────────────────────────────────────────────────────┐
│                         STM32G474RE                              │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  TIM1 (20kHz) → PWM_FL (CH1/2), PWM_FR (CH3/4)         │   │
│  │  TIM8 (20kHz) → PWM_RL (CH1/2), PWM_RR (CH3/4)         │   │
│  │  TIM3 (20kHz) → PWM_STEER (CH1/2)                       │   │
│  │  TIM2 (Quadrature) → ENC_A, ENC_B                        │   │
│  │  GPIO → EN_xx, RELAY_xx                                  │   │
│  │  ADC1 → PEDAL                                            │   │
│  │  FDCAN1 → CAN Bus (500 kbps)                            │   │
│  │  I2C1 → TCA9548A → INA226 (×6)                          │   │
│  │  OneWire → DS18B20 (×5)                                  │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
         ↓ PWM/DIR/EN              ↓ CAN              ↓ Sensores
    ┌────────────┐          ┌──────────────┐    ┌─────────────────┐
    │ BTS7960 ×5 │          │   ESP32-S3   │    │ INA226 + DS18B20│
    └────────────┘          └──────────────┘    └─────────────────┘
         ↓                         ↓                      ↓
    ┌────────────┐          ┌──────────────┐    ┌─────────────────┐
    │  Motores   │          │  HMI/Display │    │  Monitorización │
    │  (4+1)     │          └──────────────┘    └─────────────────┘
    └────────────┘
```

## BTS7960 H-Bridge Motor Drivers

### Especificaciones Técnicas

**Modelo**: BTS7960 (Infineon)
**Cantidad**: 5 unidades
- 4× Motores de tracción (FL, FR, RL, RR)
- 1× Motor de dirección

**Características Principales:**
- Corriente continua: 43A por canal
- Corriente pico: 100A (≤5s)
- Tensión operación: 5.5V - 27V
- Frecuencia PWM recomendada: 20-25 kHz
- Protección térmica integrada: 150°C
- Protección cortocircuito
- Diagnóstico de fallos
- Eficiencia: >95% @ 20kHz

### Configuración de Pines BTS7960

Cada BTS7960 tiene:
```
┌─────────────────────────────────┐
│         BTS7960 Module          │
├─────────────────────────────────┤
│ VCC    → 3.3V (lógica control) ⚠️  │
│ GND    → GND común               │
│ RPWM   → PWM directo (forward)   │
│ LPWM   → PWM inverso (reverse)   │
│ R_EN   → Enable derecho (HIGH)   │
│ L_EN   → Enable izquierdo (HIGH) │
│ R_IS   → Current sense derecho   │
│ L_IS   → Current sense izquierdo │
│ VMOT   → 24V/12V (motor)         │
│ M+     → Terminal + motor        │
│ M-     → Terminal - motor        │
└─────────────────────────────────┘

> ⚠️ **IMPORTANTE — VCC del IBT-2 a 3.3 V, NO 5 V:**
> El módulo IBT-2 incluye un buffer 74HC244 cuyo V_IH(min) = 0.7 × VCC.
> A VCC = 5 V → V_IH(min) = 3.5 V, y las señales de 3.3 V del STM32 quedan **por debajo** del umbral (no fiable).
> A VCC = 3.3 V → V_IH(min) = 2.31 V, y las señales de 3.3 V del STM32 están **por encima** del umbral (correcto).
> Por ello, este diseño alimenta VCC del IBT-2 desde el rail de 3.3 V del STM32 (ver `project_config.h`).
```

### Esquema de Conexión STM32 → BTS7960

> ⚠️ **SECCIÓN OBSOLETA** — La arquitectura actual usa RPWM/LPWM directo (no DIR+PWM).
> Los pines correctos están en `Core/Inc/project_config.h`.

#### Motor Frontal Izquierdo (FL)
```
STM32              BTS7960_FL
PA8  (TIM1_CH1) → RPWM
PA9  (TIM1_CH2) → LPWM
PC5  (EN_FL)    → R_EN + L_EN
```

#### Motor Frontal Derecho (FR)
```
STM32              BTS7960_FR
PA10 (TIM1_CH3) → RPWM
PC3  (TIM1_CH4) → LPWM
PC0  (EN_FR)    → R_EN + L_EN
```

#### Motor Trasero Izquierdo (RL)
```
STM32              BTS7960_RL
PC6  (TIM8_CH1) → RPWM
PC7  (TIM8_CH2) → LPWM
PC1  (EN_RL)    → R_EN + L_EN
```

#### Motor Trasero Derecho (RR)
```
STM32              BTS7960_RR
PC8  (TIM8_CH3) → RPWM
PC9  (TIM8_CH4) → LPWM
PC13 (EN_RR)    → R_EN + L_EN
```

#### Motor de Dirección (STEER)
```
STM32              BTS7960_STEER
PA6  (TIM3_CH1) → RPWM
PA7  (TIM3_CH2) → LPWM
PC4  (EN_STEER) → R_EN + L_EN
```

### Lógica de Control PWM con Dirección

```c
// Pseudocódigo de control
if (reverse) {
    RPWM = 0;        // Canal forward desactivado
    LPWM = pwm_value; // Canal reverse con PWM
    DIR_PIN = 1;     // Indicador dirección
} else {
    RPWM = pwm_value; // Canal forward con PWM
    LPWM = 0;         // Canal reverse desactivado
    DIR_PIN = 0;      // Indicador dirección
}

// Habilitación global
R_EN = EN_PIN;
L_EN = EN_PIN;
```

### Protecciones Implementadas en Hardware

1. **Módulo 4-ch opto relé 12V (SRD-12VDC-SL-C)** — driver etapa 1 relés de potencia
   - Módulo: 4 canales con optoacopladores, relés SRD-**12**VDC-SL-C (bobina **12V** DC)
   - Alimentación módulo: **12V** DC (VCC / GND)
   - Trigger: High/Low level (compatible 3.3V STM32)
   - Contactos: 10A 250VAC / 10A 30VDC por canal
   - Asignación de canales:
     - CH1 → PC11 STM32 — driver relé potencia TRAC (24V, 50A)
     - CH2 → PC12 STM32 — driver relé potencia DIR (12V, 20A)
     - CH3 → libre · CH4 → libre
   - PC10 **no se conecta** (GPIO libre, `INPUT_PULLDOWN`)

   **Módulo 4-ch opto relé 5V (SRD-05VDC-SL-C)** — corte LED strips + relé audio
   - Módulo: 4 canales con optoacopladores, relés SRD-**05**VDC-SL-C (bobina **5V** DC)
   - Alimentación módulo: **5V** DC (VCC / GND)
   - Trigger: High/Low level (compatible 3.3V STM32 y ESP32)
   - Contactos: 10A 250VAC / 10A 30VDC por canal
   - Asignación de canales:
     - CH1 → PB10 STM32 — corte alimentación 5V tira LED frontal (28 LEDs)
     - CH2 → PB11 STM32 — corte alimentación 5V tira LED trasera (16 LEDs)
     - **CH3 → GPIO11 ESP32 — relé audio DFPlayer (activo LOW)**
     - CH4 → libre (reserva)

2. **Snubber Circuits** (protección contra picos)
   - Diodo: 1N4148 o similar
   - Capacitor: 100nF/50V cerámico
   - Resistencia: 100Ω 1/4W

3. **Current Sensing**
   - Resistencia shunt: 0.0015Ω (1.5mΩ) por motor, 0.00075Ω (0.75mΩ) batería
   - Amplificador: INA226 (medición precisa)
   - Rango: ±20A tracción, ±10A dirección

## Relés de Potencia

### Especificación Relés

**Módulo driver etapa 1 (TRAC/DIR):** SRD-12VDC-SL-C, 12V, optoacoplado, 2 canales usados (CH1–CH2)
**Módulo LED + audio:** SRD-05VDC-SL-C, 5V, optoacoplado, 3 canales usados (CH1–CH3)
**Relé de potencia:** bobina 12V DC, accionado por contacto del módulo 4-ch 12V

| Función | Pin | Corriente | Protección | Módulo físico / Canal |
|---------|-----|-----------|------------|-----------------------|
| RELAY_TRAC | PC11 (STM32) | 40A | Fusible 50A | Módulo 4-ch **12V** CH1 |
| RELAY_DIR  | PC12 (STM32) | 15A | Fusible 20A | Módulo 4-ch **12V** CH2 |
| RELAY_LED_F | PB10 (STM32) | 3A LED 5V | Fusible 5A | Módulo 4-ch **5V** CH1 |
| RELAY_LED_R | PB11 (STM32) | 2A LED 5V | Fusible 5A | Módulo 4-ch **5V** CH2 |
| **RELAY_AUDIO** | **GPIO11 (ESP32)** | señal audio | — | Módulo 4-ch **5V** CH3 (activo LOW) |

> **PC10 está DISPONIBLE** — GPIO libre, no conectado (`INPUT_PULLDOWN`).

**Arquitectura de dos etapas (para TRAC y DIR):**
- **Etapa 1 (módulo 4-ch 12V):** CH1 y CH2. GPIO del STM32 activa las entradas IN (3.3V compatible).
- **Etapa 2 (relés de potencia):** Relés de alta corriente con bobina 12V DC. Sus bobinas están alimentadas a través de los contactos de etapa 1.

**Módulo 4-ch 5V (LED + audio):**
- CH1/CH2 cortan la alimentación 5V de las tiras LED (corriente ≤ 3A, sin etapa 2).
- CH3 conmuta la señal del altavoz entre radio y DFPlayer (corriente de señal, sin etapa 2). Activo LOW, controlado por ESP32 GPIO11.

**Características módulo 4-ch 12V (etapa 1 TRAC/DIR):**
- Bobina módulo: **12V** DC
- Contactos: 10A @ 30V DC — suficiente para bobinas de relés de potencia
- Optoacopladores integrados, flyback de bobina incluido en el módulo

**Características módulo 4-ch 5V (LED + audio):**
- Bobina módulo: **5V** DC
- Contactos: 10A @ 30V DC / 10A @ 250V AC
- Trigger: High/Low level seleccionable, compatible 3.3V STM32 y ESP32
- Optoacopladores integrados en placa

**Características relés de potencia (etapa 2 — solo TRAC/DIR):**
- Bobina: 12V DC
- Contactos: según carga (40A TRAC, 15A DIR)

### Secuencia de Activación Segura

```
Startup:
1. RELAY_TRAC = ON (tracción habilitada) 
2. Esperar 50ms (estabilización)
3. RELAY_DIR = ON (dirección habilitada)
4. Sistema listo

Shutdown:
1. RELAY_DIR = OFF (desactivar dirección primero)
2. RELAY_TRAC = OFF (corte tracción)
```

### Esquema Módulo 4-ch Opto Relé 12V (SRD-12VDC-SL-C) — canales TRAC/DIR

```
STM32 (3.3V)               Módulo 4-ch opto relé (12V)       Relé de potencia (bobina 12V)
────────────               ───────────────────────────       ─────────────────────────────

PC11 ──────────► IN1       VCC ◄── 12V                    ┌── Bobina TRAC (+) 12V
PC12 ──────────► IN2       GND ◄── GND                    │
                                                           │   (contacto 10A cierra cuando
               Optoacoplador                               │    relé del módulo activo)
               interno del    ──► Relé SRD-12VDC ──► COM ──┤
               módulo              (contacto 10A)    NO ────┴── Bobina TRAC (−) → GND
                                                       ↑
                                                  [1N4007] flyback externo

CH3/CH4: libres (reserva)
```

**Cadena completa canal TRAC (CH1):**
```
STM32 PC11 (3.3V HIGH)
    │
    ▼
Módulo opto relé CH1 → Optoacoplador LED → relé SRD-12VDC cierra
    │
    ▼ (contacto 10A del módulo pasa 12V a la bobina del relé de potencia)
Relé de potencia cierra → 24V llega a BTS7960 tracción
```

### Esquema Módulo 4-ch Opto Relé 5V (SRD-05VDC-SL-C) — LED strips + AUDIO

```
STM32/ESP32 (3.3V)         Módulo 4-ch opto relé (5V)
──────────────────         ──────────────────────────

PB10 ──────────► IN1       VCC ◄── 5V                    COM1 ──► Alimentación 5V tira LED_F
PB11 ──────────► IN2       GND ◄── GND                   COM2 ──► Alimentación 5V tira LED_R
GPIO11 (ESP32) ─► IN3                                     COM3 ──► Altavoz +
                                                          NO3  ──► DFPlayer SPKR
               Optoacoplador                              NC3  ──► Radio (normalmente conectado)
               interno del    ──► Relé SRD-05VDC
               módulo              (contacto 10A)

CH4 (IN4): libre
```

**Canal audio (CH3) — activo LOW, sin etapa 2:**
```
ESP32 GPIO11 (LOW = relé ON, activo LOW)
    │
    ▼
Módulo opto relé CH3 → contacto COM—NO cierra
    │
COM → Altavoz +
NO  → DFPlayer SPKR
NC  → Radio (normalmente conectado)
```

## Sensores de Temperatura - DS18B20

### Configuración OneWire Bus

**Bus único compartido**: PB0
**Sensores conectados**: 5

| Sensor | Ubicación | ROM Address | Temp Max |
|--------|-----------|-------------|----------|
| DS18B20_1 | Motor FL | (Auto-detectado) | 130°C |
| DS18B20_2 | Motor FR | (Auto-detectado) | 130°C |
| DS18B20_3 | Motor RL | (Auto-detectado) | 130°C |
| DS18B20_4 | Motor RR | (Auto-detectado) | 130°C |
| DS18B20_5 | Volante/Dir | (Auto-detectado) | 100°C |

**Especificaciones:**
- Resolución: 12-bit (0.0625°C)
- Rango: -55°C a +125°C
- Precisión: ±0.5°C (-10°C a +85°C)
- Tiempo conversión: 750ms @ 12-bit
- Identificación: 64-bit ROM único

**Circuito:**
```
STM32_PB0 ──── 4.7kΩ ──── VCC(3.3V)
         │
         ├────── DS18B20_1 (DQ pin)
         ├────── DS18B20_2 (DQ pin)
         ├────── DS18B20_3 (DQ pin)
         ├────── DS18B20_4 (DQ pin)
         └────── DS18B20_5 (DQ pin)

Cada DS18B20:
  VDD → 3.3V
  GND → GND
  DQ  → Bus común
```

## Sensores de Corriente - INA226

### Configuración I2C con Multiplexor TCA9548A

**Bus I2C1**: PB6 (SCL), PB7 (SDA)  
**Multiplexor**: TCA9548A @ 0x70  
**Velocidad**: 400 kHz (Fast Mode)

**Distribución canales:**
```
TCA9548A
├── Canal 0: INA226_FL    (0x40) → Motor FL  (shunt 1.5 mΩ)
├── Canal 1: INA226_FR    (0x40) → Motor FR  (shunt 1.5 mΩ)
├── Canal 2: INA226_RL    (0x40) → Motor RL  (shunt 1.5 mΩ)
├── Canal 3: INA226_RR    (0x40) → Motor RR  (shunt 1.5 mΩ)
├── Canal 4: INA226_MAIN  (0x40) → Batería 24V (shunt 0.75 mΩ)
└── Canal 5: INA226_STEER (0x40) → Motor Dir (shunt 1.5 mΩ)
```

**Especificaciones INA226:**
- VCC (lógica): 3.3 V (rango 2.7–5.5 V)
- Rango tensión bus: 0-36V
- Resolución tensión: 1.25mV/bit
- Resolución corriente: Configurable según shunt
- Velocidad conversión: 140μs - 8.244ms
- Precisión: ±0.1%

**Configuración Shunt (según BOM real — ver `project_config.h`):**
- Motores (canales 0–3, 5): **0.0015 Ω (1.5 mΩ)** — shunt 50 A / 75 mV
- Batería (canal 4): **0.00075 Ω (0.75 mΩ)** — shunt 100 A / 75 mV
- Potencia mínima shunt: 3 W
- Rango entrada shunt INA226: ±81.92 mV (máximo)

**Cálculo Corriente:**
```
Current (A) = Shunt_Voltage (mV) / Shunt_Resistance (mΩ)
Motor:   Current = Shunt_mV / 1.5
Batería: Current = Shunt_mV / 0.75
```

## Encoder de Dirección - E6B2-CWZ6C

### Especificaciones

- **Modelo**: OMRON E6B2-CWZ6C
- **Resolución**: 1200 PPR (pulses per revolution)
- **Modo**: Quadrature (incremental)
- **Counts/rev**: 4800 (1200 × 4 en quadrature)
- **Tensión**: 5V DC
- **Salidas**: Open-collector NPN
- **Frecuencia máx**: 100 kHz
- **Pulso Z**: 1 pulso por revolución (índice)

### Conexión a STM32

```
E6B2-CWZ6C          STM32G474RE
─────────────       ─────────────
  +5V         →     5V (externa)
  GND         →     GND
  A (Phase A) →     PA15 (TIM2_CH1)
  B (Phase B) →     PB3  (TIM2_CH2)
  Z (Index)   →     PB4  (EXTI4 GPIO)
```

**Circuito Pull-up:**
```
Cada línea (A, B, Z):
  Open-collector → 10kΩ → 3.3V
                 → Pin STM32
```

### Configuración TIM2 (Quadrature Encoder Mode)

```c
// TIM2 en modo encoder
TIM2->SMCR |= TIM_ENCODERMODE_TI12;  // Encoder mode 3
TIM2->ARR = 4799;  // Auto-reload: 4800 counts - 1
TIM2->CNT = 2400;  // Centro en posición media

// Resolución angular
float angle_deg = (TIM2->CNT - 2400) * 0.075f;
// 0.075° = 360° / 4800 counts
```

## Sensor de Pedal - A1324LUA-T

### Especificación

- **Tipo**: Hall Effect Linear
- **Tensión alimentación**: 4.5V - 5.5V
- **Salida**: Ratiometric (0.5V - 4.5V)
- **Sensibilidad**: 5mV/Gauss
- **Linealidad**: ±1.5%

### Acondicionamiento de Señal

**Divisor de tensión** (5V → 3.3V — ver `project_config.h`):
```
5V Output ──┬── 10kΩ ──┬── PA3 (ADC1_IN4)
            │          │
            │         6.8kΩ
            │          │
           GND ────────┴── GND

Vout_max = 5V × (6.8kΩ / (10kΩ + 6.8kΩ)) = 2.02V
Rango pedal: 0.5V–4.5V → 0.20V–1.82V en PA3
```

> **Nota:** El divisor 10 kΩ / 6.8 kΩ produce un rango más reducido (0–2.02 V) que el
> fondo de escala del ADC (3.3 V), pero proporciona un margen de seguridad amplio:
> incluso si la salida del sensor llega a 5.5 V, la tensión en PA3 sería 2.23 V < 3.3 V.

**Conversión ADC:**
```c
// 12-bit ADC (0-4095)
uint16_t adc_value = ADC1->DR;
float voltage = (adc_value / 4095.0f) * 3.3f;

// Convertir a posición pedal (0-100%)
float pedal_min = 0.33f;  // 10% de 3.3V
float pedal_max = 2.97f;  // 90% de 3.3V
float pedal_pct = ((voltage - pedal_min) / 
                   (pedal_max - pedal_min)) * 100.0f;

// Clamp a rango válido
pedal_pct = fminf(fmaxf(pedal_pct, 0.0f), 100.0f);
```

## Sensores de Velocidad de Rueda - LJ12A3-4-Z/BX

### Especificación

- **Tipo**: Inductivo NPN NO (Normally Open)
- **Distancia detección**: 4mm
- **Tensión**: 6-36V DC
- **Salida**: NPN open-collector
- **Corriente salida**: 200mA max
- **Frecuencia respuesta**: 1 kHz

### Configuración por Rueda

Cada rueda tiene:
- **6 tornillos** igualmente espaciados (360° / 6 = 60°)
- **Frecuencia máxima**: ~100 Hz @ 60 km/h

| Rueda | Pin GPIO | EXTI | Tornillos | Diámetro |
|-------|----------|------|-----------|----------|
| FL    | PA0      | EXTI0  | 6 | 50cm |
| FR    | PA1      | EXTI1  | 6 | 50cm |
| RL    | PA2      | EXTI2  | 6 | 50cm |
| RR    | PB15     | EXTI15 | 6 | 50cm |

**Circuito por sensor:**
```
VCC(12V) ──┬── LJ12A3-4-Z/BX
           │   (Brown: +)
           │   (Blue: -)
           │   (Black: Signal)
           │
     Signal ── 10kΩ ── 3.3V
           │
          GPIO ── STM32
```

### Cálculo de Velocidad

```c
// Constantes
#define WHEEL_DIAMETER_M  0.50f  // 50cm diámetro
#define SCREWS_PER_REV    6      // 6 tornillos
#define PI                3.14159265f

// Perímetro rueda
float perimeter = PI * WHEEL_DIAMETER_M;  // 1.57m

// Distancia por pulso
float dist_per_pulse = perimeter / SCREWS_PER_REV;  // 0.262m

// Velocidad basada en tiempo entre pulsos
float time_between_pulses_s = 0.050f;  // ejemplo: 50ms
float speed_ms = dist_per_pulse / time_between_pulses_s;  // 5.24 m/s
float speed_kmh = speed_ms * 3.6f;  // 18.9 km/h
```

## Bus CAN - FDCAN1

### Hardware CAN

**Transceiver**: TJA1051T/3 (NXP) — VCC = 5 V, VIO = 3.3 V (ver `ESP32_STM32_CAN_CONNECTION.md`)
**Terminación**: 120Ω en cada extremo

```
STM32_PA11 (RX) ──┐
                  │    ┌───────────┐
STM32_PA12 (TX) ──┼────┤ TJA1051T/3├──── CAN_H ── 120Ω ── CAN_L
                  │    └───────────┘
                 GND                    │
                                     Bus CAN
```

**Configuración:**
- Velocidad: 500 kbps
- Bit timing: Tq = 1/(500kHz × 16) = 125ns
- Propagation: 8 Tq
- Phase Seg1: 3 Tq
- Phase Seg2: 4 Tq

### Protocolo de Comunicación

Ver documento separado: `PROTOCOLO_CAN.md`

## Lista de Materiales (BOM)

| Componente | Cantidad | Especificación | Notas |
|------------|----------|----------------|-------|
| STM32G474RE | 1 | LQFP64, 170MHz | Microcontrolador principal |
| BTS7960 (IBT-2) | 5 | 43A H-Bridge | Drivers de motor — VCC a 3.3 V |
| TCA9548A | 1 | I2C Multiplexor | 8 canales |
| INA226 | 6 | I2C Current/Voltage | Medición corriente |
| DS18B20 | 5 | OneWire Temp | Sensores temperatura |
| E6B2-CWZ6C | 1 | 1200 PPR Encoder | Encoder dirección |
| LJ12A3-4-Z/BX | 4 | 4mm Inductive | Sensores velocidad |
| A1324LUA-T | 1 | Hall Effect Linear | Sensor pedal |
| Módulo 4-ch opto relé 12V | 1 | **SRD-12VDC-SL-C, 12V, 4 canales**; CH1=PC11(TRAC), CH2=PC12(DIR), CH3/CH4=libres | Driver etapa 1 relés potencia |
| Módulo 4-ch opto relé 5V | 1 | **SRD-05VDC-SL-C, 5V, 4 canales**; CH1=PB10(LED_F), CH2=PB11(LED_R), CH3=GPIO11(audio), CH4=libre | Corte alimentación LED strips + relé audio |
| Relé potencia (bobina 12V) | 2 | Alta corriente (≥50A TRAC, ≥20A DIR) | Accionados por CH1/CH2 del módulo 4-ch |
| TJA1051T/3 | 2 | CAN Transceiver (NXP) | VCC=5V, VIO=3.3V — uno por nodo |
| B0505S-1W | 1 | DC-DC aislado 5V→5V, 1W, 1kV | Alimentación lado aislado bus CAN (Opción A ADuM1201+DC-DC) |

| Shunt 1.5 mΩ | 5 | 50A/75mV, 3W | INA226 motores (ch 0–3, 5) |
| Shunt 0.75 mΩ | 1 | 100A/75mV, 3W | INA226 batería (ch 4) |
| Resistor 10kΩ | 20 | 1/4W | Pull-ups |
| Resistor 120Ω | 2 | 1/4W | Terminación CAN |
| Capacitor 100nF | 30 | Cerámico X7R | Desacoplo |
| Capacitor 1000μF | 1 | 25V Electrolítico | Bulk capacitor |
| Diodo 1N4007 | 4 | 1A, 1000V | Flyback relés potencia (TRAC, DIR) y LED (LED_F, LED_R) |

---

**Documento creado**: 2026-02-01  
**Versión**: 1.0  
**Autor**: Sistema de Control Coche Marcos
