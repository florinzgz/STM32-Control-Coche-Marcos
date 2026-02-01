# HARDWARE SPECIFICATION - Sistema de Control STM32G474RE

## Arquitectura General del Sistema

```
┌─────────────────────────────────────────────────────────────────┐
│                         STM32G474RE                              │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  TIM1 (20kHz) → PWM_FL, PWM_FR, PWM_RL, PWM_RR          │   │
│  │  TIM8 (20kHz) → PWM_STEER                                │   │
│  │  TIM2 (Quadrature) → ENC_A, ENC_B                        │   │
│  │  GPIO → DIR_xx, EN_xx, RELAY_xx                          │   │
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
│ VCC    → 5V (lógica control)    │
│ GND    → GND común               │
│ RPWM   → PWM directo (forward)   │
│ LPWM   → PWM inverso (reverse)   │
│ R_EN   → Enable derecho (HIGH)   │
│ L_EN   → Enable izquierdo (HIGH) │
│ R_IS   → Current sense derecho   │
│ L_IS   → Current sense izquierdo │
│ VMOT   → 12V (alimentación motor)│
│ M+     → Terminal + motor        │
│ M-     → Terminal - motor        │
└─────────────────────────────────┘
```

### Esquema de Conexión STM32 → BTS7960

#### Motor Frontal Izquierdo (FL)
```
STM32           BTS7960_FL
PA8 (PWM_FL) → RPWM (cuando DIR_FL=0)
PA8 (PWM_FL) → LPWM (cuando DIR_FL=1)
PC0 (DIR_FL) → Control lógica dirección
PC5 (EN_FL)  → R_EN + L_EN (ambos habilitados juntos)
```

#### Motor Frontal Derecho (FR)
```
STM32           BTS7960_FR
PA9 (PWM_FR) → RPWM/LPWM según DIR_FR
PC1 (DIR_FR) → Control dirección
PC6 (EN_FR)  → R_EN + L_EN
```

#### Motor Trasero Izquierdo (RL)
```
STM32           BTS7960_RL
PA10 (PWM_RL) → RPWM/LPWM según DIR_RL
PC2 (DIR_RL)  → Control dirección
PC7 (EN_RL)   → R_EN + L_EN
```

#### Motor Trasero Derecho (RR)
```
STM32           BTS7960_RR
PA11 (PWM_RR) → RPWM/LPWM según DIR_RR
PC3 (DIR_RR)  → Control dirección
PC8 (EN_RR)   → R_EN + L_EN
```

#### Motor de Dirección (STEER)
```
STM32             BTS7960_STEER
PC8 (PWM_STEER) → RPWM/LPWM según DIR_STEER
PC4 (DIR_STEER) → Control dirección
PC9 (EN_STEER)  → R_EN + L_EN
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

1. **Optoacopladores HY-M158** (aislamiento óptico)
   - Modelo: HY-M158 o equivalente
   - Aislamiento: 2500V RMS
   - Tiempo respuesta: 10μs
   - Conexión: Entre GPIO STM32 y BTS7960

2. **Snubber Circuits** (protección contra picos)
   - Diodo: 1N4148 o similar
   - Capacitor: 100nF/50V cerámico
   - Resistencia: 100Ω 1/4W

3. **Current Sensing**
   - Resistencia shunt: 0.002Ω (2mΩ) por motor
   - Amplificador: INA226 (medición precisa)
   - Rango: ±20A tracción, ±10A dirección

## Relés de Potencia

### Especificación Relés

**Modelo**: SANYOU SRD-05VDC-SL-C o equivalente

| Función | Pin | Corriente | Protección | Optoacoplador |
|---------|-----|-----------|------------|---------------|
| RELAY_MAIN | PC10 | 50A | Fusible 60A | HY-M158 |
| RELAY_TRAC | PC11 | 40A | Fusible 50A | HY-M158 |
| RELAY_DIR  | PC12 | 15A | Fusible 20A | HY-M158 |

**Características:**
- Bobina: 5V DC, 70mA
- Contactos: 30A @ 12V DC
- Vida útil: 100,000 operaciones
- Tiempo activación: 10ms
- Diodo volante: 1N4007 (protección bobina)

### Secuencia de Activación Segura

```
Startup:
1. RELAY_MAIN = ON (alimentación general)
2. Esperar 500ms (estabilización)
3. RELAY_DIR = ON (dirección habilitada)
4. Esperar 200ms
5. RELAY_TRAC = ON (tracción habilitada)
6. Sistema listo

Shutdown:
1. RELAY_TRAC = OFF (desactivar tracción primero)
2. Esperar 100ms
3. RELAY_DIR = OFF (desactivar dirección)
4. Esperar 100ms
5. RELAY_MAIN = OFF (corte total)
```

### Esquema Optoacoplador HY-M158

```
STM32 (3.3V)                     Relé (5V/12V)
    GPIO ──┬── 330Ω ──┐                  ┌──── + Bobina
           │          │                  │
           │    ┌─────┴─────┐           │
           │    │  LED (IF)  │          │
           │    │            │          │
           │    │ HY-M158    │      Transistor
           │    │            │    (NPN/Darlington)
           │    │ Photodiode │          │
           │    └─────┬─────┘           │
           │          │                  │
    GND ───┴──────────┴───────────────── - Bobina
                                          │
                                      Diodo 1N4007
                                      (Flyback)
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

**Bus I2C1**: PB8 (SCL), PB9 (SDA)  
**Multiplexor**: TCA9548A @ 0x70  
**Velocidad**: 400 kHz (Fast Mode)

**Distribución canales:**
```
TCA9548A
├── Canal 0: INA226_FL    (0x40) → Motor FL  
├── Canal 1: INA226_FR    (0x40) → Motor FR  
├── Canal 2: INA226_RL    (0x40) → Motor RL  
├── Canal 3: INA226_RR    (0x40) → Motor RR  
├── Canal 4: INA226_STEER (0x40) → Motor Dir
└── Canal 5: INA226_MAIN  (0x40) → Principal
```

**Especificaciones INA226:**
- Rango tensión: 0-36V
- Resolución tensión: 1.25mV/bit
- Resolución corriente: Configurable según shunt
- Velocidad conversión: 140μs - 8.244ms
- Precisión: ±0.1%

**Configuración Shunt:**
- Resistencia: 0.002Ω (2mΩ)
- Potencia: 3W
- Rango: ±82mV (máximo INA226)
- Corriente máxima: 41A

**Cálculo Corriente:**
```
Current (A) = Shunt_Voltage (mV) / Shunt_Resistance (mΩ)
           = Shunt_Voltage (mV) / 2
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

**Divisor de tensión** (5V → 3.3V):
```
5V Output ──┬── 1kΩ ──┬── PA3 (ADC1_IN4)
            │         │
           │         2kΩ
           │         │
          GND ───────┴── GND

Vout = 5V × (2kΩ / (1kΩ + 2kΩ)) = 3.33V (máximo)
```

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
| FL    | PB5      | EXTI5 | 6 | 50cm |
| FR    | PB6      | EXTI6 | 6 | 50cm |
| RL    | PB7      | EXTI7 | 6 | 50cm |
| RR    | PB8      | EXTI8 | 6 | 50cm |

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

**Transceiver**: TJA1050 o MCP2551
**Terminación**: 120Ω en cada extremo

```
STM32_PA11 (RX) ──┐
                  │    ┌───────────┐
STM32_PA12 (TX) ──┼────┤ TJA1050  ├──── CAN_H ── 120Ω ── CAN_L
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
| BTS7960 | 5 | 43A H-Bridge | Drivers de motor |
| TCA9548A | 1 | I2C Multiplexor | 8 canales |
| INA226 | 6 | I2C Current/Voltage | Medición corriente |
| DS18B20 | 5 | OneWire Temp | Sensores temperatura |
| E6B2-CWZ6C | 1 | 1200 PPR Encoder | Encoder dirección |
| LJ12A3-4-Z/BX | 4 | 4mm Inductive | Sensores velocidad |
| A1324LUA-T | 1 | Hall Effect Linear | Sensor pedal |
| HY-M158 | 8 | Optoacoplador | Aislamiento |
| TJA1050 | 1 | CAN Transceiver | Bus CAN |
| Relé SRD-05VDC | 3 | 30A, 5V coil | Relés potencia |
| Resistor 0.002Ω | 6 | 2mΩ, 3W | Shunt corriente |
| Resistor 10kΩ | 20 | 1/4W | Pull-ups |
| Resistor 120Ω | 2 | 1/4W | Terminación CAN |
| Capacitor 100nF | 30 | Cerámico X7R | Desacoplo |
| Capacitor 1000μF | 1 | 25V Electrolítico | Bulk capacitor |
| Diodo 1N4007 | 5 | 1A, 1000V | Flyback relés |

---

**Documento creado**: 2026-02-01  
**Versión**: 1.0  
**Autor**: Sistema de Control Coche Marcos
