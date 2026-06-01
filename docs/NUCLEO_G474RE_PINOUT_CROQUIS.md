# NUCLEO-G474RE — Croquis de Conectores con Señales del Proyecto

**Placa:** NUCLEO-G474RE (MB1367)  
**Fuente pines:** UM2505 Rev4 Tabla 16  
**Señales proyecto:** firmware STM32 + CAN con ESP32

---

## Cómo leer este croquis

```
FUNCIÓN PROYECTO    SEÑAL STM32
       │                │
       └─── ●  12       ← número de pin dentro del rectángulo (conector)
            │
         CN10           ← nombre del conector encima del rectángulo
```

- **●** = agujero físico del conector Morpho / Arduino 2.54 mm
- El **número** es el número de pin tal como está serigrafado en la PCB
- La **rayita ───** es el cable que conectas a ese pin
- **Columna izquierda** = pines impares (1,3,5…); **columna derecha** = pines pares (2,4,6…)

---

## CN4 — Conector SWD/JTAG (debug ST-Link interno, paso 1,27 mm)

```
          ┌────────────┐
 +3V3 ────┤ 1●      ●2 ├──── SWCLK  (PA14)
  GND ────┤ 3●      ●4 ├──── SWDIO  (PA13)
 NRST ────┤ 5●      ●6 ├──── SWO    (PB3)
          └────────────┘
               CN4
```

> CN4 es interno al ST-Link; no se suele cablear en el proyecto.

---

## CN3 — UART Virtual COM Port (puente ST-Link ↔ PC)

```
          ┌──────────────┐
 TX→PC ───┤ 1● T_VCP_TX  ├─── (PA2 / USART2_TX)
 RX←PC ───┤ 2● T_VCP_RX  ├─── (PA3 / USART2_RX)
     GND ─┤ 3●            │
          └──────────────┘
               CN3
```

> Usado normalmente para depuración serie; en este proyecto PA2/PA3 son también las ruedas RL/PEDAL en CN10.

---

## CN5 — Arduino Power Header (6 pines, izquierda superior)

```
          ┌────────────┐
      NC ─┤ 1●         │
   IOREF ─┤ 2●  3.3 V  │
    NRST ─┤ 3●         │
    3.3V ─┤ 4●  salida │
      5V ─┤ 5●  salida │
     GND ─┤ 6●         │
          └────────────┘
               CN5
```

---

## CN6 — Arduino Analog Header (8 pines, izquierda inferior)

```
SEÑAL STM32          PIN   FUNCIÓN PROYECTO
                   ┌──────────────────┐
PA0  WHEEL_FL ─────┤ ●1  A0           │
PA1  WHEEL_FR ─────┤ ●2  A1           │
PA4  libre    ─────┤ ●3  A2           │
PB0  1-Wire   ─────┤ ●4  A3  DS18B20  │
PC1  EN_RL    ─────┤ ●5  A4           │
PC0  EN_FR    ─────┤ ●6  A5           │
GND           ─────┤ ●7  GND          │
AREF (VREF+)  ─────┤ ●8  AREF         │
                   └──────────────────┘
                          CN6
```

---

## CN9 — Arduino Digital Header D0–D7 (8 pines, derecha inferior)

```
SEÑAL STM32              PIN   FUNCIÓN PROYECTO
                       ┌──────────────────────┐
PA3  PEDAL ADC1_IN4 ───┤ ●1  D0  (UART2 RX)   │
PA2  WHEEL_RL EXTI2 ───┤ ●2  D1  (UART2 TX)   │
PA10 RPWM_FR TIM1_3 ───┤ ●3  D2               │
PB3  ENC_B  TIM2_CH2 ──┤ ●4  D3  (TIM2 PWM)  │
PB5  STEER_CENTER ──── ┤ ●5  D4  ↓EXTI5       │
PB4  ENC_Z  ↓EXTI4 ────┤ ●6  D5               │
PB10 RELAY_LED_F  ─────┤ ●7  D6               │
PA8  RPWM_FL TIM1_1 ───┤ ●8  D7               │
                       └──────────────────────┘
                               CN9
```

---

## CN8 — Arduino Digital Header D8–D15 (10 pines, derecha superior)

```
SEÑAL STM32              PIN   FUNCIÓN PROYECTO
                       ┌──────────────────────┐
PA9  LPWM_FL TIM1_2 ───┤ ●1  D8               │
PC7  LPWM_RL TIM8_2 ───┤ ●2  D9               │
PB6  libre        ─────┤ ●3  D10 (SPI CS)     │
PA7  LPWM_STEER T3_2 ──┤ ●4  D11 (SPI MOSI)   │
PA6  RPWM_STEER T3_1 ──┤ ●5  D12 (SPI MISO)   │
PA5  LED_LD2 verde ────┤ ●6  D13 (SPI SCK)    │
GND               ─────┤ ●7  GND              │
AREF              ─────┤ ●8  AREF             │
PB9  I2C1_SDA     ─────┤ ●9  D14 / SDA        │
PB8  I2C1_SCL     ─────┤ ●10 D15 / SCL        │
                       └──────────────────────┘
                               CN8
```

---

## CN7 — Morpho Izquierdo (38 pines, 2 columnas)

```
FUNCIÓN PROYECTO              CN7               FUNCIÓN PROYECTO
                         ┌──────────┐
libre/GPIO_IN_PD  PC10 ──┤ 1●    ●2 ├── PC11   RELAY_TRACCIÓN
RELAY_DIRECCIÓN   PC12 ──┤ 3●    ●4 ├── PD2    libre
+3V3 placa         VDD ──┤ 5●    ●6 ├── E5V    +5V ext (entrada)
modo arranque    BOOT0 ──┤ 7●    ●8 ├── GND
                    NC ──┤ 9●   ●10 ├── NC
                    NC ──┤11●   ●12 ├── IOREF
SWDIO depurador  PA13 ──┤13●   ●14 ├── NRST   reset
SWCLK depurador  PA14 ──┤15●   ●16 ├── +3V3   salida 3.3V
ENC_A  TIM2_CH1  PA15 ──┤17●   ●18 ├── +5V    salida 5V
                   GND ──┤19●   ●20 ├── GND
libre              PB7 ──┤21●   ●22 ├── GND
botón USER        PC13 ──┤23●   ●24 ├── VIN    alimentación ext.
OSC32_IN          PC14 ──┤25●   ●26 ├── NC
OSC32_OUT         PC15 ──┤27●   ●28 ├── PA0    WHEEL_FL ↑EXTI0
OSC_IN cristal     PF0 ──┤29●   ●30 ├── PA1    WHEEL_FR ↑EXTI1
OSC_OUT cristal    PF1 ──┤31●   ●32 ├── PA4    libre (ADC)
VBAT pila RTC     VBAT ──┤33●   ●34 ├── PB0    1-Wire DS18B20
EN_RR enable       PC2 ──┤35●   ●36 ├── PC1    EN_RL enable
LPWM_FR TIM1_CH4   PC3 ──┤37●   ●38 ├── PC0    EN_FR enable
                         └──────────┘
                              CN7
```

---

## CN10 — Morpho Derecho (38 pines, 2 columnas)

```
FUNCIÓN PROYECTO              CN10              FUNCIÓN PROYECTO
                         ┌──────────┐
LPWM_RR TIM8_CH4   PC9 ──┤ 1●    ●2 ├── PC8    RPWM_RR TIM8_CH3
I2C1_SCL           PB8 ──┤ 3●    ●4 ├── PC6    RPWM_RL TIM8_CH1
I2C1_SDA           PB9 ──┤ 5●    ●6 ├── PC5    EN_FL enable
VREF analógico   VREFP ──┤ 7●    ●8 ├── 5VUSB  St-Link (no usar)
                   GND ──┤ 9●   ●10 ├── NC
LED verde LD2      PA5 ──┤11●   ●12 ├── PA12   CAN_TX → ESP32
RPWM_STEER TIM3_1  PA6 ──┤13●   ●14 ├── PA11   CAN_RX ← ESP32
LPWM_STEER TIM3_2  PA7 ──┤15●   ●16 ├── PB12   libre
libre              PB6 ──┤17●   ●18 ├── PB11   RELAY_LED_TRASERO
LPWM_RL TIM8_CH2   PC7 ──┤19●   ●20 ├── GND
LPWM_FL TIM1_CH2   PA9 ──┤21●   ●22 ├── PB2    libre
RPWM_FL TIM1_CH1   PA8 ──┤23●   ●24 ├── PB1    libre
RELAY_LED_FRONTAL  PB10 ──┤25●   ●26 ├── PB15   WHEEL_RR ↑EXTI15
ENC_Z ↓EXTI4       PB4 ──┤27●   ●28 ├── PB14   LED_DIAG
STEER_CENTER ↓EXTI5 PB5 ──┤29●   ●30 ├── PB13   libre
ENC_B  TIM2_CH2    PB3 ──┤31●   ●32 ├── AGND   GND analógico
RPWM_FR TIM1_CH3  PA10 ──┤33●   ●34 ├── PC4    EN_STEER enable
WHEEL_RL ↑EXTI2    PA2 ──┤35●   ●36 ├── NC
PEDAL ADC1_IN4     PA3 ──┤37●   ●38 ├── NC
                         └──────────┘
                              CN10
```

---

## Resumen de señales críticas del proyecto

| Señal proyecto       | GPIO STM32 | Conector | Pin  | Periférico        |
|----------------------|-----------|----------|------|-------------------|
| CAN_TX → ESP32       | PA12      | CN10     |  12  | FDCAN1_TX         |
| CAN_RX ← ESP32       | PA11      | CN10     |  14  | FDCAN1_RX         |
| RELAY_TRACCIÓN       | PC11      | CN7      |   2  | GPIO OUT          |
| RELAY_DIRECCIÓN      | PC12      | CN7      |   3  | GPIO OUT          |
| RELAY_LED_FRONTAL    | PB10      | CN10     |  25  | GPIO OUT          |
| RELAY_LED_TRASERO    | PB11      | CN10     |  18  | GPIO OUT          |
| RPWM_FL              | PA8       | CN10     |  23  | TIM1_CH1          |
| LPWM_FL              | PA9       | CN10     |  21  | TIM1_CH2          |
| RPWM_FR              | PA10      | CN10     |  33  | TIM1_CH3          |
| LPWM_FR              | PC3       | CN7      |  37  | TIM1_CH4          |
| RPWM_RL              | PC6       | CN10     |   4  | TIM8_CH1          |
| LPWM_RL              | PC7       | CN10     |  19  | TIM8_CH2          |
| RPWM_RR              | PC8       | CN10     |   2  | TIM8_CH3          |
| LPWM_RR              | PC9       | CN10     |   1  | TIM8_CH4          |
| RPWM_STEER           | PA6       | CN10     |  13  | TIM3_CH1          |
| LPWM_STEER           | PA7       | CN10     |  15  | TIM3_CH2          |
| ENC_A                | PA15      | CN7      |  17  | TIM2_CH1          |
| ENC_B                | PB3       | CN10     |  31  | TIM2_CH2          |
| ENC_Z (centro)       | PB4       | CN10     |  27  | EXTI4 ↓           |
| STEER_CENTER         | PB5       | CN10     |  29  | EXTI5 ↓           |
| WHEEL_FL             | PA0       | CN7      |  28  | EXTI0 ↑           |
| WHEEL_FR             | PA1       | CN7      |  30  | EXTI1 ↑           |
| WHEEL_RL             | PA2       | CN10     |  35  | EXTI2 ↑           |
| WHEEL_RR             | PB15      | CN10     |  26  | EXTI15 ↑          |
| EN_FL                | PC5       | CN10     |   6  | GPIO OUT          |
| EN_FR                | PC0       | CN7      |  38  | GPIO OUT          |
| EN_RL                | PC1       | CN7      |  36  | GPIO OUT          |
| EN_RR                | PC2       | CN7      |  35  | GPIO OUT          |
| EN_STEER             | PC4       | CN10     |  34  | GPIO OUT          |
| PEDAL_ADC            | PA3       | CN10     |  37  | ADC1_IN4          |
| 1-Wire DS18B20       | PB0       | CN7      |  34  | GPIO OD           |
| I2C1_SCL             | PB8       | CN10     |   3  | I2C1_SCL          |
| I2C1_SDA             | PB9       | CN10     |   5  | I2C1_SDA          |
| SWDIO depurador      | PA13      | CN7      |  13  | SWD               |
| SWCLK depurador      | PA14      | CN7      |  15  | SWD               |
| LED_LD2 verde        | PA5       | CN10     |  11  | GPIO OUT          |
| PC10 (sin usar)      | PC10      | CN7      |   1  | GPIO IN PD        |

---

*Generado automáticamente — Fuente: UM2505 Rev4 Tabla 16 + firmware STM32-Control-Coche-Marcos*
