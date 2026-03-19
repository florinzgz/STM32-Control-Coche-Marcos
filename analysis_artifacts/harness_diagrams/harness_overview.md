# Harness Diagrams — Point-to-Point Wiring

> **Generated:** 2026-03-19 | **Source:** `Core/Inc/main.h`, `esp32/src/*.h`

---

## 1. Motor Harness (24V Power + PWM Control)

```
                                    ┌──────────────────────────────────────┐
                                    │          MOTOR HARNESS               │
                                    │  (24V power + PWM signal bundle)     │
                                    └──────────────────────────────────────┘

BATTERY 24V (+)
     │
     ├── [4 AWG RED] ── F1 (60A blade) ── [4 AWG RED]
     │                                         │
     │                              INA226-CH4 (0.5 mΩ shunt)
     │                              (Battery current sense)
     │                                         │
     │                              RELAY_MAIN (K1, PC10)
     │                                    │
     ├────────────────────────────────────┤
     │                                    │
     │    ┌───────────────┬───────────────┼───────────────┐
     │    │               │               │               │
     │ RELAY_TRAC      RELAY_TRAC      RELAY_TRAC      RELAY_DIR
     │ (K2, PC11)      (K2, PC11)     (K2, PC11)      (K3, PC12)
     │    │               │               │               │
     │ INA226-CH0      INA226-CH1      INA226-CH2      INA226-CH5
     │ (1 mΩ)          (1 mΩ)         (1 mΩ)          (1 mΩ)
     │    │               │               │               │
     │ F2a (30A)       F2b (30A)       F2c (30A)       F3 (20A)
     │    │               │               │               │
     │  BTS7960-FL     BTS7960-FR     BTS7960-RL      BTS7960-ST
     │  B+    B−       B+    B−       B+    B−        B+    B−
     │  │     │        │     │        │     │         │     │
     │  │    GND       │    GND       │    GND        │    GND
     │  │              │              │               │
     │  M+   M−        M+   M−        M+   M−         M+   M−
     │  │     │        │     │        │     │         │     │
     │  MOTOR FL       MOTOR FR       MOTOR RL        MOTOR STEER
     │
     │                  ┌─ INA226-CH3 (1 mΩ) ─ F2d (30A)
     │                  │
     │               RELAY_TRAC (shared)
     │                  │
     │               BTS7960-RR ─── M+/M− ─── MOTOR RR
     │
     └── [4 AWG BLACK] ─────────────────────────────── GND BUS BAR

PWM Signal Bundle (22 AWG, ≤ 30 cm):
     STM32 Nucleo                    BTS7960 Module
  ┌──────────────┐              ┌──────────────────┐
  │ PA8  (Orange)├─────────────►│ RPWM   (FL)      │
  │ PA9  (Org/Wh)├─────────────►│ LPWM   (FL)      │
  │ PC5  (Red)   ├────┬────────►│ R_EN   (FL)      │
  │              │    └────────►│ L_EN   (FL)      │
  ├──────────────┤              ├──────────────────┤
  │ PA10 (Blue)  ├─────────────►│ RPWM   (FR)      │
  │ PA11 (Blu/Wh)├─────────────►│ LPWM   (FR)      │
  │ 3.3V ────────├────┬────────►│ R_EN   (FR)      │
  │              │    └────────►│ L_EN   (FR)      │
  ├──────────────┤              ├──────────────────┤
  │ PC6  (Green) ├─────────────►│ RPWM   (RL)      │
  │ PC7  (Grn/Wh)├─────────────►│ LPWM   (RL)      │
  │ 3.3V ────────├────┬────────►│ R_EN   (RL)      │
  │              │    └────────►│ L_EN   (RL)      │
  ├──────────────┤              ├──────────────────┤
  │ PC8  (Yellow)├─────────────►│ RPWM   (RR)      │
  │ PC9  (Yel/Wh)├─────────────►│ LPWM   (RR)      │
  │ PC13 (Red)   ├────┬────────►│ R_EN   (RR)      │
  │              │    └────────►│ L_EN   (RR)      │
  ├──────────────┤              ├──────────────────┤
  │ PA6  (Purple)├─────────────►│ RPWM   (STEER)   │
  │ PA7  (Pur/Wh)├─────────────►│ LPWM   (STEER)   │
  │ 3.3V ────────├────┬────────►│ R_EN   (STEER)   │
  │              │    └────────►│ L_EN   (STEER)   │
  └──────────────┘              └──────────────────┘
```

---

## 2. CAN Bus Harness

```
   STM32 Node                                    ESP32 Node
┌──────────────────┐                          ┌──────────────────┐
│ Nucleo-64        │                          │ DevKitC-1        │
│                  │                          │                  │
│ PB9 ─► TXD      │     ┌──── CAN Bus ────┐  │ GPIO4 ─► TXD    │
│ PB8 ◄─ RXD      │     │  (Twisted Pair)  │  │ GPIO5 ◄─ RXD    │
│                  │     │                  │  │                  │
│ SN65HVD230       │     │  Yellow: CANH    │  │ TJA1051T/3       │
│ ┌───────────┐    │     │  Green:  CANL    │  │ ┌───────────┐    │
│ │CANH──120Ω─┤    │     │  Black:  GND     │  │ │CANH──120Ω─┤    │
│ │    TVS     │    │     │  Bare:   Shield  │  │ │    TVS     │    │
│ │CANL──120Ω─┤    │     │                  │  │ │CANL──120Ω─┤    │
│ └─┬─────┬───┘    │     │                  │  │ └─┬─────┬───┘    │
│   │     │        │     │                  │  │   │     │        │
└───┼─────┼────────┘     │                  │  └───┼─────┼────────┘
    │     │              │                  │      │     │
    │     │    J1        │                  │  J2  │     │
    ├─────┼──[Yellow]════╪══════════════════╪══════┼─────┤ CANH
    ├─────┼──[Green]═════╪══════════════════╪══════┼─────┤ CANL
    ├─────┼──[Black]═════╪══════════════════╪══════┼─────┤ GND
    └─────┘──[Bare]══════╪══════════════════╪══════┘     │ Shield
                         │ Shield grounded  │            │ (float)
                         │ at STM32 end     │
                         │ ONLY             │
                         └──────────────────┘

Cable: Shielded twisted pair, 22 AWG, ≤ 5 m
Connector: JST-XH 4-pin (J1, J2)
```

---

## 3. Sensor Harness

```
  ┌────────────────────────────────────────────────────┐
  │              SENSOR HARNESS                        │
  │  (22 AWG signal wires, routed away from power)     │
  └────────────────────────────────────────────────────┘

  ENCODER (E6B2-CWZ6C):
  ┌─────────────────┐      ┌──────────┐      ┌─────────┐
  │ Encoder         │      │ 6N137    │      │ STM32   │
  │ Brown──► 5V     │      │ Optocplr │      │         │
  │ Blue ──► GND    │      │ ×3       │      │         │
  │ Black──► 220Ω──►│──A──►│ Out ────►│──────│ PA15    │
  │ White──► 220Ω──►│──B──►│ Out ────►│──────│ PB3     │
  │ Orange─► 220Ω──►│──Z──►│ Out ────►│──────│ PB4     │
  └─────────────────┘      └──────────┘      └─────────┘

  WHEEL SPEED SENSORS (LJ12A3 × 4):
  ┌─────────────┐      ┌──────────┐      ┌─────────┐
  │ LJ12A3      │      │ PC817    │      │ STM32   │
  │ Brown─► 12V │      │ Optocplr │      │         │
  │ Blue ─► GND │      │          │      │         │
  │ Black─► 1kΩ─┤──►   │ Out ────►├──┬──►│ PA0 (FL)│
  └─────────────┘      └──────────┘  │   │ PA1 (FR)│
       (×4 sensors)     (×4 optos)   │   │ PA2 (RL)│
                                     │   │ PB15(RR)│
                                10kΩ pull-up to 3.3V
                                     │
                                    3.3V

  STEERING CENTER (LJ12A3):
  ┌─────────────┐      ┌──────────┐      ┌─────────┐
  │ LJ12A3      │      │ PC817    │      │ STM32   │
  │ Brown─► 12V │      │          │      │         │
  │ Blue ─► GND │      │ Out ────►├──┬──►│ PB5     │
  │ Black─► 1kΩ─┤──►   │          │  │   └─────────┘
  └─────────────┘      └──────────┘  │
                                10kΩ pull-up to 3.3V

  PEDAL SENSOR (SS1324LUA-T):
  ┌──────────────┐           ┌──────────────────────┐
  │ Pedal Sensor │           │ STM32 ADC            │
  │ VCC ──► 5V   │           │                      │
  │ GND ──► GND  │           │                      │
  │ OUT ──► [10kΩ R1]──┬─────│ PA3 (ADC1_IN4)       │
  └──────────────┘     │     │                      │
                  [6.8kΩ R2]  │ BAT54S clamp to rails│
                       │     │ 100nF cap to GND     │
                      GND    └──────────────────────┘

  TEMPERATURE (DS18B20 × 5):
  ┌──────────┐
  │ DS18B20  │──► VDD (3.3V)
  │  (×5)    │──► GND
  │          │──► DQ ──┬── 4.7kΩ ──► 3.3V
  └──────────┘         │
                    PB0 (OneWire bus)
```

---

## 4. Relay Harness

```
  ┌────────────────────────────────────────────────────────────────┐
  │                    RELAY HARNESS                               │
  │  (Relay coils on 12V/5V; contacts switch 24V/5V loads)        │
  └────────────────────────────────────────────────────────────────┘

  STM32 GPIO ──► 1kΩ ──┬──► 2N7000 Gate         Relay Coil (+)──► +V
                       │                              │
                  10kΩ pull-down                 D_fw (1N4007)
                       │                         (cathode to +V)
                      GND                             │
                                              Relay Coil (−)
                                                    │
                                              2N7000 Drain
                                              2N7000 Source ──► GND

  Relay mapping:
  ┌──────────┬────────┬───────────┬──────────────────────────────┐
  │ GPIO     │ Relay  │ Coil V    │ Load                         │
  ├──────────┼────────┼───────────┼──────────────────────────────┤
  │ PC10     │ K1     │ 12V       │ COM→NO: 24V bus to traction  │
  │ PC11     │ K2     │ 12V       │ COM→NO: 24V to motor BTS7960 │
  │ PC12     │ K3     │ 12V       │ COM→NO: 24V to steer BTS7960 │
  │ PB10     │ K4     │ 5V        │ COM→NO: 5V to front LED strip│
  │ PB11     │ K5     │ 5V        │ COM→NO: 5V to rear LED strip │
  └──────────┴────────┴───────────┴──────────────────────────────┘
```

---

## 5. ESP32 HMI Harness

```
  ┌────────────────────────────────────────────────────┐
  │              ESP32-S3 HMI HARNESS                  │
  └────────────────────────────────────────────────────┘

  ESP32-S3-DevKitC-1
  ┌──────────────────────────────────────────────────┐
  │                                                  │
  │  GPIO 4 ──► CAN TX ──────────────► TJA1051 TXD  │
  │  GPIO 5 ◄── CAN RX ◄───────────── TJA1051 RXD  │
  │                                                  │
  │  GPIO 8 ──► I2C SDA ──┬── 4.7kΩ──► 3.3V         │
  │  GPIO 9 ──► I2C SCL ──┤                          │
  │                        └──► MCP23017 (Shifter)    │
  │                                                  │
  │  GPIO 10 ──► TFT CS  ─┐                          │
  │  GPIO 12 ◄── TFT MISO │                          │
  │  GPIO 13 ──► TFT MOSI ├── ST7796 Display Module  │
  │  GPIO 14 ──► TFT SCLK │   (480×320, SPI)         │
  │  GPIO 21 ──► TOUCH CS  │                          │
  │  GPIO 38 ──► TFT RST  │                          │
  │  GPIO 39 ──► TFT DC   │                          │
  │  GPIO 42 ──► TFT BL  ─┘                          │
  │                                                  │
  │  GPIO 11 ──► [Audio Relay Module] ──► Speaker     │
  │                                                  │
  │  GPIO 43 ──► 1kΩ ──► DFPlayer RX                  │
  │  GPIO 44 ◄───────── DFPlayer TX                   │
  │                                                  │
  │  GPIO 15 ◄── [Traction Switch] ──► GND (4WD)     │
  │              (pull-up: HIGH = 2WD)                │
  │                                                  │
  │  GPIO 40 ◄── [Ignition Key Sense]                 │
  │  GPIO 41 ──► [Power Hold Latch]                   │
  │                                                  │
  │  GPIO 47 ──► 330Ω ──► WS2812B Front (28 LEDs)    │
  │  GPIO 48 ──► 330Ω ──► WS2812B Rear  (16 LEDs)    │
  │                                                  │
  └──────────────────────────────────────────────────┘
```

---

## 6. Harness Grouping and Routing Recommendations

### Cable Groups (tie together with cable ties every 15 cm):

| Group | Cables | AWG | Route |
|-------|--------|-----|-------|
| **A: 24V Power** | Battery→Fuse→Relay→BTS B+ | 4–10 AWG | Along chassis frame, away from signal cables |
| **B: Motor** | BTS M+/M− to each motor | 10 AWG | Direct, shortest path to each motor |
| **C: PWM Signal** | PA8/PA9, PA10/PA11, PC6/PC7, PC8/PC9, PA6/PA7 | 22 AWG | Bundled together, ≥5 cm from power cables |
| **D: CAN Bus** | CANH/CANL (twisted pair, shielded) | 22 AWG | Separate from all other groups |
| **E: Sensor** | Encoder, wheel sensors, pedal | 22 AWG | Shielded where >50 cm; separate from power |
| **F: I2C** | SCL/SDA to TCA9548A and INA226s | 22 AWG | Short runs (≤30 cm); away from motors |
| **G: ESP32-Display** | SPI (MOSI/MISO/CLK/CS) | 22 AWG | Short (≤15 cm), keep together |
| **H: LED Data** | GPIO47/48 to LED strips | 22 AWG | Separate from power; 330Ω series R at source |
| **I: LED Power** | 5V to LED strips via relays | 18 AWG | Can share route with Group A |

### Anchor Points
- Secure every cable group at 15 cm intervals with nylon cable ties
- Use adhesive cable mounts on chassis for routing
- Provide strain relief at every connector
- Leave 10% slack for vibration absorption
- Use split loom tubing for groups A, B (power cables) for abrasion protection
