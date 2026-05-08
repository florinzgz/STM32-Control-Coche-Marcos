# Diagramas Visuales — ESP32-S3 / STM32 / CAN

> Diagramas simplificados y actualizados para taller.
>
> Fuente verificada contra firmware real:
> `Core/Inc/project_config.h`, `esp32/include/User_Setup.h`, `esp32/platformio.ini`,
> `esp32/src/main.cpp`, `esp32/src/audio_manager.h`, `esp32/src/power_manager.h`,
> `esp32/src/shifter_input.h`, `esp32/src/sensors/obstacle_sensor.h`

---

## 1. Vista general

```
┌────────────────────┐                     ┌────────────────────┐
│     ESP32-S3       │                     │   STM32G474RE      │
│                    │                     │                    │
│ GPIO4/GPIO5        │                     │ PA12 / PA11        │
│  └─ TJA1051T/3 ────┼──── CAN BUS ────────┼─ TJA1051T/3        │
│                    │                     │                    │
│ TFT ST7796         │                     │ BTS7960 x5         │
│ XPT2046 touch      │                     │ INA226 x6          │
│ TF-Mini Plus       │                     │ DS18B20 x5         │
│ DFPlayer Mini      │                     │ Encoder + wheel    │
│ MCP23017           │                     │ sensors + relays   │
└────────────────────┘                     └────────────────────┘
```

---

## 2. ESP32-S3 — Pantalla

```
ESP32-S3                  Display ST7796 + XPT2046
────────                  ─────────────────────────
GPIO10  ────────────────► CS
GPIO12  ◄───────────────► MISO / T_DO
GPIO13  ────────────────► MOSI / T_DIN
GPIO14  ────────────────► SCK  / T_CLK
GPIO21  ────────────────► T_CS
GPIO38  ────────────────► RESET
GPIO39  ────────────────► DC
GPIO42  ────────────────► BL
3.3V    ────────────────► VCC
GND     ────────────────► GND
```

---

## 3. ESP32-S3 — CAN correcto

```
ESP32-S3                 TJA1051T/3                  Bus CAN
────────                 ──────────                  ───────
GPIO4   ───────────────► TXD
GPIO5   ◄────────────── RXD
3.3V    ───────────────► VCC
GND     ───────────────► GND
GND     ───────────────► RS / slope
                         CANH ─────────────────────► CANH
                         CANL ─────────────────────► CANL
```

> El lado ESP32 usa **TJA1051T/3**.  
> El lado STM32 usa **TJA1051T/3**.

---

## 4. STM32 — CAN correcto

```
STM32G474RE              TJA1051T/3                 Bus CAN
───────────              ──────────                 ───────
PA12   ────────────────► TXD
PA11   ◄─────────────── RXD
5V     ────────────────► VCC
3.3V   ────────────────► VIO
GND    ────────────────► GND
GND    ────────────────► S
                          CANH ───────────────────► CANH
                          CANL ───────────────────► CANL
```

---

## 5. ESP32-S3 — periféricos auxiliares

```
GPIO18  ◄──── TF-Mini Plus TX (115200 bps, directo, sin divisor)
GPIO43  ───── DFPlayer RX
GPIO44  ◄──── DFPlayer TX
GPIO11  ───── Relé audio IN (active LOW)
GPIO8   ───── MCP23017 SDA
GPIO9   ───── MCP23017 SCL
GPIO40  ◄──── IGNITION_SENSE (EL817, LOW = llave ON)
GPIO41  ───── POWER_HOLD
GPIO47  ───── WS2812B frontal
GPIO48  ───── WS2812B trasera
```

---

## 6. STM32 — pines críticos de taller

```
PA8  / PA9   ── BTS7960 FL RPWM / LPWM
PA10 / PC3   ── BTS7960 FR RPWM / LPWM
PC6  / PC7   ── BTS7960 RL RPWM / LPWM
PC8  / PC9   ── BTS7960 RR RPWM / LPWM
PA6  / PA7   ── BTS7960 STEER RPWM / LPWM

PC5 ── EN_FL
PC0 ── EN_FR
PC1 ── EN_RL
PC2 ── EN_RR
PC4 ── EN_STEER

PC11 ── RELAY_TRAC
PC12 ── RELAY_DIR
PB10 ── RELAY_LED_FRONT
PB11 ── RELAY_LED_REAR

PA0 / PA1 / PA2 / PB15 ── sensores rueda
PA15 / PB3 / PB4       ── encoder A / B / Z
PB5                    ── sensor centrado
PA3                    ── pedal ADC
PB0                    ── OneWire DS18B20
PB6 / PB7              ── I2C INA226 / TCA9548A
```

---

## 7. Checklist visual rápido

- [ ] ESP32 CAN en **TJA1051T/3**
- [ ] STM32 CAN en **PA11/PA12**
- [ ] `EN_RR` en **PC2**, no en PC13
- [ ] Sensor de obstáculos actual = **TF-Mini Plus** en **GPIO18**
- [ ] MCP23017 en **GPIO8/GPIO9**
- [ ] Relé audio en **GPIO11**
- [ ] Ignición en **GPIO40**, lógica invertida, entrada optoacoplada

---

Ver también:
- `CONEXIONES_COMPLETAS.md`
- `HARDWARE_WIRING_MANUAL.md`
- `CONEXIONES_RAPIDAS_ESP32.md`
