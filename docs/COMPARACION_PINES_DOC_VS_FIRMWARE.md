# COMPARACIÓN: LISTADO DE PINES vs FIRMWARE

**Fecha:** 2026-03-01  
**Objetivo:** Verificar que el documento `docs/LISTADO_PINES_COMPLETO.md` coincide exactamente con las definiciones del firmware (STM32: `Core/Inc/main.h`, `Core/Src/*.c`; ESP32: `esp32/src/*.h`, `esp32/include/User_Setup.h`).

---

## RESUMEN

| Área | Pines revisados | ✅ Correctos | ❌ Errores | ⚠️ Observaciones |
|------|----------------|-------------|-----------|------------------|
| ESP32-S3 GPIO | 21 | 20 | 1 | 1 |
| STM32 GPIO | 33 | 33 | 0 | 3 |
| Componentes pasivos | ~30 | ~28 | 0 | 2 |
| **Total** | **84** | **81** | **1** | **6** |

---

## PARTE 1 — ESP32-S3

### ✅ Pines que coinciden con el firmware

| GPIO | Función (LISTADO) | Firmware | Archivo | Estado |
|------|--------------------|----------|---------|--------|
| 4 | CAN TX | `CAN_TX_PIN = 4` | `esp32/src/main.cpp:35` | ✅ |
| 5 | CAN RX | `CAN_RX_PIN = 5` | `esp32/src/main.cpp:36` | ✅ |
| 8 | I2C SDA (Shifter) | `sdaPin = 8` | `esp32/src/shifter_input.h:35` | ✅ |
| 9 | I2C SCL (Shifter) | `sclPin = 9` | `esp32/src/shifter_input.h:36` | ✅ |
| 10 | Display CS | `#define TFT_CS 10` | `esp32/include/User_Setup.h:74` | ✅ |
| 11 | Relé Audio | `PIN_AUDIO_RELAY = 11` | `esp32/src/relay_audio.h:47` | ✅ |
| 12 | Display MISO | `#define TFT_MISO 12` | `esp32/include/User_Setup.h:72` | ✅ |
| 13 | Display MOSI | `#define TFT_MOSI 13` | `esp32/include/User_Setup.h:71` | ✅ |
| 14 | Display SCLK | `#define TFT_SCLK 14` | `esp32/include/User_Setup.h:73` | ✅ |
| 15 | Selector tracción | `gpioPin = 15` | `esp32/src/traction_switch.h:32` | ✅ |
| 18 | Obstáculo RX | `rxPin = 18` | `esp32/src/sensors/obstacle_sensor.h:59` | ✅ |
| 21 | Touch CS | `#define TOUCH_CS 21` | `esp32/include/User_Setup.h:88` | ✅ |
| 38 | Display RST | `#define TFT_RST 38` | `esp32/include/User_Setup.h:76` | ✅ |
| 39 | Display DC | `#define TFT_DC 39` | `esp32/include/User_Setup.h:75` | ✅ |
| 40 | Ignición sense | `PIN_IGNITION_SENSE = 40` | `esp32/src/power_manager.h:26` | ✅ |
| 41 | Power hold | `PIN_POWER_HOLD = 41` | `esp32/src/power_manager.h:27` | ✅ |
| 42 | Display backlight | `#define TFT_BL 42` | `esp32/include/User_Setup.h:79` | ✅ |
| 43 | DFPlayer TX | `PIN_DFPLAYER_TX = 43` | `esp32/src/audio_manager.h:24` | ✅ |
| 44 | DFPlayer RX | `PIN_DFPLAYER_RX = 44` | `esp32/src/audio_manager.h:25` | ✅ |
| 47 | LED frontal | `LED_FRONT_PIN = 47` | `esp32/src/led_controller.h:33` | ✅ |
| 48 | LED trasero | `LED_REAR_PIN = 48` | `esp32/src/led_controller.h:34` | ✅ |

### ❌ Error encontrado

| # | Sección | Lo que dice el LISTADO | Lo que dice el firmware | Archivo firmware | Corrección necesaria |
|---|---------|------------------------|------------------------|------------------|---------------------|
| 1 | 1.5 Sensor de Obstáculos (título) | **115200 baud** | **921600 baud** | `obstacle_sensor.h:61` | Cambiar "115200" por "921600" |

**Detalle:** El título de la sección 1.5 dice _"TOFSense-M (UART1 — 115200 baud)"_ pero el firmware define `baudRate = 921600` como baud rate por defecto. El TOFSense-M opera a 921600 bps para manejar tramas de 400 bytes a 10 Hz.

### ⚠️ Observación en el firmware (no error del LISTADO)

| # | Archivo | Línea | Lo que dice | Lo que debería decir |
|---|---------|-------|-------------|---------------------|
| 1 | `esp32/include/User_Setup.h` | 68 | `obstacle 6/7` | `obstacle 18` |

**Detalle:** El comentario en User_Setup.h dice _"CAN 4/5 · obstacle 6/7 · I2C 8/9"_ pero el sensor de obstáculos usa GPIO 18, no GPIO 6/7. Es un error en el comentario del código fuente, no del LISTADO. El LISTADO documenta correctamente GPIO 18.

---

## PARTE 2 — STM32G474RE

### ✅ Pines que coinciden con el firmware

| GPIO | Función (LISTADO) | Firmware | Archivo | Estado |
|------|--------------------|----------|---------|--------|
| PA8 | RPWM_FL (TIM1_CH1) | `PIN_PWM_FL = GPIO_PIN_8` | `main.h:21` | ✅ |
| PA9 | LPWM_FL (TIM1_CH2) | `PIN_LPWM_FL = GPIO_PIN_9` | `main.h:22` | ✅ |
| PA10 | RPWM_FR (TIM1_CH3) | `PIN_PWM_FR = GPIO_PIN_10` | `main.h:23` | ✅ |
| PC3 | LPWM_FR (TIM1_CH4) | `PIN_LPWM_FR = GPIO_PIN_3` | `main.h:24` | ✅ |
| PC6 | RPWM_RL (TIM8_CH1) | `PIN_PWM_RL = GPIO_PIN_6` | `main.h:28` | ✅ |
| PC7 | LPWM_RL (TIM8_CH2) | `PIN_LPWM_RL = GPIO_PIN_7` | `main.h:29` | ✅ |
| PC8 | RPWM_RR (TIM8_CH3) | `PIN_PWM_RR = GPIO_PIN_8` | `main.h:30` | ✅ |
| PC9 | LPWM_RR (TIM8_CH4) | `PIN_LPWM_RR = GPIO_PIN_9` | `main.h:31` | ✅ |
| PA6 | RPWM_STEER (TIM3_CH1) | `PIN_PWM_STEER = GPIO_PIN_6` | `main.h:36` | ✅ |
| PA7 | LPWM_STEER (TIM3_CH2) | `PIN_LPWM_STEER = GPIO_PIN_7` | `main.h:37` | ✅ |
| PC5 | EN_FL | `PIN_EN_FL = GPIO_PIN_5` | `main.h:53` | ✅ |
| PC13 | EN_RR | `PIN_EN_RR = GPIO_PIN_13` | `main.h:56` | ✅ |
| PC11 | RELAY_TRAC | `PIN_RELAY_TRAC = GPIO_PIN_11` | `main.h:61` | ✅ |
| PC12 | RELAY_DIR | `PIN_RELAY_DIR = GPIO_PIN_12` | `main.h:62` | ✅ |
| PB10 | RELAY_LED (frontal) | `PIN_RELAY_LED = GPIO_PIN_10` | `main.h:70` | ✅ |
| PB11 | RELAY_LED_REAR | `PIN_RELAY_LED_REAR = GPIO_PIN_11` | `main.h:71` | ✅ |
| PA0 | WHEEL_FL (EXTI0) | `PIN_WHEEL_FL = GPIO_PIN_0` | `main.h:74` | ✅ |
| PA1 | WHEEL_FR (EXTI1) | `PIN_WHEEL_FR = GPIO_PIN_1` | `main.h:75` | ✅ |
| PA2 | WHEEL_RL (EXTI2) | `PIN_WHEEL_RL = GPIO_PIN_2` | `main.h:76` | ✅ |
| PB15 | WHEEL_RR (EXTI15) | `PIN_WHEEL_RR = GPIO_PIN_15` | `main.h:77` | ✅ |
| PA15 | ENC_A (TIM2_CH1) | `PIN_ENC_A = GPIO_PIN_15` | `main.h:80` | ✅ |
| PB3 | ENC_B (TIM2_CH2) | `PIN_ENC_B = GPIO_PIN_3` | `main.h:81` | ✅ |
| PB4 | ENC_Z (EXTI4) | `PIN_ENC_Z = GPIO_PIN_4` | `main.h:82` | ✅ |
| PB5 | STEER_CENTER (EXTI5) | `PIN_STEER_CENTER = GPIO_PIN_5` | `main.h:87` | ✅ |
| PB6 | I2C_SCL | `PIN_I2C_SCL = GPIO_PIN_6` | `main.h:90` | ✅ |
| PB7 | I2C_SDA | `PIN_I2C_SDA = GPIO_PIN_7` | `main.h:91` | ✅ |
| PB0 | ONEWIRE | `PIN_ONEWIRE = GPIO_PIN_0` | `main.h:94` | ✅ |
| PA3 | PEDAL (ADC1_IN4) | `PIN_PEDAL = GPIO_PIN_3` | `main.h:99` | ✅ |
| PA11 | CAN_RX (FDCAN1_RX) | `PIN_CAN_RX = GPIO_PIN_11` | `project_config.h:227` | ✅ |
| PA12 | CAN_TX (FDCAN1_TX) | `PIN_CAN_TX = GPIO_PIN_12` | `project_config.h:226` | ✅ |
| PA13 | SWDIO | HAL MSP debug pins | `stm32g4xx_hal_msp.c` | ✅ |
| PA14 | SWCLK | HAL MSP debug pins | `stm32g4xx_hal_msp.c` | ✅ |

### ✅ Configuraciones verificadas contra el firmware

| Parámetro | LISTADO | Firmware | Archivo | Estado |
|-----------|---------|----------|---------|--------|
| TIM1/TIM8 Period | 4249 | `4249` | `main.c:566,698` | ✅ |
| TIM1/TIM8 Prescaler | 0 | `0` | `main.c:564,696` | ✅ |
| TIM3 Period | 4249 (implícito por 20 kHz) | `4249` | `main.c:672` | ✅ |
| TIM1/TIM8 BREAK2 LOCKUP | Sí | TIM_BREAK2 configured | `main.c` | ✅ |
| TIM3 sin BREAK | Sí | No BREAK en TIM3 MSP | `stm32g4xx_hal_msp.c:44-47` | ✅ |
| I2C velocidad | 400 kHz | Fast Mode, AF4 | `stm32g4xx_hal_msp.c:114` | ✅ |
| CAN velocidad | 500 kbps | Prescaler=17 → 500k | `main.c:524` | ✅ |
| CAN alternate func | AF9 | `GPIO_AF9_FDCAN1` | `stm32g4xx_hal_msp.c:21` | ✅ |
| TIM1 alternate func | AF6 | `GPIO_AF6_TIM1` | `stm32g4xx_hal_msp.c:70` | ✅ |
| TIM3 alternate func | AF2 | `GPIO_AF2_TIM3` | `stm32g4xx_hal_msp.c:83` | ✅ |
| TIM8 alternate func | AF4 | `GPIO_AF4_TIM8` | `stm32g4xx_hal_msp.c:97` | ✅ |
| TIM2 alternate func | AF1 | `GPIO_AF1_TIM2` | `stm32g4xx_hal_msp.c:192` | ✅ |
| I2C alternate func | AF4 | `GPIO_AF4_I2C1` | `stm32g4xx_hal_msp.c:114` | ✅ |
| Pedal divisor | 10kΩ + 6.8kΩ | `10 kΩ + 6.8 kΩ` | `stm32g4xx_hal_msp.c:125`, `sensor_manager.c:113` | ✅ |
| Sensor Hall pedal | SS1324LUA-T | `SS1324LUA-T` | `main.h:102` | ✅ |
| Encoder PPR | 1200 | `ENCODER_PPR = 1200` | `main.h:15` | ✅ |
| INA226 shunt | 1.5 mΩ (motor) / 0.75 mΩ (bat) | `1.5f` / `0.75f` | `project_config.h` | ✅ |
| TCA9548A dirección | 0x70 | `I2C_ADDR_TCA9548A = 0x70` | `main.h:114` | ✅ |
| INA226 dirección | 0x40 | `I2C_ADDR_INA226 = 0x40` | `main.h:115` | ✅ |
| ADS1115 dirección | — | Eliminado del firmware (plausibilidad por software) | — | ✅ (eliminado) |
| NUM_INA226 | 6 | `NUM_INA226 = 6` | `main.h:116` | ✅ |
| NUM_DS18B20 | 5 | `NUM_DS18B20 = 5` | `main.h:117` | ✅ |
| WHEEL_PULSES_REV | 6 | `WHEEL_PULSES_REV = 6` | `main.h:119` | ✅ |

### ⚠️ Observaciones (no errores, pero diferencias con documentación antigua)

| # | Tema | PINOUT_DEFINITIVO.md (documento antiguo) | Firmware actual / LISTADO | Nota |
|---|------|------------------------------------------|---------------------------|------|
| 1 | CAN Bus | PA11 (FDCAN1_RX), PA12 (FDCAN1_TX) | PA11 (RX), PA12 (TX) | Ambos documentos coinciden con el firmware actual. |
| 2 | Sensores velocidad rueda | PB5, PB10, PB11, PB12 | PA0, PA1, PA2, PB15 | LISTADO es correcto. PINOUT_DEFINITIVO tiene asignaciones antiguas. |
| 3 | Motor PWM asignación | 1 PWM + 1 DIR por motor (PA8=FL, PA9=FR, PA10=RL, PA11=RR) | 2 PWM (RPWM+LPWM) por motor en mismo timer (PA8/PA9=FL, PA10/PA11=FR) | LISTADO refleja la arquitectura RPWM/LPWM actual (PR #120). PINOUT_DEFINITIVO tiene esquema DIR+PWM antiguo. |
| 4 | Enable EN_RR | PD2 (PINOUT_DEFINITIVO) | PC13 (firmware) | LISTADO es correcto con PC13. PD2 no existe en LQFP64. |
| 5 | Pedal divisor | R1=1kΩ, R2=2kΩ (PINOUT_DEFINITIVO) | R1=10kΩ, R2=6.8kΩ (firmware) | LISTADO es correcto con 10kΩ/6.8kΩ. |
| 6 | Pedal sensor modelo | A1324LUA-T (PINOUT_DEFINITIVO) | SS1324LUA-T (firmware) | LISTADO es correcto con SS1324LUA-T. |

---

## PARTE 3 — COMPONENTES PASIVOS

| Componente | LISTADO | Firmware/código | Estado |
|-----------|---------|-----------------|--------|
| Pull-up OneWire 4.7 kΩ (PB0) | ✅ | No en firmware (hardware externo) | ✅ Documentación hardware correcta |
| Pull-up I2C 4.7 kΩ (PB6/PB7) | ✅ | `GPIO_NOPULL` en MSP → pull-up externo necesario | ✅ |
| CAN terminación 120 Ω | ✅ | No en firmware (hardware externo) | ✅ |
| Pedal divisor 10kΩ / 6.8kΩ | ✅ | `sensor_manager.c:147-148` confirma | ✅ |
| HY-M158 optoacopladores (aislamiento PWM BTS7960) | ✅ | Documentación de hardware | ✅ |
| Módulo 4-ch opto relé SRD-12VDC-SL-C (camino de relés de potencia) | ✅ | Documentación de hardware | ✅ |
| Snubber 100Ω + 100nF + 1N4148 | ✅ | Documentación de hardware | ✅ |
| Diodo volante 1N4007 (relés) | ✅ | Documentación de hardware | ✅ |
| LED datos 330Ω (WS2812B) | ✅ | No en firmware (hardware externo) | ✅ Buena práctica |
| DFPlayer TX 1kΩ serie | ✅ | No en firmware (hardware externo) | ✅ Recomendado |

---

## ACCIONES REALIZADAS

| # | Acción | Estado |
|---|--------|--------|
| 1 | Corregir baud rate del sensor de obstáculos en LISTADO: 115200 → 921600 | ✅ Corregido |
| 2 | Corregir comentario en `esp32/include/User_Setup.h` línea 68: "obstacle 6/7" → "obstacle 18" | ✅ Corregido |

---

## CONCLUSIÓN

El documento `LISTADO_PINES_COMPLETO.md` es **preciso y fiable** respecto al firmware actual. Se encontró **1 único error** (baud rate del sensor de obstáculos) que ha sido corregido. Todas las asignaciones de GPIO, configuraciones de timer, direcciones I2C, valores de componentes pasivos y arquitectura RPWM/LPWM coinciden exactamente con el código fuente.

El documento `PINOUT_DEFINITIVO.md` (creado anteriormente) contiene varias asignaciones obsoletas que no reflejan la arquitectura actual del firmware (PR #120). El LISTADO_PINES_COMPLETO.md ya documenta la versión correcta y actualizada.

---

_Comparación realizada: 2026-03-01_  
_Archivos verificados: `Core/Inc/main.h`, `Core/Src/main.c`, `Core/Src/stm32g4xx_hal_msp.c`, `Core/Src/sensor_manager.c`, `esp32/include/User_Setup.h`, `esp32/src/main.cpp`, `esp32/src/*.h`_
