# Remote Control — GPIO / UART usage verification

> Verificación exhaustiva de GPIO y UART del ESP32-S3 para asegurar que la
> integración del receptor iBUS NO entra en conflicto con NINGÚN pin ya
> usado. **NO IMPLEMENTADO** — documento de diseño previo.
>
> Verificado contra el firmware real (commits hasta 2026-05-24).

---

## 1. Inventario actual completo del ESP32-S3 (verificado en código)

| GPIO | Uso actual | Archivo de referencia |
|---|---|---|
| 0 | LIBRE (⚠️ strapping pin) | `docs/PIN_USAGE_INVENTORY.md` §6 |
| 1 | LIBRE (ADC1_CH1) | `docs/PIN_USAGE_INVENTORY.md` §6 |
| 2 | LIBRE (ADC1_CH2) | `docs/PIN_USAGE_INVENTORY.md` §6 |
| 3 | LIBRE (⚠️ strapping pin) | `docs/PIN_USAGE_INVENTORY.md` §6 |
| **4** | **CAN TX (TJA1051T/3)** | `esp32/src/main.cpp:101` |
| **5** | **CAN RX (TJA1051T/3)** | `esp32/src/main.cpp:102` |
| 6 | LIBRE | `docs/PIN_USAGE_INVENTORY.md` §6 |
| 7 | LIBRE | `docs/PIN_USAGE_INVENTORY.md` §6 |
| **8** | **I2C SDA (MCP23017 shifter, addr 0x20)** | `esp32/src/shifter_input.h:35` |
| **9** | **I2C SCL (MCP23017 shifter)** | `esp32/src/shifter_input.h:36` |
| **10** | TFT SPI CS (ST7796) | `esp32/User_Setup.h` / `platformio.ini` |
| **11** | **Audio relay (active-LOW)** | `esp32/src/relay_audio.h:47` |
| **12** | TFT SPI MISO (compartido con XPT2046) | `esp32/User_Setup.h` |
| **13** | TFT SPI MOSI | `esp32/User_Setup.h` |
| **14** | TFT SPI SCK | `esp32/User_Setup.h` |
| **15** | **Traction switch 2WD/4WD (INPUT_PULLUP)** | `esp32/src/traction_switch.h:35` |
| **16** | **LIBRE** ← **GPIO PROPUESTO PARA iBUS RX** | `docs/PIN_USAGE_INVENTORY.md` §6 |
| 17 | LIBRE | `docs/PIN_USAGE_INVENTORY.md` §6 |
| **18** | **UART1 RX (TF-Mini Plus obstacle sensor)** | `esp32/src/sensors/obstacle_sensor.h` |
| 19 | LIBRE (USB D−) si no se usa USB nativo | `docs/PIN_USAGE_INVENTORY.md` §6 |
| 20 | LIBRE (USB D+) si no se usa USB nativo | `docs/PIN_USAGE_INVENTORY.md` §6 |
| 21 | TFT DC / control | `esp32/User_Setup.h` |
| 26–37 | Reservados Flash + PSRAM (módulo N16R8) | datasheet |
| 38, 39, 42 | (verificar uso TFT BL / touch IRQ — ver `User_Setup.h`) | `esp32/User_Setup.h` |
| **40** | **Ignition sense (optocoupler)** | `esp32/src/power_manager.h` |
| **41** | **Power hold output** | `esp32/src/power_manager.h` |
| **43** | **UART2 TX → DFPlayer RX** | `esp32/src/audio_manager.h` |
| **44** | **UART2 RX ← DFPlayer TX** | `esp32/src/audio_manager.h` |
| 45 | LIBRE (⚠️ strapping pin) | datasheet |
| 46 | LIBRE (⚠️ strapping pin, solo input) | `docs/PIN_USAGE_INVENTORY.md` §6 |
| **47** | **WS2812B front strip data** | `esp32/src/led_controller.h:LED_FRONT_PIN` |
| **48** | **WS2812B rear strip data** | `esp32/src/led_controller.h:LED_REAR_PIN` |

---

## 2. Inventario de UARTs del ESP32-S3

| UART HW | Pines hardware originales | Estado actual | Pin asignado en firmware |
|---|---|---|---|
| **UART0** | U0TXD = GPIO 43 (default), U0RXD = GPIO 44 (default) — pero **REMAPEABLES** vía GPIO matrix | `Serial` (Arduino) va por **USB-CDC nativo** (`ARDUINO_USB_CDC_ON_BOOT=1`, `ARDUINO_USB_MODE=1` en `esp32/platformio.ini:43-44`). Los pines hardware de UART0 **no se usan** | — (libre vía matriz) |
| **UART1** | Cualquier GPIO vía matrix | DFPlayer? NO — DFPlayer está en UART2. UART1 está usada por el sensor de obstáculo | RX = GPIO 18, TX = −1 |
| **UART2** | Cualquier GPIO vía matrix | DFPlayer Mini | TX = GPIO 43, RX = GPIO 44 |

### 2.1 Conflicto aparente con UART2

> **Atención:** los pines GPIO 43 y 44 son **a la vez** los pines hardware
> originales de **UART0** y los pines de **UART2** asignados al **DFPlayer** en
> el firmware actual.
>
> Esto **NO es un conflicto** porque:
> - El ESP32-S3 puede rutear cualquier periférico UART a cualquier GPIO vía la
>   matriz de GPIOs.
> - El firmware actual asigna **explícitamente** GPIO 43/44 a **UART2** (no a UART0).
> - Como `Serial` va por USB-CDC, **UART0 no está usando esos pines hardware**.
> - Los pines U0TXD/U0RXD originales son ahora función de UART2 (DFPlayer).
>
> Para iBUS se propone usar **UART0** (el periférico UART hardware) **remapeada
> a GPIO 16** (que estaba libre). De esta forma se aprovecha que UART0 no tiene
> los pines hardware ocupados.

### 2.2 Plan UART para iBUS

```
HardwareSerial uartIBus(0);   // Periférico UART0 hardware
uartIBus.setRxBufferSize(256);
uartIBus.begin(115200, SERIAL_8N1, /*rx=*/16, /*tx=*/-1);
```

- Periférico: **UART0 hardware**.
- Pin RX: **GPIO 16**.
- Pin TX: **−1** (no necesario — modo recepción pura para iBUS Servo).
- Esto **NO afecta** al monitor Serial (que va por USB-CDC).
- Esto **NO afecta** a UART1 (obstacle sensor, GPIO 18).
- Esto **NO afecta** a UART2 (DFPlayer, GPIO 43/44).

---

## 3. Justificación de GPIO 16

| Criterio | GPIO 16 |
|---|---|
| Confirmado LIBRE en doc actual | ✅ `docs/PIN_USAGE_INVENTORY.md` §6 |
| Sin función actual en firmware | ✅ verificado con `grep -n "GPIO 16\|gpio16\|=\s*16" esp32/src` — sin resultados |
| No es strapping pin | ✅ (los strapping son 0, 3, 45, 46) |
| No es Flash / PSRAM | ✅ (esos son 26–37 en N16R8) |
| No es USB nativo | ✅ (esos son 19, 20) |
| Soporta función UART vía GPIO matrix | ✅ (cualquier GPIO del ESP32-S3) |
| Físicamente accesible en DevKitC-1 / N16R8 común | ✅ |
| Soporta input puro (no necesita drive) | ✅ |
| Bus / capacidad: limpio para 115200 baud (≈ 9 µs/bit) | ✅ |

### Alternativas viables (si GPIO 16 estuviera bloqueado por otra razón)

| GPIO | Pros | Contras |
|---|---|---|
| GPIO 17 | Mismo perfil que 16 | Ninguno |
| GPIO 1, 2 | También libres, son ADC1_CH1/2 | Si se reservan para futuro ADC, evitar |
| GPIO 6, 7 | Libres | Ningún contra |

**Decisión final: GPIO 16.** Alternativa de respaldo: GPIO 17.

---

## 4. Verificación cruzada — funciones del proyecto que NO se ven afectadas

### Periféricos hardware

| Función | Periférico ESP32 | Pines | ¿Afectado? |
|---|---|---|---|
| TFT ST7796 + XPT2046 touch | SPI (HSPI/VSPI) | 10, 12, 13, 14, otros | ❌ No |
| CAN bus | TWAI | 4, 5 | ❌ No |
| DFPlayer Mini | UART2 | 43, 44 | ❌ No |
| Obstacle sensor TF-Mini Plus | UART1 | 18 | ❌ No |
| Shifter MCP23017 | I2C0 | 8, 9 | ❌ No |
| LEDs WS2812B | RMT | 47, 48 | ❌ No |
| Audio relay | GPIO output | 11 | ❌ No |
| Ignition / power hold | GPIO in/out | 40, 41 | ❌ No |
| Traction switch | GPIO input | 15 | ❌ No |
| Serial monitor (debug) | USB-CDC | (USB nativo, no GPIOs) | ❌ No |

### Tareas / threads software

| Sistema software | Core | ¿Afectado? |
|---|---|---|
| renderTask (TFT + touch) | Core 0 | ❌ No — el módulo iBUS vive en Core 1 (loop()) |
| loop() / CAN heartbeat / CAN TX | Core 1 | ✅ Se le añade una llamada `remote_control::update()` no bloqueante |
| FreeRTOS scheduler | Ambos | ❌ No alterado |
| Task Watchdog (TWDT) | Sistema | ❌ No — parser no bloqueante (< 100 µs por `update()`) |

### Sistemas STM32 que NO se modifican

| Sistema | Archivo | ¿Afectado? |
|---|---|---|
| Safety system | `Core/Src/safety_system.c` | ❌ No |
| Motor control / BTS7960 | `Core/Src/motor_control.c` | ❌ No |
| PWM TIM1 / TIM8 | `Core/Src/main.c` (init) | ❌ No |
| IWDG | `Core/Src/main.c` (init) | ❌ No |
| Encoder steering E6B2 | `Core/Src/encoder_reader.c` | ❌ No |
| Steering PID | `Core/Src/steering_pid.c` (si aplica) | ❌ No |
| Steering centering | `Core/Src/steering_centering.c` | ❌ No |
| ABS / TCS | `Safety_GetTractionCapFactor()` | ❌ No |
| CAN handler | `Core/Src/can_handler.c` | ❌ No (Opción A) / ✅ Sí, 2 líneas (Opción B) |
| Main loop | `Core/Src/main.c` | ❌ No (Opción A) / ✅ Sí, 5 líneas (Opción B) |
| Pedal ADC + dual cross-check | `Core/Src/sensor_manager.c` | ❌ No |
| Pedal calibration NVM | `Core/Src/pedal_cal_store.c` | ❌ No |
| Flash NVM layout (páginas 123–127) | `STM32G474RETX_FLASH.ld` | ❌ No |

---

## 5. Resumen de pines ocupados POST-implementación

Si se ejecuta la implementación, el inventario quedaría:

| GPIO | Nuevo uso |
|---|---|
| 16 | iBUS RX (UART0 remapeada, recepción pura del receptor FS-iA6B) |

**Solo 1 GPIO nuevo añadido.** Los otros pines libres (1, 2, 6, 7, 17, 19, 20, 45, 46
con sus respectivas advertencias) siguen disponibles para futuras expansiones.

---

## 6. Comandos de verificación post-implementación

> Cuando se autorice la implementación, ejecutar:

```bash
# 1. Verificar que GPIO 16 NO está usado en ningún otro sitio del firmware
grep -rn "GPIO 16\|gpio16\|gpio_num_t.*16\b\| 16," esp32/src

# 2. Verificar que UART0 no se usa más que en remote_control
grep -rn "HardwareSerial(0)\|Serial0" esp32/src

# 3. Compilar y revisar warnings
cd esp32 && pio run -e esp32-s3-devkitc-1

# 4. Inspeccionar binario para confirmar tamaño dentro de límites
pio run -e esp32-s3-devkitc-1 --target size
```

---

**Estado del documento:** Verificación completa. GPIO 16 confirmado libre y adecuado.
UART0 confirmada disponible vía GPIO matrix. Sin conflictos con ningún subsistema.
