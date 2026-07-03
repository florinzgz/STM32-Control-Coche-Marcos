# Especificación Completa de Hardware

> ## ⛔ DOCUMENTO LEGADO — REDIRIGIDO
>
> Este archivo mezclaba varias etapas antiguas del proyecto (pedal en `PA0`,
> shifter cableado al STM32, CAN ESP32 con `TJA1051`, etc.) y ya no es seguro
> usarlo como manual de montaje.
>
> Para evitar errores de cableado, el contenido antiguo se ha retirado y este
> documento queda como redirección a las fuentes actuales.

## Documentación vigente

- [`README.md`](../README.md) — referencia técnica consolidada
- [`HARDWARE_AND_SENSOR_MAP.md`](HARDWARE_AND_SENSOR_MAP.md) — mapa real de sensores
- [`HARDWARE_WIRING_MANUAL.md`](HARDWARE_WIRING_MANUAL.md) — manual eléctrico
- [`CONEXIONES_COMPLETAS.md`](CONEXIONES_COMPLETAS.md) — guía cable por cable
- [`MATERIALES_POR_MODULO.md`](MATERIALES_POR_MODULO.md) — componentes y extras de cableado

## Cambios críticos respecto a la versión antigua

- Pedal real: **PB1 / ADC1_IN12** (CN10_24; PA3/CN10_37 dañado/no usar)
- Palanca real: **MCP23017 en ESP32-S3 (I²C @ 400 kHz, SDA=GPIO8, SCL=GPIO9,
  dirección `0x20`)**.  Las posiciones P/R/N/D1/D2 se leen como contactos
  secos one-hot **active-LOW** con pull-ups internos del MCP23017; NEUTRAL =
  ninguna línea activa.  El gear se transmite al STM32 en `CMD_MODE` (CAN
  ID `0x102`) **byte 1** (byte 0 = mode flags).  El STM32 **NO** tiene
  GPIO de palanca: `PB12` y `PB13` están libres; `PB14` es `LED_DIAG`.
- CAN lado ESP32: **TJA1051T/3**
- CAN lado STM32: **TJA1051T/3**
- Sensor de obstáculos actual: **TF-Mini Plus en GPIO18**
- BTS7960: **RPWM/LPWM + EN** (sin pin DIR dedicado)
- `EN_RR`: **PC2**
- Relé de potencia de dirección (PC12) ahora se llama `PIN_RELAY_STEER_PWR`
  en el firmware (antes `PIN_RELAY_DIR`; nombre actual: `PIN_RELAY_STEER_PWR`).  Alimenta **el rail de 12 V del
  actuador de dirección**, no selecciona sentido de marcha.

## Fuente de verdad

- `Core/Inc/project_config.h`
- `esp32/include/User_Setup.h`
- `esp32/platformio.ini`
- `esp32/src/*.h`
