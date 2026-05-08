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
- [`HARDWARE_SPECIFICATION.md`](HARDWARE_SPECIFICATION.md) — arquitectura hardware
- [`HARDWARE_WIRING_MANUAL.md`](HARDWARE_WIRING_MANUAL.md) — manual eléctrico
- [`CONEXIONES_COMPLETAS.md`](CONEXIONES_COMPLETAS.md) — guía cable por cable
- [`MATERIALES_POR_MODULO.md`](MATERIALES_POR_MODULO.md) — componentes y extras de cableado

## Cambios críticos respecto a la versión antigua

- Pedal real: **PA3 / ADC1_IN4**
- Palanca real: **MCP23017 en ESP32 GPIO8 / GPIO9**
- CAN lado ESP32: **SN65HVD230**
- CAN lado STM32: **TJA1051T/3**
- Sensor de obstáculos actual: **TF-Mini Plus en GPIO18**
- BTS7960: **RPWM/LPWM + EN** (sin pin DIR dedicado)
- `EN_RR`: **PC2**

## Fuente de verdad

- `Core/Inc/project_config.h`
- `esp32/include/User_Setup.h`
- `esp32/platformio.ini`
- `esp32/src/*.h`
