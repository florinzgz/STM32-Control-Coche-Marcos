# 📌 PINOUT Completo - STM32G474RE

> ## ⛔ DOCUMENTO OBSOLETO — NO USAR PARA CABLEADO
>
> Este archivo se conserva **solo como redirección** porque durante varias fases del proyecto
> llegó a contener tablas antiguas con arquitectura **PWM+DIR+EN**, pines CAN incorrectos y
> asignaciones de rueda/pedal ya retiradas del firmware.
>
> Para evitar errores de montaje, el contenido obsoleto se ha eliminado de este documento.

## Usar en su lugar

- [`docs/CONEXIONES_COMPLETAS.md`](CONEXIONES_COMPLETAS.md) — guía cable por cable actualizada
- [`docs/HARDWARE_WIRING_MANUAL.md`](HARDWARE_WIRING_MANUAL.md) — manual eléctrico completo
- [`docs/HARDWARE_SPECIFICATION.md`](HARDWARE_SPECIFICATION.md) — arquitectura hardware consolidada
- [`docs/MATERIALES_POR_MODULO.md`](MATERIALES_POR_MODULO.md) — componentes y extras de cableado
- [`Core/Inc/project_config.h`](../Core/Inc/project_config.h) — fuente de verdad del pinout STM32

## Cambios críticos respecto a documentación antigua

- CAN STM32: **PA11 / PA12** (no PB8 / PB9)
- Pedal ADC: **PB1 / ADC1_IN12** (movido desde PA3 por corto a GND; PA3 dañado/no usar)
- Sensores de rueda: **PA0 / PA1 / PB2 / PB15** (RL movido de PA2 a PB2 por corto a GND; PA2 dañado)
- BTS7960: arquitectura **RPWM/LPWM + EN** (sin pin DIR)
- `EN_RR`: **PC2** (no PC13)
- `PC10`: libre / sin uso, configurado `INPUT_PULLDOWN`

## Fuente de verdad

Las asignaciones reales se verifican contra:

- `Core/Inc/project_config.h`
- `Core/Src/main.c`
- `STM32-Control-Coche-Marcos.ioc`

---

**Estado:** archivo histórico neutralizado para que no pueda reutilizarse por error en taller.
