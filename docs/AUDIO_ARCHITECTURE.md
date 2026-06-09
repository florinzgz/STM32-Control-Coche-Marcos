# AUDIO_ARCHITECTURE — Arquitectura de Audio MarcosDashboard

**Versión:** 1.1  
**Fecha:** 2026-06-09  
**Estado:** Actualizado con hardware real verificado (Songle 4CH + ULN2803A)

## 1) Estado real verificado

- Módulo **Songle 4CH** instalado, con **4 relés independientes**.
- Prueba física confirmada:
  - `IN3` conmuta solo `S3`.
  - `IN4` conmuta solo `S4`.
  - No hay acoplamiento interno entre `S3` y `S4`.
- Medición real:
  - `IN3` vs GND ≈ 5V.
  - `IN4` vs GND ≈ 5V.
- Conclusión eléctrica: entradas `INx` con **pull-up interno a 5V**.
- Regla de seguridad: **NO conectar GPIO ESP32/STM32 directamente a IN1/IN2/IN3/IN4**.

## 2) Arquitectura final de control de relés

Cadena obligatoria para todos los canales de control:

`GPIO MCU (3.3V) -> ULN2803A (B/C) -> INx Songle`

Esto elimina la exposición directa de 5V de los pull-up de `INx` a los GPIO del MCU.

## 3) Audio (ruta analógica)

Arquitectura validada:

- Conmutación de audio **antes del PAM8403**.
- Mezcla L+R a mono por resistencias **1k + 1k**.
- Condensador de acoplo **105 = 1µF** en serie en la línea de audio.
- Relé usado para conmutación de audio: **Songle S3 (IN3)**.
- Masa común de audio mantenida.

## 4) Lógica de firmware (estado actual actualizado)

### ESP32 (audio)

`PIN_AUDIO_RELAY = GPIO11`.

Con ULN2803A en el camino:

- `GPIO11 HIGH` -> ULN canal ON (sink) -> `IN3` a LOW -> **Relé audio ON**.
- `GPIO11 LOW` -> ULN canal OFF -> `IN3` sube a 5V -> **Relé audio OFF**.

> Resultado: la lógica efectiva en GPIO11 se invierte respecto al estado anterior directo.

### STM32 (relés LED/potencia)

Sin cambios funcionales de control:

- `PB10` LED frontal: `GPIO_PIN_SET = ON`.
- `PB11` LED trasero: `GPIO_PIN_SET = ON`.
- `PC11` tracción: `GPIO_PIN_SET = ON`.
- `PC12` steering power: `GPIO_PIN_SET = ON`.

## 5) Tabla definitiva GPIO -> ULN -> Songle

| Función | GPIO MCU | ULN entrada | ULN salida | Destino Songle | GPIO reposo | GPIO activo |
|---|---|---|---|---|---|---|
| LED frontal | PB10 | 1B | 1C | IN1 | LOW (OFF) | HIGH (ON) |
| LED trasero | PB11 | 2B | 2C | IN2 | LOW (OFF) | HIGH (ON) |
| Audio relay | GPIO11 | 3B | 3C | IN3 (S3) | LOW (OFF) | HIGH (ON) |
| Tracción | PC11 | 4B | 4C | IN4 o canal asignado de potencia | LOW (OFF) | HIGH (ON) |
| Steering power | PC12 | 5B | 5C | INx restante asignado | LOW (OFF) | HIGH (ON) |

> Canales ULN 6,7,8: libres.

## 6) Conexiones ULN2803A obligatorias

- `GND ULN` -> GND común ESP32 + STM32 + Songle.
- `COM ULN`:
  - Si ULN solo maneja entradas `INx` del Songle: puede dejarse sin uso.
  - Si ULN maneja cargas inductivas directas: `COM` al positivo de esa carga.
- Resistencias externas en entradas ULN: **no necesarias** (2.7k internas).
- Diodos externos para entradas Songle: **no necesarios**.

## 7) Cambio de firmware mínimo requerido

Sí hay cambio mínimo requerido en ESP32 para audio:

- `esp32/src/relay_audio.cpp`
- `esp32/src/relay_audio.h`

Motivo: adaptación de polaridad de GPIO11 para ULN2803A.

No se requieren cambios funcionales en:

- `esp32/src/audio_manager.cpp`
- `Core/Inc/project_config.h`
- `Core/Src/safety_system.c`
- `Core/Src/can_handler.c`
- `Core/Src/main.c`
