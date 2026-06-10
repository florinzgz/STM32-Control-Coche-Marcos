# AUDIO_ARCHITECTURE — Arquitectura de Audio MarcosDashboard

**Versión:** 1.2  
**Fecha:** 2026-06-10  
**Estado:** Arquitectura final oficial

## 1) Arquitectura eléctrica aprobada

Se utilizará **una única placa ULN2803A** para proteger GPIOs de ESP32-S3 y STM32.

Ruta obligatoria:

`MCU GPIO (3.3V) -> ULN2803A -> entrada relé`

Ruta prohibida:

`MCU GPIO -> entrada relé (directo)`

## 2) Estado real verificado del Songle

- Módulo Songle 4CH con relés independientes.
- `IN3` conmuta `S3`; `IN4` conmuta `S4`.
- `IN3`-GND ≈ 5V en reposo (pull-up interno).
- `IN4`-GND ≈ 5V en reposo (pull-up interno).
- Al activar por sink, `INx` cae a ≈0V.
- Este comportamiento es normal y no representa fallo.

## 3) Mapeo definitivo de control

| Función | GPIO | ULN | Destino |
|---|---|---|---|
| LED frontal | PB10 | 1B/1C | IN1 relé LED frontal |
| LED trasero | PB11 | 2B/2C | IN2 relé LED trasero |
| Audio | GPIO11 (ESP32-S3) | 3B/3C | IN3 relé S3 |
| Potencia tracción | PC11 | 4B/4C | IN4 / relé potencia tracción |
| Potencia steering | PC12 | 5B/5C | relé potencia steering |

## 4) Audio (ruta analógica aprobada)

- Conmutación antes del PAM8403.
- Mezcla L/R por `1k + 1k`.
- Condensador de acoplo `1µF (105)`.
- Pull-down recomendado `100k` en entrada PAM8403.
- Contactos relé S3:
  - `NC`: módulo original Bluetooth/FM
  - `NO`: DFPlayer
  - `COM`: entrada PAM8403

## 5) Lógica firmware de audio (relay_audio.cpp)

`PIN_AUDIO_RELAY = GPIO11`:

- `GPIO11 LOW` = Audio OFF.
- `GPIO11 HIGH` = Audio ON.

Motivo: adaptación a ULN2803A (salida sink a GND en `IN3`).

## 6) Reglas de conexión ULN2803A

- GND común obligatorio: STM32 + ESP32-S3 + ULN2803A + Songle + relés externos.
- `COM` del ULN2803A: **sin conectar** cuando solo maneja `INx`.
- No se requieren resistencias externas en `IN3`.
- No se requieren diodos externos en `IN3`.

## 7) Soluciones descartadas como principal

Referencias **OBSOLETAS** para esta arquitectura:

- GPIO directo a Songle.
- BC547 como etapa principal.
- PC817 como etapa principal.
- Relé DPDT dedicado obligatorio.
- Cualquier topología previa incompatible con ULN2803A.

## 8) ARQUITECTURA APROBADA Y VALIDADA

La referencia única para futuras modificaciones es:

`STM32/ESP32 -> ULN2803A -> Relés`
