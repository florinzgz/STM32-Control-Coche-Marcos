# AUDIO_WIRING_GUIDE — Cableado Pin a Pin (oficial)

**Versión:** 1.2  
**Fecha:** 2026-06-10  
**Estado:** Arquitectura final aprobada y validada

## 1) Regla oficial única

Se utiliza **UNA única placa ULN2803A** para proteger GPIOs de ESP32-S3 y STM32.

Cadena obligatoria:

`GPIO MCU -> ULN2803A (Bx/Cx) -> entrada relé`

Prohibido:

`GPIO MCU -> entrada relé` (directo)

## 2) Mapeo definitivo de control de relés

| Función | MCU GPIO | ULN entrada | ULN salida | Destino relé |
|---|---|---|---|---|
| LED frontal | PB10 | 1B | 1C | IN1 Songle |
| LED trasero | PB11 | 2B | 2C | IN2 Songle |
| Audio | GPIO11 (ESP32-S3) | 3B | 3C | IN3 Songle (S3) |
| Potencia tracción | PC11 | 4B | 4C | IN4 Songle / entrada relé potencia |
| Potencia steering | PC12 | 5B | 5C | Entrada relé potencia steering |

## 3) Entradas Songle IN3/IN4 (comportamiento esperado)

- `IN3` e `IN4` presentan **pull-up interno a 5V**.
- Medición típica en reposo: `INx`-GND ≈ **5V**.
- Al activar por ULN (sink): `INx`-GND ≈ **0V**.
- Es normal y **no indica fallo**.

## 4) Canal de audio oficial

Ruta de control:

`ESP32 GPIO11 -> ULN2803A canal 3 -> IN3 Songle (relé S3)`

Lógica validada en firmware (`relay_audio.cpp`):

- `GPIO11 LOW` = Audio OFF (relé OFF).
- `GPIO11 HIGH` = Audio ON (relé ON).

Conexión de contactos de audio:

- `NC` = módulo original Bluetooth/FM.
- `NO` = DFPlayer.
- `COM` = entrada PAM8403.

## 5) Conexiones obligatorias de masa y ULN

- GND común obligatorio entre: **STM32, ESP32-S3, ULN2803A, Songle y relés externos**.
- `COM` del ULN2803A: **SIN CONECTAR** cuando solo se controlan entradas `INx`.

## 6) Validación con multímetro

### Audio OFF
- `IN3`-GND ≈ 5V
- continuidad `COM` ↔ `NC`

### Audio ON
- `IN3`-GND ≈ 0V
- continuidad `COM` ↔ `NO`

## 7) Prueba manual mínima

1. Alimentar Songle a 5V.
2. Poner jumper de `IN3` en modo `LOW`.
3. Tocar `IN3` a GND.
4. Verificar clic del relé.
5. Verificar cambio `COM-NC` / `COM-NO`.

## 8) Componentes de audio que se mantienen

- Mezcla L/R con resistencias `1k + 1k`.
- Condensador de acoplo `1µF` (código 105).
- Pull-down recomendado `100k` en entrada PAM8403.

## 9) Componentes NO necesarios en IN3

- BC547.
- PC817.
- Resistencias externas en `IN3`.
- Diodos externos en `IN3`.

## 10) Referencias obsoletas

Queda marcada como **OBSOLETA** cualquier referencia a:

- GPIO directo a Songle.
- BC547 o PC817 como solución principal de control.
- Relé DPDT dedicado obligatorio para esta arquitectura.

## 11) ARQUITECTURA APROBADA Y VALIDADA

Solución oficial y única para futuras modificaciones:

`STM32/ESP32 -> ULN2803A -> Relés`
