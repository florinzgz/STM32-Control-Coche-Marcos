# AUDIO RELAY INTEGRATION — Módulo relay_audio (arquitectura final)

## Objetivo oficial

Controlar el relé de audio con protección eléctrica de GPIO:

`ESP32 GPIO11 -> ULN2803A canal 3 -> Songle IN3 (S3)`

## Regla de integración

- Siempre: `GPIO -> ULN2803A -> INx`.
- Nunca: `GPIO -> INx` directo.

## Lógica final OFF/ON (validada)

`relay_audio.cpp` está invertido para ULN2803A:

| GPIO11 | ULN canal 3 | IN3 | Estado relé audio |
|---|---|---|---|
| LOW | OFF | ≈5V (pull-up interno) | OFF |
| HIGH | ON (sink) | ≈0V | ON |

Interpretación:
- `IN3` a ~5V en reposo es normal.
- `IN3` a ~0V activado es normal.

## Esquema eléctrico de control

```
ESP32 GPIO11 ──► ULN2803A 3B
                ULN2803A 3C ──► Songle IN3

GND ESP32 ─┬─ GND ULN2803A
           └─ GND Songle
```

`COM` del ULN2803A: **sin conectar** (solo se conmutan entradas INx).

## Conmutación de audio (contactos del relé S3)

- `NC`  = fuente original Bluetooth/FM
- `NO`  = DFPlayer
- `COM` = entrada PAM8403

## Validación con multímetro

### Audio OFF
- `IN3`-GND ≈ 5V
- continuidad `COM ↔ NC`

### Audio ON
- `IN3`-GND ≈ 0V
- continuidad `COM ↔ NO`

## Prueba manual recomendada

1. Alimentar Songle a 5V.
2. Poner jumper de `IN3` en `LOW`.
3. Tocar `IN3` a GND.
4. Confirmar clic del relé.
5. Confirmar cambio `COM-NC` / `COM-NO`.

## Componentes y límites

- No se requieren BC547/PC817 en control de `IN3`.
- No se requieren resistencias ni diodos externos en `IN3`.
- Se mantiene mezcla `1k + 1k`, condensador `1µF (105)` y pull-down recomendado `100k` en entrada PAM8403.

## Máquina de estados y tiempos (sin cambios)

- Estados: `IDLE`, `ACTIVATING`, `ACTIVE`, `RELEASING`.
- `RELAY_ESTABLISH_MS = 20`
- `RELAY_RELEASE_MS = 150`
- `RELAY_MAX_ON_MS = 7000`

## Referencias obsoletas

Se consideran **OBSOLETAS** las referencias a:
- GPIO directo a Songle.
- BC547 o PC817 como solución principal.
- Arquitectura previa incompatible con ULN2803A.

## ARQUITECTURA APROBADA Y VALIDADA

La referencia oficial y única para el control es:

`STM32/ESP32 -> ULN2803A -> Relés`
