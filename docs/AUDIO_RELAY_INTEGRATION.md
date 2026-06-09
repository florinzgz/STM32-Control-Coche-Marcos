# AUDIO RELAY INTEGRATION — Módulo relay_audio (actualizado ULN2803A)

## Objetivo

Mantener el control no bloqueante del relé de audio en GPIO11, pero con etapa de protección:

`GPIO11 -> ULN2803A canal 3 -> Songle IN3`

## Polaridad vigente

Con ULN2803A:

- `GPIO11 HIGH` = relé audio ON.
- `GPIO11 LOW` = relé audio OFF.

## Máquina de estados

La máquina de estados no cambia (IDLE/ACTIVATING/ACTIVE/RELEASING).  
Solo cambia la polaridad de salida física en GPIO11.

## Temporizaciones vigentes

- `RELAY_ESTABLISH_MS = 20`
- `RELAY_RELEASE_MS = 150`
- `RELAY_MAX_ON_MS = 7000`

## Archivos involucrados

- `esp32/src/relay_audio.h`
- `esp32/src/relay_audio.cpp`
- `esp32/src/audio_manager.cpp` (sin cambio funcional)
