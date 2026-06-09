# AUDIO_RISKS_AND_LIMITATIONS — Riesgos y límites (actualizado)

**Versión:** 1.1  
**Fecha:** 2026-06-09

## Riesgos críticos

1. **Bypass del ULN2803A (prohibido):** conectar GPIO directo a `INx` Songle expone el MCU a líneas con pull-up a 5V.
2. **Cableado GND incompleto:** sin GND común, el control de relés será inestable.
3. **Polaridad de audio desactualizada:** tras ULN, `GPIO11 HIGH` activa el relé de audio.
4. **Conmutar audio después de PAM8403:** no permitido; la conmutación debe ser previa al amplificador.
5. **Eliminar la red 1k+1k + 105(1µF):** incrementa pops y distorsión.

## Límites funcionales confirmados

- El watchdog del relé de audio (`RELAY_MAX_ON_MS=7000`) sigue activo.
- La temporización anti-click (`RELAY_ESTABLISH_MS=20`, `RELAY_RELEASE_MS=150`) sigue activa.
- Los relés STM32 PB10/PB11/PC11/PC12 continúan con lógica `GPIO_PIN_SET=ON`.

## Lista previa a energizar

- [ ] Verificar continuidad de 1B..5B con GPIO correctos.
- [ ] Verificar 1C..5C a entradas correctas de relé.
- [ ] Verificar IN3 en Songle para canal audio (S3).
- [ ] Verificar GND común ULN/ESP32/STM32/Songle.
- [ ] Verificar lógica de audio: reposo `GPIO11 LOW`, activo `GPIO11 HIGH`.
