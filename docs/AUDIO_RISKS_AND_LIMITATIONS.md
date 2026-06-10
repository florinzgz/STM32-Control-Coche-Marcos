# AUDIO_RISKS_AND_LIMITATIONS — Riesgos y límites (arquitectura final)

**Versión:** 1.2  
**Fecha:** 2026-06-10

## 1) Riesgos críticos

1. **Bypass ULN2803A (prohibido):** no conectar GPIO directo a `INx`.
2. **Sin GND común:** control de relés errático o no funcional.
3. **Polaridad de audio incorrecta:** en hardware final, `GPIO11 HIGH` activa y `GPIO11 LOW` desactiva.
4. **Conmutación tras PAM8403:** no permitido; debe ser antes del amplificador.
5. **Eliminar red analógica recomendada (1k+1k, 1µF, pull-down 100k):** aumenta ruido/pops.

## 2) Límites funcionales confirmados

- `IN3` en reposo ≈ 5V por pull-up interno (normal).
- `IN3` activado ≈ 0V por sink del ULN (normal).
- `COM` del ULN2803A queda sin conectar cuando solo controla entradas `INx`.
- Watchdog y temporizaciones de `relay_audio` se mantienen (`7000/20/150 ms`).

## 3) Validación obligatoria con multímetro

### Audio OFF
- `IN3`-GND ≈ 5V
- continuidad `COM ↔ NC`

### Audio ON
- `IN3`-GND ≈ 0V
- continuidad `COM ↔ NO`

## 4) Prueba manual obligatoria

1. Alimentar Songle a 5V.
2. Configurar jumper de `IN3` en `LOW`.
3. Tocar `IN3` a GND.
4. Verificar clic.
5. Verificar cambio `COM-NC` / `COM-NO`.

## 5) Componentes no necesarios para IN3

- BC547.
- PC817.
- Resistencias externas en IN3.
- Diodos externos en IN3.

## 6) Lista previa a energizar

- [ ] Confirmar mapeo: PB10, PB11, GPIO11, PC11, PC12 -> ULN2803A -> relés.
- [ ] Confirmar `GPIO11 -> canal 3 ULN -> IN3`.
- [ ] Confirmar GND común STM32/ESP32-S3/ULN/Songle/relés externos.
- [ ] Confirmar lógica audio: LOW=OFF, HIGH=ON.
- [ ] Confirmar contactos audio: NC=Bluetooth/FM, NO=DFPlayer, COM=PAM8403.

## 7) Referencias obsoletas

Se marcan como **OBSOLETAS**:
- GPIO directo a Songle.
- BC547 o PC817 como solución principal.
- Requisito de relé DPDT dedicado obligatorio.
- Arquitectura anterior incompatible con ULN2803A.

## 8) ARQUITECTURA APROBADA Y VALIDADA

Solución oficial y única:

`STM32/ESP32 -> ULN2803A -> Relés`
