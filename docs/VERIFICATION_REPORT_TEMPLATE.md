# Informe único de validación HW/Firmware — STM32 ↔ ESP32

> **Fecha:** ____/____/________  
> **Operador:** ____________________  
> **Versión firmware STM32:** ____________________  
> **Versión firmware ESP32:** ____________________  
> **Fuente principal del proceso:** `docs/HARDWARE_VALIDATION_PROCEDURE.md`

---

## 0) Reglas de uso (obligatorio)

- [ ] Se usa este documento como **único** registro de resultados (sin listas paralelas).
- [ ] Se siguió el orden completo 1→12 definido en `HARDWARE_VALIDATION_PROCEDURE.md`.
- [ ] No se documentaron componentes pasivos como “definitivos” antes de cerrar criterios de salida.
- [ ] Todas las observaciones están basadas en mediciones reales / logs / firmware verificado.

---

## 1) Preparación del banco (TEST ENVIRONMENT SETUP)

### 1.1 Montaje y seguridad

- [ ] STM32 conectado y monitor serie operativo
- [ ] ESP32 conectado y monitor serie operativo
- [ ] Bus CAN cableado con terminación correcta
- [ ] Alimentación principal validada
- [ ] Vehículo elevado / ruedas libres / parada de emergencia accesible

### 1.2 Mediciones base

```
GND diff:       _____ V  (esperado: 0–0.05 V)
3V3 STM:        _____ V  (esperado: 3.20–3.40 V)
3V3 ESP:        _____ V  (esperado: 3.20–3.40 V)
Vbat sistema:   _____ V
```

Resultado bloque 1: [ ] PASS  [ ] FAIL  
Observaciones: ______________________________________________________________

---

## 2) BOOT VALIDATION (tests 2.1 → 2.6)

| Test | Resultado | Evidencia |
|---|---|---|
| 2.1 Reset cause STM32 | PASS / FAIL | |
| 2.2 Reset cause ESP32 | PASS / FAIL | |
| 2.3 Watchdog behavior | PASS / FAIL | |
| 2.4 FDCAN failure no brick | PASS / FAIL | |
| 2.5 I2C failure no brick | PASS / FAIL | |
| 2.6 Main loop always reached | PASS / FAIL | |

Resultado bloque 2: [ ] PASS  [ ] FAIL  
Observaciones: ______________________________________________________________

---

## 3) CAN COMMUNICATION VALIDATION (tests 3.1 → 3.4)

| Test | Resultado | Evidencia |
|---|---|---|
| 3.1 Heartbeat behavior | PASS / FAIL | |
| 3.2 Loss of ESP32 handling | PASS / FAIL | |
| 3.3 STM32 safe fallback | PASS / FAIL | |
| 3.4 Message timing | PASS / FAIL | |

Resultado bloque 3: [ ] PASS  [ ] FAIL  
Observaciones: ______________________________________________________________

---

## 4) STEERING VALIDATION (tests 4.1 → 4.7)

| Test | Resultado | Evidencia |
|---|---|---|
| 4.1 Encoder counting | PASS / FAIL | |
| 4.2 Centering procedure | PASS / FAIL | |
| 4.3 Centering fault handling | PASS / FAIL | |
| 4.4 Range protection | PASS / FAIL | |
| 4.5 Fault jump detection | PASS / FAIL | |
| 4.6 Fault frozen detection | PASS / FAIL | |
| 4.7 Fault out-of-range detection | PASS / FAIL | |

Resultado bloque 4: [ ] PASS  [ ] FAIL  
Observaciones: ______________________________________________________________

---

## 5) MOTOR & TRACTION SAFETY (tests 5.1 → 5.5)

| Test | Resultado | Evidencia |
|---|---|---|
| 5.1 No movement at boot | PASS / FAIL | |
| 5.2 Relay sequencing | PASS / FAIL | |
| 5.3 Brake and neutral behavior | PASS / FAIL | |
| 5.4 Invalid input reaction | PASS / FAIL | |
| 5.5 Emergency stop | PASS / FAIL | |

Resultado bloque 5: [ ] PASS  [ ] FAIL  
Observaciones: ______________________________________________________________

---

## 6) SENSOR FAILSAFE BEHAVIOR (tests 6.1 → 6.6)

| Test | Resultado | Evidencia |
|---|---|---|
| 6.1 Missing temperature sensors | PASS / FAIL | |
| 6.2 Missing current sensors (I2C) | PASS / FAIL | |
| 6.3 I2C recovery after failure | PASS / FAIL | |
| 6.4 Missing encoder | PASS / FAIL | |
| 6.5 Stuck pedal detection | PASS / FAIL | |
| 6.6 Missing wheel speed sensor | PASS / FAIL | |

Resultado bloque 6: [ ] PASS  [ ] FAIL  
Observaciones: ______________________________________________________________

---

## 7) DISPLAY CONSISTENCY (tests 7.1 → 7.5)

| Test | Resultado | Evidencia |
|---|---|---|
| 7.1 Boot screen CAN link status | PASS / FAIL | |
| 7.2 Screen transitions by STM32 state | PASS / FAIL | |
| 7.3 Safe screen fault flags | PASS / FAIL | |
| 7.4 Error screen on critical fault | PASS / FAIL | |
| 7.5 Drive screen live telemetry | PASS / FAIL | |

Resultado bloque 7: [ ] PASS  [ ] FAIL  
Observaciones: ______________________________________________________________

---

## 8) Cierre de criterios de salida (sección 8 del procedimiento)

| Criterio | Resultado | Evidencia |
|---|---|---|
| #1 | PASS / FAIL | |
| #2 | PASS / FAIL | |
| #3 | PASS / FAIL | |
| #4 | PASS / FAIL | |
| #5 | PASS / FAIL | |
| #6 | PASS / FAIL | |
| #7 | PASS / FAIL | |
| #8 | PASS / FAIL | |
| #9 | PASS / FAIL | |
| #10 | PASS / FAIL | |
| #11 | PASS / FAIL | |
| #12 | PASS / FAIL | |
| #13 | PASS / FAIL | |
| #14 | PASS / FAIL | |
| #15 | PASS / FAIL | |
| #16 | PASS / FAIL | |
| #17 | PASS / FAIL | |

Resultado de salida de fase: [ ] PASS  [ ] FAIL

---

## 9) Gate de documentación de componentes pasivos (obligatorio)

- [ ] Se verifica que `Resultado de salida de fase = PASS`
- [ ] Si no es PASS, no se actualiza como definitivo `docs/COMPONENTES_PASIVOS_REFERENCIA.md`
- [ ] Si es PASS, cada cambio de diodo/resistencia/condensador incluye:
  - [ ] evidencia observada de validación
  - [ ] referencia de firmware/documento de respaldo
  - [ ] sin duplicar inventarios ni crear listas alternativas

---

## 10) Veredicto final y firmas

- [ ] **PASS** — Proceso completo ejecutado y criterios de salida cerrados.
- [ ] **FAIL** — Existe al menos un bloque/criterio no superado.

**Acción siguiente (si FAIL):** ______________________________________________

| Rol | Nombre | Fecha | Firma |
|---|---|---|---|
| Tester | | | |
| Revisor | | | |
| Responsable proyecto | | | |
