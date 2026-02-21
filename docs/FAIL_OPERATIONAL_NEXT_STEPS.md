# Checklist de Auditoría: Próximos Pasos del Roadmap Fail-Operational

**Tipo:** Auditoría de Seguridad Funcional
**Fecha:** 2026-02-21
**Revisión:** 1.0
**Firmware actual:** STM32-Control-Coche-Marcos (STM32G474RE)
**Firmware original de referencia:** FULL-FIRMWARE-Coche-Marcos v2.17.1
**Documento base:** `docs/FAIL_OPERATIONAL_MIGRATION_AUDIT.md` (Fase 4 — Plan de Migración)

---

## Estado de la Migración

| Paso | Nombre | Estado |
|------|--------|--------|
| 1 | Documentar decisión relay-en-SAFE | ⚠️ PENDIENTE (parcial) |
| 2 | STANDBY→LIMP_HOME sin centering | ✅ COMPLETADO |
| 3 | Bypass servicio centering para ACTIVE | ✅ COMPLETADO |
| 4 | Pedal plausibility fail-operational | ✅ COMPLETADO |
| 5 | Driver sensor obstáculo ESP32 | ✅ COMPLETADO |
| 6 | Persistencia flash calibración dirección | ✅ COMPLETADO |
| 7 | Corte de relés en SAFE | ❌ PENDIENTE (bloqueado por Paso 1) |

**Evidencia de los pasos completados:**

- **Paso 2:** `Core/Src/safety_system.c` líneas 964-975 — `Safety_CheckCANTimeout()` permite STANDBY→LIMP_HOME sin `Steering_IsCalibrated()`, solo requiere `BootValidation_IsPassed()`.
- **Paso 4:** `Core/Src/sensor_manager.c` — `Pedal_IsContradictory()` (línea 292) implementada; `Core/Src/boot_validation.c` — `SAFETY_ERROR_SENSOR_FAULT` permitido en validación de arranque; `Core/Src/safety_system.c` — LIMP_HOME con pedal primario degradado.
- **Paso 5:** `esp32/src/sensors/obstacle_sensor.cpp` — Driver HC-SR04 con warmup, stuck-detection, rango 20-4000mm; `esp32/src/can/can_obstacle.cpp` — TX CAN 0x208 a 66ms.
- **Paso 6:** `Core/Src/steering_cal_store.c` — Persistencia flash en página 126 (0x0807E000) con CRC32; `SteeringCal_Save()` con HAL_FLASH_Program; `SteeringCal_ValidateAtBoot()` valida contra sensor inductivo físico.
- **Paso 3:** `Core/Src/safety_system.c` — `Safety_CheckCANTimeout()` guardas STANDBY→ACTIVE (línea 992) y LIMP_HOME→ACTIVE (línea 1003) modificadas: `(Steering_IsCalibrated() || !ServiceMode_IsEnabled(MODULE_STEER_CENTER))`. Si MODULE_STEER_CENTER (ID 19) está deshabilitado via CAN 0x110, el requisito de centering se omite. Comportamiento por defecto (módulo habilitado) no cambia.

---

## Pasos Pendientes — Checklist Obligatoria de Implementación

### Paso 1 — Verificar y Documentar Decisión Relay-en-SAFE

**1. Nombre del paso:**
Verificar y documentar formalmente la decisión arquitectónica sobre el comportamiento de los relés en estado SAFE.

**2. Qué comportamiento original restaura o protege:**
En el firmware original (`SafetyManager.cpp`, estado EMERGENCY), la acción de emergencia invocaba `ESP.restart()` que implícitamente cortaba toda alimentación a periféricos. El sistema actual mantiene los relés energizados en SAFE para permitir recuperación sin reinicio. Esta decisión debe documentarse formalmente como aceptación de riesgo o debe cambiarse.

**3. Evidencia (archivo y función concreta del firmware actual):**
- `Core/Src/safety_system.c` → `Safety_FailSafe()` (líneas 1280-1295): Llama a `Traction_EmergencyStop()` (PWM=0, EN pins LOW) pero **NO** llama a `Relay_PowerDown()`.
- `Core/Src/safety_system.c` → `Safety_PowerDown()` (líneas 1297-1301): **Única** función que llama a `Relay_PowerDown()`, invocada solo desde `Safety_SetState(SYS_STATE_ERROR)`.
- `Core/Src/safety_system.c` → `Safety_SetState(SYS_STATE_SAFE)` (línea ~270): Llama a `Safety_FailSafe()` sin corte de relés.

**4. Diferencia concreta con el firmware original:**
- **Original:** EMERGENCY → `ESP.restart()` → corte implícito total (reinicio del MCU).
- **Actual:** SAFE → `Traction_EmergencyStop()` → PWM=0, EN=LOW, **relés ON**. Solo ERROR → `Relay_PowerDown()`.
- **Riesgo R1 documentado:** Si un BTS7960 entra en latch-up (EN=LOW pero corriente persiste), los relés energizados permiten flujo de corriente al motor en SAFE.

**5. Archivos que deberán tocarse:**
- `docs/` — Nuevo documento o sección en documento existente con la decisión formal.
- Ningún archivo de código fuente (este paso es solo documentación y prueba física).

**6. Qué está prohibido modificar:**
- `Core/Src/safety_system.c` — No modificar `Safety_FailSafe()` ni `Safety_SetState()`.
- `Core/Src/motor_control.c` — No modificar `Traction_EmergencyStop()`.
- Ningún `.c` ni `.h` en este paso.
- Ningún ID CAN ni formato de payload.

**7. Procedimiento de prueba física real en el vehículo:**
1. Poner el vehículo en estado ACTIVE con ESP32 conectado.
2. Provocar SAFE inyectando sobrecorriente sostenida >25A (3 fallos consecutivos) mediante carga resistiva en un motor.
3. Verificar con multímetro que la bobina de relé principal mantiene tensión (relés ON).
4. Verificar con osciloscopio que las señales PWM en TIM1 CH1-4 están a 0% duty.
5. Verificar con multímetro que los pines EN de los BTS7960 están en LOW.
6. Medir corriente en terminales del motor: debe ser 0A.
7. Verificar CAN: 0x001 byte 1 = 0x04 (SAFE), 0x201 corrientes = 0A, 0x205 escalas = 0x00.
8. **Decisión:** Si la corriente del motor es 0A con EN=LOW y relés ON → documentar que EN-pin es suficiente. Si hay corriente residual → Paso 7 es obligatorio.

---

### Paso 3 — Bypass de Servicio para STANDBY→ACTIVE sin Centering

**1. Nombre del paso:**
Implementar bypass de servicio para permitir STANDBY→ACTIVE cuando MODULE_STEER_CENTER está deshabilitado en Service Mode.

**2. Qué comportamiento original restaura o protege:**
El firmware original (`calibration_menu.cpp`) proporcionaba un menú manual de calibración como alternativa cuando el sensor inductivo de centro fallaba. El sistema actual no tiene alternativa: si el sensor inductivo falla, `Steering_IsCalibrated()` permanece en `false` y el vehículo queda inmovilizado en STANDBY indefinidamente (nunca alcanza ACTIVE, solo puede ir a LIMP_HOME si se pierde CAN). Este paso restaura la capacidad del firmware original de operar con calibración manual/override cuando el hardware de centering falla.

**3. Evidencia (archivo y función concreta del firmware actual):**
- `Core/Src/safety_system.c` → `Safety_CheckCANTimeout()` (líneas 981-985):
  ```c
  if (system_state == SYS_STATE_STANDBY &&
      safety_error == SAFETY_ERROR_NONE &&
      Steering_IsCalibrated() &&        // ← BLOQUEO sin bypass
      BootValidation_IsPassed()) {
      Safety_SetState(SYS_STATE_ACTIVE);
  }
  ```
- `Core/Src/service_mode.c` (línea 57): `MODULE_STEER_CENTER` definido como `MODULE_CLASS_NON_CRITICAL` (deshabilitatable).
- `Core/Inc/service_mode.h` (línea 71): `MODULE_STEER_CENTER` enumerado.
- **Gap:** `MODULE_STEER_CENTER` existe y puede deshabilitarse via CAN 0x110, pero deshabilitarlo **no afecta** la guarda `Steering_IsCalibrated()` en la transición STANDBY→ACTIVE.

**4. Diferencia concreta con el firmware original:**
- **Original:** Menú de calibración manual permitía operar sin sensor de centro. El operador podía establecer el punto cero manualmente.
- **Actual:** Sin sensor inductivo → centering nunca completa → `Steering_IsCalibrated()` = false → STANDBY permanente (con ESP32) o LIMP_HOME (sin ESP32). No existe ruta a ACTIVE sin centering exitoso.
- **Impacto:** Sensor inductivo dañado/desconectado → vehículo inmovilizado a plena capacidad. Solo LIMP_HOME (20% potencia, 5 km/h) disponible.

**5. Archivos que deberán tocarse:**
- `Core/Src/safety_system.c` — Modificar guarda en `Safety_CheckCANTimeout()` (líneas 981-985) para que verifique `ServiceMode_IsEnabled(MODULE_STEER_CENTER)`: si MODULE_STEER_CENTER está deshabilitado, omitir `Steering_IsCalibrated()`.
- `Core/Inc/service_mode.h` — Posible adición de comentario clarificador (sin cambio funcional).

**6. Qué está prohibido modificar:**
- `Core/Src/motor_control.c` — No modificar `Steering_IsCalibrated()` ni `Steering_SetCalibrated()`.
- `Core/Src/steering_centering.c` — No modificar la lógica de centering.
- `Core/Src/steering_cal_store.c` — No modificar la persistencia flash.
- `Core/Src/boot_validation.c` — No modificar las 6 comprobaciones de arranque.
- Comportamiento por defecto: con MODULE_STEER_CENTER habilitado (por defecto), el comportamiento actual **no debe cambiar**.
- La transición STANDBY→LIMP_HOME sin centering (Paso 2) **no debe alterarse**.
- IDs CAN, formatos de payload, tasas de transmisión.

**7. Procedimiento de prueba física real en el vehículo:**
1. **Test A — Bypass activo:**
   - Desconectar cable del sensor inductivo de centro.
   - Encender con ESP32 conectado enviando heartbeat.
   - Centering falla (timeout 10s, entra en CENTERING_FAULT).
   - Enviar CAN 0x110: module_id=MODULE_STEER_CENTER, action=DISABLE.
   - Verificar CAN 0x001 byte 1 = 0x02 (ACTIVE) dentro de 1s.
   - Verificar CAN 0x204 byte 2 (calibrated) = 0x00 (no calibrado).
   - Enviar steering CAN 0x101 → verificar que PID de dirección responde (posición relativa a zero desconocido).
   - Enviar throttle → verificar que tracción responde a plena capacidad (D1=60%, D2=100%).
2. **Test B — Sin bypass (comportamiento por defecto preservado):**
   - Desconectar sensor inductivo.
   - Encender con ESP32 conectado, NO enviar comando de disable.
   - Verificar que el vehículo permanece en STANDBY (byte 1 = 0x01) indefinidamente.
   - Desconectar ESP32 → CAN timeout → LIMP_HOME (byte 1 = 0x06).
3. **Test C — Re-habilitación:**
   - Desde Test A (ACTIVE sin centering).
   - Enviar CAN 0x110: module_id=MODULE_STEER_CENTER, action=ENABLE.
   - Verificar que la guarda de centering se restaura.
   - Si centering no se ha completado → transición a DEGRADED o permanece ACTIVE (según estado de error).

---

### Paso 7 — Implementar Corte de Relés en SAFE (Condicional al Resultado del Paso 1)

**1. Nombre del paso:**
Añadir `Relay_PowerDown()` en `Safety_FailSafe()` si el Paso 1 determina que EN-pin no es suficiente.

**2. Qué comportamiento original restaura o protege:**
En el firmware original, la acción EMERGENCY cortaba toda alimentación mediante `ESP.restart()` (reinicio completo). Si las pruebas del Paso 1 demuestran que el BTS7960 puede conducir corriente con EN=LOW (latch-up), este paso protege contra torque no intencionado en estado SAFE eliminando la alimentación de potencia a través de los relés.

**3. Evidencia (archivo y función concreta del firmware actual):**
- `Core/Src/safety_system.c` → `Safety_FailSafe()` (líneas 1280-1295): Solo llama a `Traction_EmergencyStop()` (PWM=0, EN=LOW). No llama a `Relay_PowerDown()`.
- `Core/Src/safety_system.c` → `Safety_PowerDown()` (líneas 1297-1301): Llama a `Relay_PowerDown()`. Solo invocada desde `Safety_SetState(SYS_STATE_ERROR)`.
- `Core/Src/motor_control.c` → `Relay_PowerDown()`: Desactiva GPIO de los tres relés (principal, tracción, dirección) en secuencia.

**4. Diferencia concreta con el firmware original:**
- **Original:** EMERGENCY → `ESP.restart()` → corte total implícito.
- **Actual:** SAFE → relés ON, solo PWM y EN cortados. ERROR → relés OFF.
- **Trade-off:** Con relés OFF en SAFE, la recuperación automática (SAFE→ACTIVE) requiere re-energizado de relés (~70ms delay) y el vehículo pierde la capacidad de auto-recuperarse sin ciclo de alimentación.

**5. Archivos que deberán tocarse:**
- `Core/Src/safety_system.c` — Solo la función `Safety_FailSafe()`: añadir llamada a `Relay_PowerDown()` después de `Traction_EmergencyStop()`.

**6. Qué está prohibido modificar:**
- `Core/Src/safety_system.c` → `Safety_SetState()` — No modificar la lógica de transiciones.
- `Core/Src/safety_system.c` → `Safety_PowerDown()` — No modificar; es para ERROR.
- `Core/Src/safety_system.c` → `Safety_EmergencyStop()` — No modificar.
- `Core/Src/motor_control.c` → `Relay_PowerDown()` — No modificar la implementación.
- `Core/Src/motor_control.c` → `Traction_EmergencyStop()` — No modificar.
- Transición SAFE→ACTIVE: Si se implementa este paso, se debe decidir si SAFE→ACTIVE requiere ciclo de alimentación o si se añade `Relay_PowerUp()` en la recuperación.
- IDs CAN, formatos de payload.

**7. Procedimiento de prueba física real en el vehículo:**
1. Provocar SAFE inyectando sobrecorriente sostenida >25A (3 fallos consecutivos).
2. Verificar con multímetro que la bobina del relé principal pierde tensión (relés OFF).
3. Verificar que la corriente del motor es 0A.
4. Verificar CAN: 0x001 byte 1 = 0x04 (SAFE).
5. Restaurar heartbeat CAN desde ESP32 → verificar que el sistema **permanece en SAFE** (no auto-recupera sin relés).
6. Ciclo de alimentación → verificar arranque limpio a STANDBY (0x001 byte 1 = 0x01).
7. **Regresión:** Repetir Test T5 (CAN loss recovery) del plan de validación para confirmar que LIMP_HOME no se ve afectado (LIMP_HOME NO llama a `Safety_FailSafe()`).

---

## Orden Obligatorio de Implementación

```
Paso 1 → Paso 3 → Paso 7 (condicional)
  │         │         │
  │         │         └─ Solo si Paso 1 determina que EN-pin no es suficiente
  │         └─ Independiente del resultado de Paso 1
  └─ DEBE completarse ANTES de Paso 7
```

**Bloqueos:**
- Paso 7 está bloqueado por Paso 1 (necesita resultado de prueba física).
- Paso 3 puede ejecutarse en paralelo con Paso 1 (sin dependencias).
- Pasos 2, 4, 5, 6 ya están completados — no requieren acción.

---

## Resumen de Riesgos Abiertos

| Riesgo | Severidad | Paso Mitigador | Estado |
|--------|-----------|----------------|--------|
| R1 — SAFE no corta relés | ALTA | Paso 1 (decisión) + Paso 7 (implementación) | Documentado, pendiente decisión |
| R3 — Centering bloquea ACTIVE | MEDIA | Paso 2 (✅) + Paso 3 (✅) | Mitigado — bypass servicio disponible via CAN 0x110 |
| R2 — Sin persistencia NVM calibración | MEDIA | Paso 6 (✅) | Mitigado |
| R6 — Sin sensor obstáculo ESP32 | MEDIA | Paso 5 (✅) | Mitigado |
| R9 — ADS1115 fallback pedal | BAJA-MEDIA | Paso 4 (✅) | Mitigado |
| R4 — D1=60% nuevo | BAJA-MEDIA | Documentación | Aceptado por diseño |
| R5 — Sin regen AI | BAJA | Diferido Fase 5 roadmap | Aceptado |
| R7 — ESP32 requerido para ACTIVE | BAJA | LIMP_HOME por diseño | Aceptado por diseño |
| R8 — consecutive_errors no persiste | BAJA | MATCH con original | Aceptado |

---

## Metodología de Auditoría

Esta checklist se generó mediante:

1. **Análisis del firmware actual:** Lectura completa de los 18 archivos fuente en `Core/Src/` y 16 headers en `Core/Inc/`.
2. **Comparación con comportamiento documentado:** Verificación línea por línea contra `docs/ORIGINAL_REPO_COMPARATIVE_AUDIT.md` (Fase 2: Behavioral Comparison).
3. **Referencia cruzada con roadmap:** Cada paso verificado contra `docs/FAIL_OPERATIONAL_MIGRATION_AUDIT.md` (Fase 4: Migration Plan, Steps 1-7).
4. **Verificación de implementación:** Búsqueda de evidencia de cada paso en el código fuente (funciones, guardas, módulos service mode, persistencia flash).
5. **Regla aplicada:** No se inventaron features nuevas, no se optimizó arquitectura, no se saltaron fases, no se modificó diseño existente, no se escribió código.
