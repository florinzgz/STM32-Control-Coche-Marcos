# 🔧 HMI Audio Safety — Plan de Mitigación por Fases

**Versión:** 1.0  
**Fecha:** 2026-02-23  
**Basado en:** [HMI_AUDIO_SAFETY_REVIEW.md](HMI_AUDIO_SAFETY_REVIEW.md)  
**Alcance:** Plan de rollout seguro — sin código, solo planificación  
**Principio:** Un solo concepto de comportamiento auditivo por fase, verificable en vehículo real

---

## Índice

1. [Hallazgos ordenados por riesgo operacional](#1-hallazgos-ordenados-por-riesgo-operacional)
2. [Mapa de dependencias STM32 vs ESP32](#2-mapa-de-dependencias-stm32-vs-esp32)
3. [Fases de mitigación](#3-fases-de-mitigación)
4. [Resumen de fases](#4-resumen-de-fases)
5. [Primera fase segura para probar en movimiento](#5-primera-fase-segura-para-probar-en-movimiento)
6. [Criterios de aceptación global](#6-criterios-de-aceptación-global)

---

## 1. Hallazgos ordenados por riesgo operacional

Ordenados de mayor a menor riesgo **operacional real para el conductor** (no por severidad técnica genérica).

| Prioridad | ID | Hallazgo | Riesgo operacional para el conductor |
|-----------|-----|----------|--------------------------------------|
| **1** | F-04 | Burst collapse oculta EMERGENCY y OVERCURRENT → solo dice "error general" | 🔴 El conductor **nunca oye** "motor deshabilitado" ni "corriente excesiva" en el peor escenario. No sabe que el coche se ha detenido por seguridad ni por qué. |
| **2** | F-01 | `SENSOR_FAULT` mapea siempre a `SENSOR_SPEED_ERROR` — cubre pedal, encoder y velocidad | 🔴 Fallo de pedal o dirección anunciado como "sin señal de velocidad". El conductor recibe instrucciones incorrectas sobre qué hacer. |
| **3** | F-02 | Per-motor temp cutoff (130°C) sin audio | 🔴 Una rueda pierde tracción sin aviso. En curva, cambio brusco de trayectoria. |
| **4** | F-05 | Thresholds de temperatura ESP32 (85°C) vs STM32 (80°C) desincronizados | 🟠 Ventana de 5°C donde el coche ya limita potencia pero el conductor oye "error general" sin saber que es por temperatura. |
| **5** | F-03 | CAN bus-off sin mapping de audio en error codes | 🟠 Pérdida de comunicación ESP32↔STM32. Pero si CAN cae, el ESP32 no recibe frames igualmente — el problema es más profundo que el audio. |
| **6** | F-06 | Recovery DEGRADED→ACTIVE y LIMP_HOME→ACTIVE no producen SAFETY_RESET | 🟡 El conductor no sabe que el sistema se ha recuperado. Puede seguir conduciendo con precaución innecesaria. |
| **7** | F-07 | LIMP_HOME usa ERROR_GENERAL genérico — no indica que se puede conducir a 5 km/h | 🟡 El conductor puede detenerse por completo en un lugar no seguro creyendo que el coche está averiado. |
| **8** | F-12 | Startup inhibit y LIMP_HOME pedal arming sin audio | 🟡 Confusión cuando el pedal no responde, pero siempre ocurre en parado. |
| **9** | F-10 | STANDBY→ACTIVE sin confirmación auditiva | 🔵 El conductor no sabe cuándo el sistema está listo, pero la pantalla sí lo muestra. |
| **10** | F-08 | Tank turn (360°) solo reproduce BEEP | 🔵 Modo especial sin anuncio verbal, pero requiere acción deliberada del conductor. |
| **11** | F-11 | Single-slot pending queue pierde audio LOW simultáneo | 🔵 Solo afecta a eventos informativos (ABS_ON + TCS_ON en mismo ciclo). Sin impacto en seguridad. |
| **12** | SPAM-01/02/03 | ABS chattering, obstacle 2s repeat, gear rapid cycling | 🔵 Molestia auditiva, no riesgo de seguridad. |

---

## 2. Mapa de dependencias STM32 vs ESP32

| Cambio | Solo ESP32 | Requiere STM32 | Notas |
|--------|:----------:|:--------------:|-------|
| Corregir burst collapse (F-04) | ✅ | — | Lógica de `playWarning()` en `main.cpp` ESP32 |
| Diferenciar SENSOR_FAULT por subsistema (F-01) | ✅ parcial | ✅ preferible | ESP32 puede usar fault flags del byte 2 del heartbeat (ya enviados por STM32). Si los flags son insuficientes para distinguir pedal vs encoder vs velocidad, el STM32 necesita añadir un sub-código. |
| Audio per-motor cutoff 130°C (F-02) | ✅ parcial | ✅ necesario | STM32 debe enviar una flag o sub-código indicando cutoff per-motor. Actualmente el error code `OVERTEMP` no distingue entre warning global (80°C) y cutoff individual (130°C). |
| Sincronizar thresholds temperatura (F-05) | ✅ | — | Cambiar constantes en `main.cpp` ESP32: 85°C→80°C, hysteresis 80°C→75°C |
| CAN bus-off audio (F-03) | ✅ parcial | — | Añadir `CAN_BUSOFF` al switch de error codes en ESP32. Pero si CAN cae, la información nunca llega — se necesita también heartbeat timeout audio en ESP32. |
| Recovery audio DEGRADED/LIMP→ACTIVE (F-06) | ✅ | — | Ampliar condición en `main.cpp` ESP32 para trigger de `SAFETY_RESET` |
| Audio diferenciado LIMP_HOME (F-07) | ✅ | — | Nuevo branch en `main.cpp` para systemState == LIMP_HOME → audio específico (track existente o reasignar) |
| Startup inhibit audio (F-12) | ✅ parcial | ✅ preferible | El STM32 ya envía `status_flags` en el heartbeat. Si el bit de `startup_inhibit` está presente, ESP32 lo lee. Si no se envía, requiere cambio en STM32 `can_handler.c`. |
| STANDBY→ACTIVE confirmación (F-10) | ✅ | — | Detectar transición de estado en ESP32 |
| Tank turn verbal (F-08) | ✅ | — | Cambiar `BEEP` por audio verbal al activar modo 360° |
| ABS/TCS spam (SPAM-01) | ✅ | — | Suprimir ABS_OFF/TCS_OFF o añadir debounce |
| Obstacle escalamiento (SPAM-02) | ✅ | — | Modificar intervalo según zona |

### Resumen de impacto

| MCU | Fases que la tocan | Tipo de cambio |
|-----|-------------------|----------------|
| **ESP32 solamente** | Phase 1, 4, 5, 6, 7, 9, 10 | Lógica de audio en `main.cpp`, constantes en `main.cpp` |
| **ESP32 + posiblemente STM32** | Phase 2, 8 | ESP32: mapping de error codes. STM32: posible adición de fault flags en heartbeat CAN si los actuales son insuficientes. |
| **STM32 + ESP32** | Phase 3 | CAN heartbeat flags en `can_handler.c` + lógica de audio en `main.cpp` |

---

## 3. Fases de mitigación

---

### Phase 1 — Corregir burst collapse (F-04)

**Concepto auditivo modificado:** Lógica de agrupación de errores simultáneos (burst collapse)

**Justificación de prioridad:** Es el hallazgo **más grave** de la auditoría. El conductor nunca oye "motor deshabilitado" ni "corriente excesiva" en un escenario de overcurrent persistente. Solo oye "error general" repetido. Esto ocurre precisamente en el momento donde la información específica es más crítica.

**Qué cambiar:**
- Solo ESP32: modificar la función `playWarning()` en `main.cpp`
- En vez de reemplazar todos los errores del burst por `ERROR_GENERAL`, reproducir el sonido de **mayor severidad** del burst
- Orden de severidad para burst: `EMERGENCY (31)` > `OVERCURRENT (53)` > `BATTERY_CRITICAL (13)` > `SENSOR_*_ERROR` > `ERROR_GENERAL (3)`
- Mantener el contador de burst para diagnóstico, pero no usarlo para silenciar

**MCU afectada:** Solo ESP32

**Prueba física en vehículo:**
1. Vehículo en parado, ruedas elevadas (bancos/gato)
2. Provocar overcurrent sostenido en un motor (bloquear rueda manualmente con freno)
3. Esperar 3 ciclos consecutivos de overcurrent (→ transición DEGRADED → SAFE)
4. **Verificar:** Se oye "Modo de emergencia activado. Motor deshabilitado." (track 31) — **NO** "error general"
5. **Verificar alternativo:** Si EMERGENCY + OVERCURRENT llegan juntos, se oye EMERGENCY (mayor severidad), no ERROR_GENERAL

**Qué NO debe cambiar:**
- El cooldown de 4 segundos por track sigue activo
- La prioridad de interrupción (HIGH > MEDIUM > LOW) no se modifica
- Los audios de eventos únicos (no burst) siguen funcionando igual
- El cambio de marcha, luces, obstáculos — todos inalterados
- El estado del STM32 y el comportamiento de los motores no se tocan

**Criterio de éxito:** En un escenario de fallo real, el conductor oye el nombre específico del problema, no "error general".

**Riesgo de regresión:** BAJO — solo cambia qué sonido se elige dentro de un burst, no cuándo se disparan los sonidos.

---

### Phase 2 — Diferenciar SENSOR_FAULT por subsistema (F-01)

**Concepto auditivo modificado:** Mapping de error code SENSOR_FAULT → audio específico por sensor

**Justificación de prioridad:** Tres fallos diferentes (pedal, encoder dirección, velocidad rueda) producen el mismo audio incorrecto "sin señal de velocidad". El conductor recibe instrucciones erróneas.

**Qué cambiar:**
- **ESP32 (principal):** En el switch de error codes en `main.cpp`, cuando `errorCode == SENSOR_FAULT`:
  - Leer los fault flags del byte 2 del heartbeat CAN (ya contiene `FAULT_ENCODER_ERROR`, `FAULT_WHEEL_SENSOR`, etc.)
  - Si `FAULT_ENCODER_ERROR` → reproducir `ENCODER_ERROR (9)`: "Error en el sensor de dirección"
  - Si fault corresponde a pedal → reproducir `PEDAL_ERROR (5)`: "Error en el sensor del pedal"
  - Si `FAULT_WHEEL_SENSOR` → reproducir `SENSOR_SPEED_ERROR (35)`: "Sin señal de velocidad" (correcto solo para este caso)
  - Fallback si no se puede distinguir: mantener `SENSOR_SPEED_ERROR (35)` (no empeorar)

- **STM32 (si es necesario):** Si los fault flags actuales del heartbeat no distinguen suficientemente entre pedal, encoder y velocidad:
  - Añadir bits específicos en `status_flags` (byte 4 del heartbeat) para `FAULT_PEDAL_PLAUSIBILITY`
  - Esto requiere modificar `CAN_SendHeartbeat()` en `can_handler.c`

**MCU afectada:** ESP32 obligatorio. STM32 posiblemente necesario (depende de los fault flags actuales del heartbeat).

**Prueba física en vehículo:**
1. Vehículo en parado, pantalla encendida
2. **Test pedal:** Desconectar el cable I2C del ADS1115 (pedal externo). Esperar a que STM32 detecte plausibility failure.
   - **Verificar:** Se oye "Error en el sensor del pedal" (track 5) — **NO** "Sin señal de velocidad"
3. **Test encoder:** Desconectar el encoder de dirección (E6B2-CWZ6C). Esperar detección de fault.
   - **Verificar:** Se oye "Error en el sensor de dirección" (track 9) — **NO** "Sin señal de velocidad"
4. **Test velocidad (control):** Si hay sensor de velocidad de rueda, desconectar.
   - **Verificar:** Se oye "Sin señal de velocidad" (track 35) — sin cambio respecto al comportamiento actual

**Qué NO debe cambiar:**
- El audio de `CENTERING` error (encoder centering failed) sigue mapeando a `ENCODER_ERROR (9)` como antes
- El audio de `I2C_FAILURE` sigue mapeando a `SENSOR_CURRENT_ERROR (34)` como antes
- El audio de `OVERCURRENT` sigue mapeando a `OVERCURRENT (53)` — no se toca
- El comportamiento del STM32 (entrar en LIMP_HOME por SENSOR_FAULT) no cambia
- Los motores siguen aplicando torque cero por safety cuando corresponde

**Criterio de éxito:** Cada tipo de sensor genera su propio audio descriptivo.

**Riesgo de regresión:** MEDIO — si los fault flags se interpretan mal, podría reproducirse un audio incorrecto. Mitigación: el fallback mantiene el comportamiento actual.

---

### Phase 3 — Audio para per-motor temperature cutoff 130°C (F-02)

**Concepto auditivo modificado:** Nuevo aviso sonoro para pérdida de tracción asimétrica por sobretemperatura individual

**Justificación de prioridad:** Un motor individual pierde tracción sin que el conductor reciba ningún aviso. En curva a baja velocidad, esto cambia la trayectoria.

**Qué cambiar:**
- **STM32 (necesario):** En `motor_control.c`, cuando `wheel_scale[i]` se fuerza a 0.0 por temperatura > 130°C:
  - Establecer un bit o sub-flag específico en el heartbeat CAN indicando "per-motor cutoff activo"
  - Opción A: Usar un bit libre en `status_flags` (byte 4 del heartbeat)
  - Opción B: Añadir un nuevo error sub-code en la trama `STATUS_SAFETY (0x203)`
  - **No cambiar** el error code global (`OVERTEMP`) ni el estado del sistema

- **ESP32:** En `main.cpp`, cuando se detecte el nuevo flag de per-motor cutoff:
  - Reproducir `TEMP_HIGH (10)`: "Temperatura del motor elevada" con prioridad MEDIUM
  - Solo si no se ha reproducido ya en los últimos 4 segundos (cooldown existente)

**MCU afectada:** STM32 + ESP32

**Prueba física en vehículo:**
1. Vehículo en parado, ruedas elevadas
2. Calentar un motor (por carga sostenida) hasta que el STM32 aplique cutoff per-motor
   - Alternativa si no se puede calentar: inyectar temperatura simulada vía service mode o modificar temporalmente el threshold de cutoff a un valor alcanzable (ej: 60°C) solo para la prueba
3. **Verificar:** Se oye "Temperatura del motor elevada" (track 10) cuando el motor se apaga individualmente
4. **Verificar:** Los otros 3 motores siguen funcionando normalmente
5. **Verificar:** Si posteriormente la temperatura global sube a > 90°C y entra en SAFE, se oye `EMERGENCY (31)` como siempre

**Qué NO debe cambiar:**
- El threshold de cutoff per-motor sigue siendo 130°C en el STM32
- El threshold de SAFE global sigue siendo 90°C
- El comportamiento de `Safety_CheckTemperature()` para los estados DEGRADED/SAFE no cambia
- Los audios de EMERGENCY para sobretemperatura global siguen funcionando
- El heartbeat CAN sigue enviándose cada 100 ms con el mismo formato base

**Criterio de éxito:** El conductor recibe aviso verbal antes de notar la pérdida de tracción en una rueda.

**Riesgo de regresión:** MEDIO — requiere cambio en STM32 CAN handler. Mitigación: el nuevo flag es un bit adicional que no afecta a los campos existentes del heartbeat.

---

### Phase 4 — Sincronizar thresholds de temperatura ESP32 ↔ STM32 (F-05)

**Concepto auditivo modificado:** Momento en que suena el aviso de temperatura elevada (TEMP_HIGH)

**Justificación de prioridad:** El ESP32 avisa a 85°C pero el STM32 ya limita potencia a 80°C. Ventana de 5°C donde el conductor oye "error general" sin saber que es por temperatura.

**Qué cambiar:**
- Solo ESP32: En `main.cpp`, modificar las constantes de threshold de temperatura:
  - `TEMP_WARNING_THRESHOLD`: 85°C → **80°C** (coincide con STM32 `TEMP_WARNING_C`)
  - `TEMP_RECOVERY_THRESHOLD` (hysteresis): 80°C → **75°C** (coincide con STM32: 80 - 5 = 75)
- No hay nuevos tracks ni nuevas prioridades

**MCU afectada:** Solo ESP32

**Prueba física en vehículo:**
1. Vehículo en parado, monitorizar temperatura por pantalla HMI
2. Calentar un motor gradualmente (carga continua baja)
3. **Verificar a 80°C:** Se oye "Temperatura del motor elevada" (track 10) **simultáneamente** con la transición a DEGRADED del STM32
4. **Verificar a 75°C (enfriamiento):** Se oye "Temperatura del motor normalizada" (track 11) cuando la temperatura baja
5. **Verificar que NO suena a 85°C** como antes (ya habrá sonado a 80°C)

**Qué NO debe cambiar:**
- El STM32 sigue usando sus propios thresholds (80°C warning, 90°C critical, 130°C per-motor cutoff)
- El audio de EMERGENCY para > 90°C sigue funcionando
- La transición DEGRADED→SAFE por temperatura no se modifica
- Los thresholds de batería no se tocan
- El cooldown de 4 segundos sigue activo

**Criterio de éxito:** Cuando la potencia se limita por temperatura, el conductor oye "temperatura elevada" inmediatamente, no "error general" primero.

**Riesgo de regresión:** MUY BAJO — solo cambian dos constantes numéricas. El audio que suena es el mismo (track 10), solo cambia cuándo.

---

### Phase 5 — Audio para CAN bus-off y heartbeat timeout (F-03)

**Concepto auditivo modificado:** Aviso sonoro cuando se pierde la comunicación CAN con el STM32

**Justificación de prioridad:** Si CAN cae, el ESP32 no recibe datos actualizados. El conductor no tiene indicación auditiva de que la comunicación se ha perdido. Sin embargo, el riesgo práctico es menor que F-01/F-02/F-04 porque el STM32 actúa de forma autónoma (entra en LIMP_HOME) y la pantalla puede mostrar un indicador visual.

**Qué cambiar:**
- **ESP32 (principal):** Dos cambios:
  1. Añadir `CAN_BUSOFF (13)` al switch de error codes en `main.cpp` → reproducir `ERROR_GENERAL (3)` (o reasignar un track verbal como "Comunicación perdida con el controlador")
  2. Cuando el heartbeat timeout del ESP32 expire (ya detectado en `can_rx.cpp`), reproducir un audio de advertencia — por ejemplo `ERROR_GENERAL (3)` con prioridad HIGH, solo una vez (no repetir cada segundo)

**MCU afectada:** Solo ESP32

**Prueba física en vehículo:**
1. Vehículo encendido, CAN funcionando normalmente
2. **Test bus-off:** Desconectar el cable CAN-H o CAN-L del transceiver del ESP32
3. **Verificar:** Tras el timeout de heartbeat (~300-500ms), se oye un audio de advertencia
4. **Verificar:** El audio NO se repite indefinidamente (suena una vez, o máximo cada 10 segundos)
5. Reconectar CAN
6. **Verificar:** El sistema se recupera y no genera audio spam al reconectar

**Qué NO debe cambiar:**
- El STM32 sigue entrando en LIMP_HOME por su propio CAN bus-off detection
- El timeout del heartbeat en el ESP32 sigue siendo el mismo valor
- El comportamiento visual de la pantalla HMI no se modifica
- Audios de otros errores (overcurrent, temperatura, etc.) no se tocan

**Criterio de éxito:** El conductor oye un aviso cuando la comunicación CAN se pierde, y el aviso no se convierte en spam.

**Riesgo de regresión:** BAJO — se añade un caso nuevo al switch de error codes. El fallback (no matching) ya existe y no reproduce nada, por lo que el peor caso es que el nuevo caso no se active.

---

### Phase 6 — Recovery audio para DEGRADED y LIMP_HOME (F-06)

**Concepto auditivo modificado:** Confirmación sonora de que el sistema ha vuelto a estado normal desde DEGRADED o LIMP_HOME

**Justificación de prioridad:** Actualmente, `SAFETY_RESET` (track 32) solo suena al recuperar desde SAFE o ERROR. Si el sistema se recupera desde DEGRADED o LIMP_HOME, el conductor no recibe confirmación auditiva.

**Qué cambiar:**
- Solo ESP32: En `main.cpp`, ampliar la condición de trigger de `SAFETY_RESET`:
  - Condición actual: `lastState ∈ {SAFE, ERROR} && newState == ACTIVE` → play SAFETY_RESET
  - Nueva condición: `lastState ∈ {SAFE, ERROR, DEGRADED, LIMP_HOME} && newState == ACTIVE` → play SAFETY_RESET

**MCU afectada:** Solo ESP32

**Prueba física en vehículo:**
1. Vehículo en parado
2. **Test DEGRADED recovery:** Provocar un fallo menor temporal (ej: desconectar y reconectar rápidamente un sensor de temperatura)
   - STM32 entra en DEGRADED, luego recovery → ACTIVE
   - **Verificar:** Se oye "Reinicio de seguridad completado" (track 32) al volver a ACTIVE
3. **Test LIMP_HOME recovery:** Provocar CAN timeout temporal (desconectar CAN brevemente)
   - STM32 entra en LIMP_HOME, luego CAN se restaura → ACTIVE
   - **Verificar:** Se oye "Reinicio de seguridad completado" (track 32) al volver a ACTIVE
4. **Test control (no regresión):** Provocar overcurrent → SAFE → recovery
   - **Verificar:** SAFETY_RESET sigue sonando como antes

**Qué NO debe cambiar:**
- El audio de SAFETY_RESET sigue siendo el track 32 con la misma prioridad (MEDIUM)
- Las transiciones del STM32 no cambian
- La pantalla HMI no se modifica
- Los audios de error al entrar en DEGRADED/LIMP_HOME no se modifican

**Criterio de éxito:** El conductor siempre sabe cuándo el sistema vuelve a funcionar con normalidad, sin importar de qué estado viene.

**Riesgo de regresión:** MUY BAJO — se amplía una condición existente sin modificar la lógica.

---

### Phase 7 — Audio diferenciado para LIMP_HOME (F-07)

**Concepto auditivo modificado:** Mensaje verbal específico para el modo LIMP_HOME que indique al conductor que puede conducir a baja velocidad

**Justificación de prioridad:** Actualmente se oye "error general" al entrar en LIMP_HOME. El conductor puede creer que debe detenerse inmediatamente, cuando en realidad LIMP_HOME permite conducir a 5 km/h. Cambiar el audio da información operacional al conductor.

**Qué cambiar:**
- Solo ESP32: En `main.cpp`, cuando `systemState == LIMP_HOME`:
  - En vez de reproducir `ERROR_GENERAL (3)`, reproducir un audio más apropiado
  - **Opción A (sin nuevo track):** Reasignar uno de los tracks no utilizados (ej: `ENERGY_SAVE (60)` → regravar con "Modo de emergencia. Velocidad limitada a 5 kilómetros por hora.") y usarlo para LIMP_HOME
  - **Opción B (sin cambiar SD):** Reproducir `SOFT_START (56)` como placeholder temporal, que es descriptivo de "arranque limitado"
  - **Opción preferida:** Grabar un nuevo MP3 para un track no utilizado y usarlo

**MCU afectada:** Solo ESP32 (+ grabar nuevo audio en SD card)

**Prueba física en vehículo:**
1. Vehículo en parado
2. Provocar entrada en LIMP_HOME (ej: CAN timeout temporal, o pedal plausibility con un canal desconectado)
3. **Verificar:** Se oye el nuevo mensaje de LIMP_HOME (ej: "Modo de emergencia. Velocidad limitada.") — **NO** "error general"
4. **Verificar:** Si luego se pasa de LIMP_HOME a SAFE, se oye `EMERGENCY (31)` correctamente

**Qué NO debe cambiar:**
- El audio de DEGRADED (ERROR_GENERAL) no se modifica en esta fase — solo LIMP_HOME
- Los tracks de EMERGENCY, OVERCURRENT, BATTERY_CRITICAL no se tocan
- El comportamiento del STM32 en LIMP_HOME (20% torque, 5 km/h) no se modifica
- Los audios de marcha, luces, obstáculos no cambian

**Criterio de éxito:** Al entrar en LIMP_HOME, el conductor sabe que puede seguir conduciendo despacio.

**Riesgo de regresión:** BAJO — solo cambia qué track se reproduce para un estado específico.

---

### Phase 8 — Startup inhibit y LIMP_HOME pedal arming audio (F-12)

**Concepto auditivo modificado:** Aviso cuando el pedal debe soltarse antes de que el vehículo responda

**Justificación de prioridad:** El conductor puede confundir "pedal no responde" con avería. Siempre ocurre en parado, por lo que no hay riesgo de seguridad directo.

**Qué cambiar:**
- **ESP32:** Detectar la condición de startup inhibit desde los datos CAN:
  - Si el heartbeat incluye un flag `startup_inhibit` (bit en `status_flags` byte 4), reproducir `PEDAL_ERROR (5)`: "Error en el sensor del pedal" — o preferiblemente un audio más descriptivo como "Suelte el pedal para continuar"
  - Aplicar cooldown para no repetir cada 100ms
- **STM32 (si es necesario):** Si `status_flags` no incluye actualmente el bit de `startup_inhibit`:
  - Añadir el bit en `CAN_SendHeartbeat()` de `can_handler.c`
  - Coste mínimo: un OR de un bit en un byte ya existente

**MCU afectada:** ESP32 obligatorio. STM32 posiblemente necesario.

**Prueba física en vehículo:**
1. Vehículo apagado
2. Mantener el pedal presionado
3. Encender el vehículo
4. **Verificar:** Se oye un aviso de "suelte el pedal" (o "error en el sensor del pedal")
5. Soltar el pedal
6. **Verificar:** El aviso NO se repite y el sistema pasa a ACTIVE normalmente
7. **Verificar:** En arranque normal (sin pedal presionado), NO se oye ningún audio adicional

**Qué NO debe cambiar:**
- El comportamiento de startup_inhibit en el STM32 (bloquea torque hasta pedal < 3% durante 400ms)
- Los audios de bienvenida (WELCOME) y confirmación
- La secuencia de arranque del ESP32

**Criterio de éxito:** El conductor entiende inmediatamente que debe soltar el pedal.

**Riesgo de regresión:** BAJO — audio nuevo para condición nueva. No modifica audios existentes.

---

### Phase 9 — Confirmación STANDBY→ACTIVE y mejoras menores (F-10, F-08)

**Concepto auditivo modificado:** Confirmación sonora de "sistema listo" + anuncio verbal para modo tank turn

**Justificación de prioridad:** Mejoras de experiencia de usuario. Sin impacto en seguridad.

**Qué cambiar:**
- Solo ESP32:
  1. **STANDBY→ACTIVE:** Cuando `lastState == STANDBY && newState == ACTIVE`, reproducir `BEEP (68)` o `MODULE_OK (36)` con prioridad LOW como confirmación
  2. **Tank turn:** Cuando se activa el modo 360° tank turn, en vez de `BEEP (68)`, reproducir `TRACTION_4X4 (37)` (reutilización) o un track verbal no utilizado

**MCU afectada:** Solo ESP32

**Prueba física en vehículo:**
1. Encender el vehículo normalmente
2. **Verificar:** Tras `WELCOME (1)`, cuando el sistema pasa a ACTIVE se oye un beep de confirmación
3. **Verificar:** El beep NO interfiere con WELCOME (prioridad LOW, WELCOME es HIGH → WELCOME termina primero)
4. Activar modo tank turn
5. **Verificar:** Se oye un anuncio verbal, no solo un beep

**Qué NO debe cambiar:**
- WELCOME sigue sonando al arrancar
- Los audios de marcha y tracción no se modifican (excepto tank turn)
- Los errores y emergencias no se tocan

**Criterio de éxito:** El conductor sabe cuándo el sistema está listo, y sabe cuándo activa el modo tank turn.

**Riesgo de regresión:** MUY BAJO — solo se añaden triggers para estados que antes eran silenciosos.

---

### Phase 10 — Reducción de spam auditivo (SPAM-01, SPAM-02, SPAM-03)

**Concepto auditivo modificado:** Frecuencia y repetición de avisos informativos (ABS/TCS, obstáculos, marchas)

**Justificación de prioridad:** Última fase porque es solo calidad de experiencia, sin impacto en seguridad.

**Qué cambiar:**
- Solo ESP32:
  1. **ABS/TCS chattering (SPAM-01):** Suprimir audio de ABS_OFF y TCS_OFF — solo reproducir el de activación (ABS_ON/TCS_ON). La desactivación es el estado normal y no requiere notificación.
  2. **Obstacle escalamiento (SPAM-02):** Reducir intervalo de `OBSTACLE_WARN` de 2000ms a 500ms cuando zone == 4 (emergencia, < ~15cm). Mantener 2000ms para zone 3.
  3. **Gear rapid cycling (SPAM-03):** Añadir un debounce de 500ms antes de reproducir audio de marcha — si la marcha cambia de nuevo dentro de 500ms, cancelar el audio anterior y reproducir solo el de la marcha final.

**MCU afectada:** Solo ESP32

**Prueba física en vehículo:**
1. **Test ABS/TCS:** Conducir sobre superficie irregular (grava, hierba). Verificar que NO se oye "ABS desactivado" repetidamente. Solo "ABS activado" al inicio.
2. **Test obstáculo:** Acercar la mano al sensor a < 15cm. Verificar que el beep se acelera (cada 500ms en vez de 2s).
3. **Test marchas:** Cambiar rápidamente P→R→D. Verificar que solo se oye el audio de la marcha final (D1), no tres audios cortados.

**Qué NO debe cambiar:**
- ABS_ON y TCS_ON siguen sonando normalmente
- OBSTACLE_WARN sigue sonando para zone ≥ 3
- Las marchas se confirman correctamente (solo la final)
- Los cooldowns de 4 segundos para otros tracks no se modifican

**Criterio de éxito:** La experiencia auditiva es limpia sin alertas spam, manteniendo toda la información relevante.

**Riesgo de regresión:** BAJO — solo afecta a audios informativos (LOW priority).

---

## 4. Resumen de fases

| Phase | Hallazgo | Concepto modificado | MCU | Riesgo regresión | Test en movimiento |
|-------|----------|---------------------|-----|-------------------|--------------------|
| **1** | F-04 | Burst collapse → priorizar audio específico | ESP32 | BAJO | ❌ En parado |
| **2** | F-01 | SENSOR_FAULT → audio por subsistema | ESP32 (+STM32?) | MEDIO | ❌ En parado |
| **3** | F-02 | Per-motor cutoff 130°C → audio | STM32 + ESP32 | MEDIO | ❌ En parado |
| **4** | F-05 | Threshold temp ESP32 85→80°C | ESP32 | MUY BAJO | ❌ En parado |
| **5** | F-03 | CAN bus-off → audio advertencia | ESP32 | BAJO | ❌ En parado |
| **6** | F-06 | Recovery DEGRADED/LIMP→ACTIVE → SAFETY_RESET | ESP32 | MUY BAJO | ❌ En parado |
| **7** | F-07 | LIMP_HOME → audio específico "velocidad limitada" | ESP32 | BAJO | ⚠️ Verificable en movimiento |
| **8** | F-12 | Startup inhibit → "suelte el pedal" | ESP32 (+STM32?) | BAJO | ❌ En parado |
| **9** | F-10, F-08 | STANDBY→ACTIVE confirmación + tank turn verbal | ESP32 | MUY BAJO | ⚠️ Verificable en movimiento |
| **10** | SPAM-* | ABS/TCS spam, obstáculo escalamiento, gear debounce | ESP32 | BAJO | ✅ Requiere movimiento |

---

## 5. Primera fase segura para probar en movimiento

### ✅ Phase 7 es la primera fase que puede (y debe) verificarse en movimiento

**Justificación:**
- Las Phases 1–6 se verifican **completamente en parado** (ruedas elevadas o provocando fallos con cables).
- Phase 7 (audio diferenciado LIMP_HOME) es la primera que se beneficia de una prueba en movimiento porque:
  - LIMP_HOME permite conducción a 5 km/h con torque al 20%
  - El conductor necesita confirmar que entiende el mensaje "velocidad limitada" mientras conduce
  - El test consiste en provocar un CAN timeout breve mientras el vehículo se mueve a baja velocidad (< 3 km/h) y verificar el audio

**Precondiciones obligatorias para Phase 7 en movimiento:**
1. ✅ Phases 1–6 completadas y verificadas en parado
2. ✅ El burst collapse corregido (Phase 1) — garantiza que si algo sale mal, el audio es informativo
3. ✅ SENSOR_FAULT diferenciado (Phase 2) — si hay un sensor fault durante la prueba, el audio será correcto
4. ✅ Recovery audio funciona (Phase 6) — al recuperar de LIMP_HOME, el conductor oye confirmación
5. ✅ Superficie plana, sin tráfico, velocidad < 5 km/h
6. ✅ Copiloto o segundo operador presente para provocar el fallo controlado

**Protocolo de prueba en movimiento:**
1. Conducir en línea recta a < 3 km/h
2. Copiloto desconecta momentáneamente el bus CAN (cable CAN-H)
3. **Verificar:** Se oye el nuevo audio de LIMP_HOME (ej: "Velocidad limitada a 5 kilómetros por hora")
4. **Verificar:** El vehículo reduce velocidad suavemente (torque al 20%)
5. Copiloto reconecta CAN
6. **Verificar:** Se oye `SAFETY_RESET` (track 32) — Phase 6 confirma la recuperación
7. **Verificar:** El vehículo recupera potencia normal

### ⚠️ Phase 10 requiere prueba en movimiento para los tests de ABS/TCS y marchas en conducción

---

## 6. Criterios de aceptación global

Antes de pasar de una fase a la siguiente, se deben cumplir **todos** los siguientes criterios:

| # | Criterio | Verificación |
|---|----------|-------------|
| G-01 | El cambio de la fase actual funciona según su prueba física | Test manual completado y documentado |
| G-02 | Todos los audios de fases anteriores siguen funcionando | Re-test de regresión de fases previas |
| G-03 | El comportamiento de los motores no ha cambiado | Verificar PWM, enable, direction en osciloscopio o pantalla de debug |
| G-04 | El heartbeat CAN sigue llegando cada 100ms | Verificar con pantalla de debug ESP32 o CAN analyzer |
| G-05 | No hay audios nuevos inesperados en operación normal | Conducir 2 minutos en condiciones normales sin provocar fallos |
| G-06 | El DFPlayer no se bloquea (responde a comandos) | Verificar que `setVolume()` funciona después de cada prueba |

### Criterio de rollback

Si una fase falla la verificación:
1. Revertir **solo** los cambios de esa fase
2. Verificar que el comportamiento vuelve al de la fase anterior
3. Analizar la causa raíz antes de reintentar
4. **No continuar a la siguiente fase hasta resolver**

---

*Fin del plan de mitigación.*  
*Este documento NO contiene código. La implementación de cada fase debe seguirse como un PR independiente verificable.*
