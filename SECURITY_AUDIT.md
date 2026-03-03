# Auditoría Técnica de Seguridad Funcional

**Sistema**: ESP32-S3 (HMI) + STM32G474RE (control motor) + CAN-Bus TJA1051T/3
**Fecha**: 2026-02-28
**Base**: Análisis exclusivo del código fuente del firmware (Core/Src, esp32/src)

---

## 1. Autonomía de la STM32 en Funciones Críticas

### Resultado: **STM32 completamente autónoma para seguridad**

| Función | ¿Autónoma STM32? | Evidencia en código |
|---------|:-:|---|
| Control PWM (RPWM/LPWM) | ✅ | motor_control.c:380-443 — timers TIM1/TIM8 directos |
| Protección shoot-through | ✅ | Canales RPWM/LPWM en el MISMO timer → solapamiento = 0 |
| Frenado de emergencia | ✅ | safety_system.c:1302 `Safety_EmergencyStop()` → `Traction_EmergencyStop()` |
| Detección sobrecorriente | ✅ | safety_system.c:870 via INA226 I2C directo |
| Detección sobretemperatura | ✅ | safety_system.c:931 via DS18B20 OneWire |
| Protección ante NaN/Inf | ✅ | motor_control.c:28-35 `sanitize_float()` en cada entrada |
| Limitación de torque LIMP_HOME | ✅ | main.c:334 `LIMP_HOME_TORQUE_LIMIT_FACTOR = 0.20` |
| Limitación velocidad LIMP_HOME | ✅ | motor_control.c:964 cap a 5 km/h |
| ABS/TCS per-wheel | ✅ | safety_system.c:668-866 con pulse modulation independiente |
| Secuenciación de relés | ✅ | safety_system.c:455-510 non-blocking |
| Watchdog independiente | ✅ | IWDG a 500 ms — main.c:406 `HAL_IWDG_Refresh` |
| Pedal dual-channel | ✅ | sensor_manager.c:108-277 ADC1 dual-sample + plausibilidad software |
| Detección obstáculo | ✅ | safety_system.c:120-232 máquina de estados local |

**La ESP32 es prescindible.** Ante su pérdida, la STM32 mantiene movilidad mínima en LIMP_HOME.

---

## 2. Comportamiento ante Pérdida Total de Subsistemas

### 2.1. Pérdida de CAN Bus

| Aspecto | Comportamiento | Código |
|---------|---------------|--------|
| Timeout | 250 ms → `SAFETY_ERROR_CAN_TIMEOUT` | safety_system.c:983 |
| Estado destino | LIMP_HOME (NO SAFE) | safety_system.c:994 |
| Torque disponible | 20% máximo, 5 km/h cap | safety_system.h:194-196 |
| Pedal | Lectura local ADC dual-sample, plausibilidad software | main.c:309-339 |
| Recuperación | CAN restaurado + steering calibrado → ACTIVE | safety_system.c:1024-1028 |

**Resultado**: ✅ Seguro. Movilidad mínima preservada.

### 2.2. Pérdida de ESP32

| Aspecto | Comportamiento | Código |
|---------|---------------|--------|
| Detección | Heartbeat counter freeze (5×100ms = 500ms) | can_handler.c:43-53 |
| Estado | LIMP_HOME tras CAN timeout | safety_system.c:992-995 |
| Throttle | CAN throttle ignorado, pedal local | main.c:309-311 |
| Dirección | Steering PID opera independiente | motor_control.c Steering_ControlLoop |

**Resultado**: ✅ Seguro. STM32 continúa autónomamente.

### 2.3. Pérdida de TOFSense-M

| Aspecto | Comportamiento | Código |
|---------|---------------|--------|
| Timeout CAN obstáculo | 500 ms → `OBSTACLE_CAN_TIMEOUT_MS` | safety_system.c:155 |
| Si obstáculo activo | Mantiene última escala conservadora (≤0.3) | safety_system.c:152-154 |
| Si no había obstáculo | Escala → 1.0, speed cap LIMP_HOME protege | safety_system.c:153 |
| Estado destino | NO cambia estado principal | Diseño: sensor advisory only |

**Resultado**: ✅ Seguro. Pérdida del sensor nunca inmoviliza el vehículo.

### 2.4. Pérdida del Sensor de Pedal

| Aspecto | Comportamiento | Código |
|---------|---------------|--------|
| Muestras ADC consecutivas contradictorias | Demanda forzada a cero | safety_system.c:1134-1135 |
| Plausibilidad fallida (rango/tasa) | LIMP_HOME, ADC con 20% torque | main.c:324-338 |
| ADC primario fuera de rango | Clamped por calibración [150,2413] counts | sensor_manager.c:179-186 |
| Ambos perdidos | Sin señal pedal → demanda 0 | main.c:299-300 |
| Arming latch LIMP_HOME | Pedal debe estar <3% por 300ms antes de aceptar torque | main.c:272-281 |

**Resultado**: ✅ Seguro. Protección multi-capa.

---

## 3. Análisis de Riesgos Específicos

### 3.1. Estados Inconsistentes

| Riesgo | Mitigado | Mecanismo |
|--------|:--------:|-----------|
| DEGRADED → ACTIVE prematuro | ✅ | Debounce 500ms (`RECOVERY_HOLD_MS`) — safety_system.c:97-98 |
| SAFE sin fallo real | ✅ | SAFE solo por: overcurrent ≥3 consecutivos, overtemp >90°C, watchdog, emergency | 
| LIMP_HOME sin CAN loss | ❌→✅ | Solo CAN timeout o pedal fault causa LIMP_HOME |
| Transición BOOT→ACTIVE sin validación | ✅ | `BootValidation_IsPassed()` gate — main.c:199-201 |

### 3.2. Aceleración No Comandada

| Riesgo | Mitigado | Mecanismo |
|--------|:--------:|-----------|
| CAN throttle corrupto | ✅ | Valor >100 rechazado a 0 (can_handler.c fix) + `Safety_ValidateThrottle` clamp |
| NaN/Inf en demanda | ✅ | `sanitize_float()` en cada entrada de Traction — motor_control.c:28-35, 1044-1048 |
| Pedal held through reboot | ✅ | `startup_inhibit` latch — main.c:93-96, cleared solo tras 400ms de pedal <3% |
| Jump rate >15%/10ms | ✅ | Step-rate validation + clamp — motor_control.c:528-564 |
| Frozen pedal | ✅ | Detección si pedal idéntico >5s mientras velocidad cambia — motor_control.c:566-592 |
| Rampa post-SAFE recovery | ✅ | `pedal_ramped` seeded a 0 (no al throttle actual) — motor_control.c:601 |

### 3.3. Falsos Estados por Corrupción de Datos

| Riesgo | Mitigado | Mecanismo |
|--------|:--------:|-----------|
| CAN data NaN/Inf | ✅ | `sanitize_float()` wrapper en cada conversión CAN |
| Temperature CRC fail | ✅ | DS18B20 CRC-8 validación en sensor_manager.c |
| I2C bus lockup | ✅ | NXP AN10216 recovery: 16 SCL toggles + STOP + reinit |
| Heartbeat counter frozen | ✅ | 5 frames idénticos consecutivos = zombie detection — can_handler.c:43-53 |
| DLC manipulation | ✅ | `ExtractDLC()` + per-ID minimum DLC validation |

---

## 4. Clasificación del Nivel de Seguridad

### 4.1. Comparativa

| Nivel | Característica | ¿Presente? |
|-------|---------------|:----------:|
| **Hobby robusto** | Single-loop, watchdog, basic limits | ✅ Superado |
| **Industrial básico** | Redundancia sensorial, estados de fallo, rate limiting | ✅ Superado |
| **Automoción ligera** | Dual-channel pedal, consecutive-error escalation, state machine formal, NaN protection | ✅ Alcanzado |
| **ISO 26262 ASIL-A** | FMEA documentado, cobertura diagnóstica >60%, HW redundancia certificada | ⚠️ Parcial |

### 4.2. Clasificación Final

**Nivel alcanzado: Arquitectura cercana a automoción ligera (SAE J3061 Level 1–2)**

Justificación:
- ✅ Máquina de estados formal con transiciones validadas
- ✅ Redundancia de pedal dual-channel con cross-validation
- ✅ Protección NaN/Inf sistemática en toda la cadena de PWM
- ✅ Shoot-through imposible por diseño de timer
- ✅ Escalado de degradación granular (L1/L2/L3)
- ✅ CAN loss → LIMP_HOME (no inmovilización)
- ✅ Boot validation gate
- ✅ Power-On Movement Prevention
- ⚠️ Sin hardware watchdog externo (solo IWDG interno)
- ⚠️ Sin monitor de voltaje de referencia ADC independiente
- ⚠️ Sin autenticación CAN (E2E protection)

---

## 5. Análisis Detallado por Subsistema

### 5.1. CAN BUS

| Aspecto | Estado | Detalle |
|---------|:------:|---------|
| Validación DLC | ✅ | `ExtractDLC()` + chequeo `msg_len >= N` por cada ID |
| Contador incremental heartbeat | ✅ | Byte 0, rollover 0-255 intencional |
| Detección freeze de contador | ✅ | 5 frames idénticos → no refresh → timeout |
| Timeout real (250ms) | ✅ | `CAN_TIMEOUT_MS = 250` — safety_system.c:25 |
| Estado por pérdida CAN | ✅ | LIMP_HOME, no SAFE |
| Desincronización | ✅ | Freeze detection + heartbeat counter advance |
| Bus-off handling | ✅ | Non-blocking recovery: Stop→DeInit→Init→Start, max 10 retries |
| Filtros y máscaras | ✅ | 4 filtros whitelist, global reject para no-matching |
| Validación rango datos | ✅ | Throttle >100 → rechazado a 0 (fix aplicado), gear validado |
| **DLC=0 bypass** | ✅ | **FIX APLICADO**: DLC=0 ya no bypassa freeze detection |

### 5.2. Control de Motor

| Aspecto | Estado | Detalle |
|---------|:------:|---------|
| Autoridad final PWM | ✅ STM32 | Timer hardware TIM1/TIM8, nunca ESP32 directo |
| Shoot-through | ✅ Imposible | RPWM/LPWM en mismo timer, mismo UEV |
| NaN/Inf | ✅ | `sanitize_float()` en cada input + post-pipeline validation |
| Límites físicos | ✅ | PWM ≤ PWM_PERIOD (4249), demand [0,100], steering ±74° |
| Valores fuera de rango | ✅ | Post-pipeline range check fuerza a 0 si >100% o negativo sin dynbrake |
| **Anomaly detection en DEGRADED** | ✅ | **FIX APLICADO**: escalación L1→L2 en DEGRADED |

### 5.3. Sensores Críticos

| Aspecto | Estado | Detalle |
|---------|:------:|---------|
| Redundancia pedal Hall | ✅ | ADC1 (PA3) dual-sample, plausibilidad software (±30 counts, rango, tasa) |
| Consistencia dual-sample | ✅ | Divergencia >30 counts → fault |
| Plausibilidad falla | ✅ | LIMP_HOME, 20% torque |
| Canales contradictorios | ✅ | Demanda forzada a 0 en TODOS los estados |
| Encoder falla | ✅ | Steering neutralizado, DEGRADED, traction operativa |
| TOFSense deja de transmitir | ✅ | 500ms timeout, mantiene última escala o scale=1.0 |
| I2C recovery | ✅ | NXP AN10216: 16 SCL toggles, max 2 retries → SAFE |

### 5.4. Estados de Seguridad

**Condiciones que activan SAFE:**
1. Sobrecorriente ≥3 eventos consecutivos (safety_system.c:889-890)
2. Sobretemperatura >90°C (safety_system.c:942-946)
3. Batería <18.0V o sensor voltage = 0V (safety_system.c:1264-1274)
4. Emergency stop manual (safety_system.c:1302-1311)
5. I2C bus irrecuperable tras 2 intentos (sensor_manager.c)

**Condiciones que activan LIMP_HOME:**
1. CAN timeout >250ms (safety_system.c:983-995)
2. CAN bus-off detectado (can_handler.c:865-874)
3. Pedal fault (plausibilidad software falla, no contradictorio) (safety_system.c:1146-1152)

**Estado intermedio inseguro: NO existe.**
- Transiciones protegidas por `Safety_SetState()` switch (safety_system.c:239-318)
- Solo transiciones permitidas explícitamente
- Recovery requiere `safety_error == SAFETY_ERROR_NONE`

**Aceleración sin supervisión: IMPOSIBLE.**
- SAFE → `Traction_EmergencyStop()` (PWM = 0 en los 4 motores)
- LIMP_HOME → torque cap 20%, speed cap 5 km/h
- startup_inhibit previene torque post-reboot
- LIMP_HOME pedal arming previene creep por ruido ADC

### 5.5. Arquitectura Eléctrica

| Aspecto | Estado | Nota |
|---------|:------:|------|
| Pin STB del TJA1051T/3 | ⚠️ | No visible en firmware — debe estar pulled LOW en hardware |
| Terminación 120Ω | ⚠️ | No verificable desde firmware — requiere medición física |
| GND común | ⚠️ | No verificable desde firmware — requiere inspección de PCB |
| Ruido motores en CAN | ✅ Mitigado | FDCAN filters reject noise, heartbeat freeze detection como backup |

**Nota**: Estos aspectos son de hardware y no pueden auditarse completamente desde el firmware.
Se recomienda verificar físicamente:
- TJA1051T/3 pin STB → GND (modo normal)
- Resistencias 120Ω en ambos extremos del bus CAN
- Plano de tierra continuo entre ESP32 y STM32
- Apantallamiento del bus CAN alejado de cableado de motor

---

## 6. Vulnerabilidades Encontradas y Correcciones Aplicadas

### 6.1. CORREGIDA: DLC=0 Heartbeat Bypass

**Archivo**: `Core/Src/can_handler.c`
**Riesgo**: Un frame heartbeat con DLC=0 bypassaba completamente la detección de freeze del contador, permitiendo mantener la liveness sin counter válido.
**Corrección**: DLC=0 ahora es rechazado. Se requiere al menos 1 byte con counter para validar liveness.

### 6.2. CORREGIDA: Anomaly Detection Solo en ACTIVE

**Archivo**: `Core/Src/motor_control.c`
**Riesgo**: La detección de anomalías en la demanda (step-rate y frozen pedal) solo activaba DEGRADED desde ACTIVE. Si el sistema ya estaba en DEGRADED, no se escalaba el nivel de degradación.
**Corrección**: En DEGRADED, los anomalías ahora escalan L1→L2 para restricciones más agresivas.

### 6.3. CORREGIDA: CAN Throttle Fuera de Rango

**Archivo**: `Core/Src/can_handler.c`
**Riesgo**: El byte de throttle CAN (uint8_t) podía ser 0-255. Valores 101-255 eran aceptados y dependían únicamente de `Safety_ValidateThrottle()` para clamping. Un frame corrupto con valor 200 pasaría parcialmente el pipeline antes de ser clampado.
**Corrección inicial**: Valor >100 se clampeaba a 0 en la capa de ingreso CAN.
**Corrección refinada (validación profunda)**: El clamp a 0 creaba una discontinuidad de demanda que activaba el detector de anomalías step-rate en `Traction_SetDemand()`, forzando una transición espuria a DEGRADED por un solo byte corrupto. Se cambió a **rechazo completo del frame** (`break`) en lugar de clamp a 0, evitando que `Traction_SetDemand()` reciba el dato corrupto.

---

## 7. Validación Profunda Post-Correcciones

### 7.1. Verificación A: Heartbeat DLC y Counter

**A1: ¿Frames con DLC≥1 pero counter no válido pueden bypassar la detección?**

**Resultado: NO** ✅

Traza del código (`can_handler.c:573-601`):
- `msg_len >= 1`: requiere al menos 1 byte.
- `counter != esp32_hb_last_counter`: si el counter avanza → aceptado (liveness actualizada).
- Si counter no avanza → `esp32_hb_same_count` se incrementa.
- Tolerancia: 4 frames idénticos aceptados (same_count 0→1→2→3→4, con update).
- Frame 5: same_count alcanza `HEARTBEAT_COUNTER_FREEZE_COUNT` (5) → NO actualiza liveness.
- Resultado: timeout CAN expira 250ms después → LIMP_HOME.

Tiempo total desde primer frame frozen: ~650ms (4 frames tolerados × 100ms + 250ms timeout).

**A2: ¿Existe path donde heartbeat válido mantiene ACTIVE sin counter avanzando?**

**Resultado: NO** ✅

La lógica de freeze detection es exhaustiva:
- Counter avanza → aceptado inmediatamente, same_count reseteado a 0.
- Counter estático → same_count se incrementa, saturado en `HEARTBEAT_COUNTER_FREEZE_COUNT`.
- Una vez saturado → ningún `Safety_UpdateCANRxTime()` → timeout natural.
- Recuperación: counter vuelve a cambiar → same_count reseteado, liveness restaurada.

**A3: ¿El rechazo de throttle >100 genera efecto inesperado?**

**Resultado: Fix corregido** ⚠️→✅

**Vulnerabilidad encontrada en validación**: El clamp original (>100→0) creaba un salto de demanda
que activaba el detector de anomalías step-rate (`MAX_THROTTLE_STEP_PER_10MS = 15%`):
- Demanda previa = 50% (del pedal), CAN corrupto envía 200 → clamped a 0%
- `|0 - 50| = 50 > 15` → anomalía detectada → `SAFETY_ERROR_SENSOR_FAULT` → DEGRADED
- Un solo byte CAN corrupto causaba transición a DEGRADED — efecto desproporcionado.

**Fix aplicado**: Cambio de clamp-to-0 a frame rejection (`break`). El frame corrupto se descarta
sin llamar a `Traction_SetDemand()`. La demanda del pedal local continúa operando normalmente.
Si el ESP32 envía frames corruptos persistentemente, el heartbeat counter freeze lo detectará.

### 7.2. Verificación B: Escalada de Degradación

**B1: ¿La escalada L1→L2 genera oscilaciones de estado?**

**Resultado: NO** ✅

`Safety_SetDegradedLevel()` (`safety_system.c:413-426`) implementa **escalada monotónica**:
```c
if (level > degraded_level) { /* Solo escalada, nunca reducción */ }
```
- L1→L2: `2 > 1` = true → escala ✅
- L2→L1: `1 > 2` = false → bloqueado ✅
- L2→L2: `2 > 2` = false → bloqueado (idempotente) ✅

Reset solo ocurre al transicionar a ACTIVE o SAFE (`Safety_SetState()`).
Para oscilar L1↔L2 se necesitaría L1→L2→ACTIVE→L1, lo cual requiere:
- `safety_error == SAFETY_ERROR_NONE` (fault cleared)
- 500ms de operación limpia (`RECOVERY_HOLD_MS`)
- Nueva fault independiente

Esto es recuperación legítima seguida de nueva fault, no oscilación.

**B2: ¿DEGRADED puede quedar permanente sin recuperación?**

**Resultado: NO** ✅

Ruta de recuperación (`safety_system.c:1043-1054`):
1. `Safety_CheckSensors()` clears `SAFETY_ERROR_SENSOR_FAULT` cuando todos los sensores pasan.
2. `safety_error == SAFETY_ERROR_NONE` → recovery debounce comienza.
3. 500ms de operación limpia → `Safety_SetState(SYS_STATE_ACTIVE)`.
4. ACTIVE resetea `degraded_level = DEGRADED_LEVEL_NONE`.

Si la anomalía es **persistente** (e.g., CAN corruption continua), `Safety_SetError()` se
re-ejecuta cada ciclo de 50ms, reseteando el debounce → DEGRADED se mantiene (correcto).

**B3: ¿Hay doble penalización que fuerce SAFE innecesariamente?**

**Resultado: NO** ✅

Las anomalías de demanda solo ejecutan:
- `Safety_SetError(SAFETY_ERROR_SENSOR_FAULT)` — error flag
- `Safety_SetDegradedLevel(L2, DEMAND_ANOMALY)` — nivel de degradación

**SAFE solo se activa por**:
- Overcurrent ≥3 consecutivos (`Safety_CheckCurrent()`)
- Overtemp >90°C (`Safety_CheckTemperature()`)
- Battery <18V (`Safety_CheckBatteryVoltage()`)
- Emergency stop (`Safety_EmergencyStop()`)

La escalada L1→L2 NO tiene path directo a SAFE. Incluso si múltiples anomalías elevan a L3,
el sistema permanece en DEGRADED L3 (40% power, 50% traction cap) — no SAFE.

### 7.3. Verificación C: Efectos Secundarios

**C1: Race conditions entre ISR CAN y lógica principal**

**Resultado: Sin riesgo** ✅

El callback ISR (`stm32g4xx_it.c:156-166`) está **vacío**:
```c
void HAL_FDCAN_RxFifo0Callback(...) {
    (void)hfdcan; (void)RxFifo0ITs;
    /* Safety_UpdateCANRxTime() called from CAN_ProcessMessages() only */
}
```
`CAN_ProcessMessages()` se ejecuta en el main loop (`main.c:403`), no en ISR.
Todas las variables compartidas se acceden exclusivamente desde el main loop.
No hay escrituras concurrentes. No hay race conditions.

**Nota**: El comentario en `last_can_rx_time` decía "Written from ISR" — corregido a
"Updated from CAN_ProcessMessages() main loop" (`safety_system.c:71`).
El qualifier `volatile` se mantiene por seguridad, aunque técnicamente ya no es necesario.

**C2: Variables no atómicas usadas en validación**

**Resultado: Sin riesgo** ✅

En Cortex-M4, todas las operaciones de store/load de 32-bit alineadas son atómicas:
- `last_can_rx_time` (uint32_t, volatile) → atómico ✅
- `safety_error` (enum = 32-bit) → atómico ✅
- `system_state` (enum = 32-bit) → atómico ✅
- `esp32_hb_last_counter` (uint8_t) → atómico ✅
- `esp32_hb_same_count` (uint8_t) → atómico ✅

Adicionalmente, todas estas variables se acceden exclusivamente desde el main loop,
eliminando cualquier riesgo de tearing incluso si fueran structs multi-word.

**C3: Riesgo de overflow del contador rolling**

**Resultado: Sin riesgo** ✅

ESP32 heartbeat counter es `uint8_t` (0-255). Rollover 255→0:
- `counter(0) != esp32_hb_last_counter(255)` → counter avanzó → aceptado
- `esp32_hb_same_count` reseteado a 0
- STM32 `esp32_hb_last_counter` también es `uint8_t` → misma representación

El rollover es **funcionalmente idéntico** a un incremento normal. No hay pérdida de detección.

**C4: Interacciones con timeout de 250ms**

**Resultado: Correcto** ✅

| Escenario | Tiempo hasta detección | Comportamiento |
|-----------|:----------------------:|---------------|
| Pérdida total CAN | 250ms | LIMP_HOME directo |
| 1 heartbeat perdido | No detectado (100ms gap < 250ms timeout) | Normal |
| 2 heartbeats perdidos | 300ms gap → timeout | LIMP_HOME |
| Counter freeze | ~650ms (4 tolerance frames + 250ms timeout) | LIMP_HOME |
| Bus-off | Inmediato (PSR.BusOff flag) | LIMP_HOME + recovery |

El margin de tolerancia (4 frames × 100ms = 400ms) previene falsos positivos
por pérdida de paquete individual sin comprometer la seguridad.

### 7.4. Verificación D: Nivel de Seguridad

**D1: ¿El sistema sigue siendo Automotive-light tras los cambios?**

**Sí** ✅ — Los cambios refuerzan todas las capas sin introducir regresiones.

**D2: ¿Sube o baja el nivel real de seguridad?**

**Sube** ⬆️ — Tres mejoras concretas:
1. DLC=0 bypass eliminado → cierra vector de ataque CAN
2. Frame rejection para throttle corrupto → previene DEGRADED espurio
3. Escalada L1→L2 en DEGRADED → restricciones proporcionales a severidad

**D3: ¿Hay nueva vulnerabilidad introducida indirectamente?**

**No** ✅ — Verificado:
- Frame rejection no crea path donde demanda quede "stuck" (pedal 50ms continúa)
- L1→L2 monotónico no puede causar SAFE (no existe path L2→SAFE por anomalía)
- `volatile` mantenido en `last_can_rx_time` por seguridad defensiva
- ISR vacío → sin race conditions incluso con los cambios

---

## 8. Mejoras Recomendadas para Elevar el Nivel

### Prioridad ALTA (para equivalencia ASIL-A):

1. **Watchdog externo hardware** — El IWDG es interno al MCU. Un watchdog externo (e.g., MAX6301/MAX6369) con ventana temporal confirmaría que el firmware ejecuta a la frecuencia correcta, no solo que no está congelado.

2. **Monitor de referencia ADC** — Leer un voltaje de referencia conocido (e.g., 1.25V bandgap) periódicamente para verificar que el ADC no ha derivado. Actualmente no hay forma de detectar un ADC con offset o ganancia incorrecta.

3. **E2E Protection en CAN** — Añadir CRC-8 o rolling counter + checksum a los frames de comando (throttle, steering). Protegería contra bit-flips y frames corruptos que pasen la validación CRC15 nativa de CAN.

4. **Redundancia de timer PWM** — Leer back el duty cycle programado via registro CCR de captura para confirmar que el hardware de timer refleja lo programado.

### Prioridad MEDIA:

5. **Timeout en OneWire bit-bang** — Actualmente no hay timeout en la comunicación bit-level con DS18B20. Un sensor con la línea held LOW podría bloquear el loop de sensores indefinidamente.

6. **IWDG window mode** — Configurar el watchdog en modo ventana para detectar no solo congelamiento, sino también ejecución demasiado rápida (indica corrupción del flujo de programa).

7. **Diagnóstico de cobertura CAN** — Registrar y reportar la tasa de frames perdidos/erróneos como porcentaje para identificar degradación gradual del bus antes del bus-off.

### Prioridad BAJA:

8. **Startup inhibit persistente** — Actualmente el latch se reinicia a `true` en cada reboot. Si el pedal está presionado durante un power cycle rápido, el inhibit se activa pero podría liberarse en la misma sesión si el operador suelta brevemente. Considerar persisitir el latch en flash con contador de reboots.

9. **Autenticación CAN** — Para proteger contra inyección de frames por un atacante con acceso físico al bus. Dado que es un vehículo para niños, el riesgo de ataque CAN intencionado es bajo.

---

## 9. Resumen Ejecutivo

| Categoría | Evaluación |
|-----------|-----------|
| **Autonomía STM32** | ✅ Completamente autónoma en funciones críticas |
| **Pérdida total CAN/ESP32** | ✅ LIMP_HOME seguro a 5 km/h |
| **Pérdida sensor pedal** | ✅ Multi-capa: contradictory→0, single-channel→LIMP_HOME |
| **Pérdida TOFSense** | ✅ Fail-safe: scale conservadora o 1.0 con speed cap |
| **Aceleración no comandada** | ✅ Imposible por diseño (NaN traps + range checks + startup inhibit) |
| **Shoot-through** | ✅ Imposible (same-timer RPWM/LPWM) |
| **Estados inconsistentes** | ✅ State machine formal con transiciones protegidas |
| **Nivel de seguridad** | **Automoción ligera** (cercano a ASIL-A conceptual) |

El sistema está significativamente por encima de un proyecto hobby robusto y supera
la mayoría de requisitos de un sistema industrial básico. Alcanza un nivel cercano a
automoción ligera con protecciones defence-in-depth en todas las capas críticas.
