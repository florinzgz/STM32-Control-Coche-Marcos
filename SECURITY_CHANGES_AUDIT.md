# Auditoría de Cambios de Seguridad — Firmware STM32 + ESP32-S3

**Fecha:** 2026-03-06  
**Alcance:** Verificación de bugs de seguridad corregidos, protocolo CAN, coordinación entre placas, y seguridad general.  
**Método:** Análisis estático completo de todos los archivos críticos + ejecución de tests unitarios (347/347 pass).

---

## 1. VALIDACIÓN DE BUGS DE SEGURIDAD CORREGIDOS

### 1.1 sensor_manager.c — DS18B20 Temperature Validation

#### OW_ReadTemperature() (líneas 689–723) ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| CRC failure NO devuelve -999.0f | ✅ | Devuelve `0.0f` (línea 709). No existe ninguna ocurrencia de `-999.0f` en todo el archivo. |
| CRC failure lanza Safety_SetError | ✅ | Llama `Safety_SetError(SAFETY_ERROR_SENSOR_FAULT)` en línea 708. |
| Validación rango -55°C a +125°C | ✅ | Líneas 717–720: `if (temp < -55.0f \|\| temp > 125.0f)` → `Safety_SetError()` + retorno `0.0f`. |

#### Temperature_ReadAll() Skip-ROM (líneas 732–756) ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| Validación rango -55°C a +125°C | ✅ | Líneas 749–754: misma validación que OW_ReadTemperature(). |
| Error devuelve 0.0f | ✅ | `temperatures[0] = 0.0f` en línea 751. |
| Lanza Safety_SetError | ✅ | Línea 750: `Safety_SetError(SAFETY_ERROR_SENSOR_FAULT)`. |

**Nota:** El path Skip-ROM no realiza CRC (solo lee 2 bytes del scratchpad), lo cual es aceptable ya que se usa solo como fallback durante early boot cuando no hay sensores descubiertos.

---

### 1.2 safety_system.c — Obstacle_ProcessCAN()

#### Rechazo de 0xFFFF (líneas 1417–1513) ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| Rechaza distancia 0xFFFF | ✅ | Línea 1431: `if (dist == 0xFFFF)` → return sin actualizar distancia validada. |
| No procesa 65535 mm como real | ✅ | Se trata como "no data" del sensor, establece `obstacle_plausible = 0`. |
| DLC validación | ✅ | Línea 1419: `if (len < 5) return;` |

**Validaciones adicionales verificadas:**
- Stale-data detection via rolling counter (líneas 1437–1443) ✅
- Physical plausibility validation con rate limiting (líneas 1450–1472) ✅
- Stuck-sensor detection (líneas 1476–1497) ✅
- Solo actualiza `obstacle_validated_mm` si plausible + healthy + not stale (líneas 1508–1512) ✅

---

### 1.3 eps_params.c — EPS_Params_Set()

#### Validación NaN/Inf (líneas 126–147) ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| Rechaza NaN | ✅ | Línea 132: `if (isnan(value) \|\| isinf(value)) return false;` |
| Rechaza Inf | ✅ | Mismo check (línea 132). |
| Aplica a TODOS los parámetros | ✅ | El check está antes del switch sobre `id`; aplica a los 8 parámetros (EPS_PARAM_COUNT = 8). |
| Protección contra división por cero | ✅ | Líneas 139–142: `assist_vs_speed` y `return_vs_speed` rechazan `value <= 0.0f` explícitamente. Ambos se usan como divisores en el control loop EPS. |
| Rango out-of-bounds | ✅ | Línea 128: `if (id >= EPS_PARAM_COUNT) return false;` |

---

### 1.4 motor_control.c — Motor_SetSigned(), compute_ackermann_differential(), sanitize_float()

#### Motor_SetSigned() — INT16_MIN (líneas 1854–1888) ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| Protección INT16_MIN (-32768) | ✅ | Línea 1858: `if (signed_pwm == INT16_MIN) signed_pwm = INT16_MIN + 1;` — Clamp a -32767, evita UB en negación. |
| PWM duty clamped | ✅ | Línea 1862: `if (duty > PWM_PERIOD) duty = PWM_PERIOD;` |

#### compute_ackermann_differential() (líneas 687–755) ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| steer_deg pasa por sanitize_float() | ✅ | Línea 696: `steer_deg = sanitize_float(steer_deg, 0.0f);` — sanitiza al inicio de la función. |
| Outputs sanitizados | ✅ | Líneas 749–752: todos los `diff_out[i]` pasan por `sanitize_float()` antes de retorno. |
| Protección tan(0) | ✅ | Línea 709: `if (tan_angle < 0.001f) return;` — evita divisón por near-zero. |

#### sanitize_float() (líneas 28–35) ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| Detecta NaN | ✅ | Línea 30: `isnan(val)` |
| Detecta Inf | ✅ | Línea 30: `isinf(val)` |
| Reporta error | ✅ | Línea 31: `Safety_SetError(SAFETY_ERROR_SENSOR_FAULT)` |

**⚠️ Observación (no es un bug):** `sanitize_float()` no valida rangos arbitrarios (ej: throttle > 100%), solo NaN/Inf. La validación de rango se hace downstream en cada caller (ej: `Safety_ValidateThrottle()`, clamp de PWM). Esto es aceptable ya que el rango válido varía por contexto.

---

### 1.5 can_handler.c — ISR y DLC

#### Interrupciones CAN ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| ISR ligera | ✅ | `HAL_FDCAN_RxFifo0Callback()` (stm32g4xx_it.c, líneas 156–166) está **completamente vacía**. |
| Procesamiento en loop principal | ✅ | Todo el procesamiento CAN ocurre en `CAN_ProcessMessages()` llamado desde el main loop (main.c, línea 409). |
| FDCAN IRQ handlers | ✅ | Solo delegan a `HAL_FDCAN_IRQHandler()` (stm32g4xx_it.c, líneas 94–102). |

#### Validación DLC antes de lectura ✅ CORRECTO

Todos los handlers verifican DLC mínimo antes de acceder a datos:

| CAN ID | Función | DLC Check | Línea |
|---|---|---|---|
| 0x011 HEARTBEAT_ESP32 | counter | `msg_len >= 1` | 599 |
| 0x100 CMD_THROTTLE | throttle % | `msg_len >= 1` | 645 |
| 0x101 CMD_STEERING | angle | `msg_len >= 2` | 664 |
| 0x102 CMD_MODE | flags | `msg_len < 1` reject | 673 |
| 0x102 CMD_MODE | gear (opt) | `msg_len >= 2` | 699 |
| 0x110 SERVICE_CMD | command | `msg_len < 1` reject | 756 |
| 0x110 SERVICE_CMD | module_id | `msg_len >= 2` | 818 |
| 0x208 OBSTACLE_DISTANCE | full struct | `msg_len >= 5` | 883 |
| 0x209 OBSTACLE_SAFETY | zone/status | `msg_len >= 3` | 898 |
| 0x120 CMD_LED | front relay | `msg_len >= 1` | 911 |
| 0x120 CMD_LED | rear relay | `msg_len >= 2` | 913 |

---

### 1.6 main.c — startup_inhibit, Scheduler, Watchdog

#### startup_inhibit ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| Bloquea LIMP_HOME | ✅ | Línea 302: `if (startup_inhibit)` tiene prioridad máxima (primer branch del if/else). Suprime `Traction_SetDemand(0.0f)` independientemente del estado. |
| Estructura if/else segura | ✅ | Tres ramas mutuamente excluyentes: startup_inhibit → zero | IsCommandAllowed → CAN throttle | IsLimpHome → local pedal. Default `else` → zero demand (línea 344). |
| Cleared solo con pedal < 3% por 400ms | ✅ | Líneas 249–258: timer de debounce correcto. |
| Re-activates on MCU reset | ✅ | `static bool startup_inhibit = true;` (línea 94) — no persistido en NVM. |

#### Watchdog ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| Refrescado al final del loop | ✅ | Línea 412: `HAL_IWDG_Refresh(&hiwdg);` — última instrucción del while(1). |
| Timeout adecuado | ✅ | 500 ms; loop completo ejecuta en < 2 ms típico. |

#### Scheduler de tareas ✅ CORRECTO

| Tarea | Intervalo | Líneas | Funciones clave |
|---|---|---|---|
| 10 ms | 100 Hz | 187–232 | ABS, TCS, Safety checks, Steering PID, Obstacle |
| 50 ms | 20 Hz | 235–346 | Pedal, Current, Temperature, startup_inhibit, traction demand |
| 100 ms | 10 Hz | 349–373 | CAN heartbeat, status messages |
| 1000 ms | 1 Hz | 376–406 | Temperature CAN, service status, error log, lights |

**Timing:** Usa delta-tick (`now - tick_Nms >= N`) con `uint32_t` — seguro contra rollover de 32-bit (~50 días).

---

### 1.7 stm32g4xx_it.c — ISR Audit

#### ISRs ligeras ✅ CORRECTO

| ISR | Contenido | Estado |
|---|---|---|
| NMI_Handler | Vacío | ✅ |
| HardFault_Handler | Emergency motor shutdown (registros directos) + while(1) | ✅ Aceptable para emergency |
| MemManage/BusFault/UsageFault | Idéntico a HardFault | ✅ Aceptable (duplicación es estilo, no bug) |
| SysTick_Handler | `HAL_IncTick()` | ✅ |
| FDCAN1_IT0/IT1 | `HAL_FDCAN_IRQHandler()` | ✅ |
| EXTI0–2, 15_10, 9_5 | HAL EXTI handler + Wheel/Steering debounce | ✅ |
| TIM1/TIM2 | `HAL_TIM_IRQHandler()` | ✅ |
| I2C1 EV/ER | `HAL_I2C_*_IRQHandler()` | ✅ |
| HAL_FDCAN_RxFifo0Callback | **Completamente vacío** | ✅ |

**Wheel_*_IRQHandler:** Verificado — `Wheel_IRQDebounced()` (sensor_manager.c, líneas 42–52) solo incrementa un contador y guarda un timestamp. Operación O(1) sin loops ni allocations.

**SteeringCenter_IRQHandler:** Verificado — una sola línea: `steer_center_flag = 1;` (sensor_manager.c, línea 69).

**⚠️ Observación (no es un bug):** Los fault handlers (HardFault, MemManage, BusFault, UsageFault) contienen código idéntico duplicado 4 veces. Refactorizar a una función común mejoraría mantenibilidad, pero el comportamiento es correcto.

---

## 2. VERIFICACIÓN DE PROTOCOLO CAN ENTRE STM32 Y ESP32

### 2.1 CAN IDs — Coherencia entre placas ✅

| ID | STM32 (can_handler.h) | ESP32 (can_ids.h) | Coincide |
|---|---|---|---|
| 0x001 HEARTBEAT_STM32 | ✅ DLC 5 | ✅ DLC 5 | ✅ |
| 0x011 HEARTBEAT_ESP32 | ✅ DLC 1+ | ✅ DLC 1 | ✅ |
| 0x100 CMD_THROTTLE | ✅ DLC 1 | ✅ DLC 1 | ✅ |
| 0x101 CMD_STEERING | ✅ DLC 2 | ✅ DLC 2 | ✅ |
| 0x102 CMD_MODE | ✅ DLC 2 | ✅ DLC 2 | ✅ |
| 0x103 CMD_ACK | ✅ DLC 3 | ✅ DLC 3 | ✅ |
| 0x110 SERVICE_CMD | ✅ DLC 2 | ✅ DLC 2 | ✅ |
| 0x120 CMD_LED | ✅ DLC 2 | ✅ DLC 2 | ✅ |
| 0x200 STATUS_SPEED | ✅ DLC 8 | ✅ DLC 8 | ✅ |
| 0x201 STATUS_CURRENT | ✅ DLC 8 | ✅ DLC 8 | ✅ |
| 0x202 STATUS_TEMP | ✅ DLC 5 | ✅ DLC 5 | ✅ |
| 0x203 STATUS_SAFETY | ✅ DLC 3 | ✅ DLC 3 | ✅ |
| 0x204 STATUS_STEERING | ✅ DLC 3 | ✅ DLC 3 | ✅ |
| 0x205 STATUS_TRACTION | ✅ DLC 4 | ✅ DLC 4 | ✅ |
| 0x206 STATUS_TEMP_MAP | ✅ DLC 5 | ✅ DLC 5 | ✅ |
| 0x207 STATUS_BATTERY | ✅ DLC 4 | ✅ DLC 4 | ✅ |
| 0x208 OBSTACLE_DISTANCE | ✅ DLC 5 | ✅ DLC 5 | ✅ |
| 0x209 OBSTACLE_SAFETY | ✅ DLC 3/4 | ✅ DLC 4 | ✅ |
| 0x20A STATUS_LIGHTS | ✅ DLC 2 | ✅ DLC 2 | ✅ |
| 0x300 DIAG_ERROR | ✅ DLC 2 | ✅ DLC 2 | ✅ |
| 0x301–0x305 SERVICE | ✅ | ✅ | ✅ |

### 2.2 ESP32 can_rx.cpp — Heartbeat Decoder ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| Exige DLC >= 5 | ✅ | Línea 43: `if (f.data_length_code < 5) return;` — Coincide con STM32 TX de DLC=5 (can_handler.c, línea 230). |
| Lee bytes 0–4 correctamente | ✅ | aliveCounter (byte 0), systemState (byte 1), faultFlags (byte 2), errorCode (byte 3), statusFlags (byte 4). |

### 2.3 ESP32 can_rx.cpp — Traction Scale ✅ CORRECTO

| Verificación | Estado | Detalle |
|---|---|---|
| Valores limitados a <= 100% | ✅ | Línea 109: `td.scale[i] = (f.data[i] <= 100) ? f.data[i] : 100;` — Clamp explícito. |
| DLC validación | ✅ | Línea 106: `if (f.data_length_code < 4) return;` |

### 2.4 ESP32 can_rx.cpp — Todos los decoders validan DLC ✅ CORRECTO

Se verificaron los 15 decoders CAN en can_rx.cpp. Todos validan DLC mínimo antes de leer:

| Decoder | DLC Check | No OOB reads |
|---|---|---|
| decodeHeartbeat | >= 5 | ✅ |
| decodeSpeed | >= 8 | ✅ |
| decodeCurrent | >= 8 | ✅ |
| decodeTemp | >= 5 | ✅ |
| decodeSafety | >= 3 | ✅ |
| decodeSteering | >= 3 | ✅ |
| decodeTraction | >= 4 | ✅ |
| decodeTempMap | >= 5 | ✅ |
| decodeDiagError | >= 2 | ✅ |
| decodeServiceFaults | >= 4 | ✅ |
| decodeServiceEnabled | >= 4 | ✅ |
| decodeServiceDisabled | >= 4 | ✅ |
| decodeBattery | >= 4 | ✅ |
| decodeCommandAck | >= 3 | ✅ |
| decodeLights | >= 1 | ✅ |

**No se detectan lecturas fuera de buffer ni aceptación de frames corruptos.**

---

## 3. POSIBLES PROBLEMAS DE COORDINACIÓN ENTRE PLACAS

### 3.1 Desincronización de Heartbeat

| Aspecto | Estado | Detalle |
|---|---|---|
| Intervalo heartbeat | ✅ | Ambas placas: 100 ms (can_ids.h línea 152, can_handler.c línea 202). |
| Timeout CAN | ✅ | STM32: 250 ms (can_handler.h línea 67). ESP32: 250 ms (can_ids.h línea 153). |
| Freeze detection | ✅ | STM32 detecta counter frozen del ESP32 (can_handler.c, líneas 599–627). |

**Riesgo de desincronización:** Bajo. El heartbeat counter freeze detection es robusto: si el ESP32 envía heartbeats con el mismo counter (main task stuck pero ISR vivo), el STM32 deja expirar el timeout después de HEARTBEAT_COUNTER_FREEZE_COUNT repeticiones idénticas.

### 3.2 Timeout CAN

| Aspecto | Estado | Detalle |
|---|---|---|
| Heartbeat timeout configurado | ✅ | 250 ms en ambas placas. |
| Obstacle timeout | ✅ | 500 ms (can_handler.h línea 68, can_ids.h línea 159). |
| Timeout → LIMP_HOME (no SAFE) | ✅ | CAN loss no es un hazard → vehículo sigue siendo manejable. |

### 3.3 Estados Incompatibles

| Aspecto | Estado | Detalle |
|---|---|---|
| Estado LIMP_HOME → SAFE | ✅ | safety_system.c línea 286: SAFE acepta transición desde LIMP_HOME (para faults críticos como overtemp, overcurrent). |
| LIMP_HOME → ACTIVE recovery | ✅ | Cuando CAN se restaura + sistema sano, puede recuperar ACTIVE. |
| ESP32 refleja estados STM32 | ✅ | can_ids.h SystemState enum (líneas 85–93) coincide exactamente con SystemState_t del STM32. |

### 3.4 Datos Inválidos Tras Boot

| Aspecto | Estado | Detalle |
|---|---|---|
| STM32 startup_inhibit | ✅ | Siempre `true` al arrancar (línea 94 main.c). Previene movimiento hasta pedal liberado por 400 ms. |
| Obstacle data init | ✅ | `obstacle_distance_mm = 0xFFFF` (safety_system.c). Sentinel "no data" — no genera movimiento. |
| ESP32 no interpreta datos inválidos | ✅ | Todos los decoders CAN requieren DLC válido. No hay valores por defecto peligrosos. |

### 3.5 Contrato CAN Respetado

| Verificación | Estado |
|---|---|
| STM32 nunca envía datos fuera del contrato CAN | ✅ — Todas las funciones CAN_Send* usan DLC fijos coincidentes con el contrato. |
| ESP32 nunca interpreta datos inválidos como válidos | ✅ — Todos los decoders validan DLC y rechazan frames insuficientes. |

---

## 4. ANÁLISIS DE SEGURIDAD GENERAL

### 4.1 Nuevas Vulnerabilidades

**No se detectan nuevas vulnerabilidades introducidas por los cambios de seguridad.**

### 4.2 Comportamiento Indefinido en C

| Verificación | Estado | Detalle |
|---|---|---|
| Negación INT16_MIN | ✅ Protegido | motor_control.c línea 1858. |
| Divisón por cero (EPS) | ✅ Protegido | eps_params.c líneas 139–142: rechaza divisores <= 0. |
| Divisón por cero (Ackermann) | ✅ Protegido | motor_control.c línea 709: `if (tan_angle < 0.001f) return;` |
| NaN/Inf propagation | ✅ Protegido | sanitize_float() en todos los paths críticos. EPS_Params_Set() rechaza NaN/Inf. |

### 4.3 Comparaciones Float Inseguras

| Verificación | Estado | Detalle |
|---|---|---|
| NaN en comparaciones | ✅ Mitigado | `sanitize_float()` filtra NaN antes de que llegue a comparaciones. NaN bypasses all comparisons — sin sanitización, un NaN en `steer_deg` saltaría el deadband check. Corregido en línea 696. |
| Float equality | ✅ | No se detectan comparaciones `==` inseguras entre floats en paths de seguridad. Las comparaciones usan `<`, `>`, `<=`, `>=` con umbrales apropiados. |

### 4.4 Overflow/Underflow

| Verificación | Estado | Detalle |
|---|---|---|
| HAL_GetTick() arithmetic | ✅ | Usa substracción uint32_t (`now - last_tick`), seguro contra rollover de 32-bit. |
| Wheel pulse counter | ✅ | `wheel_pulse[idx]++` en ISR — volatile uint32_t, overflow a 0 es benigno (velocidad calculada = 0). |
| CAN stats counters | ✅ | uint32_t overflow es benigno (son contadores de diagnóstico). |
| Temperature int8_t cast | ✅ | main.c líneas 380–384: `(int8_t)Temperature_Get()`. Float → int8 truncation es intencional (rango -128°C a +127°C cubre DS18B20 -55°C a +125°C). |

### 4.5 Rutas Lógicas que Bypassen Seguridad

| Verificación | Estado | Detalle |
|---|---|---|
| startup_inhibit bypass | ✅ No posible | Es el primer branch del if/else (main.c línea 302). No puede ser saltado. |
| CAN throttle bypass | ✅ Protegido | can_handler.c línea 645: `!Startup_IsInhibited()` check independiente. |
| Safety state bypass | ✅ No posible | `Safety_IsCommandAllowed()` y `Safety_IsLimpHome()` son mutuamente excluyentes con startup_inhibit. Default `else` → zero demand. |
| Obstacle bypass | ✅ No posible | STM32 es autoridad primaria. ESP32 es advisory. Si CAN obstacle se pierde, speed cap de LIMP_HOME es safety net. |

---

## 5. RESULTADO DEL ANÁLISIS

### 5.1 Lista de Verificaciones Correctas ✅

1. ✅ DS18B20 CRC failure devuelve 0.0f (no -999.0f)
2. ✅ DS18B20 rango -55°C a +125°C validado en ambos paths (Match ROM y Skip ROM)
3. ✅ Safety_SetError(SAFETY_ERROR_SENSOR_FAULT) en todos los fallos de sensor
4. ✅ Obstacle_ProcessCAN() rechaza 0xFFFF explícitamente
5. ✅ EPS_Params_Set() rechaza NaN e Inf para TODOS los parámetros
6. ✅ EPS_Params_Set() protege contra división por cero (assist_vs_speed, return_vs_speed)
7. ✅ Motor_SetSigned() protege contra negación de INT16_MIN
8. ✅ compute_ackermann_differential() sanitiza steer_deg con sanitize_float()
9. ✅ sanitize_float() detecta NaN e Inf, reporta Safety error
10. ✅ CAN ISR completamente vacía — todo procesado en main loop
11. ✅ Todos los handlers CAN validan DLC antes de leer datos
12. ✅ startup_inhibit bloquea LIMP_HOME correctamente (prioridad máxima)
13. ✅ Watchdog refrescado al final del main loop
14. ✅ Scheduler 10ms/50ms/100ms/1000ms correcto
15. ✅ Todas las ISR son ligeras (flags o HAL handlers)
16. ✅ ESP32 heartbeat decoder exige DLC >= 5 (coincide con STM32 TX DLC=5)
17. ✅ ESP32 traction scale limitado a <= 100%
18. ✅ Todos los 15 decoders CAN del ESP32 validan DLC
19. ✅ No hay lecturas fuera de buffer en ESP32
20. ✅ CAN IDs coherentes entre STM32 y ESP32
21. ✅ LIMP_HOME puede transicionar a SAFE para faults críticos
22. ✅ Estado LIMP_HOME → ACTIVE recovery funcional
23. ✅ No se detecta comportamiento indefinido
24. ✅ No se detectan comparaciones float inseguras
25. ✅ No se detectan overflow/underflow peligrosos
26. ✅ No se detectan rutas que bypassen el sistema de seguridad
27. ✅ 347/347 tests unitarios pasan

### 5.2 Problemas Encontrados

**No se encontraron bugs ni errores en los cambios de seguridad auditados.**

Todos los cambios de seguridad descritos en el issue están correctamente implementados en el código actual.

### 5.3 Riesgos Potenciales (Bajo Impacto)

| # | Riesgo | Severidad | Detalle |
|---|---|---|---|
| R1 | Code duplication en fault handlers | Bajo | HardFault, MemManage, BusFault, UsageFault repiten el mismo código de emergency shutdown 4 veces. Podrían refactorizarse a una función común. No es un bug — solo mantenibilidad. |
| R2 | sanitize_float() no valida rangos | Info | Solo detecta NaN/Inf, no valores fuera de rango físico. La validación de rango se hace downstream en cada contexto (Safety_ValidateThrottle, clamp PWM). Esto es un diseño intencional, no un bug. |
| R3 | SERVICE_CMD sin autenticación | Bajo | No tiene rolling counter ni challenge-response. Aceptable para prototipo educativo cerrado. Documentado en el código (can_handler.c línea 750). |
| R4 | Skip-ROM path sin CRC | Bajo | Temperature_ReadAll() Skip-ROM solo lee 2 bytes sin verificación CRC. Solo se usa durante early boot como fallback. La validación de rango (-55 a +125°C) mitiga el riesgo. |

### 5.4 Recomendaciones de Mejora (Opcionales)

1. **Refactorizar fault handlers:** Extraer el código de emergency shutdown a una función `Emergency_MotorShutdown()` para evitar las 4 copias idénticas.
2. **Considerar CRC en Skip-ROM:** Leer los 9 bytes completos del scratchpad en el path Skip-ROM y verificar CRC, igual que en el path Match-ROM.
3. **Documentar SERVICE_CMD security:** Si el proyecto evoluciona a producción, implementar autenticación en SERVICE_CMD.

### 5.5 Confirmación Final

**✅ El firmware es SEGURO para compilar y probar en hardware.**

Todos los cambios de seguridad están correctamente implementados:
- No introducen regresiones
- No rompen la lógica existente
- No rompen el contrato CAN entre STM32 y ESP32
- Ambas placas se coordinan correctamente a través de CAN-BUS
- Los 347 tests unitarios pasan sin fallos
- No se detectan nuevas vulnerabilidades
- No hay comportamiento indefinido
- No hay rutas que permitan saltarse el sistema de seguridad
