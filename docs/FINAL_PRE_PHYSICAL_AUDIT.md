# Auditoría Final Pre-Pruebas Físicas

**Fecha:** 2026-02-21  
**Revisión:** 1.0  
**Alcance:** Verificación completa del firmware STM32 + ESP32 tal como está ahora  
**Referencia:** `docs/FAIL_OPERATIONAL_MIGRATION_AUDIT.md` (documento de referencia única)

---

## A) Fallos Críticos (rompen compilación o ejecución)

### A.1 — No se encontraron fallos que rompan compilación

Tras analizar los 18 archivos fuente en `Core/Src/`, los 16 headers en `Core/Inc/`, y todos los archivos ESP32 en `esp32/src/`:

- **Todas las funciones llamadas en main.c están declaradas** en sus headers correspondientes.
- **Todas las funciones declaradas en .h tienen definición** en sus .c correspondientes.
- **Todos los includes están presentes** — no hay dependencias rotas.
- **El Makefile lista 16 archivos fuente** que coinciden exactamente con los .c en `Core/Src/` (excluyendo correctamente test_math_safety.c y test_steering_cal_store.c que son tests de host).
- **Los CAN IDs son idénticos** entre `Core/Inc/can_handler.h` y `esp32/include/can_ids.h` (21 IDs verificados).
- **Los formatos de payload CAN son compatibles** — ambos lados usan little-endian, mismos byte positions y field widths para los 13 tipos de mensaje verificados.

**Resultado: COMPILACIÓN LIMPIA esperada** (requiere STM32 HAL drivers generados por CubeMX, que están en .gitignore por diseño).

### A.2 — Nota sobre HAL Drivers

Los `Drivers/STM32G4xx_HAL_Driver/` y `Drivers/CMSIS/` **no están en el repositorio** (están en .gitignore). Esto es correcto por diseño — deben generarse con STM32CubeMX a partir del archivo `.ioc` antes de compilar. El `Makefile` referencia 20 archivos HAL que serán generados.

---

## B) Riesgos Funcionales Ocultos

### B.1 — Código muerto: `Ackermann_Compute()` y `Ackermann_SetGeometry()`

**Archivos:** `Core/Src/motor_control.c` líneas 1689 y 1716  
**Impacto:** BAJO — no rompe nada  
**Descripción:** Estas dos funciones están definidas en motor_control.c pero **nunca se llaman** desde ningún otro archivo. No están declaradas en ningún .h. El pipeline de tracción usa `Ackermann_ComputeWheelAngles()` de `ackermann.c`/`ackermann.h` que sí está correctamente integrado.  
**Riesgo:** Ninguno funcional. Es código muerto residual que el linker eliminará con `--gc-sections`. No requiere acción.

### B.2 — Riesgo: `safety_error` es variable global compartida sin mutex

**Archivos:** `Core/Src/safety_system.c` (definición), `Core/Inc/safety_system.h` (extern)  
**Impacto:** BAJO en single-core STM32  
**Descripción:** `safety_error` se lee y escribe desde el loop principal (sin interrupciones) y potencialmente desde ISRs via `Safety_SetError()`. Sin embargo:
- El STM32G474RE es **single-core**, bare-metal, sin RTOS.
- La variable es `Safety_Error_t` (enum = int), que es una escritura atómica en ARM Cortex-M4.
- `Safety_SetError()` solo se llama desde el loop principal (10ms/50ms/100ms tasks), nunca desde ISR.
- Los ISR de EXTI solo incrementan contadores (`wheel_pulse[]`, `steer_center_flag`).  
**Riesgo:** Ninguno en la arquitectura actual. Si se añade RTOS en futuro, requeriría protección.

### B.3 — `startup_pedal_rest_since` inicializada a 0 — comportamiento correcto en power-on

**Archivo:** `Core/Src/main.c` línea 94  
**Impacto:** NINGUNO  
**Descripción:** `startup_pedal_rest_since = 0` es correcto. En el primer ciclo de 50ms, si pedal < 3%, se asigna `now` (que será ~0 en power-on). La lógica `(now - 0) >= 400` podría dar falso positivo si `HAL_GetTick()` devuelve ≥400 en el primer check. Sin embargo:
- En power-on, `HAL_GetTick()` empieza en 0.
- Para cuando el loop de 50ms ejecuta por primera vez, `now` ≈ ~30ms (init time).
- `startup_pedal_rest_since` se asigna `now` en el primer check donde pedal < 3%.
- La ventana de 400ms empieza desde esa asignación.  
**Riesgo:** Ninguno. La secuencia de boot tardará >50ms en llegar al loop principal, y el timer de 400ms empieza correctamente.

### B.4 — Comportamiento si ESP32 no está presente (power-on sin ESP32)

**Verificado en código:**
1. Boot → STANDBY (incondicional, `main.c` línea 158)
2. `Safety_CheckCANTimeout()`: `(HAL_GetTick() - last_can_rx_time) > 250ms` se cumple inmediatamente
3. `last_can_rx_time` inicializado a `HAL_GetTick()` en `Safety_Init()` → ~0
4. Tras ~250ms → `SAFETY_ERROR_CAN_TIMEOUT` + STANDBY→LIMP_HOME (si boot validation pasa)
5. En LIMP_HOME: pedal local funciona con 20% torque, 5 km/h cap
6. Steering centering sigue intentando durante STANDBY (puede completar si sensor está OK)  
**Resultado:** ✅ Correcto. Sin ESP32, el vehículo entra en LIMP_HOME a velocidad peatonal.

### B.5 — Comportamiento si CAN se pierde en cada estado

| Estado actual | CAN se pierde | Resultado | Correcto? |
|---------------|--------------|-----------|-----------|
| BOOT | No relevante | BOOT solo dura <100ms | ✅ |
| STANDBY | Timeout 250ms | STANDBY→LIMP_HOME (si boot OK) | ✅ |
| ACTIVE | Timeout 250ms | ACTIVE→LIMP_HOME | ✅ |
| DEGRADED | Timeout 250ms | DEGRADED→LIMP_HOME | ✅ |
| SAFE | Timeout 250ms | Permanece SAFE (no auto-recupera) | ✅ |
| LIMP_HOME | Ya sin CAN | Permanece LIMP_HOME | ✅ |
| ERROR | No relevante | ERROR es permanente | ✅ |

**Ningún estado queda bloqueado sin acción ante pérdida de CAN.**

### B.6 — LIMP_HOME→ACTIVE recovery requiere safety_error == NONE pero solo limpia CAN_TIMEOUT

**Archivo:** `Core/Src/safety_system.c` líneas 1002-1005  
**Impacto:** POTENCIAL RIESGO BAJO  
**Descripción:** Cuando CAN se restaura desde LIMP_HOME:
```c
if (system_state == SYS_STATE_LIMP_HOME &&
    (Steering_IsCalibrated() || !ServiceMode_IsEnabled(MODULE_STEER_CENTER))) {
    Safety_ClearError(SAFETY_ERROR_CAN_TIMEOUT);
    Safety_SetState(SYS_STATE_ACTIVE);
}
```
La transición limpia `SAFETY_ERROR_CAN_TIMEOUT` y llama a `Safety_SetState(SYS_STATE_ACTIVE)`. Pero `Safety_SetState()` guarda: solo permite LIMP_HOME→ACTIVE si `safety_error == SAFETY_ERROR_NONE`. Si había otro error previo (e.g., `SAFETY_ERROR_SENSOR_FAULT`), la transición sería rechazada.  
**Riesgo:** Bajo. El `Safety_ClearError()` limpia CAN_TIMEOUT. Si hay otro error activo, el vehículo permanece en LIMP_HOME, que es correcto.

### B.7 — ESP32: Falta CAN TX para comandos de usuario (throttle, steering, mode)

**Archivos:** `esp32/src/` — búsqueda exhaustiva  
**Impacto:** ALTO para funcionalidad de control remoto (no afecta seguridad)  
**Descripción:** El ESP32 actualmente:
- ✅ Recibe y parsea todos los mensajes de telemetría del STM32 (13 tipos)
- ✅ Envía heartbeat (0x011) cada 100ms
- ✅ Envía datos de obstáculo (0x208) cada 66ms
- ❌ **NO envía** CMD_THROTTLE (0x100) — no hay implementación de transmisión
- ❌ **NO envía** CMD_STEERING (0x101) — no hay implementación de transmisión
- ❌ **NO envía** CMD_MODE (0x102) — no hay implementación de transmisión
- ❌ **NO envía** SERVICE_CMD (0x110) — no hay implementación de transmisión

**Consecuencia:** El ESP32 es **solo HMI de visualización + sensor de obstáculos**. No puede controlar el vehículo remotamente. El vehículo solo puede conducirse con el pedal local (LIMP_HOME) o mediante otro dispositivo CAN externo.

**Nota técnica:** Esto NO es un fallo de seguridad. El STM32 procesa los mensajes correctamente si llegarán. La ausencia está en el lado ESP32 (HMI).

### B.8 — ESP32: Falta input de usuario (touch/botones)

**Archivos:** `esp32/src/screens/`, `esp32/src/ui/`  
**Impacto:** ALTO para funcionalidad HMI  
**Descripción:** Las pantallas del ESP32 son **solo lectura** — no tienen handlers de touch ni botones. No hay librería de input táctil en `platformio.ini`. Los screens renderizan datos recibidos por CAN pero no aceptan comandos del operador.

---

## C) Diferencias Pendientes Respecto al Firmware Original

Basado en `docs/FAIL_OPERATIONAL_MIGRATION_AUDIT.md` Fase 2, y verificado línea por línea:

### C.1 — Funcionalidades NO portadas (MISSING)

| # | Funcionalidad Original | Archivo original | Estado | Severidad |
|---|----------------------|-----------------|--------|-----------|
| 1 | AI Regenerative Braking | `regen_ai.cpp` | Reemplazado por frenado dinámico proporcional | BAJA — R5 aceptado |
| 2 | Adaptive Cruise Control | `adaptive_cruise.cpp` | No implementado | BAJA — no requerido para seguridad |
| 3 | NVS Config persistence | `config_storage.cpp` | RAM-only; config se pierde en reset | MEDIA — R2, mitigado por flash cal |
| 4 | NVS Bootloop counter | `system.cpp` | Reset cause classification (IWDG/BROWNOUT) | BAJA — mecanismo diferente, equivalente |
| 5 | DS18B20 hot-plug | No documentado | ROM search solo en init | BAJA — no crítico |
| 6 | CAN 0x209 decode body | N/A (nuevo) | Filtro configurado, body vacío | BAJA — informacional |
| 7 | Logger serial completo | `logger.cpp` | ServiceMode_SetFault() per-module | DIFERENTE — no equivalente directo |

### C.2 — Funcionalidades parcialmente portadas

| # | Funcionalidad | Estado actual | Faltante |
|---|--------------|--------------|----------|
| 1 | ESP32 HMI control | Display funciona, CAN RX OK | Falta CAN TX para comandos (throttle/steering/mode) |
| 2 | ESP32 input usuario | Pantallas de visualización OK | Falta procesamiento de touch/botones |
| 3 | ESP32 service menu | CAN 0x110 handler en STM32 listo | Falta UI para enviar service commands desde ESP32 |
| 4 | Relay-en-SAFE decisión | Documentado como riesgo R1 | Falta prueba física + decisión formal (Paso 1) |

### C.3 — Funcionalidades eliminadas intencionadamente

| # | Funcionalidad | Razón | Documentado en |
|---|--------------|-------|----------------|
| 1 | PCA9685 I2C PWM | Reemplazado por TIM1/TIM8 directo (20 kHz, hardware) | FAIL_OPERATIONAL_MIGRATION_AUDIT.md §2.1 |
| 2 | MCP23017 GPIO expander | Reemplazado por GPIO directo STM32 | FAIL_OPERATIONAL_MIGRATION_AUDIT.md §2.3 |
| 3 | ESP32 single-MCU architecture | Rediseñado como dual-MCU (STM32 safety + ESP32 HMI) | Arquitectura documentada |
| 4 | AI regen braking | Reemplazado por frenado proporcional determinista | R5 — decisión de diseño |

---

## D) Lista Exacta de Próximos Pasos — Orden Real de Implementación

Respetando `docs/FAIL_OPERATIONAL_MIGRATION_AUDIT.md` como referencia única:

### Pasos del roadmap fail-operational

| Orden | Paso | Estado | Dependencia | Acción |
|-------|------|--------|-------------|--------|
| 1 | Paso 1: Documentar relay-en-SAFE | ⚠️ PENDIENTE | Ninguna | Prueba física + documento de decisión |
| 2 | Paso 7: Corte relés en SAFE | ❌ PENDIENTE | Resultado Paso 1 | Solo si Paso 1 determina que EN-pin no basta |

**Pasos completados (no requieren acción):** 2, 3, 4, 5, 6

### Pasos funcionales fuera del roadmap de seguridad

| Orden | Paso | Prioridad | Acción |
|-------|------|-----------|--------|
| 3 | ESP32 CAN TX: implementar transmisión de throttle (0x100), steering (0x101), mode (0x102), service (0x110) | ALTA | Nuevo archivo `esp32/src/can/can_tx.cpp` |
| 4 | ESP32 Input: integrar touch/botones para capturar intención del operador | ALTA | Depende de hardware touch disponible |
| 5 | ESP32 HMI completo: screens con control bidireccional | MEDIA | Depende de pasos 3 y 4 |

---

## E) Cuándo Implementar la Pantalla (HMI) Completa

### Regla: La HMI NO interfiere con seguridad funcional si:

1. **Todo el código HMI está en ESP32** — no modifica ningún archivo en `Core/Src/` ni `Core/Inc/`.
2. **El STM32 ya tiene todos los handlers CAN implementados** — los parsers de 0x100, 0x101, 0x102, 0x110 ya validan, clampean y rechazan comandos inseguros.
3. **La seguridad es independiente del HMI** — LIMP_HOME funciona sin ESP32.

### Momento exacto para empezar:

**La implementación completa de la pantalla (HMI) puede empezar AHORA**, en paralelo con los pasos 1 y 7 del roadmap, porque:

- Los pasos 1 y 7 son **solo en `Core/Src/safety_system.c`** (STM32).
- La HMI es **solo en `esp32/src/`** (ESP32).
- No hay solapamiento de archivos ni de bus CAN (los IDs ya están congelados).
- El STM32 ya rechaza comandos inválidos — un ESP32 mal implementado no puede dañar la seguridad.

### Orden recomendado para HMI:

1. `esp32/src/can/can_tx.cpp` — Transmisión de comandos (throttle, steering, mode, service)
2. Touch/button input handler — Captura intención del operador
3. Drive screen → conectar sliders/pedal a CAN TX
4. Service screen → conectar enable/disable módulos a CAN 0x110
5. Test integrado: ESP32 envía commands → STM32 responde ACK (0x103) → ESP32 muestra resultado

### Archivos STM32 que NO deben tocarse durante la implementación HMI:

- `Core/Src/safety_system.c` — Solo para Pasos 1/7
- `Core/Src/motor_control.c` — Congelado
- `Core/Src/can_handler.c` — Congelado (handlers ya implementados)
- `Core/Inc/*.h` — Congelados (contratos CAN ya definidos)

---

## Coherencia con FAIL_OPERATIONAL_MIGRATION_AUDIT.md

### Verificación punto por punto:

| Sección Audit | Verificado | Resultado |
|---------------|-----------|-----------|
| Fase 1: Firmware understanding | ✅ | Boot sequence, state machine, fault triggers — TODO coincide |
| Fase 2: Behavioral comparison | ✅ | 8 subsistemas, 60+ aspectos — todos verificados en código |
| Fase 3: Risk analysis (R1-R9) | ✅ | R1-R9 todos presentes; mitigaciones coinciden |
| Fase 4: Migration Steps 1-7 | ✅ | Steps 2,3,4,5,6 implementados; Steps 1,7 pendientes |
| Step 3 code evidence | ✅ | `safety_system.c` línea 992: bypass MODULE_STEER_CENTER presente |
| Step 4 code evidence | ✅ | `sensor_manager.c`: Pedal_IsContradictory() implementada |
| Step 5 code evidence | ✅ | `esp32/src/sensors/obstacle_sensor.cpp` + `can/can_obstacle.cpp` |
| Step 6 code evidence | ✅ | `Core/Src/steering_cal_store.c` con flash + CRC32 |

### Nota sobre documento FAIL_OPERATIONAL_NEXT_STEPS.md:

El Paso 3 aparece en la sección "Pasos Pendientes" con descripción completa (líneas 76-136) pero la tabla de estado (línea 18) dice `✅ COMPLETADO`. Esto es correcto — la tabla refleja que el código ya fue implementado, y la sección de detalle permanece como referencia histórica de lo que se hizo.

---

## Resumen Ejecutivo

| Categoría | Conteo | Severidad |
|-----------|--------|-----------|
| Fallos de compilación | **0** | — |
| Riesgos funcionales ocultos | **8** identificados | 1 ALTO (B.7 HMI TX), resto BAJO |
| Funcionalidades no portadas | **7** | 1 MEDIA, resto BAJA |
| Funcionalidades parciales | **4** | 2 ALTA (ESP32 HMI) |
| Pasos roadmap pendientes | **2** (Paso 1, Paso 7) | Paso 1 es documentación, Paso 7 es condicional |

**Conclusión: El firmware STM32 está listo para pruebas físicas.** El STM32 es autosuficiente — funciona correctamente sin ESP32 (LIMP_HOME). Las carencias están en el lado ESP32 (HMI) que no afecta a la seguridad funcional.
