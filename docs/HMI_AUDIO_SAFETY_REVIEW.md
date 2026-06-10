# 🔒 HMI Audio Safety Coherence Review

**Versión:** 1.0  
**Fecha:** 2026-02-23  
**Autor:** Auditoría automática  
**Alcance:** Coherencia entre el sistema de audio ESP32 y el comportamiento real del firmware STM32  
**Firmware STM32:** Core/Src/ (safety_system.c, can_handler.c, motor_control.c, sensor_manager.c)  
**Firmware ESP32:** esp32/src/ (audio_manager.h/cpp, main.cpp, can_rx.cpp)  
**Referencia original:** [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos) → `include/alerts.h`

---

## Índice

1. [Resumen ejecutivo](#1-resumen-ejecutivo)
2. [Tabla EVENTO_REAL → ESTADO → MENSAJE_AUDIO](#2-tabla-evento_real--estado--mensaje_audio)
3. [Eventos peligrosos sin aviso sonoro](#3-eventos-peligrosos-sin-aviso-sonoro)
4. [Avisos contradictorios o engañosos](#4-avisos-contradictorios-o-engañosos)
5. [Riesgo de saturación auditiva (spam)](#5-riesgo-de-saturación-auditiva-spam)
6. [Clasificación de cada audio](#6-clasificación-de-cada-audio)
7. [Matriz de prioridad de interrupción](#7-matriz-de-prioridad-de-interrupción)
8. [Hallazgos específicos y riesgos](#8-hallazgos-específicos-y-riesgos)
9. [Recomendaciones](#9-recomendaciones)

---

## 1. Resumen ejecutivo

### Arquitectura del sistema de audio

```
STM32 (Safety Authority)           CAN Bus           ESP32 (HMI / Audio)
┌─────────────────────┐    0x001 Heartbeat     ┌──────────────────────┐
│ safety_system.c     │───────────────────────→│ can_rx.cpp           │
│  - SystemState      │    0x203 Safety        │  - Decode frames     │
│  - Safety_Error_t   │───────────────────────→│  - VehicleData store │
│  - Fault flags      │    0x207 Battery       │                      │
│  - Temp/Current/... │───────────────────────→│ main.cpp             │
│                     │    0x20A Lights        │  - Audio triggers    │
│ can_handler.c       │───────────────────────→│  - State tracking    │
│  - CAN TX 100ms     │                        │                      │
│  - Bus-off detect   │                        │ audio_manager.cpp    │
│                     │                        │  - DFPlayer UART     │
│ motor_control.c     │                        │  - Priority queue    │
│  - Per-motor cutoff │                        │  - 4s cooldown       │
│  - 130°C emergency  │                        │  - Burst collapse    │
└─────────────────────┘                        └──────┬───────────────┘
                                                      │ UART2 9600bd
                                                      ▼
                                               ┌──────────────┐
                                               │ DFPlayer Mini│
                                               │ SD Card 0001 │
                                               │  ...0068.mp3 │
                                               └──────────────┘
```

### Resumen de hallazgos

| Categoría | Cantidad | Severidad |
|-----------|----------|-----------|
| Eventos peligrosos sin audio | **7** | 🔴 ALTA |
| Avisos contradictorios/engañosos | **5** | 🟠 MEDIA-ALTA |
| Riesgos de saturación auditiva | **4** | 🟡 MEDIA |
| Tracks definidos sin trigger | **32** | 🔵 BAJA (reservados) |
| Audios correctamente mapeados | **~20** | ✅ OK |

---

## 2. Tabla EVENTO_REAL → ESTADO → MENSAJE_AUDIO

### 2.1 Eventos de sistema / estados

| # | Evento real STM32 | Estado STM32 | Error Code | Audio ESP32 actual | Track # | Prioridad | ¿Correcto? |
|---|---|---|---|---|---|---|---|
| E01 | Power-on, ESP32 arranca | — | — | `WELCOME` | 1 | HIGH | ✅ |
| E02 | Ignition key OFF | — | — | `FAREWELL` | 2 | HIGH | ✅ |
| E03 | STANDBY → ACTIVE (boot OK + CAN alive) | ACTIVE | NONE | (ninguno) | — | — | ⚠️ Sin confirmación de "sistema listo" |
| E04 | ACTIVE → DEGRADED (fault menor) | DEGRADED | variable | `ERROR_GENERAL` | 3 | HIGH | ⚠️ Genérico, no indica la causa |
| E05 | ACTIVE → LIMP_HOME (CAN timeout) | LIMP_HOME | CAN_TIMEOUT | `ERROR_GENERAL` | 3 | HIGH | 🔴 Engañoso — dice "error general" pero el vehículo sigue siendo conducible |
| E06 | ACTIVE → SAFE (overcurrent ×3) | SAFE | OVERCURRENT | `EMERGENCY` | 31 | HIGH | ✅ |
| E07 | ACTIVE → SAFE (temp > 90°C) | SAFE | OVERTEMP | `EMERGENCY` | 31 | HIGH | ✅ |
| E08 | ACTIVE → SAFE (battery < 18V) | SAFE | BATTERY_UV_CRIT | `EMERGENCY` | 31 | HIGH | ✅ |
| E09 | ACTIVE → SAFE (I2C failure, recovery exhausted) | SAFE | I2C_FAILURE | `EMERGENCY` | 31 | HIGH | ✅ |
| E10 | any → ERROR (watchdog / emergency stop) | ERROR | WATCHDOG/EMERGENCY_STOP | `EMERGENCY` | 31 | HIGH | ✅ |
| E11 | SAFE/ERROR → ACTIVE (recovery) | ACTIVE | NONE | `SAFETY_RESET` | 32 | MEDIUM | ✅ |
| E12 | DEGRADED → ACTIVE (recovery debounce 500ms) | ACTIVE | NONE | `SAFETY_RESET` | 32 | MEDIUM | ⚠️ Solo suena si viene de SAFE/ERROR, no de DEGRADED |
| E13 | LIMP_HOME → ACTIVE (CAN restored) | ACTIVE | NONE | `SAFETY_RESET` | 32 | MEDIUM | ⚠️ Solo suena si viene de SAFE/ERROR, no de LIMP_HOME |

### 2.2 Eventos de sensores / hardware

| # | Evento real STM32 | Estado STM32 | Error Code | Audio ESP32 actual | Track # | Prioridad | ¿Correcto? |
|---|---|---|---|---|---|---|---|
| E20 | Overcurrent motor (> 25A) — 1ª vez | DEGRADED | OVERCURRENT | `OVERCURRENT` (53) vía errorCode | 53 | HIGH | ✅ |
| E21 | Overcurrent motor — ≥3 consecutivas | SAFE | OVERCURRENT | `EMERGENCY` (31) vía systemState + `OVERCURRENT` (53) vía errorCode | 31+53 | HIGH | ⚠️ Doble audio simultáneo |
| E22 | Temperatura motor > 80°C (warning) | DEGRADED | OVERTEMP | `SENSOR_TEMP_ERROR` (33) vía errorCode + `ERROR_GENERAL` (3) vía systemState | 33+3 | MEDIUM+HIGH | ⚠️ Conflicto: sensor_temp_error + error_general simultáneos |
| E23 | Temperatura motor > 85°C (solo ESP32 lee) | (no cambia) | (no cambia) | `TEMP_HIGH` (10) | 10 | MEDIUM | ⚠️ Threshold mismatch: ESP32 usa 85°C, STM32 usa 80°C |
| E24 | Temperatura motor > 90°C (critical) | SAFE | OVERTEMP | `EMERGENCY` (31) + `SENSOR_TEMP_ERROR` (33) | 31+33 | HIGH | ⚠️ Doble audio |
| E25 | Temperatura motor > 130°C (per-motor cutoff) | (stays DEGRADED/SAFE) | OVERTEMP | Depende del estado global | — | — | 🔴 Sin audio específico para cutoff per-motor |
| E26 | Temperatura normalizada | (recovery) | NONE | `TEMP_NORMAL` (11) vía ESP32 threshold check | 11 | LOW | ⚠️ Puede no sonar si la normalización viene por debajo de 75°C STM32 |
| E27 | Battery < 20.0V (warning) | DEGRADED | BATTERY_UV_WARN | `BATTERY_LOW` (12) vía voltageRaw + `ERROR_GENERAL` (3) vía systemState | 12+3 | MEDIUM+HIGH | ⚠️ Doble audio |
| E28 | Battery < 18.0V (critical) | SAFE | BATTERY_UV_CRIT | `BATTERY_CRITICAL` (13) + `EMERGENCY` (31) | 13+31 | HIGH+HIGH | ⚠️ Doble audio |
| E29 | Battery sensor failure (0V reading) | SAFE | BATTERY_UV_CRIT | `EMERGENCY` (31) — pero `BATTERY_CRITICAL` NO suena (voltageRaw == 0 falla el guard `battVoltRaw > 0`) | 31 | HIGH | 🔴 Sin audio de batería crítica cuando el sensor falla |
| E30 | Pedal plausibility failure (contradictory) | LIMP_HOME | SENSOR_FAULT | `SENSOR_SPEED_ERROR` (35) vía errorCode mapping | 35 | MEDIUM | 🔴 Audio incorrecto: dice "sin señal de velocidad" pero es error de pedal |
| E31 | Pedal plausibility failure (software check) | LIMP_HOME | SENSOR_FAULT | `SENSOR_SPEED_ERROR` (35) vía errorCode mapping | 35 | MEDIUM | 🔴 Audio incorrecto: dice "sin señal de velocidad" pero es error de pedal |
| E32 | Wheel speed sensor implausible | DEGRADED | SENSOR_FAULT | `SENSOR_SPEED_ERROR` (35) | 35 | MEDIUM | ✅ Correcto para este caso |
| E33 | Steering encoder fault | DEGRADED | SENSOR_FAULT | `SENSOR_SPEED_ERROR` (35) | 35 | MEDIUM | 🔴 Audio incorrecto: dice "sin señal de velocidad" pero es error de encoder |
| E34 | Steering centering failed | DEGRADED | CENTERING | `ENCODER_ERROR` (9) vía errorCode mapping | 9 | MEDIUM | ✅ |
| E35 | I2C bus failure (TCA9548A / INA226) | SAFE | I2C_FAILURE | `SENSOR_CURRENT_ERROR` (34) vía errorCode mapping | 34 | MEDIUM | ⚠️ Parcialmente correcto: I2C puede afectar más que corriente |
| E36 | CAN bus-off detected | LIMP_HOME | CAN_BUSOFF | (ninguno — no está en el switch de errorCode) | — | — | 🔴 Sin audio para CAN bus-off |
| E37 | ABS activating | ACTIVE | NONE | `ABS_ON` (39) | 39 | LOW | ✅ |
| E38 | ABS deactivating | ACTIVE | NONE | `ABS_OFF` (40) | 40 | LOW | ✅ |
| E39 | TCS activating | ACTIVE | NONE | `TCS_ON` (41) | 41 | LOW | ✅ |
| E40 | TCS deactivating | ACTIVE | NONE | `TCS_OFF` (42) | 42 | LOW | ✅ |
| E41 | Obstacle zone ≥ 3 (emergencia) | ACTIVE | NONE | `OBSTACLE_WARN` (54) | 54 | MEDIUM | ✅ |

### 2.3 Eventos de usuario / interfaz

| # | Evento real STM32 | Estado STM32 | Audio ESP32 actual | Track # | Prioridad | ¿Correcto? |
|---|---|---|---|---|---|---|
| E50 | Gear → Park | ACTIVE | `GEAR_PARK` (24) | 24 | LOW | ✅ |
| E51 | Gear → Reverse | ACTIVE | `GEAR_REVERSE` (22) | 22 | LOW | ✅ |
| E52 | Gear → Neutral | ACTIVE | `GEAR_NEUTRAL` (23) | 23 | LOW | ✅ |
| E53 | Gear → Forward D1 | ACTIVE | `GEAR_D1` (20) | 20 | LOW | ✅ |
| E54 | Gear → Forward D2 | ACTIVE | `GEAR_D2` (21) | 21 | LOW | ✅ |
| E55 | Mode → 4x4 | ACTIVE | `TRACTION_4X4` (37) | 37 | LOW | ✅ |
| E56 | Mode → 4x2 | ACTIVE | `TRACTION_4X2` (38) | 38 | LOW | ✅ |
| E57 | Mode → 360° tank | ACTIVE | `BEEP` (68) | 68 | LOW | ⚠️ Sin anuncio verbal para modo peligroso |
| E58 | Lights toggle ON | ACTIVE | `LIGHTS_ON` (16) | 16 | LOW | ✅ |
| E59 | Lights toggle OFF | ACTIVE | `LIGHTS_OFF` (17) | 17 | LOW | ✅ |
| E60 | Startup inhibit active (pedal held) | STANDBY/ACTIVE | (ninguno) | — | — | 🔴 Sin aviso de que el pedal debe soltarse |

---

## 3. Eventos peligrosos sin aviso sonoro

### 🔴 HALLAZGO-01: Pedal plausibility failure — audio incorrecto

**Evento:** El pedal usa doble lectura ADC con plausibilidad software. Si las muestras divergen > 30 counts, el valor sale de rango, o la tasa de cambio excede el límite, el STM32 entra en LIMP_HOME con error SENSOR_FAULT.

**Audio actual:** Se reproduce `SENSOR_SPEED_ERROR` (track 35: "Sin señal de velocidad. Revise sensores de rueda.") porque el ESP32 mapea `SafetyError::SENSOR_FAULT` → `SENSOR_SPEED_ERROR`.

**Problema:** El audio es **completamente incorrecto**. El conductor oye "sin señal de velocidad" cuando el problema real es que **el pedal no responde correctamente**. No existe `PEDAL_ERROR` (track 5) en el mapping de errores en tiempo real.

**Riesgo:** El conductor puede intentar "revisar sensores de rueda" cuando el problema es el pedal. Si los canales están contradictoriamente pegados, el torque se fuerza a cero pero el conductor no entiende por qué el coche no acelera.

**Severidad:** 🔴 ALTA — fallo de comunicación crítica conductor-vehículo

---

### 🔴 HALLAZGO-02: Per-motor emergency cutoff (130°C) — sin audio

**Evento:** Cuando un motor individual supera 130°C, `motor_control.c` fuerza `wheel_scale[i] = 0.0` para ese motor. Esto NO cambia el estado global del sistema — puede seguir en ACTIVE o DEGRADED.

**Audio actual:** Ninguno específico. Si el STM32 también detecta > 90°C globalmente, se entra en SAFE y suena EMERGENCY. Pero si la lectura global aún está en el rango 80–90°C (porque sólo un motor está a 130°C y otros están fríos), **no hay audio**.

**Problema:** El conductor pierde potencia en una rueda sin ningún aviso. En curva, esto puede causar un cambio brusco de trayectoria.

**Severidad:** 🔴 ALTA — pérdida de tracción asimétrica sin aviso

---

### 🔴 HALLAZGO-03: CAN bus-off — sin audio

**Evento:** `CAN_CheckBusOff()` detecta bus-off en el FDCAN, entra LIMP_HOME y establece `SAFETY_ERROR_CAN_BUSOFF (13)`.

**Audio actual:** El ESP32 detecta el cambio de estado a LIMP_HOME y reproduce `ERROR_GENERAL`. Sin embargo, el error code `CAN_BUSOFF` **no está en el switch de error codes** del ESP32 (`main.cpp` línea 548–574), por lo que **no hay audio específico**.

**Problema adicional:** Si el CAN está en bus-off, **el ESP32 no recibe ningún frame**. El heartbeat timeout en el ESP32 no genera audio directo — sólo el cambio de estado (que nunca llega porque CAN está caído). El ESP32 se queda sin datos actualizados.

**Severidad:** 🔴 ALTA — pérdida total de comunicación sin aviso al conductor

---

### 🔴 HALLAZGO-04: Startup inhibit — sin audio

**Evento:** Si el conductor tiene el pedal presionado al encender, el STM32 activa `startup_inhibit`. El vehículo no se mueve hasta que el pedal baje a < 3% durante 400ms.

**Audio actual:** Ninguno. El conductor puede creer que el sistema está averiado.

**Severidad:** 🟠 MEDIA — confusión del conductor pero sin peligro inmediato

---

### 🔴 HALLAZGO-05: LIMP_HOME pedal arming — sin audio

**Evento:** En LIMP_HOME, el STM32 requiere que el pedal baje a < 3% durante 300ms antes de permitir torque (previene creep por offset ADC).

**Audio actual:** Ninguno. El conductor en LIMP_HOME puede no entender por qué el coche no responde al pedal.

**Severidad:** 🟠 MEDIA — confusión en situación ya degradada

---

### 🔴 HALLAZGO-06: Steering encoder fault — audio incorrecto

**Evento:** `Encoder_HasFault()` en `Safety_CheckEncoder()` detecta fallo del encoder de dirección (range, jumps, frozen). Se neutraliza el steering PID, se entra en DEGRADED con error `SENSOR_FAULT`.

**Audio actual:** Como el error code es `SENSOR_FAULT` (no `CENTERING`), el ESP32 reproduce `SENSOR_SPEED_ERROR` (35): "Sin señal de velocidad".

**Problema:** Audio completamente incorrecto. El conductor oye "velocidad" cuando el problema es la **dirección**. Existe `ENCODER_ERROR` (track 9) disponible pero sólo se usa cuando el error code es `CENTERING` (8), no `SENSOR_FAULT` (4).

**Severidad:** 🔴 ALTA — dirección comprometida con aviso de velocidad

---

### 🔴 HALLAZGO-07: Watchdog timeout — solo EMERGENCY genérico

**Evento:** El IWDG de ~4.1 s expira. El STM32 hace un hard reset. Al re-arrancar, `esp_reset_reason()` en el ESP32 puede detectar el motivo, pero el sistema arranca limpio.

**Audio actual:** Si el ESP32 también pierde la conexión, no hay audio. Si el STM32 re-arranca y el ESP32 sigue vivo, ve una transición ACTIVE → BOOT → STANDBY → ..., pero el audio de `EMERGENCY` sólo se dispara si ve transición a ERROR/SAFE.

**Problema:** Un reset watchdog del STM32 puede ser invisible para el conductor. El vehículo se detiene momentáneamente y re-arranca sin explicación.

**Severidad:** 🟠 MEDIA — interrupción temporal potencialmente confusa

---

## 4. Avisos contradictorios o engañosos

### ⚠️ CONTRADICCIÓN-01: LIMP_HOME dice "error general" — pero el coche funciona

**Audio:** `ERROR_GENERAL` (3): "Atención. Se ha detectado un error general."  
**Realidad:** LIMP_HOME permite conducción a 5 km/h con torque limitado al 20%.

**Problema:** El mensaje sugiere un fallo grave genérico. El conductor puede creer que debe detenerse inmediatamente. En realidad, LIMP_HOME es un modo diseñado para llegar a destino a baja velocidad.

**Corrección sugerida:** Usar un audio diferenciado como "Modo de emergencia de baja velocidad. Máximo 5 kilómetros por hora." o asignar un track específico.

---

### ⚠️ CONTRADICCIÓN-02: DEGRADED dice "error general" — no indica gravedad

**Audio:** `ERROR_GENERAL` (3): "Atención. Se ha detectado un error general."  
**Realidad:** DEGRADED tiene 3 niveles (L1: 70% potencia, L2: 50%, L3: 40%). El conductor no sabe si tiene un sensor menor faulted o si está cerca de SAFE.

**Problema:** No diferencia entre DEGRADED L1 (menor) y L3 (grave). Un conductor experimentado querría saber qué capacidad le queda.

---

### ⚠️ CONTRADICCIÓN-03: Temperatura — thresholds desincronizados

| Sistema | Warning | Critical |
|---------|---------|----------|
| STM32 `Safety_CheckTemperature()` | 80°C → DEGRADED | 90°C → SAFE |
| STM32 `Motor per-motor cutoff` | — | 130°C → wheel off |
| ESP32 `main.cpp` temp check | **85°C** → TEMP_HIGH | — |
| ESP32 `main.cpp` temp recovery | < **80°C** (85-5 hyst) → TEMP_NORMAL | — |

**Problema:** El ESP32 avisa a 85°C pero el STM32 ya entró en DEGRADED a 80°C. Hay una ventana de 80–85°C donde:
- El STM32 ya limitó potencia al 50% (DEGRADED L2)
- El ESP32 reprodujo `ERROR_GENERAL` (por transición a DEGRADED)
- Pero `TEMP_HIGH` ("Temperatura del motor elevada") NO suena todavía

El conductor oye "error general" sin saber que es por temperatura. Cuando finalmente suena `TEMP_HIGH` a 85°C, ya lleva 5 segundos con potencia limitada.

---

### ⚠️ CONTRADICCIÓN-04: Error SENSOR_FAULT es demasiado genérico

`SENSOR_FAULT` (4) se usa para:
- Pedal plausibility failure
- Wheel speed sensor implausible  
- Steering encoder fault

Pero el ESP32 los mapea **todos** a `SENSOR_SPEED_ERROR` (35): "Sin señal de velocidad."

Solo uno de esos tres escenarios es realmente un problema de velocidad.

---

### ⚠️ CONTRADICCIÓN-05: Double-audio en transiciones con error code

Cuando ocurre (por ejemplo) un overcurrent que causa DEGRADED → SAFE:
1. `systemState` cambia a SAFE → `playWarning(EMERGENCY, HIGH)` — track 31
2. `errorCode` cambia a OVERCURRENT → `playWarning(OVERCURRENT, HIGH)` — track 53
3. Ambos pasan por `playWarning()` incrementando `errorBurstCount`
4. Si burst ≥ 2 en < 2s: ambos se reemplazan por `ERROR_GENERAL` (3)

**Resultado:** El conductor oye "error general" en vez de "corriente excesiva" + "motor deshabilitado". El burst collapse **oculta** la información específica.

---

## 5. Riesgo de saturación auditiva (spam)

### 🟡 SPAM-01: ABS/TCS chattering a LOW priority

**Escenario:** ABS y TCS se activan/desactivan cada ciclo de control (100ms) en superficies irregulares. El ESP32 lee `absActive` y `tcsActive` del frame STATUS_SAFETY cada 100ms.

**Protección actual:** Per-sound cooldown de 4 segundos. Si ABS se activa a T=0, ABS_ON suena. A T=100ms ABS se desactiva, ABS_OFF se intenta pero cooldown de ABS_OFF no aplica (primera vez). Resultado: ABS_ON y ABS_OFF suenan con 100ms de separación.

**Problema:** En el siguiente ciclo (T=200ms), ABS se re-activa → ABS_ON bloqueado por cooldown. Pero ABS_OFF a T=300ms intentará sonar (4s desde última ABS_OFF = T=0.1s → bloqueado). **El cooldown protege parcialmente**, pero los primeros 2 mensajes suenan casi simultáneos.

**Riesgo:** Bajo — el DFPlayer tiene 100ms entre comandos, por lo que el segundo mensaje sobreescribirá al primero (misma prioridad LOW). Pero genera una experiencia confusa.

---

### 🟡 SPAM-02: Obstacle warning a 2s interval

**Escenario:** Obstáculo permanente en zona ≥ 3 (< ~30cm). El sonido se repite cada 2 segundos indefinidamente.

**Protección actual:** `OBSTACLE_WARN_INTERVAL_MS = 2000` ms.

**Problema:** En estacionamiento, el conductor puede estar rodeado de obstáculos. Un beep cada 2 segundos durante maniobras prolongadas es molesto. No hay escalamiento (más rápido cuando más cerca) ni timeout.

---

### 🟡 SPAM-03: Gear change rapid cycling

**Escenario:** El selector de marchas (MCP23017) se mueve rápidamente P→R→D.

**Protección actual:** 100ms debounce entre envíos CAN. Cada cambio genera un audio LOW.

**Problema:** P→R→D genera 3 audios en ~300ms. Con DFPlayer `CMD_INTERVAL_MS = 100ms` y `MAX_PLAY_DURATION_MS = 5000ms`, el primer audio se reproduce 100ms, luego es sobrescrito por el segundo, luego por el tercero. El conductor solo oye los primeros ~100ms de cada anuncio (cortados).

**Riesgo:** Bajo — confusión auditiva momentánea, pero la marcha final se confirma correctamente.

---

### 🟡 SPAM-04: Lights toggle CAN echo + touch duplicate

**Escenario:** El usuario pulsa el botón de luces en la pantalla.

**Audio actual:**
1. Touch handler: `audio::play(LIGHTS_ON, LOW)` — inmediato
2. STM32 confirma via STATUS_LIGHTS → `lightsNow != lastLightsRelayOn` → `audio::play(LIGHTS_ON, LOW)` — ~200ms después

**Resultado:** Dos audios de `LIGHTS_ON` con ~200ms de separación. El cooldown de 4s del segundo NO ayuda porque son el mismo track.

**Espera...** El cooldown SÍ ayuda: el segundo intento de `LIGHTS_ON` ocurrirá ~200ms después del primero, bien dentro de los 4000ms de cooldown. **El segundo se descarta.** ✅ Protegido.

---

## 6. Clasificación de cada audio

### Clasificación de seguridad: Informativo / Advertencia / Crítico

| Track | Sonido | Clasificación | Justificación |
|-------|--------|--------------|---------------|
| 1 | WELCOME | 🔵 Informativo | Saludo de arranque, sin acción requerida |
| 2 | FAREWELL | 🔵 Informativo | Despedida de apagado |
| 3 | ERROR_GENERAL | 🟠 Advertencia | Alerta genérica — acción recomendada |
| 4 | PEDAL_OK | 🔵 Informativo | Confirmación de calibración |
| 5 | PEDAL_ERROR | 🔴 Crítico | Fallo de pedal afecta control del vehículo |
| 6 | INA_OK | 🔵 Informativo | Confirmación de calibración |
| 7 | INA_ERROR | 🟠 Advertencia | Sensor de corriente degradado |
| 8 | ENCODER_OK | 🔵 Informativo | Confirmación |
| 9 | ENCODER_ERROR | 🔴 Crítico | Fallo de dirección |
| 10 | TEMP_HIGH | 🟠 Advertencia | Temperatura elevada, reducir velocidad |
| 11 | TEMP_NORMAL | 🔵 Informativo | Recuperación de temperatura |
| 12 | BATTERY_LOW | 🟠 Advertencia | Batería baja, planificar carga |
| 13 | BATTERY_CRITICAL | 🔴 Crítico | Tracción desconectándose |
| 14 | PARKING_BRAKE_ON | 🔵 Informativo | Confirmación |
| 15 | PARKING_BRAKE_OFF | 🔵 Informativo | Confirmación |
| 16 | LIGHTS_ON | 🔵 Informativo | Confirmación |
| 17 | LIGHTS_OFF | 🔵 Informativo | Confirmación |
| 18-19 | RADIO_ON/OFF | 🔵 Informativo | Confirmación |
| 20-24 | GEAR_D1/D2/R/N/P | 🔵 Informativo | Confirmación de marcha |
| 25-28 | CAL_* / MENU_HIDDEN | 🔵 Informativo | Modo servicio |
| 29-30 | TEST_SYSTEM/OK | 🔵 Informativo | Diagnóstico |
| 31 | EMERGENCY | 🔴 **CRÍTICO** | Motor deshabilitado — máxima urgencia |
| 32 | SAFETY_RESET | 🔵 Informativo | Recuperación completada |
| 33 | SENSOR_TEMP_ERROR | 🟠 Advertencia | Sensor de temperatura falla |
| 34 | SENSOR_CURRENT_ERROR | 🟠 Advertencia | Sensor de corriente falla |
| 35 | SENSOR_SPEED_ERROR | 🟠 Advertencia | Sensor de velocidad falla |
| 36 | MODULE_OK | 🔵 Informativo | Confirmación |
| 37-38 | TRACTION_4X4/4X2 | 🔵 Informativo | Cambio de modo |
| 39-42 | ABS/TCS ON/OFF | 🔵 Informativo | Estado de seguridad activa |
| 43-44 | REGEN ON/OFF | 🔵 Informativo | — no implementado — |
| 45-51 | WiFi/BT | 🔵 Informativo | — reservados, no implementados — |
| 52 | MAX_SPEED | 🟠 Advertencia | — no implementado — |
| 53 | OVERCURRENT | 🔴 **CRÍTICO** | Corriente excesiva — riesgo de daño |
| 54 | OBSTACLE_WARN | 🟠 Advertencia | Obstáculo cercano |
| 55 | PARKING_ASSIST | 🔵 Informativo | — no implementado — |
| 56 | SOFT_START | 🔵 Informativo | — no implementado — |
| 57-58 | BATTERY_50/25 | 🟠 Advertencia | — no implementados — |
| 59 | DISTANCE_1KM | 🔵 Informativo | — no implementado — |
| 60 | ENERGY_SAVE | 🔵 Informativo | — no implementado — |
| 61-63 | MODE_ECO/NORMAL/SPORT | 🔵 Informativo | — no implementados — |
| 64-68 | CONFIG_*/BEEP | 🔵 Informativo | Feedback de configuración |

---

## 7. Matriz de prioridad de interrupción

### 7.1 Reglas actuales de interrupción

```
Prioridad actual:  HIGH (2) > MEDIUM (1) > LOW (0)
Regla: si priority_new ≥ priority_current → preempt (sobreescribir)
       si priority_new <  priority_current → descartado

Cooldown: 4 segundos por track (excepto HIGH que siempre suena)
Burst collapse: ≥2 warnings en <2s → ERROR_GENERAL (HIGH)
```

### 7.2 Análisis de correctitud de la cadena de interrupción

| Situación | ¿Qué debería oírse? | ¿Qué se oye realmente? | ¿Correcto? |
|---|---|---|---|
| WELCOME jugando + EMERGENCY llega | EMERGENCY interrumpe WELCOME | EMERGENCY (HIGH ≥ HIGH → preempt) | ✅ |
| GEAR_D1 jugando + OBSTACLE_WARN llega | OBSTACLE interrumpe GEAR | OBSTACLE (MEDIUM ≥ LOW → preempt) | ✅ |
| OBSTACLE jugando + GEAR_D2 llega | OBSTACLE sigue, gear se descarta | OBSTACLE continúa (LOW < MEDIUM → skip) | ✅ |
| TEMP_HIGH jugando + BATTERY_LOW llega | BATTERY reemplaza TEMP | BATTERY (MEDIUM ≥ MEDIUM → preempt) | ⚠️ Se pierde info de temperatura |
| EMERGENCY jugando + OBSTACLE llega | EMERGENCY sigue | EMERGENCY continúa (MEDIUM < HIGH → skip) | ✅ |
| BATTERY_CRITICAL + OVERCURRENT simultáneos | Ambos son HIGH, burst collapse | `ERROR_GENERAL` | 🔴 Se pierde la info específica |
| ABS_ON + TCS_ON simultáneos (misma iteración) | Ambos LOW | Sólo el segundo (TCS_ON) queda en pending | ⚠️ ABS_ON se pierde (single-slot queue) |

### 7.3 Prioridad de interrupción recomendada

```
Nivel 0 (Máxima urgencia — NUNCA se interrumpe):
  └─ EMERGENCY (31): "Motor deshabilitado"
  └─ BATTERY_CRITICAL (13): "Desconectando tracción"

Nivel 1 (Alta — solo interrumpido por nivel 0):
  └─ OVERCURRENT (53): "Corriente excesiva"
  └─ ERROR_GENERAL (3): "Error general" (cuando se usa como burst collapse)

Nivel 2 (Media — interrumpido por niveles 0-1):
  └─ TEMP_HIGH (10): "Temperatura elevada"
  └─ BATTERY_LOW (12): "Batería baja"
  └─ OBSTACLE_WARN (54): "Obstáculo detectado"
  └─ SENSOR_*_ERROR (33-35): Errores de sensores
  └─ ENCODER_ERROR (9): "Error de dirección"
  └─ PEDAL_ERROR (5): "Error de pedal" (actualmente no implementado)

Nivel 3 (Baja — interrumpido por todo):
  └─ WELCOME (1), FAREWELL (2)
  └─ SAFETY_RESET (32)
  └─ ABS/TCS ON/OFF (39-42)
  └─ GEAR_* (20-24)
  └─ LIGHTS_ON/OFF (16-17)
  └─ TRACTION_4X4/4X2 (37-38)
  └─ BEEP (68)
```

**Nota:** El sistema actual tiene solo 3 niveles (LOW/MEDIUM/HIGH). La tabla recomendada sugiere 4 niveles para distinguir entre "emergencia inmediata" y "error alto". Sin embargo, con 3 niveles el sistema es funcional si se corrigen las asignaciones.

---

## 8. Hallazgos específicos y riesgos

### 8.1 Hallazgos críticos (requieren acción)

| ID | Hallazgo | Riesgo | Severidad |
|----|----------|--------|-----------|
| F-01 | `SENSOR_FAULT` mapea siempre a `SENSOR_SPEED_ERROR` — cubre pedal, encoder y velocidad | Conductor recibe información incorrecta sobre pedal/dirección | 🔴 ALTA |
| F-02 | Per-motor temp cutoff (130°C) sin audio — pérdida de tracción asimétrica silenciosa | Cambio de trayectoria sin aviso | 🔴 ALTA |
| F-03 | CAN bus-off no tiene mapping de audio en el switch de error codes | Pérdida de comunicación sin aviso específico | 🔴 ALTA |
| F-04 | Burst collapse oculta información específica de errores simultáneos | "Error general" cuando el conductor necesita saber qué ocurre | 🟠 MEDIA-ALTA |
| F-05 | Thresholds de temperatura desincronizados (ESP32: 85°C vs STM32: 80°C) | Ventana de 5°C sin aviso de temperatura | 🟠 MEDIA |
| F-06 | Recovery DEGRADED→ACTIVE y LIMP_HOME→ACTIVE no producen SAFETY_RESET | Conductor no sabe que el sistema se ha recuperado | 🟠 MEDIA |

### 8.2 Hallazgos menores (mejora recomendada)

| ID | Hallazgo | Riesgo | Severidad |
|----|----------|--------|-----------|
| F-07 | LIMP_HOME usa audio genérico ERROR_GENERAL — no indica que se puede conducir | Conductor puede detenerse innecesariamente | 🟡 BAJA-MEDIA |
| F-08 | Tank turn (360°) solo reproduce BEEP — modo peligroso sin anuncio verbal | Posible confusión sobre el modo activo | 🟡 BAJA |
| F-09 | 32 de 68 tracks definidos no tienen trigger en el código (WiFi, BT, telemetría, modos, etc.) | Funcionalidad no implementada pero definida | 🔵 BAJA |
| F-10 | STANDBY→ACTIVE no tiene audio de confirmación | Conductor no sabe cuándo el sistema está listo | 🔵 BAJA |
| F-11 | Single-slot pending queue puede perder audio cuando dos eventos LOW ocurren en el mismo ciclo | Evento silenciado (ej: ABS_ON perdido si TCS_ON llega en el mismo loop) | 🔵 BAJA |
| F-12 | Startup inhibit y LIMP_HOME pedal arming no tienen audio | Conductor confuso cuando pedal no responde | 🟡 BAJA-MEDIA |

### 8.3 Tracks del firmware original SIN trigger en el código actual

Los siguientes tracks están definidos en `audio_manager.h` pero **nunca se llaman** desde `main.cpp`:

| Track | Sonido | ¿Por qué no se usa? |
|-------|--------|---------------------|
| 4 | PEDAL_OK | No hay evento de calibración de pedal en runtime |
| 5 | PEDAL_ERROR | Error de pedal usa SENSOR_SPEED_ERROR por mapping genérico |
| 6 | INA_OK | No hay calibración de INA en runtime |
| 7 | INA_ERROR | I2C_FAILURE mapea a SENSOR_CURRENT_ERROR |
| 8 | ENCODER_OK | No hay confirmación de encoder en runtime |
| 14-15 | PARKING_BRAKE_ON/OFF | No hay freno de estacionamiento implementado |
| 18-19 | RADIO_ON/OFF | No hay radio implementada |
| 25-28 | MENU_HIDDEN / CAL_* | Podrían usarse en engineering screen pero no hay triggers |
| 29-30 | TEST_SYSTEM/OK | No hay función de test con audio |
| 36 | MODULE_OK | No hay confirmación individual de módulos |
| 43-44 | REGEN_ON/OFF | No hay frenado regenerativo con audio |
| 45-51 | WiFi / BT | Funcionalidad eliminada en v2.11.0 (standalone) |
| 52 | MAX_SPEED | No hay detección de velocidad máxima |
| 55 | PARKING_ASSIST | No implementado |
| 56 | SOFT_START | No hay audio para arranque suave |
| 57-60 | BATTERY_50/25 / DISTANCE / ENERGY_SAVE | Telemetría sin audio |
| 61-63 | MODE_ECO/NORMAL/SPORT | Modos de conducción no implementados |
| 64-67 | CONFIG_SAVED / RESTORED / CLEARED / REGEN_ADJ | Config store no genera audio |

---

## 9. Recomendaciones

### 9.1 Correcciones críticas (prioridad inmediata)

| # | Recomendación | Hallazgo relacionado |
|---|---|---|
| R-01 | Diferenciar `SENSOR_FAULT` por subsistema: usar los fault flags de byte 2 del heartbeat para distinguir entre `FAULT_ENCODER_ERROR`, `FAULT_WHEEL_SENSOR`, y pedal fault. Mapear a `PEDAL_ERROR (5)`, `ENCODER_ERROR (9)`, o `SENSOR_SPEED_ERROR (35)` según corresponda. | F-01 |
| R-02 | Añadir audio para per-motor temperature cutoff: cuando `OVERTEMP` error code aparece pero el sistema NO está en SAFE, reproducir `TEMP_HIGH (10)` si no se ha reproducido ya. El STM32 debería incluir una flag o sub-código para cutoff per-motor. | F-02 |
| R-03 | Añadir `CAN_BUSOFF (13)` al switch de error codes en main.cpp. Mapear a un audio apropiado (podría ser `ERROR_GENERAL` con texto "comunicación perdida" o un nuevo track). | F-03 |
| R-04 | Sincronizar threshold de temperatura ESP32 a 80°C para que coincida con el STM32 `TEMP_WARNING_C`. Ajustar hysteresis a 75°C (coincide con STM32: 80-5=75). | F-05 |
| R-05 | Ampliar el trigger de `SAFETY_RESET` para incluir transiciones desde DEGRADED y LIMP_HOME, no solo desde SAFE/ERROR. | F-06 |

### 9.2 Mejoras de seguridad (prioridad alta)

| # | Recomendación | Hallazgo relacionado |
|---|---|---|
| R-06 | Mejorar burst collapse: en vez de reemplazar todo por ERROR_GENERAL, reproducir el error de **mayor severidad** del burst (EMERGENCY > OVERCURRENT > BATTERY_CRITICAL > ...). El burst collapse debería agrupar, no silenciar. | F-04 |
| R-07 | Crear un audio específico para LIMP_HOME que indique al conductor que puede conducir a baja velocidad. Texto sugerido: "Modo de emergencia. Velocidad limitada a 5 kilómetros por hora." | F-07 |
| R-08 | Añadir audio para startup inhibit: reproducir `PEDAL_ERROR (5)` o un audio específico cuando el heartbeat indique `STARTUP_INHIBIT` activo (bit 0 de status_flags byte 4 del heartbeat). | F-12 |

### 9.3 Mejoras de experiencia (prioridad normal)

| # | Recomendación | Hallazgo relacionado |
|---|---|---|
| R-09 | Reproducir `BEEP (68)` o `MODULE_OK (36)` cuando el sistema pasa de STANDBY a ACTIVE, como confirmación de "sistema listo". | F-10 |
| R-10 | Usar `TRACTION_4X4 (37)` para modo 360° tank turn, o crear un audio verbal para este modo. | F-08 |
| R-11 | Implementar escalamiento de obstacle warning: reducir el intervalo de 2s a 500ms cuando zone == 4 (emergencia). | SPAM-02 |
| R-12 | Para ABS/TCS chattering: considerar no reproducir el audio de desactivación (ABS_OFF/TCS_OFF) — solo el de activación importa al conductor. | SPAM-01 |

---

## Apéndice A: Flujo completo de datos para un evento crítico

### Ejemplo: Overcurrent persistente (3 eventos consecutivos)

```
T=0.0s  Motor FL: 26.0A (> 25.0A MAX_CURRENT_A)
        STM32: Safety_CheckCurrent() → consecutive_errors=1
        STM32: Safety_SetError(OVERCURRENT) → error_code=1
        STM32: Safety_SetState(DEGRADED) → system_state=3
        CAN TX 0x001: [counter, 3, 0x04, 1, flags]
                                  ^DEGRADED ^OVERCURRENT
        CAN TX 0x203: [abs=0, tcs=0, error=1]

T=0.1s  ESP32 CAN RX: heartbeat → systemState=DEGRADED(3)
        ESP32 audio: st != lastAudioSystemState → playWarning(ERROR_GENERAL, HIGH)
        ESP32 audio: errorCode=1 → playWarning(OVERCURRENT, HIGH)
        ↳ Burst: errorBurstCount=2, dentro de 2s window
        ↳ Burst collapse: ≥2 → audio::play(ERROR_GENERAL, HIGH) × 2 llamadas
        🔊 Conductor oye: "Atención. Se ha detectado un error general."

T=0.1s  Motor FL: 26.5A
        STM32: consecutive_errors=2, DEGRADED L3

T=0.2s  Motor FL: 27.0A
        STM32: consecutive_errors=3 → ≥ THRESHOLD
        STM32: Safety_SetState(SAFE) → Safety_FailSafe() → motors OFF
        CAN TX 0x001: [counter, 4, 0x04, 1, flags]
                                  ^SAFE

T=0.3s  ESP32 CAN RX: heartbeat → systemState=SAFE(4)
        ESP32 audio: st != lastAudioSystemState → playWarning(EMERGENCY, HIGH)
        ↳ Burst: errorBurstCount=3, dentro de 2s
        ↳ Burst collapse: audio::play(ERROR_GENERAL, HIGH)
        🔊 Conductor oye: "Atención. Se ha detectado un error general." (de nuevo)

RESULTADO: El conductor NUNCA oye "Modo de emergencia activado. Motor deshabilitado."
           ni "Corriente excesiva detectada."
           Solo oye "Error general" repetido.
```

**Este es el hallazgo más grave de la auditoría.**

---

## Apéndice B: Mapa de error codes STM32 → Audio ESP32

| Safety_Error_t (STM32) | Valor | ¿En switch ESP32? | Audio ESP32 | ¿Correcto? |
|---|---|---|---|---|
| NONE | 0 | — (clear) | — | ✅ |
| OVERCURRENT | 1 | ✅ | OVERCURRENT (53) | ✅ |
| OVERTEMP | 2 | ✅ | SENSOR_TEMP_ERROR (33) | ✅ |
| CAN_TIMEOUT | 3 | ❌ | — | ⚠️ Cubierto por systemState change |
| SENSOR_FAULT | 4 | ✅ | SENSOR_SPEED_ERROR (35) | 🔴 Genérico incorrecto |
| MOTOR_STALL | 5 | ❌ | — | ⚠️ No implementado en STM32 |
| EMERGENCY_STOP | 6 | ❌ | — | ⚠️ Cubierto por ERROR state |
| WATCHDOG | 7 | ❌ | — | ⚠️ Provoca reset, no hay CAN |
| CENTERING | 8 | ✅ | ENCODER_ERROR (9) | ✅ |
| BATTERY_UV_WARNING | 9 | ❌ | — | ⚠️ Cubierto por voltageRaw check |
| BATTERY_UV_CRITICAL | 10 | ❌ | — | ⚠️ Cubierto por voltageRaw check + state |
| I2C_FAILURE | 11 | ✅ | SENSOR_CURRENT_ERROR (34) | ⚠️ Parcial |
| OBSTACLE | 12 | ✅ | OBSTACLE_WARN (54) | ✅ |
| CAN_BUSOFF | 13 | ❌ | — | 🔴 Sin audio |

---

*Fin del documento de auditoría.*  
*Siguiente paso: implementación de correcciones según las recomendaciones priorizadas.*
