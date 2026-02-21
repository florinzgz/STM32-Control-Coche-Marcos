# Verificación del Firmware vs. Original — Faltantes Reales

**Fecha:** 2026-02-21  
**Fuente de verdad:** `docs/FINAL_PRE_PHYSICAL_AUDIT.md`  
**Firmware original:** FULL-FIRMWARE-Coche-Marcos v2.17.1  
**Firmware actual:** STM32-Control-Coche-Marcos (STM32G474RE + ESP32-S3)

---

## 1) Verificación: Estado del documento vs. código real

Cada afirmación de `FINAL_PRE_PHYSICAL_AUDIT.md` fue verificada contra el código fuente actual:

| Afirmación del documento | Archivo verificado | Línea(s) | ¿Coincide? |
|--------------------------|--------------------|----------|------------|
| A.1 — 0 fallos de compilación | Makefile, todos los .c/.h | — | ✅ Confirmado |
| A.2 — HAL drivers en .gitignore | .gitignore, Drivers/ | — | ✅ Confirmado |
| B.1 — Código muerto Ackermann_Compute | motor_control.c | 1689, 1716 | ✅ Confirmado (definido, nunca llamado) |
| B.2 — safety_error sin mutex | safety_system.c/h | — | ✅ Confirmado (safe en single-core) |
| B.3 — startup_pedal_rest_since = 0 | main.c | 94 | ✅ Confirmado (correcto) |
| B.4 — Sin ESP32 → LIMP_HOME | safety_system.c | 964-975 | ✅ Confirmado |
| B.5 — CAN loss en cada estado | safety_system.c | 948-1030 | ✅ Confirmado (tabla correcta) |
| B.6 — LIMP_HOME→ACTIVE limpia solo CAN_TIMEOUT | safety_system.c | 1002-1005 | ✅ Confirmado |
| B.7 — ESP32 NO envía 0x100/0x101/0x102/0x110 | esp32/src/ (todos) | — | ✅ Confirmado (solo envía 0x011 y 0x208) |
| B.8 — ESP32 sin input de usuario | esp32/src/main.cpp, ui/ | 186 | ⚠️ Parcialmente correcto (ver nota 1) |
| C.1 — 7 funcionalidades NO portadas | Comparación completa | — | ✅ Confirmado |
| C.2 — 4 funcionalidades parciales | ESP32 src/ | — | ✅ Confirmado |
| C.3 — 4 funcionalidades eliminadas | Arquitectura | — | ✅ Confirmado |
| D — Pasos 2,3,4,5,6 completados | Ver evidencia abajo | — | ✅ Confirmado |
| D — Pasos 1,7 pendientes | safety_system.c | 1291-1312 | ✅ Confirmado |

### Nota 1 — Corrección menor a B.8

El documento afirma que las pantallas "no tienen handlers de touch". Esto es **casi correcto pero incompleto**:

- `esp32/src/main.cpp` línea 186: `tft.getTouch(&touchX, &touchY)` — **existe** pero solo se usa dentro de `#if RUNTIME_MONITOR` para activar el debug overlay con pulsación larga (3 segundos).
- `esp32/src/ui/mode_icons.cpp` línea 45: `ModeIcons::hitTest()` — **existe** como función definida, pero **nunca se llama** desde ningún archivo. Es código muerto.
- **No existe ningún código que envíe comandos CAN como resultado de un touch.** La pantalla es efectivamente solo lectura para el control del vehículo.

**Conclusión:** La afirmación B.8 es funcionalmente correcta — no hay control de usuario vía touch — pero la redacción "no tienen handlers de touch" debería decir "los handlers de touch existentes no están conectados a control del vehículo".

---

## 2) Verificación de pasos completados — Evidencia en código

| Paso | Código verificado | Evidencia exacta | ¿Real? |
|------|-------------------|-------------------|--------|
| **Paso 2** — STANDBY→LIMP_HOME sin centering | `safety_system.c` L964-975 | `if (system_state == SYS_STATE_STANDBY && BootValidation_IsPassed())` — sin `Steering_IsCalibrated()` | ✅ |
| **Paso 3** — Bypass centering para ACTIVE | `safety_system.c` L990-994 | `(Steering_IsCalibrated() || !ServiceMode_IsEnabled(MODULE_STEER_CENTER))` | ✅ |
| **Paso 4** — Pedal plausibility fail-op | `sensor_manager.c` L291-292 | `Pedal_IsPlausible()`, `Pedal_IsContradictory()` definidas; ADS1115 cross-validation L193-287 | ✅ |
| **Paso 5** — Driver obstáculo ESP32 | `esp32/src/sensors/obstacle_sensor.cpp` + `can/can_obstacle.cpp` | HC-SR04 driver, CAN 0x208 TX cada 66ms con zone/health/counter | ✅ |
| **Paso 6** — Persistencia flash calibración | `Core/Src/steering_cal_store.c` | `SteeringCal_Save()` L147-198, `SteeringCal_ValidateAtBoot()` L96-135, CRC32 L52-66 | ✅ |

---

## 3) Discrepancia detectada en documentación base

La tabla de `FAIL_OPERATIONAL_MIGRATION_AUDIT.md` §2.8 (línea 378) dice:

> | Calibration persistence | `eeprom_persistence.cpp` | Not present; re-centering required every boot | MISSING — see risk R2 |

Esto ya **no es correcto** — `steering_cal_store.c` implementa persistencia flash con CRC32 (Paso 6 completado). El documento de migración fue escrito antes de la implementación del Paso 6. `FAIL_OPERATIONAL_NEXT_STEPS.md` refleja correctamente el estado actual (Paso 6 ✅).

**Nota:** Esto NO es un error en FINAL_PRE_PHYSICAL_AUDIT.md (que sí lista Paso 6 como completado). Es una inconsistencia residual en el documento de referencia antiguo.

---

## 4) Funcionalidades faltantes del firmware original — Lista completa verificada

Comparación exhaustiva contra la estructura del repo original (sección 2 de `ORIGINAL_REPO_COMPARATIVE_AUDIT.md`) y el estado real del código actual.

### Tabla de funcionalidades faltantes

| Funcionalidad faltante | Archivo original | Categoría | Necesario para conducir | Prioridad real |
|------------------------|-----------------|-----------|------------------------|----------------|
| CAN TX comandos desde ESP32 (throttle 0x100, steering 0x101, mode 0x102) | N/A (nueva arquitectura) | Control del vehículo | SÍ — sin esto el ESP32 no puede mandar comandos al STM32 | ALTA |
| CAN TX service command desde ESP32 (0x110) | N/A (nueva arquitectura) | Mantenimiento/diagnóstico | NO — vehículo conduce sin service menu | MEDIA |
| Touch/botones conectados a acciones de control | `input/buttons.cpp`, `input/steering.cpp` | HMI | SÍ — operador necesita enviar comandos | ALTA |
| Decisión formal relay-en-SAFE (Paso 1) | `SafetyManager.cpp` (EMERGENCY → restart) | Seguridad | NO — vehículo conduce sin decisión, pero riesgo R1 abierto | ALTA (documentación + prueba física) |
| Corte de relés en SAFE (Paso 7, condicional) | `SafetyManager.cpp` (EMERGENCY → restart) | Seguridad | NO — depende del resultado del Paso 1 | ALTA (condicional) |
| AI Regenerative Braking | `safety/regen_ai.cpp` | Control del vehículo | NO — frenado dinámico proporcional funciona | BAJA |
| Adaptive Cruise Control | `control/adaptive_cruise.cpp` | Control del vehículo | NO — función extra, no safety-critical | BAJA |
| Config persistence (NVS) | `core/config_storage.cpp`, `eeprom_persistence.cpp` | Mantenimiento/diagnóstico | NO — config se reinicia cada boot pero vehículo funciona | MEDIA |
| Service mode persistence | `core/config_storage.cpp` | Mantenimiento/diagnóstico | NO — service settings se pierden al reiniciar | BAJA |
| Audio (DFPlayer alerts) | `audio/dfplayer.cpp`, `alerts.cpp`, `queue.cpp` | Extras | NO — sin hardware de audio | BAJA (sin hardware) |
| LED WS2812B iluminación | `lighting/led_controller.cpp` | Extras | NO — sin hardware de LEDs | BAJA (sin hardware) |
| Menú debug interactivo | `hud/menu_hidden.cpp` (46 KB) | Mantenimiento/diagnóstico | NO — debug overlay existe como alternativa | BAJA |
| Menú LED control | `hud/led_control_menu.cpp`, `menu_led_control.cpp` | Extras | NO — sin hardware de LEDs | BAJA (sin hardware) |
| Menú sensor config | `hud/menu_sensor_config.cpp` | Mantenimiento/diagnóstico | NO — service mode reemplaza parcialmente | BAJA |
| Menú power config | `hud/menu_power_config.cpp` | Mantenimiento/diagnóstico | NO — no hay equivalente | BAJA |
| Menú encoder calibration | `hud/menu_encoder_calibration.cpp` | Mantenimiento/diagnóstico | NO — centering automático + service bypass lo reemplaza | BAJA |
| Menú obstacle config | `menu/menu_obstacle_config.cpp` | Mantenimiento/diagnóstico | NO — config hardcoded en STM32 | BAJA |
| Logger serial completo | `core/logger.cpp` | Mantenimiento/diagnóstico | NO — ServiceMode_SetFault() es alternativa parcial | BAJA |
| Test suite automatizada | `test/` (7 archivos) | Mantenimiento/diagnóstico | NO — solo host tests de math_safety y steering_cal_store existen | MEDIA |
| DS18B20 hot-plug | `sensors/temperature.cpp` | Mantenimiento/diagnóstico | NO — ROM search solo al init | BAJA |
| CAN 0x209 decode body | N/A (nuevo) | Control del vehículo | NO — filtro existe, body vacío, informacional | BAJA |
| Sprite compositor / render engine | `hud/hud_compositor.cpp`, `render_engine.cpp` | HMI | NO — partial-redraw simple funciona | BAJA |
| Touch calibration | `hud/touch_calibration.cpp` | HMI | NO — calibración no necesaria si touch no se usa para control todavía | BAJA |
| NVS bootloop counter | `core/boot_guard.cpp` | Seguridad | NO — reset cause classification (IWDG/BROWNOUT) lo reemplaza | BAJA |
| HUD limp diagnostics detallado | `hud/hud_limp_diagnostics.cpp`, `hud_limp_indicator.cpp` | HMI | NO — safe/error screens muestran faults | BAJA |

---

## 5) Resumen por categoría

### Seguridad (2 items — ambos del roadmap existente)

| # | Item | Estado | Acción |
|---|------|--------|--------|
| 1 | Paso 1: Decisión relay-en-SAFE | ⚠️ PENDIENTE | Prueba física + documento formal |
| 2 | Paso 7: Corte relés en SAFE | ❌ PENDIENTE | Solo si Paso 1 determina que EN-pin no basta |

### Control del vehículo — necesario para conducir con ESP32 (3 items)

| # | Item | Estado | Acción |
|---|------|--------|--------|
| 1 | ESP32 CAN TX: throttle (0x100), steering (0x101), mode (0x102) | ❌ FALTA | Nuevo `esp32/src/can/can_tx.cpp` |
| 2 | ESP32 input usuario: touch/botones → comandos | ❌ FALTA | Conectar hitTest() y touch a can_tx |
| 3 | CAN 0x209 decode body | ❌ FALTA | Bajo, informacional |

**Nota:** Sin los items 1 y 2, el vehículo SOLO puede conducirse con pedal local en LIMP_HOME (20% torque, 5 km/h). Para modo ACTIVE con ESP32, se necesitan los CAN TX.

### HMI (3 items)

| # | Item | Estado | Acción |
|---|------|--------|--------|
| 1 | Touch calibration | ❌ FALTA | Necesario antes de input táctil fiable |
| 2 | HUD limp diagnostics detallado | ❌ FALTA | Mejora visual, no bloquea conducción |
| 3 | Sprite compositor / render engine | ❌ FALTA | Opcional, actual es funcional |

### Mantenimiento/diagnóstico (8 items)

| # | Item | Estado | Acción |
|---|------|--------|--------|
| 1 | CAN TX service command (0x110) desde ESP32 | ❌ FALTA | Parte de can_tx.cpp |
| 2 | Config persistence (NVS general) | ❌ FALTA | RAM-only actualmente |
| 3 | Service mode persistence | ❌ FALTA | RAM-only |
| 4 | Logger serial completo | ❌ FALTA | ServiceMode es alternativa parcial |
| 5 | Test suite automatizada | ⚠️ PARCIAL | Solo math_safety y steering_cal_store |
| 6 | Menús interactivos (sensor/power/debug/obstacle config) | ❌ FALTA | Requieren touch + service menu |
| 7 | Menú encoder calibration | ❌ FALTA | Centering automático lo reemplaza |
| 8 | DS18B20 hot-plug | ❌ FALTA | Solo ROM search al init |

### Extras — hardware no presente (3 items)

| # | Item | Estado | Acción |
|---|------|--------|--------|
| 1 | Audio DFPlayer | ❌ FALTA | Sin hardware en sistema actual |
| 2 | LED WS2812B | ❌ FALTA | Sin hardware en sistema actual |
| 3 | Menú LED control | ❌ FALTA | Sin hardware |

---

## 6) Tabla final consolidada

| Funcionalidad faltante | Archivo original | Necesario para conducir | Prioridad real |
|------------------------|-----------------|------------------------|----------------|
| ESP32 CAN TX comandos (0x100, 0x101, 0x102) | N/A (nueva arch.) | SÍ (para ACTIVE con ESP32) | **ALTA** |
| ESP32 input touch → comandos | `input/buttons.cpp` | SÍ (para ACTIVE con ESP32) | **ALTA** |
| Paso 1: Decisión relay-en-SAFE | `SafetyManager.cpp` | NO (funciona sin decisión) | **ALTA** (seguridad) |
| Paso 7: Corte relés en SAFE | `SafetyManager.cpp` | NO (condicional) | **ALTA** (condicional) |
| ESP32 CAN TX service (0x110) | N/A (nueva arch.) | NO | MEDIA |
| Config persistence NVS | `config_storage.cpp` | NO | MEDIA |
| Test suite completa | `test/` (7 archivos) | NO | MEDIA |
| AI Regenerative Braking | `regen_ai.cpp` | NO | BAJA |
| Adaptive Cruise Control | `adaptive_cruise.cpp` | NO | BAJA |
| Service mode persistence | `config_storage.cpp` | NO | BAJA |
| Logger serial | `logger.cpp` | NO | BAJA |
| Menús interactivos | `menu_*.cpp` (6 archivos) | NO | BAJA |
| CAN 0x209 decode body | N/A | NO | BAJA |
| DS18B20 hot-plug | `temperature.cpp` | NO | BAJA |
| Touch calibration | `touch_calibration.cpp` | NO | BAJA |
| HUD limp diagnostics | `hud_limp_*.cpp` | NO | BAJA |
| Sprite compositor | `hud_compositor.cpp` | NO | BAJA |
| NVS bootloop counter | `boot_guard.cpp` | NO | BAJA |
| Audio DFPlayer | `dfplayer.cpp` + 2 | NO (sin hardware) | BAJA |
| LED WS2812B | `led_controller.cpp` | NO (sin hardware) | BAJA |
| Menú LED control | `led_control_menu.cpp` | NO (sin hardware) | BAJA |

---

## 7) Conclusión

El documento `FINAL_PRE_PHYSICAL_AUDIT.md` **coincide con el código real** en todos los puntos verificados excepto una corrección menor en B.8 (touch existe parcialmente pero no está conectado a control).

**Para poder conducir el vehículo en modo ACTIVE con ESP32:**
- Se necesitan los **2 items de prioridad ALTA de control**: ESP32 CAN TX + touch/input handler
- Sin ellos, el vehículo solo funciona en LIMP_HOME con pedal local (20% torque, 5 km/h)

**Para seguridad:**
- Se necesita completar los **Pasos 1 y 7** del roadmap existente (sin cambiar orden)

**Todo lo demás son extras no necesarios para conducir.**
