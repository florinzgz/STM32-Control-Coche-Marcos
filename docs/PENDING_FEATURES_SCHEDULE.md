# Funcionalidades Pendientes: Menú Oculto y Persistencia EEPROM

**Fecha:** 2026-02-15
**Referencia:** `docs/PROJECT_MASTER_STATUS.md` §4–§5, `docs/FIRMWARE_MATURITY_ROADMAP.md` §5

---

## RESUMEN

| Funcionalidad | ¿Cuándo se implementa? | Phase | Depende de |
|--------------|------------------------|-------|------------|
| **Persistencia de calibración (EEPROM/Flash)** | **Phase 4 — Driver Interaction** | Después de Phases 1, 2 y 3 | STM32 Flash driver |
| **Persistencia de Service Mode (EEPROM/Flash)** | **Phase 4 — Driver Interaction** | Después de Phases 1, 2 y 3 | STM32 Flash driver |
| **Menú oculto de ingeniería (ESP32)** | **Phase 5 — Experience Features** | Después de Phases 1, 2, 3 y 4 | Persistencia implementada, UI funcional |

---

## 1) PERSISTENCIA EN EEPROM / FLASH STM32

### Estado actual

El STM32G474RE **NO tiene EEPROM dedicada**, pero sí tiene **Flash interna** con páginas de usuario que se pueden usar como almacenamiento no volátil. Actualmente:

- **Calibración de dirección** (encoder zero, offset de centrado) → se recalcula cada vez que se enciende (`Steering_Init()`, `SteeringCentering_Complete()`)
- **Service Mode** (módulos habilitados/deshabilitados) → se pierde al apagar (`module_enabled[]` es un array estático en RAM, `service_mode.c`)
- **Offsets de sensores** (pedal min/max, etc.) → hardcodeados como constantes

**Evidencia en código:**
- No hay ninguna llamada a `HAL_FLASH_Program()`, `HAL_FLASH_Erase()` ni API de escritura Flash en ningún archivo STM32
- No existe ningún driver de EEPROM emulada ni abstracción de almacenamiento

### Cuándo se implementa

**Phase 4 — Driver Interaction** (según `PROJECT_MASTER_STATUS.md` §5):

> **Goals:**
> - Add calibration persistence (steering encoder zero, sensor offsets) to STM32 flash
> - Add service mode persistence to STM32 flash

**Exit criteria:**
> - Steering encoder zero is preserved across power cycles (verified by skipping centering when already calibrated)
> - Service mode enable/disable settings survive reboot

### Qué se va a almacenar

| Dato | Tamaño | Módulo afectado |
|------|--------|----------------|
| Steering encoder zero offset | 4 bytes (int32_t) | `motor_control.c` → `Steering_Init()` |
| Steering centering done flag | 1 byte (bool) | `steering_centering.c` |
| Pedal ADC min/max calibración | 4 bytes (2× uint16_t) | `sensor_manager.c` → `Pedal_Update()` |
| Service mode enable bitmask | 4 bytes (uint32_t) | `service_mode.c` → `ServiceMode_Init()` |
| CRC-32 de integridad | 4 bytes | Nuevo |
| **Total** | **~17 bytes** | — |

### Diseño propuesto (de FIRMWARE_MATURITY_ROADMAP.md §5 item 3.3)

> Store calibration values (steering center offset, pedal min/max, enabled modules) in STM32 Flash user pages. Load on boot, save on service command. Add CRC-32 for data integrity.

**Nota:** El STM32G474RE tiene 512 KB de Flash. Las últimas 2 páginas (2 KB cada una) se pueden reservar para datos de configuración sin afectar al firmware.

### Por qué NO se hace ahora

Según las reglas del proyecto (`PROJECT_MASTER_STATUS.md` §6 regla 3):

> **"It implements a feature from a future phase"** — Phase N+1 work is blocked until all Phase N exit criteria are objectively met and documented.

La persistencia es **Phase 4**. Primero hay que completar:

1. **Phase 1** — Validación de estabilidad en hardware real
2. **Phase 2** — Fiabilidad del pipeline de control (pedal → motor)
3. **Phase 3** — Sensores de obstáculo, PID tuning, hot-plug DS18B20

---

## 2) MENÚ OCULTO DE INGENIERÍA

### Estado actual

**NO IMPLEMENTADO** en ninguno de los dos MCU.

### Qué tenía el repo original

El repo original (`FULL-FIRMWARE-Coche-Marcos`) tenía un menú oculto completo en `hud/menu_hidden.cpp` (46 KB) con:
- Código secreto de acceso: **8989**
- Calibración de pedal (min/max ADC)
- Calibración de encoder de dirección
- Habilitación/deshabilitación de módulos
- Factory restore
- Visor de log de errores
- Monitor de INA226 en tiempo real
- Configuración de sensores
- Configuración de potencia

### Cuándo se implementa

El menú oculto está planificado en **dos documentos diferentes** con diferente prioridad:

| Documento | Ubicación | Phase |
|-----------|-----------|-------|
| `PROJECT_MASTER_STATUS.md` | No listado explícitamente como goal de ninguna phase | Implícito en Phase 5 |
| `FIRMWARE_MATURITY_ROADMAP.md` | Item 4.3 | Phase 4 del roadmap |

**Propuesta de ubicación:** El menú oculto encaja mejor en **Phase 5 — Experience Features** del `PROJECT_MASTER_STATUS.md`, porque:
- No es safety-critical (clasificado como LOW risk en `FIRMWARE_MATURITY_ROADMAP.md` §7)
- El Service Mode ya es accesible por CAN commands (0x110)
- Requiere que la persistencia (Phase 4) esté implementada primero para que los cambios se guarden
- Es una mejora de experiencia de usuario, no de funcionalidad core

### Qué incluiría

Basado en el diseño original y adaptado a la arquitectura actual (dual-MCU):

| Función del menú | Dónde se ejecuta | CAN interaction |
|------------------|------------------|-----------------|
| Ver telemetría por rueda | ESP32 display | Lee CAN 0x200–0x206 (ya existe) |
| Ver salud de sensores | ESP32 display | Lee CAN 0x203, 0x301 (ya existe) |
| Habilitar/deshabilitar módulos | ESP32 → STM32 | Envía CAN 0x110 SERVICE_CMD (ya existe) |
| Factory restore | ESP32 → STM32 | Envía CAN 0x110 con action=0xFF (ya existe) |
| Calibrar pedal (min/max) | ESP32 → STM32 | Necesita nuevo CAN message o extensión de 0x110 |
| Calibrar encoder dirección | ESP32 → STM32 | Necesita nuevo CAN message o extensión de 0x110 |
| Ver log de errores | ESP32 display | Lee CAN 0x300 DIAG_ERROR (ya existe) |
| Acceso por código secreto | ESP32 local | Touch screen pattern o long-press |

**Nota:** Gran parte de la infraestructura CAN ya existe. El menú oculto es principalmente trabajo de **UI en el ESP32**, no de firmware STM32.

### Por qué NO se hace ahora

1. **Phase ordering** — Hay que completar Phases 1-4 primero
2. **Dependencia de persistencia** — Sin EEPROM/Flash, los cambios del menú no se guardan
3. **Prioridad LOW** — No afecta a la seguridad ni al funcionamiento del vehículo
4. **Service Mode ya funciona** — Se puede acceder a las mismas funciones por CAN desde un terminal serie

---

## 3) ORDEN DE IMPLEMENTACIÓN

```
AHORA       → Phase 1: Validación hardware (ver HARDWARE_VALIDATION_PROCEDURE.md)
             │
DESPUÉS     → Phase 2: Control pipeline end-to-end
             │
LUEGO       → Phase 3: Obstacle sensor + PID + hot-plug DS18B20
             │
ENTONCES    → Phase 4: ★ PERSISTENCIA EEPROM/FLASH ★
             │          + Gear/mode display en DriveScreen
             │          + ACK visual de cambio de modo
             │
POR ÚLTIMO  → Phase 5: ★ MENÚ OCULTO ★
                        + Audio (si hay hardware)
                        + LEDs (si hay hardware)
                        + Sensor fusion
                        + ADC no-bloqueante
```

### Estimación de esfuerzo

| Feature | Esfuerzo | Justificación |
|---------|----------|---------------|
| Flash driver STM32 (read/write/erase) | 1–2 días | HAL_FLASH API estándar, CRC-32 simple |
| Calibration persistence | 1 día | Integrar flash driver en `Steering_Init()` y `sensor_manager.c` |
| Service mode persistence | 0.5 días | Guardar/cargar bitmask de 32 bits |
| Menú oculto ESP32 (básico) | 3–5 días | UI screens, touch/button input, CAN commands |
| Menú oculto ESP32 (completo) | 5–8 días | Calibración interactiva, visualización de datos en tiempo real |

**Total estimado:** ~7–12 días de desarrollo para ambas funcionalidades.

---

## 4) ALTERNATIVAS TEMPORALES

Mientras no se implementan estas funcionalidades:

### Para persistencia
- **La calibración de dirección** se recalcula automáticamente en cada boot (centering con sensor inductivo, ≤10 s)
- **Service Mode** arranca con todos los módulos habilitados (defaults seguros)
- **No hay riesgo de seguridad** — los defaults hardcodeados son conservadores

### Para el menú oculto
- **Service Mode** ya es accesible por CAN (0x110) desde un terminal serie
- Se puede enviar el comando `SERVICE_CMD` con un CAN monitor (PCAN, USBtin, etc.)
- **Los datos de telemetría** ya se envían por CAN y se pueden ver con cualquier CAN sniffer

---

## REFERENCIAS

- `docs/PROJECT_MASTER_STATUS.md` §4 — Pending Features #4, #5
- `docs/PROJECT_MASTER_STATUS.md` §5 — Implementation Phases (Phase 4 y Phase 5)
- `docs/FIRMWARE_MATURITY_ROADMAP.md` §5 — Items 3.3 (config persistence) y 4.3 (engineering menu)
- `docs/FIRMWARE_MATURITY_ROADMAP.md` §7 — Risk Classification (ambos LOW risk)
- `docs/ORIGINAL_REPO_COMPARATIVE_AUDIT.md` — Referencia del menú oculto original (menu_hidden.cpp, 46 KB)
