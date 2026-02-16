# Funcionalidades Pendientes: Audio, Menú Oculto y Persistencia EEPROM

**Fecha:** 2026-02-16
**Referencia:** `docs/PROJECT_MASTER_STATUS.md` §4–§5, `docs/FIRMWARE_MATURITY_ROADMAP.md` §5, `FULL-FIRMWARE-Coche-Marcos` (repo original)

---

## RESUMEN

| Funcionalidad | ¿Cuándo se implementa? | Phase | Depende de |
|--------------|------------------------|-------|------------|
| **Persistencia de calibración (EEPROM/Flash)** | **Phase 4 — Driver Interaction** | Después de Phases 1, 2 y 3 | STM32 Flash driver |
| **Persistencia de Service Mode (EEPROM/Flash)** | **Phase 4 — Driver Interaction** | Después de Phases 1, 2 y 3 | STM32 Flash driver |
| **Menú oculto de ingeniería (ESP32)** | **Phase 5 — Experience Features** | Después de Phases 1, 2, 3 y 4 | Persistencia implementada, UI funcional |
| **Audio (DFPlayer Mini)** | **Phase 5 — Experience Features** | Después de Phases 1, 2, 3 y 4 | Hardware DFPlayer conectado, ESP32 UART libre |

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

## 3) AUDIO — DFPlayer Mini (Phase 5)

### Estado actual

**NO IMPLEMENTADO** — No hay código de audio ni hardware de audio en el repositorio actual.

`docs/PROJECT_MASTER_STATUS.md` §Audio dice textualmente:
> NOT IMPLEMENTED — no audio hardware or software exists in the codebase.

### Qué había en el repo original

El repo `FULL-FIRMWARE-Coche-Marcos` tenía un sistema de audio completo:

| Componente | Archivo original | Descripción |
|-----------|-----------------|-------------|
| **Driver DFPlayer** | `src/audio/dfplayer.cpp` | Comunicación UART 9600 con DFPlayer Mini (lazy init, play, update) |
| **Sistema de alertas** | `src/audio/alerts.cpp` | Cola de reproducción con prioridades (PRIO_NORMAL) |
| **Cola de audio** | `src/audio/queue.cpp` | Cola circular de 8 items, dispatch a DFPlayer |
| **Track guide** | `docs/AUDIO_TRACKS_GUIDE.md` | 68 tracks MP3 documentados con texto TTS sugerido |
| **Headers** | `include/dfplayer.h`, `include/alerts.h`, `include/queue.h` | Declaraciones públicas |
| **Tests** | `src/test/audio_validation_tests.cpp` | Validación de tracks y cola |

### Hardware necesario

| Componente | Especificación | Conexión |
|-----------|---------------|----------|
| **DFPlayer Mini** | Módulo MP3 con DAC + amplificador, UART 9600 baud | ESP32 GPIO TX → DFPlayer RX |
| **Tarjeta SD** | FAT32, 1–32 GB, Clase 4+ | Insertada en el DFPlayer |
| **Altavoz** | 3W 8Ω (o auriculares) | Conectado a DAC_R / DAC_L del DFPlayer |
| **68 archivos MP3** | `0001.mp3` a `0068.mp3` | En la raíz de la tarjeta SD |

### Conexión física en la arquitectura actual

En la arquitectura dual-MCU, el DFPlayer se conecta al **ESP32-S3** (no al STM32):

```
ESP32-S3                   DFPlayer Mini
┌──────────┐              ┌──────────────┐
│ GPIO TX ─┼──────────────┼─ RX          │
│ GPIO RX ─┼──────────────┼─ TX          │
│ 5V ──────┼──────────────┼─ VCC         │     ┌──────────┐
│ GND ─────┼──────────────┼─ GND         │     │ Altavoz  │
│          │              │ DAC_R ───────┼─────│ 3W 8Ω    │
│          │              │ DAC_L ───────┼─────│          │
│          │              │         [SD] │     └──────────┘
└──────────┘              └──────────────┘
```

**Pines UART para DFPlayer en el ESP32 actual:**

El ESP32-S3 actual usa los siguientes GPIOs (ver `esp32/platformio.ini` y `esp32/src/main.cpp`):
- GPIO 4: CAN_TX (TWAI)
- GPIO 5: CAN_RX (TWAI)
- GPIO 13/14/15/16/17: TFT SPI (MOSI/SCLK/CS/DC/RST)
- GPIO 21: TOUCH_CS
- GPIO 42: TFT Backlight

**✅ GPIO 19 (TX) y GPIO 20 (RX) están LIBRES** — son los mismos pines que usaba el repo original para el DFPlayer. Se pueden asignar directamente en Phase 5 usando UART1 del ESP32-S3.

### Los 68 tracks de audio

El sistema original define 68 tracks organizados por categoría:

| Categoría | Tracks | Ejemplos |
|-----------|--------|----------|
| Sistema (inicio/apagado/error) | 0001–0003 | "Bienvenido Marcos. El sistema está listo." |
| Calibración (pedal/INA/encoder) | 0004–0009 | "Calibración del pedal completada." |
| Temperatura / Batería | 0010–0013 | "Temperatura del motor elevada." |
| Freno / Luces / Multimedia | 0014–0019 | "Freno de estacionamiento activado." |
| Marchas (P/R/N/D1/D2) | 0020–0024 | "Marcha D uno activada." |
| Menú oculto | 0025–0028 | "Menú de calibración avanzado activado." |
| Test / Emergencia | 0029–0032 | "Modo de emergencia activado." |
| Errores de sensores | 0033–0036 | "Error en sensor de temperatura." |
| Tracción 4x4/4x2 | 0037–0038 | "Tracción 4x4 inteligente activada." |
| Seguridad (ABS/TCS/Regen) | 0039–0044 | "Sistema antibloqueo de frenos activado." |
| WiFi / OTA / Bluetooth | 0045–0051 | "Conexión WiFi establecida." |
| Estados del vehículo | 0052–0056 | "Velocidad máxima alcanzada." |
| Telemetría | 0057–0060 | "Nivel de batería al 50 por ciento." |
| Modos de conducción | 0061–0063 | "Modo eco activado." |
| Configuración | 0064–0068 | "Configuración guardada correctamente." |

**Cómo generar los MP3:** Usar [TTSMaker.com](https://ttsmaker.com/) (voz española) o el script Python con `gTTS` documentado en el repo original (`docs/AUDIO_TRACKS_GUIDE.md`).

### Qué hay que implementar (Phase 5)

| Tarea | Dónde | Esfuerzo | Notas |
|-------|-------|----------|-------|
| Portar `dfplayer.cpp` | `esp32/src/audio/dfplayer.cpp` | 1 día | Adaptar a pines del ESP32 actual, quitar refs a Logger/System |
| Portar `alerts.cpp` | `esp32/src/audio/alerts.cpp` | 0.5 días | Adaptar a la arquitectura CAN-based (triggers vienen de STM32) |
| Portar `queue.cpp` | `esp32/src/audio/queue.cpp` | 0.5 días | Cola circular, casi directo |
| Integrar triggers con CAN | `esp32/src/can/can_rx.cpp` | 1 día | Cuando llegan ciertos mensajes CAN → play audio track |
| Crear MP3 para SD | Tarjeta SD | 1 día | 68 tracks con TTS (ver AUDIO_TRACKS_GUIDE.md original) |
| Agregar lib `DFRobotDFPlayerMini` al platformio.ini | `esp32/platformio.ini` | 10 min | `lib_deps += DFRobotDFPlayerMini` |
| Service Mode: módulo audio | `Core/Src/service_mode.c` (STM32) | 0.5 días | Enable/disable audio module vía CAN 0x110 |
| CAN message para audio (opcional) | Ambos | 0.5 días | Nuevo CAN ID 0x20B para peticiones de audio desde STM32 |

**Esfuerzo total estimado:** ~4–5 días de desarrollo.

### Tracks relevantes para el sistema actual

De los 68 tracks, los más relevantes para el sistema dual-MCU actual son:

| Track | Evento en firmware actual | Trigger CAN existente |
|-------|--------------------------|----------------------|
| 0001 (Inicio) | Boot → ACTIVE | Heartbeat 0x001 + state=ACTIVE |
| 0003 (Error general) | Safety → ERROR | CAN 0x203 `SAFETY_STATUS` con error ≠ 0 |
| 0010 (Temp alta) | `Safety_CheckTemperature()` → WARNING | CAN 0x202 temp > 80°C |
| 0012 (Batería baja) | `BatteryVoltage_GetPct()` < 20% | CAN 0x206 battery_pct < 20 |
| 0020–0024 (Marchas) | `Traction_SetGear()` | CAN 0x20A (futuro, Phase 4) |
| 0031 (Emergencia) | `Safety_EmergencyStop()` | CAN 0x203 state=ERROR |
| 0037–0038 (4x4/4x2) | `Traction_SetMode4x4()` | CAN 0x20A (futuro, Phase 4) |

### Por qué NO se hace ahora

1. **Phase ordering** — Es Phase 5, requiere Phases 1–4 completadas primero
2. **Hardware no conectado** — El DFPlayer Mini no está cableado en la placa actual
3. **No es safety-critical** — El audio es una mejora de UX, no afecta al control del vehículo
4. **Prioridad de integración:** Primero necesitamos el CAN feedback de gear/mode (Phase 4) para que los triggers de audio de marchas funcionen

---

## 4) ORDEN DE IMPLEMENTACIÓN

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
                        ★ AUDIO DFPlayer Mini (68 tracks) ★
                        + LEDs WS2812B (si hay hardware)
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
| Audio DFPlayer (driver + alertas + cola) | 3–4 días | Portar 3 módulos, integrar con CAN triggers |
| Audio: crear 68 MP3 (TTS) + SD card | 1 día | Script Python con gTTS o TTSMaker.com |

**Total estimado:** ~12–17 días de desarrollo para todas las funcionalidades de Phase 4+5.

---

## 5) ALTERNATIVAS TEMPORALES

Mientras no se implementan estas funcionalidades:

### Para persistencia
- **La calibración de dirección** se recalcula automáticamente en cada boot (centering con sensor inductivo, ≤10 s)
- **Service Mode** arranca con todos los módulos habilitados (defaults seguros)
- **No hay riesgo de seguridad** — los defaults hardcodeados son conservadores

### Para el menú oculto
- **Service Mode** ya es accesible por CAN (0x110) desde un terminal serie
- Se puede enviar el comando `SERVICE_CMD` con un CAN monitor (PCAN, USBtin, etc.)
- **Los datos de telemetría** ya se envían por CAN y se pueden ver con cualquier CAN sniffer

### Para el audio
- **No hay alternativa funcional** — Sin el DFPlayer Mini conectado, no hay salida de audio
- **Los estados que generarían audio** ya se envían por CAN y se muestran en la pantalla:
  - Cambios de marcha → Visible en DriveScreen (hardcoded en Phase 4)
  - Alertas de temperatura → CAN 0x202, visible en telemetría de ruedas
  - Emergencia → Pantalla error_screen.cpp con estado
- **Prioridad:** El feedback visual (pantalla) es suficiente para Phase 1–4; el audio es complementario

---

## REFERENCIAS

- `docs/PROJECT_MASTER_STATUS.md` §4 — Pending Features #4, #5
- `docs/PROJECT_MASTER_STATUS.md` §5 — Implementation Phases (Phase 4 y Phase 5)
- `docs/FIRMWARE_MATURITY_ROADMAP.md` §5 — Items 3.3 (config persistence) y 4.3 (engineering menu)
- `docs/FIRMWARE_MATURITY_ROADMAP.md` §7 — Risk Classification (ambos LOW risk)
- `docs/ORIGINAL_REPO_COMPARATIVE_AUDIT.md` — Referencia del menú oculto original (menu_hidden.cpp, 46 KB)
- **Repo original:** `github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos` — Audio source: `src/audio/dfplayer.cpp`, `alerts.cpp`, `queue.cpp`
- **Repo original:** `docs/AUDIO_TRACKS_GUIDE.md` — 68 tracks MP3 con textos TTS, script Python generador
