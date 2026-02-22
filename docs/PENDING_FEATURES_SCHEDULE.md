# Funcionalidades Pendientes: Audio, Menú Oculto y Persistencia

**Fecha:** 2026-02-22
**Referencia:** `docs/PROJECT_MASTER_STATUS.md` §4–§5, `docs/FIRMWARE_MATURITY_ROADMAP.md` §5, `FULL-FIRMWARE-Coche-Marcos` (repo original)

---

## RESUMEN

| Funcionalidad | Estado | Phase | Notas |
|--------------|--------|-------|-------|
| **Persistencia de configuración (ESP32-S3 NVS)** | ✅ IMPLEMENTADA | — | `config_store.cpp` con CRC32, dirty flag, flush periódico |
| ~~**Persistencia de calibración (STM32 Flash)**~~ | ✅ NO NECESARIA | — | STM32 recalibra desde sensores hardware en cada arranque (diseño de seguridad) |
| ~~**Persistencia de Service Mode (STM32 Flash)**~~ | ✅ NO NECESARIA | — | Service mode arranca con todos los módulos habilitados (default seguro) |
| **Menú oculto de ingeniería (ESP32)** | ✅ IMPLEMENTADO | — | `engineering_screen.cpp`, código secreto 8989, 5 submenús, botón EXIT |
| **Audio (DFPlayer Mini)** | ✅ IMPLEMENTADO | — | `audio_manager.cpp`, cola con prioridades, 6 sonidos, GPIO 43/44 |

### Decisión de arquitectura: Persistencia centralizada en ESP32-S3

Toda la persistencia de configuración de usuario se maneja en la **ESP32-S3 mediante NVS** (`config_store.cpp`):

- Modo de conducción (4×4/4×2/tank)
- Brillo de pantalla
- Estado del relé LED
- Volumen de audio
- Validación CRC32 para integridad de datos
- Escrituras diferidas con dirty flag + flush cada 10 s

El STM32 **NO tiene almacenamiento no volátil por diseño**. Esto es una característica de seguridad:
- La calibración de dirección se recalcula desde el sensor inductivo en cada arranque
- El encoder zero se obtiene de TIM2 hardware quadrature
- Las temperaturas base se obtienen de DS18B20 con ROM search + CRC-8
- Los módulos de service mode arrancan todos habilitados (default seguro)

Esto garantiza que la calibración siempre refleja el estado real del hardware, evitando operar con datos de calibración obsoletos o corruptos.

---

## 1) PERSISTENCIA EN EEPROM / FLASH STM32

### Estado actual

## 1) PERSISTENCIA — DECISIÓN FINAL

### ✅ RESUELTO: ESP32-S3 NVS es la autoridad de persistencia

La persistencia de toda la configuración de usuario se maneja en la ESP32-S3 mediante NVS (`config_store.cpp`). El STM32 recalibra desde sensores hardware en cada arranque. No se necesita Flash storage en STM32.

**Justificación:**
- La calibración de dirección se recalcula automáticamente desde el sensor inductivo (PB5) en ≤10 s
- Los defaults de Service Mode (todos habilitados) son los defaults seguros
- Los offsets de sensores se obtienen de los propios sensores (INA226, DS18B20, encoder)
- Almacenar calibración stale en Flash sería un riesgo de seguridad si el hardware cambia

**Lo que SÍ se persiste (en ESP32-S3 NVS):**

| Dato | Tamaño | Módulo |
|------|--------|--------|
| Drive mode (4×4/4×2/tank) | 1 byte | `config_store::setDriveMode()` |
| Brightness | 1 byte | `config_store::setBrightness()` |
| LED relay state | 1 byte | `config_store::setLedEnabled()` |
| Audio volume | 1 byte | `config_store::setAudioVolume()` |
| CRC-32 integridad | 4 bytes | `computeCrc32()` |

**Protecciones implementadas:**
- CRC-32 en cada escritura/lectura para detectar corrupción
- Dirty flag + flush periódico cada 10 s para mitigar desgaste NVS (~100K ciclos)
- Flush en shutdown para no perder cambios pendientes
- Factory reset disponible desde menú de ingeniería

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

✅ **IMPLEMENTADO** en la ESP32-S3 (`esp32/src/screens/engineering_screen.cpp`).

Funcionalidades implementadas:
- Código secreto de acceso: **8989** (secuencia de taps en zona específica)
- 5 submenús: fault viewer, module control, factory restore, telemetry, system info
- Botón EXIT para volver a la pantalla normal
- Habilitación/deshabilitación de módulos via CAN 0x110
- Factory restore (CAN 0x110 action=0xFF + NVS factory reset)

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

✅ **IMPLEMENTADO** — `esp32/src/audio_manager.cpp` con driver DFPlayer Mini.

Funcionalidades implementadas:
- Driver DFPlayer Mini (UART2, GPIO 43/44, 9600 baud)
- Cola de reproducción con prioridades (LOW/MEDIUM/HIGH)
- 6 sonidos: WELCOME, FAREWELL, OBSTACLE_WARN, ERROR_ALERT, BATTERY_LOW, GEAR_CHANGE
- Audio de bienvenida/despedida en transiciones de encendido/apagado
- Volumen persistido en NVS (`config_store::setAudioVolume()`)

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
