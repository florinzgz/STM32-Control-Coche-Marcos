# AUDITORÍA DEL HARDWARE DE AUDIO — Resultado de la Revisión Completa del Firmware

**Sistema de Control Vehicular — ESP32-S3 HMI**
**Fecha:** 2026-02-25
**Alcance:** Todos los archivos fuente del ESP32 (`esp32/src/`, `esp32/include/`, `esp32/platformio.ini`)
**Método:** Lectura directa del código fuente + búsqueda exhaustiva de símbolos

---

## Veredicto inmediato

> **El firmware usa DFPlayer Mini por UART. No hay nada de MAX98357 ni I2S en ningún archivo del repositorio.**

---

## 1. Hardware de audio que espera el firmware

**Hardware:** DFPlayer Mini (módulo reproductor MP3 con SD card y amplificador integrado)

**Evidencia directa en código:**

```cpp
// esp32/src/audio_manager.cpp  — líneas 22-23
// DFPlayer uses UART2
static HardwareSerial dfSerial(2);

// esp32/src/audio_manager.cpp  — línea 85
dfSerial.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);

// esp32/src/audio_manager.cpp  — línea 102
Serial.println("[AUDIO] DFPlayer initialized (UART2, 9600 baud, 68 tracks)");

// esp32/src/main.cpp  — línea 292-293
// Initialize DFPlayer audio (UART2 on GPIO 43/44)
audio::init();
```

**Resultado de búsqueda de otros sistemas de audio:**

| Símbolo buscado | Archivos encontrados | Conclusión |
|----------------|---------------------|------------|
| `MAX98357` | **0** | No existe en el proyecto |
| `I2S` / `i2s` | **0** | No existe en el proyecto |
| `BCLK` / `LRCLK` / `DIN` (I2S) | **0** | No existe en el proyecto |
| `ESP32Audio` / `ESP32-audioI2S` | **0** | No existe en el proyecto |
| `i2s_config_t` / `i2s_driver_install` | **0** | No existe en el proyecto |
| `DFPlayer` / `dfplayer` | **2** (audio_manager.h, audio_manager.cpp) | Únicos archivos relevantes |
| `HardwareSerial` | **1** (audio_manager.cpp) | Protocolo UART DFPlayer |

---

## 2. Pines exactos usados para audio

**Fuente:** `esp32/src/audio_manager.h` líneas 24-25

```cpp
inline constexpr int PIN_DFPLAYER_TX = 43;   // ESP32 TX → DFPlayer RX
inline constexpr int PIN_DFPLAYER_RX = 44;   // DFPlayer TX → ESP32 RX
```

| Señal | GPIO ESP32-S3 | Dirección | Función |
|-------|---------------|-----------|---------|
| **UART2 TX** | **GPIO 43** | ESP32 → DFPlayer | Envío de comandos (reproducir, volumen, reset) |
| **UART2 RX** | **GPIO 44** | DFPlayer → ESP32 | Respuesta del módulo (busy, errores) |
| **DAC_R** | — (pin DFPlayer) | DFPlayer → Amplificador | Señal de audio analógica (salida derecha) |
| **DAC_L** | — (pin DFPlayer) | DFPlayer → Amplificador | Señal de audio analógica (salida izquierda) |
| **SPK_1 / SPK_2** | — (pins DFPlayer) | DFPlayer → Altavoz | Amplificador integrado 3W (sin amp externo) |

**GPIO 43 y GPIO 44 son los únicos pines del ESP32-S3 involucrados en el audio.**

---

## 3. ¿Existe un driver I2S activo?

**No. No existe ningún driver I2S en el proyecto.**

Evidencia:

1. **`platformio.ini`** — solo 3 dependencias de librería, ninguna relacionada con audio I2S:
   ```ini
   lib_deps =
       handmade0octopus/ESP32-TWAI-CAN@^1.0.1
       bodmer/TFT_eSPI@^2.5.43
       fastled/FastLED@^3.9.12
   ```

2. **Framework: Arduino ONLY** (no ESP-IDF) — confirmado en `platformio.ini` línea 5:
   ```ini
   framework = arduino
   ```
   Las APIs de I2S de bajo nivel (`i2s_driver_install`, `i2s_write`, `i2s_config_t`) pertenecen a ESP-IDF y no están disponibles en modo Arduino puro. No se usa ningún wrapper de I2S para Arduino.

3. **Búsqueda exhaustiva en los 44 archivos fuente del ESP32:** 0 ocurrencias de `I2S`, `i2s`, `BCLK`, `LRCLK`, `DIN` como señal de audio.

**Conclusión:** El ESP32-S3 tiene hardware I2S físico disponible pero **completamente sin usar** en este firmware. No se ha instanciado, configurado ni inicializado.

---

## 4. Tareas de reproducción de audio implementadas

**No hay tareas FreeRTOS para audio.** La reproducción es una **máquina de estados no bloqueante** ejecutada en el bucle principal Arduino (`loop()`).

### Arquitectura exacta del sistema de audio

```
setup()                       loop()
  │                             │
  └── audio::init()             └── audio::update()  ← cada iteración del loop
        │                             │
        ├── dfSerial.begin()          ├── Si playback timeout expiró → playing = false
        │   (9600 baud, UART2)        │
        ├── sendCommand(RESET)        ├── Si pendingValid && intervalo 100ms OK
        │   delay(500)                │     → sendCommand(PLAY_TRACK, 0, trackNum)
        └── sendCommand(SET_VOLUME)   │     → playing = true
                                      │     → pendingValid = false
                                      └── (fin, retorna sin bloquear)
```

```
audio::play(Sound, Priority)          Llamado desde múltiples sitios en loop():
  │                                     - Cambio de marcha (gear events)
  ├── Verificar cooldown 4s             - Cambio de luces (LED relay)
  ├── Si prioridad >= actual            - Cambios de tracción (4x4/4x2)
  │     → pendingSound = sound          - Bienvenida / despedida
  │     → pendingValid = true           - Eventos CAN (seguridad, temperatura,
  └── Si prioridad < actual               batería, ABS/TCS, obstáculos)
        → descartar (preempted)
```

### Características de la implementación

| Característica | Valor | Fuente |
|---------------|-------|--------|
| Modelo de ejecución | Máquina de estados no bloqueante | audio_manager.cpp |
| Cola de reproducción | Single-slot (1 sonido pendiente) | pendingSound / pendingValid |
| Prioridades | 3 niveles: LO (0), MEDIUM (1), HI (2) | audio_manager.h enum Priority |
| Cooldown por sonido | 4 000 ms (excepto prioridad HI) | SOUND_COOLDOWN_MS |
| Duración máxima asumida | 5 000 ms (DFPlayer no notifica fin) | MAX_PLAY_DURATION_MS |
| Intervalo mínimo entre comandos | 100 ms (requisito del DFPlayer) | CMD_INTERVAL_MS |
| Volumen por defecto | 15 / 30 | config_store.cpp KEY_VOLUME default |
| Persistencia del volumen | NVS ESP32 (Preferences, clave "vol") | config_store.cpp línea 79 |
| Burst arbitration | Sí — reproduce el sonido de mayor severidad en ventana de burst | main.cpp burstSeverity() |
| Tracks disponibles | 68 (0001.mp3–0068.mp3) | audio_manager.h enum Sound |

### Eventos que disparan audio (desde main.cpp)

| Evento | Track | Prioridad |
|--------|-------|-----------|
| Arranque / bienvenida | WELCOME (1) | HI |
| Apagado / despedida | FAREWELL (2) | HI |
| Cambio de marcha (P/R/N/D1/D2) | GEAR_* (20-24) | LO |
| Luces delanteras ON/OFF | LIGHTS_ON/OFF (16-17) | LO |
| Luces traseras ON/OFF | LIGHTS_ON/OFF (16-17) | LO |
| Tracción 4x4 | TRACTION_4X4 (37) | LO |
| Tracción 4x2 | TRACTION_4X2 (38) | LO |
| Tank turn | BEEP (68) | LO |
| Emergency / SAFE state | EMERGENCY (31) | HI |
| Overcurrent | OVERCURRENT (53) | HI |
| Recovery (DEGRADED→ACTIVE) | SAFETY_RESET (32) | MEDIUM |
| Temperatura alta | TEMP_HIGH (10) | MEDIUM |
| Temperatura normalizada | TEMP_NORMAL (11) | LO |
| ABS ON/OFF | ABS_ON/OFF (39/40) | MEDIUM |
| TCS ON/OFF | TCS_ON/OFF (41/42) | MEDIUM |
| Obstáculo detectado | OBSTACLE_WARN (54) | MEDIUM |
| Batería baja | BATTERY_LOW (12) | MEDIUM |
| Batería crítica | BATTERY_CRITICAL (13) | HI |
| Luces con CAN (0x20A) | LIGHTS_ON/OFF | LO |

---

## 5. Dependencias de tarjeta SD y DFPlayer

**Sí. El sistema de audio tiene dependencia directa de la tarjeta SD del DFPlayer Mini.**

### Dependencia de tarjeta microSD

| Requisito | Especificación |
|-----------|---------------|
| **Tipo** | microSD (slot en el DFPlayer Mini) |
| **Formato** | FAT32 |
| **Capacidad** | 1 GB – 32 GB |
| **Velocidad** | Clase 4 o superior |
| **Ubicación archivos** | **Raíz de la SD** (NO en carpetas) |
| **Archivos requeridos** | `0001.mp3` a `0068.mp3` (68 archivos) |
| **Formato audio** | MP3, mono preferido, 8–128 kbps, 22050 Hz |

### ¿Qué pasa si la SD está ausente o corrupta?

- `audio::init()` envía el comando de RESET al DFPlayer (sendCommand 0x0C) — el DFPlayer no responde correctamente si no hay SD
- `initialized = true` se setea en cualquier caso (no hay detección de error de init)
- Cada llamada a `audio::play()` pone `pendingValid = true`
- `audio::update()` envía el comando PLAY_TRACK — el DFPlayer lo ignorará silenciosamente
- **El sistema de HMI sigue funcionando normalmente** (display, CAN, shifter, etc.)
- **El firmware no detecta la ausencia de SD** — el audio simplemente no se reproduce, sin error registrado

### Dependencia del módulo DFPlayer Mini

El firmware **no usa ninguna librería de terceros** para el DFPlayer. Implementa el protocolo manualmente:

```cpp
// Protocolo DFPlayer — 10 bytes por comando
buf[0] = 0x7E;   // Start
buf[1] = 0xFF;   // Version
buf[2] = 0x06;   // Length
buf[3] = cmd;    // Command (0x03 play, 0x06 volume, 0x0C reset)
buf[4] = 0x00;   // Feedback disabled
buf[5] = param1; // Parameter high byte
buf[6] = param2; // Parameter low byte
buf[7..8]        // Checksum = -(sum bytes 1..6), big-endian
buf[9] = 0xEF;   // End
```

Comandos utilizados: `0x03` (PLAY_TRACK), `0x06` (SET_VOLUME), `0x0C` (RESET). Solo 3 de los ~20 comandos del protocolo DFPlayer.

---

## 6. Esquema de conexión eléctrica — DFPlayer Mini

### 6.1 Diagrama de conexión completo

```
╔══════════════════════════════════════════════════════════════════════════════╗
║               CONEXIÓN DFPLAYER MINI ↔ ESP32-S3                              ║
╠══════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║                    ESP32-S3 DevKitC-1                                        ║
║                    ─────────────────                                         ║
║                                                                              ║
║   3.3V ──────────────────────────────────────► VCC  (pin 1) ┐               ║
║   GND  ──────────────────────────────────────► GND  (pin 7) │ DFPlayer Mini  ║
║   GPIO43 (UART2 TX) ──[1 kΩ]──────────────► RX   (pin 2) │               ║
║   GPIO44 (UART2 RX) ────────────────────────► TX   (pin 3) │               ║
║                                              │              │               ║
║                                              │ microSD      │               ║
║                                              │ [FAT32]      │               ║
║                                              │ 0001.mp3     │               ║
║                                              │  ...         │               ║
║                                              │ 0068.mp3     │               ║
║                                              │              │               ║
║   OPCIÓN A — Altavoz directo (≤3W, 4-8Ω):   │              │               ║
║                                              │ SPK_1 (pin 6)├─► Altavoz (+) ║
║                                              │ SPK_2 (pin 8)├─► Altavoz (−) ║
║                                              │              │               ║
║   OPCIÓN B — Amplificador externo:           │              │               ║
║                                              │ DAC_R (pin 4)├─► Amp IN+     ║
║                                              │ GND  (pin 7) ├─► Amp GND     ║
║                                              │              │               ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

### 6.2 Pinout del DFPlayer Mini (módulo DFR0299 / variantes equivalentes)

```
DFPlayer Mini (vista frontal — conector inferior izquierda)

         ┌────────────────────┐
    VCC ─┤ 1              16 ├─ BUSY (LOW cuando reproduciendo)
     RX ─┤ 2              15 ├─ USB_DP (no usar)
     TX ─┤ 3              14 ├─ USB_DM (no usar)
  DAC_R ─┤ 4              13 ├─ ADKEY2 (no usar)
  DAC_L ─┤ 5              12 ├─ ADKEY1 (no usar)
  SPK_1 ─┤ 6              11 ├─ IO_2 (no usar)
    GND ─┤ 7              10 ├─ GND
  SPK_2 ─┤ 8               9 ├─ IO_1 (no usar)
         └────────────────────┘
              microSD slot
```

### 6.3 Tabla de conexiones cable por cable

| # | De (ESP32-S3) | A (DFPlayer Mini) | Cable | Notas |
|---|---------------|-------------------|-------|-------|
| 1 | **3.3V** | **VCC** (pin 1) | Rojo, ≥22 AWG | Alimentación 3.3V–5V (el módulo acepta ambas) |
| 2 | **GND** | **GND** (pin 7) | Negro, ≥22 AWG | GND_ESP32 exclusivamente |
| 3 | **GPIO43** (UART2 TX) | **RX** (pin 2) | Verde | **Añadir resistor serie 1kΩ** (ver nota 1) |
| 4 | **GPIO44** (UART2 RX) | **TX** (pin 3) | Amarillo | Conexión directa sin resistor |
| 5 | — | **microSD** | — | FAT32, 68 MP3s en la raíz |

**Opción A — Altavoz directo (amplificador interno DFPlayer):**

| # | De (DFPlayer Mini) | A (Altavoz) | Notas |
|---|-------------------|-------------|-------|
| 6 | **SPK_1** (pin 6) | Altavoz (+) | Altavoz 4–8 Ω, ≤3W |
| 7 | **SPK_2** (pin 8) | Altavoz (−) | NO conectar a GND |

**Opción B — Amplificador externo (BTL recomendado):**

| # | De (DFPlayer Mini) | A (Amplificador) | Notas |
|---|-------------------|-----------------|-------|
| 6 | **DAC_R** (pin 4) | Amplificador IN+ | Señal analógica ~0.5–2 Vpp |
| 7 | **GND** (pin 7) | Amplificador GND | Mismo GND_ESP32 |

### 6.4 Notas de instalación

**Nota 1 — Resistor serie 1kΩ en RX:**

El pin RX del DFPlayer Mini es tolerante a 3.3V, pero algunos módulos de bajo coste tienen protecciones deficientes. El resistor de 1kΩ en serie entre GPIO43 y el pin RX del DFPlayer protege ambos componentes de cortocircuitos accidentales y de diferencias de nivel entre lotes de fabricación distintos. No afecta a la comunicación a 9600 baud.

```
GPIO43 (ESP32) ──[1kΩ]──► RX (DFPlayer)

Tiempo de subida con 1kΩ y C_parásita ≈ 50pF:
  τ = 1kΩ × 50pF = 50 ns
  t_a_VIH = 60 ns << 104 µs (1 bit a 9600 baud) → sin impacto ✅
```

**Nota 2 — Alimentación del DFPlayer Mini:**

| Tensión VCC | Comportamiento |
|-------------|---------------|
| 3.3V | Funciona en la mayoría de módulos. Verificar con el módulo específico. |
| 5V | Siempre compatible. Requiere que GPIO43 use nivel de 3.3V con resistor serie o level-shifter. |

Si se alimenta a 5V, el resistor de 1kΩ en el pin RX es suficiente para proteger el GPIO43 (3.3V) siempre que la lógica interna del DFPlayer acepte 3.3V como HIGH (la mayoría lo hace). Verificar en el datasheet del lote específico.

**Nota 3 — Pin BUSY (pin 16):**

El firmware actual **no lee el pin BUSY** del DFPlayer. El estado "reproduciendo" se estima por timeout (`MAX_PLAY_DURATION_MS = 5000 ms`). Si se conecta BUSY a un GPIO libre del ESP32, podría usarse para mejorar la detección de fin de reproducción, pero requeriría cambios en `audio_manager.cpp`.

**Nota 4 — Altavoz (Opción A vs B):**

- **Opción A (SPK_1/SPK_2):** El DFPlayer tiene un amplificador integrado de 3W. Suficiente para mensajes de voz en un habitáculo pequeño. Los pines SPK son diferenciales (BTL interno) — ninguno va a GND. ✅ Sin riesgo de ground loop.

- **Opción B (DAC_R):** Para mayor potencia. Usar amplificador BTL externo. Ver `AISLAMIENTO_AUDIO_DFPLAYER.md` para análisis de ground loop y recomendaciones.

---

## 7. Resumen ejecutivo

| Pregunta | Respuesta |
|----------|-----------|
| **¿MAX98357 o DFPlayer?** | **DFPlayer Mini** — confirmado en todos los archivos fuente |
| **¿Driver I2S activo?** | **No** — cero referencias a I2S en los 44 archivos del ESP32 |
| **¿Librería de audio?** | **No** — el protocolo DFPlayer está implementado manualmente en `audio_manager.cpp` |
| **Pines de audio** | **GPIO43 (TX→DFPlayer RX), GPIO44 (RX←DFPlayer TX)** — UART2, 9600 baud |
| **¿Tareas FreeRTOS?** | **No** — máquina de estados no bloqueante en `loop()`, `audio::update()` cada ciclo |
| **¿Dependencia SD card?** | **Sí** — 68 archivos MP3 en microSD FAT32 en el DFPlayer Mini |
| **¿Qué pasa sin SD?** | Audio silencioso; el resto del HMI funciona normalmente |
| **Protocolo** | Serial UART 9600 baud, tramas de 10 bytes, sin ACK de recepción |
| **Amplificador** | Integrado en DFPlayer Mini (3W, BTL) o externo vía DAC_R |
| **Volumen** | 0–30, default 15, persistido en NVS ESP32 |
| **Cambio de firmware necesario** | **Ninguno** — el firmware ya es correcto para DFPlayer Mini |

---

> **Nota sobre el documento anterior `AISLAMIENTO_AUDIO_DFPLAYER.md`:**
> Todo su contenido es correcto y complementario a este documento.
> La cadena de audio descrita allí (DFPlayer Mini → amplificador → altavoz) es
> exactamente la que implementa el firmware. Las recomendaciones de aislamiento
> de audio (BTL amplifier o transformador 1:1) siguen siendo válidas.

---

**Archivos auditados:**
- `esp32/src/audio_manager.h` — constantes de pines, enum Sound, API pública
- `esp32/src/audio_manager.cpp` — implementación completa: init, update, play, setVolume, protocolo serial
- `esp32/src/main.cpp` — puntos de llamada: setup() init, loop() update, todos los play()
- `esp32/src/config_store.h` / `.cpp` — persistencia de volumen en NVS
- `esp32/src/power_manager.h` / `.cpp` — timing de audio en ciclos de encendido/apagado
- `esp32/platformio.ini` — dependencias, framework, sin librerías de audio
- `esp32/include/` — todos los headers, sin I2S ni MAX98357
