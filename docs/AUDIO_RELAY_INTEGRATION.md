# AUDIO RELAY INTEGRATION — Módulo relay_audio

> ⚠ **CAN rev 1.3 compatible (2026-04-23 clarification):** El relé de audio (módulo `relay_audio` en ESP32) es **independiente** del subsistema de relés de potencia del STM32 y **no se ve afectado** por la eliminación del relé MAIN. El bitmap de estado de relés en heartbeat 0x001 byte 5 conserva el layout rev 1.3 de 3 bits (bit 0 = reservado/0, bit 1 = TRAC, bit 2 = DIR) — este documento sigue siendo válido tal cual para `relay_audio`.

## Objetivo

Integrar un relé de audio de 2 canales (5 V optoacoplado) que conmuta la salida del altavoz entre:
- **Estado normal (relé OFF)** → Radio del coche
- **Estado activo (relé ON)** → Amplificador DFPlayer Mini

---

## 1. Selección de Pin — GPIO 11

### Análisis de pines disponibles en ESP32-S3-DevKitC-1 (N16R8)

| GPIO | Función actual | Disponible |
|------|----------------|-----------|
| 4    | CAN TX         | ❌ |
| 5    | CAN RX         | ❌ |
| 18   | TOFSense-M RX (UART1) | ❌ |
| 8    | I²C SDA (shifter) | ❌ |
| 9    | I²C SCL (shifter) | ❌ |
| 10   | SPI CS (display) | ❌ |
| 11   | **LIBRE**       | ✅ **Elegido** |
| 12   | SPI MISO       | ❌ |
| 13   | SPI MOSI       | ❌ |
| 14   | SPI SCK        | ❌ |
| 19   | USB D−          | ❌ |
| 20   | USB D+          | ❌ |
| 21   | Touch CS       | ❌ |
| 38   | Display RESET  | ❌ |
| 39   | Display DC/RS  | ❌ |
| 40   | Ignition sense | ❌ |
| 41   | Power hold     | ❌ |
| 43   | UART2 TX (DFPlayer) | ❌ |
| 44   | UART2 RX (DFPlayer) | ❌ |
| 42   | Display BL     | ❌ |
| 47   | WS2812B front LEDs | ❌ |
| 48   | WS2812B rear LEDs  | ❌ |

### Por qué GPIO 11 es seguro para ESP32-S3

| Criterio | Verificación |
|----------|-------------|
| ¿Es strapping pin? | **NO** — Solo GPIO 0, 3, 45 y 46 son strapping pins en ESP32-S3 |
| ¿Afecta arranque? | **NO** — GPIO 11 no tiene ninguna función especial durante el boot |
| ¿Conflicto Flash Octal (N16R8)? | **NO** — Flash usa GPIO 26–32 (interno al módulo WROOM-1) |
| ¿Conflicto PSRAM Octal (N16R8)? | **NO** — PSRAM usa GPIO 33–37 (interno al módulo WROOM-1) |
| ¿Conflicto USB? | **NO** — USB D−/D+ son GPIO 19/20 |
| ¿Conflicto JTAG? | **NO** — JTAG usa GPIO 39/40/41/42 |
| ¿Input-only? | **NO** — En ESP32-S3 no existen pines GPIO input-only |
| ¿Disponible en header DevKitC-1? | **SÍ** — Posición G11, lado derecho |
| ¿Pull-up interno disponible? | **SÍ** — Push-pull configurable |

### Estado de GPIO 11 durante el arranque

Durante el boot, antes de que `setup()` ejecute `audio::init()` → `relay_audio::init()`:
- GPIO 11 está en modo **INPUT** (alta impedancia) — valor por defecto de todos los GPIOs
- El módulo relé optoacoplado tiene resistencia limitadora de corriente interna: sin corriente activa, el relé permanece **OFF**
- Una vez que `relay_audio::init()` ejecuta `pinMode(11, OUTPUT)` + `digitalWrite(11, HIGH)`, el relé se mantiene definitivamente **OFF**

---

## 2. Diagrama de Conexión

```
ESP32-S3-DevKitC-1          Módulo Relé 5V Optoacoplado 2CH
┌─────────────────┐          ┌───────────────────────────────┐
│                 │          │                               │
│  GPIO 11 ───────┼──────────┼─► IN1  (activo LOW)           │
│                 │          │                               │
│  5V      ───────┼──────────┼─► VCC                         │
│  GND     ───────┼──────────┼─► GND                         │
│                 │          │                               │
└─────────────────┘          │  COM1 ────────► Altavoz +     │
                             │  NC1  ────────► Radio +       │
                             │  NO1  ────────► DFPlayer +    │
                             │                               │
                             └───────────────────────────────┘

Estado relé OFF (GPIO HIGH) → COM conectado a NC  → Radio del coche
Estado relé ON  (GPIO LOW)  → COM conectado a NO  → DFPlayer Mini

Nota: IN2 del módulo puede quedar sin conectar (sólo se usa 1 canal
      o se puede conectar a IN1 para conmutar ambos canales a la vez).
```

### Conexiones físicas detalladas

| Señal | Fuente | Destino |
|-------|--------|---------|
| Control IN1 | GPIO 11 (ESP32) | IN1 del módulo relé |
| Alimentación | 5 V (pin 5V del DevKit) | VCC del módulo relé |
| Tierra | GND (DevKit) | GND del módulo relé |
| Altavoz | COM del relé | + del altavoz |
| Radio | NC del relé | Salida de audio de la radio |
| DFPlayer | NO del relé | Salida SPKR del DFPlayer Mini |

---

## 3. Máquina de Estado del Relé

```
              requestOn()                  establishment timer (~20 ms)
   ┌─────────────────────────► ACTIVATING ──────────────────────────► ACTIVE
   │                                                                      │
 IDLE                                                                  release()
   ▲                                                                      │
   │        cooldown timer (~150 ms)                                      ▼
   └──────────────────────────────────────── RELEASING ◄──────────────────┘
                                                 │
                                           requestOn()  ──────────────► ACTIVE
                                           (cancela cooldown)
```

| Estado | Relé | Descripción |
|--------|------|-------------|
| `IDLE`       | OFF (GPIO HIGH) | Sin audio pendiente |
| `ACTIVATING` | ON  (GPIO LOW)  | Esperando establecimiento del contacto |
| `ACTIVE`     | ON  (GPIO LOW)  | Contacto establecido, DFPlayer puede reproducir |
| `RELEASING`  | ON  (GPIO LOW)  | Audio finalizado, cooldown anti-click en curso |

---

## 4. Secuencia de Reproducción (Diagrama de Tiempo)

```
audio::play() llamado
│
│  audio::update() → relay_audio::requestOn()
│  │
│  ├─ GPIO 11: HIGH ──►LOW  (relé se activa)
│  │
│  │  ← 20 ms (RELAY_ESTABLISH_MS) →
│  │
│  ├─ isReady() = true
│  │
│  ├─ DFPlayer comando play enviado por UART2
│  │
│  │  ← hasta 5000 ms (MAX_PLAY_DURATION_MS) →
│  │
│  ├─ relay_audio::release()
│  │
│  │  ← 150 ms (RELAY_RELEASE_MS) →
│  │
│  └─ GPIO 11: LOW ──► HIGH  (relé se desactiva)
```

---

## 5. Módulo de Código

### Archivos creados

| Archivo | Descripción |
|---------|-------------|
| `esp32/src/relay_audio.h` | Declaraciones, pin, constantes de tiempo, API |
| `esp32/src/relay_audio.cpp` | Implementación de la máquina de estados |

### Archivos modificados

| Archivo | Cambio |
|---------|--------|
| `esp32/src/audio_manager.cpp` | `#include "relay_audio.h"` + integración en `init()` y `update()` |

### Funciones exportadas

```cpp
namespace relay_audio {
    void init();        // Inicializa GPIO, relé OFF
    void requestOn();   // Activa relé (IDLE→ACTIVATING, RELEASING→ACTIVE)
    void release();     // Inicia cooldown post-audio (ACTIVE→RELEASING)
    void update();      // Tick de la máquina de estados (llamado desde audio::update)
    bool isReady();     // true solo en estado ACTIVE
}
```

---

## 6. Garantías de No-Interferencia

| Requisito | Cumplimiento |
|-----------|-------------|
| No bloquea loop principal | ✅ `relay_audio::update()` es 100% non-blocking (sin delay) |
| No bloquea tareas CAN | ✅ No modifica interrupciones ni prioridades de tareas |
| No altera watchdog timing | ✅ Sin busy-wait ni bucles bloqueantes |
| No introduce delays largos | ✅ Solo `millis()` comparisons, ejecución en microsegundos |
| No modifica prioridades de audio | ✅ El sistema de audio sigue siendo el maestro del relé |
| No afecta arranque | ✅ GPIO 11 no es strapping pin, relé OFF durante boot |
| No afecta USB | ✅ GPIO 11 no es USB (D−/D+ son GPIO 19/20) |
| No afecta CAN | ✅ GPIO 11 no interfiere con TWAI/CAN (GPIO 4/5) |
| Evita clicks en altavoz | ✅ `RELAY_ESTABLISH_MS=20` y `RELAY_RELEASE_MS=150` absorben transitorios |
| Evita activaciones múltiples | ✅ `requestOn()` es idempotente (sin efecto si ya está activo) |

---

## 7. Parámetros Configurables

En `relay_audio.h`:

```cpp
inline constexpr int           PIN_AUDIO_RELAY    = 11;   // GPIO de control
inline constexpr unsigned long RELAY_ESTABLISH_MS = 20;   // Tiempo de establecimiento (ms)
inline constexpr unsigned long RELAY_RELEASE_MS   = 150;  // Cooldown post-audio (ms)
```

Ajustar `RELAY_ESTABLISH_MS` según el tiempo de respuesta del relé físico (consultar datasheet).
Ajustar `RELAY_RELEASE_MS` si se perciben clicks residuales al desactivar.

---

_Creado: 2026-02-25_  
_Versión: 1.0_
