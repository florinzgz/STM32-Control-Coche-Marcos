# AUDIO_ARCHITECTURE — Arquitectura de Audio MarcosDashboard

**Versión:** 1.0  
**Fecha:** 2026-06-08  
**Estado:** Documentación permanente — NO modifica código ni hardware  
**Basado en:** firmware verificado, documentación existente, fotos del hardware real  
**Documentos relacionados:**
- `docs/AUDIO_HARDWARE_AUDIT.md` — auditoría firmware DFPlayer
- `docs/AUDIO_RELAY_INTEGRATION.md` — máquina de estados del relé GPIO11
- `docs/AISLAMIENTO_AUDIO_DFPLAYER.md` — análisis de masa y ruido
- `docs/AUDIO_SCHEMATIC_ASCII.md` — esquemas eléctricos ASCII
- `docs/AUDIO_WIRING_GUIDE.md` — cableado pin a pin
- `docs/AUDIO_RISKS_AND_LIMITATIONS.md` — riesgos y restricciones

---

## Índice

1. [Inventario de hardware de audio](#1-inventario-de-hardware-de-audio)
2. [Arquitectura general del sistema de audio](#2-arquitectura-general-del-sistema-de-audio)
3. [Módulo original — identificación desde fotos](#3-módulo-original--identificación-desde-fotos)
4. [DFPlayer Mini — rol y firmware](#4-dfplayer-mini--rol-y-firmware)
5. [Relé de audio — soporte existente en firmware](#5-relé-de-audio--soporte-existente-en-firmware)
6. [PAM8403 — amplificador final](#6-pam8403--amplificador-final)
7. [Conmutación de fuentes — nivel de línea vs nivel de altavoz](#7-conmutación-de-fuentes--nivel-de-línea-vs-nivel-de-altavoz)
8. [Decisiones de arquitectura justificadas](#8-decisiones-de-arquitectura-justificadas)
9. [Arquitectura final recomendada](#9-arquitectura-final-recomendada)
10. [Restricciones absolutas](#10-restricciones-absolutas)

---

## 1. Inventario de hardware de audio

| Componente | Estado | Rol en sistema | Fuente de confirmación |
|---|---|---|---|
| **DFPlayer Mini** | ✅ CONFIRMADO en firmware | Reproductor de voces y avisos del vehículo | `esp32/src/audio_manager.h` líneas 24-25, 66-67 |
| **ESP32-S3** | ✅ CONFIRMADO en firmware | MCU de control del DFPlayer (UART2) | `esp32/src/audio_manager.cpp` línea 24 |
| **PAM8403** | ✅ DOCUMENTADO como amplificador objetivo | Amplificador final clase D BTL | `docs/AISLAMIENTO_AUDIO_DFPLAYER.md` |
| **Módulo original BT/USB/SD** | ✅ CONFIRMADO existencia física | Bluetooth + USB + microSD + FM | Fotos físicas analizadas |
| **Relé de audio** | ✅ CONFIRMADO en firmware | Conmutación fuente → altavoz | `esp32/src/relay_audio.h`, `esp32/src/relay_audio.cpp` |
| **Altavoz** | ✅ ASUMIDO único | Transductor final | Por requisito del proyecto |

---

## 2. Arquitectura general del sistema de audio

El sistema de audio tiene **dos fuentes** y **una salida**:

```
FUENTE A: Módulo original (BT/USB/SD/FM)
  └── jack 3.5 mm (salida de audio)          ← punto de extracción RECOMENDADO
                                              ← PENDIENTE VERIFICAR con multímetro

FUENTE B: DFPlayer Mini (voces del vehículo)
  └── DAC_L / DAC_R (salida analógica)        ← CONFIRMADO en documentación

             RELÉ DPDT (GPIO11 ESP32-S3)      ← CONFIRMADO en relay_audio.h
            ┌──────────────────────────┐
            │ NC → Fuente A (módulo)   │  Estado reposo (GPIO HIGH) = radio
            │ NO → Fuente B (DFPlayer) │  Estado activo (GPIO LOW)  = avisos
            │ COM → PAM8403            │
            └──────────────────────────┘

                      PAM8403                 ← Amplificador clase D BTL
                         │
                    Altavoz único
```

### Principio de operación

- En **reposo**, el altavoz está conectado al módulo original.
- Cuando el ESP32 dispara un aviso de voz, el relé conmuta hacia el DFPlayer Mini.
- El relé tiene anticlick integrado en firmware (20 ms de establecimiento, 150 ms de liberación).
- Al terminar el aviso, el relé vuelve automáticamente al módulo original.

### Dependencias de firmware (nada a modificar)

| Función | Archivo | Estado |
|---|---|---|
| Control DFPlayer | `esp32/src/audio_manager.cpp` | ✅ Existente |
| Control relé GPIO11 | `esp32/src/relay_audio.cpp` | ✅ Existente |
| Prioridades y cola | `esp32/src/audio_manager.h` | ✅ Existente |

**No se requiere ninguna modificación de firmware.**

---

## 3. Módulo original — identificación desde fotos

### 3.1 Lo que se puede confirmar visualmente

| Elemento | Visible en foto | Función deducida | Estado |
|---|---|---|---|
| Slot microSD | ✅ Foto 1 (izquierda) | Almacenamiento MP3 | ✅ CONFIRMADO visualmente |
| Jack 3.5 mm negro | ✅ Foto 1 | Salida de audio | ✅ CONFIRMADO visualmente |
| SoC JieLi "U3" | ✅ Foto 1 | Chip principal MP3/BT | ✅ CONFIRMADO (logo JL visible) |
| Cristal de cuarzo | ✅ Foto 1 | Oscilador de referencia | ✅ CONFIRMADO visualmente |
| IC U4 (SOIC-8) | ✅ Foto 1 | Probable amplificador de audio | ⚠️ HIPÓTESIS (función no verificada) |
| Teclas K2, K3, K4 | ✅ Foto 1 | Control local (play/prev/next/mode) | ✅ CONFIRMADO visualmente |
| ANT1 (antena PCB) | ✅ Foto 1 | Antena Bluetooth 2.4 GHz | ✅ CONFIRMADO visualmente |
| Puerto USB tipo A | ✅ Fotos 2/3/4 | Entrada de memoria USB | ✅ CONFIRMADO visualmente |
| Display 7 segmentos | ✅ Foto 4 | Información de pista/estado | ✅ CONFIRMADO visualmente |
| "USB", "SD", BT, "FM" en display | ✅ Foto 4 | Indica modos: USB/SD/BT/FM | ✅ CONFIRMADO visualmente |
| PCB secundaria (P5470R2S1) | ✅ Fotos 2/3 | Placa de alimentación y USB | ✅ CONFIRMADO (número de modelo visible) |
| Buck converter (inductor "i5i") | ✅ Fotos 2/3 | Regulación 12V → 5V o 3.3V | ✅ CONFIRMADO (inductor y regulador SOIC visibles) |

### 3.2 Mazo de cables (4 hilos: amarillo, negro, rojo/magenta, amarillo)

> **⚠️ PENDIENTE DE MEDIR — NO conectar nada sin verificación con multímetro.**

El mazo sale de la PCB secundaria (placa de alimentación), conectado al conector blanco de 4 pines.

| Cable | Color visible | Función MÁS PROBABLE | Estado |
|---|---|---|---|
| Cable 1 | **Amarillo** | +12V alimentación (convención automoción) | ⚠️ HIPÓTESIS |
| Cable 2 | **Negro** | GND | ⚠️ HIPÓTESIS |
| Cable 3 | **Rojo/Magenta** | Altavoz + (salida amplificada) | ⚠️ HIPÓTESIS |
| Cable 4 | **Amarillo** (segundo) | Altavoz − (salida amplificada) | ⚠️ HIPÓTESIS |

**Hipótesis alternativa:**
- Rojo/Magenta = +12V
- Amarillo (uno) = Altavoz +
- Amarillo (otro) = Altavoz −
- Negro = GND

> **REGLA: NO se asume ni se usa el mazo hasta medir con multímetro.** Ver `docs/AUDIO_WIRING_GUIDE.md` §7 (procedimiento de verificación).

### 3.3 Punto de extracción de audio recomendado

| Opción | Estado | Recomendado |
|---|---|---|
| **Jack 3.5 mm** | ✅ VISIBLE y seguro | **✅ PRIMERA OPCIÓN** |
| Mazo (cables amarillo/negro/rojo) | ⚠️ Sin verificar | ❌ NO usar sin medir |
| Pads internos del PCB | ❓ No accesibles sin desmontar | ❌ No usar sin conocer pinout |
| Salida de altavoz amplificada | ⚠️ Podría existir | ❌ NO usar para línea analógica |

---

## 4. DFPlayer Mini — rol y firmware

### 4.1 Confirmado en código

```cpp
// esp32/src/audio_manager.h — líneas 24-25 (CONFIRMADO)
inline constexpr int PIN_DFPLAYER_TX = 43;   // ESP32 TX → DFPlayer RX
inline constexpr int PIN_DFPLAYER_RX = 44;   // DFPlayer TX → ESP32 RX

// esp32/src/audio_manager.cpp — línea 24 (CONFIRMADO)
static HardwareSerial dfSerial(2);   // UART2

// esp32/src/audio_manager.cpp — línea 88 (CONFIRMADO)
dfSerial.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
```

### 4.2 Pines de audio del DFPlayer

| Pin DFPlayer | Función | Uso recomendado |
|---|---|---|
| DAC_R (pin 4) | Salida analógica canal derecho | ✅ Usar para PAM8403 |
| DAC_L (pin 5) | Salida analógica canal izquierdo | ✅ Usar para PAM8403 |
| SPK_1 (pin 6) | Salida amplificada + | ❌ NO usar con PAM8403 externo |
| SPK_2 (pin 8) | Salida amplificada − | ❌ NO usar con PAM8403 externo |
| GND (pin 7) | Masa | ✅ Conectar a GND_ESP32 |

### 4.3 Señal en DAC_L / DAC_R

| Parámetro | Valor |
|---|---|
| Nivel nominal | ~0.5–1.0 Vpp a volumen máximo |
| Tipo | Analógico single-ended (SE), referenciado a GND_DFPlayer |
| Contenido | Voces (300 Hz – 3 kHz típico) |
| Offset DC | Generalmente ≈ 0 V, pero verificar con módulo específico |
| Impedancia de salida | ~1 kΩ o menos |

---

## 5. Relé de audio — soporte existente en firmware

### 5.1 Hardware asignado

```cpp
// esp32/src/relay_audio.h — línea 47 (CONFIRMADO)
inline constexpr int PIN_AUDIO_RELAY = 11;   // GPIO11, active LOW

// Tiempos anti-click (CONFIRMADO en relay_audio.h)
inline constexpr unsigned long RELAY_ESTABLISH_MS = 20;   // establecimiento
inline constexpr unsigned long RELAY_RELEASE_MS   = 150;  // cooldown post-audio
inline constexpr unsigned long RELAY_MAX_ON_MS    = 7000; // watchdog seguridad
```

### 5.2 Máquina de estados

```
IDLE (GPIO HIGH, relé OFF)
  │ requestOn()
  ▼
ACTIVATING (GPIO LOW, esperando 20 ms)
  │ timer RELAY_ESTABLISH_MS
  ▼
ACTIVE (GPIO LOW, DFPlayer puede reproducir)
  │ release()
  ▼
RELEASING (GPIO LOW, cooldown 150 ms anti-click)
  │ timer RELAY_RELEASE_MS
  ▼
IDLE (GPIO HIGH, relé OFF)
```

### 5.3 Integración en el loop

La función `relay_audio::update()` se llama desde `audio::update()` en cada iteración del loop. Es completamente no bloqueante.

### 5.4 Estado en reposo

En reposo (sin voz activa), GPIO11 está en **HIGH** → relé **OFF**.  
El firmware define este estado como "fuente normal" (radio/módulo original).

---

## 6. PAM8403 — amplificador final

| Parámetro | Valor |
|---|---|
| Topología | Clase D BTL (Bridge-Tied Load) |
| Alimentación | 5V |
| Potencia por canal | 3W (a 4Ω) / 1.5W (a 8Ω) |
| Canales | 2 (estéreo) |
| Ganancia fija | ~24 dB (20 V/V) |
| THD+N | < 0.1% a 1W |
| Ventaja BTL | Ningún terminal del altavoz conecta a GND → elimina bucle de masa |

### Por qué BTL es importante aquí

En entorno automoción con motores de alto amperaje, la topología BTL del PAM8403 garantiza que ningún terminal del altavoz está conectado a GND_chasis, eliminando el mecanismo de "hum de masa" (ruido de 20 kHz por corrientes de retorno de motores).

Ver análisis completo en `docs/AISLAMIENTO_AUDIO_DFPLAYER.md`.

---

## 7. Conmutación de fuentes — nivel de línea vs nivel de altavoz

### Veredicto: conmutar SIEMPRE a nivel de línea (antes del PAM8403)

| Aspecto | Nivel de línea (antes PAM) | Nivel de altavoz (después PAM) |
|---|---|---|
| Corriente conmutada | < 1 mA | 100–500 mA |
| Pops al conmutar | Mínimos con anti-click firmware | Grandes (alta energía) |
| Vida del relé | Alta (carga resistiva pequeña) | Reducida (carga inductiva grande) |
| Riesgo de daño | Ninguno | Posible daño al altavoz |
| Complejidad | Simple | Requiere relay de potencia mayor |
| Compatible firmware existente | ✅ Sí | ⚠️ Requeriría cambios |

**Conclusión: conmutación a nivel de línea es la única opción correcta.**

---

## 8. Decisiones de arquitectura justificadas

### Por qué relé DPDT y no mezclador pasivo

Un mezclador pasivo permanente uniría las dos salidas de audio en paralelo permanente. Esto crearía:
1. Backfeed entre fuentes (la señal del DFPlayer entraría al módulo BT y viceversa).
2. Carga mutua entre salidas → distorsión y posible daño.
3. Reproducción simultánea (voces encima de música) no controlable.

El relé DPDT conmuta exclusivamente entre fuentes, sin conexión simultánea.

### Por qué no conmutador analógico (CD4066 / TS5A3159)

El conmutador analógico es viable técnicamente pero:
- Más sensible a ESD en entorno automoción.
- Introduce resistencia de on-state (~10–100 Ω).
- Requiere lógica de control compatible (3.3V correcto para ESP32-S3).
- Añade complejidad sin ventaja real para señales de voz.
- El relé ya está previsto y soportado en firmware (GPIO11, `relay_audio.h`).

Para este caso, el **relé de señal DPDT es la opción correcta y está ya implementada en firmware**.

### Por qué PAM8403 y no otro amplificador

| Amplificador | Veredico para este proyecto |
|---|---|
| **PAM8403** | ✅ Óptimo: 5V, BTL, simple, ya planeado en el proyecto |
| PAM8406 | Ventaja marginal, no justifica cambio |
| MAX98357A | ❌ No compatible: requiere I2S, cambiaría todo el firmware |
| TPA3110 | Overkill: 15W, 12–15V, tamaño grande innecesario |
| TPA3116 | Overkill: 100W, completamente desproporcionado |

---

## 9. Arquitectura final recomendada

```
┌─────────────────────────────────────────────────────────────────┐
│                   SISTEMA DE AUDIO — VISTA COMPLETA              │
└─────────────────────────────────────────────────────────────────┘

12V (batería / alimentación de electrónica)
  │
  ├──► Módulo original BT/USB/SD (P5470R2S1) ──► jack 3.5mm
  │      (alimentación directa 12V)                  │
  │                                               L+R → mono
  │                                              [1kΩ+1kΩ sumadores]
  │                                                   │
  │                                              [1µF desacoplo]
  │                                                   │
  │                                                 NC1
  │                                                   │
  ├──► Buck 12V→5V ──► DFPlayer Mini ──────────────► NO1   ← GPIO11 controla
  │        │           (UART2/GPIO43-44)         [Relé DPDT]   (relay_audio.h)
  │        │           DAC_L/DAC_R                  COM1
  │        │          [1kΩ+1kΩ sumadores]             │
  │        │          [1µF desacoplo]               PAM8403 IN
  │        │                                           │
  │        │                                        PAM8403
  │        └──► ESP32-S3 (HMI + control)            OUT+ ──► Altavoz +
  │                                                  OUT- ──► Altavoz -
  │
  └──► Buck 12V→5V→3.3V ──► ESP32-S3 lógica
```

### Lista de componentes necesarios para implementación

| Componente | Especificación | Cantidad | Disponible en inventario |
|---|---|---|---|
| Relé DPDT señal | 5V bobina, 2×conmutado | 1 | ⚠️ Verificar |
| Resistencias 1kΩ | 1/8W o 1/4W | 4 | ✅ Kit AUKENIEN |
| Condensadores 1µF | Cerámico o film 25V+ | 2 | ✅ Kit 105 cerámico |
| Condensador 100nF | Cerámico 50V | 1 (bypass PAM8403) | ✅ Kit 104 cerámico |
| Condensador 470µF | Electrolítico 10V+ | 1 (bulk PAM8403) | ✅ Kit ALLECIN |
| Resistencias 100kΩ | 1/8W | 1 (pull-down anti-pop) | ✅ Kit AUKENIEN |
| Jack 3.5mm macho (breakout) | Estéreo 3 pines | 1 | ❓ Verificar |

---

## 10. Restricciones absolutas

Las siguientes partes del sistema **nunca se tocan** en ninguna modificación de audio:

| Sistema | Archivos | Restricción |
|---|---|---|
| STM32 Safety System | `Core/Src/safety_system.c` | NO tocar |
| CAN Bus | `Core/Src/can_handler.c`, `esp32/src/can_rx.cpp` | NO tocar |
| PID de dirección | (steering_pid) | NO tocar |
| Control de motores | `Core/Src/motor_control.c` | NO tocar |
| Relés de potencia | RELAY_TRAC, RELAY_DIR | NO tocar |
| Firmware STM32 | Cualquier archivo en `Core/` | NO modificar para audio |
| Firmware ESP32 | `audio_manager.h/cpp`, `relay_audio.h/cpp` | Solo hardware — NO modificar |

---

*Documento creado: 2026-06-08*  
*Basado en: firmware verificado (esp32/src/audio_manager.h, relay_audio.h), fotos físicas del hardware real, docs/AUDIO_HARDWARE_AUDIT.md, docs/AUDIO_RELAY_INTEGRATION.md, docs/AISLAMIENTO_AUDIO_DFPLAYER.md*
