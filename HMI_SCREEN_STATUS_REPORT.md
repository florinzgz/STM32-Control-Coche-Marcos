# Informe de Estado — Pantalla HMI y Menú Oculto ESP32

**Fecha:** 2026-02-23  
**Firmware:** ESP32-S3 HMI (`esp32/src/`)  
**Tipo:** Solo informe de estado — sin modificación de código  

---

## Resumen Ejecutivo

El sistema HMI del ESP32-S3 dispone de **6 pantallas operativas** gobernadas por una máquina de
estados que refleja el estado del STM32 vía CAN. Todas las pantallas están implementadas y
renderizan correctamente. El **menú oculto de ingeniería** es accesible pero tiene **dos
submenús incompletos** (Pedal Calibration, Encoder Calibration) que muestran texto
"placeholder" sin funcionalidad real. El submenu MODULE ENABLE/DISABLE es informativo pero
**no envía comandos CAN**. Solo FAULT VIEWER y FACTORY RESTORE funcionan completamente.

---

## 1. Hardware de Pantalla

| Parámetro | Valor |
|-----------|-------|
| Display | TFT 480×320 px (ST7796 driver) |
| Orientación | Landscape, rotación 1 |
| SPI MOSI | GPIO 13 |
| SPI SCLK | GPIO 14 |
| TFT_CS | GPIO 10 |
| TFT_DC | GPIO 39 |
| TFT_RST | GPIO 38 |
| TFT_BL (backlight) | GPIO 42 |
| Touch (XPT2046) | GPIO 21 (TOUCH_CS) |
| Velocidad SPI display | 40 MHz |
| Velocidad SPI touch | 2.5 MHz |
| Frame rate límite | 20 FPS (50 ms / frame) |
| Librería | TFT_eSPI v2.5.43+ |
| Fuentes cargadas | GLCD, Font2, Font4, Smooth Font |

---

## 2. Arquitectura de Renderizado

### 2.1 Modelo de actualización

El `ScreenManager` separa lógica de presentación:

```
loop()
  │
  ├─ can_rx::poll()          ← Decodifica tramas CAN → VehicleData
  ├─ screenManager.update()  ← Llama update(data) siempre
  │     └─ frameLimiter.shouldDraw()  ← ¿Han pasado 50 ms?
  │           └─ draw()     ← Solo si sí → renderiza pantalla activa
  └─ touch::update()         ← Detecta tap / long press
```

**Redibujado parcial:** cada elemento UI tiene valores anteriores (`prev*`). Solo se redibujan
los elementos cuyo valor cambió entre frames. Un `needsFullRedraw_` se activa al entrar en
cada pantalla para forzar el primer renderizado completo.

### 2.2 Paleta de colores (RGB565)

| Nombre | Valor | Uso |
|--------|-------|-----|
| COL_BG | `0x2104` | Fondo oscuro (gris muy oscuro) |
| COL_WHITE | `0xFFFF` | Texto general |
| COL_GREEN | `0x07E0` | OK / operativo |
| COL_AMBER | `0xFBE0` | Advertencia / cabeceras engineering |
| COL_RED | `0xF800` | Error / emergencia |
| COL_CYAN | `0x07FF` | Indicadores de ángulo 360° |
| COL_YELLOW | `0xFFE0` | Zona de peligro medio |
| COL_ORANGE | `0xFD20` | Zona de peligro alto |
| COL_GRAY | `0x8410` | Etiquetas secundarias |
| COL_DARK_GRAY | `0x4208` | Fondos de botones |

### 2.3 Zonas de la pantalla drive (480×320 px)

| Zona | Y inicio | Y fin | Contenido |
|------|----------|-------|-----------|
| Top bar | 0 | 40 | Iconos de modo (4x4/4x2/360°), toggle LED, batería |
| Sensor frontal | 40 | 85 | Distancia obstáculo + barra de proximidad |
| Car area | 85 | 230 | Carrocería top-view, 4 ruedas (par/temp), gauge dirección |
| Speed | 230 | 270 | Velocidad media grande centrada |
| Pedal bar | 270 | 300 | Barra pedal 0–100% con gradiente |
| Gear display | 300 | 320 | [P] [R] [N] [D1] [D2] plano |

---

## 3. Máquina de Estados de Pantallas

### 3.1 Transiciones

La pantalla activa se determina por `heartbeat().systemState` (CAN 0x001 byte 1):

```
SystemState (CAN)  →  Pantalla
─────────────────────────────────────────
BOOT      (0)  →  BootScreen
STANDBY   (1)  →  StandbyScreen
ACTIVE    (2)  →  DriveScreen
DEGRADED  (3)  →  DriveScreen
LIMP_HOME (6)  →  DriveScreen
SAFE      (4)  →  SafeScreen
ERROR     (5)  →  ErrorScreen
(desconocido)  →  ErrorScreen
```

Una transición activa `onExit()` en la pantalla saliente, `onEnter()` en la nueva, y fuerza
un redibujado inmediato (`forceNextFrame()`).

**Excepción:** cuando el menú de ingeniería está activo (`engineeringActive_ == true`),
las transiciones de estado se bypassean completamente hasta que el usuario pulse EXIT.

### 3.2 Pantalla BootScreen

**Archivos:** `screens/boot_screen.cpp/h`

**Estado:** ✅ Funcional

**Contenido:**
- Título "COCHE" / "MARCOS" centrado (tamaño 3)
- Subtítulo "HMI v1.0" (gris)
- Indicador CAN: "CAN: WAITING..." (rojo) / "CAN: LINKED" (verde) — redibujado parcial
- Indicador sensor obstáculo (componente `hmi::ObstacleIndicator`) — estado WAITING / VALID / INVALID

**Lógica de CAN linked:** `heartbeat.timestampMs > 0 && (now - timestampMs) < 500 ms`

### 3.3 Pantalla StandbyScreen

**Archivos:** `screens/standby_screen.cpp/h`

**Estado:** ✅ Funcional

**Contenido:**
- Header "READY" verde (tamaño 3)
- "CAN: LINKED" verde estático
- 5 temperaturas en tiempo real: FL, FR, RL, RR, AMB (°C) — redibujado parcial por sensor
- Fault flags del heartbeat (0x001 byte 2) — hex o "NO FAULTS" verde

### 3.4 Pantalla DriveScreen

**Archivos:** `screens/drive_screen.cpp/h`

**Estado:** ✅ Funcional con redibujado parcial completo

**Widgets dinámicos:**

| Widget | Fuente de datos | Descripción |
|--------|-----------------|-------------|
| ModeIcons | `data.mode().modeFlags` | Iconos 4x4 / 4x2 / 360° — toca para cambiar modo |
| LedToggle | `data.lights().relayOn` | Botón LED relay — toca para toggle |
| BatteryIndicator | `data.battery().voltageRaw` | Batería % en esquina superior derecha |
| ObstacleSensor | `data.obstacle().distanceCm` | Distancia m con 2 decimales + barra proximidad |
| CarRenderer (ruedas) | `data.traction().scale[]`, `data.tempMap().temps[]` | 4 ruedas: par % + temperatura, coloreadas |
| CarRenderer (dirección) | `data.steering().angleRaw` | Gauge circular de dirección (radio 24 px) |
| Speed | media de `data.speed().raw[]` | Velocidad grande centrada (0.1 km/h → X.X km/h) |
| PedalBar | media de `data.traction().scale[]` | Barra gradiente 0–100% |
| GearDisplay | `shifter::getGearRaw()` | Marchas físicas (MCP23017): P/R/N/D1/D2 |
| AckIndicator | `data.ack().result` | Texto breve "OK" / "REJECTED" / "TIMEOUT" — 1,5 s |

**Colores de proximidad (barra SENSOR FRONTAL):**

| Distancia | Color | Significado |
|-----------|-------|-------------|
| > 150 cm | Verde | Zona segura |
| 100–150 cm | Cyan | Precaución |
| 50–100 cm | Amarillo | Advertencia |
| 20–50 cm | Naranja | Crítico |
| < 20 cm | Rojo | Emergencia |
| 0 cm (sin lectura) | Gris | Sin datos |

### 3.5 Pantalla SafeScreen

**Archivos:** `screens/safe_screen.cpp/h`

**Estado:** ✅ Funcional

**Contenido:**
- Banner SAFE MODE ámbar (fondo ámbar, texto negro) — 480×60 px
- "Actuators inhibited" / "Controls disabled"
- Fault flags (hex, redibujado parcial)
- Error code (numérico, redibujado parcial)

### 3.6 Pantalla ErrorScreen

**Archivos:** `screens/error_screen.cpp/h`

**Estado:** ✅ Funcional

**Contenido:**
- Fondo rojo completo
- "SYSTEM ERROR" blanco sobre rojo
- "Manual reset required"
- Fault flags (0x001 byte 2) — hex
- Safety error code (0x203 byte 2) — `Code: N`
- Diagnostic info (0x300) — `E:N S:N` (error code + subsystem)

---

## 4. Menú de Ingeniería (Oculto)

### 4.1 Cómo acceder

**Código secreto "8989" — 4 taps en secuencia en la pantalla:**

| Tap # | Posición en pantalla | x coordenada |
|-------|---------------------|-------------|
| 1 | Mitad izquierda | x < 240 px |
| 2 | Mitad derecha | x ≥ 240 px |
| 3 | Mitad izquierda | x < 240 px |
| 4 | Mitad derecha | x ≥ 240 px |

**Condición:** Los 4 taps deben producirse en menos de **2000 ms** (timeout resetea el contador).
Un tap en la posición incorrecta resetea el contador a 0. Al completar la secuencia, el
`ScreenManager` muestra en Serial: `[ENG] Engineering menu activated`.

**Para salir:** Pulsar el botón EXIT (esquina inferior izquierda, 10–90 px × 280–310 px).

**Código relevante en `screen_manager.cpp`:**
```cpp
static constexpr bool SECRET_SEQUENCE[4] = { true, false, true, false }; // true=left, false=right
static constexpr unsigned long SECRET_CODE_TIMEOUT_MS = 2000;
```

### 4.2 Submenús disponibles

El menú principal muestra 5 botones (fondo gris oscuro, ancho 400 px, alto 40 px, separación 50 px):

| # | Botón | SubMenu | Estado | Descripción |
|---|-------|---------|--------|-------------|
| 0 | FAULT VIEWER | `FAULT_VIEWER` | ✅ **Funcional** | Muestra bitmasks 0x301–0x303 en tiempo real |
| 1 | MODULE ENABLE/DISABLE | `MODULE_CONTROL` | ⚠️ **Informativo** | Solo texto estático, no envía CAN |
| 2 | PEDAL CALIBRATION | `PEDAL_CAL` | ❌ **Placeholder** | Texto "Calibration interface placeholder." |
| 3 | ENCODER CALIBRATION | `ENCODER_CAL` | ❌ **Placeholder** | Texto "Calibration interface placeholder." |
| 4 | FACTORY RESTORE | (inline) | ✅ **Funcional** | Envía CAN 0x110 `action=0xFF` |

Botón **EXIT** en la esquina inferior izquierda (ámbar) devuelve a la pantalla normal.

---

### 4.3 Submenu 0 — FAULT VIEWER ✅

**Estado:** Completamente funcional.

**Datos mostrados** (redibujado parcial cuando cambian):

| Campo | CAN fuente | Color |
|-------|------------|-------|
| Fault Bits | 0x301 (`faultMask`) | Rojo si ≠ 0, verde si = 0 |
| Enabled Bits | 0x302 (`enabledMask`) | Verde |
| Disabled Bits | 0x303 (`disabledMask`) | Ámbar |

Formato: hex 32 bits `0xXXXXXXXX`. Actualización en tiempo real sin necesidad de salir.

---

### 4.4 Submenu 1 — MODULE ENABLE/DISABLE ⚠️

**Estado:** Informativo únicamente. No permite interacción.

**Lo que muestra:**
```
Use SERVICE_CMD 0x110 to enable/disable modules.
Module IDs: 0-24 (see service_mode.h)
```

**Lo que falta:** No tiene botones de módulo individuales. Para habilitar/deshabilitar módulos hay
que enviar CAN 0x110 manualmente con `byte[0]=ACTION (0=disable, 1=enable)` y `byte[1]=moduleId`.
El submenu no construye ni envía este frame.

---

### 4.5 Submenu 2 — PEDAL CALIBRATION ❌

**Estado:** Placeholder. Sin funcionalidad implementada.

**Lo que muestra:**
```
Calibration interface placeholder.
Connect engineering tool for full calibration.
```

**Lo que falta completamente:** Lectura de ADC de pedal primario y ADS1115, ajuste de valores
mínimo/máximo, persistencia de calibración en NVS, envío de parámetros al STM32 vía CAN.

---

### 4.6 Submenu 3 — ENCODER CALIBRATION ❌

**Estado:** Placeholder. Sin funcionalidad implementada.

Usa la misma función `drawCalibration("ENCODER CALIBRATION")` que el submenu anterior —
código compartido, únicamente cambia el título mostrado.

**Lo que falta completamente:** Mostrar posición del encoder de dirección (CAN 0x204), iniciar
proceso de centering desde HMI, confirmar posición centro, guardar offset en NVS, enviar
comando de calibración al STM32.

---

### 4.7 Submenu 4 — FACTORY RESTORE ✅

**Estado:** Funcional. Toca el botón FACTORY RESTORE (fondo rojo).

**Acción:** Envía inmediatamente la trama CAN:
```
ID: 0x110 (SERVICE_CMD)
DLC: 2
byte[0] = 0xFF  (SERVICE_ACTION_FACTORY_RESTORE)
byte[1] = 0x00
```
Imprime en Serial: `[ENG] Factory restore sent`

No hay confirmación visual en pantalla tras el envío. No hay diálogo "¿estás seguro?".

---

## 5. Debug Overlay (Oculto adicional)

**Estado:** ✅ Funcional (sujeto a `#define RUNTIME_MONITOR 1`)

Activado con **long press de 3 segundos** en cualquier parte de la pantalla.

**Muestra:**
- FPS actual, max, min, promedio (sobre 120 frames)
- Tiempo de render max (CAN, UI update, draw) en µs
- Contadores de redraws por zona (TOP_BAR, OBSTACLE, CAR, SPEED, PEDAL, GEAR)
- Contadores de full-redraws no intencionados

Se actualiza cada 500 ms. Dimensiones: 320×200 px en centro de pantalla.

El `RUNTIME_MONITOR` está activo por defecto (`#define RUNTIME_MONITOR 1` en `runtime_monitor.h`).

---

## 6. Sistema de Touch

**Archivos:** `touch_handler.cpp/h`

| Evento | Condición | Destino |
|--------|-----------|---------|
| TAP | Press + release < 3000 ms con debounce 200 ms | `screenManager.onTouch()` → detección código secreto |
| TAP | Zona LED toggle | Envío CAN 0x120 ON/OFF + config NVS |
| TAP | Zona mode icons | Envío CAN 0x102 modo (4x4/4x2/360°) + config NVS |
| LONG_PRESS | Press ≥ 3000 ms | Debug overlay toggle |

Cuando el menú de ingeniería está activo, los TAP se redirigen a `engineeringScreen_.handleTouch()`.
Los iconos de modo y LED **no se procesan** mientras el menú de ingeniería está abierto.

---

## 7. Persistencia (NVS)

**Archivo:** `config_store.cpp/h`

Datos persistidos entre reinicios:

| Campo | Tipo | Por defecto |
|-------|------|-------------|
| `driveMode` | uint8_t (bitmask) | 0 (4x2) |
| `brightness` | uint8_t | 100 |
| `ledEnabled` | bool | false |
| `audioVolume` | uint8_t | 15 |

Guardado con CRC32 para detectar corrupción. Flush periódico cada 10 s en `loop()`.
Se activa con `flush()` al apagado del vehículo (SHUTTING_DOWN) para no perder cambios.

---

## 8. Resumen de Estado por Componente

| Componente | Estado | Notas |
|-----------|--------|-------|
| BootScreen | ✅ Funcional | CAN link + sensor obs. status |
| StandbyScreen | ✅ Funcional | Temps + faults en tiempo real |
| DriveScreen | ✅ Funcional | Redibujado parcial, todos los widgets |
| SafeScreen | ✅ Funcional | Flags + error code |
| ErrorScreen | ✅ Funcional | Flags + safety error + diag |
| ScreenManager | ✅ Funcional | Máquina de estados + bypass eng. menu |
| FrameLimiter (20 FPS) | ✅ Funcional | 50 ms por frame |
| RuntimeMonitor | ✅ Funcional | Habilitado por defecto |
| DebugOverlay | ✅ Funcional | Long press 3 s |
| Touch handler | ✅ Funcional | Tap 200 ms debounce + long press |
| Config NVS | ✅ Funcional | Flush periódico + CRC32 |
| **Menú oculto — acceso** | ✅ **Funcional** | Código 8989 (izq-der-izq-der) |
| **Menú oculto — Fault Viewer** | ✅ **Funcional** | Bitmasks CAN 0x301-0x303 live |
| **Menú oculto — Module Control** | ⚠️ **Solo informativo** | No envía CAN 0x110 |
| **Menú oculto — Pedal Cal** | ❌ **Placeholder** | Sin implementar |
| **Menú oculto — Encoder Cal** | ❌ **Placeholder** | Sin implementar |
| **Menú oculto — Factory Restore** | ✅ **Funcional** | Envía CAN 0x110 action=0xFF |
| **Menú oculto — salir** | ✅ **Funcional** | Botón EXIT vuelve a pantalla normal |

---

## 9. Trabajo Pendiente Identificado

### Prioridad alta (afecta a funcionalidad declarada del menú)

1. **MODULE ENABLE/DISABLE:** Añadir botones individuales por módulo (IDs 0–24) con
   envío de `SERVICE_CMD 0x110` (`byte[0]=ACTION`, `byte[1]=moduleId`). El STM32 ya procesa
   este comando en `can_handler.c`.

2. **PEDAL CALIBRATION:** Implementar UI para visualizar en tiempo real el valor del pedal
   (ADC + ADS1115), ajustar min/max, y persistir en NVS.

3. **ENCODER CALIBRATION:** Implementar visualización del ángulo de dirección actual
   (0x204), proceso de centering manual, y guardado de offset.

### Prioridad baja (mejora de UX)

4. **FACTORY RESTORE sin confirmación:** Añadir diálogo de confirmación ("¿Estás seguro?")
   antes de enviar el comando, para evitar activación accidental desde el menú.

5. **Estado visual tras Factory Restore:** Mostrar confirmación/rechazo en pantalla basado
   en el ACK de STM32 (`CMD_ACK 0x103`).

---

*Fin del informe de estado.*  
*Basado en análisis de `esp32/src/` del firmware publicado en este repositorio.*
