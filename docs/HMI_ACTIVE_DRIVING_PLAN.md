# Plan de Implementación: Conducción ACTIVE desde ESP32 HMI

**Fecha:** 2026-02-21  
**Objetivo:** Habilitar control del vehículo desde la pantalla táctil ESP32 en modo ACTIVE  
**Estado actual:** ESP32 solo envía heartbeat (0x011) y obstáculo (0x208). No envía throttle/steering/mode/gear.  
**Estado objetivo:** ESP32 envía todos los comandos necesarios para conducir: throttle, steering, gear, mode.

---

## Prerrequisitos ya cumplidos (NO tocar)

| Componente | Estado | Archivo |
|------------|--------|---------|
| STM32 handler CMD_THROTTLE (0x100) | ✅ Listo | `Core/Src/can_handler.c` L487-493 |
| STM32 handler CMD_STEERING (0x101) | ✅ Listo | `Core/Src/can_handler.c` L495-502 |
| STM32 handler CMD_MODE (0x102) | ✅ Listo | `Core/Src/can_handler.c` L504-555 |
| STM32 handler SERVICE_CMD (0x110) | ✅ Listo | `Core/Src/can_handler.c` L557-593 |
| STM32 Safety_ValidateThrottle() | ✅ Listo | `Core/Src/safety_system.c` |
| STM32 Safety_ValidateSteering() | ✅ Listo | `Core/Src/safety_system.c` |
| STM32 CMD_ACK respuesta (0x103) | ✅ Listo | `Core/Src/can_handler.c` |
| ESP32 CAN IDs definidos | ✅ Listo | `esp32/include/can_ids.h` |
| ESP32 ACK tracking | ✅ Listo | `esp32/src/main.cpp` L41-83 |
| ESP32 touch hardware (TOUCH_CS=21) | ✅ Configurado | `esp32/platformio.ini` L53 |
| ESP32 tft.getTouch() funciona | ✅ Verificado | `esp32/src/main.cpp` L186 (en #if RUNTIME_MONITOR) |
| ESP32 ModeIcons::hitTest() existe | ✅ Definido | `esp32/src/ui/mode_icons.cpp` L45 (nunca llamado) |

---

## Archivos STM32 que NO se tocan

- `Core/Src/` — ningún archivo
- `Core/Inc/` — ningún archivo
- CAN IDs, payloads, formatos — congelados

---

## Orden de implementación

### Paso 1 — CAN TX: Crear módulo de transmisión de comandos

**Qué hace:** Envía frames CAN 0x100 (throttle), 0x101 (steering), 0x102 (mode+gear) con rate limiting.

**Archivos a crear:**
- `esp32/src/can/can_tx.h`
- `esp32/src/can/can_tx.cpp`

**Archivos a modificar:**
- `esp32/src/main.cpp` — añadir `#include "can/can_tx.h"`, llamar `can_tx::init()` en setup() y `can_tx::flush()` en loop()

**Dependencias:** Ninguna (paso independiente)

**Especificación funcional de `can_tx`:**
```
namespace can_tx:
  void init()                              — inicializa timestamps
  void setThrottle(uint8_t pct)            — almacena throttle, marca dirty
  void setSteering(int16_t angleRaw)       — almacena steering, marca dirty
  void setModeAndGear(uint8_t flags, uint8_t gear) — almacena mode+gear, marca dirty
  void flush()                             — envía frames pendientes respetando rate limits:
                                              throttle: cada 50ms (CMD_THROTTLE_RATE_MS)
                                              steering: cada 50ms (CMD_STEERING_RATE_MS)
                                              mode: on-demand (solo cuando dirty)
```

**Formato de frames (ya definido en CAN contract):**
- 0x100: DLC=1, byte0 = throttle_pct (0-100)
- 0x101: DLC=2, bytes0-1 = angle (int16 LE, unidades 0.1°)
- 0x102: DLC=2, byte0 = mode_flags (bit0=4x4, bit1=tank), byte1 = gear (0-4)

**Patrón a seguir:** Exactamente como `can_obstacle.cpp` — rate limiter con `millis()`, `ESP32Can.writeFrame()`.

**Criterio de validación:**
1. Conectar ESP32 al STM32 con CAN bus.
2. Llamar `can_tx::setThrottle(50)` → verificar en serial STM32 que `Traction_SetDemand(50.0)` se invoca.
3. Llamar `can_tx::setSteering(100)` → verificar en serial STM32 que `Steering_SetAngle(10.0)` se invoca (100/10=10°).
4. Llamar `can_tx::setModeAndGear(0x01, 3)` → verificar que STM32 aplica 4x4 + gear D1.
5. Verificar que throttle no se envía más de 1 vez cada 50ms.
6. Verificar que sin llamar a setThrottle(), no se envían frames de throttle.

---

### Paso 2 — Touch input: Leer touch y mapear a comandos

**Qué hace:** Lee `tft.getTouch()` en cada iteración del loop y mapea zonas de la pantalla a acciones de control.

**Archivos a crear:**
- `esp32/src/hmi/hmi_input.h`
- `esp32/src/hmi/hmi_input.cpp`

**Archivos a modificar:**
- `esp32/src/main.cpp` — añadir `#include "hmi/hmi_input.h"`, llamar `hmi_input::poll()` en loop() después de screen update, llamar `hmi_input::init()` en setup()

**Dependencias:** Paso 1 (necesita can_tx para enviar comandos)

**Especificación funcional de `hmi_input`:**
```
namespace hmi_input:
  void init()                                     — inicializa estado
  void poll(TFT_eSPI& tft, const vehicle::VehicleData& data)
    — lee tft.getTouch(&x, &y)
    — si tocado: mapea zona a acción
    — llama a can_tx::setThrottle / setSteering / setModeAndGear según zona
```

**Zonas de touch (basadas en layout existente de DriveScreen):**

| Zona pantalla | Y range | Acción | Lógica |
|---------------|---------|--------|--------|
| Gears (300-320) | 300-320 | Cambiar marcha | x position → P/R/N/D1/D2, enviar setModeAndGear |
| Mode icons (0-40) | 0-40 | Cambiar modo 4x4/4x2/360 | Usar ModeIcons::hitTest() existente |
| Pedal bar (270-300) | 270-300 | Ajustar throttle | x position → 0-100%, enviar setThrottle |
| Center car (85-230) | 85-230 | Steering | x position relativa al centro → ángulo, enviar setSteering |

**Protección de seguridad:**
- Throttle se envía a 0 cuando no hay touch activo en zona pedal (dead-man switch)
- Solo envía comandos si el sistema está en ACTIVE o DEGRADED (verificar `data.heartbeat().systemState`)
- En LIMP_HOME/SAFE/ERROR/STANDBY/BOOT: no enviar comandos de control

**Criterio de validación:**
1. Tocar zona de gear D1 → verificar CAN 0x102 con byte1=3 (D1).
2. Tocar zona de pedal en mitad → verificar CAN 0x100 con byte0≈50.
3. Tocar centro de pantalla → verificar CAN 0x101 con angle≈0.
4. Tocar borde derecho → verificar CAN 0x101 con angle positivo.
5. Sin tocar → throttle debe enviarse como 0.
6. En STANDBY → no debe enviarse ningún comando.

---

### Paso 3 — DriveScreen: Conectar gear y mode al estado real

**Qué hace:** Actualiza DriveScreen para reflejar el gear y mode que el usuario ha seleccionado via touch, en vez de mostrar siempre N y sin-modo.

**Archivos a crear:** Ninguno

**Archivos a modificar:**
- `esp32/src/hmi/hmi_input.h` — exportar estado actual de gear y mode
- `esp32/src/screens/drive_screen.cpp` — leer gear/mode del input handler en vez de hardcodear N/false

**Dependencias:** Paso 2

**Cambios concretos en `drive_screen.cpp`:**
- Línea 100: `curGear_ = ui::Gear::N;` → leer del estado guardado en hmi_input
- Líneas 103-104: `curMode_.is4x4 = false; curMode_.isTankTurn = false;` → leer del estado guardado en hmi_input

**Criterio de validación:**
1. Tocar D1 en pantalla → gear display cambia de N a D1 (resaltado verde).
2. Tocar icono 4x4 → icono se ilumina cyan.
3. Tocar icono 4x2 → icono 4x4 se desactiva, 4x2 se ilumina.
4. Cambios se reflejan inmediatamente en siguiente frame (≤50ms).

---

### Paso 4 — Throttle release safety: Enviar throttle=0 cuando no hay touch

**Qué hace:** Garantiza que cuando el operador levanta el dedo de la zona de throttle, se envía throttle=0 al STM32.

**Archivos a modificar:**
- `esp32/src/hmi/hmi_input.cpp` — en poll(), cuando no hay touch o touch fuera de zona pedal: `can_tx::setThrottle(0)`

**Dependencias:** Pasos 1 y 2

**Criterio de validación:**
1. Tocar pedal → vehículo se mueve.
2. Soltar → CAN 0x100 byte0=0 enviado en ≤50ms.
3. Verificar en STM32 serial que Traction_SetDemand(0.0) se invoca.
4. Vehículo se detiene (dynamic braking activo).

---

## Diagrama de dependencias

```
Paso 1 (can_tx)
    │
    ├── Paso 2 (hmi_input) ──── Paso 3 (drive_screen feedback)
    │       │
    │       └── Paso 4 (throttle release)
    │
    (independiente del roadmap de seguridad — Pasos 1/7 no se tocan)
```

---

## Resumen de archivos

### Archivos nuevos (4):
| Archivo | Paso | Contenido |
|---------|------|-----------|
| `esp32/src/can/can_tx.h` | 1 | Header del módulo CAN TX |
| `esp32/src/can/can_tx.cpp` | 1 | Implementación CAN TX con rate limiting |
| `esp32/src/hmi/hmi_input.h` | 2 | Header del módulo de input táctil |
| `esp32/src/hmi/hmi_input.cpp` | 2 | Implementación touch → CAN commands |

### Archivos modificados (2):
| Archivo | Paso | Cambio |
|---------|------|--------|
| `esp32/src/main.cpp` | 1,2 | Añadir includes + init + poll/flush en loop |
| `esp32/src/screens/drive_screen.cpp` | 3 | Leer gear/mode del input handler |

### Archivos NO tocados:
- `Core/Src/*` — ninguno
- `Core/Inc/*` — ninguno
- `esp32/include/can_ids.h` — congelado
- `esp32/src/vehicle_data.h` — sin cambios
- `esp32/src/can_rx.cpp` — sin cambios
- `esp32/src/screen_manager.cpp` — sin cambios
- `esp32/platformio.ini` — sin cambios (touch ya configurado)

---

## Lo que NO se hace en este plan

- ❌ No se modifica el roadmap de seguridad (Pasos 1/7 siguen pendientes)
- ❌ No se añade adaptive cruise, AI regen, audio, LEDs
- ❌ No se refactoriza código existente
- ❌ No se modifica ningún archivo STM32
- ❌ No se modifica el protocolo CAN
- ❌ No se añade SERVICE_CMD (0x110) — es mantenimiento, no necesario para conducir
- ❌ No se añade CAN 0x209 decode — informacional, no necesario para conducir

---

## Criterio final de éxito

El vehículo puede conducirse desde la pantalla táctil del ESP32:

1. ESP32 envía heartbeat → STM32 sale de STANDBY a ACTIVE
2. Operador toca gear D1 → STM32 acepta cambio de marcha (CAN ACK OK)
3. Operador toca zona de pedal → vehículo se mueve a velocidad proporcional al touch
4. Operador toca zona de steering → dirección gira proporcionalmente
5. Operador levanta el dedo → throttle=0, vehículo se detiene
6. Si el operador deja de tocar durante >250ms, no se envían comandos de control (solo heartbeat)
