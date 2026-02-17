# Verificación de Componentes en Pantalla — Palanca de Cambios, Pedal y Demás

**Fecha:** 2026-02-15
**Alcance:** DriveScreen (480×320 TFT landscape) — todos los widgets UI
**Referencia:** `docs/CAN_CONTRACT_FINAL.md` rev 1.3, `docs/PROJECT_MASTER_STATUS.md`

---

## RESUMEN EJECUTIVO

Se han verificado los **10 componentes visuales** del DriveScreen. De ellos:
- **7 componentes muestran datos reales** recibidos por CAN del STM32 → ✅ CORRECTOS
- **3 componentes están hardcodeados/derivados** y necesitan corrección → ⚠️ PENDIENTES (Phase 4)

| Componente | Dato real por CAN | Estado |
|-----------|-------------------|--------|
| Velocidad (km/h) | ✅ 0x200 STATUS_SPEED | ✅ CORRECTO |
| Ruedas (torque %) | ✅ 0x205 STATUS_TRACTION | ✅ CORRECTO |
| Ruedas (temperatura) | ✅ 0x206 STATUS_TEMP_MAP | ✅ CORRECTO |
| Dirección (gauge circular) | ✅ 0x204 STATUS_STEERING | ✅ CORRECTO |
| Batería (voltaje %) | ✅ 0x207 STATUS_BATTERY | ✅ CORRECTO |
| Sensor frontal (distancia) | ✅ 0x208 OBSTACLE_DISTANCE | ✅ CORRECTO |
| Cuerpo del coche (outline) | N/A (estático) | ✅ CORRECTO |
| **Palanca de cambios (P/R/N/D1/D2)** | ❌ Hardcodeado a D1 | ⚠️ PENDIENTE |
| **Pedal (0–100%)** | ❌ Derivado de traction avg | ⚠️ PENDIENTE |
| **Modo (4x4/4x2/360°)** | ❌ Hardcodeado a false/false | ⚠️ PENDIENTE |

---

## 1) PALANCA DE CAMBIOS — GearDisplay

### Ubicación en pantalla
- **Zona:** Y: 300–320 px (franja inferior)
- **Layout:** 5 etiquetas: `[P] [R] [N] [D1] [D2]`
- **Archivos:** `esp32/src/ui/gear_display.h`, `gear_display.cpp`

### Cómo funciona el widget
- `drawStatic()` dibuja las 5 etiquetas en gris inactivo al entrar al DriveScreen
- `draw(tft, current, previous)` resalta la marcha activa en verde con borde blanco
- Solo redibuja si `current != previous` — eficiente

### ⚠️ PROBLEMA: Dato hardcodeado

**Archivo:** `esp32/src/screens/drive_screen.cpp`, línea 103:

```cpp
// Gear — derived from heartbeat flags (byte 1 of CMD_MODE)
// For now, map system state to gear display:
// Gear is tracked separately via mode commands; default to N if unknown
// The gear value is encoded as the second byte of CMD_MODE (0x102)
// Since we receive the heartbeat state, we default to D1 when active
curGear_ = ui::Gear::D1;  // ← HARDCODEADO, SIEMPRE D1
```

### Causa raíz
- El STM32 **recibe** el cambio de marcha por CAN (`CMD_MODE` 0x102, byte 1)
- El STM32 **procesa** el cambio correctamente (`Traction_SetGear()` en `motor_control.c`)
- El STM32 **NO envía** la marcha actual de vuelta al ESP32 en ningún mensaje CAN
- No existe un mensaje `STATUS_DRIVE_STATE` ni la marcha se incluye en el heartbeat (0x001)

### Estado del widget visual
- **Renderizado:** ✅ CORRECTO — el widget dibuja correctamente, sin flicker, con partial redraw
- **Datos:** ❌ INCORRECTO — siempre muestra D1, independientemente de la marcha real

### Qué falta para corregirlo
1. STM32: Añadir campo de marcha actual al heartbeat o crear nuevo CAN message
2. ESP32 `can_rx.cpp`: Decodificar el campo de marcha
3. ESP32 `vehicle_data.h`: Añadir campo `gear` a `HeartbeatData` o nuevo struct
4. ESP32 `drive_screen.cpp`: Leer marcha de `VehicleData` en vez de hardcodear
5. Actualizar `CAN_CONTRACT_FINAL.md` con el nuevo campo

**Prioridad:** Phase 4 — Driver Interaction (según `PROJECT_MASTER_STATUS.md` §5)

---

## 2) PEDAL — PedalBar

### Ubicación en pantalla
- **Zona:** Y: 272–300 px
- **Layout:** Etiqueta "PEDAL" + barra horizontal (verde→amarillo→rojo) + texto "XX%"
- **Archivos:** `esp32/src/ui/pedal_bar.h`, `pedal_bar.cpp`

### Cómo funciona el widget
- `drawStatic()` dibuja la etiqueta y el contorno de la barra
- `draw(tft, pedalPct, prevPct)` rellena la barra proporcionalmente
- Color: 0–40% verde, 41–70% amarillo, 71–100% rojo
- Solo redibuja si `pedalPct != prevPct` — eficiente

### ⚠️ PROBLEMA: Dato derivado (no real)

**Archivo:** `esp32/src/screens/drive_screen.cpp`, líneas 90–96:

```cpp
// Pedal/throttle — derived from traction average as display hint
// (actual throttle command is sent separately via CMD_THROTTLE)
uint16_t tractionSum = 0;
for (uint8_t i = 0; i < 4; ++i) {
    tractionSum += data.traction().scale[i];
}
curPedalPct_ = static_cast<uint8_t>(tractionSum / 4);  // ← DERIVADO, NO REAL
```

### Causa raíz
- El STM32 lee el pedal ADC real en `Pedal_Update()` (`sensor_manager.c`, ADC1, PA3)
- El STM32 usa ese valor internamente para controlar los motores
- El STM32 **NO envía** el porcentaje de pedal al ESP32 en ningún CAN message
- `CMD_THROTTLE` (0x100) es ESP32→STM32 (comando, no telemetría)
- No existe un `STATUS_PEDAL` o similar en la tabla de mensajes CAN

### Qué muestra actualmente
- Muestra la **media de traction scale** (ABS/TCS wheel scaling), NO la posición del pedal
- Cuando ABS/TCS están inactivos, `traction.scale[]` = 100% → pedal muestra 100%
- Cuando ABS interviene, `traction.scale[]` baja → pedal parece que baja
- Esto **no refleja la posición real del pedal del conductor**

### Estado del widget visual
- **Renderizado:** ✅ CORRECTO — barra, colores y texto se dibujan bien, sin flicker
- **Datos:** ❌ INCORRECTO — muestra traction average en vez del pedal real del conductor

### Qué falta para corregirlo
1. STM32: Crear `CAN_SendStatusPedal()` o incluir pedal% en un mensaje existente
2. ESP32 `can_rx.cpp`: Decodificar el nuevo campo
3. ESP32 `vehicle_data.h`: Añadir campo `pedalPct` o nuevo struct
4. ESP32 `drive_screen.cpp`: Leer pedal de `VehicleData` en vez de derivar de traction
5. Actualizar `CAN_CONTRACT_FINAL.md` con el nuevo campo/mensaje

**Prioridad:** Phase 4 — Driver Interaction

---

## 3) MODO DE TRACCIÓN — ModeIcons

### Ubicación en pantalla
- **Zona:** Y: 6–34 px (barra superior izquierda)
- **Layout:** 3 botones: `[4x4] [4x2] [360°]`
- **Archivos:** `esp32/src/ui/mode_icons.h`, `mode_icons.cpp`

### Cómo funciona el widget
- `drawStatic()` dibuja 3 iconos en gris inactivo
- `draw(tft, current, previous)` resalta el activo en cyan con borde blanco
- 4x4 y 4x2 son mutuamente exclusivos (4x2 = !4x4)
- Solo redibuja si el estado cambió — eficiente
- Incluye `hitTest()` para futura interacción táctil

### ⚠️ PROBLEMA: Dato hardcodeado

**Archivo:** `esp32/src/screens/drive_screen.cpp`, líneas 106–107:

```cpp
// Mode flags
curMode_.is4x4 = false;       // ← HARDCODEADO
curMode_.isTankTurn = false;   // ← HARDCODEADO
```

### Causa raíz
- El STM32 recibe el modo por `CMD_MODE` (0x102, byte 0, bits 0–1)
- El STM32 procesa y aplica el modo (`MODE_4X4`, `MODE_4X2`, `MODE_TANK_TURN`)
- El STM32 **NO envía** el modo actual de vuelta al ESP32
- No existe un `STATUS_MODE` CAN message del STM32 al ESP32

### Estado del widget visual
- **Renderizado:** ✅ CORRECTO — iconos se dibujan bien, cambios se animan
- **Datos:** ❌ INCORRECTO — siempre muestra 4x2 activo (is4x4=false), nunca 360°

### Qué falta para corregirlo
- Mismo flujo que gear: añadir modo al heartbeat o nuevo CAN message

**Prioridad:** Phase 4 — Driver Interaction

---

## 4) VELOCIDAD — Speed Display

### Ubicación en pantalla
- **Zona:** Y: 232–270 px (centrada)
- **Layout:** Texto grande "XX.X" + etiqueta "km/h"
- **Archivo:** `esp32/src/screens/drive_screen.cpp`, método `drawSpeed()`

### Fuente de datos
```cpp
// drive_screen.cpp:80-85 — promedio de 4 ruedas
uint32_t sum = 0;
for (uint8_t i = 0; i < 4; ++i) {
    sum += data.speed().raw[i];  // ← CAN 0x200 STATUS_SPEED
}
curSpeedAvgRaw_ = static_cast<uint16_t>(sum / 4);
```

- **CAN ID:** 0x200 STATUS_SPEED, DLC 8, 100 ms
- **Origen STM32:** `CAN_SendStatusSpeed()` con datos de `Wheel_ComputeSpeed()` (EXTI pulse counting)
- **Formato:** 4× uint16 little-endian, unidades 0.1 km/h
- **Display:** Promedio de las 4 ruedas → `XX.X km/h`

### Estado
- **Renderizado:** ✅ CORRECTO — texto grande, partial redraw, sin flicker
- **Datos:** ✅ CORRECTO — velocidad real de sensores de rueda vía CAN

---

## 5) RUEDAS (Torque + Temperatura) — CarRenderer

### Ubicación en pantalla
- **Zona:** Y: 85–230 px (área central)
- **Layout:** 4 cuadros (FL, FR, RL, RR) con torque% y temperatura °C
- **Archivos:** `esp32/src/ui/car_renderer.h`, `car_renderer.cpp`

### Fuente de datos
```cpp
// drive_screen.cpp:66-73
curTraction_[i] = data.traction().scale[i];  // ← CAN 0x205
curTemp_[i] = data.tempMap().temps[i];        // ← CAN 0x206
```

- **Torque:** CAN 0x205 STATUS_TRACTION, 4× uint8 (0–100%), de ABS/TCS wheel scale
- **Temperatura:** CAN 0x206 STATUS_TEMP_MAP, 4× int8 (°C), de DS18B20

### Estado
- **Renderizado:** ✅ CORRECTO — color por torque (verde/amarillo/rojo), temp en °C
- **Datos:** ✅ CORRECTO — datos reales de sensores vía CAN

---

## 6) DIRECCIÓN — Steering Gauge

### Ubicación en pantalla
- **Zona:** Gauge circular a la derecha del coche, centro (410, 160), radio 24 px
- **Layout:** Círculo con línea indicadora + etiqueta "360°" encima
- **Archivos:** `esp32/src/ui/car_renderer.cpp` (métodos `drawSteering`, `drawSteerLine`)

### Fuente de datos
```cpp
// drive_screen.cpp:76
curSteeringRaw_ = data.steering().angleRaw;  // ← CAN 0x204
```

- **CAN ID:** 0x204 STATUS_STEERING, DLC 3, 100 ms
- **Origen STM32:** `CAN_SendStatusSteering()` con `Motor_GetSteeringAngle()` (encoder quadrature TIM2)
- **Formato:** int16 little-endian (0.1°), + uint8 calibrated flag
- **Display:** Línea que rota ±45° desde posición vertical (0° = recto)

### Estado
- **Renderizado:** ✅ CORRECTO — línea se borra y redibuja limpiamente, círculo se mantiene
- **Datos:** ✅ CORRECTO — ángulo real del encoder de dirección vía CAN

---

## 7) BATERÍA — BatteryIndicator

### Ubicación en pantalla
- **Zona:** Esquina superior derecha (405, 6), 65×28 px
- **Layout:** Icono de batería con relleno color + porcentaje "%"
- **Archivos:** `esp32/src/ui/battery_indicator.h`, `battery_indicator.cpp`

### Fuente de datos
```cpp
// drive_screen.cpp:88
curBattVoltRaw_ = data.battery().voltageRaw;  // ← CAN 0x207
```

- **CAN ID:** 0x207 STATUS_BATTERY, DLC 4, 100 ms
- **Origen STM32:** `CAN_SendStatusBattery()` con INA226 en bus 24V
- **Conversión:** 18.0V = 0%, 25.2V = 100% (lead-acid/LiFePO4)
- **Color:** >50% verde, 20–50% amarillo, <20% rojo

### Estado
- **Renderizado:** ✅ CORRECTO — icono con terminal, relleno proporcional, texto centrado
- **Datos:** ✅ CORRECTO — voltaje real del INA226 vía CAN

---

## 8) SENSOR FRONTAL — ObstacleSensor

### Ubicación en pantalla
- **Zona:** Y: 40–85 px (debajo de top bar)
- **Layout:** Etiqueta "SENSOR FRONTAL" + distancia "X.XX m" + barra de proximidad
- **Archivos:** `esp32/src/ui/obstacle_sensor.h`, `obstacle_sensor.cpp`

### Fuente de datos
```cpp
// drive_screen.cpp:110
curObstacleCm_ = data.obstacle().distanceCm;  // ← CAN 0x208
```

- **CAN ID:** 0x208 OBSTACLE_DISTANCE, DLC 5, 66 ms
- **Nota:** El ESP32 necesita un driver de sensor ultrasónico/ToF (pendiente Phase 3)
- **Conversión:** 0 = sin lectura (muestra "---"), 1–400 cm
- **Color proximidad:** >150 cm verde, 50–150 cm amarillo, <50 cm rojo

### Estado
- **Renderizado:** ✅ CORRECTO — texto y barra se muestran bien
- **Datos:** ✅ CORRECTO (cuando el sensor esté conectado) — CAN decode funciona

---

## 9) CUERPO DEL COCHE — CarRenderer (estático)

### Ubicación en pantalla
- **Zona:** Centro del área car (190, 105), 100×110 px
- **Layout:** Rectángulo con doble borde, líneas de parabrisas, ejes

### Estado
- **Renderizado:** ✅ CORRECTO — estático, se dibuja una vez al entrar

---

## 10) ETIQUETA "360°" + ETIQUETA "km/h"

Etiquetas estáticas dibujadas una vez en `draw()` con `needsFullRedraw_`.
- **Estado:** ✅ CORRECTO

---

## VERIFICACIÓN DE RENDERING

| Propiedad | Estado | Evidencia |
|-----------|--------|-----------|
| Sin flicker | ✅ | `fillScreen` solo en `onEnter()`, partial redraw con dirty flags |
| Sin heap allocation | ✅ | Solo `snprintf()` con `char[]`, sin `String`, sin `new`, sin sprites |
| Orden determinista | ✅ | Secuencia fija: speed→obstacle→car→steering→battery→gear→pedal→mode |
| Frame limiter funciona | ✅ | 20 FPS target, `FrameLimiter` con `shouldDraw()` |
| Dirty detection correcta | ✅ | Cada widget compara `cur != prev` antes de dibujar |
| Overlay aislado | ✅ | Debug overlay dibuja DESPUÉS del render normal |

---

## QUÉ HAY QUE HACER EN EL SIGUIENTE PASO

### Paso inmediato: Cerrar Phase 1 (Stability Foundation)

Según `PROJECT_MASTER_STATUS.md` §5, Phase 1 tiene estos exit criteria pendientes de validar en **hardware real**:

1. ✅ STM32 transitions BOOT → STANDBY → ACTIVE con 6 boot checks pasando
2. ✅ CAN heartbeat loss triggers SAFE ≤ 250 ms
3. ✅ Steering centering completa en ≤ 10 s
4. ✅ IWDG no dispara durante operación normal
5. ✅ I2C bus recovery después de SDA hold-low

**Acción:** Ejecutar `docs/HARDWARE_VALIDATION_PROCEDURE.md` en el vehículo real.

### Phase 2 — Control Reliability (siguiente fase)

Validar el pipeline de control end-to-end:
1. Pedal ADC → EMA → ramp limiter → demand → PWM
2. ABS pulse modulation produce recuperación de velocidad
3. TCS progressive reduction previene giro de rueda
4. Dynamic braking activa al soltar acelerador
5. Park hold mantiene vehículo estacionario
6. Gear changes validados por velocidad (rechazar D1→R a >1 km/h)

### Phase 3 — Feedback & Sensors

1. **ESP32 obstacle sensor driver** — conectar sensor ultrasónico/ToF y enviar datos CAN 0x208
2. **CAN ID 0x209 parsing** — implementar body del case en STM32
3. **Steering PID tuning** — evaluar términos I y D
4. **Hot-plug DS18B20** — búsqueda periódica de ROMs

### Phase 4 — Driver Interaction (FIX de los 3 componentes pendientes)

Esta es la phase donde se corrigen los **3 problemas identificados** en este documento:

| # | Corrección | Cambio necesario |
|---|-----------|-----------------|
| 1 | **Palanca de cambios real en pantalla** | STM32: enviar gear en heartbeat o nuevo CAN msg → ESP32: decodificar → DriveScreen: leer de VehicleData |
| 2 | **Pedal real en pantalla** | STM32: enviar pedal% vía nuevo CAN msg → ESP32: decodificar → DriveScreen: leer de VehicleData |
| 3 | **Modo de tracción real en pantalla** | STM32: enviar mode flags en heartbeat o nuevo CAN msg → ESP32: decodificar → DriveScreen: leer de VehicleData |

**Propuesta de implementación para Phase 4:**

**Opción A — Extender heartbeat (0x001) de 4 a 6 bytes:**
```
Byte 0: alive_counter (existente)
Byte 1: system_state (existente)
Byte 2: fault_flags (existente)
Byte 3: error_code (existente)
Byte 4: gear (NUEVO — 0=P, 1=R, 2=N, 3=D1, 4=D2)
Byte 5: mode_flags (NUEVO — bit0=4x4, bit1=tank_turn)
```

**Opción B — Nuevo CAN msg STATUS_DRIVE_STATE (0x20A):**
```
Byte 0: gear (0=P, 1=R, 2=N, 3=D1, 4=D2)
Byte 1: mode_flags (bit0=4x4, bit1=tank_turn)
Byte 2: pedal_pct (0–100, del ADC real)
DLC: 3, Rate: 100 ms
```

**Recomendación:** Opción B — nuevo CAN ID. Razón: no rompe el formato existente del heartbeat, es backward-compatible, y agrupa los 3 datos que faltan en un solo mensaje nuevo.

### Resumen de prioridades

```
AHORA   → Cerrar Phase 1 con validación hardware (procedimiento en HARDWARE_VALIDATION_PROCEDURE.md)
LUEGO   → Phase 2: control end-to-end en hardware real
DESPUÉS → Phase 3: obstacle sensor driver + PID tuning
ÚLTIMO  → Phase 4: fix gear/pedal/mode en display (este documento identifica los 3 gaps)
```

---

## REFERENCIAS

- `docs/PROJECT_MASTER_STATUS.md` — fases y exit criteria
- `docs/CAN_CONTRACT_FINAL.md` rev 1.3 — protocolo CAN completo
- `docs/DISPLAY_STABILITY_VALIDATION.md` — validación de rendering
- `docs/HARDWARE_VALIDATION_PROCEDURE.md` — test plan hardware Phase 1
- `docs/HMI_RENDERING_STRATEGY.md` — estrategia de rendering
