# PROMPT DE CONTINUACIÓN PARA EL SIGUIENTE AGENTE

> **Fecha:** 2026-02-22
> **Repositorio:** `florinzgz/STM32-Control-Coche-Marcos`
> **Branch activo:** `copilot/audit-firmware-comparison`
> **Última equivalencia funcional:** 82% (objetivo: 100%)

---

## 1. QUÉ ES ESTE PROYECTO

Firmware de control de un coche eléctrico infantil con arquitectura dual:
- **STM32G474RE**: Controlador de motores, seguridad, sensores (es la autoridad de seguridad)
- **ESP32-S3**: HMI (pantalla táctil 480×320), audio, LEDs, lectura de periféricos de usuario

Ambos MCUs se comunican exclusivamente por **CAN bus a 500 kbps** (CAN 2.0A, 11-bit IDs).

---

## 2. ESTADO ACTUAL DEL CÓDIGO — QUÉ FUNCIONA

### STM32 (directorio `Core/`)
| Módulo | Archivo(s) | Estado |
|--------|-----------|--------|
| Control PWM motores (4 ruedas + dirección) | `motor_control.c/h` | ✅ Completo |
| Ackermann geometría dirección | `ackermann.c/h` | ✅ Completo |
| EPS torque-assist dirección | `motor_control.c`, `eps_params.c/h` | ✅ Completo |
| Calibración centro dirección | `steering_centering.c/h`, `steering_cal_store.c/h` | ✅ Completo |
| Encoder dirección (TIM2 quadratura) | `encoder_reader.c/h` | ✅ Completo |
| Máquina de estados seguridad (7 estados) | `safety_system.c/h` | ✅ Completo |
| ABS/TCS por rueda | `safety_system.c` | ✅ Completo |
| Protección corriente/temperatura/batería | `safety_system.c` | ✅ Completo |
| Detección obstáculos (3 zonas) | `safety_system.c` (Obstacle_*) | ⚠️ Parcial (faltan 2 zonas) |
| Sensores (INA226, DS18B20, pedal, velocidad) | `sensor_manager.c/h` | ✅ Completo |
| Modo servicio (25 módulos) | `service_mode.c/h` | ✅ Completo |
| Validación de arranque | `boot_validation.c/h` | ✅ Completo |
| Matemáticas seguras (NaN/Inf) | `math_safety.c/h` | ✅ Completo (con tests) |
| CAN handler completo | `can_handler.c/h` | ✅ Completo |
| Relé LED en PB10 | `can_handler.c`, `main.c` | ✅ Completo |
| Main loop (10/50/100/1000 ms tareas) | `main.c` | ✅ Completo |

### ESP32 (directorio `esp32/`)
| Módulo | Archivo(s) | Estado |
|--------|-----------|--------|
| CAN RX/TX completo | `can_rx.cpp/h`, `can_ids.h` | ✅ Completo |
| VehicleData store | `vehicle_data.h/cpp` | ✅ Completo |
| Screen Manager (5 pantallas) | `screen_manager.cpp/h` | ✅ Completo |
| Boot Screen | `screens/boot_screen.cpp/h` | ✅ Completo |
| Standby Screen | `screens/standby_screen.cpp/h` | ✅ Completo |
| Drive Screen (telemetría completa) | `screens/drive_screen.cpp/h` | ✅ Completo |
| Safe Screen | `screens/safe_screen.cpp/h` | ✅ Completo |
| Error Screen | `screens/error_screen.cpp/h` | ✅ Completo |
| UI Components (car, pedal, gear, battery, steering, obstacle, mode icons) | `ui/*.cpp/h` | ✅ Completo |
| LED toggle UI (botón en top bar) | `ui/led_toggle.cpp/h` | ✅ Completo |
| WS2812B LED controller (FastLED, 44 LEDs) | `led_controller.cpp/h` | ✅ Completo |
| Power Manager (llave contacto GPIO 40/41) | `power_manager.cpp/h` | ✅ Completo |
| Audio Manager (DFPlayer UART2 GPIO 43/44) | `audio_manager.cpp/h` | ✅ Completo |
| Sensor obstáculo HC-SR04 | `sensors/obstacle_sensor.cpp/h` | ✅ Completo |
| CAN TX obstáculo (0x208) | `can/can_obstacle.cpp/h` | ✅ Completo |
| Heartbeat ESP32 (0x011 cada 100ms) | `main.cpp` | ✅ Completo |
| ACK tracking (non-blocking) | `main.cpp` | ✅ Completo |
| Runtime monitor / debug overlay | `ui/runtime_monitor.cpp/h`, `ui/debug_overlay.cpp/h` | ✅ Completo |

---

## 3. QUÉ FALTA POR IMPLEMENTAR (9 sistemas pendientes, en orden)

### Paso 1 — Palanca de marchas física (MCP23017) — PRIORIDAD CRÍTICA
**Qué hace:** Lee la posición física del selector de marchas (P/R/N/D1/D2) vía I2C MCP23017.
**Por qué es crítico:** Sin esto, no se envía CAN 0x102 byte 1 con la marcha real. Actualmente `drive_screen.cpp` tiene `curGear_ = ui::Gear::N` hardcodeado (línea 100).
**Archivos a crear:**
- `esp32/src/shifter_input.cpp` — Driver MCP23017 I2C
- `esp32/src/shifter_input.h` — Interfaz
**Archivos a modificar:**
- `esp32/src/main.cpp` — Llamar `shifter::update()` en loop, enviar CAN 0x102 con marcha actual
- `esp32/platformio.ini` — Puede necesitar lib_dep para MCP23017
**CAN existente:** CMD_MODE (0x102) byte 1 ya acepta gear 0-4 en STM32. El STM32 ya valida velocidad ≤1 km/h para cambio.
**GPIOs ESP32 disponibles:** Los pines I2C del ESP32 deben definirse (no hay I2C configurado aún en el ESP32; solo está CAN en GPIO 4/5, TFT en GPIO 13-17/42, touch en GPIO 21, LED en GPIO 38, power en GPIO 40/41, audio en GPIO 43/44, obstacle en GPIO 6/7).
**Enum existente en STM32:** `GearPosition_t` en `motor_control.h`: GEAR_PARK=0, GEAR_REVERSE=1, GEAR_NEUTRAL=2, GEAR_FORWARD=3, GEAR_FORWARD_D2=4

### Paso 3 — Entrada táctil completa (XPT2046) — PRIORIDAD ALTA
**Qué hay:** Touch ya funciona para LED toggle en `main.cpp` líneas 217-232 (`tft.getTouch()`). `mode_icons.cpp` ya tiene `hitTest()` implementado (devuelve 1=4x4, 2=4x2, 3=360°).
**Qué falta:** Conectar `ModeIcons::hitTest()` al touch loop para enviar CMD_MODE (0x102) al tocar iconos de modo. También falta touch para cambio de pantallas y base para el menú de ingeniería.
**Archivos a crear:**
- `esp32/src/touch_handler.cpp/h` — Lectura centralizada de touch con debounce y dispatch
**Archivos a modificar:**
- `esp32/src/main.cpp` — Reemplazar touch directo por `touch_handler`, dispatch a screen activa
- `esp32/src/screen_manager.cpp/h` — Pasar eventos touch a la pantalla activa
**CAN existente:** CMD_MODE (0x102) byte 0 bits: 0x01=4x4, 0x02=tank_turn. ACK vía 0x103.
**Patrón existente:** Ver `sendLedCommand()` en `main.cpp` y `ackBeginWait()` para cómo enviar comandos con ACK.

### Paso 4 — 5 zonas de obstáculo completas — PRIORIDAD ALTA
**Qué hay:** STM32 `safety_system.c` tiene 3 zonas: EMERGENCY <200mm (scale=0.0), CRITICAL 200-500mm, WARNING 500-1000mm.
**Qué falta:** Añadir zona Caution (1000-1500mm) y zona Alert (1500-4000mm) con factores del original.
**Archivos a modificar:**
- `Core/Src/safety_system.c` — Ampliar `Obstacle_Update()` de 3 a 5 zonas
- `Core/Inc/safety_system.h` — Añadir constantes OBSTACLE_CAUTION_MM=1500, OBSTACLE_ALERT_MM=4000
- `esp32/src/sensors/obstacle_sensor.cpp` — Actualizar mapeo de zonas
- `esp32/src/ui/obstacle_sensor.cpp` — Actualizar display
**No toca CAN:** La infraestructura CAN 0x208 ya tiene campo zone (byte 2, uint8) y obstacle_scale ya existe.

### Paso 5 — Detección de reacción infantil — PRIORIDAD ALTA
**Qué hace:** Cuando el niño suelta el pedal (reducción >10% en 500ms), modifica factores en zonas 2-3.
**Archivos a modificar:**
- `Core/Src/safety_system.c` — Lógica de detección de soltar pedal rápido
- `Core/Inc/safety_system.h` — Constantes CHILD_REACTION_THRESHOLD=10%, CHILD_REACTION_WINDOW_MS=500
**Depende de:** Paso 4 (5 zonas). `Pedal_GetPercent()` ya existe en `sensor_manager.c`.

### Paso 8 — Menú de ingeniería oculto — PRIORIDAD MEDIA
**Qué hace:** Código secreto 8989 en pantalla → submenús: calibración pedal/encoder, enable/disable módulos, factory restore, visor de errores.
**Archivos a crear:**
- `esp32/src/screens/engineering_screen.cpp/h` — Pantalla con submenús
**Archivos a modificar:**
- `esp32/src/screen_manager.cpp/h` — Añadir transición por código secreto
- Necesita touch_handler (Paso 3)
**CAN existente:** SERVICE_CMD (0x110) ya soporta enable/disable/factory-restore. SERVICE_FAULTS/ENABLED/DISABLED (0x301-0x303) ya decodificados en `can_rx.cpp`.

### Paso 9 — Persistencia NVS en ESP32 — PRIORIDAD MEDIA
**Qué hace:** Guarda configuración HMI (modo preferido, brillo, calibraciones) en NVS del ESP32.
**Archivos a crear:**
- `esp32/src/config_store.cpp/h` — Wrapper NVS con validación CRC
**No necesita librería externa:** ESP32 tiene NVS nativo (`<Preferences.h>` o `<nvs_flash.h>`).

### Paso 10 — Persistencia de log de errores — PRIORIDAD MEDIA
**Qué hace:** Guarda últimos N errores en flash del STM32 (buffer circular con CRC).
**Archivos a crear:**
- `Core/Src/error_logger.c/h` — Buffer circular en flash páginas 124-125 (126=steering cal, 127=EPS)
**Archivos a modificar:**
- `Core/Src/safety_system.c` — Llamar a error_logger en cada cambio de estado de error
- `Core/Src/can_handler.c` — Añadir comando CAN para solicitar log

### Paso 11 — Crucero adaptativo (ACC) — PRIORIDAD BAJA
**Qué hace:** PID (Kp=0.3, Ki=0.05, Kd=0.15) mantiene 500mm de distancia al obstáculo frontal.
**Archivos a crear:**
- `esp32/src/adaptive_cruise.cpp/h`
**Depende de:** Paso 4 (5 zonas). Usa CMD_THROTTLE (0x100) existente.

### Paso 12 — Frenado regenerativo — PRIORIDAD BAJA
**Qué hace:** Lookup tables velocidad/aceleración/SOC/temperatura → corriente regen en motores.
**Archivos a crear:**
- `Core/Src/regen_braking.c/h` — Lógica regen con tablas
**Archivos a modificar:**
- `Core/Src/motor_control.c` — Integrar regen en pipeline de frenado
- `Core/Inc/safety_system.h` — Umbrales protección batería para regen

---

## 4. MAPA DE GPIO — NO MOVER NADA

### STM32G474RE — Pines asignados (en `Core/Inc/main.h`)
```
PA0  = Wheel FL (EXTI0)          PA8  = PWM FL (TIM1_CH1)
PA1  = Wheel FR (EXTI1)          PA9  = PWM FR (TIM1_CH2)
PA2  = Wheel RL (EXTI2)          PA10 = PWM RL (TIM1_CH3)
PA3  = Pedal ADC (ADC1_IN4)      PA11 = PWM RR (TIM1_CH4)
PA15 = Encoder A (TIM2_CH1)

PB0  = OneWire (DS18B20)         PB6  = I2C SCL
PB3  = Encoder B (TIM2_CH2)      PB7  = I2C SDA
PB4  = Encoder Z (EXTI4)         PB8  = CAN RX (FDCAN1)
PB5  = Steer Center (EXTI5)      PB9  = CAN TX (FDCAN1)
PB10 = Relé LED                  PB15 = Wheel RR (EXTI15)

PC0-4   = DIR (FL/FR/RL/RR/Steer)
PC5-7,9 = EN (FL/FR/RL/Steer)    PC8  = PWM Steer (TIM8_CH3)
PC10-12 = Relés (Main/Trac/Dir)   PC13 = EN RR
```

### ESP32-S3 — Pines asignados (en `platformio.ini` y módulos)
```
GPIO 4  = CAN TX (TJA1051)       GPIO 5  = CAN RX (TJA1051)
GPIO 6  = HC-SR04 TRIG            GPIO 7  = HC-SR04 ECHO
GPIO 13 = TFT MOSI (SPI)          GPIO 14 = TFT SCLK (SPI)
GPIO 15 = TFT CS                   GPIO 16 = TFT DC
GPIO 17 = TFT RST                  GPIO 21 = Touch CS (XPT2046)
GPIO 38 = WS2812B data (FastLED)   GPIO 40 = Ignition key sense
GPIO 41 = Power hold output        GPIO 42 = TFT backlight
GPIO 43 = DFPlayer TX (UART2)      GPIO 44 = DFPlayer RX (UART2)
```
**Libres para MCP23017 I2C:** GPIO 1/2 o GPIO 8/9 u otro par disponible.

---

## 5. PROTOCOLO CAN — NO CAMBIAR IDs EXISTENTES

### ESP32 → STM32
| ID | Nombre | DLC | Tasa | Descripción |
|----|--------|-----|------|-------------|
| 0x011 | HEARTBEAT_ESP32 | 1 | 100ms | Alive counter |
| 0x100 | CMD_THROTTLE | 1 | 50ms | Porcentaje acelerador (0-100) |
| 0x101 | CMD_STEERING | 2 | 50ms | Ángulo dirección (int16 LE, 0.1°) |
| 0x102 | CMD_MODE | 2 | on-demand | byte0: flags (bit0=4x4, bit1=tank), byte1: gear (0-4) |
| 0x110 | SERVICE_CMD | 2 | on-demand | byte0: acción (0=disable, 1=enable, 0xFF=factory), byte1: module_id |
| 0x120 | CMD_LED | 1 | on-demand | byte0: 0=OFF 1=ON |
| 0x208 | OBSTACLE_DISTANCE | 5 | 66ms | byte0-1: dist_mm (u16 LE), byte2: zone, byte3: health, byte4: counter |
| 0x209 | OBSTACLE_SAFETY | 8 | 100ms | Informacional (reservado) |

### STM32 → ESP32
| ID | Nombre | DLC | Tasa | Descripción |
|----|--------|-----|------|-------------|
| 0x001 | HEARTBEAT_STM32 | 5 | 100ms | byte0: counter, byte1: state, byte2: fault_flags, byte3: error_code, byte4: status_flags |
| 0x103 | CMD_ACK | 3 | on-demand | byte0: cmd_id_low, byte1: result, byte2: system_state |
| 0x200 | STATUS_SPEED | 8 | 100ms | 4× u16 LE (0.1 km/h) |
| 0x201 | STATUS_CURRENT | 8 | 100ms | 4× u16 LE (0.01 A) |
| 0x202 | STATUS_TEMP | 5 | 1000ms | 5× int8 (°C) |
| 0x203 | STATUS_SAFETY | 3 | 100ms | byte0: ABS, byte1: TCS, byte2: error_code |
| 0x204 | STATUS_STEERING | 3 | 100ms | byte0-1: angle (s16 LE, 0.1°), byte2: calibrated |
| 0x205 | STATUS_TRACTION | 4 | 100ms | 4× u8 (0-100%) wheel scale |
| 0x206 | STATUS_TEMP_MAP | 5 | 1000ms | 5× int8 (FL/FR/RL/RR/Amb °C) |
| 0x207 | STATUS_BATTERY | 4 | 100ms | byte0-1: current (u16 LE, 0.01A), byte2-3: voltage (u16 LE, 0.01V) |
| 0x20A | STATUS_LIGHTS | 2 | 1000ms | byte0: relay_on, byte1: reserved |
| 0x300 | DIAG_ERROR | 2-8 | on-demand | byte0: error/tag, byte1: subsystem |
| 0x301 | SERVICE_FAULTS | 4 | 1000ms | u32 LE bitmask |
| 0x302 | SERVICE_ENABLED | 4 | 1000ms | u32 LE bitmask |
| 0x303 | SERVICE_DISABLED | 4 | 1000ms | u32 LE bitmask |

---

## 6. ARQUITECTURA DE PANTALLAS ESP32

```
ScreenManager (screen_manager.cpp)
├── BootScreen     → state == BOOT
├── StandbyScreen  → state == STANDBY
├── DriveScreen    → state == ACTIVE | DEGRADED | LIMP_HOME
├── SafeScreen     → state == SAFE
└── ErrorScreen    → state == ERROR
```

**Frame rate:** 20 FPS (FrameLimiter en `ui/frame_limiter.h`)
**Partial redraw:** Cada screen guarda `prev_` y `cur_` valores, solo redibuja lo que cambió.
**UI Components (en `esp32/src/ui/`):** `car_renderer`, `pedal_bar`, `gear_display`, `battery_indicator`, `mode_icons`, `led_toggle`, `obstacle_sensor`, `steering_display`, `runtime_monitor`, `debug_overlay`

Para añadir una nueva pantalla (ej: engineering_screen):
1. Crear `screens/engineering_screen.cpp/h` heredando de `Screen` (ver `screens/screen.h`)
2. Añadir instancia en `screen_manager.h`
3. Añadir case en `screenForState()` o lógica especial de transición

---

## 7. PATRONES DE CÓDIGO — CÓMO HACER CAMBIOS SIN ROMPER

### Para añadir un nuevo módulo ESP32:
1. Crear `esp32/src/mi_modulo.cpp` y `esp32/src/mi_modulo.h` con namespace propio
2. `init()` se llama en `setup()` de `main.cpp`
3. `update()` se llama en `loop()` de `main.cpp`
4. Si necesita datos CAN → leer de `vehicleData` (singleton en main.cpp)
5. Si envía CAN → construir `CanFrame` y usar `ESP32Can.writeFrame(frame)`
6. Si necesita ACK → llamar `ackBeginWait(cmdIdLow)` después de enviar

### Para añadir un nuevo CAN ID:
1. **ESP32:** Añadir constante en `esp32/include/can_ids.h`
2. **STM32:** Añadir `#define` en `Core/Inc/can_handler.h`
3. **STM32 RX:** Añadir case en `CAN_ProcessMessages()` en `can_handler.c`
4. **STM32 RX filter:** Actualizar `CAN_ConfigureFilters()` si el ID no cae en un rango existente
5. **ESP32 RX:** Añadir decoder en `can_rx.cpp` + struct en `vehicle_data.h`

### Para añadir datos a VehicleData:
1. Crear struct en `vehicle_data.h` (igual que `LightsData`, `ObstacleData`, etc.)
2. Añadir miembro privado + setter + getter en clase `VehicleData`
3. Añadir decoder en `can_rx.cpp`

### Para modificar safety_system.c (STM32):
- **NUNCA** cambiar el comportamiento de `Safety_SetState()` sin entender las transiciones
- Los estados SAFE y ERROR son los más restrictivos — no bajar su nivel de protección
- `Obstacle_Update()` es no-bloqueante y usa timestamps, no `HAL_Delay()`
- Toda lógica de seguridad está en el loop de 10ms (`tick_10ms` en `main.c`)

---

## 8. CÓMO COMPILAR Y PROBAR

### STM32 (ARM cross-compile)
```bash
cd /home/runner/work/STM32-Control-Coche-Marcos/STM32-Control-Coche-Marcos
make clean && make all    # Necesita arm-none-eabi-gcc
```

### Tests unitarios existentes (host, no necesitan ARM)
```bash
cd Core/Src
# Math safety tests (54 tests)
gcc -std=c11 -I../Inc -O2 -lm math_safety.c test_math_safety.c -o /tmp/test_math && /tmp/test_math
# Steering cal store tests
gcc -std=c11 -I../Inc -O2 test_steering_cal_store.c -o /tmp/test_cal && /tmp/test_cal
```

### ESP32 (PlatformIO)
```bash
cd esp32
pio run                   # Compila
pio run -t upload         # Flashea
pio device monitor        # Monitor serial
```

---

## 9. REGLAS QUE EL SIGUIENTE AGENTE DEBE SEGUIR

1. **El documento de referencia único es `FIRMWARE_MIGRATION_AUDIT.md`** — contiene la comparación completa y el plan de implementación.
2. **No inventar funcionalidades.** Solo implementar lo que está documentado en la auditoría.
3. **No rediseñar.** Usar los patrones existentes (namespaces, VehicleData, CAN dispatch, screen system).
4. **No mover GPIOs.** Las asignaciones de pines están fijadas por hardware.
5. **No cambiar CAN IDs existentes.** Solo añadir nuevos si es necesario.
6. **Partial redraw siempre.** Las pantallas usan dirty-flags, nunca `fillScreen()` en update.
7. **Non-blocking siempre.** No usar `delay()` en `loop()`. Usar timestamps y máquinas de estado.
8. **STM32 es la autoridad de seguridad.** El ESP32 propone, el STM32 valida y aplica.
9. **Después de cada cambio, verificar que los tests existentes siguen pasando.**
10. **Actualizar `FIRMWARE_MIGRATION_AUDIT.md`** al completar cada paso (marcar como ✅ IMPLEMENTADO y actualizar porcentaje).

---

## 10. ORDEN RECOMENDADO DE IMPLEMENTACIÓN

```
Siguiente paso → Paso 1: MCP23017 shifter (es el más crítico, desbloquea marchas reales)
Después        → Paso 3: Touch completo (desbloquea modos 4x4/tank + base para menú)
Después        → Paso 4: 5 zonas obstáculo (STM32 only, sin dependencias)
Después        → Paso 5: Reacción infantil (depende de Paso 4)
Después        → Paso 8: Menú ingeniería (depende de Paso 3)
Después        → Paso 9: NVS persistencia (depende de Paso 8)
Después        → Paso 10: Error logger (STM32 flash + CAN command)
Últimos        → Paso 11: ACC, Paso 12: Regen (funcionalidad avanzada)
```

---

## 11. DEUDA TÉCNICA CONOCIDA (no bloquea, pero documentar)

1. **`led_controller.cpp` línea 40:** `braking` y `reverse` están hardcodeados a `false` en `main.cpp` línea 265-266. Se resolverá cuando MCP23017 (Paso 1) proporcione la marcha real y se detecte frenado.
2. **`drive_screen.cpp` línea 100:** `curGear_ = ui::Gear::N` hardcodeado. Se resolverá con MCP23017.
3. **`drive_screen.cpp` líneas 102-104:** `curMode_.is4x4` y `curMode_.isTankTurn` siempre `false`. Se resolverá cuando STM32 envíe los mode flags por CAN (o el ESP32 trackee su propio estado tras ACK de CMD_MODE).
4. **Power manager (`power_manager.cpp`):** Falta secuencia completa de relés AUX_POWER en la etapa STARTING. Actualmente salta directo de POWER_HOLD a STARTING sin activar relés del STM32.
5. **Filtro CAN STM32:** Filter 2 es RANGE 0x110–0x120 (acepta 17 IDs). IDs intermedios 0x111-0x11F se ignoran silenciosamente en `CAN_ProcessMessages()` → no es un bug, pero documentar si se añaden IDs en ese rango.
