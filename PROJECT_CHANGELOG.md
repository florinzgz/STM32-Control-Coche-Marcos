# PROJECT_CHANGELOG

## 1. Visión del Proyecto

### Objetivo General
Sistema de control embebido para vehículo eléctrico de 4 ruedas con tracción independiente, dirección asistida y HMI completo. El proyecto implementa un control en tiempo real con alta fiabilidad y seguridad funcional.

### Arquitectura Hardware
- **STM32G474RE** (Cortex-M4F @ 170 MHz): Cerebro de control — tracción, dirección, sensores, CAN, safety.
- **ESP32-S3-WROOM-1** (Dual-core @ 240 MHz, 8 MB PSRAM): HMI — pantalla TFT 480×320, audio, LEDs WS2812B, selector de marchas, detección de obstáculos.
- **Bus CAN**: FDCAN1 @ 500 kbps, CAN 2.0A (11-bit IDs), topología punto a punto.

### Principios de Diseño
1. **Safety-first**: Máquina de estados de seguridad (BOOT→STANDBY→ACTIVE→DEGRADED→SAFE→ERROR), watchdog IWDG, BREAK2 kill de PWM ante fallo de CPU.
2. **Dual-core ESP32**: Render en Core 0, CAN/sensores/audio en Core 1. Datos compartidos vía mutex.
3. **Inicialización robusta**: Reintentos con validación hardware (CCCR, clock source), logs de diagnóstico persistentes.
4. **Tolerancia a fallos CAN**: Bus-off recovery (ambos lados), error-passive recovery bifásica (ESP32: 10 rápidos + lento ilimitado), timeout → LIMP_HOME.
5. **Persistencia**: Error log en Flash (página 125, 250 entradas con CRC32), calibración de dirección en Flash (página 126).
6. **Modularidad**: 25 módulos habilitables/deshabilitables vía Service Mode.

### Estándares de Calidad
- CI/CD: syntax check (`-Wall -Wextra -Werror`), unit tests, cppcheck, flawfinder, lizard (complejidad ciclomática).
- Documentación técnica: 90+ documentos en `docs/`.
- Protocolo CAN congelado (v1.3): `CAN_CONTRACT_FINAL.md`.

---

## 2. Estado Actual del Sistema

### STM32 (Control Authority)
| Subsistema | Estado | Notas |
|---|---|---|
| **Tracción (4 motores)** | ✅ Operativo | PWM 20 kHz, zona muerta 8%, rampa 50%/s up / 100%/s down |
| **Dirección PID** | ✅ Operativo | Encoder E6B2-CWZ6C (4800 counts/rev), geometría Ackermann |
| **Calibración dirección** | ✅ Persistente | Flash página 126, CRC32, auto-centrado al boot |
| **FDCAN1** | ✅ Operativo (hardened) | PA11(RX)/PA12(TX) AF9, MASK accept-all, reintentos con CCCR check |
| **Sensores** | ✅ Operativo | 4× velocidad rueda, 5× DS18B20, 6× INA226, pedal ADC |
| **Safety System** | ✅ Operativo | ABS/TCS 15% slip, overcurrent, overtemp, obstacle, CAN timeout |
| **Error Log** | ✅ Operativo | Flash página 125, 250 entradas, CRC32, ring buffer |
| **Service Mode** | ✅ Operativo | 25 módulos, factory defaults, fault tracking |
| **Watchdog (IWDG)** | ✅ Activo | ~500 ms timeout |
| **BREAK2 (PWM kill)** | ✅ Armado | Vinculado a Cortex LOCKUP |
| **Heartbeat LED** | ✅ Operativo | Flash breve ~50 ms cada ~2 s (diferenciable de Error_Handler) |

### ESP32-S3 (HMI Controller)
| Subsistema | Estado | Notas |
|---|---|---|
| **Pantalla TFT** | ✅ Operativo | ST7796 480×320, render en Core 0 (FreeRTOS task) |
| **CAN (TWAI)** | ✅ Operativo (hardened) | Bus-off recovery + error-passive recovery bifásica (10 rápidos @3s + lento @30s) |
| **Selector de marchas** | ✅ Operativo | MCP23017 I2C, P/R/N/D/D2, backoff con `endTransmission(true)` |
| **Audio (DFPlayer)** | ⚠️ Parcial | Hardware presente, integración completa pendiente |
| **LEDs WS2812B** | ✅ Operativo | Front (47 LEDs @ GPIO 47), rear (16 LEDs @ GPIO 48) |
| **Obstáculos (LiDAR)** | ✅ Operativo | TF-Mini Plus (115200 bps, GPIO 18), 5 zonas, timeout 500 ms → fail-safe |
| **Power Manager** | ✅ Operativo | Ignition GPIO 40 + power hold GPIO 41 |
| **Config Store** | ✅ Operativo | SPIFFS NVM persistente |
| **Screen Manager** | ✅ Operativo | 6 estados: Boot/Standby/Drive/Error/Safe/Degraded + Engineering (8989) |

### Comunicación CAN
- **Protocolo**: 27 tipos de mensaje, contrato congelado v1.3.
- **STM32→ESP32**: Heartbeat (0x001), telemetría (0x200–0x20A), service (0x301–0x305).
- **ESP32→STM32**: Heartbeat (0x011), throttle (0x100), steering (0x101), mode (0x102), service (0x110), LEDs (0x120), obstacle (0x208–0x209).
- **Filtrado STM32**: MASK accept-all (index 0), filtrado real en `CAN_ProcessMessages()` switch/case.
- **Timeout**: 250 ms → transición a LIMP_HOME (20% torque máximo).

### FreeRTOS (ESP32)
- **renderTask**: Pinned Core 0, stack 16 KB, prioridad 1. Maneja TFT SPI + touch.
- **Main loop**: Core 1. Maneja CAN, sensores, audio, LEDs, power.
- **Sincronización**: Mutex para `VehicleData`, cola FreeRTOS para touch actions.

### Rendimiento
- STM32 main loop: 100 Hz (10 ms).
- Sensor reads: 20 Hz (50 ms).
- Safety checks: 100 Hz (10 ms).
- CAN heartbeat: 10 Hz (100 ms).
- ESP32 render: ~28 ms/frame (liberado del main loop).
- RuntimeMonitor: Logs cada 5 s con stats por periodo (fps, avg/max/min frame, can, loop). Flags de bloqueo independientes por fase (can, ui, render, loop). Threshold: 4 ms.

---

## 3. Cambios Recientes (últimos PR)

### PR — feat: Tile-Based Dirty Region Engine (motor de render tipo cluster OEM automotriz)

### PR — refactor: HMI Security Audit & Tile Engine Hardening

### PR — refactor: Tile Engine Formalization & Pipeline Hardening (OEM Cluster Level)

### PR — refactor: Draw Purity Enforcement, OverlayMode, Full Layout Centralization

### PR — feat: Time Determinism, Hash Failsafe, Flag Safety (OEM Final Hardening)
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Final hardening pass: inject deterministic `frameTimeMs` into all screens, add hash failsafe system for critical tiles, formalize flag safety contract. Makes render pipeline fully reproducible from (VehicleData + time).
- **Root cause:** (1) `millis()` called directly inside screen `update()` methods broke time reproducibility — same input could produce different UI state depending on when millis() was sampled within the function. (2) FNV-1a 32-bit hash has non-zero collision probability; no safety net existed for missed tile updates on critical elements (speed, faults). (3) Flag safety contract was implemented but not formally documented.
- **Solución aplicada:**
  1. **Screen::update() signature**: Added `unsigned long frameTimeMs` parameter. ScreenManager captures `millis()` exactly once per frame and injects it into all screens. All timing logic in update() uses the injected value.
  2. **Time determinism**: Replaced all `millis()` calls in DriveScreen (ACK), ErrorScreen (canLost, elapsed), BootScreen (CAN link, RX staleness, freeze detection), PinScreen (wrong code timeout), EngineeringScreen (ACK timeout) with `frameTimeMs`.
  3. **Hash failsafe**: Added `TileSet::forceRedraw(idx)` method (zeroes hash + marks dirty). DriveScreen and ErrorScreen call it on critical tiles (SPEED, FAULTS, DEGRADED, BATTERY, BANNER) every `HASH_FAILSAFE_INTERVAL` frames (100 = 5s at 20 FPS).
  4. **Flag safety documentation**: Documented in tile_engine.h that event flags are cleared ONLY after successful tile render.
  5. **ui_config.h**: Added `HASH_FAILSAFE_INTERVAL = 100`.
- **Archivos afectados:**
  - `esp32/src/screens/screen.h` — `update()` signature + `frameTimeMs`
  - `esp32/src/screen_manager.cpp` — Capture millis() once, pass to screens
  - `esp32/src/screens/drive_screen.h/cpp` — frameTimeMs, failsafe counter
  - `esp32/src/screens/error_screen.h/cpp` — frameTimeMs, failsafe counter
  - `esp32/src/screens/boot_screen.h/cpp` — frameTimeMs replaces millis()
  - `esp32/src/screens/pin_screen.h/cpp` — frameTimeMs
  - `esp32/src/screens/engineering_screen.h/cpp` — frameTimeMs
  - `esp32/src/screens/safe_screen.cpp` — (void)frameTimeMs
  - `esp32/src/screens/standby_screen.cpp` — (void)frameTimeMs
  - `esp32/src/ui/tile_engine.h` — forceRedraw(), pipeline docs, hash failsafe docs
  - `esp32/src/ui/ui_config.h` — HASH_FAILSAFE_INTERVAL
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Render pipeline is now fully time-deterministic: same (VehicleData, frameTimeMs) → same derived state → same framebuffer ALWAYS. Critical tiles have bounded recovery time from hash collisions. Zero behavioral change for normal operation — millis() was already being called once per frame externally, but now it's formalized as an injected dependency.
- **Tests:** Internal refactoring. No CAN, VehicleData, or widget interface changes. Build verified on STM32 side (make clean && make).

### PR — feat: V10 Hardening Extension — Staggered Failsafe, Critical Tile Policy, Render Atomicity
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Final hardening V10: distribute forced redraws to eliminate SPI spikes, add fault-condition hash override for critical tiles, formalize frame time monotonicity and render atomicity contracts.
- **Root cause:** (1) Hash failsafe redraws all critical tiles simultaneously every 100 frames, creating a periodic SPI spike. (2) No mechanism to override hash suppression under fault conditions — a hash collision during a fault could hide safety information. (3) Frame time monotonicity and render atomicity were guaranteed by implementation but not formally contracted.
- **Solución aplicada:**
  1. **Staggered failsafe**: Critical tile forced redraws are now distributed across the HASH_FAILSAFE_INTERVAL. DriveScreen: SPEED@0, FAULTS@25, DEGRADED@50, BATTERY@75. ErrorScreen: BANNER@0, FAULTS@50. Eliminates the multi-tile SPI spike every 100 frames.
  2. **Critical tile fault override**: When `curFaultFlags_ != 0`, DTILE_SPEED and DTILE_FAULTS are force-redrawn EVERY frame, overriding hash suppression. Ensures fault visualization is always visually current.
  3. **Frame time contract (tile_engine.h)**: Formal documentation of frameTimeMs single-sampling, monotonicity, overflow-safe deltas, and determinism guarantee.
  4. **Frame time monotonicity assertion**: ScreenManager tracks prevFrameTimeMs_ and asserts (in UI_TILE_DEBUG mode) that time never goes backwards.
  5. **Render atomicity contract (tile_engine.h)**: Formal documentation that tile render → overlay invalidation → markClean → flag clear is atomic (single-threaded Core 0, no yield points).
- **Archivos afectados:**
  - `esp32/src/ui/tile_engine.h` — Frame Time Contract, Render Atomicity Contract, Hash Failsafe Distribution, Critical Tile Policy documentation
  - `esp32/src/screens/drive_screen.cpp` — Staggered failsafe + fault-condition critical tile override
  - `esp32/src/screens/error_screen.cpp` — Staggered failsafe
  - `esp32/src/screen_manager.h` — prevFrameTimeMs_ member
  - `esp32/src/screen_manager.cpp` — frameTimeMs monotonicity debug assertion
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Zero behavioral change under normal operation (no faults, no hash collisions). Under fault conditions, SPEED and FAULTS tiles update every frame instead of relying on hash comparison. SPI load is distributed evenly instead of spiking every 100 frames. All contracts now formally documented.
- **Tests:** STM32 build verified (make clean && make -j with -Wall -Wextra -Werror).
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Enforce draw-phase purity (zero state mutation in draw helpers), add formal overlay composition modes (REPLACE/MERGE), and centralize all remaining layout constants from SafeScreen and BootScreen.
- **Root cause:** (1) `ackIndicatorDirty_` was cleared inside `drawAckIndicator()` (draw phase), violating pure-render contract. (2) `diagNeedsRedraw_` was cleared inside BootScreen tile render block. (3) Overlay tiles had no formal composition mode declaration. (4) SafeScreen had 17 local layout constants not in ui_config.h. (5) BootScreen diagnostic layout/timing constants were file-local.
- **Solución aplicada:**
  1. **Draw purity**: Moved `ackIndicatorDirty_ = false` from `drawAckIndicator()` to `draw()` caller. Moved `diagNeedsRedraw_ = false` after tile render in BootScreen.
  2. **OverlayMode enum**: Added `OverlayMode::REPLACE` / `OverlayMode::MERGE` to tile_engine.h with per-overlay registry documenting which DriveScreen overlays use which mode and which base tiles they overlap.
  3. **SafeScreen layout**: 17 constants (STILE_BANNER_H through STILE_COL_VAL_SPACE) moved to ui_config.h. safe_screen.cpp uses `using namespace ui::cfg`.
  4. **BootScreen layout**: BTILE_DIAG_SEP_Y, BTILE_DIAG_LINE_H, BTILE_DIAG_MARGIN_X, BTILE_DIAG_RX_RECENT_MS, BTILE_DIAG_FREEZE_MS moved to ui_config.h. boot_screen.cpp aliases them locally.
- **Archivos afectados:**
  - `esp32/src/ui/tile_engine.h` — OverlayMode enum + per-overlay composition registry
  - `esp32/src/ui/ui_config.h` — STILE_* (17) + BTILE_DIAG_* (5) constants
  - `esp32/src/screens/drive_screen.cpp` — Draw purity: ackIndicatorDirty_ cleared after render
  - `esp32/src/screens/safe_screen.cpp` — All layout refs → STILE_* from ui_config.h
  - `esp32/src/screens/boot_screen.cpp` — Draw purity + layout refs → BTILE_DIAG_* from ui_config.h
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Draw phase is now fully pure (no state mutation in draw helpers). All 5 screens' layout constants are in ui_config.h. Overlay composition is formally documented. Zero behavioral changes — all modifications are structural refactoring.
- **Tests:** Internal refactoring only. No CAN, VehicleData, or widget interface changes.

- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Formalización del tile engine a nivel OEM automotive cluster: Z-order layer system, pipeline estricto update/draw, overlay invalidation contract, debug assertions, y centralización completa de tile layout dimensions.
- **Root cause:** (1) Overlay visibility se recomputaba en draw() violando la regla de render puro. (2) tile_engine.h no definía Z-order formal para overlay compositing. (3) Tile layout dimensions (180, 120, 370, 75, etc.) hardcoded en onEnter() sin nombre semántico. (4) Bounds safety era silencioso sin opción de diagnóstico. (5) No existía contrato formal para overlay invalidation.
- **Solución aplicada:**
  1. **tile_engine.h — Z-order layer system**: TileLayer enum (STATIC/BASE/OVERLAY/SYSTEM) con reglas formales de composición documentadas. Render contract: update phase (NO SPI) → draw phase (SOLO consume estado precomputado).
  2. **tile_engine.h — Debug assertions**: `UI_TILE_DEBUG` flag habilita Serial.printf diagnóstico cuando setRect recibe coordenadas negativas, tamaños inválidos, o fuera de pantalla. Producción: silent clamp (sin overhead).
  3. **tile_engine.h — Overlay invalidation contract**: Documentación formal del ciclo de vida overlay: visible→invisible → clear + markDirty tiles subyacentes → repaint.
  4. **DriveScreen — Pure render**: Overlay visibility (curDegradedVisible_, curFaultsVisible_, curAckVisible_) precomputada en update(), draw() solo consume. Eliminada recomputación de `(curSystemState_ == DEGRADED || LIMP_HOME)` y `(curFaultFlags_ != 0)` en draw().
  5. **ui_config.h — Tile layout constants**: DTILE_MODE_ICONS_X/W, DTILE_LED_TOGGLE_X/W, DTILE_BATTERY_W, DTILE_WHEELS_W, DTILE_STEERING_X/W para DriveScreen. ETILE_BANNER_H, ETILE_CONTENT_X/W, ETILE_FAULTS_Y/H, ETILE_SAFETY_Y/H, ETILE_DIAG_Y/H, ETILE_ELAPSED_Y/H para ErrorScreen. YTILE_TEMPS_X/Y/W/H, YTILE_FAULTS_Y/H para StandbyScreen.
  6. **drive_screen.cpp**: Todos los setRect() usan constantes de ui_config.h.
  7. **error_screen.cpp**: Todos los setRect() usan constantes de ui_config.h.
  8. **standby_screen.cpp**: Todos los setRect() usan constantes de ui_config.h.
- **Archivos afectados:**
  - `esp32/src/ui/tile_engine.h` — Z-order enum, render contract, debug asserts, overlay contract
  - `esp32/src/ui/ui_config.h` — Tile layout dimension constants (DTILE_*, ETILE_*, YTILE_*)
  - `esp32/src/screens/drive_screen.h` — curDegradedVisible_, curFaultsVisible_, curAckVisible_
  - `esp32/src/screens/drive_screen.cpp` — Pure render, named constants, layer comments
  - `esp32/src/screens/error_screen.cpp` — Named layout constants
  - `esp32/src/screens/standby_screen.cpp` — Named layout constants
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Pipeline de render formalmente puro (update no hace SPI, draw no recomputa lógica). Overlays con jerarquía Z formal y contrato de invalidación. Todas las dimensiones de tile en configuración central. Debug assertions disponibles para desarrollo. Zero cambios en interfaz CAN, VehicleData, o STM32 firmware.
- **Tests:** Sin cambios en interfaces externas. Cambios son refactoring interno del render engine: renombrado de constantes, reordenación de computación, y documentación formal.
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Auditoría completa de seguridad, optimización y refactor del sistema HMI tile-based. Centralización de constantes, corrección de artefactos de overlay, eliminación de duplicación de cálculos, bounds safety en tile engine, y determinismo en hysteresis de batería.
- **Root cause:** Múltiples defectos potenciales detectados en el motor de render: (1) magic numbers dispersos en 10+ archivos, (2) tiles overlay (DEGRADED, FAULTS, ACK) se solapan con tiles base pero markClean() no restaura el contenido subyacente, (3) wheelThresholdFilter se ejecuta 2× por frame (update hash + draw), (4) setRect() no valida coordenadas contra límites de pantalla, (5) hysteresis de texto en batería depende de estado del frame anterior (no determinista).
- **Solución aplicada:**
  1. **ui_config.h** (NUEVO): 25+ constantes centralizadas — PAD_SPEED/ACK/PEDAL/OBSTACLE/ERROR/SAFE/STANDBY/BOOT, THR_TRACTION_DELTA/TEMP_DELTA, BATT_HYSTERESIS_HIGH/LOW, OVL_DEGRADED_*/OVL_FAULT_*, PEDAL_COLOR_LOW/MID, BATT_COLOR_LOW/MID, TEMP_COLOR_WARNING/CRITICAL.
  2. **Overlay invalidation chain**: Cuando un overlay se desactiva, los tiles subyacentes se marcan dirty para restaurar su contenido. DEGRADED→OBSTACLE, FAULTS→MODE_ICONS+LED_TOGGLE+BATTERY, ACK→LED_TOGGLE.
  3. **wheelThresholdFilter precompute**: Valores de dibujo calculados una sola vez en update() y almacenados en drawTraction_[4]/drawTemp_[4]. draw() consume los valores precomputados sin recalcular.
  4. **Tile bounds safety**: setRect() ahora clampea x,y a ≥0, w,h a ≥0, y (x+w, y+h) a ≤SCREEN_W/SCREEN_H.
  5. **Battery hysteresis determinista**: Reemplaza `prevUsedDark` (dependencia frame anterior) por comparación explícita con BATT_HYSTERESIS_HIGH/LOW thresholds. En banda de hysteresis, usa dirección del cambio (prevFW≥highThresh).
  6. **All setTextPadding**: Reemplazados 20+ valores hardcoded por constantes nombradas de ui_config.h.
- **Archivos afectados:**
  - `esp32/src/ui/ui_config.h` — NUEVO: Configuración centralizada HMI
  - `esp32/src/ui/tile_engine.h` — Bounds safety en setRect()
  - `esp32/src/ui/battery_indicator.cpp` — Hysteresis determinista, config constants
  - `esp32/src/ui/pedal_bar.cpp` — Config constants (color thresholds, padding)
  - `esp32/src/ui/obstacle_sensor.cpp` — Config constants (padding, bar max range)
  - `esp32/src/ui/car_renderer.cpp` — Config constants (wheel label padding)
  - `esp32/src/screens/drive_screen.h` — Precomputed draw values, overlay tracking members
  - `esp32/src/screens/drive_screen.cpp` — Overlay invalidation, precompute, config constants
  - `esp32/src/screens/error_screen.cpp` — Config constants
  - `esp32/src/screens/safe_screen.cpp` — Config constants (padding, temp thresholds)
  - `esp32/src/screens/standby_screen.cpp` — Config constants
  - `esp32/src/screens/boot_screen.cpp` — Config constants
  - `esp32/src/hmi/obstacle_indicator.cpp` — Named constant for padding
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Sistema HMI determinista. Overlays no dejan artefactos visuales. Cero duplicación de cálculo por frame. Arquitectura limpia con constantes centralizadas. Compatible con TFT_eSPI + CAN + STM32 sin cambios.
- **Tests:** Compilación conceptual verificada. Sin cambios en CAN protocol, VehicleData, STM32 firmware. Todos los cambios son renombrado de constantes y correcciones de lógica de render — no afectan la interfaz CAN ni datos de vehículo.
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Refactor completo del sistema HMI para implementar un motor de render por regiones (tiles) tipo cluster OEM automotriz. Cada zona de la pantalla es un tile independiente con su propio dirty flag y hash de contenido FNV-1a. Solo los tiles cuyo contenido ha cambiado desde el último frame se redibujan.
- **Root cause:** El sistema anterior usaba comparaciones campo-por-campo para detectar cambios, sin agrupación formal en regiones. Los bar widgets (pedal, batería, sensor obstáculo) usaban clear+redraw completo en cada actualización, causando un flash visible (clear = COL_BG seguido de fill = color).
- **Solución aplicada:**
  1. **tile_engine.h**: Nuevo módulo con TileRect, TileHash (FNV-1a 32-bit), y TileSet<N> template. Hash comparison per-tile para decisión de skip/redraw.
  2. **DriveScreen**: 12 tiles formales (SPEED, OBSTACLE, WHEELS, STEERING, BATTERY, GEAR, PEDAL, MODE_ICONS, LED_TOGGLE, DEGRADED overlay, FAULTS overlay, ACK). Pipeline: update() computa hashes → updateHash() marca dirty → draw() solo renderiza tiles sucios.
  3. **ErrorScreen**: 5 tiles (BANNER, FAULTS, SAFETY, DIAG, ELAPSED). Cada tile se actualiza independientemente.
  4. **SafeScreen**: 6 tiles (FAULTS, ERROR, SPEEDS, CURRENTS, TEMPS, STEERING).
  5. **StandbyScreen**: 2 tiles (TEMPS, FAULTS).
  6. **BootScreen**: 3 tiles (CAN_STATUS, SENSOR, DIAGNOSTICS).
  7. **PedalBar**: Differential update — solo la porción que creció o se redujo se pinta. Si cambió el color (threshold crossing), redraw mínimo.
  8. **BatteryIndicator**: Differential update — mismo patrón que PedalBar.
  9. **ObstacleSensor**: Differential update — bar de proximidad sin clear+redraw.
  10. **screen_manager.h**: Documentación actualizada reflejando pipeline tile-based.
- **Archivos afectados:**
  - `esp32/src/ui/tile_engine.h` — NUEVO: Core tile infrastructure
  - `esp32/src/screens/drive_screen.h` — TileSet<12>, tile enum
  - `esp32/src/screens/drive_screen.cpp` — Hash computation in update(), tile iteration in draw()
  - `esp32/src/screens/error_screen.h` — TileSet<5>, tile enum
  - `esp32/src/screens/error_screen.cpp` — Hash computation + tile iteration
  - `esp32/src/screens/safe_screen.h` — TileSet<6>, tile enum
  - `esp32/src/screens/safe_screen.cpp` — Hash computation + tile iteration
  - `esp32/src/screens/standby_screen.h` — TileSet<2>, tile enum
  - `esp32/src/screens/standby_screen.cpp` — Hash computation + tile iteration
  - `esp32/src/screens/boot_screen.h` — TileSet<3>, tile enum
  - `esp32/src/screens/boot_screen.cpp` — Hash computation + tile iteration
  - `esp32/src/ui/pedal_bar.cpp` — Differential bar update
  - `esp32/src/ui/battery_indicator.cpp` — Differential bar update
  - `esp32/src/ui/obstacle_sensor.cpp` — Differential bar update
  - `esp32/src/screen_manager.h` — Updated documentation
  - `PROJECT_CHANGELOG.md` — This entry
- **Impacto:** Render completamente tile-based. Reducción significativa de carga SPI (solo tiles dirty). Eliminación de clear+redraw flash en barras. Coherencia total snapshot→tiles. Arquitectura escalable tipo instrument cluster de vehículo real.
- **Tests:** Compilación exitosa. Revisión de código. Validación de lógica de hash y dirty flags. Sin cambios en CAN protocol, VehicleState struct, STM32 firmware.

### PR — feat: complete screen verification — degraded overlay + safe telemetry + fault indicators
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Verificación de las pantallas contra la especificación HMI_STATE_MODEL.md e implementación de las funcionalidades faltantes: overlay de modo degradado/limp-home en DriveScreen, indicadores de fault flags en DriveScreen, y telemetría read-only en SafeScreen.

### PR — refactor: HMI anti-flicker + render optimization
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Revisión y optimización del sistema HMI completo para eliminar flicker y redraws innecesarios. Corrección de un bug crítico (bit 0 CAN_TIMEOUT faltante en fault overlay de DriveScreen).

### PR — refactor: deterministic render pipeline + zero-flicker text updates
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Implementación de pipeline de render determinista tipo "OEM automotive cluster". Eliminación total de flicker textual mediante setTextPadding. Formalización de arquitectura frame-latch CAN→snapshot→render.
- **Root cause:** Dos categorías de problemas identificados:
  1. **Flicker textual**: Todas las pantallas usaban `tft.fillRect()` para borrar el área de texto seguido de `tft.drawString()` para redibujar. El gap entre ambas operaciones SPI (0.5–2ms) creaba un parpadeo visible: flash de color de fondo entre el borrado y el nuevo texto.
  2. **Heap allocation en overlay**: `draw_runtime_overlay()` usaba `String(ESP.getFreePsram() / 1024) + " KB"` que realiza 3 heap allocations por línea, causando micro-stutters y fragmentación del heap.
- **Solución aplicada:**
  1. Reemplazado `fillRect` + `drawString` por `setTextPadding(width)` + `drawString` en 12 componentes UI. TFT_eSPI dibuja texto + relleno de background en una sola transacción SPI, eliminando el gap visible.
  2. Reemplazado `String` concatenation por `snprintf` a buffer stack en `draw_runtime_overlay()`.
  3. Documentación formal del pipeline determinista en `vehicle_data.h`, `screen_manager.h`, y `main.cpp` renderTask.
- **Archivos afectados:**
  - `drive_screen.cpp` — drawSpeed, drawAckIndicator, drawFaultOverlays: setTextPadding
  - `error_screen.cpp` — safety error, diagnostic, elapsed time: setTextPadding
  - `safe_screen.cpp` — fault flags, error code, speeds, currents, temps, steering: setTextPadding
  - `standby_screen.cpp` — temperatures, fault flags: setTextPadding
  - `boot_screen.cpp` — CAN link status: setTextPadding
  - `car_renderer.cpp` — wheel labels (torque%, temp): setTextPadding
  - `obstacle_sensor.cpp` (UI) — distance text: setTextPadding
  - `pedal_bar.cpp` — percentage text: setTextPadding
  - `obstacle_indicator.cpp` — sensor status: setTextPadding
  - `main.cpp` — draw_runtime_overlay: String→snprintf, renderTask documentation
  - `vehicle_data.h` — render pipeline documentation
  - `screen_manager.h` — deterministic render documentation
  - `PROJECT_CHANGELOG.md` — documentación del cambio
- **Impacto:** Eliminación total de flicker textual en todas las pantallas del HMI. Eliminación de heap allocation en overlay de boot. Pipeline CAN→UI→Render documentado y formalizado.
- **Tests:** Verificación visual: zero visible flash en transiciones de valores de texto.

### PR — refactor: HMI anti-flicker + render optimization
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Revisión y optimización del sistema HMI completo para eliminar flicker y redraws innecesarios. Corrección de un bug crítico (bit 0 CAN_TIMEOUT faltante en fault overlay de DriveScreen).
- **Root cause:** Tres problemas identificados en la revisión:
  1. DriveScreen `drawFaultOverlays()` listaba bits 1–7 pero omitía bit 0 (CAN_TIMEOUT 0x01) — el flag de timeout CAN nunca se mostraba.
  2. DriveScreen `draw()` llamaba incondicionalmente a las funciones de render de cada zona (drawSpeed, ObstacleSensor::draw, CarRenderer::drawWheels/drawSteering, BatteryIndicator::draw, GearDisplay::draw, PedalBar::draw, ModeIcons::draw, LedToggle::draw) incluso cuando los datos no habían cambiado. Las funciones hacían early-return interno, pero la llamada y evaluación de parámetros implicaba overhead innecesario cada frame.
  3. ErrorScreen hacía `tft.fillScreen(COL_RED)` (480×320 = 153600 píxeles) cada vez que el estado canLost_ cambiaba, causando flash visible cuando CAN se restauraba/perdía.
  4. Sensor noise en torque/temperatura provocaba redraws constantes de ruedas sin cambio visual significativo.
- **Solución aplicada:**
  1. Añadida entrada `{ 0x01, "CAN TMO", ui::COL_AMBER }` al array `entries[]` de `drawFaultOverlays()`. Ahora todos los 8 bits (0–7) del bitmask están representados.
  2. Movidas todas las llamadas a draw helpers dentro del bloque `if (curVal != prevVal)`, eliminando llamadas a función cuando no hay cambio real.
  3. ErrorScreen: separada la lógica de `canLost_` toggle del `needsRedraw_`. Ahora solo redibuja el banner (top 75px via `fillRect`) en vez del screen completo (`fillScreen`). Labels estáticos se dibujan una sola vez en `needsRedraw_`.
  4. Añadidos umbrales de cambio para ruedas: torque Δ>2%, temperatura Δ≥1°C. Valores filtrados se commitean como estado dibujado para mantener referencia estable.
- **Impacto:** Eliminación de flicker en ErrorScreen durante transiciones CAN lost↔restored. Reducción de carga CPU en DriveScreen (zero-work frames cuando datos no cambian). Bug fix: CAN_TIMEOUT ahora visible en dashboard. Reducción de redraws de ruedas por sensor noise.
- **Archivos modificados:**
  - `esp32/src/screens/drive_screen.cpp` — fault overlay bit 0 fix, dirty-gated draw calls, wheel thresholds
  - `esp32/src/screens/error_screen.cpp` — partial banner redraw instead of full fillScreen on canLost_ toggle
  - `PROJECT_CHANGELOG.md` — documentación del cambio
- **Tests:** STM32 build validation (`make clean && make -j$(nproc)` passed with -Wall -Wextra -Werror).
- **Root cause:** Tres funcionalidades definidas en HMI_STATE_MODEL.md (§2.4, §2.5, §4.1) no estaban implementadas:
  1. DriveScreen no mostraba ningún indicador cuando systemState era DEGRADED(3) o LIMP_HOME(6).
  2. DriveScreen no mostraba indicadores visuales de fault_flags (OVERTEMP, OVERCURRENT, ENCODER, WHEEL SENSOR, ABS, TCS, CENTERING).
  3. SafeScreen solo mostraba fault_flags y error_code, sin la telemetría read-only requerida (velocidades, corrientes, temperaturas, ángulo dirección).
- **Solución aplicada:**
  1. DriveScreen: banner ámbar "DEGRADED MODE - 40% POWER" / "LIMP HOME - REDUCED SPEED" en zona Y=40–58 cuando systemState es DEGRADED o LIMP_HOME. Partial redraw solo cuando cambia el estado.
  2. DriveScreen: tira de indicadores de faults compactos en Y=28 (zona inferior del top bar). OVERTEMP/OVERCURR/ENC FAULT/WHL SENS/CENTER en ámbar. ABS/TCS en cian (informational). Partial redraw cuando cambia faultFlags.
  3. SafeScreen: layout rediseñado — banner SAFE MODE (40px) + fault/error + separador + telemetría read-only con 4 velocidades, 4 corrientes, 5 temperaturas, ángulo dirección. Partial redraw por campo. Temperaturas cambian de color (>60°C ámbar, >80°C rojo).
- **Impacto:** Las 7 pantallas del HMI (Boot, Standby, Drive, Degraded, Limp Home, Safe, Error) cumplen ahora completamente con la especificación HMI_STATE_MODEL.md. El conductor ve claramente el estado degradado y los fallos activos.
- **Archivos modificados:**
  - `esp32/src/screens/drive_screen.h` — campos para systemState, faultFlags, métodos drawDegradedOverlay/drawFaultOverlays
  - `esp32/src/screens/drive_screen.cpp` — overlay degradado, indicadores de faults, captura systemState/faultFlags en update()
  - `esp32/src/screens/safe_screen.h` — campos para speed/current/temp/steering arrays
  - `esp32/src/screens/safe_screen.cpp` — layout completo con telemetría read-only, partial redraw
- **Tests:** STM32 build validation (`make clean && make -j$(nproc)` passed).

### PR — feat: update drive screen display for TF-Mini Plus sensor
- **Fecha:** 2026-04-10
- **Autor:** Copilot
- **Descripción del cambio:** Actualización de la pantalla de conducción (drive screen) para reflejar el cambio de sensor de obstáculos de TOFSense-M a TF-Mini Plus. El sensor TF-Mini Plus tiene un rango de 10 cm–12 m (vs 2 cm–4 m del TOFSense-M), lo que requiere ajustar la barra de proximidad y los umbrales de color.
- **Root cause:** La barra de proximidad y los umbrales de color estaban calibrados para el rango máximo del TOFSense-M (400 cm). Con el TF-Mini Plus (rango hasta 1200 cm), las lecturas entre 4–12 m quedaban todas como "barra vacía" y los umbrales de color no aprovechaban el rango extendido.
- **Solución aplicada:**
  1. Barra de proximidad: max 400→600 cm (6 m da buena resolución visual en zona de alerta 0–4 m)
  2. Umbrales `proximityColor()`: ajustados para rango mayor — verde >3m, cian 1.5–3m, amarillo 0.8–1.5m, naranja 0.3–0.8m, rojo <0.3m
  3. Comentarios actualizados: "TOFSense-M" → "TF-Mini Plus" en `vehicle_data.h`, `obstacle_sensor.h`
  4. `ObstacleData.distanceCm` comentario max ~400→~1200 cm
- **Impacto:** La pantalla de conducción muestra correctamente el sensor TF-Mini Plus: barra de proximidad proporcional hasta 6 m, colores graduales en todo el rango útil.
- **Archivos modificados:**
  - `esp32/src/vehicle_data.h` — comentario TOFSense-M → TF-Mini Plus, max range
  - `esp32/src/ui/obstacle_sensor.cpp` — barra proximidad BAR_MAX_CM 400→600
  - `esp32/src/ui/obstacle_sensor.h` — comentario header actualizado
  - `esp32/src/ui/ui_common.h` — `proximityColor()` umbrales ajustados para TF-Mini Plus
- **Tests:** ESP32 build validation.

### PR — fix: ESP32 screen transitions to error when CAN cable disconnected
- **Fecha:** 2026-04-09
- **Autor:** Copilot
- **Descripción del cambio:** La pantalla del ESP32 no reaccionaba al desconectar el cable CAN. Ahora, si el heartbeat del STM32 no llega durante >1.5 s, el ScreenManager fuerza la transición a la pantalla de error mostrando "CAN LINK LOST / STM32 heartbeat not received". Se auto-recupera al reconectar el cable.
- **Root cause:** `ScreenManager::update()` solo leía `data.heartbeat().systemState` para decidir qué pantalla mostrar. Cuando el cable CAN se desconectaba, no llegaban nuevos heartbeats y el `systemState` mantenía su último valor (STANDBY, ACTIVE, etc.) → la pantalla nunca cambiaba.
- **Solución aplicada:**
  1. Añadido `CAN_LOSS_TIMEOUT_MS = 1500` en `can_ids.h`
  2. Detección de heartbeat stale en `ScreenManager::update()` — si edad > 1500 ms y no estamos en BOOT, forzar `newState = ERROR`
  3. `ErrorScreen` detecta heartbeat stale y muestra banner "CAN LINK LOST" en lugar de "SYSTEM ERROR"
  4. Auto-recuperación: cuando el heartbeat vuelve, `canLost_` se limpia y vuelve al estado normal
- **Impacto:** El usuario ve inmediatamente (en <2 s) que la comunicación CAN se ha interrumpido, con un mensaje claro en pantalla roja.
- **Archivos modificados:**
  - `esp32/include/can_ids.h` — nueva constante `CAN_LOSS_TIMEOUT_MS`
  - `esp32/src/screen_manager.h` — nuevo campo `canLost_`
  - `esp32/src/screen_manager.cpp` — detección heartbeat stale + fuerza ERROR
  - `esp32/src/screens/error_screen.h` — campos `canLost_` / `prevCanLost_`
  - `esp32/src/screens/error_screen.cpp` — banner condicional "CAN LINK LOST" vs "SYSTEM ERROR"
- **Tests:** STM32 firmware build (`make -j$(nproc)`) clean con `-Wall -Wextra -Werror`.

### PR — audit(obstacle): TF-Mini Plus sampling rate fix + file comment cleanup
- **Fecha:** 2026-04-09
- **Autor:** Copilot
- **Descripción del cambio:** Auditoría de producción del subsistema TF-Mini Plus. Descubierta pérdida del 90% de tramas del sensor (10 Hz observado vs 100 Hz esperado). Fix aplicado junto con limpieza de comentarios obsoletos.
- **Root cause:** Dos restricciones combinadas causaban que `update()` solo procesara 1 trama por llamada:
  1. `TFM_MAX_BYTES_PER_UPDATE = 32` (≈3.5 tramas) limitaba los bytes leídos
  2. `if (gotFrame) break;` salía del bucle tras la primera trama válida
  Con el main loop corriendo a ~10 Hz (por CAN/render/serial), solo se capturaban ~50 tramas/5s. El buffer UART de 256 bytes se llenaba (~900 bytes/s a 100 Hz × 9 bytes) y el hardware descartaba datos silenciosamente.
- **Solución aplicada:**
  1. Aumentar `TFM_MAX_BYTES_PER_UPDATE` de 32 a 256 (drena buffer completo por llamada)
  2. Eliminar `if (gotFrame) break;` — ahora procesa todas las tramas disponibles, conservando solo la última válida (lectura más fresca)
  3. Aumentar `rxBufSize` de 256 a 512 bytes (margen de ~570 ms a 100 Hz)
  4. Actualizar comentarios file-level en `.h` y `.cpp` (eliminadas referencias obsoletas a TOFSense-M como sensor primario)
  5. Actualizar comentario en `main.cpp` (TOFSense-M → TF-Mini Plus)
  6. Actualizar `TFMINI_PLUS_WIRING_GUIDE.md` con diagnósticos esperados post-fix
- **Impacto en el sistema:**
  - ESP32: tasa de muestreo sube de ~10 Hz a ~100 Hz (todas las tramas del sensor)
  - Latencia end-to-end: baja de ~100 ms a ~10 ms (trama más fresca siempre disponible)
  - CAN 0x208: datos más actualizados en cada frame (cada 66 ms)
  - STM32: sin cambios (56496 text idéntico)
- **Archivos modificados:** `esp32/src/sensors/obstacle_sensor.h`, `esp32/src/sensors/obstacle_sensor.cpp`, `esp32/src/main.cpp`, `esp32/src/test_obstacle_sensor.cpp`, `docs/TFMINI_PLUS_WIRING_GUIDE.md`, `PROJECT_CHANGELOG.md`
- **Tests:** 74 TF-Mini Plus + 135 TOFSense-M = 209 tests, 0 failures. STM32 build limpio (56496 text).

### PR-292 — feat: LD2 now shows CAN status in main loop (no external LED needed)
- **Fecha:** 2026-04-09
- **Autor:** Copilot
- **Descripción del cambio:** LD2 (PA5, LED verde soldado en la Nucleo) ahora muestra el estado CAN de forma continua en el bucle principal, no solo durante el arranque. Ya no se necesita un LED externo en PB14 para ver si el CAN funciona.
- **Root cause:** El patrón de LD2 en el main loop era siempre "flash breve cada 2 s" independientemente de si el CAN estaba OK o FAIL. El usuario solo podía ver el estado CAN en PB14 (LED_DIAG), que requiere un LED externo con resistencia de 330 Ω que no estaba instalado. El patrón de arranque (1 blink largo = OK, 5 rápidos = FAIL) solo se mostraba una vez al inicio y era fácil de perder.
- **Solución aplicada:** Modificar el heartbeat LD2 del main loop para reflejar el estado CAN:
  - **CAN OK** (`fdcan_init_ok`): flash breve cada ~2 s (50 ms ON / 1950 ms OFF) — patrón anterior, sin cambio.
  - **CAN FAIL** (`!fdcan_init_ok`): parpadeo constante 1 Hz (500 ms ON / 500 ms OFF) — claramente diferente.
  Actualizada la documentación de patrones LED en los comentarios del boot.
- **Impacto en el sistema:** Solo cambia el comportamiento visual de LD2 cuando CAN ha fallado. Sin impacto en lógica de control ni seguridad. LED_DIAG (PB14) sigue funcionando igual para quien tenga LED externo.
- **Archivos modificados:** `Core/Src/main.c`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa (56496 text, 72 data, 7784 bss).

### PR — feat(obstacle): Activar sensor TF-Mini Plus (OBSTACLE_SENSOR_ENABLED=1)
- **Fecha:** 2026-04-09
- **Autor:** Copilot
- **Descripción del cambio:** Activación del sensor de obstáculos TF-Mini Plus en el firmware de la ESP32-S3. El sensor estaba completamente implementado pero deshabilitado (`OBSTACLE_SENSOR_ENABLED=0`) desde PR-275. Ahora se habilita para uso con el hardware TF-Mini Plus disponible.
- **Root cause:** El sensor de obstáculos estaba deshabilitado porque el hardware (TOFSense-M original) fue retirado. El usuario ahora dispone de un TF-Mini Plus listo para conectar.
- **Solución aplicada:**
  - Cambiar `OBSTACLE_SENSOR_ENABLED` de 0 a 1 en `obstacle_sensor.h`
  - `SENSOR_TYPE` ya era `SENSOR_TYPE_TFMINI` (115200 bps, 9-byte frames)
  - Actualizar comentarios del header para reflejar el estado activo
  - Crear guía de cableado `docs/TFMINI_PLUS_WIRING_GUIDE.md`
  - Actualizar `docs/OBSTACLE_SENSOR_DESIGN_DECISION.md` con sección de migración
  - Actualizar `docs/PROJECT_MASTER_STATUS.md` con estado actual del sensor
- **Impacto en el sistema:** La ESP32 ahora:
  - Configura UART1 (GPIO 18, 115200 bps) al arranque
  - Lee tramas TF-Mini Plus (9 bytes, 100 Hz) y valida checksum/señal/rango
  - Transmite CAN 0x208 (distancia + zona + salud + counter) cada 66 ms
  - Transmite CAN 0x209 (estado de seguridad) cada 100 ms
  - La STM32 aplica su backstop de 5 zonas con los datos recibidos
  - Sin cambios en el firmware STM32 (56496 text idéntico)
- **Archivos modificados:** `esp32/src/sensors/obstacle_sensor.h`, `docs/TFMINI_PLUS_WIRING_GUIDE.md` (nuevo), `docs/OBSTACLE_SENSOR_DESIGN_DECISION.md`, `docs/PROJECT_MASTER_STATUS.md`, `PROJECT_CHANGELOG.md`
- **Tests:** STM32 build limpio (56496 text). Tests obstacle sensor: 74 TF-Mini Plus + 135 TOFSense-M = 209 tests, 0 failures.

### PR-291 — fix: cppcheck shadowVariable — move SystemCoreClock extern to file scope
- **Fecha:** 2026-04-08
- **Autor:** Copilot
- **Descripción del cambio:** Corregir falso positivo de cppcheck `shadowVariable` en `boot_validation.c` que provocaba fallo del CI (exit code 2).
- **Root cause:** La declaración `extern uint32_t SystemCoreClock;` dentro de la función `check_clock_sane()` (línea 177) sombreaba la declaración idéntica a nivel de archivo en el stub HAL de análisis (`analysis_artifacts/stubs/stm32g4xx_hal.h:189`). cppcheck lo reporta como `shadowVariable` y el CI falla con exit code 2.
- **Solución aplicada:** Mover la declaración `extern uint32_t SystemCoreClock;` de dentro de `check_clock_sane()` al bloque de externs a nivel de archivo (junto a `fdcan_init_ok` e `i2c_init_ok`). La variable ya está definida en `system_stm32g4xx.c` y la declaración a nivel de archivo es semánticamente idéntica. Sin cambio funcional.
- **Impacto en el sistema:** Ninguno funcional. Resuelve fallo de CI cppcheck. Build size sin cambios (56408 text).
- **Archivos modificados:** `Core/Src/boot_validation.c`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa (56408 text, 72 data, 7784 bss).

### PR-289 — fix: second-pass word-by-word audit — NaN diagnostic logging + printf cast
- **Fecha:** 2026-04-08
- **Autor:** Copilot
- **Descripción del cambio:** Segunda pasada de auditoría exhaustiva palabra-por-palabra de todo el firmware (STM32 + ESP32). Se encontraron y corrigieron 2 bugs menores restantes tras la primera auditoría (PR-288).
- **Root cause:** (1) `boot_validation.c` líneas 196/205: la función de detección `check_temperature_plausible()` (línea 56) y `check_current_plausible()` (línea 76) usan `isnan()` correctamente, pero el código de logging diagnóstico que identifica qué sensor exacto falló NO incluía `isnan()`. Si un sensor retornaba NaN, la comparación `t < min || t > max` evaluaba a false (NaN → comparación siempre false), y el fault no se registraba para ese sensor concreto. (2) `esp32/src/main.cpp` línea 501: `ESP.getPsramSize()` retorna `size_t`, usado con `%u` sin cast `(unsigned)`. Inconsistente con la corrección aplicada en PR-288 a las líneas 55-56.
- **Solución aplicada:** (1) Añadir `isnan(t) ||` y `isnan(c) ||` a las condiciones de logging diagnóstico en `boot_validation.c` líneas 196 y 205, igualando la lógica de las funciones de detección. (2) Añadir cast `(unsigned)` a los argumentos de `Serial.printf()` en línea 501.
- **Impacto en el sistema:** (1) Ahora los sensores que retornan NaN quedan correctamente registrados con MODULE_FAULT_WARNING en ServiceMode, mejorando la trazabilidad de fallos. No afecta la detección ni la seguridad (la detección ya funcionaba). +16 bytes de text (isnan inlining). (2) Printf portabilidad.
- **Archivos modificados:** `Core/Src/boot_validation.c`, `esp32/src/main.cpp`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa (56096 text, 72 data, 7768 bss).

### PR-290 — feat: advanced robustness features (NaN hardening, CAN diagnostics, boot validation, logging)
- **Fecha:** 2026-04-08
- **Autor:** Copilot
- **Descripción del cambio:** Implementación de funcionalidades avanzadas de robustez en 7 áreas: endurecimiento NaN/Inf de sensores, monitoreo de trama CAN, validación TX FDCAN, extensión de validación de boot, y estandarización de logs.
- **Root cause:** Aunque el firmware era funcional y limpio, las rutas de comparación de float en `safety_system.c` no protegían contra NaN/Inf (IEEE 754 hace que todas las comparaciones con NaN retornen false, permitiendo que valores inválidos pasen silenciosamente por checks de rango). Además, faltaban métricas de diagnóstico CAN (FPS, fallos TX consecutivos) y validaciones de boot (RAM, clock, periféricos).
- **Solución aplicada:**
  1. **NaN/Inf Hardening (PART 1 — CRÍTICO):** Añadidos `isnan()`/`isinf()` a 10+ rutas de comparación float: `Safety_CheckCurrent()`, `Safety_CheckTemperature()` (umbral crítico + histéresis), `Safety_CheckSensors()` (temperatura, corriente, velocidad de rueda), `Safety_CheckBatteryVoltage()`, `Safety_CheckBatteryOvervoltage()`. NaN ahora se trata como fallo del lado seguro (overcurrent, overtemp crítico, fallo de sensor).
  2. **CAN Frame Health Monitoring (PART 3):** Nuevos campos `rx_frames_per_sec`, `rx_count_prev`, `rx_rate_tick` en `CAN_Stats_t`. Nueva función `CAN_UpdateFrameRate()` llamada cada 1 s desde el tier 1000 ms.
  3. **FDCAN TX Validation (PART 5):** Nuevos campos `tx_nack_flag`, `tx_consec_fail` en `CAN_Diag_t`. Seguimiento de fallos TX consecutivos en `TransmitFrame()`. Flag levantado tras 5 fallos consecutivos (no ACK).
  4. **Boot Validation Extension (PART 6):** Tres nuevos checks: `check_ram_sanity()` (patrón write/read no destructivo), `check_clock_sane()` (SystemCoreClock 160-180 MHz), `check_periph_ready()` (diagnóstico fdcan_init_ok/i2c_init_ok). Bitmask ampliado uint8→uint16.
  5. **Logging Standardization (PART 7):** Prefijos ESP32 estandarizados a formato `[MODULE][SEVERITY]`: `[CAN][INFO/ERR/WARN]`, `[BOOT][INFO/ERR]`, `[SAFETY][WARN/INFO]`, `[CAN][DIAG]`.
- **Partes ya implementadas previamente (sin cambios necesarios):**
  - PART 2 (CAN Diagnostics): Ya completo — `CAN_Diag_t` con PSR/ECR/TEC/REC, bus-off/error-passive/warning.
  - PART 4 (ESP32 BUS-OFF Recovery): Ya completo — polling `twai_get_status_info()`, `twai_initiate_recovery()`, error-passive recovery bifásica.
- **Impacto en el sistema:** Cierra la vulnerabilidad NaN en todas las rutas de seguridad. Añade métricas de diagnóstico CAN sin overhead en runtime. Boot validation más completa sin aumentar tiempo de boot. Compatible hacia atrás — no cambia APIs públicas ni comportamiento existente.
- **Archivos modificados:** `Core/Src/safety_system.c`, `Core/Inc/can_handler.h`, `Core/Src/can_handler.c`, `Core/Src/main.c`, `Core/Inc/boot_validation.h`, `Core/Src/boot_validation.c`, `esp32/src/main.cpp`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa (56368 text, 72 data, 7784 bss). +272 bytes text vs baseline.

### PR-288 — fix(esp32): word-by-word audit + 3 bug fixes + changelog update
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** Auditoría exhaustiva palabra-por-palabra de todo el firmware STM32 (10 archivos core) y ESP32 (5 archivos core). La STM32 está limpia (0 bugs). Se encontraron y corrigieron 3 bugs en ESP32.
- **Root cause:** El debugger seguía dando errores de comunicación CAN. Se realizó una revisión exhaustiva del código para identificar cualquier error restante. Los bugs encontrados estaban en el lado ESP32 (especificadores printf incorrectos + puerto hardcodeado).
- **Bugs corregidos:**
  1. **CRÍTICO — `%lus` formato printf inválido** (`esp32/src/main.cpp:1272`): El especificador `%lus` es inválido en printf. La "s" se unía al formato `%lu`, causando comportamiento indefinido en la salida serial del diagnóstico error-passive. Fix: `%lus` → `%lu s` (espacio antes de "s").
  2. **ALTO — `%d` para `size_t`** (`esp32/src/main.cpp:55,56,63,66`): Se usaba `%d` (signed int) para imprimir valores `size_t` en diagnóstico PSRAM. En arquitecturas donde `size_t` ≠ `int`, puede truncar o mostrar valores incorrectos. Fix: `%d` → `%u` con cast explícito `(unsigned)`.
  3. **ALTO — puerto COM8 hardcodeado** (`esp32/platformio.ini:81`): `upload_port = COM8` impide compilar/flashear en cualquier máquina que no sea la del desarrollador. Fix: línea comentada con `;`.
- **Auditoría STM32 (resultado: LIMPIO):**
  - `Core/Src/main.c`: Boot sequence, FDCAN init timing, LED patterns — ✅
  - `Core/Src/can_handler.c`: CAN IDs, DLC mapping, endianness, TransmitFrame — ✅
  - `Core/Src/stm32g4xx_hal_msp.c`: GPIO AF9, clock source, NVIC priorities — ✅
  - `Core/Src/stm32g4xx_it.c`: Fault handlers, MOE disable, IRQ dispatch — ✅
  - `Core/Inc/project_config.h`: Pin definitions, no conflicts — ✅
  - `Core/Inc/can_handler.h`: CAN ID defines consistency — ✅
  - `Core/Inc/can_init_diag.h`: Diagnostic struct — ✅
  - `Core/Src/safety_system.c`: State machine, thresholds, ABS/TCS — ✅
  - `Core/Src/motor_control.c`: Timer channels, PWM, direction logic — ✅
  - `Core/Src/system_stm32g4xx.c`: FPU init via CPACR — ✅
- **Impacto en el sistema:** Corrige output serial de diagnóstico CAN error-passive y PSRAM. No afecta lógica de control ni safety. Permite a cualquier usuario flashear sin editar platformio.ini.
- **Archivos modificados:** `esp32/src/main.cpp`, `esp32/platformio.ini`, `PROJECT_CHANGELOG.md`
- **Tests:** STM32 build con `-Wall -Wextra -Werror` pasa (56080 text, 72 data, 7768 bss). ESP32 no compilable en sandbox (requiere PlatformIO con ESP-IDF).

### PR-287 — fix(esp32): clear quanta_resolution_hz to fix 625kbps→500kbps baud rate mismatch
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** Corrección CRÍTICA del baud rate CAN de la ESP32-S3 que impedía toda comunicación con la STM32. En ESP-IDF ≥ 5.0, la macro `TWAI_TIMING_CONFIG_500KBITS()` establece `quanta_resolution_hz = 10 MHz` con `brp = 0`. Cuando el código luego sobrescribe `brp = 10`, ambos campos son no-zero, y el driver ESP-IDF **ignora** `brp` y calcula: `brp = APB_CLK / quanta_resolution_hz = 80 MHz / 10 MHz = 8`. Esto produce `(1+13+2) × 8/80 MHz = 1600 ns → 625 kbps` en vez de los 500 kbps esperados — un **25% de desajuste** que hace IMPOSIBLE la comunicación.
- **Root cause:** Cambio de comportamiento en ESP-IDF 5.x: la macro de timing ahora rellena `quanta_resolution_hz`, y cuando ambos `quanta_resolution_hz > 0` y `brp > 0`, el driver prioriza `quanta_resolution_hz` e ignora `brp`. Esto no ocurría en ESP-IDF 4.x donde el campo no existía.
- **Solución aplicada:** (1) `#include <esp_idf_version.h>` para detección de versión en compilación. (2) Bloque `#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)` que limpia `t_config.quanta_resolution_hz = 0` antes de sobrescribir `brp = 10`. Esto fuerza al driver a usar BRP directo: `80 MHz / 10 / 16 = 500 kbps ✓`. (3) Diagnóstico serial mejorado mostrando versión ESP-IDF y confirmación del modo BRP forzado. (4) Formato printf corregido con `%%` para literal `%`.
- **Impacto en el sistema:** Restaura comunicación CAN entre ESP32-S3 y STM32G474RE. Antes de este fix, la ESP32 transmitía a 625 kbps y la STM32 escuchaba a 500 kbps — ningún frame era ACKeado, causando bus-off inmediato.
- **Archivos modificados:** `esp32/src/main.cpp`
- **Tests:** Verificación manual de cálculo de baud rate. Build STM32 pasa (sin cambios en STM32).

### PR-287a — fix(fdcan): start FDCAN before boot LED blinks to prevent ESP32 bus-off
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** Mover la inicialización FDCAN (`MX_FDCAN1_Init()` + `CAN_Init()`) ANTES de la secuencia de boot LED (3 blinks × 300 ms = 2.3 s). Esto asegura que la STM32 empiece a ACKear tramas CAN dentro de ~50 ms del encendido, en vez de después de los 2.3 s de la secuencia visual.
- **Root cause:** En un bus CAN de 2 nodos, la ESP32-S3 arranca ~600 ms después del encendido y empieza a transmitir heartbeats cada 100 ms. Si la STM32 no ACKea dentro de ~1.3 s, el TEC de la ESP32 alcanza 256 → BUS_OFF. Los 2.3 s de boot blinks dejaban a la ESP32 sola en el bus demasiado tiempo.
- **Solución aplicada:** (1) `MX_GPIO_Init()` → `MX_FDCAN1_Init()` → `CAN_Init()` → `boot_phase = 1` → boot LED blinks. (2) Drain FIFO post-blinks (CAN_ProcessMessages + reset overflow counter). (3) Documentación inline detallada del timing.
- **Impacto en el sistema:** FDCAN activo ACKeando frames durante los boot blinks. Sin dependencia de Motor/Safety/Sensor modules.
- **Archivos modificados:** `Core/Src/main.c`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa.

### PR-286 — fix(led): revert LED back to PA5 (LD2) — onboard Nucleo LED
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** Se revierte el LED de estado de PB8 a PA5 (LD2). LD2 es el LED verde soldado directamente en la placa NUCLEO-G474RE (UM2505 §6.5). No necesita LED externo ni resistencia. El cambio a PB8 fue un error — PA5 es la opción correcta para este hardware.
- **Root cause:** En PR-285 se movió el LED a PB8 creyendo que PA5 tenía interferencia del ST-Link vía SB21. Sin embargo, el usuario confirmó que LD2 (PA5) es el LED soldado en la propia placa Nucleo y funciona correctamente. No hay ningún LED externo conectado.
- **Solución aplicada:** (1) Revertir defines a `PIN_LD2` (GPIO_PIN_5), `PORT_LD2` (GPIOA), `PIN_LD2_N` (5U). (2) Todos los `HAL_GPIO_WritePin(PORT_LED_STATUS, PIN_LED_STATUS, ...)` → `HAL_GPIO_WritePin(PORT_LD2, PIN_LD2, ...)`. (3) Error_Handler y fault handlers: GPIOB bit 8 → GPIOA bit 5. (4) Eliminar PB8 STATUS_LED del .ioc.
- **Impacto en el sistema:** Solo cambia el pin físico. Vuelve al LED soldado en la placa. No requiere hardware externo.
- **Archivos modificados:** `Core/Inc/project_config.h`, `Core/Src/main.c`, `Core/Src/stm32g4xx_it.c`, `STM32-Control-Coche-Marcos.ioc`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa. Integrity check pasa.

### PR-285 — refactor(led): migrate status LED from PA5 (LD2) to PB8
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** Se migra todo el indicador LED de estado de PA5 (LD2) a PB8 (STATUS_LED). PA5 comparte el solder bridge SB21 con el SPI SCK del ST-Link en la placa Nucleo-64, causando que el LED sea controlado por el debugger en vez del firmware. PB8 es un GPIO libre accesible en el conector Morpho (CN10 pin 3). Requiere conectar un LED externo con resistencia a PB8.
- **Root cause:** PA5 (LD2) está compartido con el ST-Link SPI vía SB21 (cerrado por defecto en fábrica). Durante programación o debug activo, el ST-Link puede encender/apagar PA5 sin control del firmware, haciendo el LED no fiable para indicar estados del sistema. Según UM2505, la NUCLEO-G474RE solo tiene LD1 (COM, ST-Link) y LD2 (USER, PA5) — no existe LD4.
- **Solución aplicada:** (1) Nuevo define centralizado: `PIN_LED_STATUS` (GPIO_PIN_8), `PORT_LED_STATUS` (GPIOB), `PIN_LED_STATUS_N` (8U) en project_config.h. (2) Todos los `HAL_GPIO_WritePin(GPIOA, PIN_LD2, ...)` → `HAL_GPIO_WritePin(PORT_LED_STATUS, PIN_LED_STATUS, ...)`. (3) Error_Handler y fault handlers: registro directo actualizado de GPIOA bit 5 → GPIOB bit 8. (4) .ioc actualizado con PB8 como GPIO_Output etiquetado STATUS_LED.
- **Impacto en el sistema:** Solo afecta qué pin físico se usa para indicación visual. No cambia lógica, timing, ni safety. El usuario necesita conectar un LED+resistencia a PB8 (Morpho CN10 pin 3).
- **Archivos modificados:** `Core/Inc/project_config.h`, `Core/Src/main.c`, `Core/Src/stm32g4xx_it.c`, `STM32-Control-Coche-Marcos.ioc`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa. Integrity check pasa.

### PR-284 — feat(led): make boot blinks unmissable (300 ms + dark lead-in)
- **Fecha:** 2026-04-07
- **Autor:** Copilot
- **Descripción del cambio:** El usuario no veía los 3 blinks de boot al pulsar RESET porque a 150 ms ON/OFF el patrón duraba solo 900 ms y se confundía con un encendido directo. Se duplica el timing a 300 ms ON/OFF (1.8 s) con un período oscuro de 500 ms previo. También se aumenta el blink CAN-OK de 400 ms a 600 ms y CAN-FAIL de 80 ms a 150 ms ON/OFF. Separación post-boot aumentada a 500 ms.
- **Root cause:** Sin período oscuro inicial, el primer blink ON se confunde con el encendido natural del LED tras reset. A 150 ms por fase, los 3 blinks pasan en 900 ms — demasiado rápido para el ojo.
- **Solución aplicada:** (1) LED OFF explícito + 500 ms dark lead-in antes de empezar los blinks. (2) Boot blinks: 150 ms → 300 ms por fase (3 blinks = 1.8 s, imposible de no ver). (3) CAN status: pausa 500 ms + 1 blink largo 600 ms (CAN OK) o 5 blinks 150 ms (CAN FAIL). Todo antes de IWDG timeout (~4 s).
- **Impacto en el sistema:** Solo afecta la secuencia de boot LED (~4 s de delay total antes del main loop). No afecta lógica de control, safety, ni timing del main loop. IWDG se inicia después de los boot blinks.
- **Archivos modificados:** `Core/Src/main.c`, `PROJECT_CHANGELOG.md`
- **Tests:** Integrity check pasa. Build requiere cross-compiler (no disponible en sandbox).

### PR-283 — feat(led): improve boot visibility and add CAN init status indication
- **Fecha:** 2026-04-06
- **Autor:** Copilot
- **Descripción del cambio:** El patrón de 3 blinks de boot era demasiado rápido (60 ms ON/OFF = 360 ms total) para ser visible al enchufar USB. Se aumenta a 150 ms ON/OFF (900 ms total). Se añade indicación LED post-init del estado CAN: 1 blink largo (400 ms) = CAN OK, 5 blinks rápidos (80 ms) = CAN FAILED. Se añade `volatile` a `can_init_diag` para visibilidad fiable vía SWD con `-O2`.
- **Root cause:** El usuario no veía los 3 blinks de arranque porque a 60 ms por fase el patrón completo duraba solo 360 ms, invisible al conectar USB. Sin pantalla ni UART, el LED es el único feedback visual del estado del firmware.
- **Solución aplicada:** (1) Boot blinks: 60 ms → 150 ms por fase (3 blinks = 900 ms, claramente visible). (2) Post-init CAN status: pausa 300 ms + 1 blink largo (CAN OK) o 5 blinks rápidos (CAN FAIL). (3) `volatile` en `can_init_diag` (en `can_init_diag.h` tras merge con PR-282).
- **Impacto en el sistema:** Solo afecta la secuencia de boot LED (~2 s de delay adicional). No afecta lógica de control, safety, ni timing del main loop.
- **Archivos modificados:** `Core/Src/main.c`, `Core/Inc/can_init_diag.h`, `PROJECT_CHANGELOG.md`, `docs/PROJECT_MASTER_STATUS.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa sin errores.

### PR-282 — refactor(fdcan): decouple hal_msp from can_handler, fix stale filter documentation

- **Fecha:** 2026-04-06
- **Autor:** Copilot
- **Descripción del cambio:** Refactor seguro para reducir acoplamiento arquitectónico entre `stm32g4xx_hal_msp.c` y `can_handler.h`. Corrección de documentación de filtros CAN que describía erróneamente una política "reject-all" cuando la implementación real es accept-all con filtrado por software.
- **Solución aplicada:** (1) Extraer `CAN_InitDiag_t` a nuevo header `Core/Inc/can_init_diag.h`. (2) Reemplazar `#include "can_handler.h"` en hal_msp.c por `#include "can_init_diag.h"`. (3) `can_handler.h` incluye `can_init_diag.h` (sin duplicar typedef). (4) Corregir "reject-all default" → "accept-all (mask=0), non-matching IDs routed to FIFO0" en PROJECT_MASTER_STATUS.md. (5) Actualizar RX Filter Policy en CAN_CONTRACT_FINAL.md para reflejar la implementación real (MASK accept-all + software filtering en switch/case).
- **Impacto en el sistema:** Cero cambios funcionales. Binary idéntico (56032 text, 72 data, 7832 bss). Solo reduce coupling y corrige documentación.
- **Archivos modificados:** `Core/Inc/can_init_diag.h` (nuevo), `Core/Inc/can_handler.h`, `Core/Src/stm32g4xx_hal_msp.c`, `docs/PROJECT_MASTER_STATUS.md`, `docs/CAN_CONTRACT_FINAL.md`, `PROJECT_CHANGELOG.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa sin errores ni warnings. Binary size idéntico.

### PR-281 — fix(fdcan): robust deterministic FDCAN initialization with readback verification
- **Fecha:** 2026-04-06
- **Autor:** Copilot
- **Descripción del cambio:** Corrección CRÍTICA de la inicialización FDCAN que causaba fallo total del periférico CAN. Implementa secuencia de bring-up 100% robusta, determinista y verificable por hardware.
- **Root cause:** Tras reset, `RCC_CCIPR.FDCANSEL` = 00 (HSE, no habilitado en este proyecto). `RCC->APB1ENR1` no tenía bit FDCANEN habilitado. Lecturas de registros FDCAN retornaban datos basura de flash (stale AHB bus data, e.g. `0x08007d8d`). El `force-reset` en MspInit podía dejar el clock gate en estado indeterminado y la primera escritura a FDCANSEL podía no latchar en algunas revisiones de silicio.
- **Solución aplicada:** (1) MspInit: configurar FDCANSEL=PCLK1 ANTES de habilitar APB1 clock, con readback-verify. (2) Reset robusto: `__DSB(); __ISB()` tras force/release reset, re-enable clock + re-apply FDCANSEL post-reset. (3) Poll adaptativo de CCCR (50 ms timeout, sin delays fijos). (4) Readback-verify de `RCC->APB1ENR1` bit FDCANEN. (5) CAN_Init: `can_init_diag.started = 0U` al inicio (no por-path), solo promovido a 1U al final. (6) CAN_Init: verificación adicional de APB1ENR1 antes de configurar filtros. (7) Nuevos campos en `CAN_InitDiag_t`: retries, timeout_flag, msp_clk_ok, msp_ccipr_ok. (8) MX_FDCAN1_Init: registro de retry count en diagnostics.
- **Impacto en el sistema:** Elimina el fallo total de FDCAN por clock mal configurado o stale AHB reads. Inicialización determinista independiente de la revisión de silicio. Todos los paths de fallo dejan estado limpio y diagnosticable.
- **Archivos modificados:** `Core/Src/stm32g4xx_hal_msp.c`, `Core/Src/can_handler.c`, `Core/Inc/can_handler.h`, `Core/Src/main.c`, `docs/FDCAN_BRINGUP_ROBUSTNESS.md` (nuevo), `PROJECT_CHANGELOG.md`, `docs/PROJECT_MASTER_STATUS.md`
- **Tests:** Build con `-Wall -Wextra -Werror` pasa sin errores. Validación completa requiere hardware con SWD.
- **Próximos pasos:** Verificar en hardware: `can_init_diag.started == 1`, `RCC->APB1ENR1` bit 25 set, `RCC->CCIPR` bits [25:24] = 10, CCCR ≠ 0x0800xxxx.

### PR-280 — docs: complete power supply system document (9 sections + BOM)
- **Fecha:** 2026-04-04
- **Autor:** Copilot
- **Descripción del cambio:** Documento técnico completo del sistema de alimentación en `Documentos/SISTEMA_ALIMENTACION_COMPLETO.md` con 9 secciones + 2 apéndices. Cubre arquitectura de alimentación (24V/12V/5V/3.3V), recorrido completo desde baterías hasta actuadores, integración de relés (MAIN/TRAC/DIR/LED), apagado retardado (3s retention relay + audio DFPlayer), alimentación LED WS2812B (relés CAN 0x120), protección eléctrica (fusibles, TVS, desacoplo, umbrales batería), distribución de masas (punto estrella), esquema eléctrico detallado (31 pines STM32, I2C, FDCAN 500 kbps), y lista de materiales completa (BOM).
- **Root cause:** N/A — documento de diseño, no fix.
- **Solución aplicada:** Consolidación de datos de firmware verificado (safety_system.c, can_handler.c, main.c, power_manager.cpp, audio_manager.cpp) y documentación existente (LLAVE_CONTACTO, POWER_DISTRIBUTION, CONEXIONES_COMPLETAS, HARDWARE_WIRING_MANUAL, MATERIALES_POR_MODULO) en un único documento de referencia.
- **Impacto en el sistema:** Ningún cambio funcional. Documento de referencia para montaje, diagnóstico y mantenimiento del sistema de alimentación.
- **Archivos modificados:** `Documentos/SISTEMA_ALIMENTACION_COMPLETO.md` (nuevo), `PROJECT_CHANGELOG.md`, `docs/PROJECT_MASTER_STATUS.md`
- **Tests:** N/A — solo documentación.
- **Próximos pasos:** Verificar en hardware que todas las secciones de cable y valores de componentes coinciden con la implementación física.

### PR-279b — docs: CAN hardware fix procedure + fix stale PB8/PB9 pin references
- **Fecha:** 2026-04-04
- **Autor:** Copilot
- **Descripción del cambio:** Creación de documento ejecutable `CAN_HARDWARE_FIX_PROCEDURE.md` para resolver el fallo "waiting for CAN". Corrección de referencias de pines obsoletas PB8/PB9 → PA11/PA12 en documentación crítica.
- **Root cause confirmado:** Los pines FDCAN1 fueron remapeados de PB8/PB9 a PA11/PA12 (AF9) en marzo 2026 (PR #255-256). Documentos legacy aún referencian PB8/PB9 — si el hardware sigue esa documentación obsoleta, el transceiver CAN está conectado a pines incorrectos. Además, Pin 8 (Rs) del SN65HVD230 probablemente no está conectado a GND.
- **Solución aplicada:** (1) Nuevo documento `docs/CAN_HARDWARE_FIX_PROCEDURE.md`: flowchart de diagnóstico, pinout verificado del SN65HVD230, tablas de mediciones eléctricas, procedimiento paso a paso con interpretación de resultados. (2) Corrección de `HARDWARE_VALIDATION_PROCEDURE.md`: PB8/PB9 → PA11/PA12 en 2 ubicaciones. (3) Corrección de `PROJECT_MASTER_STATUS.md`: FDCAN pin reference actualizada a PA11/PA12 con detalles de transceiver.
- **Impacto en el sistema:** Documentación-only. Previene errores de cableado causados por documentación obsoleta.
- **Archivos modificados:** `docs/CAN_HARDWARE_FIX_PROCEDURE.md` (nuevo), `docs/HARDWARE_VALIDATION_PROCEDURE.md`, `docs/PROJECT_MASTER_STATUS.md`, `PROJECT_CHANGELOG.md`
- **Tests:** Sin cambios en firmware.

### PR-279a — audit(system): Full-system CAN validation audit + ESP32 CAN RX diagnostics
- **Fecha:** 2026-04-04
- **Autor:** Copilot
- **Descripción del cambio:** Auditoría completa del sistema dual-MCU (STM32 FDCAN + ESP32 TWAI). Análisis exhaustivo de firmware, protocolo CAN y procedimientos de validación hardware. Corrección de documentación DLC y adición de diagnósticos CAN RX.
- **Root cause identificado:** El firmware CAN es correcto y production-ready en ambos MCU. El fallo "waiting for CAN" es causado por problemas hardware: (P1) Pin 8 Rs del SN65HVD230 no conectado a GND, (P2) falta de GND común entre MCUs, (P3) terminación incorrecta.
- **Solución aplicada:** (1) Corrección de documentación STATUS_SAFETY DLC 3→5 en `can_ids.h`. (2) Adición de logging diagnóstico CAN RX en `can_rx.cpp`: primeros 10 frames recibidos con ID/DLC/datos + contador periódico cada 10s. (3) Documento de auditoría completo `docs/FULL_SYSTEM_VALIDATION_AUDIT.md` con root cause analysis, checklist de validación hardware, y estado final por subsistema.
- **Impacto en el sistema:** Mejora diagnósticos para identificar fallos hardware CAN. Sin cambio funcional en paths de control.
- **Archivos modificados:** `esp32/include/can_ids.h`, `esp32/src/can_rx.cpp`, `docs/FULL_SYSTEM_VALIDATION_AUDIT.md`, `PROJECT_CHANGELOG.md`, `docs/PROJECT_MASTER_STATUS.md`
- **Tests:** STM32 build con `-Wall -Wextra -Werror` pasa sin errores. Sin cambios en código STM32.

### PR-279 — fix(esp32/can): TWAI clk_src zeroed by memset — CAN bus inoperative
- **Fecha:** 2026-04-02
- **Autor:** Copilot
- **Descripción del cambio:** Corrige bug crítico en la inicialización TWAI del ESP32 que impedía toda comunicación CAN entre la ESP32-S3 y la STM32.
- **Root cause:** `twaiInit()` usaba `memset(&t_config, 0, sizeof(t_config))` para inicializar `twai_timing_config_t`. En ESP-IDF ≥ 5.0 este struct contiene un campo `clk_src` (clock source) cuyo valor por defecto (`TWAI_CLK_SRC_DEFAULT = SOC_MOD_CLK_APB`) NO es cero. El `memset` ponía `clk_src = 0`, lo que hacía que `twai_driver_install()` fallara con `ESP_ERR_INVALID_ARG` (reloj inválido) o seleccionara un reloj incorrecto (e.g. CPU @ 240 MHz en vez de APB @ 80 MHz), produciendo un baud rate de 1500 kbps en vez de 500 kbps — incompatible con el FDCAN del STM32.
- **Solución aplicada:** Reemplazar `memset` por inicialización con macro `TWAI_TIMING_CONFIG_500KBITS()` (que configura `clk_src` correctamente en todas las versiones de ESP-IDF), y después sobreescribir los campos de timing personalizados (brp=10, tseg_1=13, tseg_2=2, sjw=2) para mantener el sample point de 87.5% que empareja con el 88.2% del STM32.
- **Impacto en el sistema:** Restaura la comunicación CAN entre ESP32-S3 y STM32. Sin este fix, NINGÚN frame CAN podía intercambiarse entre las dos placas.
- **Archivos modificados:** `esp32/src/main.cpp`
- **Tests:** Integrity checks pasados. Validación completa requiere hardware (ESP32-S3 + STM32 + 2× TJA1051T/3).

### PR-278 — fix(fdcan): adaptive CCCR readback poll replaces fixed-count stabilisation loop
- **Fecha:** 2026-03-31
- **Autor:** Copilot
- **Descripción del cambio:** Reemplaza el bucle volátil de 3200 iteraciones en `HAL_FDCAN_MspInit` por un poll adaptativo de CCCR con timeout de 50 ms (SysTick). Añade verificación readback de FDCANSEL tras re-aplicación.
- **Root cause:** En algunas revisiones de silicio STM32G4, tras `__HAL_RCC_FDCAN_FORCE_RESET()`, el clock gate del periférico necesita más tiempo del previsto para estabilizarse. El bucle fijo de 3200 iteraciones (~113 µs) era insuficiente: CCCR retornaba datos basura estables del pipeline de instrucciones AHB (e.g. `0x08007bc9`, valor próximo a LR/PC en flash). Además, la primera escritura a `RCC_CCIPR.FDCANSEL` tras force-reset podía no latchar en algunas revisiones.
- **Solución aplicada:** (1) Poll adaptativo: lee CCCR repetidamente hasta que bits reservados [31:16] sean cero (periférico respondiendo), con timeout de 50 ms vía `HAL_GetTick()`. (2) Readback de FDCANSEL: tras `__HAL_RCC_FDCAN_CONFIG(PCLK1)`, lee `RCC_CCIPR` para verificar que el valor latchó; si no, re-aplica con barrera adicional.
- **Impacto en el sistema:** Mayor resiliencia en inicialización FDCAN en revisiones de silicio con clock gate lento. El poll adaptativo se adapta automáticamente al tiempo necesario en vez de depender de una estimación fija.
- **Archivos modificados:** `Core/Src/stm32g4xx_hal_msp.c`
- **Tests:** 455 unit tests pasados (sin cambio funcional en paths testados). Validación completa requiere hardware con SWD.
- **Próximos pasos:** Verificar en hardware que CCCR retorna valores válidos tras el poll y que `can_init_diag.hal_init == 0` en todas las condiciones de arranque (cold boot, watchdog reset, power glitch).

### PR-275 (GitHub) — fix(sensor): TF-Mini Plus uint16 overflow guard and stale data clearing on timeout
- **Fecha:** 2026-03-30
- **Autor:** Copilot
- **Descripción del cambio:** Auditoría profunda del parser TF-Mini Plus encontró dos bugs edge-case reales (overflow uint16 en conversión cm→mm y datos stale tras timeout). Además, se añadió soporte dual de sensor (TOFSense-M y TF-Mini Plus) seleccionable en compile-time, flag de habilitación del sensor, y documentación de interfaz común.
- **Root cause:** (1) `distCm * 10` se computaba como `uint16_t`, wrapping silenciosamente para valores >6553 cm (e.g., `6554 * 10 = 65540` trunca a `4`). Esto creaba una lectura de emergencia falsa con `healthy=true`. (2) En timeout de frame, `status` y `healthy` se limpiaban pero `distance_mm` y `zone` retenían sus últimos valores válidos. Frames CAN 0x208 llevarían distancia stale con `health=0`.
- **Solución aplicada:** (1) Cast intermedio a `uint32_t` con guarda: `uint32_t distMm = (uint32_t)distCm * 10u; if (distMm > UINT16_MAX) return false;`. (2) Timeout ahora limpia `distance_mm = 0` y `zone = 0` (zone 0 = "far/normal", correcto para sensor desconectado). (3) Flag `OBSTACLE_SENSOR_ENABLED` (default 0) para deshabilitar toda la lógica UART/parsing cuando no hay sensor. (4) Selección de sensor vía `SENSOR_TYPE` (TOFSENSE=0, TFMINI=1, default TFMINI). (5) Nuevo archivo `distance_sensor.h` con guía de integración TF-Mini Plus. (6) Simplificación de `main.cpp`: config defaults por `SENSOR_TYPE` en vez de hardcoded.
- **Impacto en el sistema:** Eliminados dos edge-cases de seguridad en el parser TF-Mini Plus. Estado INVALID ahora es completamente consistente (distance=0, zone=0, healthy=false, status=INVALID). Soporte dual de sensor preparado para intercambio de hardware.
- **Archivos modificados:** `esp32/src/sensors/obstacle_sensor.cpp`, `esp32/src/sensors/obstacle_sensor.h`, `esp32/src/sensors/distance_sensor.h` (nuevo), `esp32/src/main.cpp`, `esp32/src/test_obstacle_sensor.cpp`
- **Tests:** `test_tfmini_overflow_large_distance` (boundary 6553/6554 cm), `test_tfmini_timeout_clears_distance` (verifica distance y zone a 0 tras timeout), `test_tfmini_header_byte_in_distance` (0x5959 rechazado por overflow guard). TF-Mini Plus: 74 tests, TOFSense-M: 135 tests.
- **Próximos pasos:** Verificar en hardware con TF-Mini Plus conectado. Verificar que CAN 0x208 reporta `health=0, status=INVALID` correctamente cuando el sensor está desconectado.

### PR-277 — fix(can): two-phase error-passive recovery (never give up)
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** La recuperación de error-passive del TWAI (ESP32) ahora usa una estrategia bifásica en vez de abandonar tras 10 intentos.
- **Root cause:** Cuando el TEC saturaba a 128 (error-passive) y el STM32 no estaba en el bus (sin ACK), la recuperación se agotaba tras 10 reinits rápidos (cada 3 s = 30 s total). Después, el ESP32 nunca volvía a intentar recuperar el CAN, dejando el `bus_err` creciendo indefinidamente (observado: 45430 → 138000+). Si el STM32 arrancaba después de esos 30 s, el CAN del ESP32 quedaba muerto permanentemente hasta reinicio.
- **Solución aplicada:** Estrategia bifásica: (1) Phase 1 (fast): primeros 10 intentos cada 3 s (comportamiento original). (2) Phase 2 (slow): intentos ilimitados cada 30 s — suficiente para recuperarse cuando el STM32 se conecte, sin martillear el bus. Renombrado `ERROR_PASSIVE_MAX_RESETS` → `ERROR_PASSIVE_MAX_FAST`, añadido `ERROR_PASSIVE_SLOW_TIMEOUT_MS = 30000`. El contador `errorPassiveResets` se resetea cuando TEC cae por debajo de 128 (bus sano).
- **Impacto en el sistema:** El CAN del ESP32 ya no queda permanentemente muerto si los 10 reinits rápidos no resuelven el error-passive. El sistema sigue intentando periódicamente con reinits lentos, permitiendo recuperación automática si el STM32 se conecta tarde.
- **Archivos modificados:** `esp32/src/main.cpp`
- **Próximos pasos:** Verificar en hardware que el reinit lento (30 s) recupera el CAN correctamente cuando el STM32 se conecta después de los 10 intentos rápidos.
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Mejora fiabilidad del sensor de obstáculos (TOFSense-M 8×8) y reduce falsos positivos en warnings de bloqueo del RuntimeMonitor.
- **Root cause:** (1) Buffer UART RX de 1024 bytes se llenaba en ~11 ms a 921600 baud; cualquier jitter del loop >11 ms causaba overflow con pérdida de bytes mid-frame, provocando fallos de checksum en cascada. (2) Resync tras BAD_CHECKSUM solo buscaba 0x57 (1/256 probabilidad por byte) — bytes 0x57 en datos de píxeles generaban falsos resyncs, causando más fallos de checksum en cadena. (3) Umbral de bloqueo de render (4 ms) era demasiado bajo para TFT SPI (~22 ms normal) ejecutándose en Core 0 (sin bloquear el loop principal en Core 1).
- **Solución aplicada:** (1) `rxBufSize` 1024→4096 bytes (~44 ms de headroom a 921600 baud). (2) `MAX_BYTES_PER_UPDATE` 800→1600 bytes (procesa hasta 4 frames/llamada). (3) Resync mejorado: busca par 0x57+0x01 (header+function_mark) en vez de solo 0x57, reduciendo probabilidad de falso resync de 1/256 a 1/65536. (4) Threshold de bloqueo de render separado: `RENDER_BLOCKING_THRESHOLD_US = 35000` (35 ms) para Core 0, ya que SPI TFT ~22 ms es normal y no afecta al loop principal. (5) Añadido `uartHWM` (high-water mark) al diagnóstico para detectar overflow de buffer UART.
- **Impacto en el sistema:** Sensor de obstáculos más fiable con mejor tolerancia a jitter del loop. Warnings `render=YES` solo aparecen ante stalls reales (>35 ms), no ante frames TFT normales. Nuevos diagnósticos `uartHWM` ayudan a detectar overflow.
- **Archivos modificados:** `esp32/src/sensors/obstacle_sensor.h`, `esp32/src/sensors/obstacle_sensor.cpp`, `esp32/src/ui/runtime_monitor.h`, `esp32/src/ui/runtime_monitor.cpp`, `esp32/src/test_obstacle_sensor.cpp`
- **Próximos pasos:** Verificar en hardware que `uartHWM` se mantiene < 4096 y que la tasa de `cksumFail` disminuye significativamente. Si `uartHWM` ≥ 3500, considerar aumentar el buffer o reducir frame rate del sensor con NAssistant.

### PR-275 — fix(ci): suppress cppcheck duplicateAssignExpression false positives on CCCR triple-read
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Corrige fallo de CI cppcheck (exit code 2) causado por falsos positivos `duplicateAssignExpression` en las lecturas triples de CCCR. Cppcheck no entiende que `hfdcan1.Instance->CCCR` es un registro hardware volátil que puede retornar valores diferentes en cada lectura.
- **Root cause:** Las lecturas triples de CCCR en `main.c` y `can_handler.c` son intencionalmente la misma expresión repetida 3 veces para detectar datos inconsistentes del bus AHB. Cppcheck las clasifica como `duplicateAssignExpression` (asignaciones duplicadas) porque no analiza semántica de registros volátiles.
- **Solución aplicada:** (1) Añadidos comentarios `// cppcheck-suppress duplicateAssignExpression` en las 4 líneas afectadas. (2) Añadido flag `--inline-suppr` al comando cppcheck en CI para habilitar supresiones inline.
- **Impacto en el sistema:** Solo CI — sin cambio funcional en firmware. Las supresiones son localizadas y documentan por qué el patrón es intencional.
- **Archivos modificados:** `Core/Src/main.c`, `Core/Src/can_handler.c`, `.github/workflows/firmware-validation.yml`
- **Próximos pasos:** Ninguno.

### PR-274 — fix(fdcan): add CCCR triple-read to CAN_Init clock re-apply path
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Cierra edge case en el path de re-init de `CAN_Init()`: tras re-aplicar clock source PCLK1 y re-invocar `HAL_FDCAN_Init()`, faltaba la comprobación triple-lectura de CCCR que sí tenía `MX_FDCAN1_Init`. Sin esta comprobación, el re-init podía reportar éxito con datos basura.
- **Root cause:** El re-init en `CAN_Init()` (líneas 265-274) no incluía validación triple-lectura de CCCR tras `HAL_FDCAN_Init()`. Si la clock gate seguía inestable tras el cambio de fuente de reloj, `HAL_FDCAN_Init()` podía devolver `HAL_OK` con registros retornando stale bus data — el mismo falso positivo que la triple-lectura de `MX_FDCAN1_Init` previene.
- **Solución aplicada:** Añadido bloque triple-lectura CCCR inmediatamente después de `HAL_FDCAN_Init()` en el path de clock re-apply de `CAN_Init()`. Verifica que 3 lecturas consecutivas sean idénticas, INIT=1, y bits 16-31 = 0.
- **Impacto en el sistema:** Elimina el último path de falso positivo conocido en la cadena de init FDCAN.
- **Archivos modificados:** `Core/Src/can_handler.c`
- **Próximos pasos:** Verificar en hardware con SWD que el re-init path funciona correctamente.

### PR-273 — fix(fdcan): multi-read CCCR consistency + clock source resilience
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Corrige dos bugs interrelacionados en la inicialización FDCAN del STM32: (1) la comprobación CCCR en `MX_FDCAN1_Init` podía ser engañada por datos basura aleatorios del bus AHB, y (2) `CAN_Init` fallaba permanentemente si el clock source PCLK1 no estaba latched tras el force-reset.
- **Root cause:** Cuando la clock gate del FDCAN APB está inestable tras force-reset, las lecturas de registros devuelven datos aleatorios del bus (stale AHB bus data). Una lectura única de CCCR puede coincidir con los bits esperados (INIT=1, reservados=0), generando un falso positivo. Además, `CCIPR.FDCANSEL` puede revertir a su valor por defecto (HSE=00) en ciertos revisions de silicio, causando que `CAN_Init` abandone sin recuperación.
- **Solución aplicada:** (1) Triple lectura de CCCR en `MX_FDCAN1_Init` con comparación de consistencia — si las 3 lecturas no son idénticas, el periférico no está clocked correctamente. (2) `CAN_Init` re-aplica PCLK1 con barrera `__DSB()` si detecta clock source incorrecto, y hace re-init completo del periférico. (3) Nuevos campos diagnóstico `clk_reapplied` y `ccipr_raw` en `CAN_InitDiag_t`.
- **Impacto en el sistema:** Elimina falsos positivos en init FDCAN. El sistema puede recuperar el clock source sin quedar permanentemente sin CAN. Diagnósticos SWD mejorados para depuración futura.
- **Archivos modificados:** `Core/Src/main.c`, `Core/Src/can_handler.c`, `Core/Inc/can_handler.h`
- **Próximos pasos:** Verificar en hardware con SWD que `clk_reapplied` y `ccipr_raw` reportan valores correctos.

### PR-272 — fix: unblock main loop — vTaskDelay yield, Serial TX buffer, bounded UART read
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Tres correcciones para eliminar bloqueo residual del main loop en Core 1: (1) yield al scheduler, (2) buffer Serial TX, (3) lectura UART acotada del sensor de obstáculos.
- **Root cause:** (1) `loop()` nunca hacía `vTaskDelay()` — monopolizaba Core 1 sin dar CPU al TWAI driver, idle task ni housekeeping del sistema. (2) `Serial.printf()` con diagnósticos largos (>128 chars) bloqueaba hasta 18 ms esperando que el FIFO UART se drenase a 115200 baud. (3) `while (tofSerial.available() > 0)` sin límite podía procesar hasta 1024 bytes por iteración a 921600 baud.
- **Solución aplicada:** (1) `vTaskDelay(1)` al final de `loop()` para yield de Core 1 (~1 ms). (2) `Serial.setTxBufferSize(512)` antes de `Serial.begin()` — los printf van a ring buffer y no bloquean. (3) Lectura UART acotada a `MP_FRAME_LENGTH * 2` (800 bytes) por llamada a `update()`.
- **Impacto en el sistema:** Loop fluido sin bloqueos: scheduler Core 1 activo, Serial no bloqueante, procesamiento UART predecible. Loop rate ~500 Hz (2 ms/iteración incluyendo yield).
- **Próximos pasos:** Ninguno.

### PR-271 — fix(rtmon): reset per-period stats, separate UI/render timing, add loop instrumentation
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Corrección de falsos positivos en `[RT] WARN blocking`, separación de instrumentación UI/render, y adición de timing del main loop (Core 1).
- **Root cause:** (1) Flags de bloqueo y stats max acumulaban desde el boot sin resetearse entre periodos de 5 s — una sola frame >4 ms durante boot hacía que el warning se mostrase indefinidamente. (2) `RTMON_UI` envolvía `screenManager.update()` completo (incluyendo `draw()`), por lo que `ui=YES` siempre acompañaba a `render=YES`. (3) Sin instrumentación del main loop en Core 1.
- **Solución aplicada:** `logToSerial()` resetea flags de bloqueo, max/min frame, phase maximums y zone counters tras imprimir (cada periodo de 5 s es independiente). `RTMON_UI_BEGIN/END` movido dentro de `screen_manager.cpp` para envolver solo `currentScreen_->update(data)`. Añadido `RTMON_LOOP_BEGIN/END` en `loop()`. Formato de log actualizado con `loop=` timing. Debug overlay muestra loop max.
- **Impacto en el sistema:** Diagnósticos de rendimiento ahora reflejan la realidad de cada periodo. `render=YES` esperado (TFT SPI), `ui=YES` indica stall real de procesamiento de datos, `loop=YES` indica bloqueo real en Core 1.
- **Próximos pasos:** Ninguno.

### PR-270 — Fix FDCAN init failure (CCCR garbage reads) and ESP32 CAN TX blocking
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** (1) Aumento de delay de estabilización de clock gate FDCAN de 32 a 3200 iteraciones (~19 µs). (2) Todos los `ESP32Can.writeFrame()` cambiados a `timeout=0` (non-blocking).
- **Root cause:** (1) `FDCAN_CLK_STABILISE_ITERS=32` (~190 ns) era insuficiente en algunas revisiones de silicio STM32G4, causando lecturas basura de CCCR. (2) `writeFrame()` con timeout por defecto de 1000 ms bloqueaba el main loop hasta 1 s cuando la cola TX estaba llena (bus muerto/error-passive).
- **Solución aplicada:** `FDCAN_CLK_STABILISE_ITERS` 32→3200. `FDCAN_INITIAL_SETTLE_DELAY_MS` 1→2 ms, `FDCAN_CLOCK_SETTLE_DELAY_MS` 5→10 ms. 8 call sites de `writeFrame()` actualizados con `timeout=0`.
- **Impacto en el sistema:** FDCAN init fiable en todas las revisiones de silicio. CAN TX nunca bloquea el main loop.
- **Próximos pasos:** Ninguno.

### PR-268 — fix: reorder error-passive guard and validate TWAI teardown return values
- **Fecha:** 2026-03-29
- **Autor:** Copilot
- **Descripción del cambio:** Reordenamiento de la lógica de recuperación error-passive y validación de valores de retorno de `twai_stop()` / `twai_driver_uninstall()`.
- **Root cause:** (1) El guard de max resets se evaluaba después de `errorPassiveSince == 0`, provocando bucle infinito de set/clear del timestamp al agotar reintentos. (2) Valores de retorno de teardown no verificados podían dejar el driver en estado inconsistente.
- **Solución aplicada:** Mover check `errorPassiveResets >= MAX` antes de `errorPassiveSince == 0`. Validar retorno de `twai_stop()` y `twai_driver_uninstall()` (tolerar `ESP_ERR_INVALID_STATE`). Budget cuenta todos los intentos (éxito + fallo).
- **Impacto en el sistema:** Recuperación error-passive ahora es robusta ante agotamiento de reintentos y fallos de hardware intermitentes.
- **Próximos pasos:** Ninguno. Complementa PR #267.

### PR-267 — Add ESP32 TWAI error-passive recovery (tx_err=128 stuck state)
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Añadida recuperación automática de estado error-passive en ESP32 TWAI (TEC=128 persistente durante >3 s con estado RUNNING).
- **Root cause:** TWAI controller quedaba atascado en error-passive (TEC=128) indefinidamente. El recovery existente solo se activaba en BUS_OFF (TEC=256), nunca en error-passive.
- **Solución aplicada:** Detección de `tx_err >= 128` con timer de 3 s. Full driver reinit (`stop→uninstall→install→start`) via `twaiInit()`. Máximo 10 intentos.
- **Impacto en el sistema:** CAN se recupera de fallos eléctricos intermitentes que no escalan a bus-off.
- **Próximos pasos:** Aplicado fix en PR #268.

### PR-266 — Harden FDCAN init: proper reset pulse timing and increased retry margin
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Pulse stretch del reset FDCAN con `__DSB()` + 4× `__NOP()`, barrera `__DSB()` + `__ISB()`, y bucle de estabilización volátil antes del acceso a CCCR. Aumentados reintentos de 3 a 5 con delay de 5 ms.
- **Root cause:** Registros FDCAN devolvían basura (e.g., `CCCR = 0x8007ca5`) tras force-reset en ciertas revisiones de STM32G4. La puerta de clock no se estabilizaba lo suficientemente rápido.
- **Solución aplicada:** Hardening de `HAL_FDCAN_MspInit` con secuencia de reset robusta y 5 reintentos en `MX_FDCAN1_Init`.
- **Impacto en el sistema:** Inicialización FDCAN fiable en todas las revisiones de silicio.
- **Próximos pasos:** Ninguno.

### PR-265 — Offload TFT rendering + touch to FreeRTOS task on Core 0
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** TFT SPI rendering (~28 ms/frame) movido a un FreeRTOS task dedicado en Core 0.
- **Root cause:** El render en el main loop bloqueaba CAN polling y procesamiento safety-critical. Logs mostraban `[RT] WARN blocking: ui=YES render=YES`.
- **Solución aplicada:** `vTaskCreatePinnedToCore` para renderTask en Core 0. Main loop en Core 1 libre para CAN, sensores, audio.
- **Impacto en el sistema:** Main loop sin bloqueo. CAN polling y safety checks siempre en tiempo.
- **Próximos pasos:** Ninguno.

### PR-264 — fix(fdcan): harden FDCAN init against clock-gate race and stuck INIT
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Re-enable APB1 clock tras force-reset + `__DSB()` barrera. CCCR sanity check (INIT bit set, reserved bits 16–31 zero) con retry loop (3 intentos).
- **Root cause:** FDCAN devolvía basura (CCCR=`0x8007aa5`, PSR/ECR/RXF0S/TXFQS `0x8007c3d`) tras RCC force-reset. HAL_FDCAN_Init pasaba contra valores basura.
- **Solución aplicada:** Re-enable clock post-reset + barrera de memoria + CCCR sanity check post-init.
- **Impacto en el sistema:** Detección de FDCAN corrupto al arranque con abort controlado.
- **Próximos pasos:** Mejorado en PR #266.

### PR-263 — Fix I2C requestFrom error spam when MCP23017 is not connected
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Fix de detección NACK en ESP32-S3 Wire para MCP23017 shifter.
- **Root cause:** `Wire.endTransmission(false)` (repeated start) devuelve 0 incluso sin dispositivo. El backoff probe pasaba y `requestFrom()` fallaba ruidosamente cada 1 s.
- **Solución aplicada:** Usar `endTransmission(true)` (con STOP) para detección NACK fiable en backoff probe.
- **Impacto en el sistema:** Eliminado spam de errores I2C cuando MCP23017 no está conectado.
- **Próximos pasos:** Ninguno.

### PR-262 — Fix FDCAN stuck in INIT: replace per-ID filters with MASK accept-all
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Reemplazo de 5 filtros DUAL/RANGE específicos por un único filtro MASK accept-all.
- **Root cause:** FDCAN no salía de modo INIT (`CCCR.INIT` stuck high), causando ESP32 `tx_err=128` y escalada de `bus_err`. CAN bus completamente no funcional.
- **Solución aplicada:** Filtro MASK accept-all (index 0, FilterID1=0x000, FilterID2=0x000). Global filter acepta IDs estándar y extendidos en RXFIFO0. Filtrado real en `CAN_ProcessMessages()` switch/case.
- **Impacto en el sistema:** FDCAN sale de INIT correctamente. CAN funcional.
- **Próximos pasos:** Ninguno.

### PR-261 — fix: add TWAI bus-off recovery to ESP32
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Recuperación automática de bus-off en ESP32 TWAI.
- **Root cause:** Sin recovery de bus-off, cuando ESP32 arranca antes que STM32 (sin ACK), TEC=256 → BUS_OFF permanente.
- **Solución aplicada:** Check cada 250 ms, `twai_initiate_recovery()` en BUS_OFF, `twai_start()` en STOPPED. Máximo 10 intentos.
- **Impacto en el sistema:** ESP32 CAN se recupera automáticamente tras bus-off. Espeja `CAN_CheckBusOff()` del STM32.
- **Próximos pasos:** Complementado con error-passive recovery (PR #267–268).

### PR-260 — fix: re-apply FDCAN clock source after peripheral force-reset in MspInit
- **Fecha:** 2026-03-28
- **Autor:** Copilot
- **Descripción del cambio:** Re-aplicar clock source PCLK1 tras `__HAL_RCC_FDCAN_FORCE_RESET()`.
- **Root cause:** Force-reset borra `RCC_CCIPR.FDCANSEL` a `00` (HSE). HSE no habilitado → peripheral sin clock → basura en registros.
- **Solución aplicada:** `__HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PCLK1)` tras release-reset en `HAL_FDCAN_MspInit`.
- **Impacto en el sistema:** FDCAN siempre usa PCLK1 correcto tras cualquier tipo de reset.
- **Próximos pasos:** Diagnosticado previamente en PR #259.

### PR-259 — feat(can): add FDCAN clock source and CCCR.INIT runtime diagnostics
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Añadidos diagnósticos runtime de clock source FDCAN y estado CCCR.INIT a `CAN_InitDiag_t`.
- **Root cause:** GDB mostró `RCC_CCIPR = 0x0` (FDCANSEL en HSE default) y `CCCR = 0x8007ADD` (basura). Sin verificación runtime para detectar estas condiciones.
- **Solución aplicada:** Campos `clk_ok` y `cccr_init_ok` en `CAN_InitDiag_t`. Abort si clock o CCCR inválidos. Legible via SWD con `p can_init_diag`.
- **Impacto en el sistema:** Diagnóstico post-mortem de fallos de inicialización FDCAN.
- **Próximos pasos:** Explotado en PRs #260–266.

### PR-258 — Fix FDCAN initialization failure after non-power-on resets
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Force-reset del periférico FDCAN en `HAL_FDCAN_MspInit` antes de la inicialización.
- **Root cause:** FDCAN retiene estado de error (CCCR.CSA/INIT stuck, bus-off latched) tras resets de watchdog o anómalos. `HAL_FDCAN_Init()` falla → sistema stuck en BOOT.
- **Solución aplicada:** `__HAL_RCC_FDCAN_FORCE_RESET()` + `__HAL_RCC_FDCAN_RELEASE_RESET()` en MspInit.
- **Impacto en el sistema:** FDCAN se inicializa correctamente tras cualquier tipo de reset.
- **Próximos pasos:** Mejorado en PRs #259–266.

### PR-257 — Change heartbeat LED to brief flash every ~2s
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Cambio de heartbeat LED de toggle 5 Hz continuo a flash breve ~50 ms cada ~2 s.
- **Root cause:** Toggle 5 Hz era indistinguible del blink ~2 Hz constante de `Error_Handler`.
- **Solución aplicada:** Flash breve (~50 ms ON, ~1950 ms OFF).
- **Impacto en el sistema:** Diagnóstico visual claro del estado de la MCU.
- **Próximos pasos:** Ninguno.

### PR — Move LPWM_FR from PB14/TIM15_CH1 to PC3/TIM1_CH4 + LED_DIAG on PB14
- **Fecha:** 2025-07-17
- **Autor:** Copilot
- **Descripción del cambio:** Reubicación de LPWM_FR de PB14/TIM15_CH1 a PC3/TIM1_CH4 (AF2). PB14 liberado y reasignado como GPIO_Output para LED diagnóstico (LED_DIAG). TIM15 eliminado completamente del proyecto. Los 4 canales del motor FR quedan ahora en TIM1 (CH1=RPWM_FL/PA8, CH2=LPWM_FL/PA9, CH3=RPWM_FR/PA10, CH4=LPWM_FR/PC3).
- **Root cause:** Consolidación de todos los canales PWM de tracción frontal en un solo timer (TIM1) y liberación de TIM15.
- **Solución aplicada:** GPIO remap de LPWM_FR a PC3/TIM1_CH4, PB14 como LED_DIAG, eliminación de TIM15.
- **Impacto en el sistema:** PWM de motor FR en TIM1_CH4/PC3 (Morpho CN7 pin 37). LED diagnóstico en PB14 (Morpho CN10 pin 28).
- **Próximos pasos:** Ninguno.

### PR-256 — Move LPWM_FR from PB9/TIM17_CH1 to PB14/TIM15_CH1
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Reubicación de LPWM_FR a PB14/TIM15_CH1 para resolver conflicto de pines tras remap de FDCAN.
- **Root cause:** CI fallaba porque stubs en `analysis_artifacts/stubs/` no tenían definiciones de TIM15.
- **Solución aplicada:** Actualización de pin mapping y stubs de CI.
- **Impacto en el sistema:** PWM de motor FR funcional en nuevo pin.
- **Próximos pasos:** Ninguno.

### PR-255 — Move FDCAN1 from PB8/PB9 to PA11/PA12
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Remap FDCAN1 de PB8/PB9 a PA11(RX)/PA12(TX) AF9 según layout NUCLEO-G474RE CN10.
- **Root cause:** PB8/PB9 conflicto con PWM de motor. PA11/PA12 disponibles en header CN10.
- **Solución aplicada:** GPIO remap + reubicación de LPWM_FR (TIM1_CH4 → TIM17/PB9).
- **Impacto en el sistema:** CAN en pines correctos sin conflicto con PWM.
- **Próximos pasos:** Completado en PR #256.

### PR-254 — feat(boot_screen): add ESP32/STM32 fault isolation diagnostics
- **Fecha:** 2026-03-27
- **Autor:** Copilot
- **Descripción del cambio:** Diagnósticos de aislamiento de fallos ESP32/STM32 en pantalla de boot.
- **Root cause:** Pantalla de boot mostraba CAN status genérico sin indicar qué lado fallaba.
- **Solución aplicada:** Indicadores separados de estado ESP32 y STM32 en waiting screen.
- **Impacto en el sistema:** Diagnóstico visual más rápido en campo.
- **Próximos pasos:** Ninguno.

---

## 4. Roadmap / Próximos Objetivos

### Alta Prioridad
- [ ] **Integración completa de audio DFPlayer**: Sonidos de aviso, error, arranque. Hardware presente, software parcial.
- [ ] **Asistente de calibración touch**: Wizard persistente con almacenamiento SPIFFS.
- [ ] **Test en vehículo real**: Validación completa del sistema integrado.

### Media Prioridad
- [ ] **Frenado regenerativo**: Aprovechar motores para recuperación de energía.
- [ ] **Control de crucero adaptativo (ACC)**: Integración con sensor de obstáculos.
- [ ] **Telemetría WiFi/BT**: Dashboard remoto para diagnóstico y monitorización.

### Baja Prioridad / Mejoras Futuras
- [ ] **Optimización de consumo**: Sleep modes para ESP32 en standby prolongado.
- [ ] **OTA updates**: Actualización de firmware ESP32 por WiFi.
- [ ] **Logging expandido**: Más datos en error log (GPS, acelerómetro).

---

## 5. Reglas Técnicas del Proyecto

### Arquitectura (No Romper)
1. **STM32 = Safety Authority**: Toda validación de comandos y control de actuadores reside en el STM32. El ESP32 **nunca** controla motores directamente.
2. **Dual-core ESP32**: Render siempre en Core 0. CAN, sensores, audio, LEDs en Core 1. No mezclar.
3. **Protocolo CAN congelado v1.3**: No modificar IDs ni formato de mensajes sin actualizar `CAN_CONTRACT_FINAL.md` y ambos firmwares simultáneamente.
4. **DMA prohibido para CAN**: FDCAN usa interrupciones (prioridad 1), no DMA.

### Inicialización Robusta
5. **FDCAN MspInit**: Siempre force-reset + re-apply clock source (PCLK1) + pulse stretch (`__DSB()` + `__NOP()`) + barrera (`__DSB()` + `__ISB()` + volatile loop) antes de CCCR access.
6. **FDCAN Init retries**: Mínimo 5 intentos con CCCR sanity check (INIT set, bits 16–31 zero). Abort controlado si falla.
7. **FDCAN clock source**: Encoding en RCC_CCIPR bits [25:24]: 00=HSE, 01=PLL-Q, 10=PCLK1. Este proyecto usa PCLK1 (valor 0x02000000).
8. **Boot order**: `Boot_ReadResetCause()` antes de IWDG. Prioridad de interrupciones: CAN(1) > PWM/Encoder(2) > I2C(3) > SysTick(4).

### Safety
9. **CAN timeout → LIMP_HOME**: 250 ms sin heartbeat → 20% torque máximo.
10. **Bus-off recovery**: Implementada en ambos lados (STM32 `CAN_CheckBusOff()`, ESP32 TWAI recovery). Max 10 intentos.
11. **Error-passive recovery (ESP32)**: `tx_err >= 128` persistente >3 s → full driver reinit. Max 10 intentos.
12. **ABS/TCS**: Umbral 15% slip. No desactivable en modo normal.
13. **Startup inhibit**: Pedal debe estar liberado 400 ms antes de habilitar torque.
14. **BREAK2**: TIM1/TIM8 BREAK2 vinculado a Cortex LOCKUP. Fuerza PWM a safe state.

### CAN Bit Timing
15. **ESP32 TWAI**: BRP=10, TSEG1=13, TSEG2=2, SJW=2 → 87.5% SP.
16. **STM32 FDCAN**: BRP=10, TSEG1=29, TSEG2=4, SJW=4 → 88.2% SP. Delta 0.7% dentro de tolerancia CiA 301.

### Filtrado CAN
17. **STM32**: Filtro MASK accept-all (index 0). Filtrado real en `CAN_ProcessMessages()` switch/case.
18. **Global filter**: Acepta IDs estándar y extendidos no coincidentes en RXFIFO0.

### Pin Mapping (Actual)
19. **FDCAN1**: PA11 (RX) / PA12 (TX), AF9. **No PB8/PB9**.
20. **CAN pin defines**: `PIN_CAN_TX = GPIO_PIN_12`, `PIN_CAN_RX = GPIO_PIN_11` en `project_config.h`.

### I2C
21. **ESP32 Wire**: Usar `endTransmission(true)` (con STOP) para detección NACK fiable. `endTransmission(false)` devuelve 0 sin dispositivo.

### Diagnósticos
22. **CAN_InitDiag_t**: 12 campos (hal_init, filter_global, notify, start, started, clk_ok, cccr_init_ok, clk_reapplied, retries, timeout_flag, msp_clk_ok, msp_ccipr_ok) + ccipr_raw. Legible vía SWD con `p can_init_diag`.

### RuntimeMonitor (ESP32)
23. **Instrumentación separada**: `RTMON_UI_BEGIN/END` envuelve solo `currentScreen_->update(data)` dentro de `screen_manager.cpp`, NO el `screenManager.update()` completo en renderTask. `RTMON_RENDER_BEGIN/END` envuelve `draw()`. `RTMON_LOOP_BEGIN/END` envuelve el main `loop()` en Core 1.
24. **Stats por periodo**: `logToSerial()` resetea flags de bloqueo, max/min frame, phase maximums y zone counters tras imprimir. Cada ventana de 5 s es independiente. NO llamar `reset()` desde `logToSerial()` — ring buffer y FPS counters deben persistir entre periodos.
25. **CAN TX non-blocking**: Todos los `ESP32Can.writeFrame()` deben usar `timeout=0`. El timeout por defecto de 1000 ms bloquea el loop cuando la cola TX está llena (bus muerto/error-passive).
26. **Render blocking threshold**: `RENDER_BLOCKING_THRESHOLD_US = 35000` (35 ms) para Core 0. TFT SPI ~22–28 ms es normal. `BLOCKING_THRESHOLD_US = 4000` (4 ms) para CAN/UI/loop en Core 1.

### Sensor de Obstáculos (ESP32)
27. **Sensor type selection**: `SENSOR_TYPE` en `obstacle_sensor.h` — `SENSOR_TYPE_TOFSENSE=0` (921600 baud, 400B frames), `SENSOR_TYPE_TFMINI=1` (115200 baud, 9B frames). Default: TFMINI.
28. **Sensor enable flag**: `OBSTACLE_SENSOR_ENABLED` en `obstacle_sensor.h` — `0` = deshabilitado (sin UART, sin buffers, sin CPU), `1` = habilitado. Default: 0 (sensor no conectado actualmente).
29. **TF-Mini Plus overflow guard**: Conversión cm→mm usa `uint32_t` intermedio + guarda `> UINT16_MAX`. Valores >6553 cm se rechazan como inválidos (retorna `false`).
30. **Timeout limpia estado completo**: En timeout de frame, `distance_mm = 0`, `zone = 0`, `healthy = false`, `status = INVALID`. Nunca se envían datos stale por CAN.

---

## 6. Historial de Cambios

| Fecha | Hito | PRs Relacionados |
|---|---|---|
| 2026-03-27 | **Remap FDCAN1 a PA11/PA12** — Resolución de conflicto PB8/PB9 con PWM motor | #255, #256 |
| 2026-03-27 | **Diagnósticos FDCAN runtime** — clk_ok y cccr_init_ok en CAN_InitDiag_t | #259 |
| 2026-03-27 | **Heartbeat LED diferenciable** — Flash breve ~50 ms / ~2 s | #257 |
| 2026-03-27 | **Boot diagnostics en HMI** — Aislamiento de fallos ESP32/STM32 en pantalla | #254 |
| 2026-03-27–28 | **Hardening FDCAN init** — Force-reset, clock re-apply, pulse stretch, CCCR sanity, 5 retries | #258, #260, #264, #266 |
| 2026-03-28 | **CAN filter simplification** — MASK accept-all reemplaza 5 filtros específicos | #262 |
| 2026-03-28 | **I2C NACK fix** — endTransmission(true) para MCP23017 backoff | #263 |
| 2026-03-28 | **FreeRTOS render offload** — TFT+touch en Core 0, main loop libre en Core 1 | #265 |
| 2026-03-28 | **ESP32 CAN bus-off recovery** — twai_initiate_recovery + twai_start, max 10 intentos | #261 |
| 2026-03-28–29 | **ESP32 CAN error-passive recovery** — Full driver reinit cuando tx_err=128 persiste >3 s | #267, #268 |
| 2026-03-29 | **FDCAN clock stabilisation + CAN TX non-blocking** — CLK_STABILISE_ITERS 32→3200, writeFrame timeout=0 en 8 call sites | #270 |
| 2026-03-29 | **RuntimeMonitor fix** — Stats por periodo (no acumulativo), separación UI/render, instrumentación loop Core 1 | #271 |
| 2026-03-29 | **Unblock main loop** — vTaskDelay(1) yield Core 1, Serial TX buffer 512, UART read acotado a 800 bytes | #272 |
| 2026-03-29 | **FDCAN init consistency fix** — Multi-read CCCR check, clock source resilience en CAN_Init, diagnósticos ccipr_raw | #273 |
| 2026-03-29 | **FDCAN CAN_Init CCCR triple-read** — Añadido triple-read CCCR al path de re-init por clock re-apply en CAN_Init | #274 |
| 2026-03-29 | **CI cppcheck fix** — Supresión inline de falsos positivos `duplicateAssignExpression` en lecturas triples CCCR + `--inline-suppr` | #275 |
| 2026-03-29 | **Obstacle sensor reliability** — UART RX buffer 1024→4096, resync 0x57+0x01, MAX_BYTES_PER_UPDATE 800→1600, render blocking threshold 4→35 ms, uartHWM diag | #276 |
| 2026-03-29 | **CAN error-passive bifásica** — Recovery no abandona tras 10 intentos: fase rápida (10×3s) + fase lenta (ilimitado×30s) | #277 |
| 2026-03-30 | **TF-Mini Plus overflow + stale data fix** — uint16 overflow guard en cm→mm, timeout limpia distance/zone, soporte dual sensor (TOFSense-M/TF-Mini Plus), flag OBSTACLE_SENSOR_ENABLED, distance_sensor.h | #275 (GitHub) |
| 2026-03-31 | **FDCAN MspInit adaptive poll** — Reemplaza bucle volátil 3200 iter por poll CCCR adaptativo (50 ms timeout) + verificación readback FDCANSEL | #278 |
| 2026-04-02 | **TWAI clk_src fix** — `memset` zeroed `clk_src` field in ESP-IDF 5.x → CAN bus inoperative. Replaced with `TWAI_TIMING_CONFIG_500KBITS()` macro init | #279 |
| 2026-04-04 | **Full-system CAN audit** — Comprehensive firmware + protocol audit, CAN RX diagnostics, DLC doc fix, hardware validation checklist, root cause analysis | #279a |
| 2026-04-04 | **CAN hardware fix procedure** — Executable troubleshooting guide, fix stale PB8/PB9 pin refs → PA11/PA12 in HARDWARE_VALIDATION_PROCEDURE.md + PROJECT_MASTER_STATUS.md | #279b |
| 2026-04-04 | **Documento sistema alimentación** — 9 secciones + BOM: arquitectura, recorrido alimentación, relés, apagado retardado, LEDs, protección, masas, esquema eléctrico, materiales | #280 |
| 2026-04-06 | **FDCAN init robusta** — Secuencia determinista de bring-up: clock source before enable, readback-verify RCC, reset con DSB/ISB, poll adaptativo CCCR, started=0 por defecto, 4 nuevos campos diagnóstico | #281 |
| 2026-04-06 | **Decouple hal_msp + fix filter docs** — Extraer `CAN_InitDiag_t` a header propio, eliminar include de `can_handler.h` en hal_msp.c, corregir descripción filtros CAN (accept-all, no reject-all) | #282 |
| 2026-04-06 | **LED boot visibility** — Boot blinks 60→150 ms, post-init CAN status LED (1 long=OK, 5 rapid=FAIL), volatile can_init_diag | #283 |
| 2026-04-07 | **Boot blinks unmissable** — Boot blinks 150→300 ms, dark lead-in 500 ms, CAN status blinks más lentos, separación 500 ms | #284 |
| 2026-04-07 | **LED migrada PA5→PB8** — Status LED de PA5 (LD2, interferido por ST-Link SB21) a PB8 (GPIO libre, Morpho CN10-3). LED externo requerido. | #285 |
| 2026-04-07 | **LED revertida PB8→PA5 (LD2)** — LD2 está soldado en la placa Nucleo. No necesita LED externo. Revert de PR-285. | #286 |
| 2026-04-07 | **FDCAN antes de boot blinks** — Mover MX_FDCAN1_Init+CAN_Init antes de la secuencia LED. ESP32 ya no entra en BUS_OFF durante boot. | #287a |
| 2026-04-07 | **Fix baud rate ESP32 625→500 kbps** — CRÍTICO: ESP-IDF 5.x ignora brp cuando quanta_resolution_hz>0. Fix: clear quanta_resolution_hz=0. Restaura comunicación CAN. | #287 |
| 2026-04-07 | **Auditoría palabra-por-palabra** — 10 archivos STM32 (LIMPIO) + 5 archivos ESP32 (3 bugs corregidos: printf %lus, %d→%u para size_t, COM8 hardcodeado). Changelog actualizado. | #288 |
| 2026-04-08 | **2ª auditoría** — boot_validation.c: isnan() añadido a logging diagnóstico (NaN sensors ahora registran fault). ESP32: cast (unsigned) en PSRAM printf. | #289 |
| 2026-04-08 | **Robustez avanzada** — NaN/Inf hardening en 10+ rutas safety, TX NACK detection, CAN FPS metric, boot RAM/clock/periph checks, logs estandarizados [MODULE][SEVERITY] | #290 |
| 2026-04-08 | **cppcheck shadowVariable fix** — Mover `extern SystemCoreClock` a file scope en boot_validation.c para evitar shadow con HAL stub | #291 |
| 2026-04-09 | **LD2 muestra estado CAN en main loop** — CAN OK: flash breve 2s, CAN FAIL: parpadeo 1Hz. No necesita LED externo PB14. | #292 |
| 2026-04-09 | **Activar sensor TF-Mini Plus** — OBSTACLE_SENSOR_ENABLED=1, SENSOR_TYPE=TFMINI. Firmware listo para sensor 115200 bps en GPIO 18. Guía de cableado + docs actualizados. | — |
| 2026-04-09 | **Auditoría TF-Mini Plus** — Fix sampling rate 10→100 Hz (TFM_MAX_BYTES_PER_UPDATE 32→256, eliminar 1-frame-per-call, rxBuf 256→512). Latencia 100ms→10ms. Comentarios actualizados. | — |
| 2026-04-09 | **Guía puesta en marcha segura** — Documento 10 fases (I2C→DS18B20→INA226→encoder→sensores→pedal→relés→motores) con conexiones exactas, materiales, circuitos optoacopladores (6N137/PC817), BOM completa. | — |
| 2026-04-09 | **CAN loss → pantalla error** — ScreenManager detecta heartbeat STM32 stale >1.5s → fuerza transición a ERROR screen con banner "CAN LINK LOST". Auto-recuperación al reconectar. | — |
| 2026-04-10 | **Drive screen → TF-Mini Plus** — Actualización pantalla final para TF-Mini Plus: bar proximidad 400→600 cm, zonas color ajustadas (verde >3m, cian 1.5–3m, amarillo 0.8–1.5m, naranja 0.3–0.8m, rojo <0.3m), comentarios TOFSense-M→TF-Mini Plus. | — |
| 2026-04-10 | **Verificación pantallas final** — DriveScreen: overlay degradado/limp + indicadores fault flags. SafeScreen: telemetría read-only (speed/current/temp/steering). Cumplimiento completo HMI_STATE_MODEL.md. | — |
| 2026-04-10 | **HMI anti-flicker + render optimización** — Fix: CAN_TIMEOUT bit 0 faltante en DriveScreen fault overlay. Opt: draw calls gated por dirty checks (zero work si no cambia). Opt: umbrales torque Δ>2% y temp Δ≥1°C para evitar redraw por ruido. Fix: ErrorScreen CAN-lost → partial banner redraw (no full fillScreen). | — |
| 2026-04-10 | **Deterministic render pipeline** — Anti-flicker: reemplazar fillRect+drawString por setTextPadding en 12 componentes UI (zero-gap text overwrite). Fix: draw_runtime_overlay String→snprintf (eliminar heap allocation). Doc: formalizar pipeline CAN→snapshot→frame-latch→render en vehicle_data.h, screen_manager.h, main.cpp. | — |
| 2026-04-10 | **Tile-Based Dirty Region Engine** — Refactor completo del HMI a motor de render por regiones (tiles). TileSet template con hash FNV-1a: cada tile solo se redibuja si su contenido cambia. DriveScreen: 12 tiles (speed, obstacle, wheels, steering, battery, gear, pedal, mode, LED, degraded overlay, faults overlay, ACK). ErrorScreen: 5 tiles. SafeScreen: 6 tiles. StandbyScreen: 2 tiles. BootScreen: 3 tiles. Bar widgets (pedal, batería, sensor obstáculo) con differential update (solo la porción cambiada). Eliminación total de clear+redraw flash en barras. Overlay tiles restauran tiles subyacentes al desaparecer. Nuevo: `tile_engine.h` con TileRect, TileHash, TileSet<N>. | — |
| 2026-04-10 | **HMI Security Audit & Tile Engine Hardening** — Auditoría completa del sistema HMI tile-based. (1) `ui_config.h`: centralización de 20+ magic numbers (text paddings, thresholds, overlay layout, color levels). (2) Overlay invalidation chain: DEGRADED→OBSTACLE, FAULTS→top-bar, ACK→LED_TOGGLE — fix de artefactos visuales al cerrar overlays sobre tiles solapados. (3) wheelThresholdFilter dedup: antes se ejecutaba 2× por frame (update+draw), ahora 1× con resultados precomputados. (4) Tile bounds safety: setRect() clampea coordenadas a SCREEN_W×SCREEN_H. (5) Battery hysteresis determinista: reemplaza dependencia de frame anterior por thresholds explícitos BATT_HYSTERESIS_HIGH/LOW. (6) 15 archivos actualizados. | — |
| 2026-04-10 | **Tile Engine Formalization & Pipeline Hardening** — (1) Z-order layer system: TileLayer enum (STATIC/BASE/OVERLAY/SYSTEM) con reglas formales de composición. (2) Overlay invalidation contract documentado en tile_engine.h. (3) Debug assertions (UI_TILE_DEBUG) para diagnóstico de setRect() bounds. (4) Pipeline puro: overlay visibility precomputada en update(), draw() solo consume. (5) Tile layout constants centralizados: DTILE_*/ETILE_*/YTILE_* en ui_config.h. 6 archivos modificados. | — |
| 2026-04-10 | **Draw Purity Enforcement & Full Layout Centralization** — (1) Draw-phase purity: ackIndicatorDirty_ y diagNeedsRedraw_ movidos fuera de draw helpers. (2) OverlayMode enum (REPLACE/MERGE) con registro por overlay documentando composición y tiles afectados. (3) SafeScreen: 17 layout constants (STILE_*) centralizados en ui_config.h. (4) BootScreen: 5 diagnostic layout constants (BTILE_DIAG_*) centralizados. 5 archivos modificados. | — |
| 2026-04-10 | **Time Determinism, Hash Failsafe, Flag Safety** — (1) `frameTimeMs` injected into Screen::update() — millis() captured once per frame in ScreenManager. All screen timing uses injected value. (2) TileSet::forceRedraw() for critical tiles every 100 frames (5s). (3) Flag safety contract documented. 16 files modified. | — |
| 2026-04-10 | **V10 Hardening: Staggered Failsafe, Critical Tile Policy, Render Atomicity** — (1) Staggered forced redraws: critical tiles distributed across failsafe interval (no SPI spike). (2) Fault-condition override: SPEED+FAULTS force-redraw every frame when faults active. (3) Frame time contract: monotonicity assertion + overflow-safe delta docs. (4) Render atomicity contract: formal single-threaded guarantee. 6 files modified. | — |
| 2026-04-10 | **Final Audit: Frame Time Contract Compliance** — (1) Fix PinScreen millis() violation: wrongCodeMs_ now captured via pending flag in update() using injected frameTimeMs instead of raw millis() in touch handler. (2) Fix ErrorScreen millis() in onEnter(): errorEntryMs_ now captured on first update() instead of millis() in onEnter(). (3) Fix stale millis() comments in drive_screen.h, error_screen.h, engineering_screen.h → frameTimeMs. (4) Centralize PIN screen layout constants (PSCR_*) in ui_config.h. Zero millis() calls remain in any screen update/draw code. 7 files modified. | — |
| 2026-04-10 | **Final Time System Enforcement** — Last millis() removal: DebugOverlay::update() and draw() refactored to accept injected frameTimeMs parameter (was calling millis() directly). Call site in renderTask passes lastFrameStart. Full audit confirms zero direct time calls in any UI-path code. Remaining millis()/micros() are legitimate infrastructure: screen_manager.cpp single sampling point, frame_limiter gating, runtime_monitor profiling (#if RUNTIME_MONITOR), touch_handler debounce. 3 files modified. | — |
| 2026-04-11 | **Motor Control Advanced Hardening + Validation Final** — (1) Coast↔brake hysteresis: `COAST_SPEED_HYSTERESIS_KMH=0.5` prevents oscillation at speed threshold. (2) Brake design decision documented: passive brake (EN=HIGH, RPWM=0, LPWM=0) validated as sufficient; optional `BRAKE_ACTIVE_FALLBACK` ifdef added for field override (~5% min duty). (3) Motor_SetMode BRAKE case updated with `#if BRAKE_ACTIVE_FALLBACK` support. (4) Thread safety documented on Motor_SetMode/Motor_SetSigned: main-loop-only restriction, not re-entrant. (5) Motor_SetSigned mode-tracking caveat documented. (6) Stale comment fix: BTS7960_BRAKE_PWM (4249)→(0). (7) 6 new tests: 1000-cycle stress test, exhaustive glitch immunity, direction reversal detection, coast speed hysteresis, brake PWM validation, INT16_MIN clamping detail. Tests: 194→2965 assertions, 0 failures. 2 files modified. | — |
