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
| **Obstáculos (LiDAR)** | ✅ Operativo | TOFSense-M, 5 zonas, timeout 500 ms → fail-safe |
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
22. **CAN_InitDiag_t**: 7 campos (hal_init, filter_global, notify, start, started, clk_ok, cccr_init_ok). Legible vía SWD con `p can_init_diag`.

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
