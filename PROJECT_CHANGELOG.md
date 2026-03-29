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
4. **Tolerancia a fallos CAN**: Bus-off recovery (ambos lados), error-passive recovery (ESP32), timeout → LIMP_HOME.
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
| **CAN (TWAI)** | ✅ Operativo (hardened) | Bus-off recovery + error-passive recovery, max 10 intentos |
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

---

## 3. Cambios Recientes (últimos PR)

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
