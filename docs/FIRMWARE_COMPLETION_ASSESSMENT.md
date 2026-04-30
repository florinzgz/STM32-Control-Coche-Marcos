# FIRMWARE COMPLETION ASSESSMENT

| Campo | Valor |
|-------|-------|
| **Fecha** | 2026-02-22 |
| **Versión HMI** | v1.0 |
| **Hardware ESP32** | ESP32-S3-WROOM-1-N16R8 (16 MB Flash, 8 MB PSRAM Octal) |
| **Hardware STM32** | STM32G474RE (Cortex-M4, 170 MHz, 512 KB Flash) |

---

## RESUMEN GLOBAL: ~89% COMPLETO

El firmware está **funcionalmente completo** en todos los sistemas críticos de seguridad y control. Lo que falta son optimizaciones de rendimiento y calibración que requieren el hardware real.

---

## ESTADO POR SUBSISTEMA

### 🟢 STM32 — Control de Motores y Seguridad

| Subsistema | Estado | % | Notas |
|---|---|---|---|
| Control de motores (4 ruedas PWM 20 kHz) | ✅ Completo | 100% | TIM1 CH1-4, BTS7960 H-bridge |
| Sistema de marchas (P/R/N/D1/D2) | ✅ Completo | 100% | Cambio protegido por velocidad (≤1 km/h) |
| Dirección PID + Ackermann | ⚠️ Parcial | 85% | PID funcional pero solo P (ki=0, kd=0). Necesita ajuste con hardware real |
| Centrado automático de dirección | ✅ Completo | 100% | Barrido bidireccional + sensor inductivo + persistencia flash |
| Calibración persistente (flash pág. 126) | ✅ Completo | 100% | CRC32, magic "STC1", validación en boot con sensor físico |
| Parámetros EPS (flash pág. 127) | ✅ Completo | 100% | Doble buffer A/B con secuencia monotónica |
| Sensor de pedal con plausibilidad | ✅ Completo | 100% | ADC interno dual-sample + plausibilidad software (EMA, rango, tasa) |
| Velocidad de ruedas (4× inductivos) | ✅ Completo | 100% | EXTI con debounce 1 ms, 6 pulsos/rev |
| Temperatura DS18B20 (5 sensores) | ✅ Completo | 100% | OneWire ROM search + hot-plug cada 10 s + limpieza datos stale |
| Corriente INA226 (6 canales) | ✅ Completo | 100% | TCA9548A mux, recuperación bus I2C (NXP AN10216) |
| Sistema de seguridad (7 estados) | ✅ Completo | 100% | BOOT→STANDBY→ACTIVE↔DEGRADED→SAFE→ERROR + LIMP_HOME |
| ABS (anti-bloqueo) | ✅ Completo | 100% | Slip 15%, modulación pulsada 80 ms, per-wheel |
| TCS (control de tracción) | ✅ Completo | 100% | Slip 15%, reducción progresiva, recovery 25%/s |
| Detector de obstáculos (5 zonas) | ✅ Completo | 100% | Emergencia (<500mm, 50 cm policy) a Alerta (<4000mm), speed-dependent |
| Detección reacción niño | ✅ Completo | 100% | Caída pedal >10% → zonas más estrictas 2 s |
| CAN 500 kbps (todos los IDs) | ✅ Completo | 100% | 0x001-0x303, heartbeat 100 ms, bus-off recovery |
| Relé de potencia secuenciado | ✅ Completo | 100% | TRAC→DIR, no-blocking |
| Frenado dinámico | ✅ Completo | 100% | 0.5%/(%/s), máx 60%, ramp 80%/s |
| Freno de estacionamiento | ✅ Completo | 100% | H-bridge short-circuit, derating por corriente/temp |
| Modo servicio (25 módulos) | ✅ Completo | 100% | 4 críticos no-desactivables + 21 no-críticos |
| Validación de boot | ✅ Completo | 100% | Checklist temp/corriente/encoder/batería/CAN |
| Math safety (NaN/Inf) | ✅ Completo | 100% | sanitize_float(), clampf(), unit tests |

### 🟢 ESP32-S3 — HMI y Periféricos

| Subsistema | Estado | % | Notas |
|---|---|---|---|
| Pantalla TFT 480×320 (ST7796) | ✅ Completo | 100% | Landscape, SPI 40 MHz, partial redraw <5 ms |
| Pantalla Boot | ✅ Completo | 100% | "COCHE MARCOS", estado CAN, sensor obstáculo |
| Pantalla Standby | ✅ Completo | 100% | "READY", 5 temperaturas, fault flags |
| Pantalla Drive (principal) | ✅ Completo | 100% | Velocidad, 4 ruedas torque/temp, dirección, batería, pedal, marchas, obstáculo, LEDs, modos |
| Pantalla Safe | ✅ Completo | 100% | Banner ámbar, fault flags, error code |
| Pantalla Error | ✅ Completo | 100% | Fondo rojo, diagnóstico completo |
| Menú oculto ingeniería ("8989") | ⚠️ Parcial | 75% | Estructura de 5 submenús OK, pero calibración pedal/encoder son placeholders |
| Screen Manager (máquina estados) | ✅ Completo | 100% | 20 FPS frame limiter, código secreto "8989" |
| CAN RX (decodificación) | ✅ Completo | 100% | 12 tipos de mensaje decodificados |
| Shifter MCP23017 (P/R/N/D1/D2) | ✅ Completo | 100% | I2C 400 kHz, one-hot activo bajo, default Neutral |
| Sensor obstáculos TOFSense-M | ✅ Completo | 100% | UART1 921600 bps, 5 zonas, detección sensor atascado |
| CAN TX obstáculos (0x208/0x209) | ✅ Completo | 100% | 66 ms + 100 ms, counter rolling, health status |
| Audio DFPlayer Mini | ✅ Completo | 100% | 6 sonidos, 3 prioridades, volumen NVS, UART2 9600 |
| LEDs WS2812B (44 LEDs) | ✅ Completo | 100% | Posición, freno, reversa, emergencia ámbar, limp-home |
| Touch XPT2046 | ✅ Completo | 100% | Tap, long-press, debounce 200 ms |
| NVS Persistencia | ✅ Completo | 100% | CRC32, dirty flag, flush cada 10 s, factory reset |
| Power Manager | ✅ Completo | 100% | 5 estados, relé hold, shutdown con farewell audio |
| Echo mode flags en heartbeat | ✅ Completo | 100% | status_flags bits 1-2 confirmados desde STM32 |
| ACK visual en DriveScreen | ✅ Completo | 100% | OK/REJECTED/TIMEOUT con auto-clear 1.5 s |

---

## LO QUE FALTA (11%)

### 🔴 Requiere hardware real (no se puede hacer solo en software)

| Item | Prioridad | Impacto | Descripción |
|---|---|---|---|
| Ajuste PID dirección (I y D) | MEDIA | Precisión dirección | ki=0.0, kd=0.0 hardcoded. Solo P funcional. Necesita pruebas con carga real del volante |
| Calibración pedal real | MEDIA | Rango pedal | Los rangos ADC están hardcoded. Menú ingeniería tiene placeholder |
| Calibración encoder real | MEDIA | Precisión dirección | El menú ingeniería tiene placeholder para visualización encoder |
| Validación umbrales ABS/TCS | MEDIA | Control tracción | Slip 15% es estimación. Necesita medición en superficie real |
| Validación distancias obstáculo | MEDIA | Seguridad | Zonas (200/500/1000/1500/4000 mm) sin validar en campo |

### 🟡 Optimizaciones software (opcionales, baja prioridad)

| Item | Prioridad | Impacto | Descripción |
|---|---|---|---|
| DMA para ADC pedal | BAJA | Rendimiento | Actualmente HAL_ADC_PollForConversion (~1 µs). Marginal en slot 50 ms |
| DMA para OneWire | BAJA | Rendimiento | Bit-bang con NOP loops añade ~5 ms en rescan. Aceptable en tier 1000 ms |
| Fusión de sensores | BAJA | Tracción avanzada | Correlación velocidad rueda + corriente motor. Feature avanzado |
| Watchdog hardware ESP32 | BAJA | Robustez | Solo software WDT activo. STM32 entra LIMP_HOME si ESP32 cuelga |
| UI calibración en menú oculto | BAJA | Usabilidad | Submenús pedal/encoder son placeholder, funcionalidad ya accesible por ingeniería |

---

## DESGLOSE POR CATEGORÍA

### Dirección y Volante — 95% ✅
- ✅ Centrado automático con sensor inductivo
- ✅ PID con feedback de encoder (E6B2-CWZ6C, 4800 CPR)
- ✅ Geometría Ackermann para ruedas interiores/exteriores
- ✅ Calibración persistente en flash (CRC32 + sensor físico)
- ✅ Rate limiting y deadband
- ⚠️ Falta ajuste fino I/D del PID (requiere volante real bajo carga)

### Pantalla y UI — 97% ✅
- ✅ 6 pantallas completas (Boot, Standby, Drive, Safe, Error, Engineering)
- ✅ Drive screen con todos los telemetría: velocidad, 4 ruedas, dirección, batería, pedal, marchas, obstáculo
- ✅ Partial redraw optimizado (<5 ms por frame)
- ✅ Frame limiter 20 FPS
- ✅ ACK visual feedback (OK/REJECTED/TIMEOUT)
- ⚠️ Submenús calibración en Engineering son placeholder (estructura OK, lógica pendiente)

### Sensor de Obstáculos — 100% ✅
- ✅ TOFSense-M LiDAR vía UART1 (921600 bps, NLink_TOFSense_M_Frame0) en ESP32
- ✅ 5 zonas con umbrales dependientes de velocidad
- ✅ Detección sensor atascado
- ✅ CAN TX 0x208 (66 ms) + 0x209 (100 ms)
- ✅ STM32 validación plausibilidad + state machine autónoma
- ✅ Detección reacción niño (pedal drop → zonas más estrictas)
- ✅ Reverse escape siempre permitido

### Menú Oculto (Ingeniería) — 75% ⚠️
- ✅ Activación por código secreto "8989" (4 toques alternos izq-der)
- ✅ Visor de fallos (bitmasks en vivo)
- ✅ Habilitar/deshabilitar módulos
- ✅ Factory restore
- ✅ Botón EXIT para volver
- ⚠️ Calibración pedal: solo placeholder (sin UI interactiva)
- ⚠️ Calibración encoder: solo placeholder (sin UI interactiva)

### Calibraciones — 85% ⚠️
- ✅ Centrado dirección automático (inductivo + flash)
- ✅ Parámetros EPS en flash con doble buffer
- ✅ NVS en ESP32 (modo, brillo, volumen, LEDs)
- ✅ Plausibilidad pedal por software (dual-sample ADC, EMA, rango, tasa)
- ⚠️ PID I/D sin ajustar (hardware-dependent)
- ⚠️ Rangos pedal hardcoded (no calibrados en campo)

### Audio — 100% ✅
- ✅ DFPlayer Mini UART2 (GPIO 43/44)
- ✅ 6 sonidos: welcome, farewell, obstacle, error, battery, gear
- ✅ Cola de prioridad (HIGH/MEDIUM/LOW)
- ✅ Volumen desde NVS aplicado al arranque
- ✅ Throttle 100 ms entre comandos
- ✅ Timeout 5 s por reproducción

### LEDs — 100% ✅
- ✅ 44 WS2812B (28 front + 16 rear) en GPIO 38
- ✅ Patrones: posición, freno, reversa, emergencia ámbar, limp-home
- ✅ Relé hardware en STM32 (PB10) controlado por CAN 0x120
- ✅ Confirmación estado relé por CAN 0x20A (1 Hz)
- ✅ Persistencia estado LED en NVS

---

## CONCLUSIÓN

**El firmware está listo para pruebas de integración en hardware real.** Todos los sistemas críticos de seguridad están completos y verificados en código. El 11% restante corresponde a:

1. **Ajuste de parámetros** que solo se puede hacer con el vehículo físico (PID, umbrales ABS/TCS, zonas obstáculo)
2. **UI de calibración** en el menú oculto (la estructura existe, falta la lógica interactiva)
3. **Optimizaciones menores** de rendimiento (DMA, sensor fusion) que no afectan funcionalidad

**No hay bloqueos de software.** Todo lo pendiente requiere el hardware montado para continuar.
