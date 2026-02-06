# Estado del Proyecto: STM32-Control-Coche-Marcos

**Fecha de actualización:** 2026-02-06  
**MCU:** STM32G474RE (ARM Cortex-M4F, 170 MHz)  
**Referencia:** [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos) (ESP32-S3)

---

## 📐 Arquitectura

Según el [Plan de Separación](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos/blob/main/docs/PLAN_SEPARACION_STM32_CAN.md), el sistema completo se divide en:

- **ESP32-S3 (HMI):** Display, touch, audio, LEDs, menús, detección obstáculos
- **STM32G474RE (Control):** Motores, sensores críticos, seguridad, relés, CAN

Este repositorio implementa el **firmware STM32 de control**.

---

## ✅ Módulos Implementados (vs. FULL-FIRMWARE)

### 1. Control de Motores (`motor_control.c/h`) ✅
| Funcionalidad | Estado | Equivalente FULL-FIRMWARE |
|---|---|---|
| TIM1 PWM @ 20 kHz (4 motores tracción) | ✅ Completo | `src/control/traction.cpp` |
| TIM8 PWM @ 20 kHz (motor dirección) | ✅ Completo | `src/control/steering_motor.cpp` |
| PID dirección con encoder TIM2 | ✅ Completo | `src/control/steering_motor.cpp` |
| Ackermann geometry | ✅ Completo | `src/control/steering_model.cpp` |
| Modos 4x4 / 4x2 | ✅ Completo | `Traction::setMode4x4()` |
| Tank turn (giro sobre eje) | ✅ Completo | `Traction::setAxisRotation()` |
| Emergency stop | ✅ Completo | `Traction::emergencyStop()` |
| Control individual por rueda | ✅ Completo | `Traction::WheelState` |
| Per-wheel PWM wrappers | ✅ Completo | N/A (usa PCA9685 en ESP32) |

### 2. Comunicación CAN (`can_handler.c/h`) ✅
| Funcionalidad | Estado | Equivalente FULL-FIRMWARE |
|---|---|---|
| FDCAN1 @ 500 kbps (CAN 2.0A, 11-bit) | ✅ Completo | Planificado en `PLAN_SEPARACION` |
| Heartbeat bidireccional (100 ms) | ✅ Completo | Heartbeat mutuo |
| Comandos: throttle, steering, mode | ✅ Completo | Contrato CAN sección 6.2 |
| Estado: speed, current, temp, safety, steering | ✅ Completo | Contrato CAN sección 6.2 |
| Diagnóstico: error codes | ✅ Completo | Contrato CAN errores/faults |
| Timeout 250 ms → modo seguro | ✅ Completo | Regla de autoridad 6.3 |
| Estadísticas TX/RX | ✅ Completo | N/A |

### 3. Sensores (`sensor_manager.c/h`) ✅
| Funcionalidad | Estado | Equivalente FULL-FIRMWARE |
|---|---|---|
| 4× sensores rueda (EXTI pulsos → km/h) | ✅ Completo | `src/sensors/wheels.cpp` |
| 1× encoder dirección TIM2 Quadrature | ✅ Completo | `src/input/steering.cpp` |
| 6× INA226 I²C (vía TCA9548A) | ✅ Completo | `src/sensors/current.cpp` |
| 1× Pedal ADC (0-3.3V → 0-100%) | ✅ Completo | `src/input/pedal.cpp` |
| 5× DS18B20 OneWire (temperaturas) | ⚠️ Parcial | `src/sensors/temperature.cpp` |

> **Nota DS18B20:** OneWire bit-bang implementado con Skip ROM (lee 1 sensor). Falta ROM search para direccionamiento individual de los 5 sensores. Ver TODO en sensor_manager.c.

### 4. Seguridad (`safety_system.c/h`) ✅
| Funcionalidad | Estado | Equivalente FULL-FIRMWARE |
|---|---|---|
| ABS (slip > 20% → corte throttle) | ✅ Completo | `src/safety/abs_system.cpp` |
| TCS (slip > 15% → reducción 50%) | ✅ Completo | `src/control/tcs_system.cpp` |
| Protección sobrecorriente (25A) | ✅ Completo | `SafetyManager` |
| Protección sobretemperatura (90°C) | ✅ Completo | `SafetyManager` |
| Timeout CAN → modo seguro | ✅ Completo | Regla autoridad 6.3 |
| Watchdog IWDG (500 ms) | ✅ Completo | `src/system/watchdog.cpp` |
| Emergency stop + fail-safe | ✅ Completo | `SafetyManager` |
| Power down (relés) | ✅ Completo | `src/system/power_mgmt.cpp` |
| Error tracking (enum + set/clear) | ✅ Completo | `include/error_codes.h` |
| **Máquina de estados (BOOT→STANDBY→ACTIVE→SAFE→ERROR)** | ✅ Completo | Plan Separación §6.3 |
| **Validación de comandos ESP32 (throttle, steering, mode)** | ✅ Completo | Plan Separación §6.1 "STM32 decide" |
| **Rate-limiting de dirección (200°/s)** | ✅ Completo | Seguridad funcional |
| **Filtros CAN RX (solo IDs ESP32 válidos)** | ✅ Completo | Plan Separación §6.3 arbitraje |
| **Secuenciación de relés (power-up/power-down)** | ✅ Completo | `src/system/power_mgmt.cpp` |
| **Plausibilidad de sensores (rango, coherencia)** | ✅ Completo | Safety §3.1 |
| **Heartbeat enriquecido (counter, state, fault flags)** | ✅ Completo | Protocolo CAN 0x001 |

### 5. Interrupciones (`stm32g4xx_it.c/h`) ✅
| Funcionalidad | Estado |
|---|---|
| EXTI0/1/2 + EXTI15_10 (sensores rueda) | ✅ Conectados a `Wheel_XX_IRQHandler()` |
| FDCAN1_IT0/IT1 | ✅ Conectados a `HAL_FDCAN_IRQHandler()` |
| TIM1/TIM2 | ✅ Conectados a `HAL_TIM_IRQHandler()` |
| I2C1 EV/ER | ✅ Conectados a `HAL_I2C_XX_IRQHandler()` |
| SysTick | ✅ `HAL_IncTick()` |
| FDCAN RX callback → Safety_UpdateCANRxTime | ✅ Completo |

### 6. Main Loop (`main.c`) ✅
| Funcionalidad | Estado |
|---|---|
| Inicialización periféricos | ✅ Completo |
| Inicialización módulos | ✅ Completo |
| Loop 10ms: safety + steering PID + traction | ✅ Completo |
| Loop 50ms: sensores + pedal | ✅ Completo |
| Loop 100ms: CAN heartbeat + estado | ✅ Completo |
| Loop 1000ms: temperaturas CAN | ✅ Completo |
| Watchdog refresh | ✅ Completo |

### 7. Documentación ✅
| Documento | Estado |
|---|---|
| `README.md` – Visión general y pinout | ✅ Completo |
| `docs/PINOUT.md` – Pinout STM32G474RE | ✅ Completo |
| `docs/CAN_PROTOCOL.md` – Protocolo CAN | ✅ Completo |
| `docs/MOTOR_CONTROL.md` – Control PWM | ✅ Completo |
| `docs/SAFETY_SYSTEMS.md` – ABS/TCS | ✅ Completo |
| `docs/BUILD_GUIDE.md` – Compilación | ✅ Completo |
| `docs/HARDWARE.md` – BOM y hardware | ✅ Completo |
| `docs/ESP32_STM32_CAN_CONNECTION.md` – Cableado CAN | ✅ Completo |
| `docs/QUICK_START.md` – Inicio rápido | ✅ Completo |
| `SETUP.md` – Guía de setup inicial | ✅ Completo |

---

## ❌ Funcionalidades del FULL-FIRMWARE que NO corresponden al STM32

Estos módulos **permanecen en el ESP32 HMI** según la arquitectura de separación:

| Módulo FULL-FIRMWARE | Razón de exclusión |
|---|---|
| `src/hud/` – Display TFT ST7796S 480×320 | HMI: pantalla vía SPI |
| `src/menu/` – Menús interactivos | HMI: interacción usuario |
| `src/audio/` – DFPlayer Mini | HMI: feedback auditivo |
| `src/lighting/` – LEDs WS2812B (28+16) | HMI: indicadores visuales |
| `src/hud/render_engine.cpp` – Motor render sprites | HMI: renderizado gráfico |
| `include/touch_calibration.h` – Calibración touch | HMI: pantalla táctil |
| `src/sensors/obstacle_detection.cpp` – TOFSense LiDAR | HMI: detección obstáculos (envía alertas CAN) |
| `src/safety/obstacle_safety.cpp` – Safety de obstáculos | HMI→CAN: aviso al STM32 |
| `src/logging/` – Sistema de logs | HMI: diagnóstico visual |
| `src/managers/` – Config/EEPROM managers | HMI: persistencia configuración |
| `src/utils/` – Utilidades generales | HMI: helpers internos |
| `src/test/` – Tests funcionales | HMI: validación en pantalla |

---

## ⚠️ Pendiente de Implementar / Mejorar

### Prioridad Alta
- [ ] **DS18B20 ROM Search:** Implementar búsqueda ROM para direccionar individualmente los 5 sensores DS18B20 (actualmente solo lee 1 con Skip ROM)
- [ ] **Generar .ioc en STM32CubeMX:** El archivo `.ioc` actual es placeholder; necesita regenerarse con la configuración real de pines
- [ ] **HAL MSP completo:** `stm32g4xx_hal_msp.c` necesita revisión para coincidir con el pinout definitivo

### Prioridad Media
- [ ] **Frenado regenerativo:** El FULL-FIRMWARE tiene `src/safety/regen_ai.cpp` con frenado regenerativo inteligente; podría portarse al STM32
- [ ] **Limp mode:** El FULL-FIRMWARE tiene `src/system/limp_mode.cpp` para modo degradado; podría añadirse al safety_system
- [ ] **Adaptive cruise:** `src/control/adaptive_cruise.cpp` del FULL-FIRMWARE; requiere datos de obstáculos vía CAN
- [ ] **CRC8 checksum:** Documentado en protocolo CAN pero no implementado (CAN tiene CRC propio, pero añade capa extra)

### Prioridad Baja
- [ ] **I2C recovery:** El FULL-FIRMWARE tiene `include/i2c_recovery.h`; útil si el bus I2C se bloquea
- [ ] **Watchdog window mode:** Usar WWDG además de IWDG para detección más rápida
- [ ] **SystemClock_Config:** Actualmente es stub; necesita configuración PLL real para 170 MHz

### ✅ Recientemente Completado
- [x] **Máquina de estados del sistema** (BOOT→STANDBY→ACTIVE→SAFE→ERROR)
- [x] **Validación de comandos ESP32** (throttle clamp, steering rate-limit, mode-change speed gate)
- [x] **Filtros CAN RX** (solo acepta IDs ESP32 válidos: 0x011, 0x100-0x102)
- [x] **Secuenciación de relés** (Relay_PowerUp/PowerDown con orden y delays)
- [x] **Plausibilidad de sensores** (validación de rango para temperatura, corriente, velocidad)
- [x] **Heartbeat enriquecido** (alive_counter, system_state, fault_flags)

---

## 📊 Métricas del Código

| Métrica | Valor |
|---------|-------|
| **Archivos fuente (.c)** | 6 |
| **Archivos header (.h)** | 6 |
| **Líneas de código (aprox.)** | ~2,500 |
| **Funciones implementadas** | ~65 |
| **Funciones declaradas sin implementar** | 0 |
| **Documentación (archivos .md)** | 12 |

---

## 📁 Estructura del Proyecto

```
STM32-Control-Coche-Marcos/
├── Core/
│   ├── Inc/
│   │   ├── main.h              # Definiciones pines, HAL handles, constantes
│   │   ├── motor_control.h     # Control motores + Ackermann + steering
│   │   ├── can_handler.h       # Protocolo CAN ESP32↔STM32
│   │   ├── sensor_manager.h    # Sensores: ruedas, temp, corriente, pedal
│   │   ├── safety_system.h     # ABS/TCS + protecciones + fail-safe
│   │   ├── stm32g4xx_it.h      # Prototipos ISR
│   │   └── stm32g4xx_hal_conf.h# Configuración HAL
│   └── Src/
│       ├── main.c              # Inicialización + main loop temporizado
│       ├── motor_control.c     # PWM, PID, Ackermann, 4x4/4x2, tank turn
│       ├── can_handler.c       # CAN TX/RX completo con estadísticas
│       ├── sensor_manager.c    # Lectura sensores real (EXTI, I2C, ADC, OneWire)
│       ├── safety_system.c     # ABS/TCS por rueda, overcurrent, overtemp
│       ├── stm32g4xx_it.c      # ISR conectados a handlers de módulos
│       ├── stm32g4xx_hal_msp.c # MSP init (pines AF para periféricos)
│       └── system_stm32g4xx.c  # Configuración reloj sistema
├── docs/                       # 12 documentos técnicos
├── Makefile                    # Build con arm-none-eabi-gcc
├── STM32G474RETX_FLASH.ld     # Linker script
├── startup_stm32g474retx.s    # Startup assembly
└── README.md                   # Documentación principal
```

---

## 🔗 Correspondencia Módulos STM32 ↔ FULL-FIRMWARE

```
STM32 Repository                    FULL-FIRMWARE (ESP32-S3)
────────────────                    ─────────────────────────
motor_control.c  ←────────────────→ src/control/traction.cpp
                                    src/control/steering_motor.cpp
                                    src/control/steering_model.cpp

can_handler.c    ←────────────────→ (nuevo: protocolo CAN definido en
                                    docs/PLAN_SEPARACION_STM32_CAN.md)

sensor_manager.c ←────────────────→ src/sensors/wheels.cpp
                                    src/sensors/current.cpp
                                    src/sensors/temperature.cpp
                                    src/input/pedal.cpp
                                    src/input/steering.cpp

safety_system.c  ←────────────────→ src/safety/abs_system.cpp
                                    src/control/tcs_system.cpp
                                    SafetyManager (src/system/)

main.c           ←────────────────→ src/main.cpp (control loop)
```

---

**Desarrollado por:** florinzgz  
**Licencia:** MIT
