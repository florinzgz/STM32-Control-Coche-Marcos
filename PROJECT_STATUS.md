# Estado del Proyecto: STM32-Control-Coche-Marcos

**Fecha de actualización:** 2026-02-06  
**MCU:** STM32G474RE (ARM Cortex-M4F, 170 MHz)  
**Referencia:** [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos) (ESP32-S3)

---

## 📐 Arquitectura (Plan de Separación)

El sistema monolítico original (FULL-FIRMWARE en ESP32-S3) se separa en dos MCUs según el [Plan de Separación](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos/blob/main/docs/PLAN_SEPARACION_STM32_CAN.md):

| Rol | MCU | Responsabilidades |
|-----|-----|-------------------|
| **HMI & Supervisión** | ESP32-S3 | Display TFT, touch, audio, LEDs WS2812B, menús, detección obstáculos, logs |
| **Control Determinista & Autoridad de Seguridad** | STM32G474RE | Motores (PWM, timers, encoders), sensores críticos, seguridad (ABS/TCS), relés, CAN |

**Principio clave:** El ESP32 *solicita* (intención del usuario), el STM32 *valida y ejecuta* (realidad física).

Este repositorio implementa el **firmware STM32 de control**.

---

## ✅ Lo que se ha hecho (implementado)

### 1. Control de Motores (`motor_control.c/h`) — COMPLETO ✅

| Funcionalidad | Detalles | Equivalente FULL-FIRMWARE |
|---|---|---|
| TIM1 PWM @ 20 kHz (4 motores tracción) | PA8-PA11, 4 canales, periodo 8499 | `src/control/traction.cpp` |
| TIM8 PWM @ 20 kHz (motor dirección) | PC8, canal 3 | `src/control/steering_motor.cpp` |
| PID dirección con encoder TIM2 | Kp=2.0, Ki=0.1, Kd=0.5, anti-windup | `src/control/steering_motor.cpp` |
| Ackermann geometry | Cálculo ángulos inner/outer, configurable | `src/control/steering_model.cpp` |
| Modos 4x4 / 4x2 (solo delantero) | Toggle dinámico, inhibición ruedas traseras | `Traction::setMode4x4()` |
| Tank turn (giro sobre eje) | Ruedas izquierda/derecha en sentido opuesto | `Traction::setAxisRotation()` |
| Emergency stop | Corte inmediato PWM + disable enables | `Traction::emergencyStop()` |
| Control individual por rueda | 5 wrappers PWM directos (FL/FR/RL/RR/Steer) | N/A (ESP32 usa PCA9685) |
| Estructura WheelState por rueda | demandPct, currentA, tempC, speedKmh, pwm, reverse | `Traction::WheelState` |

### 2. Comunicación CAN (`can_handler.c/h`) — COMPLETO ✅

| Funcionalidad | Detalles | Equivalente FULL-FIRMWARE |
|---|---|---|
| FDCAN1 @ 500 kbps, CAN 2.0A 11-bit | Bit-timing calculado para 170 MHz | Planificado en Plan Separación §6 |
| Heartbeat STM32→ESP32 (0x001, 100 ms) | 4 bytes: alive_counter + system_state + fault_flags | Plan Separación §6.2 |
| Heartbeat ESP32→STM32 (0x011) | Recepción + actualización timestamp | Plan Separación §6.2 |
| CMD Throttle (0x100) | Recepción + **validación** por Safety_ValidateThrottle() | Plan Separación §6.2 |
| CMD Steering (0x101) | Recepción + **rate-limiting** por Safety_ValidateSteering() | Plan Separación §6.2 |
| CMD Mode (0x102) | Recepción + **speed gate** por Safety_ValidateModeChange() | Plan Separación §6.2 |
| Status Speed (0x200, 100 ms) | 4× uint16 little-endian velocidades rueda | Plan Separación §6.2 |
| Status Current (0x201, 100 ms) | 4× uint16 corrientes motor | Plan Separación §6.2 |
| Status Temp (0x202, 1000 ms) | 5× int8 temperaturas DS18B20 | Plan Separación §6.2 |
| Status Safety (0x203, 100 ms) | ABS, TCS, error_code | Plan Separación §6.2 |
| Status Steering (0x204, 100 ms) | angle int16 + calibrated flag | Plan Separación §6.2 |
| Diag Error (0x300) | error_code + subsystem on-demand | Plan Separación §6.2 |
| **Filtros RX hardware** | Solo IDs ESP32 válidos (0x011, 0x100-0x102), rechazo global | Plan Separación §6.3 |
| Estadísticas TX/RX | tx_count, rx_count, tx_errors, rx_errors | N/A |
| Timeout 250 ms → modo seguro | Safety_CheckCANTimeout() en loop 10 ms | Regla autoridad §6.3 |

### 3. Sensores (`sensor_manager.c/h`) — COMPLETO ✅

| Funcionalidad | Detalles | Equivalente FULL-FIRMWARE |
|---|---|---|
| 4× sensores rueda (EXTI→km/h) | Conteo de pulsos EXTI0/1/2/15, cálculo velocidad por delta/tiempo | `src/sensors/wheels.cpp` |
| 1× encoder dirección TIM2 Quadrature | E6B2-CWZ6C 1200 PPR × 4 = 4800 counts/rev, 0.075°/count | `src/input/steering.cpp` |
| 6× INA226 I²C vía TCA9548A | Lectura shunt voltage + bus voltage, multiplexado canal a canal | `src/sensors/current.cpp` |
| 1× Pedal ADC (PA3, ADC1_IN4) | Conversión simple polling, 12-bit, 0-3.3V → 0-100% | `src/input/pedal.cpp` |
| 5× DS18B20 OneWire (PB0) | OneWire bit-bang completo + Search ROM (0xF0) con algoritmo Maxim AN187 + Match ROM (0x55) para lectura individual + CRC8 en ROM y scratchpad. Fallback a Skip ROM si no se descubren sensores | `src/sensors/temperature.cpp` |

### 4. Sistema de Seguridad (`safety_system.c/h`) — COMPLETO ✅

| Funcionalidad | Detalles | Equivalente FULL-FIRMWARE |
|---|---|---|
| **Máquina de estados** | BOOT→STANDBY→ACTIVE→SAFE→ERROR con transiciones controladas | Plan Separación §6.3 |
| **Validación de comandos ESP32** | Safety_ValidateThrottle (clamp 0-100%, ABS/TCS override), Safety_ValidateSteering (rate-limit 200°/s, clamp ±45°), Safety_ValidateModeChange (speed gate <1 km/h) | Plan Separación §6.1 "STM32 decide" |
| **Secuenciación de relés** | Relay_PowerUp (Main→50ms→Traction→20ms→Direction), Relay_PowerDown (inverso) | `src/system/power_mgmt.cpp` |
| **Plausibilidad de sensores** | Temperatura: -40 a 125°C, Corriente: 0-50A, Velocidad: 0-60 km/h | Safety §3.1 |
| ABS per-wheel | Slip >20% → corte throttle, bitmask por rueda, mínimo 2 km/h | `src/safety/abs_system.cpp` |
| TCS per-wheel | Slip >15% → reducción 50%, bitmask por rueda, mínimo 1 km/h | `src/control/tcs_system.cpp` |
| Protección sobrecorriente | >25A → Safety_SetError + SAFE state | `SafetyManager` |
| Protección sobretemperatura | >90°C → Safety_SetError + SAFE state | `SafetyManager` |
| Timeout CAN → SAFE | >250 ms sin heartbeat ESP32 → SAFE, recuperación automática al restaurar | Regla autoridad §6.3 |
| Emergency stop | Corte total + ERROR state + Relay_PowerDown | `SafetyManager` |
| Error tracking | enum Safety_Error_t + set/clear/get | `include/error_codes.h` |
| Fault flags bitmask | 7 bits en heartbeat: CAN_TIMEOUT, TEMP, CURRENT, ENCODER, WHEEL, ABS, TCS | Protocolo CAN 0x001 byte 2 |
| Watchdog IWDG | 500 ms timeout, refresh en main loop | `src/system/watchdog.cpp` |

### 5. Interrupciones (`stm32g4xx_it.c/h`) — COMPLETO ✅

| ISR | Conectado a |
|---|---|
| EXTI0/1/2 (ruedas FL/FR/RL) | `HAL_GPIO_EXTI_IRQHandler()` + `Wheel_XX_IRQHandler()` |
| EXTI15_10 (rueda RR en PB15) | `HAL_GPIO_EXTI_IRQHandler()` + `Wheel_RR_IRQHandler()` |
| FDCAN1_IT0/IT1 | `HAL_FDCAN_IRQHandler()` |
| TIM1_UP_TIM16, TIM2 | `HAL_TIM_IRQHandler()` |
| I2C1_EV, I2C1_ER | `HAL_I2C_XX_IRQHandler()` |
| SysTick | `HAL_IncTick()` |
| `HAL_FDCAN_RxFifo0Callback` | `Safety_UpdateCANRxTime()` |

### 6. HAL MSP (`stm32g4xx_hal_msp.c`) — COMPLETO ✅

Configuración de pines Alternate Function para cada periférico:
- FDCAN1: PB8/PB9 AF9
- I2C1: PB6/PB7 AF4
- TIM1 PWM: PA8-PA11 AF6
- TIM8 PWM: PC8 AF4
- TIM2 Encoder: PA15 AF1, PB3 AF1
- ADC1: PA3 modo analógico
- NVIC priorities configuradas

### 7. Main Loop (`main.c`) — COMPLETO ✅

| Tarea | Período | Funciones |
|---|---|---|
| Safety + Steering PID + Traction | 10 ms (100 Hz) | ABS, TCS, CheckCurrent, CheckTemp, CheckCAN, CheckSensors, Steering_ControlLoop, Traction_Update |
| Sensors + Pedal | 50 ms (20 Hz) | Pedal_Update, Current_ReadAll, Temperature conversión/lectura, validación + Traction_SetDemand |
| CAN heartbeat + status | 100 ms (10 Hz) | SendHeartbeat, SendStatusSpeed/Current/Safety/Steering |
| Temperaturas CAN | 1000 ms (1 Hz) | SendStatusTemp |
| CAN RX | Continuo | CAN_ProcessMessages() |
| Watchdog | Continuo | HAL_IWDG_Refresh() |

### 8. Infraestructura de Build — COMPLETO ✅

- **Makefile** corregido con todos los archivos fuente reales + drivers HAL
- **Linker script** (STM32G474RETX_FLASH.ld): 512KB Flash + 128KB RAM
- **Startup assembly** (startup_stm32g474retx.s)
- **STM32CubeMX .ioc** completo: 36 pines configurados (PA0-PA3, PB0/PB3/PB6-PB9/PB15, PC0-PC13), ADC1_IN4 (PA3), EXTI IRQs, FDCAN/I2C/IWDG params, RCC PLL 170 MHz
- **Scripts de setup** (setup_drivers.sh, setup_drivers.bat)

### 9. Documentación — 12 archivos ✅

| Documento | Contenido |
|---|---|
| `README.md` | Visión general, pinout completo, protocolo CAN, features |
| `docs/PINOUT.md` | Asignación de todos los pines STM32G474RE |
| `docs/PINOUT_DEFINITIVO.md` | Pinout definitivo validado |
| `docs/CAN_PROTOCOL.md` | Protocolo CAN completo con IDs, formatos, timing |
| `docs/PROTOCOLO_CAN.md` | Protocolo CAN en español |
| `docs/MOTOR_CONTROL.md` | PWM, PID, Ackermann, modos tracción |
| `docs/SAFETY_SYSTEMS.md` | ABS, TCS, protecciones, watchdog, códigos error |
| `docs/BUILD_GUIDE.md` | Instrucciones compilación |
| `docs/HARDWARE.md` | BOM y especificaciones hardware |
| `docs/HARDWARE_SPECIFICATION.md` | Especificaciones técnicas detalladas |
| `docs/ARQUITECTURA_CONTROL_MOTORES.md` | Arquitectura del control de motores |
| `docs/ESP32_STM32_CAN_CONNECTION.md` | Cableado físico CAN con TJA1051T/3 |
| `docs/QUICK_START.md` | Guía de inicio rápido |
| `SETUP.md` | Guía de setup completa |

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

## ⚠️ Lo que falta por implementar

### 🔴 Prioridad Alta — ✅ COMPLETADO

| Tarea | Estado | Descripción |
|---|---|---|
| **SystemClock_Config** | ✅ Implementado | HSI 16 MHz → PLL (PLLM=/4, PLLN=85, PLLR=/2) → 170 MHz SYSCLK. Voltage scaling BOOST, Flash 8 wait states. |
| **DS18B20 ROM Search** | ✅ Implementado | Search ROM (0xF0) con algoritmo Maxim AN187 para descubrir hasta 5 sensores. Match ROM (0x55) + CRC8 para lectura individual. Fallback a Skip ROM si no se descubren sensores. |
| **Actualizar .ioc** | ✅ Implementado | Todos los 36 pines configurados (PA0-PA3 wheel/pedal, PB0 OneWire, PB15 wheel RR, PC0-PC13 dir/en/relay). ADC corregido a IN4 (PA3). EXTI IRQs añadidos. FDCAN/I2C/IWDG params incluidos. |

### 🟡 Prioridad Media (mejoras funcionales del FULL-FIRMWARE)

| Tarea | Descripción | Fuente FULL-FIRMWARE |
|---|---|---|
| **Frenado regenerativo** | Frenado inteligente con IA que ajusta la fuerza de frenado regenerativo según velocidad, batería y superficie | `src/safety/regen_ai.cpp` (7.5 KB) |
| **Limp mode (modo degradado)** | Cuando falla un motor o sensor, permite conducción limitada en vez de parada completa | `src/system/limp_mode.cpp` (11.2 KB) |
| **Adaptive cruise** | Control de crucero adaptativo que ajusta velocidad según obstáculos detectados (requiere datos CAN del ESP32) | `src/control/adaptive_cruise.cpp` (5.5 KB) |
| **CRC8 en protocolo CAN** | Documentado en el protocolo pero no implementado. CAN tiene CRC propio a nivel hardware, pero el CRC8 por software añade una capa extra de detección de corrupción en la capa de aplicación | docs/CAN_PROTOCOL.md |
| **I2C recovery** | Mecanismo de recuperación si el bus I2C se bloquea (SDA queda low). Hace bit-banging de SCL para desbloquear | `include/i2c_recovery.h` en FULL-FIRMWARE |

### 🟢 Prioridad Baja (optimizaciones)

| Tarea | Descripción |
|---|---|
| **WWDG (Window Watchdog)** | Añadir WWDG además de IWDG para detección más rápida de cuelgues en el loop |
| **DMA para ADC** | Actualmente el pedal se lee con polling. Con DMA sería no-bloqueante |
| **DMA para I2C** | Las lecturas INA226 bloquean 50 ms por sensor. Con DMA serían asíncronas |
| **Temperature derating** | Reducir potencia gradualmente entre 60-80°C (actualmente solo actúa a 90°C) |
| **Power management avanzado** | Secuencia de encendido completa con verificación de voltaje de batería como en `src/system/power_mgmt.cpp` |
| **Error log persistente** | Guardar errores en Flash para diagnóstico post-mortem |

---

## 📊 Métricas del Código

| Métrica | Valor |
|---------|-------|
| **Archivos fuente (.c)** | 8 |
| **Archivos header (.h)** | 7 |
| **Líneas de código C (aprox.)** | ~2,800 |
| **Funciones implementadas** | ~70 |
| **Funciones declaradas sin implementar** | 0 |
| **Documentación (archivos .md)** | 14+ |
| **Módulos CAN TX** | 8 tipos de mensaje |
| **Módulos CAN RX** | 4 tipos de mensaje (filtrado HW) |
| **Periféricos configurados** | 7 (FDCAN, I2C, TIM1, TIM2, TIM8, ADC1, IWDG) |

---

## 📁 Estructura del Proyecto

```
STM32-Control-Coche-Marcos/
├── Core/
│   ├── Inc/
│   │   ├── main.h              # Definiciones pines, HAL handles, constantes
│   │   ├── motor_control.h     # Control motores + Ackermann + steering PID
│   │   ├── can_handler.h       # Protocolo CAN ESP32↔STM32 (IDs, funciones)
│   │   ├── sensor_manager.h    # Sensores: ruedas, temp, corriente, pedal
│   │   ├── safety_system.h     # ABS/TCS + state machine + validación + fail-safe
│   │   ├── stm32g4xx_it.h      # Prototipos ISR
│   │   └── stm32g4xx_hal_conf.h# Configuración HAL
│   └── Src/
│       ├── main.c              # Init periféricos + main loop temporizado
│       ├── motor_control.c     # PWM, PID, Ackermann, 4x4/4x2, tank turn
│       ├── can_handler.c       # CAN TX/RX + filtros + validación seguridad
│       ├── sensor_manager.c    # EXTI ruedas, I2C INA226, ADC pedal, OneWire DS18B20
│       ├── safety_system.c     # State machine, ABS/TCS, command gate, relay sequencing
│       ├── stm32g4xx_it.c      # ISR → handlers de módulos
│       ├── stm32g4xx_hal_msp.c # MSP init (pines AF para periféricos)
│       └── system_stm32g4xx.c  # Configuración reloj sistema
├── docs/                       # 12+ documentos técnicos
├── Makefile                    # Build con arm-none-eabi-gcc (corregido)
├── STM32G474RETX_FLASH.ld     # Linker script (512KB Flash, 128KB RAM)
├── startup_stm32g474retx.s    # Startup assembly
├── SETUP.md                    # Guía de setup completa
└── README.md                   # Documentación principal
```

---

## 🔗 Correspondencia STM32 ↔ FULL-FIRMWARE

```
STM32 Repository                    FULL-FIRMWARE (ESP32-S3)
────────────────                    ─────────────────────────
motor_control.c  ←────────────────→ src/control/traction.cpp      (27 KB)
                                    src/control/steering_motor.cpp (8.5 KB)
                                    src/control/steering_model.cpp (2 KB)

can_handler.c    ←────────────────→ (nuevo: protocolo CAN definido en
                                    docs/PLAN_SEPARACION_STM32_CAN.md)

sensor_manager.c ←────────────────→ src/sensors/wheels.cpp        (4.5 KB)
                                    src/sensors/current.cpp        (11 KB)
                                    src/sensors/temperature.cpp    (9.7 KB)
                                    src/input/pedal.cpp
                                    src/input/steering.cpp

safety_system.c  ←────────────────→ src/safety/abs_system.cpp     (5.8 KB)
                                    src/control/tcs_system.cpp     (8.5 KB)
                                    SafetyManager (src/system/)
                                    src/system/power_mgmt.cpp      (8.7 KB)

main.c           ←────────────────→ src/main.cpp                  (14.3 KB)

❌ No portado (queda en ESP32):
                                    src/safety/regen_ai.cpp        (7.5 KB)
                                    src/system/limp_mode.cpp       (11.2 KB)
                                    src/control/adaptive_cruise.cpp(5.5 KB)
                                    src/control/relays.cpp         (10.5 KB) — relay sequencing ya integrado
```

---

**Desarrollado por:** florinzgz  
**Licencia:** MIT
