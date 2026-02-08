# AUDITORÍA TÉCNICA COMPLETA — STM32-Control-Coche-Marcos

**Fecha**: 2026-02-08
**Alcance**: Firmware STM32G474RE completo (9 archivos fuente `.c`, 7 headers `.h`)
**Referencia obligatoria**: `github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos` (firmware base ESP32-S3)
**Metodología**: Análisis línea por línea del código fuente real. Toda afirmación cita archivo, función y línea.

---

## 1) ¿ESTÁ EL FIRMWARE LISTO PARA MODO STANDALONE (sin ESP32-S3)?

**NO. El firmware NO puede funcionar en modo standalone.**

### 1.1 Arranque seguro

El arranque es seguro. La secuencia `main.c:43-67` ejecuta:
1. `HAL_Init()` → configura SysTick y NVIC
2. `SystemClock_Config()` → HSI 16 MHz → PLL → 170 MHz SYSCLK
3. Inicialización de todos los periféricos (GPIO, FDCAN, I2C, TIM1/2/8, ADC1, IWDG)
4. Inicialización de módulos (`Motor_Init`, `Traction_Init`, `Steering_Init`, `Sensor_Init`, `Safety_Init`, `CAN_Init`)
5. Transición `BOOT → STANDBY`

Si cualquier `HAL_*_Init()` falla, `Error_Handler()` (`main.c:384-391`) deshabilita IRQs y pone todos los pines GPIOC (enables + relés) a LOW vía BSRR directo.

Los fault handlers (`stm32g4xx_it.c:21-48`) para HardFault, MemManage, BusFault y UsageFault ejecutan la misma acción: GPIOC LOW + bucle infinito.

**Resultado: ARRANQUE SEGURO — ✅ OK**

### 1.2 Relés

Implementados en `safety_system.c:118-135`:
- `Relay_PowerUp()`: Main (PC10) → `HAL_Delay(50ms)` → Traction (PC11) → `HAL_Delay(20ms)` → Direction (PC12)
- `Relay_PowerDown()`: Dir → Trac → Main (orden inverso, sin delays)
- `Relay_PowerUp()` se ejecuta SOLO durante la transición a `SYS_STATE_ACTIVE` (`safety_system.c:74`)

**Resultado: RELÉS OK pero solo se activan al entrar en ACTIVE — ✅ hardware OK, ⛔ depende de ACTIVE**

### 1.3 Motores de tracción (24V)

`motor_control.c:123-151` configura 4 motores (FL/FR/RL/RR) en TIM1 CH1-CH4 con PWM 20 kHz.
`Traction_Update()` (`motor_control.c:243-289`) aplica demanda como PWM.
`Traction_SetDemand()` (`motor_control.c:189-231`) incluye filtro EMA (alpha=0.15) y ramp rate limiter (50 %/s subida, 100 %/s bajada).

La demanda solo se alimenta en `main.c:106-111` si `Safety_IsCommandAllowed()` retorna `true`, lo cual requiere `system_state == SYS_STATE_ACTIVE`.

**Resultado: MOTORES OK pero solo operan en estado ACTIVE — ✅ hardware OK, ⛔ depende de ACTIVE**

### 1.4 Motor de dirección (12V)

`motor_control.c:141-143` configura motor steering en TIM8 CH3 con PWM 20 kHz.
`Steering_ControlLoop()` (`motor_control.c:344-399`) ejecuta PID con Kp=0.09 (P-only, equivalente a 1.2 en espacio de grados del firmware base).
Deadband de 0.5° (`STEERING_DEADBAND_COUNTS`). Anti-windup ±1000.
Protección ante fallo de encoder: si `enc_fault` es 1, se llama a `Steering_Neutralize()` (PWM=0, enable=0).

**Resultado: DIRECCIÓN OK — ✅ funcional independientemente de CAN**

### 1.5 Pedal

`sensor_manager.c:75-83` (`Pedal_Update()`) lee ADC1 CH4 (PA3) con polling bloqueante (10ms timeout).
Escalado: `raw * 100.0 / 4095.0` → porcentaje.
Se llama cada 50 ms desde `main.c:98`.

La señal del pedal se valida (`Safety_ValidateThrottle`) y se aplica a la tracción SOLO si `Safety_IsCommandAllowed()` (`main.c:106-111`).

**Resultado: PEDAL SE LEE SIEMPRE, pero su valor no llega a los motores sin estado ACTIVE — ✅ lectura OK, ⛔ control bloqueado**

### 1.6 Palanca de cambios (P / N / D1 / D2 / R)

**NO IMPLEMENTADO EN ESTE FIRMWARE.**

En el firmware base (`src/input/shifter.cpp`), la palanca se lee vía MCP23017 I/O expander (pines GPIOB0-B4, bus I2C). Implementa debounce de 50 ms, protección de reversa a >3 km/h, y prioridad P>R>N>D1>D2.

En el firmware STM32 actual:
- No existe ningún archivo `shifter.c` ni `shifter.h`
- No existe ningún código que lea posición de palanca
- No hay enumeración de marchas (P/R/N/D1/D2)
- No hay MCP23017 I/O expander en el bus I2C (solo TCA9548A + INA226)
- La dirección de los motores se controla exclusivamente por el signo de `demandPct` en `Traction_SetDemand()`

**Resultado: NO IMPLEMENTADO / NO EXISTE EN CÓDIGO**

### 1.7 Seguridad (SAFE / ERROR / watchdog)

**Implementada completamente:**

- **Máquina de estados** (`safety_system.c:55-95`): BOOT→STANDBY→ACTIVE⇄SAFE→ERROR
- **Protección sobrecorriente** (`safety_system.c:294-304`): >25A por INA226 → SAFE
- **Protección sobretemperatura** (`safety_system.c:308-318`): >80°C por DS18B20 → SAFE
- **Plausibilidad sensores** (`safety_system.c:350-385`): temp fuera de -40°C/125°C, corriente negativa o >50A, velocidad >60 km/h → SAFE
- **Salud encoder** (`safety_system.c:398-410`): range, jump, frozen → SAFE vía `Encoder_CheckHealth()` y `Encoder_HasFault()`
- **ABS** (`safety_system.c:216-248`): slip >15%, avg >10 km/h → corta throttle a 0%
- **TCS** (`safety_system.c:255-287`): slip >15%, avg >3 km/h → reduce throttle 50%
- **Emergency stop** (`safety_system.c:414-423`): motores off + relés off + ERROR
- **FailSafe** (`safety_system.c:425-440`): motores off + dirección a 0° (o neutralize si encoder faulted)
- **IWDG** (`main.c:373-382`): ~500 ms timeout, refresh en bucle principal

**Resultado: SEGURIDAD COMPLETA — ✅ OK**

### 1.8 Comportamiento sin pantalla ni CAN HMI

Sin ESP32 conectado, la secuencia es:
1. `main.c:67`: sistema entra en `SYS_STATE_STANDBY`
2. `main.c:87`: `Safety_CheckCANTimeout()` se ejecuta cada 10 ms
3. `safety_system.c:324`: `(HAL_GetTick() - last_can_rx_time) > 250` → verdadero (nunca se recibe heartbeat)
4. `safety_system.c:325`: `Safety_SetError(SAFETY_ERROR_CAN_TIMEOUT)` + `Safety_SetState(SYS_STATE_SAFE)`
5. `safety_system.c:83`: la transición a SAFE llama a `Safety_FailSafe()` → motores off
6. El sistema queda PERMANENTEMENTE en SAFE con error CAN_TIMEOUT
7. `Safety_IsCommandAllowed()` retorna `false` → `main.c:109`: `Traction_SetDemand(0.0f)` siempre

La auto-transición STANDBY→ACTIVE (`safety_system.c:329-331`) requiere heartbeat CAN del ESP32.
La recuperación SAFE→ACTIVE (`safety_system.c:334-338`) requiere heartbeat CAN del ESP32 + error == CAN_TIMEOUT.

**No existe ningún mecanismo para alcanzar SYS_STATE_ACTIVE sin ESP32.**

**Resultado: ⛔ BLOQUEANTE — el vehículo NO puede moverse sin ESP32**

---

## 2) FUNCIONALIDADES IMPLEMENTADAS Y ACTIVAS

### 2.1 Tracción

| Funcionalidad | Archivo | Función | Estado |
|---------------|---------|---------|--------|
| PWM 4 motores 20 kHz | `main.c:282-303`, `motor_control.c:145-150` | `MX_TIM1_Init()`, `Motor_Init()` | ✅ ACTIVO |
| Modo 4x2 (delanteros) | `motor_control.c:261-267` | `Traction_Update()` | ✅ ACTIVO (por defecto) |
| Modo 4x4 | `motor_control.c:255-260` | `Traction_Update()` | ✅ ACTIVO (vía CAN 0x102) |
| Tank turn | `motor_control.c:249-254` | `Traction_Update()` | ✅ ACTIVO (vía CAN 0x102) |
| Demanda con EMA filter | `motor_control.c:196-204` | `Traction_SetDemand()` | ✅ ACTIVO (alpha=0.15) |
| Rampa de aceleración | `motor_control.c:206-228` | `Traction_SetDemand()` | ✅ ACTIVO (50 %/s subida, 100 %/s bajada) |
| Emergency stop | `motor_control.c:291-309` | `Traction_EmergencyStop()` | ✅ ACTIVO |
| Validación cambio de modo | `safety_system.c:181-192` | `Safety_ValidateModeChange()` | ✅ ACTIVO (solo <1 km/h) |
| Per-motor enable/disable | `motor_control.c:269-277` | `Traction_Update()` | ✅ ACTIVO |

### 2.2 Dirección

| Funcionalidad | Archivo | Función | Estado |
|---------------|---------|---------|--------|
| PWM steering 20 kHz | `main.c:330-349`, `motor_control.c:141-143` | `MX_TIM8_Init()`, `Motor_Init()` | ✅ ACTIVO |
| Encoder cuadratura TIM2 | `main.c:306-328` | `MX_TIM2_Init()` | ✅ ACTIVO (E6B2-CWZ6C, 4800 CPR) |
| PID control loop | `motor_control.c:344-399` | `Steering_ControlLoop()` | ✅ ACTIVO (Kp=0.09, P-only, 100 Hz) |
| Deadband 0.5° | `motor_control.c:92` | `STEERING_DEADBAND_COUNTS` | ✅ ACTIVO |
| Clamp ±54° | `motor_control.c:327-328` | `Steering_SetAngle()` | ✅ ACTIVO |
| Rate-limit 200°/s | `safety_system.c:157-179` | `Safety_ValidateSteering()` | ✅ ACTIVO |
| Ackermann wheel angles | `ackermann.c:20-60`, `motor_control.c:338` | `Ackermann_ComputeWheelAngles()` | ✅ ACTIVO (llamado desde `Steering_SetAngle`) |
| Steering neutralize | `motor_control.c:492-499` | `Steering_Neutralize()` | ✅ ACTIVO (PWM=0, enable=0, PID reset) |
| Encoder health monitoring | `motor_control.c:430-478` | `Encoder_CheckHealth()` | ✅ ACTIVO (range, jump, frozen checks) |
| Calibración (set zero) | `motor_control.c:170-183` | `Steering_Init()` | ✅ ACTIVO (zero at current position) |

### 2.3 Seguridad

| Funcionalidad | Archivo | Función | Estado |
|---------------|---------|---------|--------|
| State machine (5 estados) | `safety_system.c:55-95` | `Safety_SetState()` | ✅ ACTIVO |
| Command gate (solo ACTIVE) | `safety_system.c:97-100` | `Safety_IsCommandAllowed()` | ✅ ACTIVO |
| Overcurrent >25A → SAFE | `safety_system.c:294-304` | `Safety_CheckCurrent()` | ✅ ACTIVO |
| Overtemp >80°C → SAFE | `safety_system.c:308-318` | `Safety_CheckTemperature()` | ✅ ACTIVO |
| CAN timeout 250ms → SAFE | `safety_system.c:322-340` | `Safety_CheckCANTimeout()` | ✅ ACTIVO |
| Sensor plausibility → SAFE | `safety_system.c:350-385` | `Safety_CheckSensors()` | ✅ ACTIVO |
| Encoder fault → SAFE | `safety_system.c:398-410` | `Safety_CheckEncoder()` | ✅ ACTIVO |
| ABS (slip >15%, >10 km/h) | `safety_system.c:216-248` | `ABS_Update()` | ✅ ACTIVO |
| TCS (slip >15%, >3 km/h) | `safety_system.c:255-287` | `TCS_Update()` | ✅ ACTIVO |
| Throttle validation | `safety_system.c:141-155` | `Safety_ValidateThrottle()` | ✅ ACTIVO |
| Steering validation | `safety_system.c:157-179` | `Safety_ValidateSteering()` | ✅ ACTIVO |
| Emergency stop | `safety_system.c:414-423` | `Safety_EmergencyStop()` | ✅ ACTIVO |
| Fail-safe | `safety_system.c:425-440` | `Safety_FailSafe()` | ✅ ACTIVO |
| Relay power sequencing | `safety_system.c:118-135` | `Relay_PowerUp/Down()` | ✅ ACTIVO |
| Fault flags bitmask | `safety_system.c:102-112` | `Safety_GetFaultFlags()` | ✅ ACTIVO |
| IWDG ~500 ms | `main.c:373-382` | `MX_IWDG_Init()` | ✅ ACTIVO |
| HardFault/BusFault/MemManage/UsageFault → GPIOC LOW | `stm32g4xx_it.c:21-48` | fault handlers | ✅ ACTIVO |
| Error_Handler → GPIOC LOW | `main.c:384-391` | `Error_Handler()` | ✅ ACTIVO |

### 2.4 Sensores

| Funcionalidad | Archivo | Función | Estado |
|---------------|---------|---------|--------|
| 4× Wheel speed (EXTI pulses) | `sensor_manager.c:22-63` | `Wheel_ComputeSpeed()` | ✅ ACTIVO (6 pulses/rev, 1.1m circumference) |
| Pedal ADC (PA3, 12-bit) | `sensor_manager.c:75-86` | `Pedal_Update()` | ✅ ACTIVO |
| 6× INA226 current (I2C via TCA9548A) | `sensor_manager.c:92-152` | `Current_ReadAll()` | ✅ ACTIVO (shunt 1 mΩ) |
| 5× DS18B20 temperature (OneWire PB0) | `sensor_manager.c:154-438` | `Temperature_ReadAll()` | ✅ ACTIVO (ROM Search + CRC-8 + Match ROM) |
| Bus voltage per INA226 | `sensor_manager.c:139-141` | internal to `Current_ReadAll()` | ✅ READ (but `Voltage_GetBus()` never called) |

### 2.5 Estado del sistema

| Funcionalidad | Archivo | Función | Estado |
|---------------|---------|---------|--------|
| CAN heartbeat TX (100 ms) | `can_handler.c:131-150` | `CAN_SendHeartbeat()` | ✅ ACTIVO (ID 0x001) |
| CAN speed status TX (100 ms) | `can_handler.c:152-166` | `CAN_SendStatusSpeed()` | ✅ ACTIVO (ID 0x200) |
| CAN current status TX (100 ms) | `can_handler.c:168-182` | `CAN_SendStatusCurrent()` | ✅ ACTIVO (ID 0x201) |
| CAN temp status TX (1000 ms) | `can_handler.c:184-195` | `CAN_SendStatusTemp()` | ✅ ACTIVO (ID 0x202) |
| CAN safety status TX (100 ms) | `can_handler.c:197-205` | `CAN_SendStatusSafety()` | ✅ ACTIVO (ID 0x203) |
| CAN steering status TX (100 ms) | `can_handler.c:207-216` | `CAN_SendStatusSteering()` | ✅ ACTIVO (ID 0x204) |
| CAN RX filter (hardware) | `can_handler.c:75-102` | `CAN_ConfigureFilters()` | ✅ ACTIVO |
| CAN command processing | `can_handler.c:243-303` | `CAN_ProcessMessages()` | ✅ ACTIVO |
| CAN statistics | `can_handler.c:19` | `can_stats` | ✅ ACTIVO |

---

## 3) FUNCIONALIDADES IMPORTANTES: ANÁLISIS DE BRECHAS

### 3a) Existen en firmware base pero NO implementadas aquí

| Funcionalidad base | Archivo base | Estado en STM32 |
|-------------------|-------------|-----------------|
| **Palanca de cambios (P/R/N/D1/D2)** | `src/input/shifter.cpp` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. El firmware base lee 5 posiciones vía MCP23017 con debounce, protección de reversa a >3 km/h, prioridad P>R>N>D1>D2, y audio feedback. |
| **Botones (inputs)** | `src/input/buttons.cpp` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. |
| **Detección de obstáculos** | `src/sensors/obstacle_detection.cpp` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. |
| **Seguridad de obstáculos** | `src/safety/obstacle_safety.cpp` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. |
| **Frenado regenerativo** | `src/safety/regen_ai.cpp` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. |
| **Cruise control adaptativo** | `src/control/adaptive_cruise.cpp` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. |
| **Modo limp (degradado)** | `src/system/limp_mode.cpp` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. |
| **Gestión de energía** | `src/system/power_mgmt.cpp` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. Relay control existe pero no monitoreo de batería (voltaje min/max, sobrecorriente batería). |
| **HUD / Display** | `src/hud/` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. El STM32 no tiene pantalla conectada. |
| **Audio** | `src/audio/` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. |
| **Iluminación** | `src/lighting/` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. |
| **Boot guard / bootloop** | `src/core/`, `boot_guard.h` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. |
| **MCP23017 I/O expander** | `mcp23017_manager.h` | NO IMPLEMENTADO / NO EXISTE EN CÓDIGO. |
| **Secuencia relés no bloqueante** | `src/control/relays.cpp` | PARCIAL. El firmware base usa secuencia no bloqueante con máquina de estados (`SEQ_EN_ENABLE_MAIN`, etc.). El STM32 usa `HAL_Delay()` bloqueante. |
| **Monitoreo batería** | `src/control/relays.cpp:update()` | NO IMPLEMENTADO. El firmware base verifica sobrecorriente batería (>120A), voltaje bajo (<20V) y alto (>30V). |
| **Detección stall motor** | Implícito en firmware base (corriente alta + velocidad cero) | `SAFETY_ERROR_MOTOR_STALL` definido como enum pero NUNCA ACTIVADO. No existe lógica de detección. |

### 3b) Implementadas pero NO conectadas / no usadas

| Funcionalidad | Archivo | Detalle |
|--------------|---------|---------|
| **`Ackermann_Compute()`** | `motor_control.c:505-530` | Función completa con fórmula correcta. Se llama internamente en el cálculo, pero `Ackermann_SetGeometry()` nunca se invoca externamente. `Ackermann_Compute()` no se usa desde ningún punto fuera de `motor_control.c`. Nota: `Ackermann_ComputeWheelAngles()` en `ackermann.c` SÍ se llama desde `Steering_SetAngle()`. |
| **`CAN_SendError()`** | `can_handler.c:218-225` | Función implementada (ID 0x300, 2 bytes) pero nunca llamada desde ningún módulo. |
| **`Voltage_GetBus()`** | `sensor_manager.c:149-152` | Función implementada y datos leídos de INA226 cada 50 ms, pero nunca consumidos. |
| **`Wheel_GetRPM_FL()`** | `sensor_manager.c:64` | Solo FL tiene getter público. FR/RL/RR RPM se calculan pero no tienen accesores. |
| **`SAFETY_ERROR_MOTOR_STALL`** | `safety_system.h:26` | Enum definido (valor 5) pero ninguna función activa este error. |
| **`SAFETY_ERROR_EMERGENCY_STOP`** | `safety_system.h:27` | Enum definido (valor 6) pero `Safety_EmergencyStop()` va directo a `SYS_STATE_ERROR` sin llamar `Safety_SetError()`. |
| **`SAFETY_ERROR_WATCHDOG`** | `safety_system.h:28` | Enum definido (valor 7) pero IWDG hace reset hardware directo, no pasa por la máquina de estados. |
| **`PIN_ENC_Z`** | `main.h:52` | Pin PB4 definido para pulso Z del encoder E6B2-CWZ6C. No hay GPIO init, no hay EXTI handler, no se usa para calibración. |

### 3c) Documentadas pero NO existen en código

| Documentación | Referencia | Estado real |
|---------------|-----------|-------------|
| Calibración ADC | `HAL_ADCEx_Calibration_Start()` recomendada por RM0440 | NO EXISTE EN CÓDIGO. No se llama en ningún punto antes de usar ADC1. |
| Derating por temperatura | Reducción gradual de potencia entre 70-80°C | NO EXISTE EN CÓDIGO. La protección es binaria: <80°C → OK, >80°C → SAFE total. |
| I2C bus recovery | Secuencia de clock pulses para desbloquear SDA | NO EXISTE EN CÓDIGO. Si I2C se bloquea, `TCA9548A_SelectChannel()` falla silenciosamente. |
| DMA para sensores | Lecturas no bloqueantes vía DMA | NO EXISTE EN CÓDIGO. Toda lectura ADC e I2C es polling bloqueante. |
| WWDG (window watchdog) | Segundo watchdog con ventana temporal | NO EXISTE EN CÓDIGO. Solo IWDG configurado. |
| Persistent logging | Escritura de errores a Flash/EEPROM | NO EXISTE EN CÓDIGO. Toda información se pierde al reset. |

---

## 4) BLOQUEADORES REALES PARA MODO STANDALONE

### 🔴 BLOQUEADOR CRÍTICO #1: CAN timeout impide estado ACTIVE

**Bloquea: MOVIMIENTO + CONTROL**

Sin ESP32:
1. Sistema arranca en BOOT → transiciona a STANDBY (`main.c:67`)
2. `Safety_CheckCANTimeout()` (`safety_system.c:324`) detecta ausencia de heartbeat tras 250 ms
3. `Safety_SetError(SAFETY_ERROR_CAN_TIMEOUT)` + `Safety_SetState(SYS_STATE_SAFE)` (`safety_system.c:325-326`)
4. `Safety_FailSafe()` → `Traction_EmergencyStop()` → todos los motores a 0 + deshabilitados
5. El sistema NUNCA alcanza `SYS_STATE_ACTIVE`
6. `Safety_IsCommandAllowed()` retorna SIEMPRE `false`
7. `main.c:109`: `Traction_SetDemand(0.0f)` se ejecuta siempre
8. Los relés NUNCA se activan (`Relay_PowerUp()` solo se llama al entrar en ACTIVE)

No existe ningún mecanismo alternativo para alcanzar ACTIVE sin heartbeat CAN del ESP32:
- La auto-transición STANDBY→ACTIVE (`safety_system.c:329-331`) requiere `last_can_rx_time` actualizado
- La recuperación SAFE→ACTIVE (`safety_system.c:334-338`) requiere heartbeat restaurado

**Consecuencia: el vehículo no puede moverse. Los motores están permanentemente deshabilitados. Los relés permanentemente apagados.**

### 🟡 BLOQUEADOR MEDIO #2: No existe palanca de cambios

**Bloquea: CONTROL (dirección de marcha)**

Incluso si se resolviera el bloqueador #1:
- No existe implementación de shifter (P/R/N/D1/D2)
- La dirección de los motores se controla SOLO por el signo de `demandPct`
- En modo pedal local: `Pedal_GetPercent()` retorna 0-100%, siempre positivo
- No hay manera de seleccionar marcha atrás sin CAN comando

En el firmware base, `src/input/shifter.cpp` lee la palanca vía MCP23017.
En el STM32, el hardware MCP23017 NO existe en el esquema de pines (`main.h`).

---

## 5) CONFIRMACIÓN EXPLÍCITA

### ¿Puede el coche moverse de forma controlada y segura sin ESP32?

**NO.**

El sistema de seguridad (`Safety_CheckCANTimeout()`) detecta la ausencia del ESP32 en 250 ms y transiciona a SAFE state, que deshabilita todos los actuadores. No existe ruta alternativa para alcanzar el estado ACTIVE sin heartbeat CAN. El pedal se lee pero su valor nunca llega a los motores.

### ¿Hay algún riesgo crítico conocido al intentarlo?

**No hay riesgo de movimiento involuntario.** El firmware falla seguro: sin ESP32, todo queda en SAFE con motores deshabilitados y relés apagados.

El riesgo es de **no-movimiento**, no de movimiento descontrolado:
- Si el coche se enciende sin ESP32 en una pendiente, los motores estarán deshabilitados y no habrá frenado activo (solo freno mecánico de estacionamiento, si existe)
- El IWDG sigue activo y reiniciará el sistema si el bucle principal se bloquea

No hay riesgo de que los motores se activen espontáneamente sin ESP32.

---

## APÉNDICE A — Mapa completo de periféricos

```
┌─────────────┬──────────┬─────────────────────┬───────────────────────────────┐
│ Periférico  │ Pin(es)  │ Config              │ Usado en                      │
├─────────────┼──────────┼─────────────────────┼───────────────────────────────┤
│ TIM1 CH1    │ PA8      │ PWM 20 kHz          │ Motor FL                      │
│ TIM1 CH2    │ PA9      │ PWM 20 kHz          │ Motor FR                      │
│ TIM1 CH3    │ PA10     │ PWM 20 kHz          │ Motor RL                      │
│ TIM1 CH4    │ PA11     │ PWM 20 kHz          │ Motor RR                      │
│ TIM8 CH3    │ PC8      │ PWM 20 kHz          │ Motor Steering                │
│ TIM2 CH1/2  │ PA15/PB3 │ Encoder quad TI12   │ Steering angle (4800 CPR)     │
│ ADC1 CH4    │ PA3      │ 12-bit single conv  │ Pedal                         │
│ FDCAN1      │ PB8/PB9  │ 500 kbps classic    │ ESP32 communication           │
│ I2C1        │ PB6/PB7  │ 400 kHz fast mode   │ INA226 / TCA9548A             │
│ IWDG        │ —        │ ~500 ms             │ Main loop watchdog            │
│ GPIOC 0-4   │ PC0-PC4  │ Output PP           │ Direction FL/FR/RL/RR/STEER   │
│ GPIOC 5-7,9 │ PC5-7,9  │ Output PP           │ Enable FL/FR/RL/RR            │
│ GPIOC 13    │ PC13     │ Output PP           │ Enable RR                     │
│ GPIOC 9     │ PC9      │ Output PP           │ Enable STEER                  │
│ GPIOC 10-12 │ PC10-12  │ Output PP           │ Relay MAIN/TRAC/DIR           │
│ GPIOA 0-2   │ PA0-PA2  │ EXTI Rising+PU      │ Wheel speed FL/FR/RL          │
│ GPIOB 15    │ PB15     │ EXTI Rising+PU      │ Wheel speed RR                │
│ GPIOB 0     │ PB0      │ OD / Input          │ OneWire DS18B20 bus           │
│ PB4         │ PB4      │ ⚠ NO CONFIGURADO    │ Encoder Z (definido, no usado)│
└─────────────┴──────────┴─────────────────────┴───────────────────────────────┘
```

## APÉNDICE B — Mapa completo de mensajes CAN

```
┌──────┬──────────────────────┬─────────┬──────┬──────────────────────────────────────────┐
│ ID   │ Nombre               │ Dir     │ Bytes│ Contenido                                │
├──────┼──────────────────────┼─────────┼──────┼──────────────────────────────────────────┤
│ 0x001│ Heartbeat STM32      │ STM→ESP │  4   │ counter, state, fault_flags, reserved    │
│ 0x011│ Heartbeat ESP32      │ ESP→STM │  —   │ Presencia (refresca CAN timeout)         │
│ 0x100│ Cmd Throttle         │ ESP→STM │  1   │ throttle_pct (0-100)                     │
│ 0x101│ Cmd Steering         │ ESP→STM │  2   │ angle_raw (int16, ×0.1°)                 │
│ 0x102│ Cmd Mode             │ ESP→STM │  1   │ bit0=4x4, bit1=tank_turn                 │
│ 0x200│ Status Speed         │ STM→ESP │  8   │ FL/FR/RL/RR ×10 km/h (uint16 LE cada)   │
│ 0x201│ Status Current       │ STM→ESP │  8   │ FL/FR/RL/RR ×100 A (uint16 LE cada)      │
│ 0x202│ Status Temperature   │ STM→ESP │  5   │ 5× int8 °C                               │
│ 0x203│ Status Safety        │ STM→ESP │  3   │ abs, tcs, error_code                     │
│ 0x204│ Status Steering      │ STM→ESP │  3   │ angle ×10° (int16 LE) + calibrated       │
│ 0x300│ Diag Error           │ Ambos   │  2   │ error_code, subsystem (NUNCA ENVIADO)    │
└──────┴──────────────────────┴─────────┴──────┴──────────────────────────────────────────┘
```

## APÉNDICE C — Flujo pedal local (standalone hipotético)

```
main.c bucle 50 ms:
  Pedal_Update()                          ← ADC1 CH4 (PA3) → pedal_raw → pedal_pct
       │
       ▼
  Safety_IsCommandAllowed()               ← retorna (system_state == SYS_STATE_ACTIVE)
       │
       ├─ SIN ESP32: retorna false         ⛔ BLOQUEADO AQUÍ
       │       └─→ Traction_SetDemand(0.0f)
       │
       └─ CON ESP32 + ACTIVE: retorna true
               │
               ▼
          Safety_ValidateThrottle(Pedal_GetPercent())
               │
               ├─ Clamp [0, 100]
               ├─ ABS activo → 0%
               ├─ TCS activo → 50%
               └─ return pct
               │
               ▼
          Traction_SetDemand(validated)
               │
               ├─ EMA filter (alpha=0.15)
               ├─ Ramp limiter (50 %/s up, 100 %/s down)
               └─ → traction_state.demandPct
               │
               ▼
          Traction_Update() [cada 10 ms]
               │
               └─→ PWM a motores TIM1 CH1-CH4
```

---

*Auditoría generada por análisis línea por línea del código fuente real. Toda función y archivo citado es verificable directamente en el repositorio. Repo base de referencia: `github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos`.*
