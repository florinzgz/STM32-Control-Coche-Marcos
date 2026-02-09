# Informe de Auditoría del Firmware STM32-Control-Coche-Marcos

**Fecha:** 2026-02-09  
**Base de análisis:** Rama actual, estado tras los últimos merges  
**Autor:** Auditoría automatizada — basada exclusivamente en el código fuente real  

---

## PARTE 1 — VERIFICACIÓN HMI (CAN → PANTALLA)

### 1.1 MOTORES DE TRACCIÓN (4 ruedas)

#### 1.1.1 Velocidad por rueda

| Dato | Detalle |
|------|---------|
| **CAN ID** | `0x200` (`CAN_ID_STATUS_SPEED`) |
| **Bytes** | 0-1: FL, 2-3: FR, 4-5: RL, 6-7: RR (uint16 little-endian) |
| **Escala** | Valor × 0.1 = km/h (se envía `Wheel_GetSpeed_XX() * 10`) |
| **Frecuencia** | 100 ms (10 Hz) — bloque `tick_100ms` en `main.c:134-138` |
| **Estado** | ✅ **ENVIADO — suficiente para HMI** |

**Referencia código:**  
- `main.c:134-138` — Packing: `(uint16_t)(Wheel_GetSpeed_FL() * 10)`  
- `can_handler.c:162-176` — `CAN_SendStatusSpeed()` empaqueta 4 × uint16 LE  
- `sensor_manager.c:56-79` — Cálculo real: pulsos EXTI → km/h via `WHEEL_CIRCUMF_M`

---

#### 1.1.2 Corriente por rueda

| Dato | Detalle |
|------|---------|
| **CAN ID** | `0x201` (`CAN_ID_STATUS_CURRENT`) |
| **Bytes** | 0-1: FL (INA226 #0), 2-3: FR (#1), 4-5: RL (#2), 6-7: RR (#3) — uint16 LE |
| **Escala** | Valor × 0.01 = Amperios (se envía `Current_GetAmps(i) * 100`) |
| **Frecuencia** | 100 ms (10 Hz) — bloque `tick_100ms` en `main.c:139-143` |
| **Estado** | ✅ **ENVIADO — suficiente para HMI** |

**Nota:** Se envían los sensores INA226 índices 0-3 (FL, FR, RL, RR). Los sensores #4 (batería) y #5 (dirección) **NO se envían** en este mensaje. Sin embargo, la información de corriente de batería y dirección está disponible internamente pero no tiene un CAN ID dedicado. Véase tabla de información no enviada en sección 1.3.

**Referencia código:**  
- `main.c:139-143` — `Current_GetAmps(0..3) * 100`  
- `can_handler.c:178-192` — `CAN_SendStatusCurrent()`  
- `sensor_manager.c:139-158` — Lectura I2C via TCA9548A

---

#### 1.1.3 Temperatura por rueda

| Dato | Detalle |
|------|---------|
| **CAN ID** | `0x202` (`CAN_ID_STATUS_TEMP`) — genérico, 5 sensores |
| **Bytes** | 0: Sensor 0, 1: Sensor 1, 2: Sensor 2, 3: Sensor 3, 4: Sensor 4 — int8_t (°C) |
| **Escala** | 1 °C por unidad (truncado a entero desde float DS18B20) |
| **Frecuencia** | 1000 ms (1 Hz) — bloque `tick_1000ms` en `main.c:157-162` |
| **Estado** | ✅ **ENVIADO** |

| Dato | Detalle |
|------|---------|
| **CAN ID** | `0x206` (`CAN_ID_STATUS_TEMP_MAP`) — mapeo explícito |
| **Bytes** | 0: Motor FL, 1: Motor FR, 2: Motor RL, 3: Motor RR, 4: Ambiente — int8_t (°C) |
| **Escala** | 1 °C por unidad |
| **Frecuencia** | 1000 ms (1 Hz) — bloque `tick_1000ms` en `main.c:163` |
| **Estado** | ✅ **ENVIADO — mapeo semántico explícito, suficiente para HMI** |

**Nota:** El mensaje `0x206` asigna explícitamente: índice 0=FL, 1=FR, 2=RL, 3=RR, 4=Ambiente. La HMI puede usar este mensaje directamente sin suposiciones de mapeo.

**Referencia código:**  
- `can_handler.c:271-281` — `CAN_SendStatusTempMap()`  
- `sensor_manager.c:421-454` — DS18B20 OneWire con ROM Search

---

#### 1.1.4 Nivel de tracción disponible por rueda (ABS / TCS real)

| Dato | Detalle |
|------|---------|
| **CAN ID** | `0x205` (`CAN_ID_STATUS_TRACTION`) |
| **Bytes** | 0: FL %, 1: FR %, 2: RL %, 3: RR % — uint8_t |
| **Escala** | 0-100 % (0 = rueda totalmente inhibida, 100 = potencia completa) |
| **Frecuencia** | 100 ms (10 Hz) — bloque `tick_100ms` en `main.c:150` |
| **Estado** | ✅ **ENVIADO — suficiente para HMI** |

**Detalle de la escala:** Se convierte desde `safety_status.wheel_scale[i]` (float 0.0–1.0) a uint8 0-100. Este valor refleja la intervención combinada de ABS y TCS:
- ABS reduce `wheel_scale` a `0.0` en la rueda que bloquea (`safety_system.c:513`)
- TCS reduce progresivamente: 40% inicial, +5% por ciclo, máx 80% (`safety_system.c:596-607`)
- Se toma el mínimo entre ABS y TCS (`safety_system.c:621-623`)

**Referencia código:**  
- `can_handler.c:242-251` — `CAN_SendStatusTraction()`  
- `safety_system.c:96-97` — `wheel_scale[4]` en `SafetyStatus_t`

---

### 1.2 MOTOR DE DIRECCIÓN (VOLANTE)

#### 1.2.1 Ángulo actual de dirección

| Dato | Detalle |
|------|---------|
| **CAN ID** | `0x204` (`CAN_ID_STATUS_STEERING`) |
| **Bytes** | 0-1: ángulo (int16 LE), 2: calibrado (bool) |
| **Escala** | Valor × 0.1 = grados (se envía `Steering_GetCurrentAngle() * 10`) |
| **Frecuencia** | 100 ms (10 Hz) — bloque `tick_100ms` en `main.c:147-149` |
| **Estado** | ✅ **ENVIADO — suficiente para HMI** |

**Referencia código:**  
- `main.c:147-149` — `(int16_t)(Steering_GetCurrentAngle() * 10)`  
- `can_handler.c:217-226` — `CAN_SendStatusSteering()`  
- `motor_control.c:422-426` — `Steering_GetCurrentAngle()` convierte encoder counts a grados

---

#### 1.2.2 Estado de calibración / centrado

| Dato | Detalle |
|------|---------|
| **CAN ID** | `0x204` (`CAN_ID_STATUS_STEERING`), Byte 2 |
| **Bytes** | Byte 2: 1 = calibrado, 0 = no calibrado |
| **Frecuencia** | 100 ms (10 Hz) |
| **Estado** | ✅ **ENVIADO** |

Adicionalmente, el **estado detallado del centrado** se puede inferir de:

| Dato | Detalle |
|------|---------|
| **CAN ID** | `0x001` (Heartbeat STM32), Byte 1 |
| **Bytes** | Byte 1 = `system_state`: 0=Boot, 1=Standby, 2=Active, 3=Degraded, 4=Safe, 5=Error |
| **Interpretación** | Si `system_state == 1 (Standby)` y `calibrated == 0`, centering en curso |
| **CAN ID** | `0x001` (Heartbeat STM32), Byte 2 bit 7 |
| **Bytes** | Byte 2 bit 7 = `FAULT_CENTERING` (1 = fallo de centrado) |
| **Estado** | ✅ **ENVIADO — suficiente para HMI** |

**Referencia código:**  
- `safety_system.h:81` — `#define FAULT_CENTERING (1U << 7)`  
- `safety_system.c:332` — `Safety_GetFaultFlags()` incluye centering fault

---

#### 1.2.3 Fallos relevantes (encoder, centering, safety)

| Fallo | CAN ID | Byte | Bit | Detalle |
|-------|--------|------|-----|---------|
| Encoder error | `0x001` | 2 | bit 3 | `FAULT_ENCODER_ERROR` — fallo del encoder detectado por `Encoder_CheckHealth()` |
| Centering fault | `0x001` | 2 | bit 7 | `FAULT_CENTERING` — centrado fallido |
| ABS activo | `0x001` | 2 | bit 5 | `FAULT_ABS_ACTIVE` |
| TCS activo | `0x001` | 2 | bit 6 | `FAULT_TCS_ACTIVE` |
| CAN timeout | `0x001` | 2 | bit 0 | `FAULT_CAN_TIMEOUT` |
| Sobretemperatura | `0x001` | 2 | bit 1 | `FAULT_TEMP_OVERLOAD` |
| Sobrecorriente | `0x001` | 2 | bit 2 | `FAULT_CURRENT_OVERLOAD` |
| Sensor rueda | `0x001` | 2 | bit 4 | `FAULT_WHEEL_SENSOR` |

| Dato adicional | CAN ID | Detalle |
|----------------|--------|---------|
| Safety summary | `0x203` (`CAN_ID_STATUS_SAFETY`) | Byte 0: ABS activo, Byte 1: TCS activo, Byte 2: error_code |
| Error detallado | `0x300` (`CAN_ID_DIAG_ERROR`) | Byte 0: error_code, Byte 1: subsystem — on-demand |
| Service faults | `0x301` (`CAN_ID_SERVICE_FAULTS`) | uint32 bitmask de módulos con fallo |
| Service enabled | `0x302` (`CAN_ID_SERVICE_ENABLED`) | uint32 bitmask de módulos habilitados |
| Service disabled | `0x303` (`CAN_ID_SERVICE_DISABLED`) | uint32 bitmask de módulos deshabilitados |

**Estado:** ✅ **ENVIADO — la información de fallos es completa para HMI**

**Referencia código:**  
- `safety_system.h:74-81` — Definiciones de `FAULT_*`  
- `safety_system.c:326-347` — `Safety_GetFaultFlags()`  
- `can_handler.c:141-160` — Heartbeat con state + fault_flags + error_code

---

### 1.3 INFORMACIÓN NO ENVIADA POR CAN

| Información | ¿Se envía? | Razón |
|-------------|-----------|-------|
| **Corriente batería (INA226 #4)** | ❌ NO | (c) No existe un CAN ID dedicado. El dato existe internamente (`Current_GetAmps(4)`) pero `CAN_SendStatusCurrent()` solo envía índices 0-3. |
| **Corriente dirección (INA226 #5)** | ❌ NO | (c) Mismo caso que batería. `Current_GetAmps(5)` existe pero no se transmite por CAN. |
| **Voltaje bus por sensor** | ❌ NO | (c) `Voltage_GetBus(i)` existe internamente pero no tiene mensaje CAN asignado. |
| **RPM por rueda** | ❌ NO | (b) Por diseño. `Wheel_GetRPM_FL()` existe solo para FL y no se envía. Las velocidades en km/h son suficientes para la HMI. |
| **Estado detallado del centrado** | ⚠️ PARCIAL | El estado (IDLE/SWEEP_LEFT/SWEEP_RIGHT/DONE/FAULT) de `CenteringState_t` no se envía directamente. Solo se puede inferir del heartbeat (system_state + fault_flags). La HMI no puede saber si está barriendo a izquierda o a derecha. **Limitación aceptable:** el centrado dura ~10s durante boot y la HMI solo necesita saber "calibrando" vs "listo" vs "fallo", lo cual SÍ está disponible via heartbeat. |
| **Ángulos Ackermann individuales FL/FR** | ❌ NO | (b) Por diseño. Se calculan internamente (`steer_fl_deg`, `steer_fr_deg` en `motor_control.c:77-78`) pero el CAN solo envía el ángulo del volante (columna). La geometría Ackermann es transparente para la HMI. |
| **Contadores ABS/TCS (activation count)** | ❌ NO | (c) `safety_status.abs_activation_count` y `tcs_activation_count` existen pero no se transmiten. |
| **Demand pedal filtrado** | ❌ NO | (b) Por diseño. La HMI envía el comando, el STM32 lo filtra y aplica. No hay retroalimentación del valor filtrado final. |
| **Estado de relés** | ❌ NO | (c) Se controlan internamente en `Relay_PowerUp()`/`Relay_PowerDown()` pero no hay feedback de estado real. Se infiere del `system_state`. |
| **Máscara ABS/TCS por rueda** | ❌ NO | (c) `abs_wheel_mask` y `tcs_wheel_mask` existen pero no se envían. Sin embargo, `wheel_scale[]` (0x205) da la misma información de forma más útil. |

---

### 1.4 RESUMEN HMI: ¿Es suficiente la información CAN?

**VEREDICTO: ✅ SÍ — La información CAN es SUFICIENTE para que la HMI muestre correctamente el estado del vehículo.**

La HMI (ESP32) dispone de:
- ✅ Velocidad por rueda (0x200) — 10 Hz
- ✅ Corriente por rueda de tracción (0x201) — 10 Hz  
- ✅ Temperatura por motor con mapeo explícito (0x206) — 1 Hz
- ✅ Tracción disponible por rueda / ABS+TCS real (0x205) — 10 Hz
- ✅ Ángulo de dirección + calibración (0x204) — 10 Hz
- ✅ Estado del sistema + fallos detallados (0x001) — 10 Hz
- ✅ Resumen safety ABS/TCS (0x203) — 10 Hz
- ✅ Service mode: faults/enabled/disabled (0x301-0x303) — 1 Hz

**Mejoras opcionales (NO necesarias):**
- Corriente de batería y dirección (INA226 #4, #5) — útil para diagnóstico avanzado
- Estado granular del centrado (SWEEP_LEFT/RIGHT) — útil solo durante boot

---

## PARTE 2 — AUDITORÍA GENERAL DEL FIRMWARE

### 2.1 Estado general del firmware

**¿Es coherente internamente?** ✅ **SÍ**

El firmware presenta una arquitectura consistente y bien organizada:

- **Separación de responsabilidades clara:** Cada módulo tiene un archivo .c/.h independiente (motor_control, sensor_manager, safety_system, can_handler, steering_centering, service_mode, ackermann).
- **Flujo de datos unidireccional:** Sensores → Safety → Actuadores. El CAN es bidireccional pero siempre pasa por la capa de validación.
- **Patrón de autoridad consistente:** El STM32 es la autoridad de seguridad. Todo comando del ESP32 pasa por `Safety_Validate*()` antes de ejecutarse.
- **Temporización coherente:** El main loop usa intervalos escalonados (10ms/50ms/100ms/1000ms) sin conflictos de recursos.

**¿Es estable según la arquitectura definida?** ✅ **SÍ**

La arquitectura STM32 = control y seguridad se mantiene rigurosamente:
- El STM32 no depende del ESP32 para ninguna función de seguridad
- El CAN timeout desencadena transición a SAFE (actuadores off)
- El watchdog IWDG (500ms) protege contra bloqueos del software
- El `Error_Handler()` usa acceso directo a registros GPIO para desactivar relays incluso si HAL está corrupto

---

### 2.2 Funcionalidades IMPLEMENTADAS y ACTIVAS

#### 2.2.1 Tracción
| Función | Archivo | Estado |
|---------|---------|--------|
| PWM 4 motores (TIM1 CH1-4, 20kHz) | `motor_control.c:124-152` | ✅ Activo |
| Modo 4x2 (solo delanteras) | `motor_control.c:279-286` | ✅ Activo |
| Modo 4x4 (las 4 ruedas) | `motor_control.c:266-278` | ✅ Activo |
| Tank turn (rotación sobre eje) | `motor_control.c:254-265` | ✅ Activo |
| Filtrado EMA pedal (α=0.15) | `motor_control.c:200-209` | ✅ Activo |
| Limitador rampa (50%/s up, 100%/s down) | `motor_control.c:211-233` | ✅ Activo |
| Per-wheel safety scaling | `motor_control.c:258,270-277,280-281` | ✅ Activo |
| Emergency stop | `motor_control.c:312-330` | ✅ Activo |

#### 2.2.2 Dirección
| Función | Archivo | Estado |
|---------|---------|--------|
| PID steering (P-only, kp=0.09 en counts) | `motor_control.c:62,365-419` | ✅ Activo |
| Deadband (0.5° ≈ 6.67 counts) | `motor_control.c:93,401-406` | ✅ Activo |
| Ackermann geometry (cálculo FL/FR) | `ackermann.c:20-60` | ✅ Activo |
| Centering automático (sweep L/R) | `steering_centering.c:1181-1271` | ✅ Activo |
| Encoder health monitoring (3 niveles) | `motor_control.c:464-512` | ✅ Activo |
| Steering neutralize (encoder fault) | `motor_control.c:526-533` | ✅ Activo |
| Calibration gate (rechaza comandos sin calibrar) | `motor_control.c:346-347` | ✅ Activo |

#### 2.2.3 Seguridad
| Función | Archivo | Estado |
|---------|---------|--------|
| State machine (BOOT→STANDBY→ACTIVE⇄DEGRADED→SAFE→ERROR) | `safety_system.c:244-301` | ✅ Activo |
| ABS per-wheel (slip > 15%, min 10 km/h) | `safety_system.c:481-537` | ✅ Activo |
| TCS per-wheel (progressive reduction) | `safety_system.c:558-640` | ✅ Activo |
| Overcurrent detection (>25A, 3-strike escalation) | `safety_system.c:647-688` | ✅ Activo |
| Overtemperature (warning 80°C, critical 90°C, 5°C hysteresis) | `safety_system.c:699-743` | ✅ Activo |
| CAN heartbeat timeout (250ms) | `safety_system.c:748-785` | ✅ Activo |
| Sensor plausibility checks | `safety_system.c:796-856` | ✅ Activo |
| Steering encoder health | `safety_system.c:873-896` | ✅ Activo |
| Command validation gate (throttle/steering/mode) | `safety_system.c:376-433` | ✅ Activo |
| Degraded mode (40% power, 50% speed limit) | `safety_system.c:319-324,394-395` | ✅ Activo |
| Recovery debounce (DEGRADED→ACTIVE, 500ms hold) | `safety_system.c:770-783` | ✅ Activo |
| Relay power sequencing (Main→Traction→Direction) | `safety_system.c:353-370` | ✅ Activo |
| Emergency stop → ERROR | `safety_system.c:900-909` | ✅ Activo |
| FailSafe (encoders → center or neutralize) | `safety_system.c:911-926` | ✅ Activo |
| IWDG watchdog (500ms) | `main.c:403-411,174` | ✅ Activo |

#### 2.2.4 Sensores
| Sensor | Archivo | Estado |
|--------|---------|--------|
| 4× LJ12A3 wheel speed (EXTI, 6 pulses/rev) | `sensor_manager.c:22-79` | ✅ Activo |
| 5× DS18B20 temperature (OneWire, ROM Search, CRC-8) | `sensor_manager.c:173-454` | ✅ Activo |
| 6× INA226 current/voltage (I2C via TCA9548A) | `sensor_manager.c:107-168` | ✅ Activo |
| 1× Hall pedal (ADC 12-bit) | `sensor_manager.c:86-102` | ✅ Activo |
| 1× E6B2-CWZ6C encoder (TIM2 quadrature, 4800 CPR) | `main.c:336-358`, `motor_control.c:112` | ✅ Activo |
| 1× LJ12A3 steering center (EXTI5) | `sensor_manager.c:41-47` | ✅ Activo |

#### 2.2.5 CAN / Telemetría
| Mensaje | ID | Dirección | Frecuencia | Estado |
|---------|------|-----------|------------|--------|
| Heartbeat STM32 | 0x001 | STM32→ESP32 | 100ms | ✅ Activo |
| Heartbeat ESP32 | 0x011 | ESP32→STM32 | 100ms | ✅ Activo |
| Throttle cmd | 0x100 | ESP32→STM32 | 50ms | ✅ Activo |
| Steering cmd | 0x101 | ESP32→STM32 | 50ms | ✅ Activo |
| Mode cmd | 0x102 | ESP32→STM32 | on-demand | ✅ Activo |
| Service cmd | 0x110 | ESP32→STM32 | on-demand | ✅ Activo |
| Speed status | 0x200 | STM32→ESP32 | 100ms | ✅ Activo |
| Current status | 0x201 | STM32→ESP32 | 100ms | ✅ Activo |
| Temp status | 0x202 | STM32→ESP32 | 1000ms | ✅ Activo |
| Safety status | 0x203 | STM32→ESP32 | 100ms | ✅ Activo |
| Steering status | 0x204 | STM32→ESP32 | 100ms | ✅ Activo |
| Traction status | 0x205 | STM32→ESP32 | 100ms | ✅ Activo |
| TempMap status | 0x206 | STM32→ESP32 | 1000ms | ✅ Activo |
| Diag error | 0x300 | Bidireccional | on-demand | ✅ Activo |
| Service faults | 0x301 | STM32→ESP32 | 1000ms | ✅ Activo |
| Service enabled | 0x302 | STM32→ESP32 | 1000ms | ✅ Activo |
| Service disabled | 0x303 | STM32→ESP32 | 1000ms | ✅ Activo |
| RX filters | — | Hardware | — | ✅ 3 filtros configurados |

#### 2.2.6 Service Mode / DEGRADED
| Función | Archivo | Estado |
|---------|---------|--------|
| 25 módulos registrados (4 critical, 21 non-critical) | `service_mode.c:28-67` | ✅ Activo |
| Enable/disable non-critical modules | `service_mode.c:102-124` | ✅ Activo |
| Critical modules no deshabilitables | `service_mode.c:110-117` | ✅ Activo |
| Fault tracking per-module | `service_mode.c:130-148` | ✅ Activo |
| Factory restore | `service_mode.c:215-225` | ✅ Activo |
| Bitmask CAN reporting | `service_mode.c:231-263` | ✅ Activo |
| ShouldBlock logic | `service_mode.c:170-185` | ✅ Activo |

---

### 2.3 Funcionalidades EXISTENTES pero con limitaciones

#### 2.3.1 Implementadas pero NO usadas

| Función | Ubicación | Detalle |
|---------|-----------|---------|
| `Wheel_GetRPM_FL()` | `sensor_manager.c:80` | Solo implementada para FL, no se usa en ningún otro módulo ni se envía por CAN. |
| `Voltage_GetBus()` | `sensor_manager.c:165-168` | Voltaje de bus de INA226 se lee pero nunca se consume ni se envía por CAN. |
| `Ackermann_Compute()` | `motor_control.c:539-564` | Función legacy duplicada. La función activa es `Ackermann_ComputeWheelAngles()` en `ackermann.c`. `Ackermann_Compute()` existe en `motor_control.c` pero NO se invoca desde ningún punto. |
| `Ackermann_SetGeometry()` | `motor_control.c:566-571` | Permite cambiar geometría en runtime, pero nunca se llama. Los valores se inicializan desde `vehicle_physics.h` y son constantes. |
| `WheelState_t.effortPct` | `motor_control.h:27` | Campo declarado en la estructura pero nunca se asigna un valor en el código. Siempre será 0. |
| `WheelState_t.demandPct` (per-wheel) | `motor_control.h:21` | Se inicializa a 0 en `Traction_Init()` pero nunca se actualiza individualmente. Solo `traction_state.demandPct` global se actualiza. |
| `CAN_SendError()` | `can_handler.c:283-290` | Función declarada e implementada, pero nunca se invoca desde ningún punto del código. Los errores se reportan via heartbeat y service status. |

#### 2.3.2 Parciales

| Función | Detalle |
|---------|---------|
| **PID Steering** | Solo usa componente P (ki=0, kd=0). Los términos I y D están implementados en `PID_Compute()` pero no contribuyen. Esto es por diseño (P-only del firmware base). |
| **RPM Wheels** | Solo FL tiene getter (`Wheel_GetRPM_FL()`). FR/RL/RR calculan RPM internamente (`wheel_rpm[]`) pero no tienen getter público. |
| **Temperature early boot** | Si `OW_SearchAll()` no descubre sensores, solo `temperatures[0]` se actualiza via Skip ROM (`sensor_manager.c:431-441`). Sensores 1-4 permanecen en 0.0. |

#### 2.3.3 Limitaciones conocidas

| Limitación | Detalle |
|------------|---------|
| **OneWire bit-bang timing** | Usa busy-wait NOP loop calibrado para 170MHz (`sensor_manager.c:209`). Si la frecuencia del sistema cambia, los timings serán incorrectos. El propio código comenta que es "simplified — production code should use dedicated OneWire library" (`sensor_manager.c:182-183`). |
| **ADC Pedal polling** | Usa `HAL_ADC_PollForConversion()` con timeout de 10ms (`sensor_manager.c:94`), lo que bloquea brevemente el main loop. En un sistema real a 170MHz esto es negligible (microsegundos), pero no es ideal. |
| **Encoder Z-index no usado** | El pin PB4 (Z-index del E6B2-CWZ6C) está definido (`main.h:52`) pero no se inicializa como EXTI ni se usa. Esto es una decisión de diseño documentada (`motor_control.c:83-89`). |
| **Single fault code** | `safety_error` es un enum simple, no una bitmask. Solo puede almacenar un fallo activo a la vez. Si ocurren dos fallos simultáneos (ej: overtemp + overcurrent), solo se retiene el último. Los fault_flags del heartbeat SÍ son bitmask, pero el `error_code` en byte 3 del heartbeat es el último fallo singular. |

---

### 2.4 Funcionalidades IMPORTANTES que FALTAN

Basándome en el análisis del código actual y los documentos del repositorio:

| Funcionalidad | Impacto | Comentario |
|---------------|---------|------------|
| **Reverse/freno regenerativo** | Medio | El sistema acepta `throttlePct` negativo internamente (motor_control.c:197-198 clamp a ±100%) y los motores pueden invertir dirección, pero el comando CAN de throttle (0x100) solo envía uint8 (0-255), impidiendo que la ESP32 envíe marcha atrás via CAN. El pedal local (ADC) sí llega a `Traction_SetDemand()` pero siempre es 0-100%. |
| **Persistencia de service mode** | Bajo | Los módulos deshabilitados no sobreviven a un reset. `ServiceMode_Init()` habilita todo. Aceptable en prototipo. |
| **Logging/telemetría extendida** | Bajo | No hay buffer circular de eventos ni timestamp en mensajes CAN. Aceptable para fase actual. |

---

### 2.5 Posibles errores, incoherencias o puntos débiles

#### 2.5.1 Lógica

1. **`Safety_GetFaultFlags()` — Lógica de comparación vs bitmask (`safety_system.c:328-333`)**  
   La función usa `if (safety_error == SAFETY_ERROR_CAN_TIMEOUT)` con comparación de igualdad. Esto significa que si `safety_error` es `SAFETY_ERROR_OVERTEMP`, el flag `FAULT_CAN_TIMEOUT` NO se activa aunque el CAN esté en timeout (el timeout ya habría sobrescrito el error). Sin embargo, los bits `FAULT_ENCODER_ERROR` y `FAULT_WHEEL_SENSOR` usan `ServiceMode_GetFault()` que es independiente, así que esos siempre se reportan correctamente. **Impacto:** Bajo — en la práctica, CAN timeout lleva a SAFE inmediatamente y se convierte en el error dominante. Las flags de ABS/TCS se usan desde `safety_status` y son independientes del error code.

2. **`Wheel_ComputeSpeed()` llamada desde getter (`sensor_manager.c:76-79`)**  
   Cada llamada a `Wheel_GetSpeed_FL()` ejecuta el cálculo completo incluyendo `HAL_GetTick()`. Si se llama múltiples veces en el mismo ciclo (main loop llama desde `Traction_Update()`, `ABS_Update()`, `TCS_Update()`, `Safety_CheckSensors()`, `CAN_SendStatusSpeed()`), el `dt` entre llamadas sucesivas puede ser 0 o muy pequeño, devolviendo 0 km/h erróneamente. **Impacto:** Medio — En la práctica, como el tick_10ms y tick_100ms son intervalos distintos, las llamadas están espaciadas. Pero dentro del bloque tick_10ms, `ABS_Update()` y `TCS_Update()` llaman a los 4 getters consecutivamente, lo que podría dar dt=0 en la segunda llamada del mismo tick.

3. **`CAN_SendStatusCurrent()` usa INA226 #0-3, no Motor FL/FR/RL/RR (`main.c:139-143`)**  
   Se asume que INA226 #0=FL, #1=FR, #2=RL, #3=RR por el orden de los canales del multiplexor TCA9548A. Esta asignación depende del cableado hardware y no está validada en software. El `service_mode.h:61-66` sí documenta esta asignación. **Impacto:** Bajo — es correcto mientras el cableado respete el orden documentado.

#### 2.5.2 Seguridad

4. **Pedal local bypass safety (`main.c:121-126`)**  
   El pedal local (ADC) pasa por `Safety_ValidateThrottle()` antes de `Traction_SetDemand()`, lo cual es correcto. Sin embargo, los comandos CAN también pasan por la misma validación. **No hay bypass.** ✅ Correcto.

5. **`last_can_rx_time` — volatile pero no atomic (`safety_system.c:215`)**  
   Declarado `volatile uint32_t` y escrito desde `Safety_UpdateCANRxTime()` (llamado desde `CAN_ProcessMessages()` en main loop, no desde ISR). La lectura en `Safety_CheckCANTimeout()` es del mismo contexto. **No hay race condition** porque ambas operaciones ocurren en el main loop. ✅ Correcto.

6. **Steering centering abort → DEGRADED (`steering_centering.c:1126`)**  
   Si el centrado falla, el sistema entra en DEGRADED (no SAFE), lo que permite "drive home". Esto es coherente con la filosofía del firmware base (limp_mode.cpp). Sin embargo, con `SAFETY_ERROR_CENTERING` activo, la dirección está neutralizada y el vehículo solo puede ir recto. **Impacto:** Aceptable por diseño — el conductor puede llevar el vehículo a reparar.

#### 2.5.3 Estados

7. **DEGRADED→ACTIVE recovery requiere `safety_error == SAFETY_ERROR_NONE` (`safety_system.c:773-774`)**  
   La recuperación solo ocurre si el error se ha limpiado previamente. Los módulos de safety limpian sus errores al detectar condición normal (ej: `Safety_ClearError(SAFETY_ERROR_OVERCURRENT)` en `Safety_CheckCurrent():683-687`). Pero si el error activo es de un módulo diferente, la limpieza no ocurre (ej: si `safety_error` es `OVERTEMP` pero el overcurrent ya se resolvió, el overtemp debe resolverse primero). **Impacto:** Comportamiento correcto por diseño — todos los fallos deben resolverse para volver a ACTIVE.

8. **`Safety_SetState(SYS_STATE_ACTIVE)` desde SAFE requiere `safety_error == NONE` (`safety_system.c:263`)**  
   Esto crea un orden de dependencia: el error debe limpiarse ANTES de la transición. En `Safety_CheckCANTimeout():764-766`, el error se limpia y luego se intenta la transición. ✅ Correcto.

#### 2.5.4 Uso de sensores

9. **DS18B20 ROM Search order no garantizado**  
   `OW_SearchAll()` descubre sensores en orden de ROM code (definido por el protocolo OneWire), no en orden físico. Si se reemplaza un sensor, el mapeo puede cambiar (sensor que era índice 0=FL podría pasar a otro índice). **Impacto:** Medio para producción. Requiere que los sensores se instalen siempre con el mismo orden de ROM o que se implemente un mapeo por ROM ID. En prototipo es aceptable.

10. **Pedal ADC sin deadzone ni protección ante desconexión**  
    `Pedal_Update()` convierte directamente `raw * 100 / 4095`. No hay detección de cable desconectado (0 raw) ni de cortocircuito (4095 raw). Un pedal desconectado podría dar 0% (seguro) o un valor flotante (impredecible). **Impacto:** Bajo — la validación en `Safety_ValidateThrottle()` limita el rango, pero no detecta la causa raíz.

#### 2.5.5 CAN / Documentación

11. **CAN ID 0x300 (`CAN_ID_DIAG_ERROR`) nunca se envía**  
    `CAN_SendError()` está implementada pero no se invoca desde ningún punto del firmware. Los errores se reportan exclusivamente via heartbeat (0x001) y service status (0x301-0x303). La documentación (`can_handler.h:33`) la lista como "Both directions" pero nunca se genera desde el STM32. **Impacto:** Bajo — la información está disponible por otros canales.

12. **`CAN_SendStatusCurrent()` envía 4 ruedas pero INA226 son 6**  
    La función acepta 4 parámetros (FL, FR, RL, RR) pero el hardware tiene 6 INA226 (4 motores + batería + dirección). No hay mensaje CAN para batería ni dirección. La documentación debería aclararlo. **Impacto:** Bajo para HMI de tracción. Si la HMI necesita monitorizar batería, falta el canal CAN.

13. **Duplicación Ackermann**  
    Existen dos implementaciones de Ackermann: `Ackermann_Compute()` en `motor_control.c:539` y `Ackermann_ComputeWheelAngles()` en `ackermann.c:20`. Solo la segunda se usa activamente. La primera es código muerto. **Impacto:** Ninguno funcional, pero genera confusión para mantenimiento.

---

### 2.6 Tabla resumen de hallazgos

| # | Tipo | Severidad | Descripción | Archivo:Línea |
|---|------|-----------|-------------|---------------|
| 1 | Lógica | Baja | `Safety_GetFaultFlags()` usa == en vez de bitmask para errores | `safety_system.c:328-333` |
| 2 | Lógica | Media | `Wheel_ComputeSpeed()` recalcula en cada getter — dt=0 posible | `sensor_manager.c:56-79` |
| 3 | Docs | Baja | INA226 #0-3 asumidos como FL/FR/RL/RR sin validación en SW | `main.c:139-143` |
| 4 | — | — | Pedal local correctamente validado por safety | `main.c:121-126` ✅ |
| 5 | — | — | `last_can_rx_time` sin race condition | `safety_system.c:215` ✅ |
| 6 | Diseño | Info | Centering fault → DEGRADED (drive home), aceptable | `steering_centering.c:1126` |
| 7 | — | — | Recovery DEGRADED→ACTIVE funciona correctamente | `safety_system.c:773-784` ✅ |
| 8 | — | — | SAFE→ACTIVE requiere error clear primero, correcto | `safety_system.c:263` ✅ |
| 9 | Sensor | Media | DS18B20 ROM order no determinista | `sensor_manager.c:366-390` |
| 10 | Sensor | Baja | Pedal ADC sin detección de desconexión | `sensor_manager.c:91-99` |
| 11 | CAN | Baja | `CAN_SendError()` nunca se invoca (código muerto) | `can_handler.c:283-290` |
| 12 | CAN | Baja | Corriente batería/dirección sin CAN ID | `can_handler.c:178` |
| 13 | Code | Info | `Ackermann_Compute()` duplicada y no usada | `motor_control.c:539-564` |

---

## CONCLUSIÓN FINAL

### ¿Está el firmware listo para seguir avanzando?

**✅ SÍ — El firmware está en un estado funcional, coherente y seguro para continuar el desarrollo.**

**Fortalezas:**
- Arquitectura de seguridad sólida (state machine, command validation, per-wheel ABS/TCS)
- CAN protocol completo para las necesidades de la HMI
- Service mode maduro con gestión de módulos critical/non-critical
- Steering centering automático con múltiples protecciones
- Encoder health monitoring de 3 niveles
- Recovery paths bien definidos (DEGRADED→ACTIVE con debounce)

**Áreas de atención (no bloquean):**
- `Wheel_ComputeSpeed()` recalculada en cada getter (hallazgo #2)
- DS18B20 ROM order depende del hardware (hallazgo #9)
- Código muerto: `CAN_SendError()`, `Ackermann_Compute()`, `effortPct` (hallazgos #11, #13)
- Corriente de batería y dirección no reportada por CAN (hallazgo #12)

**Ninguno de estos hallazgos es bloqueante para avanzar con la integración ESP32-STM32.**
