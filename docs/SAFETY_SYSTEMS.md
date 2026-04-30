# 🛡️ Sistemas de Seguridad - ABS/TCS

> ⚠ **ACTUALIZACIÓN relés (2026-04-23, CAN rev 1.3 compatible):** La secuencia de relés pasa de 3 fases (MAIN→TRAC→DIR) a **2 fases (TRAC→DIR, 50 ms settle)**. El hardware real no tiene contactor MAIN/Power-Hold. Bitmap de `relay_status` en heartbeat 0x001 byte 5 (layout 3 bits preservado): bit 0 = reservado/0, bit 1 = TRAC, bit 2 = DIR, bit 7 = SEQ_COMPLETE. `MODULE_RELAY_MAIN` renombrado a `MODULE_RELAY_TRAC` (ID 3 preservado). Ver `CAN_CONTRACT_FINAL.md`.

**Seguridad Funcional y Protección del Vehículo**

---

## 📋 Tabla de Contenidos

1. [Filosofía de Seguridad](#-filosofía-de-seguridad)
2. [Sistema ABS (Anti-lock Braking System)](#-sistema-abs-anti-lock-braking-system)
3. [Sistema TCS (Traction Control System)](#-sistema-tcs-traction-control-system)
4. [Watchdog y Auto-recuperación](#-watchdog-y-auto-recuperación)
5. [Protecciones Térmicas](#-protecciones-térmicas)
6. [Protecciones de Corriente](#-protecciones-de-corriente)
7. [Gestión de Fallos](#-gestión-de-fallos)
8. [Boot Validation Sequence](#-boot-validation-sequence)
9. [Command Acknowledgment Layer](#-command-acknowledgment-layer)

---

## 🎯 Filosofía de Seguridad

### Principios de Diseño

1. **Defense in Depth (Defensa en Profundidad)**
   - Múltiples capas de protección
   - Redundancia en sensores críticos
   - Fail-safe por hardware y software

2. **Fail-Safe por Defecto**
   - Relés abiertos en estado de reposo (LOW)
   - Motores deshabilitados sin señal activa
   - Watchdog externo corta alimentación si STM32 falla

3. **Autoridad Final del STM32**
   - ESP32 sugiere, STM32 decide
   - Validación de todos los comandos
   - Override automático en condiciones peligrosas

4. **Detección Temprana de Fallos**
   - Monitoreo continuo de sensores
   - Timeouts agresivos (250 ms CAN)
   - Alertas preventivas antes de fallo crítico

---

## 🚦 Sistema ABS (Anti-lock Braking System)

### Objetivo

Prevenir el **bloqueo de ruedas** durante frenado, manteniendo la capacidad de dirección y reduciendo distancia de frenado.

### Detección de Bloqueo

```c
#define ABS_SLIP_THRESHOLD  15  // 15% de deslizamiento = activar ABS

// Calcular deslizamiento por rueda
float slip = ((avg_speed - wheel_speed) * 100.0f) / avg_speed;
// Si slip > ABS_SLIP_THRESHOLD → rueda bloqueada
```

### Modulación por Pulsos (Pulse Modulation)

**ABS modulation upgraded from full cut to pulse reduction.**

El sistema ABS utiliza modulación por pulsos (onda cuadrada) en lugar de un
corte completo de par. Esto está alineado con el firmware de referencia
(`abs_system.cpp pressureReduction = 0.30`).

```c
#define ABS_BASE_REDUCTION   0.30f  // 30% reducción durante fase ON
#define ABS_PULSE_PERIOD_MS  80     // ciclo completo del pulso (ms)
#define ABS_PULSE_ON_RATIO   0.6f   // 60% del período = fase reducida

// Fase ON  (48 ms): wheel_scale = 0.70 → motor al 70%
// Fase OFF (32 ms): wheel_scale = 1.00 → motor al 100% (recuperación)
```

**¿Por qué pulsos en vez de corte completo?**
- Un corte del 100% (`wheel_scale = 0.0`) era demasiado agresivo
- La rueda perdía toda tracción instantáneamente, impidiendo recuperar agarre
- Los pulsos permiten que la rueda alterne entre reducción y recuperación
- Mejora el control direccional y reduce distancia de frenado
- La reducción del 30% es suficiente para romper el ciclo de bloqueo

### Algoritmo ABS

```c
void ABS_Update(void) {
    // 1. Puerta de velocidad mínima (10 km/h)
    if (avg_speed < 10.0f) return;

    // 2. Por cada rueda:
    for (uint8_t i = 0; i < 4; i++) {
        float slip = ((avg - spd[i]) * 100.0f) / avg;
        if (slip > ABS_SLIP_THRESHOLD) {
            // Rueda bloqueada → activar pulso
            // Máquina de estados por rueda (abs_pulse_phase[i])
            // Fase ON:  wheel_scale[i] = 1.0 - 0.30 = 0.70
            // Fase OFF: wheel_scale[i] = 1.0
        } else {
            // Sin bloqueo → restaurar potencia completa
            wheel_scale[i] = 1.0f;
            // Reiniciar estado del pulso
        }
    }

    // 3. Fallback global: si TODAS las ruedas bloquean → cortar tracción
    if (mask == 0x0F) {
        Traction_SetDemand(0);
    }
}
```

### Estado ABS en CAN

El estado ABS se reporta en el mensaje `0x203 - STATUS_SAFETY`:

```c
/**
 * @brief Envía estado de ABS por CAN
 */
void CAN_SendABSStatus(void) {
    uint8_t data[4];
    
    // Byte 0: Flags ABS por rueda
    data[0] = (wheel_speed.abs_active[0] << 0) |
              (wheel_speed.abs_active[1] << 1) |
              (wheel_speed.abs_active[2] << 2) |
              (wheel_speed.abs_active[3] << 3);
    
    // Byte 1: TCS flags (ver TCS)
    data[1] = 0x00;
    
    // Byte 2: Máximo deslizamiento (%)
    data[2] = get_max_slip_percent();
    
    // Byte 3: Checksum
    data[3] = CRC8(data, 3);
    
    CAN_Transmit(0x203, data, 4);
}
```

---

## 🏁 Sistema TCS (Traction Control System)

### Objetivo

Prevenir el **deslizamiento excesivo** de las ruedas motrices durante aceleración, mejorando tracción y estabilidad.

### Detección de Deslizamiento

```c
#define TCS_THRESHOLD_PERCENT  15  // 15% de deslizamiento = activar TCS

typedef struct {
    uint8_t tcs_active[4];  // Flags TCS por rueda
    uint8_t slip_percent[4];
} TCS_State_t;

TCS_State_t tcs_state;

/**
 * @brief Detecta si una rueda está deslizando (spinning)
 * @param wheel_speed: Velocidad de la rueda (mm/s)
 * @param avg_speed: Velocidad promedio del vehículo (mm/s)
 * @return 1 si detecta deslizamiento, 0 si no
 */
uint8_t TCS_DetectSpin(uint16_t wheel_speed, uint16_t avg_speed) {
    if (avg_speed < 200) return 0;  // Velocidad muy baja, permitir deslizamiento inicial
    
    // Calcular deslizamiento (rueda más rápida que vehículo)
    int16_t slip = wheel_speed - avg_speed;
    if (slip < 0) return 0;  // No hay deslizamiento
    
    uint8_t slip_percent = (slip * 100) / avg_speed;
    
    return (slip_percent > TCS_THRESHOLD_PERCENT);
}
```

### Algoritmo TCS

```c
/**
 * @brief Ejecuta el control TCS
 * @note Llamar a 100 Hz (cada 10 ms)
 */
void TCS_Update(void) {
    ABS_UpdateAverageSpeed();
    
    // Verificar cada rueda motriz
    tcs_state.tcs_active[0] = TCS_DetectSpin(wheel_speed.speed_FL, wheel_speed.speed_avg);
    tcs_state.tcs_active[1] = TCS_DetectSpin(wheel_speed.speed_FR, wheel_speed.speed_avg);
    tcs_state.tcs_active[2] = TCS_DetectSpin(wheel_speed.speed_RL, wheel_speed.speed_avg);
    tcs_state.tcs_active[3] = TCS_DetectSpin(wheel_speed.speed_RR, wheel_speed.speed_avg);
    
    // Aplicar corrección (reducir potencia en rueda deslizante)
    if (tcs_state.tcs_active[0]) {
        int8_t reduced_power = motor_FL.power_pct * 0.6f;  // Reducir 40%
        Motor_SetPower(&motor_FL, reduced_power);
    }
    if (tcs_state.tcs_active[1]) {
        Motor_SetPower(&motor_FR, motor_FR.power_pct * 0.6f);
    }
    if (tcs_state.tcs_active[2]) {
        Motor_SetPower(&motor_RL, motor_RL.power_pct * 0.6f);
    }
    if (tcs_state.tcs_active[3]) {
        Motor_SetPower(&motor_RR, motor_RR.power_pct * 0.6f);
    }
}
```

### TCS en Modo Sport

En algunos casos, puede ser deseable **deshabilitar TCS** para permitir maniobras avanzadas (drifting, etc.):

```c
uint8_t tcs_enabled = 1;  // 1 = habilitado, 0 = deshabilitado

void TCS_SetEnabled(uint8_t enable) {
    tcs_enabled = enable;
}

void TCS_Update(void) {
    if (!tcs_enabled) return;  // TCS deshabilitado, no hacer nada
    
    // ... resto del código TCS
}
```

---

## ⏱️ Watchdog y Auto-recuperación

### IWDG (Independent Watchdog)

Watchdog de hardware independiente que resetea el STM32 si no se refresca a tiempo.

```c
#define IWDG_TIMEOUT_MS  500  // 500 ms timeout

/**
 * @brief Inicializa el watchdog independiente
 */
void IWDG_Init(void) {
    IWDG_HandleTypeDef hiwdg;
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 1250;  // 500 ms @ 32 kHz / 64
    HAL_IWDG_Init(&hiwdg);
}

/**
 * @brief Refresca el watchdog (llamar cada <500 ms)
 */
void IWDG_Refresh(void) {
    HAL_IWDG_Refresh(&hiwdg);
}
```

### Detección de Reset por Watchdog

```c
/**
 * @brief Verifica si el último reset fue por watchdog
 */
void Check_Reset_Cause(void) {
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        // Reset por watchdog detectado
        error_log.watchdog_resets++;
        
        // Enviar diagnóstico por CAN
        CAN_SendError(ERR_WATCHDOG_RESET, 0x00, error_log.watchdog_resets);
        
        // Limpiar flag
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }
}
```

### Main Loop con Watchdog

```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    Peripherals_Init();
    IWDG_Init();
    
    while (1) {
        // 1. Refrescar watchdog
        IWDG_Refresh();
        
        // 2. Procesar CAN
        CAN_ProcessMessages();
        
        // 3. Leer sensores
        Sensors_Update();
        
        // 4. Ejecutar ABS/TCS
        ABS_Update();
        TCS_Update();
        
        // 5. Actualizar motores
        Motors_Update();
        
        // 6. Delay (100 Hz loop)
        HAL_Delay(10);
    }
}
```

---

## 🌡️ Protecciones Térmicas

### Umbrales de Temperatura

| Nivel | Rango (°C) | Acción | Color LED |
|-------|------------|--------|-----------|
| **Normal** | < 60°C | Sin limitación | 🟢 Verde |
| **Warning** | 60-80°C | Reducir potencia a 70% | 🟡 Amarillo |
| **Critical** | > 80°C | Reducir potencia a 30% o detener | 🔴 Rojo |

### Monitoreo de Temperatura

```c
#define TEMP_WARNING   60.0f
#define TEMP_CRITICAL  80.0f

typedef struct {
    float temp_FL;
    float temp_FR;
    float temp_RL;
    float temp_RR;
    float temp_ambient;
    float temp_max;
} Temperature_t;

Temperature_t temperature;

/**
 * @brief Lee las 5 temperaturas DS18B20
 */
void Temperature_ReadAll(void) {
    temperature.temp_FL = DS18B20_ReadTemperature(ROM_ADDR_FL);
    temperature.temp_FR = DS18B20_ReadTemperature(ROM_ADDR_FR);
    temperature.temp_RL = DS18B20_ReadTemperature(ROM_ADDR_RL);
    temperature.temp_RR = DS18B20_ReadTemperature(ROM_ADDR_RR);
    temperature.temp_ambient = DS18B20_ReadTemperature(ROM_ADDR_AMB);
    
    // Calcular máxima
    temperature.temp_max = fmax(fmax(temperature.temp_FL, temperature.temp_FR),
                                 fmax(temperature.temp_RL, temperature.temp_RR));
}
```

### Protección Térmica Activa

```c
uint8_t power_limit_percent = 100;

/**
 * @brief Aplica limitación térmica de potencia
 */
void Thermal_Protection(void) {
    Temperature_ReadAll();
    
    if (temperature.temp_max > TEMP_CRITICAL) {
        // CRÍTICO: Reducir a 30% o detener
        power_limit_percent = 30;
        set_warning_flag(WARNING_TEMP_CRITICAL);
        
        if (temperature.temp_max > 90.0f) {
            // Extremo: Detener completamente
            Mode_Safe();
            CAN_SendError(ERR_TEMP_CRITICAL, 0x01, (uint16_t)temperature.temp_max);
        }
    } else if (temperature.temp_max > TEMP_WARNING) {
        // WARNING: Reducir a 70%
        power_limit_percent = 70;
        set_warning_flag(WARNING_TEMP_HIGH);
    } else {
        // Normal: 100%
        power_limit_percent = 100;
        clear_warning_flag(WARNING_TEMP_HIGH);
    }
}

/**
 * @brief Aplica el límite térmico al comando de throttle
 */
int8_t apply_thermal_limit(int8_t requested_power) {
    return (requested_power * power_limit_percent) / 100;
}
```

### Protección Térmica de Emergencia Por Motor (130°C)

**Capa de protección hardware adicional** implementada directamente dentro de
`Traction_Update()` en `motor_control.c`. Es independiente de
`Safety_CheckTemperature()` y del estado global del sistema.

Trazabilidad: referencia firmware `traction.cpp` →
`TEMP_EMERGENCY_SHUTDOWN = 130°C` (corte inmediato por motor).

| Nivel | Umbral | Acción | Ámbito |
|-------|--------|--------|--------|
| **Emergency Cutoff** | ≥ 130°C | `wheel_scale[i] = 0.0` (motor individual) | Per-motor |
| **Recovery** | < 115°C | Restaurar `wheel_scale[i]` normal | Per-motor |

**Comportamiento:**

- Si la temperatura del motor `i` alcanza 130°C:
  - `wheel_scale[i]` se fuerza a `0.0f` → PWM = 0 para ese motor
  - Los demás motores **NO se ven afectados**
  - **NO** se fuerza estado SAFE global
  - **NO** se modifica la demanda global
  - Se registra fallo ServiceMode (`MODULE_FAULT_ERROR`) para el sensor
  - Se activa `SAFETY_ERROR_OVERTEMP` si no estaba ya activo

- Cuando la temperatura cae por debajo de 115°C (histéresis de 15°C):
  - Se permite que `wheel_scale[i]` vuelva al valor normal
  - Se limpia el fallo del módulo
  - **NO** se limpia automáticamente el estado SAFE si fue activado por otro motivo

```c
#define MOTOR_TEMP_CUTOFF_C    130.0f  /* Per-motor emergency cutoff    */
#define MOTOR_TEMP_RECOVERY_C  115.0f  /* Hysteresis recovery (15°C)    */

// Dentro de Traction_Update():
for (uint8_t i = 0; i < 4; i++) {
    float motor_temp = Temperature_Get(i);
    if (motor_temp >= MOTOR_TEMP_CUTOFF_C) {
        wheel_scale[i] = 0.0f;          // Corte inmediato
        motor_overtemp_cutoff[i] = true;
        ServiceMode_SetFault(temp_mod, MODULE_FAULT_ERROR);
    } else if (motor_overtemp_cutoff[i] && motor_temp < MOTOR_TEMP_RECOVERY_C) {
        motor_overtemp_cutoff[i] = false;  // Recuperación por histéresis
        ServiceMode_ClearFault(temp_mod);
    }
}
```

**Interacción con otras capas de seguridad:**

- Coexiste con ABS/TCS (el valor más restrictivo gana)
- Se aplica **antes** de `sanitize_float()` y **antes** del cálculo de PWM
- La fórmula final es: `FinalPWM = base_pwm × obstacle_scale × ackermann_diff[i] × wheel_scale[i]`
- La protección global de 90°C (`Safety_CheckTemperature() → SAFE`) sigue activa

### Degradación de Asistencia de Dirección en Modo DEGRADED

Cuando el sistema entra en estado `SYS_STATE_DEGRADED`, la asistencia de
dirección se reduce automáticamente según el **nivel de degradación interno**
(Phase 12) para disminuir la agresividad del motor de dirección y mejorar la
seguridad en modo limp/drive-home.

**Comportamiento:**

- La reducción se aplica **dentro de `Steering_ControlLoop()`** únicamente,
  después del cálculo PID y antes de la conversión a PWM.
- La salida PID se multiplica por `Safety_GetSteeringLimitFactor()` cuando
  `Safety_IsDegraded()` es `true`.
- El resultado pasa por `sanitize_float()` antes de cualquier uso.
- Los límites mecánicos (±45°), la validación de encoder, la neutralización
  en estado SAFE, y el rate limiter **NO se modifican**.
- `Steering_SetAngle()` **NO se modifica** — la reducción es exclusivamente
  en la etapa de control.

```c
// Dentro de Steering_ControlLoop(), después de PID_Compute y clamp:
if (Safety_IsDegraded()) {
    output *= Safety_GetSteeringLimitFactor();
}
output = sanitize_float(output, 0.0f);
```

**Interacción con otros estados:**

| Estado | Comportamiento de dirección |
|--------|-----------------------------|
| ACTIVE | PID output al 100% (sin reducción) |
| DEGRADED L1 | PID output × 0.85 (15% reducción) |
| DEGRADED L2 | PID output × 0.70 (30% reducción) |
| DEGRADED L3 | PID output × 0.60 (40% reducción) |
| SAFE | Motor neutralizado (`Steering_Neutralize()`) |

**No se modifica:** contrato CAN, máquina de estados de seguridad, ABS/TCS.

### Degradación Granular — Modo Limp Multi-Nivel (Phase 12)

El modo DEGRADED se refina internamente en tres niveles de severidad,
manteniendo `SYS_STATE_DEGRADED` como estado visible externamente por CAN.
No se modifican IDs de mensaje CAN, la máquina de estados, ni el watchdog.

#### Niveles de Degradación Internos

| Nivel | Potencia | Dirección | Tracción | Descripción |
|-------|----------|-----------|----------|-------------|
| **L1** | 70% | 85% | 80% | Fallo menor de sensor — limitación suave |
| **L2** | 50% | 70% | 60% | Advertencia térmica — limitación moderada |
| **L3** | 40% | 60% | 50% | Anomalía persistente — limitación máxima |

#### Razones de Degradación (`DegradedReason_t`)

| Razón | Valor | Nivel típico | Descripción |
|-------|-------|-------------|-------------|
| `DEGRADED_REASON_SENSOR_FAULT` | 1 | L1 | Fallo de plausibilidad de sensor individual |
| `DEGRADED_REASON_THERMAL_WARN` | 2 | L2 | Temperatura > umbral de advertencia (80°C) |
| `DEGRADED_REASON_OVERCURRENT` | 3 | L1 | Evento único de sobrecorriente |
| `DEGRADED_REASON_CENTERING_FAIL` | 4 | L1 | Fallo de centrado de dirección |
| `DEGRADED_REASON_BATTERY_UV` | 5 | L2 | Subtensión de batería (< 20.0 V) |
| `DEGRADED_REASON_DEMAND_ANOMALY` | 6 | L1 | Anomalía de demanda de acelerador |
| `DEGRADED_REASON_ENCODER_FAULT` | 7 | L1 | Fallo del encoder de dirección |
| `DEGRADED_REASON_PERSISTENT` | 8 | L3 | Múltiples fallos acumulados |

#### Pipeline de Escalación

```
Fallo detectado → Safety_SetDegradedLevel(nivel, razón)
                ↓
    Solo escalación (nunca reducción dentro de DEGRADED)
                ↓
    Nivel actual determina factores de escala:
        Safety_GetPowerLimitFactor()      → potencia
        Safety_GetSteeringLimitFactor()   → dirección
        Safety_GetTractionCapFactor()     → tracción
                ↓
    Recuperación a ACTIVE → nivel reset a DEGRADED_LEVEL_NONE
```

#### Mapeo a CAN

Todos los niveles internos reportan `SYS_STATE_DEGRADED` (= 3) en el
heartbeat CAN (0x001 byte 1).  No se añaden nuevos IDs de mensaje CAN.

```c
// can_handler.c — SIN CAMBIOS
payload[1] = (uint8_t)Safety_GetState();  // Siempre 3 para cualquier nivel
```

#### Contador de Telemetría

`Safety_GetDegradedTelemetryCount()` devuelve el número total de entradas
en modo degradado desde el último arranque.  Disponible para diagnóstico
interno; no se transmite por CAN (sin cambios de contrato).

#### Compatibilidad con Sistemas Existentes

- **ABS/TCS**: Sin cambios — operan independientemente por rueda
- **Obstacle scale**: Sin cambios — se aplica upstream
- **Ackermann**: Sin cambios — se aplica downstream
- **Watchdog**: Sin cambios — IWDG 500 ms sin bloqueo
- **Relays**: Sin cambios — secuenciación no-bloqueante preservada

---

## ⚡ Protecciones de Corriente

### Umbrales de Corriente

| Tipo | Límite | Acción |
|------|--------|--------|
| **Continuo** | 20A | Umbral normal de operación |
| **Pico** | 30A | Permitido <2 segundos |
| **Crítico** | 35A | Desconexión inmediata |

### Monitoreo de Corriente

```c
#define CURRENT_CONTINUOUS  20.0f
#define CURRENT_PEAK        30.0f
#define CURRENT_CRITICAL    35.0f

typedef struct {
    float current_FL;
    float current_FR;
    float current_RL;
    float current_RR;
    float current_STEER;
    float current_BATT;
    uint32_t overcurrent_time[4];  // Timestamp de inicio sobrecorriente
} Current_t;

Current_t current;

/**
 * @brief Lee corrientes de los 6 INA226
 */
void Current_ReadAll(void) {
    current.current_FL = INA226_ReadCurrent(INA226_ADDR_FL);
    current.current_FR = INA226_ReadCurrent(INA226_ADDR_FR);
    current.current_RL = INA226_ReadCurrent(INA226_ADDR_RL);
    current.current_RR = INA226_ReadCurrent(INA226_ADDR_RR);
    current.current_STEER = INA226_ReadCurrent(INA226_ADDR_STEER);
    current.current_BATT = INA226_ReadCurrent(INA226_ADDR_BATT);
}
```

### Protección de Corriente Activa

```c
/**
 * @brief Protección contra sobrecorriente
 */
void Current_Protection(void) {
    Current_ReadAll();
    uint32_t now = HAL_GetTick();
    
    // Motor FL
    if (current.current_FL > CURRENT_CRITICAL) {
        // Sobrecorriente crítica: Desconectar inmediatamente
        Motor_Enable(&motor_FL, 0);
        CAN_SendError(ERR_CURRENT_OVERLOAD, 0x01, (uint16_t)(current.current_FL * 10));
    } else if (current.current_FL > CURRENT_PEAK) {
        // Sobrecorriente pico: Iniciar timer
        if (current.overcurrent_time[0] == 0) {
            current.overcurrent_time[0] = now;
        } else if ((now - current.overcurrent_time[0]) > 2000) {
            // >2 segundos en pico: Reducir potencia
            Motor_SetPower(&motor_FL, motor_FL.power_pct * 0.7f);
        }
    } else {
        // Normal: Reset timer
        current.overcurrent_time[0] = 0;
    }
    
    // Repetir para FR, RL, RR...
}
```

### Monitoreo de Batería Principal

```c
#define BATTERY_LOW_VOLTAGE   20.0f  // Voltios
#define BATTERY_CRITICAL      18.0f

/**
 * @brief Protección contra batería baja
 */
void Battery_Protection(void) {
    float voltage = INA226_ReadVoltage(INA226_ADDR_BATT);
    
    if (voltage < BATTERY_CRITICAL) {
        // Batería crítica: Modo seguro
        Mode_Safe();
        CAN_SendError(ERR_BATTERY_CRITICAL, 0x00, (uint16_t)(voltage * 10));
    } else if (voltage < BATTERY_LOW_VOLTAGE) {
        // Batería baja: Advertencia
        set_warning_flag(WARNING_BATTERY_LOW);
    } else {
        clear_warning_flag(WARNING_BATTERY_LOW);
    }
}
```

---

## 🚨 Gestión de Fallos

### Códigos de Error

```c
typedef enum {
    ERR_NONE = 0x00,
    ERR_TIMEOUT_CAN = 0x01,
    ERR_TEMP_CRITICAL = 0x02,
    ERR_CURRENT_OVERLOAD = 0x03,
    ERR_ENCODER_FAULT = 0x04,
    ERR_WHEEL_SENSOR = 0x05,
    ERR_WATCHDOG_RESET = 0x10,
    ERR_I2C_TIMEOUT = 0x20,
    ERR_ONEWIRE_TIMEOUT = 0x21,
    ERR_BATTERY_CRITICAL = 0x30,
    ERR_UNKNOWN = 0xFF
} ErrorCode_t;
```

### Registro de Errores

```c
typedef struct {
    ErrorCode_t last_error;
    uint32_t    error_count;
    uint32_t    watchdog_resets;
    uint32_t    can_timeouts;
    uint32_t    temp_overloads;
    uint32_t    current_overloads;
} ErrorLog_t;

ErrorLog_t error_log = {0};

/**
 * @brief Registra un error en el log
 */
void Log_Error(ErrorCode_t error, uint8_t subsystem, uint16_t data) {
    error_log.last_error = error;
    error_log.error_count++;
    
    // Enviar por CAN
    CAN_SendError(error, subsystem, data);
}
```

### Priorización de Errores

```c
/**
 * @brief Determina la acción según severidad del error
 */
void Handle_Error(ErrorCode_t error) {
    switch (error) {
        case ERR_TIMEOUT_CAN:
        case ERR_TEMP_CRITICAL:
        case ERR_CURRENT_OVERLOAD:
        case ERR_BATTERY_CRITICAL:
            // Errores críticos → Modo seguro inmediato
            Mode_Safe();
            break;
            
        case ERR_ENCODER_FAULT:
        case ERR_WHEEL_SENSOR:
            // Errores de sensor → Limitar funcionalidad
            disable_affected_subsystem();
            break;
            
        case ERR_I2C_TIMEOUT:
        case ERR_ONEWIRE_TIMEOUT:
            // Errores de comunicación → Reintentar
            retry_communication();
            break;
            
        default:
            // Error desconocido → Continuar con precaución
            set_warning_flag(WARNING_UNKNOWN_ERROR);
            break;
    }
}
```

---

## 📊 Ciclo de Seguridad (Safety Loop)

### Main Safety Task @ 100 Hz

```c
/**
 * @brief Tarea principal de seguridad (ejecutar a 100 Hz)
 */
void Safety_Task(void) {
    // 1. Watchdog
    IWDG_Refresh();
    
    // 2. Verificar heartbeat CAN
    if ((HAL_GetTick() - last_can_heartbeat) > 250) {
        Handle_Error(ERR_TIMEOUT_CAN);
    }
    
    // 3. ABS/TCS
    ABS_Update();
    TCS_Update();
    
    // 4. Protección térmica (cada 1s)
    if (tick_counter % 100 == 0) {
        Thermal_Protection();
    }
    
    // 5. Protección de corriente (cada 100ms)
    if (tick_counter % 10 == 0) {
        Current_Protection();
        Battery_Protection();
    }
    
    // 6. Verificar sensores críticos (cada 500ms)
    if (tick_counter % 50 == 0) {
        Check_Encoder();
        Check_WheelSensors();
    }
    
    tick_counter++;
}
```

---

## 🔌 Secuenciación No-Bloqueante de Relés

### Máquina de Estados del Secuenciador de Relés

La secuenciación de relés utiliza una máquina de estados interna no-bloqueante
que se ejecuta desde el bucle de seguridad de 10 ms. Esto elimina el uso de
`HAL_Delay()` en la ruta de seguridad, permitiendo que las comprobaciones de
seguridad, CAN y watchdog continúen durante la secuencia de encendido.

```
                  Relay_PowerUp()
     ┌──────────┐  ────────────►  ┌──────────────┐
     │   IDLE   │                 │   MAIN_ON    │
     │ (relays  │  ◄────────────  │ Main relay   │
     │  off)    │  Relay_PowerDown│ energised    │
     └──────────┘                 └──────┬───────┘
          ▲                              │ 50 ms elapsed
          │ Relay_PowerDown()            ▼
          │                       ┌──────────────┐
          │                       │ TRACTION_ON  │
          ├───────────────────────│ Traction     │
          │                       │ relay ON     │
          │                       └──────┬───────┘
          │                              │ 20 ms elapsed
          │ Relay_PowerDown()            ▼
          │                       ┌──────────────┐
          └───────────────────────│  COMPLETE    │
                                  │ All relays   │
                                  │ ON           │
                                  └──────────────┘
```

### Comportamiento No-Bloqueante

| Aspecto | Antes (blocking) | Después (non-blocking) |
|---------|-------------------|------------------------|
| Mecanismo de temporización | `HAL_Delay()` | `HAL_GetTick()` + timestamps |
| Bloqueo del bucle principal | ~70 ms bloqueado | 0 ms bloqueado |
| Safety checks durante power-up | Suspendidas | Activas |
| Watchdog durante power-up | No refrescado | Refrescado normalmente |
| CAN processing durante power-up | Suspendido | Activo |

### Garantías de Temporización

- **Traction → Direction:** ≥ 50 ms (RELAY_TRAC_SETTLE_MS)
- Resolución de temporización: 10 ms (período del bucle de seguridad)
- Peor caso de latencia adicional: +10 ms por etapa (resolución del tick)

### Seguridad de Re-entrada

- Llamar `Relay_PowerUp()` durante una secuencia en progreso es un no-op
- Llamar `Relay_PowerUp()` cuando ya está en COMPLETE es un no-op
- `Relay_PowerDown()` cancela inmediatamente cualquier secuencia en progreso
  y desactiva todos los relés en orden inverso

### Integración con Estados del Sistema

| Estado del Sistema | Acción sobre relés |
|--------------------|-------------------|
| STANDBY → ACTIVE | `Relay_PowerUp()` inicia secuencia |
| ACTIVE / DEGRADED | Relés encendidos (COMPLETE) |
| → SAFE | `Relay_PowerDown()` inmediato (vía `Safety_FailSafe()`) |
| → ERROR | `Relay_PowerDown()` inmediato (vía `Safety_PowerDown()`) |

### Por Qué los Delays Bloqueantes Son Peligrosos en Firmware de Seguridad

In a safety-critical embedded control loop, calling `HAL_Delay()` inside
relay sequencing creates the following hazards:

1. **Watchdog starvation** — The IWDG (500 ms timeout) is refreshed only
   from the main loop.  A 70 ms blocking delay reduces the available
   margin and, if combined with other latencies, can trigger an
   unintended watchdog reset.

2. **Safety check blackout** — While the CPU spins inside `HAL_Delay()`,
   no overcurrent, overtemperature, CAN timeout, or sensor plausibility
   checks execute.  A fault arising during relay power-up would go
   undetected for the full duration of the delay.

3. **CAN message loss** — Incoming CAN frames from the ESP32 accumulate
   in the FDCAN FIFO during the blocking window.  If the FIFO
   overflows, heartbeat or command frames are silently dropped,
   potentially triggering a false CAN timeout after the delay ends.

4. **Determinism violation** — The 10 ms safety loop period is the
   foundation of all timing guarantees (ABS pulse modulation, TCS
   recovery ramp, steering PID).  Injecting a variable-length blocking
   call breaks the deterministic scheduling contract.

5. **Deadlock risk** — If a fault triggers `Relay_PowerDown()` from an
   ISR or higher-priority context while `Relay_PowerUp()` is still
   blocking, the power-down action is deferred until the delay
   completes — exactly the opposite of fail-safe behaviour.

The non-blocking state machine (`RelaySeqState_t`) resolves all five
hazards by returning control to the main loop within microseconds on
every call.  The relay transitions progress by comparing elapsed ticks,
ensuring that every 10 ms iteration also runs safety checks, refreshes
the watchdog, and processes CAN traffic.

### Análisis de Regresión — Secuenciación de Relés

| Aspecto | Verificación |
|---------|-------------|
| Orden de activación | Traction → Direction |
| Orden de desactivación | Direction → Traction |
| Temporización Traction → Dir | ≥ 50 ms (`RELAY_TRAC_SETTLE_MS`) — 2 etapas solamente |
| Pines GPIO utilizados | `PIN_RELAY_TRAC`, `PIN_RELAY_DIR` en GPIOC; PC10 disponible |
| Contrato CAN | Ningún mensaje añadido, modificado o eliminado |
| Máquina de estados del sistema | `Safety_SetState()` no modificada |
| Watchdog durante power-up | IWDG se refresca normalmente (no bloqueado) |
| Safety checks durante power-up | Todas activas (overcurrent, overtemp, CAN timeout, sensores) |
| Re-entrada segura | `Relay_PowerUp()` doble = no-op; `Relay_PowerDown()` cancela secuencia en progreso |
| Transición SAFE | `Safety_FailSafe()` fuerza parada de actuadores inmediata |
| Transición ERROR | `Safety_PowerDown()` → `Relay_PowerDown()` inmediato |
| ABS/TCS | No modificados — operan independientemente de la secuencia de relés |
| Obstacle safety | No modificado — `Obstacle_Update()` no afectado |
| Ackermann correction | No modificada — opera en pipeline de tracción |

---

## 🔧 CAN Bus-Off Detection and Recovery

### CAN Fault Hierarchy

The STM32 safety system distinguishes between two independent CAN fault
conditions.  Each has its own detection mechanism, error code, and recovery
path.  Both can occur simultaneously; the safety state machine handles them
independently.

```
                          CAN Fault Hierarchy
┌──────────────────────────────────────────────────────────┐
│                                                          │
│  ┌──────────────────────┐    ┌─────────────────────────┐ │
│  │ Heartbeat Timeout    │    │ Bus-Off Condition       │ │
│  │ (Application Layer)  │    │ (Physical/Data-Link)    │ │
│  ├──────────────────────┤    ├─────────────────────────┤ │
│  │ Cause: ESP32 stops   │    │ Cause: Excessive CAN    │ │
│  │ sending 0x011 msgs   │    │ errors (EMI, wiring,    │ │
│  │ for > 250 ms         │    │ short circuit, bad      │ │
│  │                      │    │ termination)            │ │
│  ├──────────────────────┤    ├─────────────────────────┤ │
│  │ Detection: Software  │    │ Detection: FDCAN PSR    │ │
│  │ timestamp comparison │    │ register BusOff flag    │ │
│  │ in Safety_Check-     │    │ polled every 10 ms      │ │
│  │ CANTimeout()         │    │ in CAN_CheckBusOff()    │ │
│  ├──────────────────────┤    ├─────────────────────────┤ │
│  │ Error code: 3        │    │ Error code: 13          │ │
│  │ SAFETY_ERROR_CAN_    │    │ SAFETY_ERROR_CAN_       │ │
│  │ TIMEOUT              │    │ BUSOFF                  │ │
│  ├──────────────────────┤    ├─────────────────────────┤ │
│  │ Action: → SAFE state │    │ Action: → SAFE state    │ │
│  │ Traction stopped     │    │ Traction stopped        │ │
│  │ Steering centered    │    │ Steering centered       │ │
│  ├──────────────────────┤    ├─────────────────────────┤ │
│  │ Recovery: Automatic  │    │ Recovery: Automatic     │ │
│  │ when heartbeat 0x011 │    │ peripheral reinit       │ │
│  │ resumes              │    │ (Stop→DeInit→Init→      │ │
│  │                      │    │ Start), then heartbeat  │ │
│  │                      │    │ timeout handles         │ │
│  │                      │    │ SAFE→ACTIVE             │ │
│  └──────────────────────┘    └─────────────────────────┘ │
│                                                          │
│  Both faults → SYS_STATE_SAFE (actuators inhibited)      │
│  Neither blocks the main loop or watchdog                │
└──────────────────────────────────────────────────────────┘
```

### Heartbeat Timeout vs. Bus-Off

| Aspect | Heartbeat Timeout | Bus-Off |
|--------|-------------------|---------|
| **Layer** | Application (CAN message arrival) | Data-link (FDCAN hardware error state) |
| **Root cause** | ESP32 crash, software hang, power loss | Excessive TX/RX errors: EMI, wiring fault, missing termination resistor, short circuit |
| **Detection** | `Safety_CheckCANTimeout()` — software timestamp comparison in 10 ms loop | `CAN_CheckBusOff()` — polls `HAL_FDCAN_GetProtocolStatus()` BusOff flag in 10 ms loop |
| **Threshold** | No heartbeat for > 250 ms | CAN standard: TX error counter ≥ 256 |
| **Error code** | `SAFETY_ERROR_CAN_TIMEOUT` (3) | `SAFETY_ERROR_CAN_BUSOFF` (13) |
| **State transition** | → SAFE | → SAFE |
| **Recovery** | Automatic when heartbeat resumes | Peripheral reinit (non-blocking, 500 ms retry interval, max 10 attempts), then heartbeat timeout handles SAFE→ACTIVE |
| **Typical scenario** | ESP32 reboot, CAN connector momentarily disconnected | CAN bus wiring degradation, missing 120 Ω termination, EMI from motor driver |

### Bus-Off Recovery Behavior

When the FDCAN hardware enters bus-off state:

1. **Immediate actions:**
   - `SAFETY_ERROR_CAN_BUSOFF` is set
   - System transitions to `SYS_STATE_SAFE` (traction stopped, steering centered)
   - `busoff_count` in CAN statistics is incremented

2. **Recovery sequence (non-blocking):**
   - First attempt starts immediately upon detection
   - Subsequent retries are spaced by 500 ms (`CAN_BUSOFF_RETRY_INTERVAL_MS`)
   - Maximum 10 recovery attempts (`CAN_BUSOFF_MAX_RETRIES`)
   - Each attempt executes: `HAL_FDCAN_Stop()` → `HAL_FDCAN_DeInit()` → `HAL_FDCAN_Init()` → `CAN_ConfigureFilters()` → `HAL_FDCAN_ActivateNotification()` → `HAL_FDCAN_Start()`
   - If any step fails, the attempt is abandoned and retried at the next interval

3. **After successful recovery:**
   - Bus-off flag is cleared
   - `SAFETY_ERROR_CAN_BUSOFF` is cleared
   - System remains in SAFE state until ESP32 heartbeat resumes
   - `Safety_CheckCANTimeout()` handles SAFE → ACTIVE transition when heartbeat arrives

4. **If recovery fails after 10 attempts:**
   - System remains in SAFE state permanently
   - No further recovery attempts (prevents infinite restart loops)
   - Operator intervention required (power cycle)

5. **Watchdog safety:**
   - The main loop continues running during all recovery attempts
   - `HAL_IWDG_Refresh()` is called every iteration — no watchdog timeout risk
   - No blocking delays in the recovery path

---

## 🔍 Demand Anomaly Detection (Security Hardening Phase 2)

Defence-in-depth layer in the traction pipeline that detects implausible
throttle demand patterns.  Traced to reference firmware `traction.cpp`
demand anomaly detection.

### Throttle Step-Rate Validation

Detects unrealistic demand jumps (e.g. 0% → 100% within < 10 ms) that
could indicate sensor faults, CAN corruption, or injection attacks.

```c
#define MAX_THROTTLE_STEP_PER_10MS  15.0f  // Max allowed raw jump (%/10ms)

// Applied to RAW input in Traction_SetDemand(), before EMA/ramp filtering.
// If exceeded:
//   1. Clamp throttle to maximum allowed step
//   2. Raise SAFETY_ERROR_SENSOR_FAULT
//   3. Transition ACTIVE → DEGRADED (NOT SAFE)
```

### Negative / Out-of-Range Demand Validation

After gear-based power scaling in `Traction_Update()`, if `effective_demand`
is > 100 % or < 0 % without dynamic braking active:

- Force demand to 0
- Raise `SAFETY_ERROR_SENSOR_FAULT`
- Transition ACTIVE → DEGRADED

Dynamic braking legitimately produces negative `effective_demand` and is
not flagged.

### Frozen Pedal Detection

If the pedal value remains identical (within 0.5 % tolerance) for more
than 5 seconds while vehicle speed changes by more than 3 km/h, a
stuck-pedal anomaly is raised.

```c
#define FROZEN_PEDAL_TIMEOUT_MS      5000   // Detection timeout (ms)
#define FROZEN_PEDAL_SPEED_DELTA_KMH 3.0f   // Speed divergence threshold

// Non-blocking, timestamp-based.
// Raises SAFETY_ERROR_SENSOR_FAULT → DEGRADED (not SAFE).
// Resets tracking after detection to prevent re-triggering every cycle.
```

### Time-Based Plausibility

All anomaly detection logic stores previous demand values and timestamps.
All checks are non-blocking — no `HAL_Delay`, no busy loops.  The
implementation uses `HAL_GetTick()` for timestamping, consistent with
existing pedal ramp and steering rate-limit logic.

### Coexistence

Demand anomaly detection operates upstream of:
- Ackermann differential torque correction
- ABS/TCS per-wheel modulation
- Obstacle scale limiter
- 4×4 axle split
- Degraded power limiter

It does NOT modify any CAN IDs, payloads, or the system state machine.
It reuses `SAFETY_ERROR_SENSOR_FAULT` — no new CAN-visible error codes.

---

## 🚀 Boot Validation Sequence

### Overview

A deterministic boot validation checklist is executed during STANDBY before
the system is allowed to transition to ACTIVE. This ensures all critical
sensor subsystems are in a plausible state before actuator commands are
accepted.

### Boot Sequence Diagram

```
BOOT ──────────► STANDBY ─────────────────────► ACTIVE
 │                  │                              ▲
 │  Peripheral      │  Boot Validation             │
 │  Init Complete   │  Checklist (10 ms loop)      │
 │                  │                              │
 │                  ├─ Temperature plausible? ─────┤
 │                  ├─ Current plausible? ─────────┤
 │                  ├─ Encoder healthy? ───────────┤
 │                  ├─ Battery ≥ 20.0 V? ──────────┤
 │                  ├─ No SAFETY_ERROR? ───────────┤
 │                  ├─ CAN not bus-off? ───────────┤
 │                  ├─ Steering calibrated? ───────┤
 │                  └─ ESP32 heartbeat alive? ─────┘
 │
 │  If ANY check fails → remain in STANDBY
 │  (no SAFE transition unless critical fault)
```

### Validation Checks

| # | Check | Source | Threshold | Failure Action |
|---|-------|--------|-----------|----------------|
| 1 | Temperature plausible | `Temperature_Get(0..4)` | -40 °C to +125 °C | Remain STANDBY, log WARNING on temp sensor |
| 2 | Current plausible | `Current_GetAmps(0..3)` | -1.0 A to 50.0 A | Remain STANDBY, log WARNING on current sensor |
| 3 | Encoder healthy | `Encoder_HasFault()` | No active fault | Remain STANDBY, log WARNING on encoder |
| 4 | Battery voltage OK | `Voltage_GetBus(4)` | ≥ 20.0 V | Remain STANDBY |
| 5 | No safety error | `Safety_GetError()` | `== SAFETY_ERROR_NONE` | Remain STANDBY |
| 6 | CAN not bus-off | `CAN_IsBusOff()` | `== false` | Remain STANDBY |

### Files

| File | Description |
|------|-------------|
| `Core/Inc/boot_validation.h` | `BootValidationStatus` struct and public API |
| `Core/Src/boot_validation.c` | `BootValidation_Run()` and `BootValidation_IsPassed()` |

### Integration Points

- **`main.c`** — `BootValidation_Run()` called in the 10 ms task loop during STANDBY
- **`safety_system.c`** — STANDBY → ACTIVE transition gated by `BootValidation_IsPassed()`
- **`can_handler.c`** — `CAN_IsBusOff()` accessor added (exposes existing `busoff_active`)

### Safety Impact

- **CAN contract**: Unchanged — no new message IDs, no heartbeat changes
- **Watchdog timing**: Unaffected — no blocking delays added
- **State machine transitions**: Unmodified — only the STANDBY → ACTIVE condition adds a boot validation gate
- **Fail-safe**: If validation fails, system remains in STANDBY (not forced to SAFE)

---

## 🔄 Command Acknowledgment Layer

### Overview (Phase 13)

The command acknowledgment layer ensures closed-loop confirmation of discrete ESP32 commands (mode changes, gear changes, service commands). The STM32 sends a CMD_ACK (0x103) frame after safety validation, informing the ESP32 whether the command was accepted or rejected.

### Design Principles

1. **Post-validation ACK** — ACK is sent only after the safety system has validated the command and the STM32 has made an accept/reject decision. No premature acknowledgment.
2. **Non-blocking** — ACK transmission uses the existing `TransmitFrame()` helper. No blocking delays on either side.
3. **Bounded timeout** — ESP32 waits at most 200 ms (`ACK_TIMEOUT_MS`) for ACK. No infinite retry loops.
4. **Selective acknowledgment** — Only on-demand commands (CMD_MODE 0x102, SERVICE_CMD 0x110) are acknowledged. High-frequency commands (throttle at 50 ms, steering at 50 ms) are confirmed implicitly via status messages.
5. **Float sanitization** — All float computations in the ACK path (e.g. average speed for gear validation) pass through `sanitize_float()`.

### ACK Result Codes

| Code | Name | Trigger |
|------|------|---------|
| 0 | ACK_OK | Command accepted and applied |
| 1 | ACK_REJECTED | Command rejected — speed too high, mode not allowed |
| 2 | ACK_INVALID | Payload malformed — bad DLC, out-of-range parameter |
| 3 | ACK_BLOCKED_BY_SAFETY | System state not ACTIVE or DEGRADED |

### ACK Flow

```
ESP32 → STM32:  CMD_MODE (0x102) or SERVICE_CMD (0x110)
    ↓
STM32: Safety_IsCommandAllowed() check
    ↓ (blocked)                    ↓ (allowed)
    ACK_BLOCKED_BY_SAFETY          Validate parameters
                                       ↓ (invalid)      ↓ (valid)
                                       ACK_INVALID       Execute command
                                                              ↓ (rejected)  ↓ (accepted)
                                                              ACK_REJECTED   ACK_OK
    ↓
STM32 → ESP32:  CMD_ACK (0x103) — 3 bytes: cmd_id_low, result, system_state
```

### ESP32 Behavior

- Before sending a mode/gear/service command, the ESP32 registers an ACK wait via `ackBeginWait(cmdIdLow)`.
- The `ackCheck()` function is called every loop iteration (non-blocking).
- If ACK arrives with matching `cmd_id_low`, the pending wait is cleared and the result is available for UI update.
- If `ACK_TIMEOUT_MS` (200 ms) expires, the ESP32 marks the ACK as timed out and logs the event. No automatic retry — the operator can re-issue the command manually.

### Safety Impact

- **CAN contract**: New message CMD_ACK (0x103) added. No existing IDs or payloads changed.
- **Heartbeat format**: Unchanged — 4 bytes, same structure.
- **SystemState enum**: Unchanged — 6 states.
- **State machine transitions**: Unmodified — ACK does not alter any safety state transition.
- **Watchdog timing**: Unaffected — no blocking delays added.
- **Attack surface**: No new attack vector — ACK is output-only from STM32, no new input path. All enums are uint8_t.

### Source Files

- **`Core/Inc/can_handler.h`** — `CAN_ID_CMD_ACK`, `CAN_AckResult_t` enum, `CAN_SendCommandAck()` prototype
- **`Core/Src/can_handler.c`** — `CAN_SendCommandAck()` implementation, ACK integration in CMD_MODE and SERVICE_CMD handlers
- **`esp32/include/can_ids.h`** — `CMD_ACK`, `AckResult` enum, `ACK_TIMEOUT_MS`
- **`esp32/src/vehicle_data.h`** — `AckData` struct
- **`esp32/src/can_rx.cpp`** — `decodeCommandAck()` decoder
- **`esp32/src/main.cpp`** — `ackBeginWait()`, `ackCheck()` non-blocking ACK tracking

---

## 📖 Referencias

- [ISO 26262 - Functional Safety](https://www.iso.org/standard/68383.html)
- [STM32 Safety Manual](https://www.st.com/resource/en/application_note/an5156-safety-manual-for-stm32-mcus-stmicroelectronics.pdf)
- [ABS/TCS Theory](https://www.bosch-mobility-solutions.com/en/products-and-services/passenger-cars-and-light-commercial-vehicles/driver-assistance-systems/anti-lock-braking-system/)

---

**Última actualización:** 2026-02-13  
**Autor:** florinzgz  
**Proyecto:** STM32-Control-Coche-Marcos
