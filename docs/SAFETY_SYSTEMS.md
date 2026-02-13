# 🛡️ Sistemas de Seguridad - ABS/TCS

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
- La fórmula final sigue siendo: `FinalPWM = base_pwm × obstacle_scale × wheel_scale[i]`
- La protección global de 90°C (`Safety_CheckTemperature() → SAFE`) sigue activa

### Degradación de Asistencia de Dirección en Modo DEGRADED

Cuando el sistema entra en estado `SYS_STATE_DEGRADED`, la asistencia de
dirección se reduce automáticamente en un **40%** (multiplicador 0.6) para
disminuir la agresividad del motor de dirección y mejorar la seguridad en
modo limp/drive-home.

**Comportamiento:**

- La reducción se aplica **dentro de `Steering_ControlLoop()`** únicamente,
  después del cálculo PID y antes de la conversión a PWM.
- La salida PID se multiplica por `0.6f` cuando `Safety_IsDegraded()` es
  `true`.
- El resultado pasa por `sanitize_float()` antes de cualquier uso.
- Los límites mecánicos (±45°), la validación de encoder, la neutralización
  en estado SAFE, y el rate limiter **NO se modifican**.
- `Steering_SetAngle()` **NO se modifica** — la reducción es exclusivamente
  en la etapa de control.

```c
// Dentro de Steering_ControlLoop(), después de PID_Compute y clamp:
if (Safety_IsDegraded()) {
    output *= 0.6f;   // 40% reducción de torque de dirección
}
output = sanitize_float(output, 0.0f);
```

**Interacción con otros estados:**

| Estado | Comportamiento de dirección |
|--------|-----------------------------|
| ACTIVE | PID output al 100% (sin reducción) |
| DEGRADED | PID output × 0.6 (40% reducción) |
| SAFE | Motor neutralizado (`Steering_Neutralize()`) |

**No se modifica:** contrato CAN, máquina de estados de seguridad, ABS/TCS.

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

## 📖 Referencias

- [ISO 26262 - Functional Safety](https://www.iso.org/standard/68383.html)
- [STM32 Safety Manual](https://www.st.com/resource/en/application_note/an5156-safety-manual-for-stm32-mcus-stmicroelectronics.pdf)
- [ABS/TCS Theory](https://www.bosch-mobility-solutions.com/en/products-and-services/passenger-cars-and-light-commercial-vehicles/driver-assistance-systems/anti-lock-braking-system/)

---

**Última actualización:** 2026-02-13  
**Autor:** florinzgz  
**Proyecto:** STM32-Control-Coche-Marcos
