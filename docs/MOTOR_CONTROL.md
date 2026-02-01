# ⚙️ Control de Motores - PWM Directo

**Control Directo de Motores sin PCA9685**

---

## 📋 Tabla de Contenidos

1. [Arquitectura de Control](#-arquitectura-de-control)
2. [Drivers BTS7960](#-drivers-bts7960)
3. [Configuración de Timers](#-configuración-de-timers)
4. [Control de Motores de Tracción](#-control-de-motores-de-tracción)
5. [Control del Motor de Dirección](#-control-del-motor-de-dirección)
6. [Modos de Operación](#-modos-de-operación)
7. [Limitaciones y Protecciones](#-limitaciones-y-protecciones)

---

## 🏗️ Arquitectura de Control

### ¿Por qué NO usar PCA9685?

El diseño original consideraba el módulo **PCA9685** (16 canales PWM vía I²C), pero se descartó por:

❌ **Desventajas del PCA9685:**
- Latencia I²C (~1-2 ms por actualización)
- Limitado a 1.6 kHz de frecuencia PWM máxima
- Requiere comunicación serial (overhead)
- Punto único de fallo (si I²C cae, se pierden todos los motores)
- Mayor complejidad de código

✅ **Ventajas del Control Directo STM32:**
- **PWM @ 20 kHz** (inaudible, reduce vibración)
- **Latencia <1 µs** (actualización instantánea)
- **Resolución 13-bit** (8500 pasos de duty cycle)
- **Hardware dedicado** (TIM1/TIM8 sin carga CPU)
- **Paralelismo total** (5 motores actualizados simultáneamente)

### Diagrama de Bloques

```
┌──────────────────────────────────────────────────────────────┐
│                         STM32G474RE                           │
│                                                                │
│  ┌─────────────────┐         ┌─────────────────┐             │
│  │  TIM1 (170 MHz) │         │  TIM8 (170 MHz) │             │
│  │   PWM @ 20 kHz  │         │   PWM @ 20 kHz  │             │
│  ├─────┬─────┬─────┤         └────────┬────────┘             │
│  │ CH1 │ CH2 │ CH3 │ CH4              │ CH3                  │
│  │ PA8 │ PA9 │PA10 │PA11              │ PC8                  │
│  └──┬──┴──┬──┴──┬──┴──┬──             └──┬──                 │
└─────┼─────┼─────┼─────┼─────────────────┼───────────────────┘
      │     │     │     │                 │
    ┌─▼───┐ │     │     │                 │
    │BTS  │ │     │     │                 │
    │7960 │ │     │     │                 │
    │ FL  │ │     │     │                 │
    └──┬──┘ │     │     │                 │
       │    │     │     │                 │
    [Motor] │     │     │              [Motor]
      FL    │     │     │             Steering
            │     │     │
          ┌─▼───┐ │     │
          │BTS  │ │     │
          │7960 │ │     │
          │ FR  │ │     │
          └──┬──┘ │     │
             │    │     │
          [Motor] │     │
            FR    │     │
                  │     │
                ┌─▼───┐ │
                │BTS  │ │
                │7960 │ │
                │ RL  │ │
                └──┬──┘ │
                   │    │
                [Motor] │
                  RL    │
                        │
                      ┌─▼───┐
                      │BTS  │
                      │7960 │
                      │ RR  │
                      └──┬──┘
                         │
                      [Motor]
                        RR
```

---

## 🔌 Drivers BTS7960

### Especificaciones del BTS7960

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Tensión máx.** | 43V | Alimentación motores |
| **Corriente continua** | 43A por canal | Half-bridge doble |
| **Frecuencia PWM** | Hasta 25 kHz | Óptimo: 20 kHz |
| **Lógica control** | 3.3V / 5V compatible | TTL/CMOS |
| **Protecciones** | Sobrecorriente, temperatura | Autolimitación |

### Señales de Control (por motor)

Cada BTS7960 requiere **3 señales** desde el STM32:

| Señal | Tipo | Función | Estado Inactivo |
|-------|------|---------|-----------------|
| **PWM** | Salida Timer (0-100%) | Modulación de potencia | 0% |
| **DIR** | GPIO Output | Dirección de giro (0=CW, 1=CCW) | LOW |
| **EN** | GPIO Output | Habilitación driver (1=ON, 0=OFF) | LOW |

### Tabla de Verdad BTS7960

| EN | DIR | PWM | Resultado |
|----|-----|-----|-----------|
| 0 | X | X | Motor DETENIDO (alta impedancia) |
| 1 | 0 | 50% | Motor gira CW a 50% potencia |
| 1 | 1 | 75% | Motor gira CCW a 75% potencia |
| 1 | X | 0% | Motor FRENADO (cortocircuito eléctrico) |

---

## ⏱️ Configuración de Timers

### TIM1 - Motores de Tracción (4 canales)

```c
// Configuración TIM1 @ 20 kHz
void TIM1_PWM_Init(void) {
    // 1. Habilitar reloj TIM1
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    // 2. Configuración base del timer
    TIM_HandleTypeDef htim1;
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;                    // Sin prescaler (170 MHz)
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 8499;                    // ARR: 170 MHz / 8500 = 20 kHz
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;            // Sin repetición
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);
    
    // 3. Configuración de canales PWM (4 canales idénticos)
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;          // PWM mode 1
    sConfigOC.Pulse = 0;                         // Duty cycle inicial = 0%
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // Activo en alto
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1); // FL
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2); // FR
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3); // RL
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4); // RR
    
    // 4. Iniciar PWM en todos los canales
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}
```

### TIM8 - Motor de Dirección (1 canal)

```c
// Configuración TIM8 @ 20 kHz (solo CH3)
void TIM8_PWM_Init(void) {
    __HAL_RCC_TIM8_CLK_ENABLE();
    
    TIM_HandleTypeDef htim8;
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 8499;                    // 20 kHz
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim8);
    
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}
```

### Cálculo de Duty Cycle

**Fórmula:**
```
Duty Cycle (%) = (CCR / (ARR + 1)) × 100
```

**Ejemplo:**
- ARR = 8499 (período)
- CCR = 4250 → 50.0% duty cycle
- CCR = 0 → 0% (motor detenido)
- CCR = 8499 → 100% (máxima potencia)

**Conversión de porcentaje a CCR:**
```c
uint16_t percent_to_CCR(uint8_t percent) {
    if (percent > 100) percent = 100;
    return (uint16_t)((percent * 8500UL) / 100);
}
```

---

## 🚗 Control de Motores de Tracción

### Estructura de Datos

```c
typedef struct {
    GPIO_TypeDef *DIR_PORT;
    uint16_t      DIR_PIN;
    GPIO_TypeDef *EN_PORT;
    uint16_t      EN_PIN;
    TIM_HandleTypeDef *htim;
    uint32_t      channel;
    int8_t        power_pct;      // -100 a +100 (negativo = reversa)
    uint8_t       enabled;
} Motor_t;

// Instancias de motores de tracción
Motor_t motor_FL = {GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_1, &htim1, TIM_CHANNEL_1, 0, 0};
Motor_t motor_FR = {GPIOC, GPIO_PIN_2, GPIOC, GPIO_PIN_3, &htim1, TIM_CHANNEL_2, 0, 0};
Motor_t motor_RL = {GPIOC, GPIO_PIN_4, GPIOC, GPIO_PIN_5, &htim1, TIM_CHANNEL_3, 0, 0};
Motor_t motor_RR = {GPIOC, GPIO_PIN_6, GPIOC, GPIO_PIN_7, &htim1, TIM_CHANNEL_4, 0, 0};
```

### Función de Control de Motor

```c
/**
 * @brief Establece la potencia de un motor con dirección
 * @param motor: Puntero a estructura Motor_t
 * @param power_pct: Potencia -100 a +100 (negativo = reversa)
 */
void Motor_SetPower(Motor_t *motor, int8_t power_pct) {
    // 1. Limitar rango
    if (power_pct > 100) power_pct = 100;
    if (power_pct < -100) power_pct = -100;
    
    // 2. Guardar valor
    motor->power_pct = power_pct;
    
    // 3. Calcular dirección y magnitud
    uint8_t direction = (power_pct >= 0) ? 0 : 1;  // 0=forward, 1=reverse
    uint8_t magnitude = (power_pct >= 0) ? power_pct : -power_pct;
    
    // 4. Aplicar dirección
    HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, direction);
    
    // 5. Aplicar PWM
    uint16_t ccr_value = percent_to_CCR(magnitude);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, ccr_value);
}

/**
 * @brief Habilita/deshabilita un motor
 */
void Motor_Enable(Motor_t *motor, uint8_t enable) {
    motor->enabled = enable;
    HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    if (!enable) {
        // Al deshabilitar, poner PWM a 0
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
    }
}
```

### Control Independiente por Rueda

```c
/**
 * @brief Establece potencia individual de las 4 ruedas
 * @note Permite control vectorial (torque vectoring)
 */
void Traction_SetIndividual(int8_t fl, int8_t fr, int8_t rl, int8_t rr) {
    Motor_SetPower(&motor_FL, fl);
    Motor_SetPower(&motor_FR, fr);
    Motor_SetPower(&motor_RL, rl);
    Motor_SetPower(&motor_RR, rr);
}

/**
 * @brief Establece potencia uniforme en todas las ruedas
 */
void Traction_SetUniform(int8_t power_pct) {
    Motor_SetPower(&motor_FL, power_pct);
    Motor_SetPower(&motor_FR, power_pct);
    Motor_SetPower(&motor_RL, power_pct);
    Motor_SetPower(&motor_RR, power_pct);
}
```

---

## 🎯 Control del Motor de Dirección

### Características Especiales

El motor de dirección tiene requisitos diferentes:

- **Control en lazo cerrado** con encoder E6B2-CWZ6C
- **Límites mecánicos** (-45° a +45°)
- **Velocidad limitada** (máx. 200°/s para seguridad)
- **Centrado automático** en modo seguro

### Estructura del Motor de Dirección

```c
typedef struct {
    Motor_t motor;
    int16_t current_position;    // -720 a +720 conteos encoder
    int16_t target_position;     // Posición deseada
    float   angle_deg;           // -45° a +45°
    uint8_t at_limit;            // Flag de límite alcanzado
} SteeringMotor_t;

SteeringMotor_t steering = {
    .motor = {GPIOC, GPIO_PIN_9, GPIOC, GPIO_PIN_10, &htim8, TIM_CHANNEL_3, 0, 0},
    .current_position = 0,
    .target_position = 0,
    .angle_deg = 0.0f,
    .at_limit = 0
};
```

### Controlador PID Simplificado

```c
#define STEERING_KP  2.0f
#define STEERING_KI  0.1f
#define STEERING_KD  0.5f

#define STEERING_MAX_SPEED  200  // °/s
#define STEERING_LIMIT_CNT  720  // ±180° en conteos encoder

float steering_integral = 0.0f;
int16_t steering_last_error = 0;

/**
 * @brief Actualiza el control PID del motor de dirección
 * @note Llamar a 100 Hz (cada 10 ms)
 */
void Steering_Update(void) {
    // 1. Leer posición actual del encoder
    steering.current_position = (int16_t)(TIM2->CNT - 32768);
    
    // 2. Calcular error
    int16_t error = steering.target_position - steering.current_position;
    
    // 3. Términos PID
    float P = STEERING_KP * error;
    steering_integral += error * 0.01f;  // dt = 10 ms
    float I = STEERING_KI * steering_integral;
    float D = STEERING_KD * (error - steering_last_error) / 0.01f;
    
    // 4. Salida PID
    float output = P + I + D;
    
    // 5. Limitar salida (anti-windup)
    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;
    
    // 6. Límites mecánicos (hard-stop)
    if (steering.current_position >= STEERING_LIMIT_CNT && output > 0) {
        output = 0;  // No permitir movimiento hacia derecha
        steering.at_limit = 1;
    } else if (steering.current_position <= -STEERING_LIMIT_CNT && output < 0) {
        output = 0;  // No permitir movimiento hacia izquierda
        steering.at_limit = 1;
    } else {
        steering.at_limit = 0;
    }
    
    // 7. Aplicar potencia al motor
    Motor_SetPower(&steering.motor, (int8_t)output);
    
    // 8. Actualizar estado
    steering.angle_deg = (float)steering.current_position * 0.25f;  // 1 cnt = 0.25°
    steering_last_error = error;
}

/**
 * @brief Establece ángulo deseado de dirección
 * @param angle_deg: Ángulo en grados (-45 a +45)
 */
void Steering_SetAngle(float angle_deg) {
    // Limitar ángulo
    if (angle_deg > 45.0f) angle_deg = 45.0f;
    if (angle_deg < -45.0f) angle_deg = -45.0f;
    
    // Convertir a conteos encoder (1° = 4 conteos)
    steering.target_position = (int16_t)(angle_deg * 4.0f);
}
```

### Calibración Inicial (Homing)

```c
/**
 * @brief Calibra el centro del motor de dirección
 * @note Ejecutar al inicio, busca el pulso Z del encoder
 */
void Steering_Calibrate(void) {
    // 1. Mover lentamente a la izquierda hasta encontrar pulso Z
    Motor_Enable(&steering.motor, 1);
    Motor_SetPower(&steering.motor, -20);  // 20% izquierda
    
    // 2. Esperar pulso Z (interrupción EXTI4 en PB4)
    while (!encoder_z_detected) {
        HAL_Delay(10);
    }
    
    // 3. Detener motor
    Motor_SetPower(&steering.motor, 0);
    
    // 4. Resetear contador del encoder al centro
    TIM2->CNT = 32768;
    steering.current_position = 0;
    steering.target_position = 0;
    
    encoder_z_detected = 0;
}
```

---

## 🎮 Modos de Operación

### Modo Normal (Control Activo)

```c
void Mode_Normal(void) {
    // Habilitar todos los motores
    Motor_Enable(&motor_FL, 1);
    Motor_Enable(&motor_FR, 1);
    Motor_Enable(&motor_RL, 1);
    Motor_Enable(&motor_RR, 1);
    Motor_Enable(&steering.motor, 1);
    
    // Habilitar relés
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); // RELAY_MAIN
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET); // RELAY_TRAC
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);  // RELAY_DIR
}
```

### Modo Seguro (Fail-Safe)

```c
void Mode_Safe(void) {
    // 1. Reducir potencia gradualmente (rampa de 1 segundo)
    for (int i = 100; i >= 0; i -= 5) {
        Traction_SetUniform(i);
        HAL_Delay(50);  // 50 ms × 20 pasos = 1 segundo
    }
    
    // 2. Deshabilitar motores de tracción
    Motor_Enable(&motor_FL, 0);
    Motor_Enable(&motor_FR, 0);
    Motor_Enable(&motor_RL, 0);
    Motor_Enable(&motor_RR, 0);
    
    // 3. Centrar dirección (posición 0°)
    Steering_SetAngle(0.0f);
    for (int i = 0; i < 200; i++) {  // 2 segundos máx
        Steering_Update();
        HAL_Delay(10);
        if (abs(steering.current_position) < 10) break;  // ±2.5° tolerancia
    }
    Motor_Enable(&steering.motor, 0);
    
    // 4. Abrir relés (cortar potencia)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); // RELAY_TRAC
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);  // RELAY_DIR
    // RELAY_MAIN permanece ON (watchdog externo lo apaga si STM32 falla)
}
```

### Modo Frenado Eléctrico

```c
/**
 * @brief Frenado regenerativo (cortocircuito eléctrico)
 * @note PWM=0% + EN=1 cortocircuita el motor (frenado fuerte)
 */
void Traction_ElectricBrake(void) {
    // Aplicar 0% PWM pero mantener habilitado
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    
    Motor_Enable(&motor_FL, 1);
    Motor_Enable(&motor_FR, 1);
    Motor_Enable(&motor_RL, 1);
    Motor_Enable(&motor_RR, 1);
}
```

---

## 🛡️ Limitaciones y Protecciones

### Protección Térmica

```c
/**
 * @brief Reduce potencia si temperatura alta
 */
void Thermal_Protection(void) {
    float temp_max = get_max_motor_temperature();
    
    if (temp_max > 80.0f) {
        // CRÍTICO: Reducir a 30%
        current_power_limit = 30;
    } else if (temp_max > 60.0f) {
        // WARNING: Reducir a 70%
        current_power_limit = 70;
    } else {
        // Normal: 100%
        current_power_limit = 100;
    }
}

/**
 * @brief Aplica límite térmico al comando de throttle
 */
int8_t apply_power_limit(int8_t requested_power) {
    int8_t limited = (requested_power * current_power_limit) / 100;
    return limited;
}
```

### Protección de Corriente

```c
#define CURRENT_LIMIT_CONTINUOUS  20.0f  // 20A por motor
#define CURRENT_LIMIT_PEAK        30.0f  // 30A por <2s

/**
 * @brief Monitorea corriente y limita si excede umbral
 */
void Current_Protection(void) {
    float current_FL = INA226_ReadCurrent(INA226_FL);
    
    if (current_FL > CURRENT_LIMIT_PEAK) {
        // Sobrecarga inmediata: Apagar motor
        Motor_Enable(&motor_FL, 0);
        set_error_flag(ERR_CURRENT_OVERLOAD);
    } else if (current_FL > CURRENT_LIMIT_CONTINUOUS) {
        // Sobrecarga sostenida: Reducir potencia
        Motor_SetPower(&motor_FL, motor_FL.power_pct * 0.7f);
    }
}
```

### Rate Limiter (Rampa de Aceleración)

```c
#define MAX_ACCEL_RATE  50  // Máx cambio: 50%/segundo

/**
 * @brief Limita la tasa de cambio de potencia
 * @param current: Potencia actual (%)
 * @param target: Potencia deseada (%)
 * @param dt: Delta tiempo (segundos)
 * @return Nueva potencia limitada
 */
int8_t rate_limit(int8_t current, int8_t target, float dt) {
    int8_t delta = target - current;
    int8_t max_delta = (int8_t)(MAX_ACCEL_RATE * dt);
    
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    
    return current + delta;
}
```

---

## 📖 Referencias

- [BTS7960 Datasheet](https://www.infineon.com/dgdl/Infineon-BTS7960-DS-v01_00-EN.pdf?fileId=db3a30433fa9412f013fbe32289b7c17)
- [STM32G4 Timer Cookbook](https://www.st.com/resource/en/application_note/an4013-introduction-to-timers-for-stm32-mcus-stmicroelectronics.pdf)
- [RM0440 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

---

**Última actualización:** 2026-02-01  
**Autor:** florinzgz  
**Proyecto:** STM32-Control-Coche-Marcos
