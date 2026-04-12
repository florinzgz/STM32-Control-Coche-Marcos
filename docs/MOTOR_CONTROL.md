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
- **Resolución ~12-bit** (4250 pasos de duty cycle, center-aligned)
- **Hardware dedicado** (TIM1/TIM8 sin carga CPU)
- **Paralelismo total** (5 motores actualizados simultáneamente)

### Diagrama de Bloques

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                              STM32G474RE                                     │
│                                                                              │
│  ┌──────────────────────┐  ┌──────────────────────┐  ┌────────────────────┐ │
│  │   TIM1 (advanced)    │  │   TIM8 (advanced)    │  │  TIM3 (general)    │ │
│  │   Center-aligned     │  │   Center-aligned     │  │  Center-aligned    │ │
│  │   BREAK2 → LOCKUP    │  │   BREAK2 → LOCKUP    │  │  No BREAK (sw)     │ │
│  │   ARR = 4249         │  │   ARR = 4249         │  │  ARR = 4249        │ │
│  │   20 kHz             │  │   20 kHz             │  │  20 kHz            │ │
│  ├──────┬───────┬───────┤  ├──────┬───────┬───────┤  ├──────┬─────────────┤ │
│  │ CH1  │  CH2  │ CH3   │  │ CH1  │  CH2  │ CH3   │  │ CH1  │  CH2        │ │
│  │ PA8  │  PA9  │ PA10  │  │ PC6  │  PC7  │ PC8   │  │ PA6  │  PA7        │ │
│  │RPWM  │ LPWM  │RPWM   │  │RPWM  │ LPWM  │RPWM   │  │RPWM  │ LPWM       │ │
│  │ FL   │  FL   │ FR    │  │ RL   │  RL   │ RR    │  │STEER │ STEER       │ │
│  │      │       │ CH4   │  │      │       │ CH4   │  │      │             │ │
│  │      │       │ PA11  │  │      │       │ PC9   │  │      │             │ │
│  │      │       │LPWM   │  │      │       │LPWM   │  │      │             │ │
│  │      │       │ FR    │  │      │       │ RR    │  │      │             │ │
│  └──┬───┴──┬────┴──┬────┘  └──┬───┴──┬────┴──┬────┘  └──┬───┴──┬──────────┘ │
└─────┼──────┼───────┼──────────┼──────┼───────┼──────────┼──────┼────────────┘
      │      │       │          │      │       │          │      │
    ┌─▼──┐ ┌─▼──┐  ┌─▼──┐    ┌─▼──┐ ┌─▼──┐  ┌─▼──┐    ┌─▼──┐ ┌─▼──┐
    │BTS │ │BTS │  │BTS │    │BTS │ │BTS │  │BTS │    │BTS │ │BTS │
    │7960│ │7960│  │7960│    │7960│ │7960│  │7960│    │7960│ │7960│
    │RPWM│ │LPWM│  │RPWM│    │RPWM│ │LPWM│  │RPWM│    │RPWM│ │LPWM│
    └──┬─┘ └──┬─┘  └──┬─┘    └──┬─┘ └──┬─┘  └──┬─┘    └──┬─┘ └──┬─┘
       └──┬───┘       └──┬───┘    └──┬───┘       └──┬───┘    └──┬───┘
       [Motor]         [Motor]    [Motor]         [Motor]    [Motor]
         FL              FR         RL              RR       STEER
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

Cada BTS7960 requiere **2 señales PWM** desde el STM32 (RPWM y LPWM del mismo timer):

| Señal | Tipo | Función | Estado Inactivo |
|-------|------|---------|-----------------|
| **RPWM** | Salida Timer (0-100%) | Giro hacia adelante | 0% |
| **LPWM** | Salida Timer (0-100%) | Giro hacia atrás | 0% |
| **EN** | GPIO Output o 3.3V fijo | Habilitación driver (1=ON, 0=OFF) | Según motor |

> **Nota:** Solo PC5 (EN_FL) y PC13 (EN_RR) son GPIO activos. Los demás BTS7960
> tienen R_EN/L_EN conectados fijo a 3.3 V.

### Tabla de Verdad BTS7960 (RPWM/LPWM)

| EN | RPWM | LPWM | Resultado |
|----|------|------|-----------|
| 0 | X | X | Motor DETENIDO (alta impedancia) |
| 1 | 50% | 0% | Motor gira CW a 50% potencia |
| 1 | 0% | 75% | Motor gira CCW a 75% potencia |
| 1 | 0% | 0% | Motor FRENADO (cortocircuito eléctrico) |

> **Nunca** se generan RPWM y LPWM distintos de cero simultáneamente.

---

## ⏱️ Configuración de Timers

### TIM1 - Motores de Tracción FL/FR (4 canales: RPWM + LPWM por motor)

```c
// Configuración TIM1 @ 20 kHz (center-aligned)
void TIM1_PWM_Init(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    TIM_HandleTypeDef htim1;
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;                    // Sin prescaler (170 MHz)
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = 4249;                    // Center-aligned: 170 MHz / (2 × 4250) = 20 kHz
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);
    
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1); // RPWM_FL (PA8)
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2); // LPWM_FL (PA9)
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3); // RPWM_FR (PA10)
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4); // LPWM_FR (PC3)
    
    // BREAK2 armed to Cortex LOCKUP — hardware PWM kill on CPU fault
    TIM_BreakDeadTimeConfigTypeDef sBreakCfg = {0};
    sBreakCfg.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakCfg.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakCfg.Break2State = TIM_BREAK2_ENABLE;
    sBreakCfg.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakCfg);
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}
```

### TIM8 - Motores de Tracción RL/RR (4 canales: RPWM + LPWM por motor)

```c
// Configuración TIM8 @ 20 kHz (center-aligned, BREAK2 → LOCKUP)
void TIM8_PWM_Init(void) {
    __HAL_RCC_TIM8_CLK_ENABLE();
    
    TIM_HandleTypeDef htim8;
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim8.Init.Period = 4249;                    // 170 MHz / (2 × 4250) = 20 kHz
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim8);
    
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1); // RPWM_RL (PC6)
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2); // LPWM_RL (PC7)
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3); // RPWM_RR (PC8)
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4); // LPWM_RR (PC9)
    
    // BREAK2 armed to Cortex LOCKUP
    TIM_BreakDeadTimeConfigTypeDef sBreakCfg = {0};
    sBreakCfg.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakCfg.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakCfg.Break2State = TIM_BREAK2_ENABLE;
    sBreakCfg.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakCfg);
    
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}
```

### TIM3 - Motor de Dirección (2 canales: RPWM + LPWM)

```c
// Configuración TIM3 @ 20 kHz (center-aligned, sin BREAK — protección por software)
void TIM3_PWM_Init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    TIM_HandleTypeDef htim3;
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim3.Init.Period = 4249;                    // 20 kHz
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.RepetitionCounter = 0;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim3);
    
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1); // RPWM_STEER (PA6)
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2); // LPWM_STEER (PA7)
    
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}
// Note: TIM3 has no BREAK input. Fault handlers (HardFault, BusFault, UsageFault)
// write TIM3->CCR1 = 0 and TIM3->CCR2 = 0 directly to stop steering on CPU fault.
```

### Cálculo de Duty Cycle

**Fórmula (center-aligned):**
```
Duty Cycle (%) = (CCR / ARR) × 100
```

**Ejemplo:**
- ARR = 4249 (período center-aligned)
- CCR = 2125 → 50.0% duty cycle
- CCR = 0 → 0% (motor detenido)
- CCR = 4249 → 100% (máxima potencia)

**Conversión de porcentaje a CCR:**
```c
uint16_t percent_to_CCR(uint8_t percent) {
    if (percent > 100) percent = 100;
    return (uint16_t)((percent * 4249UL) / 100);
}
```

---

## 🚗 Control de Motores de Tracción

### Arquitectura de Control (RPWM/LPWM directo)

El firmware actual genera **dos señales PWM por motor** (RPWM y LPWM) directamente
desde los timers hardware. No se usan pines DIR separados. La dirección se codifica
seleccionando qué canal (RPWM o LPWM) recibe el duty cycle:

```c
// API de control por motor — solo un canal activo a la vez
void Motor_SetSignedPWM_FL(int16_t speed);   // + = RPWM (avance), - = LPWM (retroceso)
void Motor_SetSignedPWM_FR(int16_t speed);
void Motor_SetSignedPWM_RL(int16_t speed);
void Motor_SetSignedPWM_RR(int16_t speed);
void Motor_SetSignedPWM_STEER(int16_t speed);
```

**Asignación de pines por motor:**

| Motor | RPWM | LPWM | Timer | EN |
|-------|------|------|-------|----|
| FL | PA8 (TIM1_CH1) | PA9 (TIM1_CH2) | TIM1 | PC5 (GPIO) |
| FR | PA10 (TIM1_CH3) | PC3 (TIM1_CH4) | TIM1 | PC0 (GPIO) |
| RL | PC6 (TIM8_CH1) | PC7 (TIM8_CH2) | TIM8 | PC1 (GPIO) |
| RR | PC8 (TIM8_CH3) | PC9 (TIM8_CH4) | TIM8 | PC13 (GPIO) |
| STEER | PA6 (TIM3_CH1) | PA7 (TIM3_CH2) | TIM3 | PC4 (GPIO) |

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

## 🔄 Corrección Diferencial Ackermann

### Geometría Ackermann para Distribución de Par

Cuando el vehículo gira, las ruedas interiores recorren un arco más corto que
las exteriores. Sin compensación, las ruedas interiores recibirían más par del
necesario, provocando subviraje y desgaste desigual de neumáticos.

El sistema utiliza la geometría Ackermann simplificada para ajustar
automáticamente el par de cada rueda durante los giros.

### Fórmulas

```
R = wheelbase / tan(|steering_angle|)      Radio de giro (centro del eje trasero)

correction = (track_width / 2) / R         Diferencia fraccional de velocidad

inside_mult  = 1.0 - correction            Ruedas interiores (reducidas)
outside_mult = 1.0 + correction            Ruedas exteriores (limitado a 1.0)
```

Donde:
- `wheelbase` = 0.95 m (distancia entre ejes)
- `track_width` = 0.70 m (distancia entre ruedas)
- `steering_angle` = ángulo actual de dirección (de `Steering_GetCurrentAngle()`)

### Parámetros

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| **Zona muerta** | ±2° | Sin corrección para conducción recta |
| **Diferencial máximo** | ±15% | Límite de desbalance de par |
| **Límite por rueda** | ≤ 1.0 | Nunca excede el par base |

### Posición en el Pipeline de Par

```
base_pwm
→ axle_split (4x4 50/50)
→ degraded_limit
→ obstacle_scale
→ ackermann_diff[i]          ← NUEVO
→ wheel_scale[i] (ABS/TCS)
→ final PWM
```

### Comportamiento por Ángulo de Dirección

| Ángulo | Corrección | Rueda interior | Rueda exterior |
|--------|-----------|----------------|----------------|
| 0° – 2° | 0% | 1.000 | 1.000 |
| 10° | 6.5% | 0.935 | 1.000 |
| 20° | 13.4% | 0.866 | 1.000 |
| ≥ 22.2° | 15% (capped) | 0.850 | 1.000 |
| 54° (max) | 15% (capped) | 0.850 | 1.000 |

### Convención de Signos

- **Giro izquierdo** (ángulo positivo): ruedas izquierdas = interiores
- **Giro derecho** (ángulo negativo): ruedas derechas = interiores

### Compatibilidad

- ✅ **ABS**: `wheel_scale[i]` se aplica DESPUÉS de `ackermann_diff[i]` — ambos son multiplicativos
- ✅ **TCS**: Mismo mecanismo que ABS — corrección Ackermann no interfiere
- ✅ **obstacle_scale**: Aplicado ANTES de Ackermann — se acumulan multiplicativamente
- ✅ **Modo degradado**: Power limit se aplica upstream — no afecta la corrección
- ✅ **Giro sobre eje (tank turn)**: Corrección Ackermann desactivada
- ✅ **NaN/Inf**: Todos los valores pasan por `sanitize_float()` (default: 1.0)

---

## 📖 Referencias

- [BTS7960 Datasheet](https://www.infineon.com/dgdl/Infineon-BTS7960-DS-v01_00-EN.pdf?fileId=db3a30433fa9412f013fbe32289b7c17)
- [STM32G4 Timer Cookbook](https://www.st.com/resource/en/application_note/an4013-introduction-to-timers-for-stm32-mcus-stmicroelectronics.pdf)
- [RM0440 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

---

**Última actualización:** 2026-02-13  
**Autor:** florinzgz  
**Proyecto:** STM32-Control-Coche-Marcos
