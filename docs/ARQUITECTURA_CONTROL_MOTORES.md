# ARQUITECTURA DE CONTROL DE MOTORES

## Descripción General

Sistema de control de motores para vehículo eléctrico 4x4 con:
- **4 motores de tracción** independientes (FL, FR, RL, RR)
- **1 motor de dirección** con encoder de posición
- **Control PWM directo** mediante TIM1/TIM8 (eliminando PCA9685)
- **Geometría Ackermann** para diferencial virtual en curvas
- **Modos 4x4/4x2** configurables
- **Rotación sobre eje** (tank turn) opcional

## Comparativa: Control Directo vs PCA9685

### Método Anterior: PCA9685 (I2C)
```
STM32 → I2C (400kHz) → PCA9685 → 16 canales PWM
```

**Limitaciones:**
- ⚠️ Latencia I2C: ~2-3ms por actualización
- ⚠️ Frecuencia PWM fija: 50-1500Hz (limitada)
- ⚠️ Resolución: 12-bit (0-4095)
- ⚠️ Dependencia externa (punto único de fallo)
- ⚠️ Comunicación serie susceptible a ruido

### Método Actual: TIM1/TIM8 Directo
```
STM32 → Timer Hardware → PWM Pins
```

**Ventajas:**
- ✅ Latencia < 100μs (actualización inmediata)
- ✅ Frecuencia PWM: 20kHz (silencioso, eficiente)
- ✅ Resolución: 13-bit efectivo (0-8499 @ 170MHz)
- ✅ Hardware dedicado (sin overhead CPU)
- ✅ Sincronización perfecta entre canales
- ✅ Control simultáneo 4 motores sin delay

### Configuración TIM1/TIM8

**TIM1**: Motores de tracción (4 canales)
```c
// Configuración TIM1
TIM1->PSC = 0;          // Sin prescaler (170 MHz)
TIM1->ARR = 8499;       // Auto-reload: 170MHz / 8500 = 20kHz
TIM1->CCR1 = 0-8499;    // PWM Motor FL (PA8)
TIM1->CCR2 = 0-8499;    // PWM Motor FR (PA9)
TIM1->CCR3 = 0-8499;    // PWM Motor RL (PA10)
TIM1->CCR4 = 0-8499;    // PWM Motor RR (PA11)

// PWM Mode 1 (active high)
TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
TIM1->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
```

**TIM8**: Motor de dirección (1 canal)
```c
// Configuración TIM8
TIM8->PSC = 0;          // Sin prescaler (170 MHz)
TIM8->ARR = 8499;       // Auto-reload: 20kHz
TIM8->CCR3 = 0-8499;    // PWM Motor Dirección (PC8)
```

**Cálculo Duty Cycle:**
```c
// Conversión porcentaje a PWM
uint16_t pwm_value = (uint16_t)((demand_pct / 100.0f) * 8499.0f);

// Aplicar a timer
TIM1->CCR1 = pwm_value;  // Motor FL
```

## Geometría Ackermann

### Fundamento Teórico

En un vehículo con dirección frontal, las ruedas delanteras deben girar en **ángulos diferentes** para seguir trayectorias concéntricas sin deslizamiento lateral.

```
        Exterior (radio mayor)
             ↑
             │   ┌─────────┐
             │   │         │
         R_out  │  Centro │
             │   │  Giro   │
             ↓   └─────────┘
                      ↑
                      │ R_inner
                      │
                      ↓
        Interior (radio menor)
```

**Fórmula Ackermann básica:**
```
tan(δ_inner) = L / R
tan(δ_outer) = L / (R + T)

donde:
  δ_inner = ángulo rueda interior
  δ_outer = ángulo rueda exterior  
  L = distancia entre ejes (wheelbase) = 0.95m
  T = ancho de vía (track) = 0.70m
  R = radio de giro al centro del eje trasero
```

### Implementación Algoritmo (Portado desde ESP32)

**Archivo original**: `ESP32/src/control/steering_model.cpp`

```c
// Constantes geometría
#define WHEELBASE_M     0.95f   // Distancia entre ejes
#define TRACK_M         0.70f   // Ancho de vía
#define MAX_INNER_ANGLE 54.0f   // Ángulo máximo rueda interior

/**
 * Calcula ángulos Ackermann para ruedas delanteras
 * @param wheelAngleDeg: Ángulo del volante (-54° a +54°)
 * @return: Estructura con ángulos inner/outer
 */
AckermannResult_t Ackermann_Compute(float wheelAngleDeg) {
    AckermannResult_t result = {0};
    
    // Validación entrada
    if (!isfinite(wheelAngleDeg)) {
        result.innerDeg = 0.0f;
        result.outerDeg = 0.0f;
        return result;
    }
    
    // Clamp a rango seguro
    wheelAngleDeg = fminf(fmaxf(wheelAngleDeg, -MAX_INNER_ANGLE), 
                          MAX_INNER_ANGLE);
    
    // Calcular radio de giro en el eje trasero
    float wheelAngleRad = wheelAngleDeg * (M_PI / 180.0f);
    float tanAngle = tanf(fabsf(wheelAngleRad));
    
    // Protección división por cero
    if (tanAngle < 0.001f) {
        // Recta: ambas ruedas al mismo ángulo
        result.innerDeg = wheelAngleDeg;
        result.outerDeg = wheelAngleDeg;
        return result;
    }
    
    float R = WHEELBASE_M / tanAngle;  // Radio al centro eje trasero
    
    // Ángulo rueda interior (mayor ángulo)
    float innerAngleRad = atanf(WHEELBASE_M / R);
    result.innerDeg = innerAngleRad * (180.0f / M_PI);
    
    // Ángulo rueda exterior (menor ángulo)
    float outerAngleRad = atanf(WHEELBASE_M / (R + TRACK_M));
    result.outerDeg = outerAngleRad * (180.0f / M_PI);
    
    // Aplicar signo según dirección giro
    if (wheelAngleDeg < 0.0f) {
        result.innerDeg = -result.innerDeg;
        result.outerDeg = -result.outerDeg;
    }
    
    return result;
}
```

## Diferencial Virtual (Virtual Differential)

### Concepto

En curvas, la rueda **exterior** recorre mayor distancia que la **interior**, por lo que debe girar más rápido. El diferencial virtual simula esto mediante software.

```
Curva a la derecha:
┌────────────────────────────────┐
│  FL (exterior) → RÁPIDO 100%   │
│  FR (interior) → LENTO  70%    │
│  RL (exterior) → RÁPIDO 100%   │
│  RR (interior) → LENTO  70%    │
└────────────────────────────────┘
```

### Algoritmo de Escalado (Portado desde ESP32)

**Archivo original**: `ESP32/src/control/traction.cpp`

```c
/**
 * Calcula factor de reducción para rueda interior en curva
 * Curva progresiva:
 *   30° → 85% (reducción 15%)
 *   45° → 77.5% (reducción 22.5%)  
 *   60° → 70% (reducción 30%)
 */
float Calculate_InnerWheelFactor(float steeringAngle) {
    float absAngle = fabsf(steeringAngle);
    
    // Validación
    if (!isfinite(absAngle)) {
        return 1.0f;  // Sin reducción si inválido
    }
    
    // Normalizar ángulo (0-60° → 0-1)
    float angleNormalized = absAngle / 60.0f;
    angleNormalized = fminf(angleNormalized, 1.0f);
    
    // Curva exponencial: x^1.2 (más suave que lineal)
    float x_pow_1_2 = powf(angleNormalized, 1.2f);
    
    // Escala: 1.0 → 0.7 (reducción máxima 30%)
    float scale = 1.0f - (x_pow_1_2 * 0.3f);
    
    // Clamp a rango seguro [0.70, 1.0]
    scale = fminf(fmaxf(scale, 0.70f), 1.0f);
    
    return scale;
}
```

**Gráfica de Reducción:**
```
Factor
1.00 │●
     │  ●
0.90 │    ●
     │      ●
0.85 │        ●     (30°)
     │          ●
0.80 │            ●
     │              ●
0.77 │                ●  (45°)
     │                  ●
0.70 │                    ●●●●● (60°)
     └──────────────────────────────
      0°   15°  30°  45°  60°
```

### Aplicación a 4 Ruedas

```c
void Traction_Update(float steeringAngle, float demandPct) {
    // Factores iniciales (sin curva)
    float factorFL = 1.0f;
    float factorFR = 1.0f;
    float factorRL = 1.0f;
    float factorRR = 1.0f;
    
    // Calcular factor de reducción
    float innerFactor = Calculate_InnerWheelFactor(steeringAngle);
    
    // Aplicar reducción a rueda interior según dirección
    if (steeringAngle > 0.0f) {
        // Giro a derecha: reducir rueda DERECHA (interior)
        factorFR = innerFactor;
        factorRR = innerFactor;
    } else if (steeringAngle < 0.0f) {
        // Giro a izquierda: reducir rueda IZQUIERDA (interior)
        factorFL = innerFactor;
        factorRL = innerFactor;
    }
    
    // Modo 4x4 vs 4x2
    float frontDemand, rearDemand;
    if (tractionState.mode4x4) {
        // 4x4: 50% delante, 50% atrás
        frontDemand = demandPct * 0.5f;
        rearDemand = demandPct * 0.5f;
    } else {
        // 4x2: 100% delante, 0% atrás
        frontDemand = demandPct;
        rearDemand = 0.0f;
    }
    
    // Aplicar demanda con factores
    tractionState.wheels[MOTOR_FL].demandPct = frontDemand * factorFL;
    tractionState.wheels[MOTOR_FR].demandPct = frontDemand * factorFR;
    tractionState.wheels[MOTOR_RL].demandPct = rearDemand * factorRL;
    tractionState.wheels[MOTOR_RR].demandPct = rearDemand * factorRR;
    
    // Convertir a PWM y aplicar
    for (int i = 0; i < 4; i++) {
        float demand = tractionState.wheels[i].demandPct;
        
        // Validación seguridad
        if (!isfinite(demand)) {
            demand = 0.0f;
        }
        
        // Clamp a rango [0, 100]
        demand = fminf(fmaxf(demand, 0.0f), 100.0f);
        
        // Convertir a PWM (0-8499)
        uint16_t pwm = (uint16_t)((demand / 100.0f) * 8499.0f);
        
        // Aplicar a hardware
        Motor_SetPWM(i, pwm, false);
    }
}
```

## Modo Rotación sobre Eje (Tank Turn)

### Descripción

Permite girar el vehículo sobre su eje central, útil para maniobras en espacios reducidos.

```
Vista superior (giro derecha):
     FL ↑        FR ↓
      ●            ●
      │     ✕      │
      │   (eje)    │
      ●            ●
     RL ↑        RR ↓
```

### Implementación

```c
void Traction_SetAxisRotation(bool enable, float rotationDemand) {
    if (!enable) {
        return;  // Modo normal
    }
    
    // Dirección: +100 = giro derecha, -100 = giro izquierda
    if (rotationDemand > 0.0f) {
        // Giro a derecha
        tractionState.wheels[MOTOR_FL].demandPct = rotationDemand;   // →
        tractionState.wheels[MOTOR_RL].demandPct = rotationDemand;   // →
        tractionState.wheels[MOTOR_FR].demandPct = -rotationDemand;  // ←
        tractionState.wheels[MOTOR_RR].demandPct = -rotationDemand;  // ←
    } else {
        // Giro a izquierda
        float absRotation = fabsf(rotationDemand);
        tractionState.wheels[MOTOR_FL].demandPct = -absRotation;  // ←
        tractionState.wheels[MOTOR_RL].demandPct = -absRotation;  // ←
        tractionState.wheels[MOTOR_FR].demandPct = absRotation;   // →
        tractionState.wheels[MOTOR_RR].demandPct = absRotation;   // →
    }
    
    tractionState.axisRotation = true;
}
```

## Validaciones de Seguridad

### Checks Obligatorios (desde auditorías ESP32)

```c
// 1. Validación NaN/Inf
bool Safe_IsValid(float value) {
    return isfinite(value);
}

// 2. Validación antes de PWM
void Safe_SetPWM(uint8_t motor, float demandPct) {
    // Check NaN/Inf
    if (!Safe_IsValid(demandPct)) {
        demandPct = 0.0f;
        // LOG ERROR
    }
    
    // Clamp rango
    demandPct = fminf(fmaxf(demandPct, 0.0f), 100.0f);
    
    // Check temperatura
    if (tractionState.wheels[motor].tempC > 130.0f) {
        demandPct = 0.0f;  // EMERGENCY STOP
        // LOG CRITICAL
    }
    
    // Check corriente
    if (tractionState.wheels[motor].currentA > 25.0f) {
        demandPct *= 0.5f;  // Reduce al 50%
        // LOG WARNING
    }
    
    // Aplicar PWM
    uint16_t pwm = (uint16_t)((demandPct / 100.0f) * 8499.0f);
    Motor_SetPWM_Raw(motor, pwm);
}

// 3. División por cero protegida
float Safe_Divide(float numerator, float denominator) {
    if (fabsf(denominator) < 0.001f) {
        return 0.0f;  // Evitar división por cero
    }
    return numerator / denominator;
}

// 4. Emergency stop global
void Traction_EmergencyStop(void) {
    // Desactivar todos los motores inmediatamente
    for (int i = 0; i < 4; i++) {
        Motor_SetPWM_Raw(i, 0);
        GPIO_WritePin(EN_PINS[i], GPIO_PIN_RESET);
    }
    
    // Resetear estado
    memset(&tractionState, 0, sizeof(tractionState));
    
    // LOG CRITICAL EVENT
}
```

## Diagramas de Bloques

### Arquitectura General

```
┌────────────────────────────────────────────────────────┐
│                    MAIN CONTROL LOOP                    │
│                         (100 Hz)                         │
└────────────────┬───────────────────────────────────────┘
                 │
     ┌───────────┴────────────┐
     │                        │
     ▼                        ▼
┌─────────────┐      ┌──────────────────┐
│  CAN Handler│      │  Sensor Manager  │
│  (ESP32)    │      │  (I2C/OneWire)   │
└──────┬──────┘      └────────┬─────────┘
       │                      │
       │ Commands             │ Telemetry
       ▼                      ▼
┌──────────────────────────────────────┐
│         TRACTION CONTROL             │
│  - Ackermann geometry                │
│  - Virtual differential              │
│  - 4x4/4x2 modes                     │
└───────────┬──────────────────────────┘
            │
            ▼
┌───────────────────────────────────────┐
│         SAFETY VALIDATIONS            │
│  - Temperature limits                 │
│  - Current limits                     │
│  - NaN/Inf checks                     │
└────────────┬──────────────────────────┘
             │
             ▼
┌─────────────────────────────────────┐
│         PWM OUTPUT (TIM1/TIM8)      │
│  FL, FR, RL, RR, STEER              │
└──────────────┬──────────────────────┘
               │
               ▼
        ┌──────────────┐
        │  BTS7960 ×5  │
        └──────┬───────┘
               │
               ▼
          [ MOTORES ]
```

### Flujo Actualización Motor

```
┌─────────────────┐
│  CAN Command    │
│  Throttle: 75%  │
│  Steering: -20° │
└────────┬────────┘
         │
         ▼
┌──────────────────────────┐
│  Ackermann_Compute()     │
│  Inner: -22°             │
│  Outer: -18°             │
└──────────┬───────────────┘
           │
           ▼
┌────────────────────────────┐
│  Virtual Differential      │
│  FL: 75% × 0.85 = 63.75%   │
│  FR: 75% × 1.00 = 75%      │
│  RL: 75% × 0.85 = 63.75%   │
│  RR: 75% × 1.00 = 75%      │
└──────────┬─────────────────┘
           │
           ▼
┌────────────────────────────┐
│  Safety Validation         │
│  - isfinite() ✓            │
│  - Temp < 130°C ✓          │
│  - Current < 25A ✓         │
└──────────┬─────────────────┘
           │
           ▼
┌────────────────────────────┐
│  PWM Conversion            │
│  FL: 5418 (63.75% × 8499)  │
│  FR: 6374 (75% × 8499)     │
│  RL: 5418                  │
│  RR: 6374                  │
└──────────┬─────────────────┘
           │
           ▼
┌────────────────────────────┐
│  TIM1->CCRx = PWM          │
│  Hardware Output @ 20kHz   │
└────────────────────────────┘
```

## Ventajas del Sistema Implementado

### Respecto a PCA9685
1. **Latencia reducida**: 2-3ms → <100μs (mejora 20-30×)
2. **Frecuencia superior**: 50Hz → 20kHz (400× más rápido)
3. **Resolución mejorada**: 12-bit → 13-bit efectivo
4. **Sincronización perfecta**: Actualización simultánea 4 canales
5. **Sin dependencias I2C**: Hardware dedicado más robusto

### Respecto a Control Básico
1. **Ackermann correcto**: Reduce desgaste neumáticos en curvas
2. **Diferencial virtual**: Elimina deslizamiento lateral
3. **Modos flexibles**: 4x4/4x2 según necesidad
4. **Tank turn**: Maniobras en espacios reducidos
5. **Seguridad multinivel**: Validaciones exhaustivas

## Referencias

- **ESP32 Firmware Original**: `florinzgz/FULL-FIRMWARE-Coche-Marcos`
- **Ackermann Algorithm**: `src/control/steering_model.cpp`
- **Traction Control**: `src/control/traction.cpp`
- **Security Audits**: `AUDITORIA_SENSORES_CONTROL_v2.17.0.md`

---

**Documento creado**: 2026-02-01  
**Versión**: 1.0  
**Autor**: Sistema de Control Coche Marcos
