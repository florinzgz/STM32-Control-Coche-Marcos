# Análisis de Viabilidad: Dirección Asistida Eléctrica (EPS) con Hardware Actual

> **Alcance**: Análisis técnico exclusivamente — sin modificación de código.
> **Objetivo**: Evaluar si el hardware actual permite reemplazar el control
> de posición PID por un modelo de asistencia al par tipo EPS.
>
> **Archivos fuente analizados**: `Core/Src/motor_control.c`,
> `Core/Inc/motor_control.h`, `Core/Src/main.c`, `Core/Inc/main.h`,
> `Core/Inc/vehicle_physics.h`, `Core/Src/steering_centering.c`,
> `Core/Src/can_handler.c`, `Core/Src/stm32g4xx_hal_msp.c`.
>
> **Documentos relacionados**: `docs/TIMER_PWM_ANALYSIS.md`,
> `docs/BTS7960_MOTOR_DRIVER_AUDIT.md`, `docs/STEERING_CALIBRATION.md`.

---

## Contexto: Comportamiento Actual vs. EPS

### Control actual: Servo de posición

```
ESP32 (CAN) ──► Steering_SetAngle(angle_deg)
                      │
                      ▼
              steering_pid.setpoint = angle × 4800/360
                      │
                      ▼
              Steering_ControlLoop() [100 Hz]
                      │
              ┌───────┴───────┐
              │  PID P-only   │
              │  kp = 0.09    │
              │  ki = kd = 0  │
              └───────┬───────┘
                      │
              ┌───────┴───────┐
              │ error < 0.5°? │──SÍ──► BTS7960_BRAKE_PWM (100%) = freno EM
              │  (deadband)   │
              └───────┬───────┘
                      │NO
                      ▼
              Motor_SetPWM(motor_steer, pwm)
              Motor_SetDirection(motor_steer, dir)
              Motor_Enable(motor_steer, |output| > 1%)
```

**Problemas inherentes del servo de posición**:

1. **Persecución microscópica**: El PID intenta corregir errores de
   fracciones de grado continuamente. Con kp=0.09 y error=1 count
   (0.075°), genera output=0.09 % → pwm ≈ 4 counts. Insuficiente para
   mover el motor (zona muerta del BTS7960), pero suficiente para
   inyectar corriente y producir vibración.

2. **Freno electromagnético permanente**: Dentro del deadband (0.5° =
   ~6.67 counts), el motor recibe PWM=100% (BTS7960_BRAKE_PWM=4249).
   Los terminales del motor se cortocircuitan a través del H-bridge,
   produciendo un freno activo constante. El conductor siente el volante
   "rígido" cuando está cerca del setpoint.

3. **El conductor NO manda**: El ESP32 envía un ángulo objetivo y el
   motor persigue esa posición. Si el conductor intenta mover el volante
   manualmente, el motor se opone (corrige el "error"). Esto es un servo,
   no una dirección asistida.

4. **Oscilación en el deadband**: La transición entre "PID activo" y
   "freno electromagnético" es binaria. Si el error oscila alrededor de
   0.5°, el sistema alterna entre freno total y PID, produciendo vibraciones
   perceptibles.

### Modelo EPS propuesto

```
Encoder TIM2 ──► posición (counts) ──► derivada ──► velocidad angular
                      │                                    │
                      │                              ┌─────┴──────┐
                      │                              │ |ω| > umbral│
                      │                              │ (girando)   │
                      │                              └─────┬──────┘
                      │                                SÍ  │  NO
                      │                                    │
                      ▼                              ┌─────┴──────┐
              torque_center = Kc × θ           torque_assist = Ka × ω
              (solo cuando |ω| < umbral)       (ayuda proporcional
               → autocentrado virtual)          al giro del conductor)
                      │                              │
                      └──────────┬───────────────────┘
                                 ▼
                          torque_total = torque_assist + torque_center
                                 │
                                 ▼
                          PWM = |torque_total| × PWM_PERIOD / 100
                          DIR = sign(torque_total)
                          EN  = |torque_total| > umbral_mínimo
```

**Diferencia fundamental**: No hay setpoint de posición. No hay PID.
El motor aplica par proporcional a lo que el conductor hace (velocidad
angular) o un muelle virtual suave cuando el conductor suelta el volante.

---

## A) ¿La Arquitectura Actual lo Permite?

### **SÍ. El hardware es completamente compatible con el modelo EPS.**

Evaluación componente por componente:

### A.1 Encoder de dirección (E6B2-CWZ6C en TIM2)

| Parámetro | Valor | Adecuación EPS |
|-----------|-------|----------------|
| Resolución | 1200 PPR × 4 = 4800 CPR | ✅ Excelente — 0.075°/count |
| Lectura | TIM2 cuadratura hardware | ✅ Sin overhead CPU, lectura instantánea |
| Rango | 32 bits (TIM2 ARR=0xFFFFFFFF) | ✅ Sin overflow posible |
| Filtro de glitch | IC1/IC2Filter = 6 (~210 ns) | ✅ Rechaza ruido eléctrico |
| Interfaz | `__HAL_TIM_GET_COUNTER(&htim2)` | ✅ Lectura de registro directo, < 10 ns |

**Velocidad angular derivada del encoder**:

A 100 Hz (período 10 ms), la resolución mínima de velocidad es:
```
Δθ_min = 1 count = 0.075°
ω_min = 0.075° / 0.01 s = 7.5 °/s
```

Esto es suficiente para detectar giros del conductor. Un giro lento del
volante (~30°/s) produce ~4 counts/ciclo, claramente distinguible de ruido.

**Riesgo**: A velocidades angulares muy bajas (< 7.5°/s), la cuantización
del encoder produce una señal de velocidad ruidosa (0 o 1 count por ciclo).
Un filtro paso bajo en la velocidad estimada es necesario (ver Sección E.1).

### A.2 Motor de dirección + BTS7960/IBT-2

| Parámetro | Valor | Adecuación EPS |
|-----------|-------|----------------|
| Timer PWM | TIM8 CH3, center-aligned, 20 kHz | ✅ Frecuencia inaudible, resolución 4250 pasos |
| Señales | DIR (PC4) + PWM (PC8) + EN (PC9) | ✅ Control total de par, dirección y habilitación |
| Corriente máx. | 50 A (shunt 1 mΩ, INA226 ch 5) | ✅ Medición disponible para protección |
| Coast mode | EN=LOW → motor flotante | ✅ Necesario para "sin asistencia" |
| Freno activo | PWM=100% → cortocircuito EM | ✅ Disponible para freno de estacionamiento |

**Capacidad de control de par**: El BTS7960 en modo DIR + single PWM
aplica un voltaje medio proporcional al duty cycle. En un motor DC brushed,
el par es proporcional a la corriente:
```
τ = Kt × I = Kt × (V_pwm - V_bemf) / R_winding
```

A bajas velocidades (velocidad de volante << velocidad máxima del motor),
la back-EMF es despreciable:
```
τ ≈ Kt × V_pwm / R_winding = Kt × (duty × V_bus) / R_winding
```

**El duty cycle controla directamente el par** (control voltage-mode que
se aproxima a control de par a baja velocidad). Esto es exactamente lo que
un EPS necesita.

### A.3 Ciclo de control (main.c)

| Parámetro | Valor | Adecuación EPS |
|-----------|-------|----------------|
| Período de control | 10 ms (100 Hz) | ✅ Suficiente para EPS (ver Sección D) |
| Ubicación | `Steering_ControlLoop()` en tarea 10 ms | ✅ Mismo punto de integración |
| Pedal actualización | 50 ms (20 Hz) independiente | ✅ No interfiere |
| Seguridad | Checks en mismo ciclo 10 ms | ✅ Coexistencia preservada |

### A.4 Comunicación CAN

| Aspecto | Estado actual | Adaptación EPS |
|---------|---------------|----------------|
| Comando | `CAN_ID_CMD_STEERING` → `Steering_SetAngle()` | ✅ Podría enviar `Kassist`/`Kcenter` o mantenerse como referencia de ángulo para el autocentrado |
| Telemetría | Ángulo actual ya reportado | ✅ Podría añadir velocidad angular como dato diagnóstico |
| Heartbeat | Incluye estado de calibración | ✅ Sin cambios |

### A.5 Conclusión: Compatibilidad Hardware

**Todos los elementos hardware necesarios están presentes y son
funcionales**. No se requiere nuevo hardware. La transformación es
100 % firmware.

| Recurso | Necesario para EPS | ¿Disponible? |
|---------|-------------------|--------------|
| Sensor de posición angular | Sí (para θ y ω) | ✅ TIM2 encoder, 4800 CPR |
| Actuador de par | Sí (aplicar asistencia) | ✅ BTS7960 + TIM8 PWM |
| Control de dirección de par | Sí (asistir en dirección correcta) | ✅ DIR pin (PC4) |
| Capacidad de "soltar" el motor | Sí (sin asistencia = motor libre) | ✅ EN=LOW → coast |
| Freno para estacionamiento | Sí (mantener posición parado) | ✅ PWM=100% → freno EM |
| Medición de corriente | Recomendable (protección) | ✅ INA226 ch 5 a 20 Hz |
| Sensor de velocidad del vehículo | Útil (adaptar asistencia) | ✅ 4× wheel speed EXTI |

---

## B) Partes del Firmware Actual que Sobran o Estorban

### B.1 Componentes que SOBRAN (incompatibles con EPS)

#### 1. PID de posición (`steering_pid`, `PID_Compute()`)

**Código actual**:
```c
static PID_t steering_pid = {0.09f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
```

**Problema**: El PID persigue un setpoint de posición. En el modelo EPS
no hay setpoint de posición — el motor responde a la velocidad angular
del conductor. El PID completo (struct, compute, integral, derivative)
es **incompatible** con el modelo EPS.

**Lo que estorba específicamente**:
- `steering_pid.setpoint`: No existe concepto de "posición objetivo" en EPS.
- `PID_Compute()`: Calcula error = setpoint - medido. En EPS no hay error
  de posición que corregir.
- `steering_pid.integral`: Acumula error de posición. Sin sentido en EPS.
- `steering_pid.prev_error` / derivative: Derivada del error de posición.
  En EPS se usa derivada de la posición directamente (velocidad angular),
  no derivada del error.

#### 2. `Steering_SetAngle()` como comando de posición

**Código actual**:
```c
void Steering_SetAngle(float angle_deg)
{
    steering_pid.setpoint = angle_deg * (float)ENCODER_CPR / 360.0f;
}
```

**Problema**: Esta función establece la posición objetivo del servo.
En EPS, el ESP32 no debería comandar "ve a 30°". Debería comandar
"asiste al conductor con estos parámetros (Kassist, Kcenter)" o
simplemente no comandar nada (el MCU lee la intención del conductor
localmente).

#### 3. Freno electromagnético en deadband

**Código actual**:
```c
if (absError < STEERING_DEADBAND_COUNTS) {
    Motor_SetPWM(&motor_steer, BTS7960_BRAKE_PWM);  /* 100% duty */
    Motor_Enable(&motor_steer, 1);
    return;
}
```

**Problema**: Cuando el volante está "cerca" del setpoint, el motor
aplica freno total. Esto endurece el volante artificialmente. En EPS,
cuando el conductor no gira (ω ≈ 0), el motor debe estar en coast
(EN=LOW) o aplicar un muelle virtual suave, **no un freno total**.

#### 4. `Steering_Neutralize()` como freno activo

**Código actual**:
```c
void Steering_Neutralize(void)
{
    Motor_SetPWM(&motor_steer, BTS7960_BRAKE_PWM);
    Motor_Enable(&motor_steer, 1);
}
```

**Problema**: "Neutralizar" = freno total. En EPS, "neutralizar" debería
significar "motor en coast" (EN=LOW), no "motor frenado activamente".
El freno activo solo debería usarse en estacionamiento o fallo.

### B.2 Componentes que se PRESERVAN

| Componente | Razón |
|-----------|-------|
| `Motor_Init()` | Inicializa TIM1/TIM8/TIM2, PWM start, encoder start — 100 % válido |
| `Motor_SetPWM()`, `Motor_SetDirection()`, `Motor_Enable()` | Primitivas de bajo nivel — necesarias tal cual |
| `motor_steer` (Motor_t struct) | Configuración de timer/channel/pins — sin cambios |
| `Steering_Init()` | Inicializa encoder, calibración — necesario con adaptación menor |
| `Encoder_CheckHealth()` | Monitorización de fallo de encoder — crítico de seguridad, sin cambios |
| `Steering_GetCurrentAngle()` | Lectura de posición — necesaria para el muelle virtual de autocentrado |
| `Steering_IsCalibrated()` / `Steering_SetCalibrated()` | Gestión de calibración — sin cambios |
| Steering centering module | Auto-búsqueda de centro físico — independiente del modo de control |
| `sanitize_float()` | Validación NaN/Inf — seguridad, sin cambios |
| Safety degraded scaling | `Safety_GetSteeringLimitFactor()` — aplicable al par de asistencia |
| BTS7960_BRAKE_PWM | Constante útil para freno de estacionamiento (GEAR_PARK) |

### B.3 Componentes que requieren MODIFICACIÓN

| Componente | Modificación necesaria |
|-----------|----------------------|
| `Steering_ControlLoop()` | Reescritura completa: de PID de posición a cálculo de par EPS |
| `Steering_SetAngle()` | Cambiar semántica: de "setpoint de posición" a "parámetros de asistencia" o eliminar |
| `Steering_Neutralize()` | Cambiar de freno activo a coast (EN=LOW, PWM=0) |
| CAN handler `CAN_ID_CMD_STEERING` | Adaptar payload: de ángulo objetivo a parámetros de asistencia |
| Encoder health: frozen check | Ajustar: en EPS el motor puede estar parado legítimamente cuando el conductor no gira |

---

## C) Integración en el Bucle Principal

### C.1 Punto de integración: mismo que el actual

```c
/* main.c — tarea 10 ms (100 Hz) */
if ((now - tick_10ms) >= 10) {
    tick_10ms = now;

    /* Safety checks (sin cambios) */
    ABS_Update();
    TCS_Update();
    Safety_CheckCurrent();
    Safety_CheckTemperature();
    Safety_CheckCANTimeout();
    CAN_CheckBusOff();
    Safety_CheckSensors();
    Safety_CheckEncoder();

    /* Steering centering (sin cambios) */
    if (!SteeringCentering_IsComplete() && ...) {
        SteeringCentering_Step();
    }

    /* ─── AQUÍ: Steering_ControlLoop() se reemplaza ─── */
    Steering_ControlLoop();   /* Actual: PID posición → Nuevo: EPS par */

    Traction_Update();
}
```

**No se necesita cambiar la estructura del bucle**. La función
`Steering_ControlLoop()` se ejecuta en el mismo slot de 10 ms.
Internamente, su lógica cambia de PID de posición a cálculo de par EPS.

### C.2 Flujo de datos EPS dentro de Steering_ControlLoop()

```
Steering_ControlLoop() [llamada cada 10 ms]
│
├─ 1. Leer encoder: position = __HAL_TIM_GET_COUNTER(&htim2)
│
├─ 2. Calcular velocidad angular:
│     velocity = (position - prev_position) / dt
│     prev_position = position
│     velocity_filtered = EMA(velocity, alpha)
│
├─ 3. Calcular ángulo (para autocentrado):
│     angle_deg = position × 360 / ENCODER_CPR
│
├─ 4. Determinar modo:
│     if |velocity_filtered| > ASSIST_THRESHOLD → modo ASISTENCIA
│     else                                      → modo AUTOCENTRADO
│
├─ 5. Calcular par:
│     if modo == ASISTENCIA:
│         torque = Kassist × velocity_filtered
│     else:
│         torque = Kcenter × angle_deg
│         (muelle virtual hacia centro)
│
├─ 6. Aplicar límites:
│     torque = clamp(torque, -MAX_TORQUE, +MAX_TORQUE)
│     if Safety_IsDegraded():
│         torque *= Safety_GetSteeringLimitFactor()
│
├─ 7. Convertir a PWM:
│     pwm = |torque| × PWM_PERIOD / 100
│     dir = sign(torque)
│     en  = |torque| > MIN_TORQUE_THRESHOLD
│
└─ 8. Aplicar a motor:
      Motor_SetPWM(&motor_steer, pwm)
      Motor_SetDirection(&motor_steer, dir)
      Motor_Enable(&motor_steer, en)
```

### C.3 Variables de estado necesarias

```c
/* Reemplazan a steering_pid */
static float    eps_prev_position  = 0.0f;  /* Posición anterior (counts) */
static float    eps_velocity       = 0.0f;  /* Velocidad angular (°/s) */
static float    eps_velocity_filt  = 0.0f;  /* Velocidad filtrada (°/s) */
static uint32_t eps_last_tick      = 0;     /* Timestamp anterior */

/* Parámetros de asistencia (configurables vía CAN o constantes) */
static float    eps_Kassist        = 1.5f;  /* Ganancia de asistencia */
static float    eps_Kcenter        = 0.3f;  /* Ganancia de autocentrado */
static float    eps_assist_thresh  = 15.0f; /* Umbral velocidad (°/s) */
```

### C.4 Interacción con otros módulos

| Módulo | Interacción actual | Interacción EPS |
|--------|-------------------|-----------------|
| **Steering centering** | Busca centro, luego PID | Busca centro, luego EPS — sin cambios |
| **CAN handler** | Envía ángulo objetivo | Envía parámetros Kassist/Kcenter o nada |
| **Safety system** | Limita output PID | Limita output par EPS — mismo mecanismo |
| **Encoder health** | Detecta fallo → neutraliza | Detecta fallo → coast (no freno) |
| **Traction** | Independiente | Independiente — sin cambios |
| **Ackermann** | Recibe ángulo del PID setpoint | No necesario si no hay setpoint de ángulo |

### C.5 Ackermann: ¿Se elimina?

En el modelo actual, `Steering_SetAngle()` llama a
`Ackermann_ComputeWheelAngles()` para calcular los ángulos FL/FR.
Estos ángulos se usan para telemetría CAN y para la corrección
diferencial de par en tracción.

En el modelo EPS:
- El ángulo del volante es la posición real del encoder (no un setpoint).
- `Steering_GetCurrentAngle()` ya devuelve el ángulo real.
- Los ángulos Ackermann pueden calcularse desde el ángulo real en lugar
  del setpoint, manteniendo la funcionalidad de telemetría y corrección
  diferencial.

**Ackermann no se elimina** — se reubica para usar el ángulo medido.

---

## D) Frecuencia Mínima de Control

### D.1 Análisis de requisitos de frecuencia

La frecuencia de control del EPS está limitada por tres factores:

#### Factor 1: Resolución de velocidad angular del encoder

```
A 100 Hz (10 ms): ω_min = 0.075° / 0.01s = 7.5 °/s
A 200 Hz (5 ms):  ω_min = 0.075° / 0.005s = 15 °/s
A 500 Hz (2 ms):  ω_min = 0.075° / 0.002s = 37.5 °/s
A 50 Hz  (20 ms): ω_min = 0.075° / 0.02s = 3.75 °/s
```

**A mayor frecuencia, peor resolución de velocidad** (menos counts por
período → más cuantización). A menor frecuencia, mejor resolución pero
peor respuesta dinámica.

**100 Hz es el punto óptimo**: 7.5°/s de resolución mínima, con un filtro
EMA para suavizar la cuantización. Un giro normal del volante (30–200°/s)
produce 4–27 counts por ciclo, ampliamente medible.

#### Factor 2: Respuesta al giro del conductor

El conductor gira el volante con una dinámica de ~1–5 Hz (tiempo de
reacción humano). Para seguir esa dinámica sin desfase perceptible:
```
f_control ≥ 10 × f_conductor_max = 10 × 5 = 50 Hz (mínimo absoluto)
f_control ≥ 20 × f_conductor_max = 20 × 5 = 100 Hz (recomendado)
```

#### Factor 3: Estabilidad del lazo de autocentrado

El muelle virtual de autocentrado es un lazo de control de primer orden:
```
τ_motor = Kcenter × θ
```

La constante de tiempo del muelle virtual depende de la inercia del
volante y la ganancia Kcenter. Para evitar oscilaciones:
```
f_control > 1 / (2π × τ_sistema)
```

Con τ_sistema típico de ~100 ms para dirección asistida:
```
f_control > 1 / (2π × 0.1) ≈ 1.6 Hz
```

Esto es muy conservador — 100 Hz proporciona un margen de ×60.

### D.2 Conclusión: Frecuencia

| Requisito | Frecuencia mínima | 100 Hz actual |
|-----------|-------------------|---------------|
| Resolución de velocidad angular | ≥ 50 Hz | ✅ 100 Hz |
| Respuesta al conductor | ≥ 50 Hz | ✅ 100 Hz |
| Estabilidad de autocentrado | ≥ 2 Hz | ✅ 100 Hz |
| Margen de seguridad | ×2–5 sobre mínimo | ✅ ×2 sobre mínimo |

**100 Hz (10 ms) es suficiente**. No se necesita aumentar la frecuencia
del bucle. El slot de 10 ms actual en `main.c` es adecuado.

Si en el futuro se quisiera mayor suavidad, 200 Hz (5 ms) mejoraría la
resolución del filtro de velocidad, pero no es necesario para un vehículo
de baja velocidad.

---

## E) Problemas Físicos Previsibles

### E.1 Cuantización de la velocidad angular

**Problema**: A 100 Hz, la velocidad angular se calcula como:
```
ω = Δcounts × (360° / 4800) / 0.01s
```

Cuando el conductor gira lentamente (< 7.5°/s), Δcounts oscila entre
0 y 1, produciendo una velocidad estimada que salta entre 0°/s y 7.5°/s.
El par de asistencia oscilaría entre 0 y `Kassist × 7.5`, produciendo
vibraciones.

**Mitigación**: Filtro EMA (Exponential Moving Average) en la velocidad:
```
ω_filt = α × ω_raw + (1 - α) × ω_filt_prev
```

Con α = 0.3 y 100 Hz, la constante de tiempo es:
```
τ = dt / α = 0.01 / 0.3 = 33 ms    (fc ≈ 4.8 Hz)
```

Esto suaviza las oscilaciones de cuantización sin añadir desfase
perceptible (33 ms << tiempo de reacción humano ~200 ms).

**Alternativa superior**: Calcular velocidad sobre una ventana de N
muestras (e.g. N=4 → 40 ms). Resolución efectiva: 7.5/4 = 1.875°/s.
Trade-off: más latencia pero mucho menos ruido.

### E.2 Fricción estática del motor (stiction)

**Problema**: El motor DC brushed tiene fricción estática en las escobillas
y cojinetes. Para que el motor comience a girar, el voltaje aplicado debe
superar un umbral mínimo (típicamente 1–2 V para motores de 24 V).
En el modelo EPS:

- Si `Kassist × ω` produce un duty cycle < ~6 % (motor dead zone), el
  motor no girará pero consumirá corriente → calor sin asistencia útil.
- Cuando el par de asistencia supera el umbral de stiction, el motor
  arranca bruscamente → micro-tirón perceptible.

**Mitigación**: Compensación de zona muerta en la salida:
```
if |torque_command| > 0 AND |torque_command| < MIN_EFFECTIVE_TORQUE:
    torque_output = MIN_EFFECTIVE_TORQUE × sign(torque_command)
elif |torque_command| == 0:
    torque_output = 0  (coast)
```

Donde `MIN_EFFECTIVE_TORQUE` es el duty mínimo para que el motor
comience a girar (~6–8 % según calibración). Esto elimina la zona
muerta pero puede producir un salto de par al inicio del giro —
aceptable si `MIN_EFFECTIVE_TORQUE` es pequeño.

### E.3 Backlash mecánico (holgura)

**Problema**: El mecanismo de dirección (cremallera, columna, juntas
universales) tiene holgura mecánica. Cuando el conductor invierte la
dirección del giro, hay un rango angular donde el encoder gira pero
las ruedas no se mueven. En ese rango:

- La velocidad angular del encoder es no-nula → el EPS aplica par
  de asistencia.
- Pero las ruedas no se mueven → la asistencia empuja a través de
  la holgura de golpe → tirón.

**Impacto real**: Para un vehículo de estas dimensiones (wheelbase 0.95 m,
track 0.70 m), el backlash es típicamente < 1° en el volante. A 4800 CPR,
1° = 13.3 counts. La velocidad angular durante la inversión de dirección
es momentáneamente alta (el conductor empuja contra la holgura), por lo
que el EPS aplica asistencia correctamente durante la mayor parte del
rango de backlash.

**Mitigación**: El filtro EMA de velocidad (E.1) atenúa naturalmente
los transitorios rápidos del backlash. No se requiere compensación
específica si el backlash es < 2°.

### E.4 Autocentrado vs. fricción de la dirección

**Problema**: El muelle virtual de autocentrado aplica:
```
torque_center = Kcenter × angle_deg
```

Si la dirección tiene fricción significativa (cremallera seca, juntas
apretadas), el par de autocentrado puede ser insuficiente para mover
el mecanismo de vuelta al centro. El volante se queda "atascado" en
un ángulo no-cero.

**Mitigación**: Dos opciones:
1. **Aumentar Kcenter** → riesgo de oscilación si Kcenter es demasiado alto.
2. **Añadir compensación de fricción**: Si |angle| > 2° y |velocity| < threshold:
   ```
   torque_center += Kfriction × sign(angle)
   ```
   Este término constante supera la fricción estática sin ser proporcional
   al ángulo (no amplifica oscilaciones).

### E.5 Calentamiento del motor durante asistencia sostenida

**Problema**: En una curva prolongada, el conductor mantiene el volante
girado. La velocidad angular es ~0 (volante estático). El autocentrado
aplica un par constante:
```
torque_center = Kcenter × angle_deg
```

Si `angle_deg = 30°` y `Kcenter = 0.3`, torque_center = 9 %, que
produce una corriente continua a través del motor. A 24 V, 9 % duty:
```
V_applied = 0.09 × 24 = 2.16 V
I_stall = 2.16 / R_winding ≈ 6 A  (asumiendo R = 0.35 Ω)
P_diss = I² × R = 36 × 0.35 = 12.6 W
```

12.6 W continuos en un motor pequeño pueden alcanzar temperaturas
elevadas en minutos.

**Mitigación**: La monitorización de temperatura existente (DS18B20 a
1 Hz) y corriente (INA226 ch 5 a 20 Hz) ya protegen contra este escenario.
El `Safety_CheckTemperature()` en la tarea de 10 ms detectaría sobrecalentamiento
y entraría en DEGRADED con `Safety_GetSteeringLimitFactor()` reducido.

### E.6 Comportamiento al soltar el volante bruscamente

**Problema**: Si el conductor suelta el volante mientras gira rápidamente,
la velocidad angular pasa de ~200°/s a ~0°/s en pocos milisegundos
(amortiguación mecánica). Durante la desaceleración:

1. `|velocity_filtered| > threshold` → modo ASISTENCIA sigue activo.
2. La velocidad decrece → el par de asistencia decrece → desaceleración
   suave del motor.
3. `|velocity_filtered| < threshold` → modo AUTOCENTRADO activa.
4. El muelle virtual empuja el volante de vuelta al centro.

Esto es el comportamiento deseado — similar a una dirección asistida
real. El filtro EMA previene transiciones bruscas entre modos.

**Riesgo**: Si el filtro EMA es demasiado lento (α muy pequeño), la
transición a autocentrado se retrasará y el volante se quedará en la
posición donde el conductor lo soltó durante decenas de milisegundos
antes de empezar a autocentrar. Con α = 0.3 (τ = 33 ms), el retraso
es imperceptible.

### E.7 Seguridad: Volante activo sin conductor

**Problema**: Si el autocentrado está activo y las ruedas están giradas
(e.g. después de aparcar), el muelle virtual moverá el volante hacia
el centro al arrancar. Sin conductor, el volante gira solo.

**Mitigación**: El autocentrado solo se activa después de la calibración
(`steering_calibrated = 1`). La calibración requiere el módulo de centering
(que mueve el motor activamente). Si el sistema arranca y el volante
está girado, el centering buscará el centro antes de activar el control
EPS. Después del centering, el volante ya está centrado → el muelle
virtual no produce movimiento.

Si el conductor gira y apaga el vehículo con el volante girado, al
re-arrancar el centering re-centrará. Esto es comportamiento esperado
y seguro.

### E.8 Resumen de problemas y mitigaciones

| Problema | Severidad | Mitigación | Complejidad |
|----------|-----------|------------|-------------|
| E.1 Cuantización de velocidad | Media | Filtro EMA (α=0.3) | Muy baja |
| E.2 Stiction del motor | Media | Compensación de zona muerta | Baja |
| E.3 Backlash mecánico | Baja | Filtro EMA (natural) | Ninguna |
| E.4 Fricción vs. autocentrado | Media | Término de fricción constante | Baja |
| E.5 Calentamiento sostenido | Baja | Protección existente (temp/corriente) | Ninguna |
| E.6 Soltar volante | Baja | Comportamiento natural del modelo | Ninguna |
| E.7 Volante sin conductor | Baja | Centering module existente | Ninguna |

---

## Apéndice: Comparación de Constantes

### Constantes del PID actual (para referencia, a eliminar)

| Constante | Valor | Uso PID |
|-----------|-------|---------|
| `steering_pid.kp` | 0.09 | Ganancia proporcional (error posición) |
| `steering_pid.ki` | 0.0 | No usado |
| `steering_pid.kd` | 0.0 | No usado |
| `STEERING_DEADBAND_COUNTS` | ~6.67 counts (0.5°) | Freno en deadband |

### Constantes EPS propuestas (valores de partida, requieren calibración)

| Constante | Valor sugerido | Unidades | Propósito |
|-----------|---------------|----------|-----------|
| `EPS_KASSIST` | 1.0–3.0 | %/°/s | Ganancia de asistencia (par por velocidad angular) |
| `EPS_KCENTER` | 0.2–0.5 | %/° | Ganancia de autocentrado (par por ángulo) |
| `EPS_ASSIST_THRESHOLD` | 10–20 | °/s | Velocidad angular mínima para activar asistencia |
| `EPS_VELOCITY_EMA_ALPHA` | 0.2–0.4 | — | Coeficiente de filtro de velocidad |
| `EPS_MIN_EFFECTIVE_DUTY` | 250–350 | counts | ~6–8 % → supera zona muerta del motor |
| `EPS_MAX_TORQUE_PCT` | 60–80 | % | Límite de par máximo (protección del motor) |
| `EPS_KFRICTION` | 1.0–3.0 | % | Compensación de fricción estática (constante) |

Todos estos valores requieren calibración empírica en el vehículo real.
Los rangos propuestos están basados en las características conocidas del
hardware (24 V bus, motor DC brushed, zona muerta ~6 %, PWM 4250 pasos).
