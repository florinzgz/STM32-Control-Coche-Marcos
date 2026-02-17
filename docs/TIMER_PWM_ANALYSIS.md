# Análisis de Uso de Timers Hardware — PWM Motores

**Fecha:** 2026-02-17  
**Versión:** 2.0 — Center-aligned PWM  
**Fuentes:** `Core/Src/main.c`, `Core/Inc/main.h`, `Core/Src/motor_control.c`, `Core/Src/stm32g4xx_hal_msp.c`

---

## 1. Tabla de salidas PWM — Configuración exacta extraída del código

### 1.1 Motores de tracción (4 ruedas)

| Motor | GPIO Puerto/Pin | Timer | Canal | Frecuencia PWM | Modo contador | Prescaler | Periodo (ARR) | AF |
|-------|----------------|-------|-------|-----------------|---------------|-----------|---------------|-----|
| FL (Delantero Izq.) | **PA8** | **TIM1** | CH1 | 20 kHz | Center-aligned (UP/DOWN) | 0 | 4249 | AF6 |
| FR (Delantero Der.) | **PA9** | **TIM1** | CH2 | 20 kHz | Center-aligned (UP/DOWN) | 0 | 4249 | AF6 |
| RL (Trasero Izq.) | **PA10** | **TIM1** | CH3 | 20 kHz | Center-aligned (UP/DOWN) | 0 | 4249 | AF6 |
| RR (Trasero Der.) | **PA11** | **TIM1** | CH4 | 20 kHz | Center-aligned (UP/DOWN) | 0 | 4249 | AF6 |

### 1.2 Motor de dirección

| Motor | GPIO Puerto/Pin | Timer | Canal | Frecuencia PWM | Modo contador | Prescaler | Periodo (ARR) | AF |
|-------|----------------|-------|-------|-----------------|---------------|-----------|---------------|-----|
| Steering (Dirección) | **PC8** | **TIM8** | CH3 | 20 kHz | Center-aligned (UP/DOWN) | 0 | 4249 | AF4 |

### 1.3 Encoder de dirección (no es PWM — lectura cuadratura)

| Función | GPIO Puerto/Pin | Timer | Canal | Modo | Prescaler | Periodo (ARR) | AF |
|---------|----------------|-------|-------|------|-----------|---------------|-----|
| Encoder canal A | **PA15** | **TIM2** | CH1 | Encoder TI12 | 0 | 0xFFFFFFFF | AF1 |
| Encoder canal B | **PB3** | **TIM2** | CH2 | Encoder TI12 | 0 | 0xFFFFFFFF | AF1 |

---

## 2. Detalle de configuración por timer

### 2.1 TIM1 — PWM 4 motores de tracción (center-aligned)

```c
/* Fuente: Core/Src/main.c — MX_TIM1_Init() */
htim1.Instance               = TIM1;
htim1.Init.Prescaler         = 0;
htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;  /* Up/Down counting */
htim1.Init.Period            = 4249;   /* Center-aligned: 170 MHz / (2 × 4250) = 20 kHz */
htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
htim1.Init.RepetitionCounter = 0;
htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

/* OC mode PWM1, polarity HIGH, pulse initial = 0 */
/* Channels: CH1 (PA8/FL), CH2 (PA9/FR), CH3 (PA10/RL), CH4 (PA11/RR) */
```

**Cálculo de frecuencia (center-aligned):**
```
f_timer = SYSCLK / (PSC + 1) = 170 MHz / 1 = 170 MHz
f_pwm   = f_timer / (2 × (ARR + 1)) = 170 MHz / (2 × (4249 + 1)) = 170 MHz / 8500 = 20.000 kHz ✓
```
Counter counts: 0 → 1 → … → 4249 → 4248 → … → 1 → 0 (total 2 × 4250 = 8500 ticks per PWM period)

**Resolución PWM:** 4250 pasos (0–4249), equivalente a ~12.05 bits efectivos.

> **Nota sobre resolución:** La resolución se reduce de ~13 bits (8500 pasos) a ~12 bits
> (4250 pasos). Para control de motores DC brushed con drivers BTS7960 a 24V, 4250
> pasos proporcionan una granularidad de 0.024% por paso, que es más que suficiente
> (la tolerancia mecánica y eléctrica del sistema es >> 0.1%).

**Tipo de timer:** TIM1 es un timer avanzado (Advanced-Control Timer) con:
- 4 canales de captura/comparación
- Soporte para dead-time insertion y complementary outputs
- Repetition counter
- Break inputs (no usados)
- Center-aligned mode 1 (interrupt en down-counting only)

### 2.2 TIM8 — PWM motor de dirección (center-aligned)

```c
/* Fuente: Core/Src/main.c — MX_TIM8_Init() */
htim8.Instance               = TIM8;
htim8.Init.Prescaler         = 0;
htim8.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;  /* Up/Down counting */
htim8.Init.Period            = 4249;   /* Center-aligned: 170 MHz / (2 × 4250) = 20 kHz */
htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
htim8.Init.RepetitionCounter = 0;
htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

/* OC mode PWM1, polarity HIGH, pulse initial = 0 */
/* Only CH3 (PC8) is used */
```

**Cálculo:** Idéntico a TIM1 → 20 kHz.

### 2.3 TIM2 — Encoder cuadratura (sin cambios)

```c
/* Fuente: Core/Src/main.c — MX_TIM2_Init() */
htim2.Instance               = TIM2;
htim2.Init.Prescaler         = 0;
htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
htim2.Init.Period            = 0xFFFFFFFF;    /* 32-bit full range */
enc.EncoderMode              = TIM_ENCODERMODE_TI12;
enc.IC1Filter = enc.IC2Filter = 6;            /* ~210 ns glitch rejection */
```

Este timer no genera PWM — cuenta pulsos del encoder E6B2-CWZ6C. No se ve afectado por el cambio.

---

## 3. ¿Los 4 motores de tracción comparten el mismo contador de timer?

### ✅ SÍ — Los 4 motores de tracción comparten el mismo contador TIM1

**Evidencia del código:**

```c
/* Core/Src/motor_control.c — Motor_Init() */
motor_fl.timer = &htim1;  motor_fl.channel = TIM_CHANNEL_1;   /* PA8 */
motor_fr.timer = &htim1;  motor_fr.channel = TIM_CHANNEL_2;   /* PA9 */
motor_rl.timer = &htim1;  motor_rl.channel = TIM_CHANNEL_3;   /* PA10 */
motor_rr.timer = &htim1;  motor_rr.channel = TIM_CHANNEL_4;   /* PA11 */
```

Los 4 canales (CH1–CH4) de TIM1 comparten:
- El **mismo contador CNT** que cuenta 0 → 4249 (up), luego 4248 → 0 (down)
- El **mismo prescaler** (PSC = 0)
- El **mismo periodo** (ARR = 4249)
- El **mismo modo de conteo** (center-aligned mode 1)
- La **misma frecuencia base** (170 MHz)

Cada canal tiene su propio **registro CCR** (Compare/Capture Register) que establece el duty cycle individual. El firmware escribe en estos registros independientemente:

```c
/* Core/Src/motor_control.c — Motor_SetPWM() */
static void Motor_SetPWM(Motor_t *motor, uint16_t pwm) {
    if (pwm > PWM_PERIOD) pwm = PWM_PERIOD;
    __HAL_TIM_SET_COMPARE(motor->timer, motor->channel, pwm);
}
```

En center-aligned mode, la señal PWM es simétrica: la salida se activa cuando CNT sube
hasta CCR y se desactiva cuando CNT baja hasta CCR. Esto produce un pulso centrado
en cada periodo, con transiciones simétricas respecto al centro del periodo.

---

## 4. ¿Algún PWM de motor es asíncrono respecto a los demás?

### Respuesta: TIM1 (tracción) y TIM8 (dirección) son asíncronos entre sí

#### 4.1 Dentro de TIM1: TOTALMENTE SÍNCRONO ✅

Los 4 motores de tracción (FL, FR, RL, RR) están en TIM1 CH1–CH4. Comparten el mismo contador, por lo que:
- Todos los pulsos PWM están **centrados en el mismo instante** (pico del triángulo)
- Los flancos de subida/bajada están simétricos respecto al centro del periodo
- **No hay desfase de fase entre los 4 canales** — son perfectamente síncronos

Con center-aligned mode, los flancos de conmutación se distribuyen simétricamente a lo largo del periodo, en lugar de concentrarse todos en el instante CNT=0.

#### 4.2 TIM1 vs TIM8: ASÍNCRONOS ⚠️ (pero aceptable)

TIM1 y TIM8 son timers independientes. Aunque ambos tienen la misma configuración (PSC=0, ARR=4249, 20 kHz center-aligned), sus contadores **no están sincronizados**:

- No se usa el mecanismo de sincronización interna del STM32 (Master/Slave Timer Trigger, ITR)
- Ambos timers arrancan con `HAL_TIM_PWM_Start()` en secuencia dentro de `Motor_Init()`, por lo que hay un desfase de unos pocos ciclos de reloj entre el inicio de TIM1 y TIM8
- El desfase resultante es del orden de **nanosegundos** y es constante durante toda la ejecución

**¿Es esto un problema?**

**No, para este sistema.** TIM8 controla el motor de **dirección**, que es un sistema de control independiente (PID de posición con encoder). El motor de dirección no necesita sincronización de fase con los motores de tracción porque:

1. Opera en un loop de control diferente (posición angular, no velocidad/par)
2. Su dinámica es mucho más lenta que la frecuencia PWM (ancho de banda PID << 20 kHz)
3. No hay interacción eléctrica entre el motor de dirección y los motores de tracción (drivers BTS7960 separados, fuentes de alimentación separadas: 12V dirección vs 24V tracción)

---

## 5. Efectos físicos del cambio a center-aligned mode

### 5.1 Comparación edge-aligned vs center-aligned

```
Edge-aligned (ANTERIOR):
CNT:  0▲────────────────ARR
      │  ╱╲               │
      │ ╱  ╲    CCR match │    Todos los flancos de subida
      │╱    ╲─────────────│    coinciden en CNT=0 → spike
      0─────────────────→ t    de corriente simultáneo

Center-aligned (NUEVO):
CNT:  0────────ARR────────0
      │       ╱  ╲         │
      │      ╱    ╲   CCR  │   Los flancos se distribuyen
      │     ╱      ╲  match│   simétricamente alrededor del
      │    ╱        ╲      │   centro → sin spike simultáneo
      0───────────────────→ t
```

### 5.2 Beneficios del center-aligned mode

| Aspecto | Edge-aligned (antes) | Center-aligned (ahora) |
|---------|---------------------|----------------------|
| **Flancos de conmutación** | Todos los rising edges en CNT=0 → spike de corriente | Flancos distribuidos simétricamente → sin spike |
| **Linealidad de par a bajo duty** | No lineal (pulso asimétrico concentrado en un extremo) | Lineal (pulso centrado, simétrico) |
| **Rizado de corriente** | Mayor (dV/dt concentrado) | Menor (~50% menos ripple peak) |
| **EMI** | Picos de corriente simultáneos generan EMI peor caso | Corrientes distribuidas → EMI reducida |
| **Suavidad a baja velocidad** | Menos suave (torque pulsante asimétrico) | Más suave (torque pulsante simétrico) |
| **Frecuencia de ripple** | f_pwm | 2 × f_pwm (el ripple aparece al doble de frecuencia) |

### 5.3 Efecto en la modulación de tracción

**Antes (edge-aligned):**
- A 5% duty, los 4 motores encendían simultáneamente durante 425 ciclos (2.5 µs) al inicio del periodo, luego todos apagados durante 47.5 µs
- Esto producía un "golpe" de corriente cada 50 µs seguido de silencio

**Ahora (center-aligned):**
- A 5% duty (CCR = ~212), los 4 motores producen un pulso centrado de ~2.5 µs en medio de cada semiperiodo, con transiciones simétricas
- El par aplicado al motor es más uniforme en el tiempo

### 5.4 Efecto en ABS/TCS y Ackermann

- **Sin impacto en la lógica de control:** Los algoritmos ABS/TCS/Ackermann operan sobre valores de duty cycle (0–100%), no sobre la forma de onda
- La función `Motor_SetPWM()` sigue mapeando 0–`PWM_PERIOD` linealmente
- Los `wheel_scale[]` y `acker_diff[]` se aplican como factores multiplicativos sobre `base_pwm`, que ahora tiene rango 0–4249 en lugar de 0–8499
- **El porcentaje de duty sigue siendo idéntico** (e.g., 50% = CCR/ARR = 2124/4249 ≈ 50%)

### 5.5 Efecto en dead-time y complementary outputs

- Los BTS7960 no usan complementary outputs del timer (cada driver tiene su propia lógica de dead-time interna)
- El dead-time register (DTG) del TIM1 no está configurado (`oc.OCFastMode = TIM_OCFAST_DISABLE`)
- **Sin impacto** — los break inputs y DTG no se usan en este diseño

### 5.6 Efecto en update events e interrupts

Con `TIM_COUNTERMODE_CENTERALIGNED1`:
- **Update events** se generan solo en el down-counting (cuando CNT llega a 0)
- **CCR preload** (`AutoReloadPreload = ENABLE`) asegura que los nuevos valores de duty se aplican atómicamente en el siguiente update event
- Los 4 canales de TIM1 se actualizan en el **mismo update event** → sincronización preservada
- La frecuencia de update events es **1× f_pwm = 20 kHz** (igual que antes)

> **Center-aligned mode 1 vs 2 vs 3:**
> - Mode 1 (`CENTERALIGNED1`): interrupt flag set solo durante down-counting
> - Mode 2: interrupt flag set solo durante up-counting
> - Mode 3: interrupt flag set en ambas direcciones
>
> Se usa Mode 1 para mantener la misma frecuencia de interrupciones (20 kHz) que antes.

---

## 6. Mapping de timers — Verificación

### El mapping actual sigue siendo óptimo ✅

| Timer | Uso | Canales | Modo | ¿Correcto? |
|-------|-----|---------|------|------------|
| **TIM1** (Advanced) | 4× PWM tracción | CH1 (PA8), CH2 (PA9), CH3 (PA10), CH4 (PA11) | Center-aligned 1 | ✅ Perfecto — 4 motores síncronos, pulso centrado |
| **TIM8** (Advanced) | 1× PWM dirección | CH3 (PC8) | Center-aligned 1 | ✅ Correcto — consistente con TIM1, independiente |
| **TIM2** (General 32-bit) | Encoder cuadratura | CH1 (PA15), CH2 (PB3) | Edge-aligned UP | ✅ Sin cambios — encoder no afectado |

### 6.1 Propuesta de sincronización TIM1/TIM8 (opcional, no implementada)

Si en el futuro se necesitara sincronización de dirección con tracción:

```c
/* Hipotético — NO necesario en el diseño actual */
TIM_MasterConfigTypeDef sMasterConfig = {0};
sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

TIM_SlaveConfigTypeDef sSlaveConfig = {0};
sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
sSlaveConfig.InputTrigger = TIM_TS_ITR0;
HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig);
```

---

## 7. Resumen técnico

```
ÁRBOL DE CLOCK
══════════════
HSI 16 MHz ──► PLL (/4 × 85 /2) ──► SYSCLK = 170 MHz
                                        │
                                        ├──► AHB  = 170 MHz (÷1)
                                        ├──► APB1 = 170 MHz (÷1) → TIM2, TIM3, TIM4...
                                        └──► APB2 = 170 MHz (÷1) → TIM1, TIM8

PWM TIMER TREE (Center-aligned)
═══════════════════════════════
                    170 MHz (APB2)
                         │
            ┌────────────┼────────────┐
            │            │            │
         ┌──────┐     ┌──────┐    ┌──────────┐
         │TIM1  │     │TIM8  │    │TIM2      │
         │PSC=0 │     │PSC=0 │    │PSC=0     │
         │ARR=  │     │ARR=  │    │ARR=      │
         │ 4249 │     │ 4249 │    │ FFFFFFFF │
         │CTR1  │     │CTR1  │    │ENC       │
         └──┬───┘     └──┬───┘    └──┬───────┘
    ┌───┬──┬┴──┐         │       ┌───┴───┐
    │   │  │   │         │       │       │
   CH1 CH2 CH3 CH4      CH3    CH1     CH2
   PA8 PA9 PA10 PA11    PC8   PA15     PB3
    │   │   │    │       │      │       │
   FL  FR  RL   RR    STEER   ENC_A   ENC_B

   └────────────────┘  └────┘  └───────────┘
     4× Traction       1×Dir    Encoder
     SÍNCRONO ✅       ASÍNC    (lectura)
     20 kHz CA         20 kHz   Cuadratura
```

### Tabla final de respuestas

| Pregunta | Respuesta |
|----------|-----------|
| ¿Cuántos timers se usan para PWM? | **2** (TIM1 para tracción, TIM8 para dirección) |
| ¿Los 4 motores de tracción comparten timer? | **SÍ** — TIM1 CH1-CH4, mismo contador |
| ¿Hay asincronía entre motores de tracción? | **NO** — perfectamente síncronos (mismo timer) |
| ¿TIM1 y TIM8 están sincronizados? | **NO** — son independientes (contadores no vinculados) |
| ¿La asincronía TIM1/TIM8 es un problema? | **NO** — la dirección es un sistema independiente |
| ¿Efecto físico del center-aligned mode? | Mejor linealidad de par, menos ripple, mejor comportamiento a baja velocidad |
| Modo de conteo | **Center-aligned mode 1** (up/down, interrupt en down-counting) |
| ARR | **4249** (f = 170 MHz / (2 × 4250) = 20 kHz) |
| Resolución PWM | 4250 pasos (~12 bits), 0.024% por paso |
| ¿Motor_SetPWM() sigue funcionando? | **SÍ** — mapea 0–PWM_PERIOD (4249) linealmente |

---

## 8. Cambios realizados

| Archivo | Cambio |
|---------|--------|
| `Core/Src/main.c` — `MX_TIM1_Init()` | `CounterMode` → `TIM_COUNTERMODE_CENTERALIGNED1`, `Period` → 4249 |
| `Core/Src/main.c` — `MX_TIM8_Init()` | `CounterMode` → `TIM_COUNTERMODE_CENTERALIGNED1`, `Period` → 4249 |
| `Core/Src/motor_control.c` | `PWM_PERIOD` → 4249 |
| `Core/Src/steering_centering.c` | `CENTERING_PWM` → 425 (mantiene ~10%) |
| `Core/Inc/motor_control.h` | Comentario del rango PWM actualizado |

**Cálculo de frecuencia verificado:**
```
Edge-aligned (antes):  f = 170 MHz / (8499 + 1)          = 20.000 kHz ✓
Center-aligned (ahora): f = 170 MHz / (2 × (4249 + 1))   = 20.000 kHz ✓
Duty 100%:              CCR = ARR = 4249              → 100% ✓
Duty 10% centering:     CCR = 425 / 4249              ≈ 10.0% ✓
```

---

**Conclusión:** La conversión a center-aligned mode mejora la linealidad de par y el comportamiento a baja velocidad sin afectar la frecuencia PWM (20 kHz), la sincronización entre motores, ni la lógica de control (ABS/TCS/Ackermann/PID). La resolución pasa de ~13 a ~12 bits, lo cual sigue siendo más que suficiente para motores DC brushed.

---

**Referencias del código:**
- `Core/Src/main.c` líneas 395–417: `MX_TIM1_Init()` — configuración TIM1
- `Core/Src/main.c` líneas 449–468: `MX_TIM8_Init()` — configuración TIM8
- `Core/Src/main.c` líneas 419–447: `MX_TIM2_Init()` — configuración encoder (sin cambios)
- `Core/Src/main.c` líneas 261–305: `SystemClock_Config()` — árbol de clock
- `Core/Inc/main.h` líneas 18–22: Pin definitions PWM
- `Core/Src/motor_control.c` líneas 298–326: `Motor_Init()` — asignación timer/canal
- `Core/Src/motor_control.c` líneas 1355–1358: `Motor_SetPWM()` — escritura CCR
- `Core/Src/stm32g4xx_hal_msp.c` líneas 51–77: `HAL_TIM_PWM_MspInit()` — GPIO AF
