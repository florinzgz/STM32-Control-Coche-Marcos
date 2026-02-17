# Análisis de Uso de Timers Hardware — PWM Motores

**Fecha:** 2026-02-17  
**Versión:** 1.0  
**Fuentes:** `Core/Src/main.c`, `Core/Inc/main.h`, `Core/Src/motor_control.c`, `Core/Src/stm32g4xx_hal_msp.c`

---

## 1. Tabla de salidas PWM — Configuración exacta extraída del código

### 1.1 Motores de tracción (4 ruedas)

| Motor | GPIO Puerto/Pin | Timer | Canal | Frecuencia PWM | Modo contador | Prescaler | Periodo (ARR) | AF |
|-------|----------------|-------|-------|-----------------|---------------|-----------|---------------|-----|
| FL (Delantero Izq.) | **PA8** | **TIM1** | CH1 | 20 kHz | Edge-aligned (UP) | 0 | 8499 | AF6 |
| FR (Delantero Der.) | **PA9** | **TIM1** | CH2 | 20 kHz | Edge-aligned (UP) | 0 | 8499 | AF6 |
| RL (Trasero Izq.) | **PA10** | **TIM1** | CH3 | 20 kHz | Edge-aligned (UP) | 0 | 8499 | AF6 |
| RR (Trasero Der.) | **PA11** | **TIM1** | CH4 | 20 kHz | Edge-aligned (UP) | 0 | 8499 | AF6 |

### 1.2 Motor de dirección

| Motor | GPIO Puerto/Pin | Timer | Canal | Frecuencia PWM | Modo contador | Prescaler | Periodo (ARR) | AF |
|-------|----------------|-------|-------|-----------------|---------------|-----------|---------------|-----|
| Steering (Dirección) | **PC8** | **TIM8** | CH3 | 20 kHz | Edge-aligned (UP) | 0 | 8499 | AF4 |

### 1.3 Encoder de dirección (no es PWM — lectura cuadratura)

| Función | GPIO Puerto/Pin | Timer | Canal | Modo | Prescaler | Periodo (ARR) | AF |
|---------|----------------|-------|-------|------|-----------|---------------|-----|
| Encoder canal A | **PA15** | **TIM2** | CH1 | Encoder TI12 | 0 | 0xFFFFFFFF | AF1 |
| Encoder canal B | **PB3** | **TIM2** | CH2 | Encoder TI12 | 0 | 0xFFFFFFFF | AF1 |

---

## 2. Detalle de configuración por timer

### 2.1 TIM1 — PWM 4 motores de tracción

```c
/* Fuente: Core/Src/main.c — MX_TIM1_Init() */
htim1.Instance               = TIM1;
htim1.Init.Prescaler         = 0;
htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;       /* Edge-aligned */
htim1.Init.Period            = 8499;
htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
htim1.Init.RepetitionCounter = 0;
htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

/* OC mode PWM1, polarity HIGH, pulse initial = 0 */
/* Channels: CH1 (PA8/FL), CH2 (PA9/FR), CH3 (PA10/RL), CH4 (PA11/RR) */
```

**Cálculo de frecuencia:**
```
f_timer = SYSCLK / (PSC + 1) = 170 MHz / 1 = 170 MHz
f_pwm   = f_timer / (ARR + 1) = 170 MHz / 8500 = 20.000 kHz ✓
```

**Resolución PWM:** 8500 pasos (0–8499), equivalente a ~13.05 bits efectivos.

**Tipo de timer:** TIM1 es un timer avanzado (Advanced-Control Timer) con:
- 4 canales de captura/comparación
- Soporte para dead-time insertion y complementary outputs
- Repetition counter
- Break inputs (no usados)

### 2.2 TIM8 — PWM motor de dirección

```c
/* Fuente: Core/Src/main.c — MX_TIM8_Init() */
htim8.Instance               = TIM8;
htim8.Init.Prescaler         = 0;
htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;       /* Edge-aligned */
htim8.Init.Period            = 8499;
htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
htim8.Init.RepetitionCounter = 0;
htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

/* OC mode PWM1, polarity HIGH, pulse initial = 0 */
/* Only CH3 (PC8) is used */
```

**Cálculo:** Idéntico a TIM1 → 20 kHz.

### 2.3 TIM2 — Encoder cuadratura (no PWM)

```c
/* Fuente: Core/Src/main.c — MX_TIM2_Init() */
htim2.Instance               = TIM2;
htim2.Init.Prescaler         = 0;
htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
htim2.Init.Period            = 0xFFFFFFFF;    /* 32-bit full range */
enc.EncoderMode              = TIM_ENCODERMODE_TI12;
enc.IC1Filter = enc.IC2Filter = 6;            /* ~210 ns glitch rejection */
```

Este timer no genera PWM — cuenta pulsos del encoder E6B2-CWZ6C.

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
- El **mismo contador CNT** que cuenta de 0 a 8499
- El **mismo prescaler** (PSC = 0)
- El **mismo periodo** (ARR = 8499)
- El **mismo modo de conteo** (UP / edge-aligned)
- La **misma frecuencia base** (170 MHz)

Cada canal tiene su propio **registro CCR** (Compare/Capture Register) que establece el duty cycle individual. El firmware escribe en estos registros independientemente:

```c
/* Core/Src/motor_control.c — Motor_SetPWM() */
static void Motor_SetPWM(Motor_t *motor, uint16_t pwm) {
    if (pwm > PWM_PERIOD) pwm = PWM_PERIOD;
    __HAL_TIM_SET_COMPARE(motor->timer, motor->channel, pwm);
}
```

Esto permite que cada motor tenga un **duty cycle independiente** (para Ackermann differential, ABS/TCS per-wheel scaling, etc.) mientras que todos los flancos PWM están **sincronizados al mismo instante** dentro de cada periodo de 50 µs.

---

## 4. ¿Algún PWM de motor es asíncrono respecto a los demás?

### Respuesta: TIM1 (tracción) y TIM8 (dirección) son asíncronos entre sí

#### 4.1 Dentro de TIM1: TOTALMENTE SÍNCRONO ✅

Los 4 motores de tracción (FL, FR, RL, RR) están en TIM1 CH1–CH4. Comparten el mismo contador, por lo que:
- Todos los flancos de subida PWM ocurren en el **mismo ciclo de reloj** (cuando CNT = 0)
- Todos los flancos de bajada ocurren cuando CNT alcanza el valor CCRx respectivo
- **No hay desfase de fase entre los 4 canales** — son perfectamente síncronos

Esto es **correcto y deseable** para control de tracción. Los 4 motores reciben su señal PWM en fase, lo que asegura que las actualizaciones de duty cycle aplicadas en `Traction_Update()` producen cambios de par simultáneos en las 4 ruedas.

#### 4.2 TIM1 vs TIM8: ASÍNCRONOS ⚠️ (pero aceptable)

TIM1 y TIM8 son timers independientes. Aunque ambos tienen la misma configuración (PSC=0, ARR=8499, 20 kHz), sus contadores **no están sincronizados**:

- No se usa el mecanismo de sincronización interna del STM32 (Master/Slave Timer Trigger, ITR)
- Ambos timers arrancan con `HAL_TIM_PWM_Start()` en secuencia dentro de `Motor_Init()`, por lo que hay un desfase de unos pocos ciclos de reloj entre el inicio de TIM1 y TIM8
- El desfase resultante es del orden de **nanosegundos** y es constante durante toda la ejecución

**¿Es esto un problema?**

**No, para este sistema.** TIM8 controla el motor de **dirección**, que es un sistema de control independiente (PID de posición con encoder). El motor de dirección no necesita sincronización de fase con los motores de tracción porque:

1. Opera en un loop de control diferente (posición angular, no velocidad/par)
2. Su dinámica es mucho más lenta que la frecuencia PWM (ancho de banda PID << 20 kHz)
3. No hay interacción eléctrica entre el motor de dirección y los motores de tracción (drivers BTS7960 separados, fuentes de alimentación separadas: 12V dirección vs 24V tracción)

---

## 5. Efectos físicos reales sobre la estabilidad de tracción

### 5.1 Escenario actual: TIM1 con 4 canales — Análisis

**Efecto positivo: Sincronización perfecta de par en las 4 ruedas**

Al usar un solo timer (TIM1) para las 4 ruedas:
- Las actualizaciones de duty cycle se aplican atómicamente en el mismo ciclo PWM
- No hay "jitter" entre ruedas que pueda causar diferencial de par no intencionado
- El Ackermann differential, ABS y TCS actúan simultáneamente en las 4 ruedas

**Cálculo del efecto de un hipotético desfase:**

Si los motores usaran timers separados (hipotético caso incorrecto), un desfase de fase máximo de medio periodo PWM (25 µs) causaría:
- Con demand de 50% duty (momento máximo de diferencial): un motor estaría aplicando par mientras otro no
- A 20 kHz esto dura 25 µs — tiempo insuficiente para que la inercia del motor/rueda responda (constante eléctrica del motor DC >> 25 µs)
- **Efecto físico real:** imperceptible en este caso, pero el principio de diseño correcto es mantener la sincronización

### 5.2 El diseño actual es correcto ✅

La decisión de usar **TIM1 CH1-CH4 para las 4 ruedas** es la correcta por estas razones:

| Aspecto | Beneficio |
|---------|-----------|
| **Sincronización de par** | Los 4 motores cambian de estado PWM en el mismo instante — no hay diferencial de par transitorio entre ruedas |
| **ABS/TCS coherencia** | Cuando ABS reduce el duty de una rueda individual (via `wheel_scale[]`), el cambio se aplica en el mismo ciclo PWM que las demás ruedas |
| **Ackermann diferencial** | Los 4 factores `acker_diff[i]` se aplican simultáneamente — no hay latencia de actualización entre ruedas interiores y exteriores |
| **EMI reducida** | Al conmutar las 4 ruedas simultáneamente, los picos de corriente se suman pero ocurren en el mismo instante — no hay spreading de EMI en el tiempo |
| **Simplificación de firmware** | Un solo timer para inicializar y controlar — menos código, menos fallos posibles |

---

## 6. ¿Se necesita corrección del mapping de timers?

### NO — El mapping actual ya es óptimo ✅

El firmware actual ya utiliza la configuración ideal:

| Timer | Uso | Canales | ¿Correcto? |
|-------|-----|---------|------------|
| **TIM1** (Advanced) | 4× PWM tracción | CH1 (PA8), CH2 (PA9), CH3 (PA10), CH4 (PA11) | ✅ Perfecto — 4 motores síncronos en un solo timer avanzado |
| **TIM8** (Advanced) | 1× PWM dirección | CH3 (PC8) | ✅ Correcto — sistema independiente, no requiere sincronización con tracción |
| **TIM2** (General 32-bit) | Encoder cuadratura | CH1 (PA15), CH2 (PB3) | ✅ Correcto — timer 32-bit para evitar overflow del encoder |

**¿Por qué no se necesita cambiar?**

1. **TIM1 ya tiene los 4 motores de tracción** — Esta es exactamente la configuración recomendada. Un solo timer avanzado con 4 canales es la mejor práctica para control de tracción multi-rueda.

2. **TIM8 para dirección es independiente por diseño** — El motor de dirección opera en un loop de control completamente diferente (PID de posición) con dinámica independiente. No se beneficiaría de sincronización con TIM1.

3. **No hay timers "desperdiciados"** — TIM2 se usa para el encoder cuadratura (requiere timer de 32 bits). TIM1 y TIM8 se usan para PWM. Los timers restantes (TIM3, TIM4, TIM6, TIM7, etc.) están libres para futuras expansiones.

### 6.1 Propuesta alternativa (solo si se quisiera sincronizar TIM1 y TIM8)

Si en el futuro se necesitara que el motor de dirección estuviera sincronizado con los motores de tracción (por ejemplo, para coordinación de steering-by-wire con torque vectoring avanzado), se podría configurar TIM1 como Master y TIM8 como Slave usando el Internal Trigger (ITR):

```c
/* Hipotético — NO necesario en el diseño actual */
/* TIM1 como Master: genera trigger en Update Event */
TIM_MasterConfigTypeDef sMasterConfig = {0};
sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

/* TIM8 como Slave: reset del contador en trigger de TIM1 */
TIM_SlaveConfigTypeDef sSlaveConfig = {0};
sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
sSlaveConfig.InputTrigger = TIM_TS_ITR0;  /* TIM1 → TIM8 via ITR0 */
HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig);
```

**Esta sincronización NO está implementada ni es necesaria** en el sistema actual.

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

PWM TIMER TREE
══════════════
                    170 MHz (APB2)
                         │
            ┌────────────┼────────────┐
            │            │            │
         ┌──────┐     ┌──────┐    ┌──────────┐
         │TIM1  │     │TIM8  │    │TIM2      │
         │PSC=0 │     │PSC=0 │    │PSC=0     │
         │ARR=  │     │ARR=  │    │ARR=      │
         │ 8499 │     │ 8499 │    │ FFFFFFFF │
         │UP    │     │UP    │    │ENC       │
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
     20 kHz            20 kHz   Cuadratura
```

### Tabla final de respuestas

| Pregunta | Respuesta |
|----------|-----------|
| ¿Cuántos timers se usan para PWM? | **2** (TIM1 para tracción, TIM8 para dirección) |
| ¿Los 4 motores de tracción comparten timer? | **SÍ** — TIM1 CH1-CH4, mismo contador |
| ¿Hay asincronía entre motores de tracción? | **NO** — perfectamente síncronos (mismo timer) |
| ¿TIM1 y TIM8 están sincronizados? | **NO** — son independientes (contadores no vinculados) |
| ¿La asincronía TIM1/TIM8 es un problema? | **NO** — la dirección es un sistema independiente |
| ¿Efecto físico de la asincronía en tracción? | **Ninguno** — la tracción ya es síncrona |
| ¿Se necesita corrección del mapping? | **NO** — el diseño actual ya es óptimo |
| ¿Qué cambiar si se quisiera sincronizar TIM1+TIM8? | Configurar Master/Slave via ITR (código ejemplo en §6.1) |

---

**Conclusión:** El diseño de timers del STM32G474RE en este proyecto es **correcto y óptimo**. Los 4 motores de tracción están en un solo timer avanzado (TIM1) con sincronización perfecta de fase. El motor de dirección está en un timer independiente (TIM8) lo cual es apropiado para su loop de control separado (PID de posición). No se requiere ninguna corrección.

---

**Referencias del código:**
- `Core/Src/main.c` líneas 395–417: `MX_TIM1_Init()` — configuración TIM1
- `Core/Src/main.c` líneas 449–468: `MX_TIM8_Init()` — configuración TIM8
- `Core/Src/main.c` líneas 419–447: `MX_TIM2_Init()` — configuración encoder
- `Core/Src/main.c` líneas 261–305: `SystemClock_Config()` — árbol de clock
- `Core/Inc/main.h` líneas 18–22: Pin definitions PWM
- `Core/Src/motor_control.c` líneas 298–326: `Motor_Init()` — asignación timer/canal
- `Core/Src/motor_control.c` líneas 1355–1358: `Motor_SetPWM()` — escritura CCR
- `Core/Src/stm32g4xx_hal_msp.c` líneas 51–77: `HAL_TIM_PWM_MspInit()` — GPIO AF
