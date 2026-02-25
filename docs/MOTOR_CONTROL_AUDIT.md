# Auditoría técnica — Control de motores RPWM/LPWM (doble PWM sin lógica externa)

> Auditoría objetiva extraída directamente del código fuente.
> Sin propuestas de mejora ni cambios.
> Ficheros de referencia: `Core/Src/motor_control.c`, `Core/Src/main.c`,
> `Core/Src/stm32g4xx_it.c`, `Core/Src/safety_system.c`,
> `Core/Src/stm32g4xx_hal_msp.c`.

---

## 1. Asignación real de timers por motor

Extraído de `Motor_Init()` en `motor_control.c` líneas 382–410:

| Motor  | RPWM Timer | RPWM Canal       | LPWM Timer | LPWM Canal       | Mismo timer |
|--------|-----------|------------------|-----------|------------------|-------------|
| FL     | TIM1      | TIM_CHANNEL_1    | TIM8      | TIM_CHANNEL_1    | **No**      |
| FR     | TIM1      | TIM_CHANNEL_2    | TIM8      | TIM_CHANNEL_2    | **No**      |
| RL     | TIM1      | TIM_CHANNEL_3    | TIM3      | TIM_CHANNEL_1    | **No**      |
| RR     | TIM1      | TIM_CHANNEL_4    | TIM3      | TIM_CHANNEL_2    | **No**      |
| STEER  | TIM8      | TIM_CHANNEL_3    | TIM8      | TIM_CHANNEL_4    | **Sí**      |

---

## 2. Modo de funcionamiento de los timers

### TIM1 — `MX_TIM1_Init()`, `main.c` líneas 554–580

```c
htim1.Init.Prescaler         = 0;
htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
htim1.Init.Period            = 4249;
htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
oc.OCFastMode = TIM_OCFAST_DISABLE;
oc.OCPreload  = TIM_OCPRELOAD_ENABLE;
```

| Parámetro            | Valor                      |
|----------------------|----------------------------|
| Counter mode         | Center-Aligned Mode 1      |
| Frecuencia PWM       | 170 MHz / (2 × 4250) = **20 000 Hz** |
| Prescaler            | 0                          |
| ARR                  | 4249                       |
| OC Preload           | **ENABLE**                 |
| Fast mode            | **DISABLE**                |

### TIM3 — `MX_TIM3_Init()`, `main.c` líneas 612–636

```c
htim3.Init.Prescaler         = 0;
htim3.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
htim3.Init.Period            = 4249;
htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
oc.OCFastMode = TIM_OCFAST_DISABLE;
oc.OCPreload  = TIM_OCPRELOAD_ENABLE;
```

| Parámetro            | Valor                      |
|----------------------|----------------------------|
| Counter mode         | Center-Aligned Mode 1      |
| Frecuencia PWM       | 170 MHz / (2 × 4250) = **20 000 Hz** |
| Prescaler            | 0                          |
| ARR                  | 4249                       |
| OC Preload           | **ENABLE**                 |
| Fast mode            | **DISABLE**                |

### TIM8 — `MX_TIM8_Init()`, `main.c` líneas 638–661

```c
htim8.Init.Prescaler         = 0;
htim8.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
htim8.Init.Period            = 4249;
htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
oc.OCFastMode = TIM_OCFAST_DISABLE;
oc.OCPreload  = TIM_OCPRELOAD_ENABLE;
```

| Parámetro            | Valor                      |
|----------------------|----------------------------|
| Counter mode         | Center-Aligned Mode 1      |
| Frecuencia PWM       | 170 MHz / (2 × 4250) = **20 000 Hz** |
| Prescaler            | 0                          |
| ARR                  | 4249                       |
| OC Preload           | **ENABLE**                 |
| Fast mode            | **DISABLE**                |

---

## 3. Seguridad contra shoot-through

### Código exacto de `Motor_SetSigned()`, `motor_control.c` líneas 1822–1851

```c
static void Motor_SetSigned(Motor_t *motor, int16_t signed_pwm)
{
    uint16_t duty = (signed_pwm >= 0) ? (uint16_t)signed_pwm
                                      : (uint16_t)(-signed_pwm);
    if (duty > PWM_PERIOD) duty = PWM_PERIOD;

    if (signed_pwm > 0) {
        /* Forward: clear LPWM first, then set RPWM */
        __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);   // ← apaga LPWM
        __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, duty); // ← enciende RPWM
        motor->direction = 1;
    } else if (signed_pwm < 0) {
        /* Reverse: clear RPWM first, then set LPWM */
        __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);   // ← apaga RPWM
        __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, duty); // ← enciende LPWM
        motor->direction = -1;
    } else {
        /* Stop / passive brake: both channels to zero */
        __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);   // ← apaga RPWM
        __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);   // ← apaga LPWM
        motor->direction = 0;
    }
    ...
}
```

**Análisis shoot-through:**

`__HAL_TIM_SET_COMPARE` escribe directamente en el registro `CCRx` del timer.
Con `OCPreload = ENABLE`, el valor nuevo queda en el registro shadow (preload) y se
transfiere al registro activo al inicio del **siguiente período** del timer
(evento Update, UEV), **no inmediatamente**.

Esto significa:

1. La escritura `0U` al CCR inactivo actualiza el shadow register del canal inactivo.
2. La escritura `duty` al CCR activo actualiza el shadow register del canal activo.
3. Ambos cambios se aplican en el **mismo UEV** si el timer genera el evento antes
   de que finalice la función (periodo = 50 µs a 20 kHz).

**¿Existe algún instante posible donde RPWM > 0 y LPWM > 0 simultáneamente?**

- **Para motor STEER** (RPWM y LPWM en TIM8): los dos shadow registers se
  actualizan en la misma llamada dentro del mismo período de timer. Dado que
  OCPreload está activo, el canal previamente activo permanece en su valor anterior
  hasta el UEV, y el nuevo canal activo también parte de 0. El intervalo entre las
  dos escrituras de CCR es de 2–4 ciclos de CPU (~12–24 ns a 170 MHz). Los shadow
  registers de TIM8 se cargan simultáneamente en el UEV. En un ciclo normal, no
  hay instante donde ambos > 0 en el pin físico.

- **Para motores FL/FR/RL/RR** (RPWM en TIM1, LPWM en TIM8 o TIM3): los timers
  son **independientes** (ver sección 7). Después de la transición de dirección
  existe una ventana de hasta un período completo (50 µs) donde el canal anterior
  del timer A aún no ha cargado el 0U desde su shadow register, mientras el nuevo
  canal del timer B ya ha cargado el valor `duty`. **Durante este intervalo de hasta
  50 µs ambos canales pueden ser simultáneamente > 0 en el pin físico.**

---

## 4. Estado de frenado (velocidad = 0)

Cuando `Motor_SetSigned(motor, 0)` se ejecuta (rama `else` de `Motor_SetSigned`):

```c
__HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);
__HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);
```

| Canal | Valor escrito en CCR |
|-------|----------------------|
| RPWM  | 0                    |
| LPWM  | 0                    |

`#define BTS7960_BRAKE_PWM  0U` — `motor_control.c` línea 168.

**Comportamiento eléctrico en BTS7960:**
Con RPWM = 0 y LPWM = 0, ambas entradas de control están permanentemente LOW.
Esto pone el BTS7960 en su estado de **freno pasivo** (passive brake):
ambos FETs superiores OFF, ambos FETs inferiores OFF → terminales M+ y M-
flotantes. El motor puede girar libremente (**coast**, no freno activo).

> Nota: el freno activo del BTS7960 se consigue con RPWM = 100% o LPWM = 100%,
> lo que conecta ambos terminales del motor al mismo potencial a través de los
> FETs inferiores. El firmware actual no genera ese estado.

---

## 5. Estado al arrancar MCU

### Secuencia de inicialización en `main()` (líneas 116–137)

```
1. HAL_Init()
2. Boot_ReadResetCause()
3. SystemClock_Config()
4. MX_GPIO_Init()        ← GPIO EN configurados como salidas, LOW por defecto
5. MX_ADC1_Init()
6. MX_FDCAN1_Init()
7. MX_I2C1_Init()
8. MX_TIM1_Init()        ← HAL_TIM_PWM_Init: configura TIM1, CCRx = 0 (Pulse=0)
9. MX_TIM2_Init()
10. MX_TIM3_Init()        ← HAL_TIM_PWM_Init: configura TIM3, CCRx = 0
11. MX_TIM8_Init()        ← HAL_TIM_PWM_Init: configura TIM8, CCRx = 0
12. MX_IWDG_Init()
13. Motor_Init()          ← HAL_TIM_PWM_Start ×10 + Motor_SetSigned(0) ×5
```

### Estado eléctrico de los pines RPWM/LPWM durante el arranque

**Entre reset y `MX_TIM1_Init()` (pasos 1–7):**

Los pines PA8–PA11 (RPWM_FL..RR) y PC6–PC9 (LPWM/RPWM STEER, LPWM_FL/FR),
PA6–PA7 (LPWM_RL/RR) no tienen aún su modo alternativo configurado.
Son pines de GPIO en modo `reset default` del STM32G4:
- Estado: **Input flotante** (GPIO_Mode = INPUT por defecto tras reset).
- Nivel eléctrico: indeterminado hasta que el modo AF se configure.

`HAL_TIM_PWM_MspInit()` en `stm32g4xx_hal_msp.c` reconfigura los pines como
`GPIO_MODE_AF_PP` con `GPIO_NOPULL` cuando se llama `HAL_TIM_PWM_Init()`.
Antes de esa llamada, los pines están en input flotante sin pull.

**Entre `MX_TIM1/3/8_Init()` y `HAL_TIM_PWM_Start()` en `Motor_Init()`:**

`HAL_TIM_PWM_ConfigChannel` establece `Pulse = 0` para todos los canales
(`oc.Pulse = 0`, líneas 569, 630, 651). Los pines ya están en modo AF.
El timer NO está contando aún (MOE/CEN no están activos hasta `HAL_TIM_PWM_Start`).

Con el timer parado y `TIM_OCMODE_PWM1` + `Pulse=0`:
- El pin de salida del canal está en el estado inactivo = **LOW** (OCPolarity=HIGH,
  output LOW cuando CCR=0 y el timer no cuenta).

**Estado EN antes de `Motor_Init()`:**

`MX_GPIO_Init()` configura PC5 (EN_FL) y PC13 (EN_RR) como GPIO salida
(`GPIO_MODE_OUTPUT_PP`). Tras `HAL_GPIO_Init`, el bit de salida es 0 → **EN = LOW**.

**Después de `HAL_TIM_PWM_Start()` y antes de `Motor_SetSigned(0)`:**

`HAL_TIM_PWM_Start` activa el contador (CEN=1) y, para TIM1 (advanced),
también activa el Main Output Enable (MOE). Con `CCR=0` y center-aligned PWM1,
la salida permanece **LOW** en todo el período. No hay pulso.

Inmediatamente después, `Motor_SetSigned(&motor_X, 0)` escribe `CCR=0U` en
RPWM y LPWM → estado garantizado LOW.

### ¿Puede el motor recibir un pulso espurio?

**No existe pulso espurio** si el proceso de arranque sigue la secuencia anterior.
El único riesgo teórico sería si los pines PA8–PA11/PC6–PC9/PA6–PA7 presentaran
HIGH flotante antes de `HAL_TIM_PWM_MspInit()`. Con `GPIO_NOPULL` el nivel en un
pin flotante es indeterminado, pero el tiempo en ese estado es del orden de
microsegundos (el tiempo que tarda `HAL_TIM_PWM_Init` en llamar a `HAL_TIM_PWM_MspInit`
y devolver control). Los drivers BTS7960 necesitan una señal estable de varios
microsegundos para que su circuito de bootstrap interno genere corriente de gate.

**Tiempo total hasta primer valor válido de PWM:**

La secuencia completa de boot (pasos 1–13) incluyendo `SystemClock_Config` y
calibración ADC (`HAL_ADCEx_Calibration_Start`) tarda aproximadamente **5–15 ms**
antes de que `Motor_Init()` escriba el primer CCR=0 válido.

---

## 6. Manejo de fallo

### SAFE (`Safety_FailSafe()`, `safety_system.c` líneas 1313–1328)

```c
void Safety_FailSafe(void)
{
    Traction_EmergencyStop();  // → Motor_SetSigned(0) ×5
    if (Encoder_HasFault()) {
        Steering_Neutralize();  // → Motor_SetSigned(&motor_steer, 0)
    } else {
        Steering_SetAngle(0.0f);  // PID hacia 0°; Motor_SetSigned con valor calculado
    }
}
```

`Traction_EmergencyStop()` (`motor_control.c` líneas 1355–1371):

```c
Motor_SetSigned(&motor_fl,    0);  // RPWM_FL=0, LPWM_FL=0
Motor_SetSigned(&motor_fr,    0);  // RPWM_FR=0, LPWM_FR=0
Motor_SetSigned(&motor_rl,    0);  // RPWM_RL=0, LPWM_RL=0
Motor_SetSigned(&motor_rr,    0);  // RPWM_RR=0, LPWM_RR=0
Motor_SetSigned(&motor_steer, 0);  // RPWM_STEER=0, LPWM_STEER=0
```

| Pin   | Valor escrito | EN GPIO (PC5 / PC13) |
|-------|---------------|----------------------|
| RPWM  | 0             | LOW (duty=0 → EN=RESET) |
| LPWM  | 0             | LOW |

EN_FR, EN_RL, EN_STEER = tied a 3.3 V en hardware (siempre HIGH).

> Nota: en SAFE el relé TRAC también se abre físicamente (vía `Safety_FailSafe` →
> `Relay_PowerDown` en el camino `SYS_STATE_ERROR`). Para `SYS_STATE_SAFE` el relé
> **no se desconecta** — solo se corta el PWM (motores en coast).

### Watchdog (IWDG timeout)

El IWDG está configurado con reload = 4095, prescaler = 32:
- Timeout = 4096 × 32 / 32000 Hz = **~4096 ms** (≈4 s).
- Al expirar, genera un **MCU reset hardware** (no una ISR).
- Tras el reset, el MCU arranca desde el vector de reset, repite la secuencia de
  boot descrita en sección 5. No hay código de "watchdog handler".
- Estado de pines RPWM/LPWM durante el reset: idéntico al arranque frío (sección 5).

### Fault driver (HardFault / MemManage / BusFault / UsageFault)

`stm32g4xx_it.c` líneas 21–50:

```c
void HardFault_Handler(void)
{
    GPIOC->BSRR = (uint32_t)(PIN_EN_FL | PIN_EN_RR
                  | PIN_RELAY_MAIN | PIN_RELAY_TRAC | PIN_RELAY_DIR) << 16U;
    while (1) { }
}
```

| Señal       | Acción firmware             |
|-------------|-----------------------------|
| RPWM (TIM)  | **Sin acción directa** — los timers siguen corriendo con su último CCR |
| LPWM (TIM)  | **Sin acción directa** — ídem |
| EN_FL (PC5) | Forzado a **LOW** vía BSRR reset bit |
| EN_RR (PC13)| Forzado a **LOW** vía BSRR reset bit |
| EN_FR, EN_RL, EN_STEER | Tied a 3.3V en HW → **permanecen HIGH** |

Conclusión: en fault de Cortex-M, los timers continúan generando PWM con el
último CCR grabado, pero EN_FL y EN_RR se fuerzan LOW. Los motores FL y RR
se desactivan vía EN GPIO. Los motores FR, RL y STEER no tienen EN controlable
por firmware — quedan energizados con el último PWM activo hasta que el relé
físico TRAC se abre (pin PC11, también forzado LOW por el mismo BSRR).

---

## 7. Sincronización entre timers distintos

Afecta a FL (TIM1 / TIM8), FR (TIM1 / TIM8), RL (TIM1 / TIM3), RR (TIM1 / TIM3).

### Arranque de timers

Los timers se arrancan en `Motor_Init()` con llamadas secuenciales a
`HAL_TIM_PWM_Start`:

```
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // t₀
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // t₁ = t₀ + ~a
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // t₂ = t₀ + ~2a
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // t₃ = t₀ + ~3a
HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // t₄ = t₀ + ~4a  ← CEN de TIM8 se activa aquí
HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // t₅
...
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // t₈  ← CEN de TIM3 se activa aquí
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // t₉
```

donde `a` ≈ 10–20 ciclos de CPU (función HAL ligera).

### Diferencia máxima de fase entre UEV de timers distintos

TIM1 y TIM3/TIM8 son instancias separadas. Cada uno tiene su propio contador.
TIM1 se activa primero. TIM8 y TIM3 se activan varios µs después.

- Desfase inicial entre TIM1 y TIM8: del orden de **4 llamadas HAL × ~15 ciclos CPU
  × 6 ns/ciclo ≈ 360 ns**.
- Desfase inicial entre TIM1 y TIM3: similar, ~360–720 ns.
- Estos timers **no tienen mecanismo de sincronización master/slave configurado**
  en el firmware (no hay `TIM_MasterConfigTypeDef` con `MasterSlaveMode` activo en
  ninguno de los tres). Los contadores derivan libremente.

### Duración máxima donde ambos canales podrían quedar activos

Con OCPreload habilitado, una transición de dirección (p. ej., de RPWM=X a LPWM=X
en motor FL) implica:

1. Se escribe `CCR_LPWM(TIM8_CH1) = 0U` → entra en shadow.
2. Se escribe `CCR_RPWM(TIM1_CH1) = 0U` → entra en shadow.
   Ambos escritos antes del próximo UEV de sus timers respectivos.

Pero si **en el ciclo anterior** el motor estaba en avance y se comanda retroceso:

- TIM1 tiene `CCR_RPWM = duty_prev` activo hasta su próximo UEV.
- TIM8 tiene `CCR_LPWM = 0` activo (ya venía de 0).
- Tras la transición, se escribe `CCR_RPWM(TIM1) = 0` y `CCR_LPWM(TIM8) = duty_new`.
- TIM1 cargará `CCR = 0` en su UEV → máximo 1 período (50 µs) más tarde.
- TIM8 cargará `CCR = duty_new` en su UEV → máximo 1 período (50 µs) más tarde.

El peor caso ocurre cuando TIM1 acaba de pasar su UEV justo **antes** de las
escrituras de CCR: TIM1 tardará hasta 50 µs en aplicar el nuevo 0, mientras
TIM8 puede aplicar `duty_new` en su próximo UEV (hasta 50 µs pero posiblemente
antes si está más adelantado en fase).

**Duración máxima de ambos canales activos simultáneamente: hasta 1 período
completo = 50 µs** (en el peor caso de desfase entre TIM1 y TIM8/TIM3).

> Para el motor STEER (TIM8_CH3 y TIM8_CH4 en el mismo timer), los dos shadow
> registers se cargan en el mismo UEV, por lo que no existe esta ventana.
> El overlap es cero para STEER.
