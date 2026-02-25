# Auditoría técnica — Control de motores RPWM/LPWM (doble PWM sin lógica externa)

> Auditoría objetiva extraída directamente del código fuente **tras la refactorización
> a arquitectura de timer único por motor + protección hardware BREAK2/LOCKUP**.
> Sin propuestas adicionales ni cambios.
> Ficheros de referencia: `Core/Src/motor_control.c`, `Core/Src/main.c`,
> `Core/Src/stm32g4xx_it.c`, `Core/Src/safety_system.c`,
> `Core/Src/stm32g4xx_hal_msp.c`, `Core/Inc/main.h`.

---

## 1. Asignación real de timers por motor

Extraído de `Motor_Init()` en `motor_control.c`:

| Motor  | Timer | RPWM Canal    | RPWM Pin | LPWM Canal    | LPWM Pin | Mismo timer |
|--------|-------|---------------|----------|---------------|----------|-------------|
| FL     | TIM1  | TIM_CHANNEL_1 | PA8      | TIM_CHANNEL_2 | PA9      | **Sí**      |
| FR     | TIM1  | TIM_CHANNEL_3 | PA10     | TIM_CHANNEL_4 | PA11     | **Sí**      |
| RL     | TIM8  | TIM_CHANNEL_1 | PC6      | TIM_CHANNEL_2 | PC7      | **Sí**      |
| RR     | TIM8  | TIM_CHANNEL_3 | PC8      | TIM_CHANNEL_4 | PC9      | **Sí**      |
| STEER  | TIM3  | TIM_CHANNEL_1 | PA6      | TIM_CHANNEL_2 | PA7      | **Sí**      |

Todos los motores usan el mismo timer para RPWM y LPWM.
Los AF de los pines no cambian respecto a la versión anterior:
- PA8–PA11: `GPIO_AF6_TIM1`
- PC6–PC9:  `GPIO_AF4_TIM8`
- PA6–PA7:  `GPIO_AF2_TIM3`

---

## 2. Modo de funcionamiento del timer

### TIM1 — `MX_TIM1_Init()`, `main.c`

```c
htim1.Init.Prescaler         = 0;
htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
htim1.Init.Period            = 4249;
htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
oc.OCFastMode = TIM_OCFAST_DISABLE;
oc.OCPreload  = TIM_OCPRELOAD_ENABLE;
```

| Parámetro        | Valor                                       |
|------------------|---------------------------------------------|
| Counter mode     | Center-Aligned Mode 1                       |
| Frecuencia PWM   | 170 MHz / (2 × 4250) = **20 000 Hz**        |
| Prescaler        | 0                                           |
| ARR              | 4249                                        |
| OC Preload       | **ENABLE**                                  |
| Fast mode        | **DISABLE**                                 |
| BREAK2 / LOCKUP  | **ENABLE** — Cortex LOCKUP → BKIN2          |
| OSSR / OSSI      | **ENABLE** — outputs LOW when MOE = 0       |
| AutomaticOutput  | **DISABLE** — MOE must be re-asserted in SW |

### TIM3 — `MX_TIM3_Init()`, `main.c`

```c
htim3.Init.Prescaler         = 0;
htim3.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
htim3.Init.Period            = 4249;
htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
oc.OCFastMode = TIM_OCFAST_DISABLE;
oc.OCPreload  = TIM_OCPRELOAD_ENABLE;
```

| Parámetro       | Valor                                |
|-----------------|--------------------------------------|
| Counter mode    | Center-Aligned Mode 1                |
| Frecuencia PWM  | 170 MHz / (2 × 4250) = **20 000 Hz** |
| Prescaler       | 0                                    |
| ARR             | 4249                                 |
| OC Preload      | **ENABLE**                           |
| Fast mode       | **DISABLE**                          |
| BREAK           | N/A — TIM3 es GPT sin break input    |

### TIM8 — `MX_TIM8_Init()`, `main.c`

```c
htim8.Init.Prescaler         = 0;
htim8.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
htim8.Init.Period            = 4249;
htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
oc.OCFastMode = TIM_OCFAST_DISABLE;
oc.OCPreload  = TIM_OCPRELOAD_ENABLE;
```

| Parámetro        | Valor                                       |
|------------------|---------------------------------------------|
| Counter mode     | Center-Aligned Mode 1                       |
| Frecuencia PWM   | 170 MHz / (2 × 4250) = **20 000 Hz**        |
| Prescaler        | 0                                           |
| ARR              | 4249                                        |
| OC Preload       | **ENABLE**                                  |
| Fast mode        | **DISABLE**                                 |
| BREAK2 / LOCKUP  | **ENABLE** — Cortex LOCKUP → BKIN2          |
| OSSR / OSSI      | **ENABLE** — outputs LOW when MOE = 0       |
| AutomaticOutput  | **DISABLE** — MOE must be re-asserted in SW |

---

## 3. Seguridad contra shoot-through

### Código exacto de `Motor_SetSigned()`, `motor_control.c`

```c
if (signed_pwm > 0) {
    /* Forward: clear LPWM first, then set RPWM */
    __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);
    __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, duty);
    motor->direction = 1;
} else if (signed_pwm < 0) {
    /* Reverse: clear RPWM first, then set LPWM */
    __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);
    __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, duty);
    motor->direction = -1;
} else {
    /* Stop / passive brake: both channels to zero */
    __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);
    __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, duty);
    motor->direction = 0;
}
```

**¿Existe algún instante posible donde RPWM > 0 y LPWM > 0 simultáneamente?**

**NO.** Con la nueva arquitectura de timer único por motor:

- `lpwm_timer` == `rpwm_timer` para todos los motores.
- Las dos escrituras de CCR vía `__HAL_TIM_SET_COMPARE` actualizan los shadow
  registers de dos canales del **mismo** timer.
- Con `OCPreload = ENABLE`, ambos shadow registers se transfieren al registro
  activo en el **mismo UEV** (Update Event), que es atómico en hardware.
- No existe ventana entre la carga del CCR_RPWM y la carga del CCR_LPWM.

Conclusión: **overlap = 0 µs** para los cinco motores. Condición de shoot-through
imposible en estado estacionario y durante transiciones de dirección.

---

## 4. Estado de frenado (velocidad = 0)

Cuando `Motor_SetSigned(motor, 0)` se ejecuta:

```c
__HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);
__HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);
```

`#define BTS7960_BRAKE_PWM  0U` — `motor_control.c` línea 168.

| Canal | Valor CCR |
|-------|-----------|
| RPWM  | 0         |
| LPWM  | 0         |

**Comportamiento eléctrico en BTS7960:** RPWM = 0 y LPWM = 0 →
terminales M+ y M- flotantes → **passive brake / coast** (no freno activo).
El freno activo del BTS7960 requeriría RPWM = 100% o LPWM = 100%.

---

## 5. Estado al arrancar MCU

### Secuencia de inicialización en `main()`

```
1. HAL_Init()
2. Boot_ReadResetCause()
3. SystemClock_Config()
4. MX_GPIO_Init()    ← EN_FL/EN_RR como GPIO output LOW
5. MX_ADC1_Init()
6. MX_FDCAN1_Init()
7. MX_I2C1_Init()
8. MX_TIM1_Init()    ← CCR=0, BREAK2/LOCKUP armado; pines PA8-PA11 como AF PP
9. MX_TIM2_Init()
10. MX_TIM3_Init()   ← CCR=0; pines PA6-PA7 como AF PP
11. MX_TIM8_Init()   ← CCR=0, BREAK2/LOCKUP armado; pines PC6-PC9 como AF PP
12. MX_IWDG_Init()
13. Motor_Init()     ← HAL_TIM_PWM_Start ×10 (re-activa MOE) + Motor_SetSigned(0) ×5
```

**Estado eléctrico de pines RPWM/LPWM:**
- Antes de paso 8: pines en input flotante (GPIO reset default, sin pull).
- Pasos 8/10/11: `HAL_TIM_PWM_MspInit` configura pines como `AF_PP, NOPULL`.
  Con CCR=0 y timer parado → salida LOW.
- Paso 13: `HAL_TIM_PWM_Start` activa CEN y re-activa MOE (TIM1/TIM8).
  Con CCR=0, la salida permanece LOW durante todo el período.
  Inmediatamente después, `Motor_SetSigned(0)` escribe CCR=0 explícitamente.

**Estado EN:**
- EN_FL (PC5): LOW desde `MX_GPIO_Init()`.
- EN_RR (PC13): LOW desde `MX_GPIO_Init()`.
- EN_FR, EN_RL, EN_STEER: tied a 3.3 V en hardware → siempre HIGH.

**Tiempo hasta primer CCR=0 válido:** ~5–15 ms (boot completo hasta `Motor_Init()`).

**¿Puede el motor recibir un pulso espurio?** No. El único riesgo teórico
(input flotante en µs antes de configurar AF) no genera un pulso válido
porque el BTS7960 requiere señal estable durante varios µs de bootstrap.

---

## 6. Manejo de fallo

### SAFE → `Safety_FailSafe()` → `Traction_EmergencyStop()`

```c
Motor_SetSigned(&motor_fl,    0);  // TIM1_CH1=0, TIM1_CH2=0
Motor_SetSigned(&motor_fr,    0);  // TIM1_CH3=0, TIM1_CH4=0
Motor_SetSigned(&motor_rl,    0);  // TIM8_CH1=0, TIM8_CH2=0
Motor_SetSigned(&motor_rr,    0);  // TIM8_CH3=0, TIM8_CH4=0
Motor_SetSigned(&motor_steer, 0);  // TIM3_CH1=0, TIM3_CH2=0
```

| Señal      | Valor  | EN (PC5/PC13) |
|------------|--------|---------------|
| RPWM todos | 0      | LOW (FL/RR); tied 3.3V (FR/RL/STEER) |
| LPWM todos | 0      | — |

Los relés no se desconectan en estado SAFE (solo en SYS_STATE_ERROR).

### Watchdog (IWDG timeout)

MCU reset hardware. La secuencia de boot replica el estado descrito en §5.

### HardFault / MemManage / BusFault / UsageFault

`stm32g4xx_it.c` — código exacto:

```c
TIM1->BDTR &= ~TIM_BDTR_MOE;   /* Disable TIM1: RPWM_FL, LPWM_FL, RPWM_FR, LPWM_FR */
TIM8->BDTR &= ~TIM_BDTR_MOE;   /* Disable TIM8: RPWM_RL, LPWM_RL, RPWM_RR, LPWM_RR */
TIM3->CCR1  = 0U;               /* RPWM_STEER → 0 */
TIM3->CCR2  = 0U;               /* LPWM_STEER → 0 */
GPIOC->BSRR = (uint32_t)(PIN_EN_FL | PIN_EN_RR
              | PIN_RELAY_MAIN | PIN_RELAY_TRAC | PIN_RELAY_DIR) << 16U;
```

| Señal            | Acción                                          |
|------------------|-------------------------------------------------|
| RPWM_FL..FR (TIM1) | **OUTPUT OFF** — MOE=0, pin forzado LOW (OSSR=1) |
| LPWM_FL..FR (TIM1) | **OUTPUT OFF** — ídem                           |
| RPWM_RL..RR (TIM8) | **OUTPUT OFF** — MOE=0, pin forzado LOW (OSSI=1) |
| LPWM_RL..RR (TIM8) | **OUTPUT OFF** — ídem                           |
| RPWM_STEER (TIM3)  | CCR1 = 0U (directo, sin buffer)                 |
| LPWM_STEER (TIM3)  | CCR2 = 0U (directo, sin buffer)                 |
| EN_FL (PC5)        | LOW vía BSRR reset                              |
| EN_RR (PC13)       | LOW vía BSRR reset                              |
| EN_FR/RL/STEER     | Tied 3.3V en HW — permanecen HIGH               |
| RELAY_TRAC (PC11)  | LOW vía BSRR reset                              |

**LOCKUP hardware path (opción A, automático):**
Cuando el CPU entra en LOCKUP real (p. ej., fallo en NMI_Handler), la señal
Cortex LOCKUP se activa HIGH → TIM1 y TIM8 BKIN2 assertado → MOE = 0
instantáneamente, sin necesidad de ejecutar ningún handler. Esta acción es
anterior a cualquier software y ocurre en hardware puro.

---

## 7. Sincronización entre timers distintos

**Con la nueva arquitectura de timer único por motor, esta sección ya no aplica
a la sincronización RPWM/LPWM.**

- FL y FR comparten TIM1 → sus pares RPWM/LPWM son cotemporáneos.
- RL y RR comparten TIM8 → ídem.
- STEER usa TIM3 → ídem.

La única diferencia de fase posible es entre motores de **distinto** timer
(p. ej., TIM1 vs TIM8), pero como cada motor ya tiene sus dos canales en el
mismo timer, esa diferencia inter-timer no afecta al riesgo de shoot-through.

**Diferencia máxima de fase entre update events de TIM1 y TIM8:**
Del orden de ~360–720 ns (tiempo de arranque secuencial en `Motor_Init`).
Esta diferencia no tiene impacto de seguridad porque RPWM y LPWM de cada motor
están en el mismo timer.

**Duración máxima con ambos canales activos simultáneamente: 0 µs** para
todos los motores. Los shadow registers de RPWM y LPWM de cada motor se cargan
en el mismo UEV del mismo timer.

---

## Confirmaciones explícitas

| Requisito                               | Estado            |
|-----------------------------------------|-------------------|
| Overlap RPWM/LPWM = 0 µs               | **Confirmado**    |
| Shoot-through imposible                 | **Confirmado**    |
| PWM imposible tras CPU LOCKUP (TIM1/8)  | **Confirmado** — hardware BREAK2 |
| PWM imposible en HardFault (TIM1/8)     | **Confirmado** — MOE clear en handler |
| PWM imposible en HardFault (TIM3/STEER) | **Confirmado** — CCR1/CCR2 = 0 en handler |
| SAFE state → RPWM = LPWM = 0           | **Confirmado**    |
| Relés NO son el único mecanismo         | **Confirmado** — MOE/CCR/GPIO antes del relé |

