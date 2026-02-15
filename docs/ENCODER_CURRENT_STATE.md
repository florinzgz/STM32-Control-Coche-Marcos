# Informe Técnico: Estado Actual del Encoder Incremental E6B2-CWZ6C

> **Fecha:** 2026-02-15  
> **Tipo:** Auditoría de solo lectura — sin cambios en el código  
> **Encoder:** Omron E6B2-CWZ6C (1200 PPR, NPN open-collector)  
> **MCU:** STM32G474RETx (Cortex-M4, 170 MHz)

---

## 1) Dónde se lee actualmente el encoder

### Archivos implicados

| Archivo | Rol |
|---------|-----|
| `Core/Src/main.c` | Inicialización de TIM2 en modo encoder (`MX_TIM2_Init`, línea 364) |
| `Core/Src/stm32g4xx_hal_msp.c` | GPIO AF + clock enable (`HAL_TIM_Encoder_MspInit`, línea 144) |
| `Core/Src/motor_control.c` | Arranque del encoder, PID de dirección, lectura del contador, health check |
| `Core/Src/steering_centering.c` | Lectura del encoder durante auto-centrado |
| `Core/Src/safety_system.c` | Delegación a `Encoder_CheckHealth()` y gestión de fallos |
| `Core/Src/can_handler.c` | Envío del ángulo por CAN (`CAN_SendStatusSteering`, línea 250) |
| `Core/Inc/main.h` | Definición de constantes `ENCODER_PPR`, `ENCODER_CPR`, pines |

### Función exacta que obtiene la posición

La posición se lee directamente del registro hardware del timer mediante la macro HAL:

```c
// motor_control.c:1067
int32_t encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
```

La conversión a grados se realiza en:

```c
// motor_control.c:1104-1108
float Steering_GetCurrentAngle(void)
{
    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    return (float)cnt * 360.0f / (float)ENCODER_CPR;
}
```

### Quién llama a esa función y cada cuánto

| Llamador | Archivo:Línea | Frecuencia | Descripción |
|----------|---------------|------------|-------------|
| `Steering_ControlLoop()` | `motor_control.c:1067` | 100 Hz (cada 10 ms) | Lee `__HAL_TIM_GET_COUNTER(&htim2)` directamente para el PID |
| `Steering_GetCurrentAngle()` | `motor_control.c:1104` | 10 Hz (cada 100 ms) | Llamada desde `main.c:175` vía `CAN_SendStatusSteering()` |
| `Encoder_CheckHealth()` | `motor_control.c:1154` | 100 Hz (cada 10 ms) | Lectura para verificación de salud |
| `Centering_IsStalled()` | `steering_centering.c:116` | 100 Hz (cada 10 ms) | Durante el auto-centrado |
| `Centering_RangeExceeded()` | `steering_centering.c:134` | 100 Hz (cada 10 ms) | Durante el auto-centrado |
| `compute_ackermann_differential()` | `motor_control.c:897` | 100 Hz (cada 10 ms) | Indirectamente vía `Steering_GetCurrentAngle()` |

---

## 2) Método de lectura usado

**→ Timer en encoder mode (hardware quadrature decoder)**

El TIM2 del STM32G474 está configurado en **modo encoder cuadratura**, que es un modo hardware nativo del periférico timer. No se usa GPIO polling, ni interrupciones EXTI, ni librería externa, ni simulación.

Evidencia:

```c
// main.c:375-389
TIM_Encoder_InitTypeDef enc = {0};
enc.EncoderMode  = TIM_ENCODERMODE_TI12;
// ...
if (HAL_TIM_Encoder_Init(&htim2, &enc) != HAL_OK) {
    Error_Handler();
}
```

```c
// motor_control.c:309
HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
```

El modo `TIM_ENCODERMODE_TI12` (Encoder Mode 3) cuenta en **ambos flancos de ambos canales** (A y B), proporcionando resolución ×4. El conteo se realiza íntegramente en hardware por el periférico TIM2 sin intervención de la CPU.

---

## 3) Pines usados actualmente

### Canal A (TIM2_CH1)

| Propiedad | Valor | Referencia |
|-----------|-------|------------|
| Puerto/Pin | **PA15** | `main.h:50`, `.ioc:107-108` |
| Modo | `GPIO_MODE_AF_PP` (Alternate Function Push-Pull) | `stm32g4xx_hal_msp.c:154` |
| Pull-up/Pull-down | `GPIO_NOPULL` | `stm32g4xx_hal_msp.c:155` |
| Velocidad | `GPIO_SPEED_FREQ_LOW` | `stm32g4xx_hal_msp.c:156` |
| Alternate Function | `GPIO_AF1_TIM2` | `stm32g4xx_hal_msp.c:157` |
| Inicialización | `HAL_TIM_Encoder_MspInit()` | `stm32g4xx_hal_msp.c:144-169` |

### Canal B (TIM2_CH2)

| Propiedad | Valor | Referencia |
|-----------|-------|------------|
| Puerto/Pin | **PB3** | `main.h:51`, `.ioc:118-119` |
| Modo | `GPIO_MODE_AF_PP` (Alternate Function Push-Pull) | `stm32g4xx_hal_msp.c:161` |
| Pull-up/Pull-down | `GPIO_NOPULL` | `stm32g4xx_hal_msp.c:162` |
| Velocidad | `GPIO_SPEED_FREQ_LOW` | `stm32g4xx_hal_msp.c:163` |
| Alternate Function | `GPIO_AF1_TIM2` | `stm32g4xx_hal_msp.c:164` |
| Inicialización | `HAL_TIM_Encoder_MspInit()` | `stm32g4xx_hal_msp.c:144-169` |

### Canal Z (Index)

| Propiedad | Valor | Referencia |
|-----------|-------|------------|
| Puerto/Pin | **PB4** (definido, no inicializado) | `main.h:52` |
| Modo | No configurado (sin inicialización GPIO) | Ver Sección 8 |
| Nota | Definido como `PIN_ENC_Z` pero **no se usa en ningún código** | `main.h:52` |

### Observaciones sobre pull-up

Los pines PA15 y PB3 están configurados como `GPIO_NOPULL`. El encoder E6B2-CWZ6C tiene salida NPN open-collector, por lo que **requiere resistencias pull-up externas** en la PCB. El firmware no proporciona pull-up interno.

---

## 4) Configuración de temporizadores

### TIM2 — Encoder de dirección

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Instancia | **TIM2** | `main.c:366` |
| Modo | **Encoder Mode 3** (`TIM_ENCODERMODE_TI12`) | `main.c:376` |
| Prescaler | **0** (sin prescaler, f = f_timer) | `main.c:367` |
| Period (ARR) | **0xFFFFFFFF** (4.294.967.295) | `main.c:369` |
| Nota sobre ARR | TIM2 es un timer de **32 bits** en el STM32G474. El firmware usa todo el rango para evitar overflow en ±350° de recorrido (±4667 counts a 4800 CPR). | `main.c:369-371` |
| Clock Division | `TIM_CLOCKDIVISION_DIV1` | `main.c:372` |
| Auto-reload Preload | `TIM_AUTORELOAD_PRELOAD_DISABLE` | `main.c:373` |
| Counter Mode | `TIM_COUNTERMODE_UP` (ignorado en encoder mode) | `main.c:368` |
| IC1 Polarity | `TIM_ICPOLARITY_RISING` | `main.c:377` |
| IC1 Selection | `TIM_ICSELECTION_DIRECTTI` | `main.c:378` |
| IC1 Prescaler | `TIM_ICPSC_DIV1` (sin prescaler de input capture) | `main.c:379` |
| **IC1 Filter** | **6** | `main.c:380` |
| IC2 Polarity | `TIM_ICPOLARITY_RISING` | `main.c:385` |
| IC2 Selection | `TIM_ICSELECTION_DIRECTTI` | `main.c:386` |
| IC2 Prescaler | `TIM_ICPSC_DIV1` | `main.c:387` |
| **IC2 Filter** | **6** | `main.c:388` |
| Interrupciones | TIM2_IRQn habilitado, prioridad 2/0 | `stm32g4xx_hal_msp.c:167-168`, `.ioc:78` |
| ISR | `TIM2_IRQHandler()` → `HAL_TIM_IRQHandler(&htim2)` | `stm32g4xx_it.c:114-117` |
| Clock de entrada | 170 MHz (APB1 timer clock, sin prescaler APB1) | `.ioc:253` |

### Discrepancia .ioc vs firmware

| Parámetro | Valor en `.ioc` | Valor en firmware (`main.c`) |
|-----------|-----------------|------------------------------|
| TIM2.Period | `65535` (16-bit) | `0xFFFFFFFF` (32-bit completo) |

El firmware **sobreescribe** el valor generado por CubeMX. El `.ioc` indica period=65535 (`.ioc:311`), pero el código compilado usa `0xFFFFFFFF` (`main.c:369`). Esto es correcto para aprovechar el rango completo de 32 bits de TIM2, pero la discrepancia debería documentarse.

---

## 5) Frecuencia real de actualización

### Dónde se calcula la posición final

La posición angular se calcula en dos puntos:

1. **PID de dirección** — `Steering_ControlLoop()` (`motor_control.c:1067`):
   ```c
   int32_t encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
   float measured = (float)encoder_count;
   ```
   Opera directamente en counts (no convierte a grados para el PID).

2. **Consulta de ángulo** — `Steering_GetCurrentAngle()` (`motor_control.c:1104-1108`):
   ```c
   int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
   return (float)cnt * 360.0f / (float)ENCODER_CPR;
   ```

### En qué momento del loop principal se actualiza

```c
// main.c:84-126 — bloque de 10 ms (100 Hz)
if ((now - tick_10ms) >= 10) {
    // ...
    Safety_CheckEncoder();      // main.c:96  — llama Encoder_CheckHealth()
    // ...
    Steering_ControlLoop();     // main.c:124 — lee encoder para PID
    // ...
}
```

```c
// main.c:154-178 — bloque de 100 ms (10 Hz)
if ((now - tick_100ms) >= 100) {
    // ...
    CAN_SendStatusSteering(     // main.c:174-176
        (int16_t)(Steering_GetCurrentAngle() * 10),
        Steering_IsCalibrated());
}
```

### Latencia estimada entre pulso físico y valor leído

| Etapa | Latencia | Notas |
|-------|----------|-------|
| Filtro hardware IC | **~210 ns** | 6 × fDTS a 170 MHz ≈ 6 × 5.88 ns = ~35 ns por muestra × 6 muestras. Comentario en `main.c:380-384`. |
| Decodificación hardware TIM2 | **~0 ns** | El contador se actualiza automáticamente por hardware |
| Lectura por firmware (PID) | **0–10 ms** | Polling cada 10 ms en `Steering_ControlLoop()` |
| Envío por CAN | **0–100 ms** adicionales | `CAN_SendStatusSteering()` cada 100 ms |
| **Latencia total PID** | **≤ 10 ms + 210 ns** | Dominada por el período del loop de control |
| **Latencia total CAN** | **≤ 100 ms + 210 ns** | Para el valor reportado al ESP32 |

---

## 6) Resolución actual obtenida

### Ticks por vuelta asumidos por el firmware

```c
// main.h:14-15
#define ENCODER_PPR        1200
#define ENCODER_CPR        (ENCODER_PPR * 4)  /* 4800 counts/rev */
```

### Multiplicación x4

**Sí, se usa multiplicación ×4.** El modo `TIM_ENCODERMODE_TI12` cuenta en ambos flancos de ambos canales (A y B), multiplicando por 4 la resolución nativa del encoder:

- **PPR nativo:** 1200 pulsos/revolución
- **CPR con ×4:** 4800 counts/revolución
- **Resolución angular:** 360° / 4800 = **0.075° por count**

Evidencia del comentario en `main.h:12-13`:
```
/* Quadrature mode: 1200 PPR × 4 = 4800 counts/revolution */
/* Resolution: 360° / 4800 = 0.075° per count              */
```

### Conversiones a grados

1. **Counts → grados** (`motor_control.c:1107`):
   ```c
   return (float)cnt * 360.0f / (float)ENCODER_CPR;
   ```

2. **Grados → counts (setpoint PID)** (`motor_control.c:1034`):
   ```c
   steering_pid.setpoint = angle_deg * (float)ENCODER_CPR / 360.0f;
   ```

3. **Deadband en counts** (`motor_control.c:249`):
   ```c
   #define STEERING_DEADBAND_COUNTS  (0.5f * (float)ENCODER_CPR / 360.0f)
   // 0.5° × 4800/360 ≈ 6.67 counts
   ```

4. **Transmisión CAN** — en unidades de 0.1° (`main.c:175`):
   ```c
   (int16_t)(Steering_GetCurrentAngle() * 10)
   ```

No existe conversión a porcentaje en el firmware STM32.

---

## 7) Protección contra rebotes o ruido

### Filtros hardware configurados

Los filtros de input capture del TIM2 están configurados con valor **6** en ambos canales:

```c
// main.c:380-384
enc.IC1Filter    = 6;  /* Digital filter: 6 × fDTS rejects noise from
                        * NPN open-collector outputs (E6B2-CWZ6C).
                        * At 170 MHz ≈ 35 ns per sample → ~210 ns
                        * glitch rejection.  Sufficient for 1200 PPR
                        * at typical steering rates.                */
```

```c
// main.c:388
enc.IC2Filter    = 6;  /* Same digital filter as IC1 for symmetry   */
```

Según RM0440 (Reference Manual STM32G4), un filtro de valor N requiere N muestras consecutivas estables para validar una transición. A `CK_INT` = 170 MHz y `CKD` = DIV1, la ventana de rechazo de glitch es ~210 ns.

### Filtros software

No hay filtros software adicionales (EMA, media móvil, etc.) aplicados al valor del encoder. El PID opera directamente sobre el valor crudo del contador.

### Debounce

No hay debounce software para el encoder. El filtro hardware IC (valor 6) cumple esta función.

### Validación de dirección

La validación de dirección es implícita en el modo encoder cuadratura del hardware. El periférico TIM2 determina automáticamente la dirección de giro basándose en la fase relativa de los canales A y B. No hay validación de dirección software adicional.

Sin embargo, existen tres chequeos de salud que detectan indirectamente problemas de dirección:

1. **Rango máximo** (`motor_control.c:1160`):
   ```c
   if (count > ENC_MAX_COUNTS || count < -ENC_MAX_COUNTS)
   ```
   Con `ENC_MAX_COUNTS = (54° + 20°) × 4800/360 ≈ 987 counts` (`motor_control.c:250`).

2. **Salto implausible** (`motor_control.c:1170`):
   ```c
   if (delta > ENC_MAX_JUMP)  // ENC_MAX_JUMP = 100 counts/ciclo
   ```

3. **Valor congelado** (`motor_control.c:1184`):
   ```c
   if ((now - enc_last_change_tick) > ENC_FROZEN_TIMEOUT_MS)  // 200 ms
   ```

---

## 8) Uso del canal Z (index)

### Definición

El pin está definido pero **no inicializado ni utilizado**:

```c
// main.h:52
#define PIN_ENC_Z          GPIO_PIN_4   /* PB4  - EXTI4 (index pulse) */
```

### Dónde se usa

**En ningún lugar.** No existe:
- Inicialización GPIO de PB4 en `MX_GPIO_Init()` (`main.c:256-306`)
- Handler de EXTI4 en `stm32g4xx_it.c`
- Ninguna referencia a `PIN_ENC_Z` en código ejecutable

### Justificación documentada en el código

```c
// motor_control.c:236-244
/* ---- Encoder fault detection state ----
 * The E6B2-CWZ6C encoder Z-index pulse (PB4/EXTI4) is intentionally NOT used:
 *   1. No EXTI4 hardware initialisation exists in MX_GPIO_Init().
 *   2. Steering uses relative positioning zeroed at Steering_Init(); an
 *      absolute index reference would require a known mechanical alignment
 *      that is not guaranteed by the chassis design.
 *   3. Fault detection is achieved through range, jump and frozen-value
 *      checks on the A/B quadrature channels, which are sufficient for
 *      safety without the Z pulse.
 */
```

### ¿Resetea posición?

**No.** El reset de posición se hace manualmente en dos puntos:

1. `Steering_Init()` (`motor_control.c:351`):
   ```c
   __HAL_TIM_SET_COUNTER(&htim2, 0);
   ```

2. `Centering_Complete()` (`steering_centering.c:104`):
   ```c
   __HAL_TIM_SET_COUNTER(&htim2, 0);
   ```

El centrado usa un **sensor inductivo LJ12A3** en PB5/EXTI5 (no el canal Z del encoder) para detectar el centro mecánico de la cremallera.

---

## 9) Dependencias externas

### Structs CAN que transportan el ángulo

**STM32 → ESP32:**

| CAN ID | Nombre | Contenido | Frecuencia | Referencia |
|--------|--------|-----------|------------|------------|
| `0x204` | `CAN_ID_STATUS_STEERING` | `int16_t angle` (0.1° LE) + `uint8_t calibrated` | 100 ms | `can_handler.h:30`, `can_handler.c:250-258` |

Empaquetado (`can_handler.c:250-258`):
```c
void CAN_SendStatusSteering(int16_t angle, bool calibrated) {
    uint8_t steering_data[3];
    steering_data[0] = (uint8_t)(angle & 0xFF);
    steering_data[1] = (uint8_t)((angle >> 8) & 0xFF);
    steering_data[2] = calibrated ? 1 : 0;
    TransmitFrame(CAN_ID_STATUS_STEERING, steering_data, 3);
}
```

Llamada desde `main.c:174-176`:
```c
CAN_SendStatusSteering(
    (int16_t)(Steering_GetCurrentAngle() * 10),
    Steering_IsCalibrated());
```

**ESP32 → STM32:**

| CAN ID | Nombre | Contenido | Referencia |
|--------|--------|-----------|------------|
| `0x101` | `CAN_ID_CMD_STEERING` | `int16_t angle_raw` (0.1° LE) | `can_handler.h:24`, `can_handler.c:465-470` |

Recepción (`can_handler.c:465-470`):
```c
case CAN_ID_CMD_STEERING:
    int16_t angle_raw = (int16_t)(rx_payload[0] | (rx_payload[1] << 8));
    float requested_deg = (float)angle_raw / 10.0f;
    float validated_deg = Safety_ValidateSteering(requested_deg);
    Steering_SetAngle(validated_deg);
```

### Módulos que consumen el valor del encoder

| Módulo | Archivo | Uso |
|--------|---------|-----|
| **PID de dirección** | `motor_control.c` | `Steering_ControlLoop()` — control en lazo cerrado |
| **Ackermann** | `motor_control.c` | `compute_ackermann_differential()` vía `Steering_GetCurrentAngle()` — diferencial de tracción |
| **Safety** | `safety_system.c` | `Safety_CheckEncoder()` — detección de fallos |
| **Centering** | `steering_centering.c` | Detección de stall y rango durante auto-centrado |
| **CAN handler** | `can_handler.c` | `CAN_SendStatusSteering()` — telemetría al ESP32 |

### Relación con pantalla o control de conducción

**ESP32 (lado HMI):**

| Archivo ESP32 | Uso | Referencia |
|---------------|-----|------------|
| `esp32/src/vehicle_data.h:74` | Struct `SteeringData` con `int16_t angleRaw` (0.1°) |
| `esp32/src/can_rx.cpp:98` | Decodificación: `sd.angleRaw = readS16LE(&f.data[0])` |
| `esp32/src/can_rx.cpp:181` | Dispatch: `case can::STATUS_STEERING: decodeSteering(frame, data)` |
| `esp32/src/screens/drive_screen.cpp:76` | `curSteeringRaw_ = data.steering().angleRaw` |
| `esp32/src/ui/car_renderer.cpp:102-134` | Renderizado de indicador circular de dirección |
| `esp32/src/ui/steering_display.cpp` | Display de dirección (delegado al gauge circular) |

---

## 10) Riesgos detectados

### R1: Pérdida de pulsos — Riesgo BAJO

El modo encoder hardware de TIM2 decodifica pulsos a velocidad del periférico (170 MHz). A 1200 PPR y velocidad máxima de dirección estimada (~200 °/s), la frecuencia máxima de pulsos sería:

```
f_max = 1200 × (200/360) = 667 Hz
```

Esto está muy por debajo de la capacidad del hardware (MHz). **Riesgo negligible** mientras la conexión eléctrica sea fiable.

### R2: Aliasing — Riesgo BAJO

No hay muestreo software del encoder. El conteo es hardware continuo. El PID lee a 100 Hz, pero el contador refleja el valor acumulado real. No existe efecto aliasing en la lectura del contador.

### R3: Overflow del contador — Riesgo MUY BAJO

TIM2 es un timer de 32 bits. Con ARR = `0xFFFFFFFF` (`main.c:369`):

```
Rango: ±2.147.483.647 counts
En grados: ±2.147.483.647 × 360 / 4800 = ±161.061.273°
En vueltas: ±447.392
```

El recorrido mecánico máximo es ±54° ≈ ±720 counts. **Overflow prácticamente imposible.**

Sin embargo, **hay una discrepancia** entre el `.ioc` (Period=65535, 16 bits) y el firmware (0xFFFFFFFF). Si alguien regenera el código con CubeMX sin ajustar, el ARR se reduciría a 65535, lo que aún cubre ±2460° (suficiente), pero eliminaría el margen de 32 bits intencionado.

### R4: Carga CPU — Riesgo MUY BAJO

El encoder opera en hardware puro. La CPU solo realiza:
- 1× lectura de registro de 32 bits cada 10 ms (PID): `__HAL_TIM_GET_COUNTER(&htim2)` — ~1 ciclo de bus
- 1× lectura cada 10 ms (health check)
- 1× lectura cada 100 ms (CAN status)

**Carga CPU del encoder: despreciable** (< 0.001%).

### R5: Jitter en la lectura del PID — Riesgo BAJO-MEDIO

El loop de control en `main.c` usa `HAL_GetTick()` (resolución 1 ms) con comparación `>=`:

```c
// main.c:85
if ((now - tick_10ms) >= 10) {
```

Esto produce un jitter de hasta **±1 ms** en el período del PID (10 ms ± 10%). El `dt` del PID se calcula correctamente (`motor_control.c:1064`):

```c
float dt = (float)(now - last_time) / 1000.0f;
```

El impacto es limitado porque el PID es proporcional puro (Kp=0.09, Ki=0, Kd=0) — sin término integral ni derivativo que amplifiquen el jitter temporal.

### R6: Falta de pull-up interno — Riesgo MEDIO

Los pines PA15 y PB3 tienen `GPIO_NOPULL` (`stm32g4xx_hal_msp.c:155,162`). El E6B2-CWZ6C tiene salida NPN open-collector que **requiere** pull-up. Si las resistencias pull-up externas no están en la PCB o se desconectan, las señales flotarán y el encoder contará erróneamente.

El filtro IC (valor 6) mitiga parcialmente el ruido, pero no sustituye un pull-up adecuado.

### R7: Canal Z no utilizado — Riesgo INFORMATIVO

El canal Z está definido (`main.h:52`) pero no conectado al firmware. Esto es una decisión de diseño documentada (`motor_control.c:236-244`). No representa un riesgo directo, pero impide:
- Corrección de drift acumulado (si existiera)
- Verificación absoluta de posición
- Detección de pérdida de counts entre vueltas

### R8: Fallo del encoder es latched (no recuperable) — Riesgo por diseño

```c
// motor_control.c:1148-1152
if (enc_fault) return;  // Fault is latched — only system reset clears it
```

Una vez detectado un fallo de encoder, la dirección queda neutralizada permanentemente hasta un reset completo del sistema. Esto es una decisión de seguridad consciente, documentada en el comentario, pero implica que un glitch transitorio (ej: vibración mecánica en conector) requiere parada y reinicio del vehículo.

### R9: Discrepancia .ioc vs código — Riesgo BAJO

| Parámetro | `.ioc` (línea 311) | Firmware (`main.c:369`) |
|-----------|--------------------|------------------------|
| TIM2.Period | 65535 | 0xFFFFFFFF |

Regenerar código con CubeMX sobreescribiría el ARR a 65535. Aunque 65535 sigue siendo suficiente para el rango mecánico, reduce el margen de seguridad contra overflow de 32 bits a 16 bits.

### R10: Velocidad GPIO configurada como LOW — Riesgo MUY BAJO

Los pines del encoder tienen `GPIO_SPEED_FREQ_LOW` (`stm32g4xx_hal_msp.c:156,163`). Para una señal de encoder a ~667 Hz máximo, esto es más que suficiente. La velocidad GPIO afecta al slew rate de la salida, no a la entrada. En modo Alternate Function de entrada, el parámetro Speed tiene impacto mínimo.

---

## Resumen de archivos y líneas clave

| Referencia | Descripción |
|------------|-------------|
| `Core/Inc/main.h:11-15` | Definiciones ENCODER_PPR, ENCODER_CPR |
| `Core/Inc/main.h:50-52` | Definiciones PIN_ENC_A, PIN_ENC_B, PIN_ENC_Z |
| `Core/Src/main.c:29` | Declaración `htim2` |
| `Core/Src/main.c:56` | Llamada a `MX_TIM2_Init()` |
| `Core/Src/main.c:96` | Llamada a `Safety_CheckEncoder()` (10 ms) |
| `Core/Src/main.c:124` | Llamada a `Steering_ControlLoop()` (10 ms) |
| `Core/Src/main.c:174-176` | Llamada a `CAN_SendStatusSteering()` (100 ms) |
| `Core/Src/main.c:364-392` | `MX_TIM2_Init()` — config completa del encoder |
| `Core/Src/stm32g4xx_hal_msp.c:144-169` | `HAL_TIM_Encoder_MspInit()` — GPIO + clocks |
| `Core/Src/motor_control.c:199` | PID gains: Kp=0.09, Ki=0, Kd=0 |
| `Core/Src/motor_control.c:247-267` | Constantes encoder: deadband, max counts, max jump, frozen timeout |
| `Core/Src/motor_control.c:309` | `HAL_TIM_Encoder_Start()` |
| `Core/Src/motor_control.c:345-362` | `Steering_Init()` — zero counter, init health |
| `Core/Src/motor_control.c:1010-1035` | `Steering_SetAngle()` — grados→counts |
| `Core/Src/motor_control.c:1037-1102` | `Steering_ControlLoop()` — PID con lectura encoder |
| `Core/Src/motor_control.c:1104-1108` | `Steering_GetCurrentAngle()` — counts→grados |
| `Core/Src/motor_control.c:1146-1192` | `Encoder_CheckHealth()` — 3 checks de salud |
| `Core/Src/steering_centering.c:104` | Zero encoder en centrado |
| `Core/Src/steering_centering.c:116` | Lectura encoder para stall detection |
| `Core/Src/safety_system.c:987-1012` | `Safety_CheckEncoder()` — wrapper de seguridad |
| `Core/Src/can_handler.c:250-258` | `CAN_SendStatusSteering()` — empaquetado CAN |
| `Core/Src/can_handler.c:465-470` | Recepción de comando de dirección |
| `Core/Src/stm32g4xx_it.c:114-117` | `TIM2_IRQHandler()` |
| `STM32-Control-Coche-Marcos.ioc:107-119` | CubeMX pin config PA15/PB3 encoder |
| `STM32-Control-Coche-Marcos.ioc:309-311` | CubeMX TIM2 encoder mode + period |
| `esp32/src/vehicle_data.h:74` | Struct `SteeringData` en ESP32 |
| `esp32/src/can_rx.cpp:98,181` | Decodificación CAN del ángulo en ESP32 |
