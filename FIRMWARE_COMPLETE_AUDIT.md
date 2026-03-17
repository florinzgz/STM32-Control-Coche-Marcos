# Auditoría Completa del Firmware STM32G474RE

**Proyecto:** STM32-Control-Coche-Marcos  
**MCU:** STM32G474RETx (LQFP64, Cortex-M4F, 170 MHz)  
**CubeMX:** v6.17.0  
**HAL:** STM32Cube FW_G4 V1.6.2  
**Fecha:** 2026-03-17  
**Tipo:** Auditoría de solo lectura — ningún archivo fue modificado

---

## Resumen ejecutivo

Se revisaron todos los archivos del firmware STM32: `.ioc`, `.cproject`, `.ld`, `system_stm32g4xx.c`, `stm32g4xx_hal_conf.h`, `stm32g4xx_hal_msp.c`, `stm32g4xx_it.c`, `startup_stm32g474retx.s`, `main.c`, `Makefile`, `makefile.init`, y todos los módulos de usuario (`motor_control`, `can_handler`, `sensor_manager`, `safety_system`, `steering_centering`, `boot_validation`, `encoder_reader`, `eps_params`, `error_log`, `steering_cal_store`, `service_mode`, `ackermann`, `math_safety`).

Se detectaron **17 hallazgos** clasificados por severidad:

| Severidad | Cantidad |
|-----------|----------|
| 🔴 CRÍTICO | 1 |
| 🟠 ALTO | 4 |
| 🟡 MEDIO | 7 |
| 🔵 BAJO | 5 |

---

## 1. 🔴 CRÍTICO — Directiva FPU del startup assembly incoherente con toolchain

**Archivo:** `Core/Startup/startup_stm32g474retx.s`, línea 30  
**Valor actual:** `.fpu softvfp`  
**Valor esperado:** `.fpu fpv4-sp-d16`

**Descripción:**  
El archivo de arranque assembly declara `.fpu softvfp` (emulación de punto flotante por software), pero el resto de la cadena de compilación está configurada para usar FPU hardware:

| Componente | Configuración FPU | Archivo/Línea |
|---|---|---|
| startup .s | `.fpu softvfp` ❌ | `startup_stm32g474retx.s:30` |
| Makefile | `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` ✅ | `Makefile:78` |
| .cproject | `fpv4-sp-d16` + `hard` ABI ✅ | `.cproject:23-24` |
| system_stm32g4xx.c | `FPU->FPCCR |= ((3UL << 30) | (3UL << 28))` ✅ | `system_stm32g4xx.c:13` |

**Impacto:**  
El archivo de startup se ensambla con convención de llamada software para flotantes, mientras que el código C generado usa convención hardware. El vector de reset, la copia de `.data`, y el borrado de `.bss` se ejecutan antes de `main()` — si cualquier función de librería llamada en esa fase (e.g., `__libc_init_array`) usa operaciones float, el ABI sería incompatible. En la práctica, dado que el startup solo copia memoria y salta a `main()`, el impacto real es mínimo, pero es una violación del ABI que puede causar errores sutiles si se modifica el startup o se añaden constructores globales con float.

**Recomendación:**  
Cambiar `.fpu softvfp` a `.fpu fpv4-sp-d16` en `startup_stm32g474retx.s` línea 30 para que coincida con el toolchain.

---

## 2. 🟠 ALTO — TIM2 Period: discrepancia entre .ioc y firmware

**Archivo .ioc:** línea 362 → `TIM2.Period=65535`  
**Archivo main.c:** línea 706 → `htim2.Init.Period = 0xFFFFFFFF;`

**Descripción:**  
El `.ioc` configura TIM2 con Period=65535 (16 bits), pero el código en `MX_TIM2_Init()` lo sobreescribe intencionalmente a `0xFFFFFFFF` (rango completo de 32 bits). TIM2 en STM32G474 es un timer de 32 bits, por lo que el valor del código es correcto para uso como encoder en cuadratura (evita wrap del contador en el rango de ±350° de recorrido).

**Impacto:**  
Si CubeMX regenera el código, revertirá el Period a 65535, lo que causaría overflow del encoder a ±4667 counts (~350° de recorrido mecánico), con wrap silencioso y pérdida de posición de dirección.

**Recomendación:**  
Actualizar el `.ioc` para que `TIM2.Period=4294967295` (0xFFFFFFFF) y así proteger contra regeneraciones de CubeMX. Alternativamente, documentar que `MX_TIM2_Init()` no debe ser regenerado por CubeMX.

---

## 3. 🟠 ALTO — TIM8_UP interrupt habilitado en .ioc pero sin handler

**Archivo .ioc:** línea 84 → `NVIC.TIM8_UP_IRQn=true\:2\:0\:true\:false\:true\:true\:true`  
**Archivo stm32g4xx_it.c:** NO existe `TIM8_UP_IRQHandler`  
**Archivo stm32g4xx_hal_msp.c:** líneas 52-55 → NVIC habilitado en `HAL_TIM_Base_MspInit()` para TIM8

**Descripción:**  
El `.ioc` configura la interrupción TIM8_UP con prioridad 2. El código en `HAL_TIM_Base_MspInit()` (stm32g4xx_hal_msp.c:52-55) tiene código para habilitar `TIM8_UP_IRQn` en NVIC, pero esta función solo es invocada por `HAL_TIM_Base_Init()`, y TIM8 se inicializa con `HAL_TIM_PWM_Init()` (que llama a `HAL_TIM_PWM_MspInit()` en su lugar). Por tanto, la interrupción NO está habilitada en el NVIC en la práctica.

Sin embargo, si alguien cambiara la inicialización a `HAL_TIM_Base_Init()`, o si CubeMX regenerara el código con la interrupción habilitada, `TIM8_UP_IRQHandler` no existe — el vector apunta al `Default_Handler` (bucle infinito), causando un cuelgue del sistema al primer update event de TIM8.

**Impacto:**  
Sin impacto actual (NVIC no habilitado), pero riesgo latente de cuelgue del sistema si se modifica la inicialización o se regenera con CubeMX.

**Recomendación:**  
Opción A: Añadir `void TIM8_UP_IRQHandler(void) { HAL_TIM_IRQHandler(&htim8); }` en `stm32g4xx_it.c`.  
Opción B: Desactivar `NVIC.TIM8_UP_IRQn` en el `.ioc` si la interrupción no es necesaria (TIM8 solo genera PWM).

---

## 4. 🟠 ALTO — FDCAN1_IT1_IRQHandler definido pero no habilitado en NVIC

**Archivo stm32g4xx_it.c:** líneas 100-103 → `FDCAN1_IT1_IRQHandler` definido  
**Archivo .ioc:** línea 71 → Solo `FDCAN1_IT0_IRQn` habilitado  
**Archivo stm32g4xx_hal_msp.c:** líneas 26-27 → Solo `FDCAN1_IT0_IRQn` habilitado en `HAL_FDCAN_MspInit()`

**Descripción:**  
El firmware define `FDCAN1_IT1_IRQHandler` en `stm32g4xx_it.c`, pero la interrupción `FDCAN1_IT1_IRQn` no está habilitada en NVIC ni en el `.ioc`. El handler es código muerto — nunca se ejecutará. Si se esperaba que FDCAN1 utilizara la línea de interrupción IT1 para alguna función (e.g., Tx complete, error), no está funcionando.

**Impacto:**  
Código muerto. Si FDCAN1 genera eventos que deberían ser manejados por IT1 (e.g., interrupciones de transmisión), no serán atendidos.

**Recomendación:**  
Verificar si FDCAN1 IT1 es necesario. Si no, eliminar el handler muerto. Si sí, habilitar `FDCAN1_IT1_IRQn` en NVIC tanto en el `.ioc` como en `HAL_FDCAN_MspInit()`.

---

## 5. 🟠 ALTO — HAL_TIM_Base_MspInit() contiene código de NVIC que nunca se ejecuta

**Archivo:** `stm32g4xx_hal_msp.c`, líneas 31-56

**Descripción:**  
`HAL_TIM_Base_MspInit()` configura NVIC para TIM1 (TIM1_UP_TIM16_IRQn), TIM2 (TIM2_IRQn), y TIM8 (TIM8_UP_IRQn). Sin embargo:

- **TIM1:** Se inicializa con `HAL_TIM_PWM_Init()` → llama `HAL_TIM_PWM_MspInit()`, NO `HAL_TIM_Base_MspInit()`. El NVIC para TIM1_UP nunca se habilita.
- **TIM3:** Se inicializa con `HAL_TIM_PWM_Init()` → mismo caso. El código en `HAL_TIM_Base_MspInit` no tiene NVIC para TIM3 (correcto, ya que TIM3 no necesita interrupt).
- **TIM8:** Mismo caso que TIM1.
- **TIM2:** Se inicializa con `HAL_TIM_Encoder_Init()` → llama `HAL_TIM_Encoder_MspInit()`. El NVIC para TIM2 se habilita en `HAL_TIM_Encoder_MspInit()` (líneas 204-205), pero `HAL_TIM_Encoder_Start()` (no `_IT`) se usa, por lo que el DIER del timer no habilita la interrupción de actualización.

El bloque completo de `HAL_TIM_Base_MspInit()` es efectivamente código muerto excepto por la habilitación de clocks (que se duplica en `HAL_TIM_PWM_MspInit()`/`HAL_TIM_Encoder_MspInit()`).

**Impacto:**  
Los handlers `TIM1_UP_TIM16_IRQHandler` y `TIM2_IRQHandler` en `stm32g4xx_it.c` son código muerto ya que las interrupciones nunca se disparan. Esto no causa fallos pero indica incoherencia entre la estructura MSP y la inicialización real.

**Recomendación:**  
Si no se necesitan interrupciones de timer update, eliminar las llamadas `HAL_NVIC_SetPriority`/`HAL_NVIC_EnableIRQ` de `HAL_TIM_Base_MspInit()` y de `HAL_TIM_Encoder_MspInit()`, así como los handlers muertos en `stm32g4xx_it.c`. Si se necesitan, cambiar a `HAL_TIM_PWM_Start_IT()` o `HAL_TIM_Encoder_Start_IT()`.

---

## 6. 🟡 MEDIO — HAL_DMA_MODULE_ENABLED activo sin uso de DMA

**Archivo:** `Core/Inc/stm32g4xx_hal_conf.h`, línea 14  
**Valor:** `#define HAL_DMA_MODULE_ENABLED`

**Descripción:**  
El módulo DMA de la HAL está habilitado en la configuración, y los archivos de drivers DMA (`stm32g4xx_hal_dma.c`, `stm32g4xx_hal_dma_ex.c`) están incluidos en el Makefile (líneas 52-53). Sin embargo, no hay ningún handle DMA (`DMA_HandleTypeDef`), ninguna llamada a funciones HAL_DMA_*, y ningún canal DMA configurado en el `.ioc` ni en el código.

**Impacto:**  
Incremento innecesario del tamaño del binario (~2-4 KB de flash). No causa errores de funcionamiento.

**Recomendación:**  
Eliminar `#define HAL_DMA_MODULE_ENABLED` del hal_conf.h y remover los archivos DMA del Makefile para reducir el tamaño del binario.

---

## 7. 🟡 MEDIO — Doble inicialización de GPIO para pines PWM de timer

**Archivo:** `stm32g4xx_hal_msp.c`

**Descripción:**  
Los pines PWM se configuran como GPIO con alternate function en DOS callbacks MSP:

| Timer | Pines | HAL_TIM_PWM_MspInit() | HAL_TIM_MspPostInit() |
|-------|-------|----------------------|----------------------|
| TIM1 | PA8-PA11 | Líneas 62-73 | Líneas 146-155 |
| TIM3 | PA6-PA7 | Líneas 76-86 | Líneas 157-166 |
| TIM8 | PC6-PC9 | Líneas 89-100 | Líneas 169-177 |

Ambas funciones configuran los mismos pines con la misma configuración (AF_PP, NOPULL, SPEED_LOW, mismo Alternate Function). `HAL_TIM_PWM_MspInit()` se llama durante `HAL_TIM_PWM_Init()`, y `HAL_TIM_MspPostInit()` no parece ser llamada desde ningún punto del código (es generada por CubeMX pero no invocada tras la migración manual).

**Impacto:**  
Si `HAL_TIM_MspPostInit()` no se llama, es código muerto. Si se llama, es una doble configuración innecesaria que no causa daño pero desperdicia ciclos. Si la función se eliminara pero era necesaria, los pines podrían no configurarse correctamente.

**Recomendación:**  
Verificar si `HAL_TIM_MspPostInit()` es invocada. Si no, eliminarla. Si los pines ya se configuran en `HAL_TIM_PWM_MspInit()`, la doble configuración es redundante.

---

## 8. 🟡 MEDIO — Toggle de LED (PA5) desde ISR y main loop sin sincronización

**Archivo stm32g4xx_it.c:** línea 178 → `HAL_GPIO_TogglePin(GPIOA, PIN_LD2);` en `HAL_FDCAN_RxFifo0Callback()` (contexto ISR)  
**Archivo main.c:** línea 422 → `HAL_GPIO_TogglePin(GPIOA, PIN_LD2);` en bucle principal (contexto thread)

**Descripción:**  
El LED PA5 (LD2) se conmuta tanto desde el callback RX de CAN (contexto de interrupción) como desde el bucle principal cada 200 ms. `HAL_GPIO_TogglePin()` ejecuta internamente una lectura-modificación-escritura del registro BSRR que no es atómica respecto a interrupciones.

El comentario en el código (stm32g4xx_it.c:173-177) reconoce esta situación y la considera intencional para indicar visualmente actividad CAN.

**Impacto:**  
En STM32, `HAL_GPIO_TogglePin()` lee IDR y escribe a BSRR. Si una interrupción CAN ocurre entre la lectura y la escritura del main loop, el toggle del main loop podría perderse. El resultado es un parpadeo impredecible del LED, pero sin impacto funcional ya que LD2 es solo un indicador visual.

**Recomendación:**  
Es un comportamiento aceptable para un LED indicador. Si se desea determinismo, usar `HAL_GPIO_WritePin()` (SET/RESET) en vez de `TogglePin` en uno de los dos contextos, o deshabilitar momentáneamente las interrupciones alrededor del toggle.

---

## 9. 🟡 MEDIO — OneWire usa retardo busy-wait dependiente de frecuencia de CPU

**Archivo:** `Core/Src/sensor_manager.c`, ~línea 490

**Descripción:**  
La implementación OneWire para DS18B20 usa un retardo por busy-wait con NOPs:
```c
static void OW_DelayUs(uint32_t us) {
    uint32_t loops = (uint32_t)us * 42;  /* 170MHz / 4 cycles ≈ 42 */
    while (loops--) { __NOP(); }
}
```

El factor 42 es una aproximación para 170 MHz y asume un número fijo de ciclos por iteración. Si la frecuencia del CPU cambiara, la optimización del compilador cambiara (O0 vs O2), o se añadieran interrupciones que preemptaran el bucle, los tiempos OneWire se desviarían.

**Impacto:**  
Lecturas de temperatura incorrectas o fallo de comunicación con DS18B20 si la temporización se desvía. Las interrupciones EXTI (wheel speed, CAN) pueden preemptar la comunicación OneWire, estirando los pulsos.

**Recomendación:**  
Considerar usar el contador DWT (Data Watchpoint and Trace) para retardos microsegundo precisos e independientes de la optimización:
```c
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
```
Alternativamente, deshabilitar interrupciones (`__disable_irq()`/`__enable_irq()`) durante las secuencias OneWire críticas.

---

## 10. 🟡 MEDIO — LSI no habilitado explícitamente para IWDG

**Archivo:** `Core/Src/main.c`, línea 462  
**Valor:** `RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;`

**Descripción:**  
El IWDG utiliza el oscilador LSI (32 kHz), pero `SystemClock_Config()` solo configura HSI y PLL. La flag `RCC_OSCILLATORTYPE_LSI` no está incluida en `OscillatorType`.

En STM32G4, el IWDG habilita LSI automáticamente al iniciarse (`HAL_IWDG_Init()` escribe en el registro de control del IWDG, que enciende LSI implícitamente). Por tanto, funciona correctamente en la práctica.

**Impacto:**  
Sin impacto funcional real. Sin embargo, es una mala práctica no declarar explícitamente LSI en la configuración RCC, ya que:
- Herramientas de análisis estático pueden reportar el uso de un oscilador no configurado.
- Si se cambiara la inicialización del IWDG, LSI podría no estar disponible.

**Recomendación:**  
Añadir `RCC_OSCILLATORTYPE_LSI` a OscillatorType en `SystemClock_Config()` y configurar `RCC_OscInitStruct.LSIState = RCC_LSI_ON`.

---

## 11. 🟡 MEDIO — Defines de pines "freed" (PC0-PC4) en main.h sin correspondencia en .ioc

**Archivo:** `Core/Inc/main.h`, líneas 42-46

```c
#define PIN_DIR_FL         GPIO_PIN_0   /* PC0 — freed */
#define PIN_DIR_FR         GPIO_PIN_1   /* PC1 — freed */
#define PIN_DIR_RL         GPIO_PIN_2   /* PC2 — freed */
#define PIN_DIR_RR         GPIO_PIN_3   /* PC3 — freed */
#define PIN_DIR_STEER      GPIO_PIN_4   /* PC4 — freed */
```

**Descripción:**  
Estos defines corresponden a pines de dirección que ya no se usan en el firmware (los comentarios dicen "freed"). Los pines PC0-PC4 no están declarados en el `.ioc`, no están inicializados en `MX_GPIO_Init()`, y no se usan en ningún módulo del firmware. Los defines permanecen como documentación.

**Impacto:**  
Sin impacto funcional. Sin embargo, los pines PC0-PC4 quedan en estado por defecto del reset (analog input), lo cual es seguro (alta impedancia, bajo consumo). Si un diseñador futuro conectara hardware a estos pines confiando en los defines, podrían quedar flotantes.

**Recomendación:**  
Considerar eliminar los defines o añadir un comentario más visible indicando que estos pines no están configurados y no deben usarse. Si están físicamente conectados en el PCB, inicializarlos explícitamente en `MX_GPIO_Init()` como output LOW o input con pull-down.

---

## 12. 🟡 MEDIO — Defines PIN_EN_FR, PIN_EN_RL, PIN_EN_STEER son alias de pines de timer

**Archivo:** `Core/Inc/main.h`, líneas 54-57

```c
#define PIN_EN_FR          GPIO_PIN_6   /* PC6  — repurposed: TIM8_CH1 (RPWM_RL) */
#define PIN_EN_RL          GPIO_PIN_7   /* PC7  — repurposed: TIM8_CH2 (LPWM_RL) */
#define PIN_EN_RR          GPIO_PIN_13  /* PC13 — GPIO output, active HIGH */
#define PIN_EN_STEER       GPIO_PIN_9   /* PC9  — repurposed: TIM8_CH4 (LPWM_RR) */
```

**Descripción:**  
`PIN_EN_FR` (PC6), `PIN_EN_RL` (PC7) y `PIN_EN_STEER` (PC9) apuntan a pines que ahora están asignados como salidas de timer (TIM8 CH1, CH2, CH4). Los defines son engañosos — sugieren que son pines de enable GPIO cuando en realidad son salidas PWM. Solo `PIN_EN_FL` (PC5) y `PIN_EN_RR` (PC13) son GPIO outputs reales.

Si alguien usara `HAL_GPIO_WritePin(GPIOC, PIN_EN_FR, GPIO_PIN_SET)` creyendo que controla un enable, estaría intentando escribir en un pin configurado como alternate function, lo que no tendría efecto o podría causar comportamiento inesperado.

**Impacto:**  
Confusión para desarrolladores. Riesgo de uso incorrecto si se modifica el código de control de enables.

**Recomendación:**  
Eliminar los defines `PIN_EN_FR`, `PIN_EN_RL`, `PIN_EN_STEER` o renombrarlos con un prefijo `_LEGACY_` y añadir un `#warning` o comentario prominente. Los comentarios actuales son correctos pero fáciles de pasar por alto.

---

## 13. 🔵 BAJO — PREFETCH_ENABLE desactivado a 170 MHz

**Archivo:** `Core/Inc/stm32g4xx_hal_conf.h`, línea 63  
**Valor:** `#define PREFETCH_ENABLE 0U`

**Descripción:**  
El prefetch de instrucciones está desactivado. A 170 MHz con 8 wait states de flash, el prefetch buffer mejora significativamente el rendimiento al pre-cargar la siguiente línea de instrucciones mientras se ejecuta la actual.

**Impacto:**  
Rendimiento subóptimo en ejecución de código desde flash. No afecta la corrección funcional.

**Recomendación:**  
Activar `PREFETCH_ENABLE` a `1U` para mejorar el rendimiento del core a 170 MHz.

---

## 14. 🔵 BAJO — Heap (0x200) y Stack (0x400) muy pequeños

**Archivo .ioc:** líneas 263, 274  
```
ProjectManager.HeapSize=0x200    (512 bytes)
ProjectManager.StackSize=0x400   (1024 bytes)
```

**Archivo linker:** `STM32G474RETX_FLASH.ld`, líneas 29-30  
```c
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;
```

**Descripción:**  
Con 128 KB de RAM disponibles, el heap de 512 bytes y stack de 1024 bytes son muy conservadores. El firmware usa:
- Estructuras de estado de seguridad, traction, y steering
- Arrays de temperatura DS18B20 (5 sensores × 8 bytes ROM + caches)
- Buffers CAN (RX/TX + headers)
- INA226 caches (6 canales × datos de corriente/voltaje)
- Ring buffer de error log (250 entries × 16 bytes = 4000 bytes en RAM estática)
- Operaciones float con stack frames para PID, Ackermann, safety checks

Si las funciones anidadas (main → Traction_Update → compute_ackermann → Ackermann_ComputeWheelAngles → funciones math) consumen mucho stack, podría ocurrir overflow silencioso.

**Impacto:**  
Riesgo de stack overflow en anidamiento profundo de funciones, especialmente en funciones que usan arrays locales o cálculos float intensivos.

**Recomendación:**  
Incrementar `_Min_Stack_Size` a al menos `0x800` (2048 bytes) y considerar `0x1000` (4096 bytes) para margen de seguridad. Verificar con `-fstack-usage` flag del compilador.

---

## 15. 🔵 BAJO — Makefile no incluye archivos test_*.c pero están en Core/Src/

**Archivo Makefile:** líneas 13-32 (lista de C_SOURCES)  
**Archivos no listados:**
- `Core/Src/test_eps_params.c`
- `Core/Src/test_error_log.c`
- `Core/Src/test_math_safety.c`
- `Core/Src/test_service_mode.c`
- `Core/Src/test_steering_cal_store.c`

**Descripción:**  
Cinco archivos de test existen en `Core/Src/` pero no están incluidos en la lista `C_SOURCES` del Makefile. Esto es probablemente intencional (los tests no deben compilarse en el binario de producción), pero los archivos conviven en el mismo directorio que el código de producción sin una separación clara (no hay un directorio `test/` separado).

**Impacto:**  
Sin impacto en el binario. Riesgo menor de confusión en la organización del proyecto.

**Recomendación:**  
Mover los archivos de test a un subdirectorio `Core/Test/` o `test/` para separar claramente código de test del firmware de producción.

---

## 16. 🔵 BAJO — I2C speed GPIO_SPEED_FREQ_LOW para 400 kHz Fast Mode

**Archivo:** `stm32g4xx_hal_msp.c`, líneas 112-117 (HAL_I2C_MspInit)

```c
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

**Descripción:**  
Los pines I2C1 (PB6/PB7) están configurados con `GPIO_SPEED_FREQ_LOW` para I2C a 400 kHz Fast Mode. `GPIO_SPEED_FREQ_LOW` corresponde a un slew rate máximo de ~2 MHz. Aunque I2C es un protocolo de baja frecuencia y el modo open-drain inherentemente limita el slew rate, la especificación I2C Fast Mode puede requerir transiciones más rápidas en líneas con capacitancia alta.

En la práctica, I2C funciona correctamente con `SPEED_LOW` en la mayoría de configuraciones con líneas cortas y pull-ups adecuados. El modo open-drain depende más de los pull-ups que del slew rate del driver.

**Impacto:**  
Probablemente sin efecto en un bus I2C con líneas cortas. En buses con alta capacitancia (largos o con muchos dispositivos en el multiplexor TCA9548A), podría causar flancos de subida lentos y errores de comunicación ocasionales.

**Recomendación:**  
Considerar cambiar a `GPIO_SPEED_FREQ_MEDIUM` para los pines I2C si se experimentan errores de comunicación intermitentes.

---

## 17. 🔵 BAJO — PWM GPIO speed LOW para 20 kHz en motores de tracción

**Archivo:** `stm32g4xx_hal_msp.c`, líneas 70-71, 84-85, 97-98

**Descripción:**  
Los pines PWM de TIM1 (PA8-PA11), TIM3 (PA6-PA7) y TIM8 (PC6-PC9) están configurados con `GPIO_SPEED_FREQ_LOW`. A 20 kHz PWM con Period=4249, las transiciones son lentas. `GPIO_SPEED_FREQ_LOW` limita el slew rate a ~2 MHz, pero la señal PWM es de solo 20 kHz. Los drivers BTS7960 tienen entradas con histéresis que toleran transiciones lentas.

**Impacto:**  
Sin impacto funcional a 20 kHz. `GPIO_SPEED_FREQ_LOW` es la configuración correcta para minimizar EMI y consumo en señales de baja frecuencia.

**Recomendación:**  
Ninguna acción necesaria. La configuración es apropiada para la frecuencia PWM utilizada.

---

## Validación de coherencia global

### Coherencia Pin Map (.ioc vs MX_GPIO_Init vs HAL_MSP)

| Pin | .ioc | MX_GPIO_Init | HAL MSP | Uso en código | Estado |
|-----|------|-------------|---------|---------------|--------|
| PA0 (WHEEL_FL) | EXTI0, PULLUP ✅ | IT_RISING, PULLUP ✅ | — | sensor_manager.c ✅ | ✅ OK |
| PA1 (WHEEL_FR) | EXTI1, PULLUP ✅ | IT_RISING, PULLUP ✅ | — | sensor_manager.c ✅ | ✅ OK |
| PA2 (WHEEL_RL) | EXTI2, PULLUP ✅ | IT_RISING, PULLUP ✅ | — | sensor_manager.c ✅ | ✅ OK |
| PA3 (PEDAL) | ADC1_IN4 ✅ | — | ADC MspInit: ANALOG ✅ | sensor_manager.c ✅ | ✅ OK |
| PA5 (LD2) | GPIO_Output ✅ | OUTPUT_PP ✅ | — | main.c toggle ✅ | ✅ OK |
| PA6 (RPWM_STEER) | TIM3_CH1 ✅ | — | PWM MspInit: AF2_TIM3 ✅ | motor_control.c ✅ | ✅ OK |
| PA7 (LPWM_STEER) | TIM3_CH2 ✅ | — | PWM MspInit: AF2_TIM3 ✅ | motor_control.c ✅ | ✅ OK |
| PA8 (RPWM_FL) | TIM1_CH1 ✅ | — | PWM MspInit: AF6_TIM1 ✅ | motor_control.c ✅ | ✅ OK |
| PA9 (LPWM_FL) | TIM1_CH2 ✅ | — | PWM MspInit: AF6_TIM1 ✅ | motor_control.c ✅ | ✅ OK |
| PA10 (RPWM_FR) | TIM1_CH3 ✅ | — | PWM MspInit: AF6_TIM1 ✅ | motor_control.c ✅ | ✅ OK |
| PA11 (LPWM_FR) | TIM1_CH4 ✅ | — | PWM MspInit: AF6_TIM1 ✅ | motor_control.c ✅ | ✅ OK |
| PA15 (ENC_A) | TIM2_CH1 ✅ | — | Encoder MspInit: AF1_TIM2 ✅ | motor_control.c ✅ | ✅ OK |
| PB0 (ONEWIRE) | GPIO_Output ✅ | OUTPUT_OD, PULLUP ✅ | — | sensor_manager.c ✅ | ✅ OK |
| PB3 (ENC_B) | TIM2_CH2 ✅ | — | Encoder MspInit: AF1_TIM2 ✅ | motor_control.c ✅ | ✅ OK |
| PB4 (ENC_Z) | GPIO_Input, PULLUP ✅ | INPUT, PULLUP ✅ | — | No usado (intencional) ✅ | ✅ OK |
| PB5 (STEER_CENTER) | EXTI5, PULLUP ✅ | IT_RISING, PULLUP ✅ | — | sensor_manager.c ✅ | ✅ OK |
| PB6 (I2C_SCL) | I2C1_SCL ✅ | — | I2C MspInit: AF_OD, AF4 ✅ | sensor_manager.c ✅ | ✅ OK |
| PB7 (I2C_SDA) | I2C1_SDA ✅ | — | I2C MspInit: AF_OD, AF4 ✅ | sensor_manager.c ✅ | ✅ OK |
| PB8 (FDCAN_RX) | FDCAN1_RX ✅ | — | FDCAN MspInit: AF9 ✅ | can_handler.c ✅ | ✅ OK |
| PB9 (FDCAN_TX) | FDCAN1_TX ✅ | — | FDCAN MspInit: AF9 ✅ | can_handler.c ✅ | ✅ OK |
| PB10 (RELAY_LED) | GPIO_Output ✅ | OUTPUT_PP ✅ | — | can_handler.c ✅ | ✅ OK |
| PB11 (RELAY_LED_REAR) | GPIO_Output ✅ | OUTPUT_PP ✅ | — | can_handler.c ✅ | ✅ OK |
| PB15 (WHEEL_RR) | EXTI15, PULLUP ✅ | IT_RISING, PULLUP ✅ | — | sensor_manager.c ✅ | ✅ OK |
| PC5 (EN_FL) | GPIO_Output ✅ | OUTPUT_PP ✅ | — | motor_control.c ✅ | ✅ OK |
| PC6 (RPWM_RL) | TIM8_CH1 ✅ | — | PWM MspInit: AF4_TIM8 ✅ | motor_control.c ✅ | ✅ OK |
| PC7 (LPWM_RL) | TIM8_CH2 ✅ | — | PWM MspInit: AF4_TIM8 ✅ | motor_control.c ✅ | ✅ OK |
| PC8 (RPWM_RR) | TIM8_CH3 ✅ | — | PWM MspInit: AF4_TIM8 ✅ | motor_control.c ✅ | ✅ OK |
| PC9 (LPWM_RR) | TIM8_CH4 ✅ | — | PWM MspInit: AF4_TIM8 ✅ | motor_control.c ✅ | ✅ OK |
| PC10 (RELAY_MAIN) | GPIO_Output ✅ | OUTPUT_PP ✅ | — | safety_system.c ✅ | ✅ OK |
| PC11 (RELAY_TRAC) | GPIO_Output ✅ | OUTPUT_PP ✅ | — | safety_system.c ✅ | ✅ OK |
| PC12 (RELAY_DIR) | GPIO_Output ✅ | OUTPUT_PP ✅ | — | safety_system.c ✅ | ✅ OK |
| PC13 (EN_RR) | GPIO_Output ✅ | OUTPUT_PP ✅ | — | motor_control.c ✅ | ✅ OK |

**Resultado:** Todos los 30 pines activos del .ioc están correctamente configurados en firmware. No hay pines declarados en .ioc que falten en el firmware, ni pines en el firmware que falten en .ioc.

### Coherencia de Clocks

| Parámetro | .ioc | SystemClock_Config() | system_stm32g4xx.c | Estado |
|-----------|------|---------------------|-------------------|--------|
| HSI | 16 MHz | RCC_HSI_ON ✅ | SystemCoreClock=170M ✅ | ✅ OK |
| PLLM | DIV4 | RCC_PLLM_DIV4 ✅ | — | ✅ OK |
| PLLN | 85 | 85 ✅ | — | ✅ OK |
| PLLR | DIV2 | RCC_PLLR_DIV2 ✅ | — | ✅ OK |
| SYSCLK | 170 MHz | PLLCLK ✅ | 170000000 ✅ | ✅ OK |
| AHB | DIV1 | DIV1 ✅ | — | ✅ OK |
| APB1 | DIV1 | DIV1 ✅ | — | ✅ OK |
| APB2 | DIV1 | DIV1 ✅ | — | ✅ OK |
| Flash WS | — | FLASH_LATENCY_8 ✅ | — | ✅ OK (170 MHz reqs 8 WS per RM0440) |
| FDCAN CLK | PCLK1 | RCC_FDCANCLKSOURCE_PCLK1 ✅ | — | ✅ OK |

### Coherencia FDCAN bit timing

| Parámetro | .ioc | MX_FDCAN1_Init() | Cálculo |
|-----------|------|------------------|---------|
| Prescaler | 17 | 17 ✅ | — |
| TimeSeg1 | 14 | 14 ✅ | — |
| TimeSeg2 | 5 | 5 ✅ | — |
| SJW | 1 | 1 ✅ | — |
| Baud rate | 500 kbps | — | 170M / (17 × (1+14+5)) = 500 kbps ✅ |

### Coherencia del Linker Script y Flash NVM

| Región | Dirección | Tamaño | Uso | Estado |
|--------|-----------|--------|-----|--------|
| FLASH (código) | 0x08000000 | 500K (→ 0x0807D000) | Programa | ✅ OK |
| Page 125 | 0x0807D000 | 4K | error_log.c ✅ | ✅ OK |
| Page 126 | 0x0807E000 | 4K | steering_cal_store.c ✅ | ✅ OK |
| Page 127 (EPS_PARAMS) | 0x0807F000 | 4K | eps_params.c ✅ | ✅ OK |
| RAM | 0x20000000 | 128K | Datos | ✅ OK |

- No hay solapamiento entre FLASH (500K) y las páginas NVM (páginas 125-127).
- El region EPS_PARAMS en el linker (0x0807F000, 4K) coincide con `EPS_FLASH_BASE` en eps_params.c.
- Las páginas 125 y 126 no están declaradas como regiones en el linker (se acceden por dirección absoluta).

### Coherencia NVIC

| Interrupt | .ioc habilitado | NVIC en MSP | Handler en IT.c | Estado |
|-----------|----------------|-------------|-----------------|--------|
| EXTI0 | ✅ (prio 2) | MX_GPIO_Init ✅ | EXTI0_IRQHandler ✅ | ✅ OK |
| EXTI1 | ✅ (prio 2) | MX_GPIO_Init ✅ | EXTI1_IRQHandler ✅ | ✅ OK |
| EXTI2 | ✅ (prio 2) | MX_GPIO_Init ✅ | EXTI2_IRQHandler ✅ | ✅ OK |
| EXTI9_5 | ✅ (prio 2) | MX_GPIO_Init ✅ | EXTI9_5_IRQHandler ✅ | ✅ OK |
| EXTI15_10 | ✅ (prio 2) | MX_GPIO_Init ✅ | EXTI15_10_IRQHandler ✅ | ✅ OK |
| FDCAN1_IT0 | ✅ (prio 1) | FDCAN MspInit ✅ | FDCAN1_IT0_IRQHandler ✅ | ✅ OK |
| FDCAN1_IT1 | ❌ No en .ioc | ❌ No en MSP | ⚠️ Handler existe (muerto) | ⚠️ Hallazgo #4 |
| I2C1_EV | ✅ (prio 3) | I2C MspInit ✅ | I2C1_EV_IRQHandler ✅ | ✅ OK |
| I2C1_ER | ✅ (prio 3) | I2C MspInit ✅ | I2C1_ER_IRQHandler ✅ | ✅ OK |
| TIM1_UP | ✅ (prio 2) | Base MspInit (no llamado) | TIM1_UP_TIM16_IRQHandler ✅ | ⚠️ Hallazgo #5 |
| TIM2 | ✅ (prio 2) | Encoder MspInit ✅ | TIM2_IRQHandler ✅ | ⚠️ Hallazgo #5 |
| TIM8_UP | ✅ (prio 2) | Base MspInit (no llamado) | ❌ No existe | ⚠️ Hallazgo #3 |

### Coherencia de Makefile con .ioc y .cproject

| Aspecto | Makefile | .cproject | .ioc | Estado |
|---------|---------|-----------|------|--------|
| MCU Target | STM32G474xx | STM32G474RETx | STM32G474RETx | ✅ OK |
| CPU | cortex-m4 | cortex-m4 | M4 | ✅ OK |
| FPU | fpv4-sp-d16 hard | fpv4-sp-d16 hard | — | ✅ OK |
| Optimization | -O2 | -O0 (Debug) | — | ℹ️ Diferente pero esperado |
| Defines | STM32G474xx, USE_HAL_DRIVER | STM32G474xx, USE_HAL_DRIVER, DEBUG | — | ✅ OK (DEBUG solo en .cproject) |
| Linker script | STM32G474RETX_FLASH.ld | STM32G474RETX_FLASH.ld | — | ✅ OK |
| Include paths | 5 paths ✅ | 5 paths ✅ | — | ✅ OK |

### Orden de inicialización en main()

| # | Función | Orden .ioc | Estado |
|---|---------|-----------|--------|
| 1 | HAL_Init() | — | ✅ OK (antes de todo) |
| 2 | Boot_ReadResetCause() | — | ✅ OK (antes de IWDG) |
| 3 | SystemClock_Config() | #2 | ✅ OK |
| 4 | MX_GPIO_Init() | #1 | ✅ OK |
| 5 | MX_ADC1_Init() | #9 | ✅ OK (orden diferente a .ioc pero sin dependencia) |
| 6 | MX_FDCAN1_Init() | #3 | ✅ OK |
| 7 | MX_I2C1_Init() | #4 | ✅ OK |
| 8 | MX_TIM1_Init() | #5 | ✅ OK |
| 9 | MX_TIM2_Init() | #6 | ✅ OK |
| 10 | MX_TIM3_Init() | #7 | ✅ OK |
| 11 | MX_TIM8_Init() | #8 | ✅ OK |
| 12 | MX_IWDG_Init() | #10 | ✅ OK (último, después de leer reset cause) |

Todas las funciones `MX_*` declaradas en el `.ioc` son llamadas en `main()`. El orden difiere del `.ioc` pero no hay dependencias que lo requieran.

### Seguridad eléctrica al arranque

| Pin/Señal | Estado al reset | Estado tras MX_GPIO_Init | Riesgo | Estado |
|-----------|----------------|--------------------------|--------|--------|
| EN_FL (PC5) | Reset LOW | Output LOW ✅ | Motor apagado | ✅ OK |
| EN_RR (PC13) | Reset LOW | Output LOW ✅ | Motor apagado | ✅ OK |
| RELAY_MAIN (PC10) | Reset LOW | Output LOW ✅ | Relé abierto | ✅ OK |
| RELAY_TRAC (PC11) | Reset LOW | Output LOW ✅ | Relé abierto | ✅ OK |
| RELAY_DIR (PC12) | Reset LOW | Output LOW ✅ | Relé abierto | ✅ OK |
| RELAY_LED (PB10) | Reset LOW | Output LOW ✅ | LED OFF | ✅ OK |
| RELAY_LED_REAR (PB11) | Reset LOW | Output LOW ✅ | LED OFF | ✅ OK |
| PWM TIM1 (PA8-11) | AF reset HIGH-Z | AF mode, Pulse=0 ✅ | No PWM | ✅ OK |
| PWM TIM3 (PA6-7) | AF reset HIGH-Z | AF mode, Pulse=0 ✅ | No PWM | ✅ OK |
| PWM TIM8 (PC6-9) | AF reset HIGH-Z | AF mode, Pulse=0 ✅ | No PWM | ✅ OK |
| ONEWIRE (PB0) | Reset ANALOG | OD + PULLUP + SET ✅ | Bus idle | ✅ OK |
| ENC_Z (PB4) | Reset ANALOG | Input + PULLUP ✅ | No flotante | ✅ OK |

Todos los pines de salida críticos arrancan en estado seguro (LOW = motores OFF, relés abiertos, PWM duty 0%).

---

## Conclusión

El firmware presenta una buena calidad general con:
- ✅ Coherencia completa entre pines .ioc y firmware
- ✅ Clocks correctamente configurados y verificados
- ✅ Flash NVM sin solapamientos
- ✅ Arranque seguro (todos los actuadores OFF)
- ✅ Protección hardware (BREAK2/LOCKUP en TIM1/TIM8)
- ✅ Protecciones software exhaustivas (safety system, fault handlers)
- ✅ FDCAN bit timing correcto a 500 kbps

Los hallazgos más relevantes son:
1. 🔴 La directiva FPU del startup assembly debe corregirse
2. 🟠 El TIM2 Period del .ioc debe actualizarse para prevenir regeneración incorrecta
3. 🟠 Los handlers/NVIC de timer muertos deben limpiarse
4. 🟡 El módulo DMA puede eliminarse para reducir el binario
5. 🟡 Los defines de pines reasignados deben clarificarse

Ninguno de los hallazgos causa un fallo funcional inmediato en el firmware actual, pero el hallazgo #1 (FPU startup) es una violación de ABI que debería corregirse a la mayor brevedad.
