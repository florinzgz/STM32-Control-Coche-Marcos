# AUDITORÍA TÉCNICA COMPLETA — STM32-Control-Coche-Marcos

**Fecha**: 2026-02-08  
**Alcance**: Revisión completa del firmware STM32G474RE (8 archivos fuente, 7 headers)  
**Metodología**: Análisis línea por línea del código fuente real. Toda afirmación cita archivo y función.

---

## TABLA 1 – SISTEMAS CONFIRMADOS (100% reales)

| Sistema | Archivo | Función | Estado | Comentario |
|---------|---------|---------|--------|------------|
| **Reloj 170 MHz** | `main.c:160-203` | `SystemClock_Config()` | ✅ ACTIVO | HSI 16 MHz → PLL (/4 × 85 /2) = 170 MHz. Flash latency 8. Regulador SCALE1_BOOST. |
| **PWM Tracción (TIM1)** | `main.c:281-302` | `MX_TIM1_Init()` | ✅ ACTIVO | 4 canales (CH1-CH4), periodo 8499 → 170 MHz / 8500 = **20 kHz**. Pines PA8-PA11. |
| **PWM Dirección (TIM8)** | `main.c:329-348` | `MX_TIM8_Init()` | ✅ ACTIVO | Canal CH3, periodo 8499 → **20 kHz**. Pin PC8. |
| **Encoder Dirección (TIM2)** | `main.c:305-327` | `MX_TIM2_Init()` | ✅ ACTIVO | Modo cuadratura TI12, periodo 65535. Pines PA15 (CH1) + PB3 (CH2). Encoder E6B2-CWZ6C 1200 PPR → 4800 CPR. |
| **ADC Pedal (ADC1)** | `main.c:350-369` | `MX_ADC1_Init()` | ✅ ACTIVO | Canal 4 (PA3), 12 bits, single conversion, sampling 47.5 ciclos. |
| **Lectura Pedal** | `sensor_manager.c:75-83` | `Pedal_Update()` | ✅ ACTIVO | Polling bloqueante (10 ms timeout). Escalado lineal: `raw * 100 / 4095`. Sin filtrado. |
| **Validación Throttle** | `safety_system.c:142-156` | `Safety_ValidateThrottle()` | ✅ ACTIVO | Clamp 0-100%. Corta a 0% si ABS activo. Reduce 50% si TCS activo. Solo funciona en estado ACTIVE. |
| **Integración Pedal→Motor** | `main.c:105-110` | bucle 50 ms | ✅ ACTIVO | `Pedal_GetPercent()` → `Safety_ValidateThrottle()` → `Traction_SetDemand()`. Solo si `Safety_IsCommandAllowed()`. |
| **PID Dirección** | `motor_control.c:211-240` | `Steering_ControlLoop()` | ✅ ACTIVO | Kp=2.0, Ki=0.1, Kd=0.5. Anti-windup ±1000. Salida clamped ±100%. Ejecuta cada 10 ms. |
| **Ángulo Dirección** | `motor_control.c:242-246` | `Steering_GetCurrentAngle()` | ✅ ACTIVO | `encoder_count * 360.0 / 4800.0` → grados. |
| **Límite Dirección ±45°** | `motor_control.c:205-206` | `Steering_SetAngle()` | ✅ ACTIVO | Clamp hardware a ±45° antes de PID. |
| **Rate-limit Dirección** | `safety_system.c:158-179` | `Safety_ValidateSteering()` | ✅ ACTIVO | Máximo 200°/s. Calcula delta vs tiempo real. Ignora dt < 1 ms. |
| **Tracción 4x2** | `motor_control.c:149-154` | `Traction_Update()` | ✅ ACTIVO | Solo motores delanteros (FL/FR). Traseros deshabilitados (PWM=0, enable=0). Modo por defecto. |
| **Tracción 4x4** | `motor_control.c:143-148` | `Traction_Update()` | ✅ ACTIVO | 4 ruedas mismo PWM y dirección. Activado por CAN comando 0x102 bit 0. |
| **Tank Turn** | `motor_control.c:137-142` | `Traction_Update()` | ✅ ACTIVO | Ruedas izquierdas sentido opuesto a derechas. Activado por CAN comando 0x102 bit 1. |
| **Validación Cambio Modo** | `safety_system.c:182-193` | `Safety_ValidateModeChange()` | ✅ ACTIVO | Solo permite cambio a velocidad < 1 km/h (media 4 ruedas). |
| **GPIO Dirección/Enable/Relés** | `main.c:206-247` | `MX_GPIO_Init()` | ✅ ACTIVO | DIR: PC0-PC4. EN: PC5-PC7,PC9,PC13. Relés: PC10-PC12. Output push-pull. |
| **EXTI Ruedas** | `main.c:229-246` | `MX_GPIO_Init()` | ✅ ACTIVO | PA0(EXTI0), PA1(EXTI1), PA2(EXTI2), PB15(EXTI15). Rising edge + pullup. Prioridad 2. |
| **ISR Ruedas** | `stm32g4xx_it.c:79-101` | `EXTI*_IRQHandler()` | ✅ ACTIVO | 4 handlers que llaman a `Wheel_XX_IRQHandler()` → incrementan contadores de pulsos. |
| **Velocidad Ruedas** | `sensor_manager.c:40-58` | `Wheel_ComputeSpeed()` | ✅ ACTIVO | Pulsos → revoluciones (6 pulsos/rev) → distancia (1.2 m circunf.) → km/h y RPM. |
| **ABS por Rueda** | `safety_system.c:217-248` | `ABS_Update()` | ✅ ACTIVO | Detección de slip >20% por rueda individual (bitmask). Intervención: corta throttle a 0 **globalmente**. Umbral mínimo 2 km/h. Ejecuta cada 10 ms. |
| **TCS por Rueda** | `safety_system.c:256-288` | `TCS_Update()` | ✅ ACTIVO | Detección de slip >15% por rueda individual (bitmask). Intervención: reduce throttle 50% **globalmente**. Umbral mínimo 1 km/h. Ejecuta cada 10 ms. |
| **INA226 (6 sensores)** | `sensor_manager.c:92-152` | `Current_ReadAll()` | ✅ ACTIVO | I2C via TCA9548A (0x70). Lee shunt voltage + bus voltage. Shunt 1 mΩ. Conversión: µV / mΩ / 1000 = A. Lectura cada 50 ms. |
| **DS18B20 (5 sensores)** | `sensor_manager.c:154-438` | `Temperature_ReadAll()` | ✅ ACTIVO | OneWire bit-bang en PB0. ROM Search completo con CRC-8. Match ROM individual. Fallback Skip ROM si no hay sensores descubiertos. |
| **Protección Sobrecorriente** | `safety_system.c:296-304` | `Safety_CheckCurrent()` | ✅ ACTIVO | Límite 25A por sensor INA226. Transición a SAFE. Ejecuta cada 10 ms. |
| **Protección Sobretemperatura** | `safety_system.c:309-319` | `Safety_CheckTemperature()` | ✅ ACTIVO | Límite 90°C por sensor DS18B20. Transición a SAFE. Ejecuta cada 10 ms. |
| **Plausibilidad Sensores** | `safety_system.c:351-386` | `Safety_CheckSensors()` | ✅ ACTIVO | Temperatura: -40°C a 125°C. Corriente: 0A a 50A. Velocidad: 0 a 60 km/h. Fallo → SAFE. |
| **Máquina de Estados** | `safety_system.c:56-96` | `Safety_SetState()` | ✅ ACTIVO | BOOT→STANDBY→ACTIVE⇄SAFE→ERROR. Transiciones validadas con switch/case. Solo ACTIVE permite comandos. |
| **Secuencia Relés** | `safety_system.c:119-136` | `Relay_PowerUp/Down()` | ✅ ACTIVO | PowerUp: Main→50ms→Traction→20ms→Direction. PowerDown: Dir→Trac→Main. HAL_Delay (bloqueante). |
| **Emergency Stop** | `safety_system.c:390-399` | `Safety_EmergencyStop()` | ✅ ACTIVO | Deshabilita todos los motores + corta todos los relés + estado ERROR. |
| **FailSafe** | `safety_system.c:401-405` | `Safety_FailSafe()` | ✅ ACTIVO | Traction_EmergencyStop() + dirección a 0°. Ejecutado al entrar en SAFE. |
| **FDCAN 500 kbps** | `main.c:249-265` | `MX_FDCAN1_Init()` | ✅ ACTIVO | Prescaler=17, Seg1=14, Seg2=5 → 170 MHz/(17×20) = 500 kbps. Classic CAN (no FD). PB8(RX)/PB9(TX). |
| **Filtros Hardware CAN** | `can_handler.c:75-102` | `CAN_ConfigureFilters()` | ✅ ACTIVO | Filtro dual: 0x011 heartbeat. Filtro rango: 0x100-0x102 comandos. Rechazo global de todos los demás. |
| **CAN RX + Validación** | `can_handler.c:243-303` | `CAN_ProcessMessages()` | ✅ ACTIVO | Procesa FIFO0. Throttle → `Safety_ValidateThrottle()`. Steering → `Safety_ValidateSteering()`. Mode → `Safety_ValidateModeChange()`. |
| **CAN TX Heartbeat** | `can_handler.c:131-150` | `CAN_SendHeartbeat()` | ✅ ACTIVO | ID 0x001, 4 bytes: counter + state + fault_flags + reserved. Cada 100 ms. |
| **CAN TX Status** | `can_handler.c:152-225` | `CAN_SendStatus*()` | ✅ ACTIVO | Velocidades (0x200), corrientes (0x201), temperaturas (0x202), seguridad (0x203), dirección (0x204). |
| **CAN Timeout Watchdog** | `safety_system.c:322-341` | `Safety_CheckCANTimeout()` | ✅ ACTIVO | 250 ms timeout. Si expira → CAN_TIMEOUT + SAFE. Si se recupera heartbeat → intenta volver a ACTIVE. Auto-transición STANDBY→ACTIVE al recibir primer heartbeat. |
| **CAN RxFifo0 Callback** | `stm32g4xx_it.c:125-128` | `HAL_FDCAN_RxFifo0Callback()` | ✅ ACTIVO | Actualiza `last_can_rx_time` via `Safety_UpdateCANRxTime()`. |
| **IWDG 500 ms** | `main.c:372-381` | `MX_IWDG_Init()` | ✅ ACTIVO | Prescaler 32, reload 4095 → ~500 ms. Refresh en bucle principal (`main.c:152`). |
| **HardFault Handler** | `stm32g4xx_it.c:22-27` | `HardFault_Handler()` | ✅ ACTIVO | Pone a LOW todos los enables + relés via registro BSRR directo. Bucle infinito. |
| **BusFault Handler** | `stm32g4xx_it.c:36-41` | `BusFault_Handler()` | ✅ ACTIVO | Misma acción que HardFault: GPIOC → LOW + loop infinito. |
| **MemManage Handler** | `stm32g4xx_it.c:29-34` | `MemManage_Handler()` | ✅ ACTIVO | Misma acción que HardFault. |
| **UsageFault Handler** | `stm32g4xx_it.c:43-48` | `UsageFault_Handler()` | ✅ ACTIVO | Misma acción que HardFault. |
| **Error_Handler** | `main.c:383-391` | `Error_Handler()` | ✅ ACTIVO | `__disable_irq()` + GPIOC BSRR directo (todo LOW) + bucle infinito. |
| **I2C1 400 kHz** | `main.c:267-279` | `MX_I2C1_Init()` | ✅ ACTIVO | Timing 0x10909CEC para Fast Mode @ 170 MHz. PB6(SCL)/PB7(SDA) open-drain. |
| **HAL MSP FDCAN** | `stm32g4xx_hal_msp.c` | `HAL_FDCAN_MspInit()` | ✅ ACTIVO | PB8/PB9 AF9, NVIC prioridad 1. |
| **HAL MSP TIM PWM** | `stm32g4xx_hal_msp.c` | `HAL_TIM_PWM_MspInit()` | ✅ ACTIVO | PA8-PA11 AF6 (TIM1), PC8 AF4 (TIM8). |
| **HAL MSP TIM Encoder** | `stm32g4xx_hal_msp.c` | `HAL_TIM_Encoder_MspInit()` | ✅ ACTIVO | PA15 AF1 + PB3 AF1 (TIM2). |
| **HAL MSP I2C** | `stm32g4xx_hal_msp.c` | `HAL_I2C_MspInit()` | ✅ ACTIVO | PB6/PB7 AF4 open-drain, NVIC prioridad 3. |
| **HAL MSP ADC** | `stm32g4xx_hal_msp.c` | `HAL_ADC_MspInit()` | ✅ ACTIVO | PA3 modo analógico. |
| **Estadísticas CAN** | `can_handler.c:19` | `can_stats` | ✅ ACTIVO | TX/RX counters, errores TX/RX, último heartbeat ESP32. |

---

## TABLA 2 – CONFIGURADO PERO NO ACTIVO

| Sistema | Archivo | Qué falta | Riesgo |
|---------|---------|-----------|--------|
| **Ackermann Geometry** | `motor_control.c:257-289` | `Ackermann_Compute()` y `Ackermann_SetGeometry()` están implementados con fórmula correcta (radio de giro, ángulo interior/exterior), PERO **ningún código los llama**. `Traction_Update()` aplica PWM uniforme a todas las ruedas. `Steering_ControlLoop()` no diferencia ángulos inner/outer. | **MEDIO** — Sin Ackermann, las ruedas interiores y exteriores reciben mismo ángulo. En un vehículo real con dirección mecánica de un solo motor, esto podría no ser un problema (el mecanismo físico hace el Ackermann). Pero para tracción diferencial por software sí falta. |
| **Encoder Index Pulse (Z)** | `main.h:51` | `PIN_ENC_Z` definido en PB4 como EXTI4, pero **no hay GPIO init**, **no hay EXTI handler**, **no se usa para calibración**. El encoder E6B2-CWZ6C tiene pulso Z para referencia absoluta. | **MEDIO** — Sin pulso Z no hay referencia absoluta de posición. La "calibración" actual (`Steering_Init`) simplemente pone el contador a 0 en la posición donde esté al arrancar. |
| **Error MOTOR_STALL** | `safety_system.h:26` | `SAFETY_ERROR_MOTOR_STALL = 5` definido en el enum pero **ninguna función lo detecta ni lo activa**. No hay detección de motor bloqueado (corriente alta + velocidad cero). | **ALTO** — Un motor bloqueado con corriente aplicada puede sobrecalentarse. La protección por sobrecorriente (25A) podría mitigarlo parcialmente, pero no hay detección específica de stall. |
| **Error EMERGENCY_STOP** | `safety_system.h:27` | `SAFETY_ERROR_EMERGENCY_STOP = 6` definido pero `Safety_EmergencyStop()` en `safety_system.c:390` va directo a `SYS_STATE_ERROR` sin llamar a `Safety_SetError(SAFETY_ERROR_EMERGENCY_STOP)`. | **BAJO** — Funcional pero sin trazabilidad del tipo de error. |
| **Error WATCHDOG** | `safety_system.h:28` | `SAFETY_ERROR_WATCHDOG = 7` definido pero **nunca activado**. El IWDG hace reset de hardware, no pasa por la máquina de estados. | **BAJO** — Es correcto que IWDG cause reset directo. El código de error queda sin uso. |
| **CAN_SendError()** | `can_handler.c:218-225` | Función implementada (ID 0x300, 2 bytes), pero **nunca llamada** desde ningún punto del código. | **BAJO** — Los errores se transmiten via heartbeat (fault_flags), pero los mensajes de diagnóstico detallados no se envían. |
| **RPM Getters (FR/RL/RR)** | `sensor_manager.c:64` | Solo `Wheel_GetRPM_FL()` tiene getter público. `wheel_rpm[1-3]` se calculan internamente pero **no tienen función de acceso**. | **MUY BAJO** — Solo FL RPM es accesible. Las demás ruedas tienen RPM calculado pero inaccesible desde fuera del módulo. |
| **Voltage_GetBus()** | `sensor_manager.c:149-152` | Función implementada y funcional pero **nunca llamada** desde ningún módulo. `voltage_bus[]` se lee de INA226 pero no se usa ni se transmite por CAN. | **BAJO** — Dato disponible pero no visible externamente. |
| **Secuencia Relés Bloqueante** | `safety_system.c:119-128` | `Relay_PowerUp()` usa `HAL_Delay()` (50ms + 20ms = 70ms). Esto **bloquea el bucle principal** incluyendo el watchdog IWDG. Con un timeout de 500 ms esto no es un problema inmediato, pero si los delays aumentan sí lo sería. | **MEDIO** — Funcional pero no óptimo. Bloquea el sistema 70 ms durante transición STANDBY→ACTIVE. |

---

## TABLA 3 – NO IMPLEMENTADO (aunque parezca que sí)

| Sistema | Motivo | Evidencia |
|---------|--------|-----------|
| **Integración Ackermann en tracción** | `Ackermann_Compute()` existe en `motor_control.c:257` con fórmula correcta (wheelbase=0.95m, track=0.70m, max inner=54°), pero **no es invocada** por `Traction_Update()`, `Steering_ControlLoop()`, ni `main.c`. Los 4 motores reciben **exactamente el mismo PWM**. | `grep -rn "Ackermann_Compute" Core/` → solo declaración y definición, ninguna llamada. |
| **ABS per-wheel modulation** | ABS detecta slip por rueda individual (`abs_wheel_mask`), pero la intervención corta throttle **global** a 0%: `Traction_SetDemand(0)` en `safety_system.c:244`. No hay modulación individual de PWM por rueda bloqueada. | `safety_system.c:244` — siempre `Traction_SetDemand(0)` sin usar el bitmask para intervención diferencial. |
| **TCS per-wheel modulation** | TCS detecta slip por rueda individual (`tcs_wheel_mask`), pero reduce throttle **global** 50%: `Traction_SetDemand(Pedal_GetPercent() / 2.0f)` en `safety_system.c:283`. No hay reducción individual por rueda con tracción perdida. | `safety_system.c:283` — intervención global sin usar bitmask. |
| **Calibración real de dirección** | `Steering_Init()` en `motor_control.c:100-108` simplemente pone el contador TIM2 a 0 y marca `steering_calibrated = 1`. **No hay secuencia de búsqueda de límites, ni uso del pulso Z del encoder, ni verificación de rango.** | `motor_control.c:106-107` — `__HAL_TIM_SET_COUNTER(&htim2, 0); steering_calibrated = 1;` |
| **Protección ante fallo de encoder** | Si el encoder falla (cable roto, señal perdida), el PID seguiría operando con un valor estático de TIM2. **No hay detección de encoder parado, timeout, ni señal de fallo.** | No existe ningún chequeo de "encoder alive" en todo el código. `Safety_CheckSensors()` no verifica el encoder. |
| **Filtrado/Antirrebote pedal** | El ADC lee un valor crudo y lo escala directamente: `pedal_pct = pedal_raw * 100.0f / 4095.0f`. **No hay filtro paso bajo, media móvil, zona muerta, ni histéresis.** | `sensor_manager.c:82` — conversión directa sin procesamiento. |
| **Debounce sensores de rueda** | Las EXTI disparan en flanco de subida (`GPIO_MODE_IT_RISING`) y cada ISR incrementa el contador inmediatamente. **No hay filtrado por tiempo mínimo entre pulsos, ni detección de rebote.** | `sensor_manager.c:28-31` — `wheel_pulse[n]++` directamente en ISR sin ningún filtro temporal. |
| **Rampas de aceleración/frenado** | `Traction_SetDemand()` acepta el valor directamente (clamped a ±100%). **No hay rampa de subida ni bajada**. Un cambio de 0% a 100% se aplica instantáneamente al siguiente `Traction_Update()`. | `motor_control.c:114-119` — clamp directo sin rampa. |
| **Calibración ADC** | `HAL_ADCEx_Calibration_Start()` **no se llama** antes de empezar conversiones. El STM32G4 recomienda calibración antes del primer uso para máxima precisión. | Ausente en `MX_ADC1_Init()` (`main.c:350-369`) y `Sensor_Init()`. |
| **Derating por temperatura** | La protección por temperatura es binaria: por debajo de 90°C todo normal, por encima → SAFE (corte total). **No hay reducción gradual** de potencia entre, por ejemplo, 70°C y 90°C. | `safety_system.c:311-316` — solo `if (t > 90) → SAFE`. |
| **Detección stall de motor** | No hay lógica que detecte corriente alta + velocidad cero simultánea (motor bloqueado). El error `SAFETY_ERROR_MOTOR_STALL` existe como enum pero **nunca se genera**. | `SAFETY_ERROR_MOTOR_STALL` no aparece en ningún `Safety_SetError()` de todo el código. |
| **Regenerative braking** | No existe código de frenado regenerativo. Los motores se cortan a PWM 0 y enable LOW. | Ausente en todo el proyecto. |
| **Limp mode** | No existe un modo de operación degradada. De SAFE se pasa a ACTIVE completo o no se pasa. | Solo estados BOOT/STANDBY/ACTIVE/SAFE/ERROR implementados. |
| **I2C bus recovery** | Si el bus I2C queda bloqueado (SDA LOW permanente), no hay secuencia de recovery (clock pulses). `TCA9548A_SelectChannel()` falla silenciosamente (pone 0.0A). | `sensor_manager.c:126-130` — en error simplemente pone 0, sin recovery. |
| **DMA para sensores** | Toda la lectura de ADC e I2C es bloqueante (polling). No se usa DMA en ningún periférico. | `HAL_ADC_PollForConversion()` y `HAL_I2C_Mem_Read()` con timeout en `sensor_manager.c`. |
| **WWDG** | No configurado. Solo IWDG activo. | Ausente en `main.c` y `stm32g4xx_hal_conf.h` (HAL_WWDG_MODULE_ENABLED no definido). |
| **Persistent logging** | No hay escritura a Flash ni EEPROM de errores o eventos. Al hacer reset se pierde toda la información. | Ausente en todo el proyecto. |
| **Battery monitoring** | No hay lectura de tensión de batería directa. Solo `Voltage_GetBus()` por INA226 (tensión del bus del motor, no batería). | `Voltage_GetBus()` existe pero no se usa, y no es tensión de batería. |

---

## TABLA 4 – RESUMEN DE CONFIANZA

### ¿Puede el vehículo moverse de forma segura?

**SÍ, con reservas.** La cadena completa funciona:

```
ESP32 → CAN 0x100 (throttle %) → Safety_ValidateThrottle() → Traction_SetDemand() → Traction_Update() → PWM a motores
```

Pero hay **puntos débiles reales**:
- El pedal local (ADC) no tiene **filtrado** — un pico de ruido en PA3 se traduce directamente en PWM
- Los sensores de rueda no tienen **debounce** — pulsos espurios incrementan el contador
- No hay **rampas de aceleración**, un salto de 0→100% es instantáneo
- El encoder de dirección no tiene **protección ante fallo**: si se desconecta, el PID opera con un valor fijo sin detectar el problema

La máquina de estados es sólida: solo en ACTIVE se permiten comandos. Los fault handlers (HardFault, etc.) cortan todo vía registro BSRR directo. El IWDG de 500 ms reinicia si el bucle principal se bloquea.

### ¿La tracción inteligente está completa o parcial?

**PARCIAL.**

| Subsistema | Detección | Intervención | Conclusión |
|------------|-----------|--------------|------------|
| 4x2 / 4x4 | N/A | ✅ Diferencial por modo | **Completo** |
| Tank turn | N/A | ✅ Inversión izquierda/derecha | **Completo** |
| ABS | ✅ Per-wheel (bitmask) | ❌ Global (corta todo) | **Parcial** — debería modular por rueda |
| TCS | ✅ Per-wheel (bitmask) | ❌ Global (reduce 50% todo) | **Parcial** — debería modular por rueda |
| Ackermann | ✅ Cálculo implementado | ❌ No integrado en la cadena | **No activo** |

### ¿El pedal y la dirección están correctamente validados?

**Pedal**: **Parcialmente.** Pasa por `Safety_ValidateThrottle()` (clamp + ABS/TCS override) pero le falta filtrado de ruido ADC, zona muerta, y rampa de aceleración.

**Dirección**: **Sí, razonablemente.** Tiene PID con anti-windup, rate-limit de 200°/s, clamp ±45°, y validación por estado ACTIVE. Le falta protección ante fallo de encoder y calibración real con pulso Z.

### ¿Hay algún punto crítico oculto?

| # | Punto Crítico | Severidad | Detalle |
|---|---------------|-----------|---------|
| 1 | **Encoder sin detección de fallo** | 🔴 ALTA | Si el cable del encoder se corta, `__HAL_TIM_GET_COUNTER(&htim2)` devuelve el último valor. El PID cree que el motor está en posición y no aplica corrección. La dirección física se mueve sin retroalimentación. |
| 2 | **Motor stall sin detección** | 🔴 ALTA | Un motor bloqueado mecánicamente consume corriente máxima. La protección de 25A puede tardar en activar si el arranque es gradual. No hay detección corriente+velocidad=0. |
| 3 | **Pedal sin filtrado** | 🟡 MEDIA | Ruido en PA3 se traduce directamente en demanda. En un entorno con motores e inversores, el ruido EMI puede causar picos de throttle falsos. |
| 4 | **ABS/TCS intervención global** | 🟡 MEDIA | Si una rueda pierde tracción, se corta TODA la potencia en vez de solo esa rueda. Esto puede causar pérdida total de propulsión innecesaria. |
| 5 | **Relay PowerUp bloqueante** | 🟡 MEDIA | 70 ms de `HAL_Delay()` durante los cuales no se ejecuta el bucle principal, no se refrescan sensores, y no se procesan mensajes CAN. El IWDG de 500 ms da margen suficiente, pero es un patrón frágil. |
| 6 | **EXTI15_10 compartida** | 🟢 BAJA | PIN_WHEEL_RR está en PB15 (EXTI15, handler compartido EXTI15_10). Si se añade otro pin en EXTI10-14, el handler actual llamaría a `Wheel_RR_IRQHandler()` espuriamente. Actualmente no hay conflicto. |
| 7 | **OneWire busy-wait** | 🟢 BAJA | `OW_DelayUs()` en `sensor_manager.c:190-195` usa bucle de NOPs calibrado a ~170 MHz. Si cambia la frecuencia del reloj, los timings de OneWire se desajustan. En la práctica el reloj es fijo a 170 MHz. |

---

## APÉNDICE A – Mapa completo de periféricos

```
┌─────────────┬──────────┬────────────────────┬─────────────────────────────┐
│ Periférico  │ Pin(es)  │ Config             │ Usado en                    │
├─────────────┼──────────┼────────────────────┼─────────────────────────────┤
│ TIM1 CH1    │ PA8      │ PWM 20 kHz         │ Motor FL                    │
│ TIM1 CH2    │ PA9      │ PWM 20 kHz         │ Motor FR                    │
│ TIM1 CH3    │ PA10     │ PWM 20 kHz         │ Motor RL                    │
│ TIM1 CH4    │ PA11     │ PWM 20 kHz         │ Motor RR                    │
│ TIM8 CH3    │ PC8      │ PWM 20 kHz         │ Motor Steering              │
│ TIM2 CH1/2  │ PA15/PB3 │ Encoder quad TI12  │ Steering angle              │
│ ADC1 CH4    │ PA3      │ 12-bit single conv │ Pedal                       │
│ FDCAN1      │ PB8/PB9  │ 500 kbps classic   │ ESP32 communication         │
│ I2C1        │ PB6/PB7  │ 400 kHz fast mode  │ INA226 / TCA9548A           │
│ IWDG        │ —        │ ~500 ms            │ Main loop watchdog          │
│ GPIOC 0-4   │ PC0-PC4  │ Output PP          │ Direction FL/FR/RL/RR/STEER │
│ GPIOC 5-7,9 │ PC5-7,9  │ Output PP          │ Enable FL/FR/RL/RR/STEER    │
│ GPIOC 13    │ PC13     │ Output PP          │ Enable RR                   │
│ GPIOC 10-12 │ PC10-12  │ Output PP          │ Relay MAIN/TRAC/DIR         │
│ GPIOA 0-2   │ PA0-PA2  │ EXTI Rising+PU     │ Wheel speed FL/FR/RL        │
│ GPIOB 15    │ PB15     │ EXTI Rising+PU     │ Wheel speed RR              │
│ GPIOB 0     │ PB0      │ OD / Input         │ OneWire DS18B20 bus         │
│ PB4         │ PB4      │ ⚠️ NO CONFIGURADO  │ Encoder Z (definido, no usado) │
└─────────────┴──────────┴────────────────────┴─────────────────────────────┘
```

## APÉNDICE B – Mapa completo de mensajes CAN

```
┌──────┬──────────────────────┬─────────┬──────┬──────────────────────────────────────────┐
│ ID   │ Nombre               │ Dir     │ Bytes│ Contenido                                │
├──────┼──────────────────────┼─────────┼──────┼──────────────────────────────────────────┤
│ 0x001│ Heartbeat STM32      │ STM→ESP │  4   │ counter, state, fault_flags, reserved    │
│ 0x011│ Heartbeat ESP32      │ ESP→STM │  —   │ Presencia (refresca CAN timeout)         │
│ 0x100│ Cmd Throttle         │ ESP→STM │  1   │ throttle_pct (0-100)                     │
│ 0x101│ Cmd Steering         │ ESP→STM │  2   │ angle_raw (int16, ×0.1°)                 │
│ 0x102│ Cmd Mode             │ ESP→STM │  1   │ bit0=4x4, bit1=tank_turn                 │
│ 0x200│ Status Speed         │ STM→ESP │  8   │ FL/FR/RL/RR ×10 km/h (uint16 LE cada)   │
│ 0x201│ Status Current       │ STM→ESP │  8   │ FL/FR/RL/RR ×100 A (uint16 LE cada)      │
│ 0x202│ Status Temperature   │ STM→ESP │  5   │ 5× int8 °C                               │
│ 0x203│ Status Safety        │ STM→ESP │  3   │ abs, tcs, error_code                     │
│ 0x204│ Status Steering      │ STM→ESP │  3   │ angle ×10° (int16 LE) + calibrated       │
│ 0x300│ Diag Error           │ Ambos   │  2   │ error_code, subsystem ⚠️ NUNCA ENVIADO   │
└──────┴──────────────────────┴─────────┴──────┴──────────────────────────────────────────┘
```

---

## APÉNDICE C – Flujo completo de un comando de throttle

```
ESP32 envía CAN 0x100 [throttle_pct]
       │
       ▼
FDCAN1 RX FIFO0 → HAL_FDCAN_RxFifo0Callback()
       │                    │
       │                    └─→ Safety_UpdateCANRxTime()  [refresca watchdog CAN]
       ▼
CAN_ProcessMessages() [en main loop]
       │
       ▼
case CAN_ID_CMD_THROTTLE:
       │
       ▼
Safety_ValidateThrottle(requested_pct)
       │
       ├─ ¿Estado == ACTIVE? No → return 0.0
       ├─ Clamp [0, 100]
       ├─ ¿ABS activo? → return 0.0
       ├─ ¿TCS activo? → return pct * 0.5
       └─ return pct
       │
       ▼
Traction_SetDemand(validated_pct)
       │
       ▼
Traction_Update() [cada 10 ms]
       │
       ├─ pwm = |demand| * 8499 / 100
       ├─ dir = demand >= 0 ? 1 : -1
       │
       ├─ ¿Tank turn? → izquierda inversa, derecha normal
       ├─ ¿4x4? → 4 ruedas mismo PWM
       └─ ¿4x2? → solo FL/FR, RL/RR deshabilitados
              │
              ▼
       __HAL_TIM_SET_COMPARE(TIM1, CHx, pwm)  →  señal PWM a driver de motor
```

---

*Auditoría generada por análisis línea por línea del código fuente. Toda función y archivo citado es verificable directamente en el repositorio.*
