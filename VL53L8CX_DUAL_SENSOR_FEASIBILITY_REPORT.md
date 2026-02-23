# Informe Técnico de Viabilidad — Segundo Sensor Frontal VL53L8CX en STM32G474

**Fecha:** 2026-02-23  
**Firmware base:** `florinzgz/STM32-Control-Coche-Marcos` (rama principal)  
**Tipo:** Solo informe técnico — sin modificación de código ni diseño de implementación  
**Autor:** Análisis arquitectónico de hardware + firmware  

---

## Resumen Ejecutivo

La adición de un segundo sensor frontal VL53L8CX conectado directamente a la STM32G474RE es
**viable con complejidad media-alta**. El MCU dispone de pines y periféricos libres, pero el sensor
compartiría el bus I2C1 ya ocupado por TCA9548A/INA226/ADS1115, lo que introduce riesgos de
contención de bus. La implicación arquitectónica más relevante es que el STM32 pasaría de recibir
datos de obstáculos de forma advisory via CAN (desde el ESP32) a leer directamente un sensor local,
lo que cambia el rol del módulo de seguridad y crea un nuevo modo de fallo: el desacuerdo entre
ambos sensores frontales.

---

## 1. Hardware

### 1.1 Mapa de pines usado actualmente (STM32G474RE — LQFP64)

| Puerto | Pin | Función actual | Periférico |
|--------|-----|----------------|------------|
| PA0 | GPIO_PIN_0 | Sensor velocidad rueda FL | EXTI0 |
| PA1 | GPIO_PIN_1 | Sensor velocidad rueda FR | EXTI1 |
| PA2 | GPIO_PIN_2 | Sensor velocidad rueda RL | EXTI2 |
| PA3 | GPIO_PIN_3 | Pedal acelerador (canal primario) | ADC1_IN4 |
| PA8 | GPIO_PIN_8 | PWM motor FL | TIM1_CH1 |
| PA9 | GPIO_PIN_9 | PWM motor FR | TIM1_CH2 |
| PA10 | GPIO_PIN_10 | PWM motor RL | TIM1_CH3 |
| PA11 | GPIO_PIN_11 | PWM motor RR | TIM1_CH4 |
| PA15 | GPIO_PIN_15 | Encoder dirección canal A | TIM2_CH1 |
| PB0 | GPIO_PIN_0 | Bus OneWire DS18B20 | GPIO bit-bang |
| PB3 | GPIO_PIN_3 | Encoder dirección canal B | TIM2_CH2 |
| PB4 | GPIO_PIN_4 | Encoder dirección índice Z | EXTI4 |
| PB5 | GPIO_PIN_5 | Sensor inductivo centro dirección | EXTI5 |
| PB6 | GPIO_PIN_6 | I2C1 reloj (SCL) | I2C1_SCL |
| PB7 | GPIO_PIN_7 | I2C1 datos (SDA) | I2C1_SDA |
| PB8 | GPIO_PIN_8 | CAN bus RX | FDCAN1_RX |
| PB9 | GPIO_PIN_9 | CAN bus TX | FDCAN1_TX |
| PB10 | GPIO_PIN_10 | Relay LED tiras WS2812B | GPIO salida |
| PB15 | GPIO_PIN_15 | Sensor velocidad rueda RR | EXTI15 |
| PC0–PC4 | — | Señales de dirección motores FL/FR/RL/RR/Steer | GPIO salidas |
| PC5–PC7 | — | Señales Enable FL/FR/RL | GPIO salidas |
| PC8 | GPIO_PIN_8 | PWM motor dirección | TIM8_CH3 |
| PC9 | GPIO_PIN_9 | Enable motor dirección | GPIO salida |
| PC10–PC12 | — | Relays Main/Traction/Direction | GPIO salidas |
| PC13 | GPIO_PIN_13 | Enable motor RR | GPIO salida |

**Pines libres identificados** (sin función asignada en el firmware actual):

| Puerto | Pines libres |
|--------|-------------|
| PA | PA4, PA5, PA6, PA7, PA12 |
| PB | PB1, PB2, PB11, PB12, PB13, PB14 |
| PC | PC14, PC15 (generalmente reservados para oscilador externo) |

**Conclusión:** Hay suficientes pines GPIO libres para implementar el VL53L8CX (mínimo 2 pines:
SDA/SCL si comparte bus I2C1; o 4 pines si se usa I2C2 dedicado + XSHUT + INT).

---

### 1.2 Periférico I2C recomendado

El VL53L8CX utiliza I2C hasta 1 MHz (Fast Mode Plus). Se presentan dos opciones:

#### Opción A — Compartir bus I2C1 existente (PB6/PB7, 400 kHz)

El bus I2C1 actual ya gestiona tres dispositivos:
- TCA9548A multiplexor I2C (dirección `0x70`)
- 6× INA226 sensores de corriente (dirección `0x40`, acceso a través de TCA9548A)
- ADS1115 ADC de pedal (dirección `0x48`)

El VL53L8CX tiene dirección I2C por defecto `0x52`, que **no colisiona** con ningún dispositivo
existente. Pines necesarios: ninguno nuevo (solo XSHUT y opcionalmente INT en GPIO libre).

**Riesgo principal:** contención del bus. El ciclo de lectura de INA226 ya ocupa varias
transacciones I2C en el slot de 50 ms, y el ADS1115 introduce un bloqueo de 8 ms dentro de
`Pedal_ReadADS1115()`. La lectura del VL53L8CX transfiere entre 256 y ~2.500 bytes por ciclo
(dependiendo de la configuración del driver ST). A 400 kHz: 256 bytes × 9 bits / 400 kHz ≈ 5,76 ms
de bus ocupado. Si se usa DMA (recomendado), la CPU queda libre durante la transferencia, pero el
bus físico sigue bloqueado ese tiempo, potencialmente demorando otros dispositivos I2C.

La función `I2C_BusRecovery()` existente (`sensor_manager.c`) también se vería afectada: si el
VL53L8CX retiene SDA durante una transferencia larga y ocurre un fallo, el algoritmo de
recuperación (16 pulsos SCL, basado en NXP AN10216) debe ejecutarse coordinando los tres tipos de
dispositivos en el mismo bus.

#### Opción B — I2C2 dedicado

La STM32G474RE (RM0440) expone I2C2 en los pines `PF0/PF1` (no disponibles en LQFP64) y en pines
alternativos que en el paquete LQFP64 pueden incluir `PB11` (AF4 = I2C2_SDA) y `PB12` (requiere
verificación en tabla de funciones alternas del RM0440 para el paquete específico). En el caso de
que `PB11/PB12` o `PB13/PB14` estén disponibles con AF I2C2, se tendría un bus independiente para
el sensor, eliminando la contención.

**Ventaja:** bus dedicado, sin interferencia con INA226/ADS1115.  
**Riesgo:** requiere verificación rigurosa de la tabla de funciones alternas del LQFP64 antes de
reservar esos pines. Un error en el mapeo de AF forzaría rediseño de PCB.

#### Pines adicionales para VL53L8CX

| Señal | Dirección | Pin sugerido | Descripción |
|-------|-----------|-------------|-------------|
| XSHUT | Salida STM32 | PB1 (libre) | Apagado/reset activo bajo. Necesario si se quiere reprogramar la dirección I2C |
| INT | Entrada STM32 | PB2 (libre) | Interrupción de dato listo. Recomienda EXTI libre (ej. EXTI2 ya usado en PA2; usar EXTI line 1/2 desde PB requiere colisión; usar PB13 → EXTI13 libre) |

**Líneas EXTI disponibles** (no usadas actualmente): EXTI3, EXTI6, EXTI7, EXTI8, EXTI9, EXTI10,
EXTI11, EXTI12, EXTI13, EXTI14. Se recomienda usar una de ellas para la señal INT del VL53L8CX.

---

### 1.3 Conflictos posibles con periféricos actuales

| Periférico | Conflicto | Nivel | Detalle |
|------------|-----------|-------|---------|
| I2C1 (TCA9548A / INA226 / ADS1115) | **Moderado** | Bus compartido | Contención de bus si el VL53L8CX ocupa el bus durante lecturas simultáneas de corriente o pedal |
| EXTI (sensores velocidad rueda, encoder) | **Bajo** | Líneas libres disponibles | No hay colisión si se elige línea EXTI no usada |
| DMA (no usado actualmente explícitamente) | **Bajo** | DMA disponible | DMA1/2 del STM32G474RE tiene múltiples canales libres; I2C1_RX/TX pueden mapearse |
| IWDG (500 ms timeout) | **Bajo-Moderado** | Tiempo de ciclo | Si la inicialización del VL53L8CX bloquea el bus I2C más de ~500 ms, se dispararía el watchdog. La inicialización del driver VL53L8CX (ST ULD) puede tardar cientos de ms |
| TIM1/TIM2/TIM8 (PWM, encoder) | **Ninguno** | — | No comparten pines ni recursos con I2C |
| FDCAN1 (CAN bus) | **Ninguno** | — | Bus físico independiente |
| ADC1 (pedal) | **Ninguno** | — | Canal único, independiente |
| OneWire DS18B20 (PB0, bit-bang) | **Ninguno** | — | Software bit-bang, no usa I2C |

**Riesgo de watchdog durante inicialización:** La librería ULD (User Library Driver) de ST para
VL53L8CX incluye secuencias de firmware upload al sensor que pueden tardar entre 100 ms y 400 ms.
Con IWDG a 500 ms, hay un margen reducido. Sería necesario refrescar el watchdog durante la
inicialización del sensor (`HAL_IWDG_Refresh()` intercalado), técnica ya no compatible con el
modo actual de inicialización secuencial en `main()`.

---

### 1.4 Consumo CPU estimado y carga de interrupciones

| Operación | Frecuencia | Tiempo estimado (DMA) | Carga CPU |
|-----------|------------|----------------------|-----------|
| EXTI INT (dato listo) | 15 Hz (66 ms) | < 1 µs (solo setea flag) | Insignificante |
| I2C DMA transfer (256 B @ 400 kHz) | 15 Hz | ~6 ms bus, CPU libre | < 1 % CPU (solo ISR DMA finalización) |
| Procesamiento distancias 8×8 | 15 Hz | ~50–200 µs (media de 64 zonas) | < 1 % CPU |
| Total adicional estimado | — | — | **~1–2 % CPU adicional** |

Con DMA la carga CPU es mínima. Sin DMA (polling bloqueante), el slot de 50 ms se vería afectado
con bloqueos de ~6 ms, impactando `Current_ReadAll()` y `Pedal_ReadADS1115()`.

---

## 2. Arquitectura de Seguridad

### 2.1 Escenario con dos sensores frontales

La configuración resultante sería:

| Sensor | Tipo | Conexión | Autoridad |
|--------|------|----------|-----------|
| TOFSense-M S (existente) | LiDAR 8×8 UART 921.600 bps | ESP32 → CAN 0x208/0x209 → STM32 | Advisory (CAN) |
| VL53L8CX (nuevo) | ToF multizona I2C 400 kHz–1 MHz | STM32 directo | Primario local |

Este cambio **transforma la arquitectura de seguridad**: el STM32 ya no dependería exclusivamente
de datos de obstáculos desde el ESP32 vía CAN, pasando a tener información local y directa. Esto
es arquitectónicamente positivo (elimina la dependencia del CAN para seguridad de obstáculos), pero
introduce la necesidad de gestionar dos fuentes de datos sobre el mismo espacio físico.

### 2.2 Estrategias de fusión

#### 2.2.1 Redundancia de seguridad (AND conservador)

Usar la distancia mínima de ambos sensores como distancia efectiva:

```
distancia_efectiva = min(dist_VL53L8CX, dist_TOFSense_via_CAN)
```

El sistema reacciona al sensor más pesimista. Un sensor en fallo (distancia = 0 o indefinida) no
dispara parada si el otro está sano → necesita lógica de validez de cada fuente.

**Ventaja:** Máxima seguridad, nunca ignora una amenaza.  
**Riesgo:** Un sensor con ruido genera frenadas innecesarias (falsos positivos). La vida útil
percibida del sistema se reduce.

#### 2.2.2 Plausibilidad cruzada (validación mutua)

Los dos sensores deben concordar dentro de un margen tolerado (p. ej. ±200 mm en zona de alerta).
Si concuerdan, se usa el mínimo como distancia validada. Si discrepan más allá del umbral, se
activa un modo de "desacuerdo de sensores" con escala conservadora (ej. `0.5`).

```
si |dist_A – dist_B| < UMBRAL_CONCORDANCIA:
    dist_validada = min(dist_A, dist_B)
    modo = NORMAL
si |dist_A – dist_B| >= UMBRAL_CONCORDANCIA:
    modo = SENSOR_DISAGREEMENT → escala conservadora
```

**Ventaja:** Filtra ruido de un sensor individual; el sistema solo actúa con confianza cuando
ambos coinciden. Es el enfoque utilizado ya en el firmware para el pedal dual-canal
(`Pedal_IsPlausible()` / `Pedal_IsContradictory()` en `sensor_manager.c`).  
**Riesgo:** Si los dos sensores apuntan con ángulos ligeramente distintos (FOV diferente o
desplazamiento físico), medirán distancias distintas para el mismo objeto. Requiere calibración de
alineación física y márgenes de tolerancia ajustados.

#### 2.2.3 Fusión ponderada con confianza dinámica

Asignar un peso variable a cada sensor según su calidad de señal (`signal_strength` en VL53L8CX,
`health flag` del TOFSense via 0x208 byte 3):

```
dist_fusionada = (w_A × dist_A + w_B × dist_B) / (w_A + w_B)
```

donde `w_A`, `w_B` ∈ [0.0, 1.0] se calculan a partir de la calidad de señal y el historial de
plausibilidad de cada sensor.

**Ventaja:** Aprovecha la información de los dos sensores incluso cuando uno está degradado.
Suaviza fluctuaciones de lectura.  
**Riesgo:** Complejidad de implementación alta. En escenario de emergencia (objeto a < 200 mm), la
fusión ponderada podría suavizar la respuesta al combinar una lectura crítica con una alejada.
Necesita "modo de emergencia" que anule la ponderación y tome el mínimo.

---

### 2.3 Nuevo modo de fallo: desacuerdo entre sensores

El modo de fallo más relevante que aparece con dos sensores frontales es el **desacuerdo
persistente** entre ambos. Este modo no existe en el firmware actual.

| Causa posible | Detección | Respuesta recomendada |
|---------------|-----------|----------------------|
| Sensor A bloqueado (objeto pegado al cristal) | Distancia fija mientras el vehículo se mueve (stuck detection, ya existe en firmware) | `SENSOR_FAULT` → escala 0.3 |
| Sensor B (CAN) con ruido electromagnético | Lecturas implausibles > límite de cambio 8 m/s (ya validado en `Obstacle_ProcessCAN()`) | Rechazo de lectura, retener último valor válido |
| Desalineación física de los sensores | Discrepancia sistemática > calibración | Ajuste de offset en inicialización |
| Un sensor ve objeto fuera del FOV del otro | Discrepancia legítima (objeto lateral) | No tratar como fallo; el mínimo es conservador |
| Ambos sensores fallan simultáneamente | Imposible discriminar | Máxima restricción, alerta de error `SAFETY_ERROR_OBSTACLE` |

**Nuevo código de error recomendado** (si se implementase): `SAFETY_ERROR_OBSTACLE_DISAGREEMENT`
entre `SAFETY_ERROR_OBSTACLE` (código 12) y `SAFETY_ERROR_CAN_BUSOFF` (código 13) en el enum
`Safety_Error_t` de `safety_system.h`.

---

### 2.4 Impacto en los estados SAFE / DEGRADED / LIMP_HOME

| Estado | Comportamiento actual (1 sensor advisory) | Con 2 sensores (local + advisory) |
|--------|-------------------------------------------|------------------------------------|
| `SYS_STATE_ACTIVE` | Datos CAN 0x208; `obstacle_scale` = 0.0–1.0 | Sensor local VL53L8CX como fuente primaria; CAN como secundario de validación |
| `SYS_STATE_DEGRADED` | `obstacle_scale` aplicada con límites de potencia L1-L3 | Ídem; desacuerdo de sensores activa `DEGRADED_L2` o `DEGRADED_L3` según persistencia |
| `SYS_STATE_LIMP_HOME` | CAN perdido → `obstacle_scale = 1.0` (comentario: "LIMP_HOME speed cap provides safety") | Con sensor local: obstáculo sigue detectándose aunque se pierda CAN; el LIMP_HOME sigue siendo válido pero `obstacle_scale` ya no sería 1.0 incondicional |
| `SYS_STATE_SAFE` | Activado por CAN timeout activo con obstacle bloqueado | Activado por VL53L8CX local si distancia < umbral de emergencia sin recuperación |
| `SYS_STATE_ERROR` | Fallos irrecuperables (watchdog, emergency stop) | Ídem; la falla de inicialización del VL53L8CX podría añadir una ruta adicional si se clasifica como CRITICAL |

**Cambio más importante:** en `LIMP_HOME`, el firmware actual establece `obstacle_scale = 1.0`
cuando se pierde la comunicación CAN (ver `Obstacle_Update()` en `safety_system.c`, líneas
1587–1589). Si el VL53L8CX está conectado directamente al STM32, el obstáculo seguiría siendo
visible en LIMP_HOME, mejorando sustancialmente la seguridad de este estado. Esta es la ganancia
arquitectónica más significativa de la propuesta.

---

## 3. Integración con Firmware Existente

### 3.1 Módulos afectados (solo identificación de archivos y funciones)

| Archivo | Función/Sección | Tipo de afectación |
|---------|-----------------|-------------------|
| `Core/Src/safety_system.c` | `Obstacle_Update()`, `Obstacle_ProcessCAN()`, `Safety_Init()` | Principal: nueva fuente de datos local; fusión de distancias; nuevo estado `SENSOR_DISAGREEMENT` |
| `Core/Inc/safety_system.h` | `Safety_Error_t`, `ObstacleState_t`, `SafetyStatus_t` | Nuevos códigos de error; nuevo estado de máquina de obstáculos; posible campo `obstacle_scale_local` en `SafetyStatus_t` |
| `Core/Src/sensor_manager.c` | `Sensor_Init()`, `Current_ReadAll()`, nuevo módulo `VL53L8CX_Read()` | Inicialización del sensor I2C; posible conflicto con bus I2C1 existente |
| `Core/Inc/sensor_manager.h` | Declaraciones públicas de funciones de obstáculo local | Nueva API para lectura del VL53L8CX |
| `Core/Src/main.c` | `MX_I2C1_Init()` o nueva `MX_I2C2_Init()`, slot de 10 ms (`Obstacle_Update()`), slot de 50 ms (`Sensor_Init()`) | Inicialización de periférico I2C2 (si opción B); posible slot de lectura del sensor |
| `Core/Inc/main.h` | Definiciones de pines (XSHUT, INT), handles I2C2, direcciones I2C | Nuevas constantes de hardware |
| `Core/Src/can_handler.c` | `CAN_SendStatusSafety()` o nueva función de telemetría | Si se quiere reportar estado del sensor local al ESP32 para diagnóstico en HMI |
| `Core/Inc/can_handler.h` | Nuevos IDs CAN (p. ej. `0x20B` para estado sensor local STM32) | Ampliación del protocolo CAN si se requiere telemetría |
| `Core/Src/service_mode.c` | `MODULE_OBSTACLE_DETECT` y posible `MODULE_OBSTACLE_LOCAL` | Nuevo módulo de servicio para el sensor local |

### 3.2 Cambios necesarios en el pipeline de obstáculos

El pipeline actual es lineal:

```
ESP32 sensor → CAN 0x208 → Obstacle_ProcessCAN() → obstacle_validated_mm → Obstacle_Update() → obstacle_scale → Traction_Update()
```

Con el segundo sensor, el pipeline se bifurca en la entrada:

```
ESP32 sensor → CAN 0x208 → Obstacle_ProcessCAN() ──┐
                                                     ├──► Fusion() → obstacle_validated_mm → Obstacle_Update() → obstacle_scale
VL53L8CX (I2C local) → VL53_ReadZones() ───────────┘
```

La función de fusión (`Fusion()`) sería la pieza central nueva. Dependiendo de la estrategia
elegida (sección 2.2), esta función puede ser tan simple como `min()` o tan compleja como un
filtro de Kalman simplificado.

### 3.3 Impacto en tiempos de ciclo 10 ms / 50 ms

#### Ciclo de 10 ms (100 Hz) — `ABS_Update()`, `Safety_CheckCurrent()`, `Obstacle_Update()`

`Obstacle_Update()` se llama cada 10 ms y actualmente solo consulta variables en RAM (sin I2C).
Con el sensor local, si se adopta un modelo de interrupción por INT:
- La ISR de EXTI solo setea un flag (`vl53_data_ready = 1`)
- `Obstacle_Update()` verifica el flag y procesa el último dato leído (sin bloqueo)
- La lectura real vía DMA I2C se lanzó en background al recibir la INT

Impacto neto en el ciclo de 10 ms: **< 5 µs** (solo comprobación de flag y aritmética de fusión).

#### Ciclo de 50 ms (20 Hz) — `Pedal_Update()`, `Current_ReadAll()`, `Temperature_StartConversion()`

Si el I2C1 está compartido, la lectura DMA del VL53L8CX podría solaparse con `Current_ReadAll()`
o `Pedal_ReadADS1115()`. El bus I2C es single-master; las transacciones son serie. Estimación:

| Operación I2C existente | Duración estimada |
|--------------------------|-------------------|
| `Pedal_ReadADS1115()` | ~8 ms (espera conversión ADS1115) + ~0.3 ms transferencia |
| `Current_ReadAll()` (6 canales × 2 reg) | ~3–5 ms (12 transacciones × ~0.3 ms) |
| VL53L8CX lectura (256 B DMA @ 400 kHz) | ~6 ms (bus ocupado, CPU libre) |

Total adicional en el slot de 50 ms: **~6 ms** → el slot de 50 ms tiene margen (actualmente ocupa
~13–16 ms), quedando el margen en ~21–28 ms (suficiente). Sin embargo, hay que verificar que la
suma no supere los 50 ms disponibles si se añaden otros módulos en el futuro.

Si se usa I2C2 dedicado (opción B), el impacto en el slot de 50 ms sería despreciable.

### 3.4 Impacto en protocolo CAN

El protocolo CAN actual (definido en `can_handler.h`) tiene los IDs 0x208 y 0x209 ya asignados para
datos de obstáculos del ESP32 hacia el STM32. No sería necesario cambiar estos IDs ni su protocolo.

Si se quisiera reportar al ESP32/HMI el estado del sensor local VL53L8CX (para diagnóstico en la
pantalla del ESP32), se podría añadir un ID nuevo:

| ID CAN | Dirección | Descripción | Cambio necesario |
|--------|-----------|-------------|-----------------|
| `0x20B` | STM32 → ESP32 | Estado sensor VL53L8CX local (distancia mínima, zona, health, desacuerdo) | Nuevo — opcional para diagnóstico |

La carga del bus CAN a 500 kbps no se vería afectada de forma relevante (un frame adicional cada
66 ms = ~1% de carga adicional sobre la carga ya existente estimada en ~20%).

---

## 4. Estrategias Recomendadas

### Estrategia 1 — Redundancia de seguridad pura (AND conservador)

**Descripción:** El STM32 lee el VL53L8CX local y toma el mínimo entre la distancia local y la
distancia CAN del ESP32. `obstacle_validated_mm = min(dist_local, dist_CAN)`.

**Ventajas:**
- Implementación simple (una sola línea de lógica de fusión)
- Máxima cobertura: ningún sensor puede "ocultar" una amenaza al otro
- Compatible con el estado `LIMP_HOME` (el sensor local sigue activo sin CAN)
- Hereda todo el mecanismo de validación existente en `Obstacle_ProcessCAN()` y `Obstacle_Update()`
- Arquitectura comprensible y auditable

**Riesgos:**
- Falsos positivos: si el VL53L8CX da lecturas erróneas (reflejo en suelo, polvo, luz solar directa),
  el vehículo frena innecesariamente
- El FOV del VL53L8CX (45°×45°) puede diferir del TOFSense-M S (65°), generando diferencias
  legítimas que se interpretan como amenaza del sensor más cercano
- Si el VL53L8CX falla (distancia = 0 o desconexión), `min()` aplicaría escala = 0 (parada total)
  → requiere validación de la salud de cada sensor antes de aplicar el mínimo

**Complejidad:** Baja en lógica de fusión; media en detección de validez de cada fuente.

---

### Estrategia 2 — Sensor local principal + confirmación por sensor CAN

**Descripción:** El VL53L8CX (local, STM32) actúa como sensor de referencia principal. El
TOFSense-M S vía CAN confirma o invalida las detecciones locales. La detección del sensor local
se acepta sin esperar confirmación CAN si el sensor está sano; la discrepancia CAN activa una
degradación de nivel (`DEGRADED_L1`).

```
si sensor_local_sano:
    dist_efectiva = dist_VL53
    si |dist_VL53 – dist_CAN| > UMBRAL y CAN_valido:
        → DEGRADED_L1 (desacuerdo), dist_efectiva = min(dist_VL53, dist_CAN)
si !sensor_local_sano:
    dist_efectiva = dist_CAN (modo fallback, como actualmente)
```

**Ventajas:**
- Elimina la dependencia de CAN para la funcionalidad de obstáculos
- En LIMP_HOME, el sensor local garantiza protección real (mejora sustancial sobre diseño actual)
- La lógica es familiar: sigue el patrón `pedal dual-canal` ya implementado en `sensor_manager.c`
  (canal primario ADC + canal de plausibilidad ADS1115)
- La degradación `DEGRADED_L1` por desacuerdo de sensores no inmoviliza el vehículo (coherente
  con la filosofía "never immobilize")

**Riesgos:**
- Si el sensor local VL53L8CX tiene un fallo permanente (daño físico), el sistema cae al modo
  CAN advisory que ya existía — no hay pérdida de funcionalidad respecto al diseño actual
- El nuevo código de gestión de desacuerdo debe estar bien probado; un bug podría inhibir la
  respuesta de seguridad
- La clasificación del umbral de desacuerdo requiere calibración empírica según montaje físico

**Complejidad:** Media. Requiere nueva lógica de gestión de salud del sensor local, integración
con `DegradedLevel_t` y posible nuevo `Safety_Error_t`.

---

### Estrategia 3 — Fusión inteligente con confianza dinámica

**Descripción:** Se pondera la contribución de cada sensor según su métrica de calidad. El
VL53L8CX reporta `signal_per_spad` (intensidad de señal) por zona; el TOFSense-M S reporta
`health` flag y `zone confidence` en su trama UART (accesible vía CAN). Se construye un índice de
confianza [0.0–1.0] para cada sensor, y la distancia efectiva se calcula como:

```
w_local = f(signal_strength_VL53, stuck_flag, plausibility_local)
w_CAN   = f(health_flag_CAN, stale_count, plausibility_CAN)
dist_efectiva = (w_local × dist_local + w_CAN × dist_CAN) / (w_local + w_CAN)
```

En zona de emergencia (< 200 mm) se ignora la ponderación y se toma el mínimo.

**Ventajas:**
- Aprovecha toda la información disponible; la fusión es más robusta ante degradaciones parciales
- Reduce falsos positivos comparado con la estrategia AND pura
- Se puede adaptar dinámicamente a condiciones ambientales (polvo, lluvia)

**Riesgos:**
- Mayor complejidad de implementación y verificación
- En zonas de emergencia, la fusión ponderada podría suavizar peligrosamente una lectura crítica
  si el sensor de mayor peso reporta una distancia segura — requiere overrides por zona hardcodeados
- El cálculo de peso dinámico añade ~200–500 µs por ciclo de 10 ms si se hace por zona
- Difícil de auditar y validar en un sistema de seguridad funcional (traza a base firmware difícil)
- Clasificación como ASIL requeriría análisis FMEA adicional sobre el algoritmo de fusión

**Complejidad:** Alta. No recomendado para la primera iteración en un sistema de seguridad para
vehículo de niños.

---

## 5. Síntesis y Recomendación de Arquitectura

| Criterio | Estrategia 1 (AND) | Estrategia 2 (Principal+Confirm) | Estrategia 3 (Fusión) |
|----------|--------------------|----------------------------------|-----------------------|
| Seguridad | ★★★★★ | ★★★★☆ | ★★★☆☆ |
| Robustez ante fallos | ★★★☆☆ | ★★★★★ | ★★★★☆ |
| Simplicidad | ★★★★★ | ★★★★☆ | ★★☆☆☆ |
| Falsos positivos | ★★☆☆☆ | ★★★★☆ | ★★★★★ |
| Mejora en LIMP_HOME | ★★★★★ | ★★★★★ | ★★★★☆ |
| Complejidad firmware | Baja | Media | Alta |

**Recomendación:** **Estrategia 2** (Sensor local principal + confirmación CAN) como equilibrio
óptimo. Sigue el patrón arquitectónico ya establecido en el firmware para el pedal dual-canal,
es auditable, mejora LIMP_HOME, y no inmoviliza el vehículo ante fallo del sensor secundario.

La Estrategia 1 es viable como primera versión de seguridad mínima viable (MVP), con la condición
de que se añada validación de salud de cada sensor antes de aplicar el `min()`.

La Estrategia 3 queda reservada para una iteración futura si se dispone de recursos de validación
y se requiere certificación de seguridad funcional.

---

## 6. Precondiciones para la Implementación

Los siguientes puntos deben resolverse antes de iniciar la implementación:

1. **Verificación de pines I2C2 en LQFP64:** Consultar la tabla de funciones alternas (RM0440,
   Table 14) para confirmar qué pines `PB11/PB12` o `PB13/PB14` son mapeables como I2C2_SDA/SCL
   en el paquete específico. Si no son disponibles, la opción I2C compartida es la única vía sin
   cambios de PCB.

2. **Integración del driver VL53L8CX (ST ULD):** La librería ULD de ST para el VL53L8CX está
   escrita en C y es portable a bare-metal ARM Cortex-M, pero requiere provisión de funciones
   `platform_read_i2c()`, `platform_write_i2c()`, y `platform_delay_ms()`. Con IWDG activo, la
   función `platform_delay_ms()` debe refrescar el watchdog internamente.

3. **Montaje físico y alineación:** Los dos sensores (TOFSense en ESP32, VL53L8CX en STM32) deben
   apuntar al mismo espacio frontal con el menor desplazamiento angular posible para minimizar
   discrepancias legítimas. El umbral de concordancia (`UMBRAL_CONCORDANCIA`) se calibrará en
   banco de pruebas.

4. **Evaluación de carga I2C1:** Si se comparte el bus, medir empíricamente el tiempo de ciclo
   del bus I2C1 con osciloscopio antes y después de añadir el VL53L8CX para verificar que no se
   supera el slot de 50 ms ni se generan timeouts en el mecanismo de recovery existente.

---

*Fin del informe técnico.*  
*Este documento no contiene código ni especificación de implementación. Toda la información está*  
*basada en el análisis del firmware publicado en `Core/Src` y `Core/Inc` de este repositorio.*
