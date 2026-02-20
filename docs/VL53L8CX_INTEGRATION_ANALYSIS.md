# VL53L8CX — Análisis de Integración en STM32G474RE

## Resumen ejecutivo

Este documento analiza la integración de un sensor **VL53L8CX** (Time-of-Flight multizona 8×8, I2C) conectado directamente a la STM32G474RE como **sensor de seguridad primaria independiente del ESP32**. Se cubre arquitectura software, prioridad frente al sistema CAN existente, modos de fallo, estrategia de limitación de par, y análisis completo de hardware incluyendo pinout, periférico I2C y protección del bus de control del motor.

**No se incluye código** — solo razonamiento de ingeniería completo.

---

## Índice

1. [Ubicación en la arquitectura](#1-ubicación-en-la-arquitectura)
2. [Prioridad frente a obstacle_scale CAN](#2-prioridad-frente-a-obstacle_scale-can)
3. [Análisis de modos de fallo](#3-análisis-de-modos-de-fallo)
4. [Evitar falsos positivos sin perder seguridad](#4-evitar-falsos-positivos-sin-perder-seguridad)
5. [Estrategia de limitación de par](#5-estrategia-de-limitación-de-par)
6. [Garantía de movilidad sin impacto](#6-garantía-de-movilidad-sin-impacto)
7. [Necesidad de multiplexores I2C](#7-necesidad-de-multiplexores-i2c)
8. [Disponibilidad de pines en la STM32G474RE](#8-disponibilidad-de-pines-en-la-stm32g474re)
9. [Selección de periférico I2C](#9-selección-de-periférico-i2c)
10. [Asignación concreta de pines y conexión física](#10-asignación-concreta-de-pines-y-conexión-física)
11. [Protección del control de motor frente a bloqueo I2C](#11-protección-del-control-de-motor-frente-a-bloqueo-i2c)

---

## 1. Ubicación en la arquitectura

### Módulo propietario: `safety_system.c`

El VL53L8CX debe ser propiedad exclusiva de **`safety_system.c`**, no de `sensor_manager.c`.

**Razonamiento:**

| Criterio | `sensor_manager.c` | `safety_system.c` |
|----------|--------------------|--------------------|
| Rol actual | Lectura de sensores (pedal, temperatura, corriente, velocidad) | Autoridad de seguridad, estado del sistema, obstacle_scale |
| Filosofía | Adquisición de datos, conversión de unidades | Decisión de seguridad, limitación de par |
| Obstacle existente | No participa | `Obstacle_Update()`, `Obstacle_ProcessCAN()`, `Obstacle_GetScale()` |
| Acceso a estado | No decide sobre modos | Controla `SYS_STATE_*`, `DegradedLevel_t` |

El sensor VL53L8CX no es un sensor genérico de telemetría — es un **actuador de seguridad** que directamente reduce par. La cadena de decisión completa (leer → validar → filtrar → limitar) debe estar contenida dentro del módulo que ya posee esa autoridad.

**Separación interna recomendada:**

```
safety_system.c
├── Obstacle_Update()            ← ya existe, decisión final de obstacle_scale
├── Obstacle_ProcessCAN()        ← ya existe, datos del ESP32
├── VL53L8CX_ReadDistance()      ← NUEVO: lectura I2C + extracción de mínima distancia
├── VL53L8CX_Validate()          ← NUEVO: plausibilidad, stuck, rate-of-change
└── Obstacle_FuseLocal()         ← NUEVO: fusión local + CAN → obstacle_scale final
```

El driver I2C de bajo nivel del VL53L8CX (inicialización de registros, configuración de ranging, lectura de buffer) puede vivir en un archivo separado (`vl53l8cx_driver.c`) como capa HAL, pero la **decisión de seguridad** permanece en `safety_system.c`.

**Justificación adicional:** `sensor_manager.c` ya tiene la responsabilidad del bus I2C1 (TCA9548A + INA226 + ADS1115). Mezclar un segundo sensor de seguridad crítica en ese módulo crearía acoplamiento innecesario y haría que un fallo en la lectura de corriente pudiera afectar la lectura de obstáculos. Al usar un **periférico I2C separado** (ver punto 9), la independencia es total.

---

## 2. Prioridad frente a obstacle_scale CAN

### Regla fundamental: **el más restrictivo siempre gana**

```
obstacle_scale_final = min(obstacle_scale_CAN, obstacle_scale_local)
```

**Razonamiento completo:**

| Escenario | CAN (ESP32) | Local (VL53L8CX) | Resultado | Por qué |
|-----------|-------------|-------------------|-----------|---------|
| Ambos ven obstáculo | 0.3 | 0.3 | 0.3 | Concordancia — confianza alta |
| Solo CAN ve obstáculo | 0.3 | 1.0 | 0.3 | ESP32 tiene FoV más amplio, podría ver un lateral |
| Solo local ve obstáculo | 1.0 | 0.3 | 0.3 | Sensor directo tiene prioridad de proximidad |
| CAN caído, local OK | timeout → 1.0¹ | 0.5 | 0.5 | Local independiente, no necesita CAN |
| CAN caído, local fallo | timeout → 1.0¹ | fault → 0.3² | 0.3 | Modo conservador |
| Ambos OK, no hay obstáculo | 1.0 | 1.0 | 1.0 | Vía libre confirmada por ambos |

¹ El comportamiento actual de `Obstacle_Update()` cuando CAN cae sin obstáculo activo es `scale = 1.0` (movimiento permitido).
² En fallo de sensor local, se aplica escala conservadora 0.3 (ver punto 3).

**¿Por qué `min()` y no fusión ponderada?**

- Un sistema de seguridad nunca debe **promediar** fuentes — si una dice "peligro" y otra dice "seguro", la respuesta correcta es "peligro".
- La fusión ponderada introduce un escalar donde un sensor defectuoso puede "diluir" una detección real.
- `min()` es **monótono** y **fail-safe**: cualquier fuente que detecte peligro reduce el par, ninguna puede aumentarlo.

**¿Puede el sensor local invalidar una alarma CAN?**

No. El local solo puede **añadir** restricción, nunca **relajar** lo que CAN reporta. La operación `min()` garantiza esto algebraicamente.

---

## 3. Análisis de modos de fallo

### 3a. El sensor falla (no responde a I2C)

**Detección:** La lectura I2C devuelve `HAL_TIMEOUT` o `HAL_ERROR` en la transacción.

**Respuesta:**

1. Incrementar contador de fallos consecutivos (`vl53_fail_count++`)
2. Si `vl53_fail_count < 3`: mantener última lectura válida (hold-last-good), marcar `VL53_STALE`
3. Si `vl53_fail_count >= 3`: transicionar a `obstacle_scale_local = 0.3` (modo conservador, no inmovil)
4. Registrar `SAFETY_ERROR_OBSTACLE` con suberror `VL53_I2C_FAIL`
5. Si persiste > 5 s: considerar `DEGRADED_L2` (sensor de seguridad comprometido)

**Por qué 0.3 y no 0.0:**

Un sensor que falla no es un obstáculo confirmado. Cortar completamente el par por un fallo de comunicación I2C violaría el principio de que **el vehículo nunca debe quedar inmovilizado por fallo de sensor**. El valor 0.3 es coherente con el existente `OBSTACLE_SCALE_CRITICAL` (200–500 mm) y permite que el vehículo vuelva a casa a velocidad muy reducida.

**Por qué no 1.0:**

Si el sensor es la última línea de defensa (CAN caído), perder el sensor local y mantener 1.0 dejaría al vehículo completamente ciego. 0.3 es la escala de compromiso que el sistema ya usa para zona crítica.

### 3b. El sensor se congela (responde pero dato estático)

**Detección:** Mismo patrón que `Obstacle_ProcessCAN()` usa para detectar contador estancado:

```
Si vehicle_speed > 1.0 km/h AND distancia no ha cambiado en > 1000 ms → STUCK
```

Un vehículo en movimiento con un sensor de distancia que no cambia es físicamente imposible (el entorno se mueve relativo al sensor).

**Respuesta:**

1. Transicionar a `OBS_STATE_SENSOR_FAULT`
2. Aplicar `obstacle_scale_local = 0.3`
3. La fusión `min(CAN, local)` sigue operativa — si CAN ve vía libre, el resultado será 0.3 (conservador)
4. Si el vehículo está parado (`speed < 1.0 km/h`), el dato estático es **válido** (no se declara stuck)

**Caso borde:** El sensor se congela en "0 mm" (obstáculo pegado). Esto es indistinguible de un obstáculo real. La respuesta correcta es mantener `scale = 0.0` ya que no hay forma de saber si es real o falso. El conductor puede usar reversa (escape reverso ya implementado en `Traction_Update()`).

### 3c. Lecturas erróneas (valores fuera de rango)

**Detección triple:**

1. **Rango absoluto:** VL53L8CX especifica 0–4000 mm. Cualquier valor fuera → descartar.
2. **Rate-of-change:** Un obstáculo no puede acercarse más rápido que la velocidad máxima del vehículo + peatón rápido (~30 km/h = ~8.3 m/s). Ya implementado como `MAX_APPROACH_RATE_MM_PER_S = 8000` en `Obstacle_ProcessCAN()`. Aplicar la misma validación al dato local.
3. **Concordancia zonal:** El VL53L8CX da una matriz 8×8 (64 zonas). Un obstáculo real afecta **múltiples zonas adyacentes**. Un spike en una sola zona aislada es ruido y debe descartarse.

**Respuesta ante dato erróneo:**

- Dato fuera de rango → se descarta, se usa última lectura válida
- Rate-of-change violado → dato marcado como implausible, conteo de anomalías
- 3 lecturas implausibles consecutivas → `VL53_SENSOR_FAULT`, escala 0.3
- Recuperación: 5 lecturas plausibles consecutivas → volver a `VL53_NORMAL`

### 3d. La I2C se bloquea (SDA queda LOW indefinidamente)

**Detección:** Timeout de transacción I2C (`HAL_I2C_Master_Receive` retorna `HAL_TIMEOUT`).

**Respuesta en capas:**

1. **Capa 1 — Reintento:** Hasta 2 reintentos inmediatos de la transacción
2. **Capa 2 — Bus recovery (NXP AN10216):** Toggle SCL 16 veces + condición STOP forzada (ya implementado en `sensor_manager.c` para I2C1). Implementar equivalente para el periférico I2C dedicado.
3. **Capa 3 — Reset periférico:** `HAL_I2C_DeInit()` + `HAL_I2C_Init()` del periférico
4. **Capa 4 — Reset hardware:** Si GPIO de INT/XSHUT del VL53L8CX está conectado, pulsar XSHUT LOW 10 ms → HIGH para reiniciar el sensor
5. **Capa 5 — Fallback:** Si la recuperación falla tras 3 intentos, declarar sensor perdido, `obstacle_scale_local = 0.3`, notificar vía CAN (`STATUS_SAFETY 0x203`)

**Punto crítico:** Este es precisamente el motivo por el que el VL53L8CX debe estar en un **periférico I2C separado** del I2C1 (INA226/ADS1115). Un bloqueo de I2C del sensor de obstáculos no debe afectar la lectura de corriente ni la plausibilidad del pedal. Ver punto 11 para detalle.

---

## 4. Evitar falsos positivos sin perder seguridad

### Estrategia de 5 capas

**Capa 1 — Filtrado espacial (matriz 8×8):**

El VL53L8CX proporciona 64 zonas independientes. Un obstáculo real produce un **cluster** de zonas con distancias similares. Estrategia:

- Definir ROI (Region of Interest): solo las 4×4 zonas centrales (~30° FoV horizontal) son relevantes para colisión frontal
- Descartar zonas del borde (suelo, techo, laterales fuera de carril)
- Requisito de **quórum**: al menos 3 zonas adyacentes deben reportar distancia < umbral para considerar obstáculo válido
- Esto elimina: reflejos especulares, polvo, insectos, interferencia solar

**Capa 2 — Filtrado temporal (confirmación multi-frame):**

Ya implementado en `Obstacle_Update()` como `OBSTACLE_CONFIRM_TIME_MS = 200`:

- Una detección debe persistir al menos 200 ms (equivale a ~4–6 frames del VL53L8CX a 15–30 Hz) antes de activar limitación
- Esto elimina transitorios de 1–2 frames (glitches eléctricos, EMI)
- El tiempo de 200 ms es seguro porque a 5 km/h el vehículo recorre solo 28 cm, y el umbral de warning empieza a 1000 mm

**Capa 3 — Filtrado de rate-of-change:**

- Un obstáculo real tiene trayectoria continua (la distancia decrece suavemente o en escalones coherentes con la velocidad del vehículo)
- Un falso positivo aparece como un "salto" instantáneo de 2000 mm a 100 mm
- Validación: `|Δd| / Δt ≤ MAX_APPROACH_RATE` (8 m/s). Si se viola, el dato es sospechoso
- Se permite 1 violación (transición real de "no detectado" → "detectado"), pero 2 consecutivas → implausible

**Capa 4 — Concordancia con velocidad del vehículo:**

- Si el vehículo está parado (`speed < 0.5 km/h`) y el sensor reporta un obstáculo acercándose rápidamente, es plausible (peatón acercándose)
- Si el vehículo viaja a 5 km/h y el obstáculo se acerca a 15 m/s, es implausible
- Fórmula: `approach_rate_max = vehicle_speed + PEDESTRIAN_MAX_SPEED` (≈ 12 km/h total)

**Capa 5 — Histéresis de limpieza:**

Ya implementado como `OBSTACLE_CLEAR_TIME_MS = 1000`:

- Una vez activada la limitación, se necesitan 1000 ms de lecturas limpias para declarar "limpio"
- Esto previene que un obstáculo que se mueve ligeramente fuera de rango cause oscilación on/off
- 1000 ms es conservador sin ser frustrante para el conductor

### Balance seguridad vs. falsos positivos

| Parámetro | Valor | Más seguro sería... | Pero causaría... |
|-----------|-------|---------------------|-------------------|
| Quórum zonas | 3 de 16 | 1 de 16 | Falsos positivos por reflejo aislado |
| Confirm time | 200 ms | 50 ms | Activaciones por EMI transitoria |
| Clear time | 1000 ms | 5000 ms | Conductor frustrado, no puede reanudar |
| ROI | 4×4 central | 8×8 completa | Detección del suelo como obstáculo |

Estos valores son ajustables sin cambiar la arquitectura.

---

## 5. Estrategia de limitación de par

### Combinación: rampa suave + clamp mínimo + corte de emergencia

**No un solo mecanismo, sino tres coordinados:**

### Nivel 1 — Rampa suave (Warning zone: 500–1000 mm)

```
obstacle_scale = 0.7   (30% reducción)
Transición: rampa descendente a 1.0 → 0.7 en 200 ms (rate-limited)
```

**Por qué rampa y no escalón:** Un corte instantáneo del 30% del par produce una sacudida que:
- Es incómoda para el pasajero
- Podría interpretarse como fallo mecánico
- Genera picos de corriente en la fase de retroceso mecánico

La rampa de 200 ms (alineada con `OBSTACLE_CONFIRM_TIME_MS`) suaviza la transición sin comprometer la seguridad — a 5 km/h, el vehículo recorre 28 cm durante esos 200 ms, y aún está a 500–1000 mm del obstáculo.

### Nivel 2 — Torque clamp progresivo (Critical zone: 200–500 mm)

```
obstacle_scale = 0.3   (70% reducción)
Transición: rampa descendente de 0.7 → 0.3, rate = escala/velocidad
```

**Por qué clamp y no corte:** A 0.3 el vehículo aún puede moverse lentamente (~1.5 km/h con pedal al 100%). Esto es deliberado:
- Permite correcciones de posición finas (maniobra de aparcamiento)
- Permite reversa de escape si el obstáculo está al frente
- No inmoviliza en una pendiente

### Nivel 3 — Corte duro (Emergency zone: < 200 mm)

```
obstacle_scale = 0.0   (par cero)
Transición: inmediata (sin rampa)
Activación de SYS_STATE_SAFE si persiste > 2 s
```

**Por qué corte duro aquí:** A < 200 mm no hay margen para rampa. La distancia es menor que la distancia de frenado incluso a 1 km/h. El corte inmediato es la única opción segura.

**Pero con escape reverso:** El corte duro aplica solo a movimiento hacia delante. `Traction_Update()` ya implementa:
```c
if (effective_obstacle < 0.01f && current_gear == GEAR_REVERSE && Obstacle_IsForwardBlocked()) {
    effective_obstacle = 1.0f;  // Reverse escape allowed
}
```

### ¿Por qué esta combinación?

| Mecanismo solo | Problema |
|----------------|----------|
| Solo corte duro | Sacudidas, inmovilización prematura, falsos positivos paralizan el vehículo |
| Solo rampa | Demasiado lento a distancias cortas, el vehículo podría impactar antes de completar la rampa |
| Solo clamp | No detiene completamente ante colisión inminente |
| Combinación | Suave cuando hay margen, agresivo cuando no lo hay, nunca inmoviliza permanentemente |

### Integración con pipeline existente

```
FinalPWM[i] = base_pwm
              × obstacle_scale_final      ← min(CAN, local)
              × ackermann_diff[i]          ← diferencial por rueda
              × safety_status.wheel_scale[i]  ← ABS/TCS por rueda
```

La `obstacle_scale_final` sustituye al actual `safety_status.obstacle_scale` en la multiplicación. El punto de inserción no cambia — solo la fuente del valor.

---

## 6. Garantía de movilidad sin impacto

### El dilema fundamental

> "El vehículo nunca debe quedar inmovilizado, pero tampoco puede chocar."

Estos requisitos son contradictorios en el caso extremo (obstáculo real a 0 mm). La resolución es **asimétrica por dirección**:

### Regla de resolución

| Situación | Forward | Reverse | Justificación |
|-----------|---------|---------|---------------|
| Obstáculo frontal confirmado | BLOQUEADO (`scale = 0.0`) | PERMITIDO (`scale = 1.0`) | El vehículo puede alejarse del peligro |
| Sensor local fallido, CAN OK | CAN `obstacle_scale` aplica | CAN `obstacle_scale` aplica | Una fuente sigue operativa |
| Sensor local fallido, CAN caído | `scale = 0.3` (modo conservador) | `scale = 0.3` | Velocidad muy reducida pero móvil |
| Ambos sensores fallidos + parado | `scale = 0.3` | `scale = 0.3` | Se puede ir a casa a paso de tortuga |
| Ambos sensores OK, vía libre | `scale = 1.0` | `scale = 1.0` | Operación normal |

### Análisis formal de los invariantes

**Invariante 1 — No impacto:**
- Siempre que al menos un sensor funcione (local o CAN), el sistema tiene información de distancia
- La fusión `min()` garantiza que la detección más conservadora gana
- El corte duro a < 200 mm tiene margen de frenado incluso a velocidad máxima (5 km/h en LIMP_HOME)

**Invariante 2 — No inmovilización:**
- El escape reverso está siempre habilitado cuando `forward_blocked = true`
- Ningún modo de fallo de sensor produce `scale = 0.0` — el mínimo por fallo es `scale = 0.3`
- `scale = 0.0` solo se produce por **confirmación positiva de obstáculo** a < 200 mm
- LIMP_HOME (CAN perdido) preserva movilidad a 20% de par y 5 km/h

**Invariante 3 — Degradación grácil:**
```
Normal (1.0) → Warning (0.7) → Critical (0.3) → Emergency (0.0+reversa)
     ↑              ↑                ↑                    ↑
  Ambos OK     Aproximación    Zona peligrosa     Colisión inminente
                                                   (solo bloquea forward)
```

Nunca se salta directamente de 1.0 a 0.0 excepto si la primera lectura es < 200 mm (que la confirmación temporal de 200 ms valida antes de aplicar).

### Caso extremo: pendiente

Si el vehículo está en una pendiente descendente con obstáculo al frente y `scale = 0.0`:
- Forward bloqueado → el par de tracción es cero
- La gravedad tira del vehículo hacia delante
- **Solución:** El freno de estacionamiento (Gear P, implementado como `PARK_HOLD_CURRENT`) se activa automáticamente cuando `demand = 0` y `speed > 0` por más de 2 s
- El conductor puede seleccionar reversa para subir la pendiente

---

## 7. Necesidad de multiplexores I2C

### Respuesta: **NO hace falta multiplexor para el VL53L8CX**

**Análisis:**

| Criterio | ¿Necesita mux? | Detalle |
|----------|-----------------|---------|
| Conflicto de dirección | No | VL53L8CX: 0x29 (default), 0x52 (configurable). No colisiona con I2C1: TCA9548A (0x70), INA226 (0x40), ADS1115 (0x48) |
| Bus compartido | No recomendado | El VL53L8CX debe estar en un periférico I2C separado (ver punto 9) |
| Múltiples VL53L8CX | Sí, si se añaden más | Todos los VL53L8CX comparten la misma dirección base. Para 2+ sensores se necesita: (a) TCA9548A, o (b) control individual vía pin XSHUT (apagar todos, encender uno, cambiar dirección, repetir) |

### ¿Cuándo SÍ haría falta multiplexor?

1. **2+ sensores VL53L8CX:** Para cobertura frontal + trasera, o frontal + lateral. La técnica XSHUT es más eficiente que un TCA9548A para 2–3 sensores (ahorra componente, usa 1 GPIO por sensor extra).

2. **Sensor en el mismo I2C1:** Si por limitación de pines se forzara el VL53L8CX al I2C1 existente, sería **obligatorio** un TCA9548A adicional o dedicar un canal del TCA9548A existente. Pero esto no es recomendable (ver punto 11).

### Recomendación para sensor único

Un solo VL53L8CX en su propio periférico I2C no necesita multiplexor. El cableado es directo:

```
STM32 I2Cx_SCL ──── VL53L8CX SCL
STM32 I2Cx_SDA ──── VL53L8CX SDA
STM32 GPIOx    ──── VL53L8CX XSHUT  (reset hardware)
STM32 GPIOx    ──── VL53L8CX INT    (dato listo, opcional)
VCC 3.3V       ──── VL53L8CX VIN
GND            ──── VL53L8CX GND
```

### Recomendación para expansión futura (2 sensores)

```
STM32 I2Cx_SCL ──── VL53L8CX_FRONT SCL ──── VL53L8CX_REAR SCL
STM32 I2Cx_SDA ──── VL53L8CX_FRONT SDA ──── VL53L8CX_REAR SDA
STM32 GPIOa    ──── VL53L8CX_FRONT XSHUT
STM32 GPIOb    ──── VL53L8CX_REAR  XSHUT
```

Secuencia de init: XSHUT_FRONT=LOW, XSHUT_REAR=LOW → XSHUT_FRONT=HIGH, asignar addr 0x30 → XSHUT_REAR=HIGH (queda en 0x29). Ambos conviven sin multiplexor usando 2 GPIOs.

---

## 8. Disponibilidad de pines en la STM32G474RE

### Inventario de pines usados (del firmware actual)

| Puerto | Pines usados | Función |
|--------|-------------|---------|
| PA0–PA3 | 4 | Wheel speed FL/FR/RL + Pedal ADC |
| PA8–PA11 | 4 | TIM1 CH1–CH4 (PWM motores) |
| PA15 | 1 | TIM2_CH1 (encoder A) |
| PB0 | 1 | OneWire DS18B20 |
| PB3–PB5 | 3 | Encoder B, encoder Z-index, steering center |
| PB6–PB9 | 4 | I2C1_SCL, I2C1_SDA, FDCAN1_RX, FDCAN1_TX |
| PB10 | 1 | (Reservado/wheel sensor alternativo — verificar HW) |
| PB12–PB14 | 3 | Shifter FWD/NEU/REV |
| PB15 | 1 | Wheel speed RR (EXTI15) |
| PC0–PC7 | 8 | Motor DIR (4) + EN (4) |
| PC8–PC13 | 6 | PWM steering, EN steering, Relays (3), EN RR |
| PD2 | 1 | Relay DIR |
| **Total** | **~37** | |

### Pines libres confirmados

| Pin | Alternate Functions disponibles | Estado |
|-----|-------------------------------|--------|
| **PA4** | SPI1_NSS, DAC1_OUT1, USART2_CK | ✅ Libre |
| **PA5** | SPI1_SCK, DAC1_OUT2, TIM2_CH1 | ✅ Libre |
| **PA6** | SPI1_MISO, TIM3_CH1, TIM16_CH1 | ✅ Libre |
| **PA7** | SPI1_MOSI, TIM3_CH2, TIM17_CH1 | ✅ Libre |
| **PA12** | USART1_RTS, TIM1_ETR, USB_DP | ✅ Libre |
| **PA13** | SWDIO | ⚠️ Debug — no usar |
| **PA14** | SWCLK | ⚠️ Debug — no usar |
| **PB1** | TIM3_CH4, COMP4_OUT | ✅ Libre |
| **PB2** | LPTIM1_OUT, RTC_OUT2 | ✅ Libre |
| **PB11** | **I2C2_SDA** (AF4), USART3_RX, TIM2_CH4 | ✅ Libre — **candidato SDA** |
| **PC14** | OSC32_IN | ✅ Libre (si no se usa LSE) |
| **PC15** | OSC32_OUT | ✅ Libre (si no se usa LSE) |

**Total de pines libres: ~9–11** (suficientes para VL53L8CX + expansión)

### ¿Hay pines suficientes?

**Sí.** El VL53L8CX necesita:
- 2 pines para I2C (SCL + SDA)
- 1 pin para XSHUT (reset hardware, recomendado)
- 1 pin para INT (dato listo, opcional pero recomendado)

**Total: 3–4 pines.** Hay 9+ pines libres. No hay restricción de hardware.

---

## 9. Selección de periférico I2C

### Recomendación: **I2C3** en PA8/PC9... **NO.** Esos pines están usados.

### Análisis de periféricos I2C disponibles

| Periférico | Pines posibles | ¿Libres? | Conflictos |
|------------|---------------|----------|------------|
| **I2C1** | PB6/PB7 | ❌ Ya usado | TCA9548A + INA226 + ADS1115 |
| **I2C2** | PA9/PA10 | ❌ TIM1_CH2/CH3 | PWM motores FR/RL |
| **I2C2** | PB10/PB11 (AF4) | PB10 ⚠️ / PB11 ✅ | PB10 posible conflicto wheel sensor |
| **I2C3** | PA8/PC9 (AF2/AF8) | ❌ / ❌ | TIM1_CH1 (PWM FL) / EN_STEER |
| **I2C3** | PC8/PC9 | ❌ / ❌ | TIM8_CH3 (PWM steering) / EN_STEER |
| **I2C4** | PA13/PA14 | ❌ | Debug SWD |
| **I2C4** | PB6/PB7 | ❌ | Ya I2C1 |

### Solución recomendada: **I2C2 con bit-bang de SCL** o **pines alternativos de I2C2**

Tras el análisis exhaustivo, la mejor opción depende del estado real de PB10:

**Opción A — I2C2 en PA8 (AF4) + PB11 (AF4):**
- PA8 está asignado a TIM1_CH1 (PWM motor FL) — conflicto irreconciliable
- **Descartado**

**Opción B — Software I2C (bit-bang) en pines libres:**

Esta es la opción más limpia dado que todos los periféricos I2C hardware tienen conflictos de pines con el firmware actual.

| Pin | Función bit-bang |
|-----|-----------------|
| **PA4** | Soft-SCL (GPIO output open-drain) |
| **PA5** | Soft-SDA (GPIO output open-drain + input) |
| **PA6** | XSHUT (GPIO output push-pull) |
| **PA7** | INT (GPIO input, EXTI opcional) |

**Ventajas del bit-bang para este caso:**
- VL53L8CX opera a 400 kHz o 1 MHz I2C. A 170 MHz de reloj, un bit-bang a 400 kHz es trivial (~212 ciclos por medio-bit)
- **Independencia total del periférico I2C1** — un bloqueo del bus de sensores no afecta
- Los pines PA4/PA5 no tienen restricciones de alternate function que importen
- El firmware ya implementa bit-bang para OneWire (PB0, `sensor_manager.c`) con timing de microsegundos, por lo que la infraestructura de delay preciso ya existe

**Opción C — Reasignar PB10 y usar I2C2 hardware:**

Si PB10 no está realmente en uso en el hardware final:
- **PB10 (AF4) = I2C2_SCL**
- **PB11 (AF4) = I2C2_SDA**
- PA6 = XSHUT, PA7 = INT

Esta sería la opción ideal si PB10 está libre. Requiere verificación del hardware físico.

### Recomendación final

1. **Si PB10 está libre → I2C2 hardware (PB10/PB11)** — máximo rendimiento, DMA posible
2. **Si PB10 está ocupado → Software I2C en PA4/PA5** — independencia garantizada, complejidad moderada

En ambos casos, **XSHUT en PA6** y **INT en PA7** (o PB1/PB2 como alternativa).

---

## 10. Asignación concreta de pines y conexión física

### Esquema de conexión recomendado (Opción preferida: I2C2 hardware)

```
STM32G474RE                    VL53L8CX Breakout
┌─────────┐                    ┌──────────┐
│         │                    │          │
│    PB10 ├───[4.7kΩ]──┬──────┤ SCL      │
│         │             │      │          │
│    PB11 ├───[4.7kΩ]──┬──────┤ SDA      │
│         │             │      │          │
│     PA6 ├─────────────┼──────┤ XSHUT    │
│         │             │      │          │
│     PA7 ├─────────────┼──────┤ INT      │
│         │             │      │          │
│    3.3V ├─────────────┼──────┤ VIN      │
│         │             │      │          │
│     GND ├─────────────┼──────┤ GND      │
│         │             │      │          │
└─────────┘             │      └──────────┘
                        │
                    Pull-ups a 3.3V
```

### Esquema alternativo (Software I2C)

```
STM32G474RE                    VL53L8CX Breakout
┌─────────┐                    ┌──────────┐
│         │                    │          │
│     PA4 ├───[4.7kΩ]──┬──────┤ SCL      │
│         │             │      │          │
│     PA5 ├───[4.7kΩ]──┬──────┤ SDA      │
│         │             │      │          │
│     PA6 ├─────────────┼──────┤ XSHUT    │
│         │             │      │          │
│     PA7 ├─────────────┼──────┤ INT      │
│         │             │      │          │
│    3.3V ├─────────────┼──────┤ VIN      │
│         │             │      │          │
│     GND ├─────────────┼──────┤ GND      │
│         │             │      │          │
└─────────┘             │      └──────────┘
                        │
                    Pull-ups a 3.3V
```

### Especificaciones de conexión

| Señal | Pin STM32 | Configuración GPIO | Notas |
|-------|-----------|-------------------|-------|
| SCL | PB10 (o PA4) | AF4 Open-Drain (o GPIO OD) | Pull-up 4.7 kΩ a 3.3V |
| SDA | PB11 (o PA5) | AF4 Open-Drain (o GPIO OD) | Pull-up 4.7 kΩ a 3.3V |
| XSHUT | PA6 | GPIO Push-Pull, default LOW | LOW = sensor en reset, HIGH = operativo |
| INT | PA7 | GPIO Input, pull-up interno | Active LOW, indica dato listo |

### Consideraciones de PCB/cableado

1. **Longitud máxima del cable I2C:** 30 cm para 400 kHz, 15 cm para 1 MHz. El sensor debe estar montado en el parachoques frontal del vehículo — si la distancia al PCB principal es > 30 cm, reducir velocidad a 100 kHz (el VL53L8CX soporta 100 kHz–1 MHz según datasheet ST) o usar cable apantallado. A 100 kHz la lectura del buffer de ranging (~256 bytes) tarda ~20 ms, lo que excedería el presupuesto del ciclo de 10 ms. En ese caso, leer en dos ciclos alternados (128 bytes cada uno) o reducir la resolución a 4×4 (64 bytes).

2. **Pull-ups:** 4.7 kΩ es estándar para 400 kHz. Si el cable es corto (< 10 cm), las pull-ups del breakout del VL53L8CX pueden ser suficientes (la mayoría de breakouts incluyen pull-ups de 2.2 kΩ o 4.7 kΩ). **No duplicar pull-ups** — si el breakout ya las tiene, no añadir externas (la resistencia efectiva paralela sería demasiado baja).

3. **Desacoplo:** Condensador de 100 nF lo más cerca posible del VIN del sensor. Si el cable es > 15 cm, añadir 10 µF electrolítico en el extremo del sensor.

4. **Protección EMI:** El entorno del vehículo tiene motores BTS7960 conmutando a 20 kHz. La línea I2C debe:
   - No correr paralela a cables de potencia de motor
   - Usar cable trenzado (SDA+SCL+GND mínimo)
   - Si es posible, añadir ferrita en VIN del sensor

5. **Montaje del sensor:** El VL53L8CX tiene un FoV de 65° × 65°. Montado centrado en el frontal del vehículo, a la altura del parachoques, orientado horizontalmente. Inclinación hacia abajo de 5–10° para compensar la altura de montaje y detectar obstáculos a nivel del suelo.

---

## 11. Protección del control de motor frente a bloqueo I2C

### El riesgo

Si el VL53L8CX bloquea la línea SDA (slave hold — el sensor retiene SDA LOW esperando un clock que no llega), y el sensor comparte bus con los INA226, se produciría:

1. Fallo de lectura de corriente → sin monitorización de sobrecorriente
2. Fallo de lectura del ADS1115 → sin plausibilidad de pedal
3. Si el bloqueo dura > 500 ms → timeout de watchdog I2C → intento de recovery
4. Durante el recovery, **todas las lecturas de sensores I2C se detienen**

Esto es **inaceptable** para un sistema de seguridad.

### Solución: aislamiento por periférico I2C separado

```
Bus I2C1 (PB6/PB7)                    Bus I2C2 o Soft-I2C (PB10/PB11 o PA4/PA5)
┌──────────────────────┐               ┌──────────────────────┐
│ TCA9548A (0x70)      │               │ VL53L8CX (0x29)      │
│  ├─ INA226 ×6        │               │                      │
│ ADS1115 (0x48)       │               │                      │
└──────────────────────┘               └──────────────────────┘
         │                                       │
    Sensores de corriente                  Sensor de obstáculos
    + plausibilidad pedal                  (seguridad primaria)
         │                                       │
    Fallo aquí NO afecta ──────── X ────── Fallo aquí NO afecta
    al sensor de obstáculos                a corriente ni pedal
```

### Mecanismos de protección (5 capas)

**Capa 1 — Aislamiento físico:**
- Periféricos I2C separados con buses eléctricos independientes
- Un bloqueo de SDA en un bus no afecta al otro
- Pull-ups independientes en cada bus

**Capa 2 — Timeout de transacción:**
- Cada lectura I2C del VL53L8CX tiene un timeout de 5 ms (suficiente para una transacción de 256 bytes a 400 kHz)
- Si timeout → incrementar `vl53_fail_count`, no bloquear el bucle principal
- La función `HAL_I2C_Master_Receive()` con timeout es no-bloqueante respecto al resto del sistema

**Capa 3 — Lectura en ventana temporal:**
- El VL53L8CX se lee una vez por ciclo de 10 ms (en la sección de `Safety_Check*()` del bucle principal)
- La lectura completa (seleccionar resultado + leer buffer) debe completarse en ≤ 2 ms
- Si excede 2 ms → abortar, usar último dato válido
- El control del motor (PWM, ABS, TCS) ejecuta en la misma iteración de 10 ms pero **después** de la lectura — nunca se bloquea esperando al sensor

**Capa 4 — Bus recovery independiente:**
- Si el bus del VL53L8CX se bloquea, la recovery (toggle SCL 16×, STOP condition) solo afecta a los pines del bus del VL53L8CX
- Los INA226/ADS1115 siguen operando normalmente en I2C1
- El control del motor sigue teniendo corriente, temperatura y velocidad

**Capa 5 — Reset hardware vía XSHUT:**
- Si la bus recovery falla, el pin XSHUT permite reiniciar el VL53L8CX por hardware
- XSHUT LOW durante 10 ms → HIGH → esperar 2 ms → reiniciar configuración de ranging
- Esto resuelve el 99% de los bloqueos de slave I2C (el slave libera SDA al reiniciarse)
- Si tras 3 resets el sensor sigue sin responder → declarar sensor perdido, `obstacle_scale_local = 0.3`

### Impacto en el bucle de control de 10 ms

```
Iteración de 10 ms (100 Hz):
├── [0.0-0.5 ms] Safety checks (corriente, temp, batería)     ← I2C1
├── [0.5-2.0 ms] VL53L8CX read + validate                     ← I2C2/Soft (independiente)
├── [2.0-2.5 ms] Obstacle_Update() fusion (CAN + local)
├── [2.5-4.0 ms] Steering PID
├── [4.0-6.0 ms] Traction_Update() (aplica obstacle_scale)
├── [6.0-7.0 ms] CAN TX status
└── [7.0-10.0 ms] Margen / IWDG refresh
```

**Peor caso (VL53L8CX bloqueado):**
- La lectura del VL53L8CX retorna timeout a los 5 ms
- El bucle se extiende a ~13.5 ms en esa iteración (3.5 ms extra sobre el presupuesto de 1.5 ms de la ventana de lectura)
- El siguiente ciclo usa `last_valid_distance` — sin impacto en el control del motor
- El watchdog IWDG (4.1 s timeout) no se dispara por un retraso de 3.5 ms
- Si el timeout se repite, los reintentos se eliminan (fail_count ≥ 3) y la lectura se omite en ciclos posteriores, restaurando el timing de 10 ms

**Peor caso (bus recovery activa):**
- El recovery (16 toggles + STOP) toma ~200 µs
- Se ejecuta **una vez** cuando se detecta bloqueo, no en cada ciclo
- Impacto: 1 ciclo de 10 ms se extiende a ~10.7 ms — imperceptible

---

## Resumen de decisiones de diseño

| Decisión | Elección | Alternativa descartada | Motivo |
|----------|----------|----------------------|--------|
| Módulo propietario | `safety_system.c` | `sensor_manager.c` | Coherencia con autoridad de seguridad |
| Fusión de fuentes | `min(CAN, local)` | Promedio ponderado | Fail-safe algebraico |
| Fallo de sensor | `scale = 0.3` | `scale = 0.0` o `scale = 1.0` | Ni inmoviliza ni deja ciego |
| Falsos positivos | Quórum 3/16 + 200 ms confirm | Single-zone trigger | Elimina reflejos sin perder velocidad |
| Limitación de par | Rampa + clamp + corte | Solo corte duro | Suave en warning, duro en emergencia |
| Movilidad | Escape reverso + min 0.3 | Corte total bidireccional | Siempre puede volver a casa |
| Bus I2C | Periférico separado | Compartir I2C1 | Aislamiento de fallos |
| Multiplexor | No necesario (sensor único) | TCA9548A adicional | Dirección única, bus dedicado |
| Pines | PB10/PB11 (I2C2) o PA4/PA5 (soft) | PA8/PC9 (I2C3) | Sin conflictos con PWM |
| Reset hardware | XSHUT en PA6 | Solo software recovery | Garantía de desbloqueo |
| Protección motor | 5 capas de aislamiento | Single-bus con timeout | Independencia eléctrica total |

---

## Próximos pasos (cuando se apruebe la arquitectura)

1. Verificar estado real de PB10 en hardware físico → decidir I2C2 hardware vs. soft-I2C
2. Implementar driver VL53L8CX de bajo nivel (registros ST, configuración de ranging)
3. Integrar `VL53L8CX_ReadDistance()` en el bucle de 10 ms de `safety_system.c`
4. Implementar `Obstacle_FuseLocal()` con lógica `min(CAN, local)`
5. Añadir diagnóstico del sensor al CAN `STATUS_SAFETY` (0x203)
6. Validar timing del bucle con osciloscopio (verificar que no excede 10 ms)
7. Test de regresión: verificar que ABS, TCS, LIMP_HOME, y escape reverso siguen funcionando
8. Test de inyección de fallos: simular bloqueo I2C, sensor congelado, lecturas erróneas
