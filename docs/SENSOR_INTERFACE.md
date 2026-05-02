# Interfaz de Sensores — Sistema de Control de Vehículo

**MCUs:** STM32G474RE (control en tiempo real) + ESP32-S3 (HMI y periféricos)

> Este documento describe las interfaces de sensores **exactas** implementadas en firmware.
> No se incluye hardware especulativo ni planificado.

---

## Índice

1. [INA226 — Sensores de Corriente/Tensión (×6)](#1-ina226--sensores-de-corrientetensión-6)
2. [DS18B20 — Sensores de Temperatura (×5)](#2-ds18b20--sensores-de-temperatura-5)
3. [LJ12A3 — Sensores de Velocidad de Rueda (×4)](#3-lj12a3--sensores-de-velocidad-de-rueda-4)
4. [Pedal Acelerador — Doble Canal Redundante](#4-pedal-acelerador--doble-canal-redundante)
5. [E6B2-CWZ6C — Encoder de Dirección (Cuadratura)](#5-e6b2-cwz6c--encoder-de-dirección-cuadratura)
6. [LJ12A3 — Sensor de Centro de Dirección](#6-lj12a3--sensor-de-centro-de-dirección)
7. [TF-Mini Plus — Sensor LiDAR de Obstáculos (Benewake)](#7-tf-mini-plus--sensor-lidar-de-obstáculos-benewake)
8. [MCP23017 — Selector de Marchas (Shifter)](#8-mcp23017--selector-de-marchas-shifter)
9. [Interruptor de Tracción 2WD/4WD](#9-interruptor-de-tracción-2wd4wd)
10. [Sensor de Contacto (Ignition Sense)](#10-sensor-de-contacto-ignition-sense)

---

## 1. INA226 — Sensores de Corriente/Tensión (×6)

Seis sensores INA226 miden corriente y tensión de los cuatro motores, la batería de 24 V y la dirección. Todos comparten dirección I2C 0x40 y se seleccionan mediante un multiplexor TCA9548A.

### Tabla de pines

| Señal | Pin STM32 | Periférico | Nota |
|-------|-----------|------------|------|
| SCL | PB6 | I2C1_SCL | 400 kHz Fast-Mode |
| SDA | PB7 | I2C1_SDA | 400 kHz Fast-Mode |

### Canales TCA9548A (dirección 0x70)

| Canal | Sensor | Shunt | Corriente máx. |
|-------|--------|-------|-----------------|
| 0 | Motor FL | 1.5 mΩ | 50 A |
| 1 | Motor FR | 1.5 mΩ | 50 A |
| 2 | Motor RL | 1.5 mΩ | 50 A |
| 3 | Motor RR | 1.5 mΩ | 50 A |
| 4 | Batería 24 V | 0.75 mΩ | 100 A |
| 5 | Dirección | 1.5 mΩ | 50 A |

### Resistencias y protección

- **Pull-ups I2C:** 4.7 kΩ a 3.3 V en SDA y SCL (una sola vez en el bus, no por sensor).
- **Shunt motores:** 1.5 mΩ — tolerancia ≤1 %, potencia ≥3.75 W (P = 50² × 0.0015 = 3.75 W).
- **Shunt batería:** 0.75 mΩ — tolerancia ≤1 %, potencia ≥7.5 W (P = 100² × 0.00075 = 7.5 W).

### Alimentación

- INA226: 3.3 V desde el regulador del STM32.
- TCA9548A: 3.3 V.
- Lado de alta potencia (shunts): línea de bus 24 V de batería.

### Motivo técnico

- **400 kHz:** necesario para leer 6 sensores dentro de la ventana de control de 10 ms.
- **TCA9548A:** permite que todos los INA226 tengan la misma dirección 0x40, simplificando el PCB y el firmware.
- **Shunt 0.75 mΩ para batería:** reduce la caída de tensión a 75 mV @ 100 A (aceptable).
- **Shunt 1.5 mΩ para motores:** maximiza la resolución del ADC del INA226 (LSB = 2.5 µV → 1.67 A/bit a 1.5 mΩ).

### Qué ocurre si falla

- **Bus I2C sin respuesta (NACK/timeout):** el firmware marca la lectura como inválida y activa limitación de corriente por defecto (safe current limit).
- **TCA9548A bloqueado:** reset por línea de reset del multiplexor; si persiste, se entra en estado SAFE (todos los motores deshabilitados).
- **Shunt abierto:** la lectura de corriente cae a ~0 A; la protección de sobre-corriente por hardware del BTS7960 sigue activa como respaldo.
- **Lectura de tensión anómala (<18 V o >30 V):** se activa alerta de batería y se limita la potencia de salida.

### Esquema de conexión

```
                    Bus I2C (PB6/PB7)
                         │
           ┌─────────────┴─────────────┐
           │       TCA9548A (0x70)      │
           │  3.3V ── VCC    GND ── GND │
           │  PB6 ── SCL    SDA ── PB7  │
           │  A0=A1=A2=GND → 0x70       │
           ├────────────────────────────┤
           │ CH0  CH1  CH2  CH3  CH4  CH5│
           └──┬───┬────┬────┬────┬────┬──┘
              │   │    │    │    │    │
           INA226 INA226 ... (cada uno 0x40)
              │
     ┌────────┴────────┐
     │  VCC ── 3.3V    │
     │  VS+ ──┐        │
     │        ├─ Shunt (1.5 mΩ o 0.75 mΩ)
     │  VS- ──┘        │
     │  GND ── GND     │
     │  A0=A1=GND→0x40 │
     └─────────────────┘

  Pull-ups (una vez en el bus):
  SDA ──┤4.7kΩ├── 3.3V
  SCL ──┤4.7kΩ├── 3.3V
```

---

## 2. DS18B20 — Sensores de Temperatura (×5)

Cinco sensores Dallas DS18B20 en un único bus OneWire para monitorizar la temperatura de los cuatro motores y la temperatura ambiente/dirección.

### Tabla de pines

| Señal | Pin STM32 | Periférico | Nota |
|-------|-----------|------------|------|
| DATA | PB0 | GPIO (OneWire bit-bang) | Bus único, 5 sensores |

### Posiciones de los sensores

| Sensor | Ubicación |
|--------|-----------|
| 1 | Motor FL |
| 2 | Motor FR |
| 3 | Motor RL |
| 4 | Motor RR |
| 5 | Dirección / Ambiente |

### Resistencias y protección

- **Pull-up OneWire:** 4.7 kΩ entre DATA (PB0) y 3.3 V. Obligatoria: el DS18B20 es open-drain.
- **Protección ESD:** TVS unidireccional en la línea DATA (recomendado si el cableado supera 50 cm).

### Alimentación

- DS18B20: 3.3 V (modo alimentación externa, **no** parasitaria). VDD a 3.3 V, GND a GND.

### Motivo técnico

- **Bus único con 5 sensores:** cada DS18B20 tiene una ROM única de 64 bits; el firmware los direcciona individualmente.
- **Pull-up 4.7 kΩ:** valor estándar para OneWire a 3.3 V; valores menores reducen el margen de nivel bajo, valores mayores ralentizan los flancos.
- **Rango -40 °C a +125 °C:** cubre el rango operativo de los motores con margen.

### Umbrales térmicos

| Nivel | Temperatura | Acción |
|-------|-------------|--------|
| Advertencia | 80 °C | DEGRADED, reducción de potencia (`TEMP_WARNING_C` en firmware) |
| Estado SAFE | 90 °C | Parada completa, MOE borrado (`TEMP_CRITICAL_C` en firmware) |
| Corte de emergencia por motor | 130 °C | Motor individual deshabilitado (`wheel_scale = 0`) |
| Recuperación por motor | 115 °C | Motor rehabilitado tras enfriar por debajo de 115 °C |
| Recuperación DEGRADED→ACTIVE | 75 °C | Todas las temps bajo 75 °C (histéresis 5 °C bajo advertencia) |

### Qué ocurre si falla

- **Sensor desconectado (CRC inválido o sin presencia):** el firmware marca ese canal como fallo y aplica limitación de corriente preventiva al motor asociado.
- **Lectura atascada (mismo valor >10 s):** se trata como sensor fallido.
- **Todos los sensores sin respuesta (bus cortado):** el sistema entra en estado SAFE (potencia reducida en todos los motores).

### Esquema de conexión

```
  3.3V ──┬──────────────────────────────────┐
         │                                  │
        ┤4.7kΩ├                             │
         │                                  │
  PB0 ───┴───┬──────┬──────┬──────┬─────────┤
             │      │      │      │         │
          ┌──┴──┐┌──┴──┐┌──┴──┐┌──┴──┐  ┌──┴──┐
          │DS   ││DS   ││DS   ││DS   │  │DS   │
          │18B20││18B20││18B20││18B20│  │18B20│
          │ FL  ││ FR  ││ RL  ││ RR  │  │Dir/ │
          │     ││     ││     ││     │  │Amb  │
          └──┬──┘└──┬──┘└──┬──┘└──┬──┘  └──┬──┘
             │      │      │      │        │
            GND    GND    GND    GND      GND
```

---

## 3. LJ12A3 — Sensores de Velocidad de Rueda (×4)

Cuatro sensores inductivos de proximidad LJ12A3-4-Z/BX (NPN NO) detectan los pulsos de los dientes de engranaje montados en cada rueda.

> **Adaptación de señal:** los sensores operan a 12 V y sus señales pasan por el
> **Board 1 de la placa EL817 de 4 canales** (aislamiento galvánico + adaptación de nivel 12 V → 3,3 V).
> Ver `docs/EL817_WIRING_REFERENCE.md` para el cableado completo.

### Tabla de pines

| Rueda | Pin STM32 | Línea EXTI | Flanco | Canal EL817 Board 1 |
|-------|-----------|------------|--------|---------------------|
| FL | PA0 | EXTI0 | Subida (rising) | CH2 |
| FR | PA1 | EXTI1 | Subida (rising) | CH1 |
| RL | PA2 | EXTI2 | Subida (rising) | CH4 |
| RR | PB15 | EXTI15 | Subida (rising) | CH3 |

### Parámetros

| Parámetro | Valor |
|-----------|-------|
| Tipo de salida | NPN NO (colector abierto) |
| Pulsos por revolución | 6 |
| Circunferencia de rueda | 1.1 m |
| Velocidad máx. válida | 25 km/h |
| Debounce µs (DWT, pre-filtro EMI) | **200 µs** (`SENSOR_DEBOUNCE_US`) |
| Debounce ms (HAL_GetTick, filtro secundario) | 1 ms (`WHEEL_MIN_PULSE_INTERVAL_MS`) |

### FILTRADO DE SEÑAL (DEBOUNCE SOFTWARE)

**Problema:** Los optoacopladores EL817 introducen jitter en los flancos de salida cuando el LED se aproxima al umbral de saturación. Además, en entornos automotrices con motores de CC conmutados (PWM 20 kHz), los transitorios EMI inducidos en el cableado pueden generar flancos espurios adicionales. Ambos fenómenos producen el mismo síntoma: pulsos extra dentro de una ventana de microsegundos alrededor del flanco real.

**Solución:** Se implementa un filtro temporal de **dos capas** en el handler de EXTI:

| Capa | Mecanismo | Ventana | Propósito |
|------|-----------|---------|-----------|
| **Pre-filtro µs** | DWT->CYCCNT (170 ciclos/µs) | **200 µs** | Elimina bursts EMI y jitter del EL817 |
| Filtro ms | HAL_GetTick (1 ms resolución) | 1 ms | Rechaza rebotes mecánicos residuales |

**Implementación (en `sensor_manager.c`, función `Wheel_IRQDebounced`):**
```c
// Pre-filtro EMI — PRIMER CHECK en el handler, antes de cualquier procesamiento
uint32_t cyc_now = DWT->CYCCNT;
if ((cyc_now - wheel_last_edge_cyc[idx]) < sensor_debounce_cycles)
    return;
wheel_last_edge_cyc[idx] = cyc_now;
```

**Validación:**
- A 25 km/h (máximo), período de pulso ≈ 26 ms → 200 µs = **0,77 %** del período → sin riesgo de pérdida de pulsos reales
- Los bursts EMI típicos duran 1–50 µs → completamente absorbidos por la ventana de 200 µs
- `sensor_debounce_cycles` = `SENSOR_DEBOUNCE_US × (SystemCoreClock / 1 000 000)` = `200 × 170` = **34 000 ciclos** (precomputado en `Sensor_Init()`)
- Variables de estado por canal: `wheel_last_edge_cyc[0..3]` y `steer_last_edge_cyc` — nunca compartidas

**Impacto funcional:** Ninguno en condiciones nominales. Solo filtra pulsos separados < 200 µs entre sí, lo que es físicamente imposible en este sistema a cualquier velocidad real.

#### Validación matemática (auditoría — frecuencias reales)

| Métrica | Valor | Cálculo |
|---------|-------|---------|
| Velocidad máx. del sistema | 25 km/h | Especificación de seguridad |
| Velocidad lineal máx. | 6,944 m/s | 25 / 3,6 |
| Circunferencia de rueda | 1,1 m | Hardware |
| Pulsos por revolución | 6 | Diente de engranaje |
| **Frecuencia máx. de pulsos** | **37,9 Hz** | (6,944 / 1,1) × 6 |
| **Período mínimo entre pulsos** | **26 316 µs** | 1 / 37,9 × 1 000 000 |
| Ventana debounce | 200 µs | `SENSOR_DEBOUNCE_US` |
| **Margen relativo** | **0,76 %** | 200 / 26 316 |

**Criterio de auditoría:**
- margen > 5 % → conservador (riesgo si el sistema cambia)
- 1 % ≤ margen ≤ 5 % → adecuado
- margen < 1 % → **óptimo** ✅

**Decisión final:** mantener **200 µs**.
- Margen actual (0,76 %) ya está por debajo del 1 % → categoría óptima.
- Reducir a 100 µs no aporta beneficio funcional medible (margen pasaría a 0,38 %, irrelevante a 38 Hz).
- Reducir a 100 µs aumentaría la exposición a bursts EL817 prolongados (datasheet: t_off típico ~10 µs, hasta 50 µs en saturación marginal). 200 µs absorbe **2× el peor caso documentado**.
- Aumentar > 200 µs está prohibido por la especificación.

### Resistencias y protección

- **Pull-up salida EL817:** 2,7 kΩ integrado on-board + 470 Ω serie (pull-up efectivo 3,17 kΩ). No se añade pull-up externo.
- **Pull-up interno STM32:** (~40 kΩ) puede permanecer habilitado — el pull-up externo on-board domina.
- **R entrada EL817:** 100 Ω + 2,7 kΩ integrados on-board — no se añade resistencia externa.
- **Protección automotriz:** TVS **P6KE18CA** entre `n+` y `n-` de cada canal (4× en Board 1). Sustituye al BAT54 y al 1N4148 anteriores.

### Alimentación

- LJ12A3: 12 V del vehículo (marrón = VCC, azul = GND_vehicle, negro = NPN out).
- La señal adaptada a 3,3 V sale del pin `O1`–`O4` de la placa EL817.
- EL817 Board 1: `V` → +3,3 V (regulador externo AMS1117-3.3), `G` → GND_logic.

### Motivo técnico

- **6 pulsos/rev:** con circunferencia 1.1 m, a 25 km/h (6.94 m/s) se generan ~37.8 pulsos/s, suficiente resolución sin saturar la ISR.
- **Pre-filtro 200 µs (DWT):** elimina jitter EL817 y EMI antes de actualizar cualquier contador. Sin impacto en velocidad de cómputo.
- **Filtro ms (1 ms):** segundo nivel de rechazo para rebotes residuales; período mínimo de pulso real es 26 ms → sin pérdida.
- **EXTI (interrupciones):** captura exacta del instante del pulso sin polling, crítico para cálculo preciso de velocidad.
- **EL817 Board 1:** el pull-up integrado de 2,7 kΩ da un τ ≈ 3,2 µs con 1 nF de cable — más rápido que la resistencia externa de 4,7 kΩ usada anteriormente.

### Qué ocurre si falla

- **Sin pulsos (sensor desconectado o cable roto):** velocidad calculada = 0 km/h. Si los otros 3 sensores reportan movimiento, se detecta fallo de sensor individual.
- **Pulsos espurios (interferencia):** el pre-filtro DWT (200 µs) los absorbe antes de que lleguen al contador. El filtro ms (1 ms) actúa como segunda barrera. Si la velocidad calculada supera 25 km/h, se descarta como lectura inválida.
- **Cortocircuito a GND permanente:** se interpreta como velocidad infinita → el filtro de velocidad máxima (25 km/h) lo detecta y marca como fallo.

### Esquema de conexión (con EL817 Board 1)

```
  12V ──────────────────────────────────────────────────► Marrón (VCC LJ12A3)
  12V ────────────────────────────────► n+  EL817 Board 1 (CHn)
                                         │
                                  [100Ω + 2,7kΩ on-board]
                                         │
                                      EL817 LED
                                         │
                                         └──► n-  ──► Negro (NPN out LJ12A3)
                                                            │ (NPN tira a GND cuando activo)
  GND_vehicle ──────────────────────────────────────────► Azul (GND LJ12A3)

  [P6KE18CA entre n+ y n-]  ← protección automotriz (obligatorio)

  ────────────────── LADO LÓGICO ──────────────────
  +3,3V ──[2,7kΩ on-board]──► colector EL817 ──[470Ω]──► On ──► STM32 PAx
  GND_logic ──────────────────────────────────────────────────── G EL817

  Flujo de señal (en ISR):
  Flanco EXTI → pre-filtro 200 µs DWT → filtro 1 ms HAL_GetTick → flood-check → wheel_pulse++
```

---

## 4. Pedal Acelerador — Doble Canal Redundante

El acelerador utiliza un sensor de efecto Hall SS1324LUA-T con dos canales de lectura independientes para validación cruzada.

### Canal primario — ADC interno del STM32

| Señal | Pin STM32 | Periférico | Nota |
|-------|-----------|------------|------|
| Señal escalada | PA3 | ADC1_IN4 | Divisor resistivo 10 kΩ + 6.8 kΩ |

### Plausibilidad por software (sin ADS1115)

> ⚠️ **CAMBIO:** El ADS1115 externo ha sido eliminado. La plausibilidad se realiza por software con el ADC interno:
> dual-sample consistencia (±30 counts), rango (30–2800 counts), tasa de cambio (35%/50ms), filtro EMA (α=0.3).

### Parámetros del sensor Hall SS1324LUA-T

| Parámetro | Valor |
|-----------|-------|
| Alimentación | 5 V |
| Salida | 0.3 V – 4.8 V (ratiométrica) |

### Canal primario (PA3 con divisor)

| Parámetro | Valor |
|-----------|-------|
| Divisor | 10 kΩ (serie) + 6.8 kΩ (a GND) |
| Ratio | 6.8 / (10 + 6.8) = 0.4048 |
| Tensión en PA3 | 0.121 V – 1.943 V |
| Resolución ADC | 12 bits (0–4095) |
| Cuentas válidas | 150 – 2413 |
| Tiempo de muestreo | 47.5 ciclos @ 42.5 MHz (~1.1 µs) |

### Plausibilidad por software

| Parámetro | Valor |
|-----------|-------|
| Método | Doble lectura ADC consecutiva (~2 µs total) |
| PEDAL_ADC_FAULT_LO | 30 counts (circuito abierto) |
| PEDAL_ADC_FAULT_HI | 2800 counts (cortocircuito) |
| PEDAL_SAMPLE_TOLERANCE | ±30 counts entre muestras |
| PEDAL_MAX_RATE_PCT | 35%/ciclo (50 ms) |
| Filtro EMA | α=0.3 (~150 ms settling) |

### Validación de plausibilidad

| Verificación | Criterio | Acción |
|-------------|----------|--------|
| Consistencia dual-sample | Diferencia > ±30 counts entre lecturas consecutivas | `pedal_channels_contradict = true`, skip ciclo |
| Rango válido | < 30 o > 2800 counts | `pedal_plausible = false` |
| Tasa de cambio | > 35% por ciclo (50 ms) | `pedal_plausible = false` |
| Filtro EMA | α=0.3 (~150 ms settling) | Suaviza ruido |

### Resistencias y protección

- **Divisor resistivo:** 10 kΩ (serie) + 6.8 kΩ (a GND). Precisión ≥1 % para mantener el ratio calibrado.
- **Condensador anti-ruido:** 100 nF cerámico en PA3 a GND (filtro RC con 6.8 kΩ → fc ≈ 234 Hz, suficiente para señal de pedal <10 Hz).
- **Plausibilidad software:** dual-sample ADC consistencia, validación de rango, límite de tasa de cambio, filtro EMA. No requiere hardware adicional.

### Alimentación

- Sensor Hall SS1324LUA-T: **5 V** regulado.
- Divisor resistivo: sin alimentación propia (pasivo).
- Sin ADS1115: eliminado. Plausibilidad por software.

### Motivo técnico

- **Doble canal:** normativa de seguridad funcional (ISO 26262 inspirado); un solo canal de pedal es un punto único de fallo inaceptable.
- **Divisor 10k+6.8k:** escala la señal de 5 V a <2 V para el ADC de 3.3 V con margen. Valores altos minimizan la corriente drenada del sensor (~0.3 mA).
- **Plausibilidad software:** doble lectura ADC consecutiva con 4 capas de validación (consistencia, rango, tasa de cambio, filtro EMA).
- **47.5 ciclos de muestreo:** compromiso entre velocidad y precisión; la impedancia de fuente (~4 kΩ del divisor) requiere ≥30 ciclos para carga del sample-and-hold.

### Qué ocurre si falla

- **Canal primario fuera de rango (<150 o >2413 cuentas):** se interpreta como cable roto o cortocircuito → acelerador forzado a 0 %.
- **Fallo de plausibilidad software (dual-sample diverge, rango fuera, tasa excesiva):** acelerador forzado a 0 %, modo degradado.
- **Divergencia >5 % durante >200 ms:** se asume fallo de un canal → acelerador forzado a 0 % hasta que los canales converjan.
- **Dato obsoleto >500 ms:** el firmware fuerza acelerador a 0 % (pérdida de señal).

### Esquema de conexión

```
               Sensor Hall SS1324LUA-T
              ┌────────────────────┐
   5V ────────┤ VCC            OUT ├───┬─────────────────────┐
              │                    │   │                     │
   GND ───────┤ GND                │   │ Señal 0.3–4.8V     │
              └────────────────────┘   │                     │
                                       │                     │
          Canal Primario (STM32)       │  Plausibilidad Software
                                       │  (dual-sample ADC interno)
                                       │
                              ┌────────┘
                              │
                           10kΩ (1%)
                              │
  STM32 PA3 (ADC1_IN4) ──────┤    ← Lectura 1 + Lectura 2 (~2 µs)
                              │
                           6.8kΩ (1%)
                              │
                             GND
                              │
                          100nF (C)
                              │
                             GND

  Tensión en PA3: 0.3V×0.4048=0.121V  a  4.8V×0.4048=1.943V
```

---

## 5. E6B2-CWZ6C — Encoder de Dirección (Cuadratura)

Encoder incremental de 1200 PPR que mide la posición angular del volante mediante decodificación en cuadratura por hardware (TIM2).

### Tabla de pines

| Señal | Pin STM32 | Periférico | Nota |
|-------|-----------|------------|------|
| Fase A | PA15 | TIM2_CH1 | Entrada de cuadratura |
| Fase B | PB3 | TIM2_CH2 | Entrada de cuadratura |
| Índice Z | PB4 | EXTI4 | Pulso de referencia (1/rev) |

### Parámetros

| Parámetro | Valor |
|-----------|-------|
| PPR (pulsos por revolución) | 1200 |
| CPR (cuentas por revolución, ×4) | 4800 |
| Resolución angular | 360° / 4800 = 0.075°/cuenta |
| Recorrido del volante | ±350° (±4667 cuentas) |
| Filtro de entrada | 6 × fDTS (~210 ns de rechazo de glitch) |

### Resistencias y protección

- **Aislamiento y conversión de nivel 5 V → 3.3 V:** obligatorio. El E6B2-CWZ6C tiene salida push-pull de 5 V; conectar directamente a los pines 3.3 V del STM32 los destruye.
  - **✅ Solución adoptada — 3× 6N137 optoacopladores:** aislamiento galvánico 2500 V + conversión 5 V→3.3 V simultáneos. 10 Mbps — margen ×500 respecto a la frecuencia máxima del encoder (~20 kHz). Protege el STM32 de picos inductivos del motor de dirección adyacente. Ver esquema completo en `docs/ENCODER_WIRING_6N137.md`.
  - **❌ NO usar TXS0108E:** no proporciona aislamiento galvánico. El motor de dirección (BTS7960, 20 kHz) genera picos inductivos y bucles de masa que corrompan los pulsos de cuadratura.
  - **❌ NO usar divisor resistivo:** sin aislamiento, deforma flancos, no protege contra picos inductivos.
  - **❌ NO usar BSS138:** sin aislamiento galvánico.
  - **⚠️ NO usar transistores genéricos** (2N2222, BC337, 2N3904, etc.) ni diodos zener del inventario como level shifters del encoder. La asimetría en tiempo de propagación entre canales A y B corrompe la decodificación en cuadratura.
- **Filtro fDTS (hardware TIM2):** 6 muestras consecutivas a frecuencia de reloj del timer; rechaza pulsos <282 ns (ruido electromagnético de los motores). Compatible con el retardo del 6N137 (120 ns máx < 282 ns).

### Alimentación

- Encoder E6B2-CWZ6C: **5 V** DC (consumo ~80 mA con line driver).
- Optoacopladores 6N137: lado lógico 3.3 V (STM32); lado encoder 5 V (aislado).

### Motivo técnico

- **1200 PPR (4800 CPR):** con recorrido de ±350°, se obtienen ±4667 cuentas, resolución de 0.075° — suficiente para control de dirección asistida.
- **TIM2 en modo encoder:** decodificación por hardware sin carga de CPU; el contador se incrementa/decrementa automáticamente.
- **Filtro 6×fDTS:** los motores BTS7960 generan EMI significativa a la frecuencia de PWM; el filtro digital elimina estos glitches sin perder pulsos reales del encoder.
- **Pulso Z (índice):** permite recalibrar la posición absoluta en cada revolución completa, compensando la acumulación de error.

### Qué ocurre si falla

- **Fase A o B desconectada:** el contador deja de incrementar o se mueve en una sola dirección → el firmware detecta velocidad de giro = 0 durante movimiento y entra en modo LIMP HOME (sin asistencia de dirección, control manual).
- **Señal Z perdida:** se pierde la recalibración periódica; el firmware usa el sensor de centro (PB5) como referencia alternativa.
- **Ruido excesivo (cuentas erráticas):** el filtro de 6×fDTS lo mitiga; si persiste, el firmware compara la velocidad angular con la corriente de dirección para detectar inconsistencia.
- **Level shifter fallido (señal atascada):** se detecta como encoder estático → modo LIMP HOME.

### Esquema de conexión

```
             Encoder E6B2-CWZ6C                 3× 6N137                   STM32G474RE
            ┌──────────────────┐                                             (3.3 V)
   5V ──────┤ VCC (marrón)     │    Lado encoder (5V)   Lado lógico (3.3V)
   GND ─────┤ GND (azul)       │    ────────────────     ──────────────────
            │                  │
            │ Fase A (negro) ──┤──[330Ω]──▷|── 6N137 #1 ──[4.7kΩ]──┬── +3.3V
            │                  │    R_IN    LED           Vo         └── PA15 (TIM2_CH1)
            │                  │                          EN ──────────── +3.3V
            │ Fase B (blanco) ─┤──[330Ω]──▷|── 6N137 #2 ──[4.7kΩ]──┬── +3.3V
            │                  │                          Vo         └── PB3  (TIM2_CH2)
            │                  │                          EN ──────────── +3.3V
            │ Z (naranja) ─────┤──[330Ω]──▷|── 6N137 #3 ──[4.7kΩ]──┬── +3.3V
            └──────────────────┘                          Vo         └── PB4  (EXTI4)
                                                          EN ──────────── +3.3V
                                              GND_encoder ─ K (todos)
                                                          GND_lógico ─── GND_STM32
```

> ⚠️ GND_encoder y GND_STM32 son eléctricamente independientes (aislamiento galvánico). No unirlos en este punto.
> ℹ️ La salida del 6N137 es invertida. Al invertir A y B a la vez, la cuadratura se preserva. Si el sentido de conteo es incorrecto, intercambiar A↔B en el conector STM32.

---

## 6. LJ12A3 — Sensor de Centro de Dirección

Sensor inductivo que detecta un tornillo de referencia en el centro mecánico de la cremallera de dirección.

> **Adaptación de señal:** el sensor opera a 12 V y su señal pasa por el
> **Board 2, CH1 de la placa EL817 de 4 canales** (aislamiento galvánico + adaptación de nivel 12 V → 3,3 V).
> Ver `docs/EL817_WIRING_REFERENCE.md` §9 para el cableado completo.

### Tabla de pines

| Señal | Pin STM32 | Periférico | Flanco | Canal EL817 Board 2 |
|-------|-----------|------------|--------|---------------------|
| Salida | PB5 | EXTI5 | **Bajada (falling)** | CH1 |

> **Flanco FALLING (bajada), no RISING:** el EL817 invierte la señal.
> Cuando el tornillo entra en el campo del sensor, el NPN conduce → EL817 output va LOW → FALLING edge.
> El firmware captura este flanco descendente como el instante exacto del centro (`GPIO_MODE_IT_FALLING`).

### FILTRADO DE SEÑAL (DEBOUNCE SOFTWARE)

La señal de centro de dirección es especialmente sensible: un flanco espurio produce una recalibración incorrecta del encoder. El pre-filtro DWT de 200 µs en `SteeringCenter_IRQHandler` elimina cualquier transitorio EMI antes de activar el flag.

| Parámetro | Valor |
|-----------|-------|
| Ventana debounce | **200 µs** (`SENSOR_DEBOUNCE_US`) |
| Mecanismo | DWT->CYCCNT (pre-filtro ISR) |
| Variable de estado | `steer_last_edge_cyc` (exclusiva, no compartida) |
| Impacto funcional | Ninguno — el rack tarda >100 ms en recorrer la zona del sensor |

### Resistencias y protección

- **Pull-up salida EL817:** 2,7 kΩ on-board + 470 Ω serie. No se añade pull-up externo.
- **Pull-up interno STM32:** (~40 kΩ) puede permanecer habilitado — el pull-up externo on-board domina.
- **Protección automotriz:** TVS **P6KE18CA** entre `n+` y `n-` del CH1 del Board 2 (obligatorio).

### Alimentación

- LJ12A3: 12 V del vehículo (marrón = VCC, azul = GND_vehicle, negro = NPN out).
- EL817 Board 2: `V` → +3,3 V, `G` → GND_logic.

### Motivo técnico

- **Sensor inductivo (no mecánico):** sin desgaste, sin contacto, funciona en entornos sucios/húmedos del chasis.
- **Detección de tornillo:** un tornillo de acero en la cremallera activa el sensor inductivo al pasar por el centro; solución robusta y barata.
- **EXTI5 con flanco de bajada:** captura el instante exacto del paso por centro para resetear el contador del encoder de dirección. El flanco de bajada (inicio del pulso) es más preciso que el de subida (fin del pulso) para la detección del punto central.
- **Aislamiento galvánico EL817:** protege el STM32 de la masa del vehículo y de los transitorios inductivos generados por el motor de dirección adyacente.

### Qué ocurre si falla

- **Sensor desconectado:** la línea queda en alto por pull-up on-board; no se genera interrupción de centro. El firmware no puede recalibrar el punto cero → se marca la calibración como no válida y se depende únicamente del encoder. El sistema entra en modo LIMP HOME si no hay calibración previa almacenada.
- **Activación permanente (cortocircuito a GND en lado sensor):** interrupciones constantes en EXTI5 → el firmware detecta frecuencia anómala y deshabilita la entrada, usando la última calibración conocida.

### Esquema de conexión (con EL817 Board 2, CH1)

```
  12V ──────────────────────────────────────────────────► Marrón (VCC LJ12A3 centro)
  12V ────────────────────────────────► n+  EL817 Board 2 CH1
                                         │
                                  [100Ω + 2,7kΩ on-board]
                                         │
                                      EL817 LED
                                         │
                                         └──► n-  ──► Negro (NPN out LJ12A3)
                                                            │ (NPN tira a GND cuando activo)
  GND_vehicle ──────────────────────────────────────────► Azul (GND LJ12A3)

  [P6KE18CA entre n+ y n-]  ← protección automotriz (obligatorio)

  ────────────────── LADO LÓGICO ──────────────────
  +3,3V ──[2,7kΩ on-board]──► colector EL817 ──[470Ω]──► O1 ──► STM32 PB5 (EXTI5)
  GND_logic ────────────────────────────────────────────────────── G EL817 Board 2

  Firmware: GPIO_MODE_IT_FALLING + GPIO_PULLUP → ISR llama SteeringCenter_IRQHandler()
```

---

## 7. TF-Mini Plus — Sensor LiDAR de Obstáculos (Benewake)

Sensor LiDAR frontal de un único punto (Time-of-Flight) gestionado por el ESP32-S3 para detección de obstáculos con 5 zonas de distancia. Comunicación UART a 115200 bps, tramas de 9 bytes.

> **Sensor activo en firmware**: Benewake TF-Mini Plus (`SENSOR_TYPE = SENSOR_TYPE_TFMINI`).
> El driver también soporta en tiempo de compilación el TOFSense-M 8×8 de Nooploop
> (`SENSOR_TYPE_TOFSENSE`), actualmente deshabilitado. Intercambiar únicamente requiere
> cambiar el `#define SENSOR_TYPE` en `obstacle_sensor.h` y ajustar el cableado de alimentación.

### Tabla de pines

| Señal | Pin ESP32-S3 | Dirección | Nota |
|-------|-------------|-----------|------|
| UART1 RX | GPIO18 | Entrada | Sensor TX → ESP32 RX (datos del sensor) |
| UART1 TX | — | — | No conectado (recepción unidireccional) |

### Conector del sensor (JST-GH 4 pines)

| Pin | Señal | Conexión |
|-----|-------|----------|
| 1 | VCC | **5 V** (obligatorio, ~140 mA pico) |
| 2 | GND | GND común |
| 3 | TX | ESP32 GPIO18 (UART1 RX) — **conexión directa, sin divisor** |
| 4 | RX | No conectado (recepción unidireccional) |

> **Niveles lógicos:** El TF-Mini Plus emite UART a **3.3 V** — compatible directo con el ESP32-S3.
> **No se requiere divisor de tensión ni level shifter.**
> La alimentación del sensor debe ser **5 V** (no funciona a 3.3 V).

### Parámetros

| Parámetro | Valor |
|-----------|-------|
| Rango útil | 100 – 12 000 mm (10 cm – 12 m) |
| Frecuencia de salida | 100 Hz (defecto de fábrica) |
| Velocidad UART | 115 200 bps, 8N1 |
| Formato de trama | 9 bytes: `[0x59 0x59 DIST_L DIST_H STR_L STR_H TEMP_L TEMP_H CHK]` |
| Unidades de distancia | cm → convertido a mm en firmware |
| Calidad de señal (Strength) | uint16 LE; rechazado si < 100 o == 65535 (saturación) |
| Checksum | Suma de bytes [0..7] & 0xFF == byte[8] |
| Timeout sin tramas | 500 ms → estado INVALID |

### Zonas de distancia

| Zona | Rango | Factor de escala de velocidad |
|------|-------|-------------------------------|
| Emergency | < 500 mm | 0.0 (parada total) |
| Critical | 500–1000 mm | 0.3 |
| Warning | 1000–1500 mm | 0.7 |
| Caution | 1500–2000 mm | 0.85 |
| Alert | 2000–4000 mm | 0.95 |
| Sin obstáculo | ≥ 4000 mm | 1.0 |

### Resistencias y protección

- **UART RX (GPIO18):** conexión directa — TF-Mini Plus emite a 3.3 V TTL. Sin divisor de tensión ni level shifter.
- **Condensador de desacoplo:** 100 nF en VCC del sensor cerca del conector.
- **Corriente pico:** ~140 mA en la línea de 5 V — asegurar que el regulador tenga margen suficiente.

### Alimentación

- TF-Mini Plus: **5 V obligatorio** (pin 1 del conector JST-GH, consumo ~100 mA típico, 140 mA pico). El sensor no arranca correctamente a 3.3 V.

### Motivo técnico

- **TF-Mini Plus:** sensor LiDAR de un solo punto, compacto (42 mm × 15 mm × 16 mm), robusto frente a luz ambiental. No afectado por temperatura ni interferencias acústicas.
- **5 zonas con factores de escala:** reducción progresiva de velocidad en lugar de parada binaria — conducción más suave y segura.
- **Detección de sensor atascado:** si la distancia no varía más de ±10 mm durante 1000 ms con el vehículo en movimiento, se marca STUCK y el STM32 aplica factor conservador.
- **115 200 bps / 100 Hz:** a esta cadencia, 9 bytes por trama → ~0.8 ms por trama; no hay riesgo de desbordamiento de buffer. No se necesita buffer de 4 KB — el driver usa ~256 bytes.

### Qué ocurre si falla

- **Sin tramas UART (sensor desconectado):** tras 500 ms sin tramas válidas, el estado pasa a INVALID. El STM32 recibe health=0 en CAN 0x208 y aplica `obstacle_scale = 0.0` → modo SAFE.
- **Checksum incorrecto o señal baja (Strength < 100):** la trama se descarta silenciosamente; se espera la siguiente.
- **Sensor atascado (distancia no varía):** detectado como STUCK tras 1000 ms sin variación con vehículo en movimiento.
- **Saturación (Strength == 65535):** indicativo de objetivo a muy corta distancia o luz solar directa — lectura descartada.

### Esquema de conexión

```
      TF-Mini Plus (JST-GH 4-pin)
     ┌────────────────────┐
5V ──┤ pin 1  VCC         │        ⚠ 5V obligatorio
GND ─┤ pin 2  GND         │
     │ pin 3  TX ─────────┼────────► ESP32-S3 GPIO18 (UART1 RX)
     │ pin 4  RX  (n/c)   │          3.3V TTL — sin divisor
     └────────────────────┘

Desacoplo: 100 nF entre VCC y GND del sensor
```

---

## 8. MCP23017 — Selector de Marchas (Shifter)

Expansor de I/O I2C que lee la posición del selector de marchas (P, R, N, D1, D2) en el ESP32-S3.

### Tabla de pines

| Señal | Pin ESP32-S3 | Periférico | Nota |
|-------|-------------|------------|------|
| SDA | GPIO8 | I2C | MCP23017 @ 0x20 |
| SCL | GPIO9 | I2C | — |

### Entradas del MCP23017 (Puerto A)

| Pin MCP23017 | Bit | Marcha | Cable de la palanca |
|--------------|-----|--------|---------------------|
| GPA0 | 0 | P (Park)      | azul + morado    |
| GPA1 | 1 | D2 (Drive 2)  | azul + verde     |
| GPA2 | 2 | D1 (Drive 1)  | azul + amarillo  |
| GPA3 | 3 | R (Reverse)   | azul + blanco    |
| —    | — | N (Neutral)   | *(sin contacto físico — estado de reposo)* |
| GPA4 | — | sin uso       | — |

### Resistencias y protección

- **Pull-ups I2C:** 4.7 kΩ a 3.3 V en SDA (GPIO8) y SCL (GPIO9).
- **Pull-ups en Puerto A:** el MCP23017 tiene pull-ups internos configurables (100 kΩ, habilitados por registro GPPU). Cada entrada conectada a un interruptor SPST a GND.
- **Dirección I2C:** A0=A1=A2=GND → 0x20.
- **Debounce:** 100 ms en software (no requiere hardware RC adicional).

### Alimentación

- MCP23017: **3.3 V** (rango operativo: 1.8–5.5 V; a 3.3 V los niveles lógicos coinciden con el ESP32-S3).

### Motivo técnico

- **MCP23017 en vez de GPIOs directos:** ahorra 5 GPIOs del ESP32-S3 (que ya tiene restricciones de pines con la pantalla y CAN). Un solo chip I2C con 16 I/O.
- **Dirección 0x20:** la más simple (todos los pines de dirección a GND).
- **Pull-ups internos de 100 kΩ:** suficiente para interruptores con cable <1 m; no se necesitan pull-ups externos.
- **Debounce 100 ms:** un cambio de marcha es una acción deliberada del conductor (~500 ms); 100 ms elimina rebotes mecánicos del selector sin retrasar la respuesta perceptible.

### Qué ocurre si falla

- **MCP23017 sin respuesta I2C:** la marcha se congela en la última válida conocida. Si la última marcha era D1 o D2, se limita la velocidad. Se emite alerta en HMI.
- **Múltiples marchas activas simultáneamente:** se detecta como fallo de cableado → se fuerza Neutral (N) por seguridad.
- **Ninguna marcha activa:** se trata como Neutral.

### Esquema de conexión

```
  3.3V ──┬───────────────────────────────┐
         │                               │
      4.7kΩ  4.7kΩ                       │
         │     │                         │
  GPIO8 ─┴─ SDA│    ┌──────────────┐     │
  GPIO9 ───────┴────┤  MCP23017    │     │
                    │  (0x20)      │     │
                    │  VDD ────────┼─── 3.3V
                    │  VSS ────────┼─── GND
                    │  A0,A1,A2 ───┼─── GND
                    │              │
                    │  GPA0 ───────┼──── SW P  ──── COM ──── GND
                    │  GPA1 ───────┼──── SW D2 ──── COM ──── GND
                    │  GPA2 ───────┼──── SW D1 ──── COM ──── GND
                    │  GPA3 ───────┼──── SW R  ──── COM ──── GND
                    │  GPA4 ───────┼──── (sin uso, dejar al aire)
                    │              │
                    │  GPPU habilitado (100kΩ int.)│
                    └──────────────┘
```

---

## 9. Interruptor de Tracción 2WD/4WD

Interruptor simple que selecciona el modo de tracción. Gestionado por el ESP32-S3.

### Tabla de pines

| Señal | Pin ESP32-S3 | Configuración | Nota |
|-------|-------------|---------------|------|
| Tracción | GPIO15 | INPUT_PULLUP | LOW = 4WD, HIGH = 2WD |

### Resistencias y protección

- **Pull-up interno:** activado en el ESP32-S3 (~45 kΩ a 3.3 V).
- **Interruptor SPST a GND:** al cerrar, GPIO15 → LOW (4WD). Al abrir, pull-up → HIGH (2WD).
- **Sin resistencia externa necesaria** para cables <50 cm.

### Alimentación

- Interruptor: sin alimentación (contacto seco a GND).

### Parámetros de debounce

| Parámetro | Valor |
|-----------|-------|
| Intervalo de muestreo | 50 ms |
| Lecturas estables requeridas | 3 consecutivas (150 ms total) |
| Puerta de velocidad | Rechaza cambio si velocidad > 0.5 km/h |

### Motivo técnico

- **Pull-up + SPST a GND:** topología más simple y fiable. No requiere alimentación del interruptor.
- **3 lecturas estables × 50 ms:** filtro anti-rebote conservador para un interruptor mecánico robusto.
- **Puerta de velocidad (>0.5 km/h):** cambiar de 2WD a 4WD (o viceversa) en movimiento puede causar estrés mecánico en el diferencial → se permite solo con vehículo prácticamente detenido.

### Qué ocurre si falla

- **Cable roto (pin siempre HIGH):** el sistema permanece en 2WD (modo seguro por defecto, menor estrés mecánico).
- **Cortocircuito a GND (pin siempre LOW):** el sistema permanece en 4WD permanente. Menos eficiente pero seguro.
- **Rebote excesivo (>150 ms):** el filtro de 3 lecturas lo rechaza; no se produce cambio de modo.

### Esquema de conexión

```
  ESP32-S3 GPIO15
    (pull-up interno ~45kΩ a 3.3V)
         │
         │
         ├──── Interruptor SPST ──── GND
         │
    HIGH = 2WD (abierto)
    LOW  = 4WD (cerrado)

  Condición de cambio:
  - 3 lecturas estables cada 50 ms
  - Velocidad < 0.5 km/h
```

---

## 10. Sensor de Contacto (Ignition Sense)

Entrada digital que detecta si la llave de contacto está en posición ON. Gestionada por el ESP32-S3.
La señal llega **aislada galvánicamente** a través del canal `IN5` del módulo HY-M158 (PC817 ×8).

### Tabla de pines

| Señal | Pin ESP32-S3 | Configuración | Nota |
|-------|-------------|---------------|------|
| Contacto | GPIO40 | INPUT_PULLUP | LOW = Llave ON (PC817 invierte la señal) |

### Lógica y pull-up

El PC817 **invierte** la señal del vehículo (+12 V ACC):

| Estado llave | LED PC817 | GPIO 40 |
|---|---|---|
| ON (+12 V en entrada) | Conduce → colector a GND | **LOW** |
| OFF (sin corriente) | Apagado → colector flotante | **HIGH** (pull-up externo 10 kΩ + INPUT_PULLUP ~45 kΩ) |

> ⚠️ **La placa PC817 NO tiene pull-up onboard** (verificado por medición). Son necesarios:
> - **Pull-up externo 10 kΩ ¼ W** entre GPIO 40 y 3.3 V (soldar en PCB — obligatorio)
> - **INPUT_PULLUP** en firmware (~45 kΩ interno ESP32-S3 — ya configurado, red de seguridad)

### Resistencia LED de entrada (lado 12 V)

- **No añadir** — el módulo HY-M158 ya integra **3 kΩ SMD `302`** en serie con el LED de cada canal:
  `I_LED = (12 V − 1.2 V) / 3 kΩ ≈ 3.6 mA` → dentro del rango nominal del PC817, CTR ≥ 50 % satura el fototransistor.
- Antiguamente este documento pedía 1 kΩ externa, pero con el HY-M158 confirmado **no es necesaria**.

### Alimentación

- La señal de contacto viene del circuito ACC (+12 V cuando llave ON).
- El lado de salida del PC817 usa 3.3 V del ESP32/STM32 (compartido).

### Qué ocurre si falla

- **Cable roto / desconectado (pin en HIGH):** el firmware ve llave OFF → SHUTTING_DOWN (fail-safe seguro).
- **Cortocircuito a GND (pin siempre LOW):** el sistema cree que la llave está siempre puesta. El apagado depende de timeout o lógica CAN del STM32.
- **Ruido en la línea de ignición:** el debounce de 50 ms en `power_manager.cpp` absorbe transitorios mecánicos. Añadir condensador 100 nF entre GPIO 40 y GND para filtrar picos EMI del arranque.

### Esquema de conexión

```
  +12V ACC (llave ON) ──[fusible 1 A]──► HY-M158 V5 (3 kΩ on-board en serie con LED PC817)
  GND vehículo        ─────────────────► HY-M158 G (lado V)

  3.3V ──[10 kΩ ext.]──┬──► HY-M158 IN5 (colector PC817) ──► GPIO 40 (INPUT_PULLUP)
                       │
                      HY-M158 G (lado IN) ──► GND (3.3V ESP32, común con STM32)

  LOW  = Llave ON
  HIGH = Llave OFF (estado seguro)
```

## Resumen de buses y pines

### STM32G474RE

| Pin | Función | Sensor/Periférico |
|-----|---------|-------------------|
| PB6 | I2C1_SCL | TCA9548A → 6× INA226 |
| PB7 | I2C1_SDA | TCA9548A → 6× INA226 |
| PB0 | OneWire DATA | 5× DS18B20 |
| PA0 | EXTI0 rising | Velocidad rueda FL (via EL817 Board 1) |
| PA1 | EXTI1 rising | Velocidad rueda FR (via EL817 Board 1) |
| PA2 | EXTI2 rising | Velocidad rueda RL (via EL817 Board 1) |
| PB15 | EXTI15 rising | Velocidad rueda RR (via EL817 Board 1) |
| PA3 | ADC1_IN4 | Pedal acelerador (primario, divisor 10k+6.8k) |
| PA15 | TIM2_CH1 | Encoder dirección A (via 6N137) |
| PB3 | TIM2_CH2 | Encoder dirección B (via 6N137) |
| PB4 | EXTI4 | Encoder dirección Z (via 6N137) |
| PB5 | EXTI5 falling | Sensor centro dirección (via EL817 Board 2) |
| PA11 | FDCAN1_RX (AF9) | CAN bus (transceptor SN65HVD230) |
| PA12 | FDCAN1_TX (AF9) | CAN bus (transceptor SN65HVD230) |
| PC11 | GPIO out | Relé tracción 24 V (activo HIGH) |
| PC12 | GPIO out | Relé dirección 12 V (activo HIGH) |
| PB10 | GPIO out | Relé tira LED frontal 5 V (activo HIGH) |
| PB11 | GPIO out | Relé tira LED trasera 5 V (activo HIGH) |
| PA5 | GPIO out | LED LD2 on-board Nucleo-G474RE |
| PB14 | GPIO out | LED diagnóstico externo (330 Ω serie) |

### ESP32-S3

| Pin | Función | Sensor/Periférico |
|-----|---------|-------------------|
| GPIO4 | CAN TX (TWAI) | Transceptor SN65HVD230 |
| GPIO5 | CAN RX (TWAI) | Transceptor SN65HVD230 |
| GPIO8 | I2C SDA | MCP23017 (selector de marchas, 0x20) |
| GPIO9 | I2C SCL | MCP23017 (selector de marchas, 0x20) |
| GPIO15 | Entrada INPUT_PULLUP | Interruptor tracción 2WD/4WD |
| GPIO18 | UART1 RX | TF-Mini Plus LiDAR (Benewake, 115200 bps) |
| GPIO40 | Entrada INPUT_PULLUP | Sensor de contacto (ignición) via PC817 (LOW=ON) |
| GPIO41 | Salida GPIO | Power hold — HIGH = mantener alimentación post-ignición |
| GPIO43 | UART2 TX | DFPlayer Mini (audio, 9600 bps) |
| GPIO44 | UART2 RX | DFPlayer Mini (audio, 9600 bps) |
| GPIO47 | Data WS2812B | Tira LED frontal (28 LEDs, FastLED) |
| GPIO48 | Data WS2812B | Tira LED trasera (16 LEDs, FastLED) |

### Dispositivos I2C

| Dispositivo | Dirección | Bus | MCU |
|-------------|-----------|-----|-----|
| TCA9548A | 0x70 | I2C1 (PB6/PB7) | STM32 |
| INA226 (×6) | 0x40 (via mux) | I2C1 (via TCA9548A) | STM32 |
| MCP23017 | 0x20 | I2C (GPIO8/9) | ESP32-S3 |

---

## 11. Debounce Diagnostics Counters (DWT 200 µs filter)

Contadores de instrumentación añadidos para validar empíricamente la cadena de mitigación EMI hardware (TVS → optoacoplador EL817) **sin alterar el comportamiento funcional** del sistema.

### Qué miden

Cada canal con filtro DWT (Step 0 del debounce, ventana de **200 µs**) mantiene un contador 32-bit del número de **flancos rechazados** por ese filtro:

- 4 contadores por rueda: `sensor_dbg_filtered_count[0..3]` (FL, FR, RL, RR), incrementados sólo desde `Wheel_IRQDebounced`.
- 1 contador para el sensor de centro de dirección: `steer_dbg_filtered_count`, incrementado sólo desde `SteeringCenter_IRQHandler`.

Los contadores se incrementan dentro del bloque de rechazo del filtro DWT, antes del `return`. **El camino aceptado no cambia ni un ciclo.**

### Garantías de seguridad / timing

- **ISR sigue O(1)**: añade ~4 instrucciones (load + compare + add + store, Cortex-M4). Coste a 200 Hz @ 170 MHz: < 0.012 % de CPU.
- **Sin condiciones de carrera**: cada contador es escrito únicamente desde su propia EXTI. No hay reentrada en una misma EXTI line. Lectura de `volatile uint32_t` desde el getter es atómica en M4 → no se necesita `__disable_irq` ni `LDREX/STREX`.
- **Sin overflow**: incremento clamped (`if (count < 0xFFFFFFFF) count++;`). Si el sistema se inunda de EMI durante días, el contador queda saturado en `0xFFFFFFFF` — comportamiento estable y observable.
- **Diagnóstico puro**: ninguna ruta de control / safety consulta estos contadores.

### API expuesta

```c
uint32_t Sensor_GetFilteredCount(uint8_t idx);   /* idx 0..3, fuera de rango → 0 */
uint32_t Sensor_GetSteerFilteredCount(void);
```

### Exposición CAN (1 Hz, aditivo)

Dos frames diagnósticos nuevos, sin impacto en IDs existentes:

| ID | DLC | Layout | Notas |
|----|-----|--------|-------|
| `0x306` (`CAN_ID_DIAG_DEBOUNCE`) | 8 | `u16 LE` × 4: FL, FR, RL, RR | Ruedas truncadas/saturadas a `0xFFFF` |
| `0x307` (`CAN_ID_DIAG_DEBOUNCE_STEER`) | 4 | `u32 LE` | Volante completo |

Periodicidad: 1000 ms, sentido STM32 → ESP32. Carga de bus añadida ≈ 220 bps sobre 500 kbps (0.044 %).

### Cómo interpretar los valores

| Valor observado | Interpretación |
|---|---|
| `0` constante | Ambiente limpio. Cadena hardware (TVS + opto) está absorbiendo todo el ruido o no hay ruido. |
| Pequeño spike puntual (< 5) en un evento (relé, arranque de tracción) | Normal. El filtro está haciendo su trabajo. |
| Crecimiento lineal sostenido (cuentas/segundo) | EMI persistente o jitter del optoacoplador EL817. Revisar cableado, blindaje, masa común. |
| `65535+` mostrado en pantalla (rueda) | Contador interno > 65 535. El valor real sigue disponible vía `Sensor_GetFilteredCount`. Indica problema crónico que requiere intervención hardware. |
| `0xFFFFFFFF` (saturado) | Inundación EMI extrema o sensor "ladrando". Inspección física requerida. |

### Ejemplos reales esperados

- **Arranque de motor de tracción** (transitorio inductivo): típicamente 1–3 cuentas en la rueda más cercana al BTS7960 activo. Aceptable.
- **Conmutación de relé sin TVS bidireccional** (regresión hardware): decenas a centenas de cuentas por evento, escalando con la frecuencia de conmutación.
- **Cable EL817 sin par trenzado, &gt; 30 cm en el bus de motores**: cuentas/segundo crecientes — corregir trenzando o acortando.

### Visualización en HMI

Los valores son visibles en el menú oculto del ESP32 (PIN `8989` → submenú **DEBOUNCE DEBUG**). Render simple: una fila por canal con el contador decimal alineado a la derecha. Re-render natural a 1 Hz (cadencia CAN). Sin animaciones, sin sprites, no toca pantallas principales (`drive_screen`, etc.).
