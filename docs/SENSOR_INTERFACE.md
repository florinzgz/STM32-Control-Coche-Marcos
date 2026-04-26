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
7. [TOFSense-M — Sensor LiDAR de Obstáculos (Nooploop)](#7-tofsense-m--sensor-lidar-de-obstáculos-nooploop)
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

### Tabla de pines

| Rueda | Pin STM32 | Línea EXTI | Flanco |
|-------|-----------|------------|--------|
| FL | PA0 | EXTI0 | Subida (rising) |
| FR | PA1 | EXTI1 | Subida (rising) |
| RL | PA2 | EXTI2 | Subida (rising) |
| RR | PB15 | EXTI15 | Subida (rising) |

### Parámetros

| Parámetro | Valor |
|-----------|-------|
| Tipo de salida | NPN NO (colector abierto) |
| Pulsos por revolución | 6 |
| Circunferencia de rueda | 1.1 m |
| Velocidad máx. válida | 25 km/h |
| Debounce | 1 ms (software) |

### Resistencias y protección

- **Pull-up interno:** activado en el STM32 (~40 kΩ). La salida NPN NO tira a GND al detectar metal; el pull-up lleva la línea a 3.3 V en reposo.
- **Si la tensión de salida del sensor supera 3.3 V:** usar divisor resistivo (10 kΩ + 6.8 kΩ) o level shifter. El LJ12A3 típicamente opera a 6–36 V DC; la salida NPN con pull-up a 3.3 V del MCU es segura si la tensión de colector no excede VDD+0.3 V.
- **Diodo de clamp:** BAT54 entre pin y 3.3 V (ánodo al pin, cátodo a 3.3 V) para proteger contra transitorios inductivos.

### Alimentación

- LJ12A3: 6–36 V DC (alimentado desde 12 V o 24 V del vehículo).
- La señal NPN NO no necesita alimentación propia al pin del MCU; el pull-up del STM32 proporciona el nivel alto.

### Motivo técnico

- **6 pulsos/rev:** con circunferencia 1.1 m, a 25 km/h (6.94 m/s) se generan ~37.8 pulsos/s, suficiente resolución sin saturar la ISR.
- **Debounce 1 ms:** a frecuencia máxima (~38 Hz), el período es ~26 ms; 1 ms de debounce es <4 % del período, filtra rebotes sin perder pulsos.
- **EXTI (interrupciones):** captura exacta del instante del pulso sin polling, crítico para cálculo preciso de velocidad.

### Qué ocurre si falla

- **Sin pulsos (sensor desconectado o cable roto):** velocidad calculada = 0 km/h. Si los otros 3 sensores reportan movimiento, se detecta fallo de sensor individual.
- **Pulsos espurios (interferencia):** el debounce de 1 ms los filtra. Si la velocidad calculada supera 25 km/h, se descarta como lectura inválida.
- **Cortocircuito a GND permanente:** se interpreta como velocidad infinita → el filtro de velocidad máxima (25 km/h) lo detecta y marca como fallo.

### Esquema de conexión

```
  24V ───────────────┐
                     │
               ┌─────┴─────┐
               │  LJ12A3   │
               │  (NPN NO) │
               │            │
               │  Marrón=V+ │── 24V
               │  Azul=GND  │── GND
               │  Negro=OUT │──┐
               └────────────┘  │
                               │  (colector abierto, tira a GND)
                               │
  STM32 PAx ──────────────────┘
    (pull-up interno ~40kΩ a 3.3V)

  Protección opcional:
  PAx ──┤BAT54├── 3.3V  (clamp contra sobretensión)
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

### Tabla de pines

| Señal | Pin STM32 | Periférico | Nota |
|-------|-----------|------------|------|
| Salida | PB5 | EXTI5 | Flanco de subida, pull-up |

### Resistencias y protección

- **Pull-up interno:** activado en STM32 (~40 kΩ a 3.3 V). Mismo principio que los sensores de velocidad (NPN NO, colector abierto).
- **Diodo de clamp:** BAT54 entre PB5 y 3.3 V si la tensión del colector puede exceder 3.6 V.

### Alimentación

- LJ12A3: 6–36 V DC (desde 12 V o 24 V del vehículo).

### Motivo técnico

- **Sensor inductivo (no mecánico):** sin desgaste, sin contacto, funciona en entornos sucios/húmedos del chasis.
- **Detección de tornillo:** un tornillo de acero en la cremallera activa el sensor inductivo al pasar por el centro; solución robusta y barata.
- **EXTI5 con flanco de subida:** captura el instante exacto del paso por centro para resetear el contador del encoder de dirección.

### Qué ocurre si falla

- **Sensor desconectado:** la línea queda en alto por pull-up; no se genera interrupción de centro. El firmware no puede recalibrar el punto cero → se marca la calibración como no válida y se depende únicamente del encoder. El sistema entra en modo LIMP HOME si no hay calibración previa almacenada.
- **Activación permanente (cortocircuito a GND):** interrupciones constantes en EXTI5 → el firmware detecta frecuencia anómala y deshabilita la entrada, usando la última calibración conocida.

### Esquema de conexión

```
  24V ──────────────┐
                    │
              ┌─────┴─────┐
              │  LJ12A3   │
              │  (NPN NO) │
              │            │
              │  Marrón=V+ │── 24V
              │  Azul=GND  │── GND
              │  Negro=OUT │──┐
              └────────────┘  │
                              │ (colector abierto)
                              │
  STM32 PB5 (EXTI5) ─────────┘
    (pull-up interno ~40kΩ a 3.3V)
    Flanco de subida → ISR resetea contador TIM2

  Protección:
  PB5 ──┤BAT54├── 3.3V
```

---

## 7. TOFSense-M — Sensor LiDAR de Obstáculos (Nooploop)

Sensor LiDAR frontal 8×8 (Time-of-Flight) gestionado por el ESP32-S3 para detección de obstáculos con 5 zonas de distancia. Comunicación UART a 921600 bps, protocolo NLink_TOFSense_M_Frame0.

Referencia: [TOFSense-M User Manual V3.0](https://ftp.nooploop.com/downloads/tofsense/TOFSense-M_User_Manual_V3.0_en.pdf)

### Tabla de pines

| Señal | Pin ESP32-S3 | Dirección | Nota |
|-------|-------------|-----------|------|
| UART1 RX | GPIO18 | Entrada | Sensor TX → ESP32 RX (datos del sensor) |
| UART1 TX | — | — | No conectado (recepción unidireccional) |

### Conector del sensor (GH1.25 4 pines)

| Pin | Señal (modo UART) | Conexión |
|-----|-------------------|----------|
| 1 | VCC | **5 V** (obligatorio, el sensor no funciona a 3.3 V) |
| 2 | GND | GND común |
| 3 | RX | No conectado (recepción unidireccional) |
| 4 | TX | ESP32 GPIO18 (UART1 RX) vía divisor de tensión o level shifter BSS138 |

> **Nota sobre niveles lógicos:** Aunque el datasheet V3.0 indica UART TTL 3.3 V, las mediciones reales muestran **3.5–3.6 V** en el pin TX. El ESP32-S3 tiene un máximo absoluto de **3.6 V** en GPIO → se requiere **protección obligatoria**: divisor de tensión (R1=1 kΩ + R2=4.7 kΩ) o level shifter BSS138 (ver sección "Resistencias y protección" más abajo).  
> **Nota sobre alimentación:** El sensor requiere 5 V en VCC. Alimentar a 3.3 V provocará funcionamiento inestable o ausencia de datos (estado INVALID).

### Parámetros

| Parámetro | Valor |
|-----------|-------|
| Rango útil | 20 – 4000 mm |
| Frecuencia de salida | ~10 Hz (modo activo) |
| Velocidad UART | 921600 bps, 8N1 |
| Tamaño de trama | 400 bytes (NLink_TOFSense_M_Frame0) |
| Píxeles | 64 (8×8 matriz) |
| Timeout sin tramas | 500 ms → INVALID |

### Zonas de distancia

| Zona | Rango | Factor de escala de velocidad |
|------|-------|-------------------------------|
| Emergency | <500 mm | 0.0 (parada total) |
| Critical | 500–1000 mm | 0.3 |
| Warning | 1000–1500 mm | 0.7 |
| Caution | 1500–2000 mm | 0.85 |
| Alert | 2000–4000 mm | 0.95 |

### Resistencias y protección

- **UART RX (GPIO18):** ⚠️ **Protección obligatoria.** El TX del TOFSense-M emite 3.5–3.6 V (medido), por encima del 3.3 V nominal del datasheet. El ESP32-S3 tiene máx. absoluto de 3.6 V en GPIO. Elegir **una** de estas opciones:
  - **Opción 1 — Divisor de tensión:** R1=1 kΩ (serie) + R2=4.7 kΩ (a GND) → ~2.9 V en GPIO 18.
  - **Opción 2 — Level shifter BSS138:** módulo tipo SparkFun BOB-12009, Adafruit 757, o genérico "Logic Level Converter 3.3V–5V" → 3.3 V exactos en GPIO 18. ⚠️ NO usar TXS0108E (oscilaciones a 921600 bps).
- **Condensador de desacoplo:** 100 nF en VCC del TOFSense-M cerca del sensor.

### Alimentación

- TOFSense-M: **5 V obligatorio** (conector GH1.25 pin 1, consumo ~200 mA típico). El sensor no funciona a 3.3 V — la alimentación insuficiente causa estado INVALID en el firmware.

> **Referencia:** TOFSense-M Datasheet V3.0 (Nooploop): "Power Supply: 5V", "Communication Interface UART and CAN, TTL signal line level 3.3V".

### Motivo técnico

- **TOFSense-M:** sensor LiDAR 8×8 de alta precisión, mayor rango y fiabilidad que ultrasonido. No afectado por temperatura, viento ni interferencias acústicas.
- **5 zonas con factores de escala:** reducción progresiva de velocidad en vez de parada binaria, para una conducción más suave.
- **Mínima distancia de 64 píxeles:** se usa el valor mínimo de todos los píxeles válidos como distancia de referencia, proporcionando detección de obstáculos de campo amplio.
- **UART 921600 bps:** alto throughput necesario para los 400 bytes/trama a ~10 Hz. Compatible con UART1 del ESP32-S3.

### Qué ocurre si falla

- **Sin tramas UART (sensor desconectado):** tras 500 ms sin tramas válidas, el estado pasa a INVALID. El STM32 aplica factor conservador vía su timeout de CAN 0x208.
- **Checksum incorrecto:** la trama se descarta silenciosamente. El sensor sigue intentando recibir la siguiente trama.
- **Lecturas estáticas (sensor obstruido):** detección de sensor atascado — si la distancia no varía más de 10 mm durante 1000 ms mientras el vehículo se mueve, se marca como STUCK.

### Esquema de conexión

**Opción 1 — Con divisor de tensión:**

```
          TOFSense-M (GH1.25)
         ┌──────────────┐
  5V ────┤ VCC (pin 1)  │       ⚠ 5V obligatorio
         │              │
  GND ───┤ GND (pin 2)  │
         │              │
     n/c ┤ RX  (pin 3)  │
         │              │
         │ TX  (pin 4)  ├──┐     ⚠ TX medido = 3.5–3.6V
         └──────────────┘  │
                      ┌────┘
                      ├── R1=1kΩ ──┬── ESP32 GPIO18 (UART1 RX)
                      │            │
                      │       R2=4.7kΩ  (divisor de tensión)
                      │            │
                      │           GND

  Desacoplo: 100nF entre VCC y GND del TOFSense-M
```

**Opción 2 — Con level shifter BSS138 (alternativa sin resistencias):**

```
          TOFSense-M (GH1.25)         Level Shifter BSS138
         ┌──────────────┐            ┌──────────────────┐
  5V ────┤ VCC (pin 1)  ├────────────┤ HV (5V)          │
         │              │            │                  │
  GND ───┤ GND (pin 2)  ├────────────┤ GND              │
         │              │            │                  │    ESP32-S3
     n/c ┤ RX  (pin 3)  │            │       LV (3.3V) ─┼─── 3.3V
         │              │            │                  │
         │ TX  (pin 4)  ├────────────┤ HV1    LV1 ──────┼─── GPIO18
         └──────────────┘            └──────────────────┘    (UART1 RX)

  Desacoplo: 100nF entre VCC y GND del TOFSense-M
  ⚠️ Usar solo BSS138 (NO TXS0108E)
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

| Pin MCP23017 | Bit | Marcha |
|--------------|-----|--------|
| GPA0 | 0 | P (Park) |
| GPA1 | 1 | R (Reverse) |
| GPA2 | 2 | N (Neutral) |
| GPA3 | 3 | D1 (Drive 1) |
| GPA4 | 4 | D2 (Drive 2) |

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
                    │  GPA0 ───────┼──── SW P  ──── GND
                    │  GPA1 ───────┼──── SW R  ──── GND
                    │  GPA2 ───────┼──── SW N  ──── GND
                    │  GPA3 ───────┼──── SW D1 ──── GND
                    │  GPA4 ───────┼──── SW D2 ──── GND
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
La señal llega **aislada galvánicamente** a través del canal A7 de la placa PC817 de 8 canales.

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

- **1 kΩ ¼ W** entre +12 V ACC y ánodo del PC817:
  `I_LED = (12 V − 1.2 V) / 1 kΩ ≈ 10.8 mA` → seguro (<20 mA, ciclo de trabajo ~100%)
- **NO usar 330 Ω** (33 mA continuo → excede corriente nominal → degrada CTR)

### Alimentación

- La señal de contacto viene del circuito ACC (+12 V cuando llave ON).
- El lado de salida del PC817 usa 3.3 V del ESP32/STM32 (compartido).

### Qué ocurre si falla

- **Cable roto / desconectado (pin en HIGH):** el firmware ve llave OFF → SHUTTING_DOWN (fail-safe seguro).
- **Cortocircuito a GND (pin siempre LOW):** el sistema cree que la llave está siempre puesta. El apagado depende de timeout o lógica CAN del STM32.
- **Ruido en la línea de ignición:** el debounce de 50 ms en `power_manager.cpp` absorbe transitorios mecánicos. Añadir condensador 100 nF entre GPIO 40 y GND para filtrar picos EMI del arranque.

### Esquema de conexión

```
  +12V ACC (llave ON) ──[1 A]── 1 kΩ ──→ IN+ (ánodo LED PC817 ch A7)
  GND vehículo        ─────────────────→ IN- (cátodo LED)

  3.3V ──[10 kΩ ext.]──┬──→ OUT (colector PC817) ──→ GPIO 40 (INPUT_PULLUP)
                       │
                      GND (3.3V ESP32)

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
| PA0 | EXTI0 | Velocidad rueda FL |
| PA1 | EXTI1 | Velocidad rueda FR |
| PA2 | EXTI2 | Velocidad rueda RL |
| PB15 | EXTI15 | Velocidad rueda RR |
| PA3 | ADC1_IN4 | Pedal acelerador (primario) |
| PA15 | TIM2_CH1 | Encoder dirección A |
| PB3 | TIM2_CH2 | Encoder dirección B |
| PB4 | EXTI4 | Encoder dirección Z |
| PB5 | EXTI5 | Sensor centro dirección |

### ESP32-S3

| Pin | Función | Sensor/Periférico |
|-----|---------|-------------------|
| GPIO18 | UART1 RX | TOFSense-M LiDAR (Nooploop) |
| GPIO8 | I2C SDA | MCP23017 (shifter) |
| GPIO9 | I2C SCL | MCP23017 (shifter) |
| GPIO15 | Entrada pull-up | Interruptor 2WD/4WD |
| GPIO40 | Entrada INPUT_PULLUP | Sensor de contacto via PC817 (LOW=ON) |

### Dispositivos I2C

| Dispositivo | Dirección | Bus | MCU |
|-------------|-----------|-----|-----|
| TCA9548A | 0x70 | I2C1 (PB6/PB7) | STM32 |
| INA226 (×6) | 0x40 (via mux) | I2C1 (via TCA9548A) | STM32 |
| MCP23017 | 0x20 | I2C (GPIO8/9) | ESP32-S3 |
