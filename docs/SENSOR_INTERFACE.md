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
7. [HC-SR04 — Sensor Ultrasónico de Obstáculos](#7-hc-sr04--sensor-ultrasónico-de-obstáculos)
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
| 0 | Motor FL | 1 mΩ | 50 A |
| 1 | Motor FR | 1 mΩ | 50 A |
| 2 | Motor RL | 1 mΩ | 50 A |
| 3 | Motor RR | 1 mΩ | 50 A |
| 4 | Batería 24 V | 0.5 mΩ | 100 A |
| 5 | Dirección | 1 mΩ | 50 A |

### Resistencias y protección

- **Pull-ups I2C:** 4.7 kΩ a 3.3 V en SDA y SCL (una sola vez en el bus, no por sensor).
- **Shunt motores:** 1 mΩ — tolerancia ≤1 %, potencia ≥2.5 W (P = 50² × 0.001 = 2.5 W).
- **Shunt batería:** 0.5 mΩ — tolerancia ≤1 %, potencia ≥5 W (P = 100² × 0.0005 = 5 W).

### Alimentación

- INA226: 3.3 V desde el regulador del STM32.
- TCA9548A: 3.3 V.
- Lado de alta potencia (shunts): línea de bus 24 V de batería.

### Motivo técnico

- **400 kHz:** necesario para leer 6 sensores dentro de la ventana de control de 10 ms.
- **TCA9548A:** permite que todos los INA226 tengan la misma dirección 0x40, simplificando el PCB y el firmware.
- **Shunt 0.5 mΩ para batería:** reduce la caída de tensión a 50 mV @ 100 A (aceptable) frente a 100 mV con 1 mΩ.
- **Shunt 1 mΩ para motores:** maximiza la resolución del ADC del INA226 (LSB = 2.5 µV → 2.5 A/bit a 1 mΩ).

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
     │        ├─ Shunt (1 mΩ o 0.5 mΩ)
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

### Canal de plausibilidad — ADS1115 externo

| Señal | Pin STM32 | Periférico | Nota |
|-------|-----------|------------|------|
| SCL | (bus I2C compartido) | I2C | ADS1115 @ 0x48 |
| SDA | (bus I2C compartido) | I2C | ADDR → GND |
| Señal sin escalar | — | ADS1115 AIN0 | Lectura directa 0.3–4.8 V |

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

### Canal de plausibilidad (ADS1115)

| Parámetro | Valor |
|-----------|-------|
| Dirección I2C | 0x48 (ADDR a GND) |
| Entrada | AIN0, señal directa 0.3–4.8 V |
| PGA | ±6.144 V (cubre rango de 5 V completo) |
| Resolución | 16 bits |
| Cuentas válidas | 1600 – 25600 |
| Modo | Single-shot, 128 SPS |
| Tiempo de conversión | ~8 ms |

### Validación cruzada

| Parámetro | Valor |
|-----------|-------|
| Tolerancia | ±5 % entre canales (tras normalizar a %) |
| Timeout de divergencia | 200 ms (si divergen >5 % durante 200 ms → fallo) |
| Timeout de dato obsoleto | 500 ms (si no se recibe lectura nueva en 500 ms → fallo) |

### Resistencias y protección

- **Divisor resistivo:** 10 kΩ (serie) + 6.8 kΩ (a GND). Precisión ≥1 % para mantener el ratio calibrado.
- **Condensador anti-ruido:** 100 nF cerámico en PA3 a GND (filtro RC con 6.8 kΩ → fc ≈ 234 Hz, suficiente para señal de pedal <10 Hz).
- **ADS1115:** alimentado a 3.3 V; la entrada AIN0 tolera hasta VDD+0.3 V, pero con PGA ±6.144 V la medida es válida hasta 4.8 V sin saturar (la lectura digital escala correctamente aunque VDD=3.3 V, gracias al PGA interno).

### Alimentación

- Sensor Hall SS1324LUA-T: **5 V** regulado.
- Divisor resistivo: sin alimentación propia (pasivo).
- ADS1115: **3.3 V** (VDD), entrada analógica recibe señal de 5 V directamente (permitido con PGA ±6.144 V).

### Motivo técnico

- **Doble canal:** normativa de seguridad funcional (ISO 26262 inspirado); un solo canal de pedal es un punto único de fallo inaceptable.
- **Divisor 10k+6.8k:** escala la señal de 5 V a <2 V para el ADC de 3.3 V con margen. Valores altos minimizan la corriente drenada del sensor (~0.3 mA).
- **ADS1115 con PGA ±6.144V:** permite leer la señal completa de 5 V sin divisor, proporcionando una referencia independiente del hardware del canal primario.
- **47.5 ciclos de muestreo:** compromiso entre velocidad y precisión; la impedancia de fuente (~4 kΩ del divisor) requiere ≥30 ciclos para carga del sample-and-hold.

### Qué ocurre si falla

- **Canal primario fuera de rango (<150 o >2413 cuentas):** se interpreta como cable roto o cortocircuito → acelerador forzado a 0 %.
- **ADS1115 sin respuesta I2C:** se pierde la validación cruzada → el sistema reduce potencia máxima al 50 % y emite alerta.
- **Divergencia >5 % durante >200 ms:** se asume fallo de un canal → acelerador forzado a 0 % hasta que los canales converjan.
- **Dato obsoleto >500 ms:** el firmware fuerza acelerador a 0 % (pérdida de señal).

### Esquema de conexión

```
               Sensor Hall SS1324LUA-T
              ┌────────────────────┐
   5V ────────┤ VCC            OUT ├───┬──────────────────────┐
              │                    │   │                      │
   GND ───────┤ GND                │   │ Señal 0.3–4.8V      │
              └────────────────────┘   │                      │
                                       │                      │
          Canal Primario (STM32)       │   Canal Plausibilidad
                                       │        (ADS1115)
                                       │                      │
                              ┌────────┤                      │
                              │        │               ┌──────┴──────┐
                           10kΩ (1%)   │               │  ADS1115    │
                              │        │               │  (0x48)     │
  STM32 PA3 (ADC1_IN4) ──────┤        └───────────────┤ AIN0        │
                              │                        │             │
                           6.8kΩ (1%)                  │ VDD── 3.3V  │
                              │                        │ GND── GND   │
                             GND                       │ SCL── I2C   │
                                                       │ SDA── I2C   │
                              │                        │ ADDR── GND  │
                          100nF (C)                    └─────────────┘
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

- **Level shifter 5 V → 3.3 V:** obligatorio. El E6B2-CWZ6C tiene salida de 5 V (line driver o open-collector según versión).
  - **Opción 1 — BSS138 MOSFET:** level shifter bidireccional con pull-ups de 10 kΩ a cada lado (3.3 V y 5 V). Se necesitan 3 canales (A, B, Z).
  - **Opción 2 — Divisor resistivo:** 10 kΩ (serie) + 15 kΩ (a GND), ratio 0.6 → 5 V × 0.6 = 3.0 V. Más simple pero añade impedancia.
- **Filtro fDTS (hardware TIM2):** 6 muestras consecutivas a frecuencia de reloj del timer; rechaza pulsos <210 ns (ruido electromagnético de los motores).

### Alimentación

- Encoder E6B2-CWZ6C: **5 V** DC (consumo ~80 mA con line driver).
- Level shifters: 3.3 V y 5 V.

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
             Encoder E6B2-CWZ6C
            ┌──────────────────┐
   5V ──────┤ VCC (marrón)     │
   GND ─────┤ GND (azul)      │
            │                  │
            │ Fase A (blanco) ─┤──┐
            │ Fase B (negro) ──┤──┼──┐
            │ Z (naranja) ─────┤──┼──┼──┐
            └──────────────────┘  │  │  │
                                  │  │  │
         Level Shift 5V→3.3V     │  │  │
         (BSS138 × 3 canales)    │  │  │
              ┌───────┐          │  │  │
   5V─┤10kΩ├─┤       ├─┤10kΩ├─3.3V  │  │
              │BSS138 │          │  │  │
   Encoder A──┤S    D├──────── PA15 (TIM2_CH1)
              └───────┘             │  │
              ┌───────┐             │  │
   Encoder B──┤BSS138 ├──────── PB3  (TIM2_CH2)
              └───────┘                │
              ┌───────┐                │
   Encoder Z──┤BSS138 ├──────── PB4  (EXTI4)
              └───────┘
```

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

## 7. HC-SR04 — Sensor Ultrasónico de Obstáculos

Sensor ultrasónico frontal gestionado por el ESP32-S3 para detección de obstáculos con 5 zonas de distancia.

### Tabla de pines

| Señal | Pin ESP32-S3 | Dirección | Nota |
|-------|-------------|-----------|------|
| TRIG | GPIO6 | Salida | Pulso de disparo 10 µs |
| ECHO | GPIO7 | Entrada | Ancho de pulso proporcional a distancia |

### Parámetros

| Parámetro | Valor |
|-----------|-------|
| Rango útil | 20 – 4000 mm |
| Frecuencia de muestreo | ≥20 Hz |
| Retardo de confirmación | 200 ms (evita falsas alarmas) |
| Retardo de despeje | 1000 ms (antes de subir nivel de velocidad) |

### Zonas de distancia

| Zona | Rango | Factor de escala de velocidad |
|------|-------|-------------------------------|
| Emergency | <200 mm | 0.0 (parada total) |
| Critical | 200–500 mm | 0.3 |
| Warning | 500–1000 mm | 0.7 |
| Caution | 1000–1500 mm | 0.85 |
| Alert | 1500–4000 mm | 0.95 |

### Resistencias y protección

- **TRIG:** sin resistencia adicional; GPIO6 del ESP32-S3 en push-pull 3.3 V. El HC-SR04 acepta TRIG a nivel lógico ≥2.0 V (TTL compatible con 3.3 V).
- **ECHO:** el HC-SR04 genera un pulso a **5 V**. El ESP32-S3 **no es tolerante a 5 V**. Requiere divisor resistivo:
  - 10 kΩ (serie desde ECHO) + 20 kΩ (a GND) → ratio 20/(10+20) = 0.667 → 5 V × 0.667 = 3.33 V.
  - O usar level shifter BSS138.
- **Condensador de desacoplo:** 100 nF en VCC del HC-SR04 cerca del sensor.

### Alimentación

- HC-SR04: **5 V** (consumo ~15 mA activo, ~2 mA standby).

### Motivo técnico

- **HC-SR04:** sensor barato, ampliamente disponible, rango suficiente (hasta 4 m) para un vehículo de baja velocidad (≤25 km/h).
- **5 zonas con factores de escala:** reducción progresiva de velocidad en vez de parada binaria, para una conducción más suave.
- **Confirmación 200 ms / despeje 1000 ms:** histéresis temporal asimétrica — se frena rápido (200 ms) pero se acelera lento (1000 ms) para seguridad.
- **≥20 Hz de muestreo:** a 25 km/h (6.94 m/s), en 50 ms se recorren 0.35 m; 20 Hz (50 ms) es suficiente para reaccionar dentro de la zona de alerta.

### Qué ocurre si falla

- **Sin eco (sensor desconectado o bloqueado):** se interpreta como obstáculo a distancia desconocida → el sistema aplica factor de escala conservador (0.3, zona Critical).
- **Eco permanente (sensor defectuoso):** distancia calculada = 0 → zona Emergency (parada). El firmware detecta lecturas estáticas durante >5 s y emite alerta de sensor fallido.
- **Lecturas erráticas (interferencia ultrasónica):** el retardo de confirmación de 200 ms filtra picos aislados.

### Esquema de conexión

```
              HC-SR04
         ┌──────────────┐
  5V ────┤ VCC      TRIG├──────────── ESP32 GPIO6
         │              │
  GND ───┤ GND      ECHO├──┐
         └──────────────┘  │
                           │  5V señal
                        10kΩ
                           │
  ESP32 GPIO7 ─────────────┤
                           │
                        20kΩ
                           │
                          GND

  Tensión en GPIO7: 5V × 20k/(10k+20k) = 3.33V ✓

  Desacoplo: 100nF entre VCC y GND del HC-SR04
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

### Tabla de pines

| Señal | Pin ESP32-S3 | Configuración | Nota |
|-------|-------------|---------------|------|
| Contacto | GPIO40 | INPUT_PULLDOWN | HIGH = Llave ON |

### Resistencias y protección

- **Pull-down interno:** activado en el ESP32-S3 (~45 kΩ a GND). Cuando la llave está desconectada, GPIO40 = LOW.
- **Señal de contacto:** la llave de contacto del vehículo proporciona 3.3 V (o señal regulada a 3.3 V desde un divisor resistivo si la fuente es 12/24 V).
- **Si la señal de contacto proviene de 12 V:** usar divisor 10 kΩ + 4.7 kΩ (ratio 4.7/14.7 = 0.32 → 12 V × 0.32 = 3.84 V; ajustar a 10 kΩ + 3.9 kΩ para 3.36 V) o regulador LDO.
- **Diodo de protección:** Zener de 3.3 V en GPIO40 a GND (cátodo a GPIO40) para proteger contra sobretensión del circuito de ignición.

### Alimentación

- La señal de contacto viene del circuito de llave del vehículo.
- No alimenta nada; es solo una señal de lectura.

### Motivo técnico

- **INPUT_PULLDOWN (no pull-up):** semántica natural — sin llave la entrada está en LOW (sistema apagado); con llave se suministra un HIGH activo.
- **GPIO40:** pin disponible en el ESP32-S3 sin conflicto con otros periféricos asignados (pantalla, CAN, I2C).
- **Detección por ESP32-S3 (no STM32):** el ESP32-S3 gestiona el HMI y la secuencia de encendido; necesita saber primero si hay contacto para inicializar la pantalla y comunicarse con el STM32.

### Qué ocurre si falla

- **Cable roto (pin siempre LOW):** el sistema no arranca (fail-safe). El ESP32-S3 interpreta como llave OFF y no habilita la secuencia de encendido.
- **Cortocircuito a alto (pin siempre HIGH):** el sistema cree que la llave está siempre puesta. El apagado dependerá de un timeout o de la lógica de parada por CAN desde el STM32.
- **Ruido en la línea de ignición:** picos de tensión del arranque del motor → el Zener de 3.3 V y un filtro RC (10 kΩ + 100 nF, fc ~159 Hz) protegen el GPIO y eliminan transitorios.

### Esquema de conexión

```
  Circuito de llave del vehículo
         │
         │  12V (cuando llave ON)
         │
      10kΩ (divisor)
         │
  ESP32 GPIO40 ──────────┤
  (INPUT_PULLDOWN)        │
                       3.9kΩ (a GND)
                          │
                         GND

  Protección:
  GPIO40 ──┤Zener 3.3V├── GND
            (cátodo a GPIO40)

  Filtro anti-transitorio:
  Divisor ──┤10kΩ├──┬── GPIO40
                    │
                  100nF
                    │
                   GND

  HIGH = Llave ON (~3.36V)
  LOW  = Llave OFF (0V, pull-down)
```

---

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
| GPIO6 | Salida TRIG | HC-SR04 ultrasónico |
| GPIO7 | Entrada ECHO | HC-SR04 ultrasónico |
| GPIO8 | I2C SDA | MCP23017 (shifter) |
| GPIO9 | I2C SCL | MCP23017 (shifter) |
| GPIO15 | Entrada pull-up | Interruptor 2WD/4WD |
| GPIO40 | Entrada pull-down | Sensor de contacto |

### Dispositivos I2C

| Dispositivo | Dirección | Bus | MCU |
|-------------|-----------|-----|-----|
| TCA9548A | 0x70 | I2C1 (PB6/PB7) | STM32 |
| INA226 (×6) | 0x40 (via mux) | I2C1 (via TCA9548A) | STM32 |
| ADS1115 | 0x48 | I2C (STM32) | STM32 |
| MCP23017 | 0x20 | I2C (GPIO8/9) | ESP32-S3 |
