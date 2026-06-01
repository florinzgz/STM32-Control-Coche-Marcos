# 🔬 Guía del Analizador Lógico — Cómo Comprobar Cada Sensor

> **Para quién es esta guía:** Para Marcos (o cualquier persona que esté aprendiendo).
> Explicado paso a paso, como si nunca hubieras usado un analizador lógico.
>
> **Referencia de firmware:** Todos los pines y direcciones provienen de
> `Core/Inc/project_config.h` y `Core/Src/sensor_manager.c`.

---

## 📋 Índice

1. [¿Qué es un analizador lógico?](#1-qué-es-un-analizador-lógico)
2. [Reglas de seguridad](#2-reglas-de-seguridad)
3. [Dónde encontrar los pines en la placa Nucleo](#3-dónde-encontrar-los-pines-en-la-placa-nucleo)
4. [Prueba 1 — I2C: TCA9548A + INA226 (corriente y tensión)](#prueba-1--i2c-tca9548a--ina226)
5. [Prueba 2 — OneWire: DS18B20 (temperatura)](#prueba-2--onewire-ds18b20-temperatura)
6. [Prueba 3 — Encoder E6B2-CWZ6C (dirección)](#prueba-3--encoder-e6b2-cwz6c-dirección)
7. [Prueba 4 — PWM de motores (BTS7960)](#prueba-4--pwm-de-motores-bts7960)
8. [Prueba 5 — Sensores de velocidad de rueda](#prueba-5--sensores-de-velocidad-de-rueda)
9. [Prueba 6 — CAN Bus (FDCAN1)](#prueba-6--can-bus-fdcan1)
10. [Prueba 7 — Pedal acelerador (ADC)](#prueba-7--pedal-acelerador-adc)
11. [Orden recomendado de pruebas](#orden-recomendado-de-pruebas)
12. [Solución de problemas comunes](#solución-de-problemas-comunes)

---

## 1. ¿Qué es un analizador lógico?

Imagina que es un "osciloscopio digital barato" que **graba** las señales eléctricas
de los cables y te las muestra en la pantalla del ordenador.

- **Solo mide señales digitales** (HIGH = 3.3V, LOW = 0V).
- **No inyecta corriente** → no rompe nada si lo conectas bien.
- Se usa con el programa **PulseView** (gratuito) o **Saleae Logic**.

### Piezas que necesitas

| Pieza | Para qué |
|-------|----------|
| Analizador lógico (8 canales mínimo) | Capturar señales |
| Cables dupont hembra-hembra | Conectar pines |
| PulseView instalado en el PC | Ver y decodificar señales |
| Cable USB del analizador | Conectar al PC |

---

## 2. Reglas de seguridad

> ⚠️ **LEE ESTO ANTES DE TOCAR NADA**

1. **SIEMPRE** conecta primero el cable **GND** (tierra) del analizador
   a un pin **GND** de la Nucleo. Si no, las lecturas serán basura.
2. **NUNCA** conectes el analizador a señales de más de **5V**.
   La Nucleo G474RE trabaja a **3.3V** → es seguro.
3. **NUNCA** conectes las puntas del analizador entre sí con la alimentación encendida.
4. Conecta los cables **con la placa APAGADA**. Enciende después.
5. Si ves humo o hueles algo raro → **desconecta inmediatamente**.

---

## 3. Dónde encontrar los pines en la placa Nucleo

La placa NUCLEO-G474RE tiene dos filas de pines a cada lado llamadas **Morpho connectors**:

```
        CN7 (izquierda)              CN10 (derecha)
        ┌─────────────┐              ┌─────────────┐
  pin 1 │ o         o │ pin 2  pin 1 │ o         o │ pin 2
  pin 3 │ o         o │ pin 4  pin 3 │ o         o │ pin 4
        │   ...       │              │   ...       │
 pin 37 │ o         o │ pin 38 pin 37│ o         o │ pin 38
        └─────────────┘              └─────────────┘
```

### Pines que usarás en esta guía

| Pin STM32 | Función en el coche | Conector Morpho |
|-----------|---------------------|-----------------|
| **PB8** | I2C1_SCL (reloj) | CN10 pin 3 |
| **PB9** | I2C1_SDA (datos) | CN10 pin 5 |
| **PB0** | OneWire (temperatura) | CN7 pin 34 |
| **PA15** | Encoder Fase A | CN7 pin 38 |
| **PB3** | Encoder Fase B | CN10 pin 31 |
| **PB4** | Encoder Fase Z (índice) | CN10 pin 27 |
| **PA0** | Rueda FL (velocidad) | CN7 pin 28 |
| **PA1** | Rueda FR (velocidad) | CN7 pin 30 |
| **PA2** | Rueda RL (velocidad) | CN7 pin 32 |
| **PB15** | Rueda RR (velocidad) | CN10 pin 26 |
| **PA12** | CAN TX | CN10 pin 12 |
| **PA11** | CAN RX | CN10 pin 14 |
| **GND** | Tierra (¡SIEMPRE conectar!) | CN7 pin 8, CN10 pin 9, y otros marcados GND |

---

## Prueba 1 — I2C: TCA9548A + INA226

### ¿Qué estamos comprobando?

El bus I2C conecta el STM32 con:
- **1× TCA9548A** (multiplexor) → dirección `0x70`
- **6× INA226** (sensores de corriente/tensión) → dirección `0x40` cada uno

El TCA9548A es como un **interruptor con 8 salidas**: el STM32 le dice
"abre el canal 3" y entonces puede hablar con el INA226 que está en ese canal.

```
                    ┌─────────────┐
  STM32 ──I2C──────┤  TCA9548A   ├──Canal 0──→ INA226 (motor FL)
  PB8=SCL          │  (0x70)     ├──Canal 1──→ INA226 (motor FR)
  PB9=SDA          │             ├──Canal 2──→ INA226 (motor RL)
                   │             ├──Canal 3──→ INA226 (motor RR)
                   │             ├──Canal 4──→ INA226 (batería 24V)
                   │             ├──Canal 5──→ INA226 (dirección)
                   │             ├──Canal 6──→ (libre)
                   │             ├──Canal 7──→ (libre)
                   └─────────────┘
```

### Conexión del analizador

```
ANALIZADOR          NUCLEO
──────────          ──────
  CH0  ──────────→  PB8  (SCL)    Morpho CN10 pin 3
  CH1  ──────────→  PB9  (SDA)    Morpho CN10 pin 5
  GND  ──────────→  GND           Morpho CN7  pin 8
```

**Solo 3 cables.** Así de sencillo.

### Configuración en PulseView

1. Abre PulseView
2. **Sample rate:** `4 MHz` (el I2C va a 400 kHz, necesitamos al menos 10× más)
3. **Samples:** `2M` (2 millones — captura ~0.5 segundos)
4. Pulsa el botón **Run** (capturar)
5. Ve a **Decode → I2C**
6. Configura: SCL = CH0, SDA = CH1

### ¿Qué verás si funciona?

Al encender el STM32, el firmware hace esto automáticamente:

**Paso 1 — Comprobar que el TCA9548A existe:**
```
[START] [0xE0] [ACK] [STOP]
         │
         └─→ 0x70 × 2 = 0xE0 (dirección del TCA en modo escritura)
```

Si ves **ACK** (reconocimiento) → ✅ El multiplexor responde.
Si ves **NACK** → ❌ El multiplexor NO responde (ver solución de problemas).

**Paso 2 — Seleccionar un canal (ejemplo: canal 0):**
```
[START] [0xE0] [ACK] [0x01] [ACK] [STOP]
                      │
                      └─→ 0x01 = bit 0 activado = canal 0
```

Los valores para cada canal:
| Canal | Byte que se envía | INA226 conectado |
|-------|-------------------|------------------|
| 0 | `0x01` | Motor FL |
| 1 | `0x02` | Motor FR |
| 2 | `0x04` | Motor RL |
| 3 | `0x08` | Motor RR |
| 4 | `0x10` | Batería 24V |
| 5 | `0x20` | Dirección |

**Paso 3 — Leer el INA226 en ese canal:**
```
[START] [0x80] [ACK] [0x01] [ACK] [RSTART] [0x81] [ACK] [MSB] [ACK] [LSB] [NACK] [STOP]
         │            │                      │             │            │
         │            │                      │             └────────────└─→ Datos (16 bits)
         │            └─→ Registro 0x01 = tensión shunt (corriente)
         └─→ 0x40 × 2 = 0x80 (dirección INA226 en modo escritura)
                         0x81 = modo lectura
```

### Registros del INA226 que lee el firmware

| Registro | Dirección | Qué mide |
|----------|-----------|----------|
| Config | `0x00` | Configuración (el firmware escribe `0x4327`) |
| Shunt Voltage | `0x01` | Caída de tensión en la resistencia shunt → se convierte a amperios |
| Bus Voltage | `0x02` | Tensión del bus (voltios de la batería o motor) |

### Secuencia completa que verás (6 sensores)

```
Para cada canal i = 0, 1, 2, 3, 4, 5:
  1. START → 0xE0 → (1<<i) → STOP           ← Seleccionar canal del TCA9548A
  2. START → 0x80 → 0x01 → RSTART → 0x81 → MSB → LSB → STOP   ← Leer shunt
  3. START → 0x80 → 0x02 → RSTART → 0x81 → MSB → LSB → STOP   ← Leer bus voltage
```

Esta secuencia se repite cada **50 ms** (20 veces por segundo).

### ❌ Problemas y soluciones

| Lo que ves en el analizador | Significado | Qué hacer |
|----|----|----|
| **NACK después de 0xE0** | El TCA9548A no responde | 1. Verifica que el TCA tiene alimentación 3.3V<br>2. Comprueba que hay resistencias pull-up (4.7kΩ a 3.3V) en SDA y SCL<br>3. Revisa las soldaduras |
| **ACK en 0xE0 pero NACK en 0x80** | El TCA responde pero el INA226 no | El canal del mux funciona, pero el INA226 de ese canal tiene un problema. Revisa alimentación y soldaduras del INA |
| **SCL se queda siempre LOW** | Bus I2C bloqueado | El firmware tiene recuperación automática (16 pulsos en SCL). Si persiste, apaga y enciende todo |
| **Datos siempre 0x0000** | INA226 responde pero no mide nada | No hay corriente pasando por el shunt, o el shunt no está soldado |
| **Solo un canal funciona** | Solo un INA226 está bien conectado | Verifica los demás uno a uno cambiando el canal |

---

## Prueba 2 — OneWire: DS18B20 (temperatura)

### ¿Qué estamos comprobando?

Los 5 sensores de temperatura DS18B20 comparten **un solo cable de datos** (PB0).
El protocolo se llama **OneWire** (1-Wire): un solo cable para todo.

```
                4.7kΩ
   3.3V ──────┤├──────┬──── PB0 (STM32)
                      │
              ┌───────┼───────┐
              │       │       │
           DS18B20 DS18B20 DS18B20 ...  (hasta 5)
              │       │       │
             GND     GND     GND
```

> **Importante:** La resistencia de 4.7kΩ entre 3.3V y el cable de datos
> es **obligatoria**. Sin ella, nada funciona.

### Conexión del analizador

```
ANALIZADOR          NUCLEO
──────────          ──────
  CH0  ──────────→  PB0  (OneWire)    Morpho CN7 pin 34
  GND  ──────────→  GND               Morpho CN7 pin 8
```

**Solo 2 cables.**

### Configuración en PulseView

1. **Sample rate:** `1 MHz` (los pulsos OneWire duran 6–480 µs)
2. **Samples:** `5M` (5 millones — captura ~5 segundos, suficiente para ver un ciclo completo)
3. **Decode → 1-Wire link layer** → OWR = CH0
4. Opcionalmente añade encima: **1-Wire transport** y **DS18B20**

### ¿Qué verás si funciona?

**Paso 1 — Reset (el STM32 "llama a la puerta"):**

```
        El STM32 tira         Suelta        El DS18B20
        la línea a LOW        la línea      responde con LOW
              │                  │                │
Señal: ───┐  │                  │  ┌──────┐      │     ┌────────
          │  │← 480 µs LOW →   │  │      │      │     │
          └──┘                  │  │      └──────┘     │
                                │  │                   │
                         Espera 70 µs       60-240 µs LOW
                                              (presencia)
```

- Si el DS18B20 está **conectado**: la línea baja brevemente (pulso de presencia) ✅
- Si **NO** hay sensor: la línea se queda HIGH todo el rato ❌

**Paso 2 — Comandos que envía el firmware:**

| Comando | Byte | ¿Qué hace? |
|---------|------|------------|
| Search ROM | `0xF0` | Busca las direcciones de todos los sensores (al arrancar) |
| Skip ROM | `0xCC` | "Le hablo a todos a la vez" |
| Match ROM | `0x55` + 8 bytes | "Le hablo a un sensor específico" |
| Convert T | `0x44` | "Empieza a medir la temperatura" |
| Read Scratchpad | `0xBE` | "Dime la temperatura que has medido" |

**Secuencia completa de lectura:**

```
1. Reset → Presencia → 0xCC (Skip ROM) → 0x44 (Convert T)
2. Esperar 750 ms (tiempo de conversión)
3. Reset → Presencia → 0x55 (Match ROM) + 8 bytes dirección → 0xBE (Read Scratchpad)
4. Leer 9 bytes → Los 2 primeros son la temperatura
```

### Cómo interpretar la temperatura

Los dos primeros bytes del scratchpad son la temperatura en formato "complemento a 2 con 4 bits decimales":

```
Ejemplo: LSB = 0x90, MSB = 0x01
  Valor = 0x0190 = 400 en decimal
  Temperatura = 400 / 16 = 25.0 °C ✅

Ejemplo: LSB = 0x50, MSB = 0x05
  Valor = 0x0550 = 1360 en decimal
  Temperatura = 1360 / 16 = 85.0 °C ← ¡CUIDADO! Este es el valor por defecto.
  Significa que la conversión NO se completó.
```

### ❌ Problemas y soluciones

| Lo que ves | Significado | Qué hacer |
|----|----|----|
| **Reset sin pulso de presencia** (línea siempre HIGH) | Ningún sensor detectado | 1. ¿Está la resistencia de 4.7kΩ?<br>2. ¿Tiene el DS18B20 alimentación? (VDD a 3.3V, GND a GND)<br>3. ¿Está bien soldado? |
| **Línea siempre LOW** | Cortocircuito | Desconecta todo y comprueba cable por cable |
| **Temperatura = 85.0 °C siempre** | Conversión no terminada | El STM32 lee antes de que pasen 750 ms. Problema de software (raro, el firmware lo gestiona) |
| **CRC error** | Datos corruptos | Ruido eléctrico. Acorta los cables, aleja de motores |
| **Solo detecta 3 de 5 sensores** | Algunos sensores con mal contacto | Comprueba soldaduras de cada sensor individualmente |

---

## Prueba 3 — Encoder E6B2-CWZ6C (dirección)

### ¿Qué estamos comprobando?

El encoder rotativo mide el **ángulo del volante**. Tiene 3 señales:

```
  ENCODER E6B2-CWZ6C
  ┌──────────────────┐
  │                  │
  │  Fase A ─────────├───→ PA15 (TIM2_CH1)    Morpho CN7 pin 38
  │  Fase B ─────────├───→ PB3  (TIM2_CH2)    Morpho CN10 pin 31
  │  Fase Z ─────────├───→ PB4  (GPIO input)   Morpho CN10 pin 27
  │                  │
  │  VCC (marrón) ───├───→ 5V
  │  GND (azul)  ────├───→ GND
  │  Blindaje ───────├───→ GND
  └──────────────────┘
```

- **Fase A y Fase B:** Dos señales cuadradas desfasadas 90°. El STM32 las cuenta con TIM2 en modo cuadratura.
- **Fase Z:** Un pulso por vuelta completa (índice). Se lee por GPIO.
- **Resolución:** 1200 PPR × 4 (cuadratura) = **4800 counts por vuelta** = 0.075° por count

### Conexión del analizador

```
ANALIZADOR          NUCLEO
──────────          ──────
  CH0  ──────────→  PA15 (Fase A)    Morpho CN7  pin 38
  CH1  ──────────→  PB3  (Fase B)    Morpho CN10 pin 31
  CH2  ──────────→  PB4  (Fase Z)    Morpho CN10 pin 27
  GND  ──────────→  GND              Morpho CN7  pin 8
```

**4 cables.**

### Configuración en PulseView

1. **Sample rate:** `500 kHz` (a velocidad lenta del volante, la frecuencia es ~1200 Hz)
2. **Samples:** `2M`
3. Si PulseView tiene decoder **Encoder**, úsalo. Si no, mira las señales a ojo.

### ¿Qué verás si funciona?

**Volante parado (sin mover):**
```
Fase A: ──────────────────────── (nivel constante, HIGH o LOW)
Fase B: ──────────────────────── (nivel constante)
Fase Z: ──────────────────────── (nivel constante)
```

**Giro a la derecha (sentido horario):**
```
Fase A: ──┐  ┌──┐  ┌──┐  ┌──
          └──┘  └──┘  └──┘

Fase B: ───┐  ┌──┐  ┌──┐  ┌─    ← B va RETRASADA 90° respecto a A
           └──┘  └──┘  └──┘
```

**Giro a la izquierda (sentido antihorario):**
```
Fase A: ──┐  ┌──┐  ┌──┐  ┌──
          └──┘  └──┘  └──┘

Fase B: ─┐  ┌──┐  ┌──┐  ┌───    ← B va ADELANTADA 90° respecto a A
         └──┘  └──┘  └──┘
```

**Pulso Z (índice) — una vez por vuelta completa:**
```
Fase Z: ──────────────────┐    ┌──────────────────
                          └────┘
                          ↑
                     Un pulso por revolución
```

### Frecuencias esperadas

| Velocidad de giro | Frecuencia por canal | Período |
|----|----|----|
| Muy lento (~0.5 rev/s) | 600 Hz | ~1.67 ms |
| Lento (~1 rev/s) | 1200 Hz | ~833 µs |
| Rápido (~3 rev/s) | 3600 Hz | ~278 µs |

### ❌ Problemas y soluciones

| Lo que ves | Significado | Qué hacer |
|----|----|----|
| **A y B siempre en el mismo nivel** | Encoder no conectado o sin alimentación | Verifica que el encoder recibe **5V** (cable marrón) y GND (cable azul) |
| **Señal con muchos glitches (picos rápidos)** | Ruido EMI de los motores | Usa cable **apantallado** (el blindaje del encoder va a GND). Aleja cables de los motores |
| **B siempre copia a A (sin desfase)** | Cable de B suelto o cortado | Revisa las soldaduras del cable de Fase B |
| **Z nunca pulsa** | Cable Z desconectado | Verifica el cable de Fase Z y que PB4 está como input |
| **Cuenta salta de golpe** | Interferencia o mal contacto | El firmware detecta esto como `Encoder_HasFault()` |

---

## Prueba 4 — PWM de motores (BTS7960)

### ¿Qué estamos comprobando?

Cada motor tiene 3 señales: **RPWM** (adelante), **LPWM** (atrás), **EN** (habilitar).

```
  STM32          BTS7960 (IBT-2)           MOTOR
  ─────          ───────────────           ─────
  RPWM ────────→ RPWM (adelante) ────┐
  LPWM ────────→ LPWM (atrás)   ────┼───→ Motor DC
  EN   ────────→ EN (habilitar)  ────┘
```

> **Regla de oro:** RPWM y LPWM **NUNCA** deben estar activas al mismo tiempo.
> El firmware lo previene automáticamente.

### Tabla de pines de todos los motores

| Motor | RPWM | LPWM | EN | Timer |
|-------|------|------|----|-------|
| **FL** (delantero izq.) | PA8 (TIM1_CH1) | PA9 (TIM1_CH2) | PC5 | TIM1 |
| **FR** (delantero der.) | PA10 (TIM1_CH3) | PC3 (TIM1_CH4) | PC0 | TIM1 |
| **RL** (trasero izq.) | PC6 (TIM8_CH1) | PC7 (TIM8_CH2) | PC1 | TIM8 |
| **RR** (trasero der.) | PC8 (TIM8_CH3) | PC9 (TIM8_CH4) | PC13 | TIM8 |
| **Dirección** | PA6 (TIM3_CH1) | PA7 (TIM3_CH2) | PC4 | TIM3 |

### Conexión del analizador (ejemplo: motor de dirección)

```
ANALIZADOR          NUCLEO
──────────          ──────
  CH0  ──────────→  PA6  (RPWM Steer)
  CH1  ──────────→  PA7  (LPWM Steer)
  CH2  ──────────→  PC4  (EN Steer)
  GND  ──────────→  GND
```

### Configuración en PulseView

1. **Sample rate:** `1 MHz`
2. **Samples:** `2M`
3. No necesitas decoder — mira las señales directamente

### ¿Qué verás si funciona?

**Motor parado (EN = HIGH, PWM = 0%):**
```
RPWM: ───────────────────── LOW
LPWM: ───────────────────── LOW
EN:   ───────────────────── HIGH
```

**Motor adelante al 50%:**
```
RPWM: ──┐  ┌──┐  ┌──┐  ┌──    ← PWM al 50% (mitad HIGH, mitad LOW)
        └──┘  └──┘  └──┘
LPWM: ───────────────────── LOW  ← Siempre LOW cuando va adelante
EN:   ───────────────────── HIGH
```

**Motor atrás al 30%:**
```
RPWM: ───────────────────── LOW  ← Siempre LOW cuando va atrás
LPWM: ─┐ ┌────┐ ┌────┐ ┌───    ← PWM al 30% (30% HIGH, 70% LOW)
        └─┘    └─┘    └─┘
EN:   ───────────────────── HIGH
```

### ❌ Problemas y soluciones

| Lo que ves | Significado | Qué hacer |
|----|----|----|
| **RPWM y LPWM activas al mismo tiempo** | ¡PELIGRO! Cortocircuito en el puente H | Apaga inmediatamente. Problema de software (no debería pasar, el firmware lo previene) |
| **EN siempre LOW** | Motor deshabilitado | El firmware desactiva EN cuando está en modo SAFE o durante el arranque |
| **PWM con duty 100% constante** | Motor a máxima velocidad sin control | Problema de software o cable de retroalimentación desconectado |
| **Sin actividad en ningún canal** | Motor no inicializado | Normal durante el arranque. Verifica que el sistema ha pasado el boot |

---

## Prueba 5 — Sensores de velocidad de rueda

### ¿Qué estamos comprobando?

Cada rueda tiene un **sensor inductivo LJ12A3** que detecta 6 tornillos por vuelta.
Genera un pulso cada vez que pasa un tornillo.

```
  Rueda girando
  ┌───────────────┐
  │ ●   ●   ●    │    6 tornillos
  │   ●   ●   ●  │    por vuelta
  └───────────────┘
        ↕
   Sensor LJ12A3
        │
        └───→ Pin EXTI del STM32 (interrupción)
```

### Pines

| Rueda | Pin | Morpho |
|-------|-----|--------|
| FL (delantero izq.) | PA0 (EXTI0) | CN7 pin 28 |
| FR (delantero der.) | PA1 (EXTI1) | CN7 pin 30 |
| RL (trasero izq.) | PA2 (EXTI2) | CN7 pin 32 |
| RR (trasero der.) | PB15 (EXTI15) | CN10 pin 26 |

### Conexión del analizador

```
ANALIZADOR          NUCLEO
──────────          ──────
  CH0  ──────────→  PA0  (rueda FL)    CN7 pin 28
  CH1  ──────────→  PA1  (rueda FR)    CN7 pin 30
  CH2  ──────────→  PA2  (rueda RL)    CN7 pin 32
  CH3  ──────────→  PB15 (rueda RR)    CN10 pin 26
  GND  ──────────→  GND                CN7 pin 8
```

### Configuración en PulseView

1. **Sample rate:** `10 kHz` (la frecuencia máxima de los sensores es 200 Hz)
2. **Samples:** `500K`
3. Puedes usar **Frequency counter** si PulseView lo tiene

### Frecuencias esperadas

| Velocidad del coche | Frecuencia de pulsos | Período |
|----|----|-----|
| Parado | 0 Hz | — |
| 5 km/h | ~8 Hz | ~125 ms |
| 10 km/h | ~15 Hz | ~67 ms |
| 25 km/h (máxima) | ~38 Hz | ~26 ms |

> El firmware rechaza frecuencias por encima de **200 Hz** como ruido.
> Si el sensor da 500 ms sin pulsos, la velocidad se pone a 0.

### ❌ Problemas y soluciones

| Lo que ves | Significado | Qué hacer |
|----|----|----|
| **Sin pulsos con la rueda girando** | Sensor no detecta los tornillos | 1. ¿Está alimentado? (12V o según modelo)<br>2. ¿Está lo suficientemente cerca de los tornillos? (2-4 mm)<br>3. ¿Los tornillos son metálicos? (el LJ12A3 solo detecta metal) |
| **Pulsos muy rápidos (>200 Hz)** | Ruido eléctrico | Aleja cables de los motores. Añade condensador de 100nF cerca del sensor |
| **Pulsos irregulares** | Tornillos desiguales o sensor suelto | Verifica que los tornillos están todos a la misma distancia y el sensor está fijo |

---

## Prueba 6 — CAN Bus (FDCAN1)

### ¿Qué estamos comprobando?

El bus CAN conecta el STM32 con el ESP32 (pantalla HMI). Funciona a **500 kbps**.

```
  STM32                Transceiver              ESP32
  ─────                CAN (MCP2551)            ─────
  PA12 (TX) ────────→ TXD    CANH ◄──────►  CANH
  PA11 (RX) ←──────── RXD    CANL ◄──────►  CANL
```

### Conexión del analizador

```
ANALIZADOR          NUCLEO
──────────          ──────
  CH0  ──────────→  PA12 (CAN TX)    CN10 pin 12
  CH1  ──────────→  PA11 (CAN RX)    CN10 pin 14
  GND  ──────────→  GND              CN10 pin 9
```

> **NOTA:** Estás midiendo las señales **antes** del transceiver (nivel lógico 3.3V).
> El bus CAN real (CANH/CANL) es diferencial y necesitaría un osciloscopio.

### Configuración en PulseView

1. **Sample rate:** `10 MHz` (CAN va a 500 kbps, necesitas mucha resolución)
2. **Samples:** `10M`
3. **Decode → CAN** → TX = CH0

### ¿Qué verás si funciona?

Ráfagas de datos seguidas de pausas. Cada trama CAN tiene:
- Bit de inicio
- ID (11 bits)
- Datos (0-8 bytes)
- CRC
- ACK

El firmware envía tramas periódicamente (status, temperaturas, corrientes, velocidades).

### ❌ Problemas y soluciones

| Lo que ves | Significado | Qué hacer |
|----|----|----|
| **TX activo pero RX siempre recesivo (HIGH)** | El STM32 envía pero no recibe | Verifica el transceiver CAN y la terminación 120Ω |
| **Tramas con errores** | Problemas de sincronización | Verifica que ambos nodos usan 500 kbps |
| **Sin actividad** | CAN no inicializado | Normal hasta que el boot completa. Verifica alimentación del transceiver |

---

## Prueba 7 — Pedal acelerador (ADC)

### ¿Qué estamos comprobando?

El pedal tiene un sensor Hall (SS1324LUA-T) que genera una tensión analógica de 0.3V a 4.8V.
Un divisor de tensión la reduce a 0–3.3V para el ADC del STM32.

```
  Pedal (5V)          Divisor de tensión         STM32
  ──────────          ──────────────────         ─────
  Señal ───── 10kΩ ──┬── 6.8kΩ ── GND           PA3 (ADC1_IN4)
                     │
                     └──────────────────────→ PA3
```

> **NOTA:** El analizador lógico **NO** puede medir señales analógicas.
> Solo verás HIGH o LOW según un umbral. Para ver la señal real necesitas un **osciloscopio**.
> Sin embargo, puedes verificar que el pin no está cortocircuitado o desconectado.

### Conexión del analizador

```
ANALIZADOR          NUCLEO
──────────          ──────
  CH0  ──────────→  PA3  (Pedal ADC)    CN7 pin 37
  GND  ──────────→  GND                 CN7 pin 8
```

### ¿Qué verás?

| Estado del pedal | Lo que ves en CH0 |
|-----|------|
| Sin pisar (0%) | Cerca de LOW (tensión baja) |
| Pisado a fondo (100%) | HIGH (tensión alta, ~3.3V) |
| Medio pisado | Puede oscilar entre HIGH y LOW dependiendo del umbral del analizador |

---

## Orden recomendado de pruebas

Empieza por lo más fácil y sube la dificultad:

| # | Prueba | Canales | Sample rate | Dificultad |
|---|--------|---------|-------------|------------|
| **1** | 🔴 I2C: ¿Responde el TCA9548A? (ACK a 0xE0) | 2 | 4 MHz | ⭐ Fácil |
| **2** | 🔴 I2C: ¿Lee los INA226? (shunt + bus) | 2 | 4 MHz | ⭐⭐ Media |
| **3** | 🟡 OneWire: ¿Hay pulso de presencia DS18B20? | 1 | 1 MHz | ⭐ Fácil |
| **4** | 🟡 OneWire: ¿Lee temperatura correcta? | 1 | 1 MHz | ⭐⭐ Media |
| **5** | 🟡 Encoder: ¿Señales A/B al girar el volante? | 3 | 500 kHz | ⭐ Fácil |
| **6** | 🟢 PWM: ¿RPWM/LPWM nunca activas a la vez? | 3 | 1 MHz | ⭐ Fácil |
| **7** | 🟢 Ruedas: ¿Pulsos al girar la rueda? | 4 | 10 kHz | ⭐ Fácil |
| **8** | 🟢 CAN: ¿Se ven tramas? | 2 | 10 MHz | ⭐⭐⭐ Difícil |

**🔴 = Prioridad alta** (si falla, el coche entra en modo seguro)
**🟡 = Prioridad media** (importantes para el funcionamiento)
**🟢 = Prioridad baja** (se pueden comprobar después)

---

## Solución de problemas comunes

### "No veo nada en el analizador"

1. ¿Está el cable **GND** conectado? (Es el error más común)
2. ¿Está la placa **encendida**?
3. ¿Es correcto el **pin** donde has conectado? (Cuenta los pines con cuidado)
4. ¿El **sample rate** es suficiente? (Debe ser al menos 10× la frecuencia de la señal)

### "Veo señales pero el decoder no las interpreta"

1. Verifica que has asignado los canales correctos en el decoder
2. Sube el sample rate
3. Asegúrate de que el GND del analizador y el GND de la Nucleo están conectados

### "La señal tiene mucho ruido"

1. Usa cables **cortos** (menos de 20 cm)
2. **Aleja** los cables del analizador de los cables de los motores
3. Conecta el cable GND del analizador **lo más cerca posible** del pin que estás midiendo

### "El I2C no funciona pero los cables están bien"

1. Comprueba las **resistencias pull-up** (4.7kΩ a 3.3V) en SDA y SCL.
   Sin ellas, I2C no puede funcionar.
2. Mide con un **multímetro** que hay 3.3V entre VCC y GND del TCA9548A
3. El firmware tiene **recuperación automática** del bus I2C:
   si detecta 3 fallos seguidos, envía 16 pulsos de reloj en SCL para desbloquear

---

## Resumen rápido de direcciones I2C

| Dispositivo | Dirección (7 bits) | En el bus (escritura) | En el bus (lectura) |
|---|---|---|---|
| TCA9548A | `0x70` | `0xE0` | `0xE1` |
| INA226 | `0x40` | `0x80` | `0x81` |

> **¿Por qué se duplica la dirección?** I2C envía la dirección de 7 bits + 1 bit
> que indica lectura (1) o escritura (0).
> Así: `0x70 << 1 = 0xE0` (escribir), `0xE0 | 1 = 0xE1` (leer).

---

## Resumen rápido de constantes del firmware

| Constante | Valor | Archivo |
|---|---|---|
| `I2C_ADDR_TCA9548A` | `0x70` | project_config.h |
| `I2C_ADDR_INA226` | `0x40` | project_config.h |
| `NUM_INA226` | 6 | project_config.h |
| `NUM_DS18B20` | 5 | project_config.h |
| `ENCODER_PPR` | 1200 | project_config.h |
| `ENCODER_CPR` | 4800 (1200 × 4) | project_config.h |
| `WHEEL_PULSES_REV` | 6 | project_config.h |
| `WHEEL_MAX_FREQ_HZ` | 200 | project_config.h |
| `WHEEL_STALE_TIMEOUT_MS` | 500 | project_config.h |
| `INA226_CONFIG_VALUE` | `0x4327` | sensor_manager.c |
| `INA226_SHUNT_MOHM_MOTOR` | 1.5 mΩ | project_config.h |
| `INA226_SHUNT_MOHM_BATTERY` | 0.75 mΩ | project_config.h |
| `I2C_FAIL_THRESHOLD` | 3 | sensor_manager.c |
| `OW_RESCAN_INTERVAL_MS` | 10000 (10 s) | sensor_manager.c |

---

> **Última actualización:** Abril 2026 — basado en el firmware actual del repositorio.
