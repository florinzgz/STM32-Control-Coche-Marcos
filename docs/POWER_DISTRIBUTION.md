# Distribución de Potencia — Arquitectura Eléctrica

Documentación completa de la distribución de potencia del vehículo controlado por STM32G474RE + ESP32-S3.
Todos los valores proceden exclusivamente del firmware y de la especificación hardware existente.

---

## 📋 Índice

1. [Arquitectura General](#1--arquitectura-general)
2. [Baterías y Niveles de Tensión](#2--baterías-y-niveles-de-tensión)
3. [Conversión DC-DC](#3--conversión-dc-dc)
4. [Punto de Masa Estrella](#4--punto-de-masa-estrella-star-ground)
5. [Secciones de Cable](#5--secciones-de-cable)
6. [Relés y Secuencia de Encendido](#6--relés-y-secuencia-de-encendido)
7. [Sensores de Corriente INA226](#7--sensores-de-corriente-ina226)
8. [Componentes de Protección](#8--componentes-de-protección)
9. [Análisis de Modos de Fallo](#9--análisis-de-modos-de-fallo)

---

## 1 — Arquitectura General

```
 ┌─────────────┐
 │ Batería 24 V│
 │             │
 └──────┬──────┘
        │
        │  4 mm²
        ▼
 ┌──────────────┐    INA226 ch4 (0.75 mΩ, 100 A)
 │  Fusible 60A │    ← sensor ANTES del relé principal
 └──────┬───────┘
        │
        ▼
 ┌──────────────┐  PC10
 └──────┬───────┘
        │
   ┌────┴─────────────────────────────────────────┐
   │                                              │
   ▼                                              ▼
 ┌──────────────┐  PC11                    ┌─────────────┐
 │  RELAY_TRAC  │──── STM32                │  DC-DC 24→5V│
 │  Fusible 50A │                          │  (LM2596)   │
 └──────┬───────┘                          └──────┬──────┘
        │ 2.5 mm²                                 │
   ┌────┼────┬────┐                          5 V lógica BTS7960
   │    │    │    │                          5 V WS2812B (vía relés LED)
   ▼    ▼    ▼    ▼
  INA  INA  INA  INA   ← ch0/ch1/ch2/ch3 (1.5 mΩ, 50 A)
   │    │    │    │
  BTS  BTS  BTS  BTS   ← BTS7960 drivers
   │    │    │    │
  M_FL M_FR M_RL M_RR  ← Motores de tracción


 ┌─────────────┐
 │ Batería 12 V│
 └──────┬──────┘
        │  4 mm²
        ▼
 ┌──────────────┐  PC12
 │  RELAY_DIR   │──── STM32 (vía módulo 4-ch opto relé SRD-12VDC-SL-C)
 │  Fusible 20A │
 └──────┬───────┘
        │ 2.5 mm²
        ▼
  INA226 ch5 (1.5 mΩ, 50 A)
        │
       BTS7960
        │
   Motor Dirección


 ┌──────────────────────────────────────────┐
 │  Regulador 3.3 V (desde 5 V o 12 V)     │
 │  → STM32G474RE, señales digitales, CAN   │
 └──────────────────────────────────────────┘
```

---

## 2 — Baterías y Niveles de Tensión

| Nivel   | Fuente                  | Consumidores principales                       |
|---------|-------------------------|-------------------------------------------------|
| **24 V** | Batería de tracción     | 4× motores tracción (vía BTS7960)              |
| **12 V** | Batería de dirección    | Motor de dirección (vía BTS7960)                |
| **5 V**  | DC-DC buck (LM2596)    | Tiras WS2812B, **transceiver CAN TJA1051T/3** (VCC mín 4.5 V) |
| **3.3 V**| Regulador lineal / LDO | STM32G474RE, TCA9548A, INA226, **lógica BTS7960 (IBT-2 VCC)** |

### Umbrales de tensión de batería 24 V (firmware)

Definidos en `Core/Src/safety_system.c`:

| Estado         | Condición                  | Acción del sistema              |
|----------------|----------------------------|---------------------------------|
| **NORMAL**     | V > 20.0 V                | Operación completa              |
| **DEGRADED**   | V ≤ 20.0 V (warning)      | Potencia reducida (DEGRADED_L1) |
| **SAFE**       | V < 18.0 V (critical)     | Actuadores OFF, modo seguro     |
| **Recuperación** | V > 18.5 V (histéresis 0.5 V) | Salida de SAFE si se mantiene |

> **Nota sobre histéresis:** Entre 18.0 V y 20.0 V el sistema permanece en DEGRADED. La recuperación desde SAFE requiere superar 18.5 V (18.0 V + 0.5 V de histéresis). La recuperación desde DEGRADED requiere superar 20.0 V.

```c
#define BATTERY_UV_WARNING_V    20.0f
#define BATTERY_UV_CRITICAL_V   18.0f
#define BATTERY_UV_HYST_V        0.5f
```

---

## 3 — Conversión DC-DC

### 24 V → 5 V (LM2596)

- **Tipo:** Buck síncrono, hasta 3 A.
- **Salida:** 5 V para alimentación de tiras LED WS2812B y transceiver CAN TJA1051T/3 (VCC).
- **Nota:** Los módulos BTS7960 (IBT-2) se alimentan a **3.3 V** en su pin VCC (no 5 V), para que la lógica del buffer 74HC244 sea compatible con señales de 3.3 V del STM32. La etapa de potencia se alimenta directamente desde la batería correspondiente (24 V o 12 V) a través de los pines B+ / B−.

### 5 V → 3.3 V

- **Tipo:** Regulador lineal o LDO integrado en la placa Nucleo-G474RE.
- **Salida:** 3.3 V para el MCU STM32G474RE, bus I²C (TCA9548A + INA226) y señales digitales.
- **⚠️ IMPORTANTE:** El transceiver CAN TJA1051T/3 **NO** se alimenta de 3.3 V. Requiere **5 V** (VCC mín = 4.5 V según datasheet NXP). El sufijo "/3" solo indica que los pines I/O (TXD, RXD) operan a niveles lógicos de 3.3 V.

### Aislamiento CAN (opcional)

Documentado en `docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md`:
- DC-DC aislado **5 V** / 200 mA (p. ej. RECOM R0505S o TMR1-0511) para alimentar el transceiver CAN aislado. El TJA1051T/3 requiere VCC = 4.5–5.5 V.
- Aislador digital ADuM1201 entre STM32 y TJA1051T/3.

---

## 4 — Punto de Masa Estrella (Star Ground)

Todas las masas del sistema **deben** converger en un único punto de conexión (estrella) para evitar lazos de tierra y diferencias de potencial entre subsistemas.

```
                         ★ PUNTO ESTRELLA
                         │
          ┌──────────────┼──────────────┬──────────────┐
          │              │              │              │
     GND Bat 24V   GND Bat 12V   GND STM32/3.3V   GND BTS7960
                                       │
                                  GND INA226
                                  GND TCA9548A
                                  GND WS2812B
                                  GND CAN Bus
```

**Reglas:**
- El cable de retorno de cada batería va directamente al punto estrella, no en cadena (daisy-chain).
- Los condensadores de desacoplo se conectan lo más cerca posible del IC correspondiente, con retorno al punto estrella.
- El bus CAN usa su propia masa de referencia (GND_CAN) que se une al punto estrella en un solo lugar, preferentemente a través del aislamiento galvánico.

---

## 5 — Secciones de Cable

| Tramo                        | Sección mínima | Justificación                         |
|------------------------------|----------------|---------------------------------------|
| Batería 24 V → Relé TRAC    | 4 mm²          | Corriente máxima de bus ≤ 100 A       |
| Relé TRAC → BTS7960 motores | 2.5 mm²        | Corriente por motor ≤ 25 A            |
| BTS7960 → Motor              | 2.5 mm²        | Corriente por motor ≤ 25 A            |
| Batería 12 V → Relé DIR     | 4 mm²          | Corriente de arranque dirección       |
| Relé DIR → BTS7960 dirección| 2.5 mm²        | Corriente dirección ≤ 25 A            |
| Señales lógicas (PWM, I²C)  | 0.5 mm²        | Baja corriente (< 100 mA)            |
| Bus CAN (CANH/CANL)         | 0.5 mm² par trenzado | Impedancia 120 Ω, señal diferencial |

> **Nota:** Todas las secciones son valores mínimos recomendados. Usar cables con aislamiento para ≥ 50 V en los tramos de potencia.

---

## 6 — Relés y Secuencia de Encendido

### Mapa de relés

| Función          | Pin STM32 | Puerto | Carga controlada                | Fusible |
|------------------|-----------|--------|---------------------------------|---------|
| `RELAY_TRAC`     | PC11      | GPIOC  | Motores de tracción 24 V        | 50 A    |
| `RELAY_DIR`      | PC12      | GPIOC  | Motor de dirección 12 V         | 20 A    |
| `RELAY_LED`      | PB10      | GPIOB  | Tira WS2812B frontal (5 V)      | —       |
| `RELAY_LED_REAR` | PB11      | GPIOB  | Tira WS2812B trasera (5 V)      | —       |

> **Nota:** **PC10 está DISPONIBLE (libre).** PC10 se configura como `GPIO_MODE_INPUT` con `GPIO_PULLDOWN` (estado seguro determinista). El firmware solo controla **RELAY_TRAC (PC11)** y **RELAY_DIR (PC12)**.

Los relés de potencia se controlan en **dos etapas**:

1. **Etapa 1 — Módulo intermedio:** Módulo de 4 canales con optoacopladores y relés
   SRD-12VDC-SL-C (bobina 12 V DC, contactos 10 A). El STM32 activa las entradas
   IN1–IN4 del módulo con 3.3 V (high/low level trigger). El módulo se alimenta
   desde la línea de 12 V (DC+ / DC−).

2. **Etapa 2 — Relés de potencia:** Los contactos del módulo intermedio conmutan
   los 12 V hacia las bobinas de los relés de potencia de alta corriente. Estos
   relés de potencia (bobina 12 V DC) conmutan la línea de 24 V (tracción) o
   la línea de 12 V (dirección) hacia los drivers BTS7960.

Esquema de la cadena de control:
```
STM32 GPIO (3.3V) ──► Módulo 4-ch opto relé (SRD-12VDC-SL-C, 12V)
                          │
                          ├── CH2 contacto → Bobina relé potencia TRAC (12V) → conmuta 24V tracción
                          └── CH3 contacto → Bobina relé potencia DIR  (12V) → conmuta 12V dirección
```

### Secuencia de encendido (`Relay_PowerUp`)

Implementada en `Core/Src/safety_system.c` como máquina de estados no bloqueante, llamada desde el bucle de seguridad cada 10 ms:

```
 t=0 ms        t=50 ms           t=70 ms
   │              │                 │
   ▼              ▼                 ▼
  RELAY_TRAC ON → espera 50 ms → RELAY_DIR ON
  (PC11 SET)      (settle)       (PC12 SET)
```

```c
#define RELAY_TRACTION_SETTLE_MS 20   /* retardo supresión de arco              */
```

### Secuencia de apagado (`Relay_PowerDown`)

Orden inverso, inmediato (sin retardos):

```
RELAY_DIR OFF → RELAY_TRAC OFF
  (PC12)          (PC11)
```

---

## 7 — Sensores de Corriente INA226

### Multiplexor I²C: TCA9548A @ 0x70

Todos los INA226 comparten la dirección I²C **0x40**. El TCA9548A selecciona el canal activo.

```
STM32 I²C1
    │
    ▼
 TCA9548A (0x70)
    │
    ├── Canal 0 ── INA226 (0x40) ── Motor FL   [1.5 mΩ,  50 A]
    ├── Canal 1 ── INA226 (0x40) ── Motor FR   [1.5 mΩ,  50 A]
    ├── Canal 2 ── INA226 (0x40) ── Motor RL   [1.5 mΩ,  50 A]
    ├── Canal 3 ── INA226 (0x40) ── Motor RR   [1.5 mΩ,  50 A]
    ├── Canal 4 ── INA226 (0x40) ── Batería 24V [0.75 mΩ, 100 A]
    └── Canal 5 ── INA226 (0x40) ── Dirección   [1.5 mΩ,  50 A]
```

### Ubicación física de cada sensor

| Canal | Sensor        | Shunt   | Rango  | Ubicación                                                   |
|-------|---------------|---------|--------|-------------------------------------------------------------|
| 0     | INA226_FL     | 1.5 mΩ | 50 A   | **ANTES** del BTS7960 FL (entre salida RELAY_TRAC y B+)     |
| 1     | INA226_FR     | 1.5 mΩ | 50 A   | **ANTES** del BTS7960 FR (entre salida RELAY_TRAC y B+)     |
| 2     | INA226_RL     | 1.5 mΩ | 50 A   | **ANTES** del BTS7960 RL (entre salida RELAY_TRAC y B+)     |
| 3     | INA226_RR     | 1.5 mΩ | 50 A   | **ANTES** del BTS7960 RR (entre salida RELAY_TRAC y B+)     |
| 4     | INA226_MAIN   | 0.75 mΩ | 100 A  | **ANTES** del RELAY_TRAC (mide corriente total batería 24V)   |
| 5     | INA226_STEER  | 1.5 mΩ | 50 A   | **ANTES** del BTS7960 DIR (entre salida RELAY_DIR y B+)     |

> **Crítico — Canal 4:** El sensor de batería está colocado **antes del RELAY_TRAC**. Esto permite que `Voltage_GetBus(INA226_CHANNEL_BATTERY)` lea la tensión de la batería incluso con los relés abiertos. Es esencial para:
> - Validación de batería en el arranque (boot validation), antes de cerrar relés.
> - Detección de fallo de batería durante operación.
> - Lectura de tensión en modo LIMP_HOME si los relés se abren.

### Diagrama de ubicación — Cadena de potencia de tracción

```
 Batería 24V (+)
       │
       ▼
  ┌──────────┐
  │ INA226   │ ← Canal 4 (0.75 mΩ, 100 A) — mide tensión y corriente total
  │ ch4      │
  └────┬─────┘
       │
  ┌────┴──────┐
  │RELAY_TRAC │ (PC11)
  └────┬─────┘
       │
  ┌────┼────────┬────────┬────────┐
  │    │        │        │        │
  ▼    ▼        ▼        ▼        ▼
 INA  INA      INA      INA
 ch0  ch1      ch2      ch3
 1.5mΩ 1.5mΩ   1.5mΩ    1.5mΩ
  │    │        │        │
 BTS  BTS      BTS      BTS
 7960 7960     7960     7960
  │    │        │        │
 M_FL M_FR    M_RL     M_RR
```

### Umbrales de corriente (firmware)

```c
#define MAX_CURRENT_A   25.0f   /* máximo por motor individual */
```

| Parámetro                | Valor  | Acción                                |
|--------------------------|--------|---------------------------------------|
| Corriente máx. por motor | 25 A   | Sobrecorriente → DEGRADED o SAFE     |
| Corriente máx. total bus | 50 A   | Fusible 60 A en línea 24V (margen 20 % sobre límite SW) |

---

## 8 — Componentes de Protección

### Condensadores de desacoplo y filtrado

| Componente             | Valor              | Ubicación                                |
|------------------------|--------------------|------------------------------------------|
| Cerámico               | 100 nF             | En 3.3 V del STM32 (VDD, lo más cerca posible de los pines) |
| Electrolítico          | 10 µF              | En 3.3 V del STM32 (bulk, junto al regulador) |
| Cerámico               | 100 nF             | En VCC (5 V) de cada módulo BTS7960      |
| Electrolítico          | 1000 µF / 35 V     | En bus 24 V cerca de los relés           |
| Electrolítico          | 470 µF / 25 V      | En bus 12 V (dirección)                  |

### Protección de relés

| Componente                    | Valor                    | Ubicación                          |
|-------------------------------|--------------------------|------------------------------------|
| Diodo flyback                 | 1N4007                   | En cada bobina de relé (antiparalelo) |
| Snubber RC en contactos       | 100 Ω + 100 nF / 250 V  | En paralelo con los contactos de cada relé |

### Protección de motores

| Componente                    | Valor              | Ubicación                                |
|-------------------------------|--------------------|-----------------------------------------|
| Cerámico (snubber)            | 100 nF / 50 V      | En cada terminal de motor                |

### Bus CAN

| Componente                    | Valor              | Ubicación                                |
|-------------------------------|--------------------|-----------------------------------------|
| Resistencia de terminación    | 120 Ω × 2          | En cada extremo del bus CAN              |

### Protección de encoders

| Componente                    | Valor / Tipo        | Ubicación                               |
|-------------------------------|---------------------|-----------------------------------------|
| 3× 6N137 optoacoplador | R_IN 330 Ω + pull-up 4,7 kΩ | Aislamiento galvánico + conversión 5 V → 3.3 V en señales encoder (A, B, Z) |

---

## 9 — Análisis de Modos de Fallo

### Fallo de batería

| Fallo                         | Detección                              | Respuesta del sistema                  |
|-------------------------------|----------------------------------------|----------------------------------------|
| Tensión baja (< 20 V)        | INA226 ch4 lectura continua            | Estado DEGRADED, potencia reducida     |
| Tensión crítica (< 18 V)     | INA226 ch4 lectura continua            | Estado SAFE, todos los actuadores OFF  |
| Desconexión de batería        | INA226 ch4 lee 0 V                    | Estado SAFE inmediato                  |
| Cortocircuito interno batería | Fusible 60 A (línea 24V) se funde    | Circuito abierto, sistema sin potencia |

### Fallo de relé

| Fallo                         | Detección                              | Respuesta del sistema                  |
|-------------------------------|----------------------------------------|----------------------------------------|
| RELAY_TRAC no cierra          | INA226 ch0-ch3 sin corriente           | Motores de tracción inoperativos       |
| RELAY_DIR no cierra           | INA226 ch5 sin corriente               | Dirección inoperativa → SAFE           |
| Bobina de relé en corto       | Módulo opto relé SRD-12VDC limita corriente | Fusible de bobina protege STM32        |

### Fallo de BTS7960

| Fallo                         | Detección                              | Respuesta del sistema                  |
|-------------------------------|----------------------------------------|----------------------------------------|
| Cortocircuito interno         | INA226 del motor afectado > 25 A       | DEGRADED → SAFE si persiste            |
| Circuito abierto              | INA226 lee 0 A con duty > 0            | Motor individual sin tracción          |
| Sobrecalentamiento            | Señal IS del BTS7960 (si conectada)    | Reducción de duty o apagado            |

### Fallo de INA226 / TCA9548A

| Fallo                         | Detección                              | Respuesta del sistema                  |
|-------------------------------|----------------------------------------|----------------------------------------|
| INA226 no responde (I²C NACK) | Timeout I²C en boot validation         | Canal marcado como no disponible       |
| TCA9548A no responde          | Ningún INA226 accesible                | Estado SAFE, sin monitorización        |
| Lectura errónea de shunt      | Valores fuera de rango (validación boot) | Estado SAFE si falla validación       |

### Fallo de cableado

| Fallo                         | Detección                              | Respuesta del sistema                  |
|-------------------------------|----------------------------------------|----------------------------------------|
| Cable de potencia suelto      | INA226 detecta caída de corriente      | DEGRADED o SAFE según severidad        |
| Masa suelta (star ground)     | Lecturas de tensión erróneas en INA226 | Comportamiento impredecible — diseño debe prevenir |
| Cable de señal CAN roto       | Timeout de comunicación ESP32 ↔ STM32  | Estado DEGRADED, modo autónomo STM32   |

### Fallo de fusibles

| Fusible          | Consecuencia al fundirse                                           |
|------------------|--------------------------------------------------------------------|
| 60 A (línea 24V)| Sistema completo sin potencia de tracción; INA226 ch4 lee 0 A     |
| 50 A (RELAY_TRAC)| Motores de tracción sin alimentación; dirección sigue operativa    |
| 20 A (RELAY_DIR) | Dirección sin alimentación; tracción sigue operativa → SAFE       |

---

> **Documento generado a partir del firmware (`Core/Src/safety_system.c`, `Core/Inc/main.h`) y especificaciones hardware (`HARDWARE_SPECIFICATION.md`, `HARDWARE_WIRING_MANUAL.md`). No contiene hardware inventado.**
