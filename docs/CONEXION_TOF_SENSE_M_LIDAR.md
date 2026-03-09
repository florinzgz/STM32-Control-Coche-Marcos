# Conexión Práctica — Sensor de Obstáculos ToF Sense M LiDAR a ESP32-S3

**Basado en:** Plan de integración del proyecto (`INTEGRATION_PLAN.md`, `TOFSENSE_M_WIRING_GUIDE.md`, `OBSTACLE_SYSTEM_ARCHITECTURE.md`)  
**Sensor:** TOFSense-M S (Nooploop) — LiDAR 8×8 Time-of-Flight  
**MCU:** ESP32-S3 DevKitC-1  

---

## 1. Tabla de conexiones — Sensor → ESP32-S3

| Pin sensor (GH1.25) | Señal   | Dirección           | Conectar a               | Notas                                    |
|----------------------|---------|----------------------|---------------------------|------------------------------------------|
| Pin 1                | **VCC** | Entrada alimentación | **5 V** (regulador/fuente) | ⚠️ **Obligatorio 5 V**, NO 3.3 V         |
| Pin 2                | **GND** | Referencia           | **GND** común con ESP32-S3 | Tierra compartida con todo el sistema    |
| Pin 3                | **RX**  | Entrada al sensor    | **No conectar**            | No se envían comandos al sensor          |
| Pin 4                | **TX**  | Salida del sensor    | **GPIO 18** del ESP32-S3 **a través de divisor de tensión** | UART1 RX del ESP32-S3 (921600 bps, 8N1). ⚠️ Usar divisor R1=1 kΩ + R2=4.7 kΩ |

> **Resumen:** Se conectan 3 cables (VCC, GND, TX) más un **divisor de tensión obligatorio** (R1=1 kΩ en serie + R2=4.7 kΩ a GND) entre el TX del sensor y GPIO 18 del ESP32-S3.  
> El pin RX del sensor queda al aire.

---

## 2. Voltaje de alimentación

| Parámetro       | Valor                 |
|------------------|-----------------------|
| Voltaje VCC      | **5 V DC** (obligatorio) |
| Consumo típico   | ~200 mA               |
| Conector         | GH1.25 4P hembra      |

- El sensor **no funciona a 3.3 V**. A ese voltaje el láser interno no alcanza la potencia necesaria y no transmitirá tramas válidas.
- Usar regulador de 5 V o la salida de 5 V del ESP32-S3 DevKitC (pin 5V) si la fuente lo permite.

---

## 3. Tipo de comunicación

| Parámetro         | Valor                         |
|--------------------|-------------------------------|
| Protocolo          | **UART** (conexión serial)    |
| Baudrate           | **921600 bps**                |
| Formato            | **8N1** (8 bits, sin paridad, 1 stop bit) |
| Control de flujo   | Ninguno                       |
| Modo de transmisión | **Automático** (el sensor transmite continuamente sin necesidad de comandos) |
| Nivel lógico UART  | **3.5–3.6 V** medido (nominal 3.3 V TTL según datasheet) |

> **No se usa I2C ni CAN para comunicar el sensor con el ESP32.** La comunicación es UART unidireccional (sensor TX → divisor de tensión → ESP32 RX).
>
> ⚠️ **PELIGRO:** El pin TX del sensor emite **3.5–3.6 V** (medido), por encima de los 3.3 V del datasheet. El ESP32-S3 tiene un máximo absoluto de **3.6 V** en GPIO. Se requiere **divisor de tensión obligatorio** (ver sección 4).

---

## 4. ¿Se necesitan resistencias, conversores de nivel o protección?

| Componente                  | ¿Necesario? | Motivo                                                   |
|------------------------------|-------------|----------------------------------------------------------|
| **Divisor de tensión (R1=1 kΩ + R2=4.7 kΩ)** | **SÍ** — Opción 1 | El TX del sensor emite 3.5–3.6 V medidos. El ESP32-S3 tiene máx. absoluto 3.6 V. El divisor reduce a ~2.9 V (seguro) |
| **Level shifter BSS138** | **SÍ** — Opción 2 (alternativa al divisor) | Módulo con MOSFET BSS138 convierte 3.5 V → 3.3 V exactos. Señal más limpia que el divisor. Usar módulos tipo SparkFun BOB-12009, Adafruit 757, o genérico "Logic Level Converter 3.3V–5V" |
| Resistencias pull-up/pull-down     | **NO**      | UART no requiere pull-ups (a diferencia de I2C)          |
| Condensador de desacoplo 100 nF    | **SÍ** (recomendado) | Entre VCC y GND del sensor, lo más cerca posible del conector, para filtrar ruido de alimentación |

> ⚠️ **NO usar level shifters basados en TXS0108E** — generan oscilaciones a 921600 bps. Usar **solo BSS138** (MOSFET con pull-ups).

> Elegir **una sola opción** (divisor **o** level shifter), nunca ambas a la vez. Ver detalles completos en `docs/TOFSENSE_M_WIRING_GUIDE.md`, sección 2.4.

### Opción 1 — Divisor de tensión (detalle)

```
  Sensor TX (pin 4)          GPIO 18 (ESP32-S3)
       │                          │
       ├──── R1 = 1 kΩ ───────────┤
       │                          │
       │                     R2 = 4.7 kΩ
       │                          │
       │                         GND
```

| Parámetro | Valor |
|-----------|-------|
| R1 (serie) | **1 kΩ** |
| R2 (a GND) | **4.7 kΩ** |
| Vout con 3.6 V entrada | **2.97 V** (seguro, < 3.3 V) |
| Vout con 3.5 V entrada | **2.88 V** (seguro) |
| VIH ESP32-S3 | **2.475 V** (umbral HIGH) → todas las salidas OK |
| Impedancia total | 5.7 kΩ → RC ≈ 85 ns (compatible con 921600 bps) |

### Opción 2 — Level shifter BSS138 (alternativa sin resistencias)

En lugar de las resistencias, se puede usar un **módulo level shifter basado en BSS138**:

| Módulo compatible | Chip | Precio aprox. |
|-------------------|------|---------------|
| SparkFun BOB-12009 | BSS138 | ~2 € |
| Adafruit 757 | BSS138 | ~3 € |
| Genérico "Logic Level Converter 3.3V–5V" | BSS138 | ~0.50 € |

**Conexión:** HV=5V, LV=3.3V, HV1=sensor TX, LV1=GPIO 18, GND=común.

> ⚠️ **NO usar TXS0108E** — genera oscilaciones a 921600 bps. Solo BSS138.
>
> Detalles completos, diagramas y comparativa en `docs/TOFSENSE_M_WIRING_GUIDE.md`, sección 2.4.

---

## 5. Pines TX/RX en el ESP32-S3

| Función ESP32-S3 | GPIO        | Conectado a         | Uso                              |
|-------------------|-------------|----------------------|----------------------------------|
| **UART1 RX**      | **GPIO 18** | Pin 4 (TX) del sensor | Recibe tramas de distancia       |
| **UART1 TX**      | **No usado** (txPin = -1) | —                   | El ESP32 no envía datos al sensor |

- En el firmware, el UART1 del ESP32-S3 se configura con `rxPin = 18` y `txPin = -1` (desactivado).
- Referencia en código: `esp32/src/sensors/obstacle_sensor.h`, línea 59.

---

## 6. Esquema de cableado paso a paso

### Paso 1: Identificar el conector GH1.25 del sensor

El TOFSense-M S tiene un conector GH1.25 de 4 pines con la secuencia **V G R T** (VCC, GND, RX, TX):

```
Conector GH1.25 (vista frontal del sensor):
  ┌─────────────────┐
  │ 1   2   3   4   │
  │VCC GND  RX  TX  │
  └─────────────────┘
```

### Paso 2: Conectar los cables (con divisor de tensión obligatorio)

```
  Fuente 5V ──────────────┐
                          │
                 ┌────────┴──────────────┐
                 │  TOFSense-M S         │
                 │  (Conector GH1.25)    │
  5V ───────────►│ Pin 1 (VCC)           │
                 │                       │
  GND ──────────►│ Pin 2 (GND)           │
                 │                       │
  (sin conectar) │ Pin 3 (RX)            │    No conectar
                 │                       │
                 │ Pin 4 (TX) ───────────┼──┐
                 │                       │  │  Divisor de tensión
                 └───────────────────────┘  │  OBLIGATORIO
                       │     │              │
                      ┌┤     ├┐             │
                      │100 nF│              │
                      └┤     ├┘        ┌────┘
                       │     │         │
                                  ┌────┴────┐
                                  │  R1=1kΩ │  (serie)
                                  └────┬────┘
                                       │
                                       ├──── ESP32 GPIO18
                                       │
                                  ┌────┴────┐
                                  │ R2=4.7kΩ│  (a GND)
                                  └────┬────┘
                                       │
                                      GND
```

### Paso 3: Conexión en el ESP32-S3

```
                 ┌─────────────────────┐
                 │     ESP32-S3        │
                 │     DevKitC-1       │
                 │                     │
  Divisor out ──►│ GPIO 18 (UART1 RX)  │  ← Señal reducida a ~2.9 V
                 │                     │     (NO conectar TX directo)
  GND común ────►│ GND                 │
                 │                     │
                 │ 5V ─────────────────┼──► Alimentación sensor (si la fuente
                 │                     │    del DevKit lo permite, o usar
                 └─────────────────────┘    regulador externo de 5V)
```

### Diagrama completo del sistema

```
  ┌──────────────┐                                  ┌─────────────────────┐
  │ TOFSense-M S │                                  │     ESP32-S3        │
  │              │                                  │                     │
  │ Pin 1 (VCC)  ├────── 5V DC ◄────────────────────┤ 5V (o fuente ext.) │
  │ Pin 2 (GND)  ├────── GND  ◄────────────────────┤ GND                │
  │ Pin 3 (RX)   ├  n/c                            │                     │
  │ Pin 4 (TX)   ├──┐                              │                     │
  │              │  │  Divisor de tensión           │                     │
  └──────┬──┬────┘  │  (obligatorio)                │                     │
         │  │       │                               │                     │
        100nF   R1=1kΩ                              │                     │
                    │                               │                     │
                    ├──────── R2=4.7kΩ ── GND       │                     │
                    │                               │                     │
                    └──── ~2.9V ────────────────────► GPIO 18 (UART1 RX) │
                              921600 bps            │                     │
                                                    └─────────────────────┘
```

---

## 7. Verificación tras el cableado

| Paso | Verificación                              | Valor esperado                                |
|------|-------------------------------------------|-----------------------------------------------|
| 1    | Medir VCC del sensor con multímetro       | **5.00 V ± 0.2 V**                            |
| 2    | Medir GND común sensor-ESP32              | **0 V** (continuidad)                          |
| 3    | Medir TX del sensor sin divisor (pin 4)   | **3.5–3.6 V** (nivel idle)                     |
| 4    | Medir salida del divisor (punto medio R1-R2) | **2.8–3.0 V** (reducido por divisor)       |
| 5    | Encender sistema y revisar monitor serial | `[OBSTACLE] TOFSense-M initialized (UART1, 921600 bps)` |
| 6    | Poner la mano frente al sensor            | La distancia debe cambiar en el display/serial |
| 7    | Estado del sensor                         | Debe cambiar de `WAITING` → `VALID`           |

> ⚠️ **Si mides ~2.1 V en el paso 4** en vez de ~2.9 V, el resistor R1 es incorrecto (probablemente 3.3 kΩ en vez de 1 kΩ). Ver `docs/TOFSENSE_M_WIRING_GUIDE.md`, sección 5 para diagnóstico detallado con códigos de color y tabla de voltajes.
>
> ⚠️ **Si el estado alterna VALID→INVALID rápidamente**, abrir el monitor serie y buscar las líneas `[OBSTACLE] Diag`. Si `cksumFail >> OK`, la señal es marginal — verificar voltaje y resistencias.

---

## Resumen rápido

| Concepto                    | Valor                                  |
|------------------------------|----------------------------------------|
| Sensor                       | TOFSense-M S (Nooploop LiDAR 8×8)     |
| MCU                          | ESP32-S3 DevKitC-1                     |
| Comunicación                 | UART unidireccional, 921600 bps, 8N1   |
| Pin sensor TX → ESP32 RX     | Pin 4 → **GPIO 18**                    |
| Alimentación sensor          | **5 V DC** (obligatorio)               |
| Nivel lógico UART            | 3.5–3.6 V medido (nominal 3.3 V TTL) — **protección obligatoria** |
| Protección (elegir una)      | **Opción 1:** Divisor resistivo R1=1 kΩ + R2=4.7 kΩ → ~2.9 V. **Opción 2:** Level shifter BSS138 → 3.3 V exactos |
| Level shifter recomendado    | BSS138 (SparkFun BOB-12009, Adafruit 757, o genérico). ⚠️ NO usar TXS0108E |
| Condensador desacoplo        | 100 nF entre VCC y GND (recomendado)   |
| Cables necesarios            | 3 (VCC, GND, TX→GPIO18) + protección   |

---

_Extraído del plan de integración del proyecto Control Coche Marcos._  
_Archivos fuente: `INTEGRATION_PLAN.md`, `TOFSENSE_M_WIRING_GUIDE.md`, `OBSTACLE_SYSTEM_ARCHITECTURE.md`, `obstacle_sensor.h`._
