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
| Pin 4                | **TX**  | Salida del sensor    | **GPIO 18** del ESP32-S3   | UART1 RX del ESP32-S3 (921600 bps, 8N1) |

> **Resumen:** Solo se conectan 3 cables: VCC (5 V), GND y TX del sensor → GPIO 18 del ESP32-S3.  
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
| Nivel lógico UART  | **3.3 V TTL**                 |

> **No se usa I2C ni CAN para comunicar el sensor con el ESP32.** La comunicación es UART unidireccional (sensor TX → ESP32 RX).

---

## 4. ¿Se necesitan resistencias, conversores de nivel o protección?

| Componente                  | ¿Necesario? | Motivo                                                   |
|------------------------------|-------------|----------------------------------------------------------|
| Level shifter (conversor de nivel) | **NO**      | Las señales UART del sensor ya son 3.3 V TTL, igual que el ESP32-S3 |
| Resistencias pull-up/pull-down     | **NO**      | UART no requiere pull-ups (a diferencia de I2C)          |
| Divisor de tensión                 | **NO**      | Niveles lógicos compatibles directamente                 |
| Condensador de desacoplo 100 nF    | **SÍ** (recomendado) | Entre VCC y GND del sensor, lo más cerca posible del conector, para filtrar ruido de alimentación |

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

### Paso 2: Conectar los cables

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
  ESP32 GPIO18 ◄─│ Pin 4 (TX)            │    Sensor TX → ESP32 RX
                 │                       │
                 └───────────────────────┘
                       │     │
                      ┌┤     ├┐
                      │100 nF│  ← Condensador desacoplo
                      └┤     ├┘    entre VCC y GND
                       │     │     (lo más cerca del sensor)
```

### Paso 3: Conexión en el ESP32-S3

```
                 ┌─────────────────────┐
                 │     ESP32-S3        │
                 │     DevKitC-1       │
                 │                     │
  Sensor TX ────►│ GPIO 18 (UART1 RX)  │
                 │                     │
  GND común ────►│ GND                 │
                 │                     │
                 │ 5V ─────────────────┼──► Alimentación sensor (si la fuente
                 │                     │    del DevKit lo permite, o usar
                 └─────────────────────┘    regulador externo de 5V)
```

### Diagrama completo del sistema

```
  ┌──────────────┐         Cable directo           ┌─────────────────────┐
  │ TOFSense-M S │         (sin intermediarios)     │     ESP32-S3        │
  │              │                                  │                     │
  │ Pin 1 (VCC)  ├────── 5V DC ◄────────────────────┤ 5V (o fuente ext.) │
  │ Pin 2 (GND)  ├────── GND  ◄────────────────────┤ GND                │
  │ Pin 3 (RX)   ├  n/c                            │                     │
  │ Pin 4 (TX)   ├──────────────────────────────────► GPIO 18 (UART1 RX) │
  │              │     3.3V TTL, 921600 bps         │                     │
  └──────┬──┬────┘                                  └─────────────────────┘
         │  │
        100nF  ← Condensador de desacoplo VCC-GND
```

---

## 7. Verificación tras el cableado

| Paso | Verificación                              | Valor esperado                                |
|------|-------------------------------------------|-----------------------------------------------|
| 1    | Medir VCC del sensor con multímetro       | **5.00 V ± 0.2 V**                            |
| 2    | Medir GND común sensor-ESP32              | **0 V** (continuidad)                          |
| 3    | Encender sistema y revisar monitor serial | `[OBSTACLE] TOFSense-M initialized (UART1, 921600 bps)` |
| 4    | Poner la mano frente al sensor            | La distancia debe cambiar en el display/serial |
| 5    | Estado del sensor                         | Debe cambiar de `WAITING` → `VALID`           |

---

## Resumen rápido

| Concepto                    | Valor                                  |
|------------------------------|----------------------------------------|
| Sensor                       | TOFSense-M S (Nooploop LiDAR 8×8)     |
| MCU                          | ESP32-S3 DevKitC-1                     |
| Comunicación                 | UART unidireccional, 921600 bps, 8N1   |
| Pin sensor TX → ESP32 RX     | Pin 4 → **GPIO 18**                    |
| Alimentación sensor          | **5 V DC** (obligatorio)               |
| Nivel lógico UART            | 3.3 V TTL (directo, sin conversor)     |
| Level shifter                | No necesario                           |
| Resistencias                 | No necesarias                          |
| Condensador desacoplo        | 100 nF entre VCC y GND (recomendado)   |
| Cables necesarios            | 3 (VCC, GND, TX→GPIO18)               |

---

_Extraído del plan de integración del proyecto Control Coche Marcos._  
_Archivos fuente: `INTEGRATION_PLAN.md`, `TOFSENSE_M_WIRING_GUIDE.md`, `OBSTACLE_SYSTEM_ARCHITECTURE.md`, `obstacle_sensor.h`._
