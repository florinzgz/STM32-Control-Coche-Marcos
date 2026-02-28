# Guía de Conexión TOFSense-M — Basada en Manual Oficial V3.0

**Fecha:** 2026-02-28  
**Sensor:** TOFSense-M S (Nooploop) — LiDAR 8×8 Time-of-Flight  
**MCU:** ESP32-S3 DevKitC-1 (UART1, GPIO18 RX)  
**Fuente oficial:** [TOFSense-M User Manual V3.0](https://ftp.nooploop.com/downloads/tofsense/TOFSense-M_User_Manual_V3.0_en.pdf), [TOFSense-M Datasheet V3.0](https://ftp.nooploop.com/downloads/tofsense/TOFSense-M_Datasheet_V3.0_en.pdf)  
**Código de referencia Nooploop:** [nlink_tofsensem_frame0.c](https://github.com/nooploop-dev/nlink_unpack/blob/master/nlink_tofsensem_frame0.c)

---

## 1. Alimentación correcta

### Según el manual oficial V3.0:

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| **Voltaje VCC** | **5 V DC** (obligatorio) | Datasheet V3.0, sección "Typical Specifications" |
| **Consumo típico** | ~200 mA | Datasheet V3.0 |
| **Conector** | GH1.25 4P hembra | Datasheet V3.0 |

### ¿Funciona a 3.3 V?

**NO.** El sensor requiere **5 V** en el pin VCC. Alimentar a 3.3 V provocará:
- Funcionamiento inestable o ausencia total de datos
- El firmware reportará estado `INVALID` al no recibir tramas UART válidas
- El láser interno puede no alcanzar la potencia necesaria para mediciones fiables

### Niveles lógicos UART:

Según el datasheet V3.0: *"Communication Interface UART and CAN, TTL signal line level 3.3V"*

Esto significa:
- **VCC = 5 V** (alimentación del sensor: láser, procesador interno, etc.)
- **UART TX/RX = 3.3 V TTL** (las señales de datos operan a 3.3 V lógico)
- Ambos voltajes son correctos y no hay contradicción: el sensor se alimenta a 5 V internamente pero sus líneas UART son 3.3 V TTL

---

## 2. Interfaz de comunicación — Cableado UART

### 2.1 Pinout del conector GH1.25 (modo UART)

Según el datasheet V3.0, la secuencia de pines es **V G R T** (VCC, GND, RX, TX):

| Pin | Señal | Dirección (desde sensor) | Conexión a ESP32-S3 |
|-----|-------|--------------------------|---------------------|
| 1 | **VCC** | Entrada alimentación | **5 V** del regulador o fuente |
| 2 | **GND** | Referencia | **GND** común con ESP32 |
| 3 | **RX** | Entrada al sensor | **No conectado** (modo recepción unidireccional) |
| 4 | **TX** | Salida del sensor | **GPIO18** (UART1 RX del ESP32) |

### 2.2 ¿Se deben cruzar TX↔RX?

**SÍ, como en cualquier conexión UART estándar:**
- Sensor **TX** (pin 4) → ESP32 **RX** (GPIO18)
- Sensor **RX** (pin 3) → ESP32 **TX** (no conectado en este caso)

El cruce TX↔RX es inherente al protocolo UART: el transmisor de un lado se conecta al receptor del otro.

### 2.3 ¿Se conectan ambos (TX y RX)?

**No, solo TX del sensor → RX del ESP32.** En este sistema:
- El sensor transmite datos de distancia continuamente (modo activo)
- El ESP32 solo **recibe** datos — no envía comandos al sensor
- El pin RX del sensor (pin 3) queda **sin conectar**
- En el código, `txPin = -1` desactiva la función TX del UART del ESP32

### 2.4 ¿Se necesitan resistencias, divisor de tensión o level shifter?

**NO.** Según el datasheet V3.0:
- Las señales UART del TOFSense-M son **TTL 3.3 V**
- El ESP32-S3 también opera a **3.3 V lógico**
- La conexión es directa: sensor TX → ESP32 RX, sin componentes intermedios

**Único componente recomendado:** condensador de desacoplo de **100 nF** entre VCC y GND del sensor, lo más cerca posible del conector, para filtrar ruido de alimentación.

### 2.5 Diagrama de conexión

```
                    TOFSense-M S (GH1.25)
                   ┌──────────────────────┐
  5V (regulador) ──┤ VCC  (pin 1)         │
                   │                      │     ⚠ VCC = 5V obligatorio
  GND ─────────────┤ GND  (pin 2)         │     UART = 3.3V TTL
                   │                      │
           n/c ────┤ RX   (pin 3)         │     (no se envían comandos)
                   │                      │
  ESP32 GPIO18 ────┤ TX   (pin 4)         │     Sensor TX → ESP32 UART1 RX
                   └──────────────────────┘
                         │    │
                        100nF (desacoplo VCC-GND)
```

---

## 3. Motivo del estado INVALID

### 3.1 Condiciones que provocan INVALID en el firmware

El firmware (`obstacle_sensor.cpp`) reporta `SensorStatus::INVALID` en los siguientes casos:

| Condición | Causa probable | Verificación |
|-----------|---------------|--------------|
| **Timeout de tramas** | No se reciben tramas válidas durante >500 ms | Verificar cableado TX→RX, baudrate, VCC |
| **Distancia fuera de rango** | Medida < 20 mm o > 4000 mm | Verificar que hay un objeto en el rango del sensor |
| **Sensor atascado (stuck)** | Distancia no varía ±10 mm durante 1000 ms mientras el vehículo se mueve | Verificar que el sensor no está obstruido |
| **Checksum incorrecto** | Trama corrupta o baudrate incorrecto | Verificar baudrate = 921600, cableado limpio |
| **Header inválido** | Byte 0 ≠ 0x57 o byte 1 ≠ 0x01 | Verificar que el sensor está en modo UART (no CAN) |
| **Todos los píxeles inválidos** | `dis_status ≠ 0` en los 64 píxeles | Superficie demasiado oscura, ángulo extremo, o sensor sin inicializar |

### 3.2 Condiciones de `dis_status` (por píxel, protocolo NLink)

Según el manual oficial:
- **`dis_status = 0`**: Medida válida — el valor de distancia es fiable
- **`dis_status ≠ 0`**: Medida inválida — el píxel no tiene lectura fiable. Causas:
  - Objeto fuera de rango (muy cerca o muy lejos)
  - Superficie con baja reflectividad (negra, absorbente)
  - Ángulo de incidencia extremo
  - Interferencia IR (luz solar directa intensa, otro sensor ToF cercano)
  - Señal demasiado débil o saturación del receptor

### 3.3 ¿Qué señales deberían verse en el pin TX del sensor al arrancar?

Al alimentar el sensor con 5 V:
1. **Periodo de inicialización:** ~300–500 ms sin datos (el sensor calibra internamente)
2. **Inicio de transmisión:** el pin TX comienza a enviar tramas de 400 bytes a 921600 bps
3. **Señal esperada con osciloscopio:**
   - Nivel de reposo: **3.3 V** (UART idle = HIGH)
   - Primer byte de cada trama: **0x57** (frame_header)
   - Frecuencia de tramas: ~10 Hz (una trama cada ~100 ms)
   - Duración de cada trama: 400 bytes × 10 bits / 921600 bps ≈ **4.3 ms**

**Si no se ve actividad en el pin TX:**
- Verificar VCC = 5 V (no 3.3 V)
- Verificar GND común entre sensor y osciloscopio
- Verificar que el sensor está configurado en modo UART (no CAN) — usar NAssistant si es necesario
- Verificar que el cable GH1.25 no está invertido (pins invertidos = VCC a GND → daño posible)

---

## 4. Configuración UART

### 4.1 Baudrate por defecto

| Parámetro | Valor de fábrica | Configurable |
|-----------|------------------|--------------|
| **Baudrate** | **921600 bps** | Sí, con NAssistant (también 115200, 230400, 460800, etc.) |
| **Data bits** | 8 | No |
| **Paridad** | Ninguna | No |
| **Stop bits** | 1 | No |
| **Flow control** | Ninguno | No |

### 4.2 Formato de trama (NLink_TOFSense_M_Frame0)

Según el código de referencia oficial de Nooploop (`nlink_tofsensem_frame0.c`):

**Estructura de trama completa (400 bytes para modo 8×8):**

| Offset | Campo | Tamaño | Valor/Descripción |
|--------|-------|--------|-------------------|
| 0 | `frame_header` | 1 B | **0x57** (constante) |
| 1 | `function_mark` | 1 B | **0x01** (Frame0 del TOFSense-M) |
| 2 | `reserved` | 1 B | Reservado |
| 3 | `id` | 1 B | ID del sensor |
| 4–7 | `system_time` | 4 B | Timestamp en ms (little-endian) |
| 8 | `pixel_count` | 1 B | 64 (8×8) o 16 (4×4) |
| 9–392 | `pixels[64]` | 384 B | 64 píxeles × 6 bytes cada uno |
| 393–398 | `reserved1[6]` | 6 B | Reservado |
| 399 | `sum` | 1 B | Checksum: suma de bytes [0..398] mod 256 |

**Estructura de cada píxel (6 bytes):**

| Offset (relativo) | Campo | Tamaño | Formato |
|-------------------|-------|--------|---------|
| 0–2 | `dis` | 3 B | int24 LE con signo (dividir por 1000.0 → metros) |
| 3 | `dis_status` | 1 B | 0 = válido, ≠ 0 = inválido |
| 4–5 | `signal_strength` | 2 B | uint16 LE |

### 4.3 ¿El sensor transmite automáticamente o espera comandos?

**Transmite automáticamente** (modo activo por defecto):
- El sensor envía tramas de distancia continuamente sin necesidad de polling
- Frecuencia: ~10 Hz (una trama cada ~100 ms)
- No se requiere enviar comandos al sensor para recibir datos
- Por eso el pin RX del sensor queda sin conectar en esta aplicación

**Modo query (opcional):**
- Se puede configurar con NAssistant para que el sensor solo transmita cuando se le envía un comando de lectura
- No se usa en este proyecto (modo activo es más simple y fiable)

---

## Resumen de errores comunes

| Síntoma | Causa más probable | Solución |
|---------|-------------------|----------|
| Estado INVALID permanente | VCC = 3.3 V en vez de 5 V | Cambiar a fuente de 5 V |
| Sin datos UART | Cable TX/RX no cruzado | Sensor TX (pin 4) → ESP32 RX (GPIO18) |
| Checksum fallido | Baudrate incorrecto | Verificar 921600 bps en ambos lados |
| Todos los píxeles inválidos | Sensor en modo CAN | Reconfigurar a modo UART con NAssistant |
| Lecturas erráticas | Cable largo sin desacoplo | Añadir 100 nF en VCC-GND cerca del sensor |
| Bootloop ESP32 | HardwareSerial en constructor global | Usar `new (std::nothrow)` en `init()` |

---

## Referencias oficiales

- [TOFSense-M User Manual V3.0 (PDF)](https://ftp.nooploop.com/downloads/tofsense/TOFSense-M_User_Manual_V3.0_en.pdf)
- [TOFSense-M Datasheet V3.0 (PDF)](https://ftp.nooploop.com/downloads/tofsense/TOFSense-M_Datasheet_V3.0_en.pdf)
- [Nooploop Hardware Connection Guide](https://support.nooploop.com/tofsense/hardware/)
- [Nooploop Protocol Documentation](https://support.nooploop.com/tofsense/protocol/)
- [Nooploop NLink Parser (GitHub)](https://github.com/nooploop-dev/nlink_unpack)
- [NAssistant Configuration Tool](https://support.nooploop.com/tofsense/download/)
