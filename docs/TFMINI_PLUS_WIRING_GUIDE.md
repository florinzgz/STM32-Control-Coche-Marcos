# Guía de Conexión TF-Mini Plus — ESP32-S3

**Fecha:** 2026-04-09  
**Sensor:** Benewake TF-Mini Plus — LiDAR Time-of-Flight de punto único  
**MCU:** ESP32-S3 DevKitC-1 (UART1, GPIO18 RX)  
**Firmware:** `esp32/src/sensors/obstacle_sensor.h` con `OBSTACLE_SENSOR_ENABLED=1`, `SENSOR_TYPE=SENSOR_TYPE_TFMINI`  
**Referencia protocolo CAN:** `docs/CAN_CONTRACT_FINAL.md` rev 1.3 (0x208, 0x209)

---

## 1. Especificaciones del sensor TF-Mini Plus

| Parámetro | Valor |
|-----------|-------|
| **Voltaje VCC** | **5 V DC** (obligatorio, no funciona a 3.3 V) |
| **Consumo típico** | ~120 mA (pico ~140 mA) |
| **Interfaz** | UART TTL 3.3 V, 115200 bps, 8N1 |
| **Rango** | 10 cm – 12 m (condiciones nominales) |
| **Frecuencia** | 100 Hz (trama cada ~10 ms) |
| **Trama** | 9 bytes: `[0x59][0x59][DIST_L][DIST_H][STR_L][STR_H][TEMP_L][TEMP_H][CHK]` |
| **Conector** | Cable de 4 hilos con terminales (rojo, negro, verde, blanco) |
| **Dimensiones** | 35 × 21 × 18 mm |

### Niveles lógicos UART

El TF-Mini Plus transmite a **3.3 V TTL** nativo. A diferencia del TOFSense-M (que produce 3.5–3.6 V), el TF-Mini Plus **NO necesita divisor de tensión ni level shifter** para conectarse directamente al ESP32-S3.

> ✅ **Conexión directa:** El nivel lógico del TX del TF-Mini Plus es 3.3 V, compatible directamente con los GPIO del ESP32-S3 (máximo absoluto 3.6 V). Hay margen suficiente (300 mV) para operación segura.

---

## 2. Identificación de cables

El TF-Mini Plus viene con un cable de 4 hilos. **Los colores estándar son:**

| Color del cable | Señal | Función |
|-----------------|-------|---------|
| 🔴 **Rojo** | **VCC** | Alimentación +5 V |
| ⚫ **Negro** | **GND** | Masa / referencia |
| 🟢 **Verde** | **TX** (salida del sensor) | Datos del sensor → ESP32 RX |
| ⚪ **Blanco** | **RX** (entrada al sensor) | Comandos ESP32 TX → sensor (opcional) |

> ⚠️ **IMPORTANTE:** Verifica los colores con la documentación de tu lote específico. Algunos fabricantes pueden variar los colores. El datasheet de Benewake es la referencia definitiva.

---

## 3. Esquema de conexión con ESP32-S3

### 3.1 Conexión mínima (modo solo lectura — configuración recomendada)

```
TF-Mini Plus                          ESP32-S3 DevKitC-1
┌──────────────┐                      ┌───────────────────┐
│              │                      │                   │
│  VCC (rojo)  ├──────────────────────┤  5V (pin VBUS)    │
│              │                      │                   │
│  GND (negro) ├──────────────────────┤  GND              │
│              │                      │                   │
│  TX  (verde) ├──────── directo ─────┤  GPIO 18 (UART1 RX)│
│              │                      │                   │
│  RX (blanco) ├── NO CONECTADO       │                   │
│              │                      │                   │
└──────────────┘                      └───────────────────┘
```

**Solo 3 cables:** rojo (5V), negro (GND), verde (TX → GPIO 18).

### 3.2 ¿Se necesita divisor de tensión?

**NO.** El TF-Mini Plus opera a 3.3 V TTL nativo, compatible directamente con el ESP32-S3. No se requiere ningún componente adicional entre el sensor y la ESP32.

### 3.3 ¿Se necesita conectar el cable blanco (RX del sensor)?

**No para operación normal.** El TF-Mini Plus transmite datos de distancia continuamente en su modo de fábrica (modo activo, 115200 bps, 100 Hz). El firmware del ESP32 funciona en modo solo lectura (`Config::txPin = -1`).

El cable blanco (RX) solo es necesario si se desea enviar comandos de configuración al sensor por UART (cambiar frecuencia, modo, etc.). Para uso normal, **dejarlo sin conectar**.

---

## 4. Alimentación

### 4.1 Fuente de 5 V

El TF-Mini Plus requiere **5 V** para funcionar correctamente. Opciones de alimentación:

| Fuente | Pin ESP32 | Notas |
|--------|-----------|-------|
| **VBUS (USB)** | Pin `5V` o `VBUS` | Solo cuando USB está conectado |
| **Regulador externo 5 V** | — | Recomendado para instalación permanente |
| **VIN del sistema** | — | Si la placa tiene regulador 5 V onboard |

> ⚠️ **No alimentar a 3.3 V.** El láser interno necesita 5 V. A 3.3 V el sensor no transmite datos o produce lecturas erráticas.

### 4.2 GND común

El GND del sensor **DEBE** estar conectado al GND del ESP32-S3. Sin referencia de masa común, la comunicación UART no funcionará.

---

## 5. Protocolo de datos

### 5.1 Formato de trama (9 bytes)

```
Byte:  [0]   [1]   [2]     [3]     [4]     [5]     [6]     [7]     [8]
       0x59  0x59  DIST_L  DIST_H  STR_L   STR_H   TEMP_L  TEMP_H  CHECKSUM
       ────  ────  ──────────────  ──────────────  ──────────────  ────────
       Header      Distance (cm)   Strength        Temperature     Sum[0..7]&0xFF
```

| Campo | Bytes | Formato | Descripción |
|-------|-------|---------|-------------|
| Header | 0–1 | `0x59 0x59` | Inicio de trama fijo |
| Distance | 2–3 | uint16 LE (cm) | Distancia al obstáculo en centímetros |
| Strength | 4–5 | uint16 LE | Intensidad de señal (< 100 = no fiable, 65535 = saturado) |
| Temperature | 6–7 | uint16 LE | Temperatura del chip (reservado, no usado en el driver) |
| Checksum | 8 | uint8 | Suma de bytes [0..7] mod 256 |

### 5.2 Validaciones del driver

El firmware (`obstacle_sensor.cpp`) aplica las siguientes validaciones:

1. **Header:** bytes 0 y 1 deben ser `0x59`
2. **Checksum:** suma de bytes [0..7] & 0xFF == byte [8]
3. **Distancia inválida:** 0 cm (sin target) o 65535 cm (fuera de rango) → rechazado
4. **Señal débil:** strength < 100 → rechazado (lectura no fiable)
5. **Overflow:** cm × 10 > 65535 → rechazado (protección uint16_t)
6. **Rango:** clamping a [100 mm, 12000 mm] (configurable)
7. **Sensor atascado:** misma distancia (±10 mm) durante > 1 s con vehículo en movimiento → `stuck = true`

### 5.3 Zonas de distancia (mapping ESP32 → CAN → STM32)

| Zona | Distancia | Color HMI | Escala potencia | Descripción |
|------|-----------|-----------|-----------------|-------------|
| 4 | < 200 mm | 🔴 Rojo | 0.0 (parada total) | Emergencia |
| 3 | 200–500 mm | 🟠 Naranja | 0.3 (30%) | Crítico |
| 2 | 500–1000 mm | 🟡 Amarillo | 0.7 (70%) | Advertencia |
| 1 | 1000–1500 mm | 🟢 Verde claro | 0.85 (85%) | Precaución |
| 0 | > 1500 mm | 🟢 Verde | 1.0 (100%) | Normal |

---

## 6. Verificación post-conexión

### 6.1 Verificar alimentación

1. Conectar rojo → 5V, negro → GND
2. El LED azul del TF-Mini Plus debe encenderse
3. Medir con multímetro: VCC del sensor debe ser ≥ 4.8 V

### 6.2 Verificar datos UART

1. Flashear firmware: `pio run -t upload`
2. Abrir monitor serie: `pio device monitor -b 115200`
3. Buscar el mensaje de inicialización:
   ```
   [OBSTACLE] TF-Mini Plus init (UART1, 115200 bps, rxBuf 512, rxPin 18, range 100–12000 mm)
   ```
4. Esperar diagnóstico (cada 5 segundos):
   ```
   [OBSTACLE] Diag 5s: OK=~500 cksumFail=0 hdrFail=0 noTarget=0 ...
   ```
   - `OK ≈ 500` (100 Hz × 5 s): lectura completa de todas las tramas ✅
   - `OK > 0` pero < 100: el main loop no drena todas las tramas ⚠️
   - `cksumFail > 0`: problema de cableado o baud rate ⚠️
   - Sin datos: verificar conexión TX → GPIO 18 y alimentación 5 V ❌

### 6.3 Verificar CAN

1. Con la STM32 encendida y el bus CAN activo, verificar que la STM32 recibe frames 0x208
2. El rolling counter (byte 4) debe incrementar en cada frame
3. El `sensor_health` (byte 3) debe ser 1 cuando hay obstáculo en rango

---

## 7. Diagnóstico de problemas

| Síntoma | Causa probable | Solución |
|---------|---------------|----------|
| No aparece mensaje `[OBSTACLE]` al boot | `OBSTACLE_SENSOR_ENABLED=0` | Verificar que `obstacle_sensor.h` tiene `OBSTACLE_SENSOR_ENABLED 1` |
| `Diag: no UART data` | Cable TX no conectado a GPIO 18 | Verificar cable verde → GPIO 18 |
| `Diag: no UART data` | Sensor sin alimentación 5V | Verificar cable rojo → 5V, LED azul encendido |
| `cksumFail` alto | Baud rate incorrecto | TF-Mini Plus usa 115200 por defecto |
| `noTarget` alto | Sensor apuntando al vacío o superficie reflectante | Probar con obstáculo sólido a 30–50 cm |
| Distancias siempre 0 | Conector mal insertado o cable dañado | Probar continuidad de cables |
| `stuck=true` en logs | Sensor cubierto / obstruido | Limpiar la lente del sensor |

---

## 8. Diferencias con TOFSense-M

| Característica | TF-Mini Plus | TOFSense-M 8×8 |
|----------------|-------------|-----------------|
| **Tipo** | Punto único | Matriz 8×8 (64 píxeles) |
| **Rango** | 10 cm – 12 m | 2 cm – 4 m |
| **Baud rate** | 115200 | 921600 |
| **Trama** | 9 bytes | 400 bytes |
| **Nivel TX** | 3.3 V (directo) | 3.5–3.6 V (necesita divisor) |
| **Consumo** | ~120 mA | ~200 mA |
| **Conector** | Cable 4 hilos | GH1.25 4P |
| **FOV** | ~3.6° | ~45° (8×8) |
| **Frecuencia** | 100 Hz | 10 Hz |
| **Precio** | ~15–20 € | ~60–80 € |

> 📌 **Ventaja del TF-Mini Plus para este proyecto:** mayor rango (12 m vs 4 m), conexión directa sin divisor de tensión, menor consumo, y frecuencia más alta (100 Hz vs 10 Hz). Su FOV más estrecho (3.6°) es suficiente para detección frontal de obstáculos en un vehículo eléctrico infantil que se mueve a baja velocidad (< 10 km/h).

---

*Documento generado: 2026-04-09*  
*Referencia firmware: `esp32/src/sensors/obstacle_sensor.h`, `esp32/src/sensors/obstacle_sensor.cpp`*  
*Referencia CAN: `docs/CAN_CONTRACT_FINAL.md` rev 1.3 (§3.4, frames 0x208 y 0x209)*
