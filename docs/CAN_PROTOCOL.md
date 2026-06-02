# 📡 Protocolo CAN - ESP32↔STM32

> ⚠️ **DEPRECATED:** This document is superseded by [`CAN_CONTRACT_FINAL.md`](CAN_CONTRACT_FINAL.md) (rev 1.3), which is the authoritative CAN protocol reference. Refer to that document for current pin assignments (PA11/PA12), message definitions, and payload formats.

**Comunicación Determinística para Control Vehicular**

---

## 📋 Tabla de Contenidos

1. [Especificaciones Técnicas](#-especificaciones-técnicas)
2. [Arquitectura de Comunicación](#-arquitectura-de-comunicación)
3. [Mensajes de Control (ESP32→STM32)](#-mensajes-de-control-esp32stm32)
4. [Mensajes de Estado (STM32→ESP32)](#-mensajes-de-estado-stm32esp32)
5. [Mensajes de Heartbeat](#-mensajes-de-heartbeat)
6. [Mensajes de Diagnóstico](#-mensajes-de-diagnóstico)
7. [Gestión de Errores](#-gestión-de-errores)
8. [Timing y Periodicidad](#-timing-y-periodicidad)

---

## 🔧 Especificaciones Técnicas

### Parámetros del Bus CAN

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Standard** | CAN 2.0A | 11-bit IDs (NO CAN-FD) |
| **Bitrate** | 500 kbps | Máxima fiabilidad para automotive |
| **Transceptor** | TJA1051T/3 | High-speed CAN transceiver |
| **Terminación** | 120Ω × 2 | En ambos extremos del bus |
| **Longitud máx. bus** | 40 metros @ 500 kbps | |
| **Nodos** | 2 (ESP32-S3 + STM32G474RE) | |
| **Topología** | Lineal (punto a punto) | |

### Configuración FDCAN1 (STM32G474RE)

```c
// Configuración FDCAN1 @ 500 kbps
FDCAN_InitTypeDef hfdcan1;
hfdcan1.ClockDivider = FDCAN_CLOCK_DIV1;
hfdcan1.FrameFormat = FDCAN_FRAME_CLASSIC;       // CAN 2.0A
hfdcan1.Mode = FDCAN_MODE_NORMAL;
hfdcan1.AutoRetransmission = ENABLE;
hfdcan1.TransmitPause = DISABLE;

// Nominal Bit Timing @ 500 kbps (170 MHz APB1)
hfdcan1.NominalPrescaler = 17;                   // 170 MHz / 17 = 10 MHz
hfdcan1.NominalTimeSeg1 = 14;                    // 14 TQ
hfdcan1.NominalTimeSeg2 = 5;                     // 5 TQ
hfdcan1.NominalSyncJumpWidth = 1;                // 1 TQ
// Total: (1 + 14 + 5) = 20 TQ → 10 MHz / 20 = 500 kbps
```

---

## 🏗️ Arquitectura de Comunicación

### Roles de los Nodos

```
┌───────────────────────────────────────────────────────────────┐
│                         CAN Bus @ 500 kbps                     │
└───────────┬─────────────────────────────────────┬─────────────┘
            │                                     │
    ┌───────▼─────────┐                  ┌────────▼────────┐
    │   ESP32-S3      │                  │   STM32G474RE   │
    │   (MAESTRO)     │                  │   (ESCLAVO)     │
    │                 │                  │                 │
    │ - HMI Display   │  ──Commands──►   │ - Control Final │
    │ - User Input    │                  │ - Motores       │
    │ - Audio/Visual  │  ◄──Status────   │ - Sensores      │
    │ - Diagnóstico   │                  │ - ABS/TCS       │
    └─────────────────┘                  └─────────────────┘
```

**Principios de Diseño:**
1. **ESP32 sugiere, STM32 decide:** El ESP32 envía comandos de usuario, pero el STM32 tiene autoridad final sobre actuadores.
2. **Heartbeat mutuo:** Ambos nodos deben enviar señales de vida cada 100 ms.
3. **Timeout de seguridad:** Si el ESP32 no responde en 250 ms, el STM32 entra en LIMP_HOME (no SAFE). Si el STM32 no responde, el ESP32 muestra alerta crítica.
4. **Validación de comandos:** El STM32 valida todos los comandos antes de ejecutarlos.

---

## 🎮 Mensajes de Control (ESP32→STM32)

### 0x011 - HEARTBEAT_ESP32

**Propósito:** Indicar que el ESP32-S3 está operativo.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0 | `alive_counter` | uint8_t | 0-255 | - | Contador cíclico (incrementa cada mensaje) |

**DLC:** 1

**Frecuencia:** 100 ms (10 Hz)

**Ejemplo:**
```
ID: 0x011  DLC: 1  Data: [0x42]
```

**Detección de freeze:** El STM32 detecta 5 tramas consecutivas con el mismo valor de `alive_counter`
como ESP32 congelado y entra en modo LIMP_HOME (no SAFE).

---

### 0x100 - CMD_THROTTLE

**Propósito:** Comando de aceleración desde pedal/HMI.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0 | `throttle_pct` | uint8_t | 0-100 | % | Porcentaje de aceleración solicitado |

**DLC:** 1

**Frecuencia:** 50 ms (20 Hz) cuando hay cambio, 0 si inactivo

**Validación STM32:**
- Si `throttle_pct > 100` → Rechazar (error)
- Si shifter != FORWARD → Limitar a 0% (seguridad)
- Si ABS/TCS activo → Puede reducir throttle automáticamente

**Ejemplo:**
```
ID: 0x100  DLC: 1  Data: [0x4B]  // 75% throttle
```

---

### 0x101 - CMD_STEERING

**Propósito:** Comando de dirección desde HMI o control automático.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0-1 | `steering_angle_x10` | int16_t LE | -450 a +450 | 0.1° | Ángulo × 10, little-endian. Ej: -250 = -25.0° |

**DLC:** 2

**Frecuencia:** 50 ms (20 Hz)

**Decodificación STM32:**
```c
int16_t angle_raw = (int16_t)(rx_payload[0] | (rx_payload[1] << 8));
float requested_deg = (float)angle_raw / 10.0f;  // resolución 0.1°
```

**Validación STM32:**
- Verificar límites de encoder (-720 a +720 conteos)
- Limitar velocidad de cambio (rate limiter @ 200°/s max)

**Ejemplo:**
```
ID: 0x101  DLC: 2  Data: [0x06, 0xFF]  // 0xFF06 = -250 → -25.0°
```

---

### 0x102 - CMD_MODE

**Propósito:** Comando de cambio de modo de conducción y marcha.

| Byte | Campo | Tipo | Rango | Notas |
|------|-------|------|-------|-------|
| 0 | `mode_flags` | uint8_t | Bitmask | bit0=4×4, bit1=tank_turn |
| 1 | `gear` | uint8_t | 0-4 | 0=Park, 1=Reverse, 2=Neutral, 3=Forward, 4=Forward_D2 (opcional) |

**DLC:** 1 o 2 (byte 1 es opcional — si se omite, la marcha no cambia)

**Frecuencia:** On-demand (solo cuando cambia shifter o interruptor 2WD/4WD)

**Validación STM32:**
- Cambio de modo (4x4/tank): solo permitido si velocidad < 0.5 km/h
- Cambio de marcha: solo permitido si velocidad ≤ 1 km/h
- Envía CMD_ACK (0x103) con resultado

**Ejemplo:**
```
ID: 0x102  DLC: 2  Data: [0x01, 0x03]  // 4×4 activo, marcha Forward
```

---

### 0x103 - CMD_ACK (STM32→ESP32)

**Propósito:** Confirmación de recepción y resultado de un comando.

| Byte | Campo | Tipo | Rango | Notas |
|------|-------|------|-------|-------|
| 0 | `cmd_id_low` | uint8_t | - | Byte bajo del ID del comando confirmado (ej. 0x02 para CMD_MODE) |
| 1 | `result` | uint8_t | 0-3 | 0=OK, 1=Rejected, 2=Invalid, 3=Blocked_by_safety |
| 2 | `system_state` | uint8_t | 0-6 | Estado actual del sistema |

**DLC:** 3

**Frecuencia:** On-demand (tras CMD_MODE o SERVICE_CMD)

---

### 0x110 - SERVICE_CMD (ESP32→STM32)

**Propósito:** Comando de Service Mode para habilitar/deshabilitar módulos individuales y disparar calibraciones persistentes.

| Byte | Campo | Tipo | Rango | Notas |
|------|-------|------|-------|-------|
| 0 | `action` | uint8_t | 0/1/0xE0/0xF0–0xF6/0xFE/0xFF | 0=Disable, 1=Enable, 0xE0=Relay override, 0xF0–0xF4=Factory resets parciales, 0xF5=**Pedal calibration**, 0xF6=**I2C service scan**, 0xFE=Clear error log, 0xFF=Factory_Restore |
| 1 | `module_id` / `sub-opcode` / `mask` | uint8_t | 0-N | Significado dependiente de `action` |

**DLC:** 2

**Frecuencia:** On-demand

#### 0xF5 — Calibración persistente del pedal acelerador

Cuando `action == 0xF5`, el byte 1 es un sub-opcode que selecciona la operación.

| Sub-opcode | Nombre | Efecto | Gates de seguridad |
|-----------:|--------|--------|--------------------|
| `0x01` | `CAPTURE_MIN` | Muestrea 8 lecturas ADC del pedal a 50 ms y guarda la media en *pending MIN* si la dispersión ≤ 8 cuentas. | STANDBY + startup_inhibit + pedal<3% + plausible + rueda<0.3 km/h |
| `0x02` | `CAPTURE_MAX` | Idem para *pending MAX*. | Mismas gates |
| `0x03` | `SAVE` | Valida el par pendiente y persiste en página 124 (0x0807C000). Aplica los endpoints inmediatamente. | Mismas gates + validación dura: `min ≥ 50`, `max ≤ 2600`, `max > min`, `max − min ≥ 800` |
| `0x04` | `RESET_DEFAULTS` | Persiste los defaults de fábrica (150 / 2413) y los aplica. | Mismas gates |
| `0x05` | `QUERY` | Solicita un burst de 10 frames `0x308` a 10 Hz (1 segundo). | Sin gates |

Cada sub-opcode responde con `CAN_ID_CMD_ACK (0x103)` y los resultados estándar `OK`/`INVALID`/`REJECTED`/`BLOCKED_BY_SAFETY`. **DLC de CMD_ACK preservado (3 bytes)** — sin cambios de contrato.

**Fallback:** Si la página 124 contiene CRC/magic inválido o un par fuera de rango, el STM32 carga los defaults compile-time (150 / 2413) silenciosamente — el boot nunca se bloquea y el pedal nunca queda inutilizable.

#### 0xF6 — Barrido I2C de modo servicio (diagnóstico)

Cuando `action == 0xF6` (sin sub-opcode), el STM32 ejecuta un sondeo I2C **activo** (`Sensor_RunI2CServiceScan()`) y responde con dos tramas de diagnóstico además del `CMD_ACK` estándar:

- **0x30B DIAG_I2C_SCAN** — presencia de mux/INA226, nivel de reposo SDA/SCL y resultado de `I2C_BusRecovery()`.
- **0x30C DIAG_FDCAN** — volcado de contadores de error FDCAN (TEC/REC/LEC/estado).

Sólo lectura: no altera ningún estado de control ni de seguridad. Pensado para localizar fallos del lado I2C (dirección incorrecta, SDA/SCL bloqueadas, mux/INA muertos) — preguntas G/H/I/J de la auditoría.

---

### 0x120 - CMD_LED (ESP32→STM32)

**Propósito:** Control de relés de alimentación LED WS2812B.

| Byte | Campo | Tipo | Rango | Notas |
|------|-------|------|-------|-------|
| 0 | `front_relay` | uint8_t | 0/1 | 0=OFF, 1=ON (relé PB10 — tira frontal 28 LEDs) |
| 1 | `rear_relay` | uint8_t | 0/1 | 0=OFF, 1=ON (relé PB11 — tira trasera 16 LEDs) |

**DLC:** 2

**Frecuencia:** On-demand

---

## 📊 Mensajes de Estado (STM32→ESP32)

### 0x001 - HEARTBEAT_STM32

**Propósito:** Indicar que el STM32 está operativo y en qué estado.

| Byte | Campo | Tipo | Rango | Notas |
|------|-------|------|-------|-------|
| 0 | `alive_counter` | uint8_t | 0-255 | Contador cíclico |
| 1 | `system_state` | uint8_t | 0-6 | 0=Boot, 1=Standby, 2=Active, 3=Degraded, 4=Safe, 5=Error, 6=LimpHome |
| 2 | `fault_flags` | uint8_t | Bitmask | Ver tabla de fallos |
| 3 | `error_code` | uint8_t | 0-13 | Safety_Error_t — código de fallo específico para HMI |
| 4 | `status_flags` | uint8_t | Bitmask | bit0=startup_inhibit, bit1=4x4, bit2=tank_turn, bit3-5=DS18B20 count |

**DLC:** 5

**Frecuencia:** 100 ms (10 Hz)

**Fault Flags (Byte 2):**

| Bit | Fallo | Descripción |
|-----|-------|-------------|
| 0 | `CAN_TIMEOUT` | No se recibe heartbeat ESP32 >250 ms |
| 1 | `TEMP_OVERLOAD` | Algún motor >80°C |
| 2 | `CURRENT_OVERLOAD` | Corriente >umbral seguro |
| 3 | `ENCODER_ERROR` | Encoder dirección desconectado |
| 4 | `WHEEL_SENSOR_ERROR` | Sensor de rueda desconectado |
| 5 | `ABS_ACTIVE` | ABS interviniendo |
| 6 | `TCS_ACTIVE` | TCS interviniendo |
| 7 | `CENTERING` | Centrado de dirección fallido |

**Error Codes (Byte 3):**

| Valor | Error | Descripción |
|-------|-------|-------------|
| 0 | NONE | Sin error |
| 1 | OVERCURRENT | Sobrecorriente >25 A |
| 2 | OVERTEMP | Sobretemperatura >80°C |
| 3 | CAN_TIMEOUT | Timeout CAN ESP32 |
| 4 | SENSOR_FAULT | Fallo de sensor |
| 5 | MOTOR_STALL | Reservado (no implementado) |
| 6 | EMERGENCY_STOP | Parada de emergencia |
| 7 | WATCHDOG | Reset por watchdog |
| 8 | CENTERING | Centrado de dirección fallido |
| 9 | BATTERY_UV_WARN | Batería <20.0 V |
| 10 | BATTERY_UV_CRIT | Batería <18.0 V |
| 11 | I2C_FAILURE | Bus I2C bloqueado |
| 12 | OBSTACLE | Emergencia por obstáculo |
| 13 | CAN_BUSOFF | Bus-off FDCAN |

**Status Flags (Byte 4):**

| Bit | Flag | Descripción |
|-----|------|-------------|
| 0 | `STARTUP_INHIBIT` | Power-On Movement Prevention activa |
| 1 | `MODE_4X4` | Modo 4×4 activo |
| 2 | `TANK_TURN` | Tank turn activo |
| 3-5 | `TEMP_COUNT` | Número de sensores DS18B20 detectados (0-5) |

**Ejemplo:**
```
ID: 0x001  DLC: 5  Data: [0x7F, 0x02, 0x20, 0x00, 0x02]
// alive=0x7F, state=Active(2), faultFlags=ABS(0x20), error=NONE(0), status=4x4(0x02)
```

---

### 0x200 - STATUS_SPEED

**Propósito:** Velocidades de las 4 ruedas.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0-1 | `speed_FL` | uint16_t LE | 0-65535 | 0.1 km/h | Velocidad rueda FL |
| 2-3 | `speed_FR` | uint16_t LE | 0-65535 | 0.1 km/h | Velocidad rueda FR |
| 4-5 | `speed_RL` | uint16_t LE | 0-65535 | 0.1 km/h | Velocidad rueda RL |
| 6-7 | `speed_RR` | uint16_t LE | 0-65535 | 0.1 km/h | Velocidad rueda RR |

**DLC:** 8

**Frecuencia:** 100 ms (10 Hz)

**Codificación STM32:**
```c
float_to_u16_clamped(Wheel_GetSpeed_FL() * 10)  // km/h × 10 → 0.1 km/h units
```

**Ejemplo:**
```
ID: 0x200  DLC: 8  Data: [0x2C, 0x01, 0x30, 0x01, 0x28, 0x01, 0x2A, 0x01]
// FL=30.0 km/h, FR=30.4, RL=29.6, RR=29.8
```

---

### 0x201 - STATUS_CURRENT

**Propósito:** Corrientes de los 4 motores de tracción.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0-1 | `current_FL` | uint16_t LE | 0-65535 | 0.01 A | Motor FL |
| 2-3 | `current_FR` | uint16_t LE | 0-65535 | 0.01 A | Motor FR |
| 4-5 | `current_RL` | uint16_t LE | 0-65535 | 0.01 A | Motor RL |
| 6-7 | `current_RR` | uint16_t LE | 0-65535 | 0.01 A | Motor RR |

**DLC:** 8

**Frecuencia:** 100 ms (10 Hz)

**Codificación STM32:**
```c
float_to_u16_clamped(Current_GetAmps(0) * 100)  // Amps × 100 → 0.01 A units
```

**Nota:** Corriente de dirección y batería se envían en 0x207 (STATUS_BATTERY).

**Ejemplo:**
```
ID: 0x201  DLC: 8  Data: [0x2C, 0x01, 0x2E, 0x01, 0x2A, 0x01, 0x2C, 0x01]
// FL=3.00A, FR=3.02A, RL=2.98A, RR=3.00A
```

---

### 0x202 - STATUS_TEMP

**Propósito:** Temperaturas de los 5 sensores DS18B20.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0 | `temp_FL` | int8_t | -128 a +127 | °C | Temperatura motor FL |
| 1 | `temp_FR` | int8_t | -128 a +127 | °C | Temperatura motor FR |
| 2 | `temp_RL` | int8_t | -128 a +127 | °C | Temperatura motor RL |
| 3 | `temp_RR` | int8_t | -128 a +127 | °C | Temperatura motor RR |
| 4 | `temp_AMB` | int8_t | -128 a +127 | °C | Temperatura ambiente |

**DLC:** 5

**Frecuencia:** 1000 ms (1 Hz)

**Umbrales de Protección (firmware):**
- < 80°C → Normal
- ≥ 80°C → DEGRADED (`TEMP_WARNING_C`)
- ≥ 90°C → SAFE (`TEMP_CRITICAL_C`)
- ≥ 130°C → Corte individual por motor

**Ejemplo:**
```
ID: 0x202  DLC: 5  Data: [0x37, 0x39, 0x38, 0x36, 0x19]
// FL=55°C, FR=57°C, RL=56°C, RR=54°C, AMB=25°C
```

---

### 0x203 - STATUS_SAFETY

**Propósito:** Estado de sistemas de seguridad (ABS/TCS).

| Byte | Campo | Tipo | Rango | Notas |
|------|-------|------|-------|-------|
| 0 | `abs_active` | uint8_t | 0/1 | 1 = ABS interviniendo |
| 1 | `tcs_active` | uint8_t | 0/1 | 1 = TCS interviniendo |
| 2 | `error_code` | uint8_t | 0-13 | Safety_Error_t (mismo que heartbeat byte 3) |

**DLC:** 3

**Frecuencia:** 100 ms (10 Hz)

**Ejemplo:**
```
ID: 0x203  DLC: 3  Data: [0x01, 0x00, 0x02]
// ABS activo, TCS inactivo, error=OVERTEMP(2)
```

---

### 0x204 - STATUS_STEERING

**Propósito:** Posición angular del motor de dirección.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0-1 | `angle` | int16_t LE | ±4667 | conteos | Posición encoder, little-endian |
| 2 | `calibrated` | uint8_t | 0/1 | - | 1 = centrado calibrado |

**DLC:** 3

**Frecuencia:** 100 ms (10 Hz)

**Conversión:**
- 1 conteo = 0.075° (encoder E6B2-CWZ6C 1200 PPR × 4 = 4800 conteos/rev)
- Centro = 0 conteos (calibrado en inicio)
- Rango de viaje: ±350° (±4667 conteos)

**Ejemplo:**
```
ID: 0x204  DLC: 3  Data: [0xB4, 0xFF, 0x01]
// Posición: -76 conteos = -5.7°, calibrado
```

---

### 0x205 - STATUS_TRACTION

**Propósito:** Nivel de tracción disponible por rueda (escala ABS/TCS).

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0 | `traction_FL` | uint8_t | 0-100 | % | Tracción disponible rueda FL |
| 1 | `traction_FR` | uint8_t | 0-100 | % | Tracción disponible rueda FR |
| 2 | `traction_RL` | uint8_t | 0-100 | % | Tracción disponible rueda RL |
| 3 | `traction_RR` | uint8_t | 0-100 | % | Tracción disponible rueda RR |

**Frecuencia:** 100 ms (10 Hz)

**Escala:**
- 100 % = potencia completa (sin intervención ABS/TCS en esta rueda)
- 0 % = rueda completamente inhibida (ABS ha cortado esta rueda)
- Valores intermedios = TCS está limitando progresivamente esta rueda

**Codificación:** `(uint8_t)(safety_status.wheel_scale[i] * 100.0f)`

**Ejemplo:**
```
ID: 0x205  DLC: 4  Data: [0x64, 0x64, 0x00, 0x4B]
// FL=100% (OK), FR=100% (OK), RL=0% (ABS activo), RR=75% (TCS limitando)
```

---

### 0x206 - STATUS_TEMP_MAP

**Propósito:** Mapa explícito de temperaturas por sensor DS18B20.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0 | `temp_motor_FL` | int8_t | -128 a +127 | °C | Temperatura motor FL (sensor índice 0) |
| 1 | `temp_motor_FR` | int8_t | -128 a +127 | °C | Temperatura motor FR (sensor índice 1) |
| 2 | `temp_motor_RL` | int8_t | -128 a +127 | °C | Temperatura motor RL (sensor índice 2) |
| 3 | `temp_motor_RR` | int8_t | -128 a +127 | °C | Temperatura motor RR (sensor índice 3) |
| 4 | `temp_ambient` | int8_t | -128 a +127 | °C | Temperatura ambiente (sensor índice 4) |

**Frecuencia:** 1000 ms (1 Hz)

**Correspondencia de sensores:**

| Byte | Índice sensor | Ubicación física |
|------|--------------|------------------|
| 0 | `Temperature_Get(0)` | Motor delantero izquierdo (FL) |
| 1 | `Temperature_Get(1)` | Motor delantero derecho (FR) |
| 2 | `Temperature_Get(2)` | Motor trasero izquierdo (RL) |
| 3 | `Temperature_Get(3)` | Motor trasero derecho (RR) |
| 4 | `Temperature_Get(4)` | Ambiente |

**Relación con Service Mode:** Si un sensor está deshabilitado en Service Mode, el valor se sigue reportando (el último valor leído). El sistema de seguridad maneja los umbrales de forma independiente.

**Relación con STATUS_TEMP (0x202):** El mensaje existente 0x202 se mantiene sin cambios. Este mensaje proporciona la misma información con un mapeo byte-a-sensor explícito e inequívoco para el HMI.

**Ejemplo:**
```
ID: 0x206  DLC: 5  Data: [0x37, 0x39, 0x38, 0x36, 0x19]
// Motor FL=55°C, Motor FR=57°C, Motor RL=56°C, Motor RR=54°C, Ambiente=25°C
```

---

### 0x207 - STATUS_BATTERY (STM32→ESP32)

**Propósito:** Corriente y tensión del bus de batería 24 V (INA226 canal 4).

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0-1 | `current_x100` | uint16_t LE | 0-65535 | 0.01 A | Corriente × 100 (ej. 1250 = 12.50 A) |
| 2-3 | `voltage_x100` | uint16_t LE | 0-65535 | 0.01 V | Tensión × 100 (ej. 2400 = 24.00 V) |

**DLC:** 4

**Frecuencia:** 100 ms (10 Hz)

---

### 0x208 - OBSTACLE_DISTANCE (ESP32→STM32)

**Propósito:** Distancia al obstáculo desde sensor TOFSense-M LiDAR (UART1 GPIO 18 del ESP32).

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0-1 | `distance_mm` | uint16_t LE | 20-4000 | mm | Distancia medida |
| 2 | `zone` | uint8_t | 0-4 | - | 0=Normal, 1=Caution, 2=Warning, 3=Critical, 4=Emergency |
| 3 | `health` | uint8_t | 0/1/2 | - | 0=OK, 1=Stuck, 2=NoData |
| 4 | `counter` | uint8_t | 0-255 | - | Contador cíclico |

**DLC:** 5

**Frecuencia:** 66 ms (15 Hz)

---

### 0x209 - OBSTACLE_SAFETY (ESP32→STM32)

**Propósito:** Estado de seguridad del sensor de obstáculos (informativo, para cross-validation).

**DLC:** 8

**Frecuencia:** 100 ms (10 Hz)

---

### 0x20A - STATUS_LIGHTS (STM32→ESP32)

**Propósito:** Estado actual de los relés de iluminación LED.

| Byte | Campo | Tipo | Rango | Notas |
|------|-------|------|-------|-------|
| 0 | `front_relay_state` | uint8_t | 0/1 | Estado relé PB10 (tira frontal) |
| 1 | `rear_relay_state` | uint8_t | 0/1 | Estado relé PB11 (tira trasera) |

**DLC:** 2

**Frecuencia:** 1000 ms (1 Hz)

---

## 🔔 Mensajes de Heartbeat

### Lógica de Heartbeat Mutuo

```c
// STM32: Verificación de heartbeat ESP32
// CAN timeout → LIMP_HOME (NOT SAFE) — communication loss is not a hazard
// Vehicle remains mobile at walking speed with local pedal control.

void Safety_CheckCANTimeout(void) {
    if ((HAL_GetTick() - last_esp32_heartbeat) > CAN_TIMEOUT_HEARTBEAT_MS) {
        // ESP32 no responde → LIMP_HOME (20% torque, 5 km/h cap)
        Safety_SetState(SYS_STATE_LIMP_HOME);
    }
}
```

### Detección de Freeze (5 tramas iguales)

El STM32 compara el `alive_counter` de cada trama ESP32 con la anterior. Si
`HEARTBEAT_COUNTER_FREEZE_COUNT` (5) tramas consecutivas llevan el mismo
contador, el ESP32 se considera congelado → transición a LIMP_HOME.

### Respuesta ante Pérdida de Heartbeat

| Nodo | Timeout | Acción |
|------|---------|--------|
| **STM32** | >250 ms sin ESP32 | LIMP_HOME: 20% torque, 5 km/h, pedal local, sin torque vectoring |
| **ESP32** | >250 ms sin STM32 | 1. Mostrar alerta crítica<br>2. Audio de advertencia<br>3. Log de evento<br>4. Esperar reconexión |

---

## 🚨 Mensajes de Diagnóstico

### 0x300 - DIAG_ERROR

**Propósito:** Reporte de errores críticos.

| Byte | Campo | Tipo | Notas |
|------|-------|------|-------|
| 0 | `error_code` | uint8_t | Código de error (Safety_Error_t) |
| 1 | `subsystem` | uint8_t | 0=Global, 1=Motor, 2=Sensor, 3=CAN |

**DLC:** 2

**Frecuencia:** On-demand (solo cuando ocurre error)

**Códigos de Error (Safety_Error_t):**

| Código | Nombre | Descripción |
|--------|--------|-------------|
| 0 | `NONE` | Sin error |
| 1 | `OVERCURRENT` | Corriente excesiva (>25 A) |
| 2 | `OVERTEMP` | Temperatura ≥80°C |
| 3 | `CAN_TIMEOUT` | Pérdida de heartbeat CAN |
| 4 | `SENSOR_FAULT` | Fallo de sensor / NaN |
| 5 | `MOTOR_STALL` | Motor bloqueado (reservado) |
| 6 | `EMERGENCY_STOP` | Parada de emergencia |
| 7 | `WATCHDOG` | Reset por watchdog |
| 8 | `CENTERING` | Fallo de centrado de dirección |
| 9 | `BATTERY_UV_WARN` | Batería < 20.0 V |
| 10 | `BATTERY_UV_CRIT` | Batería < 18.0 V |
| 11 | `I2C_FAILURE` | Bus I2C bloqueado |
| 12 | `OBSTACLE` | Emergencia por obstáculo o timeout CAN obstáculo |
| 13 | `CAN_BUSOFF` | Condición bus-off FDCAN |

**Ejemplo:**
```
ID: 0x300  DLC: 2  Data: [0x02, 0x01]
// Error: OVERTEMP(2), Subsystem: Motor(1)
```

---

### 0x301 - SERVICE_FAULTS (STM32→ESP32)

**Propósito:** Bitmask de módulos con fallo detectado por Service Mode.

| Byte | Campo | Tipo | Notas |
|------|-------|------|-------|
| 0-3 | `fault_bitmask` | uint32_t LE | Bit N = 1 → módulo N tiene fallo |

**DLC:** 4 · **Frecuencia:** 1000 ms

---

### 0x302 - SERVICE_ENABLED (STM32→ESP32)

**Propósito:** Bitmask de módulos habilitados por Service Mode.

| Byte | Campo | Tipo | Notas |
|------|-------|------|-------|
| 0-3 | `enabled_bitmask` | uint32_t LE | Bit N = 1 → módulo N habilitado |

**DLC:** 4 · **Frecuencia:** 1000 ms

---

### 0x303 - SERVICE_DISABLED (STM32→ESP32)

**Propósito:** Bitmask de módulos deshabilitados por Service Mode.

| Byte | Campo | Tipo | Notas |
|------|-------|------|-------|
| 0-3 | `disabled_bitmask` | uint32_t LE | Bit N = 1 → módulo N deshabilitado |

**DLC:** 4 · **Frecuencia:** 1000 ms

---

### 0x308 - DIAG_PEDAL_CAL (STM32→ESP32)

**Propósito:** Telemetría de calibración del pedal (lectura cruda, endpoints almacenados, endpoints pendientes, gates de seguridad). El STM32 emite ráfagas de 10 frames a 10 Hz (1 s) **solo bajo demanda** tras `SERVICE_CMD/0xF5/QUERY` — sin tráfico permanente en el bus.

El STM32 alterna dos variantes de frame dentro de la ráfaga (selecciona bit 6 de flags) para enviar el par almacenado y el par pendiente en sólo 8 bytes:

| Byte | Campo | Tipo | Notas |
|------|-------|------|-------|
| 0    | `flags` | uint8_t bitmask | bit0=pending MIN capturado, bit1=pending MAX capturado, bit2=par pendiente válido, bit3=slot flash válido, bit4=gates de seguridad OK, bit5=plausible, **bit6=0 → bytes 3-6 son PENDING; bit6=1 → bytes 3-6 son STORED**, bit7=reservado |
| 1-2  | `raw_adc` | uint16_t LE | Lectura cruda 12-bit post-EMA |
| 3-4  | `min` | uint16_t LE | MIN almacenado o pendiente (según bit 6) |
| 5-6  | `max` | uint16_t LE | MAX almacenado o pendiente (según bit 6) |
| 7    | `pedal_percent` | uint8_t | 0..100 saturante |

**DLC:** 8 · **Frecuencia:** On-demand, 10 Hz durante 1 s tras QUERY · **Timeout:** 1 s (10 frames × 100 ms)

**Compatibilidad backward:** nodos que ignoren 0x308 no se ven afectados — la trama no se emite si no hay QUERY pendiente.

---

### 0x309 - DIAG_I2C (STM32→ESP32)

**Propósito:** Diagnóstico de topología del bus I2C para la pantalla **Safe Mode**. Permite distinguir un TCA9548A (mux, 0x70) ausente de un INA226 (0x40) muerto/ausente detrás de un canal concreto, y ver si los fallos son intermitentes (mala pull-up / borne flojo) o permanentes (no conectado). Sólo informativo — ningún camino de control o seguridad lo consume.

| Byte | Campo | Tipo | Notas |
|------|-------|------|-------|
| 0    | `mux_present` | uint8_t | 0 = TCA9548A 0x70 no responde, 1 = presente |
| 1    | `ina_ok_mask` | uint8_t bitmask | bit i = INA226 respondió tras seleccionar canal i del mux. bit0=FL, bit1=FR, bit2=RL, bit3=RR, bit4=BAT, bit5=STEER |
| 2    | `i2c_fail_count` | uint8_t | Transacciones I2C fallidas en el último ciclo de lectura |
| 3    | `i2c_recovery_attempts` | uint8_t | Intentos de recuperación de bus (persistente, se resetea en un ciclo OK) |
| 4    | `flags` | uint8_t bitmask | bit0 = "ever OK" (al menos un INA226 visto sano desde el arranque) |
| 5    | `ina_expected_mask` | uint8_t bitmask | bit i = la rama de alimentación del canal i está energizada esta fase, por lo que su INA226 **debe** responder. Mismo orden de bits que `ina_ok_mask`. Permite a la HMI mostrar una rama sin alimentar como **WAIT PWR** (cian) en vez de FAIL (rojo). Aditivo: firmware previo (DLC 5) omite este byte |

**DLC:** 6 · **Frecuencia:** 1000 ms (1 Hz) · **Prioridad:** Baja

**Diagnóstico por fases de alimentación:** los INA226 de motores (canales 0..3 = FL/FR/RL/RR) están cableados *detrás* del relé de tracción y el de dirección (canal 5) detrás del relé de potencia de dirección. En SAFE/STANDBY esas ramas no tienen alimentación, por lo que sus INA226 no responden por I2C — esto es **normal**. El STM32 sólo cuenta como fallo de bus (recuperación / Error Code 11) los timeouts de canales cuyo bit está activo en `ina_expected_mask`. El INA de batería (canal 4) está siempre alimentado y sigue siendo obligatorio.

**Compatibilidad backward:** nodos que ignoren 0x309 no se ven afectados (aditivo, STM32 → ESP32). Consumidores DLC 5 previos ignoran el byte 5 (la HMI usa máscara por defecto = todos los canales → comportamiento FAIL heredado).

---

### 0x30A - DIAG_CAN_META (STM32→ESP32)

**Propósito:** Meta-diagnóstico de la *entrega* de la trama 0x309. Responde a las preguntas A–D de la auditoría "0x309: NO DATA": ¿se ejecuta `CAN_SendI2CDiag()`?, ¿corre el bloque de 1 Hz?, ¿`TransmitFrame()` encola bien o falla?, ¿se descartan tramas por FIFO TX llena? Todos los contadores saturan (no hacen wrap). Sólo informativo.

| Byte | Campo | Tipo | Notas |
|------|-------|------|-------|
| 0-1  | `diag309_call_count` | uint16_t LE | [A] nº de invocaciones a `CAN_SendI2CDiag()` (satura a 0xFFFF) |
| 2-3  | `tick_1000ms_count` | uint16_t LE | [B] nº de iteraciones del bloque scheduler de 1 Hz (satura) |
| 4    | `diag309_tx_ok` | uint8_t | [C] 0x309 encolada OK en FDCAN (satura a 0xFF) |
| 5    | `diag309_tx_err` | uint8_t | [C] `TransmitFrame()` de 0x309 devolvió error (satura) |
| 6    | `tx_fifo_full_drops` | uint8_t | [D] tramas descartadas por FIFO TX llena (satura) |
| 7    | `flags` | uint8_t bitmask | bit0 = `fdcan_init_ok` |

**DLC:** 8 · **Frecuencia:** 1000 ms (1 Hz, junto a 0x309) · **Prioridad:** Baja

**Compatibilidad backward:** aditivo, STM32 → ESP32; nodos que la ignoren no se ven afectados.

---

### 0x30B - DIAG_I2C_SCAN (STM32→ESP32, on-demand)

**Propósito:** Reporte de un barrido I2C *activo* disparado por `SERVICE_CMD` (0x110) byte0 = `SERVICE_ACTION_I2C_SERVICE` (0xF6). Sondea presencia de mux/INA226, nivel de reposo de SDA/SCL e intenta `I2C_BusRecovery()` si SDA está atascada. Cubre las preguntas G/H/I/J (dirección incorrecta, líneas bloqueadas). Sólo se emite tras el comando; sólo informativo.

| Byte | Campo | Tipo | Notas |
|------|-------|------|-------|
| 0    | `bus_flags` | uint8_t bitmask | bit0 SCL idle high, bit1 SDA idle high, bit2 recovery intentado, bit3 recovery con éxito |
| 1    | `mux_present` | uint8_t | TCA9548A 0x70 respondió |
| 2    | `ina_present_mask` | uint8_t bitmask | bit0..5 = INA226 0x40 respondió tras seleccionar canal 0..5 |
| 3    | `i2c_fail_count` | uint8_t | Contador de fallos I2C del último ciclo |
| 4    | `i2c_recovery_attempts` | uint8_t | Intentos de recuperación de bus (persistente) |
| 5-7  | reservado | — | 0 |

**DLC:** 8 · **Frecuencia:** on-demand (tras 0xF6) · **Prioridad:** Baja

**Compatibilidad backward:** aditivo, STM32 → ESP32; nodos que la ignoren no se ven afectados.

---

### 0x30C - DIAG_FDCAN (STM32→ESP32, on-demand)

**Propósito:** Volcado de los contadores de error del periférico FDCAN (TEC/REC/LEC/estado) para confirmar la salud física del bus desde el lado STM32. Se emite junto a 0x30B tras el comando 0xF6. Espeja el bloque `can_diag`. Sólo informativo.

| Byte | Campo | Tipo | Notas |
|------|-------|------|-------|
| 0    | `last_error_code` | uint8_t | PSR.LEC (0=none, 1=stuff, 2=form, 3=ack, 4=bit1, 5=bit0, 6=CRC) |
| 1    | `state_flags` | uint8_t bitmask | bit0 error_passive, bit1 bus_off, bit2 warning |
| 2    | `tec` | uint8_t | Transmit Error Counter |
| 3    | `rec` | uint8_t | Receive Error Counter |
| 4    | `tx_nack_flag` | uint8_t | 1 = fallos TX repetidos detectados |
| 5    | `tx_consec_fail` | uint8_t | Fallos TX consecutivos (0-255) |

**DLC:** 6 · **Frecuencia:** on-demand (tras 0xF6) · **Prioridad:** Baja

**Compatibilidad backward:** aditivo, STM32 → ESP32; nodos que la ignoren no se ven afectados.

---

### Prioridades de Mensajes

| ID | Mensaje | Prioridad CAN | Descripción |
|----|---------|---------------|-------------|
| 0x001 | HEARTBEAT_STM32 | **Alta** | Crítico para seguridad |
| 0x011 | HEARTBEAT_ESP32 | **Alta** | Crítico para seguridad |
| 0x100 | CMD_THROTTLE | Media | Control tiempo real |
| 0x101 | CMD_STEERING | Media | Control tiempo real |
| 0x102 | CMD_MODE | Media | Cambio de marcha/modo |
| 0x103 | CMD_ACK | Media | Confirmación de comando |
| 0x110 | SERVICE_CMD | Baja | Diagnóstico |
| 0x120 | CMD_LED | Baja | Control LED |
| 0x200-0x207 | STATUS_* | Baja | Telemetría |
| 0x208-0x209 | OBSTACLE_* | Media | Seguridad obstáculos |
| 0x20A | STATUS_LIGHTS | Baja | Telemetría LED |
| 0x300 | DIAG_ERROR | **Alta** | Errores críticos |
| 0x301-0x303 | SERVICE_* | Baja | Diagnóstico Service Mode |
| 0x304-0x307 | DIAG_* | Baja | Diagnóstico (error log, debounce) |
| 0x308 | DIAG_PEDAL_CAL | Baja | Telemetría calibración pedal (on-demand, 1 s tras QUERY) |
| 0x309 | DIAG_I2C | Baja | Diagnóstico bus I2C (mux + INA226 por canal, 1 Hz) |
| 0x30A | DIAG_CAN_META | Baja | Meta-diagnóstico de entrega de 0x309 (call/tick/tx/fifo, 1 Hz) |
| 0x30B | DIAG_I2C_SCAN | Baja | Barrido I2C activo (on-demand, tras SERVICE 0xF6) |
| 0x30C | DIAG_FDCAN | Baja | Volcado contadores error FDCAN (on-demand, tras SERVICE 0xF6) |

### Retransmisión Automática

- **Habilitada** (`AutoRetransmission = ENABLE`)
- Hardware FDCAN retransmite automáticamente hasta que el mensaje es aceptado
- Si el bus entra en bus-off → recuperación con intervalo de 500 ms, máx. 10 reintentos

### Filtros CAN (STM32)

```c
// Filter 0: ESP32 heartbeat (0x011) — FDCAN_FILTER_DUAL
// Filter 1: ESP32 commands (0x100–0x102) — FDCAN_FILTER_RANGE
// Filter 2: Service + LED (0x110–0x120) — FDCAN_FILTER_RANGE
// Filter 3: Obstacle data (0x208–0x209) — FDCAN_FILTER_RANGE
// Global: Reject all non-matching standard and extended IDs
```

---

## ⏱️ Timing y Periodicidad

### Tabla de Periodicidad

| Mensaje | Frecuencia | Período | Prioridad | Dir |
|---------|------------|---------|-----------|-----|
| HEARTBEAT_STM32 (0x001) | 10 Hz | 100 ms | Alta | STM32→ESP32 |
| HEARTBEAT_ESP32 (0x011) | 10 Hz | 100 ms | Alta | ESP32→STM32 |
| CMD_THROTTLE (0x100) | 20 Hz | 50 ms | Media | ESP32→STM32 |
| CMD_STEERING (0x101) | 20 Hz | 50 ms | Media | ESP32→STM32 |
| CMD_MODE (0x102) | On-demand | - | Media | ESP32→STM32 |
| CMD_ACK (0x103) | On-demand | - | Media | STM32→ESP32 |
| SERVICE_CMD (0x110) | On-demand | - | Baja | ESP32→STM32 |
| CMD_LED (0x120) | On-demand | - | Baja | ESP32→STM32 |
| STATUS_SPEED (0x200) | 10 Hz | 100 ms | Baja | STM32→ESP32 |
| STATUS_CURRENT (0x201) | 10 Hz | 100 ms | Baja | STM32→ESP32 |
| STATUS_TEMP (0x202) | 1 Hz | 1000 ms | Baja | STM32→ESP32 |
| STATUS_SAFETY (0x203) | 10 Hz | 100 ms | Baja | STM32→ESP32 |
| STATUS_STEERING (0x204) | 10 Hz | 100 ms | Baja | STM32→ESP32 |
| STATUS_TRACTION (0x205) | 10 Hz | 100 ms | Baja | STM32→ESP32 |
| STATUS_TEMP_MAP (0x206) | 1 Hz | 1000 ms | Baja | STM32→ESP32 |
| STATUS_BATTERY (0x207) | 10 Hz | 100 ms | Baja | STM32→ESP32 |
| OBSTACLE_DISTANCE (0x208) | 15 Hz | 66 ms | Media | ESP32→STM32 |
| OBSTACLE_SAFETY (0x209) | 10 Hz | 100 ms | Media | ESP32→STM32 |
| STATUS_LIGHTS (0x20A) | 1 Hz | 1000 ms | Baja | STM32→ESP32 |
| DIAG_ERROR (0x300) | On-demand | - | Alta | Both |
| SERVICE_FAULTS (0x301) | 1 Hz | 1000 ms | Baja | STM32→ESP32 |
| SERVICE_ENABLED (0x302) | 1 Hz | 1000 ms | Baja | STM32→ESP32 |
| SERVICE_DISABLED (0x303) | 1 Hz | 1000 ms | Baja | STM32→ESP32 |

**Total (peor caso):** ~6.5 kbps de 500 kbps disponibles (**1.3% de utilización**)

### Latencia Máxima

- **Comando → Ejecución:** <10 ms (50 ms período + procesamiento)
- **Error → Notificación:** <5 ms (mensaje de alta prioridad)
- **Heartbeat → Detección fallo:** <250 ms (2.5× período)

---

## 📖 Referencias

- [CAN 2.0A Specification](https://www.iso.org/standard/63648.html)
- [FDCAN HAL Driver](https://www.st.com/resource/en/user_manual/um2319-description-of-stm32g4-hal-and-lowlayer-drivers-stmicroelectronics.pdf)
- [TJA1051T/3 Datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf)

---

**Última actualización:** 2026-02-01  
**Autor:** florinzgz  
**Proyecto:** STM32-Control-Coche-Marcos
