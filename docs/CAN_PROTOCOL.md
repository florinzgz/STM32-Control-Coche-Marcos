# 📡 Protocolo CAN - ESP32↔STM32

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
hfdcan1.NominalPrescaler = 20;                   // 170 MHz / 20 = 8.5 MHz
hfdcan1.NominalTimeSeg1 = 13;                    // 13 TQ
hfdcan1.NominalTimeSeg2 = 3;                     // 3 TQ
hfdcan1.NominalSyncJumpWidth = 1;                // 1 TQ
// Total: (1 + 13 + 3) = 17 TQ → 8.5 MHz / 17 = 500 kbps
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
3. **Timeout de seguridad:** Si un nodo no responde en 250 ms, el otro entra en modo seguro.
4. **Validación de comandos:** El STM32 valida todos los comandos antes de ejecutarlos.

---

## 🎮 Mensajes de Control (ESP32→STM32)

### 0x011 - HEARTBEAT_ESP32

**Propósito:** Indicar que el ESP32-S3 está operativo.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0 | `alive_counter` | uint8_t | 0-255 | - | Contador cíclico (incrementa cada mensaje) |
| 1 | `system_state` | uint8_t | 0-3 | - | Estado HMI (0=Boot, 1=Normal, 2=Warning, 3=Error) |
| 2 | `reserved` | uint8_t | 0 | - | Reservado para futuro uso |
| 3 | `checksum` | uint8_t | CRC8 | - | CRC8 de bytes 0-2 |

**Frecuencia:** 100 ms (10 Hz)

**Ejemplo:**
```
ID: 0x011  DLC: 4  Data: [0x42, 0x01, 0x00, 0xA3]
```

---

### 0x100 - CMD_THROTTLE

**Propósito:** Comando de aceleración desde pedal/HMI.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0 | `throttle_pct` | uint8_t | 0-100 | % | Porcentaje de aceleración solicitado |
| 1 | `checksum` | uint8_t | CRC8 | - | CRC8 de byte 0 |

**Frecuencia:** 50 ms (20 Hz) cuando hay cambio, 0 si inactivo

**Validación STM32:**
- Si `throttle_pct > 100` → Rechazar (error)
- Si shifter != FORWARD → Limitar a 0% (seguridad)
- Si ABS/TCS activo → Puede reducir throttle automáticamente

**Ejemplo:**
```
ID: 0x100  DLC: 2  Data: [0x4B, 0xE7]  // 75% throttle
```

---

### 0x101 - CMD_STEERING

**Propósito:** Comando de dirección desde HMI o control automático.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0 | `steering_angle` | int8_t | -100 a +100 | % | Ángulo solicitado (-100=izq max, +100=der max) |
| 1 | `checksum` | uint8_t | CRC8 | - | CRC8 de byte 0 |

**Frecuencia:** 50 ms (20 Hz)

**Mapeo:**
- -100% → Giro completo izquierda (~-45°)
- 0% → Centro (0°)
- +100% → Giro completo derecha (~+45°)

**Validación STM32:**
- Verificar límites de encoder (-720 a +720 conteos)
- Limitar velocidad de cambio (rate limiter @ 200°/s max)

**Ejemplo:**
```
ID: 0x101  DLC: 2  Data: [0xE7, 0x3C]  // -25% (giro ligero izquierda)
```

---

### 0x102 - CMD_MODE

**Propósito:** Comando de cambio de modo de conducción.

| Byte | Campo | Tipo | Rango | Notas |
|------|-------|------|-------|-------|
| 0 | `drive_mode` | uint8_t | 0-2 | 0=REVERSE, 1=NEUTRAL, 2=FORWARD |
| 1 | `checksum` | uint8_t | CRC8 | CRC8 de byte 0 |

**Frecuencia:** On-demand (solo cuando cambia shifter)

**Validación STM32:**
- Comparar con shifter físico (PB12/PB13/PB14)
- Cambio solo permitido si velocidad < 1 km/h
- Ignora comando si no coincide con shifter hardware

**Ejemplo:**
```
ID: 0x102  DLC: 1  Data: [0x02]  // Solicitar FORWARD
```

---

## 📊 Mensajes de Estado (STM32→ESP32)

### 0x001 - HEARTBEAT_STM32

**Propósito:** Indicar que el STM32 está operativo y en qué estado.

| Byte | Campo | Tipo | Rango | Notas |
|------|-------|------|-------|-------|
| 0 | `alive_counter` | uint8_t | 0-255 | Contador cíclico |
| 1 | `system_state` | uint8_t | 0-4 | 0=Boot, 1=Standby, 2=Active, 3=Safe, 4=Error |
| 2 | `fault_flags` | uint8_t | Bitmask | Ver tabla de fallos |
| 3 | `checksum` | uint8_t | CRC8 | CRC8 de bytes 0-2 |

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
| 7 | `RESERVED` | Reservado |

**Ejemplo:**
```
ID: 0x001  DLC: 4  Data: [0x7F, 0x02, 0x20, 0xB4]
// Estado: Active, ABS activo (bit 5 = 1)
```

---

### 0x200 - STATUS_SPEED

**Propósito:** Velocidades de las 4 ruedas.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0-1 | `speed_FL` | uint16_t | 0-65535 | mm/s | Velocidad rueda FL (LSB first) |
| 2-3 | `speed_FR` | uint16_t | 0-65535 | mm/s | Velocidad rueda FR |
| 4-5 | `speed_RL` | uint16_t | 0-65535 | mm/s | Velocidad rueda RL |
| 6-7 | `speed_RR` | uint16_t | 0-65535 | mm/s | Velocidad rueda RR |

**Frecuencia:** 100 ms (10 Hz)

**Conversión:**
- Velocidad lineal (mm/s) = `(pulsos/segundo × perímetro_rueda_mm) / pulsos_por_rev`
- Ejemplo: 300 mm rueda, 2 PPR → 1 pulso/s = 471 mm/s

**Ejemplo:**
```
ID: 0x200  DLC: 8  Data: [0x2C, 0x01, 0x30, 0x01, 0x28, 0x01, 0x2A, 0x01]
// FL=300 mm/s, FR=304, RL=296, RR=298 (uniforme)
```

---

### 0x201 - STATUS_CURRENT

**Propósito:** Corrientes de los 5 motores + batería principal.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0 | `current_FL` | uint8_t | 0-255 | 0.1A | Motor FL (0-25.5A) |
| 1 | `current_FR` | uint8_t | 0-255 | 0.1A | Motor FR |
| 2 | `current_RL` | uint8_t | 0-255 | 0.1A | Motor RL |
| 3 | `current_RR` | uint8_t | 0-255 | 0.1A | Motor RR |
| 4 | `current_STEER` | uint8_t | 0-255 | 0.1A | Motor dirección |
| 5 | `current_BATT` | uint8_t | 0-255 | 0.1A | Batería principal (suma) |
| 6-7 | `reserved` | - | - | - | Reservado |

**Frecuencia:** 100 ms (10 Hz)

**Ejemplo:**
```
ID: 0x201  DLC: 8  Data: [0x1E, 0x1F, 0x1D, 0x1E, 0x05, 0x7D, 0x00, 0x00]
// FL=3.0A, FR=3.1A, RL=2.9A, RR=3.0A, STEER=0.5A, BATT=12.5A
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
| 5-7 | `reserved` | - | - | - | Reservado |

**Frecuencia:** 1000 ms (1 Hz) - baja prioridad

**Umbrales de Protección:**
- < 60°C → Normal (verde)
- 60-80°C → Warning (amarillo), reducir potencia al 70%
- > 80°C → Critical (rojo), limitar a 30% o detener

**Ejemplo:**
```
ID: 0x202  DLC: 8  Data: [0x37, 0x39, 0x38, 0x36, 0x19, 0x00, 0x00, 0x00]
// FL=55°C, FR=57°C, RL=56°C, RR=54°C, AMB=25°C
```

---

### 0x203 - STATUS_SAFETY

**Propósito:** Estado de sistemas de seguridad (ABS/TCS).

| Byte | Campo | Tipo | Rango | Notas |
|------|-------|------|-------|-------|
| 0 | `abs_flags` | uint8_t | Bitmask | ABS activo por rueda (bit 0=FL, 1=FR, 2=RL, 3=RR) |
| 1 | `tcs_flags` | uint8_t | Bitmask | TCS activo por rueda (bit 0=FL, 1=FR, 2=RL, 3=RR) |
| 2 | `slip_max` | uint8_t | 0-100 | % de deslizamiento máximo detectado |
| 3 | `checksum` | uint8_t | CRC8 | CRC8 de bytes 0-2 |

**Frecuencia:** 100 ms (10 Hz)

**Ejemplo:**
```
ID: 0x203  DLC: 4  Data: [0x08, 0x00, 0x12, 0xC5]
// ABS activo en RR (bit 3=1), deslizamiento 18%
```

---

### 0x204 - STATUS_STEERING

**Propósito:** Posición angular del motor de dirección.

| Byte | Campo | Tipo | Rango | Unidad | Notas |
|------|-------|------|-------|--------|-------|
| 0-1 | `encoder_position` | int16_t | -720 a +720 | conteos | Posición encoder (LSB first) |
| 2 | `steering_angle` | int8_t | -100 a +100 | % | Ángulo normalizado |
| 3 | `checksum` | uint8_t | CRC8 | CRC8 de bytes 0-2 |

**Frecuencia:** 100 ms (10 Hz)

**Conversión:**
- 1 conteo = 0.075° (encoder E6B2-CWZ6C 1200 PPR × 4 = 4800 conteos/rev)
- Centro = 0 conteos (calibrado en inicio)
- Límites físicos: ±54° = ±720 conteos (máximo ángulo Ackermann)

**Ejemplo:**
```
ID: 0x204  DLC: 4  Data: [0xB4, 0xFF, 0xE7, 0x4A]
// Posición: -76 conteos = -19° → -42% (giro moderado izquierda)
```

---

## 🔔 Mensajes de Heartbeat

### Lógica de Heartbeat Mutuo

```c
// STM32: Verificación de heartbeat ESP32
uint32_t last_esp32_heartbeat = HAL_GetTick();

void FDCAN1_IT0_IRQHandler(void) {
    if (rx_msg.Identifier == 0x011) {
        last_esp32_heartbeat = HAL_GetTick();
        // ESP32 está vivo
    }
}

void main_loop(void) {
    if ((HAL_GetTick() - last_esp32_heartbeat) > 250) {
        // ESP32 no responde → Modo seguro
        enter_safe_mode();
    }
}
```

### Respuesta ante Pérdida de Heartbeat

| Nodo | Timeout | Acción |
|------|---------|--------|
| **STM32** | >250 ms sin ESP32 | 1. Detener motores suavemente<br>2. Abrir relés tracción<br>3. Centrar dirección<br>4. Enviar DIAG_ERROR 0x300 |
| **ESP32** | >250 ms sin STM32 | 1. Mostrar alerta crítica<br>2. Audio de advertencia<br>3. Log de evento<br>4. Esperar reconexión |

---

## 🚨 Mensajes de Diagnóstico

### 0x300 - DIAG_ERROR

**Propósito:** Reporte detallado de errores críticos.

| Byte | Campo | Tipo | Notas |
|------|-------|------|-------|
| 0 | `error_code` | uint8_t | Código de error (ver tabla) |
| 1 | `subsystem` | uint8_t | Subsistema afectado (0=Global, 1=Motor, 2=Sensor, 3=CAN) |
| 2-3 | `error_data` | uint16_t | Datos específicos del error |
| 4-7 | `timestamp` | uint32_t | Timestamp del error (ms desde boot) |

**Frecuencia:** On-demand (solo cuando ocurre error)

**Códigos de Error:**

| Código | Nombre | Descripción |
|--------|--------|-------------|
| 0x01 | `ERR_TIMEOUT_CAN` | Pérdida de heartbeat CAN |
| 0x02 | `ERR_TEMP_CRITICAL` | Temperatura >80°C |
| 0x03 | `ERR_CURRENT_OVERLOAD` | Corriente excesiva |
| 0x04 | `ERR_ENCODER_FAULT` | Encoder desconectado/inválido |
| 0x05 | `ERR_WHEEL_SENSOR` | Sensor de rueda fallo |
| 0x10 | `ERR_WATCHDOG_RESET` | Reset por watchdog |
| 0x20 | `ERR_I2C_TIMEOUT` | Timeout I2C (INA226) |
| 0x21 | `ERR_ONEWIRE_TIMEOUT` | Timeout OneWire (DS18B20) |
| 0xFF | `ERR_UNKNOWN` | Error desconocido |

**Ejemplo:**
```
ID: 0x300  DLC: 8  Data: [0x02, 0x01, 0x55, 0x00, 0x10, 0x27, 0x00, 0x00]
// Error: TEMP_CRITICAL, Motor FL, Temp=85°C, Time=10000ms
```

---

## ⚙️ Gestión de Errores

### Prioridades de Mensajes

| ID | Mensaje | Prioridad CAN | Descripción |
|----|---------|---------------|-------------|
| 0x001 | HEARTBEAT_STM32 | **Alta** | Crítico para seguridad |
| 0x011 | HEARTBEAT_ESP32 | **Alta** | Crítico para seguridad |
| 0x100 | CMD_THROTTLE | Media | Control tiempo real |
| 0x101 | CMD_STEERING | Media | Control tiempo real |
| 0x200-0x204 | STATUS_* | Baja | Telemetría |
| 0x300 | DIAG_ERROR | **Alta** | Errores críticos |

### Retransmisión Automática

- **Habilitada** para todos los mensajes
- Máximo 3 reintentos
- Si falla después de 3 intentos → `ERR_CAN_BUS_OFF`

### Filtros CAN (STM32)

```c
// Filtro 1: Aceptar solo mensajes de control (0x011, 0x100-0x102)
FDCAN_FilterTypeDef filter1;
filter1.IdType = FDCAN_STANDARD_ID;
filter1.FilterType = FDCAN_FILTER_RANGE;
filter1.FilterID1 = 0x011;
filter1.FilterID2 = 0x102;
filter1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

// Filtro 2: Rechazar todo lo demás
filter1.FilterConfig = FDCAN_FILTER_REJECT;
```

---

## ⏱️ Timing y Periodicidad

### Tabla de Periodicidad

| Mensaje | Frecuencia | Período | Prioridad | Ancho de Banda |
|---------|------------|---------|-----------|----------------|
| HEARTBEAT_STM32 | 10 Hz | 100 ms | Alta | 640 bps |
| HEARTBEAT_ESP32 | 10 Hz | 100 ms | Alta | 640 bps |
| CMD_THROTTLE | 20 Hz | 50 ms | Media | 640 bps |
| CMD_STEERING | 20 Hz | 50 ms | Media | 640 bps |
| CMD_MODE | On-demand | - | Media | <100 bps |
| STATUS_SPEED | 10 Hz | 100 ms | Baja | 1280 bps |
| STATUS_CURRENT | 10 Hz | 100 ms | Baja | 1280 bps |
| STATUS_TEMP | 1 Hz | 1000 ms | Baja | 128 bps |
| STATUS_SAFETY | 10 Hz | 100 ms | Baja | 640 bps |
| STATUS_STEERING | 10 Hz | 100 ms | Baja | 640 bps |
| DIAG_ERROR | On-demand | - | Alta | <500 bps |

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
