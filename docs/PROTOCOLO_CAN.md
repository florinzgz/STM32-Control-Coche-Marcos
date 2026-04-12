# PROTOCOLO CAN - Comunicación ESP32 ↔ STM32

> ⚠️ **DOCUMENTO DESACTUALIZADO**  
> Este documento fue el diseño inicial y contiene valores que difieren del firmware real.  
> **Referencia autorizada:** [`docs/CAN_CONTRACT_FINAL.md`](CAN_CONTRACT_FINAL.md) (rev 1.3)  
> Las correcciones aplicadas abajo se marcan con `[CORREGIDO]`.


## Especificaciones Bus CAN

### Configuración Física

- **Tipo**: FDCAN1 (compatible CAN 2.0B y CAN-FD)
- **Velocidad**: 500 kbps (CAN Classic)
- **Terminación**: 120Ω en ambos extremos del bus
- **Transceiver**: TJA1051T/3 (uno por nodo — ver `docs/ESP32_STM32_CAN_CONNECTION.md`)
- **Cables**: Par trenzado (CAN_H, CAN_L)
- **Longitud máxima**: 40 metros @ 500 kbps

### Parámetros Temporización

```c
// Bit timing para 500 kbps con APB1 = 170 MHz  [CORREGIDO — valores reales del firmware main.c]
// NominalPrescaler     = 17
// NominalTimeSeg1 (BS1) = 14
// NominalTimeSeg2 (BS2) = 5
// NominalSyncJumpWidth  = 1

// Cálculo velocidad:
// Bitrate = APB1_CLK / (Prescaler × (1 + TimeSeg1 + TimeSeg2))
// Bitrate = 170 MHz / (17 * (1 + 14 + 5)) = 170 MHz / (17 * 20) = 500 kbps
// Punto de muestreo = (1 + 14) / (1 + 14 + 5) = 15/20 = 75 %
```

### Topología Red

```
┌──────────────┐         ┌──────────────┐
│   ESP32-S3   │         │  STM32G474RE │
│   (Master)   │         │   (Slave)    │
│              │         │              │
│  CAN_TX ─────┼─────┬───┼───── CAN_RX  │
│  CAN_RX ─────┼───┐ │   │   ┌─ CAN_TX  │
└──────────────┘   │ │   └───┼──────────┘
                   │ │       │
                ┌──┴─┴───────┴──┐
                │   TJA1051T/3     │
                │  Transceiver  │
                └───┬───────┬───┘
                    │       │
                 CAN_H   CAN_L
                    │       │
                  120Ω   120Ω
              (terminación)
```

## Formato Mensajes CAN

### Estructura Frame Estándar

```
┌──────────┬────────┬─────┬────────┬─────┬─────┬─────┐
│   SOF    │  ID    │ RTR │  DLC   │ DATA│ CRC │ EOF │
│  (1bit)  │(11bit) │(1b) │ (4bit) │(0-8)│(15b)│(7b) │
└──────────┴────────┴─────┴────────┴─────┴─────┴─────┘

- SOF: Start of Frame (dominant bit)
- ID: Identificador CAN (0x000 - 0x7FF)
- RTR: Remote Transmission Request
- DLC: Data Length Code (0-8 bytes)
- DATA: Datos útiles (0-8 bytes)
- CRC: Cyclic Redundancy Check
- EOF: End of Frame
```

### Prioridades (IDs)

IDs menores tienen **mayor prioridad** en arbitraje:

| Rango ID | Tipo | Prioridad | Descripción |
|----------|------|-----------|-------------|
| 0x001-0x0FF | Sistema | CRÍTICA | Heartbeat, emergencias |
| 0x100-0x1FF | Comandos | ALTA | Control ESP32→STM32 |
| 0x200-0x2FF | Telemetría | MEDIA | Estado STM32→ESP32 |
| 0x300-0x3FF | Debug | BAJA | Diagnóstico |

## Tabla de Mensajes

### Sistema y Control (0x001 - 0x1FF)

#### 0x001: Heartbeat STM32  `[CORREGIDO]`
**Dirección**: STM32 → ESP32  
**Frecuencia**: 10 Hz (cada 100ms)  
**DLC**: 5 bytes *(era 2 — corregido)*

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | alive_counter | uint8 | 0-255 | Contador cíclico (validado por ESP32) |
| 1 | system_state | uint8 | 0-6 | Estado del sistema (Boot/Standby/Active/Degraded/Safe/Error/LimpHome) |
| 2 | fault_flags | uint8 | bitmask | Flags de fallo activos |
| 3 | error_code | uint8 | 0-255 | Código de error específico (Safety_Error_t) |
| 4 | status_flags | uint8 | bitmask | Bit0=StartupInhibit, Bit1=4x4, Bit2=TankTurn, Bits3-5=DS18B20 count |

*Fuente: `can_handler.c` → `CAN_SendHeartbeat()` — ver `docs/CAN_CONTRACT_FINAL.md` §4.1*

#### 0x011: Heartbeat ESP32  `[CORREGIDO — ID era 0x101, DLC era 2]`
**Dirección**: ESP32 → STM32  
**Frecuencia**: 10 Hz (cada 100ms)  
**DLC**: 1 byte *(era 2 — corregido)*

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | rolling_counter | uint8 | 0-255 | Contador cíclico (el STM32 detecta si se congela) |

**Timeout**: Si STM32 no recibe en **250 ms** → estado LIMP_HOME *(era 500 ms — corregido)*

*Fuente: `main.cpp` → heartbeat TX loop — ver `docs/CAN_CONTRACT_FINAL.md` §4.2*

---

### Comandos ESP32 → STM32 (0x100 - 0x12F)  `[CORREGIDO — rango era 0x200-0x2FF]`

#### 0x100: CMD_Throttle  `[CORREGIDO — ID era 0x200, DLC era 2]`
**Dirección**: ESP32 → STM32  
**Frecuencia**: 50 Hz (cada 20ms) (según firmware — hasta 50 ms)  
**DLC**: 1 byte *(DLC era 2; el firmware solo usa 1 byte: el porcentaje de acelerador)*

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | Throttle | uint8 | 0-100 | Acelerador porcentaje |
| 1 | Flags | uint8 | bitfield | Modificadores |

**Flags Bitfield (Byte 1):**
```
Bit 0: Reverse (1=marcha atrás)
Bit 1: Mode 4x4 (1=4x4, 0=4x2)
Bit 2: Tank turn (rotación sobre eje)
Bits 3-7: Reservados
```

**Ejemplo:**
```
ID: 0x100
Data: [75]  // Byte 0: throttle 75%
→ Throttle = 75%, Modo 4x4, sin reverse, sin tank turn
```

#### 0x101: CMD_Steering  `[CORREGIDO — ID era 0x201]`
**Dirección**: ESP32 → STM32  
**Frecuencia**: 50 Hz (cada 20ms)  
**DLC**: 2 bytes

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | Angle_LSB | uint8 | 0-255 | Byte bajo ángulo |
| 1 | Angle_MSB | uint8 | 0-255 | Byte alto ángulo |

**Conversión ángulo:**
```c
// Envío (ESP32)
int16_t angle_raw = (int16_t)(steering_deg * 100.0f); // -5400 a +5400
uint8_t lsb = (uint8_t)(angle_raw & 0xFF);
uint8_t msb = (uint8_t)((angle_raw >> 8) & 0xFF);

// Recepción (STM32)
int16_t angle_raw = (int16_t)((msb << 8) | lsb);
float steering_deg = (float)angle_raw / 100.0f;
```

**Ejemplo:**
```
ID: 0x101
Data: [0xF4, 0xF4]  // 0xF4F4 = -2828 → -28.28°
→ Steering = -28.28° (izquierda)
```

#### 0x102: CMD_Mode  `[CORREGIDO — ID era 0x202]`
**Dirección**: ESP32 → STM32  
**Frecuencia**: Bajo demanda  
**DLC**: 2 bytes

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | Command | uint8 | 0-255 | Comando específico |
| 1 | Parameter | uint8 | 0-255 | Parámetro opcional |

**Comandos (Byte 0):**
- `0x10`: Set 4x4 mode (param: 1=enable, 0=disable)
- `0x11`: Set tank turn (param: 1=enable, 0=disable)
- `0x20`: Calibrate steering (param: 0)
- `0x30`: Reset errors (param: 0)
- `0xFF`: Emergency stop (param: 0)

**Ejemplo:**
```
ID: 0x102
Data: [0x10, 0x01]
→ Activar modo 4x4
```

---

### Telemetría STM32 → ESP32 (0x200 - 0x20F)  `[CORREGIDO — rango era 0x300-0x3FF]`

#### 0x200: STATUS_SPEED  `[CORREGIDO — ID era 0x300]`
**Dirección**: STM32 → ESP32  
**Frecuencia**: 20 Hz (cada 50ms)  
**DLC**: 8 bytes

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | FL_Speed_LSB | uint8 | 0-255 | Velocidad FL byte bajo |
| 1 | FL_Speed_MSB | uint8 | 0-255 | Velocidad FL byte alto |
| 2 | FR_Speed_LSB | uint8 | 0-255 | Velocidad FR byte bajo |
| 3 | FR_Speed_MSB | uint8 | 0-255 | Velocidad FR byte alto |
| 4 | RL_Speed_LSB | uint8 | 0-255 | Velocidad RL byte bajo |
| 5 | RL_Speed_MSB | uint8 | 0-255 | Velocidad RL byte alto |
| 6 | RR_Speed_LSB | uint8 | 0-255 | Velocidad RR byte bajo |
| 7 | RR_Speed_MSB | uint8 | 0-255 | Velocidad RR byte alto |

**Conversión velocidad:**
```c
// Envío (STM32)
uint16_t speed_raw = (uint16_t)(speed_kmh * 100.0f);  // 0.01 km/h resolución
uint8_t lsb = speed_raw & 0xFF;
uint8_t msb = (speed_raw >> 8) & 0xFF;

// Recepción (ESP32)
uint16_t speed_raw = (msb << 8) | lsb;
float speed_kmh = (float)speed_raw / 100.0f;
```

**Ejemplo:**
```
ID: 0x200
Data: [0x58, 0x1B, 0x60, 0x1B, 0x55, 0x1B, 0x5D, 0x1B]
→ FL=70.00 km/h, FR=70.08 km/h, RL=69.97 km/h, RR=70.05 km/h
```

#### 0x201: STATUS_CURRENT  `[CORREGIDO — ID era 0x301]`
**Dirección**: STM32 → ESP32  
**Frecuencia**: 20 Hz (cada 50ms)  
**DLC**: 8 bytes

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | FL_Current_LSB | uint8 | 0-255 | Corriente FL byte bajo |
| 1 | FL_Current_MSB | uint8 | 0-255 | Corriente FL byte alto |
| 2 | FR_Current_LSB | uint8 | 0-255 | Corriente FR byte bajo |
| 3 | FR_Current_MSB | uint8 | 0-255 | Corriente FR byte alto |
| 4 | RL_Current_LSB | uint8 | 0-255 | Corriente RL byte bajo |
| 5 | RL_Current_MSB | uint8 | 0-255 | Corriente RL byte alto |
| 6 | STEER_Curr_LSB | uint8 | 0-255 | Corriente Dir byte bajo |
| 7 | STEER_Curr_MSB | uint8 | 0-255 | Corriente Dir byte alto |

**Conversión corriente:**
```c
// Envío (STM32) - con signo
int16_t current_raw = (int16_t)(current_A * 100.0f);  // 0.01A resolución
uint8_t lsb = current_raw & 0xFF;
uint8_t msb = (current_raw >> 8) & 0xFF;

// Recepción (ESP32)
int16_t current_raw = (int16_t)((msb << 8) | lsb);
float current_A = (float)current_raw / 100.0f;
```

#### 0x202: STATUS_TEMP  `[CORREGIDO — ID era 0x302]`
**Dirección**: STM32 → ESP32  
**Frecuencia**: 5 Hz (cada 200ms)  
**DLC**: 5 bytes

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | FL_Temp | int8 | -128 a +127 | Temperatura FL (°C) |
| 1 | FR_Temp | int8 | -128 a +127 | Temperatura FR (°C) |
| 2 | RL_Temp | int8 | -128 a +127 | Temperatura RL (°C) |
| 3 | RR_Temp | int8 | -128 a +127 | Temperatura RR (°C) |
| 4 | STEER_Temp | int8 | -128 a +127 | Temperatura Dir (°C) |

**Conversión temperatura:**
```c
// Directo en grados Celsius (entero con signo)
int8_t temp_byte = (int8_t)temperature_C;
```

**Ejemplo:**
```
ID: 0x202
Data: [45, 47, 43, 46, 38]
→ FL=45°C, FR=47°C, RL=43°C, RR=46°C, Steer=38°C
```

#### 0x203: STATUS_SAFETY  `[CORREGIDO — ID era 0x303]`
**Dirección**: STM32 → ESP32  
**Frecuencia**: 10 Hz (cada 100ms)  
**DLC**: 4 bytes

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | Flags | uint8 | bitfield | Estados seguridad |
| 1 | Error_Code | uint8 | 0-255 | Código error activo |
| 2 | Reserved | uint8 | - | Reservado |
| 3 | Reserved | uint8 | - | Reservado |

**Flags Bitfield (Byte 0):**
```
Bit 7: Emergency stop activo
Bit 6: Overtemperature detectada
Bit 5: Overcurrent detectada
Bit 4: CAN timeout
Bit 3: Encoder fault
Bit 2: ABS activo
Bit 1: TCS activo
Bit 0: Steering not calibrated
```

**Error Codes (Byte 1):**
- `0x00`: No error
- `0x10`: Temperature FL > 130°C
- `0x11`: Temperature FR > 130°C
- `0x12`: Temperature RL > 130°C
- `0x13`: Temperature RR > 130°C
- `0x14`: Temperature Steer > 100°C
- `0x20`: Current FL > 25A
- `0x21`: Current FR > 25A
- `0x22`: Current RL > 25A
- `0x23`: Current RR > 25A
- `0x24`: Current Steer > 15A
- `0x30`: CAN timeout (>500ms)
- `0x40`: Encoder Z signal lost
- `0x50`: BTS7960 fault detected
- `0xFF`: Multiple errors

#### 0x204: STATUS_STEERING  `[CORREGIDO — ID era 0x304]`
**Dirección**: STM32 → ESP32  
**Frecuencia**: 20 Hz (cada 50ms)  
**DLC**: 4 bytes

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | Angle_LSB | uint8 | 0-255 | Ángulo actual LSB |
| 1 | Angle_MSB | uint8 | 0-255 | Ángulo actual MSB |
| 2 | Encoder_Cnt_L | uint8 | 0-255 | Contador encoder bajo |
| 3 | Encoder_Cnt_H | uint8 | 0-255 | Contador encoder alto |

**Formato idéntico a CMD_Steering** para ángulo.

---

## Manejo de Errores y Timeouts

### Detección Timeout

```c
// STM32: Detectar pérdida de heartbeat ESP32
#define CAN_TIMEOUT_MS  500

uint32_t last_heartbeat_time = 0;

void CAN_CheckTimeout(void) {
    uint32_t current_time = HAL_GetTick();
    
    if ((current_time - last_heartbeat_time) > CAN_TIMEOUT_MS) {
        // Timeout detectado
        Traction_EmergencyStop();
        safety_flags |= FLAG_CAN_TIMEOUT;
        
        // Enviar alerta
        CAN_SendMessage(0x303, error_data, 4);
    }
}

void CAN_HeartbeatReceived(void) {
    last_heartbeat_time = HAL_GetTick();
    safety_flags &= ~FLAG_CAN_TIMEOUT;
}
```

### Recuperación Automática

```c
// Secuencia recuperación tras timeout
void CAN_Recovery(void) {
    // 1. Verificar heartbeat recibido
    if (!(safety_flags & FLAG_CAN_TIMEOUT)) {
        // 2. Resetear error code
        error_code = 0x00;
        
        // 3. Esperar confirmación ESP32
        delay_ms(100);
        
        // 4. Reactivar sistema gradualmente
        Traction_Init();
        Steering_Init();
    }
}
```

### Filtros CAN

```c
// Configuración filtros en STM32  [CORREGIDO — IDs y API actualizados al FDCAN real]
// Fuente real: Core/Src/can_handler.c → CAN_ConfigureFilters()
static void CAN_ConfigureFilters(void)
{
    FDCAN_FilterTypeDef filter = {0};

    // Filtro 0: Heartbeat ESP32 (0x011) — Dual exact match
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x011;
    filter.FilterID2    = 0x011;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    // Filtro 1: Comandos ESP32 (0x100–0x102) — Range
    filter.FilterIndex  = 1;
    filter.FilterType   = FDCAN_FILTER_RANGE;
    filter.FilterID1    = 0x100;  // CMD_THROTTLE
    filter.FilterID2    = 0x102;  // CMD_MODE
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    // Filtro 2: Service + LED (0x110–0x120) — Range
    filter.FilterIndex  = 2;
    filter.FilterID1    = 0x110;  // SERVICE_CMD
    filter.FilterID2    = 0x120;  // CMD_LED
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    // Filtro 3: Obstacle data (0x208–0x209) — Range
    filter.FilterIndex  = 3;
    filter.FilterID1    = 0x208;  // OBSTACLE_DISTANCE
    filter.FilterID2    = 0x209;  // OBSTACLE_SAFETY
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
}
```

## Ejemplo Implementación

### ESP32: Enviar Comando Throttle

```cpp
// [CORREGIDO — en firmware real CMD_THROTTLE (0x100) usa DLC=1, solo el byte de throttle]
// La dirección y modo 4x4 se envían en CMD_MODE (0x102) al cambiar de estado.
void ESP32_SendThrottle(float throttle_pct) {
    CanFrame frame = {};
    frame.identifier       = 0x100;  // CMD_THROTTLE (corregido: era 0x200)
    frame.extd             = 0;
    frame.data_length_code = 1;
    frame.data[0]          = (uint8_t)constrain(throttle_pct, 0, 100);
    ESP32Can.writeFrame(frame);
}
```

### STM32: Recibir y Procesar

```c
// [CORREGIDO — firmware real usa FDCAN, IDs corregidos]
// Fuente real: Core/Src/can_handler.c → CAN_ProcessMessages()
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        switch (rx_header.Identifier) {
            case 0x011:  // HEARTBEAT_ESP32 (corregido: era 0x101)
                // Validar contador cíclico, actualizar timestamp
                break;
                
            case 0x100:  // CMD_THROTTLE (corregido: era 0x200), DLC=1
                Safety_ValidateThrottle(rx_data[0]);
                break;
                
            case 0x101:  // CMD_STEERING (corregido: era 0x201), DLC=2
                int16_t angle_raw = (int16_t)(rx_data[0] | (rx_data[1] << 8));
                Safety_ValidateSteering((float)angle_raw / 100.0f);
                break;
        }
    }
}
```

## Diagrama Secuencia

```
ESP32                              STM32
  │  [IDs corregidos respecto al original]   │
  ├──── Heartbeat 0x011 ────────────>│ (cada 100ms)  [era 0x101]
  │                                  │
  │<──── Heartbeat 0x001 ────────────┤ (cada 100ms)
  │                                  │
  ├──── CMD_Throttle 0x100 ─────────>│ (cada 50ms)   [era 0x200]
  ├──── CMD_Steering 0x101 ─────────>│ (cada 50ms)   [era 0x201]
  │                                  │
  │                                  │ [Procesamiento]
  │                                  │ [Ackermann]
  │                                  │ [Seguridad]
  │                                  │ [PWM Output]
  │                                  │
  │<──── STATUS_SPEED   0x200 ───────┤ (cada 100ms)  [era 0x300]
  │<──── STATUS_CURRENT 0x201 ───────┤ (cada 100ms)  [era 0x301]
  │<──── STATUS_TEMP    0x202 ───────┤ (cada 1000ms) [era 0x302]
  │<──── STATUS_SAFETY  0x203 ───────┤ (cada 100ms)  [era 0x303]
  │<──── STATUS_STEERING 0x204 ──────┤ (cada 100ms)  [era 0x304]
  │                                  │
```

## Uso de Ancho de Banda

| Mensaje | Tamaño | Frecuencia | Bytes/s | % Bus @ 500kbps |
|---------|--------|------------|---------|-----------------|
| 0x001 Heartbeat STM32 (DLC 5) | 13 bytes | 10 Hz | 130 | 0.21% |
| 0x011 Heartbeat ESP32 (DLC 1) [era 0x101] | 9 bytes | 10 Hz | 90 | 0.14% |
| 0x100 CMD_Throttle (DLC 1) [era 0x200] | 9 bytes | 20 Hz | 180 | 0.29% |
| 0x101 CMD_Steering (DLC 2) [era 0x201] | 10 bytes | 20 Hz | 200 | 0.32% |
| 0x200 STATUS_SPEED (DLC 8) [era 0x300] | 16 bytes | 10 Hz | 160 | 0.26% |
| 0x201 STATUS_CURRENT (DLC 8) [era 0x301] | 16 bytes | 10 Hz | 160 | 0.26% |
| 0x202 STATUS_TEMP (DLC 5) [era 0x302] | 13 bytes | 1 Hz | 13 | 0.02% |
| 0x203 STATUS_SAFETY (DLC 3) [era 0x303] | 11 bytes | 10 Hz | 110 | 0.18% |
| 0x204 STATUS_STEERING (DLC 3) [era 0x304] | 11 bytes | 10 Hz | 110 | 0.18% |
| **TOTAL** | | | **~1,153** | **~1.85%** |

**Conclusión**: Uso extremadamente bajo del bus (~2%), dejando amplio margen para expansiones.

---

**Documento creado**: 2026-02-01  
**Versión**: 1.0  
**Autor**: Sistema de Control Coche Marcos
