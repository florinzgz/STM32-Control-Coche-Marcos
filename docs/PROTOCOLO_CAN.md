# PROTOCOLO CAN - Comunicación ESP32 ↔ STM32

## Especificaciones Bus CAN

### Configuración Física

- **Tipo**: FDCAN1 (compatible CAN 2.0B y CAN-FD)
- **Velocidad**: 500 kbps (CAN Classic)
- **Terminación**: 120Ω en ambos extremos del bus
- **Transceiver**: TJA1050, MCP2551 o compatible
- **Cables**: Par trenzado (CAN_H, CAN_L)
- **Longitud máxima**: 40 metros @ 500 kbps

### Parámetros Temporización

```c
// Bit timing para 500 kbps con APB1 = 170 MHz
#define CAN_PRESCALER    20
#define CAN_BS1          12
#define CAN_BS2          4
#define CAN_SJW          1

// Cálculo velocidad:
// Bitrate = APB1_CLK / (PRESCALER × (1 + BS1 + BS2))
// Bitrate = 170MHz / (20 × (1 + 12 + 4)) = 500 kbps
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
                │   TJA1050     │
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

#### 0x001: Heartbeat STM32
**Dirección**: STM32 → ESP32  
**Frecuencia**: 10 Hz (cada 100ms)  
**DLC**: 2 bytes

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | Magic | uint8 | 0x01 | Constante de validación |
| 1 | Status | uint8 | 0x00-0xFF | Estado sistema (bitfield) |

**Status Bitfield (Byte 1):**
```
Bit 7: Emergency stop activo (1=activo)
Bit 6: Modo 4x4 (1=4x4, 0=4x2)
Bit 5: Calibración dirección OK
Bit 4: CAN timeout detectado
Bit 3: Overtemperature
Bit 2: Overcurrent
Bit 1: Relé tracción ON
Bit 0: Relé dirección ON
```

**Ejemplo:**
```
ID: 0x001
Data: [0x01, 0x63]
→ Magic OK, Status = 0x63 = 0b01100011
  - Modo 4x4 activo
  - Calibración OK
  - Relés ON
```

#### 0x101: Heartbeat ESP32
**Dirección**: ESP32 → STM32  
**Frecuencia**: 10 Hz (cada 100ms)  
**DLC**: 2 bytes

| Byte | Campo | Tipo | Rango | Descripción |
|------|-------|------|-------|-------------|
| 0 | Magic | uint8 | 0xE5 | Constante de validación ESP32 |
| 1 | Mode | uint8 | 0-3 | Modo operación actual |

**Modes (Byte 1):**
- `0x00`: IDLE (reposo)
- `0x01`: MANUAL (control directo)
- `0x02`: AUTO (navegación autónoma)
- `0x03`: EMERGENCY (parada emergencia)

**Timeout**: Si STM32 no recibe en 500ms → Emergency stop

---

### Comandos ESP32 → STM32 (0x200 - 0x2FF)

#### 0x200: CMD_Throttle
**Dirección**: ESP32 → STM32  
**Frecuencia**: 50 Hz (cada 20ms)  
**DLC**: 2 bytes

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
ID: 0x200
Data: [75, 0x02]
→ Throttle = 75%, Modo 4x4, sin reverse, sin tank turn
```

#### 0x201: CMD_Steering
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
ID: 0x201
Data: [0xF4, 0xF4]  // 0xF4F4 = -2828 → -28.28°
→ Steering = -28.28° (izquierda)
```

#### 0x202: CMD_Mode
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
ID: 0x202
Data: [0x10, 0x01]
→ Activar modo 4x4
```

---

### Telemetría STM32 → ESP32 (0x300 - 0x3FF)

#### 0x300: TELEM_Speed
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
ID: 0x300
Data: [0x58, 0x1B, 0x60, 0x1B, 0x55, 0x1B, 0x5D, 0x1B]
→ FL=70.00 km/h, FR=70.08 km/h, RL=69.97 km/h, RR=70.05 km/h
```

#### 0x301: TELEM_Current
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

#### 0x302: TELEM_Temperature
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
ID: 0x302
Data: [45, 47, 43, 46, 38]
→ FL=45°C, FR=47°C, RL=43°C, RR=46°C, Steer=38°C
```

#### 0x303: TELEM_Safety
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

#### 0x304: TELEM_Steering
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
// Configuración filtros en STM32
void CAN_ConfigFilters(void) {
    CAN_FilterTypeDef filter;
    
    // Filtro 0: Aceptar heartbeat ESP32 (0x101)
    filter.FilterIdHigh = 0x101 << 5;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x7FF << 5;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    
    // Filtro 1: Aceptar comandos (0x200-0x2FF)
    filter.FilterIdHigh = 0x200 << 5;
    filter.FilterMaskIdHigh = 0x700 << 5;
    filter.FilterBank = 1;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
}
```

## Ejemplo Implementación

### ESP32: Enviar Comando Throttle

```cpp
void ESP32_SendThrottle(float throttle_pct, bool mode4x4, bool reverse) {
    uint8_t data[2];
    
    // Byte 0: Throttle 0-100
    data[0] = (uint8_t)throttle_pct;
    
    // Byte 1: Flags
    data[1] = 0x00;
    if (reverse) data[1] |= 0x01;
    if (mode4x4) data[1] |= 0x02;
    
    // Enviar CAN
    CAN_SendMessage(0x200, data, 2);
}
```

### STM32: Recibir y Procesar

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, 
                              &rx_header, rx_data) == HAL_OK) {
        
        switch (rx_header.StdId) {
            case 0x101:  // Heartbeat ESP32
                CAN_HeartbeatReceived();
                break;
                
            case 0x200:  // CMD_Throttle
                float throttle = (float)rx_data[0];
                bool mode4x4 = (rx_data[1] & 0x02) != 0;
                bool reverse = (rx_data[1] & 0x01) != 0;
                
                Traction_SetDemand(throttle);
                Traction_SetMode4x4(mode4x4);
                break;
                
            case 0x201:  // CMD_Steering
                int16_t angle_raw = (int16_t)((rx_data[1] << 8) | rx_data[0]);
                float angle_deg = (float)angle_raw / 100.0f;
                
                Steering_SetAngle(angle_deg);
                break;
        }
    }
}
```

## Diagrama Secuencia

```
ESP32                              STM32
  │                                  │
  ├──── Heartbeat 0x101 ────────────>│ (cada 100ms)
  │                                  │
  │<──── Heartbeat 0x001 ────────────┤ (cada 100ms)
  │                                  │
  ├──── CMD_Throttle 0x200 ─────────>│ (cada 20ms)
  ├──── CMD_Steering 0x201 ─────────>│ (cada 20ms)
  │                                  │
  │                                  │ [Procesamiento]
  │                                  │ [Ackermann]
  │                                  │ [Seguridad]
  │                                  │ [PWM Output]
  │                                  │
  │<──── TELEM_Speed 0x300 ──────────┤ (cada 50ms)
  │<──── TELEM_Current 0x301 ────────┤ (cada 50ms)
  │<──── TELEM_Temperature 0x302 ────┤ (cada 200ms)
  │<──── TELEM_Safety 0x303 ─────────┤ (cada 100ms)
  │<──── TELEM_Steering 0x304 ───────┤ (cada 50ms)
  │                                  │
```

## Uso de Ancho de Banda

| Mensaje | Tamaño | Frecuencia | Bytes/s | % Bus @ 500kbps |
|---------|--------|------------|---------|-----------------|
| 0x001 Heartbeat STM | 10 bytes | 10 Hz | 100 | 0.16% |
| 0x101 Heartbeat ESP | 10 bytes | 10 Hz | 100 | 0.16% |
| 0x200 Throttle | 10 bytes | 50 Hz | 500 | 0.80% |
| 0x201 Steering | 10 bytes | 50 Hz | 500 | 0.80% |
| 0x300 Speed | 16 bytes | 20 Hz | 320 | 0.51% |
| 0x301 Current | 16 bytes | 20 Hz | 320 | 0.51% |
| 0x302 Temperature | 13 bytes | 5 Hz | 65 | 0.10% |
| 0x303 Safety | 12 bytes | 10 Hz | 120 | 0.19% |
| 0x304 Steering | 12 bytes | 20 Hz | 240 | 0.38% |
| **TOTAL** | | | **2,265** | **3.62%** |

**Conclusión**: Uso extremadamente bajo del bus (<4%), dejando amplio margen para expansiones.

---

**Documento creado**: 2026-02-01  
**Versión**: 1.0  
**Autor**: Sistema de Control Coche Marcos
