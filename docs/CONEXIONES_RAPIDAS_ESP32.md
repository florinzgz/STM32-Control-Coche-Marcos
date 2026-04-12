# REFERENCIA RÁPIDA - CONEXIONES ESP32-S3
## Pines de Pantalla, CAN-Bus y Sensor TF-Mini Plus

---

## PANTALLA TFT (Display ST7796 480×320)

### Conexiones Principales

| Display Pin | → | ESP32-S3 GPIO | Función |
|-------------|---|---------------|---------|
| VCC | → | 3.3V | Alimentación |
| GND | → | GND | Tierra |
| CS | → | GPIO 10 | Chip Select |
| RESET | → | GPIO 38 | Reset |
| DC/RS | → | GPIO 39 | Data/Command |
| SDI (MOSI) | → | GPIO 13 | Datos SPI |
| SCK | → | GPIO 14 | Reloj SPI |
| LED | → | GPIO 42 | Backlight |
| SDO (MISO) | → | GPIO 12 | Datos SPI (touch data out) |

### Touch Panel

| Touch Pin | → | ESP32-S3 GPIO | Función |
|-----------|---|---------------|---------|
| T_CS | → | GPIO 21 | Touch Chip Select |
| T_DIN | → | GPIO 13 | Compartido con MOSI |
| T_CLK | → | GPIO 14 | Compartido con SCK |
| T_DO | → | GPIO 12 | Compartido con MISO |
| T_IRQ | → | No conectar | No usado |

---

## CAN BUS (Transceiver TJA1051)

### Conexiones ESP32 ↔ TJA1051

| ESP32-S3 | → | TJA1051 Pin | Nombre |
|----------|---|-------------|--------|
| GPIO 4 | → | Pin 1 | TXD (Transmisión) |
| GPIO 5 | → | Pin 4 | RXD (Recepción) |
| 5V (ext.) | → | Pin 3 | VCC (Alimentación 5 V) |
| **3.3V** | → | **Pin 5** | **VIO (Nivel lógico I/O) — ⚠️ OBLIGATORIO** |
| GND | → | Pin 2 | GND (Tierra) |
| GND | → | Pin 8 | S (Modo Normal) |

> ⚠️ **CRÍTICO:** El pin 5 (VIO) **DEBE** conectarse a 3.3 V. Si se deja flotante o a 5 V, el pin RXD producirá niveles de 5 V que **destruyen el GPIO5 del ESP32-S3** (máx. absoluto 3.6 V).

### Bus CAN Físico

| TJA1051 | → | Bus | Notas |
|---------|---|-----|-------|
| Pin 7 (CANH) | → | Cable naranja | Par trenzado |
| Pin 6 (CANL) | → | Cable blanco | Par trenzado |
| 120Ω | ↔ | CANH ↔ CANL | En cada extremo |

---

## SENSOR DE OBSTÁCULOS (TF-Mini Plus via UART1)

### Conexiones TF-Mini Plus → ESP32-S3

| TF-Mini Plus | → | ESP32-S3 / Fuente | Función |
|--------------|---|-------------------|---------|
| Red (VCC) | → | 5V (regulador) | Alimentación 5V |
| Black (GND) | → | GND | Tierra común |
| White/Green (TX) | → | GPIO 18 | UART1 RX (datos de distancia, 3.3V directo) |

- **Baudrate:** 115200 bps, 8N1
- **Trama:** 9 bytes por frame
- **Nivel lógico UART:** 3.3V TTL — conexión directa a ESP32, NO requiere divisor de tensión
- **Condensador desacoplo:** 100 nF entre VCC y GND del sensor (recomendado)

---

## ALIMENTACIÓN

| Componente | Voltaje | Corriente |
|-----------|---------|-----------|
| ESP32-S3 + Display | 3.3V | ~500 mA |
| TJA1051 CAN | 5V | ~70 mA |
| TF-Mini Plus | 5V | ~120 mA |

---

## VERIFICACIÓN

### Con Multímetro

1. ESP32 3.3V → GND: **3.30V ± 0.1V**
2. Display VCC → GND: **3.30V ± 0.1V**
3. TJA1051 VCC (pin 3) → GND: **5.00V ± 0.2V**
4. TJA1051 VIO (pin 5) → GND: **3.30V ± 0.1V** ⚠️ Si ≠ 3.3V, NO conectar ESP32
5. TF-Mini Plus VCC → GND: **5.00V ± 0.2V**
5. CANH → GND (idle): **~2.5V**
6. CANL → GND (idle): **~2.5V**
7. CANH ↔ CANL (resistencia): **60Ω** (con terminación)

### En Monitor Serial

1. ESP32 arranca: `[HMI] ESP32 HMI CAN bring-up booted`
2. TFT inicializado: `[TFT] Display initialized`
3. CAN funciona: `[CAN] Initialized at 500 kbps`
4. TF-Mini Plus: `[OBSTACLE] TF-Mini Plus init (UART1, 115200 bps)`
5. Heartbeat: `[HMI] heartbeat` cada 1 segundo

---

## SOLUCIÓN RÁPIDA DE PROBLEMAS

### Solución de Problemas

- **Pantalla negra** → Verificar GPIO 42 = HIGH (backlight)
- **No responde touch** → Verificar GPIO 21 a T_CS y GPIO 12 a T_DO
- **Líneas en pantalla** → Cables SPI muy largos, agregar capacitor 100nF

### CAN Bus

- **Sin comunicación** → Verificar resistencias 120Ω en ambos extremos
- **Errores frecuentes** → Pin S (pin 8) debe ir a GND, no flotar
- **Bus-off** → Verificar 500 kbps en ambos MCUs

### TF-Mini Plus

- **Estado INVALID permanente** → Verificar VCC = 5V (no 3.3V)
- **Sin datos** → Verificar que sensor TX (White/Green) está conectado a GPIO 18
- **Checksum fallido** → Verificar baudrate 115200 en firmware

---

## DIAGRAMA SIMPLIFICADO

```
┌─────────────┐
│  ESP32-S3   │
│             │
│  GPIO 12 ◄──┼─── Display MISO + Touch DO
│  GPIO 13 ───┼──→ Display MOSI + Touch DIN
│  GPIO 14 ───┼──→ Display SCK + Touch CLK
│  GPIO 10 ───┼──→ Display CS
│  GPIO 39 ───┼──→ Display DC/RS
│  GPIO 38 ───┼──→ Display RESET
│  GPIO 21 ───┼──→ Touch CS
│  GPIO 42 ───┼──→ Display LED
│             │
│  GPIO 4  ───┼──→ CAN TX (a TJA1051)
│  GPIO 5  ◄──┼─── CAN RX (de TJA1051)
│             │
│  GPIO 18 ◄──┼─── TF-Mini Plus TX (UART1 RX, 115200 bps, 3.3V direct)
│             │
│  3.3V ──────┼──→ Display VCC + TJA1051 VIO (pin 5) ⚠️
│  5V (ext.) ─┼──→ TJA1051 VCC (pin 3) + TF-Mini Plus VCC
│  GND ───────┼──→ Común (todos)
│             │
└─────────────┘
```

---

**Ver documentación completa en:**
- `ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md` (guía detallada)
- `DIAGRAMA_PINES_VISUAL.md` (diagramas visuales)
- `TFMINI_PLUS_WIRING_GUIDE.md` (guía de conexión TF-Mini Plus)

**Configuración firmware:**
- `esp32/include/User_Setup.h` (definiciones de pines TFT_eSPI)
- `esp32/src/main.cpp` (código principal)

---

_Control Coche Marcos - ESP32-S3 HMI System_  
_Versión 1.0 - 2026-02-19_
