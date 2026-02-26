# REFERENCIA RÁPIDA - CONEXIONES ESP32-S3
## Pines de Pantalla y CAN-Bus

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
| 5V | → | Pin 3 | VCC (Alimentación) |
| GND | → | Pin 2 | GND (Tierra) |
| GND | → | Pin 8 | S (Modo Normal) |

### Bus CAN Físico

| TJA1051 | → | Bus | Notas |
|---------|---|-----|-------|
| Pin 7 (CANH) | → | Cable naranja | Par trenzado |
| Pin 6 (CANL) | → | Cable blanco | Par trenzado |
| 120Ω | ↔ | CANH ↔ CANL | En cada extremo |

---

## ALIMENTACIÓN

| Componente | Voltaje | Corriente |
|-----------|---------|-----------|
| ESP32-S3 + Display | 3.3V | ~500 mA |
| TJA1051 CAN | 5V | ~70 mA |

---

## VERIFICACIÓN

### Con Multímetro

1. ESP32 3.3V → GND: **3.30V ± 0.1V**
2. Display VCC → GND: **3.30V ± 0.1V**
3. TJA1051 VCC → GND: **5.00V ± 0.2V**
4. CANH → GND (idle): **~2.5V**
5. CANL → GND (idle): **~2.5V**
6. CANH ↔ CANL (resistencia): **60Ω** (con terminación)

### En Monitor Serial

1. ESP32 arranca: `[HMI] ESP32 HMI CAN bring-up booted`
2. TFT inicializado: `[TFT] Display initialized`
3. CAN funciona: `[CAN] Initialized at 500 kbps`
4. Heartbeat: `[HMI] heartbeat` cada 1 segundo

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
│  3.3V ──────┼──→ Display VCC
│  5V ────────┼──→ TJA1051 VCC
│  GND ───────┼──→ Común (todos)
│             │
└─────────────┘
```

---

**Ver documentación completa en:**
- `ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md` (guía detallada)
- `DIAGRAMA_PINES_VISUAL.md` (diagramas visuales)

**Configuración firmware:**
- `esp32/include/User_Setup.h` (definiciones de pines TFT_eSPI)
- `esp32/src/main.cpp` (código principal)

---

_Control Coche Marcos - ESP32-S3 HMI System_  
_Versión 1.0 - 2026-02-19_
