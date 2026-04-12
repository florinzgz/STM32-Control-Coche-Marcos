# DIAGRAMA VISUAL DE PINES - ESP32-S3
## Guía Rápida de Conexiones: Display TFT y CAN-Bus

---

## VISTA GENERAL DEL SISTEMA

```
                                   ┌────────────────────────────┐
                                   │       ESP32-S3-DevKitC-1   │
                                   │    (Controlador Principal)  │
                                   │                            │
                                   │  16MB Flash + 8MB PSRAM    │
                                   │  Dual Core @ 240MHz        │
                                   └─────────┬──────────────────┘
                                             │
                    ┌────────────────────────┼────────────────────────┐
                    │                        │                        │
                    ▼                        ▼                        ▼
         ┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐
         │  Display TFT     │    │   CAN Transceiver│    │   Alimentación   │
         │  ST7796          │    │   TJA1051        │    │                  │
         │  480×320 pixels  │    │   500 kbps       │    │   3.3V / 5V      │
         └──────────────────┘    └──────────────────┘    └──────────────────┘
```

---

## 1. CONEXIONES DISPLAY TFT ST7796

### Esquema de Pines del Display

```
   Vista Frontal del Display (480×320)
   ┌────────────────────────────────────┐
   │                                    │
   │         PANTALLA TFT               │
   │        (lado visible)              │
   │                                    │
   └────────────────────────────────────┘
            │  Conector de Pines
            ▼
   ┌──────────────────────────────────────────────┐
   │ PIN │ NOMBRE  │ FUNCIÓN                      │
   ├─────┼─────────┼──────────────────────────────┤
   │  1  │ VCC     │ Alimentación 3.3V            │
   │  2  │ GND     │ Tierra                       │
   │  3  │ CS      │ Chip Select Display          │
   │  4  │ RESET   │ Reset Display                │
   │  5  │ DC/RS   │ Data/Command                 │
   │  6  │ SDI     │ MOSI (Datos SPI)             │
   │  7  │ SCK     │ Reloj SPI                    │
   │  8  │ LED     │ Backlight (retroiluminación) │
   │  9  │ SDO     │ MISO (compartido con T_DO)   │
   │ 10  │ T_CLK   │ Touch Clock                  │
   │ 11  │ T_CS    │ Touch Chip Select            │
   │ 12  │ T_DIN   │ Touch Data In                │
   │ 13  │ T_DO    │ Touch Data Out (→ GPIO 12)   │
   │ 14  │ T_IRQ   │ Touch Interrupt (no usado)   │
   └─────┴─────────┴──────────────────────────────┘
```

### Tabla de Conexión Display ↔ ESP32-S3

```
┌──────────────┬──────────────┬──────────────┬──────────────────────────┐
│ Pin Display  │   Señal      │  GPIO ESP32  │        Función           │
├──────────────┼──────────────┼──────────────┼──────────────────────────┤
│   VCC        │ Alimentación │    3.3V      │ Fuente regulada 3.3V     │
│   GND        │ Tierra       │    GND       │ Común con ESP32          │
├──────────────┼──────────────┼──────────────┼──────────────────────────┤
│   CS         │ Chip Select  │   GPIO 10    │ Selección Display        │
│   RESET      │ Reset        │   GPIO 38    │ Reset hardware           │
│   DC/RS      │ Data/Command │   GPIO 39    │ Modo datos/comandos      │
│   SDI (MOSI) │ Datos SPI    │   GPIO 13    │ Master Out Slave In      │
│   SCK        │ Reloj SPI    │   GPIO 14    │ Clock de sincronización  │
│   LED        │ Backlight    │   GPIO 42    │ Control de brillo        │
│   SDO (MISO) │ Lectura SPI  │   GPIO 12    │ Compartido con T_DO      │
├──────────────┼──────────────┼──────────────┼──────────────────────────┤
│   T_CLK      │ Touch Clock  │   GPIO 14    │ Compartido con SCK       │
│   T_CS       │ Touch CS     │   GPIO 21    │ Selección Touch Panel    │
│   T_DIN      │ Touch Data   │   GPIO 13    │ Compartido con MOSI      │
│   T_DO       │ Touch Out    │   GPIO 12    │ Compartido con MISO      │
│   T_IRQ      │ Touch IRQ    │    NC        │ No usado (polling)       │
└──────────────┴──────────────┴──────────────┴──────────────────────────┘
```

### Diagrama de Conexión Física - Display

```
ESP32-S3 DevKitC-1                           Display TFT ST7796
┌─────────────────┐                          ┌─────────────────┐
│                 │                          │                 │
│      3.3V  ─────┼──────────────────────────┼───► VCC         │
│      GND   ─────┼──────────────────────────┼───► GND         │
│                 │                          │                 │
│   GPIO 12  ◄────┼──────────────────────────┼───◄ SDO (MISO)  │
│   GPIO 13  ─────┼──────────────────────────┼───► SDI (MOSI)  │
│   GPIO 14  ─────┼──────────────────────────┼───► SCK         │
│   GPIO 10  ─────┼──────────────────────────┼───► CS          │
│   GPIO 39  ─────┼──────────────────────────┼───► DC/RS       │
│   GPIO 38  ─────┼──────────────────────────┼───► RESET       │
│   GPIO 42  ─────┼──────────────────────────┼───► LED         │
│                 │                          │                 │
│   GPIO 12  ◄────┼──────────────────────────┼───◄ T_DO        │
│   GPIO 13  ─────┼──────────┬───────────────┼───► T_DIN       │
│   (shared) │                          │                 │
│   GPIO 14  ─────┼──────────┼───────────────┼───► T_CLK       │
│   (shared) │                          │                 │
│   GPIO 21  ─────┼──────────────────────────┼───► T_CS        │
│                 │                          │                 │
│                 │                          │                 │
└─────────────────┘                          └─────────────────┘

NOTAS:
• GPIO12 es compartido entre Display MISO (SDO) y Touch Data Out (T_DO)
• GPIO13 y GPIO14 son compartidos entre Display SPI y Touch Panel
• LED (GPIO42) controla la retroiluminación: HIGH=encendido, LOW=apagado
• MISO (GPIO12) es necesario para que el XPT2046 envíe coordenadas touch
• Touch panel usa polling en lugar de interrupciones (T_IRQ no conectado)
```

---

## 2. CONEXIONES CAN BUS

### Esquema del Transceiver TJA1051

```
   Vista Superior del Módulo TJA1051
   ┌─────────────────────────────┐
   │    TJA1051T/3 CAN Module    │
   │  High-Speed CAN Transceiver │
   └─────────────────────────────┘
            │  Pinout
            ▼
   ┌──────────────────────────────────────────┐
   │ PIN │ NOMBRE │ FUNCIÓN                   │
   ├─────┼────────┼───────────────────────────┤
   │  1  │ TXD    │ Transmit Data (del MCU)   │
   │  2  │ GND    │ Ground                    │
   │  3  │ VCC    │ 5V Power Supply           │
   │  4  │ RXD    │ Receive Data (al MCU)     │
   │  5  │ VREF   │ Voltage Reference (NC)    │
   │  6  │ CANL   │ CAN Low (bus)             │
   │  7  │ CANH   │ CAN High (bus)            │
   │  8  │ S      │ Silent Mode (a GND)       │
   └─────┴────────┴───────────────────────────┘
```

### Tabla de Conexión CAN - ESP32-S3 lado

```
┌──────────────┬──────────────┬──────────────┬──────────────────────────┐
│  GPIO ESP32  │   Señal      │  Pin TJA1051 │        Función           │
├──────────────┼──────────────┼──────────────┼──────────────────────────┤
│   GPIO 4     │ CAN TX       │   Pin 1 TXD  │ Transmisión CAN          │
│   GPIO 5     │ CAN RX       │   Pin 4 RXD  │ Recepción CAN            │
│     5V       │ Alimentación │   Pin 3 VCC  │ Fuente 5V regulada       │
│     GND      │ Tierra       │   Pin 2 GND  │ Común con ESP32          │
│     GND      │ Normal Mode  │   Pin 8 S    │ Desactiva modo silencioso│
│    3.3V      │ Nivel I/O    │   Pin 5 VIO  │ ⚠️ OBLIGATORIO → 3.3V    │
└──────────────┴──────────────┴──────────────┴──────────────────────────┘
```

### Diagrama Completo de CAN Bus

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      SISTEMA CAN BUS COMPLETO                                │
│                     ESP32-S3 ←→ CAN ←→ STM32G474RE                          │
└─────────────────────────────────────────────────────────────────────────────┘

ESP32-S3               TJA1051 #1           BUS CAN          TJA1051 #2         STM32G474RE
DevKitC-1           (lado ESP32)      (cable trenzado)    (lado STM32)           Nucleo-64

┌──────────┐      ┌──────────────┐    ┌──────────────┐  ┌──────────────┐      ┌──────────┐
│          │      │              │    │              │  │              │      │          │
│  GPIO4 ──┼─────►│1 TXD    CANH├────┤7  ════════  7├──┤7 CANH    TXD│◄─────┼── PA12   │
│   (TX)   │      │         (7)  │    │    CANH      │  │  (7)         │      │  (TX)    │
│          │      │              │    │     ║║       │  │              │      │          │
│  GPIO5 ◄─┼──────│4 RXD         │    │    120Ω      │  │         RXD 4├──────┼─► PA11   │
│   (RX)   │      │              │    │     ║║       │  │              │      │  (RX)    │
│          │      │         CANL├────┤6  ════════  6├──┤6 CANL        │      │          │
│   5V  ───┼─────►│3 VCC    (6)  │    │    CANL      │  │  (6)     VCC 3│◄─────┼── 5V     │
│          │      │              │    │     ║║       │  │              │      │          │
│   GND ───┼─────►│2 GND         │    │    120Ω      │  │         GND 2│◄─────┼── GND    │
│          │      │              │    │              │  │              │      │          │
│   GND ───┼─────►│8 S (Silent)  │    └──────────────┘  │ S (Silent) 8 │◄─────┼── GND    │
│          │      │   (a GND)    │                      │   (a GND)    │      │          │
└──────────┘      └──────┬───────┘                      └───────┬──────┘      └──────────┘
                         │                                      │
                      ┌──┴──┐                                ┌──┴──┐
                      │100nF│   Capacitor de desacoplo       │100nF│
                      └──┬──┘   (muy cerca de VCC)           └──┬──┘
                         │                                      │
                        GND                                    GND


DISTANCIA MÁXIMA: ~40 metros a 500 kbps
CABLE RECOMENDADO: Par trenzado (CANH y CANL juntos)
TERMINACIÓN: 120Ω en CADA extremo del bus (entre CANH y CANL)
```

### Detalle de las Resistencias de Terminación

```
Extremo ESP32-S3:                    Extremo STM32G474RE:

TJA1051 #1                           TJA1051 #2
┌─────────┐                          ┌─────────┐
│         │                          │         │
│   CANH──┼──┐                  ┌────┼── CANH  │
│   (7)   │  │                  │    │   (7)   │
│         │ 120Ω                120Ω │         │
│   CANL──┼──┘                  └────┼── CANL  │
│   (6)   │                          │   (6)   │
│         │                          │         │
└─────────┘                          └─────────┘

Resistencias: 1/4W, ±5% tolerancia
Medición esperada entre CANH-CANL con ambas instaladas: ~60Ω
(120Ω en paralelo con 120Ω = 60Ω)
```

---

## 3. RESUMEN DE PINES - ESP32-S3

### Tabla Consolidada

```
┌──────────┬──────────────┬────────────────────────────────────────────┐
│   GPIO   │   Función    │              Conecta a                     │
├──────────┼──────────────┼────────────────────────────────────────────┤
│    4     │  CAN TX      │ TJA1051 pin 1 (TXD)                        │
│    5     │  CAN RX      │ TJA1051 pin 4 (RXD)                        │
├──────────┼──────────────┼────────────────────────────────────────────┤
│   10     │  SPI CS      │ Display CS (Chip Select)                   │
│   12     │  SPI MISO    │ Display SDO + Touch T_DO (compartido)      │
│   13     │  SPI MOSI    │ Display SDI + Touch T_DIN (compartido)     │
│   14     │  SPI SCK     │ Display SCK + Touch T_CLK (compartido)     │
│   39     │  DC/RS       │ Display DC/RS (Data/Command)               │
│   38     │  RESET       │ Display RESET                              │
│   21     │  Touch CS    │ Touch Panel T_CS                           │
│   42     │  Backlight   │ Display LED (retroiluminación)             │
├──────────┼──────────────┼────────────────────────────────────────────┤
│   3.3V   │ Alimentación │ Display VCC                                │
│    5V    │ Alimentación │ TJA1051 VCC (pin 3)                        │
│   3.3V   │ Nivel I/O    │ TJA1051 VIO (pin 5) ⚠️ OBLIGATORIO         │
│   GND    │ Tierra       │ Común: Display, TJA1051, STM32            │
└──────────┴──────────────┴────────────────────────────────────────────┘
```

### Mapa Visual de GPIOs en el DevKitC-1

```
                 ESP32-S3-DevKitC-1
                 Vista Superior
    
    ┌─────────────────────────────────────────┐
    │  [USB-C]                                │
    │                                         │
    │  3V3  ●●●●●●●●●●●●●●●●●●●●● GND        │
    │  RST                           G0       │
    │  4 ◄──CAN TX                   G1       │
    │  5 ◄──CAN RX                   G2       │
    │  6                             G3       │
    │  7                             G8       │
    │  8                             G9       │
    │  9                             G10 ─►CS │
    │  10 ─►TFT CS                   G11      │
    │  19                            G12◄─MISO│
    │  20                            G13 ─►MOSI
    │  21 ◄─Touch CS                 G14 ─►SCK│
    │  47                            G33 (PSRAM-internal)
    │  48                            G34 (PSRAM-internal)
    │  45 (strapping)              G35 (PSRAM-internal)
    │  GND                           G36 (PSRAM-internal)
    │  5V                            G37 (PSRAM-internal)
    │                                G38 ─►RST│
    │                                G39 ─►DC/RS
    │                                G40      │
    │                                G41      │
    │                                G42      │
    │                                         │
    └─────────────────────────────────────────┘
    
    Leyenda:
    ● = Pines de alimentación y tierra
    ◄ = Pin de salida del ESP32 (output)
    ─► = Pin de entrada al ESP32 (input)
    GPIO 26–32 no están en los headers (reservados para Flash/PSRAM)
    GPIO 33–37 no están accesibles en el módulo WROOM-1-N16R8 (bus interno Octal PSRAM)
```

---

## 4. CABLES NECESARIOS

### Lista de Materiales de Cableado

```
┌──────┬────────────────────────┬──────────┬──────────────────────────┐
│ Cant │      Componente        │ Longitud │          Uso             │
├──────┼────────────────────────┼──────────┼──────────────────────────┤
│  1   │ Cable Dupont 20 pines  │  15 cm   │ ESP32 ↔ Display          │
│  1   │ Cable Dupont 5 pines   │  10 cm   │ ESP32 ↔ TJA1051 (TXD,RXD,VCC,VIO,GND) │
│  1   │ Par trenzado 24 AWG    │  Var.    │ CANH/CANL (CAN bus)      │
│  2   │ Resistencia 120Ω 1/4W  │   —      │ Terminación CAN          │
│  2   │ Capacitor 100nF        │   —      │ Desacoplo VCC (display)  │
│  1   │ Fuente 3.3V @ 500mA    │   —      │ ESP32 + Display          │
│  1   │ Fuente 5V @ 100mA      │   —      │ TJA1051 transceiver      │
└──────┴────────────────────────┴──────────┴──────────────────────────┘
```

### Código de Colores Recomendado

```
┌────────────┬──────────┬────────────────────────────────┐
│   Señal    │  Color   │           Propósito            │
├────────────┼──────────┼────────────────────────────────┤
│ VCC (3.3V) │  Rojo    │ Alimentación positiva          │
│ GND        │  Negro   │ Tierra común                   │
│ CAN TX     │  Amarillo│ Transmisión CAN                │
│ CAN RX     │  Verde   │ Recepción CAN                  │
│ CANH       │  Naranja │ CAN High (bus)                 │
│ CANL       │  Blanco  │ CAN Low (bus)                  │
│ SPI MOSI   │  Azul    │ Datos SPI                      │
│ SPI SCK    │  Violeta │ Reloj SPI                      │
│ Otros GPIO │  Marrón  │ Señales de control             │
└────────────┴──────────┴────────────────────────────────┘
```

---

## 5. VERIFICACIÓN PASO A PASO

### Checklist de Conexión

```
┌────┬──────────────────────────────────────────────────┬────┐
│ ☐  │ 1. Verificar 3.3V en ESP32-S3 con multímetro     │    │
│ ☐  │ 2. Conectar GND común (ESP32, display, TJA1051)  │    │
│ ☐  │ 3. Conectar VCC display (3.3V)                   │    │
│ ☐  │ 4. Conectar pines SPI display (13, 14, 15)       │    │
│ ☐  │ 5. Conectar control display (16, 17, 42)         │    │
│ ☐  │ 6. Conectar touch panel (21)                     │    │
│ ☐  │ 7. Cargar firmware de prueba display             │    │
│ ☐  │ 8. Verificar pantalla se enciende                │    │
│ ☐  │ 9. Conectar 5V a TJA1051 VCC (pin 3)              │    │
│ ☐  │ 9b. Conectar 3.3V a TJA1051 VIO (pin 5) ⚠️       │    │
│ ☐  │ 10. Conectar GPIO 4 y 5 a TJA1051                │    │
│ ☐  │ 11. Conectar pin S (Silent) a GND                │    │
│ ☐  │ 12. Instalar capacitor 100nF en VCC del TJA1051  │    │
│ ☐  │ 13. Conectar CANH y CANL al otro transceiver     │    │
│ ☐  │ 14. Instalar resistencias 120Ω en ambos extremos │    │
│ ☐  │ 15. Medir 60Ω entre CANH-CANL (verificación)     │    │
│ ☐  │ 16. Cargar firmware completo                     │    │
│ ☐  │ 17. Verificar heartbeat CAN con monitor          │    │
└────┴──────────────────────────────────────────────────┴────┘
```

---

## CONTACTO Y SOPORTE

- **Repositorio:** florinzgz/STM32-Control-Coche-Marcos
- **Documentación Completa:** `docs/ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md`
- **Firmware:** `esp32/src/main.cpp`
- **Configuración:** `esp32/platformio.ini`

---

**IMPORTANTE:** Este documento es una guía visual complementaria. Para detalles técnicos completos, especificaciones eléctricas y troubleshooting avanzado, consultar `ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md`.

---

_Última actualización: 2026-02-19_
