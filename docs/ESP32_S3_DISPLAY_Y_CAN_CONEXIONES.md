# Guía de Conexiones - ESP32-S3: Pantalla TFT y CAN Bus

**Fecha:** 2026-02-19  
**Versión:** 1.0  
**Proyecto:** Control Coche Marcos - HMI ESP32-S3  

---

## RESUMEN

Este documento describe **TODAS** las conexiones de pines entre:
1. **ESP32-S3** ↔ **Pantalla TFT ST7796 (480×320) con Touch**
2. **ESP32-S3** ↔ **Módulo CAN Bus TJA1051** ↔ **STM32G474RE**

---

## 1. CONEXIONES DE PANTALLA TFT

### 1.1 Hardware de la Pantalla

La pantalla utilizada es:
- **Tipo:** TFT LCD de 480×320 píxeles
- **Driver:** ST7796
- **Interface:** SPI (Serial Peripheral Interface)
- **Touch Panel:** Capacitivo o resistivo, chip select independiente
- **Voltaje:** 3.3V

### 1.2 Mapa de Pines - Pantalla → ESP32-S3

La pantalla tiene los siguientes pines según tu descripción:
- **t_irq, t_do, t_din, t_cs, t_clk** — Touch panel
- **sdo/miso, led, sck, sdi(mosi), dc/rs, reset, cs** — Display TFT
- **gnd, vcc** — Alimentación

#### Tabla Completa de Conexiones

| Pin en Pantalla | Función | Pin ESP32-S3 | GPIO | Notas |
|-----------------|---------|--------------|------|-------|
| **VCC** | Alimentación 3.3V | 3.3V | — | Fuente de alimentación regulada |
| **GND** | Tierra | GND | — | Común con ESP32 |
| **LED** | Backlight (Retroiluminación) | GPIO42 | 42 | Control de brillo (HIGH=encendido) |
| **CS** | Chip Select Display | GPIO15 | 15 | Selección display (activo bajo) |
| **RESET** | Reset Display | GPIO17 | 17 | Reset del display (activo bajo) |
| **DC/RS** | Data/Command | GPIO16 | 16 | HIGH=data, LOW=command |
| **SDI (MOSI)** | Master Out Slave In | GPIO13 | 13 | Datos SPI del ESP32 al display |
| **SCK** | SPI Clock | GPIO14 | 14 | Reloj SPI |
| **SDO (MISO)** | Master In Slave Out | *No conectado* | -1 | No usado en este proyecto |
| **T_CLK** | Touch Clock | GPIO14 | 14 | Compartido con SCK del display |
| **T_CS** | Touch Chip Select | GPIO21 | 21 | Selección touch panel |
| **T_DIN** | Touch Data In | GPIO13 | 13 | Compartido con SDI (MOSI) |
| **T_DO** | Touch Data Out | *No usado* | — | No configurado en platformio.ini |
| **T_IRQ** | Touch Interrupt | *No usado* | — | Polling mode en lugar de interrupciones |

### 1.3 Diagrama de Conexión - Display

```
┌─────────────────────────────────────────────────────────────────┐
│                           ESP32-S3                               │
│                        DevKitC-1 N16R8                           │
│                                                                  │
│  3.3V ─────────────────────────────────────────► VCC (Display)  │
│  GND  ─────────────────────────────────────────► GND (Display)  │
│                                                                  │
│  GPIO13 (SPI MOSI) ─────────────────────────────► SDI (MOSI)    │
│  GPIO14 (SPI SCK)  ─────────────────────────────► SCK           │
│  GPIO15 (SPI CS)   ─────────────────────────────► CS            │
│  GPIO16 (DC/RS)    ─────────────────────────────► DC/RS         │
│  GPIO17 (RESET)    ─────────────────────────────► RESET         │
│  GPIO42 (Backlight)─────────────────────────────► LED           │
│                                                                  │
│  GPIO13 (compartido)─────────────────────────────► T_DIN        │
│  GPIO14 (compartido)─────────────────────────────► T_CLK        │
│  GPIO21 (Touch CS) ──────────────────────────────► T_CS         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 1.4 Configuración SPI del Display

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Frecuencia Display** | 40 MHz | Configurado en platformio.ini |
| **Frecuencia Touch** | 2.5 MHz | Más lento para estabilidad |
| **Modo SPI** | Modo 0 | CPOL=0, CPHA=0 |
| **Bits por transferencia** | 8 bits | — |
| **Orden de bits** | MSB first | — |
| **Rotación** | 1 (Landscape) | 480×320 horizontal |

### 1.5 Configuración en Código

Las siguientes definiciones están en `esp32/platformio.ini`:

```ini
-DUSER_SETUP_LOADED=1
-DST7796_DRIVER=1
-DTFT_WIDTH=320
-DTFT_HEIGHT=480
-DTFT_MISO=-1
-DTFT_MOSI=13
-DTFT_SCLK=14
-DTFT_CS=15
-DTFT_DC=16
-DTFT_RST=17
-DTFT_BL=42
-DTFT_BACKLIGHT_ON=1
-DSPI_FREQUENCY=40000000
-DSPI_READ_FREQUENCY=20000000
-DTOUCH_CS=21
-DSPI_TOUCH_FREQUENCY=2500000
```

---

## 2. CONEXIONES CAN BUS

### 2.1 Hardware CAN

- **Transceiver:** TJA1051T/3 (High-Speed CAN)
- **Velocidad:** 500 kbps
- **Topología:** ESP32-S3 ↔ TJA1051 ↔ Bus CAN ↔ TJA1051 ↔ STM32G474RE
- **Terminación:** 120Ω en cada extremo del bus

### 2.2 Pines CAN - ESP32-S3

| Pin ESP32-S3 | Función | Conecta a | Notas |
|--------------|---------|-----------|-------|
| **GPIO4** | CAN TX | Pin 1 (TXD) del TJA1051 | Transmisión CAN |
| **GPIO5** | CAN RX | Pin 4 (RXD) del TJA1051 | Recepción CAN |

### 2.3 Conexiones TJA1051 - Lado ESP32-S3

#### Tabla de Pines TJA1051 (ESP32)

| Pin TJA1051 | Nombre | Conecta a | Función |
|-------------|--------|-----------|---------|
| **1** | TXD | GPIO4 (ESP32) | Entrada de transmisión desde ESP32 |
| **2** | GND | GND común | Tierra del transceiver |
| **3** | VCC | +5V | Alimentación 5V del transceiver |
| **4** | RXD | GPIO5 (ESP32) | Salida de recepción al ESP32 |
| **5** | VREF | No conectado | Referencia de voltaje (opcional) |
| **6** | CANL | Bus CAN Low | Cable trenzado al otro transceiver |
| **7** | CANH | Bus CAN High | Cable trenzado al otro transceiver |
| **8** | S (Silent) | GND | Modo normal (conectar a GND) |

### 2.4 Diagrama de Conexión CAN Completo

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                          Sistema CAN Completo                                 │
└──────────────────────────────────────────────────────────────────────────────┘

ESP32-S3                    TJA1051 #1              Bus CAN          TJA1051 #2              STM32G474RE
┌──────────┐            ┌──────────────┐     ┌──────────────┐   ┌──────────────┐            ┌──────────┐
│          │            │              │     │              │   │              │            │          │
│  GPIO4 ──┼───────────►│ 1 TXD   CANH├─────┤──── CANH ────├───┤ CANH   TXD 1 │◄───────────┤ PB9 (TX) │
│  (TX)    │            │              │     │              │   │              │            │          │
│          │            │    7         │     │              │   │        7     │            │          │
│  GPIO5 ◄─┼────────────│ 4 RXD        │     │   120Ω       │   │        RXD 4 ├───────────►│ PB8 (RX) │
│  (RX)    │            │              │     │    ║         │   │              │            │          │
│          │            │    6    CANL├─────┤──── CANL ────├───┤ CANL   6     │            │          │
│  +5V  ───┼───────────►│ 3 VCC        │     │              │   │        VCC 3 │◄───────────┤ +5V      │
│          │            │              │     │   120Ω       │   │              │            │          │
│  GND  ───┼───────────►│ 2 GND        │     │              │   │        GND 2 │◄───────────┤ GND      │
│          │            │              │     │              │   │              │            │          │
│  GND  ───┼───────────►│ 8 S (Silent) │     └──────────────┘   │ S (Silent) 8 │◄───────────┤ GND      │
│          │            │              │                         │              │            │          │
└──────────┘            └──────────────┘                         └──────────────┘            └──────────┘
                              │                                          │
                            100nF                                      100nF
                              │                                          │
                             GND                                        GND
```

### 2.5 Especificaciones del Bus CAN

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Velocidad** | 500 kbps | Configurado en firmware |
| **Terminación** | 120Ω en cada extremo | 2 resistencias necesarias |
| **Cable** | Par trenzado | Longitud máxima ~40 metros a 500 kbps |
| **Voltaje diferencial (dominante)** | ~2V | CANH=3.5V, CANL=1.5V |
| **Voltaje diferencial (recesivo)** | ~0V | CANH=2.5V, CANL=2.5V |
| **Capacitores de desacoplo** | 100nF | Cerca de pin VCC de cada TJA1051 |

### 2.6 Configuración en Código ESP32

En `esp32/src/main.cpp`:

```cpp
// CAN transceiver pins (TJA1051)
static constexpr int CAN_TX_PIN = 4;
static constexpr int CAN_RX_PIN = 5;

// En setup():
ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
ESP32Can.setRxQueueSize(5);
ESP32Can.setTxQueueSize(5);

if (ESP32Can.begin(ESP32Can.convertSpeed(500))) {
    Serial.println("[CAN] Initialized at 500 kbps");
}
```

### 2.7 Mensajes CAN - ESP32 ↔ STM32

El ESP32-S3 envía y recibe los siguientes mensajes CAN:

#### Mensajes Enviados por ESP32

| ID CAN | Nombre | Frecuencia | Datos |
|--------|--------|------------|-------|
| **0x011** | HEARTBEAT_ESP32 | 100 ms | 1 byte: contador (0-255) |

#### Mensajes Recibidos por ESP32

| ID CAN | Nombre | DLC | Descripción |
|--------|--------|-----|-------------|
| **0x001** | HEARTBEAT_STM32 | 4 | Estado del sistema, contadores, errores |
| **0x200** | STATUS_SPEED | 8 | Velocidad de 4 ruedas (2 bytes cada una) |
| **0x201** | STATUS_CURRENT | 8 | Corriente de 4 motores (2 bytes cada uno) |
| **0x202** | STATUS_TEMP | 5 | Temperaturas de 5 sensores (1 byte cada uno) |
| **0x203** | STATUS_SAFETY | 3 | Estado ABS, TCS, errores |
| **0x204** | STATUS_STEERING | 3 | Ángulo de dirección calibrado |
| **0x205** | STATUS_TRACTION | 4 | Escala de tracción por rueda |
| **0x206** | STATUS_TEMP_MAP | 5 | Mapa térmico de motores |
| **0x207** | STATUS_BATTERY | 4 | Voltaje y corriente de batería |
| **0x300** | DIAG_ERROR | 2+ | Códigos de error de diagnóstico |
| **0x301** | SERVICE_FAULTS | 4 | Bitmask de fallos de servicio |
| **0x302** | SERVICE_ENABLED | 4 | Funciones habilitadas en modo servicio |
| **0x303** | SERVICE_DISABLED | 4 | Funciones deshabilitadas |
| **0x501** | CMD_ACK | 3 | Acuse de recibo de comandos |

---

## 3. LISTA DE MATERIALES

### 3.1 Display TFT

| Cantidad | Componente | Especificaciones |
|----------|-----------|------------------|
| 1 | Display TFT LCD | 480×320 píxeles, ST7796, 3.3V, SPI |
| 1 | Touch Panel | Resistivo o capacitivo, integrado |
| 1 | Cable Dupont Hembra | Mínimo 12 pines para conexiones |

### 3.2 CAN Bus

| Cantidad | Componente | Especificaciones |
|----------|-----------|------------------|
| 1 | Módulo TJA1051T/3 | Transceiver CAN, 5V, compatible 3.3V I/O |
| 1 | Resistencia 120Ω | 1/4W, terminación CAN |
| 1 | Capacitor 100nF | Cerámico, desacoplo VCC |
| 1 | Cable par trenzado | 2 conductores, 24 AWG, hasta 40m |

### 3.3 Alimentación

| Fuente | Voltaje | Corriente | Uso |
|--------|---------|-----------|-----|
| Regulador ESP32-S3 | 3.3V | ~500mA | Display + ESP32 |
| Fuente externa | 5V | ~100mA | TJA1051 transceiver |

---

## 4. PROCEDIMIENTO DE CONEXIÓN

### 4.1 Orden de Montaje Recomendado

1. **Verificar alimentación:**
   - Conectar 3.3V y GND al ESP32-S3
   - Verificar con multímetro: 3.3V estables

2. **Conectar Display TFT:**
   - Comenzar con VCC y GND del display
   - Conectar pines SPI: MOSI (13), SCK (14), CS (15)
   - Conectar pines de control: DC (16), RESET (17), LED (42)
   - Conectar Touch: T_CS (21), T_DIN y T_CLK compartidos

3. **Probar Display:**
   - Cargar firmware de prueba
   - Verificar que la pantalla se enciende
   - Verificar respuesta táctil

4. **Conectar CAN Bus:**
   - Montar TJA1051 con capacitor 100nF en VCC
   - Conectar GPIO4 y GPIO5 al transceiver
   - Conectar 5V y GND al TJA1051
   - Conectar pin S (Silent) a GND

5. **Conectar Bus CAN físico:**
   - Conectar CANH y CANL al otro transceiver (STM32)
   - Instalar resistencias de 120Ω en ambos extremos

6. **Probar CAN:**
   - Cargar firmware completo
   - Verificar heartbeat en monitor CAN
   - Confirmar mensajes bidireccionales

### 4.2 Verificación con Multímetro

| Punto de Medición | Voltaje Esperado | Notas |
|-------------------|------------------|-------|
| ESP32 3.3V → GND | 3.30V ±0.1V | Alimentación estable |
| Display VCC → GND | 3.30V ±0.1V | Mismo que ESP32 |
| TJA1051 VCC → GND | 5.00V ±0.2V | Fuente 5V |
| CANH → GND (idle) | ~2.5V | Sin transmisión |
| CANL → GND (idle) | ~2.5V | Sin transmisión |
| CANH - CANL (idle) | ~0V | Diferencial en reposo |
| CANH - CANL (activo) | ~2V | Durante transmisión |
| CANH ↔ CANL (resistencia) | 60Ω | Con bus terminado (2×120Ω en paralelo) |

---

## 5. SOLUCIÓN DE PROBLEMAS

### 5.1 Problemas de Display

| Síntoma | Causa Probable | Solución |
|---------|----------------|----------|
| Pantalla negra | Sin alimentación o backlight apagado | Verificar VCC=3.3V, GPIO42=HIGH |
| Display blanco | Reset no funcionando | Verificar GPIO17, agregar delay después de reset |
| Imagen invertida/rotada | Rotación incorrecta | Cambiar `tft.setRotation(1)` en código |
| Touch no responde | T_CS no conectado | Verificar GPIO21 a T_CS del display |
| Líneas horizontales | Interferencia SPI | Cables más cortos, agregar capacitor 100nF en VCC del display |

### 5.2 Problemas de CAN Bus

| Síntoma | Causa Probable | Solución |
|---------|----------------|----------|
| Sin comunicación CAN | Falta terminación 120Ω | Instalar resistencias en ambos extremos |
| Errores de bus frecuentes | Pin S (Silent) mal conectado | Conectar pin 8 del TJA1051 a GND |
| Bus-off state | Bit timing incorrecto | Verificar 500 kbps en ambos MCUs |
| Recepción OK, transmisión falla | TXD no conectado | Verificar GPIO4 → pin 1 del TJA1051 |
| Transmisión OK, recepción falla | RXD no conectado | Verificar GPIO5 → pin 4 del TJA1051 |
| Voltajes CAN incorrectos | TJA1051 sin alimentación 5V | Verificar pin 3 = 5V |
| Resistencia 120Ω medida | Terminación duplicada en un extremo | Quitar una resistencia |

---

## 6. ESPECIFICACIONES TÉCNICAS

### 6.1 ESP32-S3 DevKitC-1

| Parámetro | Valor |
|-----------|-------|
| **CPU** | Xtensa LX7 Dual-Core @ 240 MHz |
| **RAM** | 512 KB SRAM |
| **Flash** | 16 MB (N16R8) |
| **WiFi** | 802.11 b/g/n |
| **Bluetooth** | BLE 5.0 |
| **GPIOs** | 45 GPIOs disponibles |
| **SPI** | 4 controllers (SPI0/1 para Flash, SPI2/3 para periféricos) |
| **CAN (TWAI)** | 1 controller integrado |
| **Voltaje I/O** | 3.3V tolerante |

### 6.2 Display TFT ST7796

| Parámetro | Valor |
|-----------|-------|
| **Resolución** | 480×320 píxeles |
| **Driver** | ST7796 |
| **Interface** | SPI 4-wire |
| **Colores** | 262K (RGB 6-6-6) |
| **Voltaje** | 2.8V - 3.3V |
| **Corriente** | ~40mA @ 3.3V (sin backlight) |
| **Backlight** | ~100mA @ 3.3V (LED) |

### 6.3 TJA1051T/3

| Parámetro | Valor |
|-----------|-------|
| **Tipo** | High-Speed CAN Transceiver |
| **Voltaje** | 4.75V - 5.25V |
| **Velocidad** | Hasta 1 Mbps |
| **Corriente (activo)** | ~70 mA |
| **Corriente (standby)** | ~5 mA |
| **Niveles lógicos I/O** | Compatible 3.3V y 5V |
| **Temperatura** | -40°C a +125°C |

---

## 7. REFERENCIAS

### 7.1 Documentos del Proyecto

- `docs/ESP32_FIRMWARE_DESIGN.md` — Diseño del firmware ESP32
- `docs/ESP32_STM32_CAN_CONNECTION.md` — Conexión CAN detallada
- `docs/CAN_CONTRACT_FINAL.md` — Protocolo CAN rev 1.3
- `docs/HMI_RENDERING_STRATEGY.md` — Estrategia de renderizado HMI
- `esp32/platformio.ini` — Configuración de pines y build

### 7.2 Datasheets

- **ESP32-S3:** [Espressif ESP32-S3 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- **ST7796:** [ST7796 LCD Controller Datasheet](https://www.displayfuture.com/Display/datasheet/controller/ST7796s.pdf)
- **TJA1051:** [NXP TJA1051 High-Speed CAN Transceiver](https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf)
- **TFT_eSPI:** [Bodmer TFT_eSPI Library](https://github.com/Bodmer/TFT_eSPI)
- **ESP32-TWAI-CAN:** [ESP32 TWAI/CAN Library](https://github.com/handmade0octopus/ESP32-TWAI-CAN)

---

## 8. NOTAS IMPORTANTES

### ⚠️ Advertencias de Seguridad

1. **NUNCA conectar 5V directo a pines GPIO del ESP32** — Daño permanente
2. **NUNCA conectar CANH/CANL directo al ESP32** — Usar TJA1051 obligatoriamente
3. **Verificar polaridad VCC/GND** antes de encender
4. **GND común obligatorio** entre ESP32, display, y transceiver CAN

### ✅ Buenas Prácticas

1. Usar cables cortos (<20cm) para conexiones SPI del display
2. Instalar capacitor 100nF cerca de VCC de cada chip (display, TJA1051)
3. Usar par trenzado para CANH/CANL en instalaciones permanentes
4. Etiquetar cables durante el montaje
5. Documentar cualquier cambio en este archivo

---

## 9. HISTORIAL DE REVISIONES

| Versión | Fecha | Cambios |
|---------|-------|---------|
| 1.0 | 2026-02-19 | Creación inicial del documento con todos los pines display y CAN |

---

**Documento creado para:** florinzgz/STM32-Control-Coche-Marcos  
**Autor:** Sistema de documentación automática  
**Licencia:** MIT (ver LICENSE en el repositorio)
