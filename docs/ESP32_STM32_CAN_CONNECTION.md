# 🔌 Conexión CAN entre ESP32-S3 y STM32G474RE

## Resumen

Este documento describe la conexión física y de software entre el microcontrolador ESP32-S3 (HMI) y el STM32G474RE (Control) mediante el bus CAN con transreceptores TJA1051T/3.

## Arquitectura del Sistema

```
┌──────────────────┐                            ┌──────────────────┐
│   ESP32-S3       │    CAN Bus @ 500 kbps      │  STM32G474RE     │
│   (HMI)          │ ◄────────────────────────► │  (Control)       │
│                  │    TJA1051T/3 × 2          │                  │
└──────────────────┘                            └──────────────────┘
```

### Separación de Responsabilidades

| Componente | Función Principal | Responsabilidades |
|------------|------------------|-------------------|
| **ESP32-S3** | Interfaz Humano-Máquina (HMI) | • Display TFT ST7796S 480×320<br>• Touch XPT2046<br>• Audio DFPlayer Mini<br>• LEDs WS2812B<br>• Menús y diagnóstico<br>• Detección de obstáculos (visual/aviso) |
| **STM32G474RE** | Control Seguro | • Control de motores de tracción (×4)<br>• Control de motor de dirección<br>• Sensores críticos (corriente, temperatura)<br>• Encoder de dirección<br>• Lógica de seguridad (ABS/TCS)<br>• Relés de potencia |

## Hardware Requerido

### Transreceptores CAN

Se requieren **DOS (2) transreceptores TJA1051T/3**:

1. **TJA1051T/3 #1**: Conectado al STM32G474RE
2. **TJA1051T/3 #2**: Conectado al ESP32-S3

#### Especificaciones del TJA1051T/3

| Característica | Valor |
|----------------|-------|
| Estándar | ISO 11898-2 (High-Speed CAN) |
| Velocidad máxima | 1 Mbps |
| Alimentación | 5V (típico) |
| Lógica TXD/RXD | Compatible 3.3V y 5V |
| Temperatura | -40°C a +125°C |
| Protección ESD | ±8 kV (HBM) |
| Encapsulado | SO8 |

### Pinout del TJA1051T/3

```
        ┌─────────┐
   TXD  │1      8│  CANH
   GND  │2      7│  CANL
   VCC  │3      6│  Vref (NC)
   RXD  │4      5│  S (Standby)
        └─────────┘
```

## Conexiones

### STM32G474RE → TJA1051T/3 #1

| Pin STM32 | Señal | Pin TJA1051 | Función |
|-----------|-------|-------------|---------|
| **PB9** | FDCAN1_TX | Pin 1 (TXD) | Transmisión CAN |
| **PB8** | FDCAN1_RX | Pin 4 (RXD) | Recepción CAN |
| +5V | Alimentación | Pin 3 (VCC) | Alimentación transreceptor |
| GND | Tierra | Pin 2 (GND) | Tierra común |
| GND | Control | Pin 5 (S) | Modo normal (S=GND) |

### ESP32-S3 → TJA1051T/3 #2

| Pin ESP32 | Señal | Pin TJA1051 | Función |
|-----------|-------|-------------|---------|
| **GPIO20** | TWAI_TX | Pin 1 (TXD) | Transmisión CAN |
| **GPIO21** | TWAI_RX | Pin 4 (RXD) | Recepción CAN |
| +5V | Alimentación | Pin 3 (VCC) | Alimentación transreceptor |
| GND | Tierra | Pin 2 (GND) | Tierra común |
| GND | Control | Pin 5 (S) | Modo normal (S=GND) |

**Nota**: Los pines GPIO20 y GPIO21 del ESP32-S3 son configurables. Estos son los pines propuestos en el diseño.

### Bus CAN

| Señal | Conexión | Notas |
|-------|----------|-------|
| CANH | TJA1051#1 Pin 8 ↔ TJA1051#2 Pin 8 | Terminación 120Ω en ambos extremos |
| CANL | TJA1051#1 Pin 7 ↔ TJA1051#2 Pin 7 | Terminación 120Ω en ambos extremos |

#### Resistencias de Terminación

```
CANH ────────┬────────────────────────┬──────── CANH
             │                        │
            120Ω                     120Ω
             │                        │
CANL ────────┴────────────────────────┴──────── CANL
       (Extremo STM32)         (Extremo ESP32)
```

## Diagrama Completo de Conexión

```
                    SISTEMA DE COMUNICACIÓN CAN
                    ESP32-S3 (HMI) ↔ STM32G474RE (Control)

┌─────────────────────────────────────────────────────────────────────┐
│                           ESP32-S3 (HMI)                             │
│  • Display TFT, Touch, Audio, LEDs                                   │
│  • Menús y diagnóstico                                               │
│                                                                       │
│  GPIO 20 (TWAI_TX) ─────┐                                            │
│  GPIO 21 (TWAI_RX) ─────┤                                            │
└─────────────────────────┼────────────────────────────────────────────┘
                          │
                          ▼
                ┌──────────────────┐
                │   TJA1051T/3 #2  │
                │   Pin 8 (CANH)───┼──┐
                │   Pin 7 (CANL)───┼──┼──┐
                └──────────────────┘  │  │
                                     120Ω │
                  ┌───────────────────┴──┴──────────────┐
                  │        BUS CAN @ 500 kbps            │
                  │        Par trenzado 120Ω             │
                  └───────────────────┬──┬──────────────┘
                                     120Ω │
                ┌──────────────────┐  │  │
                │   TJA1051T/3 #1  │  │  │
                │   Pin 8 (CANH)───┼──┘  │
                │   Pin 7 (CANL)───┼─────┘
                └──────────────────┘
                          │
                          ▼
┌─────────────────────────┼────────────────────────────────────────────┐
│  PB9 (FDCAN1_TX) ───────┤                                            │
│  PB8 (FDCAN1_RX) ───────┘                                            │
│                                                                       │
│                      STM32G474RE (Control)                            │
│  • Control de motores, sensores críticos                             │
│  • Lógica de seguridad (ABS/TCS)                                     │
└───────────────────────────────────────────────────────────────────────┘
```

## Configuración de Software

### STM32G474RE (FDCAN1)

El STM32G474RE tiene soporte FDCAN integrado configurado para Classic CAN a 500 kbps:

- **Puerto**: FDCAN1
- **Modo**: Classic CAN (no CAN-FD)
- **Velocidad**: 500 kbps
- **Pines**: PB8 (RX), PB9 (TX)
- **Función alternativa**: AF9

Configuración en archivo `.ioc`:
- Ver `STM32G474RE-Control.ioc` para configuración completa de FDCAN1

### ESP32-S3 (TWAI)

El ESP32-S3 usa el controlador TWAI (Two-Wire Automotive Interface) compatible con CAN:

- **Controlador**: TWAI (CAN 2.0)
- **Velocidad**: 500 kbps
- **Pines propuestos**: GPIO20 (TX), GPIO21 (RX)

**Nota**: La implementación CAN en ESP32-S3 está documentada en el repositorio [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos).

## Protocolo de Comunicación

El protocolo CAN entre ESP32-S3 y STM32G474RE está definido en el documento `docs/CAN_PROTOCOL.md`.

### Mensajes Principales

| ID | Dirección | Nombre | Descripción |
|----|-----------|--------|-------------|
| 0x100 | STM32→ESP32 | Heartbeat STM32 | Señal de vida |
| 0x101 | ESP32→STM32 | Heartbeat ESP32 | Señal de vida |
| 0x200 | ESP32→STM32 | CMD Throttle | Comando acelerador |
| 0x201 | ESP32→STM32 | CMD Steering | Comando dirección |
| 0x300 | STM32→ESP32 | Status Speed | Velocidades ruedas |
| 0x301 | STM32→ESP32 | Status Current | Corrientes motores |
| 0x302 | STM32→ESP32 | Status Temp | Temperaturas |
| 0x303 | STM32→ESP32 | Status Safety | Estado ABS/TCS/Errores |

### Reglas de Autoridad

- **STM32 tiene autoridad final**: Puede rechazar comandos del ESP32
- **Heartbeat obligatorio**: Pérdida de heartbeat → modo seguro
- **Timeout**: 250 ms sin comunicación → inhibición de movimiento
- **ESP32 nunca controla potencia directamente**: Solo solicita acciones

## Referencias

### Documentación del Proyecto

- **Repositorio ESP32-S3**: [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos)
- **Plan de separación**: `docs/PLAN_SEPARACION_STM32_CAN.md` en repo ESP32
- **Manual transreceptores**: `docs/MANUAL_TRANSRECEPTORES_STM32_ESP32.md` en repo ESP32
- **Protocolo CAN**: `docs/CAN_PROTOCOL.md` (este repositorio)

### Datasheets

- [TJA1051T/3 Datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf)
- [STM32G474RE Datasheet](https://www.st.com/resource/en/datasheet/stm32g474re.pdf)
- [ESP32-S3 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)

## Notas de Implementación

1. **Cable CAN**: Usar par trenzado con impedancia característica de 120Ω
2. **Longitud máxima**: ~3-5 metros a 500 kbps
3. **Alimentación común**: Asegurar GND común entre todos los componentes
4. **Protección**: Los TJA1051T/3 incluyen protecciones EMI/ESD integradas
5. **Modo Standby**: Pin S del transreceptor puede usarse para modo seguro (desconectar bus)

## Verificación de la Instalación

### Checklist de Hardware

- [ ] TJA1051T/3 #1 instalado y conectado al STM32G474RE
- [ ] TJA1051T/3 #2 instalado y conectado al ESP32-S3
- [ ] Bus CAN conectado (CANH-CANH, CANL-CANL)
- [ ] Resistencias de terminación 120Ω instaladas en ambos extremos
- [ ] Alimentación 5V para ambos transreceptores
- [ ] GND común conectado entre todos los componentes

### Checklist de Software

- [ ] FDCAN1 configurado en STM32G474RE (PB8/PB9)
- [ ] TWAI configurado en ESP32-S3 (GPIO20/GPIO21)
- [ ] Velocidad configurada a 500 kbps en ambos lados
- [ ] Heartbeat implementado en ambos firmwares
- [ ] Timeout de 250 ms configurado
- [ ] Protocolo CAN implementado según `docs/CAN_PROTOCOL.md`

## Autor

**Florin Zgureanu** (@florinzgz)

## Licencia

MIT License
