# ESP32-S3 to STM32G474RE CAN Physical Connection Guide

## Overview

This document describes the hardware setup required to establish CAN communication between the ESP32-S3 (HMI controller) and STM32G474RE (vehicle control unit).

## Hardware Requirements

### CAN Transceivers
- **Quantity**: 2× TJA1051T/3 High-Speed CAN transceivers (NXP/Nexperia)
- **Purpose**: Convert MCU digital signals to differential CAN bus signals
- **VCC (Power Supply)**: **5 V** (4.5–5.5 V según datasheet NXP)
- **VIO (I/O Level)**: 2.8–5.5 V — conectar a la tensión del MCU (3.3 V)
- **Bus Speed**: Up to 1 Mbps (using 500 kbps in this application)

> ⚠️ **IMPORTANTE — TJA1051T/3 vs TJA1051T/4:**
> - El sufijo **/3** indica que el chip tiene un pin **VIO** (pin 5) que permite adaptar los niveles lógicos de TXD y RXD a la tensión del MCU (3.3 V en este proyecto).
> - El sufijo **/4** **NO** tiene pin VIO — los niveles lógicos siguen VCC (5 V). **No usar TJA1051T/4** con el ESP32-S3 (que NO es tolerante a 5 V) a menos que se añadan level shifters externos.
> - **VCC siempre debe ser 5 V** (4.5–5.5 V) para ambas variantes. El TJA1051T/3 NO se alimenta de 3.3 V.

### Termination Resistors
- **Quantity**: 2× 120Ω resistors (1/4W minimum)
- **Placement**: One at each end of the CAN bus
- **Purpose**: Prevents signal reflections and ensures proper bus operation

## Pin Connections — TJA1051T/3 (8 pins, SOT96-1 / SO8)

| Pin | Nombre | Función |
|-----|--------|---------|
| 1 | TXD | Entrada de datos TX desde el MCU |
| 2 | GND | Masa |
| 3 | VCC | Alimentación **5 V** (4.5–5.5 V) |
| 4 | RXD | Salida de datos RX hacia el MCU |
| 5 | **VIO** | **Tensión de referencia I/O** — conectar a la alimentación del MCU |
| 6 | CANL | CAN Low (bus diferencial) |
| 7 | CANH | CAN High (bus diferencial) |
| 8 | S | Silent mode — **conectar a GND** para modo normal |

### STM32G474RE Side

| STM32 Pin | Function | TJA1051T/3 Pin | Description |
|-----------|----------|----------------|-------------|
| PA12 (AF9) | FDCAN1_TX | Pin 1 (TXD) | Transmit data to transceiver |
| PA11 (AF9) | FDCAN1_RX | Pin 4 (RXD) | Receive data from transceiver |
| +5V (ext.) | Power | Pin 3 (VCC) | **5 V** — desde DC-DC buck (LM2596) |
| +3.3V | I/O Level | Pin 5 (VIO) | **3.3 V** — nivel lógico I/O del STM32 |
| GND | Ground | Pin 2 (GND) | Common ground reference |
| GND | Normal mode | Pin 8 (S) | **Connect to GND** — enables normal mode |

> **Nota:** PA11/PA12 del STM32G474RE son pines FT (5V-tolerant), por lo que incluso si VIO se conectase a 5 V por error, los pines del STM32 **no se dañarían**. Sin embargo, para el correcto funcionamiento del bus, VIO debe coincidir con la tensión lógica del MCU (3.3 V).

### Ubicación física en la placa Nucleo-64 (MB1367)

Los pines PA11 y PA12 están accesibles en el **conector morpho CN10** (conector derecho de la
placa Nucleo-64, visto con el USB arriba). Consultar **Tabla 17** del manual de usuario
**UM2505** (STMicroelectronics) para el mapeado completo.

| Señal | Conector Nucleo | Pin del conector | Notas |
|-------|-----------------|------------------|-------|
| PA12 (FDCAN1_TX) | **CN10** | **Pin 12** | AF9 — Transmisión CAN |
| PA11 (FDCAN1_RX) | **CN10** | **Pin 14** | AF9 — Recepción CAN |
| GND | **CN10** | **Pin 20** | Masa común — usar para GND del TJA1051T/3 |
| 5V | **Fuente ext.** | — | 5 V DC-DC (LM2596) → VCC del TJA1051T/3 |
| 3V3 | **CN10** | **Pin 7** | 3.3 V regulados → VIO del TJA1051T/3 |

```
    Conector CN10 (vista frontal, USB arriba, lado derecho de la placa)

    Pin 11  [ ][ ]  Pin 12 [PA12]   ← FDCAN1_TX  → TJA1051T/3 pin 1 (TXD)
    Pin 13  [ ][ ]  Pin 14 [PA11]   ← FDCAN1_RX  → TJA1051T/3 pin 4 (RXD)
      ...
    Pin 19  [ ][ ]  Pin 20 [GND]    ← GND         → TJA1051T/3 pins 2, 8
```

### ESP32-S3 Side

| ESP32 Pin | Function | TJA1051T/3 Pin | Description |
|-----------|----------|----------------|-------------|
| GPIO4 | CAN_TX | Pin 1 (TXD) | Transmit data to transceiver |
| GPIO5 | CAN_RX | Pin 4 (RXD) | Receive data from transceiver |
| +5V (ext.) | Power | Pin 3 (VCC) | **5 V** — fuente externa regulada |
| **+3.3V** | **I/O Level** | **Pin 5 (VIO)** | **3.3 V — OBLIGATORIO para proteger el ESP32-S3** |
| GND | Ground | Pin 2 (GND) | Common ground reference |
| GND | Normal mode | Pin 8 (S) | **Connect to GND** — enables normal mode |

> ⚠️ **CRÍTICO — Pin 5 (VIO) en el lado ESP32-S3:**
> El ESP32-S3 **NO es tolerante a 5 V** en sus GPIO (máx. absoluto 3.6 V). Si el pin VIO del TJA1051T/3 se deja flotante o se conecta a 5 V, el pin RXD (pin 4) producirá niveles de 5 V que **destruirán el GPIO5 del ESP32-S3**. Conectar VIO a **3.3 V** es **obligatorio**.

## Pin 8 (S) — Silent Mode / Standby Pin

El pin 8 (S) del TJA1051T/3 controla el modo de operación del transceiver.

| Pin 8 Level | Mode | Effect |
|-------------|------|--------|
| LOW (GND) | **Normal mode** ✅ | Transceiver activo — **usar este modo** |
| HIGH (VCC) | Silent mode | Solo escucha, no transmite |
| Floating | Undefined | **No dejar flotante** — conectar a GND |

**Required action**: Connect pin 8 (S) to GND on **both** TJA1051T/3 modules.

## CAN Bus Wiring

### Differential Pair

Connect the two transceivers together using twisted pair wire (recommended):

| Signal | TJA1051T/3 Pin | Wire Color (Suggested) | Notes |
|--------|----------------|------------------------|-------|
| CANH | Pin 7 | Orange/White | CAN High signal |
| CANL | Pin 6 | Orange | CAN Low signal |

**Important**: Use twisted pair cable to minimize EMI. Maximum length at 500 kbps: ~40 meters.

### Termination

Install one 120Ω resistor **at each end** of the bus, connecting CANH to CANL locally:

```
STM32 End:                                    ESP32 End:
CANH ──┬──────────────────────────────────┬── CANH
       │                                  │
     [120Ω]                             [120Ω]
       │                                  │
CANL ──┴──────────────────────────────────┴── CANL
```

**Important**: Each termination resistor connects CANH to CANL at its own end of the bus.
The combined parallel resistance (both ends powered) should measure ~60Ω across CANH–CANL.

## Schematic Diagram

```
STM32G474RE        TJA1051T/3 #1 (STM32 side)   Bus (Twisted Pair)   TJA1051T/3 #2 (ESP32 side)     ESP32-S3
   PA12 TX ──→   TXD [1]                                                   TXD [1]  ←── GPIO4 TX
   PA11 RX ←──   RXD [4]  [7] CANH ──┬──────────────────────┬── CANH [7]   RXD [4]  ──→ GPIO5 RX
     +5V   ──→   VCC [3]              │                      │              VCC [3]  ←── +5V (ext.)
    3.3V   ──→   VIO [5]           [120Ω]                 [120Ω]            VIO [5]  ←── 3.3V  ⚠️ OBLIGATORIO
      GND  ──→   GND [2]  [6] CANL ──┴──────────────────────┴── CANL [6]   GND [2]  ←── GND
      GND  ──→     S [8]                                                      S [8]  ←── GND
                          │                                          │
                        100nF (VCC-GND)                            100nF (VCC-GND)
                          │                                          │
                         GND                                        GND
```

Note: The 120Ω resistors connect CANH to CANL **locally at each end** of the bus.
The 100nF decoupling capacitors are placed between VCC and GND, close to each transceiver.

## Power Supply Considerations

- **VCC = 5 V** (4.5–5.5 V) — ambos transceivers se alimentan desde el rail de 5 V del DC-DC buck (LM2596, 24 V → 5 V). **NO alimentar VCC desde 3.3 V** — el transceiver no funcionará.
- **VIO = 3.3 V** — conectar al rail de 3.3 V del MCU correspondiente. Esto establece los niveles lógicos de TXD/RXD a 3.3 V, compatibles con ESP32-S3 y STM32G474RE.
- **Common GND is mandatory**: The GND of the ESP32 system, the STM32 system, and both TJA1051T/3 transceivers must all share a common ground reference. Without common GND, the differential CAN signals have no stable reference and communication will fail or be unreliable.
- Maximum current draw per TJA1051T/3: ~70 mA (typical ~10 mA active)
- Use decoupling capacitors (100 nF ceramic) close to VCC pin of each transceiver

## Testing the Connection

### Step 1: Visual Inspection
1. Verify all wiring matches the pin connections table
2. Check termination resistors are installed at both ends
3. Measure resistance between CANH and CANL: should be ~60Ω (two 120Ω in parallel)
4. **Verificar con multímetro ANTES de conectar los MCUs:**
   - VCC (pin 3) → GND: **5.0 V ± 0.2 V**
   - VIO (pin 5) → GND: **3.3 V ± 0.1 V**
   - Si VIO ≠ 3.3 V, **NO conectar el ESP32-S3** — riesgo de destrucción del GPIO

### Step 2: Voltage Levels
With both systems powered but not transmitting:
- CANH voltage: ~2.5 V (referido a GND)
- CANL voltage: ~2.5 V (referido a GND)
- Differential voltage (CANH - CANL): ~0 V

During transmission (use oscilloscope):
- Dominant bit: CANH ~3.5 V, CANL ~1.5 V (difference ~2.0 V)
- Recessive bit: CANH ~2.5 V, CANL ~2.5 V (difference ~0 V)

### Step 3: Software Test
1. Flash both MCUs with test firmware
2. Monitor CAN traffic using logic analyzer or CAN bus analyzer
3. Verify heartbeat messages exchanged at 10 Hz (100ms interval)
4. Check bit timing: 500 kbps = 2μs per bit

## Troubleshooting

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| No communication | Missing termination | Install 120Ω resistors at both ends |
| Intermittent errors | Poor grounding | Ensure solid GND connection between systems |
| Bus-off state | Incorrect bit timing | Verify FDCAN prescaler settings (see CAN_PROTOCOL.md) |
| High error rate | Wire too long | Reduce cable length or lower bitrate |
| No activity | Silent mode active | Connect TJA1051T/3 pin 8 (S) to GND |
| No activity | VCC not 5 V | Medir VCC con multímetro — debe ser 4.5–5.5 V |
| ESP32 GPIO damaged | VIO not connected to 3.3V | ⚠️ Si VIO está flotante o a 5 V, RXD será 5 V → destruye ESP32 |

## Voltage Safety

> ⚠️ **PELIGRO — Resumen de tensiones del TJA1051T/3:**
>
> | Pin | Tensión correcta | Error peligroso | Consecuencia |
> |-----|------------------|-----------------|--------------|
> | VCC (pin 3) | **5 V** (4.5–5.5 V) | Conectar 3.3 V | Transceiver no funciona |
> | VIO (pin 5) | **3.3 V** (= MCU supply) | Dejar flotante o conectar 5 V | RXD a 5 V → **destruye GPIO del ESP32-S3** |
> | S (pin 8) | **GND** | Dejar flotante | Modo indefinido, CAN no funciona |

## Safety Notes

⚠️ **Critical**: This is a safety-critical vehicle control system.

- Always test CAN communication on the bench before vehicle installation
- Use automotive-grade components for production deployment
- Implement watchdog timeout for CAN communication loss
- Ensure EMI shielding in production wiring harness
- Follow automotive wiring standards (e.g., ISO 11898)
- **Verificar VIO = 3.3 V con multímetro antes de conectar MCUs**

## References

- [TJA1051T/3 Datasheet (NXP)](https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf)
- ISO 11898 Road vehicles — Controller area network (CAN)
- STM32G4 FDCAN Configuration: See `CAN_PROTOCOL.md`
- ESP32-S3 TWAI Configuration: See ESP32 firmware repository
- **UM2505** — STM32G4 Nucleo-64 boards (MB1367) User Manual, STMicroelectronics.
  Tabla 17: *«Pin assignment of the ST morpho connectors»* — mapeo de PA12/PA11 a CN10 pin 12/14.
  Disponible en: https://www.st.com/resource/en/user_manual/um2505-stm32g4-nucleo64-boards-mb1367-stmicroelectronics.pdf
