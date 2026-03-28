# ESP32-S3 to STM32G474RE CAN Physical Connection Guide

## Overview

This document describes the hardware setup required to establish CAN communication between the ESP32-S3 (HMI controller) and STM32G474RE (vehicle control unit).

## Hardware Requirements

### CAN Transceivers
- **Quantity**: 2× SN65HVD230 High-Speed CAN transceivers (Texas Instruments)
- **Purpose**: Convert MCU digital signals to differential CAN bus signals
- **Operating Voltage**: 3.3V (absolute max 4.0V)
- **Bus Speed**: Up to 1 Mbps (using 500 kbps in this application)

### Termination Resistors
- **Quantity**: 2× 120Ω resistors (1/4W minimum)
- **Placement**: One at each end of the CAN bus
- **Purpose**: Prevents signal reflections and ensures proper bus operation

## Pin Connections

### STM32G474RE Side

| STM32 Pin | Function | SN65HVD230 Pin | Description |
|-----------|----------|----------------|-------------|
| PA11 (AF9) | FDCAN1_RX | Pin 4 (R) | Receive data from transceiver |
| PA12 (AF9) | FDCAN1_TX | Pin 1 (D) | Transmit data to transceiver |
| +3.3V | Power | Pin 3 (VCC) | Transceiver power supply |
| GND | Ground | Pin 2 (GND) | Common ground reference |
| GND | Slope/Standby | Pin 8 (Rs) | **Connect to GND** — enables high-speed mode |

### Ubicación física en la placa Nucleo-64 (MB1367)

Los pines PA11 y PA12 están accesibles en el **conector morpho CN10** (conector derecho de la
placa Nucleo-64, visto con el USB arriba). Consultar **Tabla 17** del manual de usuario
**UM2505** (STMicroelectronics) para el mapeado completo.

| Señal | Conector Nucleo | Pin del conector | Notas |
|-------|-----------------|------------------|-------|
| PA12 (FDCAN1_TX) | **CN10** | **Pin 12** | AF9 — Transmisión CAN |
| PA11 (FDCAN1_RX) | **CN10** | **Pin 14** | AF9 — Recepción CAN |
| GND | **CN10** | **Pin 20** | Masa común — usar para GND del SN65HVD230 |
| 3V3 | **CN10** | **Pin 7** | 3.3 V regulados — usar para VCC del SN65HVD230 |

```
    Conector CN10 (vista frontal, USB arriba, lado derecho de la placa)

    Pin 11  [ ][ ]  Pin 12 [PA12]   ← FDCAN1_TX  → SN65HVD230 pin 1 (D)
    Pin 13  [ ][ ]  Pin 14 [PA11]   ← FDCAN1_RX  → SN65HVD230 pin 4 (R)
      ...
    Pin 19  [ ][ ]  Pin 20 [GND]    ← GND         → SN65HVD230 pins 2, 8
```

### ESP32-S3 Side

| ESP32 Pin | Function | SN65HVD230 Pin | Description |
|-----------|----------|----------------|-------------|
| GPIO4 | CAN_TX | Pin 1 (D) | Transmit data to transceiver |
| GPIO5 | CAN_RX | Pin 4 (R) | Receive data from transceiver |
| +3.3V | Power | Pin 3 (VCC) | Transceiver power supply |
| GND | Ground | Pin 2 (GND) | Common ground reference |
| GND | Slope/Standby | Pin 8 (Rs) | **Connect to GND** — enables high-speed mode |

## Pin 8 (Rs) — Slope Control / Standby Pin

The SN65HVD230 pin 8 (Rs) controls the slope of the CAN output transitions and
can place the transceiver in standby mode.

| Label on module | TI datasheet name | Meaning |
|-----------------|-------------------|---------|
| Rs | Rs (Slope control) | SN65HVD230 official name |
| SLOPE | SLOPE | Alternative label on some breakout boards |

**For the SN65HVD230 used in this project:**

| Pin 8 Level | Mode | Effect |
|-------------|------|--------|
| LOW (GND) | **High-speed mode** ✅ | Maximum slew rate, full-speed operation — **use this** |
| HIGH (VCC) | Standby mode | Low-power standby, TX and RX disabled |
| Resistor to GND (10–100 kΩ) | Slope-control mode | Reduced slew rate for lower EMI |
| Floating | Undefined | **Do not leave floating** — connect to GND |

**Required action**: Connect pin 8 (Rs) to GND on **both** SN65HVD230 modules.

> ⚠️ If your module labels the pin differently, verify the part number printed
> on the IC. The SN65HVD230 (TI) uses GND for high-speed mode and VCC for standby.
> Connecting to GND is correct for normal operation.

## CAN Bus Wiring

### Differential Pair

Connect the two transceivers together using twisted pair wire (recommended):

| Signal | SN65HVD230 Pin | Wire Color (Suggested) | Notes |
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
STM32G474RE        SN65HVD230 #1 (STM32 side)   Bus (Twisted Pair)   SN65HVD230 #2 (ESP32 side)   ESP32-S3
   PA12 TX ──→    D [1]                                                    D [1]  ←── GPIO4 TX
   PA11 RX ←──    R [4]  [7] CANH ──┬──────────────────────┬── CANH [7]    R [4]  ──→ GPIO5 RX
      3.3V ──→  VCC [3]              │                      │             VCC [3]  ←── 3.3V
       GND ──→  GND [2]           [120Ω]                 [120Ω]          GND [2]  ←── GND
       GND ──→   Rs [8]  [6] CANL ──┴──────────────────────┴── CANL [6]   Rs [8]  ←── GND
                          │                                      │
                        100nF                                  100nF
                          │                                      │
                         GND                                    GND
```

Note: The 120Ω resistors connect CANH to CANL **locally at each end** of the bus.
The 100nF decoupling capacitors are placed between VCC and GND, close to each transceiver.

## Power Supply Considerations

- Both transceivers require regulated 3.3V supply
- **Common GND is mandatory**: The GND of the ESP32 system, the STM32 system, and both SN65HVD230 transceivers must all share a common ground reference. Without common GND, the differential CAN signals have no stable reference and communication will fail or be unreliable.
- Maximum current draw per SN65HVD230: ~70mA (typical ~10mA active)
- Use decoupling capacitors (100nF ceramic) close to VCC pin of each transceiver

## Testing the Connection

### Step 1: Visual Inspection
1. Verify all wiring matches the pin connections table
2. Check termination resistors are installed at both ends
3. Measure resistance between CANH and CANL: should be ~60Ω (two 120Ω in parallel)

### Step 2: Voltage Levels
With both systems powered but not transmitting:
- CANH voltage: ~1.65V
- CANL voltage: ~1.65V
- Differential voltage (CANH - CANL): ~0V

During transmission (use oscilloscope):
- Dominant bit: CANH ~2.75V, CANL ~0.55V (difference ~2.2V)
- Recessive bit: CANH ~1.65V, CANL ~1.65V (difference ~0V)

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
| No activity | Standby mode active | Connect SN65HVD230 pin 8 (Rs) to GND |

## Voltage Safety

The SN65HVD230 operates at 3.3 V supply — matching both the ESP32-S3 and STM32G474RE GPIO levels. No voltage dividers or level shifters are needed. **Do NOT apply 5 V to VCC** — the absolute maximum rating is 4.0 V.

## Safety Notes

⚠️ **Critical**: This is a safety-critical vehicle control system.

- Always test CAN communication on the bench before vehicle installation
- Use automotive-grade components for production deployment
- Implement watchdog timeout for CAN communication loss
- Ensure EMI shielding in production wiring harness
- Follow automotive wiring standards (e.g., ISO 11898)

## References

- SN65HVD230 Datasheet: Texas Instruments
- ISO 11898 Road vehicles — Controller area network (CAN)
- STM32G4 FDCAN Configuration: See `CAN_PROTOCOL.md`
- ESP32-S3 TWAI Configuration: See ESP32 firmware repository
- **UM2505** — STM32G4 Nucleo-64 boards (MB1367) User Manual, STMicroelectronics.
  Tabla 17: *«Pin assignment of the ST morpho connectors»* — mapeo de PA11/PA12 a CN10 pin 14/12.
  Disponible en: https://www.st.com/resource/en/user_manual/um2505-stm32g4-nucleo64-boards-mb1367-stmicroelectronics.pdf
