# ESP32-S3 to STM32G474RE CAN Physical Connection Guide

## Overview

This document describes the hardware setup required to establish CAN communication between the ESP32-S3 (HMI controller) and STM32G474RE (vehicle control unit).

## Hardware Requirements

### CAN Transceivers
- **Quantity**: 2× TJA1051T/3 High-Speed CAN transceivers
- **Purpose**: Convert MCU digital signals to differential CAN bus signals
- **Operating Voltage**: 4.75V - 5.25V typical
- **Bus Speed**: Up to 1 Mbps (using 500 kbps in this application)

### Termination Resistors
- **Quantity**: 2× 120Ω resistors (1/4W minimum)
- **Placement**: One at each end of the CAN bus
- **Purpose**: Prevents signal reflections and ensures proper bus operation

## Pin Connections

### STM32G474RE Side

| STM32 Pin | Function | TJA1051 Pin | Description |
|-----------|----------|-------------|-------------|
| PB8 | FDCAN1_RX | Pin 4 (RXD) | Receive data from transceiver |
| PB9 | FDCAN1_TX | Pin 1 (TXD) | Transmit data to transceiver |
| +5V | Power | Pin 3 (VCC) | Transceiver power supply |
| GND | Ground | Pin 2 (GND) | Common ground reference |
| GND | Silent/Normal | Pin 8 (S/SLNT) | **Connect to GND** — enables normal (TX+RX) mode |

### ESP32-S3 Side

| ESP32 Pin | Function | TJA1051 Pin | Description |
|-----------|----------|-------------|-------------|
| GPIO4 | CAN_TX | Pin 1 (TXD) | Transmit data to transceiver |
| GPIO5 | CAN_RX | Pin 4 (RXD) | Receive data from transceiver |
| +5V | Power | Pin 3 (VCC) | Transceiver power supply |
| GND | Ground | Pin 2 (GND) | Common ground reference |
| GND | Silent/Normal | Pin 8 (S/SLNT) | **Connect to GND** — enables normal (TX+RX) mode |

## Pin 8 (S / SLNT / STB) — Silent Mode Pin

The TJA1051T/3 pin 8 controls whether the transceiver can transmit on the bus.
Different module labels and datasheet revisions use different names for this pin:

| Label on module | NXP datasheet name | Meaning |
|-----------------|--------------------|---------|
| S | S (Silent) | TJA1051T/3 official name |
| SLNT | SLNT (Silent) | Alternative label on some breakout boards |
| STB | STB (Standby) | Name used on TJA1050 and other TJA105x variants — connect to GND for normal mode |

**For the TJA1051T/3 used in this project:**

| Pin 8 Level | Mode | Effect |
|-------------|------|--------|
| LOW (GND) | **Normal mode** ✅ | Both TX and RX active — **use this** |
| HIGH (VCC) | Silent mode | RX only, TX disabled — bus-off from the MCU perspective |
| Floating | Undefined | **Do not leave floating** — connect to GND |

**Required action**: Connect pin 8 (S/SLNT) to GND on **both** TJA1051 modules.

> ⚠️ If your module labels the pin "STB" instead of "S" or "SLNT", verify the part number printed
> on the IC. The TJA1051T/3 (NXP) uses active-LOW normal mode. The TJA1050 also uses active-LOW.
> Connecting to GND is correct for both parts.

## CAN Bus Wiring

### Differential Pair

Connect the two transceivers together using twisted pair wire (recommended):

| Signal | TJA1051 Pin | Wire Color (Suggested) | Notes |
|--------|-------------|------------------------|-------|
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
STM32G474RE        TJA1051 #1 (STM32 side)    Bus (Twisted Pair)   TJA1051 #2 (ESP32 side)   ESP32-S3
   PB9 TX ──→  TXD [1]                                                  TXD [1]  ←── GPIO4 TX
   PB8 RX ←──  RXD [4]  [7] CANH ──┬──────────────────────┬── CANH [7]  RXD [4]  ──→ GPIO5 RX
      +5V ──→  VCC [3]              │                      │             VCC [3]  ←── +5V
      GND ──→  GND [2]           [120Ω]                 [120Ω]          GND [2]  ←── GND
      GND ──→   S  [8]  [6] CANL ──┴──────────────────────┴── CANL [6]   S  [8]  ←── GND
                          │                                      │
                        100nF                                  100nF
                          │                                      │
                         GND                                    GND
```

Note: The 120Ω resistors connect CANH to CANL **locally at each end** of the bus.
The 100nF decoupling capacitors are placed between VCC and GND, close to each transceiver.

## Power Supply Considerations

- Both transceivers require regulated 5V supply
- **Common GND is mandatory**: The GND of the ESP32 system, the STM32 system, and both TJA1051 transceivers must all share a common ground reference. Without common GND, the differential CAN signals have no stable reference and communication will fail or be unreliable.
- Maximum current draw per TJA1051: 70mA @ 5V (typical 5mA standby)
- Use decoupling capacitors (100nF ceramic) close to VCC pin of each transceiver

## Testing the Connection

### Step 1: Visual Inspection
1. Verify all wiring matches the pin connections table
2. Check termination resistors are installed at both ends
3. Measure resistance between CANH and CANL: should be ~60Ω (two 120Ω in parallel)

### Step 2: Voltage Levels
With both systems powered but not transmitting:
- CANH voltage: ~2.5V
- CANL voltage: ~2.5V
- Differential voltage (CANH - CANL): ~0V

During transmission (use oscilloscope):
- Dominant bit: CANH ~3.5V, CANL ~1.5V (difference ~2V)
- Recessive bit: CANH ~2.5V, CANL ~2.5V (difference ~0V)

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
| No activity | Silent mode active | Connect TJA1051 pin 8 (S) to GND |

## Safety Notes

⚠️ **Critical**: This is a safety-critical vehicle control system.

- Always test CAN communication on the bench before vehicle installation
- Use automotive-grade components for production deployment
- Implement watchdog timeout for CAN communication loss
- Ensure EMI shielding in production wiring harness
- Follow automotive wiring standards (e.g., ISO 11898)

## References

- TJA1051T/3 Datasheet: NXP Semiconductors
- ISO 11898 Road vehicles — Controller area network (CAN)
- STM32G4 FDCAN Configuration: See `CAN_PROTOCOL.md`
- ESP32-S3 TWAI Configuration: See ESP32 firmware repository
