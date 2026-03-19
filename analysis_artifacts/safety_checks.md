# Safety Checks & Verification Procedures

> **Project:** STM32 + ESP32-S3 Vehicle Control System  
> **Source of truth:** `Core/Inc/main.h`, `esp32/include/*.h`, `esp32/src/*.h`  
> **Generated:** 2026-03-19 | **Branch:** `analysis/wiring-docs`

---

## Table of Contents

1. [Pre-Energization Visual Inspection](#1-pre-energization-visual-inspection)
2. [Continuity Tests (No Power)](#2-continuity-tests-no-power)
3. [Power Rail Tests (First Power-Up)](#3-power-rail-tests-first-power-up)
4. [CAN Bus Verification](#4-can-bus-verification)
5. [Motor Driver Tests](#5-motor-driver-tests)
6. [Sensor Verification](#6-sensor-verification)
7. [Relay Tests](#7-relay-tests)
8. [ESP32-S3 Peripheral Tests](#8-esp32-s3-peripheral-tests)
9. [Emergency Disconnect Procedure](#9-emergency-disconnect-procedure)
10. [What NOT To Do](#10-what-not-to-do)

---

## 1. Pre-Energization Visual Inspection

**Equipment needed:** Visual inspection only, magnifying glass recommended.

| # | Check Item | Pass Criteria | ☐ |
|---|------------|---------------|---|
| V-01 | Battery polarity (24V) | Red wire to (+), black to (−); reversed polarity diode installed | ☐ |
| V-02 | Main fuse (F1, 60A) present | Fuse inserted in holder; correct rating visible | ☐ |
| V-03 | Per-motor fuses (F2, 30A ×4) present | All 4 fuses inserted and rated correctly | ☐ |
| V-04 | Steering fuse (F3, 20A) present | Fuse inserted, correct rating | ☐ |
| V-05 | All relay flyback diodes installed | 1N4007 across each relay coil; cathode to +12V/+5V | ☐ |
| V-06 | CAN termination resistors | 120 Ω at STM32 node AND at ESP32 node | ☐ |
| V-07 | I2C pull-ups installed | 4.7 kΩ on SCL (PB6) and SDA (PB7) to 3.3V | ☐ |
| V-08 | Pedal ADC divider installed | 10 kΩ + 6.8 kΩ divider on PA3; check orientation | ☐ |
| V-09 | ADC protection diode | BAT54S clamp diode between PA3 and rails | ☐ |
| V-10 | Encoder optocouplers (6N137) | 3× 6N137 for ENC_A/B/Z signals; powered | ☐ |
| V-11 | Wheel sensor optocouplers (PC817) | 4× PC817 for wheel speed signals | ☐ |
| V-12 | No bare wires or solder bridges | All connections insulated; no shorts visible | ☐ |
| V-13 | Power and signal cables separated | ≥ 5 cm separation; no parallel runs > 30 cm | ☐ |
| V-14 | Ground bus connected | All modules share common ground; star topology preferred | ☐ |
| V-15 | BTS7960 EN pins correct | FL: PC5, RR: PC13 (GPIO); FR/RL/STEER: tied to 3.3V | ☐ |
| V-16 | 10 kΩ pull-down on relay MOSFETs | Each relay MOSFET gate has pull-down to prevent float at boot | ☐ |
| V-17 | Decoupling capacitors on MCU | 100 nF per VDD pin + 10 µF bulk near STM32 | ☐ |
| V-18 | LED data resistors | 330 Ω series on GPIO47 and GPIO48 data lines | ☐ |
| V-19 | LED bulk caps | 1000 µF/10V at each LED strip power input | ☐ |
| V-20 | OneWire pull-up | 4.7 kΩ on PB0 to 3.3V for DS18B20 bus | ☐ |

---

## 2. Continuity Tests (No Power)

**Equipment:** Digital multimeter (continuity/resistance mode).  
**⚠ Battery DISCONNECTED for all tests in this section.**

| # | Test | Probe (+) | Probe (−) | Expected | Fail Action | ☐ |
|---|------|-----------|-----------|----------|-------------|---|
| C-01 | Battery to main fuse | Battery (+) | Fuse input | < 0.5 Ω | Check cable/connection | ☐ |
| C-02 | Fuse to RELAY_MAIN coil | Fuse output | Relay common | < 0.5 Ω | Check wiring | ☐ |
| C-03 | Ground bus continuity | Battery (−) | STM32 GND | < 0.5 Ω | Check ground path | ☐ |
| C-04 | Ground bus to ESP32 | Battery (−) | ESP32 GND | < 0.5 Ω | Check ground path | ☐ |
| C-05 | CAN termination (STM32) | CANH | CANL | 120 Ω ±5% | Install/replace termination R | ☐ |
| C-06 | CAN termination (ESP32) | CANH | CANL | 120 Ω ±5% | Install/replace termination R | ☐ |
| C-07 | CAN bus end-to-end | CANH (STM32) | CANH (ESP32) | 60 Ω (two 120 Ω in parallel) | Check cable or terminators | ☐ |
| C-08 | I2C SCL pull-up | PB6 | 3.3V rail | 4.7 kΩ ±10% | Install pull-up | ☐ |
| C-09 | I2C SDA pull-up | PB7 | 3.3V rail | 4.7 kΩ ±10% | Install pull-up | ☐ |
| C-10 | Pedal divider R1 | PA3 side of R1 | 5V side of R1 | 10 kΩ ±5% | Check resistor | ☐ |
| C-11 | Pedal divider R2 | PA3 | GND | 6.8 kΩ ±5% | Check resistor | ☐ |
| C-12 | Motor FL wiring | BTS1 M+ | Motor FL terminal 1 | < 1 Ω | Check motor cable | ☐ |
| C-13 | Motor FL no-short | Motor FL terminal 1 | Motor FL terminal 2 | > 0.5 Ω (motor winding) | Motor shorted internally | ☐ |
| C-14 | Relay flyback D1 | RELAY_MAIN coil (+) | RELAY_MAIN coil (−) | Diode forward ~0.6V | Diode reversed/missing | ☐ |
| C-15 | No power-to-signal short | 24V bus | Any STM32 GPIO | Open (∞) | CRITICAL: Find short | ☐ |
| C-16 | OneWire bus pull-up | PB0 | 3.3V rail | 4.7 kΩ ±10% | Install pull-up | ☐ |
| C-17 | Each BTS7960 B+ to fuse | BTS B+ | Fuse output | < 0.5 Ω (through relay contact) | Check fuse/relay path | ☐ |
| C-18 | ESP32 I2C pull-up SCL | GPIO9 | 3.3V | 4.7 kΩ ±10% | Install pull-up for MCP23017 | ☐ |
| C-19 | ESP32 I2C pull-up SDA | GPIO8 | 3.3V | 4.7 kΩ ±10% | Install pull-up for MCP23017 | ☐ |

---

## 3. Power Rail Tests (First Power-Up)

**Equipment:** Digital multimeter (DC voltage mode), current-limited power supply recommended.  
**Procedure:** Connect battery. Do NOT connect motors yet (leave BTS7960 M+/M− disconnected).

| # | Test | Measure Point | Expected | Tolerance | Fail Action | ☐ |
|---|------|---------------|----------|-----------|-------------|---|
| P-01 | Battery voltage | Battery terminals | 24.0–28.8V | ±0.5V | Check battery charge | ☐ |
| P-02 | 12V rail | 12V regulator output | 12.0V | ±0.5V | Check regulator | ☐ |
| P-03 | 5V rail | 5V regulator output | 5.0V | ±0.25V | Check regulator | ☐ |
| P-04 | 3.3V rail (STM32) | Nucleo 3.3V pin | 3.3V | ±0.1V | Nucleo regulator fault | ☐ |
| P-05 | 3.3V rail (ESP32) | ESP32 3V3 pin | 3.3V | ±0.1V | DevKit regulator fault | ☐ |
| P-06 | STM32 draws current | USB or VIN current | 20–100 mA | — | MCU not running | ☐ |
| P-07 | ESP32 draws current | USB current | 50–200 mA | — | ESP32 not booting | ☐ |
| P-08 | 24V after main relay (OFF) | BTS7960 B+ (any) | 0V | — | Relay stuck closed | ☐ |
| P-09 | No smoke/heat | All components | Room temperature | — | **DISCONNECT IMMEDIATELY** | ☐ |
| P-10 | LED LD2 blinking | PA5 (Nucleo LED) | Visible blink | — | Firmware not running | ☐ |

---

## 4. CAN Bus Verification

**Equipment:** Oscilloscope (preferred) or logic analyzer.

| # | Test | Method | Expected | Fail Action | ☐ |
|---|------|--------|----------|-------------|---|
| CAN-01 | CANH resting voltage | Measure CANH to GND | 2.5V (recessive) | Check transceiver power | ☐ |
| CAN-02 | CANL resting voltage | Measure CANL to GND | 2.5V (recessive) | Check transceiver power | ☐ |
| CAN-03 | Differential recessive | CANH − CANL | 0V (±0.1V) | Wiring or termination issue | ☐ |
| CAN-04 | STM32 heartbeat TX | Oscilloscope on CANH | 0x001 frame every 100 ms | Firmware/transceiver issue | ☐ |
| CAN-05 | ESP32 heartbeat TX | Oscilloscope on CANH | 0x011 frame every 100 ms | Firmware/transceiver issue | ☐ |
| CAN-06 | Dominant voltage | During TX: CANH−CANL | 1.5–3.0V differential | Termination/wiring issue | ☐ |
| CAN-07 | No bus errors | CAN error counters | TEC/REC < 10 | Check wiring/termination | ☐ |
| CAN-08 | Both nodes communicate | STM32 receives ESP32 HB | HEARTBEAT_ESP32 (0x011) seen | Check filters/wiring | ☐ |

---

## 5. Motor Driver Tests

**Equipment:** Multimeter, oscilloscope (optional), current-limited supply.  
**⚠ Test with motors DISCONNECTED first (BTS7960 M+/M− open). Then with light-load motor.**

### Phase 1: No-Load Test (Motors Disconnected)

| # | Test | Method | Expected | ☐ |
|---|------|--------|----------|---|
| M-01 | BTS7960 logic power | Measure BTS7960 VCC pin | 3.3V–5V | ☐ |
| M-02 | PWM at PA8 (RPWM_FL) | Oscilloscope on PA8 | 20 kHz, variable duty cycle | ☐ |
| M-03 | PWM at PA6 (RPWM_STEER) | Oscilloscope on PA6 | 20 kHz, 0% duty at idle | ☐ |
| M-04 | EN_FL (PC5) initial state | Multimeter on PC5 | 0V (LOW) during boot inhibit | ☐ |
| M-05 | BTS M+ output (no load) | Measure BTS M+ to GND | Switching between 0V and B+ | ☐ |

### Phase 2: Motor Connected (Low Speed)

| # | Test | Method | Expected | ☐ |
|---|------|--------|----------|---|
| M-06 | Motor FL spins | Apply 10% throttle | Motor turns slowly | ☐ |
| M-07 | Motor direction correct | Apply forward gear + throttle | All motors same direction | ☐ |
| M-08 | Steering responds | Steering command via CAN | Steering motor moves | ☐ |
| M-09 | Current within limits | INA226 reading (CAN 0x201) | < 25A per motor | ☐ |
| M-10 | Temperature stable | DS18B20 (CAN 0x202) | < 50°C after 30s run | ☐ |

---

## 6. Sensor Verification

| # | Sensor | Test Method | Expected Reading | ☐ |
|---|--------|-------------|------------------|---|
| S-01 | Pedal (PA3) idle | Measure PA3 voltage | 0.17–0.25V (pedal released) | ☐ |
| S-02 | Pedal (PA3) full | Measure PA3 voltage | 1.8–2.0V (pedal fully pressed) | ☐ |
| S-03 | Encoder A (PA15) | Rotate steering shaft | Square wave on oscilloscope | ☐ |
| S-04 | Encoder B (PB3) | Rotate steering shaft | 90° phase shift from A | ☐ |
| S-05 | Encoder Z (PB4) | Full rotation | 1 pulse per revolution | ☐ |
| S-06 | Steer center (PB5) | Move past center screw | Clean HIGH→LOW transition | ☐ |
| S-07 | Wheel FL (PA0) | Rotate wheel | Pulses (6 per revolution) | ☐ |
| S-08 | Wheel FR (PA1) | Rotate wheel | Pulses (6 per revolution) | ☐ |
| S-09 | Wheel RL (PA2) | Rotate wheel | Pulses (6 per revolution) | ☐ |
| S-10 | Wheel RR (PB15) | Rotate wheel | Pulses (6 per revolution) | ☐ |
| S-11 | DS18B20 temp | Read CAN 0x202 | 20–30°C (ambient) | ☐ |
| S-12 | INA226 voltage | Read CAN 0x207 | ~24V battery voltage | ☐ |
| S-13 | INA226 current (no load) | Read CAN 0x201 | < 1A (quiescent) | ☐ |

---

## 7. Relay Tests

| # | Relay | Test | Expected | ☐ |
|---|-------|------|----------|---|
| R-01 | RELAY_MAIN (PC10) | Toggle via firmware | Audible click; 24V appears at BTS B+ | ☐ |
| R-02 | RELAY_TRAC (PC11) | Toggle via firmware | Audible click; traction motors powered | ☐ |
| R-03 | RELAY_DIR (PC12) | Toggle via firmware | Audible click; steering motor powered | ☐ |
| R-04 | RELAY_LED_F (PB10) | Toggle via CAN 0x120 byte0=1 | Front LED strip power on (measure 5V) | ☐ |
| R-05 | RELAY_LED_R (PB11) | Toggle via CAN 0x120 byte1=1 | Rear LED strip power on (measure 5V) | ☐ |
| R-06 | Relay release spike | Oscilloscope on relay coil | Spike clamped to < V_supply + 1V by flyback | ☐ |
| R-07 | Boot state (all relays) | Power on MCU | All relays OFF (pull-down holds gate LOW) | ☐ |

---

## 8. ESP32-S3 Peripheral Tests

| # | Peripheral | Test | Expected | ☐ |
|---|------------|------|----------|---|
| E-01 | TFT Display | Power on ESP32 | Display shows HMI screen | ☐ |
| E-02 | Touch screen | Tap on display | Touch coordinates change | ☐ |
| E-03 | CAN reception | Check vehicle data screen | Speed/current/temp values update | ☐ |
| E-04 | DFPlayer audio | Trigger welcome sound | "Bienvenido Marcos" plays | ☐ |
| E-05 | Audio relay | During playback | Relay clicks ON, then OFF after audio | ☐ |
| E-06 | Front LEDs | Verify KITT animation | Red scanner pattern on 28 LEDs | ☐ |
| E-07 | Rear LEDs | Verify tail lights | Dim red on center LEDs (3-12) | ☐ |
| E-08 | Gear shifter | Move shifter to each position | Gear display updates on HMI | ☐ |
| E-09 | Traction switch | Toggle 2WD/4WD | Traction mode updates on HMI | ☐ |
| E-10 | Ignition sense | Turn key OFF | Shutdown sequence starts; farewell plays | ☐ |

---

## 9. Emergency Disconnect Procedure

### Immediate Power Cut Sequence

1. **Turn ignition key to OFF position** — this triggers controlled shutdown
2. **If emergency (smoke/fire/shock):**
   - **Disconnect battery negative (−) terminal FIRST** — safest disconnect
   - **Then disconnect battery positive (+) terminal**
   - **Do NOT touch both terminals simultaneously**
3. **If MCU is unresponsive:**
   - Remove main fuse (F1, 60A) — instantly cuts all motor power
   - The 5V/12V auxiliary circuits may remain powered via USB/separate supply
4. **After emergency disconnect:**
   - Wait 30 seconds for capacitors to discharge
   - Verify 0V on 24V bus before touching any wiring
   - Inspect for burn marks, melted insulation, or hot components

### Safe Re-Energization After Emergency

1. Identify and fix the root cause
2. Repeat ALL continuity tests (Section 2)
3. Use current-limited power supply for first re-power
4. Verify no excessive current draw before connecting battery

---

## 10. What NOT To Do — Common Errors That Damage Components

### ❌ ERROR 1: Connecting 24V or 12V directly to STM32 GPIO
**What happens:** Instant destruction of GPIO pin and possibly the MCU.  
**Prevention:** All signals from higher-voltage domains MUST go through optocouplers (6N137/PC817) or voltage dividers.

### ❌ ERROR 2: Omitting flyback diode on relay coil
**What happens:** When the relay turns OFF, the collapsing magnetic field generates a voltage spike (50–100V) that destroys the MOSFET driver.  
**Prevention:** Always install 1N4007 diode in anti-parallel across the relay coil.

### ❌ ERROR 3: Missing CAN termination
**What happens:** Signal reflections cause bit errors, intermittent communication, and bus-off events.  
**Prevention:** 120 Ω termination at BOTH ends of the CAN bus. Measure 60 Ω between CANH and CANL with both terminators installed.

### ❌ ERROR 4: Connecting encoder directly to STM32 without level shifting
**What happens:** The E6B2-CWZ6C encoder outputs 5V logic. STM32G474 GPIO absolute max is 4.0V (FT pins) or 3.6V (non-FT). Exceeding this damages the input buffer.  
**Prevention:** Use 6N137 high-speed optocouplers for 5V-to-3.3V isolation.

### ❌ ERROR 5: Using PC817 for encoder signals
**What happens:** PC817 has 4–6 µs propagation delay. At 1200 PPR, the encoder pulses can be as short as ~200 µs at moderate speeds, but TIM2 digital filter is set to 282 ns threshold. The PC817 delay causes pulse distortion.  
**Prevention:** Use 6N137 (75–120 ns delay) for encoder channels A/B/Z. PC817 is fine for wheel speed sensors (< 1 kHz).

### ❌ ERROR 6: Connecting 5V pedal output directly to ADC
**What happens:** PA3 ADC input sees 4.8V from pedal when max is 3.6V → GPIO damage.  
**Prevention:** Voltage divider (10 kΩ / 6.8 kΩ) reduces max to ~2.0V. Add BAT54S clamp diode as secondary protection.

### ❌ ERROR 7: Wrong resistor values in pedal divider (1 kΩ / 2 kΩ)
**What happens:** With 1 kΩ / 2 kΩ, max output = 5 × 2/(1+2) = 3.33V. This is within spec BUT just barely at the 3.3V rail, leaving zero margin. Also current draw = 5V/3k = 1.67 mA (vs. 0.3 mA with 10k/6.8k).  
**Prevention:** Use the firmware-specified 10 kΩ / 6.8 kΩ values (Vmax = 2.02V, large safety margin).

### ❌ ERROR 8: Swapping CAN_TX and CAN_RX at transceiver
**What happens:** No communication; both nodes transmit but neither receives. Can cause bus-off.  
**Prevention:** STM32 PB9 (TX) goes to transceiver TXD. STM32 PB8 (RX) goes to transceiver RXD. Double-check: TX→TXD, RX→RXD (NOT crossed like UART).

### ❌ ERROR 9: Using GPIO 26-37 on ESP32-S3-WROOM-1-N16R8
**What happens:** GPIO 26–32 are the QSPI flash bus; driving them corrupts flash and bricks the module. GPIO 33–37 are the Octal PSRAM bus; driving them causes memory corruption and random crashes.  
**Prevention:** Only use GPIO 0–21 and 38–48 (with strapping pin awareness).

### ❌ ERROR 10: Connecting motors before verifying PWM outputs
**What happens:** If firmware has a bug or the wrong timer config, motors could receive full duty cycle and spin at maximum speed instantly.  
**Prevention:** Always verify PWM waveforms with oscilloscope on BTS7960 RPWM/LPWM pins BEFORE connecting motors. Check that idle state is 0% duty.

---

## First Power-Up Procedure (Step by Step)

### Prerequisites
- [ ] All visual inspections passed (Section 1)
- [ ] All continuity tests passed (Section 2)
- [ ] Motors DISCONNECTED from BTS7960 M+/M−
- [ ] Current-limited power supply available (recommended)
- [ ] Multimeter and oscilloscope ready

### Sequence

1. **Connect 5V/3.3V only** (via USB to Nucleo and ESP32 DevKit)
   - Verify P-04 (STM32 3.3V) and P-05 (ESP32 3.3V)
   - Verify STM32 LD2 LED blinking (firmware running)
   - Verify ESP32 display initializes

2. **Test CAN communication** (5V only, no motor power)
   - Verify CAN-01 through CAN-08
   - Confirm both heartbeats visible

3. **Connect 24V battery** (through current-limited supply if possible)
   - Verify P-01 (battery voltage)
   - Verify P-08 (relays OFF at boot)
   - Check no excessive current (< 500 mA quiescent)

4. **Activate relays one by one**
   - RELAY_MAIN first → verify 24V at BTS B+ inputs
   - RELAY_TRAC → verify voltage downstream
   - RELAY_DIR → verify steering BTS powered

5. **Verify PWM outputs** (oscilloscope, motors still disconnected)
   - Check 20 kHz, 0% duty on all RPWM/LPWM pins
   - Send 10% throttle command via CAN → verify duty changes

6. **Connect ONE motor** (front-left recommended)
   - Apply minimal throttle (5%)
   - Verify direction and current
   - If OK, connect remaining motors one by one

7. **Full system test**
   - All motors connected
   - Test forward/reverse/steering
   - Monitor temperatures and currents via CAN/HMI
   - Run for 5 minutes and check for overheating
