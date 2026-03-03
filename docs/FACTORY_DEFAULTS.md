# Factory Defaults / Valores de Fábrica

## Overview / Descripción

The factory defaults system allows resetting specific calibration categories to their
default values when a calibration has failed or was performed incorrectly.

El sistema de valores de fábrica permite restablecer categorías específicas de calibración
a sus valores por defecto cuando una calibración ha fallado o se ha realizado incorrectamente.

## Access / Acceso

1. Enter engineering mode: Touch code **8989** on the HMI screen
2. Select **FACTORY DEFAULTS** from the engineering menu
3. Tap the desired reset option
4. The STM32 will acknowledge the command (OK/REJECTED/BLOCKED)

## Reset Categories / Categorías de Reset

### 1. Reset Steering PID (`0xF0`)

**What it resets / Qué resetea:**
- Re-enables MODULE_STEER_CENTER (module 19)
- Re-enables MODULE_STEER_ENCODER (module 20)
- Clears faults on both modules

**When to use / Cuándo usar:**
- Steering PID calibration failed or produced incorrect behavior
- Encoder is reporting errors after a sensor replacement
- Steering center sensor not detecting center position

**Note**: PID gains (Kp=2.0, Ki=0.1, Kd=0.5) are compile-time constants and
are not modified by this reset. For PID gain changes, a firmware update is required.

### 2. Reset Wheel Sensors (`0xF1`)

**What it resets / Qué resetea:**
- Re-enables MODULE_WHEEL_SPEED_FL (module 15)
- Re-enables MODULE_WHEEL_SPEED_FR (module 16)
- Re-enables MODULE_WHEEL_SPEED_RL (module 17)
- Re-enables MODULE_WHEEL_SPEED_RR (module 18)
- Clears faults on all 4 wheel speed sensors

**When to use / Cuándo usar:**
- Wheel speed sensor disconnected/reconnected
- False fault reports after wheel or sensor replacement
- ABS/TCS behavior inconsistent due to wheel sensor issues

### 3. Reset INA226 / Shunts (`0xF2`)

**What it resets / Qué resetea:**
- Re-enables MODULE_CURRENT_SENSOR_0 through MODULE_CURRENT_SENSOR_5 (modules 9–14)
- Clears faults on all 6 current sensor channels

**When to use / Cuándo usar:**
- INA226 sensor replacement or reconnection
- I2C bus errors cleared but sensors still showing faulted
- Current readings incorrect after hardware changes

**Note**: Shunt resistance values (1mΩ for motors, 0.5mΩ for battery) are compile-time
constants and are not modified by this reset. For shunt calibration changes, a firmware
update is required.

### 4. Reset Traction Motor Force (`0xF3`)

**What it resets / Qué resetea:**
- Re-enables MODULE_ABS (module 21)
- Re-enables MODULE_TCS (module 22)
- Clears faults on both modules

**When to use / Cuándo usar:**
- ABS or TCS incorrectly disabled after a fault
- Motor behavior inconsistent after a safety event
- Traction control not engaging properly

**Note**: Maximum motor current limit (25A per motor) is a compile-time constant
and is not modified by this reset.

### 5. Reset Steering Motor Force (`0xF4`)

**What it resets / Qué resetea:**
- Re-enables MODULE_STEER_CENTER (module 19)
- Re-enables MODULE_STEER_ENCODER (module 20)
- Re-enables MODULE_ACKERMANN (module 23)
- Clears faults on all 3 modules

**When to use / Cuándo usar:**
- Steering motor not responding after a fault
- Ackermann geometry corrections disabled
- Steering calibration failed and needs to be re-attempted

### 6. Reset All / Factory Restore (`0xFF`)

**What it resets / Qué resetea:**
- Re-enables ALL 25 modules
- Clears all manual disable states
- Clears `MODULE_FAULT_DISABLED` faults (real faults persist)

**When to use / Cuándo usar:**
- Multiple subsystems have been disabled and you want a clean start
- After any major hardware change or repair
- Before handing the vehicle to a different operator

## CAN Protocol / Protocolo CAN

All factory defaults commands use CAN ID `0x110` (SERVICE_CMD):

| Byte 0 | Byte 1 | Action |
|--------|--------|--------|
| `0xF0` | `0x00` | Reset steering PID |
| `0xF1` | `0x00` | Reset wheel sensors |
| `0xF2` | `0x00` | Reset INA226/shunts |
| `0xF3` | `0x00` | Reset traction motor force |
| `0xF4` | `0x00` | Reset steering motor force |
| `0xFF` | `0x00` | Factory restore (all) |

**Response**: ACK on CAN ID `0x103` with result OK (0x00).

## Safety / Seguridad

- Factory reset commands are **always accepted** — restoring defaults is inherently safe
- If the underlying hardware fault persists, the fault will reappear on the next sensor check cycle
- The vehicle should be **stationary** when performing factory resets
- A **reboot may be required** for full effect (especially for steering calibration)

## Limitations / Limitaciones

The following parameters are **compile-time constants** and cannot be changed via the
factory defaults menu:

| Parameter | Value | File |
|-----------|-------|------|
| Steering PID Kp | 2.0 | `motor_control.c` |
| Steering PID Ki | 0.1 | `motor_control.c` |
| Steering PID Kd | 0.5 | `motor_control.c` |
| Max motor current | 25A | `safety_system.c` |
| Max motor temperature | 80°C warning, 90°C critical | `safety_system.c` |
| INA226 shunt resistance | 1mΩ (motor), 0.5mΩ (battery) | `sensor_manager.c` |
| ABS/TCS slip threshold | 15% | `safety_system.c` |

To modify these values, a firmware update is required.

## Future Enhancements / Mejoras Futuras

- ⬜ **Persistent PID tuning**: Store PID gains in Flash, adjustable via engineering menu
- ⬜ **Persistent motor force limits**: Store max current per motor in Flash
- ⬜ **Persistent INA226 calibration**: Store shunt resistance and offset per channel in Flash
- ⬜ **DFPlayer audio feedback**: Play confirmation/error sounds during factory reset
