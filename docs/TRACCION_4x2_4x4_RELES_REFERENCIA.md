# Tracción 4x2 / 4x4 y Relés de Potencia — Referencia Completa

> **Fuentes verificadas:** `esp32/src/traction_switch.h`, `esp32/src/traction_switch.cpp`,
> `Core/Src/motor_control.c`, `Core/Src/safety_system.c`, `Core/Src/can_handler.c`,
> `Core/Inc/project_config.h`

---

## 1. Relés de potencia del STM32

El STM32 controla **dos relés de potencia** en GPIOC:

| Pin STM32 | Nombre firmware         | Tensión | Función                                                  |
|-----------|-------------------------|---------|----------------------------------------------------------|
| **PC11**  | `PIN_RELAY_TRAC`        | 24 V    | Relé de tracción — alimenta los 4 BTS7960 de los motores |
| **PC12**  | `PIN_RELAY_STEER_PWR`   | 12 V    | Relé de dirección — alimenta el BTS7960 del actuador de dirección |

Hay además dos relés de LED en GPIOB (`PB10` frente, `PB11` trasero) que no son de potencia de motores.

### Secuencia de encendido (automática, al pasar a estado ACTIVE)

```
PC11 TRAC ON  →  espera 50 ms (RELAY_TRACTION_SETTLE_MS)  →  PC12 STEER_PWR ON
```

- El firmware activa primero la tracción (24 V), espera 50 ms para que se estabilice el
  inrush, y luego activa la dirección.
- Los subsistemas deben comprobar `Safety_IsPowerReady()` antes de usar los motores.
- Al apagar el orden es **inverso**: PC12 OFF → PC11 OFF (sin retardo entre ellos).

### Módulos de retardo externos

Los módulos de retardo hardware externos deben configurarse a **retardo cero**.
El firmware gestiona todo el timing internamente.

### Fuente verificada

```c
// safety_system.c (línea ~134)
#define RELAY_TRACTION_SETTLE_MS 50   // Tiempo entre TRAC ON y STEER_PWR ON

// Encendido: PC11 → espera → PC12
HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_SET);
// ... tras 50 ms ...
HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR, GPIO_PIN_SET);

// Apagado (safety_system.c ~816): ambos OFF simultáneamente y atómico
GPIOC->BSRR = (uint32_t)(PIN_RELAY_TRAC | PIN_RELAY_STEER_PWR) << 16U;
```

---

## 2. INA226 y relés — qué está alimentado cuando

| Canal TCA9548A | INA226   | Posición en el circuito        | Alimentado cuando...             |
|----------------|----------|-------------------------------|----------------------------------|
| CH0            | Motor FL | DESPUÉS del relé de tracción   | Relé TRAC (PC11) cerrado         |
| CH1            | Motor FR | DESPUÉS del relé de tracción   | Relé TRAC (PC11) cerrado         |
| CH2            | Motor RL | DESPUÉS del relé de tracción   | Relé TRAC (PC11) cerrado         |
| CH3            | Motor RR | DESPUÉS del relé de tracción   | Relé TRAC (PC11) cerrado         |
| CH4            | Batería  | ANTES del relé (en paralelo)   | **Siempre**, incluso con relé abierto |
| CH5            | Dirección| DESPUÉS del relé de dirección  | Relé STEER_PWR (PC12) cerrado    |

El firmware usa una máscara `ina_expected_mask` para no marcar error I2C (Code 11) en los
INA que están sin alimentación porque su relé está abierto.

---

## 3. Cambio de tracción 4x2 ↔ 4x4 — el interruptor físico

### Qué tipo de interruptor se usa

Un **interruptor de balancín DPDT (doble polo doble tiro)** usado como SPDT (solo un polo).
Es un interruptor **que se queda fijo en la posición** (latching / de enganche), no
momentáneo — es decir, queda enclavado en 4x2 o en 4x4 hasta que lo cambias.

### Conexión al ESP32

Solo se usan **dos cables** del interruptor:

| Cable del interruptor | Conectar a...           |
|-----------------------|-------------------------|
| **Común (COM)**       | **GPIO 15 del ESP32**   |
| **Salida lado 4WD**   | **GND**                 |

El GPIO 15 tiene **pull-up interno activado por firmware**. La lógica es:

```
Interruptor en 4WD  →  COM conectado a GND  →  GPIO 15 = LOW   →  4x4 activo
Interruptor en 2WD  →  COM libre (pull-up)  →  GPIO 15 = HIGH  →  4x2 activo
```

**El segundo polo y el cable NC (normalmente cerrado) del DPDT no se usan** — quedan libres.

### Por qué pull-up y GPIO LOW = 4WD

Esta elección es fail-safe: si el cable del interruptor se desconecta, el GPIO flota a HIGH
por el pull-up interno → el firmware interpreta 2WD (modo más seguro, solo ruedas delanteras).

### Código verificado (traction_switch.cpp)

```cpp
// traction_switch.cpp
static Mode readRaw() {
    int level = digitalRead(cfg_.gpioPin);   // cfg_.gpioPin = 15 por defecto
    // LOW = conectado a GND = 4WD, HIGH = pull-up (abierto) = 2WD
    return (level == LOW) ? Mode::FOUR_WD : Mode::TWO_WD;
}

void init(const Config& cfg) {
    pinMode(cfg_.gpioPin, INPUT_PULLUP);   // Pull-up interno
    ...
}
```

---

## 4. Protecciones software del cambio de modo

El firmware NO permite cambiar entre 4x2 y 4x4 si el vehículo está en marcha:

| Condición           | Comportamiento                                         |
|---------------------|--------------------------------------------------------|
| Velocidad > 0,5 km/h | Cambio **bloqueado**; se retiene el modo anterior      |
| Velocidad ≤ 0,5 km/h | Cambio **aceptado** tras 50 ms de anti-rebote software |

El debounce software requiere **3 lecturas estables** separadas por 10 ms cada una, más un
tiempo de estabilización de 50 ms desde el primer cambio de flanco.

```cpp
// traction_switch.h — valores por defecto
struct Config {
    int      gpioPin         = 15;    // GPIO ESP32
    uint32_t debounceMs      = 50;    // Tiempo mínimo estable
    uint8_t  stableCount     = 3;     // Lecturas consecutivas iguales requeridas
    uint32_t pollMs          = 10;    // Intervalo de muestreo
    float    speedThreshKmh  = 0.5f;  // Velocidad máxima para permitir el cambio
};
```

---

## 5. Flujo completo del cambio de modo (4x2 ↔ 4x4)

```
[Interruptor físico cambia posición]
          │
          ▼
[ESP32 GPIO 15 detecta flanco LOW/HIGH]
          │
          ▼
[Debounce 50 ms + 3 lecturas estables]
          │
          ▼ (solo si velocidad ≤ 0,5 km/h)
[traction_sw::hasChanged() → true]
          │
          ▼
[esp32/src/main.cpp: sendModeCommand(modeFlags)]
  CAN 0x102 (CMD_MODE), byte 0:
    bit 0 = 1 → 4WD
    bit 0 = 0 → 2WD
    bit 1 = estado del modo tank (independiente)
          │
          ▼
[STM32 recibe CAN 0x102]
  Valida: vehículo parado (pedal < 3 %, velocidad ≤ 1 km/h)
  Llama: Traction_SetMode4x4(enable_4x4)
  Envía ACK: CAN 0x103
          │
          ▼
[STM32 heartbeat 0x001 (100 ms)]
  byte 4 statusFlags: bit 1 = modo 4x4 activo (eco)
          │
          ▼
[ESP32 recibe eco y actualiza HMI]
  Icono "4x4" o "4x2" en la pantalla TFT
  Audio: TRACTION_4X4 (track 37) o TRACTION_4X2 (track 38)
```

---

## 6. Comportamiento del firmware en 4x2 vs 4x4

### En DRIVE (acelerando)

| Modo | Ruedas delanteras (FL, FR) | Ruedas traseras (RL, RR) | PWM base por rueda |
|------|---------------------------|--------------------------|-------------------|
| **4x2** | Tracción activa            | Freno pasivo             | `base_pwm` completo |
| **4x4** | Tracción activa            | Tracción activa          | `base_pwm / 2` (split 50/50 por eje) |

En 4x4 el PWM total se divide entre los dos ejes para mantener la misma potencia total.

### En COAST (rueda libre, sin pedal)

| Modo | Ruedas delanteras | Ruedas traseras |
|------|-------------------|-----------------|
| **4x2** | Rueda libre (Hi-Z) | Freno pasivo (BTS7960_BRAKE_PWM) |
| **4x4** | Rueda libre (Hi-Z) | Rueda libre (Hi-Z) |

### En BRAKE (parado)

Todos los motores: freno pasivo (PWM=0 + EN=HIGH) independientemente del modo.

### Código verificado (motor_control.c ~1843)

```c
} else if (traction_state.mode4x4) {
    // 4x4: todos los motores, PWM dividido por 2
    uint16_t axle_pwm = base_pwm / 2;
    for (uint8_t i = 0; i < 4; i++) {
        desired_pwm[i] = (uint16_t)(axle_pwm * acker_diff[i] * wheel_scale[i]);
        desired_en[i]  = 1;
    }
} else {
    // 4x2: solo ruedas delanteras, traseras en freno
    desired_pwm[MOTOR_FL] = (uint16_t)(base_pwm * acker_diff[FL] * wheel_scale[FL]);
    desired_pwm[MOTOR_FR] = (uint16_t)(base_pwm * acker_diff[FR] * wheel_scale[FR]);
    desired_en[MOTOR_FL]  = 1;
    desired_en[MOTOR_FR]  = 1;
    desired_pwm[MOTOR_RL] = BTS7960_BRAKE_PWM; desired_en[MOTOR_RL] = 1;
    desired_pwm[MOTOR_RR] = BTS7960_BRAKE_PWM; desired_en[MOTOR_RR] = 1;
}
```

---

## 7. Telemetría CAN relacionada

| CAN ID | Dirección      | Campo                         | Significado                          |
|--------|----------------|-------------------------------|--------------------------------------|
| 0x001  | STM32 → ESP32  | byte 4, bit 1 (`STATUS_FLAG_MODE_4X4`) | Eco del modo 4x4 activo en STM32 |
| 0x001  | STM32 → ESP32  | byte 5, bit 1                 | Relé TRAC (PC11) comandado ON        |
| 0x001  | STM32 → ESP32  | byte 5, bit 2                 | Relé STEER_PWR (PC12) comandado ON   |
| 0x001  | STM32 → ESP32  | byte 5, bit 7                 | Secuencia de relés completa          |
| 0x102  | ESP32 → STM32  | byte 0, bit 0                 | Solicitud 4x4 (1) o 4x2 (0)         |
| 0x102  | ESP32 → STM32  | byte 0, bit 1                 | Modo tank turn                       |
| 0x103  | STM32 → ESP32  | ACK                           | Confirmación del cambio de modo      |

---

## 8. Esquema de conexión del interruptor de tracción

```
Interruptor DPDT latching (balancín ON-ON)
            ┌───────────────────────┐
            │  DPDT ROCKER SWITCH   │
            │                       │
    GND ───►│ (salida 4WD)          │
            │       ↕               │
            │ (COM) ────────────────┼──────► GPIO 15 ESP32-S3
            │                       │         (INPUT_PULLUP interno)
            │ (salida 2WD)          │
            │   (no conectar / NC)  │
            └───────────────────────┘

Cuando el balancín está en 4WD:
  COM → GND  →  GPIO 15 = LOW  →  4x4 activo

Cuando el balancín está en 2WD:
  COM → abierto → pull-up → GPIO 15 = HIGH  →  4x2 activo
```

**Solo son necesarios 2 cables:** el del COM al GPIO 15, y el del lado 4WD a GND.
El lado 2WD del interruptor se deja libre (el pull-up interno hace esa función).

---

## 9. Mando RC — canal CH6

El cambio de tracción también puede venir del mando RC a través del canal CH6.
El ESP32 combina ambas fuentes (interruptor físico y mando) y envía siempre el mismo
CAN 0x102 al STM32. El interruptor físico tiene prioridad en cuanto se detecta cambio.

---

## 10. Configuración persistente

El modo de tracción (4x2/4x4) **no se persiste en flash** al arrancar.
Al encender, el ESP32 lee el estado físico del interruptor (GPIO 15) como estado inicial,
y en el primer loop lo envía al STM32 vía CAN 0x102.

El modo tank turn sí se persiste (`config_store::setDriveMode()`).

---

## 11. Resumen de pines

| Dispositivo | Pin / GPIO  | Dirección | Función                                  |
|-------------|-------------|-----------|------------------------------------------|
| STM32       | PC11        | Salida    | Relé tracción 24 V (BTS7960 x4)          |
| STM32       | PC12        | Salida    | Relé alimentación dirección 12 V          |
| STM32       | PB10        | Salida    | Relé LEDs frontales 5 V                  |
| STM32       | PB11        | Salida    | Relé LEDs traseros 5 V                   |
| ESP32-S3    | GPIO 15     | Entrada   | Interruptor 4WD/2WD (pull-up, LOW=4WD)  |

---

## 12. Documentos relacionados

- `docs/MOTOR_CONTROL.md` — Control de motores por rueda
- `docs/TRACTION_PER_WHEEL.md` — ABS/TCS por rueda (4x2/4x4)
- `docs/CAN_CONTRACT_FINAL.md` §4.5 — CMD_MODE (0x102) y ACK (0x103)
- `docs/POWER_DISTRIBUTION.md` — Distribución de potencia y relés
- `docs/RELE_RETARDO_ENCENDIDO_APAGADO.md` — Relé de retención del sistema (encendido/apagado, diferente de los relés de tracción)
- `Documentos/RELAY_STATUS_VISUALIZATION.md` — Visualización del estado de relés en HMI
