# Remote Control Architecture — MarcosDashboard v10 Final

> **Estado:** Propuesta técnica. **NO IMPLEMENTADO**. Documento de diseño previo a cualquier
> cambio de firmware. Verificado contra el código real del repositorio.
>
> **Última verificación:** 2026-05-24

---

## 1. Resumen ejecutivo

Este documento describe la arquitectura propuesta para añadir **control remoto inalámbrico**
al vehículo MarcosDashboard v10 Final manteniendo:

- **STM32G474RE como única autoridad de seguridad** (safety authority).
- **`safety_system.c`, `motor_control.c`, PWM timers, watchdogs, ABS/TCS, encoder steering
  y renderTask Core 0 intactos**.
- **Protocolo CAN existente sin cambios** (mismos IDs 0x100 / 0x101, mismo formato).
- **El mando remoto NUNCA puede bypassar las capas de seguridad**.

El mando se conecta exclusivamente al **ESP32-S3**, que ya es el productor existente de
comandos CAN hacia el STM32. El STM32 ve los mismos frames que vería viniendo de la HMI
existente; **no sabe ni necesita saber** que el origen es un mando RC.

---

## 2. Validación del hardware mostrado en la foto

La foto adjunta muestra un **FlySky FS-i6 + receptor FS-iA6**.

### 2.1 Veredicto: ❌ **NO COMPRAR este conjunto**

El receptor **FS-iA6 (sin "B") NO tiene salida iBUS**. Solo dispone de:

- 6 salidas PWM independientes (1 pulso de 1–2 ms por canal por trama de 20 ms).
- Salida PPM (CPPM) en CH1 (no recomendada por jitter y resolución).

Esto significa:

- ❌ No hay UART digital de 115200 baud lista para conectar al ESP32-S3.
- ❌ Para leer PWM hay que ocupar 4–6 GPIO con interrupciones de captura por flanco
  (`pulseIn` o interrupt-based timing). Más código, más ruido, más jitter, más
  superficie de fallo.
- ❌ Sin telemetría inversa (no se puede mandar info del coche al mando).
- ❌ Sin checksum a nivel de protocolo — solo la coherencia del pulso.

### 2.2 Conjunto correcto a comprar

| Item | Modelo recomendado | Justificación |
|---|---|---|
| **Transmisor (mando)** | **FlySky FS-i6X** | 10 canales nativos sin hack de firmware; mismo formfactor, ergonomía y AFHDS 2A; LCD; failsafe configurable por canal desde el menú |
| **Receptor** | **FlySky FS-iA6B** | Salida **iBUS dedicada** (UART 115200 baud, 3.3 V TTL); failsafe hardware; antena dual diversity; alimentación 4.0–6.5 V (compatible con rail 5 V del proyecto) |

### 2.3 FS-i6 vs FS-i6X (diferencias reales)

| Característica | FS-i6 | FS-i6X |
|---|---|---|
| Canales nativos | 6 | **10** |
| Canales máximos | 10 (con firmware no oficial / hack) | 10 (oficial) |
| AFHDS 2A | Sí | Sí |
| Failsafe configurable por canal | Sí | Sí |
| Memoria de modelos | Menor | Mayor |
| Mixing avanzado | Limitado | Mejorado |
| Soporte oficial actualizaciones | Limitado | Sí |

**Conclusión:** No tiene sentido ahorrar 5–10 € en el FS-i6 cuando el FS-i6X aporta
4 canales adicionales nativos (kill switch, modo eco/sport, source select LOCAL/REMOTE,
luz de freno manual) sin recurrir a firmware no oficial.

### 2.4 FS-iA6 vs FS-iA6B (diferencias reales)

| Característica | FS-iA6 | FS-iA6B |
|---|---|---|
| Salidas PWM | 6 | 6 |
| PPM (CPPM) | Sí (en CH1) | Sí (en CH1) |
| **iBUS Servo (UART 115200)** | **❌ NO** | **✅ SÍ (puerto dedicado)** |
| **iBUS Telemetría** | ❌ NO | ✅ SÍ |
| Failsafe HW | Sí | Sí |
| Antena dual diversity | Sí | Sí |
| Alimentación | 4.0–6.5 V | 4.0–6.5 V |

**Conclusión:** El FS-iA6B es **obligatorio** para esta arquitectura. El FS-iA6 sería
viable únicamente con un enfoque PWM de captura por interrupciones — descartado por
fragilidad ante EMI, jitter y consumo de GPIO.

---

## 3. ¿Por qué iBUS y no otro sistema RC?

| Sistema RC | Veredicto | Motivo |
|---|---|---|
| **iBUS (FlySky)** | ✅ **ELEGIDO** | UART normal 3.3 V 115200, FHSS 142 canales, failsafe HW, 7 ms por trama, sin componentes adicionales entre receptor y ESP32 |
| SBUS (FrSky) | ⚠️ Aceptable | UART **invertido** lógicamente — necesita transistor inversor entre receptor y ESP32. Más complejidad por 0 beneficio frente a iBUS |
| CRSF (TBS Crossfire) | ⚠️ Sobrecosto | Excelente protocolo (latencia ~3 ms) pero TX + RX cuestan 5–10× más. Justificado solo para FPV racing |
| **ESP-NOW** | ❌ | Requiere construir el mando con un segundo ESP32 (sin joysticks de fábrica). Sin FHSS real (canal fijo) → vulnerable a EMI 2.4 GHz. Sin failsafe HW. Comparte radio WiFi/BT del ESP32-S3 |
| **BLE Gamepad** | ❌ | Reconexión 2–5 s tras pérdida (inaceptable). Alcance 5–10 m. Sin failsafe HW. Ocupa radio BT del ESP32-S3 |
| **NRF24L01+PA** | ❌ | Hardware custom obligatorio en ambos extremos. SPI adicional. Sin FHSS. Sin joysticks de fábrica |
| **PWM directo (FS-iA6)** | ❌ | Consume 4–6 GPIO. Captura por ISR. Jitter. Sin checksum. Sin telemetría |
| **PPM (CPPM)** | ❌ | Susceptible a jitter. Resolución limitada. Requiere ISR de captura precisa |

**Decisión final: FlySky FS-i6X + FS-iA6B con iBUS.**

---

## 4. Arquitectura completa: mando → receptor → ESP32 → CAN → STM32

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ CAPA RF — 2.4 GHz AFHDS 2A (FHSS 142 canales, ~500 m alcance LoS)            │
│                                                                              │
│   ┌─────────────────┐                          ┌─────────────────────────┐   │
│   │  FS-i6X (TX)    │  ── 2.4 GHz RF ────▶     │  FS-iA6B (RX)           │   │
│   │                 │                          │  Alim: 5 V rail proyecto│   │
│   │  CH1: Steering  │                          │  Antena dual diversity  │   │
│   │  CH2: Throttle  │                          │  Salida iBUS:           │   │
│   │  CH3: (reserva) │                          │  UART 115200 8N1, 3.3V  │   │
│   │  CH4: (reserva) │                          │                         │   │
│   │  CH5: Kill SW   │                          │  Failsafe HW:           │   │
│   │  CH6: Modo eco  │                          │  - Si pierde RF >~1 s   │   │
│   │  CH10: Source   │                          │    canales → posiciones │   │
│   │       LOCAL/REM │                          │    configuradas en TX   │   │
│   └─────────────────┘                          └────────────┬────────────┘   │
└──────────────────────────────────────────────────────────────┬───────────────┘
                                                               │ iBUS
                                                               │ UART 115200 8N1
                                                               │ 32 bytes / 7 ms
                                                               │ Señal 3.3 V TTL
                                                               │ Cable Dupont
                                                               ▼ (GPIO 16 ESP32)
┌──────────────────────────────────────────────────────────────────────────────┐
│ ESP32-S3 — Módulo `remote_control` (futuro, NO implementado todavía)         │
│                                                                              │
│  Hilo: loop() de Core 1 (mismo donde corre la lógica CAN actual)             │
│  NO toca Core 0 (renderTask, touch, TFT)                                     │
│                                                                              │
│  1. Parser iBUS:                                                             │
│     - Lee buffer UART (no bloqueante)                                        │
│     - Sincroniza con header 0x20 0x40                                        │
│     - Valida longitud (32 bytes)                                             │
│     - Valida checksum 16-bit LE (= 0xFFFF − Σbytes)                          │
│     - Descarta paquetes corruptos sin alterar estado anterior                │
│     - Extrae 10 canales uint16 (1000–2000)                                   │
│                                                                              │
│  2. Failsafe FSM (software, en cascada al failsafe HW del receptor):         │
│     - IDLE → ACTIVE → DEGRADED → FAILSAFE                                    │
│     - Sin trama válida > 150 ms (≈21 tramas) → FAILSAFE                      │
│     - >5 checksum fails consecutivos → DEGRADED                              │
│     - CH10 en LOCAL → REMOTE_INACTIVE (mando ignorado, no se envía CAN)      │
│                                                                              │
│  3. Procesamiento idéntico al pipeline del pedal local:                      │
│     - Deadzone ±30 cuentas (≈3%) alrededor del centro 1500                   │
│     - EMA α = 0.15 (mismo valor que motor_control.c)                         │
│     - Ramp limiter 50 %/s up / 100 %/s down (idéntico a motor_control.c)     │
│     - CH1 → steering angle: lerp(1000..2000) → (−30°..+30°)                  │
│     - CH2 → throttle: lerp(1500..2000) → (0..100 %), <1500 = 0 %             │
│       (mitad inferior del stick reservada para "freno regenerativo" futuro)  │
│                                                                              │
│  4. Inyección en CAN TX (mismas APIs ya existentes en main.cpp):             │
│     - Solo si source = REMOTE && failsafe = OK && stm32IsAlive               │
│     - Escribe CAN ID 0x100 (CMD_THROTTLE) → 1 byte (0–100)                   │
│     - Escribe CAN ID 0x101 (CMD_STEERING) → 2 bytes int16 LE (1/10°)         │
│     - Cadencia 50 ms (alineada con el ciclo actual)                          │
│     - Si el módulo está inactivo → no escribe nada (comportamiento actual    │
│       totalmente inalterado)                                                 │
└──────────────────────────────────────────────────────────┬───────────────────┘
                                                           │ CAN 500 kbps
                                                           │ TJA1051T/3 (3.3 V VIO)
                                                           │ Frames 0x100 / 0x101
                                                           │ MISMO formato actual
                                                           ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│ STM32G474RE — SIN CAMBIOS de safety / motor / timers / watchdogs             │
│                                                                              │
│  `Core/Src/can_handler.c` — handlers EXISTENTES (no se modifican):           │
│    case CAN_ID_CMD_THROTTLE (0x100):                                         │
│      → Startup_IsInhibited() — bloqueo arranque                              │
│      → Range check 0..100                                                    │
│      → Safety_ValidateThrottle()                                             │
│      → Traction_SetDemand()                                                  │
│    case CAN_ID_CMD_STEERING (0x101):                                         │
│      → int16 LE / 10 → grados                                                │
│      → Safety_ValidateSteering()                                             │
│      → Steering_SetAngle()                                                   │
│                                                                              │
│  `Core/Src/safety_system.c` — INTACTO:                                       │
│    - CAN timeout 250 ms (heartbeat) → LIMP_HOME                              │
│    - Overcurrent INA226 → SAFE                                               │
│    - IWDG ~4.1 s → reset                                                     │
│    - Emergency stop → kill PWM                                               │
│    - ABS/TCS → traction_cap_factor                                           │
│                                                                              │
│  `Core/Src/motor_control.c` — INTACTO:                                       │
│    - EMA α = 0.15 + ramp 50%/s up, 100%/s down                               │
│    - Demand anomaly detector                                                 │
│    - BTS7960 PWM 20 kHz (TIM1 / TIM8)                                        │
│                                                                              │
│  El STM32 no distingue mando RC de pedal HMI: ve frames CAN normales.        │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Protocolo iBUS — Especificación completa

### 5.1 Capa física

| Parámetro | Valor |
|---|---|
| Tipo | UART asíncrono full-duplex (solo RX usado) |
| Baudrate | **115200 baud** |
| Formato | **8N1** (8 data, no parity, 1 stop) |
| Nivel lógico | **3.3 V TTL** (directo a ESP32-S3, sin level shifter) |
| Polaridad | Normal (no invertida, a diferencia de SBUS) |
| Cableado | 3 hilos: VCC (5 V) / GND / iBUS-Servo (señal) |

### 5.2 Capa de trama (iBUS Servo)

Trama de **32 bytes** transmitida cada **~7 ms** (~143 Hz):

```
Byte 0:    0x20    (header / longitud — siempre 0x20 = 32)
Byte 1:    0x40    (comando — 0x40 = servo data)
Byte 2-3:  CH1     (uint16 LE, 1000–2000)
Byte 4-5:  CH2     (uint16 LE)
...
Byte 28-29: CH14   (uint16 LE)
Byte 30-31: CHKSUM (uint16 LE) = 0xFFFF − Σ(byte[0..29])
```

**Canales lógicos:**
- 14 canales en la trama (siempre), aunque solo 6 o 10 lleven datos reales según el mando.
- Centro = **1500** (servo neutro).
- Rango operativo nominal: **1000–2000**.

### 5.3 Algoritmo de checksum

```
sum = 0xFFFF
for i in 0..29:
    sum = sum − byte[i]
checksum = sum & 0xFFFF
```

Verificación en recepción:
```
calc = 0xFFFF
for i in 0..29:
    calc = calc − byte[i]
if (calc & 0xFFFF) == ((byte[31] << 8) | byte[30]):
    frame_valid
```

### 5.4 Detección de pérdida y resincronización

- **Header sync:** buscar siempre la secuencia `0x20 0x40` en el buffer antes de leer
  los 30 bytes restantes. Esto evita arrastrar offset si llega un byte espurio.
- **Inter-frame gap:** la ausencia de datos > 21 ms (3 tramas perdidas) se considera
  pérdida transitoria y se entra en estado `STALE`. Tras 150 ms se entra en `FAILSAFE`.
- **Tras `FAILSAFE`:** se sigue intentando leer; al primer paquete válido se vuelve
  a `ACTIVE` sin penalización temporal — pero solo si el switch CH10 está en REMOTE.

---

## 6. Decisión de UART y GPIO

### 6.1 Estado actual de UARTs del ESP32-S3 (verificado en firmware real)

| UART HW | Pines actuales | Función actual | Disponible |
|---|---|---|---|
| **UART0 HW (U0TXD/U0RXD GPIOs)** | No usados | `Serial` va por **USB-CDC nativo** (`ARDUINO_USB_CDC_ON_BOOT=1`, `ARDUINO_USB_MODE=1` confirmados en `esp32/platformio.ini:41-44`) | ✅ **LIBRE — los pines hardware de UART0 NO están ocupados** |
| UART1 | RX=GPIO 18, TX=−1 | TF-Mini Plus obstacle sensor (`esp32/src/sensors/obstacle_sensor.h`) | ❌ Ocupada |
| UART2 | TX=GPIO 43, RX=GPIO 44 | DFPlayer Mini (`esp32/src/audio_manager.h`) | ❌ Ocupada |

### 6.2 GPIO elegido

**Propuesta: instanciar `HardwareSerial(0)` reasignada a GPIO 16 como RX, TX = −1.**

Justificación de **GPIO 16**:

- Confirmado **LIBRE** en `docs/PIN_USAGE_INVENTORY.md` (sección 6 — ESP32-S3 pines libres).
- No es strapping pin (los strapping son 0, 3, 45, 46).
- No es Flash/PSRAM (esos son 26–37 en N16R8).
- No es USB nativo (esos son 19 y 20).
- Soporta cualquier función vía GPIO matrix del ESP32-S3 — incluido UART.
- Físicamente accesible en módulos DevKitC-1 / N16R8.

Cualquier otro pin libre (1, 2, 6, 7, 17) también funcionaría; **GPIO 16** se elige
por estar físicamente cerca del header lateral en módulos comunes y no ser ADC ni
strapping. **Esta asignación NO altera ningún pin actual del proyecto**.

### 6.3 Compatibilidad detallada

| Recurso del proyecto | ¿Conflicto? | Por qué |
|---|---|---|
| **USB-CDC (Serial monitor)** | ❌ No | UART0 HW está libre porque el monitor va por USB nativo (no por GPIO 43/44 ni por los pines UART0 originales) |
| **UART1 — Obstacle sensor (GPIO 18)** | ❌ No | UART independiente, pin distinto |
| **UART2 — DFPlayer (GPIO 43/44)** | ❌ No | UART independiente, pines distintos |
| **CAN TX/RX (GPIO 4/5)** | ❌ No | Periférico TWAI dedicado |
| **I2C MCP23017 shifter (GPIO 8/9)** | ❌ No | I2C dedicado |
| **SPI TFT + XPT2046 (GPIO 10/12/13/14)** | ❌ No | SPI dedicado |
| **WS2812B (GPIO 47/48)** | ❌ No | RMT dedicado |
| **Audio relay (GPIO 11)** | ❌ No | GPIO output dedicado |
| **Ignition + Power hold (GPIO 40/41)** | ❌ No | GPIO dedicado |
| **Traction switch (GPIO 15)** | ❌ No | GPIO input dedicado |
| **renderTask Core 0** | ❌ No | Core 0 sigue dedicado a TFT/touch. El parser iBUS corre en Core 1, dentro del `loop()` |
| **FreeRTOS scheduler** | ❌ No | El parser es no bloqueante (sin `delay()`); se invoca como una llamada más dentro de `loop()` |
| **CAN_ProcessMessages / heartbeat** | ❌ No | El módulo solo añade frames TX adicionales (0x100/0x101); el código de heartbeat actual no cambia |
| **Watchdog ESP32 (TWDT loopTask)** | ❌ No | Parser no bloqueante — `update()` retorna < 100 µs |

---

## 7. Source arbitration: pedal local vs mando remoto

### 7.1 Comportamiento actual del STM32 (verificado en `Core/Src/main.c:483-541`)

Cada 50 ms el STM32 ejecuta:

```c
if (startup_inhibit)
    Traction_SetDemand(0)
else if (Safety_IsCommandAllowed())   // ACTIVE / DEGRADED
    Traction_SetDemand(Safety_ValidateThrottle(Pedal_GetPercent()))
else if (Safety_IsLimpHome())
    Traction_SetDemand(pedal * 0.20)  // hard clamp 20%
else
    Traction_SetDemand(0)
```

Y en `Core/Src/can_handler.c:1617-1645`, cada vez que llega un frame `0x100`:

```c
if (!Startup_IsInhibited() && requested_pct <= 100.0f) {
    float validated = Safety_ValidateThrottle(requested_pct);
    Traction_SetDemand(validated);
}
```

**Implicación:** ambos caminos llaman a `Traction_SetDemand()`. El último en escribir gana.
Como `CAN_ProcessMessages()` se ejecuta en CADA iteración del `while(1)` (no solo en
tick_50ms), y `Pedal_GetPercent()` se aplica solo cada 50 ms, los frames CAN 0x100
recibidos entre dos ticks de 50 ms tomarán el control durante esos 50 ms — pero serán
sobrescritos por el pedal local en el siguiente tick.

### 7.2 Tres opciones de arbitración (a decidir en fase de implementación)

#### Opción A — **Recomendada**: Source select desde el mando (CH10)

- Cuando CH10 está en posición **LOCAL** (failsafe del mando), el ESP32 **no envía**
  los frames 0x100/0x101. El STM32 funciona exactamente como hoy (pedal local + steering local).
- Cuando CH10 está en posición **REMOTE**, el ESP32 envía 0x100/0x101 a 50 ms.
- **Sigue habiendo competencia** entre pedal local y CAN throttle: si el conductor pisa
  el pedal físico mientras el mando también demanda throttle, el último valor escrito
  durante el ciclo prevalece. El EMA α=0.15 + ramp limiter del motor suavizan cualquier
  oscilación residual.
- **NO requiere modificar el STM32.** Reversible al 100%.

#### Opción B — Cambio mínimo en STM32 (5 líneas en `main.c`)

Añadir flag `can_throttle_override` que se activa al recibir 0x100 y se desactiva tras
200 ms sin frames:

```c
// en can_handler.c (CAN_ID_CMD_THROTTLE handler)
can_throttle_last_rx_ms = HAL_GetTick();
can_throttle_override = 1;

// en main.c tick 50 ms (sustituyendo línea 505)
} else if (Safety_IsCommandAllowed()) {
    if (can_throttle_override &&
        (HAL_GetTick() - can_throttle_last_rx_ms) < 200) {
        // CAN throttle override activo — no sobrescribir
    } else {
        can_throttle_override = 0;
        Traction_SetDemand(Safety_ValidateThrottle(Pedal_GetPercent()));
    }
}
```

Esto da control total al mando sin tocar `safety_system.c`, `motor_control.c`, PWM ni
watchdogs. Es un cambio quirúrgico y reversible.

#### Opción C — `Pedal_GetPercent()` ya retorna max(pedal, throttle_can)

Lógica de "mayor prevalece" implementada en el ESP32 o el STM32. Útil si se quiere que
mando y pedal se sumen — **no recomendado** por riesgo de overshoot al combinar.

### 7.3 Recomendación final

**Empezar por Opción A** durante la fase de validación. Si la coexistencia
pedal/mando resulta inaceptable en bench testing, pasar a **Opción B** como cambio
quirúrgico mínimo.

---

## 8. Capas tocadas vs intactas (resumen final)

### ✅ Se tocaría (cuando llegue la implementación, NO ahora):

| Archivo | Cambio | Líneas estimadas |
|---|---|---|
| `esp32/src/remote_control.h` | NUEVO — interfaz parser iBUS + FSM failsafe | ~80 |
| `esp32/src/remote_control.cpp` | NUEVO — implementación | ~300 |
| `esp32/src/main.cpp` | +3 líneas: include, init(16), update() | 3 |
| `Core/Src/main.c` (Opción B) | +5 líneas: flag `can_throttle_override` con timeout | 5 |
| `esp32/platformio.ini` | Sin cambios | 0 |

### ❌ NO se toca:

| Archivo / sistema | Razón |
|---|---|
| `Core/Src/safety_system.c` | STM32 safety authority intacta |
| `Core/Src/motor_control.c` | Pipeline PWM / EMA / ramp / anomaly detector intacto |
| `Core/Src/can_handler.c` (excepto opcionalmente añadir 1 flag) | Protocolo CAN, IDs y handlers sin cambio funcional |
| TIM1 / TIM8 (PWM 20 kHz BTS7960) | Sin modificación de timers |
| IWDG (~4.1 s) | Watchdog intacto |
| `Core/Src/encoder_reader.c` | Encoder steering sin cambios |
| `Core/Src/steering_centering.c` | Centrado intacto |
| ABS / TCS (`safety_system.c::Safety_GetTractionCapFactor`) | Modulación de seguridad intacta |
| `renderTask` Core 0 (TFT + touch) | Render intacto |
| `Core/Src/sensor_manager.c` (Pedal_*) | Lectura ADC pedal intacta |
| `esp32/src/can_rx.cpp` | Recepción CAN ESP32 sin cambios |
| `esp32/src/power_manager.cpp` | Lógica ignition / power hold intacta |
| `esp32/src/audio_manager.cpp` (DFPlayer + UART2) | Audio intacto |
| `esp32/src/sensors/obstacle_sensor.cpp` (UART1) | Sensor de obstáculo intacto |
| `esp32/src/shifter_input.cpp` (I2C MCP23017) | Palanca de cambios intacta |
| `esp32/src/led_controller.cpp` (WS2812B) | LEDs intactos |

---

## 9. Recomendación final clara y justificada

| Pregunta | Respuesta |
|---|---|
| **¿Qué mando comprar?** | **FlySky FS-i6X** (10 canales nativos, AFHDS 2A, ~45 €) |
| **¿Qué receptor comprar?** | **FlySky FS-iA6B** (con salida iBUS dedicada, suele venir con el FS-i6X en bundle, ~12 € suelto) |
| **¿Protocolo?** | **iBUS Servo** — UART 115200 8N1, 32 bytes / 7 ms, checksum 16-bit |
| **¿UART?** | **HardwareSerial(0)** reasignada (UART0 HW está libre porque Serial va por USB-CDC) |
| **¿GPIO?** | **GPIO 16** como RX (TX no necesario — modo recepción pura). Sin cambios en GPIOs actuales |
| **¿Cambios en STM32?** | **Ninguno** (Opción A) o **5 líneas en main.c** (Opción B si se necesita override) |
| **¿Failsafe?** | 4 capas en cascada: receptor HW → parser SW → CAN heartbeat → IWDG |
| **¿Por qué NO el FS-i6 + FS-iA6 de la foto?** | El FS-iA6 **NO tiene iBUS** — solo PWM y PPM. Forzaría una arquitectura inferior (captura por ISR de 6 GPIOs, sin checksum, susceptible a jitter y EMI) |

### Coste total estimado del hardware adicional

| Item | Coste |
|---|---|
| FlySky FS-i6X + FS-iA6B (bundle) | ~50 € |
| Cable Dupont 3 pines (VCC/GND/SIGNAL) | ~1 € |
| Condensador cerámico 100 nF (filtrado VCC receptor) | ~0,10 € |
| Electrolítico 10 µF / 25 V (estabilización VCC) | ~0,30 € |
| Ferrita snap-on para cable receptor → ESP32 | ~1 € |
| Mini DC-DC buck step-down 5 V (si no hay rail 5 V accesible) | ~3 € |
| **TOTAL** | **~55 €** |

---

## 10. Referencias cruzadas

- `docs/REMOTE_CONTROL_WIRING.md` — esquema eléctrico y routing anti-EMI
- `docs/REMOTE_CONTROL_FAILSAFE.md` — estrategia de failsafe en 4 capas
- `docs/REMOTE_CONTROL_IMPLEMENTATION_PLAN.md` — fases de implementación
- `docs/REMOTE_CONTROL_GPIO_USAGE.md` — verificación GPIO exacto
- `docs/REMOTE_CONTROL_RISK_ANALYSIS.md` — análisis de riesgos completo
- `docs/PIN_USAGE_INVENTORY.md` — inventario actual de pines
- `docs/CAN_CONTRACT_FINAL.md` — contrato CAN existente
- `docs/CONEXIONES_COMPLETAS.md` — cableado proyecto

---

**Estado del documento:** Propuesta validada técnicamente contra el firmware real.
Pendiente decisión de compra y autorización del usuario para pasar a la fase de
implementación.
