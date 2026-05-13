# PLAN DE PUESTA EN MARCHA SEGURA — Paso a Paso Sin Romper Nada

**Proyecto:** STM32G474RE — Control de vehículo eléctrico  
**Fecha:** 2026-04-09  
**Referencia firmware:** `Core/Inc/project_config.h`, `Core/Inc/boot_validation.h`, `Core/Src/main.c`  
**Documentos relacionados:** `MATERIALES_POR_MODULO.md`, `CABLEADO_AISLAMIENTO_DEFINITIVO.md`, `HARDWARE_VALIDATION_PROCEDURE.md`

> ⚠️ **NOTA DE HARDWARE:** Las referencias a **HY-M158** en este documento corresponden
> al módulo de 8 canales PC817 que fue el diseño original. El hardware físico actual usa
> **2× módulos NPN 4 canales (Active Low)** equivalentes.
> Referencia válida: `docs/EL817_WIRING_REFERENCE.md`.
> La lógica eléctrica es idéntica (NPN open-collector, salida Active Low, llave ON = LOW).

> **Filosofía:** Cada paso se verifica de forma aislada ANTES de conectar el siguiente módulo. Nunca conectes todo a la vez. El sistema de seguridad del firmware te protege — el SAFE MODE es tu amigo.

---

## Índice

0. [Fase 0 — Lo que ya tienes funcionando](#fase-0--lo-que-ya-tienes-funcionando)
1. [Fase 1 — Bus I2C: TCA9548A + INA226 de batería](#fase-1--bus-i2c-tca9548a--ina226-de-batería)
2. [Fase 2 — Sensores de temperatura (5× DS18B20)](#fase-2--sensores-de-temperatura-5-ds18b20)
3. [Fase 3 — Sensores INA226 de motores](#fase-3--sensores-ina226-de-motores-4-tracción--1-dirección)
4. [Fase 4 — Encoder de dirección (6N137 obligatorio)](#fase-4--encoder-de-dirección-con-aislamiento-6n137-obligatorio)
5. [Fase 5 — Sensor de centrado + sensores de velocidad de rueda](#fase-5--sensor-de-centrado--sensores-de-velocidad-de-rueda)
6. [Fase 6 — Pedal acelerador (ADC con divisor resistivo)](#fase-6--pedal-acelerador-adc-con-divisor-resistivo)
7. [Fase 7 — Relés de potencia (solo lógica, SIN carga)](#fase-7--relés-de-potencia-solo-lógica-sin-carga)
8. [Fase 8 — Un solo motor (BTS7960 + Motor FL)](#fase-8--un-solo-motor-bts7960--motor-fl--primera-prueba-con-ruedas)
9. [Fase 9 — Restantes 3 motores de tracción + motor de dirección](#fase-9--restantes-3-motores-de-tracción--motor-de-dirección)
10. [Resumen de materiales totales (BOM)](#resumen-de-materiales-totales-bom)
11. [Reglas de oro](#reglas-de-oro)

---

## Fase 0 — Lo que ya tienes funcionando

Según las pruebas anteriores, esto ya funciona:

- ✅ STM32G474RE Nucleo-64 (encendida por USB)
- ✅ ESP32-S3 + Pantalla TFT ST7796 (muestra datos)
- ✅ CAN bus entre STM32 y ESP32 (la pantalla muestra heartbeat, fault flags, etc.)
- ✅ TF-Mini Plus sensor de obstáculos (conectado GPIO18 del ESP32)

**Estado actual:** La pantalla muestra SAFE MODE porque faltan los sensores INA226 (batería). Esto es **correcto y esperado**.

---

## Fase 1 — Bus I2C: TCA9548A + INA226 de batería

**Objetivo:** Conseguir que boot_validation pase `BOOT_CHECK_BATTERY_OK` y el sistema pueda transicionar de SAFE → STANDBY → LIMP_HOME.

**Referencia firmware:**
- `project_config.h:182-183` — `PIN_I2C_SCL` = PB6, `PIN_I2C_SDA` = PB7
- `project_config.h:240` — `I2C_ADDR_TCA9548A` = 0x70
- `project_config.h:241` — `I2C_ADDR_INA226` = 0x40
- `project_config.h:266-267` — Shunt batería = 0.75 mΩ, canal 4
- `boot_validation.h:40` — `BOOT_CHECK_BATTERY_OK`

### Materiales necesarios

| Qty | Componente | Especificación | Precio aprox. |
|-----|-----------|---------------|--------------|
| 1 | TCA9548A breakout board | Adafruit 2717 o genérica (I2C addr 0x70) | ~3€ |
| 1 | INA226 breakout board | GY-INA226 o similar (addr 0x40) | ~3€ |
| 1 | Resistencia shunt | **0.75 mΩ** ±1%, 5W, tamaño 2512 | ~2€ |
| 2 | Resistencia pull-up I2C | **4.7 kΩ** (1/4W) | ~0.10€ |
| 3 | Condensador cerámico | **100 nF** X7R 16V | ~0.30€ |
| 1 | Fuente de alimentación | **24V** banco de potencia o batería | — |

### Conexiones EXACTAS

#### STM32 → TCA9548A

```
STM32 Nucleo-64                TCA9548A
─────────────────              ────────
PB6 (I2C1_SCL) ─────────────→ SCL
PB7 (I2C1_SDA) ─────────────→ SDA
3.3V            ─────────────→ VCC
GND             ─────────────→ GND
GND             ─────────────→ A0 (addr bit 0 = 0)
GND             ─────────────→ A1 (addr bit 1 = 0)
GND             ─────────────→ A2 (addr bit 2 = 0)
                               → Dirección resultante: 0x70 ✅

Pull-ups I2C (OBLIGATORIOS si no están en el breakout):
  - 4.7 kΩ entre SCL (PB6) y 3.3V
  - 4.7 kΩ entre SDA (PB7) y 3.3V
  (Muchos breakout boards ya los incluyen — mide con multímetro
   entre SCL/SDA y VCC: si lees ~4.7kΩ, ya están)
```

#### TCA9548A → INA226 (batería, canal CH4)

```
TCA9548A Canal 4 (CH4)         INA226 (Batería)
──────────────────             ─────────────────
SD4 ─────────────────────────→ SDA
SC4 ─────────────────────────→ SCL
                               VCC → 3.3V
                               GND → GND
                               A0 → GND  (dirección = 0x40)
                               A1 → GND
```

#### INA226 shunt connections

```
INA226 shunt connections:
  - IN+ → cable positivo de la batería 24V (ANTES del relé principal)
  - IN- → cable positivo DESPUÉS de la resistencia shunt de 0.75 mΩ
  - La shunt va EN SERIE con el cable positivo de la batería
  - BUS+ → cable positivo 24V (mide el voltaje del bus)
  - BUS- → GND de batería
```

#### Esquema shunt

```
                    Shunt 0.75 mΩ
Batería 24V+ ──────┤ IN+ ├──┤ IN- ├────→ al relé RELAY_TRAC
                    │  INA226  │
                    │ BUS+ BUS-│
                    │   │    │ │
                    │   └────┘ │
                    │  24V bus │
                    └──────GND─┘
```

### Verificación

1. **Sin alimentación de 24V todavía** — solo USB al STM32
2. Con multímetro en modo continuidad, verifica que **SCL no toca SDA** y que **ninguno toca GND**
3. Enciende el STM32 por USB
4. El LED LD2 (PA5) debería parpadear. Si hace blink 1Hz (500ms ON/500ms OFF) = CAN fail. Si hace flash breve cada 2s (50ms ON/1950ms OFF) = CAN OK
5. Si tienes acceso serial (115200 baud por ST-Link VCP), verifica que no hay errores I2C
6. **Conecta la alimentación de 24V** (o fuente de banco) al shunt → INA226
7. La pantalla del ESP32 debería mostrar voltaje de batería (mensaje CAN 0x207)
8. **Si el voltaje se lee correctamente (≥20V)**, `BOOT_CHECK_BATTERY_OK` pasará y el sistema podrá salir de SAFE

---

## Fase 2 — Sensores de temperatura (5× DS18B20)

**Objetivo:** Pasar `BOOT_CHECK_TEMP_PLAUSIBLE`. Necesario para boot validation completa.

**Referencia firmware:**
- `project_config.h:217` — `PIN_ONEWIRE` = PB0 (open-drain, pull-up)
- `project_config.h:247` — `NUM_DS18B20` = 5
- `boot_validation.h:37` — `BOOT_CHECK_TEMP_PLAUSIBLE`

### Materiales

| Qty | Componente | Especificación |
|-----|-----------|---------------|
| 5 | DS18B20 | Formato TO-92 o encapsulado estanco |
| 1 | Resistencia pull-up | **4.7 kΩ** entre DATA y 3.3V |
| 5 | Condensador cerámico | **100 nF** junto a cada DS18B20 entre VDD y GND |

### Conexión (bus OneWire — TODOS en paralelo)

```
STM32 PB0 (OneWire) ──────────────┬─── DS18B20 #1 (DQ pin 2)
                                   ├─── DS18B20 #2 (DQ pin 2)
                                   ├─── DS18B20 #3 (DQ pin 2)
                                   ├─── DS18B20 #4 (DQ pin 2)
                                   └─── DS18B20 #5 (DQ pin 2)
                                   │
                              4.7 kΩ
                                   │
                                 3.3V

Cada DS18B20:
  Pin 1 (GND) → GND
  Pin 2 (DQ)  → Bus OneWire (PB0)
  Pin 3 (VDD) → 3.3V
  100nF entre pin 3 y pin 1 (junto al sensor)
```

**⚠️ CUIDADO con el pinout DS18B20:**
- Mirando la cara plana del TO-92 de izquierda a derecha: **GND – DQ – VDD**
- Si lo conectas al revés, **se quemará al instante** y huele fatal

### Verificación

1. Conecta primero **solo 1** DS18B20
2. Enciende el STM32 — comprueba que la pantalla del ESP32 muestra una temperatura razonable (~20-25°C ambiente) en el mensaje CAN 0x202
3. Si funciona, añade los otros 4 de uno en uno
4. El firmware detecta cada sensor por su dirección ROM de 64 bits, así que el orden no importa — pero los mapea por el mensaje CAN 0x206

---

## Fase 3 — Sensores INA226 de motores (4× tracción + 1× dirección)

**Objetivo:** Pasar `BOOT_CHECK_CURRENT_PLAUSIBLE`. Estos van en TCA9548A canales CH0-CH3 y CH5.

**Referencia firmware:**
- `project_config.h:246` — `NUM_INA226` = 6
- `project_config.h:265` — `INA226_SHUNT_MOHM_MOTOR` = 1.5 mΩ
- `boot_validation.h:38` — `BOOT_CHECK_CURRENT_PLAUSIBLE`

### Materiales (por cada sensor)

| Qty | Componente | Especificación |
|-----|-----------|---------------|
| 5 | INA226 breakout | GY-INA226, addr 0x40 |
| 5 | Resistencia shunt | **1.5 mΩ** ±1%, 5W, tamaño 2512 |
| 5 | Condensador cerámico | **100 nF** junto a cada INA226 |

### Conexión por canal

```
TCA9548A CH0 → INA226 (Motor FL) → Shunt 1.5mΩ en cable B+ del BTS7960 FL
TCA9548A CH1 → INA226 (Motor FR) → Shunt 1.5mΩ en cable B+ del BTS7960 FR
TCA9548A CH2 → INA226 (Motor RL) → Shunt 1.5mΩ en cable B+ del BTS7960 RL
TCA9548A CH3 → INA226 (Motor RR) → Shunt 1.5mΩ en cable B+ del BTS7960 RR
TCA9548A CH5 → INA226 (Steering) → Shunt 1.5mΩ en cable B+ del BTS7960 STEER
```

**⚠️ NO CONECTES LOS MOTORES TODAVÍA** — solo los INA226 midiendo 0A en un cable sin motor. El firmware acepta 0A como valor plausible al arrancar.

### Verificación

1. Con multímetro mide la resistencia entre IN+ e IN- de cada INA226 — debería dar ~1.5 mΩ (prácticamente cortocircuito)
2. Enciende el sistema — la pantalla debería mostrar corrientes ~0A en CAN 0x201
3. Si los 6 INA226 responden, el `BOOT_CHECK_CURRENT_PLAUSIBLE` pasa ✅

---

## Fase 4 — Encoder de dirección (con aislamiento 6N137 OBLIGATORIO)

**Objetivo:** Pasar `BOOT_CHECK_ENCODER_HEALTHY`. El encoder E6B2-CWZ6C necesita **aislamiento galvánico con 6N137** porque el filtro hardware de TIM2 rechaza flancos más lentos que 282 ns.

**Referencia firmware:**
- `project_config.h:168-170` — `PIN_ENC_A` = PA15, `PIN_ENC_B` = PB3, `PIN_ENC_Z` = PB4
- `boot_validation.h:39` — `BOOT_CHECK_ENCODER_HEALTHY`
- `AISLAMIENTO_GALVANICO_6N137.md` — análisis detallado 6N137 vs PC817

### Materiales

| Qty | Componente | Especificación |
|-----|-----------|---------------|
| 1 | Encoder E6B2-CWZ6C | 1200 PPR, salida NPN open-collector, 5V |
| 2 | Módulo doble 6N137 | Para canales A y B |
| 1 | PC817 (1 canal de la placa de 8) | Para canal Z (índice, baja frecuencia) |
| 2 | Resistencia LED 6N137 | **330 Ω** (lado encoder, 5V) |
| 2 | Resistencia pull-up 6N137 | **1 kΩ** (lado STM32, 3.3V) |
| 1 | Resistencia LED PC817 | **330 Ω** (lado encoder, 5V) |
| 1 | Resistencia pull-up PC817 | **10 kΩ** (lado STM32, 3.3V) |
| 1 | Fuente 5V aislada | Para alimentar el encoder (SEPARADA del 3.3V STM32) |

### Circuito Canal A (6N137) — idéntico para Canal B

```
LADO ENCODER (5V aislado)          │  LADO STM32 (3.3V)
                                   │
Encoder CH_A ──┐                   │
               │                   │       1 kΩ
              330 Ω                │  3.3V──┤
               │                   │        │
               └──→ Pin 2 (Ánodo)  │  Pin 6 (Vo) ──→ PA15 (TIM2_CH1)
                    6N137          │  6N137
  GND_encoder ──→ Pin 3 (Cátodo)  │  Pin 5 (GND_stm32)──→ GND STM32
                                   │  Pin 8 (VCC) ──→ 3.3V STM32
  (Pin 1 NC)                       │  Pin 7 (VE enable) ──→ GND STM32
                                   │
──────────────── BARRERA ──────────│──────────────────────
    GND encoder ≠ GND STM32       │  (masas separadas)
```

**⚠️ La salida del 6N137 es ACTIVE-LOW (invertida).** El encoder NPN open-collector + 6N137 produce: encoder activo → LED ON → salida LOW. El TIM2 en modo cuadratura acepta esto sin problema porque cuenta flancos, no niveles.

### Circuito Canal Z (PC817)

```
LADO ENCODER (5V)                  │  LADO STM32 (3.3V)
                                   │
Encoder CH_Z ──┐                   │
               │                   │       10 kΩ
              330 Ω                │  3.3V──┤
               │                   │        │
               └──→ Pin 1 (Ánodo)  │  Pin 4 (Colector) ──→ PB4 (EXTI4)
                    PC817          │  PC817
  GND_encoder ──→ Pin 2 (Cátodo)  │  Pin 3 (Emisor) ──→ GND STM32
```

### Verificación

1. **SIN motor de dirección conectado**, gira manualmente el eje del encoder
2. Con un debugger o serial, verifica que `TIM2->CNT` cambia al girar
3. Alternativamente, la pantalla ESP32 mostrará el ángulo de dirección en CAN 0x204

---

## Fase 5 — Sensor de centrado + sensores de velocidad de rueda

**Objetivo:** Completar la instrumentación de baja frecuencia. Todos usan **PC817** para aislamiento.

**Referencia firmware:**
- `project_config.h:134-137` — Wheel speed pins (PA0, PA1, PA2, PB15)
- `project_config.h:177` — `PIN_STEER_CENTER` = PB5
- `project_config.h:156` — `WHEEL_MAX_FREQ_HZ` = 200 Hz

### Materiales (módulo HY-M158, PC817 ×8)

| Qty | Componente | Especificación |
|-----|-----------|---------------|
| 5 | Canales del **HY-M158** | Canales `IN1`–`IN5` (4 ruedas + llave) |
| 4 | Sensores inductivos LJ12A3-4-Z/BX | NPN NO, 6-36V, para ruedas |
| ~~1~~ | ~~Sensor LJ12A3 centro dirección~~ | **No se cablea por este módulo** — función cubierta por el ENC_Z aislado por 6N137 |
| ~~5~~ | ~~Resistencia LED 1 kΩ~~ | **No necesaria** — la HY-M158 ya integra **3 kΩ SMD `302`** en serie con cada LED del PC817 |
| 4 | Resistencia pull-up | **4× 4.7 kΩ 1/8 W** entre `IN1`–`IN4` (pull-up rápido para ruedas) y 3.3 V. Obligatorias: el HY-M158 NO tiene pull-up onboard |
| 1 | Resistencia pull-up | **1× 10 kΩ 1/8 W** entre `IN5` (llave) y 3.3 V |
| — | **Jumpers rojos** | **Quitar los 8** — cortocircuitan `G(V)` con `G(IN)` y anulan el aislamiento galvánico |

> **Por qué la 3 kΩ integrada del HY-M158 es suficiente para 12 V:**
> `I_LED = (12 V − 1.2 V) / 3 kΩ = 3.6 mA` — dentro del rango nominal del PC817
> (1–20 mA). Con CTR ≥ 50 % el fototransistor satura contra el pull-up de 4,7 kΩ a 3,3 V.
> La idea anterior de pedir 1 kΩ externa partía del supuesto de que el módulo no traía
> resistencia. Ya no aplica.

### Circuito tipo (para cada sensor inductivo LJ12A3-NPN-NO → HY-M158 → STM32)

> **Importante — cableado del LJ12A3 NPN open-collector:** la salida (cable negro) **sólo puede sumir corriente a GND** cuando detecta metal; nunca entrega tensión positiva. Por eso el LED del PC817 se alimenta desde +12 V a través de la resistencia limitadora **integrada en el HY-M158**, y la salida NPN se conecta al **cátodo** del LED (lado `G` del bloque V). De este modo el LED conduce cuando el sensor detecta metal y la NPN cierra a GND.

```
LADO SENSOR (12 V vehículo)            │  LADO STM32 (3.3 V)
                                       │
+12V_sensor ──┬──► Marrón LJ12A3       │  4.7 kΩ (externo, obligatorio — NO onboard)
              │                        │  3.3V ──┤
              └─► HY-M158 V_n          │          │
                  (3 kΩ on-board       │          │
                   en serie con LED)   │  HY-M158 IN_n (Colector) ──► PAx / PBx (EXTI)
                                       │
LJ12A3 Negro ──► HY-M158 G (lado V)    │  HY-M158 G (lado IN) ──► GND STM32
(salida NPN)                           │
                                       │  (jumpers rojos retirados → barrera galvánica intacta)
GND_sensor ──► Azul LJ12A3             │
```

**Lógica resultante:** sensor detecta metal → NPN cierra → LED conduce → fototransistor del PC817 satura → **EXTI ve flanco de bajada (LOW)**. Es la convención que ya espera el firmware (`wheel_speed.c` cuenta flancos sin importar polaridad).

### Asignación de pines (HY-M158, de abajo a arriba — empezando por `IN1`)

| Sensor | Pin STM32 / ESP32 | EXTI | Canal HY-M158 |
|--------|-------------------|------|---------------|
| Rueda **FR** | PA1 | EXTI1 | **`IN1`** (abajo) |
| Rueda **FL** | PA0 | EXTI0 | **`IN2`** |
| Rueda **RR** | PB15 | EXTI15 | **`IN3`** |
| Rueda **RL** | PA2 | EXTI2 | **`IN4`** |
| **IGNITION (ESP32)** | GPIO 40 ESP32-S3 | — | **`IN5`** (llave de contacto) |
| *(libres — reserva)* | — | — | `IN6` / `IN7` / `IN8` |

> Centro de dirección (PB5): no se cablea por este módulo en la configuración acordada.
> ENC_Z (PB4): movido al **6N137** junto con A y B (ver `docs/ENCODER_WIRING_6N137.md`).

### Verificación

1. Conecta **solo 1 sensor de rueda** primero
2. Acerca un objeto metálico al sensor — el LED del PC817 debe encenderse
3. La pantalla ESP32 debería registrar pulsos en CAN 0x200 (wheel speeds)
4. Repite para los otros 4 sensores

---

## Fase 5b — Llave de contacto (IGNITION) vía PC817 al ESP32-S3

**Objetivo:** Aislar galvánicamente la línea de llave de +12 V del bus 3.3 V del ESP32-S3 reutilizando un canal libre de la placa PC817 ya instalada.

**Referencia firmware:** `esp32/src/power_manager.{h,cpp}` — `PIN_IGNITION_SENSE = GPIO 40`.

### ⚠️ Lógica INVERTIDA

El PC817 invierte la señal:

| Estado de la llave | LED PC817 | Fototransistor | GPIO 40 ESP32 |
|---|---|---|---|
| **OFF** | Apagado | Abierto | **HIGH** (pull-up externo 10 kΩ + INPUT_PULLUP ~45 kΩ) |
| **ON**  | Conduce | Saturado | **LOW** |

El firmware `power_manager.cpp` ya contempla esta inversión (`raw = (digitalRead == LOW)`). **No tocar** el firmware salvo si se cambiase la topología del aislamiento.

### Materiales

| Qty | Componente | Especificación |
|-----|-----------|---------------|
| 1 | Canal libre HY-M158 | Canal **`IN5`** (junto a las 4 ruedas en `IN1`–`IN4`) |
| ~~1~~ | ~~Resistencia limitadora LED 1 kΩ~~ | **No necesaria** — la HY-M158 ya integra **3 kΩ SMD `302`** en serie con el LED |
| 1 | Fusible en línea | **1 A rápido** entre ACC/IGN del bombín y la entrada `V5` del HY-M158 |
| 1 | Cable apantallado o trenzado | 0.5 mm², desde la llave hasta el módulo |
| 1 | Resistencia pull-up | **1× 10 kΩ 1/8 W** entre `IN5` y 3.3 V — **obligatoria**: el HY-M158 NO tiene pull-up onboard |

### Circuito

```
LADO 12 V VEHÍCULO                    │  LADO 3.3 V (común STM32 ↔ ESP32)
                                      │
+12V IGNITION ──[1 A]──► HY-M158 V5   │  10 kΩ (externo, ver §pull-up)
                         (3 kΩ on-    │  3.3V ─┤
                          board en    │        │
                          serie con   │  HY-M158 IN5 (Colector PC817) ──► GPIO 40 ESP32
                          el LED)     │
GND_vehículo ──► HY-M158 G (lado V)   │  HY-M158 G (lado IN) ──► GND ESP32 (= GND STM32)
                                      │
─────────── BARRERA GALVÁNICA ────────│──────────────────────────────────────
        (jumpers rojos retirados)     │
```

### Verificación de pull-up onboard — **RESULTADO: NO HAY PULL-UP**

> **✅ Medición realizada.** Se midió con polímetro entre el pin OUT y el pin VCC del lado de salida de la placa PC817 (placa desalimentada) → **circuito abierto / infinito**. La placa **NO** tiene pull-up onboard en ningún canal.

**Acción obligatoria — doble pull-up (hardware + firmware):**

| Medida | Qué | Dónde | Por qué |
|---|---|---|---|
| **Hardware (obligatorio)** | Soldar **10 kΩ ¼ W** entre GPIO 40 y pin 3.3 V del ESP32-S3 | En la PCB o en el propio cable | Pull-up bajo para señal limpia y buena inmunidad al ruido en entorno automoción |
| **Firmware (red de seguridad)** | `pinMode(GPIO40, INPUT_PULLUP)` — ya en `power_manager.cpp` | Código | Previene flotante si el resistor externo no está aún montado o falla la conexión (~45 kΩ interno ESP32-S3) |

Sin ninguno de los dos, la salida del colector del PC817 **flota** cuando la llave está OFF → lecturas erráticas → falsos arranques/apagados.

> **Nota sobre pull-ups en todos los demás canales:** los canales de ruedas usan el mismo módulo HY-M158 sin pull-up onboard, por lo que **también** requieren resistencia externa: **4× 4.7 kΩ entre `IN1`–`IN4` y 3.3 V** (`IN1`=FR/PA1, `IN2`=FL/PA0, `IN3`=RR/PB15, `IN4`=RL/PA2). Verificar igualmente con polímetro.

> **HY-M158 — comparte `G` único en cada lado para los 8 canales.** Como GND_STM32 = GND_ESP32 (ya unidas por CAN/UART), el lado de salida del HY-M158 puede compartirse sin problema entre ambas MCUs. La barrera galvánica sigue intacta porque sólo separa el dominio +12 V del vehículo del dominio 3.3 V común — **siempre que se hayan retirado los 8 jumpers rojos del módulo**.

### Cálculo de corriente del LED (con la 3 kΩ integrada del HY-M158)

```
I_LED = (V_BAT - V_F) / R_serie_on-board
      = (12 V - 1.2 V) / 3 000 Ω
      = 3.6 mA  → ✅ rango nominal PC817 (1–20 mA), CTR ≥ 50 % satura el fototransistor
```

| R_serie efectiva | I_LED a 12 V | Veredicto |
|---|---|---|
| 330 Ω | ~32.7 mA continuos | ❌ Excede I_F nominal — irrelevante con HY-M158 (ya integra 3 kΩ) |
| 1 kΩ | 10.8 mA | ✅ Histórica (cuando se asumía módulo sin R) |
| **3 kΩ (on-board HY-M158)** | **3.6 mA** | ✅ **Configuración real del módulo** — sin componentes externos |
| 2.2 kΩ + 3 kΩ on-board (= 5.2 kΩ a 12 V) | 2.1 mA | ⚠️ No añadir resistencia externa adicional |

### Verificación funcional

1. Llave en **OFF** → `digitalRead(40) == HIGH` (multímetro: ~3.3 V) → estado `OFF`.
2. Gira llave a **ON** → `digitalRead(40) == LOW` (multímetro: ~0.2 V) → en ≤50 ms (debounce) entra en `POWER_HOLD` → `STARTING` → `RUNNING`.
3. Gira llave a **OFF** → en ≤50 ms (debounce) entra en `SHUTTING_DOWN` durante 3 s (audio de despedida) → `OFF`. Verifica que el relé de retardo de 12 V mantiene la alimentación durante > 3 s.
4. **No deben aparecer rebotes**: el debounce (`DEBOUNCE_MS = 50`) absorbe el rebote típico de una llave (≤20 ms).

---

## Fase 6 — Pedal acelerador (ADC con divisor resistivo)

**Objetivo:** Tener input de acelerador para poder mover motores.

**Referencia firmware:**
- `project_config.h:222-229` — `PIN_PEDAL` = PA3 (ADC1_IN4), divisor 10kΩ + 6.8kΩ

### Materiales

| Qty | Componente | Especificación |
|-----|-----------|---------------|
| 1 | Sensor Hall SS1324LUA-T | Alimentación 5V, salida 0.3-4.8V |
| 1 | Resistencia | **10 kΩ** (R_high — entre salida pedal y PA3) |
| 1 | Resistencia | **6.8 kΩ** (R_low — entre PA3 y GND) |
| 1 | Condensador cerámico | **100 nF** entre PA3 y GND (filtro EMI) |
| 1 | Cable apantallado | 28 AWG, para la señal del pedal |

### Circuito divisor

```
Pedal Hall 5V out ──── 10 kΩ ────┬────→ PA3 (ADC1_IN4)
                                  │
                                6.8 kΩ
                                  │
                                 GND

Factor: 6.8/(10+6.8) = 0.404
Max: 4.8V × 0.404 = 1.94V (seguro para 3.3V ADC)
Min: 0.3V × 0.404 = 0.12V
```

**⚠️ IMPORTANTÍSIMO:** El cable del pedal debe ir **apantallado** (malla a GND en un solo extremo) y **separado de los cables de potencia** de los motores. El ruido PWM de 20 kHz puede inducir falsos valores.

### Verificación

1. Sin motores conectados, mueve el pedal
2. La pantalla ESP32 debería mostrar el porcentaje de acelerador
3. El firmware tiene detección de pedal "stuck-high" y "stuck-low" — si el pedal no vuelve a 0%, el sistema lo detecta

---

## Fase 7 — Relés de potencia (solo lógica, SIN carga)

**Objetivo:** Verificar que los relés conmutan antes de conectar motores.

**Referencia firmware:**
- `project_config.h:116-118` — `PIN_RELAY_TRAC` = PC11, `PIN_RELAY_STEER_PWR` = PC12
- PC10 está DISPONIBLE (sin uso)

### Materiales

| Qty | Componente | Especificación |
|-----|-----------|---------------|
| 3 | Módulo relé con optoacoplador | Módulo 4-ch SRD-12VDC-SL-C (12V, trigger 3.3V) |
| 3 | Cable señal | 24 AWG, del STM32 al módulo relé |

### Conexiones

```
PC11 (RELAY_TRAC) ──→ Módulo relé #1 IN (relé tracción 24V)
PC12 (RELAY_DIR)  ──→ Módulo relé #2 IN (relé dirección 12V)

Todos los módulos:
  VCC → 5V externo (NO del STM32 — los relés consumen 70mA cada uno)
  GND → GND común con STM32
```

> **PC10 está DISPONIBLE** — GPIO libre, no conectado (`INPUT_PULLDOWN`).

**Secuencia del firmware:**
1. RELAY_TRAC ON → espera 50ms
2. RELAY_DIR ON → sistema listo

### Verificación

1. **SIN NADA conectado a la salida de los relés**, enciende el sistema
2. Deberías escuchar los relés hacer "clic" en secuencia al arrancar
3. Con multímetro en modo continuidad, verifica que los contactos cierran
4. Si el sistema está en LIMP_HOME (CAN OK + boot validation pasada), los relés se activarán automáticamente

---

## Fase 8 — Un solo motor (BTS7960 + Motor FL) — PRIMERA PRUEBA CON RUEDAS

**⚠️ RUEDAS EN ALTO (coche en caballetes/gatos). NUNCA en el suelo la primera vez.**

**Referencia firmware:**
- `project_config.h:71-72` — `PIN_PWM_FL` = PA8 (TIM1_CH1), `PIN_LPWM_FL` = PA9 (TIM1_CH2)
- `project_config.h:107` — `PIN_EN_FL` = PC5 (GPIO output, active HIGH)

### Materiales para 1 motor

| Qty | Componente | Especificación |
|-----|-----------|---------------|
| 1 | BTS7960 IBT-2 módulo | Driver H-bridge 43A |
| 1 | Condensador electrolítico | **470 µF / 35V** 105°C baja ESR (entre B+ y GND del BTS7960) |
| 1 | Condensador cerámico | **100 nF / 50V** X7R (entre B+ y GND, junto al IC) |
| 1 | Condensador cerámico | **100 nF / 50V** X7R (entre VCC lógico y GND lógico) |
| 1 | Condensador cerámico | **100 nF / 50V** X7R (entre M+ y M- del motor) |
| 1 | Fusible slow-blow | **30A** + portafusible |
| 1 | Motor DC 24V | Motor de tracción del coche |

### Conexiones BTS7960 FL

```
SEÑALES (24 AWG, SEPARADAS de potencia):
  PA8  ──→ RPWM (avance)
  PA9  ──→ LPWM (retroceso)
  PC5  ──→ R_EN + L_EN (puenteados juntos)
  3.3V ──→ VCC lógico del módulo IBT-2
  GND  ──→ GND lógico

POTENCIA (14 AWG / 2.5mm² mínimo):
  Batería 24V+ → Fusible 30A → Shunt 1.5mΩ INA226 → B+ del BTS7960
  Batería 24V- → B- del BTS7960
  Motor terminal 1 → M+ del BTS7960
  Motor terminal 2 → M- del BTS7960

CONDENSADORES (soldados directamente en el módulo):
  470µF/35V entre B+ y B- (lo más cerca posible del módulo)
  100nF/50V entre B+ y B- (junto al IC BTS7960)
  100nF/50V entre VCC lógico y GND lógico
  100nF/50V entre bornes del motor (en el conector del motor, no en el driver)
```

### Verificación

1. **Coche en alto, rueda FL libre**
2. Enciende el sistema — espera a que arranque (LIMP_HOME o ACTIVE)
3. Pisa el pedal **muy suavemente** (< 10%)
4. La rueda FL debería girar lentamente
5. Suelta el pedal — la rueda debe parar
6. Verifica que la corriente en la pantalla (CAN 0x201) es razonable (< 10A sin carga)
7. Toca el BTS7960 con cuidado — no debería estar caliente
8. **Si algo huele a quemado o el STM32 se reinicia → DESCONECTA inmediatamente**

---

## Fase 9 — Restantes 3 motores de tracción + motor de dirección

Repite el mismo proceso de la Fase 8 para cada motor:

| Motor | Pins PWM | Pin EN | Timer | Referencia firmware |
|-------|----------|--------|-------|---------------------|
| FR | PA10 (RPWM), PC3 (LPWM) | PC0 (EN) | TIM1 CH3/CH4 | `project_config.h:73-74,106` |
| RL | PC6 (RPWM), PC7 (LPWM) | PC1 (EN) | TIM8 CH1/CH2 | `project_config.h:78-79,107` |
| RR | PC8 (RPWM), PC9 (LPWM) | PC13 (EN) | TIM8 CH3/CH4 | `project_config.h:80-81,110` |
| STEER | PA6 (RPWM), PA7 (LPWM) | PC4 (EN) | TIM3 CH1/CH2 | `project_config.h:86-87,111` |

**Todos los motores tienen EN dedicado por GPIO** — el firmware controla la habilitación.
Conecta R_EN + L_EN de cada BTS7960 al pin EN correspondiente (PC0, PC1, PC4, PC5, PC13).

**Motor de dirección:** Alimentación **12V** (no 24V) a través de RELAY_DIR (PC12).

### Verificación secuencial

1. Añade **un motor a la vez**
2. Después de añadir cada motor, haz la prueba del pedal
3. Si todo funciona con los 4 de tracción, prueba la dirección (la pantalla ESP32 envía comandos de dirección por CAN 0x101)

---

## Resumen de materiales totales (BOM)

| Qty | Componente | Para qué |
|-----|-----------|----------|
| 1 | TCA9548A breakout | Multiplexor I2C |
| 6 | INA226 breakout | Corriente: 4 motores + 1 batería + 1 dirección |
| 5 | Shunt 1.5 mΩ 5W | Motores (project_config.h:265) |
| 1 | Shunt 0.75 mΩ 5W | Batería (project_config.h:266) |
| 5 | DS18B20 | Temperatura motores |
| 1 | E6B2-CWZ6C | Encoder dirección |
| 2 | Módulo 6N137 doble | Encoder A/B (aislamiento rápido) |
| 1 | Placa PC817 8ch | Sensores lentos (ruedas, centro, encoder Z) |
| 5 | LJ12A3-4-Z/BX | 4 velocidad rueda + 1 centro dirección |
| 1 | SS1324LUA-T | Pedal Hall |
| 5 | BTS7960 IBT-2 | 4 tracción + 1 dirección |
| 1 | Módulo 4-ch opto relé | SRD-12VDC-SL-C (Main + Trac + Dir) |
| 5 | Fusible 30A slow-blow | 4 motores + 1 dirección |
| ~30 | Condensador 100nF | Desacoplo general |
| 5 | Condensador 470µF/35V | Bulk BTS7960 tracción |
| 1 | Condensador 470µF/25V | Bulk BTS7960 dirección |
| ~10 | Resistencias varias | Pull-ups, divisores (4.7k, 10k, 6.8k, 330Ω, 1k) |
| 1 | Fuente 5V aislada | Encoder + sensores |

---

## Reglas de oro

1. **Nunca conectes potencia (24V) sin condensadores de desacoplo ya soldados**
2. **Cables de señal (24 AWG) SIEMPRE separados de cables de potencia (14 AWG)** — mínimo 5 cm de separación
3. **Un módulo nuevo a la vez** — enciende, verifica, apaga, siguiente
4. **Coche SIEMPRE en alto** hasta que todo funcione individualmente
5. **Fusibles en cada línea de potencia** — si algo se cortocircuita, el fusible salta antes de que se queme algo caro
6. **Las masas (GND) de señal y potencia deben unirse en UN SOLO PUNTO** (star ground) — nunca en múltiples puntos
7. **El aislamiento galvánico (6N137/PC817) protege al STM32** de picos de los motores — no lo saltes
