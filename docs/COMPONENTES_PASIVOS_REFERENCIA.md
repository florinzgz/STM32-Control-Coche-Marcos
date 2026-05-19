# 🔌 Referencia Completa de Componentes Pasivos

> **NOTA PARA FUTURAS SESIONES:** Este archivo es la referencia canónica de todos los
> condensadores, resistencias y componentes de protección del sistema. Cuando el usuario
> pida instrucciones de montaje o conexión, consultar este documento primero para recordar
> qué va dónde y por qué.

---

## 📋 Índice

1. [Condensadores Electrolíticos](#1-condensadores-electrolíticos) — 470 µF / 1000 µF (35V y 50V) / 10 µF / **10 µF 16V (MCP23017)**
2. [Condensadores Cerámicos](#2-condensadores-cerámicos)
3. [Condensadores de Polipropileno (Snubbers)](#3-condensadores-de-polipropileno-snubbers)
4. [Resistencias](#4-resistencias)
5. [Diodos de Protección](#5-diodos-de-protección)
6. [Transistores de Control de Relés](#6-transistores-de-control-de-relés)
7. [Mapa Visual del Sistema](#7-mapa-visual-del-sistema)
8. [Reglas de Montaje](#8-reglas-de-montaje)
9. [Convertidores DC-DC Aislados](#9-convertidores-dc-dc-aislados) — **B0505S-1W** (aislamiento bus CAN)

---

## 1. Condensadores Electrolíticos

### 1.1 → 470 µF / 35V / 105°C — **10 unidades compradas**

**¿Por qué 35V?** El bus es de 24V; la regla es usar ≥1.5× la tensión de trabajo (24V × 1.5 = 36V → el de 35V es válido con el margen de diseño).  
**¿Por qué 105°C?** Están cerca de los BTS7960 que disipan calor. Los de 85°C envejecen rápido en ambientes cálidos.

| # | Ubicación exacta | Conexión | Función |
|---|-----------------|----------|---------|
| 1 | BTS7960 Motor FL | Entre pin **B+** y **GND** del módulo, lo más cerca posible del chip | Reserva local de energía. Entrega los picos de corriente de arranque (10–30A en µs) sin que el bus de 24V se desplome |
| 2 | BTS7960 Motor FR | Ídem | Ídem |
| 3 | BTS7960 Motor RL | Ídem | Ídem |
| 4 | BTS7960 Motor RR | Ídem | Ídem |
| 5 | BTS7960 Motor DIR (dirección) | Ídem (35V válido para bus de 24V; también sustituye al de 25V especificado en el DIR) | Ídem |
| 6 | Bus 24V principal | Entre el rail de **24V y GND** en el punto de entrada a la PCB, **después del fusible** y del P-MOSFET de antipolaridad | Filtro bulk del bus: absorbe transitorios de encendido y picos de demanda simultánea de todos los BTS7960 |
| 7–10 | **Stock / recambios** | — | Reserva de sustitución; son los componentes más sometidos a estrés térmico |

**Orientación:** Electrolítico polarizado → patilla **larga (+)** al rail positivo, **corta (–) o banda blanca** a GND.

---

### 1.2 → 1000 µF / 35V y 1000 µF / 50V — **2 + 10 unidades**

**Verificación del componente recibido:** en la foto aportada se lee la etiqueta
`50V1000UF-13x25` (bolsa de **10 uds**), por tanto se añade como variante compatible
del mismo valor de capacidad.

**¿Para qué sirve?** Igual que el 1000 µF/35V: condensador bulk de bus para reducir
caídas de tensión e inrush en el rail de 24V (cierre de relé y picos de demanda de motores).

**¿Dónde va?** Mismo punto que el 1000 µF/35V: bus 24V principal, en paralelo con el
bulk existente (470 µF).

| # | Ubicación exacta | Conexión | Función |
|---|-----------------|----------|---------|
| 1 | Bus 24V principal | En **paralelo** con el 470 µF del bus principal (mismo punto, mismos pines) | Aumenta la capacidad bulk total del bus. A mayor capacidad, menor caída de tensión en arranques simultáneos: ΔV = I·Δt / C |
| 2 | **Stock / recambio** | — | Reserva (válido tanto para 35V como para 50V) |

---

### 1.3 → 1000 µF / 10V — **2 unidades compradas**

| # | Ubicación exacta | Conexión | Función |
|---|-----------------|----------|---------|
| 1 | Alimentación tiras LED WS2812B (primera tira) | Entre **5V (o 12V según tensión LED) y GND** en el conector de entrada de la tira, lo más cerca posible del primer LED | Amortigua los transitorios de corriente pulsada de los WS2812B. Sin él, cuando todos los LEDs cambian de color simultáneamente la caída de tensión corrompe el protocolo de datos |
| 2 | Alimentación tiras LED WS2812B (segunda tira) | Ídem para la segunda tira | Ídem |

> ⚠️ **Importante WS2812B:** Además de este condensador, conectar una resistencia de **300–500 Ω** en serie en la línea de datos (DATA), lo más cerca posible del primer LED, para proteger la entrada del IC.

---

### 1.4 → 10 µF / 10V — **1 referencia × 2 unidades físicas**

| # | Ubicación exacta | Conexión | Función |
|---|-----------------|----------|---------|
| 1 | STM32G474RE — pin **VDD / VDDA** | Entre **3.3V y GND** junto al STM32, en paralelo con los cerámicos de 100 nF | Desacoplo de media frecuencia (1 kHz – 100 kHz). Los ADC internos del STM32 son muy sensibles al ruido de la propia alimentación; sin este condensador las lecturas del pedal y sensores tienen jitter excesivo |
| 2 | ESP32-S3 / TJA1051 — pin **VCC** | Entre **3.3V y GND** junto al ESP32 o al transceptor CAN | Ídem: estabiliza la alimentación del ESP32 (que tiene picos cuando transmite WiFi/BT) y del transceptor CAN |

---

### 1.5 → 10 µF / 16V — **1 unidad (MCP23017 VDD)**

**¿Por qué 16V y no 10V?** Aunque el rail es 3.3V, los electrolíticos trabajan mejor (menor ESR, mayor vida útil) cuando la tensión de trabajo es ≤1/3 de la nominal. Con 16V el margen es suficiente para pasar a VDD=3.3V y es el valor comercial estándar por encima del 10V con disponibilidad garantizada.

**¿Por qué hace falta aquí?** El ESP32-S3 tiene WiFi/BT integrado. Cuando la radio transmite, genera picos de corriente de 200–400 mA en el rail de 3.3V que duran decenas de microsegundos. Sin un condensador bulk local en el MCP23017, ese hundimiento del rail provoca lecturas I²C erróneas o pérdida de ACK, haciendo que el driver entre en backoff y la palanca se quede en PARK de forma espuria. El 100 nF cerámico ya presente cubre las frecuencias altas (>1 MHz), pero es insuficiente para la energía de esos picos de WiFi/BT; el 10 µF electrolítico cubre la franja 1 kHz–1 MHz y actúa como reserva de carga local.

| # | Ubicación exacta | Conexión | Función |
|---|-----------------|----------|---------|
| 1 | **MCP23017** — pin **VDD** (pin 9) | Entre **3.3V y GND** (pin 10 VSS), en paralelo con el 100 nF cerámico, lo más cerca posible del IC | Reserva de carga bulk para los picos de corriente del radio WiFi/BT del ESP32-S3. Evita el hundimiento del rail 3.3V que causa lecturas I²C erróneas en el expansor de GPIO de la palanca de cambios |

**Esquema de conexión:**

```
3.3V ──┬──────┬── VDD (pin 9 del MCP23017)
       │      │
     10µF   100nF   ← el 100nF cerámico ya está montado
       │      │
GND ───┴──────┴── VSS (pin 10 del MCP23017)
```

**Orientación:** Electrolítico polarizado → patilla **larga (+)** a 3.3V, **corta (–) o banda blanca** a GND.

---

## 2. Condensadores Cerámicos

### 2.1 → 100 nF / 50V cerámico X7R 0805 — **30 unidades compradas**

**¿Por qué X7R?** Mantiene la capacidad estable con temperatura (-55°C a +125°C).  
**¿Por qué 50V?** Los BTS7960 trabajan en un bus de 24V con picos; el margen de 50V es de seguridad para los transitorios rápidos.  
**¿Por qué 0805?** Encapsulado SMD soldable a mano cómodamente.

| Cantidad | Ubicación exacta | Conexión | Función |
|----------|-----------------|----------|---------|
| 1 por BTS7960 (×5 = **5 uds**) | Pin **B+** y **GND** de cada BTS7960, en **paralelo** con el electrolítico de 470 µF | En paralelo con el electrolítico (mismos puntos) | Bypass de alta frecuencia. Los electrolíticos son ineficaces >1 MHz por su inductancia serie parásita (ESL). El cerámico filtra los picos de conmutación PWM (20 kHz – varios MHz) |
| 1 por BTS7960 (×5 = **5 uds**) | Pin **VCC (lógica 5V)** y **GND** de cada BTS7960 | Entre VCC del pin de lógica y GND | Desacoplo de la alimentación lógica del BTS7960; evita que el ruido del lado de potencia entre al lado de control |
| 2 por motor (×5 = **10 uds**) | Directamente en el **conector del motor** (M+ y M–) | Uno entre M+ y GND, otro entre M– y GND (o un único entre M+ y M–) | Absorbe el ruido EMI de las escobillas del motor brushed en la fuente misma, antes de que se propague por los cables a la PCB y afecte ADC, encoders o bus CAN |
| **10 uds** | **Stock / margen** | — | Reserva para STM32, ESP32, sensores adicionales según necesidad |

---

### 2.2 → 100 nF / 16V cerámico X7R — **10 unidades compradas**

**¿Por qué 16V?** La lógica trabaja a 3.3V / 5V; 16V da margen suficiente y estos condensadores son más pequeños/baratos que los de 50V.

| Cantidad | Ubicación exacta | Conexión | Función |
|----------|-----------------|----------|---------|
| 1 por pin VDD del STM32 (**2–4 uds**) | Junto al STM32, un condensador por cada pin **VDD** y **VDDA** | Entre cada pin VDD y el GND más cercano, lo más pegado al pin posible | Desacoplo HF del microcontrolador. Cada instrucción ejecutada genera picos de corriente de nanosegundos; sin esto el ruido entra en los ADC y genera lecturas erróneas del pedal |
| **2 uds** | ESP32-S3 y/o TJA1051 | Entre **VCC y GND** de cada IC | Desacoplo HF del ESP32 y del transceptor CAN |
| **2 uds** | Circuito del **pedal de acelerador** | Uno en el divisor de tensión del pedal (entre la salida del divisor y GND), uno en la entrada del ADC del STM32 | Forma un filtro RC paso-bajo con la resistencia del divisor. Elimina el jitter mecánico del potenciómetro/Hall y el ruido eléctrico antes del ADC. Sin él la lectura puede tener ±50–100 LSB de ruido |
| **2 uds** | Sensor **ToF VL53L1X** (o TFMini/ToFSense) | Entre pin **AVDD y GND** del sensor | El ToF tiene consumo pulsado cuando dispara el láser IR. El 100 nF local evita que ese pulso de corriente afecte al bus I²C o a otros periféricos |
| **2 uds** | **Stock / margen** | — | Reserva |

---

## 3. Condensadores de Polipropileno (Snubbers)

### 3.1 → 100 nF / 250V polipropileno — **5 unidades compradas**

**¿Por qué polipropileno?** Tiene muy baja ESR, no se polariza y aguanta los picos de tensión transitorios de 250–500V que generan los arcos en los contactos del relé.  
**¿Por qué no cerámico o electrolítico?** El cerámico X7R pierde capacidad a alta tensión (piezoelectricidad inversa); el electrolítico es polarizado y demasiado lento.  
**¿Por qué 250V?** En un bus de 24V con carga inductiva, el pico transitorio puede llegar a 4–10× la tensión nominal (96–240V). El margen de 250V es de seguridad obligatorio.

**Configuración: Red RC snubber**

Cada unidad se monta **en serie con una resistencia de 100 Ω / 0.5W** formando una red RC:

```
Contacto COM ─── [100 Ω / 0.5W] ─── [100nF/250V] ─── Contacto NC/NO
```

Esta red RC se coloca **en paralelo con los contactos de potencia** del relé (NO con la bobina).

| # | Relé | Ubicación | Función |
|---|------|-----------|---------|
| 1 | RELAY_TRAC (Tracción) | En paralelo con los contactos COM–NO del relé | Al abrir el relé, la inductancia del cableado genera un arco. El snubber RC absorbe la energía del spike (que puede llegar a 300–500V en 24V inductive) protegiendo los contactos y eliminando la EMI que afecta al STM32 y al bus CAN |
| 2 | RELAY_TRAC (Tracción) | Ídem | Ídem; este relé conmuta la mayor corriente (50A), por lo que el arco es más severo |
| 3 | RELAY_DIR (Dirección) | Ídem | Ídem |
| 4 | Relé extra #4 | Ídem | Ídem |
| 5 | Relé extra #5 | Ídem | Ídem |

> **Tip de montaje:** Los cables del snubber deben ser lo más cortos posible (<5 cm). Los cables largos añaden inductancia que reduce la eficacia del snubber.

---

## 4. Resistencias

### 4.1 Terminación CAN — 120 Ω / 0.25W

| # | Ubicación | Conexión | Función |
|---|-----------|----------|---------|
| 1 | Nodo STM32 (extremo del bus CAN) | Entre **CANH y CANL** en el conector CAN del STM32 | Termina el bus CAN en impedancia característica (120Ω). Sin terminación hay reflexiones en el bus que corrompen los frames a 500 kbps |
| 2 | Nodo ESP32 (extremo del bus CAN) | Entre **CANH y CANL** en el conector CAN del ESP32 | Ídem. SOLO los dos extremos físicos del bus llevan resistencia |

### 4.2 Pull-up I²C — 4.7 kΩ / 0.25W

| # | Ubicación | Conexión | Función |
|---|-----------|----------|---------|
| 1 | Bus I²C | Entre **SCL (PB6) y 3.3V** | Mantiene la línea en HIGH cuando ningún dispositivo la está tirando a LOW. Obligatorio en I²C open-drain |
| 2 | Bus I²C | Entre **SDA (PB7) y 3.3V** | Ídem para SDA |

### 4.3 Pull-up OneWire — 4.7 kΩ / 0.25W

| # | Ubicación | Conexión | Función |
|---|-----------|----------|---------|
| 1 | Bus DS18B20 | Entre **PB5 y 3.3V** | Mantiene el bus 1-Wire en HIGH. También proporciona corriente parásita para alimentar los DS18B20 en modo parásito si se usa |

### 4.4 Pull-up GPIO — 10 kΩ / 0.25W

| Cantidad | Ubicación | Conexión | Función |
|----------|-----------|----------|---------|
| 4 | Sensores de rueda (×4) | Entre señal del sensor y **3.3V** | Garantiza nivel HIGH definido cuando el sensor está en reposo (señal activa LOW) |
| 4 | Shifter F/N/R + botón emergencia | Entre pin GPIO y **3.3V** | Ídem para entradas digitales activas LOW |
| 2 | Stock | — | Reserva |

### 4.5 Shunt de corriente INA226 — Motores

| Valor | Cantidad | Ubicación | Función |
|-------|----------|-----------|---------|
| **1.5 mΩ ≥ 2W** (Bourns CSS2H o similar) | 5 | En serie con el **cable de potencia de cada motor** (entre BTS7960 salida M+ y motor) | El INA226 mide la caída de tensión sobre este shunt para calcular la corriente. Con 1.5mΩ: 10A → 15mV |
| **0.75 mΩ ≥ 4W** | 1 | En serie con el **cable positivo de la batería 24V** | Monitoriza la corriente total del sistema. Con 0.75mΩ: 50A → 37.5mV |

### 4.6 Resistencia base transistor NPN (relés) — 1 kΩ / 0.25W

| Cantidad | Ubicación | Conexión | Función |
|----------|-----------|----------|---------|
| 5 (1 por relé) | Entre pin GPIO del STM32 y base del transistor NPN (2N2222) | GPIO → [1kΩ] → Base NPN; Emisor a GND; Colector a bobina del relé | Limita la corriente de base. Con 3.3V GPIO y VBE≈0.7V: Ib = (3.3-0.7)/1000 = 2.6mA, suficiente para saturar el 2N2222 con Ic ≤ 150mA (bobina relé) |

### 4.7 Resistencia datos WS2812B — 300–500 Ω / 0.25W

| Cantidad | Ubicación | Conexión | Función |
|----------|-----------|----------|---------|
| 1 (por tira) | En serie en la línea DATA de cada tira LED | GPIO ESP32 → [300-500Ω] → DATA del primer WS2812B | Protege la entrada del primer LED del spike de tensión por la capacitancia del cable. Sin ella el primer IC del la tira falla antes que el resto |

---

## 5. Diodos de Protección

### 5.1 Flyback de relés — 1N4007

| Cantidad | Ubicación | Conexión | Función |
|----------|-----------|----------|---------|
| 5 (1 por relé) | En paralelo con la **bobina** de cada relé | Ánodo a GND (pin –), cátodo al pin + de la bobina | Cuando el transistor corta la bobina, la energía inductiva genera un spike positivo. El 1N4007 conduce ese spike de vuelta a través de la bobina (recirculación). Sin él el spike destruye el transistor NPN |

### 5.2 ESD en bus CAN — PESD2CAN (2 unidades)

| # | Ubicación | Conexión | Función |
|---|-----------|----------|---------|
| 1 | Nodo CAN del STM32 (TJA1051) | Entre **CANH y CANL** en el conector exterior del cable CAN | Clamp diferencial ESD. Protege el TJA1051 de descargas electrostáticas o transitorios por el cable. El PESD2CAN está optimizado para la impedancia diferencial de CAN (no usa un TVS genérico) |
| 2 | Nodo CAN del ESP32 | Ídem | Ídem |

### 5.3 TVS en bus de potencia BTS7960 — SMBJ30A (5 unidades)

| Cantidad | Ubicación | Conexión | Función |
|----------|-----------|----------|---------|
| 5 (1 por BTS7960) | En el rail **B+** de cada BTS7960 | Entre **B+ y GND**, en paralelo con el condensador de 470 µF | Clamp de back-EMF. Cuando se hace freno regenerativo brusco o el PWM corta la corriente del motor, la inductancia del bobinado genera picos de tensión que pueden superar 40–60V en un bus de 24V. El SMBJ30A (Vclamp=30V) conduce ese pico antes de que dañe el BTS7960 |

### 5.4 Schottky freewheeling motores — SB560 (5 unidades)

| Cantidad | Ubicación | Conexión | Función |
|----------|-----------|----------|---------|
| 5 (1 por motor) | En paralelo con los terminales del motor (M+ y M–) | Ánodo a M–, cátodo a M+ | Diodo de recirculación externo. Proporciona un camino de baja impedancia para la corriente del motor en el instante en que el BTS7960 corta la potencia. Reduce el estrés en los diodos body internos del BTS7960, especialmente en frenado agresivo |

### 5.5 P-MOSFET antipolaridad — ≥40V / ≥80A (1 unidad)

| # | Ubicación | Conexión | Función |
|---|-----------|----------|---------|
| 1 | En el **cable positivo principal de la batería**, entre el conector y el fusible | Source a batería (+), Drain hacia PCB (+), Gate al Drain a través de resistencia de pull-down (10kΩ a GND) | Protección de polaridad inversa. Con batería correcta: Vgs < 0 → MOSFET conduce. Con batería invertida: Vgs > 0 → MOSFET en corte → circuito abierto. Ventaja sobre diodo serie: R_DS(on) de ≈ 2–5 mΩ → pérdida << 0.1V a 50A, vs. ≈0.7V de un diodo |

---

## 6. Transistores de Control de Relés

### 6.1 NPN 2N2222 (o BC547) — 5 unidades (1 por relé)

| Parámetro | Valor |
|-----------|-------|
| Tipo | NPN BJT |
| Vce máx | 40V (2N2222) |
| Ic máx | 600 mA |
| Bobina relé | ≈ 70–100 mA a 5V |

**Circuito completo por relé:**
```
STM32 GPIO (3.3V) ──[1kΩ]──► Base (2N2222)
                              │
GND ─────────────────────────► Emisor
                              │
                              Colector ──── Bobina relé (–)
                                                │
5V ──────────────────── [1N4007 cátodo] ──── Bobina relé (+)
                              │
                         [1N4007 ánodo]
```

---

## 7. Mapa Visual del Sistema

```
BATERÍA 24V
    │
    ├─[P-MOSFET ≥40V/80A antipolaridad]
    │
    ├─[Fusible principal 60–80A]
    │
    ├─[SMBJ30A TVS ×1] ── Bus 24V principal ── [470µF + 1000µF electrolíticos]
    │
    ├─── BTS7960 FL ── [SMBJ30A TVS] ── [470µF] ── [100nF×2]
    │         └──── Motor FL ──[1.5mΩ shunt INA226]── [100nF×2 en terminales]
    │                                                     └── [SB560 Schottky]
    │
    ├─── BTS7960 FR ── [SMBJ30A TVS] ── [470µF] ── [100nF×2]
    │         └──── Motor FR ──[1.5mΩ shunt]── [100nF×2] ── [SB560]
    │
    ├─── BTS7960 RL ── [SMBJ30A TVS] ── [470µF] ── [100nF×2]
    │         └──── Motor RL ──[1.5mΩ shunt]── [100nF×2] ── [SB560]
    │
    ├─── BTS7960 RR ── [SMBJ30A TVS] ── [470µF] ── [100nF×2]
    │         └──── Motor RR ──[1.5mΩ shunt]── [100nF×2] ── [SB560]
    │
    ├─── BTS7960 DIR ─ [SMBJ30A TVS] ── [470µF] ── [100nF×2]
    │         └──── Motor DIR ──[1.5mΩ shunt]── [100nF×2] ── [SB560]
    │
    ├─── RELAY_TRAC ─[1N4007 flyback]─[2N2222]─[1kΩ]─ GPIO STM32
    │         └──── Snubber RC: [100nF/250V + 100Ω] en paralelo con contactos
    │
    ├─── RELAY_TRAC ─[1N4007]─[2N2222]─[1kΩ]─ GPIO STM32
    │         └──── Snubber RC: [100nF/250V + 100Ω]
    │
    ├─── RELAY_DIR ──[1N4007]─[2N2222]─[1kΩ]─ GPIO STM32
    │         └──── Snubber RC: [100nF/250V + 100Ω]
    │
    └─── Regulador DC-DC 5V → 3.3V
              │
              ├── STM32G474RE ── [10µF] ── [100nF × N (uno por pin VDD)]
              │       │
              │       ├── ADC Pedal ── [100nF filtro RC] ── Hall sensor
              │       └── I²C → [4.7kΩ pull-up SCL] + [4.7kΩ pull-up SDA]
              │              └── TCA9548A + 6× INA226 ──[0.75/1.5mΩ shunts]
              │
              ├── TJA1051 CAN ── [100nF] ──── CANH/CANL ──[PESD2CAN]── conector
              │       STM32 CAN lado ──[120Ω terminación]── bus
              │
              ├── ESP32-S3 ─── [10µF] ── [100nF]
              │       ├── TJA1051 CAN ESP32 ──[PESD2CAN]── bus CAN
              │       │       ESP32 CAN lado ──[120Ω terminación]── bus
              │       └── MCP23017 (palanca) ── [10µF/16V + 100nF en VDD] ── [10kΩ pull-up RESET]
              │               SDA/SCL ──[4.7kΩ pull-up ×2]── GPIO8/GPIO9 ESP32
              │
              ├── ToF VL53L1X ── [100nF en AVDD]
              │
              └── WS2812B LEDs
                      ├── [1000µF / 10V] en entrada de alimentación
                      └── GPIO ESP32 ──[300-500Ω]── DATA tira LED
```

---

## 8. Reglas de Montaje

### ✅ Reglas generales

1. **Distancia:** Los condensadores de bypass (100 nF) deben ir a **<10 mm** del pin de alimentación que protegen. A mayor distancia, mayor inductancia de pista y menor eficacia.

2. **GND compartido:** El GND del condensador de bypass debe conectar al **via de GND más cercano**, no ir por pistas largas. El camino de retorno importa tanto como el de ida.

3. **Polaridad:** Los electrolíticos siempre con la **banda blanca/raya (–) a GND**. Invertirlos los destruye con cortocircuito o explosión.

4. **Orden en paralelo:** Para el filtrado óptimo, el cerámico de 100 nF y el electrolítico deben estar en paralelo. El cerámico maneja las altas frecuencias, el electrolítico las bajas.

5. **Snubbers RC:** Los cables del snubber deben ser **<5 cm**. Cables largos añaden inductancia que anula el efecto del condensador a alta frecuencia.

6. **Resistencias shunt:** Conectar con las 4 patas del Kelvin (4-wire sense si el encapsulado lo permite) o en el caso de shunts de 2 pines, los pines de sense del INA226 a los terminales del shunt, no a la pista de potencia.

### ❌ Errores comunes a evitar

- No poner el condensador de bypass lejos del chip que protege
- No olvidar el 100 nF en la lógica VCC del BTS7960 (solo el electrolítico en B+ no es suficiente)
- No usar un solo condensador de 1000 µF en lugar de varios pequeños en paralelo para el bus (peor ESR)
- No olvidar el snubber de polipropileno en los relés (el 1N4007 en la bobina no protege los contactos)
- No conectar las 2 resistencias de terminación CAN de 120Ω en el mismo nodo (deben estar en los dos extremos físicos del cable CAN)

---

## 9. Convertidores DC-DC Aislados

### 9.1 → B0505S-1W — **10 unidades compradas**

**¿Qué es el B0505S-1W?**  
Convertidor DC-DC aislado en encapsulado SIP-4 (4 pines, montaje vertical).  
- **Entrada:** 5V DC (rango nominal 4.5–5.5V)  
- **Salida:** 5V DC **aislada galvánicamente** (GND de entrada y GND de salida son eléctricamente independientes)  
- **Potencia:** 1W → corriente de salida máxima **200 mA** a 5V  
- **Aislamiento galvánico:** ≥ **1000 V** entre entrada y salida (barrera dieléctrica interna)  
- **Encapsulado:** SIP-4 (Single In-line Package, 4 pines, 7.62 mm pitch)  
- **Fabricante:** BOSHIDA / múltiples fuentes compatibles (serie B05xxS de varios fabricantes es intercambiable)

**Pinout SIP-4:**

```
Pin 1 → +Vin  (5V entrada)
Pin 2 → -Vin  (GND entrada / GND_STM32)
Pin 3 → -Vout (GND salida / GND_CAN — ¡dominio aislado!)
Pin 4 → +Vout (5V salida aislada)
```

---

**¿Para qué sirve en este proyecto?**

Proporciona la alimentación aislada necesaria para la **barrera galvánica del bus CAN** (Opción A: ADuM1201 + DC-DC separado, descrita en `docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md` sección 9.4).

El TJA1051T/3 requiere **VCC = 4.5–5.5V** (no funciona a 3.3V). El lado aislado del aislador digital (ADuM1201 pin Vdd2) necesita su propia alimentación en el dominio GND_CAN, aislado de GND_STM32. El B0505S-1W resuelve ambas necesidades con un único componente:

```
Lado STM32 (GND_STM32)              Barrera B0505S-1W              Lado CAN (GND_CAN)
───────────────────────              ─────────────────              ──────────────────

5V_rail  ── Pin1(+Vin)  [B0505S-1W]  Pin4(+Vout) ──► 5V_aislada ──► TJA1051T/3 VCC
GND      ── Pin2(-Vin)               Pin3(-Vout) ──► GND_CAN    ──► TJA1051T/3 GND

                                                  ──► ADuM1201 Vdd2 (via LDO 3.3V opcional)
```

> ⚠️ **GND_CAN ≠ GND_STM32.** Una vez instalado el B0505S-1W, los dominios de tierra están separados. NO conectar GND_CAN a GND_STM32 en ningún punto; eso cortocircuitaría la barrera galvánica.

---

**¿Cuántas unidades se necesitan?**

| # | Uso | Notas |
|---|-----|-------|
| 1 | Aislamiento del bus CAN (barrera STM32 ↔ ESP32, Opción A) | 1 B0505S-1W por barrera galvánica de bus CAN |
| 9 | **Stock / reserva** | Recambios y posibles usos futuros en otras barreras de aislamiento |

> **Nota:** Si en el futuro se añade aislamiento galvánico en otros buses (p.ej. I2C de sensores en zona de alta tensión), cada barrera adicional requeriría 1 unidad más. Con 10 unidades hay margen amplio.

---

**Consumo del lado aislado (verificación de potencia):**

| Componente | Corriente típica |
|-----------|-----------------|
| TJA1051T/3 VCC (standby/activo) | ~5–50 mA |
| ADuM1201 Vdd2 (a 5V o 3.3V via LDO) | ~1–5 mA |
| **Total estimado** | **< 60 mA** |
| **Capacidad B0505S-1W** | **200 mA** |
| **Margen disponible** | ≈ 140 mA (factor 3×) ✅ |

---

**Última actualización:** 2026-05-18  
**Autor:** florinzgz (documentado por Copilot Agent)  
**Proyecto:** STM32-Control-Coche-Marcos
