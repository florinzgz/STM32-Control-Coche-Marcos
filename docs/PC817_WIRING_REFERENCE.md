# REFERENCIA COMPLETA DE CABLEADO PC817
## Sistema de Control Vehicular — STM32G474RE + ESP32-S3
### Fuente de verdad: `project_config.h` y `power_manager.h`

> **Este documento es la referencia consolidada para conectar el módulo PC817
> de 8 canales. Todos los valores han sido verificados en el código fuente.**

---

## RESUMEN DE CORRECCIONES (rev. 2026-04-28b — módulo HY-M158)

| # | Corrección | Impacto |
|---|-----------|---------|
| C1 | Topología de entrada LJ12A3 corregida — el cable NEGRO (señal NPN) va a IN-, no a GND | **Crítico** — error de cableado que impediría funcionar |
| C2 | Descripción de aislamiento GND corregida — STM32 y ESP32 **comparten GND lógico** | Importante |
| C3 | Diodo 1N4148 antiparalelo añadido como protección automotriz | Recomendado |
| C4 | **Canal A6 (señal Z) MOVIDO al 6N137** — ya no usa PC817. Ver `docs/ENCODER_WIRING_6N137.md` | **Cambio de cableado** |
| C5 | Módulo físico identificado: **HY-M158** de 8 canales — los SMD `302` (3 kΩ) ya están a bordo en serie con cada LED → **no hace falta añadir la 1 kΩ externa de entrada** | Simplificación |
| C6 | **Jumpers rojos del módulo: QUITAR todos.** Con ellos puestos cortocircuitan GND_vehicle con GND_logic y anulan el aislamiento galvánico | **Crítico** — montaje |
| C7 | Reasignación de canales (5 en uso, 3 libres): de **abajo a arriba** FR-FL-RR-RL-Llave (`IN1`→`IN5`); `IN6`–`IN8` reserva | Cableado |

---

## RESUMEN RÁPIDO — Módulo HY-M158 (rev. 2026-04-28b)

> **Sentido físico de los terminales en el módulo (mirándolo con el texto `HY-M158` arriba):**
> el bloque de terminales del lado **izquierdo** (`V1…V8` + `G`, junto a los jumpers rojos) es el
> **lado de 12 V** (entrada, ánodo del LED). El bloque del lado **derecho** (`IN1…IN8` + `G`)
> es el **lado lógico de 3,3 V** (salida, colector del fototransistor).
> Numeración: `IN1` queda en la parte **inferior** del módulo (junto al texto `817 Module`),
> `IN8` en la **superior** (junto al texto `HY-M158`). **Empezamos a cablear por `IN1`** y
> subimos hasta `IN5`; `IN6`–`IN8` quedan libres.

### Asignación de canales (de abajo a arriba, lado IN — empezando por `IN1`)

> Numeración del módulo HY-M158: el pin `1` del PC817 corresponde al canal **`IN1`**
> (terminal inferior del módulo en la foto). Cableamos secuencialmente desde `IN1`
> hacia arriba; los canales sin uso quedan en la parte superior (`IN6`–`IN8`).

| Canal módulo | Señal | Destino | Pull-up salida |
|--------------|-------|---------|----------------|
| **IN1 (abajo)** | Sensor rueda **FR** | STM32 **PA1** (EXTI1) | **4.7 kΩ** a 3.3V |
| **IN2** | Sensor rueda **FL** | STM32 **PA0** (EXTI0) | **4.7 kΩ** a 3.3V |
| **IN3** | Sensor rueda **RR** | STM32 **PB15** (EXTI15) | **4.7 kΩ** a 3.3V |
| **IN4** | Sensor rueda **RL** | STM32 **PA2** (EXTI2) | **4.7 kΩ** a 3.3V |
| **IN5** | **Llave de contacto** | ESP32 **GPIO 40** | **10 kΩ** a 3.3V |
| IN6 / IN7 / IN8 | *libre — reserva* | — | — |

**Señales que ya NO usan este módulo PC817:**

| Señal | Motivo | Va a |
|-------|--------|------|
| Encoder Z (PB4) | Aislamiento galvánico de mejor calidad necesario por proximidad al BTS7960 de dirección | **6N137** — ver `docs/ENCODER_WIRING_6N137.md` |
| Sensor centro dirección LJ12A3 (PB5) | Función cubierta por el pulso Z del encoder ya aislado por 6N137 | **No se cablea por este módulo.** Si se mantiene físicamente, requeriría un canal PC817 propio o un canal libre (`IN6`–`IN8`) |

> **Resistencia de entrada (lado 12 V):** **NO añadir resistencia externa.**
> El módulo HY-M158 ya integra **3 kΩ (SMD `302`)** en serie con cada LED del PC817.
> Con 12 V → I_LED = (12 V − 1,2 V) / 3 kΩ ≈ **3,6 mA** (dentro de spec del PC817, CTR ≥ 50 % garantiza saturación con pull-up de 4,7 kΩ a 3,3 V en la salida).

---

## COMPONENTES A AÑADIR AL MÓDULO HY-M158 (PC817 ×8)

> ⚠️ El módulo HY-M158 trae los **3 kΩ (SMD `302`) de entrada** y los LED indicadores
> ya soldados, pero **NO incluye pull-up de salida** (la salida es colector abierto en `IN_n`).
> Verificado por inspección visual y por medición (comentario en `power_manager.h`).

### Por cada canal en uso (×5 canales: 4 ruedas + llave):

| Componente | Valor | Dónde va |
|-----------|-------|----------|
| ~~Resistencia entrada~~ | ~~1 kΩ ¼W~~ | ~~lado 12V~~ — **NO añadir, ya integrada en placa (3 kΩ SMD `302`)** |
| **Pull-up salida** (4 ruedas FR/FL/RR/RL) | **4.7 kΩ 1/8W** | Entre `IN_n` (colector) y **+3.3V**, lado lógico |
| **Pull-up salida** (Llave de contacto) | **10 kΩ 1/8W** | Entre `IN_n` (colector) y **+3.3V**, lado ESP32 |
| **1N4148** *(recomendado)* | Diodo señal 100V/200mA | Antiparalelo con el LED PC817, protección automotriz |

### Por qué la 3 kΩ integrada del HY-M158 es suficiente para 12 V:
```
I_LED = (12V − 1.2V) / 3kΩ = 3.6 mA  ← dentro del rango nominal del PC817 (1–20 mA) ✅
                                       Con CTR mín. 50 % → I_C ≈ 1.8 mA en la salida.
                                       Suficiente para saturar contra pull-up de 4.7 kΩ a 3.3V.
```
> Si en algún momento se decide alimentar el lado de entrada a **24 V** en vez de 12 V,
> hay que recalcular: I_LED = (24 − 1.2) / 3 kΩ = 7.6 mA — sigue siendo válido,
> el módulo HY-M158 está especificado para 12–24 V de entrada de fábrica.

### Por qué 4.7 kΩ (no 10 kΩ) en los sensores de rueda:
El pull-up interno del STM32 (~40 kΩ) es demasiado lento para los flancos del
sensor inductivo con cable de hasta 5 m. El 4.7 kΩ externo (en paralelo con el
interno = 4.2 kΩ efectivo) garantiza t_flanco ≤ 5 µs con hasta 1 nF de capacidad
de cable. Ver cálculo completo en `docs/VALIDACION_CAN_PULLUP_PC817.md §2`.

```
τ = 4.2 kΩ × 1 nF = 4.2 µs   → t_to_VIH ≈ 5.1 µs con cable ≤ 5 m ✅
τ = 10 kΩ  × 1 nF = 10 µs    → t_to_VIH ≈ 12 µs → flancos lentos, ruido ❌
```

### Por qué 10 kΩ (no 4.7 kΩ) para la llave de contacto (ESP32):
La llave de contacto es una señal de baja frecuencia (~1 Hz). No se requiere
flanco rápido. La 10 kΩ reduce la corriente de reposo cuando el transistor está
en corte, ahorrando ~0.3 mA por canal respecto a 4.7 kΩ. Compatible con la
resistencia `INPUT_PULLUP` interna del ESP32 (~45 kΩ).

---

## ⚠️ JUMPERS ROJOS DEL MÓDULO HY-M158 — **QUITAR TODOS**

El módulo HY-M158 trae 8 jumpers rojos en el lado de los terminales `V1…V8` (lado 12 V).
**Con esos jumpers puestos se establece continuidad eléctrica entre el `G` del lado de 12 V
y el `G` del lado de 3,3 V — esto cortocircuita los dos dominios de masa y anula por completo
el aislamiento galvánico que es la razón de ser del PC817.**

### Acción obligatoria de montaje:

1. **Retirar los 8 jumpers rojos** (los 8, aunque algunos canales no se usen).
2. **Verificar con polímetro en modo continuidad:**
   - Entre `G` del lado V (12 V) y `G` del lado IN (3,3 V) → **debe estar abierto** (sin pitido). ✅
   - Entre `G` y `G` adyacentes del **mismo lado** → puede estar abierto o cerrado, no afecta al aislamiento.
3. Si después de quitar los 8 jumpers sigue habiendo continuidad entre `G` de un lado y `G` del otro,
   significa que hay una pista del PCB que une los dos `G` (algunas variantes del módulo lo tienen).
   En ese caso el módulo **no aísla galvánicamente** por sí mismo y hay que:
   - **Cortar la pista** del PCB que une los dos planos de GND, o
   - Sustituir el módulo por una versión correctamente aislada.

### Cableado de las dos masas con los jumpers retirados:

```
LADO 12 V (terminales V1…V8 + sus G del MISMO lado):
  GND_vehicle ──► 1 cable común al `G` de V1 (los demás `G` del lado V quedan en el aire,
                  o se unen entre ellos físicamente con un puente de cable corto si quieres
                  un único punto de tierra; pero NUNCA se conectan al `G` del lado IN).

LADO 3.3 V (terminales IN1…IN8 + sus G del MISMO lado):
  GND_logic   ──► 1 cable al `G` de IN1 (común a STM32 + ESP32; sirve para los 5 canales).
```

> Los jumpers rojos **no aportan ninguna función eléctrica útil para esta aplicación**
> (en el diseño original del HY-M158 son configuradores de fábrica para venderlo
> como módulo "no aislado" si el cliente quiere). En **este proyecto el aislamiento es
> obligatorio**, por tanto van fuera.

---

## PROTECCIÓN AUTOMOTRIZ — DIODO 1N4148 *(RECOMENDADO)*

> ⚠️ **RECOMENDADO para entornos automotrices.** Los transitorios negativos en
> el bus de 12V (arranque de motor, cargas inductivas) pueden superar −1V en la
> línea de señal del sensor, lo que dañaría el LED del PC817 por corriente inversa.

Añadir **1 diodo 1N4148 en antiparalelo** con el LED del PC817 por canal:

```
       Ánodo LED (IN+)
           │
           ▼  LED
           │  PC817
           │
       Cátodo LED (IN-)
           │

Diodo 1N4148 antiparalelo:
  Cátodo 1N4148 → IN+ (mismo nodo que ánodo LED)
  Ánodo  1N4148 → IN- (mismo nodo que cátodo LED)

  Corriente directa PC817: normal (LED conduce)
  Transición negativa:     1N4148 conduce → limita V_inversa a ~0.7V → LED protegido ✅
```

| Componente | Valor | Montaje | Propósito |
|-----------|-------|---------|-----------|
| 1N4148 × 7 | Diodo de señal 100V/200mA | Antiparalelo con el LED PC817 | Protección contra picos negativos automotrices |

*Si no se añade: el PC817 admite V_R máx. = 6V (datasheet). En entorno automotriz limpio
esto puede ser suficiente, pero se recomienda el 1N4148 como medida de robustez de bajo costo.*

---

## ESQUEMA DE CONEXIÓN — Sensores LJ12A3 de rueda (canales `IN1`–`IN4`)

> ⚠️ **CORRECCIÓN CRÍTICA respecto a versión anterior:**
> El cable NEGRO del LJ12A3 es la **salida NPN**, no el GND.
> El cable AZUL es GND. El error anterior conectaba los colores al revés,
> lo que impediría que el LED del PC817 condujera nunca.

El sensor LJ12A3-4-Z/BX es NPN de colector abierto:
- **MARRÓN** = +12V (alimentación del sensor)
- **AZUL** = GND (masa del sensor)
- **NEGRO** = Salida NPN (colector abierto, se cierra a GND cuando detecta metal)

Topología correcta para excitar el LED del PC817 (módulo HY-M158, **resistencia de entrada
3 kΩ ya integrada**, no se añade externa):

```
════════════ LADO 12V (entrada — terminal V_n del HY-M158) ════════════

  +12V ──────────────────────────────────────────────► MARRÓN (VCC sensor LJ12A3)

  +12V ─────────────────────────────────► HY-M158 V_n (entrada del canal)
                                              │
                                          [3kΩ on-board (302)]   ← integrado en el módulo
                                              │
                                              ▼  ←── LED conduce cuando
                                            PC817      NEGRO está a GND
                                            LED
                                              │
                                              └─► HY-M158 G (lado V)  ────► NEGRO (salida NPN sensor LJ12A3)
                                                                                 │
                                                                                 ▼ (transistor NPN interno)
  GND_vehicle ──────────────────────────────────────────────────────────────► AZUL (GND sensor LJ12A3)
                       (mismo nodo que `G` del lado V del HY-M158, **NUNCA** unido a `G` del lado IN)

  [1N4148 antiparalelo opcional: cátodo → V_n, ánodo → G del lado V]   ← RECOMENDADO

════════════ LADO 3.3V (salida — terminal IN_n del HY-M158) ════════════

  +3.3V ──[4.7kΩ 1/8W]──┬──► HY-M158 IN_n (colector del fototransistor)
                          │
                     ──►  STM32 GPIO pin destino (ver tabla de canales)

  HY-M158 G (lado IN) ──► GND_logic (masa STM32 / ESP32, lado lógico)
```

**Secuencia de funcionamiento:**
1. Metal detectado → transistor NPN del LJ12A3 se satura → NEGRO = GND
2. Corriente: +12V → 3kΩ on-board → LED PC817 → G(V) → NEGRO → GND_vehicle = **3.6 mA** → LED encendido
3. PC817 salida: transistor de salida satura → `IN_n` ≈ 0,2 V (LOW)
4. STM32 GPIO lee **LOW** → firmware interpreta como **pulso activo** ✅

**Sin metal:**
1. Transistor NPN del LJ12A3 en corte → NEGRO = flotante (alta impedancia)
2. Sin corriente por el LED → LED apagado
3. PC817 salida: transistor de salida en corte → `IN_n` = HIGH (vía pull-up 4.7 kΩ a 3,3 V)
4. STM32 GPIO lee **HIGH** → firmware interpreta como **sin pulso** ✅

## CANAL A6 — Señal Z del encoder ya **NO** usa este módulo

> **La señal Z del encoder E6B2-CWZ6C ya no pasa por el PC817 / HY-M158.**
> Pasa por el **6N137** junto con los canales A y B del encoder, para tener el mismo
> aislamiento galvánico de alta calidad y eliminar bucles de masa cerca del BTS7960
> de dirección. Ver `docs/ENCODER_WIRING_6N137.md` (3 canales 6N137: A, B, Z).
>
> El pin del firmware no cambia: `PIN_ENC_Z` sigue siendo **PB4** (entrada GPIO sondeada
> a baja frecuencia para detectar el centro mecánico del volante).

## ESQUEMA CANAL DE LLAVE DE CONTACTO (ESP32) — `IN5` del HY-M158

```
════════════ LADO 12V (entrada — terminal V5 del HY-M158) ════════════

  Terminal ON del contacto (+12 V cuando llave en ON) ──► HY-M158 V5
                                                              │
                                                          [3kΩ on-board]
                                                              │
                                                              ▼
                                                            LED PC817
                                                              │
                                                              └──► HY-M158 G (lado V) ──► GND_vehicle

  [1N4148 antiparalelo opcional: cátodo → V5, ánodo → G del lado V]   ← RECOMENDADO

════════════ LADO 3.3V (salida — terminal IN5 del HY-M158) ════════════

  +3.3V ──[10kΩ 1/8W]──┬──► HY-M158 IN5 (colector del fototransistor)
                         │
                    ──►  ESP32 GPIO 40

  HY-M158 G (lado IN) ──► GND_logic (masa ESP32 / STM32 — compartida)
```

---

## LÓGICA INVERTIDA (PC817 invierte la señal)

### Sensores de rueda (FR/FL/RR/RL → STM32):
| Estado sensor | LED PC817 | GPIO STM32 | Interpretación firmware |
|---------------|-----------|------------|------------------------|
| Detecta metal (activo) | Encendido | **LOW** | Pulso detectado ✅ |
| No detecta | Apagado | **HIGH** (pull-up) | Sin pulso |

El firmware configura estos pines como `GPIO_PULLUP` + `EXTI` en flanco de bajada.

### Llave de contacto (`IN5` → ESP32 GPIO 40):
| Posición llave | LED PC817 | GPIO 40 ESP32 | Interpretación firmware |
|----------------|-----------|---------------|------------------------|
| **ON** (+12V) | Encendido | **LOW** | Llave encendida ✅ |
| **OFF** (0V) | Apagado | **HIGH** (pull-up) | Llave apagada |

```cpp
// power_manager.cpp — lectura de la llave
bool raw = (digitalRead(PIN_IGNITION_SENSE) == LOW);  // LOW = llave ON
```
El ESP32 usa `INPUT_PULLUP` como respaldo de seguridad, pero el 10 kΩ externo
es **obligatorio** para inmunidad al ruido automotriz.

### Señal Z del encoder — fuera de este módulo
La señal Z (PB4) ya **no** pasa por el PC817 / HY-M158. Pasa por el **6N137**.
Ver `docs/ENCODER_WIRING_6N137.md` para la tabla de niveles lógicos del Z.

### Sensor LJ12A3 de centro de dirección (PB5) — fuera de este módulo
Si se decide mantener este sensor en el coche, debe cablearse aparte (otro PC817
suelto, otro 6N137, o uno de los canales libres `IN6`–`IN8` del HY-M158).
En la configuración acordada de 5 canales (4 ruedas + llave) **no se cablea por
este módulo**, y la función de "centro mecánico" la cubre el pulso Z del encoder
ya aislado por 6N137.

---

## PEDAL ACELERADOR — NO usa PC817

El pedal (SS1324LUA-T, Hall-effect 5V) va directo al STM32 con divisor de tensión:

```
Pedal OUT (0.3V–4.8V a 5V) ──[10kΩ]──┬──► PA3 STM32 (ADC1_IN4)
                                        │
                                     [6.8kΩ]
                                        │
                                       GND

Voltaje en PA3: 0–3.3V (compatible con ADC interno de 3.3V)
```

---

## ENCODER E6B2-CWZ6C — NO usa PC817, usa 6N137

Los canales A y B del encoder usan aisladores **6N137** (no PC817), con:
- Pull-up externo **4.7 kΩ** a 3.3V en la salida (pin 6 del 6N137)
- El firmware usa `GPIO_NOPULL` para ENC_A (PA15/TIM2_CH1) y ENC_B (PB3/TIM2_CH2)
- Si se omite este pull-up, **la entrada del TIM2 flota y se pierde el control de dirección**

Ver `docs/ENCODER_WIRING_6N137.md` para el circuito completo del encoder.

---

## LISTA DE COMPRA DE RESISTENCIAS Y DIODOS (módulo HY-M158, 5 canales en uso)

| Cant. | Valor | Tipo | Para |
|-------|-------|------|------|
| ~~5~~ | ~~1 kΩ ¼W~~ | — | **NO necesarias** — la HY-M158 ya integra 3 kΩ (SMD `302`) en cada canal |
| 4 | **4.7 kΩ 1/8 W** | Film metal | Pull-up salida `IN1`–`IN4` (4 ruedas → STM32) |
| 1 | **10 kΩ 1/8 W** | Film metal | Pull-up salida `IN5` (llave → ESP32 GPIO 40) |
| 5 | **1N4148** | Diodo señal 100V/200mA | Antiparalelo LED PC817, protección automotriz *(recomendado, 1 por canal en uso)* |

*(Para el divisor del pedal: 1× 10 kΩ + 1× 6.8 kΩ — pedido por separado)*
*(Para el encoder E6B2-CWZ6C ver `docs/ENCODER_WIRING_6N137.md` — usa 3× 6N137 con sus propias R_IN y pull-ups)*

---

## NOTAS DE MONTAJE Y AISLAMIENTO DE GND

### Regla de GND — CORREGIDA

> ⚠️ La versión anterior decía "no conectar GND de ambos lados al mismo punto".
> Esto era **incompleto y engañoso**. La regla correcta es:

| GND | Qué incluye | Qué NO incluye |
|-----|-------------|----------------|
| **GND_vehicle** (lado 12V del PC817 — `G` del lado V del HY-M158) | Masa del vehículo, sensores LJ12A3 cable AZUL, llave de contacto | Masa lógica del STM32 / ESP32 |
| **GND_logic** (lado 3.3V del PC817 — `G` del lado IN del HY-M158) | STM32 GND, ESP32 GND, emisor PC817 salida | Masa del vehículo / motor |

**El aislamiento es SOLO entre el dominio de 12V y el dominio lógico de 3.3V.**

**STM32 y ESP32 DEBEN compartir GND_logic.** El bus CAN ya lo requiere por construcción
(los transceivers TJA1051T/3 de ambos nodos comparten el mismo GND_CAN).

```
GND_vehicle (12V) ─── G del lado V del HY-M158 ───┐
                                                   │ ← BARRERA GALVÁNICA (PC817)
                                                   │   (jumpers rojos QUITADOS)
GND_logic   (3.3V) ─── G del lado IN del HY-M158 ─┤
                       └──────── STM32 GND ───────┤
                       └──────── ESP32 GND ───────┘
                       (mismo nodo, obligatorio para CAN)
```

### Otras notas de montaje

1. **Quitar los 8 jumpers rojos del módulo HY-M158** antes de conectar nada
   (ver sección dedicada arriba). Verificar con polímetro que ya no hay
   continuidad entre `G` del lado V y `G` del lado IN.
2. Las resistencias de pull-up (4.7 kΩ / 10 kΩ) se pueden soldar directamente
   sobre los pines `IN_n` y un raíl de +3,3 V montado en el lado IN del módulo,
   o en una placa de prototipos aparte muy próxima al módulo.
3. La resistencia de entrada **NO se añade**: ya está integrada en el HY-M158
   (3 kΩ SMD `302` por canal).
4. El módulo necesita **alimentación 3.3V en el lado lógico** (no 5V) para los
   pull-ups externos. Usar el LDO externo AMS1117-3.3, no el pin 3.3V de la Nucleo-64.
5. Los diodos 1N4148 antiparalelo se sueldan junto a cada terminal de entrada
   `V_n` del lado 12 V del módulo (cátodo a `V_n`, ánodo a `G` del lado V).
6. Los 3 canales libres (`IN6`, `IN7`, `IN8`) se dejan **sin cablear** por ambos
   lados (ni en V ni en IN). Constituyen reserva para futuras señales aisladas
   12 V → 3,3 V (p. ej. luces de freno, sensor de marcha atrás, etc.).

## BUS CAN (PA11/PA12) — NO usa PC817 ni 6N137

El bus CAN del STM32 (FDCAN1) **no se aisla con optoacopladores PC817 ni 6N137**.
Usa el transceiver **TJA1051T/3** en una de estas dos configuraciones:

### Configuración A — Actual (3.3 V, sin aislamiento galvánico) ✅ Funcional

| Pin TJA1051T/3 | Valor actual | Estado vs. datasheet |
|----------------|-------------|----------------------|
| **VCC** | **3.3 V** | ⚠️ Fuera de spec. (4.5–5.5 V) — funciona en práctica |
| **VIO** | **3.3 V** | ✅ Dentro de spec. (2.8–5.5 V) |
| **TXD** ← STM32 PA12 | Conexión directa | ✅ |
| **RXD** → STM32 PA11 | Conexión directa | ✅ |

**Riesgos de VCC = 3.3 V:** margen de ruido reducido, sin garantía en temperatura extendida,
posible inestabilidad en entorno con motores DC de 24 V.

**GND:** STM32, ESP32 y ambos TJA1051 comparten el mismo GND_logic (sin barrera galvánica).

### Configuración B — Recomendada para vehículo (5 V + ADuM1201) 🔵

Añade barrera galvánica de **2500 V** entre GND_logic (STM32/ESP32) y GND_CAN (chasis):

```
STM32 PA12 ──► ADuM1201 AI │══════│ AO ──► TJA1051T/3 TXD ──► Bus CAN
STM32 PA11 ◄── ADuM1201 BO │══════│ BI ◄── TJA1051T/3 RXD ◄── Bus CAN
```

> ✅ Verificación de canales sin inversiones:
> TX: PA12 → ADuM **AI** → **AO** → TJA TXD ✓ | RX: TJA RXD → ADuM **BI** → **BO** → PA11 ✓

Alimentación Configuración B:
- ADuM1201 **V1/G1** → 3.3V_STM32 / GND_logic (lado STM32)
- ADuM1201 **V2/G2** → 3.3V_aislada / GND_CAN (lado CAN, del DC-DC aislado + LDO)
- TJA1051 **VCC** → **5V_aislada** (DC-DC aislado) — ✅ dentro de spec.
- ⚠️ GND_logic y GND_CAN **NO deben conectarse directamente** en Configuración B

**DC-DC aislado:** RECOM RxxP5.0S o Murata MEE1S0505SC. No usar convertidores no aislados.

**Ningún cambio de firmware** en ninguna configuración.
Ver `docs/CONEXIONES_COMPLETAS.md §9` para el esquema completo y tablas de conexión detalladas.

---

*Documento creado a partir de `power_manager.h`, `project_config.h` y*
*`docs/VALIDACION_CAN_PULLUP_PC817.md`. Revisado y corregido: 2026-04-28b.*
*Cambios rev. 2026-04-28b: módulo identificado como **HY-M158** (PC817 ×8); jumpers rojos*
*marcados como **a quitar**; resistencia de entrada externa eliminada (3 kΩ SMD ya en placa);*
*canal Z del encoder retirado del PC817 y movido al 6N137 (ver `ENCODER_WIRING_6N137.md`);*
*reasignación física (de abajo a arriba): IN1=FR, IN2=FL, IN3=RR, IN4=RL, IN5=Llave;*
*IN6–IN8 reserva.*
