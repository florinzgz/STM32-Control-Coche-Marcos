# Remote Control — Esquema de conexión eléctrica y anti-EMI

> Conexión propuesta del receptor **FlySky FS-iA6B** al **ESP32-S3** del proyecto
> MarcosDashboard v10 Final. **NO IMPLEMENTADO** — documento de diseño previo.

---

## 1. Conexionado eléctrico

### 1.1 Pinout del receptor FS-iA6B

El FS-iA6B tiene 7 grupos de 3 pines (S/+/−):

```
┌─────────────────────────────────────────────────┐
│  FS-iA6B                                        │
│                                                 │
│   CH1 [S][+][−]                                 │
│   CH2 [S][+][−]                                 │
│   CH3 [S][+][−]   ← PWM por canal (no usado)    │
│   CH4 [S][+][−]                                 │
│   CH5 [S][+][−]                                 │
│   CH6 [S][+][−]                                 │
│ SENS/iBUS [S][+][−] ← AQUÍ SE CONECTA           │
│                                                 │
│   Antena 1 ────│                                │
│   Antena 2 ────│                                │
└─────────────────────────────────────────────────┘
```

- **S** = señal (data line). En el puerto `SENS/iBUS` es la salida iBUS UART 3.3 V TTL.
- **+** = VCC (todos los grupos comparten el rail interno; alimentar UNO ya da tensión a todo).
- **−** = GND (común).

### 1.2 Cableado al ESP32-S3

Solo se necesitan **3 hilos**:

| Receptor FS-iA6B | Cable Dupont | Destino ESP32-S3 |
|---|---|---|
| `SENS/iBUS — S` (señal) | hilo 1 (idealmente trenzado con GND) | **GPIO 16** (RX) |
| `SENS/iBUS — +` (VCC) | hilo 2 | **Rail +5 V** del proyecto (NO al 3.3 V del ESP32) |
| `SENS/iBUS — −` (GND) | hilo 3 (referencia, idealmente trenzado con señal) | **GND común** del proyecto |

> **Importante:** la señal iBUS es **3.3 V TTL** aunque el receptor se alimente a 5 V.
> El driver UART del receptor está pensado para FCs de drones que mayoritariamente
> operan a 3.3 V. **No se necesita level shifter ni divisor resistivo.**

### 1.3 Diagrama ASCII

```
                                                    +5 V rail proyecto
                                                    (mismo que MCP23017,
                                                     DFPlayer, etc.)
                                                          │
                                                   ┌──────┴───────┐
                                                   │ Ferrita SBN  │
                                                   │ snap-on      │
                                                   └──────┬───────┘
                                                          │
                                                  ┌───────┴─────────┐
                                          C1 ═══ │                  │ C2 ═══ ┐
                                         100 nF  │   FS-iA6B (RX)   │ 10 µF  │
                                          GND ═══│                  │ GND ═══┘
                                                  │                  │
                                                  │    iBUS─signal───┼─────► GPIO 16 (ESP32-S3)
                                                  │      (3.3 V)     │       (RX, INPUT)
                                                  │                  │
                                                  │  GND ────────────┼─────► GND común
                                                  │                  │
                                                  │  Ant1 ▒▒▒▒▒▒▒▒▒  │
                                                  │  Ant2 ▒▒▒▒▒▒▒▒▒  │
                                                  └──────────────────┘
```

- **C1 = 100 nF cerámico** lo más cerca posible del pin VCC del receptor (filtra HF).
- **C2 = 10 µF electrolítico o cerámico** en paralelo con C1 (filtra ripple).
- **Ferrita snap-on** en el cable de alimentación, lo más cerca posible del receptor.

---

## 2. Estrategia anti-EMI

El BTS7960 genera EMI significativa por conmutación PWM a 20 kHz con corrientes de
hasta 43 A por canal. Las medidas siguientes se basan en buenas prácticas
documentadas en `docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md` y
`docs/BTS7960_IBT2_POWER_LOGIC_VALIDATION.md`.

### 2.1 Separación física

| Componente | Distancia mínima recomendada al BTS7960 / cables de potencia |
|---|---|
| Antenas del receptor FS-iA6B | **≥ 15 cm**, idealmente orientadas perpendicularmente al plano del motor |
| Cuerpo del receptor | **≥ 10 cm** |
| Cable receptor ↔ ESP32 | **≥ 5 cm** de cables de potencia |

### 2.2 Routing

- Cable receptor → ESP32: **trenzado** (TX + GND como par mínimo) o, mejor,
  cable apantallado con malla conectada solo en el extremo ESP32 (un único punto
  de masa para evitar ground loops).
- Antenas del receptor: **NUNCA paralelas** a cables de potencia. Colocar en
  forma de V a 90°.
- Mantener el receptor en la parte "lógica" del chasis (junto al ESP32 y el TFT),
  alejado del compartimento de motores y de las baterías.

### 2.3 Filtrado

Por orden de proximidad al receptor:

1. **C1 = 100 nF cerámico X7R**: en los pines `+` y `−` del puerto iBUS,
   con patillas < 5 mm. Filtra alta frecuencia (>1 MHz).
2. **C2 = 10 µF electrolítico** (o cerámico 10 µF X5R): en paralelo con C1.
   Filtra ripple de la alimentación del proyecto.
3. **Ferrita snap-on** tipo SBN/MnZn (impedancia ≥ 100 Ω a 100 MHz): abrazando
   el cable VCC + GND a la salida del receptor.
4. **(Opcional)** Choke en modo común en el cable de señal si se detectan
   problemas de checksum.

### 2.4 Tierra única

Toda la masa del proyecto debe converger en un único punto (star ground).
El GND del receptor se conecta al GND común que ya comparten ESP32, STM32,
TJA1051T/3 y el resto de periféricos. **No conectar** al chasis metálico ni
al GND de potencia del BTS7960 — esos deben tener su retorno separado hasta
el star ground.

---

## 3. Alimentación

### 3.1 Opción A — Rail 5 V existente (recomendado)

Si el proyecto ya tiene un buck 24 V → 5 V para alimentar los demás módulos
(DFPlayer, MCP23017, etc.), el receptor se conecta directamente a ese rail.

**Consumo del FS-iA6B:** ~30 mA típico, ~80 mA pico durante TX de telemetría.
**Margen del rail:** verificar que el buck del proyecto tiene > 100 mA de margen.

### 3.2 Opción B — Buck dedicado (si el rail 5 V está saturado)

Mini DC-DC buck (LM2596, MP1584, etc.) con entrada en el rail 24 V y salida 5 V.
Coste ~3 €. Asegurar:

- Condensador de 100 µF en la entrada
- Condensador de 47 µF + 100 nF en la salida
- Diodo de protección Schottky inverso (SS34) entre VOUT y la entrada por seguridad

### 3.3 ¿Por qué NO alimentar el receptor desde el 3.3 V del ESP32?

- El 3.3 V del ESP32-S3 está limitado a ~500 mA totales y ya alimenta el TFT
  y otros periféricos.
- El receptor FS-iA6B está especificado para 4.0–6.5 V — fuera de rango a 3.3 V
  produce comportamiento errático del transmisor RF.

---

## 4. Lista de materiales (BOM) específica

| Cantidad | Ref | Componente | Coste aprox |
|---|---|---|---|
| 1 | TX1 | FlySky FS-i6X (transmisor 10ch AFHDS 2A) | ~45 € |
| 1 | RX1 | FlySky FS-iA6B (receptor con iBUS) | viene con TX1 (o ~12 € suelto) |
| 1 | CBL1 | Cable Dupont 3 pines hembra-hembra, 20 cm | ~0,50 € |
| 1 | C1 | Condensador cerámico 100 nF X7R 50 V (0805 o through-hole) | ~0,05 € |
| 1 | C2 | Condensador electrolítico 10 µF / 25 V (o cerámico 10 µF X5R) | ~0,15 € |
| 1 | FB1 | Ferrita snap-on (≥100 Ω @ 100 MHz, ID 3 mm) | ~1 € |
| 0–1 | U1 | Mini DC-DC buck 24 V→5 V (si rail 5 V saturado) | ~3 € |
| **Total mínimo** | | | **~46,70 €** |
| **Total con buck dedicado** | | | **~49,70 €** |

> Nota: en línea con la memoria del proyecto, **no se duplica este BOM en
> otro inventario**. Cuando se autorice la implementación, estos pasivos
> deben integrarse en `docs/COMPONENTES_PASIVOS_REFERENCIA.md` en una nueva
> subsección "Remote control receiver", **sin crear listas paralelas**.

---

## 5. Procedimiento de montaje (cuando se autorice)

> ⚠️ Este procedimiento está documentado únicamente como referencia futura.
> No ejecutarlo todavía.

1. Desconectar batería del vehículo.
2. Montar el receptor FS-iA6B sobre el bastidor del chasis lógico (junto al
   ESP32, lejos del BTS7960 y de las baterías).
3. Fijar con velcro de doble cara o tornillos M2 nylon (no metal cerca de antenas).
4. Soldar C1 y C2 entre VCC y GND del puerto iBUS, lo más cerca posible.
5. Pasar el cable Dupont por una ferrita snap-on.
6. Conectar VCC al rail +5 V proyecto, GND al star ground.
7. Dejar GPIO 16 del ESP32 desconectado durante el primer arranque.
8. Encender vehículo en modo Standby (PWM motores deshabilitado).
9. Verificar con osciloscopio: en el pin S del iBUS debe verse tráfico UART 3.3 V
   a 115200 baud, tramas de 32 bytes cada ~7 ms.
10. Verificar con USB-Serial externo (FTDI o similar) conectando RX del FTDI al
    pin S del iBUS, lanzando un parser iBUS de PC para confirmar que las tramas
    son válidas y reflejan los movimientos del joystick.
11. Solo tras validar (10), conectar el pin S al **GPIO 16** del ESP32.

---

## 6. Tabla de tensiones esperadas

| Punto | Tensión | Notas |
|---|---|---|
| FS-iA6B `+` | 5,0 V ± 0,2 V | Alimentación |
| FS-iA6B `−` (GND) | 0 V | Referencia |
| FS-iA6B `S` iBUS — reposo (idle) | **3,3 V** | UART idle state (HIGH) |
| FS-iA6B `S` iBUS — actividad | bursts 0 V ↔ 3,3 V cada ~7 ms | 32 bytes a 115200 baud |
| GPIO 16 ESP32-S3 | mismo perfil que la señal | INPUT digital, modo UART |

---

**Estado del documento:** Diseño verificado. Pendiente autorización del usuario para
ejecutar el procedimiento de montaje.
