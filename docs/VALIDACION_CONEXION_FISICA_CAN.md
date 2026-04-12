# Validación Práctica — Conexión Física CAN: ESP32-S3 ↔ TJA1051T/3 ↔ STM32G474RE

**Fecha:** 2026-03-02  
**Versión:** 1.0  
**Componentes:** ESP32-S3 · TJA1051T/3 (×2) · STM32G474RE · Bus CAN (CANH/CANL)

---

## 1. ¿Es correcta la arquitectura propuesta?

**SÍ. La arquitectura es correcta.** Validación conexión por conexión:

| Conexión propuesta | Estado | Detalle |
|-------------------|--------|---------|
| ESP32-S3 GPIO4 (TX) → TJA1051T/3 #1 pin 1 (TXD) | ✅ CORRECTO | GPIO4 es el pin TWAI TX del ESP32 |
| ESP32-S3 GPIO5 (RX) → TJA1051T/3 #1 pin 4 (RXD) | ✅ CORRECTO | GPIO5 es el pin TWAI RX del ESP32 |
| TJA1051T/3 #1 VCC (pin 3) → +5V, VIO (pin 5) → +3.3V | ✅ CORRECTO | VCC del TJA1051T/3 = 5V, VIO = 3.3V |
| TJA1051T/3 #1 GND (pin 2) → GND | ✅ CORRECTO | Obligatorio |
| TJA1051T/3 #1 CANH (pin 7) → cable trenzado → TJA1051T/3 #2 CANH (pin 7) | ✅ CORRECTO | Par trenzado reduce EMI |
| TJA1051T/3 #1 CANL (pin 6) → cable trenzado → TJA1051T/3 #2 CANL (pin 6) | ✅ CORRECTO | Par trenzado reduce EMI |
| TJA1051T/3 #2 TXD (pin 1) ← STM32 PA12 (FDCAN1_TX, AF9) | ✅ CORRECTO | PA12 es el pin FDCAN1 TX del STM32G474RE |
| TJA1051T/3 #2 RXD (pin 4) → STM32 PA11 (FDCAN1_RX, AF9) | ✅ CORRECTO | PA11 es el pin FDCAN1 RX del STM32G474RE |
| TJA1051T/3 #2 VCC (pin 3) → +5V, VIO (pin 5) → +3.3V | ✅ CORRECTO | VCC del TJA1051T/3 = 5V, VIO = 3.3V |
| TJA1051T/3 #2 GND (pin 2) → GND | ✅ CORRECTO | Obligatorio |
| 120Ω solo en los extremos | ✅ CORRECTO | Uno en el lado ESP32, uno en el lado STM32 |

---

## 2. ¿Falta alguna conexión obligatoria?

**SÍ. Faltan tres conexiones que son obligatorias:**

### 2.1 Pin 8 (S) — OBLIGATORIO en AMBOS transceivers

**Este pin DEBE conectarse a GND en los dos módulos TJA1051T/3.**

| Transceiver | Pin 8 | Debe conectarse a |
|-------------|-------|-------------------|
| TJA1051T/3 #1 (lado ESP32) | S (silent mode) | GND del sistema ESP32 |
| TJA1051T/3 #2 (lado STM32) | S (silent mode) | GND del sistema STM32 |

Si este pin queda flotante o conectado a VCC, el transceiver entra en **modo standby**:
puede recibir pero **no transmite**. Resultado: comunicación unidireccional o nula.

### 2.2 Condensador de desacoplo 100nF — RECOMENDADO en ambos transceivers

Colocar un condensador cerámico de 100nF entre VCC (pin 3) y GND (pin 2) de cada TJA1051T/3,
**lo más cerca posible del chip**. Sin él, el bus puede tener ruido en las transiciones de bit.

### 2.3 GND común entre ambos nodos — OBLIGATORIO

Ver sección 4.

---

## 3. Pin 8 (S) — Cómo conectarlo exactamente

### 3.1 Nombres equivalentes para el mismo pin físico

Dependiendo del módulo o versión del datasheet, el pin 8 puede aparecer etiquetado como:

| Etiqueta en el módulo | Nombre en datasheet NXP | ¿Es el mismo pin? |
|-----------------------|-------------------------|-------------------|
| **S** | S (Silent mode) | ✅ Sí — nombre oficial TJA1051T/3 (NXP) |
| **Rs** | S (Silent mode) | ✅ Sí — etiqueta alternativa en algunos módulos breakout |
| **SLOPE** | SLOPE | ✅ Sí — etiqueta alternativa en algunos módulos breakout |
| **STB** | STB (Standby) | ⚠️ Verificar — algunos módulos usan STB; para el TJA1051T/3 el comportamiento es: GND = normal mode |

> **Regla práctica:** Sea cual sea la etiqueta (S, Rs, SLOPE o STB), conéctalo a GND.
> Para el transceiver CAN de alta velocidad NXP TJA1051T/3, GND en este pin = modo normal.

### 3.2 Tabla de comportamiento del pin 8 en el TJA1051T/3

| Nivel en pin 8 | Modo | TX activo | RX activo |
|----------------|------|-----------|-----------|
| LOW → GND | **Normal** ✅ | Sí | Sí |
| HIGH → VCC | Silent | **NO** ❌ | Sí |
| Flotante | Indefinido | Imprevisible | Imprevisible |

### 3.3 Conexión exacta

```
TJA1051T/3 #1 (lado ESP32):
  Pin 8 (S) ──────────────────→ GND del ESP32

TJA1051T/3 #2 (lado STM32):
  Pin 8 (S) ──────────────────→ GND del STM32
```

Cable corto, conexión directa. No se necesita resistencia ni condensador en este pin.

---

## 4. ¿Es necesario GND común entre ambos nodos?

**SÍ. Es OBLIGATORIO.**

El bus CAN usa señalización diferencial (CANH − CANL), pero ambos transceivers necesitan
una referencia de tensión común para que los umbrales de detección de bits sean correctos.

### 4.1 Qué ocurre sin GND común

| Escenario | Consecuencia |
|-----------|-------------|
| GND del ESP32 ≠ GND del STM32 | Diferencia de potencial entre los dos GNDs → los niveles de CANH/CANL son incorrectos desde la perspectiva del receptor |
| Sin GND común | Bus-off constante, errores de bit, o comunicación aparentemente unidireccional |
| Diferencia de GND > 2V | Los transceivers pueden dañarse permanentemente |

### 4.2 Conexión requerida

```
ESP32 GND ───┬──────────────────────────────── STM32 GND
             │
        GND común
        (cable dedicado o plano de masa compartido)
```

- El cable GND de interconexión debe ser de **sección similar** al cable de señal
- Si los dos sistemas están en placas separadas, conectar GND con un cable dedicado (22–24 AWG es suficiente para esta distancia corta)
- El GND de ambos TJA1051T/3 ya está conectado al GND de su respectivo nodo; no hace falta un cable GND adicional entre los transceivers si el GND de los MCUs ya está unido

---

## 5. Verificación eléctrica antes de alimentar

Realizar estos pasos **en orden** antes de aplicar alimentación al sistema completo.

### Paso 1 — Todo desconectado de alimentación (bus en reposo sin tensión)

| Medición | Puntos | Valor esperado | Si falla |
|----------|--------|----------------|----------|
| Resistencia CANH–CANL | Entre los pines 7 y 6 de cualquier TJA1051T/3 | **~60 Ω** | Verificar que hay 2 resistencias de 120Ω, una en cada extremo |
| Sin cortocircuito VCC–GND (TJA1051T/3 #1) | Pin 3 vs pin 2 | > 1 kΩ | Revisar soldadura o cableado del transceiver ESP32 |
| Sin cortocircuito VCC–GND (TJA1051T/3 #2) | Pin 3 vs pin 2 | > 1 kΩ | Revisar soldadura o cableado del transceiver STM32 |
| CANH no cortocircuitado a GND | Pin 7 vs GND | > 60 Ω | Revisar cableado del bus |
| CANL no cortocircuitado a GND | Pin 6 vs GND | > 60 Ω | Revisar cableado del bus |

> Si CANH–CANL mide < 60 Ω → hay más de dos terminaciones o un cortocircuito.  
> Si CANH–CANL mide > 130 Ω → falta una terminación de 120Ω.

### Paso 2 — Alimentado, sin firmware activo (MCUs en reset o sin programar)

| Medición | Punto | Valor esperado | Si falla |
|----------|-------|----------------|----------|
| Tensión VCC TJA1051T/3 #1 | Pin 3 vs GND | **4.5 V – 5.5 V** | Verificar fuente 5V y cableado |
| Tensión VCC TJA1051T/3 #2 | Pin 3 vs GND | **4.5 V – 5.5 V** | Verificar fuente 5V y cableado |
| Tensión VIO TJA1051T/3 #1 | Pin 5 vs GND | **3.1 V – 3.5 V** | Verificar conexión VIO a 3.3V ⚠️ |
| Tensión VIO TJA1051T/3 #2 | Pin 5 vs GND | **3.1 V – 3.5 V** | Verificar conexión VIO a 3.3V ⚠️ |
| Tensión CANH en reposo | Pin 7 vs GND | **~2.5 V** | Si es 0V o 3.3V, el bus está en error; verificar alimentación y GND común |
| Tensión CANL en reposo | Pin 6 vs GND | **~2.5 V** | Igual que arriba |
| Diferencial CANH − CANL | Medir con multímetro entre pin 7 y pin 6 | **~0 V** | Si hay diferencial, hay tráfico activo o un transceiver está transmitiendo de forma incorrecta |

### Paso 3 — Alimentado, firmware activo en ambos MCUs

Con osciloscopio (recomendado) o analizador lógico de CAN:

| Observación | Valor esperado |
|-------------|----------------|
| Actividad en CANH/CANL | Ráfagas periódicas de ~10 Hz (heartbeat) |
| Nivel bit dominante: CANH | ~3.5 V |
| Nivel bit dominante: CANL | ~1.5 V |
| Diferencial dominante | ~2.0 V |
| Nivel bit recesivo: CANH y CANL | ~2.5 V (igual) |
| Diferencial recesivo | ~0 V |
| Forma de onda | Sin reflejos visibles; transiciones limpias |

Con analizador CAN (PCAN-USB, CANable, etc.):
- Conectar el analizador **en paralelo** al bus (no en serie)
- Comprobar que se reciben tramas con ID `0x001` (heartbeat STM32) y `0x011` (heartbeat ESP32)
- Verificar que no hay tramas de error CAN en el bus

---

## 6. Diagrama completo de conexión validado

```
ESP32-S3                TJA1051T/3 #1             Bus CAN              TJA1051T/3 #2           STM32G474RE
                       (lado ESP32)             (par trenzado)         (lado STM32)

GPIO4 (TX) ──────→  TXD [1]                                             TXD [1] ←────── PA12 (FDCAN1_TX)
GPIO5 (RX) ←──────  RXD [4]  [7] CANH ──┬──────────────────────┬── CANH [7]   RXD [4] ──────→ PA11 (FDCAN1_RX)
    +5V ───────────→ VCC [3]             │                      │             VCC [3] ←──────── +5V (ext.)
   +3.3V ──────────→ VIO [5] ⚠️          │                      │             VIO [5] ←──────── +3.3V ⚠️
      GND ─────────→ GND [2]          [120Ω]                 [120Ω]          GND [2] ←──────── GND
      GND ─────────→   S [8]  [6] CANL ──┴──────────────────────┴── CANL [6]   S [8] ←──────── GND
                       │                                               │
                     100nF ←→ GND                             100nF ←→ GND
                     (desacoplo VCC)                          (desacoplo VCC)

GND ESP32 ─────────────────────────────────────────────────────────────────── GND STM32
          (cable dedicado GND común — OBLIGATORIO entre los dos sistemas)
```

---

## 7. Lista de verificación antes de encender

- [ ] TJA1051T/3 #1 pin 1 (TXD) conectado a GPIO4 del ESP32
- [ ] TJA1051T/3 #1 pin 4 (RXD) conectado a GPIO5 del ESP32
- [ ] TJA1051T/3 #1 pin 3 (VCC) conectado a **+5V** (fuente externa)
- [ ] TJA1051T/3 #1 **pin 5 (VIO) conectado a +3.3V** ← ⚠️ OBLIGATORIO — protege ESP32
- [ ] TJA1051T/3 #1 pin 2 (GND) conectado a GND ESP32
- [ ] TJA1051T/3 #1 **pin 8 (S) conectado a GND** ← fácil de olvidar
- [ ] TJA1051T/3 #1 100nF entre VCC y GND
- [ ] CANH (pin 7) y CANL (pin 6) del TJA1051T/3 #1 al cable trenzado
- [ ] 120Ω entre CANH y CANL en el extremo ESP32
- [ ] CANH y CANL del cable trenzado al TJA1051T/3 #2 (pines 7 y 6)
- [ ] 120Ω entre CANH y CANL en el extremo STM32
- [ ] TJA1051T/3 #2 pin 1 (TXD) conectado a PA12 del STM32
- [ ] TJA1051T/3 #2 pin 4 (RXD) conectado a PA11 del STM32
- [ ] TJA1051T/3 #2 pin 3 (VCC) conectado a **+5V** (fuente externa)
- [ ] TJA1051T/3 #2 **pin 5 (VIO) conectado a +3.3V** ← ⚠️ OBLIGATORIO
- [ ] TJA1051T/3 #2 pin 2 (GND) conectado a GND STM32
- [ ] TJA1051T/3 #2 **pin 8 (S) conectado a GND** ← fácil de olvidar
- [ ] TJA1051T/3 #2 100nF entre VCC y GND
- [ ] **GND ESP32 y GND STM32 unidos con cable dedicado** ← OBLIGATORIO
- [ ] Resistencia CANH–CANL medida = ~60 Ω (verificación eléctrica previa)
- [ ] Sin cortocircuitos en VCC o GND de los transceivers

---

## 8. Verificación cruzada con firmware

Todos los valores de pines y configuración de este documento han sido verificados
directamente contra el código fuente del firmware:

| Parámetro | Valor documentado | Fuente firmware | Estado |
|-----------|-------------------|-----------------|--------|
| ESP32-S3 CAN TX pin | GPIO4 | `esp32/src/main.cpp`: `CAN_TX_PIN = 4` | ✅ |
| ESP32-S3 CAN RX pin | GPIO5 | `esp32/src/main.cpp`: `CAN_RX_PIN = 5` | ✅ |
| STM32 FDCAN1 TX pin | PA12 (AF9) | `Core/Inc/main.h`: `PIN_CAN_TX = GPIO_PIN_12` + `Core/Src/stm32g4xx_hal_msp.c`: `GPIO_AF9_FDCAN1` | ✅ |
| STM32 FDCAN1 RX pin | PA11 (AF9) | `Core/Inc/main.h`: `PIN_CAN_RX = GPIO_PIN_11` + `Core/Src/stm32g4xx_hal_msp.c`: `GPIO_AF9_FDCAN1` | ✅ |
| Velocidad de bus | 500 kbps | `esp32/src/main.cpp`: `convertSpeed(500)` + `Core/Src/main.c`: `170 MHz / (17 × 20) = 500 kbps` | ✅ |
| Pin S (pin 8) | Conectar a GND | Hardware externo — no controlado por firmware | ✅ |
| Terminación | 120Ω en cada extremo | Hardware externo — no controlado por firmware | ✅ |
| GND común | Obligatorio | Hardware externo — sin GND común los transceivers no operan correctamente | ✅ |

---

## 9. Seguridad de voltaje — TJA1051T/3 (NXP, VCC=5V, VIO=3.3V)

> ✅ **El TJA1051T/3 requiere VCC = 5V pero tiene un pin VIO para adaptar los niveles lógicos I/O.**
> Con VIO conectado a 3.3V, los niveles de TXD y RXD son de 3.3V, compatibles
> directamente con la ESP32-S3 y la STM32G474RE sin necesidad de divisores resistivos.

### TJA1051T/3 (NXP, VCC=5V, VIO=3.3V) — SEGURO con VIO conectado

El TJA1051T/3 se alimenta a **5V** (VCC, pin 3) pero el pin **VIO** (pin 5) conectado a **3.3V**
establece los niveles lógicos de RXD (pin 4) a 3.3V → **no hay riesgo** de dañar ni la ESP32
ni la STM32, siempre que VIO esté correctamente conectado a 3.3V.

| MCU | Pin RX | Nivel lógico transceiver | Consecuencia |
|-----|--------|-------------------------|--------------|
| **ESP32-S3** | GPIO5 | 3.3 V | ✅ Seguro (máx 3.6 V) |
| **STM32G474RE** | PA11 | 3.3 V | ✅ Seguro |

### Nota sobre transceivers sin pin VIO (MCP2551, TJA1050, TJA1051T/4, etc.)

El proyecto usa el **TJA1051T/3** que tiene pin VIO para adaptar niveles lógicos a 3.3 V.
Si por error se usa un transceiver CAN **sin pin VIO** (como el MCP2551, TJA1050, o TJA1051T**/4**),
la salida RX será de **5V** porque sigue VCC:

| MCU | Pin RX | ¿Tolera 5 V? | Consecuencia sin protección |
|-----|--------|---------------|----------------------------|
| **ESP32-S3** | GPIO5 | ❌ No (máx 3.6 V) | **Destrucción del GPIO o del chip** |
| **STM32G474RE** | PA11 | ✅ Sí (pin FT) | Seguro, pero recomendable proteger |

### Solución si se tiene un transceiver de 5V sin pin VIO

Añadir un **divisor resistivo** (1 kΩ en serie + 2 kΩ a GND) en la línea RX
de **cada** transceiver para bajar los 5 V a 3.3 V.

> ⚠️ **La solución correcta para este proyecto es usar TJA1051T/3** (con pin VIO) y conectar VIO a 3.3 V.

```
Transceiver pin 4 (RX, 5V) ──── [1 kΩ] ──┬──→ GPIO del MCU (3.3 V)
                                          │
                                       [2 kΩ]
                                          │
                                         GND

    Vout = 5 V × 2 kΩ / (1 kΩ + 2 kΩ) = 3.33 V ✅
```

La línea TX (MCU → transceiver) **no necesita divisor** porque 3.3 V es
suficiente para que el transceiver detecte un nivel HIGH (V_IH min = 2.0 V).

> **Recomendación:** Usar siempre el **TJA1051T/3** (NXP) con VIO=3.3V. **No usar TJA1051T/4** (sin VIO)
> con el ESP32-S3 sin divisores externos. Si se ha comprado un transceiver de 5V sin VIO,
> añadir los divisores resistivos **antes de alimentar**.
> Consultar `docs/ESP32_STM32_CAN_CONNECTION.md` para el diagrama completo.

---

## 10. Referencias

- `docs/ESP32_STM32_CAN_CONNECTION.md` — Guía completa de conexión CAN
- `docs/CAN_BUS_AUDIT_REPORT.md` — Auditoría firmware y protocolo CAN
- `docs/CAN_CONTRACT_FINAL.md` — Protocolo CAN rev 1.3 (IDs y DLCs)
- NXP TJA1051T/3 Datasheet: High-Speed CAN Transceiver with VIO (NXP/Nexperia)
- ISO 11898-2: Road vehicles — High-speed CAN physical layer
