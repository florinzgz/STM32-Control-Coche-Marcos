# Guía de Conexión: Encoder E6B2-CWZ6C → 3× 6N137 → STM32G474RE

> **Versión:** 3.0
> **Fecha:** 2026-04-26
> **Hardware:** 1× Omron E6B2-CWZ6C (1200 PPR, salida push-pull 5 V)
> **Interfaz:** 3× módulo breakout 6N137 canal único (flyelectronic, Taobao/AliExpress)
> **MCU:** STM32G474RE (3.3 V, TIM2 encoder mode)
> **Módulo confirmado:** 6N137 breakout single-channel — R_IN, R_PU y C_BP ya incluidos en PCB

---

## ⚠️ Aclaración crítica: tipo de salida del encoder

| Variante | Sufijo | Tipo de salida |
|----------|--------|----------------|
| E6B2-CWZ1X | **1X** | NPN colector abierto |
| E6B2-CWZ3E | **3E** | NPN colector abierto |
| E6B2-CWZ5B | **5B** | NPN colector abierto |
| **E6B2-CWZ6C** | **6C** | **Push-pull (voltage output)** ← este proyecto |

El **CWZ6C** tiene salida push-pull: el driver activo lleva la línea a +5 V (HIGH) o a GND (LOW). **No requiere resistencias pull-up externas** en el lado del encoder.

---

## 1. Pinout del encoder E6B2-CWZ6C

| Color cable (Omron estándar) | Señal | Descripción |
|------------------------------|-------|-------------|
| Marrón | VCC | +5 V alimentación |
| Azul | GND | Masa encoder (aislada de la masa STM32) |
| Negro | A | Canal A — cuadratura (push-pull 5 V) |
| Blanco | B | Canal B — cuadratura (push-pull 5 V) |
| Naranja | Z | Canal Z / índice (push-pull 5 V) |

---

## 2. Por qué 6N137 y no otras soluciones

| Opción | Viable | Motivo |
|--------|--------|--------|
| **6N137 optoacoplador** | ✅ **Elegida** | Aislamiento galvánico 2500 V + conversión 5V→3.3V simultáneas. 10 Mbps — margen de 2000× a la frecuencia del encoder (~6 kHz). Protege el STM32 de picos inductivos del motor de dirección adyacente. |
| TXS0108E | ❌ Descartada | Sin aislamiento galvánico. El motor de dirección (BTS7960, 20 kHz) genera EMI y picos inductivos que corrompan los pulsos de cuadratura. Bucles de masa entre dominio 5 V del encoder y 3.3 V del STM32 degradan la señal. |
| Divisor resistivo (R1/R2) | ❌ Descartada | Sin aislamiento. Introduce impedancia que deforma los flancos. No protege contra picos inductivos. |
| BSS138 level-shifter | ❌ Descartada | Sin aislamiento galvánico. Mismo problema que TXS0108E. |

El 6N137 es la **única solución** que proporciona aislamiento galvánico real (necesario dado que el encoder está físicamente próximo al motor de dirección con su BTS7960 conmutando a 20 kHz y corrientes de hasta 10 A).

---

## 3. Esquema de conexión — un canal (repetir ×3: A, B, Z)

```
Lado ruidoso (encoder, 5 V)          6N137              Lado lógico (STM32, 3.3 V)
───────────────────────────      ┌─────────┐         ──────────────────────────────

Encoder señal push-pull (5 V) ──[R_IN 330Ω]── Pin 2 (A)│         │ Pin 6 (Vo) ──┬── PA15 / PB3 / PB4
                                               │  6N137  │              │
GND_encoder ─────────────────────── Pin 3 (K) │         │         [4.7 kΩ]
                                               │         │              │
                                               │         │ Pin 5 (GND) ─┴── GND_STM32
                                               │         │
                                               │         │ Pin 7 (EN) ─── +3.3V (STM32)
                                               └─────────┘
                                               (Pin 1, 8 = NC)
```

**Notas:**
- `R_IN = 330 Ω` para I_F ≈ 10.6 mA con VCC encoder de 5 V: I = (5V − 1.5V) / 330Ω ≈ 10.6 mA ✅ (rango nominal 6N137: 2–15 mA)
- Pull-up de **4.7 kΩ** a **+3.3 V** en el pin de salida (Vo) hacia el STM32.
- Pin 7 (ENABLE) del 6N137 debe ir a **HIGH** (+3.3 V del lado lógico).
- GND del lado encoder y GND del lado STM32 son **eléctricamente separados** — objetivo del aislamiento. No deben conectarse entre sí en este punto.

---

## 4. Esquema completo (3 canales)

```
╔═══════════════════════════════════════════════════════════════════════════╗
║        DOMINIO ENCODER (5 V)          │    DOMINIO LÓGICO (3.3 V)        ║
╠═══════════════════════════════════════════════════════════════════════════╣

ENCODER E6B2-CWZ6C              6N137 ×3                   STM32G474RE
(salida push-pull 5V)      (aislamiento galvánico)            (3.3V)
─────────────────────      ──────────────────────         ──────────────
VCC  (marrón) ────────── +5V_encoder (fuente)
GND  (azul)   ────────── GND_encoder ─── K (pin 3) ─ (todos los 6N137)

A (negro)   ──[330Ω]──► A (pin 2) │6N137 #1│ Vo (pin 6) ──[4.7kΩ]──┬── +3.3V
                                   │        │                        └── PA15 (TIM2_CH1 AF1)
                                   │        │ EN (pin 7) ──────────────── +3.3V
                                   │        │ GND (pin 5) ─────────────── GND_STM32

B (blanco)  ──[330Ω]──► A (pin 2) │6N137 #2│ Vo (pin 6) ──[4.7kΩ]──┬── +3.3V
                                   │        │                        └── PB3  (TIM2_CH2 AF1)
                                   │        │ EN (pin 7) ──────────────── +3.3V
                                   │        │ GND (pin 5) ─────────────── GND_STM32

Z (naranja) ──[330Ω]──► A (pin 2) │6N137 #3│ Vo (pin 6) ──[4.7kΩ]──┬── +3.3V
                                   │        │                        └── PB4  (EXTI4)
                                   │        │ EN (pin 7) ──────────────── +3.3V
                                   │        │ GND (pin 5) ─────────────── GND_STM32

╚═══════════════════════════════════════════════════════════════════════════╝
```

> **⚠️ Importante — señal invertida:** La salida del 6N137 es **lógicamente invertida** respecto a la entrada (cuando el encoder da HIGH, la salida del optoacoplador da LOW). Dado que **ambos canales A y B se invierten simultáneamente**, la relación de cuadratura entre ellos se preserva. El único efecto es que el contador TIM2 incrementa en la dirección opuesta (CCW en lugar de CW). Corrección: intercambiar el cable A y B en el conector STM32, o cambiar el signo del ángulo en el firmware (`encoder_reader.c`).

---

## 5. Lista de componentes necesarios

### 5.1 Módulo breakout 6N137 (canal único) — componentes YA INCLUIDOS en PCB

Se utilizan **3 módulos breakout individuales 6N137** (proveedor: flyelectronic, Taobao/AliExpress). La PCB de cada módulo ya incorpora todos los componentes discretos necesarios:

| Componente en módulo | Marcado SMD | Valor real | Función | Estado |
|----------------------|-------------|-----------|---------|--------|
| R_IN (resistencia serie LED) | **5600** | 560 Ω | Limita I_F: (5V−1.5V)/560Ω = **6.25 mA** ✅ (rango 2–15 mA) | ✅ Incluido |
| R_PU (pull-up salida) | **472** | 4.7 kΩ | Pull-up pin 6 (Vo) → +3.3 V | ✅ Incluido |
| C_BP (desacoplo VCC) | componente amarillo | ~100 nF cerámica | Desacoplo VCC del 6N137 lado 3.3 V | ✅ Incluido |
| 6N137 (IC optoacoplador) | **6N137** (DIP-8) | — | Aislamiento galvánico + conversión 5V→3.3V | ✅ Incluido |

> **Nota sobre R_IN = 560 Ω:** El valor de 560 Ω (frente a 330 Ω nominal) da I_F = 6.25 mA, dentro del rango nominal del 6N137 (2–15 mA). La velocidad de conmutación es ligeramente inferior pero completamente irrelevante a 20 kHz frente al límite de 10 Mbps del 6N137. **No es necesario sustituir esta resistencia.**

### 5.2 Componentes EXTERNOS aún necesarios (no incluidos en el módulo)

| Ref | Componente | Valor | Cantidad | Obligatorio | Dónde va |
|-----|------------|-------|----------|-------------|----------|
| C_BULK | Condensador electrolítico | **10 µF / 25 V** | 1 | ✅ Sí | Rail +5V encoder, junto al conector de alimentación |
| FB1 | Ferrita SMD | 100 Ω @ 100 MHz | 1 | ⚠️ Recomendable | En serie en el rail +5V encoder antes del C_BULK |

> ✅ **Conclusión:** Con los 3 módulos breakout 6N137 y el condensador de 10 µF en el rail de 5 V del encoder, el circuito de aislamiento está **completamente equipado**. No hace falta ninguna resistencia adicional externa.

**Resistencias pull-up en encoder:** ❌ **No necesarias** — la salida push-pull del CWZ6C define activamente el nivel HIGH y LOW; el 6N137 solo necesita corriente de LED, que provee la salida activa del encoder.

---

## 5.3 Pinout y conexión del módulo breakout 6N137

El módulo breakout tiene pads/pines en ambos extremos. La nomenclatura típica es:

```
┌──────────────────────────────────────────────────────┐
│  Lado ENTRADA (encoder 5 V)    │  Lado SALIDA (STM32 3.3 V)  │
│                                │                              │
│  IN  ─── señal encoder (A/B/Z) │  OUT ─── PA15 / PB3 / PB4   │
│  GND ─── GND_encoder           │  VCC ─── +3.3 V (STM32)      │
│  VCC ─── +5 V encoder          │  GND ─── GND_STM32           │
└──────────────────────────────────────────────────────┘
        [R_IN 560Ω incluido]  [R_PU 4.7kΩ incluido]
                              [C_BP 100nF incluido]
```

> **⚠️ Crítico:** Los pads GND de cada lado deben ir a masas **separadas** (GND_encoder y GND_STM32 respectivamente). No conectar GND_encoder directamente a GND_STM32 — eso cortocircuitaría el aislamiento galvánico.

> **⚠️ Pin ENABLE (EN, pin 7):** En los módulos breakout ya suele estar conectado internamente a VCC del lado lógico. Si no, conectar al pad VCC del lado STM32 (+3.3 V).

---



### 6.1 Análisis de frecuencia

```
Velocidad máxima estimada del eje del encoder: ~300 RPM (reducción mecánica de la cremallera)
f_max = 1200 PPR × 300 RPM / 60 = 6 000 Hz por canal

Caso extremo teórico (1000 RPM):
f_max = 1200 × 1000 / 60 = 20 000 Hz = 20 kHz

Frecuencia máxima 6N137: 10 Mbps = 10 000 000 bps
Margen: 10.000.000 / 20.000 = ×500 — completamente seguro
```

### 6.2 Tiempos de propagación

| Parámetro | 6N137 (típico) | Máximo tolerable | ¿OK? |
|-----------|----------------|-----------------|-------|
| Retardo propagación (t_PLH / t_PHL) | 75 ns típ / 120 ns máx | 25 µs @ 20 kHz | ✅ |
| Porcentaje del periodo mínimo | 120 ns / 50 µs = **0.24 %** | — | ✅ |
| Deriva de fase A↔B | < 1 ns (canales independientes idénticos) | 200 ns | ✅ |

### 6.3 Compatibilidad con el filtro digital TIM2

El TIM2 está configurado con `IC1Filter = 6` y `IC2Filter = 6`.
Con f_DTS = 170 MHz, filtro 0110 = f_DTS/8, N=6 muestras:

```
Tiempo de rechazo de glitch = 6 × (8 / 170 MHz) ≈ 282 ns
Propagación máxima 6N137 = 120 ns < 282 ns → flancos reales pasan, glitches se rechazan ✅
```

Un flanco real del encoder atraviesa el 6N137 en 120 ns máximo, tiempo inferior al umbral de rechazo del filtro (282 ns). **No se generan falsas transiciones, no se pierden pulsos reales.**

### 6.4 Aislamiento galvánico

| Parámetro | 6N137 | Necesario |
|-----------|-------|-----------|
| Tensión de aislamiento | 2 500 V rms (VDE 0884) | Sí — bucle de masa motor 12V/24V |
| Separación GND encoder / GND STM32 | Total | Sí — motor y encoder en el mismo chasis |
| Protección contra picos inductivos | Sí — el LED absorbe el pico antes de la barrera | Sí — BTS7960 conmuta con cargas inductivas |

### 6.5 Integridad de señal

Con salida push-pull del encoder hacia el LED del 6N137:
- **Flanco de subida (encoder 0→5V):** el driver activo fuerza 5 V → corriente en LED ≈ 10.6 mA en < 10 ns → LED activa → salida LOW en 75–120 ns.
- **Flanco de bajada (encoder 5→0V):** el driver activo fuerza GND → corriente cae a 0 → LED desactiva → salida HIGH (pull-up) en 75–120 ns.

Flancos perfectamente reproducidos en el lado 3.3 V. El TIM2 ve señales limpias.

### 6.6 Veredicto de fiabilidad

| Criterio | Resultado |
|----------|-----------|
| Aislamiento galvánico real | ✅ 2 500 V — protege STM32 de picos del motor |
| Nivel de tensión compatible (5V→3.3V) | ✅ pull-up a 3.3V en la salida |
| Frecuencia dentro de límite | ✅ 20 kHz ≪ 10 Mbps |
| Retardo tolerable por filtro TIM2 | ✅ 120 ns ≪ 282 ns |
| Ruido EMI rechazado | ✅ barrera galvánica + IC filter TIM2 |
| Sin pull-up en encoder necesario | ✅ push-pull activo |
| Detección de fallo por firmware | ✅ `Encoder_CheckHealth()` activo |

**Conclusión: el circuito E6B2-CWZ6C → 3× 6N137 → STM32G474RE es la solución óptima y más robusta para este proyecto.**

---

## 7. Configuración STM32 (ya implementada en firmware)

| Parámetro | Valor | Archivo |
|-----------|-------|---------|
| Timer | TIM2, Encoder Mode 3 (`TIM_ENCODERMODE_TI12`) | `main.c` |
| Canal A | PA15, AF1 (TIM2_CH1), GPIO_NOPULL | `stm32g4xx_hal_msp.c:154` |
| Canal B | PB3, AF1 (TIM2_CH2), GPIO_NOPULL | `stm32g4xx_hal_msp.c:161` |
| Canal Z | PB4, EXTI4 — capturado en `EncoderZ_IRQHandler` (referencia de centro secundaria) | `main.h:52`, `encoder_reader.c` |
| IC1 Filter | 6 (≈ 282 ns glitch rejection) | `main.c:380` |
| IC2 Filter | 6 (≈ 282 ns glitch rejection) | `main.c:388` |
| ARR | 0xFFFFFFFF (TIM2 32-bit) | `main.c:369` |
| Resolución | 4800 counts/rev (1200 PPR × 4) | `main.h:14-15` |
| Resolución angular | 0.075° por count | `main.h:12-13` |

**`GPIO_NOPULL` es correcto:** La señal de salida del 6N137 tiene pull-up externo de 4.7 kΩ a 3.3 V. No se necesita pull-up interno del STM32.

**Nota sobre señal invertida:** Si el TIM2 cuenta en dirección opuesta a la esperada, intercambiar físicamente los cables A y B en el conector del STM32 (PA15 ↔ PB3), o ajustar el signo en `encoder_reader.c`. No se necesita cambio de configuración del periférico.

**Canal Z (PB4) como referencia de centro secundaria:** El pulso de índice del
encoder se usa como referencia de **precisión secundaria** para el centrado del
volante. El sensor inductivo PB5 (LJ12A3) sigue siendo la referencia **primaria
física y de seguridad**; el canal Z nunca centra por sí solo y un pulso Z sin
PB5 **no** es un centro. Con recorrido ≈710° (encoder 1:1 con el volante)
existen ≈2 pulsos Z dentro del rango; el Z válido es el capturado mientras PB5
confirma la zona de centro. Ver [`STEERING_Z_CENTER.md`](STEERING_Z_CENTER.md).

---

## 8. Procedimiento de verificación hardware

Una vez cableado, verificar con osciloscopio o analizador lógico (ver `docs/GUIA_ANALIZADOR_LOGICO.md`):

1. **Alimentación encoder:** +5 V estable entre cable marrón (VCC) y azul (GND). Rizado < 100 mV.
2. **Corriente LED (canal A, entrada 6N137):** Con encoder girando, verificar que la corriente alterna entre 0 y ~10 mA. Si hay un resistor de medición en serie, la caída de tensión alterna entre 0 y 3.3 V.
3. **Canal A (salida 6N137, PA15):** Señal cuadrada **0–3.3 V invertida** respecto al encoder. Al girar el volante en sentido horario, PA15 alterna entre 0 y 3.3 V.
4. **Canal B (PB3):** Idéntico al canal A pero con desfase de 90° en cuadratura.
5. **Contador TIM2:** Cambia al girar el volante (incremento en un sentido, decremento en el otro). Si ambos sentidos están intercambiados, invertir A↔B.
6. **Health check:** `Encoder_CheckHealth()` debe reportar `ENCODER_OK` durante el giro.

### Medidas esperadas con multímetro (encoder girando lentamente)

| Punto | Medida esperada |
|-------|-----------------|
| VCC encoder (marrón–azul) | 4.85 – 5.15 V |
| PA15 (GND–PA15 con encoder girando) | oscilante 0 – 3.3 V |
| PB3 (GND–PB3 con encoder girando) | oscilante 0 – 3.3 V, 90° desfasado de PA15 |
| GND_encoder – GND_STM32 | No deben medirse directamente (aislamiento galvánico) |

---

## 9. Referencias

- [Omron E6B2-CWZ6C Datasheet](https://www.ia.omron.com/products/family/487/) — "Voltage output" en especificaciones de salida
- [6N137 / HCPL-2601 Datasheet](https://www.broadcom.com/products/optocouplers/industrial-plastic/optocouplers/logic-gate-output/6n137) — velocidad, propagación, aislamiento
- `docs/AISLAMIENTO_GALVANICO_6N137.md` — plan completo de aislamiento galvánico del sistema
- `docs/ENCODER_CURRENT_STATE.md` — análisis detallado del firmware de lectura
- `docs/GUIA_ANALIZADOR_LOGICO.md` — procedimiento de verificación con analizador lógico
- `docs/HARDWARE.md` — especificación completa del hardware del sistema
