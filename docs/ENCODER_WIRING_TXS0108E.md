# Guía de Conexión: Encoder E6B2-CWZ6C → TXS0108E → STM32G474RE

> **Versión:** 1.0  
> **Fecha:** 2026-04-26  
> **Hardware:** 1× Omron E6B2-CWZ6C (1200 PPR, salida push-pull 5 V)  
> **Level-shifter:** Texas Instruments TXS0108E (8 canales bidireccional 1.2–3.6 V / 1.65–5.5 V)  
> **MCU:** STM32G474RE (3.3 V, TIM2 encoder mode)

---

## ⚠️ Aclaración crítica: tipo de salida del encoder

| Variante | Sufijo | Tipo de salida |
|----------|--------|----------------|
| E6B2-CWZ1X | **1X** | NPN colector abierto |
| E6B2-CWZ3E | **3E** | NPN colector abierto |
| E6B2-CWZ5B | **5B** | NPN colector abierto |
| **E6B2-CWZ6C** | **6C** | **Push-pull (voltage output)** ← este proyecto |

El **CWZ6C** tiene salida push-pull: el driver activo lleva la línea a +5 V (HIGH) o a GND (LOW). **No requiere resistencias pull-up externas.** La documentación anterior que indica "NPN open-collector" era incorrecta para esta variante.

---

## 1. Pinout del encoder E6B2-CWZ6C

| Color cable (Omron estándar) | Señal | Descripción |
|------------------------------|-------|-------------|
| Marrón | VCC | +5 V alimentación |
| Azul | GND | Masa común |
| Negro | A | Canal A — cuadratura (push-pull 5 V) |
| Blanco | B | Canal B — cuadratura (push-pull 5 V) |
| Naranja | Z | Canal Z / índice (push-pull 5 V) |

---

## 2. Por qué se necesita un level-shifter

El STM32G474RE opera a **3.3 V**. Sus pines GPIO toleran 5 V **solo en modo analógico**; en modo digital el límite es VDD + 0.3 V = **3.6 V máximo** (datasheet STM32G4, §6.3.1). Las salidas del E6B2-CWZ6C alcanzan los **5 V del VCC del encoder**, por lo que conectar directamente dañaría el MCU.

### Por qué TXS0108E y no alternativas

| Opción | Viable | Motivo |
|--------|--------|--------|
| **TXS0108E** | ✅ Sí | Push-pull ↔ push-pull, bidireccional, hasta 110 Mbps. Ideal para señales push-pull. |
| Divisor resistivo (R1/R2) | ⚠️ Marginal | Introduce impedancia y aumenta el tiempo de subida; puede causar errores a alta frecuencia. |
| 6N137 optoacoplador | ✅ Sí (overkill) | Proporciona aislamiento galvánico. Añade retardo ~50 ns e invierte la señal. Usar solo si hay riesgo de diferencial de masa real entre encoder y STM32. |
| SN74AHCT125 | ✅ Sí | Más robusto unidireccionalmente, pero requiere cuatro CI para A+B+Z+reserva. |

El TXS0108E es la **solución óptima** para este caso: señal push-pull, velocidad muy inferior a su límite (667 Hz vs 110 Mbps), y solo 1 CI para los tres canales.

---

## 3. Esquema de conexión completo

```
╔═══════════════════════════════════════════════════════════════════════════╗
║              FUENTE 5 V                      FUENTE 3.3 V                ║
║                  │                               │                       ║
║      ┌───────────┤                   ┌───────────┤                       ║
║      │           │                   │           │                       ║
║      C3 (10µF)  C1 (100nF)          C2 (100nF)  │                       ║
║      │           │                   │           │                       ║
║      └─────┬─────┘                   └─────┬─────┘                       ║
║            │                               │                             ║
║            GND                             GND                           ║
╚═══════════════════════════════════════════════════════════════════════════╝

 ENCODER E6B2-CWZ6C              TXS0108E                 STM32G474RE
 (salida push-pull 5V)     (level-shifter 5V/3.3V)          (3.3V)
 ─────────────────────     ───────────────────────       ──────────────
                            VB (pin 10) ◄─── +5V
                            VA (pin 1)  ◄─── +3.3V
                            OE (pin 19) ◄─── +3.3V   ← siempre habilitado
                            GND (pin 11) ── GND

VCC (marrón) ─────────────────────────────────────── +5V (fuente)
GND (azul)   ──────────────────────────────────────── GND

A (negro)    ──────────► B1 (pin 2) ──A1 (pin 18)──► PA15  [TIM2_CH1, AF1]
B (blanco)   ──────────► B2 (pin 4) ──A2 (pin 17)──► PB3   [TIM2_CH2, AF1]
Z (naranja)  ──────────► B3 (pin 6) ──A3 (pin 16)──► PB4   [EXTI4 — reserva]

                          B4–B8: no conectar
                          A4–A8: no conectar
```

> **Nota:** B1–B8 = lado 5 V del TXS0108E. A1–A8 = lado 3.3 V.  
> El sentido de la traducción lo determina automáticamente el circuito de detección de flanco del TXS0108E; no hay que configurar nada.

---

## 4. Lista de componentes adicionales necesarios

| Ref | Componente | Valor / Referencia | Cantidad | Obligatorio |
|-----|------------|-------------------|----------|-------------|
| U1 | Level-shifter | TXS0108E (SOIC-20 o módulo breakout) | 1 | ✅ Sí |
| C1 | Condensador cerámico | 100 nF / 10 V (X7R) | 1 | ✅ Sí (desacoplo VA) |
| C2 | Condensador cerámico | 100 nF / 10 V (X7R) | 1 | ✅ Sí (desacoplo VB) |
| C3 | Condensador electrolítico | 10 µF / 10 V | 1 | ✅ Sí (bulk 5 V encoder) |
| FB1 | Ferrita SMD | 100 Ω @ 100 MHz (p.ej. BLM18AG121SN1) | 1 | ⚠️ Recomendable |

**Resistencias pull-up:** ❌ **No necesarias** — la salida push-pull del CWZ6C define activamente el nivel HIGH y LOW; no hay estado de alta impedancia.

**Diodos:** ❌ **No necesarios** — no hay riesgo de inversión de tensión en este circuito.

---

## 5. Verificación de fiabilidad con TXS0108E

### 5.1 Análisis de frecuencia

```
Frecuencia máxima del encoder:
  f_max = PPR × (ω_max / 360°)
        = 1200 × (200°/s / 360°)     ← velocidad máxima de volante
        = 667 Hz  (canal A o B)

Frecuencia máxima TXS0108E: 110 Mbps (push-pull)
Margen: 110.000.000 / 667 ≈ ×165.000 — completamente seguro
```

### 5.2 Tiempos de propagación

| Parámetro | TXS0108E (típico) | Máximo tolerable | ¿OK? |
|-----------|-------------------|-----------------|-------|
| Retardo propagación (B→A) | ~3.5 ns | 750 ns @ 667 Hz | ✅ |
| Tiempo de subida/bajada | ~10 ns | 750 ns | ✅ |
| Deriva de fase A↔B | < 1 ns (ambos canales del mismo CI) | 200 ns | ✅ |

El retardo de 3.5 ns del TXS0108E es **negligible** frente a los filtros de entrada del TIM2 (IC filter = 6 → ~210 ns). El decodificador de cuadratura no ve ninguna diferencia respecto a una señal directa.

### 5.3 Compatibilidad con el filtro digital TIM2

El TIM2 está configurado con `IC1Filter = 6` y `IC2Filter = 6` (6 muestras a 170 MHz ≈ 210 ns de ventana de rechazo). El retardo máximo del TXS0108E es ~10 ns, muy inferior a los 210 ns del filtro. **No hay pérdida de pulsos ni errores de decodificación.**

### 5.4 Integridad de señal

Con salida push-pull:
- **Flanco de bajada:** muy rápido (el driver activo fuerza GND)
- **Flanco de subida:** muy rápido (el driver activo fuerza 5 V)
- **Sin tiempo de carga RC:** no hay resistencia pull-up que limite la corriente de carga

El TXS0108E reproduce fielmente ambos flancos en el lado 3.3 V.

### 5.5 Alimentación y ruido

| Riesgo | Mitigación |
|--------|------------|
| Ruido de alta frecuencia en VCC 5 V (PWM 20 kHz de BTS7960) | C3 (10 µF bulk) + FB1 (ferrita 100 Ω) en la línea +5 V del encoder |
| Oscilación del TXS0108E | C1 + C2 (100 nF cerámico junto al CI) |
| Señal flotante si cable encoder se desconecta | Encoder push-pull siempre fuerza un nivel → sin flote. Con cable desconectado la entrada del TXS0108E flota, pero el IC filter del TIM2 lo detectaría como ruido y la función `Encoder_CheckHealth()` (ENC_FROZEN_TIMEOUT_MS = 200 ms) activaría FAULT_ENCODER. |

### 5.6 Veredicto de fiabilidad

| Criterio | Resultado |
|----------|-----------|
| Nivel de tensión compatible | ✅ 5 V → 3.3 V dentro de rango TXS0108E |
| Frecuencia dentro de límite | ✅ 667 Hz ≪ 110 Mbps |
| Retardo tolerable por filtro TIM2 | ✅ 10 ns ≪ 210 ns |
| Ruido rechazado | ✅ C1, C2, C3 + IC filter TIM2 |
| Sin pull-up necesario | ✅ Push-pull activo |
| Detección de fallo por firmware | ✅ `Encoder_CheckHealth()` activo |

**Conclusión: el circuito E6B2-CWZ6C → TXS0108E → STM32G474RE es completamente funcional y fiable en este proyecto.**

---

## 6. Configuración STM32 (ya implementada en firmware)

| Parámetro | Valor | Archivo |
|-----------|-------|---------|
| Timer | TIM2, Encoder Mode 3 (`TIM_ENCODERMODE_TI12`) | `main.c` |
| Canal A | PA15, AF1 (TIM2_CH1), GPIO_NOPULL | `stm32g4xx_hal_msp.c:154` |
| Canal B | PB3, AF1 (TIM2_CH2), GPIO_NOPULL | `stm32g4xx_hal_msp.c:161` |
| Canal Z | PB4, EXTI4 — definido, no activo en firmware v1 | `main.h:52` |
| IC1 Filter | 6 (≈ 210 ns glitch rejection) | `main.c:380` |
| IC2 Filter | 6 (≈ 210 ns glitch rejection) | `main.c:388` |
| ARR | 0xFFFFFFFF (TIM2 32-bit) | `main.c:369` |
| Resolución | 4800 counts/rev (1200 PPR × 4) | `main.h:14-15` |
| Resolución angular | 0.075° por count | `main.h:12-13` |

**`GPIO_NOPULL` es correcto** con salida push-pull: la señal nunca flota, el driver activo del encoder define el nivel en todo momento. No hace falta pull-up interno ni externo.

---

## 7. Procedimiento de verificación hardware

Una vez cableado, verificar con osciloscopio o analizador lógico (ver `docs/GUIA_ANALIZADOR_LOGICO.md`):

1. **Alimentación:** +5 V estable entre VCC encoder (marrón) y GND (azul). Rizado < 100 mV.
2. **Canal A (lado 5 V, B1 TXS0108E):** señal cuadrada 0–5 V al girar el volante.
3. **Canal A (lado 3.3 V, A1 TXS0108E / PA15):** misma señal cuadrada 0–3.3 V. Desfase < 10 ns respecto al lado 5 V.
4. **Canal B (B2/A2):** idéntico al canal A pero 90° desfasado en cuadratura.
5. **Contador TIM2:** incrementa girando en sentido horario, decrementa en antihorario (o viceversa según montaje mecánico — invertir A y B si es al revés).
6. **Health check:** `Encoder_CheckHealth()` debe reportar `ENCODER_OK` durante el giro.

---

## 8. Referencias

- [Omron E6B2-CWZ6C Datasheet](https://www.ia.omron.com/products/family/487/) — confirmar "Voltage output" en especificaciones de salida
- [TI TXS0108E Datasheet](https://www.ti.com/lit/ds/symlink/txs0108e.pdf)
- `docs/ENCODER_CURRENT_STATE.md` — análisis detallado del firmware de lectura
- `docs/GUIA_ANALIZADOR_LOGICO.md` — procedimiento de verificación con analizador lógico
- `docs/HARDWARE.md` — especificación completa del hardware del sistema
