# Protección Inrush — Configuración Buck 5V Lógica + Buck 5V LEDs

Análisis y guía de conexión para la configuración de doble Buck separado:
- **Buck 1:** 5V / 3A → STM32 Nucleo, ESP32-S3, Encoder E6B2-CWZ6C
- **Buck 2:** 5V / 5A → Tiras WS2812B (28 LEDs frontal + 16 LEDs trasera)

---

## 1. Distribución de cargas

| Buck | Tensión / Corriente | Consumidores | Consumo estimado |
|------|---------------------|-------------|-----------------|
| **Buck 1** (lógica) | 5V / 3A | STM32 Nucleo (~100 mA) + ESP32-S3 (~500 mA) + Encoder E6B2-CWZ6C (~80 mA) + CAN transceivers ×2 (~140 mA) + DFPlayer Mini (~200 mA) | ~1.02 A típico |
| **Buck 2** (LEDs) | 5V / 5A | Tira frontal 28× WS2812B (máx 1.68 A) + Tira trasera 16× WS2812B (máx 0.96 A) | 2.64 A máx / ~0.8 A típico |

El Buck 1 tiene margen amplio (3A disponibles, ~1A de uso). El Buck 2 cubre el pico máximo de 2.64A con margen.

---

## 2. Por qué son necesarios los condensadores de entrada

Cuando un Buck arranca (al conectar la batería 24V), sus condensadores de salida están descargados y piden una corriente de carga elevada durante los primeros microsegundos (**inrush current**). Sin protección:

- Se produce una caída brusca de tensión en la batería
- Puede provocar un reset del STM32 / ESP32 si comparten bus de alimentación
- Genera interferencias EMI en el bus CAN, I2C y ADC

El **condensador electrolítico en la entrada** del Buck (lado 24V) absorbe este pico y lo "suaviza" para que la batería no lo vea directamente.

---

## 3. Componentes necesarios por Buck

### Buck 1 — 5V / 3A (lógica + encoder)

| Componente | Valor | Ubicación | Función |
|-----------|-------|----------|---------|
| Condensador electrolítico | **470 µF / 35V** | Entrada Buck 1 (lado VIN 24V), lo más cerca posible de los pines VIN y GND | Absorbe el pico inrush al arrancar |
| Condensador cerámico | **100 nF / 50V X7R** | En paralelo con el electrolítico de entrada | Filtra ruido de alta frecuencia del switching |
| Condensador electrolítico | **47 µF / 10V** | Salida Buck 1 (5V), antes de distribuir a las cargas | Estabiliza la salida, "bulk" 5V lógica |
| Condensador cerámico | **100 nF / 10V X7R** | En paralelo con el electrolítico de salida | Desacoplo rápido (<1 ns) para lógica digital |
| Fusible blade slow-blow | **5A** | Salida 5V del Buck 1, antes de la distribución | Protección contra cortocircuito |

### Buck 2 — 5V / 5A (tiras LED)

| Componente | Valor | Ubicación | Función |
|-----------|-------|----------|---------|
| Condensador electrolítico | **1000 µF / 35V** | Entrada Buck 2 (lado VIN 24V) | Mayor capacidad por mayor corriente de las tiras |
| Condensador cerámico | **100 nF / 50V X7R** | En paralelo con el electrolítico de entrada | Filtro HF |
| Condensador electrolítico ×2 | **1000 µF / 10V** (uno por tira) | Salida 5V, lo más cerca posible del conector VCC de cada tira WS2812B | Absorbe el inrush al encender los 44 LEDs |
| Fusible blade slow-blow | **5A** | Salida 5V del Buck 2 | Protección contra cortocircuito de las tiras |

---

## 4. Lista de compra (BOM)

| Qty | Componente | Valor | Para qué |
|-----|-----------|-------|---------|
| 1 | Condensador electrolítico | **470 µF / 35V** | Entrada Buck 1 |
| 1 | Condensador electrolítico | **1000 µF / 35V** | Entrada Buck 2 |
| 2 | Condensador cerámico | **100 nF / 50V X7R** | Entrada de cada Buck |
| 1 | Condensador electrolítico | **47 µF / 10V** | Salida Buck 1 |
| 2 | Condensador electrolítico | **1000 µF / 10V** | Salida Buck 2 (uno por tira) |
| 2 | Condensador cerámico | **100 nF / 10V X7R** | Salida de cada Buck |
| 2 | Fusible blade slow-blow | **5A** | Salida de cada Buck |

---

## 5. Esquema de conexión

```
BAT 24V (+) ────┬──────────────────────────────────────────────┐
                │                                              │
           [470µF/35V] ║ [100nF/50V]                    [1000µF/35V] ║ [100nF/50V]
           (entrada)   ║ (en paralelo)                  (entrada)    ║ (en paralelo)
                │                                              │
                ▼                                              ▼
          ┌──────────┐                                   ┌──────────┐
          │  BUCK 1  │                                   │  BUCK 2  │
          │  5V / 3A │                                   │  5V / 5A │
          └────┬─────┘                                   └────┬─────┘
               │  5V                                          │  5V
          [Fusible 5A]                                   [Fusible 5A]
               │                                              │
     [47µF/10V] ║ [100nF/10V]                   [1000µF/10V]  [1000µF/10V]
     (salida)   ║ (en paralelo)                 (tira frontal) (tira trasera)
               │                                              │
     ┌─────────┼──────────────┐                    ┌─────────┴─────────┐
     │         │              │                    │                   │
 STM32 5V  ESP32-S3 5V   Encoder E6B2               Relé PB10          Relé PB11
 (Nucleo)  (DevKitC)     CWZ6C (VCC 5V)             │                   │
                                                Tira frontal       Tira trasera
                                                28× WS2812B        16× WS2812B

BAT 24V (−) ────────── GND ──── ★ PUNTO ESTRELLA ─── GND Buck 1 + GND Buck 2
```

### Regla de conexión de masas (OBLIGATORIO)

```
                    ★ PUNTO ESTRELLA (barra de masa)
                    │
     ┌──────────────┼──────────────┬──────────────┐
     │              │              │              │
  GND Buck 1    GND Buck 2    GND Batería    GND Relés / BTS7960
  (lógica)      (LEDs)          24V
```

**Los dos Bucks DEBEN compartir el mismo GND en el punto estrella.** Si no comparten GND, la señal de datos del ESP32 (Buck 1) hacia las tiras WS2812B (Buck 2) no tiene referencia de masa común → los LEDs no funcionarán o mostrarán colores aleatorios.

---

## 6. Encoder E6B2-CWZ6C — Protección de pines STM32

El encoder se alimenta a **5V** (desde Buck 1) pero sus salidas son también a 5V. Los pines del STM32 son de 3.3V → **se necesita divisor resistivo** en cada señal (A, B, Z):

```
Encoder A (5V) ──[1kΩ]──┬──► PA15 (STM32 TIM2_CH1, 3.3V)
                         │
                       [2.2kΩ]
                         │
                        GND

Encoder B (5V) ──[1kΩ]──┬──► PB3  (STM32 TIM2_CH2, 3.3V)
                         │
                       [2.2kΩ]
                         │
                        GND

Encoder Z (5V) ──[1kΩ]──┬──► PB4  (STM32 EXTI4, 3.3V)
                         │
                       [2.2kΩ]
                         │
                        GND
```

Tensión resultante en cada pin STM32: 5V × 2.2/(1+2.2) = **3.44V** ✅ (dentro del rango del pin)

Alternativa con mejor aislamiento: optoacoplador **6N137** en cada señal (A, B, Z) con resistencia de 220Ω en serie.

---

## 7. Secuencia de arranque

El firmware ya gestiona la secuencia con `RELAY_MAIN_SETTLE_MS = 50 ms`:

1. Al conectar batería → los Bucks arrancan; los condensadores de entrada absorben el inrush
2. STM32 y ESP32-S3 arrancan (~50-100 ms)
3. Firmware espera 50 ms antes de activar RELAY_MAIN
4. Activa RELAY_TRAC → tracción disponible
5. Activa RELAY_DIR → dirección disponible
6. Activa RELAY_LED (PB10) y RELAY_LED_REAR (PB11) → LEDs disponibles

Los condensadores de salida del Buck 2 (1000 µF × 2) absorben el inrush de las tiras LED al activar los relés.

---

## 8. Referencias del proyecto

- `docs/POWER_DISTRIBUTION.md` — Arquitectura completa de distribución de potencia
- `Documentos/SISTEMA_ALIMENTACION_COMPLETO.md` — BOM completo y esquemas detallados
- `docs/ENCODER_CURRENT_STATE.md` — Estado del encoder E6B2-CWZ6C
- `Core/Src/safety_system.c` — Umbrales de batería y lógica de seguridad
- `Core/Inc/project_config.h` — `RELAY_MAIN_SETTLE_MS = 50`
