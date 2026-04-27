# REFERENCIA COMPLETA DE CABLEADO PC817
## Sistema de Control Vehicular — STM32G474RE + ESP32-S3
### Fuente de verdad: `project_config.h` y `power_manager.h`

> **Este documento es la referencia consolidada para conectar el módulo PC817
> de 8 canales. Todos los valores han sido verificados en el código fuente.**

---

## RESUMEN RÁPIDO

| Canal | Señal | Destino | Pull-up salida |
|-------|-------|---------|----------------|
| A1 | Sensor rueda FL | STM32 **PA0** (EXTI0) | **4.7 kΩ** a 3.3V |
| A2 | Sensor rueda FR | STM32 **PA1** (EXTI1) | **4.7 kΩ** a 3.3V |
| A3 | Sensor rueda RL | STM32 **PA2** (EXTI2) | **4.7 kΩ** a 3.3V |
| A4 | Sensor rueda RR | STM32 **PB15** (EXTI15) | **4.7 kΩ** a 3.3V |
| A5 | Sensor centro dirección | STM32 **PB5** (EXTI5) | **4.7 kΩ** a 3.3V |
| A6 | Encoder Z *(reservado)* | STM32 **PB4** (EXTI4) | **4.7 kΩ** a 3.3V |
| A7 | **Llave de contacto** | ESP32 **GPIO 40** | **10 kΩ** a 3.3V |

**Resistencia de entrada (lado 12V) TODOS los canales: 1 kΩ ¼W**

---

## COMPONENTES A AÑADIR AL MÓDULO PC817

> ⚠️ El módulo PC817 de 8 canales **NO incluye pull-up de salida**.
> Verificado por medición (comentario en `power_manager.h`).
> Deben soldarse externamente.

### Por cada canal (×7 canales usados):

| Componente | Valor | Dónde va |
|-----------|-------|----------|
| **Resistencia entrada** | **1 kΩ ¼W** | En serie con **IN+** (ánodo LED), lado 12V |
| **Pull-up salida** (A1–A5, A6) | **4.7 kΩ 1/8W** | Entre **colector (OUT)** y **+3.3V**, lado lógico |
| **Pull-up salida** (A7 — llave) | **10 kΩ 1/8W** | Entre **colector (OUT)** y **+3.3V**, lado ESP32 |

### Por qué 1 kΩ en la entrada (no 330 Ω):
```
I_LED = (12V − 1.2V) / 1kΩ = 10.8 mA  ← dentro del límite continuo (20 mA)
I_LED = (12V − 1.2V) / 330Ω = 32.7 mA ← EXCEDE el límite continuo ❌
```

### Por qué 4.7 kΩ (no 10 kΩ) en los sensores de rueda:
El pull-up interno del STM32 (~40 kΩ) es demasiado lento para los flancos del
sensor inductivo con cable de hasta 5 m. El 4.7 kΩ externo (en paralelo con el
interno = 4.2 kΩ efectivo) garantiza t_flanco ≤ 5 µs con hasta 1 nF de capacidad
de cable. Ver cálculo completo en `docs/VALIDACION_CAN_PULLUP_PC817.md §2`.

---

## ESQUEMA DE CONEXIÓN (igual para todos los canales A1–A6)

```
════════════ LADO 12V (entrada) ════════════

  +12V  ──[1kΩ ¼W]──► PC817 IN+  (ánodo LED)
  GND   ─────────────► PC817 IN-  (cátodo LED)

  Sensores LJ12A3-4-Z/BX (inductivos NPN):
    Cable AZUL  (+12V)  → a través de 1kΩ → IN+
    Cable NEGRO (GND)   → IN-
    Cable MARRÓN (señal) → no usar directamente (salida colector abierto)

    NOTA: los sensores LJ12A3 son NPN salida activa-baja.
    Conectar +12V al IN+ y GND al IN- es lo correcto:
    cuando el sensor detecta metal → su salida va a LOW → IN- baja → LED conduce.

════════════ LADO 3.3V (salida) ════════════

  +3.3V ──[4.7kΩ 1/8W]──┬──► PC817 OUT (colector)
                          │
                     ──►  STM32 GPIO pin destino (ver tabla arriba)

  PC817 GND (emisor)  ──► GND lógico (masa STM32)
```

## ESQUEMA CANAL A7 — Llave de contacto (ESP32)

```
════════════ LADO 12V (entrada) ════════════

  Terminal ON del contacto → [1kΩ ¼W] → PC817 A7 IN+
  GND del vehículo         →            PC817 A7 IN-

════════════ LADO 3.3V (salida) ════════════

  +3.3V ──[10kΩ 1/8W]──┬──► PC817 A7 OUT (colector)
                         │
                    ──►  ESP32 GPIO 40

  PC817 GND (emisor)  ──► GND ESP32
```

---

## LÓGICA INVERTIDA (PC817 invierte la señal)

### Sensores de rueda / centro dirección (A1–A6 → STM32):
| Estado sensor | LED PC817 | GPIO STM32 | Interpretación firmware |
|---------------|-----------|------------|------------------------|
| Detecta metal (activo) | Encendido | **LOW** | Pulso detectado ✅ |
| No detecta | Apagado | **HIGH** (pull-up) | Sin pulso |

El firmware configura estos pines como `GPIO_PULLUP` + `EXTI` en flanco de bajada.

### Llave de contacto (A7 → ESP32 GPIO 40):
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

## LISTA DE COMPRA DE RESISTENCIAS (solo PC817)

| Cant. | Valor | Tipo | Para |
|-------|-------|------|------|
| 7 | **1 kΩ ¼W** | Carbón o film | Entrada lado 12V de cada canal (A1–A7) |
| 6 | **4.7 kΩ 1/8W** | Film metal | Pull-up salida A1–A6 (sensores → STM32) |
| 1 | **10 kΩ 1/8W** | Film metal | Pull-up salida A7 (llave → ESP32 GPIO 40) |

*(Para el divisor del pedal: 1× 10 kΩ + 1× 6.8 kΩ — pedido por separado)*

---

## NOTAS DE MONTAJE

1. **Lado 12V** (IN+/IN-) y **lado 3.3V** (OUT/GND) son galvánicamente aislados.
   No conectar GND de ambos lados al mismo punto.
2. Las resistencias de pull-up (4.7 kΩ / 10 kΩ) se pueden soldar directamente
   sobre los pines OUT del módulo o en una placa de prototipos aparte.
3. La resistencia de entrada 1 kΩ puede ir en el módulo o en el extremo del cable
   junto al sensor (reduce ruido en el cable).
4. El módulo necesita **alimentación 3.3V en el lado lógico** (no 5V).
   Usar el LDO externo AMS1117-3.3, no el pin 3.3V de la Nucleo-64.

---

*Documento creado a partir de `power_manager.h`, `project_config.h` y*
*`docs/VALIDACION_CAN_PULLUP_PC817.md`. Fecha: 2026-04-27.*
