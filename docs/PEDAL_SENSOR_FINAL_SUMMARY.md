# ESTADO FINAL DEL SENSOR DE PEDAL — Resumen Visual

> **Fecha:** 2026-03-03
> **Estado:** ✅ Actualizado — ADC interno STM32 con plausibilidad software
> **Sensor:** SS1324LUA-T (Hall magnético, 5V, salida 0.3V–4.8V)
> **Arquitectura:** ADC interno dual-sample + plausibilidad software

---

## 1. ESQUEMA DE CONEXIÓN COMPLETO

```
                              ┌─────────────────────────┐
                              │   PEDAL ACELERADOR      │
                              │   (SS1324LUA-T, 5V)     │
                              │                         │
                              │  Pin 1 (VCC) ──► 5V     │
                              │  Pin 2 (GND) ──► GND    │
                              │  Pin 3 (Señal) ────────►│ Salida: 0.3V – 4.8V
                              └─────────────────┼───────┘
                                                │
                                                │
                      ┌─────────────────────────┤
                      │   DIVISOR DE TENSIÓN    │
                      │                         │
                      │   Señal (5V) ──┐        │
                      │                R1       │
                      │            10 kΩ (1%)   │
                      │                │        │
                      │           NODO ◄────────┼───► PA3 (ADC1_IN4)
                      │                │        │
                      │                R2       │
                      │            6.8 kΩ (1%)  │
                      │                │        │
                      │               GND       │
                      └─────────────────────────┘
                                       │
                                       ▼
                      ┌───────────────────────────┐
                      │    STM32G474RE             │
                      │                            │
                      │    PA3 (ADC1_IN4)          │ ◄── Señal dividida: 0.12V – 1.94V
                      │                            │
                      │    Plausibilidad software: │
                      │    · Dual-sample ADC       │
                      │    · Filtro EMA (α=0.3)    │
                      │    · Validación de rango   │
                      │    · Límite tasa de cambio │
                      └────────────────────────────┘
```

---

## 2. CABLES — Lista exacta

### Canal único (divisor → PA3)

| Nº | De | A | Función |
|----|-----|---|---------|
| 1 | Pedal Pin 1 (VCC) | **5V** fuente | Alimentación sensor |
| 2 | Pedal Pin 2 (GND) | **GND** fuente | Masa sensor |
| 3 | Pedal Pin 3 (Señal) | **R1 entrada** (10 kΩ) | Señal 0.3V–4.8V |
| 4 | R1 salida / R2 entrada (nodo) | **PA3** (STM32) | Señal dividida ~0.12V–1.94V |
| 5 | R2 salida | **GND** | Cierre divisor |

> **Total: 5 cables** (divisor de tensión solamente — ADS1115 eliminado)

---

## 3. CÁLCULO DEL DIVISOR

```
Fórmula:  Vout = Vin × R2/(R1+R2)

R1 = 10 kΩ  (resistencia serie, 1% precisión)
R2 = 6.8 kΩ (resistencia a GND, 1% precisión)

Ratio = 6.8 / (10 + 6.8) = 0.4048

Pedal suelto (0.3V): 0.3 × 0.4048 = 0.121V → ADC: ~150 counts
Pedal pisado (4.8V): 4.8 × 0.4048 = 1.943V → ADC: ~2413 counts

MÁXIMO absoluto en PA3: 5.0 × 0.4048 = 2.024V (muy por debajo de 3.3V)
```

---

## 4. QUÉ HACE EL FIRMWARE — Paso a paso

### Cada 50 ms (ciclo principal del main loop):

```
1. Pedal_Update() se ejecuta:
   │
   ├── Paso 1: Doble lectura ADC interno (~2 µs total)
   │   Lectura 1: HAL_ADC_Start → PollForConversion → GetValue → Stop
   │   Lectura 2: HAL_ADC_Start → PollForConversion → GetValue → Stop
   │   pedal_raw_adc + pedal_raw_adc2 = dos valores 12-bit
   │
   ├── Paso 2: Verificación de consistencia dual-sample
   │   Si |sample1 - sample2| > 30 counts (PEDAL_SAMPLE_TOLERANCE):
   │   ❌ pedal_channels_contradict = true → fallo transitorio
   │
   ├── Paso 3: Media de ambas muestras
   │   avg_raw = (pedal_raw_adc + pedal_raw_adc2) / 2
   │
   ├── Paso 4: Validación de rango
   │   Si avg_raw < 30 (FAULT_LO) → cable abierto / sensor sin alimentar
   │   Si avg_raw > 2800 (FAULT_HI) → cortocircuito a alimentación
   │   ❌ pedal_plausible = false
   │
   ├── Paso 5: Conversión a porcentaje (0–100%)
   │   pedal_pct_raw = mapeo lineal de [150..2413] a [0%..100%]
   │
   ├── Paso 6: Filtro EMA (Exponential Moving Average)
   │   pedal_ema = 0.3 × pedal_pct_raw + 0.7 × pedal_ema_anterior
   │   Suaviza ruido sin añadir lag significativo (~150 ms settling)
   │
   ├── Paso 7: Límite de tasa de cambio
   │   Si |pedal_pct_raw - valor_anterior| > 35% por ciclo:
   │   ❌ pedal_plausible = false → salto no físicamente posible
   │
   └── Paso 8: Todo OK → actualizar salida
       ✅ pedal_plausible = true
       pedal_pct = pedal_ema (valor filtrado para control)
```

### En Safety_CheckSensors() (cada 50 ms también):

```c
if (!Pedal_IsPlausible()) {
    Traction_SetDemand(0.0f);        // Fuerza acelerador a 0%
    Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
    Safety_SetState(SYS_STATE_LIMP_HOME);
}
```

---

## 5. CALIBRACIÓN EN EL FIRMWARE

### Fichero: `Core/Src/sensor_manager.c`

| Constante | Valor | Significado |
|-----------|-------|-------------|
| `PEDAL_ADC_MIN` | 150 | ADC 12-bit para 0.3V (pedal suelto), con divisor |
| `PEDAL_ADC_MAX` | 2413 | ADC 12-bit para 4.8V (pedal pisado), con divisor |
| `PEDAL_ADC_FAULT_LO` | 30 | Umbral bajo para detección circuito abierto |
| `PEDAL_ADC_FAULT_HI` | 2800 | Umbral alto para detección cortocircuito |
| `PEDAL_SAMPLE_TOLERANCE` | 30 | Tolerancia entre muestras consecutivas (counts) |
| `PEDAL_MAX_RATE_PCT` | 35.0% | Máximo cambio por ciclo (50 ms) |
| `PEDAL_EMA_ALPHA` | 0.3 | Coeficiente filtro EMA |
| `PEDAL_PLAUSIBILITY_PCT` | 5.0% | Tolerancia de plausibilidad |

---

## 6. PINES GPIO USADOS

| Pin STM32 | Función | Periférico | Señal |
|-----------|---------|------------|-------|
| **PA3** | ADC1_IN4 | ADC1 | Pedal (0.12V–1.94V vía divisor) |

> **Nota:** PB8/PB9 (I2C1) ya NO se usan para el pedal. El bus I2C
> queda dedicado exclusivamente a INA226/TCA9548A.

---

## 7. MATERIAL NECESARIO (BOM pedal)

| Cantidad | Componente | Especificación | Precio aprox. |
|----------|-----------|----------------|---------------|
| 1 | SS1324LUA-T | Sensor Hall, alimentar a 5V | (ya lo tienes) |
| 1 | Resistencia 10 kΩ | 1% precisión, ¼W | ~0.05 € |
| 1 | Resistencia 6.8 kΩ | 1% precisión, ¼W | ~0.05 € |

> **ADS1115 eliminado** — ya no es necesario.

---

## 8. JUSTIFICACIÓN TÉCNICA — Por qué se eliminó el ADS1115

### Problemas del ADS1115 para el pedal:

| Aspecto | ADS1115 (I2C) | ADC interno STM32 |
|---------|---------------|-------------------|
| **Latencia** | ~10 ms (8 ms conversión + I2C) | ~1 µs × 2 = ~2 µs |
| **Bloqueo** | HAL_Delay(8) bloquea el bucle | Sin bloqueo |
| **Frecuencia** | 128 SPS máximo | >100 kSPS (12-bit) |
| **Bus compartido** | I2C con INA226/TCA9548A → contención | ADC dedicado |
| **Determinismo** | Variable (I2C arbitración, recovery) | Determinístico |
| **Componentes** | Módulo externo + 6 cables extra | Ninguno adicional |
| **Fallos** | I2C puede colgarse → recovery complejo | Solo ADC, fiable |

### Conclusión:
El ADC interno del STM32G474RE (12-bit, ~1 µs, calibrado) es **5000× más rápido**
y **completamente determinístico**. La plausibilidad se consigue con técnicas software
(dual-sample, EMA, rango, tasa de cambio) que son más robustas que depender de un
bus I2C compartido que puede fallar.

---

## 9. NOTAS IMPORTANTES

### ⚠️ Lo que NO debes hacer:
- **NO conectar** la señal de 5V del pedal directamente a PA3 → quemaría el STM32 (máx 3.6V)
- **NO alimentar** el SS1324LUA-T a 3.3V → mínimo es 4.5V según datasheet Allegro
- **NO usar** resistencias de 5% → pueden dar error de calibración de ±3%

### ✅ Lo que SÍ está implementado:
- Doble lectura ADC instantánea (~2 µs) con verificación de consistencia
- Filtro EMA para rechazo de ruido sin retardo significativo
- Detección de circuito abierto y cortocircuito por rango
- Detección de saltos imposibles por tasa de cambio
- Acelerador se corta a 0% automáticamente ante cualquier falta
- Sin dependencia de bus I2C para control del acelerador

### 🔧 Ajuste si tu pedal es ligeramente diferente:
Si al medir con multímetro el rango es distinto de 0.3V–4.8V, ajusta en `sensor_manager.c`:
```c
#define PEDAL_ADC_MIN   xxx    // medida real con pedal suelto
#define PEDAL_ADC_MAX   xxx    // medida real con pedal pisado a fondo
```

---

## 10. FICHEROS DEL FIRMWARE QUE IMPLEMENTAN ESTO

| Fichero | Qué hace |
|---------|----------|
| `Core/Inc/main.h` | Define PIN_PEDAL (PA3), extern hadc1 |
| `Core/Src/main.c` | Inicializa ADC1: 12-bit, PA3 canal 4, calibración single-ended |
| `Core/Src/stm32g4xx_hal_msp.c` | HAL_ADC_MspInit: habilita reloj ADC12, configura PA3 analógico |
| `Core/Inc/stm32g4xx_hal_conf.h` | HAL_ADC_MODULE_ENABLED (activa driver HAL ADC) |
| `Core/Src/sensor_manager.c` | Pedal_Update(): dual-sample ADC + plausibilidad software |
| `Core/Inc/sensor_manager.h` | API: Pedal_GetPercent(), Pedal_IsPlausible(), Pedal_GetRawPercent() |
| `Core/Src/safety_system.c` | Safety_CheckSensors(): fuerza throttle=0% si !Pedal_IsPlausible() |
| `docs/CONEXIONES_COMPLETAS.md` | Esquema de cableado completo, sección 6 |
