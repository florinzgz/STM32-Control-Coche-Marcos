# ESTADO FINAL DEL SENSOR DE PEDAL — Resumen Visual

> **Fecha:** 2026-02-17
> **Estado:** ✅ Confirmado y compilado — NO se modifica más
> **Sensor:** SS1324LUA-T (Hall magnético, 5V, salida 0.3V–4.8V)
> **Arquitectura:** Doble canal redundante (como en automoción real)

---

## 1. ESQUEMA DE CONEXIÓN COMPLETO

```
                              ┌─────────────────────────┐
                              │   PEDAL ACELERADOR      │
                              │   (SS1324LUA-T, 5V)     │
                              │                         │
                              │  Pin 1 (VCC) ──► 5V     │
                              │  Pin 2 (GND) ──► GND    │
                              │  Pin 3 (Señal) ─┬──────►│ Salida: 0.3V – 4.8V
                              └─────────────────┼───────┘
                                                │
                 ┌──────────────────────────────┤
                 │                              │
     ╔═══════════════════════╗       ╔═══════════════════════╗
     ║   CANAL PRIMARIO      ║       ║  CANAL PLAUSIBILIDAD  ║
     ║   (rápido, ~1 µs)     ║       ║  (preciso, ~8 ms)     ║
     ╚═══════════════════════╝       ╚═══════════════════════╝
                 │                              │
                 │                              │
     ┌───────────▼──────────┐        ┌──────────▼──────────┐
     │  DIVISOR DE TENSIÓN  │        │      ADS1115        │
     │                      │        │   (I2C, 16-bit)     │
     │  Señal (5V) ──┐     │        │                     │
     │               R1     │        │  VDD ──► 5V         │
     │           10 kΩ (1%) │        │  GND ──► GND        │
     │               │      │        │  A0  ◄── Señal 5V   │
     │          NODO ◄──────┼────►   │  ADDR ──► GND       │
     │               │      │  PA3   │  SCL ──► PB6        │
     │               R2     │        │  SDA ──► PB7        │
     │           6.8 kΩ (1%)│        │  (I2C addr: 0x48)   │
     │               │      │        └─────────────────────┘
     │              GND     │
     └──────────────────────┘
                 │
                 ▼
     ┌───────────────────────┐
     │    STM32G474RE        │
     │                       │
     │    PA3 (ADC1_IN4)     │ ◄── Señal dividida: 0.12V – 1.94V
     │    PB6 (I2C1_SCL)    │ ◄──► Bus I2C (pull-up 4.7kΩ a 3.3V)
     │    PB7 (I2C1_SDA)    │ ◄──► Bus I2C (pull-up 4.7kΩ a 3.3V)
     └───────────────────────┘
```

---

## 2. CABLES — Lista exacta

### Canal primario (divisor → PA3)

| Nº | De | A | Función |
|----|-----|---|---------|
| 1 | Pedal Pin 1 (VCC) | **5V** fuente | Alimentación sensor |
| 2 | Pedal Pin 2 (GND) | **GND** fuente | Masa sensor |
| 3 | Pedal Pin 3 (Señal) | **R1 entrada** (10 kΩ) | Señal 0.3V–4.8V |
| 4 | R1 salida / R2 entrada (nodo) | **PA3** (STM32) | Señal dividida ~0.12V–1.94V |
| 5 | R2 salida | **GND** | Cierre divisor |

### Canal plausibilidad (ADS1115 → I2C)

| Nº | De | A | Función |
|----|-----|---|---------|
| 6 | Pedal Pin 3 (Señal) | **ADS1115 A0** | Señal 5V sin dividir |
| 7 | ADS1115 VDD | **5V** fuente | Alimentación módulo |
| 8 | ADS1115 GND | **GND** | Masa módulo |
| 9 | ADS1115 ADDR | **GND** | Fija dirección I2C = 0x48 |
| 10 | ADS1115 SCL | **PB6** (STM32) | Bus I2C reloj |
| 11 | ADS1115 SDA | **PB7** (STM32) | Bus I2C datos |

> **Total: 11 cables** (5 para divisor + 6 para ADS1115)

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
   ├── Paso 1: Lee canal primario (ADC interno, ~1 µs)
   │   HAL_ADC_Start → PollForConversion → GetValue → Stop
   │   pedal_raw_adc = valor 12-bit (150–2413)
   │   pedal_pct = mapeo lineal a 0–100%
   │
   ├── Paso 2: Lee canal plausibilidad (ADS1115, ~8 ms)
   │   I2C Transmit config → HAL_Delay(8) → I2C Read resultado
   │   pedal_raw_ads = valor 16-bit (1600–25600)
   │   pedal_pct_ads = mapeo lineal a 0–100%
   │
   └── Paso 3: Validación cruzada
       │
       ├── Si AMBOS canales coinciden (diferencia < 5%):
       │   ✅ pedal_plausible = true
       │   El acelerador funciona normal con pedal_pct
       │
       ├── Si los canales DIVERGEN > 5% durante > 200 ms:
       │   ❌ pedal_plausible = false
       │   Safety_CheckSensors() detecta esto →
       │     → Fuerza acelerador a 0%
       │     → Sistema entra en DEGRADED
       │
       └── Si ADS1115 I2C FALLA > 500 ms:
           ⚠️ pedal_plausible = false (modo degradado)
           El canal primario (ADC) sigue funcionando solo
           El conductor puede parar con seguridad
```

### En Safety_CheckSensors() (cada 50 ms también):

```c
if (!Pedal_IsPlausible()) {
    Traction_SetDemand(0.0f);        // Fuerza acelerador a 0%
    Safety_SetError(SAFETY_ERROR_PEDAL);
    Safety_SetState(SYS_STATE_DEGRADED);
}
```

---

## 5. CALIBRACIÓN EN EL FIRMWARE

### Fichero: `Core/Src/sensor_manager.c`

| Constante | Valor | Significado |
|-----------|-------|-------------|
| `PEDAL_ADC_MIN` | 150 | ADC 12-bit para 0.3V (pedal suelto), con divisor |
| `PEDAL_ADC_MAX` | 2413 | ADC 12-bit para 4.8V (pedal pisado), con divisor |
| `PEDAL_ADS_MIN` | 1600 | ADS1115 16-bit para 0.3V (pedal suelto) |
| `PEDAL_ADS_MAX` | 25600 | ADS1115 16-bit para 4.8V (pedal pisado) |
| `PEDAL_PLAUSIBILITY_PCT` | 5.0% | Tolerancia máxima entre canales |
| `PEDAL_DIVERGE_TIMEOUT_MS` | 200 ms | Tiempo antes de falta por divergencia |
| `PEDAL_ADS_STALE_TIMEOUT_MS` | 500 ms | Timeout si I2C ADS1115 no responde |

---

## 6. PINES GPIO USADOS

| Pin STM32 | Función | Periférico | Señal |
|-----------|---------|------------|-------|
| **PA3** | ADC1_IN4 | ADC1 | Pedal canal primario (0.12V–1.94V vía divisor) |
| **PB6** | I2C1_SCL | I2C1 | Bus compartido: ADS1115 + TCA9548A + INA226 |
| **PB7** | I2C1_SDA | I2C1 | Bus compartido: ADS1115 + TCA9548A + INA226 |

---

## 7. MATERIAL NECESARIO (BOM pedal)

| Cantidad | Componente | Especificación | Precio aprox. |
|----------|-----------|----------------|---------------|
| 1 | SS1324LUA-T | Sensor Hall, alimentar a 5V | (ya lo tienes) |
| 1 | ADS1115 módulo | 16-bit ADC I2C, con pines | ~2–4 € |
| 1 | Resistencia 10 kΩ | 1% precisión, ¼W | ~0.05 € |
| 1 | Resistencia 6.8 kΩ | 1% precisión, ¼W | ~0.05 € |
| 2 | Pull-up 4.7 kΩ | Para I2C (si no están en el módulo ADS1115) | ~0.10 € |

---

## 8. NOTAS IMPORTANTES

### ⚠️ Lo que NO debes hacer:
- **NO conectar** la señal de 5V del pedal directamente a PA3 → quemaría el STM32 (máx 3.6V)
- **NO alimentar** el SS1324LUA-T a 3.3V → mínimo es 4.5V según datasheet Allegro
- **NO usar** resistencias de 5% → pueden dar error de calibración de ±3%

### ✅ Lo que SÍ está implementado:
- Lectura primaria instantánea (~1 µs) para control en tiempo real
- Lectura de verificación precisa (~8 ms) para seguridad
- Protección contra cable roto, sensor congelado, I2C colgado
- Acelerador se corta a 0% automáticamente ante cualquier falta
- El bus I2C es compartido con los sensores INA226 sin conflicto (dirección 0x48 ≠ 0x70/0x40)

### 🔧 Ajuste si tu pedal es ligeramente diferente:
Si al medir con multímetro el rango es distinto de 0.3V–4.8V, ajusta en `sensor_manager.c`:
```c
#define PEDAL_ADC_MIN   xxx    // medida real con pedal suelto
#define PEDAL_ADC_MAX   xxx    // medida real con pedal pisado a fondo
#define PEDAL_ADS_MIN   xxx    // ADS1115 con pedal suelto
#define PEDAL_ADS_MAX   xxx    // ADS1115 con pedal pisado a fondo
```

---

## 9. FICHEROS DEL FIRMWARE QUE IMPLEMENTAN ESTO

| Fichero | Qué hace |
|---------|----------|
| `Core/Inc/main.h` | Define PIN_PEDAL (PA3), I2C_ADDR_ADS1115 (0x48), extern hadc1 |
| `Core/Src/main.c` | Inicializa ADC1: 12-bit, PA3 canal 4, calibración single-ended |
| `Core/Src/stm32g4xx_hal_msp.c` | HAL_ADC_MspInit: habilita reloj ADC12, configura PA3 analógico |
| `Core/Inc/stm32g4xx_hal_conf.h` | HAL_ADC_MODULE_ENABLED (activa driver HAL ADC) |
| `Core/Src/sensor_manager.c` | Pedal_Update(): doble lectura + validación cruzada |
| `Core/Inc/sensor_manager.h` | API: Pedal_GetPercent(), Pedal_IsPlausible(), Pedal_GetADSPercent() |
| `Core/Src/safety_system.c` | Safety_CheckSensors(): fuerza throttle=0% si !Pedal_IsPlausible() |
| `docs/CONEXIONES_COMPLETAS.md` | Esquema de cableado completo, sección 6 |
