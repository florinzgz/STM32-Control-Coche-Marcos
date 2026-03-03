# Plan de Integración de Componentes — STM32-Control-Coche-Marcos

> **Revisión:** 1.0  
> **Fecha:** 2026-03-03  
> **MCU Principal:** STM32G474RE (Cortex-M4F, 170 MHz, LQFP64)  
> **MCU HMI:** ESP32-S3 (16 MB Flash, 8 MB PSRAM)  
> **Comunicación:** FDCAN1 @ 500 kbps (CAN 2.0A clásico)

---

## Índice

1. [Punto de partida exacto](#1-punto-de-partida-exacto)
2. [Orden recomendado de integración](#2-orden-recomendado-de-integración)
3. [Verificación de pines ocupados y conflictos](#3-verificación-de-pines-ocupados-y-conflictos)
4. [Validación de consumo eléctrico y estabilidad](#4-validación-de-consumo-eléctrico-y-estabilidad)
5. [Pasos concretos por componente](#5-pasos-concretos-por-componente)
6. [Checklist de seguridad anti-bloqueos](#6-checklist-de-seguridad-anti-bloqueos)
7. [Validación de interrupciones, temporizadores y buses](#7-validación-de-interrupciones-temporizadores-y-buses)
8. [Estrategia de pruebas progresivas](#8-estrategia-de-pruebas-progresivas)
9. [Documentación y rollback](#9-documentación-y-rollback)
10. [Criterios de estabilidad por etapa](#10-criterios-de-estabilidad-por-etapa)

---

## 1. Punto de partida exacto

### 1.1 Estado actual del firmware STM32

El firmware actual ejecuta 18 módulos C compilados con ARM GCC (`arm-none-eabi-gcc`, optimización `-O2`, LTO). El bucle principal opera a **100 Hz (10 ms)** con capas de temporización a 20 Hz, 10 Hz y 1 Hz.

**Periféricos activos:**

| Periférico | Función | Pines | Config |
|------------|---------|-------|--------|
| **TIM1** | PWM motores FL/FR | PA8, PA9, PA10, PA11 | 20 kHz, center-aligned, ARR=4249 |
| **TIM8** | PWM motores RL/RR | PC6, PC7, PC8, PC9 | 20 kHz, center-aligned, ARR=4249 |
| **TIM3** | PWM dirección | PA6, PA7 | 20 kHz, center-aligned |
| **TIM2** | Encoder cuadratura | PA15, PB3 | 32-bit, 4800 CPR |
| **ADC1** | Pedal acelerador | PA3 | 12-bit, IN4, muestreo ~1.1 µs |
| **FDCAN1** | Comunicación ESP32 | PB8 (RX), PB9 (TX) | 500 kbps, prescaler 17, AF9 |
| **I2C1** | Sensores corriente/ADC | PB6 (SCL), PB7 (SDA) | 400 kHz fast mode, open-drain |
| **IWDG** | Watchdog independiente | — | ~500 ms (prescaler 32, reload 4095) |
| **OneWire** | DS18B20 temperaturas | PB0 | Bit-bang GPIO |
| **EXTI** | Ruedas + encoder | PA0, PA1, PA2, PB15, PB4, PB5 | Rising edge, prioridad 2 |
| **SWD** | Depuración | PA13, PA14 | Debug (no modificar) |

**Dispositivos I2C en bus:**

| Dirección | Dispositivo | Canal TCA9548A | Función |
|-----------|-------------|---------------|---------|
| 0x70 | TCA9548A | — | Multiplexor I2C 8 canales |
| 0x40 | INA226 ×6 | Canales 0–5 | Sensores corriente (1 mΩ/0.5 mΩ shunt) |
| 0x48 | ADS1115 | Directo | ADC 16-bit pedal plausibilidad |

**Mensajes CAN activos (27 tipos):**

| Rango IDs | Dirección | Función |
|-----------|-----------|---------|
| 0x001 | STM32→ESP32 | Heartbeat STM32 (100 ms) |
| 0x011 | ESP32→STM32 | Heartbeat ESP32 (100 ms) |
| 0x100–0x102 | ESP32→STM32 | Comandos (throttle, steering, mode) |
| 0x110 | ESP32→STM32 | Servicio |
| 0x120 | ESP32→STM32 | Control LED |
| 0x200–0x20A | STM32→ESP32 | Telemetría (speed, current, temp, safety...) |
| 0x208–0x209 | ESP32→STM32 | Obstáculos |
| 0x300–0x305 | STM32→ESP32 | Diagnóstico y error log |

### 1.2 Qué revisar ANTES de añadir cualquier componente

**Hardware — verificar físicamente:**

1. Medir voltaje 3.3 V en pin VDD del STM32 bajo carga normal (debe ser 3.3 V ±5%)
2. Medir voltaje VDDA (referencia ADC) — debe ser estable y filtrado
3. Verificar bus I2C con osciloscopio: señales SCL/SDA limpias a 400 kHz, sin ringing
4. Verificar bus CAN: niveles CANH/CANL correctos (diferencial ~2V), terminación 120 Ω en ambos extremos
5. Medir consumo total actual en reposo y bajo carga máxima
6. Confirmar que el regulador 3.3 V tiene margen para componentes adicionales
7. Confirmar que las pull-ups I2C (4.7 kΩ típico) son adecuadas para la capacitancia actual del bus

**Firmware — verificar con depurador:**

1. Confirmar que `IWDG` se refresca correctamente (no hay resets espurios)
2. Verificar que el bucle principal cumple el tick de 10 ms sin overflow
3. Confirmar estado de transacciones I2C: sin errores `HAL_I2C_ERROR_*` recurrentes
4. Verificar contadores CAN TX/RX en `can_stats` (sin pérdidas)
5. Confirmar que `BootValidation_Run()` pasa todos los checks
6. Verificar que Flash pages 125–127 no están corruptas (CRC válido)
7. Confirmar compilación limpia sin warnings con `-Wall -Wextra`

### 1.3 Diferencias clave respecto a diseño base

El proyecto actual tiene las siguientes particularidades respecto a un proyecto CubeMX base:

- **Pines DIR (PC0–PC4) liberados**: Originalmente para señales de dirección de motor, ahora libres al usar esquema RPWM/LPWM dual-PWM con BTS7960
- **BREAK2 armado a LOCKUP**: TIM1/TIM8 tienen BREAK2 conectado internamente a Cortex-M4 LOCKUP — cualquier hard fault corta PWM por hardware
- **Flash pages 125–127 reservadas**: Error log (125), calibración dirección (126), parámetros EPS (127) — NO usar para otros fines
- **Watchdog agresivo**: 500 ms timeout implica que cualquier operación bloqueante >500 ms causa reset
- **Filtro FDCAN estricto**: Solo acepta IDs conocidos — componentes nuevos con CAN requieren actualizar el filtro

---

## 2. Orden recomendado de integración

### Prioridad 1 — Críticos para seguridad (integrar primero)

| # | Componente | Bus | Justificación |
|---|-----------|-----|---------------|
| 1a | Sensor de corriente adicional (INA226) | I2C1 via TCA9548A | Monitorización de nuevos consumidores de potencia — ya hay infraestructura, canal TCA libre (6, 7) |
| 1b | Sensor de temperatura adicional (DS18B20) | OneWire (PB0) | Monitorización térmica de nuevos componentes — solo añadir ROM al bus existente |

### Prioridad 2 — Comunicación y control

| # | Componente | Bus/Pin | Justificación |
|---|-----------|---------|---------------|
| 2a | UART externo (GPS, telemetría, Bluetooth) | PA4 (TX), PA5 (RX) → USART2 o LPUART1 | Pines libres, periférico disponible |
| 2b | SPI externo (IMU, sensor barométrico, SD card) | PB12 (NSS), PB13 (SCK), PB14 (MISO), PB2 (MOSI) | SPI2 disponible, pines libres |
| 2c | Sensor analógico adicional (sensor de presión, nivel) | PB1 → ADC1_IN12 | Canal ADC libre, sin conflicto |

### Prioridad 3 — Auxiliares y expansión

| # | Componente | Bus/Pin | Justificación |
|---|-----------|---------|---------------|
| 3a | GPIO de señalización (buzzer, LED estado) | PC0–PC4 (freed DIR pins) | GPIOs liberados, sin conflicto |
| 3b | Segundo bus I2C (I2C3 o I2C4) | Requiere reasignación de pines | Solo si I2C1 se satura |
| 3c | PWM auxiliar (ventilador, servo) | PB1 (TIM3_CH4) o PC0–PC4 con timer mapeado | Revisar AF mux del STM32G474 |

### Regla de oro

> **No avanzar al grupo N+1 hasta que el grupo N esté completamente validado y estable durante mínimo 1 hora de funcionamiento continuo.**

---

## 3. Verificación de pines ocupados y conflictos

### 3.1 Mapa completo de pines GPIO — STM32G474RE LQFP64

#### Puerto A

| Pin | Estado | Función actual | AF | Alternativas disponibles |
|-----|--------|----------------|-----|--------------------------|
| PA0 | **OCUPADO** | EXTI0 — Wheel FL | — | — |
| PA1 | **OCUPADO** | EXTI1 — Wheel FR | — | — |
| PA2 | **OCUPADO** | EXTI2 — Wheel RL | — | — |
| PA3 | **OCUPADO** | ADC1_IN4 — Pedal | — | — |
| PA4 | **LIBRE** | — | AF8: USART2_TX, AF12: LPUART1_TX | UART TX candidato |
| PA5 | **LIBRE** | — | AF8: USART2_RX, AF5: SPI1_SCK | UART RX candidato / SPI1 SCK |
| PA6 | **OCUPADO** | TIM3_CH1 — RPWM Steer | AF2 | — |
| PA7 | **OCUPADO** | TIM3_CH2 — LPWM Steer | AF2 | — |
| PA8 | **OCUPADO** | TIM1_CH1 — RPWM FL | AF6 | — |
| PA9 | **OCUPADO** | TIM1_CH2 — LPWM FL | AF6 | — |
| PA10 | **OCUPADO** | TIM1_CH3 — RPWM FR | AF6 | — |
| PA11 | **OCUPADO** | TIM1_CH4 — LPWM FR | AF6 | — |
| PA12 | **OCUPADO** | — | — | Puede estar libre en IOC, verificar |
| PA13 | **RESERVADO** | SWDIO — Debug | — | NO tocar |
| PA14 | **RESERVADO** | SWCLK — Debug | — | NO tocar |
| PA15 | **OCUPADO** | TIM2_CH1 — Encoder A | AF1 | — |

#### Puerto B

| Pin | Estado | Función actual | AF | Alternativas disponibles |
|-----|--------|----------------|-----|--------------------------|
| PB0 | **OCUPADO** | OneWire — DS18B20 | GPIO | — |
| PB1 | **LIBRE** | — | ADC1_IN12, TIM3_CH4 | ADC o PWM auxiliar |
| PB2 | **LIBRE** | — | AF5: SPI3_MOSI | SPI MOSI candidato |
| PB3 | **OCUPADO** | TIM2_CH2 — Encoder B | AF1 | — |
| PB4 | **OCUPADO** | EXTI4 — Encoder Z | — | — |
| PB5 | **OCUPADO** | EXTI5 — Steering center | — | — |
| PB6 | **OCUPADO** | I2C1_SCL | AF4 | — |
| PB7 | **OCUPADO** | I2C1_SDA | AF4 | — |
| PB8 | **OCUPADO** | FDCAN1_RX | AF9 | — |
| PB9 | **OCUPADO** | FDCAN1_TX | AF9 | — |
| PB10 | **OCUPADO** | Relay LED Front | GPIO | — |
| PB11 | **OCUPADO** | Relay LED Rear | GPIO | — |
| PB12 | **LIBRE** | — | AF5: SPI2_NSS, AF6: I2C2_SMBA | SPI2 NSS candidato |
| PB13 | **LIBRE** | — | AF5: SPI2_SCK | SPI2 SCK candidato |
| PB14 | **LIBRE** | — | AF5: SPI2_MISO | SPI2 MISO candidato |
| PB15 | **OCUPADO** | EXTI15 — Wheel RR | — | — |

#### Puerto C

| Pin | Estado | Función actual | AF | Alternativas disponibles |
|-----|--------|----------------|-----|--------------------------|
| PC0 | **LIBRE** | (ex-DIR_FL) | ADC1_IN6, GPIO | GPIO salida / ADC auxiliar |
| PC1 | **LIBRE** | (ex-DIR_FR) | ADC1_IN7, GPIO | GPIO salida / ADC auxiliar |
| PC2 | **LIBRE** | (ex-DIR_RL) | ADC1_IN8, GPIO | GPIO salida / ADC auxiliar |
| PC3 | **LIBRE** | (ex-DIR_RR) | ADC1_IN9, GPIO | GPIO salida / ADC auxiliar |
| PC4 | **LIBRE** | (ex-DIR_STEER) | ADC1_IN5, USART1_TX | GPIO salida / UART alternativo |
| PC5 | **OCUPADO** | EN_FL | GPIO output | — |
| PC6 | **OCUPADO** | TIM8_CH1 — RPWM RL | AF4 | — |
| PC7 | **OCUPADO** | TIM8_CH2 — LPWM RL | AF4 | — |
| PC8 | **OCUPADO** | TIM8_CH3 — RPWM RR | AF4 | — |
| PC9 | **OCUPADO** | TIM8_CH4 — LPWM RR | AF4 | — |
| PC10 | **OCUPADO** | Relay Main | GPIO output | — |
| PC11 | **OCUPADO** | Relay Trac | GPIO output | — |
| PC12 | **OCUPADO** | Relay Dir | GPIO output | — |
| PC13 | **OCUPADO** | EN_RR | GPIO output | — |

#### Otros

| Pin | Estado | Función |
|-----|--------|---------|
| PD2 | **LIBRE** | GPIO disponible (verificar en LQFP64) |
| PH0 | **OCUPADO** | HSE OSC_IN |
| PH1 | **OCUPADO** | HSE OSC_OUT |

### 3.2 Resumen de pines libres para expansión

| Pin | Mejor uso recomendado | Notas |
|-----|----------------------|-------|
| **PA4** | USART2_TX o LPUART1_TX | Ideal para UART (GPS, BT, telemetría) |
| **PA5** | USART2_RX | Par con PA4 para UART completo |
| **PB1** | ADC1_IN12 o TIM3_CH4 PWM | Sensor analógico o PWM ventilador |
| **PB2** | SPI MOSI (si se usa SPI3) | Alternativa SPI |
| **PB12** | SPI2_NSS | Chip select SPI |
| **PB13** | SPI2_SCK | Reloj SPI |
| **PB14** | SPI2_MISO | Datos SPI entrada |
| **PC0** | GPIO salida (buzzer, LED) | Pin DIR liberado |
| **PC1** | GPIO salida / ADC1_IN7 | Pin DIR liberado |
| **PC2** | GPIO salida / ADC1_IN8 | Pin DIR liberado |
| **PC3** | GPIO salida / ADC1_IN9 | Pin DIR liberado |
| **PC4** | GPIO salida / USART1_TX alt | Pin DIR liberado |

### 3.3 Protocolo anti-conflictos

Antes de asignar cualquier pin libre:

1. **Verificar en CubeMX** (.ioc) que el pin no tiene asignación oculta
2. **Verificar la AF (Alternate Function)** en el datasheet STM32G474RE para confirmar compatibilidad
3. **Verificar conflictos de EXTI**: EXTI0–15 comparten línea por número de pin (ej: PA0/PB0/PC0 comparten EXTI0) — PA0 ya usa EXTI0, por tanto PB0 y PC0 **no pueden** usar EXTI en ese número
4. **Verificar en `hal_msp.c`** que el `HAL_xxx_MspInit()` no configura pines colaterales
5. **Actualizar `main.h`** con define del nuevo pin
6. **Actualizar esta tabla** después de cada asignación

**Conflictos EXTI conocidos (líneas ya ocupadas):**

| Línea EXTI | Pin ocupado | Pines bloqueados para EXTI |
|------------|-------------|---------------------------|
| EXTI0 | PA0 (Wheel FL) | PB0, PC0 |
| EXTI1 | PA1 (Wheel FR) | PB1, PC1 |
| EXTI2 | PA2 (Wheel RL) | PB2, PC2 |
| EXTI4 | PB4 (Encoder Z) | PA4, PC4 |
| EXTI5 | PB5 (Steer center) | PA5, PC5 |
| EXTI15 | PB15 (Wheel RR) | PA15, PC15 |

> **IMPORTANTE:** Los pines libres PA4, PA5, PB1, PB2, PC0–PC4 NO pueden usarse como EXTI si su línea ya está ocupada. Usar solo como GPIO salida, ADC, UART o timer.

---

## 4. Validación de consumo eléctrico y estabilidad

### 4.1 Presupuesto de corriente actual (estimado)

| Componente | Consumo típico | Consumo pico | Fuente |
|-----------|---------------|-------------|--------|
| STM32G474RE | 30 mA | 50 mA | 3.3 V regulado |
| FDCAN transceiver (TJA1050/1051) | 10 mA | 75 mA | 5 V |
| TCA9548A | 0.01 mA | 0.1 mA | 3.3 V |
| INA226 ×6 | 2 mA | 3 mA | 3.3 V |
| ADS1115 | 0.15 mA | 0.2 mA | 3.3 V |
| DS18B20 ×5 | 5 mA | 7.5 mA | 3.3 V |
| Pull-ups I2C (4.7 kΩ ×2) | 1.4 mA | 1.4 mA | 3.3 V |
| BTS7960 ×5 (lógica) | 25 mA | 50 mA | 5 V |
| Relés ×5 (bobina) | 250 mA | 500 mA | 12/24 V |
| **TOTAL 3.3 V** | **~40 mA** | **~62 mA** | — |
| **TOTAL 5 V** | **~35 mA** | **~125 mA** | — |

### 4.2 Procedimiento antes de conectar cada módulo nuevo

1. **Medir corriente 3.3 V actual** con amperímetro en serie antes de conectar nada
2. **Calcular margen**: `margen = capacidad_regulador - consumo_actual - consumo_nuevo_componente`
3. **Regla**: Mantener ≥30% de margen en el regulador 3.3 V
4. **Si el margen es insuficiente**: Añadir regulador dedicado para el nuevo componente
5. **Desacoplo obligatorio**: Condensador 100 nF cerámico lo más cerca posible de VDD de cada IC nuevo
6. **Para componentes I2C**: Verificar que la capacitancia total del bus no excede 400 pF (límite I2C fast mode)
7. **Para componentes SPI**: Línea de reloj ≤15 cm sin buffer; >15 cm requiere buffer de línea

### 4.3 Señales de problemas de alimentación

| Síntoma | Causa probable | Acción |
|---------|---------------|--------|
| Reset aleatorio STM32 | Brown-out por caída de VDD | Añadir condensador bulk 47 µF + verificar regulador |
| Lecturas ADC ruidosas | VDDA inestable | Filtro LC en VDDA (ferrita + 10 µF) |
| I2C NACK aleatorio | Glitch en SCL/SDA por ruido | Verificar ruteo, añadir filtro RC si necesario |
| CAN bus-off espontáneo | Interferencia EMI en CANH/CANL | Verificar terminación, blindaje, tierra |
| DS18B20 desaparece | Caída OneWire pull-up | Pull-up 2.2 kΩ a 3.3 V (no 4.7 kΩ) |

---

## 5. Pasos concretos por componente

### 5.1 Sensor INA226 adicional (I2C, Prioridad 1a)

#### Conexión física
- Conectar al bus I2C1 existente (PB6/PB7) **a través del TCA9548A** en canal 6 o 7 (actualmente canales 0–5 ocupados)
- Configurar dirección I2C del nuevo INA226 a **0x40** (misma que los demás — el TCA9548A aísla)
- Shunt resistor según corriente: 1 mΩ (motores, ≤25 A) o 0.5 mΩ (batería, ≤100 A)
- Condensador desacoplo 100 nF entre VDD-GND del INA226

#### Configuración en firmware
```c
/* sensor_manager.c — añadir canal al array existente */
// En Current_ReadAll(): extender bucle a SENSOR_COUNT+1
// Añadir define: #define INA226_CHANNEL_NEW  6  /* canal TCA9548A */
```

**Archivos a modificar:**
- `Core/Inc/sensor_manager.h` — Incrementar `CURRENT_SENSOR_COUNT` (de 6 a 7)
- `Core/Src/sensor_manager.c` — Añadir canal 6 al bucle `Current_ReadAll()`
- `Core/Src/can_handler.c` — Incluir nuevo sensor en telemetría CAN (0x201)
- `Core/Src/safety_system.c` — Añadir check de overcurrent para el nuevo canal

#### Inicialización segura
1. Seleccionar canal TCA9548A escribiendo byte a 0x70
2. Verificar ACK del INA226 con `HAL_I2C_IsDeviceReady()` (3 intentos, 50 ms timeout)
3. Configurar registro CONFIG (0x00): averaging, conversion time
4. Leer Manufacturer ID (0xFE) = 0x5449 para validación
5. Si falla: marcar canal como inactivo, continuar sin bloquear

#### Pruebas unitarias aisladas
- [ ] I2C scan: confirmar presencia en canal TCA correcto
- [ ] Lectura de Manufacturer ID = 0x5449
- [ ] Lectura de corriente con carga conocida: error ≤5%
- [ ] Lectura de tensión de bus: error ≤1%
- [ ] Recovery: desconectar sensor, verificar que el sistema no se bloquea

#### Pruebas integradas
- [ ] `Current_ReadAll()` completa en <5 ms para 7 sensores
- [ ] Telemetría CAN transmite los 7 valores correctamente
- [ ] `Safety_CheckCurrent()` detecta overcurrent en el nuevo canal
- [ ] Boot validation pasa con el nuevo sensor
- [ ] 1 hora de funcionamiento continuo sin errores I2C

---

### 5.2 Sensor DS18B20 adicional (OneWire, Prioridad 1b)

#### Conexión física
- Conectar al bus OneWire existente en **PB0**
- Alimentación parásita NO recomendada: usar alimentación externa 3.3 V
- Pull-up 2.2 kΩ a 3.3 V (revisar que el pull-up existente soporta +1 sensor)
- Máximo recomendado en un bus: 10 sensores (actualmente 5)

#### Configuración en firmware
```c
/* sensor_manager.c — el rescan periódico detecta automáticamente nuevos DS18B20 */
// Temperature_PeriodicRescan() ejecuta Search ROM cada 10s
// Incrementar DS18B20_MAX_SENSORS si es necesario
```

**Archivos a modificar:**
- `Core/Inc/sensor_manager.h` — Verificar `DS18B20_MAX_SENSORS` (actualmente 5, incrementar si <10)
- `Core/Src/sensor_manager.c` — Verificar que arrays tienen espacio para +1 sensor
- `Core/Src/can_handler.c` — Extender telemetría de temperatura (0x202) si payload lo permite

#### Inicialización segura
1. El firmware ya ejecuta `Search ROM` con validación CRC-8/MAXIM
2. Nuevo sensor se detecta automáticamente en el siguiente ciclo de rescan (10 s)
3. Si CRC falla: sensor ignorado, resto del bus sigue funcionando
4. Verificar que el tiempo de conversión total (750 ms × sensores) cabe en el ciclo de 1 Hz

#### Pruebas unitarias aisladas
- [ ] Sensor aparece en Search ROM con CRC válido
- [ ] Lectura de temperatura: valor coherente con ambiente (15–35 °C)
- [ ] Desconexión del sensor: los demás siguen leyendo correctamente
- [ ] Hot-plug: reconexión detectada en ≤10 s

#### Pruebas integradas
- [ ] `Temperature_ReadAll()` lee todos los sensores sin timeout
- [ ] Telemetría CAN transmite temperaturas correctas
- [ ] `Safety_CheckTemperature()` detecta overtemp en el nuevo sensor
- [ ] Sin impacto en timing del bucle principal (verificar con GPIO toggle + osciloscopio)

---

### 5.3 UART externo (Prioridad 2a)

#### Conexión física
- **Pines recomendados:** PA4 (TX) + PA5 (RX)
- **Periférico:** USART2 o LPUART1 (verificar AF en datasheet)
- Nivel lógico: 3.3 V — si el dispositivo externo usa 5 V, usar level shifter bidireccional
- Conectar GND común entre STM32 y dispositivo externo

#### Configuración en firmware (CubeMX + código)

**En `.ioc` (CubeMX):**
1. Activar USART2 (o LPUART1) en modo Asynchronous
2. Asignar PA4 → USART2_TX, PA5 → USART2_RX
3. Configurar: baudrate (9600/115200), 8N1, no flow control
4. Habilitar interrupción USART2 en NVIC (prioridad 3–4, menor que CAN y timers)

**Archivos a crear:**
- `Core/Inc/uart_handler.h` — API: `UART_Init()`, `UART_Send()`, `UART_ProcessRX()`
- `Core/Src/uart_handler.c` — Implementación con buffer circular RX (≥256 bytes)

**Archivos a modificar:**
- `Core/Src/main.c` — Añadir `MX_USART2_UART_Init()` y llamada en `main()`
- `Core/Src/stm32g4xx_hal_msp.c` — Añadir `HAL_UART_MspInit()` para USART2
- `Core/Src/stm32g4xx_it.c` — Añadir handler `USART2_IRQHandler()`
- `Makefile` — Añadir `Core/Src/uart_handler.c` a C_SOURCES y HAL UART driver

#### Inicialización segura
1. Inicializar UART **después** de CAN y I2C (menor prioridad)
2. Enviar string de prueba y verificar eco si el dispositivo soporta loopback
3. Timeout de recepción: usar interrupción IDLE line (no polling bloqueante)
4. Nunca llamar `HAL_UART_Receive()` en modo bloqueante — usar IT o DMA

#### Pruebas unitarias aisladas
- [ ] TX: enviar "HELLO\r\n" y verificar con terminal/osciloscopio
- [ ] RX: enviar datos desde PC/terminal y verificar buffer circular
- [ ] Baudrate correcto: verificar con osciloscopio (bit time)
- [ ] Overflow: enviar más datos que el buffer y verificar que no hay crash
- [ ] Desconexión: retirar cable RX, verificar que no hay bloqueo

#### Pruebas integradas
- [ ] UART funciona simultáneamente con CAN sin pérdida de mensajes
- [ ] UART funciona simultáneamente con I2C sin errores
- [ ] Interrupciones UART no interfieren con timing de 10 ms
- [ ] Watchdog no se dispara durante ráfagas UART

---

### 5.4 Bus SPI (Prioridad 2b)

#### Conexión física
- **Pines recomendados:** PB13 (SCK), PB14 (MISO), PB12 (NSS/CS)
- **MOSI:** PB2 (AF5: SPI3_MOSI) — **ATENCIÓN**: verificar AF exacta para SPI2 vs SPI3
- **Periférico:** SPI2 (si todos los pines soportan SPI2 AF) — consultar tabla AF del datasheet
- CS manejado por software (GPIO manual) para máxima flexibilidad
- Longitud de cable SCK ≤15 cm sin buffer de línea

#### Configuración en firmware

**En `.ioc` (CubeMX):**
1. Activar SPI2 en modo Full-Duplex Master
2. Asignar pines SPI2: PB13 (SCK), PB14 (MISO), PB2 (MOSI)
3. PB12 como GPIO_Output (CS manual)
4. Prescaler: empezar con baja velocidad (Fpclk/256) y subir gradualmente
5. CPOL/CPHA según datasheet del dispositivo esclavo

**Archivos a crear:**
- `Core/Inc/spi_handler.h` — API: `SPI_Init()`, `SPI_Transfer()`, `SPI_ReadReg()`, `SPI_WriteReg()`
- `Core/Src/spi_handler.c` — Implementación con CS manual y timeout

**Archivos a modificar:**
- `Core/Src/main.c` — Añadir `MX_SPI2_Init()` y llamada
- `Core/Src/stm32g4xx_hal_msp.c` — Añadir `HAL_SPI_MspInit()` para SPI2
- `Makefile` — Añadir `Core/Src/spi_handler.c` y HAL SPI driver (`stm32g4xx_hal_spi.c`)

#### Inicialización segura
1. CS HIGH antes de inicializar SPI (desactivar esclavo)
2. Enviar byte dummy (0xFF) y verificar que MISO responde
3. Leer WHO_AM_I register del dispositivo (si existe) para validación
4. Timeout en cada transferencia: máximo 10 ms
5. Si falla: deshabilitar SPI, marcar como no disponible, continuar

#### Pruebas unitarias aisladas
- [ ] Señal SCK visible en osciloscopio con frecuencia correcta
- [ ] Lectura WHO_AM_I del dispositivo SPI
- [ ] Escritura/lectura de registro conocido
- [ ] CS toggle correcto (activo LOW, inactivo HIGH)
- [ ] Desconexión del esclavo: timeout sin bloqueo

#### Pruebas integradas
- [ ] SPI y I2C funcionan simultáneamente sin interferencia
- [ ] SPI no afecta timing de PWM (verificar jitter en osciloscopio)
- [ ] Watchdog no se dispara durante transferencias SPI
- [ ] CAN sigue operando durante actividad SPI intensa

---

### 5.5 Sensor analógico adicional (ADC, Prioridad 2c)

#### Conexión física
- **Pin recomendado:** PB1 → ADC1_IN12
- Divisor de tensión si señal >3.3 V (ej: 5 V → divisor 2:1 con R 10 kΩ + 10 kΩ)
- Filtro RC anti-aliasing: R=1 kΩ + C=100 nF (fc ≈ 1.6 kHz)
- Referencia: VDDA (debe estar filtrado y estable)

#### Configuración en firmware

**En `.ioc` (CubeMX):**
1. Añadir ADC1 Channel 12 (PB1)
2. Configurar como canal adicional en secuencia de conversión
3. Mismo sampling time que el pedal (~1.1 µs) o mayor si baja frecuencia

**Archivos a modificar:**
- `Core/Src/main.c` — Añadir canal IN12 a configuración ADC1 (`sConfig.Channel = ADC_CHANNEL_12`)
- `Core/Src/sensor_manager.c` — Añadir función de lectura del nuevo canal
- `Core/Inc/sensor_manager.h` — Exponer nueva API de lectura

#### Inicialización segura
1. Calibrar ADC1 si no se ha hecho (`HAL_ADCEx_Calibration_Start()`)
2. Leer valor en reposo sin carga: debe ser ~0 o valor esperado
3. Aplicar filtro EMA (α=0.1) para suavizar lecturas
4. Rango válido: configurar umbrales y marcar como fallo si fuera de rango

#### Pruebas unitarias aisladas
- [ ] Lectura con entrada conocida (ej: divisor de tensión desde 3.3 V → ~2048 cuentas)
- [ ] Linealidad: medir 3 puntos (0 V, 1.65 V, 3.3 V)
- [ ] Ruido: calcular desviación estándar de 100 lecturas (debe ser <10 LSB)
- [ ] Sin interferencia con pedal ADC: leer ambos canales secuencialmente

#### Pruebas integradas
- [ ] Lectura ADC no introduce latencia en bucle de 10 ms
- [ ] Valor coherente durante operación de motores (EMI test)
- [ ] Telemetría CAN transmite el valor si se añade al protocolo

---

### 5.6 GPIO de señalización (Prioridad 3a)

#### Conexión física
- **Pines recomendados:** PC0 (buzzer), PC1 (LED estado), PC2–PC4 (auxiliares)
- Buzzer activo: usar MOSFET N-channel (2N7002) si buzzer >20 mA
- LED: resistencia serie (330 Ω para LED estándar a 3.3 V)
- NO conectar cargas >20 mA directamente al GPIO del STM32

#### Configuración en firmware
```c
/* main.h — añadir defines */
#define PIN_BUZZER_Port    GPIOC
#define PIN_BUZZER_Pin     GPIO_PIN_0
#define PIN_LED_STATUS_Port GPIOC
#define PIN_LED_STATUS_Pin  GPIO_PIN_1
```

**Archivos a modificar:**
- `Core/Inc/main.h` — Añadir defines de pin
- `Core/Src/main.c` — Añadir GPIO init en `MX_GPIO_Init()` (push-pull, no pull, low speed)
- `Core/Src/safety_system.c` — Activar buzzer en estados de error/warning
- `Core/Src/can_handler.c` — LED toggle en recepción CAN (indicador de actividad)

#### Inicialización segura
1. Estado inicial: buzzer OFF, LED OFF
2. Test al boot: parpadeo LED 3× + beep corto (100 ms) para confirmar funcionamiento
3. Verificar que el GPIO no está configurado como entrada en otro módulo

#### Pruebas
- [ ] Toggle con osciloscopio: señal limpia
- [ ] Buzzer suena a volumen adecuado
- [ ] LED visible en todas las condiciones de luz
- [ ] Sin efecto en otros periféricos al activar/desactivar

---

## 6. Checklist de seguridad anti-bloqueos

### 6.1 Antes de cada integración

- [ ] **Watchdog**: Confirmar que `HAL_IWDG_Refresh()` se llama cada <500 ms en todas las ramas de código
- [ ] **Timeout en todas las comunicaciones**: Ninguna llamada HAL sin timeout (I2C: 50 ms, SPI: 10 ms, UART: 10 ms)
- [ ] **No usar `HAL_Delay()` en contexto de interrupción**: Nunca — causa deadlock
- [ ] **No usar `while(flag)` sin timeout**: Todo bucle de espera tiene `tick + TIMEOUT` como condición de salida
- [ ] **BREAK2 intacto**: TIM1/TIM8 BREAK2 input sigue conectado a LOCKUP — verificar registros BKR/BKR2
- [ ] **Error_Handler() funcional**: Verifica que corta TIM1/TIM8 MOE, zeros TIM3 CCR, fuerza relés LOW

### 6.2 Después de cada integración

- [ ] **Bucle de 10 ms no excedido**: Medir con GPIO toggle + osciloscopio — si >9.5 ms, optimizar
- [ ] **I2C no bloqueado**: Verificar que `hi2c1.State == HAL_I2C_STATE_READY` entre transacciones
- [ ] **CAN sin pérdidas**: Comparar `can_stats.tx_count` y `can_stats.rx_count` con los esperados
- [ ] **Stack no desbordado**: Revisar patrón de stack (`0xDEADBEEF`) en la parte baja del stack
- [ ] **Flash no corrompida**: Verificar CRC de pages 125–127 después de integración
- [ ] **Sin resets espurios**: Verificar `RCC_CSR` reset flags al boot — debe ser solo POR/software

### 6.3 Trampas comunes a evitar

| Trampa | Consecuencia | Prevención |
|--------|-------------|------------|
| I2C polling bloqueante en ISR | Watchdog reset | Solo I2C en bucle principal, nunca en ISR |
| Acceso a Flash sin deshabilitar IRQ | Corrupción de datos | `__disable_irq()` durante write, refrescar IWDG antes |
| Printf/sprintf en producción | Stack overflow, lentitud | Usar solo con `#ifdef DEBUG`, formato binario en producción |
| DMA buffer no alineado | Hard fault | `__attribute__((aligned(4)))` en buffers DMA |
| EXTI compartido sin check de pin | Handler ejecuta código incorrecto | Verificar `__HAL_GPIO_EXTI_GET_IT()` al inicio del handler |
| CAN TX sin espacio en buzón | HAL_BUSY infinito | Verificar `HAL_FDCAN_GetTxFifoFreeLevel()` antes de enviar |
| Conversión float en ISR | Jitter excesivo en FPU | Pre-calcular constantes, usar enteros en ISR |

---

## 7. Validación de interrupciones, temporizadores y buses compartidos

### 7.1 Mapa de interrupciones actual y prioridades

| IRQ | Prioridad | Handler | Función |
|-----|-----------|---------|---------|
| FDCAN1_IT0 | 1 | `FDCAN1_IT0_IRQHandler()` | CAN RX/TX (máxima prioridad aplicación) |
| TIM1_BRK_TIM15 | 0 | Configurado por HAL | BREAK2/LOCKUP PWM kill (prioridad sistema) |
| EXTI0 | 2 | `EXTI0_IRQHandler()` | Wheel FL speed |
| EXTI1 | 2 | `EXTI1_IRQHandler()` | Wheel FR speed |
| EXTI2 | 2 | `EXTI2_IRQHandler()` | Wheel RL speed |
| EXTI9_5 | 2 | `EXTI9_5_IRQHandler()` | PB5: Steering center |
| EXTI15_10 | 2 | `EXTI15_10_IRQHandler()` | PB15: Wheel RR + PB4: Encoder Z |
| I2C1_EV | 3 | `I2C1_EV_IRQHandler()` | I2C eventos (si IT mode) |
| SysTick | 15 | `SysTick_Handler()` | HAL tick 1 ms |

### 7.2 Reglas para añadir nuevas interrupciones

1. **Prioridad de nuevos periféricos:**
   - UART RX: prioridad **3** (igual o menor que I2C, mayor que SysTick)
   - SPI: prioridad **4** (menor que UART — SPI puede esperar)
   - GPIO auxiliar: prioridad **5** (mínima)

2. **No anidar interrupciones**: STM32 usa NVIC con preemption — configurar preemption bits ≤4

3. **Duración máxima de ISR**: <10 µs para mantener latencia de CAN y wheel speed

4. **Regla de ISR**: Set flag + exit. Todo el procesamiento pesado en el bucle principal.
   ```c
   void NEW_IRQHandler(void) {
       if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_x)) {
           __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_x);
           volatile_flag = 1;  // Solo flag, nada más
       }
   }
   ```

### 7.3 Validación de temporizadores

| Timer | Usado para | Verificación |
|-------|-----------|-------------|
| TIM1 | PWM FL/FR (20 kHz) | Frecuencia con osciloscopio en PA8, duty cycle correcto |
| TIM2 | Encoder cuadratura | Verificar que CNT incrementa/decrementa con giro manual |
| TIM3 | PWM dirección (20 kHz) | Frecuencia en PA6, duty cycle correcto |
| TIM8 | PWM RL/RR (20 kHz) | Frecuencia en PC6, duty cycle correcto |
| TIM6 | **LIBRE** | Disponible para temporización auxiliar |
| TIM7 | **LIBRE** | Disponible para temporización auxiliar |
| TIM15 | Compartido con TIM1_BRK | **NO usar** — asociado a BREAK |
| TIM16, TIM17, TIM20 | **LIBRE** | Disponibles |

> **PRECAUCIÓN**: No modificar prescaler ni period de TIM1/TIM3/TIM8 — afecta directamente a la frecuencia PWM de los motores.

### 7.4 Validación de buses compartidos

**I2C1 — bus compartido con TCA9548A, INA226 ×6, ADS1115:**

1. Medir tiempo total de `Current_ReadAll()` + `Pedal_Update()` (ADS1115)
2. Verificar que no excede 5 ms por ciclo de 50 ms (20 Hz)
3. Cada nuevo dispositivo I2C añade ~0.5–1 ms por transacción
4. Si >7 dispositivos directos (sin TCA): considerar segundo bus I2C
5. Tras añadir: verificar con osciloscopio que no hay clock stretching excesivo

**FDCAN1 — bus compartido STM32 ↔ ESP32:**

1. Verificar carga del bus: a 500 kbps con 27 mensajes, la carga es ~15%
2. Cada mensaje nuevo (8 bytes + overhead) ocupa ~240 µs = ~0.5% de carga
3. Máximo recomendado: 70% de carga del bus
4. Nuevos IDs: actualizar filtro FDCAN en `CAN_Init()` o ampliar máscara
5. Actualizar `can_ids.h` en ambos firmwares (STM32 + ESP32) simultáneamente

**OneWire — bus compartido DS18B20 ×5:**

1. Cada sensor añade ~750 ms de conversión (si secuencial)
2. Actualmente: conversión en broadcast → lectura secuencial → ~100 ms total
3. Cada sensor nuevo añade ~10 ms de lectura individual
4. Máximo práctico: 10 sensores por bus sin problemas de timing

---

## 8. Estrategia de pruebas progresivas

### 8.1 Niveles de prueba

```
Nivel 0: Compilación limpia (0 warnings con -Wall -Wextra)
   ↓
Nivel 1: Prueba unitaria del componente aislado (solo el nuevo módulo activo)
   ↓
Nivel 2: Prueba con componentes del mismo bus (I2C, CAN, etc.)
   ↓
Nivel 3: Prueba integrada con todos los sistemas activos
   ↓
Nivel 4: Prueba de estrés (operación continua ≥1 hora)
   ↓
Nivel 5: Prueba de fallo (desconexión, sobrecarga, EMI)
```

### 8.2 Procedimiento por nivel

**Nivel 0 — Compilación:**
```bash
make clean && make -j4 2>&1 | grep -E "warning|error"
# Resultado esperado: 0 errores, 0 warnings
```

**Nivel 1 — Unitario aislado:**
1. Desactivar temporalmente otros módulos no críticos en `main.c`
2. Inicializar solo el nuevo componente
3. Verificar lectura/escritura básica
4. Verificar timeout y recovery
5. Restaurar módulos desactivados

**Nivel 2 — Bus compartido:**
1. Activar todos los dispositivos del mismo bus
2. Ejecutar ciclo de lecturas completo
3. Medir tiempos con GPIO toggle
4. Verificar que no hay colisiones ni NACK inesperados

**Nivel 3 — Integración completa:**
1. Activar todos los sistemas
2. Ejecutar secuencia completa: boot → standby → active → safe
3. Verificar telemetría CAN con monitor externo
4. Verificar que todas las lecturas de sensores son coherentes
5. Verificar timing del bucle principal

**Nivel 4 — Estrés:**
1. Operar sistema completo durante ≥1 hora
2. Monitorizar: temperatura MCU, consumo eléctrico, contadores de error CAN
3. Verificar: sin resets, sin errores I2C, sin bus-off CAN
4. Verificar: lecturas de sensores estables (sin drift anormal)

**Nivel 5 — Fallo provocado:**
1. Desconectar el nuevo sensor durante operación → sistema debe degradar sin crash
2. Cortocircuitar bus I2C brevemente → recovery automático
3. Saturar bus CAN con tráfico externo → verificar que no hay pérdida de heartbeat
4. Cortar alimentación y reconectar → boot limpio sin corrupción de Flash

### 8.3 Criterio de paso/fallo

| Criterio | PASS | FAIL |
|----------|------|------|
| Compilación | 0 errors, 0 warnings | Cualquier error o warning nuevo |
| Timing bucle | ≤9.5 ms en peor caso | >9.5 ms en cualquier iteración |
| I2C | 0 errores en 10000 transacciones | ≥1 error no recuperado |
| CAN | 0 pérdidas en 1 hora | ≥1 mensaje perdido sin recovery |
| Watchdog | 0 resets espurios en 1 hora | ≥1 reset no provocado |
| Degradación | Sistema transiciona a modo seguro | Sistema se bloquea o no reacciona |
| Recovery | Sistema vuelve a ACTIVE tras fallo temporal | Sistema queda en SAFE permanente |

---

## 9. Documentación y rollback

### 9.1 Documentación obligatoria por paso

Para cada componente integrado, crear o actualizar:

1. **Commit Git con mensaje descriptivo:**
   ```
   feat(sensor): add INA226 channel 6 on TCA9548A
   
   - Added current sensor on I2C mux channel 6
   - Updated Current_ReadAll() for 7 sensors
   - Added overcurrent check in safety_system
   - Tested: 1h continuous operation, 0 I2C errors
   ```

2. **Actualizar este documento** (`INTEGRATION_PLAN.md`):
   - Marcar componente como integrado con fecha
   - Registrar pin final usado
   - Registrar cualquier desviación del plan

3. **Actualizar `main.h`** con nuevos defines de pin

4. **Actualizar `README.md`** con nueva tabla de pines si cambia

5. **Actualizar `.ioc`** en CubeMX si se añaden periféricos

### 9.2 Estrategia de rollback

**Antes de cada integración:**
```bash
# Crear rama de feature
git checkout -b feature/add-ina226-ch6

# Commit del estado actual (pre-integración)
git add -A && git commit -m "checkpoint: pre INA226 ch6 integration"
```

**Si la integración falla:**
```bash
# Opción 1: Revertir último commit
git revert HEAD

# Opción 2: Volver al checkpoint
git reset --hard HEAD~N  # N = número de commits de la integración

# Opción 3: Descartar rama completa
git checkout main
git branch -D feature/add-ina226-ch6
```

**Regla**: Nunca integrar directamente en `main`. Siempre usar ramas de feature.

### 9.3 Registro de cambios

| Fecha | Componente | Estado | Rama | Notas |
|-------|-----------|--------|------|-------|
| — | INA226 ch6 | Pendiente | — | — |
| — | DS18B20 +1 | Pendiente | — | — |
| — | UART (PA4/PA5) | Pendiente | — | — |
| — | SPI2 (PB12-14) | Pendiente | — | — |
| — | ADC PB1 | Pendiente | — | — |
| — | GPIO PC0-PC4 | Pendiente | — | — |

---

## 10. Criterios de estabilidad por etapa

### 10.1 Definición de "estable"

Una etapa se considera **ESTABLE** cuando cumple **TODOS** estos criterios simultáneamente:

| # | Criterio | Medición | Umbral |
|---|----------|----------|--------|
| 1 | **Sin resets** | Contador de resets en RCC_CSR | 0 resets espurios en 1 hora |
| 2 | **Timing cumplido** | GPIO toggle en osciloscopio | Bucle ≤9.5 ms (95% del periodo) |
| 3 | **I2C limpio** | Contador errores I2C | 0 errores no recuperados en 1 hora |
| 4 | **CAN operativo** | `can_stats` TX/RX | 0 mensajes perdidos, 0 bus-off en 1 hora |
| 5 | **Sensores coherentes** | Lecturas de telemetría | Valores dentro de rangos físicos esperados |
| 6 | **Safety funcional** | Provocar fallo controlado | Sistema transiciona a estado seguro correctamente |
| 7 | **Boot limpio** | `BootValidation_Run()` | Todos los checks pasan al reiniciar |
| 8 | **Flash íntegra** | CRC pages 125–127 | CRC válido tras 1 hora de operación |
| 9 | **Consumo estable** | Amperímetro en línea 3.3 V | Variación <10% respecto a línea base |
| 10 | **Watchdog OK** | Monitor de resets | 0 resets por IWDG en 1 hora |

### 10.2 Procedimiento de certificación de etapa

```
1. Ejecutar sistema completo durante 1 hora mínimo
2. Registrar todos los contadores (CAN stats, I2C errors, resets)
3. Provocar al menos 1 fallo controlado (desconexión de sensor)
4. Verificar recuperación automática
5. Reiniciar y verificar boot limpio
6. Comparar valores con la línea base pre-integración
7. Si todos los criterios PASS → etapa ESTABLE → permitir avance
8. Si cualquier criterio FAIL → investigar, corregir, repetir desde paso 1
```

### 10.3 Tabla de progreso de estabilidad

| Etapa | Componentes | Estado | Fecha certificación | Operador |
|-------|------------|--------|--------------------:|----------|
| Base | Firmware actual (18 módulos) | **Pendiente verificación** | — | — |
| 1a | + INA226 ch6 | Pendiente | — | — |
| 1b | + DS18B20 +1 | Pendiente | — | — |
| 2a | + UART | Pendiente | — | — |
| 2b | + SPI2 | Pendiente | — | — |
| 2c | + ADC PB1 | Pendiente | — | — |
| 3a | + GPIO PC0-PC4 | Pendiente | — | — |
| Final | Todos integrados | Pendiente | — | — |

---

## Apéndice A: Mapa de memoria Flash reservada

| Página | Dirección | Tamaño | Uso | Módulo |
|--------|-----------|--------|-----|--------|
| 125 | 0x0807C000 | 4 KB | Error log (ring buffer) | `error_log.c` |
| 126 | 0x0807E000 | 4 KB | Calibración dirección | `steering_cal_store.c` |
| 127 | 0x0807F000 | 4 KB | Parámetros EPS (double-buffer) | `eps_params.c` |
| 0–124 | 0x08000000–0x0807BFFF | 496 KB | Firmware | Linker script |

> **REGLA**: No usar páginas 125–127 para datos nuevos. Si se necesita almacenamiento Flash adicional, usar página 124 (0x0807A000) con el mismo patrón de magic + CRC32.

## Apéndice B: Referencia rápida de AF (Alternate Functions) para pines libres

| Pin | AF1 | AF2 | AF4 | AF5 | AF7 | AF8 | AF12 |
|-----|-----|-----|-----|-----|-----|-----|------|
| PA4 | — | — | — | SPI1_NSS | USART2_CK | — | LPUART1_TX |
| PA5 | — | — | — | SPI1_SCK | — | — | — |
| PB1 | — | TIM3_CH4 | — | — | — | — | LPUART1_DE |
| PB2 | — | — | — | SPI3_MOSI | — | — | — |
| PB12 | — | — | I2C2_SMBA | SPI2_NSS | — | — | — |
| PB13 | — | — | — | SPI2_SCK | — | — | — |
| PB14 | — | — | — | SPI2_MISO | — | — | — |
| PC0 | — | — | — | — | — | — | — |
| PC1 | — | — | — | — | — | — | — |
| PC2 | — | — | — | — | — | — | — |
| PC3 | — | — | — | — | — | — | — |
| PC4 | — | — | — | — | USART1_TX | — | — |

> **Nota**: Verificar siempre la tabla completa de AF en el datasheet oficial STM32G474RE (DS12288) ya que las AF pueden variar entre revisiones del silicon.

## Apéndice C: Plantilla de test para nuevo componente

```c
/* test_new_component.c — plantilla de prueba unitaria */
#include "main.h"
#include "new_component.h"

typedef struct {
    uint8_t init_ok;
    uint8_t read_ok;
    uint8_t write_ok;
    uint8_t recovery_ok;
    uint8_t timing_ok;
    uint32_t avg_time_us;
    uint32_t max_time_us;
    uint32_t error_count;
} ComponentTestResult_t;

ComponentTestResult_t Test_NewComponent(void) {
    ComponentTestResult_t result = {0};
    uint32_t start, elapsed;
    
    /* Test 1: Inicialización */
    result.init_ok = (NewComponent_Init() == HAL_OK) ? 1 : 0;
    if (!result.init_ok) return result;
    
    /* Test 2: Lectura básica */
    uint16_t value;
    result.read_ok = (NewComponent_Read(&value) == HAL_OK) ? 1 : 0;
    
    /* Test 3: Timing (100 iteraciones) */
    uint32_t total_time = 0;
    result.max_time_us = 0;
    for (int i = 0; i < 100; i++) {
        start = DWT->CYCCNT;
        NewComponent_Read(&value);
        elapsed = (DWT->CYCCNT - start) / 170; /* 170 MHz → µs */
        total_time += elapsed;
        if (elapsed > result.max_time_us) result.max_time_us = elapsed;
    }
    result.avg_time_us = total_time / 100;
    result.timing_ok = (result.max_time_us < 5000) ? 1 : 0; /* <5 ms */
    
    /* Test 4: Recovery (simular fallo) */
    /* Desactivar componente, intentar leer, verificar timeout sin crash */
    HAL_StatusTypeDef status = NewComponent_Read(&value);
    result.recovery_ok = (status == HAL_TIMEOUT || status == HAL_ERROR) ? 1 : 0;
    
    return result;
}
```

---

*Documento generado a partir del análisis del firmware real del proyecto STM32-Control-Coche-Marcos. Todas las direcciones, pines, periféricos y configuraciones reflejan el estado actual del código fuente.*
