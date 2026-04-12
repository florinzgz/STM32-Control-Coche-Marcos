# 📌 Especificación Completa de Hardware

**Sistema de Control Vehicular - STM32G474RE**

---

## 📋 Tabla de Contenidos

1. [Arquitectura del Sistema](#-arquitectura-del-sistema)
2. [Microcontrolador STM32G474RE](#-microcontrolador-stm32g474re)
3. [Sensores](#-sensores)
4. [Actuadores y Motores](#-actuadores-y-motores)
5. [Comunicación](#-comunicación)
6. [Alimentación](#-alimentación)
7. [Interfaz de Usuario](#-interfaz-de-usuario)
8. [Lista de Materiales (BOM)](#-lista-de-materiales-bom)

---

## 🏗️ Arquitectura del Sistema

### Diagrama de Bloques

```
┌─────────────────────────────────────────────────────────────────────┐
│                           VEHÍCULO ELÉCTRICO                         │
│                                                                       │
│  ┌──────────────────┐         ┌──────────────────────────────────┐  │
│  │   BATERÍA 24V    │────────►│        RELÉS DE POTENCIA         │  │
│  │   (Principal)    │         │  ├─ RELAY_MAIN (Power-Hold)      │  │
│  └──────────────────┘         │  ├─ RELAY_TRAC (Tracción)        │  │
│                                │  └─ RELAY_DIR (Dirección)        │  │
│                                └──────────┬───────────────────────┘  │
│                                           │                          │
│  ┌────────────────────────────────────────┼─────────────────────┐   │
│  │              STM32G474RE CONTROL BOARD │                     │   │
│  │                                        │                     │   │
│  │  ┌──────────────┐     ┌────────────────▼──────────────────┐ │   │
│  │  │ Sensores (5) │     │      DRIVERS MOTORES (5×)         │ │   │
│  │  │  - 4 Ruedas  │────►│                                    │ │   │
│  │  │  - 1 Encoder │     │  ┌──────┐  ┌──────┐  ┌──────┐    │ │   │
│  │  └──────────────┘     │  │BTS   │  │BTS   │  │BTS   │... │ │   │
│  │                       │  │7960  │  │7960  │  │7960  │    │ │   │
│  │  ┌──────────────┐     │  │ FL   │  │ FR   │  │ RL   │    │ │   │
│  │  │ Temp (5×)    │     │  └──┬───┘  └──┬───┘  └──┬───┘    │ │   │
│  │  │  - DS18B20   │     └─────┼─────────┼─────────┼────────┘ │   │
│  │  └──────────────┘           │         │         │          │   │
│  │                          ┌───▼─────┐ ┌─▼─────┐ ┌─▼─────┐  │   │
│  │  ┌──────────────┐        │ Motor  │ │ Motor │ │ Motor │  │   │
│  │  │ Corriente    │        │  FL    │ │  FR   │ │  RL   │  │   │
│  │  │ (6× INA226)  │        │ 250W   │ │ 250W  │ │ 250W  │  │   │
│  │  └──────────────┘        └────────┘ └───────┘ └───────┘  │   │
│  │                                                            │   │
│  │  ┌──────────────┐     CAN Bus @ 500 kbps                  │   │
│  │  │  Pedal Hall  │◄────────────────────────────────────┐   │   │
│  │  │  Shifter F/N/R│                                    │   │   │
│  │  └──────────────┘                                     │   │   │
│  └──────────────────────────────────────────────────────┬┘   │   │
│                                                          │    │   │
│  ┌───────────────────────────────────────────────────────▼──┐│   │
│  │                     ESP32-S3 (HMI)                        ││   │
│  │  - Display TFT Touch                                      ││   │
│  │  - Audio DFPlayer                                         ││   │
│  │  - LEDs WS2812B                                           ││   │
│  │  - Menús y Diagnóstico                                    ││   │
│  └───────────────────────────────────────────────────────────┘│   │
└───────────────────────────────────────────────────────────────────┘
```

---

## 💻 Microcontrolador STM32G474RE

### Especificaciones Técnicas

| Característica | Valor | Notas |
|----------------|-------|-------|
| **Núcleo** | ARM Cortex-M4F | FPU de hardware |
| **Frecuencia** | 170 MHz | Máxima del STM32G4 |
| **Flash** | 512 KB | Código de programa |
| **RAM** | 128 KB | 96KB SRAM1 + 32KB CCM-SRAM |
| **Encapsulado** | LQFP64 | 64 pines |
| **Tensión** | 3.3V | 2.0-3.6V tolerante |
| **ADC** | 5× 12-bit | Hasta 4 MSPS |
| **DAC** | 4× 12-bit | Generación señales analógicas |
| **Timers** | 11 (incluye TIM1/TIM8 avanzados) | PWM de alta resolución |
| **FDCAN** | 3 instancias | CAN-FD compatible (usamos 1) |
| **I²C** | 4 instancias | Fast-mode+ hasta 1 MHz |
| **SPI** | 3 instancias | Hasta 42.5 Mbps |
| **USART** | 5 instancias | UART, IRDA, LIN |
| **GPIO** | 51 pines I/O | 5V tolerante en algunos |

### Consumo de Potencia

| Modo | Corriente | Notas |
|------|-----------|-------|
| **Run (170 MHz)** | ~80 mA | Operación normal |
| **Sleep** | ~25 mA | CPU detenido, periféricos activos |
| **Stop** | ~5 µA | Retención de RAM |
| **Standby** | ~1 µA | Sin retención (excepto backup) |

### Características Especiales

- ✅ **FPU (Floating Point Unit):** Aceleración punto flotante hardware
- ✅ **DSP Instructions:** SIMD para procesamiento de señales
- ✅ **MPU (Memory Protection Unit):** Protección de memoria
- ✅ **DMA (8 canales):** Transferencias sin CPU
- ✅ **Watchdog:** IWDG independiente + WWDG ventana
- ✅ **RTC:** Real-Time Clock con batería backup

---

## 🔍 Sensores

### 1. Sensores de Rueda (4 unidades)

**Función:** Medir velocidad de cada rueda para ABS/TCS y odometría.

| Especificación | Valor |
|----------------|-------|
| **Tipo** | Efecto Hall / Reed switch |
| **Pulsos/rev** | Variable (típico 1-4 PPR) |
| **Tensión** | 3.3V - 5V |
| **Salida** | Digital (pull-up activo) |
| **Pines STM32** | PB0 (FL), PB1 (FR), PB2 (RL), PB10 (RR) |

**Conexión:**
```
Sensor → Pull-up 10kΩ a 3.3V → STM32 GPIO
```

### 2. Encoder de Dirección E6B2-CWZ6C

**Función:** Medir posición angular del volante/dirección (control en lazo cerrado).

| Especificación | Valor |
|----------------|-------|
| **Modelo** | Omron E6B2-CWZ6C |
| **Resolución** | 1200 PPR (Pulses Per Revolution) |
| **Modo** | Incremental cuadratura (A/B/Z) |
| **Conteos/rev** | 4800 (1200 × 4 en quadrature) |
| **Resolución angular** | 0.075° por count (360° / 4800) |
| **Tensión** | 5V - 12V |
| **Salida** | Line driver (compatible 3.3V) |
| **Pines STM32** | PA15 (A), PB3 (B), PB4 (Z) |

**Resolución Angular:**
- 1 conteo = 360° / 1440 = **0.25°**
- Rango típico: ±180° (±720 conteos)

### 3. Sensores de Temperatura DS18B20 (5 unidades)

**Función:** Monitoreo térmico de motores y ambiente.

| Especificación | Valor |
|----------------|-------|
| **Modelo** | Dallas DS18B20 |
| **Protocolo** | 1-Wire (bus compartido) |
| **Rango** | -55°C a +125°C |
| **Resolución** | 0.0625°C (12-bit) |
| **Precisión** | ±0.5°C (-10°C a +85°C) |
| **Tiempo conversión** | 750 ms (12-bit) |
| **Pin STM32** | PB5 (bus OneWire) |

**Distribución:**
- 1× Motor FL (tracción)
- 1× Motor FR (tracción)
- 1× Motor RL (tracción)
- 1× Motor RR (tracción)
- 1× Ambiente (referencia)

**Conexión:**
```
DS18B20 (5×) ─┬─ Pull-up 4.7kΩ ─ 3.3V
              └─ PB5 (Open-Drain)
```

### 4. Sensores de Corriente INA226 (6 unidades)

**Función:** Monitoreo de corriente y tensión en motores y batería.

| Especificación | Valor |
|----------------|-------|
| **Modelo** | Texas Instruments INA226 |
| **Protocolo** | I²C @ 400 kHz |
| **Rango tensión** | 0-36V |
| **Rango corriente** | ±54.6A (motor, shunt 1.5 mΩ) / ±109.2A (batería, shunt 0.75 mΩ) |
| **Resolución** | 1.25 mA (corriente), 1.25 mV (tensión) |
| **Precisión** | ±0.1% (ganancia) |
| **Direcciones I²C** | Todas 0x40 (aisladas por canal TCA9548A) |
| **Multiplexor** | TCA9548A (8 canales) @ 0x70 |
| **Pines STM32** | PB6 (SCL), PB7 (SDA) |

**Distribución (todos a dirección I2C 0x40, separados por canal TCA9548A):**
- Canal 0: INA226 — Motor FL (1.5 mΩ)
- Canal 1: INA226 — Motor FR (1.5 mΩ)
- Canal 2: INA226 — Motor RL (1.5 mΩ)
- Canal 3: INA226 — Motor RR (1.5 mΩ)
- Canal 4: INA226 — Batería 24V (0.75 mΩ)
- Canal 5: INA226 — Motor Dirección (1.5 mΩ)

**Shunt Resistor:** 0.0015Ω (1.5 mΩ) motores / 0.00075Ω (0.75 mΩ) batería — @ 2W mínimo

### 5. Pedal Analógico Hall

**Función:** Sensor de aceleración sin contacto.

| Especificación | Valor |
|----------------|-------|
| **Tipo** | Sensor Hall lineal |
| **Salida** | Analógica 0-3.3V |
| **Pin STM32** | PA0 (ADC1_IN1) |
| **Resolución ADC** | 12-bit (4096 valores) |
| **Frecuencia muestreo** | 200 Hz (trigger TIM3) |

**Mapeo:**
- 0V → 0% throttle (reposo)
- 3.3V → 100% throttle (máximo)

### 6. Shifter Mecánico F/N/R

**Función:** Selector de marcha (Forward/Neutral/Reverse).

| Especificación | Valor |
|----------------|-------|
| **Tipo** | Switch mecánico 3 posiciones |
| **Posiciones** | Forward, Neutral, Reverse |
| **Lógica** | Activo BAJO (pull-up interno) |
| **Pines STM32** | PB12 (FWD), PB13 (NEU) — PB14 reasignado a LED_DIAG |

**Exclusividad:** Solo UNA posición puede estar activa simultáneamente (hardware).

---

## ⚙️ Actuadores y Motores

### 1. Motores de Tracción (4 unidades)

**Especificación:**

| Parámetro | Valor |
|-----------|-------|
| **Tipo** | DC Brushed / Brushless |
| **Potencia** | 250W por motor (1000W total) |
| **Tensión nominal** | 24V |
| **Corriente nominal** | ~10A |
| **Corriente pico** | ~20A |
| **RPM máximas** | 3000 RPM |
| **Par** | ~0.8 Nm |

**Distribución:**
- Motor FL (Front Left)
- Motor FR (Front Right)
- Motor RL (Rear Left)
- Motor RR (Rear Right)

### 2. Motor de Dirección

| Parámetro | Valor |
|-----------|-------|
| **Tipo** | DC Brushed con reductor |
| **Potencia** | 100W |
| **Tensión** | 24V |
| **Corriente nominal** | ~4A |
| **Reductor** | 1:20 (aprox.) |
| **Par salida** | ~15 Nm |

### 3. Drivers BTS7960 (5 unidades)

**Especificación del Driver:**

| Parámetro | Valor |
|-----------|-------|
| **Modelo** | Infineon BTS7960 |
| **Configuración** | H-Bridge doble |
| **Tensión máx.** | 5.5V - 27V (motores) |
| **Corriente continua** | 43A por canal |
| **Frecuencia PWM** | Hasta 25 kHz (óptimo: 20 kHz) |
| **Lógica control** | 3.3V / 5V compatible |
| **Protecciones** | Sobrecorriente, sobretemperatura |

**Señales de Control (por motor):**
- **PWM:** Modulación de potencia (0-100%)
- **DIR:** Dirección de giro (LOW=CW, HIGH=CCW)
- **EN:** Habilitación (HIGH=activo, LOW=deshabilitado)

---

## 📡 Comunicación

### 1. CAN Bus

**Especificación:**

| Parámetro | Valor |
|-----------|-------|
| **Standard** | CAN 2.0A (11-bit IDs) |
| **Bitrate** | 500 kbps |
| **Transceptor** | TJA1051T/3 (NXP/Nexperia) |
| **Tensión transceptor** | 5V |
| **Pines STM32** | PA11 (RX), PA12 (TX) |
| **Terminación** | 120Ω en ambos extremos |

**Topología:**
```
STM32 ─── 120Ω ─── [CAN_H/CAN_L] ─── 120Ω ─── ESP32
          Nodo 1      (max 40m)              Nodo 2
```

**Longitud Máxima del Bus:**
- @ 500 kbps: 40 metros
- @ 250 kbps: 100 metros

### 2. I²C

**Especificación:**

| Parámetro | Valor |
|-----------|-------|
| **Velocidad** | 400 kHz (Fast-mode) |
| **Pull-ups** | 4.7 kΩ a 3.3V |
| **Pines STM32** | PB6 (SCL), PB7 (SDA) |
| **Dispositivos** | 6× INA226 + 1× TCA9548A |

**Direcciones I²C:**
- TCA9548A: 0x70 (multiplexor)
- INA226 #1-6: todas 0x40 (aisladas por canales TCA9548A 0–5)

### 3. OneWire

**Especificación:**

| Parámetro | Valor |
|-----------|-------|
| **Protocolo** | Dallas 1-Wire |
| **Velocidad** | Standard (16 kbps) |
| **Pull-up** | 4.7 kΩ a 3.3V |
| **Pin STM32** | PB5 (Open-Drain) |
| **Dispositivos** | 5× DS18B20 |

---

## 🔋 Alimentación

### Arquitectura de Alimentación

```
┌─────────────────┐
│  Batería 24V    │
│  (20Ah Li-Ion)  │
└────────┬────────┘
         │
         ├──────► [BTS7960 × 5] ──► Motores (24V)
         │
         ├──────► DC-DC 5V (3A) ──► TJA1051T/3, Sensores
         │
         └──────► DC-DC 3.3V (2A) ──► STM32, Lógica
```

### Fuentes de Alimentación

| Tensión | Corriente | Uso |
|---------|-----------|-----|
| **24V** | 50A pico | Motores (via relés) |
| **5V** | 3A | Transceptores, sensores |
| **3.3V** | 2A | STM32, lógica |

**Reguladores:**
- **24V → 5V:** LM2596 (DC-DC Buck, hasta 3A)
- **5V → 3.3V:** AMS1117-3.3 (LDO, 1A) o otro Buck

### Relés de Potencia (3 unidades)

**Especificación:**

| Relé | Tensión Bobina | Corriente Contacto | Función |
|------|----------------|-------------------|---------|
| **RELAY_MAIN** | 5V | 30A | Power-hold (mantener sistema ON) |
| **RELAY_TRAC** | 5V | 50A | Alimentación motores tracción |
| **RELAY_DIR** | 5V | 20A | Alimentación motor dirección |

**Control:**
- Transistor NPN (ej. 2N2222) para conmutar bobina
- Diodo flyback (1N4007) para protección
- Estado por defecto: **LOW** (fail-safe, relés abiertos)

---

## 🎮 Interfaz de Usuario

### Hardware de Entrada

| Componente | Especificación |
|------------|----------------|
| **Pedal Hall** | Analógico 0-3.3V |
| **Shifter F/N/R** | 3× GPIO input (pull-up) |
| **Botón Emergencia** | NO (Normally Open) a GND |

### Hardware de Salida (vía ESP32)

| Componente | Especificación |
|------------|----------------|
| **Display TFT** | 3.5" ILI9486 320×480 Touch |
| **LEDs WS2812B** | 12× LEDs RGB direccionables |
| **Buzzer/Speaker** | DFPlayer Mini MP3 |

---

## 📦 Lista de Materiales (BOM)

### Componentes Principales

| Cantidad | Componente | Referencia | Precio Aprox. |
|----------|------------|------------|---------------|
| 1 | STM32G474RE (NUCLEO) | NUCLEO-G474RE | €20 |
| 5 | Driver BTS7960 | BTS7960 H-Bridge | €15 (5×) |
| 5 | Motor DC 24V 250W | - | €100 (5×) |
| 1 | Encoder E6B2-CWZ6C | E6B2-CWZ6C-1200P/R | €40 |
| 6 | INA226 breakout | INA226 | €12 (6×) |
| 1 | TCA9548A multiplexor | TCA9548A | €3 |
| 5 | DS18B20 temperatura | DS18B20 | €5 (5×) |
| 1 | Transceptor CAN | TJA1051T/3 | €2 |
| 1 | Pedal Hall | Hall sensor linear | €15 |
| 1 | Shifter F/N/R | 3-pos switch | €10 |
| 3 | Relé 30A/50A | SRD-05VDC-SL-C | €6 (3×) |
| 1 | Batería 24V 20Ah | Li-Ion pack | €150 |
| 1 | DC-DC 24V→5V 3A | LM2596 | €5 |
| 1 | LDO 5V→3.3V 1A | AMS1117-3.3 | €1 |

**Total aproximado:** ~€400 (sin incluir chasis, ruedas, estructura)

### Componentes Pasivos

| Cantidad | Componente | Valor |
|----------|------------|-------|
| 2 | Resistencia terminación CAN | 120Ω 1/4W |
| 5 | Shunt resistor (INA226 motores) | 0.0015Ω (1.5 mΩ) ≥4W |
| 1 | Shunt resistor (INA226 batería) | 0.00075Ω (0.75 mΩ) ≥8W |
| 2 | Pull-up I²C | 4.7kΩ 1/4W |
| 1 | Pull-up OneWire | 4.7kΩ 1/4W |
| 10 | Pull-up GPIO | 10kΩ 1/4W |
| 5 | Diodo flyback (relés) | 1N4007 |
| 5 | Transistor NPN (relés) | 2N2222 |
| 10 | Condensador bypass | 100nF cerámico |
| 5 | Condensador bulk | 100µF electrolítico |

---

## 📖 Referencias

- [STM32G474RE Datasheet](https://www.st.com/resource/en/datasheet/stm32g474re.pdf)
- [BTS7960 Datasheet](https://www.infineon.com/dgdl/Infineon-BTS7960-DS-v01_00-EN.pdf)
- [E6B2-CWZ6C Manual](https://www.ia.omron.com/products/family/487/)
- [INA226 Datasheet](https://www.ti.com/lit/ds/symlink/ina226.pdf)
- [TJA1051T/3 Datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf)
- [DS18B20 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf)

---

**Última actualización:** 2026-02-01  
**Autor:** florinzgz  
**Proyecto:** STM32-Control-Coche-Marcos
