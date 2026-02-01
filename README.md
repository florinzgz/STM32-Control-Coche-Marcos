# STM32-Control-Coche-Marcos

**Firmware de Control Seguro para Vehículo Eléctrico Inteligente**

[![Platform](https://img.shields.io/badge/Platform-STM32G474RE-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32g474re.html)
[![CAN Bus](https://img.shields.io/badge/CAN-500%20kbps-green.svg)](https://www.iso.org/standard/63648.html)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-In%20Development-orange.svg)]()

---

## 🎯 Descripción del Proyecto

Este repositorio contiene el **firmware de control seguro** basado en **STM32G474RE** para un vehículo eléctrico de 4 ruedas con dirección Ackermann. El sistema gestiona:

- ✅ **Control de motores:** 4 motores de tracción independientes + motor de dirección
- ✅ **Sistemas de seguridad:** ABS, TCS, protección térmica, watchdog
- ✅ **Sensores críticos:** Encoder dirección, velocidad de ruedas, corriente, temperatura
- ✅ **Comunicación CAN:** Protocolo robusto @ 500 kbps con ESP32-S3 (HMI)

---

## 🏗️ Arquitectura del Sistema

```
┌─────────────────────────┐         CAN Bus        ┌──────────────────────────┐
│     ESP32-S3 (HMI)      │◄────── 500 kbps ──────►│   STM32G474RE (Control)  │
│                         │      TJA1051T/3        │                          │
│ - Display TFT + Touch   │                        │ - Motores tracción (4×)  │
│ - Audio DFPlayer        │     Comandos HMI       │ - Motor dirección        │
│ - LEDs WS2812B          │  ─────────────────►    │ - Encoder A/B/Z          │
│ - Menús + Diagnóstico   │                        │ - Sensores rueda (4×)    │
│ - Detección obstáculos  │  ◄─────────────────    │ - INA226 (corrientes)    │
│                         │    Estado del sistema  │ - DS18B20 (temperaturas) │
│                         │                        │ - Pedal + Shifter        │
│                         │                        │ - ABS/TCS + Safety       │
└─────────────────────────┘                        └──────────────────────────┘
```

**Separación de responsabilidades:**
- **ESP32-S3 (HMI):** Interfaz de usuario, visualización, feedback audible/visual
- **STM32G474RE (Control):** Control de tiempo real, seguridad funcional, decisión final

---

## ✨ Características Principales

### ⚡ Control de Motores (PWM Directo)

- **TIM1** (4 canales) - Motores de tracción FL/FR/RL/RR @ 20 kHz
- **TIM8** (1 canal) - Motor de dirección @ 20 kHz
- **Frecuencia PWM:** 20 kHz (inaudible, baja vibración)
- **Resolución:** ~13 bits (8500 pasos)
- **NO se usa PCA9685** - Control directo desde STM32 para mínima latencia

### Hardware STM32G474RE

| Especificación | Valor |
|----------------|-------|
| **MCU** | ARM Cortex-M4F @ 170 MHz |
| **Flash** | 512 KB |
| **RAM** | 128 KB |
| **FPU** | ✅ Sí (cálculos punto flotante) |
| **FDCAN** | 3 instancias (usamos FDCAN1) |
| **ADC** | 5× 12-bit, hasta 4 MSPS |
| **Timers** | 11 (TIM1/TIM8 avanzados para PWM) |
| **I²C** | 4 instancias @ 400 kHz |
| **GPIO** | 51 pines I/O |

### Periféricos Conectados

#### Motores y Actuadores
- **5× BTS7960** - Drivers H-Bridge: 4 motores tracción + 1 motor dirección (control PWM DIRECTO, NO PCA9685)
- **3× Relés** - Main Power, Tracción, Dirección (fail-safe LOW)

#### Sensores
- **5× Sensores de rueda** - 4 sensores rueda + 1 encoder dirección E6B2-CWZ6C (360 PPR, 1440 conteos/rev)
- **6× INA226** - Monitoreo de corriente: 4 motores tracción + 1 motor dirección + 1 batería (vía I²C + TCA9548A)
- **5× DS18B20** - Sensores de temperatura: 4 motores + 1 ambiente (OneWire)
- **Pedal Hall** - Sensor analógico sin contacto
- **Shifter F/N/R** - Selector de marcha mecánico

#### Comunicación
- **TJA1051T/3** - Transreceptor CAN High-Speed @ 500 kbps
- **FDCAN1** - Controlador CAN interno del STM32

---

## 🚀 Inicio Rápido

### Requisitos Previos

**Software:**
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) ≥ 1.14.0
- [Git](https://git-scm.com/)
- [ST-Link Utility](https://www.st.com/en/development-tools/stsw-link004.html) (opcional)

**Hardware:**
- NUCLEO-G474RE o placa compatible
- Transreceptor CAN TJA1051T/3
- 2× Resistencias 120Ω (terminación CAN)
- Cable USB para programación
- Fuente de alimentación 5V regulada

### Instalación y Compilación

```bash
# 1. Clonar el repositorio
git clone https://github.com/florinzgz/STM32-Control-Coche-Marcos.git
cd STM32-Control-Coche-Marcos

# 2. Abrir en STM32CubeIDE
# File → Open Projects from File System → Seleccionar carpeta raíz

# 3. Compilar
# Project → Build Project (Ctrl+B)
# O desde terminal:
make all

# 4. Flashear
# Run → Debug (F11) o usar ST-Link CLI
```

---

## 📚 Documentación

### 📖 Documentos Principales

| Documento | Descripción | Link |
|-----------|-------------|------|
| **HARDWARE.md** | 📌 Especificación completa de hardware | [docs/HARDWARE.md](docs/HARDWARE.md) |
| **CAN_PROTOCOL.md** | 📡 Protocolo CAN ESP32↔STM32 | [docs/CAN_PROTOCOL.md](docs/CAN_PROTOCOL.md) |
| **PINOUT.md** | 🔧 Pinout definitivo STM32G474RE | [docs/PINOUT.md](docs/PINOUT.md) |
| **MOTOR_CONTROL.md** | ⚙️ Control de motores y PWM | [docs/MOTOR_CONTROL.md](docs/MOTOR_CONTROL.md) |
| **SAFETY_SYSTEMS.md** | 🛡️ ABS/TCS y seguridad funcional | [docs/SAFETY_SYSTEMS.md](docs/SAFETY_SYSTEMS.md) |
| **BUILD_GUIDE.md** | 🔨 Guía de compilación y deploy | [docs/BUILD_GUIDE.md](docs/BUILD_GUIDE.md) |

### 🔗 Referencias Externas

- **Repo ESP32-S3 HMI:** [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos)
- **STM32G4 Reference Manual:** [RM0440](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- **FDCAN Documentation:** [AN5348](https://www.st.com/resource/en/application_note/an5348-fdcan-protocol-for-stm32g4-series-stmicroelectronics.pdf)

---

## 🔧 Configuración de Pines (Resumen)

```c
// Motores Tracción (TIM1 - PWM @ 20 kHz)
PWM_FL: PA8   DIR_FL: PC0   EN_FL: PC1   // Front Left
PWM_FR: PA9   DIR_FR: PC2   EN_FR: PC3   // Front Right
PWM_RL: PA10  DIR_RL: PC4   EN_RL: PC5   // Rear Left
PWM_RR: PA11  DIR_RR: PC6   EN_RR: PC7   // Rear Right

// Motor Dirección (TIM8 - PWM @ 20 kHz)
PWM_STEER: PC8   DIR_STEER: PC9   EN_STEER: PC10

// Encoder Dirección (TIM2 - Modo Quadrature)
ENC_A: PA15 (TIM2_CH1)   ENC_B: PB3 (TIM2_CH2)   ENC_Z: PB4 (EXTI4)

// Sensores Rueda (4 sensores GPIO + EXTI, el 5to sensor es el encoder de dirección)
WHEEL_FL: PB0   WHEEL_FR: PB1   WHEEL_RL: PB2   WHEEL_RR: PB10

// I²C (INA226 × 6: 4 tracción + 1 dirección + 1 batería, vía TCA9548A)
I2C_SCL: PB6   I2C_SDA: PB7

// CAN Bus (FDCAN1 @ 500 kbps)
CAN_TX: PB9 (FDCAN1_TX, AF9)   CAN_RX: PB8 (FDCAN1_RX, AF9)

// Pedal Analógico (ADC1 con trigger TIM3 @ 200 Hz)
PEDAL: PA0 (ADC1_IN1)

// Shifter F/N/R (GPIO Input, pull-up, activo bajo)
FWD: PB12   NEU: PB13   REV: PB14

// Relés (GPIO Output, default LOW)
RELAY_MAIN: PC11   RELAY_TRAC: PC12   RELAY_DIR: PD2

// Temperatura (OneWire - DS18B20 × 5: 4 motores + 1 ambiente)
TEMP: PB5 (GPIO open-drain, pull-up 4.7kΩ)
```

Ver [docs/PINOUT.md](docs/PINOUT.md) para detalles completos.

---

## 📡 Protocolo CAN

### Especificaciones

- **Velocidad:** 500 kbps
- **Standard:** CAN 2.0A (11-bit IDs)
- **Transceptor:** TJA1051T/3
- **Terminación:** 120Ω en ambos extremos

### Mensajes Principales

| ID (Hex) | Dirección | Contenido | DLC | Frecuencia |
|----------|-----------|-----------|-----|------------|
| **0x001** | STM32→ESP32 | HEARTBEAT_STM32 | 4 | 100 ms |
| **0x011** | ESP32→STM32 | HEARTBEAT_ESP32 | 4 | 100 ms |
| **0x100** | ESP32→STM32 | CMD_THROTTLE (0-100%) | 2 | 50 ms |
| **0x101** | ESP32→STM32 | CMD_STEERING (-100 a +100%) | 2 | 50 ms |
| **0x102** | ESP32→STM32 | CMD_MODE (F/N/R) | 1 | On-demand |
| **0x200** | STM32→ESP32 | STATUS_SPEED (4 ruedas) | 8 | 100 ms |
| **0x201** | STM32→ESP32 | STATUS_CURRENT (motores) | 8 | 100 ms |
| **0x202** | STM32→ESP32 | STATUS_TEMP (sensores) | 8 | 1000 ms |
| **0x203** | STM32→ESP32 | STATUS_SAFETY (ABS/TCS) | 4 | 100 ms |
| **0x204** | STM32→ESP32 | STATUS_STEERING (posición) | 4 | 100 ms |
| **0x300** | Ambos | DIAG_ERROR | 8 | On-demand |

Ver [docs/CAN_PROTOCOL.md](docs/CAN_PROTOCOL.md) para formato detallado de cada mensaje.

---

## 🛡️ Seguridad y Fail-Safe

### Características de Seguridad

1. **Autoridad Final:** El STM32 tiene control absoluto sobre actuadores
2. **Heartbeat Mutuo:** Si ESP32 no responde en >250ms → Modo seguro
3. **Watchdog:** IWDG interno del STM32 para recuperación ante bloqueos
4. **Relés Fail-Safe:** Estado por defecto LOW (sin potencia)
5. **Validación de Comandos:** Todos los comandos CAN son validados antes de ejecutarse
6. **ABS/TCS:** Inhibición automática de tracción ante deslizamiento
7. **Protección Térmica:** Limitación de potencia si temperatura >80°C
8. **Protección de Corriente:** Desconexión si corriente excede umbral seguro

---

## 📊 Estado del Proyecto

- ✅ **Arquitectura:** Definida y documentada
- ✅ **Pinout:** Congelado y validado (5 ruedas, 5 temps, 6 corrientes)
- ✅ **Protocolo CAN:** Especificado completo
- ✅ **Documentación:** Completa (6 documentos)
- ✅ **Firmware base:** Completado (100%) - main.c, motor_control.c, can_handler.c, sensor_manager.c, safety_system.c
- ✅ **Headers:** Completados (6 archivos .h)
- ✅ **Control PWM:** Implementado (directo TIM1/TIM8, NO PCA9685)
- ⏳ **Integración hardware:** Pendiente pruebas físicas
- ⏳ **Calibración sensores:** Pendiente (ROM DS18B20, INA226)

---

## 🤝 Contribuciones

Este proyecto es de desarrollo personal, pero sugerencias y mejoras son bienvenidas:

1. Fork el repositorio
2. Crea una rama (`git checkout -b feature/mejora`)
3. Commit cambios (`git commit -am 'Añadir mejora X'`)
4. Push a la rama (`git push origin feature/mejora`)
5. Abre un Pull Request

---

## 📄 Licencia

Este proyecto está bajo la licencia **MIT**. Ver [LICENSE](LICENSE) para más detalles.

---

## 🔗 Enlaces Relacionados

- **Repositorio ESP32-S3 HMI:** [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos)
- **STM32CubeG4:** [STM32 HAL Documentation](https://www.st.com/en/embedded-software/stm32cubeg4.html)
- **TJA1051 Datasheet:** [NXP](https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf)
- **E6B2-CWZ6C Encoder:** [Omron](https://www.ia.omron.com/products/family/487/)

---

**Desarrollado con ❤️ para control vehicular seguro y determinístico**

*Última actualización: 2026-02-01*