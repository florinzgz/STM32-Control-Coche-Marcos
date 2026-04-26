# SISTEMA DE ALIMENTACIÓN COMPLETO — STM32G474RE + ESP32-S3

> **Documento técnico de diseño del sistema de alimentación.**
> Fecha de creación: 2026-04-04
> Versión: 1.0
> Fuentes: firmware verificado (Core/Src/, esp32/src/), documentación existente (docs/), archivo .ioc

---

## Índice

1. [Arquitectura General del Sistema de Alimentación](#1-arquitectura-general-del-sistema-de-alimentación)
2. [Recorrido Completo de Alimentación (desde baterías hasta actuadores)](#2-recorrido-completo-de-alimentación)
3. [Integración de Relés de Potencia](#3-integración-de-relés-de-potencia)
4. [Apagado Retardado con Audio DFPlayer](#4-apagado-retardado-con-audio-dfplayer)
5. [Alimentación de Luces LED WS2812B](#5-alimentación-de-luces-led-ws2812b)
6. [Protección Eléctrica](#6-protección-eléctrica)
7. [Distribución de Masas — Punto Estrella](#7-distribución-de-masas--punto-estrella)
8. [Esquema Eléctrico Detallado (Textual)](#8-esquema-eléctrico-detallado-textual)
9. [Lista de Materiales (BOM)](#9-lista-de-materiales-bom)

---

## 1. Arquitectura General del Sistema de Alimentación

### 1.1 Descripción del sistema

El vehículo eléctrico de Marcos utiliza una arquitectura de alimentación dual con dos MCUs:

| MCU | Modelo | Reloj | Rol | Alimentación |
|-----|--------|-------|-----|--------------|
| **STM32** | STM32G474RE (Nucleo-64) | 170 MHz | Autoridad de seguridad, control de actuadores | 3.3 V (LDO integrado en Nucleo) |
| **ESP32** | ESP32-S3 DevKitC-1 (N16R8) | 240 MHz dual-core | HMI, audio, display, comunicación | 5 V (USB/regulador) → 3.3 V (LDO integrado) |

Comunicación inter-MCU: **FDCAN / TWAI a 500 kbps** (23 IDs de mensaje).

### 1.2 Niveles de tensión del sistema

```
┌─────────────────────────────────────────────────────────────────┐
│                    NIVELES DE TENSIÓN                           │
├─────────┬──────────────────┬────────────────────────────────────┤
│ Nivel   │ Fuente           │ Consumidores                      │
├─────────┼──────────────────┼────────────────────────────────────┤
│ 24 V    │ Batería tracción │ 4× motores DC (vía BTS7960)       │
│ 12 V    │ Batería/regulador│ Motor dirección (vía BTS7960),    │
│         │                  │ llave de contacto, relé retención │
│ 5 V     │ DC-DC LM2596     │ Lógica BTS7960 VCC, LED WS2812B, │
│         │                  │ transceiver CAN, DFPlayer Mini    │
│ 3.3 V   │ LDO (Nucleo/ESP) │ STM32G474RE, ESP32-S3, TCA9548A, │
│         │                  │ INA226, sensores lógicos           │
└─────────┴──────────────────┴────────────────────────────────────┘
```

### 1.3 Diagrama de bloques

```
 BAT 24V ──►[Fusible 60A]──►[RELAY_MAIN PC10]──►┬──►[RELAY_TRAC PC11]──►[Fusible 50A]──► 4× BTS7960 tracción
             │                                    │
             │                                    └──►[INA226 ch4 (0.75mΩ)]──► Monitoreo batería
             │
             └──►[DC-DC 24V→5V LM2596]──► 5V lógica ──►┬── BTS7960 VCC (×5)
                                                         ├── WS2812B (vía relés PB10/PB11)
                                                         ├── Transceiver CAN SN65HVD230 (×2)
                                                         ├── DFPlayer Mini
                                                         └── ESP32-S3 (5V → 3.3V LDO)

 BAT 12V ──►[Fusible 20A]──►[RELAY_DIR PC12]──►[BTS7960 dirección]──► Motor dirección
             │
             └──►[Llave contacto]──► GPIO 40 (ESP32) + Relé retención (GPIO 41)

 3.3V ─────► STM32G474RE (Nucleo LDO)
             ├── TCA9548A (I2C mux)
             ├── 6× INA226 (vía TCA9548A)
             ├── Sensores inductivos (vía optoacopladores)
             └── Encoder E6B2-CWZ6C (vía 3× 6N137)
```

---

## 2. Recorrido Completo de Alimentación

### 2.1 Circuito de tracción (24 V)

```
BAT 24V (+)
   │
   ├──► Fusible maxi 60 A (slow-blow)
   │       │
   │       ▼
   │    RELAY_MAIN (PC10, GPIOC)  ←── STM32 safety_system.c
   │       │
   │       ├──► INA226 canal 4 (shunt 0.75 mΩ, 100 A) ──► Lectura tensión/corriente batería
   │       │
   │       ▼
   │    RELAY_TRAC (PC11, GPIOC)  ←── STM32 safety_system.c
   │       │
   │       ├──► Fusible 30 A ──► BTS7960 FL (TIM1_CH1 PA8 / TIM1_CH2 PA9)  ──► Motor FL
   │       ├──► Fusible 30 A ──► BTS7960 FR (TIM1_CH3 PA10 / TIM1_CH4 PC3)  ──► Motor FR
   │       ├──► Fusible 30 A ──► BTS7960 RL (TIM8_CH1 PC6 / TIM8_CH2 PC7)  ──► Motor RL
   │       └──► Fusible 30 A ──► BTS7960 RR (TIM8_CH3 PC8 / TIM8_CH4 PC9)  ──► Motor RR
   │
BAT 24V (−) ──► PUNTO ESTRELLA (masa central)
```

**Características del PWM de motores:**
- Frecuencia: **20 kHz** (inaudible, evita silbidos)
- Período (ARR): **4249** (center-aligned)
- Prescaler: **0** (PCLK1 170 MHz directo)
- Duty cycle: `CCR / 4250 × 100%`
- Protección BREAK2 vinculada a Cortex-M4 LOCKUP (TIM1)

### 2.2 Circuito de dirección (12 V)

```
BAT 12V (+) / Regulador 12V
   │
   ├──► Fusible blade 15 A
   │       │
   │       ▼
   │    RELAY_DIR (PC12, GPIOC)  ←── STM32 safety_system.c
   │       │
   │       ▼
   │    BTS7960 STEER (TIM3_CH1 PA6 / TIM3_CH2 PA7)
   │       │
   │       ▼
   │    Motor dirección DC 12V (con PID + encoder E6B2-CWZ6C)
   │
BAT 12V (−) ──► PUNTO ESTRELLA
```

### 2.3 Circuito lógico (5 V / 3.3 V)

```
24V ──► DC-DC LM2596 (24V→5V, ≥5A)
           │
           ├──► Fusible blade 5 A
           │       │
           │       ├──► BTS7960 VCC lógica (×5 módulos, ~50 mA c/u)
           │       ├──► SN65HVD230 transceiver CAN (×2, ~70 mA c/u)
           │       ├──► DFPlayer Mini (~200 mA con altavoz activo)
           │       ├──► LED WS2812B frontal 28 LEDs (vía RELAY_LED PB10, máx ~1.7 A)
           │       ├──► LED WS2812B trasero 16 LEDs (vía RELAY_LED_REAR PB11, máx ~1 A)
           │       │
           │       └──► ESP32-S3 DevKitC (5V pin) ──► LDO 3.3V integrado
           │               ├── CPU + WiFi/BT (~250 mA)
           │               ├── Display TFT ST7796 480×320 (~80 mA)
           │               └── Periféricos GPIO
           │
           └──► Nucleo-64 (5V pin) ──► LDO 3.3V integrado (AMS1117)
                   ├── STM32G474RE (~100 mA)
                   ├── TCA9548A I2C mux (0x70)
                   ├── 6× INA226 sensores corriente (0x40 vía mux)
                   └── Sensores varios (vía optoacopladores)
```

### 2.4 Secuencia temporal de encendido

| Tiempo | Componente | Acción | Fichero fuente |
|--------|-----------|--------|----------------|
| t=0 ms | **Llave contacto** | Girada a ON → 12V al relé retención | — |
| t=0 ms | **Relé retención** | Cierra contacto → 12V al regulador 5V | hardware |
| t~50 ms | **ESP32-S3** | Arranca (boot + setup ~510 ms total) | esp32/src/main.cpp |
| t~50 ms | **STM32** | Arranca (HAL_Init + SystemClock_Config) | Core/Src/main.c |
| t~100 ms | **ESP32** | `power_mgr::init()` detecta GPIO 40 HIGH | esp32/src/power_manager.cpp |
| t~100 ms | **ESP32** | GPIO 41 → HIGH (POWER_HOLD activo) | esp32/src/power_manager.cpp |
| t~100 ms | **ESP32** | Estado → POWER_HOLD → STARTING | esp32/src/power_manager.cpp |
| t~300 ms | **ESP32** | Estado → RUNNING (tras STARTUP_DELAY_MS=200 ms) | esp32/src/power_manager.cpp |
| t~300 ms | **ESP32** | Reproducción audio WELCOME (Track 1) | esp32/src/audio_manager.cpp |
| t~560 ms | **ESP32** | setup() completo → heartbeat CAN 0x011 cada 100 ms | esp32/src/main.cpp |
| continuo | **STM32** | Recibe heartbeat → BOOT → STANDBY → ACTIVE | Core/Src/safety_system.c |
| continuo | **STM32** | `Relay_PowerUp()` inicia secuencia de relés | Core/Src/safety_system.c |

### 2.5 Secuencia temporal de apagado

| Tiempo | Componente | Acción | Fichero fuente |
|--------|-----------|--------|----------------|
| t=0 ms | **Llave contacto** | Girada a OFF → 12V cortado en llave | — |
| t~50 ms | **ESP32** | GPIO 40 lee LOW (debounce 50 ms) → SHUTTING_DOWN | esp32/src/power_manager.cpp |
| t~50 ms | **ESP32** | `config_store::flush()` → guarda configuración | esp32/src/power_manager.cpp |
| t~50 ms | **ESP32** | Reproducción audio FAREWELL (Track 2) | esp32/src/audio_manager.cpp |
| t~3050 ms | **ESP32** | GPIO 41 → LOW (POWER_HOLD liberado) | esp32/src/power_manager.cpp |
| t~3100 ms | **Relé retención** | Se abre → sin alimentación al sistema | hardware |
| t~3100 ms | **STM32** | Pierde alimentación → relés PC10/PC11/PC12 → LOW | Core/Src/safety_system.c |

---

## 3. Integración de Relés de Potencia

### 3.1 Mapa de relés del sistema

| Relé | Pin STM32 | Puerto | Función | Fusible | Corriente máx |
|------|-----------|--------|---------|---------|---------------|
| **RELAY_MAIN** | PC10 | GPIOC | Alimentación general 24V | 60 A maxi | 100 A (batería) |
| **RELAY_TRAC** | PC11 | GPIOC | Motores tracción 24V | 50 A (4×30 A indiv.) | 4×25 A = 100 A |
| **RELAY_DIR** | PC12 | GPIOC | Motor dirección 12V | 15 A blade | 25 A |
| **RELAY_LED** | PB10 | GPIOB | Tira WS2812B frontal 5V | — | ~1.7 A |
| **RELAY_LED_REAR** | PB11 | GPIOB | Tira WS2812B trasera 5V | — | ~1 A |

### 3.2 Secuencia de encendido de relés (no bloqueante)

La función `Relay_PowerUp()` en `Core/Src/safety_system.c` implementa una máquina de estados no bloqueante:

```
t=0 ms:        RELAY_MAIN ON  (PC10 → HIGH)
               HAL_GPIO_WritePin(GPIOC, PIN_RELAY_MAIN, GPIO_PIN_SET)

t=0..50 ms:    Espera RELAY_MAIN_SETTLE_MS = 50 ms  (asentamiento inrush)

t=50 ms:       RELAY_TRAC ON  (PC11 → HIGH)
               HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_SET)

t=50..70 ms:   Espera RELAY_TRACTION_SETTLE_MS = 20 ms  (supresión de arco)

t=70 ms:       RELAY_DIR ON  (PC12 → HIGH)
               HAL_GPIO_WritePin(GPIOC, PIN_RELAY_DIR, GPIO_PIN_SET)

               ✅ Todos los relés activos → motores alimentados
```

**Constantes de temporización (safety_system.c):**
```c
#define RELAY_MAIN_SETTLE_MS      50   /* Asentamiento corriente inrush 24V */
#define RELAY_TRACTION_SETTLE_MS  20   /* Supresión arco en contactos */
```

### 3.3 Secuencia de apagado de relés (inmediata, orden inverso)

```c
void Relay_PowerDown(void) {
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_DIR,  GPIO_PIN_RESET);  // Primero dirección
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_RESET);  // Segundo tracción
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_MAIN, GPIO_PIN_RESET);  // Último principal
}
```

El apagado es **inmediato y en orden inverso** (LIFO) para garantizar que los motores se desconectan antes de cortar la alimentación general.

### 3.4 Circuito de activación de relés

Cada relé de potencia (MAIN, TRAC, DIR) se controla mediante una **arquitectura de dos etapas**:

```
STM32 GPIO (3.3V) ──► Módulo 4-ch opto relé (SRD-12VDC-SL-C, 12V)
                         │
                         ├── Optoacoplador + driver integrado en módulo
                         └── Contacto (10A) cierra circuito 12V

                      ──► Bobina relé de potencia (12V DC)
                         │
                         ├── Contacto de alta corriente (50A/40A/15A)
                         ├── Diodo flyback 1N4007 (en bobina relé potencia)
                         └── Snubber RC en contactos (100Ω + 100nF/250V)
```

### 3.5 Control de relés LED por CAN

Los relés LED (PB10, PB11) se controlan mediante mensaje **CAN ID 0x120** enviado por el ESP32:

```
CAN ID: 0x120 (ESP32 → STM32)
├── Byte 0: Relé frontal  (0x01=ON, 0x00=OFF) → PB10
└── Byte 1: Relé trasero  (0x01=ON, 0x00=OFF) → PB11
```

**Funciones en can_handler.c:**
```c
void LED_Relay_Set(bool on) {
    HAL_GPIO_WritePin(GPIOB, PIN_RELAY_LED, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LED_Relay_Rear_Set(bool on) {
    HAL_GPIO_WritePin(GPIOB, PIN_RELAY_LED_REAR, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
```

---

## 4. Apagado Retardado con Audio DFPlayer

### 4.1 Principio de funcionamiento

Cuando el usuario gira la llave a OFF, el sistema necesita **3 segundos adicionales** de alimentación para:
1. Guardar la configuración persistente (`config_store::flush()`)
2. Reproducir el audio de despedida (Track 2: "Cerrando sistemas. Hasta pronto.")
3. Apagar los relés de potencia de forma ordenada

### 4.2 Circuito del relé de retención

```
                    ┌─────────────────────────────────┐
                    │      RELÉ RETENCIÓN              │
                    │    (SRD-05VDC o similar)          │
                    │                                   │
 Llave 12V ──►D1───┤── Bobina (+)                      │
                    │      │                            │
 ESP32 GPIO 41 ──►  │      │                            │
   │                │      │                            │
   └── R=1kΩ ──►    │      │                            │
       Base BC547 ──┤      │                            │
       Emisor GND   │      ▼                            │
                    │   D_flyback (1N4007)               │
                    │      │                            │
                    │   Bobina (−) ──► GND               │
                    │                                   │
                    │   COM ──► BAT 12V (+)              │
                    │   NO  ──► Regulador 5V ──►        │
                    │           ESP32 + STM32            │
                    │   NC  ──► (no conectado)           │
                    └─────────────────────────────────┘

D1, D2 = 1N4148 (diodo OR: cualquiera de las dos fuentes activa el relé)
```

**Funcionamiento:**
1. **Encendido:** La llave de contacto (12V) activa el relé directamente a través de D1
2. **Retención:** ESP32 detecta la llave (GPIO 40 HIGH) y activa GPIO 41 → transistor NPN → bobina
3. **Apagado:** Llave OFF → GPIO 40 LOW → ESP32 mantiene GPIO 41 HIGH durante 3000 ms
4. **Corte:** GPIO 41 → LOW → relé se abre → sistema se apaga completamente

### 4.3 Divisor de tensión para GPIO 40 (detección de llave)

```
Llave 12V ──► R1=33kΩ ──┬──► GPIO 40 (ESP32)
                          │
                          R2=10kΩ
                          │
                          GND

Vtensión = 12V × 10k/(33k+10k) = 2.79 V  ✓ (seguro para ESP32, umbral HIGH ~2.0V)
Corriente = 12V / 43kΩ ≈ 0.28 mA  (consumo despreciable)
```

### 4.4 Máquina de estados del Power Manager (ESP32)

```
                    ┌──────────┐
         Arranque → │   INIT   │
                    └────┬─────┘
                         │ GPIO 40 = HIGH
                         ▼
                    ┌──────────┐
                    │POWER_HOLD│ ← GPIO 41 → HIGH
                    └────┬─────┘
                         │ +50ms
                         ▼
                    ┌──────────┐
                    │ STARTING │
                    └────┬─────┘
                         │ +200ms (STARTUP_DELAY_MS)
                         ▼
                    ┌──────────┐
                    │ RUNNING  │ ← Estado normal operación
                    └────┬─────┘
                         │ GPIO 40 = LOW (debounce 50ms)
                         ▼
                    ┌────────────────┐
                    │ SHUTTING_DOWN  │ ← config_store::flush()
                    │                │ ← Audio FAREWELL (Track 2)
                    └────┬───────────┘
                         │ +3000ms (SHUTDOWN_DELAY_MS)
                         ▼
                    ┌──────────┐
                    │  POWER   │ ← GPIO 41 → LOW
                    │   OFF    │ ← Relé se abre → sin alimentación
                    └──────────┘
```

**Constantes (power_manager.h):**
```c
#define DEBOUNCE_MS          50    /* Debounce lectura llave */
#define STARTUP_DELAY_MS    200    /* Retardo antes de RUNNING */
#define SHUTDOWN_DELAY_MS  3000    /* Tiempo para audio + flush */
```

### 4.5 Configuración del DFPlayer Mini

| Parámetro | Valor | Pin ESP32 |
|-----------|-------|-----------|
| UART TX | 9600 bps | GPIO 43 (PIN_DFPLAYER_TX) |
| UART RX | 9600 bps | GPIO 44 (PIN_DFPLAYER_RX) |
| Alimentación | 5V (desde regulador) | — |
| Altavoz | 4–8 Ω, ≥2W | Salida SPK+/SPK− del DFPlayer |

**Relé de audio (GPIO 11):**
| Constante | Valor | Función |
|-----------|-------|---------|
| PIN_AUDIO_RELAY | GPIO 11 | Control ON/OFF amplificador (activo LOW) |
| RELAY_ESTABLISH_MS | 20 ms | Tiempo para que cierre el contacto del relé |
| RELAY_RELEASE_MS | 150 ms | Enfriamiento tras audio |
| RELAY_MAX_ON_MS | 7000 ms | Watchdog de seguridad |

**Tracks de audio relevantes para encendido/apagado:**

| Track | Sonido | Prioridad |
|-------|--------|-----------|
| 1 | WELCOME: "Bienvenido Marcos. El sistema está listo." | HI (2) |
| 2 | FAREWELL: "Cerrando sistemas. Hasta pronto." | HI (2) |
| 31 | EMERGENCY: "Modo de emergencia activado." | HI (2) |

---

## 5. Alimentación de Luces LED WS2812B

### 5.1 Arquitectura de alimentación LED

Las tiras LED WS2812B se alimentan desde la fuente de 5V pero a través de **relés dedicados** controlados por el STM32. El ESP32 genera la señal de datos (protocolo WS2812B de 800 kHz).

```
Fuente 5V (≥3A frontal, ≥2A trasera)
   │
   ├──► RELAY_LED (PB10) ──► Contacto NO ──► Tira WS2812B frontal (28 LEDs)
   │                                             │
   │                                             └── Señal datos ← ESP32 GPIO
   │
   └──► RELAY_LED_REAR (PB11) ──► Contacto NO ──► Tira WS2812B trasera (16 LEDs)
                                                      │
                                                      └── Señal datos ← ESP32 GPIO
```

### 5.2 Consumo eléctrico de las tiras LED

| Tira | LEDs | Consumo máx (blanco 100%) | Consumo típico | Fuente requerida |
|------|------|---------------------------|----------------|------------------|
| Frontal | 28 | 28 × 60 mA = 1.68 A | ~0.5 A (colores) | ≥3 A |
| Trasera | 16 | 16 × 60 mA = 0.96 A | ~0.3 A (colores) | ≥2 A |
| **Total** | **44** | **2.64 A** | **~0.8 A** | **≥5 A** |

### 5.3 Protección del circuito LED

```
Fuente 5V ──► [1000 µF / 10V electrolítico] ──► Relé NO ──► Tira WS2812B VCC
                                                     │
ESP32 GPIO ──► [R=330 Ω serie] ──────────────────────┴──► Tira WS2812B DIN

Protecciones en el relé:
├── Diodo flyback 1N4007 (antiparalelo con bobina)
├── Snubber RC: 100 Ω + 100 nF / 250V (paralelo con contactos)
└── Módulo con optoacoplador (aislamiento 3.3V ↔ 5V bobina)
```

**Resistencia serie de 330 Ω** en la línea de datos WS2812B: protege contra reflexiones y limita corriente de pico en el primer LED.

### 5.4 Control por CAN

El ESP32 envía el comando CAN ID **0x120** al STM32 para encender/apagar las tiras:

```
Encendido sistema → ESP32 envía 0x120 [0x01, 0x01] → STM32 activa PB10 + PB11
Apagado sistema  → ESP32 envía 0x120 [0x00, 0x00] → STM32 desactiva PB10 + PB11
```

Cuando el STM32 pierde alimentación (apagado), los pines PB10/PB11 quedan en **LOW** → relés abiertos → LEDs desconectados (fail-safe).

---

## 6. Protección Eléctrica

### 6.1 Fusibles del sistema

| Fusible | Tipo | Corriente | Ubicación | Consecuencia al fundirse |
|---------|------|-----------|-----------|--------------------------|
| **F1** | Maxi slow-blow | 60 A | BAT 24V → RELAY_MAIN | Sistema completo sin tracción; INA226 ch4 lee 0A |
| **F2** | Midi slow-blow | 30 A ×4 | RELAY_TRAC → cada BTS7960 | Motor individual desconectado |
| **F3** | Blade slow-blow | 15 A | BAT 12V → RELAY_DIR | Dirección sin alimentación → estado SAFE |
| **F4** | Blade slow-blow | 5 A | 5V → lógica | Sistema lógico sin alimentación |

### 6.2 Protección de relés

Cada relé incluye:

| Componente | Valor | Función | Ubicación |
|-----------|-------|---------|-----------|
| Diodo flyback | 1N4007 (1A/1000V) | Absorbe pico inductivo al abrir bobina | Antiparalelo con bobina |
| Snubber RC | 100 Ω + 100 nF/250V ac | Supresión de arco en contactos | Paralelo con contactos NO |
| Optoacoplador | Módulo 4-ch SRD-12VDC-SL-C | Aislamiento galvánico 3.3V ↔ 12V (etapa intermedia) | Módulo de relé intermedio |

### 6.3 Protección de motores

| Componente | Valor | Función | Ubicación |
|-----------|-------|---------|-----------|
| Condensador snubber | 100 nF / 50V X7R | Anti-EMI en terminales motor | Cada terminal M+/M− |
| Condensador bulk | 470 µF / 35V electrolítico | Absorción inrush en alimentación BTS7960 | B+ a GND del BTS7960 |
| Condensador bypass | 100 nF / 50V X7R | Desacoplo rápido alimentación BTS7960 | VCC a GND del BTS7960 |
| Diodo freewheeling | SB560 (5A/60V Schottky) | Protección contra CEMF regenerativa | Opcional, por motor |

### 6.4 Protección del bus CAN

| Componente | Valor | Función | Ubicación |
|-----------|-------|---------|-----------|
| Resistencia terminación | 120 Ω ×2 | Terminación diferencial (impedancia bus) | Cada extremo del bus |
| TVS diferencial | PESD2CAN | Protección ESD + sobretensión | Junto a cada transceiver |
| Condensador VCC | 100 nF / 10V X7R | Desacoplo alimentación transceiver | Pin VCC de SN65HVD230 |
| Condensador bulk | 10 µF / 10V electrolítico | Filtrado baja frecuencia | Pin VCC, lado ESP32 |

### 6.5 Protección de entradas del STM32

| Entrada | Sensor | Peligro | Solución |
|---------|--------|---------|----------|
| PA15, PB3, PB4 | Encoder E6B2-CWZ6C (5V) | 5V destruye pin 3.3V + picos inductivos motor | 3× 6N137 (R_IN 330Ω, pull-up 4.7kΩ a 3.3V) — aislamiento galvánico 2500 V |
| PA3 | Pedal Hall SS1324 (5V) | Sobretensión ADC | Divisor: 10kΩ + 6.8kΩ → máx 2.0V |
| PA0/PA1/PA2/PB15 | LJ12A3 inductivos (6–36V) | Pico inductivo | Optoacoplador PC817 obligatorio |
| PA11/PA12 | Bus CAN | Sobretensión diferencial | Solo conectar al SN65HVD230 (nunca directo) |

### 6.6 Red de desacoplo para alimentación lógica

```
3.3V (Nucleo) ──┬──[100 nF X7R cerámico]──┬── GND   (respuesta <1 ns)
                 │                          │
                 └──[10 µF electrolítico]───┘         (bulk 1–100 µs)

5V (lógica) ────┬──[100 nF X7R cerámico]──┬── GND   (por cada BTS7960)
                 │                          │
                 └──[47 µF electrolítico]───┘         (punto entrada fuente 5V)
```

### 6.7 Filtrado para alimentación de transceiver CAN

```
5V_sucio ──[Ferrita BLM18AG601SN1D o 100µH]── 5V_limpio ──► SN65HVD230 VCC
                                                           └── [10µF] a GND
```

### 6.8 Umbrales de tensión de batería

El sistema de seguridad (`safety_system.c`) monitoriza la tensión de batería 24V mediante INA226 canal 4:

| Estado | Condición | Acción firmware |
|--------|-----------|-----------------|
| **NORMAL** | V > 20.0 V | Operación completa (ACTIVE) |
| **DEGRADED** | V ≤ 20.0 V | Potencia reducida (DEGRADED_L1) |
| **SAFE** | V < 18.0 V | Actuadores OFF, todos relés abiertos |
| Recovery → NORMAL | V > 20.0 V | Vuelta a ACTIVE |
| Recovery → DEGRADED | V > 18.5 V | Salida de SAFE (histéresis 0.5V) |

```c
#define BATTERY_UV_WARNING_V   20.0f   /* → DEGRADED */
#define BATTERY_UV_CRITICAL_V  18.0f   /* → SAFE */
#define BATTERY_UV_HYST_V       0.5f   /* Histéresis para evitar oscilación */
```

### 6.9 Verificación con multímetro antes del primer encendido

| Punto de medida | Valor esperado | Si falla |
|----------------|----------------|----------|
| 3.3V (Nucleo) | 3.25–3.35 V | No encender hasta corregir |
| 5V (lógica) | 4.85–5.15 V | Ajustar fuente |
| PA15 encoder (máx) | ≤3.3 V | Revisar 6N137 y pull-up 4.7kΩ |
| PA3 pedal (máx) | ≤2.1 V | Revisar divisor 10kΩ+6.8kΩ |
| GND–GND (STM32 vs BTS7960) | <0.1 V | Si >0.3V revisar cableado masa |

---

## 7. Distribución de Masas — Punto Estrella

### 7.1 Principio

Todas las masas (GND) del sistema convergen en un **único punto central** llamado "punto estrella". Esta topología previene:
- **Bucles de masa** que generan ruido electromagnético
- **Diferenciales de potencial** entre masas que corrompen señales
- **Interferencia cruzada** entre circuitos de potencia y lógica

### 7.2 Diagrama del punto estrella

```
                         ★ PUNTO ESTRELLA
                         │ (conexión central única)
                         │
        ┌────────────────┼────────────────────────────────┐
        │                │                                │
        ▼                ▼                                ▼
   GND BAT 24V      GND BAT 12V                    GND Lógica
   (cable 4mm²)     (cable 4mm²)                   (cable 1.5mm²)
        │                │                                │
        │                │                    ┌───────────┼───────────┐
        │                │                    │           │           │
        ▼                ▼                    ▼           ▼           ▼
   4× BTS7960       BTS7960 STEER        STM32       ESP32-S3    TCA9548A
   tracción GND      GND                 Nucleo GND   GND         INA226 GND
        │                                    │
        │                                    ▼
        │                               SN65HVD230
        │                               CAN GND *
        │
        └──► Bus CAN GND (unido al punto estrella en UN SOLO lugar)
```

### 7.3 Reglas de cableado de masa

| Regla | Descripción | Justificación |
|-------|-------------|---------------|
| **R1** | Cable directo al punto estrella, NO en cadena (daisy-chain) | Previene caída de tensión acumulada |
| **R2** | Condensadores de desacoplo lo más cerca posible del IC | Reduce inductancia del lazo de retorno |
| **R3** | Bus CAN con masa propia unida al punto estrella en un solo punto | Aislamiento de ruido de potencia |
| **R4** | Cables de retorno de batería van directos al punto estrella | Evita que corriente de motor pase por masa lógica |
| **R5** | Verificar diferencial GND–GND < 0.1V entre módulos | Indicador de conexión correcta |

### 7.4 Secciones de cable para masas

| Tramo | Sección mínima | Justificación |
|-------|----------------|---------------|
| BAT 24V GND → punto estrella | 4 mm² (12 AWG) | Corriente ≤100 A |
| BAT 12V GND → punto estrella | 4 mm² (12 AWG) | Corriente de arranque |
| BTS7960 GND → punto estrella | 2.5 mm² (14 AWG) | Corriente por motor ≤25 A |
| STM32/ESP32 GND → punto estrella | 1.5 mm² (16 AWG) | Corriente lógica <1 A |
| CAN GND → punto estrella | 0.5 mm² (22 AWG) | Solo referencia, <100 mA |

---

## 8. Esquema Eléctrico Detallado (Textual)

### 8.1 Asignación completa de pines STM32G474RE

| # | Pin | Puerto | Función | Componente | Periférico | Tensión |
|---|-----|--------|---------|-----------|------------|---------|
| 1 | PA0 | GPIOA | Wheel speed FL | LJ12A3 | EXTI0 | 3.3V (vía PC817) |
| 2 | PA1 | GPIOA | Wheel speed FR | LJ12A3 | EXTI1 | 3.3V (vía PC817) |
| 3 | PA2 | GPIOA | Wheel speed RL | LJ12A3 | EXTI2 | 3.3V (vía PC817) |
| 4 | PA3 | GPIOA | Pedal acelerador | Hall SS1324 | ADC1_IN4 | 0–2.0V (divisor) |
| 5 | PA6 | GPIOA | RPWM STEER | BTS7960 | TIM3_CH1 | 3.3V PWM |
| 6 | PA7 | GPIOA | LPWM STEER | BTS7960 | TIM3_CH2 | 3.3V PWM |
| 7 | PA8 | GPIOA | RPWM FL | BTS7960 FL | TIM1_CH1 | 3.3V PWM |
| 8 | PA9 | GPIOA | LPWM FL | BTS7960 FL | TIM1_CH2 | 3.3V PWM |
| 9 | PA10 | GPIOA | RPWM FR | BTS7960 FR | TIM1_CH3 | 3.3V PWM |
| 10 | PA11 | GPIOA | FDCAN1_RX | SN65HVD230 | FDCAN1 AF9 | 3.3V lógico |
| 11 | PA12 | GPIOA | FDCAN1_TX | SN65HVD230 | FDCAN1 AF9 | 3.3V lógico |
| 12 | PA15 | GPIOA | Encoder A | E6B2-CWZ6C | TIM2_CH1 | 3.3V (vía 6N137) |
| 13 | PB0 | GPIOB | OneWire | DS18B20 ×5 | GPIO bit-bang | 3.3V |
| 14 | PB3 | GPIOB | Encoder B | E6B2-CWZ6C | TIM2_CH2 | 3.3V (vía 6N137) |
| 15 | PB4 | GPIOB | Encoder Z | E6B2-CWZ6C | EXTI4 | 3.3V (vía 6N137) |
| 16 | PB5 | GPIOB | Centro dirección | LJ12A3 | EXTI5 | 3.3V (vía PC817) |
| 17 | PB6 | GPIOB | I2C SCL | TCA9548A | I2C1_SCL | 3.3V open-drain |
| 18 | PB7 | GPIOB | I2C SDA | TCA9548A | I2C1_SDA | 3.3V open-drain |
| 19 | PB10 | GPIOB | Relay LED frontal | WS2812B | GPIO Output | 3.3V (driver) |
| 20 | PB11 | GPIOB | Relay LED trasero | WS2812B | GPIO Output | 3.3V (driver) |
| 21 | PC3 | GPIOC | LPWM FR | BTS7960 FR | TIM1_CH4 | 3.3V PWM |
| 22 | PB15 | GPIOB | Wheel speed RR | LJ12A3 | EXTI15 | 3.3V (vía PC817) |
| 23 | PC5 | GPIOC | Enable FL | BTS7960 FL | GPIO Output | 3.3V digital |
| 24 | PC6 | GPIOC | RPWM RL | BTS7960 RL | TIM8_CH1 | 3.3V PWM |
| 25 | PC7 | GPIOC | LPWM RL | BTS7960 RL | TIM8_CH2 | 3.3V PWM |
| 26 | PC8 | GPIOC | RPWM RR | BTS7960 RR | TIM8_CH3 | 3.3V PWM |
| 27 | PC9 | GPIOC | LPWM RR | BTS7960 RR | TIM8_CH4 | 3.3V PWM |
| 28 | PC10 | GPIOC | RELAY_MAIN | Relé potencia | GPIO Output | 3.3V (driver) |
| 29 | PC11 | GPIOC | RELAY_TRAC | Relé potencia | GPIO Output | 3.3V (driver) |
| 30 | PC12 | GPIOC | RELAY_DIR | Relé potencia | GPIO Output | 3.3V (driver) |
| 31 | PC13 | GPIOC | Enable RR | BTS7960 RR | GPIO Output | 3.3V digital |

**Total pines STM32 en uso: 31**

### 8.2 Asignación de pines ESP32-S3 (relevantes a alimentación)

| Pin | GPIO | Función | Componente |
|-----|------|---------|-----------|
| GPIO 40 | Input | Detección llave contacto | Divisor 33kΩ+10kΩ |
| GPIO 41 | Output | POWER_HOLD (retención) | Base BC547 + 1kΩ |
| GPIO 43 | Output | UART TX DFPlayer | 9600 bps |
| GPIO 44 | Input | UART RX DFPlayer | 9600 bps |
| GPIO 11 | Output | Relé audio (activo LOW) | Módulo relé + optoacoplador |

### 8.3 Bus I2C — Sensores de corriente

```
STM32 PB6 (SCL) ──┬──[4.7kΩ pull-up]──► 3.3V
                    │
STM32 PB7 (SDA) ──┬──[4.7kΩ pull-up]──► 3.3V
                    │
                    ▼
               TCA9548A (0x70) ── 400 kHz
                    │
        ┌───────────┼───────────────────────────────────┐
        │           │           │           │           │           │
       CH0         CH1         CH2         CH3         CH4         CH5
        │           │           │           │           │           │
     INA226      INA226      INA226      INA226      INA226      INA226
     (0x40)      (0x40)      (0x40)      (0x40)      (0x40)      (0x40)
        │           │           │           │           │           │
     Motor FL    Motor FR    Motor RL    Motor RR   BAT 24V    Motor STEER
     1.5 mΩ      1.5 mΩ      1.5 mΩ      1.5 mΩ    0.75 mΩ    1.5 mΩ
     50 A         50 A         50 A         50 A      100 A       50 A
```

> **Nota crítica (Canal 4):** El sensor de batería está ubicado **ANTES** del RELAY_MAIN, lo que permite leer tensión/corriente incluso con el relé abierto. Esto es esencial para validación de arranque y operación LIMP_HOME.

### 8.4 Configuración FDCAN (500 kbps)

| Parámetro | Valor | Derivación |
|-----------|-------|------------|
| SYSCLK | 170 MHz | HSI 16 MHz → PLL /4×85 = 340 MHz VCO → /2 |
| PCLK1 | 170 MHz | Sin prescaler |
| Fuente reloj FDCAN | PCLK1 | Directo |
| Prescaler nominal | 10 | TQ = 170M/10 = 17 MHz |
| Sync Jump Width | 4 TQ | ±11.8% tolerancia oscilador |
| Time Segment 1 | 29 TQ | — |
| Time Segment 2 | 4 TQ | — |
| **Bit Time** | **34 TQ** | 1+29+4 |
| **Sample Point** | **88.2%** | (1+29)/34 (CiA recomienda 87.5%) |
| **Baud Rate** | **500 kbps** | 17 MHz / 34 |
| Auto Retransmission | ENABLE | Reenvío automático en error |
| Transmit Pause | ENABLE | ≥2 TQ idle entre tramas |
| Transceiver | SN65HVD230 | 3.3V, Pin 8 (Rs) a GND |

### 8.5 Máquina de estados del sistema de seguridad STM32

```
BOOT ──► STANDBY ──► ACTIVE ⇄ DEGRADED ──► SAFE ──► ERROR
              ↘ LIMP_HOME ↗
```

| Estado | Descripción | Relés | Trigger |
|--------|-------------|-------|---------|
| **BOOT** | Periféricos inicializando | Todos OFF | Power-on |
| **STANDBY** | Listo, esperando CAN | Todos OFF | Boot validation OK |
| **ACTIVE** | Operación normal | Todos ON | Heartbeat ESP32 recibido |
| **DEGRADED** | Potencia reducida | Todos ON | Fallo no crítico (V<20V) |
| **SAFE** | Peligro, actuadores OFF | Todos OFF | Sobrecorriente, V<18V |
| **LIMP_HOME** | Velocidad mínima, pedal local | MAIN+TRAC ON | Pérdida CAN (timeout ESP32) |
| **ERROR** | Irrecuperable | Todos OFF | Watchdog, parada emergencia |

---

## 9. Lista de Materiales (BOM)

### 9.1 Microcontroladores y HMI

| Qty | Componente | Referencia | Función |
|-----|-----------|------------|---------|
| 1 | STM32G474RE Nucleo-64 | NUCLEO-G474RE | MCU seguridad + actuadores |
| 1 | ESP32-S3 DevKitC-1 | N16R8 (16MB Flash + 8MB PSRAM) | HMI + comunicación |
| 1 | Display TFT | ST7796 480×320 + XPT2046 touch | Pantalla + táctil |
| 1 | DFPlayer Mini | — | Reproductor audio MP3 |
| 1 | Altavoz | 4–8 Ω, ≥2W | Salida audio |
| 1 | Tarjeta microSD | ≥1GB, class 10 | Almacenamiento audio (68 tracks) |

### 9.2 Drivers de motor

| Qty | Componente | Referencia | Función |
|-----|-----------|------------|---------|
| 5 | Módulo BTS7960 (IBT-2) | — | 4× tracción + 1× dirección |

### 9.3 Motores

| Qty | Componente | Tensión | Función |
|-----|-----------|---------|---------|
| 4 | Motor DC brushed 24V | 24V, 5–25A | Tracción (FL, FR, RL, RR) |
| 1 | Motor DC brushed 12V | 12V | Dirección |

### 9.4 Sensores de corriente y tensión

| Qty | Componente | Referencia | Función |
|-----|-----------|------------|---------|
| 6 | Módulo INA226 (breakout) | Dirección 0x40 | Medición V+I por motor + batería |
| 1 | Módulo TCA9548A | Dirección 0x70 | Multiplexor I2C 8 canales |

### 9.5 Sensores de velocidad y posición

| Qty | Componente | Referencia | Función |
|-----|-----------|------------|---------|
| 4 | Sensor inductivo | LJ12A3 | Velocidad ruedas (FL, FR, RL, RR) |
| 1 | Encoder rotativo | E6B2-CWZ6C 1200 PPR | Posición dirección |
| 1 | Sensor inductivo | LJ12A3 | Detección centro dirección |
| 1 | Sensor Hall | SS1324LUA-T | Pedal acelerador |
| 5 | Sensor temperatura | DS18B20 (OneWire) | Temperaturas motor/batería |

### 9.6 Comunicación CAN

| Qty | Componente | Referencia | Función |
|-----|-----------|------------|---------|
| 2 | Transceiver CAN | SN65HVD230 (3.3V) | Interfaz física CAN |
| 2 | Resistencia terminación | 120 Ω ¼W | Terminación bus CAN |
| 2 | TVS diferencial | PESD2CAN | Protección ESD bus CAN |

### 9.7 Relés y módulos de relé

| Qty | Componente | Referencia | Función |
|-----|-----------|------------|---------|
| 1 | Módulo 4-ch opto relé | SRD-12VDC-SL-C | Etapa 1: MAIN, TRAC, DIR (CH4 disponible) |
| 3 | Relé de potencia (bobina 12V) | Alta corriente | Etapa 2: MAIN (50A), TRAC (40A), DIR (15A) |
| 2 | Módulo relé con optoacoplador | SRD-05VDC o similar | LED, LED_REAR |
| 1 | Relé retención 12V | SRD-05VDC o similar | Mantiene alimentación post-llave |
| 1 | Módulo relé audio | Con optoacoplador | Control altavoz DFPlayer |

### 9.8 Resistencias (todas ¼W salvo indicación)

| Qty | Valor | Función |
|-----|-------|---------|
| 1 | 33 kΩ ±1% | R1 divisor llave contacto (12V→2.79V) |
| 1 | 10 kΩ ±1% | R2 divisor llave contacto |
| 1 | 10 kΩ ±1% | R1 divisor pedal (5V→2.0V) |
| 1 | 6.8 kΩ ±1% | R2 divisor pedal |
| 3 | 330 Ω ±5% | R_IN serie LED 6N137 encoder (A, B, Z) |
| 3 | 4.7 kΩ ±5% | Pull-up salida Vo 6N137 encoder a +3.3V |
| 4 | 4.7 kΩ ±5% | Pull-up I2C (SCL+SDA ×2 buses) |
| 1 | 4.7 kΩ ±5% | Pull-up OneWire (PB0) |
| 5 | 10 kΩ ±5% | Pull-down base transistor/relé |
| 4 | 820 Ω ±5% | Corriente LED optoacoplador PC817 |
| 2 | 330 Ω ±5% | Serie datos WS2812B (frontal + trasero) |
| 1 | 1 kΩ ±5% | Base transistor BC547 (relé retención) |
| 1 | 1 kΩ ±5% | Serie TX DFPlayer |
| 5 | 100 Ω ½W | Snubber RC serie (relés) |
| 1 | 1 kΩ ±5% | Serie UART TOFSense |
| 1 | 4.7 kΩ ±5% | Serie UART TOFSense |
| 1 | 10 kΩ ±5% | Pull-up RESET MCP23017 |

### 9.9 Resistencias shunt (INA226)

| Qty | Valor | Potencia | Encapsulado | Función |
|-----|-------|----------|-------------|---------|
| 5 | 1.5 mΩ ±1% | 5W | 2512 | Shunt motores (50A máx) |
| 1 | 0.75 mΩ ±1% | 10W | 2512 | Shunt batería (100A máx) |

### 9.10 Condensadores

**Por cada BTS7960 (×5 módulos):**

| Qty total | Valor | Tipo | Función |
|-----------|-------|------|---------|
| 5 | 470 µF / 35V | Electrolítico | Bulk B+ a GND |
| 5 | 100 nF / 50V | X7R cerámico | Bypass alimentación |
| 5 | 100 nF / 50V | X7R cerámico | Bypass lógica VCC |
| 5 | 100 nF / 50V | X7R cerámico | Snubber motor M+/M− |

**Alimentación principal:**

| Qty | Valor | Tipo | Función |
|-----|-------|------|---------|
| 1 | 1000 µF / 35V | Electrolítico | Bulk bus 24V (junto a relés) |
| 1 | 470 µF / 25V | Electrolítico | Bulk bus 12V (dirección) |
| 2 | 1000 µF / 10V | Electrolítico | Bulk 5V para tiras LED |
| 1 | 47 µF / 10V | Electrolítico | Bulk 5V lógica |

**Control y desacoplo:**

| Qty | Valor | Tipo | Función |
|-----|-------|------|---------|
| 2 | 100 nF / 16V | X7R cerámico | Desacoplo 3.3V STM32 Nucleo |
| 1 | 10 µF / 10V | Electrolítico | Bulk 3.3V STM32 |
| 2 | 100 nF / 10V | X7R cerámico | Desacoplo VCC SN65HVD230 |
| 1 | 10 µF / 10V | Electrolítico | Bulk CAN lado ESP32 |
| 1 | 100 nF / 16V | X7R cerámico | Desacoplo ADC pedal |
| 5 | 100 nF / 250V ac | Cerámico | Snubber RC (C) paralelo contactos relé |

### 9.11 Diodos

| Qty | Componente | Valor | Función |
|-----|-----------|-------|---------|
| 5 | Diodo flyback | 1N4007 (1A/1000V) | Anti-pico inductivo bobina relé |
| 2 | Diodo OR | 1N4148 | Circuito OR relé retención |
| 1 | Diodo flyback | 1N4007 | Bobina relé retención |
| 5 | Schottky freewheeling | SB560 (5A/60V) | Anti-CEMF motor (opcional) |
| 5 | TVS potencia | SMBJ30A | Protección B+ BTS7960 (opcional) |

### 9.12 Semiconductores activos

| Qty | Componente | Referencia | Función |
|-----|-----------|------------|---------|
| 1 | Transistor NPN | BC547 o 2N2222 | Driver bobina relé retención |
| 5 | Optoacoplador baja velocidad | PC817 | Aislamiento sensores inductivos |
| 3 | Optoacoplador alta velocidad | 6N137 | Aislamiento encoder (A, B, Z) |
| 1 | MOSFET P-channel | ≥40V/80A | Protección anti-polaridad (opcional) |

### 9.13 Baterías y reguladores

| Qty | Componente | Especificación | Función |
|-----|-----------|---------------|---------|
| 1 | Batería 24V | ≥20 Ah, ≥30A descarga continua | Tracción |
| 1 | Regulador 12V | ≥5A | Dirección + llave |
| 1 | Regulador DC-DC 24V→5V | LM2596, ≥5A | Lógica + LEDs |

### 9.14 Fusibles

| Qty | Tipo | Corriente | Función |
|-----|------|-----------|---------|
| 1 | Maxi slow-blow | 60 A | Protección BAT 24V → RELAY_MAIN |
| 4 | Midi slow-blow | 30 A | Protección individual motor tracción |
| 1 | Blade slow-blow | 15 A | Protección motor dirección |
| 1 | Blade slow-blow | 5 A | Protección 5V lógica |

### 9.15 Cableado

| Sección | AWG equiv. | Función |
|---------|------------|---------|
| 4 mm² | 12 AWG | BAT 24V/12V distribución potencia |
| 2.5 mm² | 14 AWG | Motor → BTS7960 alimentación |
| 1.5 mm² | 16 AWG | Motor dirección, masas lógicas |
| 24 AWG | — | PWM, GPIO, I2C, señales lógicas |
| 28 AWG apantallado | — | Encoder A/B/Z, pedal ADC, UART TOFSense |
| 22 AWG par trenzado apantallado | — | CAN CANH+CANL |

### 9.16 Varios

| Qty | Componente | Función |
|-----|-----------|---------|
| 1 | Ferrita | BLM18AG601SN1D o 100µH, filtrado 5V→CAN |
| 1 | MCP23017 | Expansor GPIO I2C (dirección personalizable) |
| 1 | TOFSense-M S | LiDAR 8×8, 5V, UART 921600 bps (obstáculos) |

---

## Apéndice A: Inhibición de Arranque y Validación de Boot

### Inhibición de arranque

```c
bool startup_inhibit = true;  // Inicialmente true en main.c
```

- **Función:** Prevenir movimiento de motor si el pedal está presionado al encender
- **Condición de liberación:** Pedal < 3% durante 400 ms continuos
- **Referencia:** `Core/Src/main.c`

### Validación de boot (6 comprobaciones obligatorias)

1. ✅ Plausibilidad sensor pedal
2. ✅ Sensores INA226 disponibles
3. ✅ Posición encoder sincronizada
4. ✅ Sensores velocidad ruedas respondiendo
5. ✅ Sensores temperatura lectura válida
6. ✅ Heartbeat CAN del ESP32 recibido

Solo después de pasar las 6 comprobaciones, el sistema transiciona de BOOT → STANDBY → ACTIVE.

---

## Apéndice B: Resumen de Corrientes Máximas por Circuito

| Circuito | Tensión | Corriente máx | Fusible | Protección |
|----------|---------|---------------|---------|------------|
| Tracción total | 24V | 100 A | 60 A (main) | INA226 ch4 + RELAY_MAIN |
| Motor individual | 24V | 25 A | 30 A | INA226 ch0–3 |
| Dirección | 12V | 25 A | 15 A | INA226 ch5 |
| Lógica 5V | 5V | ~3.5 A | 5 A | Regulador LM2596 |
| LED frontal | 5V | 1.68 A | — | RELAY_LED PB10 |
| LED trasero | 5V | 0.96 A | — | RELAY_LED_REAR PB11 |
| CAN transceivers | 5V (filtrado) | 0.14 A | — | Ferrita + desacoplo |
| DFPlayer audio | 5V | 0.2 A | — | Relé audio GPIO 11 |

---

> **Documento generado a partir del firmware verificado del repositorio STM32-Control-Coche-Marcos.**
> Ficheros fuente principales: `Core/Src/safety_system.c`, `Core/Src/can_handler.c`, `Core/Src/main.c`,
> `esp32/src/power_manager.cpp`, `esp32/src/audio_manager.cpp`, `STM32-Control-Coche-Marcos.ioc`,
> `docs/LLAVE_CONTACTO_ENCENDIDO_APAGADO.md`, `docs/POWER_DISTRIBUTION.md`, `docs/CONEXIONES_COMPLETAS.md`,
> `docs/HARDWARE_WIRING_MANUAL.md`, `docs/MATERIALES_POR_MODULO.md`
