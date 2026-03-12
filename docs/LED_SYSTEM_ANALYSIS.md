# Análisis Completo del Sistema de LEDs — STM32-Control-Coche-Marcos

> **Fecha:** 2026-03-12  
> **Firmware analizado:** STM32G474RET6 + ESP32-S3 HMI  
> **Alcance:** Identificación y documentación de todos los LEDs (físicos y "LEDs inteligentes de estado")  
> **Tipo de documento:** Solo lectura — ningún archivo del proyecto fue modificado.

---

## Índice

1. [Resumen Ejecutivo](#1-resumen-ejecutivo)
2. [Configuración de Pines GPIO para LEDs (archivo .ioc)](#2-configuración-de-pines-gpio-para-leds-archivo-ioc)
3. [Defines y Macros Relacionados con LEDs](#3-defines-y-macros-relacionados-con-leds)
4. [Uso de HAL_GPIO_WritePin / TogglePin / ReadPin](#4-uso-de-hal_gpio_writepin--togglepin--readpin)
5. [Archivos que Controlan los LEDs](#5-archivos-que-controlan-los-leds)
6. [Comportamiento Funcional de los LEDs](#6-comportamiento-funcional-de-los-leds)
7. [LEDs Inteligentes de Estado](#7-leds-inteligentes-de-estado)
8. [Funcionamiento Detallado de los LEDs Inteligentes](#8-funcionamiento-detallado-de-los-leds-inteligentes)
9. [Conexión Eléctrica](#9-conexión-eléctrica)
10. [Tablas Resumen](#10-tablas-resumen)
11. [LEDs Ocultos de Debug o Heartbeat](#11-leds-ocultos-de-debug-o-heartbeat)
12. [Conclusiones](#12-conclusiones)

---

## 1. Resumen Ejecutivo

El sistema de LEDs del proyecto STM32-Control-Coche-Marcos utiliza una **arquitectura distribuida de dos procesadores**:

| Componente | Procesador | Función |
|---|---|---|
| **Relés de potencia LED** | STM32G474RET6 | Controla los relés de 5 V que alimentan las tiras WS2812B |
| **Animaciones LED (WS2812B)** | ESP32-S3 | Genera patrones en las tiras LED addressable vía FastLED |

**Hallazgos clave:**

- **2 pines GPIO** del STM32 están dedicados a LEDs: **PB10** (relé frontal) y **PB11** (relé trasero).
- El STM32 **NO** controla directamente ningún LED individual. Solo activa/desactiva la alimentación de 5 V a las tiras WS2812B.
- **NO existe** LED de usuario LD2 (PA5) de la placa Nucleo configurado en el firmware.
- **NO existen** LEDs de heartbeat, status, error, debug ni ningún LED de estado en el STM32.
- **NO se usa** `HAL_GPIO_TogglePin` en ningún archivo del proyecto.
- Todas las animaciones, patrones y señalización visual son responsabilidad exclusiva del **ESP32-S3**.
- La comunicación STM32↔ESP32 para LEDs se realiza vía **CAN bus** (mensajes 0x120 y 0x20A).

---

## 2. Configuración de Pines GPIO para LEDs (archivo .ioc)

### Archivo: `STM32-Control-Coche-Marcos.ioc`

#### PB10 — Relé LED Frontal (`RELAY_LED`)

```
Mcu.Pin18=PB10
PB10.GPIOParameters=GPIO_Label
PB10.GPIO_Label=RELAY_LED
PB10.Locked=true
PB10.Signal=GPIO_Output
```

| Parámetro | Valor |
|---|---|
| **Pin** | PB10 (LQFP64 pin 29) |
| **Etiqueta GPIO_Label** | `RELAY_LED` |
| **Modo** | GPIO_Output (Push-Pull) |
| **Pull** | No Pull (GPIO_NOPULL) |
| **Velocidad** | Low (GPIO_SPEED_FREQ_LOW) |
| **Estado inicial** | LOW (GPIO_PIN_RESET = relé apagado) |
| **Función** | Controla la alimentación de 5 V a la tira frontal WS2812B (28 LEDs) |

#### PB11 — Relé LED Trasero (`RELAY_LED_REAR`)

```
Mcu.Pin19=PB11
PB11.GPIOParameters=GPIO_Label
PB11.GPIO_Label=RELAY_LED_REAR
PB11.Locked=true
PB11.Signal=GPIO_Output
```

| Parámetro | Valor |
|---|---|
| **Pin** | PB11 (LQFP64 pin 30) |
| **Etiqueta GPIO_Label** | `RELAY_LED_REAR` |
| **Modo** | GPIO_Output (Push-Pull) |
| **Pull** | No Pull (GPIO_NOPULL) |
| **Velocidad** | Low (GPIO_SPEED_FREQ_LOW) |
| **Estado inicial** | LOW (GPIO_PIN_RESET = relé apagado) |
| **Función** | Controla la alimentación de 5 V a la tira trasera WS2812B (16 LEDs) |

#### LED de Usuario LD2 (PA5) de la placa Nucleo

**NO CONFIGURADO.** El pin PA5 no aparece en el archivo `.ioc` ni en ningún define del firmware. La placa NUCLEO-G474RE tiene un LED de usuario LD2 conectado a PA5, pero el firmware **no lo utiliza**.

---

## 3. Defines y Macros Relacionados con LEDs

### Core/Inc/main.h (líneas 64-71)

```c
/* ---- LED Power Relays (GPIOB) ----
 * Front relay (PB10): controls 5V supply to front WS2812B LED strip (28 LEDs).
 * Rear  relay (PB11): controls 5V supply to rear  WS2812B LED strip (16 LEDs).
 * The ESP32 drives the WS2812B data lines; the STM32 controls the
 * power relays for safety cutoff.  Toggled via CAN command 0x120.
 *   Byte 0 = front relay, Byte 1 = rear relay.                              */
#define PIN_RELAY_LED       GPIO_PIN_10  /* PB10 — front LED strip relay */
#define PIN_RELAY_LED_REAR  GPIO_PIN_11  /* PB11 — rear  LED strip relay */
```

### Core/Inc/can_handler.h (líneas 26, 37)

```c
#define CAN_ID_CMD_LED            0x120  // ESP32 → STM32 (on-demand) LED relay control
#define CAN_ID_STATUS_LIGHTS      0x20A  // STM32 → ESP32 (1000ms) LED relay + light state
```

### Macros / defines NO encontrados

Los siguientes nombres **NO existen** en el firmware STM32:

- `STATUS_LED` — no definido
- `ERROR_LED` — no definido
- `DEBUG_LED` — no definido
- `HEARTBEAT_LED` — no definido
- `LD2` — no definido
- `LED_Pin` — no definido

---

## 4. Uso de HAL_GPIO_WritePin / TogglePin / ReadPin

### 4.1 HAL_GPIO_WritePin — Uso para LEDs

| Archivo | Línea | Código | Función |
|---|---|---|---|
| `main.c` | 505 | `HAL_GPIO_WritePin(GPIOB, PIN_RELAY_LED \| PIN_RELAY_LED_REAR, GPIO_PIN_RESET)` | Inicialización: ambos relés OFF |
| `can_handler.c` | 517-518 | `HAL_GPIO_WritePin(GPIOB, PIN_RELAY_LED, on ? GPIO_PIN_SET : GPIO_PIN_RESET)` | Control del relé frontal |
| `can_handler.c` | 527-528 | `HAL_GPIO_WritePin(GPIOB, PIN_RELAY_LED_REAR, on ? GPIO_PIN_SET : GPIO_PIN_RESET)` | Control del relé trasero |

#### HAL_GPIO_WritePin — Uso NO relacionado con LEDs

| Archivo | Función | Puerto/Pin | Propósito |
|---|---|---|---|
| `safety_system.c` | Relay power sequencing | GPIOC / PIN_RELAY_MAIN, TRAC, DIR | Relés de tracción/dirección |
| `sensor_manager.c` | I2C bus recovery | GPIOB / PIN_I2C_SCL | Recuperación del bus I2C |
| `sensor_manager.c` | OneWire protocol | GPIOB / PIN_ONEWIRE | Protocolo DS18B20 |

### 4.2 HAL_GPIO_TogglePin

**NO SE USA.** Cero llamadas a `HAL_GPIO_TogglePin` en todo el código fuente del STM32.

### 4.3 HAL_GPIO_ReadPin — Uso NO relacionado con LEDs

| Archivo | Función | Propósito |
|---|---|---|
| `sensor_manager.c` | I2C bus recovery | Lectura de SDA para detectar esclavo atascado |
| `sensor_manager.c` | OneWire reset/read | Protocolo de temperatura DS18B20 |
| `steering_cal_store.c` | Steering center validation | Lectura del sensor inductivo de centro |

Ninguna llamada a `HAL_GPIO_ReadPin` está relacionada con LEDs.

### 4.4 Acceso directo a registros (Error_Handler)

```c
/* main.c línea 827 */
GPIOB->BSRR = (uint32_t)(PIN_RELAY_LED | PIN_RELAY_LED_REAR) << 16U;
```

Este acceso directo al registro BSRR fuerza ambos relés LED a OFF usando escritura directa al hardware (sin HAL), garantizando el apagado incluso si el HAL está corrupto.

---

## 5. Archivos que Controlan los LEDs

| Archivo | Interacción con LEDs | Tipo |
|---|---|---|
| **`Core/Src/main.c`** | Inicialización GPIO (PB10, PB11 como output PP, estado OFF). Error_Handler fuerza relés OFF vía registro BSRR. Bucle principal llama `CAN_SendStatusLights()` cada 1000 ms. | Inicialización + Safety |
| **`Core/Src/can_handler.c`** | Funciones `LED_Relay_Set()`, `LED_Relay_Rear_Set()`, `LED_Relay_Get()`, `LED_Relay_Rear_Get()`, `CAN_SendStatusLights()`. Procesa el mensaje CAN 0x120 (CMD_LED). | Control principal |
| **`Core/Inc/main.h`** | Define `PIN_RELAY_LED` (GPIO_PIN_10), `PIN_RELAY_LED_REAR` (GPIO_PIN_11). | Definiciones |
| **`Core/Inc/can_handler.h`** | Define `CAN_ID_CMD_LED` (0x120), `CAN_ID_STATUS_LIGHTS` (0x20A). Declara las funciones LED_Relay_*. | Declaraciones |
| `Core/Src/safety_system.c` | **NO interactúa** con relés LED. Solo controla relés de tracción (GPIOC). | Sin relación |
| `Core/Src/sensor_manager.c` | **NO interactúa** con LEDs. Solo usa GPIO para I2C y OneWire. | Sin relación |
| `Core/Src/motor_control.c` | **NO interactúa** con LEDs. Solo controla PWM de motores. | Sin relación |

---

## 6. Comportamiento Funcional de los LEDs

### 6.1 Cuándo se encienden

Los relés LED se encienden cuando el ESP32 envía un mensaje CAN **0x120 (CMD_LED)**:

```
CAN ID: 0x120
Dirección: ESP32 → STM32
DLC: 2 bytes
Byte 0: Relé frontal (0 = OFF, 1 = ON)
Byte 1: Relé trasero (0 = OFF, 1 = ON)
```

El procesamiento en `can_handler.c` (líneas 903-920):

```c
case CAN_ID_CMD_LED:
    if (msg_len >= 1) {
        LED_Relay_Set(rx_payload[0] != 0);       // Relé frontal
        if (msg_len >= 2) {
            LED_Relay_Rear_Set(rx_payload[1] != 0); // Relé trasero
        }
        CAN_SendCommandAck(CAN_ID_CMD_LED & 0xFF, ACK_OK);
    } else {
        CAN_SendCommandAck(CAN_ID_CMD_LED & 0xFF, ACK_INVALID);
    }
    break;
```

### 6.2 Cuándo se apagan

Los relés LED se apagan en tres situaciones:

1. **Comando CAN 0x120** con byte = 0 (apagado normal vía ESP32).
2. **Inicialización del sistema** — ambos relés inician en OFF (`GPIO_PIN_RESET`).
3. **Error_Handler** — apagado de emergencia vía registro BSRR directo.

### 6.3 Parpadeo

**NO existe parpadeo en el STM32.** El firmware STM32 nunca implementa patrones de parpadeo. El STM32 solo actúa como un interruptor ON/OFF binario para los relés de potencia.

Todo el parpadeo y las animaciones son responsabilidad del **ESP32-S3** usando la librería FastLED.

### 6.4 Indicación de estados del sistema

Los relés LED del STM32 **NO indican estados del sistema**. Son controles de potencia pasivos que simplemente encienden o apagan la alimentación de 5 V a las tiras LED WS2812B.

La indicación visual de estados se realiza completamente en el **ESP32-S3** mediante los modos de animación:

| Estado | Tira frontal (28 LEDs) | Tira trasera (16 LEDs) |
|---|---|---|
| Inactivo (sin acelerador) | KITT scanner rojo | Luces de posición (rojo tenue 20 %) |
| Acelerador 1-25 % | Chase rojo→naranja | Luces de posición |
| Acelerador 25-50 % | Chase naranja→amarillo | Luces de posición |
| Acelerador 50-75 % | Chase amarillo→verde | Luces de posición |
| Acelerador 75-100 % | Arcoíris | Luces de posición |
| Marcha atrás | KITT scanner blanco | Luces blancas de reversa |
| Frenado | Normal | Rojo brillante 100 % |
| Frenado de emergencia | Normal | Rojo intermitente |
| ABS activo | Flash rojo/blanco | Normal |
| TCS activo | Flash naranja/negro | Normal |
| Frenado regenerativo | Normal | Pulso azul |

---

## 7. LEDs Inteligentes de Estado

### 7.1 ¿Existen LEDs de estado en el STM32?

**NO.** El firmware del STM32 no implementa ningún "LED inteligente de estado". No hay ningún LED conectado al STM32 que represente:

| Estado buscado | ¿Existe en STM32? | Observación |
|---|---|---|
| Sistema encendido | ❌ No | No hay LED de power-on |
| Error de seguridad | ❌ No | Los errores se reportan vía CAN (heartbeat 0x001 byte 4) |
| Watchdog activo | ❌ No | El IWDG opera internamente sin indicación visual |
| Comunicación CAN activa | ❌ No | El estado CAN se monitoriza vía contadores internos |
| Modo servicio | ❌ No | El modo servicio se reporta vía CAN (0x301) |
| Fallo de sensor | ❌ No | Los fallos se reportan vía CAN (heartbeat flags + error log) |
| Fallo de motor | ❌ No | Los fallos se reportan vía CAN |
| Arranque del sistema | ❌ No | La secuencia de boot se verifica internamente |
| Heartbeat del firmware | ❌ No | El heartbeat es un mensaje CAN (0x001), no un LED |

### 7.2 ¿Dónde se indican estos estados?

Todos los estados del sistema se comunican al ESP32 vía **CAN bus**, donde se muestran en la **pantalla TFT** (ILI9488) y opcionalmente mediante los **LEDs WS2812B**:

- **CAN 0x001 (Heartbeat STM32):** Enviado cada 100 ms, incluye byte de estado del sistema y byte de flags de fallo.
- **CAN 0x20A (Status Lights):** Enviado cada 1000 ms, confirma el estado de los relés LED.
- **CAN 0x301 (Service Status):** Estado del modo servicio.

---

## 8. Funcionamiento Detallado de los LEDs Inteligentes

Dado que el STM32 **no tiene LEDs inteligentes de estado propios**, esta sección describe cómo funcionan los LEDs WS2812B controlados por el ESP32 en respuesta a datos del STM32.

### 8.1 Eventos del firmware que activan cambios de LED

| Evento en STM32 | Mensaje CAN | Reacción en ESP32 (WS2812B) |
|---|---|---|
| ESP32 envía CMD_LED (0x120) | 0x120 → STM32 activa relé | Los LEDs se alimentan y muestran el patrón activo |
| Cambio de acelerador | 0x201 (STATUS_MOTOR) con % throttle | Modo frontal cambia (KITT → Chase → Rainbow) |
| Frenado detectado | 0x201 (STATUS_MOTOR) | Traseros cambian a rojo brillante |
| ABS activo | 0x001 heartbeat flags | Frontales flash rojo/blanco |
| TCS activo | 0x001 heartbeat flags | Frontales flash naranja/negro |
| Marcha atrás | Comando de dirección | Frontales KITT blanco + traseros blancos |
| Frenado de emergencia | Safety system flag | Traseros parpadeo rojo |

### 8.2 Funciones del código que controlan los LEDs

#### En el STM32 (`can_handler.c`):

| Función | Propósito |
|---|---|
| `LED_Relay_Set(bool on)` | Activa/desactiva relé frontal (PB10) |
| `LED_Relay_Rear_Set(bool on)` | Activa/desactiva relé trasero (PB11) |
| `LED_Relay_Get(void)` | Lee estado actual del relé frontal |
| `LED_Relay_Rear_Get(void)` | Lee estado actual del relé trasero |
| `CAN_SendStatusLights(void)` | Envía estado de relés al ESP32 vía CAN 0x20A |

#### En el ESP32 (`led_controller.cpp`):

| Función | Propósito |
|---|---|
| `led_ctrl::init()` | Inicializa FastLED para ambas tiras |
| `led_ctrl::update()` | Actualización maestra (~20 Hz) con animaciones |
| `led_ctrl::setFrontMode(FrontMode)` | Establece modo de animación frontal |
| `led_ctrl::setFrontFromThrottle(float)` | Establece modo frontal según % acelerador |
| `led_ctrl::setRearMode(RearMode)` | Establece modo trasero (posición, freno, etc.) |
| `led_ctrl::setTurnSignal(TurnSignal)` | Controla intermitentes (L/R/Hazard) |
| `led_ctrl::startEmergencyFlash(uint8_t)` | Activa flash de emergencia |

### 8.3 Temporizadores y ciclos

- **STM32:** No usa temporizadores para LEDs. Los relés se controlan bajo demanda (CAN) y el estado se reporta cada 1000 ms en el bucle principal.
- **ESP32:** Las animaciones usan `millis()` internamente en `led_ctrl::update()`, llamado desde el bucle principal a ~20 Hz. No se usan interrupciones para las animaciones LED.

### 8.4 Dependencia de interrupciones

- **STM32:** La recepción del mensaje CAN 0x120 llega vía interrupción FDCAN, pero el procesamiento se difiere al bucle principal (`CAN_ProcessMessages()`). Los relés LED se modifican en el contexto del hilo principal, no en ISR.
- **ESP32:** Las animaciones LED no dependen de interrupciones. Se ejecutan en el loop principal.

---

## 9. Conexión Eléctrica

### 9.1 Relé LED Frontal (PB10)

```
                     Módulo Relé 5V
STM32 PB10 ──────►[ IN  ┃ COM ]──── +5V (fuente)
(3.3V GPIO)        [ GND ┃ NO  ]──── +5V de tira WS2812B frontal
                   [     ┃ NC  ]     (28 LEDs)
                          │
                         GND ──── GND común
```

| Parámetro | Valor |
|---|---|
| **Pin STM32** | PB10 (GPIO Output Push-Pull) |
| **Lógica** | Activo ALTO (GPIO_PIN_SET = relé activado) |
| **Nivel de señal** | 3.3 V |
| **Tipo de carga** | Módulo relé de 5 V (bobina + contacto NO) |
| **Resistencia** | No necesaria (la entrada del módulo relé tiene su propia resistencia de pull-up y optoacoplador) |
| **Protección** | Diodo flyback en la bobina del relé (incluido en módulos estándar) |
| **Corriente por el GPIO** | < 5 mA (entrada del optoacoplador del módulo relé) |

### 9.2 Relé LED Trasero (PB11)

```
                     Módulo Relé 5V
STM32 PB11 ──────►[ IN  ┃ COM ]──── +5V (fuente)
(3.3V GPIO)        [ GND ┃ NO  ]──── +5V de tira WS2812B trasera
                   [     ┃ NC  ]     (16 LEDs)
                          │
                         GND ──── GND común
```

| Parámetro | Valor |
|---|---|
| **Pin STM32** | PB11 (GPIO Output Push-Pull) |
| **Lógica** | Activo ALTO (GPIO_PIN_SET = relé activado) |
| **Nivel de señal** | 3.3 V |
| **Tipo de carga** | Módulo relé de 5 V (bobina + contacto NO) |
| **Resistencia** | No necesaria |
| **Protección** | Diodo flyback incluido en módulo relé |
| **Corriente por el GPIO** | < 5 mA |

### 9.3 Tiras WS2812B (controladas por ESP32, alimentadas por STM32)

#### Tira Frontal (28× WS2812B)

```
ESP32-S3 GPIO 47 ──►[ DATA IN ]──── Tira WS2812B frontal (28 LEDs)
                                     │
Relé frontal (NO) ──────────────► +5V de la tira
                                     │
                                    GND ──── GND común (ESP32 + STM32 + Fuente)
```

| Parámetro | Valor |
|---|---|
| **Pin de datos** | ESP32-S3 GPIO 47 |
| **Alimentación** | 5 V vía relé PB10 del STM32 |
| **Corriente máxima** | 28 × 60 mA = 1.68 A (blanco pleno) |
| **Resistencia de datos** | 330 Ω en serie con la línea DATA (recomendado) |
| **Capacitor de desacoplo** | 1000 µF electrolítico en la entrada de 5 V de la tira |

#### Tira Trasera (16× WS2812B)

```
ESP32-S3 GPIO 48 ──►[ DATA IN ]──── Tira WS2812B trasera (16 LEDs)
                                     │
Relé trasero (NO) ──────────────► +5V de la tira
                                     │
                                    GND ──── GND común
```

| Parámetro | Valor |
|---|---|
| **Pin de datos** | ESP32-S3 GPIO 48 |
| **Alimentación** | 5 V vía relé PB11 del STM32 |
| **Corriente máxima** | 16 × 60 mA = 0.96 A (blanco pleno) |
| **Resistencia de datos** | 330 Ω en serie con la línea DATA (recomendado) |
| **Capacitor de desacoplo** | 470 µF – 1000 µF en la entrada de 5 V de la tira |

---

## 10. Tablas Resumen

### A) Tabla de LEDs Detectados

| Nombre del LED | Pin STM32 | Puerto GPIO | Archivo que lo controla | Función que lo activa | Función que lo desactiva |
|---|---|---|---|---|---|
| **Relé LED Frontal** | PB10 | GPIOB | `can_handler.c` | `LED_Relay_Set(true)` | `LED_Relay_Set(false)` |
| **Relé LED Trasero** | PB11 | GPIOB | `can_handler.c` | `LED_Relay_Rear_Set(true)` | `LED_Relay_Rear_Set(false)` |
| _(LD2 Nucleo)_ | _(PA5)_ | _(GPIOA)_ | _No configurado_ | _No existe en firmware_ | _No existe en firmware_ |

> **Nota:** No se encontró ningún LED individual (status, error, heartbeat, debug) conectado directamente al STM32. Los únicos GPIO de "LED" son relés de potencia que alimentan tiras WS2812B gestionadas por el ESP32.

### B) Tabla de Estados del Sistema Mostrados por LEDs

#### Estados indicados por las tiras WS2812B (ESP32)

| Estado del Sistema | Componente Visual | Comportamiento del LED | Controlado por |
|---|---|---|---|
| **Vehículo en reposo** | Tira frontal (28 LEDs) | KITT scanner rojo | ESP32 `FrontMode::KITT_IDLE` |
| **Acelerador bajo (1-25 %)** | Tira frontal | Chase rojo→naranja | ESP32 `FrontMode::ACCEL_LOW` |
| **Acelerador medio (25-50 %)** | Tira frontal | Chase naranja→amarillo | ESP32 `FrontMode::ACCEL_MED` |
| **Acelerador alto (50-75 %)** | Tira frontal | Chase amarillo→verde | ESP32 `FrontMode::ACCEL_HIGH` |
| **Acelerador máximo (75-100 %)** | Tira frontal | Arcoíris | ESP32 `FrontMode::ACCEL_MAX` |
| **Marcha atrás** | Tira frontal | KITT scanner blanco | ESP32 `FrontMode::REVERSE` |
| **ABS activo** | Tira frontal | Flash rojo/blanco (100 ms) | ESP32 `FrontMode::ABS_ALERT` |
| **TCS activo** | Tira frontal | Flash naranja/negro (100 ms) | ESP32 `FrontMode::TCS_ALERT` |
| **Luces de posición** | Tira trasera (16 LEDs) | Rojo tenue (20 % brillo) | ESP32 `RearMode::POSITION` |
| **Frenado** | Tira trasera (centro) | Rojo brillante (100 %) | ESP32 `RearMode::BRAKE` |
| **Frenado de emergencia** | Tira trasera | Rojo intermitente | ESP32 `RearMode::BRAKE_EMERGENCY` |
| **Marcha atrás** | Tira trasera (centro) | Blanco | ESP32 `RearMode::REVERSE` |
| **Frenado regenerativo** | Tira trasera (centro) | Pulso azul | ESP32 `RearMode::REGEN_ACTIVE` |
| **Intermitente izquierdo** | Tira trasera (LEDs 0-2) | Ámbar parpadeo 500 ms | ESP32 `TurnSignal::LEFT` |
| **Intermitente derecho** | Tira trasera (LEDs 13-15) | Ámbar parpadeo 500 ms | ESP32 `TurnSignal::RIGHT` |
| **Luces de emergencia** | Tira trasera (ambos lados) | Ámbar parpadeo 500 ms | ESP32 `TurnSignal::HAZARD` |

#### Estados del STM32 reportados vía CAN (sin LED visual propio)

| Estado del STM32 | Canal de reporte | ID CAN | Frecuencia |
|---|---|---|---|
| Heartbeat del firmware | CAN mensaje | 0x001 | 100 ms (10 Hz) |
| Estado de seguridad (BOOT/STANDBY/ACTIVE/DEGRADED/SAFE/ERROR) | CAN 0x001 byte 1-3 | 0x001 | 100 ms |
| Flags de fallo (sensor, motor, CAN, watchdog) | CAN 0x001 byte 4 | 0x001 | 100 ms |
| Estado de relés LED | CAN mensaje | 0x20A | 1000 ms (1 Hz) |
| Modo servicio | CAN mensaje | 0x301 | 1000 ms |
| Error log | CAN mensaje | 0x305 | 1000 ms |

### C) Recomendación de Conexión Eléctrica para Cada LED

#### Relé frontal (PB10 → Tira WS2812B frontal)

| Elemento | Especificación |
|---|---|
| GPIO STM32 | PB10 (3.3 V Push-Pull, activo alto) |
| Módulo relé | SRD-05VDC o similar, con optoacoplador (entrada IN compatible 3.3 V) |
| Conexión GPIO → Relé | PB10 → IN del módulo relé (sin resistencia adicional) |
| Alimentación relé | 5 V (VCC del módulo), GND común |
| Contacto NO → Tira | Conectar COM a +5 V fuente, NO a +5 V de la tira frontal |
| Protección | Diodo flyback 1N4007 en bobina (incluido en módulos estándar) |
| Tira WS2812B | DATA IN ← 330 Ω ← ESP32 GPIO 47 |
| Desacoplo | 1000 µF electrolítico en +5 V de la tira |

#### Relé trasero (PB11 → Tira WS2812B trasera)

| Elemento | Especificación |
|---|---|
| GPIO STM32 | PB11 (3.3 V Push-Pull, activo alto) |
| Módulo relé | SRD-05VDC o similar, con optoacoplador |
| Conexión GPIO → Relé | PB11 → IN del módulo relé (sin resistencia adicional) |
| Alimentación relé | 5 V (VCC del módulo), GND común |
| Contacto NO → Tira | Conectar COM a +5 V fuente, NO a +5 V de la tira trasera |
| Protección | Diodo flyback 1N4007 (incluido en módulos estándar) |
| Tira WS2812B | DATA IN ← 330 Ω ← ESP32 GPIO 48 |
| Desacoplo | 470 µF – 1000 µF electrolítico en +5 V de la tira |

#### Si se quisiera añadir un LED individual de debug (hipotético)

Si en el futuro se añadiera un LED de estado directamente al STM32 (ej. heartbeat en PA5/LD2):

```
STM32 PA5 ──── R (220 Ω) ──── LED (ánodo) ──── GND (cátodo)
```

| Elemento | Especificación |
|---|---|
| GPIO | PA5 (Push-Pull, activo alto) |
| Resistencia | 220 Ω (para LED verde/rojo estándar a 3.3 V, ~7 mA) |
| LED | LED estándar 5 mm o 3 mm (Vf ≈ 2.0 V verde, 1.8 V rojo) |
| Conexión | GPIO → Resistencia → Ánodo LED → Cátodo → GND |

---

## 11. LEDs Ocultos de Debug o Heartbeat

### Resultado: NO EXISTEN

Se ha verificado exhaustivamente:

1. **Búsqueda en todo el código fuente** (`Core/Src/*.c`, `Core/Inc/*.h`):
   - No se encontraron defines con nombres `STATUS_LED`, `ERROR_LED`, `DEBUG_LED`, `HEARTBEAT_LED`.
   - No se encontraron llamadas a `HAL_GPIO_TogglePin` (que típicamente se usa para heartbeat).
   - No se encontraron patrones de parpadeo basados en `HAL_GetTick()` + GPIO.

2. **Archivo .ioc**: Solo PB10 y PB11 están configurados como GPIO_Output con etiqueta LED.

3. **PA5 (LD2 Nucleo)**: No configurado. El pin PA5 no aparece en el `.ioc`.

4. **Heartbeat del firmware**: El heartbeat existe como **mensaje CAN 0x001** (cada 100 ms), no como LED parpadeante. Usa un contador rodante (`heartbeat_counter`) y flags de estado.

5. **Error_Handler**: No parpadea ningún LED. Entra en un bucle infinito `while(1){}` después de desactivar todo el hardware.

6. **Watchdog (IWDG)**: Opera internamente. No tiene indicación LED. Se alimenta en el bucle principal.

### Implicación

Durante el desarrollo y depuración del vehículo, **no hay indicación visual local** en la placa STM32 sobre:
- Si el firmware está ejecutándose
- Si ha entrado en Error_Handler
- Si el watchdog ha reiniciado el sistema
- Si la comunicación CAN está activa

Toda la información de estado requiere acceso al **bus CAN** (a través del ESP32 y su pantalla TFT) o un **depurador SWD** conectado.

---

## 12. Conclusiones

### Arquitectura LED del Sistema

```
┌─────────────────────┐          CAN Bus          ┌─────────────────────┐
│    STM32G474RET6     │◄────── 0x120 CMD_LED ─────│     ESP32-S3        │
│                      │                            │                     │
│  PB10 ──► Relé 5V ──┼────── Alimentación ───────►│ GPIO 47 → FastLED   │
│  (Frontal)           │                            │ (28× WS2812B front) │
│                      │                            │                     │
│  PB11 ──► Relé 5V ──┼────── Alimentación ───────►│ GPIO 48 → FastLED   │
│  (Trasero)           │                            │ (16× WS2812B rear)  │
│                      │                            │                     │
│  CAN_SendStatusLights├────── 0x20A STATUS ───────►│ Sincronización HMI  │
│  (cada 1000 ms)      │                            │                     │
└─────────────────────┘                            └─────────────────────┘
```

### Resumen Final

| Aspecto | Estado |
|---|---|
| LEDs físicos controlados por STM32 | **2** (relés de potencia PB10, PB11) |
| LEDs de estado/heartbeat en STM32 | **0** (no existen) |
| LEDs WS2812B controlados por ESP32 | **44** (28 frontales + 16 traseros) |
| Protocolo de comunicación LED | CAN 2.0A (0x120 comando, 0x20A telemetría) |
| LED de usuario LD2 (PA5) | No configurado |
| LEDs ocultos de debug | No existen |
| Patrones de parpadeo en STM32 | Ninguno |
| Seguridad (Error_Handler) | Ambos relés forzados OFF vía BSRR directo |

El sistema de LEDs está diseñado con una clara **separación de responsabilidades**: el STM32 gestiona la seguridad eléctrica (puede cortar la alimentación a las tiras LED en cualquier momento), mientras que el ESP32 se encarga de toda la inteligencia visual (patrones, animaciones, señalización).
