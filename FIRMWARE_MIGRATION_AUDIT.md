# AUDITORÍA COMPLETA DE MIGRACIÓN FIRMWARE

**Firmware original:** FULL-FIRMWARE-Coche-Marcos v2.17.1 (ESP32-S3 monolítico)
**Firmware migrado:** STM32-Control-Coche-Marcos (STM32G474RE + ESP32-S3)
**Fecha:** 2026-02-21
**Método:** Comparación módulo a módulo basada exclusivamente en evidencia de código fuente.

---

## 1. ESTADO GLOBAL DE LA MIGRACIÓN

### Equivalencia funcional: 72%

| Categoría                     | Subsistemas originales | Implementados | Parciales | Faltantes | Equivalencia |
|-------------------------------|:----------------------:|:-------------:|:---------:|:---------:|:------------:|
| A. Control de tracción        | 8                      | 8             | 0         | 0         | 100%         |
| B. Dirección                  | 3                      | 3             | 0         | 0         | 100%         |
| C. Sistema de seguridad       | 6                      | 6             | 0         | 0         | 100%         |
| D. ABS / TCS                  | 4                      | 3             | 1         | 0         | 88%          |
| E. Detección de obstáculos    | 5                      | 2             | 1         | 2         | 50%          |
| F. Crucero adaptativo (ACC)   | 1                      | 0             | 0         | 1         | 0%           |
| G. Frenado regenerativo       | 1                      | 0             | 0         | 1         | 0%           |
| H. HMI / Pantalla             | 6                      | 4             | 0         | 2         | 67%          |
| I. Iluminación LED            | 1                      | 0             | 0         | 1         | 0%           |
| J. Sistema de audio           | 1                      | 0             | 0         | 1         | 0%           |
| K. Gestión de encendido       | 3                      | 0             | 0         | 3         | 0%           |
| L. Adquisición de sensores    | 5                      | 5             | 0         | 0         | 100%         |
| M. Comunicación CAN           | 4                      | 4             | 0         | 0         | 100%         |
| N. Diagnóstico / Servicio     | 3                      | 3             | 0         | 0         | 100%         |
| O. Persistencia               | 2                      | 0             | 1         | 1         | 25%          |
| P. Entradas físicas           | 2                      | 0             | 0         | 2         | 0%           |
| **TOTAL**                     | **55**                 | **38**        | **3**     | **14**    | **72%**      |

> WiFi/OTA no se contabiliza: fue eliminado intencionalmente en v2.11.0 del original por seguridad.

---

## 2. COMPARACIÓN MÓDULO A MÓDULO

### A. CONTROL DE TRACCIÓN — 100% ✅

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Control PWM por rueda | ✅ STM32 | Equivalente | `motor_control.c`: TIM1 PA8-PA11 (20 kHz directo vs PCA9685 I2C 250 Hz original) |
| Modo 4×2 | ✅ STM32 | Equivalente | `motor_control.c`: `mode4x4=false` → solo FL/FR activos |
| Modo 4×4 | ✅ STM32 | Equivalente | `motor_control.c`: `Traction_SetMode4x4(true)` vía CAN 0x102 bit 0 |
| Giro sobre eje (tank turn) | ✅ STM32 | Equivalente | `motor_control.c`: FL/RL adelante, FR/RR atrás vía CAN 0x102 bit 1 |
| Marchas P/R/N/D1/D2 | ✅ STM32 | Equivalente | `motor_control.c`: D1=60%, D2=100%, R=60% escalado. Cambio a ≤1 km/h |
| Acondicionamiento de pedal | ✅ STM32 | Equivalente | `motor_control.c`: EMA α=0.15, rampa 50%/s subida, 100%/s bajada |
| Frenado dinámico | ✅ STM32 | Equivalente | `motor_control.c`: proporcional al soltar acelerador, máx 60%, inhibido <3 km/h |
| Retención en Park | ✅ STM32 | Equivalente | `motor_control.c`: freno activo H-bridge con derating por corriente/temperatura |

### B. DIRECCIÓN — 100% ✅

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Geometría Ackermann | ✅ STM32 | Equivalente | `ackermann.c`: cálculo ángulos FL/FR con wheelbase=0.95 m, track=0.70 m |
| Control PID/torque-assist | ✅ STM32 | Mejorado | `motor_control.c`: EPS torque-assist (reemplaza PID posicional del original) |
| Calibración de centro | ✅ STM32 | Mejorado | `steering_centering.c`: barrido automático + sensor inductivo PB5. `steering_cal_store.c`: persistencia en flash página 126 |

### C. SISTEMA DE SEGURIDAD — 100% ✅

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Máquina de estados | ✅ STM32 | Mejorado | `safety_system.c`: 7 estados (BOOT→STANDBY→ACTIVE⇄DEGRADED→SAFE→ERROR + LIMP_HOME) vs 4 del original |
| Protección sobrecorriente | ✅ STM32 | Equivalente | `safety_system.c`: 25 A por motor vía INA226 → DEGRADED, persistente → SAFE |
| Protección sobretemperatura | ✅ STM32 | Mejorado | `safety_system.c`: global + per-motor cutoff a 130°C con histéresis 115°C |
| Niveles de degradación | ✅ STM32 | Equivalente | `safety_system.c`: L1=70%, L2=50%, L3=40% potencia |
| Subtensión batería | ✅ STM32 | Equivalente | `safety_system.c`: 20 V warning → DEGRADED, 18 V critical → SAFE, histéresis 0.5 V |
| Timeout CAN | ✅ STM32 | Mejorado | `safety_system.c`: 250 ms → LIMP_HOME (20% torque, 5 km/h) en vez de SAFE |

### D. ABS / TCS — 88% ✅

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| ABS por rueda | ✅ STM32 | Equivalente | `safety_system.c`: slip 15%, modulación 80 ms, mínimo 10 km/h |
| TCS por rueda | ✅ STM32 | Equivalente | `safety_system.c`: slip 15%, reducción progresiva 40%→80%, recuperación 25%/s |
| Detección de deslizamiento | ✅ STM32 | Equivalente | `safety_system.c`: cálculo slip por rueda vs velocidad media |
| Estimación G lateral | ⚠️ | Parcial | No hay cálculo explícito de G lateral. El original usaba esto para adaptar TCS a modos Eco/Normal/Sport. El migrado no tiene modos de conducción |

### E. DETECCIÓN DE OBSTÁCULOS — 50% ⚠️

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Driver de sensor (TOFSense-M S LiDAR) | ⚠️ ESP32 | Sustituido | `esp32/src/sensors/obstacle_sensor.cpp`: HC-SR04 ultrasónico (punto único) sustituye LiDAR 8×8 matrix. Funcionalidad reducida pero operativa |
| 5 zonas de colisión | ⚠️ STM32 | Parcial | `safety_system.c`: solo 3 zonas (EMERGENCY <200mm, CRITICAL 200-500mm, WARNING 500-1000mm). Faltan Zone 2 Caution (1000-1500mm) y Zone 1 Alert (1500-4000mm) |
| Detección de reacción infantil | ❌ | Falta | Original: reducción de pedal >10% en 500 ms modifica respuesta zonas 2-3. No existe código equivalente |
| Parada de emergencia (<200 mm) | ✅ STM32 | Equivalente | `safety_system.c`: obstacle_scale=0.0 con histéresis temporal 200 ms |
| Histéresis temporal | ✅ STM32 | Equivalente | `safety_system.c`: confirmación 200 ms, limpieza 1000 ms |

### F. CRUCERO ADAPTATIVO (ACC) — 0% ❌

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| PID mantenimiento de distancia | ❌ | Falta | Original: `adaptive_cruise.cpp` con Kp=0.3, Ki=0.05, Kd=0.15, target 500 mm. No existe código equivalente en ningún MCU |

### G. FRENADO REGENERATIVO — 0% ❌

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Lógica regen con lookup tables | ❌ | Falta | Original: tablas velocidad/aceleración/SOC/temperatura/pendiente → corriente regen. No existe código. Solo hay frenado dinámico por disipación. `REGEN_BRAKING_AUDIT.md` confirma: "zero regen code in entire firmware" |

### H. HMI / PANTALLA — 67% ⚠️

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Pantalla TFT (ST7796S) | ✅ ESP32 | Equivalente | `esp32/src/main.cpp`: TFT_eSPI 480×320, ST7796 |
| Pantallas estado (boot/standby/drive/safe/error) | ✅ ESP32 | Equivalente | `esp32/src/screens/`: 5 pantallas con transiciones por estado CAN |
| Telemetría por rueda | ✅ ESP32 | Equivalente | `esp32/src/ui/car_renderer.cpp`: torque %, temperatura por rueda |
| Indicadores estado (ABS, TCS, temp) | ✅ ESP32 | Equivalente | `esp32/src/screens/drive_screen.cpp`: indicadores en HUD |
| Entrada táctil para interacción | ❌ | Falta | `platformio.ini` configura XPT2046 (CS=GPIO21, freq 2.5 MHz) pero NO hay código que lea touch. `mode_icons.cpp` tiene `hitTest()` definido pero nunca llamado desde main loop |
| Menú oculto de ingeniería | ❌ | Falta | Original: código secreto 8989 → calibración pedal/encoder, enable/disable módulos, factory restore, visor de errores. No existe código equivalente |

### I. ILUMINACIÓN LED — 0% ❌

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| WS2812B (28 frontales + 16 traseros) | ❌ | Falta | No existe código WS2812B, NeoPixel, ni FastLED en ningún MCU. No hay GPIO asignado para LEDs |

### J. SISTEMA DE AUDIO — 0% ❌

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| DFPlayer Mini (alertas, cola de reproducción) | ❌ | Falta | No existe código DFPlayer, audio, mp3, ni buzzer en ningún MCU |

### K. GESTIÓN DE ENCENDIDO — 0% ❌

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Detección llave de contacto (GPIO 40/41) | ❌ | Falta | No hay código para leer llave de contacto. STM32 depende del heartbeat CAN del ESP32 |
| Etapas de potencia (OFF→POWER_HOLD→CENTERING→AUX→FULL→SHUTDOWN) | ❌ | Falta | No existe máquina de estados de potencia. Los relés se activan directamente al entrar en ACTIVE |
| Secuencia de apagado con audio | ❌ | Falta | No existe lógica de shutdown. Al perder alimentación, IWDG provoca reset |

### L. ADQUISICIÓN DE SENSORES — 100% ✅

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Velocidad ruedas (4× LJ12A3) | ✅ STM32 | Equivalente | `sensor_manager.c`: EXTI con debounce 1 ms, 6 pulsos/rev |
| Corriente (6× INA226 vía TCA9548A) | ✅ STM32 | Mejorado | `sensor_manager.c`: + recuperación bus I2C (NXP AN10216) |
| Temperatura (5× DS18B20) | ✅ STM32 | Equivalente | `sensor_manager.c`: OneWire bit-bang, CRC-8, ROM search |
| Pedal (doble canal) | ✅ STM32 | Mejorado | `sensor_manager.c`: ADC PA3 primario + ADS1115 I2C plausibilidad cruzada ±5% |
| Encoder dirección (E6B2-CWZ6C) | ✅ STM32 | Mejorado | `encoder_reader.c`: TIM2 cuadratura 4800 CPR + detección salud (rango/salto/congelado) |

### M. COMUNICACIÓN CAN — 100% ✅

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Protocolo de mensajes | ✅ | Mejorado | `can_handler.c` + `can_rx.cpp`: protocolo v1.3 con 13 TX + 7 RX IDs formalizados |
| Heartbeat bidireccional | ✅ | Equivalente | STM32 TX 0x001 cada 100 ms, ESP32 TX 0x011 cada 100 ms, timeout 250 ms |
| Telemetría (velocidad, corriente, temp, seguridad) | ✅ | Mejorado | 8 mensajes STATUS (0x200-0x207) cubren toda la telemetría |
| Comandos (acelerador, dirección, modo) | ✅ | Mejorado | 0x100/0x101/0x102 con ACK (0x103) |

### N. DIAGNÓSTICO / SERVICIO — 100% ✅

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Clasificación de módulos | ✅ STM32 | Equivalente | `service_mode.c`: 25 módulos CRITICAL/NON-CRITICAL |
| Seguimiento de fallos por módulo | ✅ STM32 | Equivalente | `service_mode.c`: estados NONE/WARNING/ERROR/DISABLED |
| Comandos CAN de servicio | ✅ STM32 | Equivalente | `can_handler.c`: RX 0x110 enable/disable/factory-restore, TX 0x301-0x303 |

### O. PERSISTENCIA — 25% ⚠️

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Almacenamiento de configuración | ⚠️ STM32 | Parcial | `steering_cal_store.c`: flash página 126 para centro dirección. `eps_params.c`: flash página 127 para EPS. Pero NO hay NVS/EEPROM en ESP32 para configuración HMI |
| Persistencia de log de errores | ❌ | Falta | No existe código para guardar historial de errores en ningún MCU |

### P. ENTRADAS FÍSICAS — 0% ❌

| Función original | Existe en migrado | Estado | Evidencia |
|---|---|---|---|
| Palanca de marchas (MCP23017) | ❌ | Falta | Original leía posición física del selector vía I2C MCP23017. ESP32 no tiene código para MCP23017. Las marchas llegan por CAN 0x102 byte 1 pero no hay código que genere ese comando desde hardware |
| Botones/interruptores de dirección | ❌ | Falta | Original tenía entradas GPIO para dirección. No hay código de lectura de botones físicos en ESP32 |

---

## 3. DETECCIÓN DE COMPORTAMIENTOS

### Comportamientos correctamente sustituidos
| Original | Migrado | Justificación |
|---|---|---|
| PCA9685 I2C PWM (250 Hz) | TIM1/TIM8 directo (20 kHz) | Mejora: menor latencia, mayor resolución, sin contención I2C |
| Máquina estados 4 niveles | Máquina estados 7 niveles | Mejora: LIMP_HOME separado de SAFE, transiciones más granulares |
| PID posicional de dirección | EPS torque-assist | Mejora: comportamiento más natural con detección de intención del conductor |
| Monolítico ESP32 | STM32 (control) + ESP32 (HMI) | Mejora: elimina bootloops documentados (30+), aísla motor de display |
| TOFSense-M S LiDAR | HC-SR04 ultrasónico | Sustitución de hardware: menos resolución pero funcional para distancia frontal |

### Comportamientos rotos
| Comportamiento | Descripción | Impacto |
|---|---|---|
| Entrada táctil | XPT2046 configurado en `platformio.ini` pero ningún código lee touch | El usuario no puede interactuar con la pantalla. Los mode_icons tienen hitTest() pero no se llama |

### Comportamientos cambiados
| Comportamiento | Original | Migrado | Impacto |
|---|---|---|---|
| Zonas de obstáculo | 5 zonas (200/500/1000/1500/4000 mm) | 3 zonas (200/500/1000 mm) | Menor rango de aviso previo. Sin zona Caution ni Alert |
| ABS modulación | 30% reducción de presión | Corte completo por rueda | Más agresivo pero efectivo |
| Arranque del sistema | Llave de contacto → etapas de potencia | Alimentación directa → CAN heartbeat | Sin secuencia de encendido controlada |

### Sistemas faltantes reales
| Sistema | Impacto en equivalencia funcional |
|---|---|
| Driver TOFSense / 5 zonas de obstáculo completas | Reducción de capacidad de detección anticipada |
| Reacción infantil en obstáculos | Sin adaptación de respuesta cuando el niño suelta el pedal |
| Crucero adaptativo (ACC) | Sin seguimiento automático de distancia |
| Frenado regenerativo | Sin recuperación de energía (solo frenado disipativo) |
| Entrada táctil funcional | Sin interacción de usuario con pantalla |
| Menú de ingeniería oculto | Sin calibración/diagnóstico avanzado accesible |
| Iluminación LED WS2812B | Sin señalización visual del vehículo |
| Audio DFPlayer Mini | Sin alertas acústicas |
| Llave de contacto y gestión de potencia | Sin secuencia de encendido/apagado controlada |
| Palanca de marchas física (MCP23017) | Sin entrada hardware para cambio de marchas |
| Persistencia NVS en ESP32 | Sin guardar configuración HMI |
| Persistencia de log de errores | Sin historial de diagnóstico |

### Sistemas eliminados intencionalmente
| Sistema | Justificación documentada |
|---|---|
| WiFi / OTA | Eliminado en v2.11.0 del original por endurecimiento de seguridad |

---

## 4. LISTA ÚNICA DE SISTEMAS FALTANTES

Ordenados por dependencia y prioridad para alcanzar equivalencia funcional.

| # | Sistema faltante | Prioridad | MCU destino |
|---|---|---|---|
| 1 | Lectura de palanca de marchas (MCP23017) | CRÍTICA | ESP32 |
| 2 | Llave de contacto y gestión de potencia | CRÍTICA | ESP32 |
| 3 | Entrada táctil funcional (XPT2046) | ALTA | ESP32 |
| 4 | 5 zonas de obstáculo completas | ALTA | STM32 |
| 5 | Detección de reacción infantil | ALTA | STM32 |
| 6 | Iluminación LED (WS2812B) | MEDIA | ESP32 |
| 7 | Audio (DFPlayer Mini) | MEDIA | ESP32 |
| 8 | Menú de ingeniería oculto | MEDIA | ESP32 |
| 9 | Persistencia NVS en ESP32 | MEDIA | ESP32 |
| 10 | Persistencia de log de errores | MEDIA | STM32+ESP32 |
| 11 | Crucero adaptativo (ACC) | BAJA | ESP32 |
| 12 | Frenado regenerativo | BAJA | STM32 |

---

## 5. PLAN ÚNICO DE IMPLEMENTACIÓN

### Paso 1 — Lectura de palanca de marchas (MCP23017)

**Qué restaura:** La capacidad de leer la posición física del selector de marchas (P/R/N/D1/D2) desde hardware. Sin esto, el ESP32 no puede enviar el comando CAN 0x102 con la marcha real del usuario.

**Archivos afectados:**
- `esp32/src/shifter_input.cpp` (nuevo) — Driver MCP23017 vía I2C
- `esp32/src/shifter_input.h` (nuevo) — Interfaz del driver
- `esp32/src/main.cpp` — Integrar lectura del shifter en loop, enviar CAN 0x102
- `esp32/platformio.ini` — Añadir lib_dep si se usa librería MCP23017

**Dependencias:** Hardware MCP23017 conectado a I2C del ESP32. Pines I2C del ESP32 deben definirse.

**Verificación:**
1. Mover la palanca física a cada posición (P, R, N, D1, D2)
2. Verificar con analizador CAN que se transmite 0x102 con byte 1 = {0,1,2,3,4} respectivamente
3. Verificar en STM32 que `Traction_SetGear()` recibe y aplica cada marcha
4. Verificar rechazo de cambio cuando velocidad > 1 km/h

---

### Paso 2 — Llave de contacto y gestión de potencia

**Qué restaura:** La secuencia de encendido/apagado controlada: detección de llave (GPIO 40/41), etapas de potencia (OFF → POWER_HOLD → CENTERING → AUX_POWER → FULL_POWER → SHUTDOWN), y secuencia de apagado segura.

**Archivos afectados:**
- `esp32/src/power_manager.cpp` (nuevo) — Máquina de estados de potencia
- `esp32/src/power_manager.h` (nuevo) — Interfaz
- `esp32/src/main.cpp` — Integrar power_manager en boot y loop
- `esp32/include/can_ids.h` — Añadir IDs CAN para comandos de potencia si necesario

**Dependencias:** Paso 1 completado (MCP23017 para leer posición P al arranque). GPIO 40 y 41 del ESP32 libres y cableados a llave.

**Verificación:**
1. Girar llave a posición ON → sistema arranca con secuencia completa
2. Verificar que cada etapa de potencia se ejecuta en orden con delays correctos
3. Girar llave a OFF → secuencia de apagado (relés se desactivan en orden inverso)
4. Verificar que sin llave el sistema no arranca (solo ESP32 en standby)

---

### Paso 3 — Entrada táctil funcional (XPT2046)

**Qué restaura:** La interacción del usuario con la pantalla TFT: selección de modos (4×2/4×4/tank turn), navegación entre pantallas, y base para el menú de ingeniería.

**Archivos afectados:**
- `esp32/src/touch_handler.cpp` (nuevo) — Lectura XPT2046, debounce, mapeo coordenadas
- `esp32/src/touch_handler.h` (nuevo) — Interfaz
- `esp32/src/main.cpp` — Integrar touch_handler en loop
- `esp32/src/ui/mode_icons.cpp` — Conectar `hitTest()` existente con touch real
- `esp32/src/screen_manager.cpp` — Pasar eventos touch a pantalla activa

**Dependencias:** XPT2046 ya configurado en `platformio.ini` (CS=GPIO21, freq=2.5 MHz). TFT_eSPI soporta touch nativamente.

**Verificación:**
1. Tocar icono 4×4 en drive_screen → verificar CAN 0x102 bit 0 cambia
2. Tocar icono 360° → verificar CAN 0x102 bit 1 cambia
3. Verificar debounce (toques rápidos no generan múltiples comandos)
4. Verificar que touch no interfiere con frame rate (20 FPS mantenido)

---

### Paso 4 — 5 zonas de obstáculo completas

**Qué restaura:** Las 5 zonas de colisión del original con sus factores de velocidad:
- Zona 5 (Emergencia): <200 mm → factor 0.0
- Zona 4 (Crítica): 200–500 mm → factor 0.1–0.4 (lineal)
- Zona 3 (Aviso): 500–1000 mm → factor 0.4–0.7
- Zona 2 (Precaución): 1000–1500 mm → factor 0.85–1.0
- Zona 1 (Alerta): 1500–4000 mm → factor 1.0 (solo aviso)

**Archivos afectados:**
- `Core/Src/safety_system.c` — Ampliar lógica de 3 a 5 zonas con factores del original
- `Core/Inc/safety_system.h` — Añadir umbrales OBSTACLE_CAUTION_MM=1500, OBSTACLE_ALERT_MM=4000 y factores correspondientes
- `esp32/src/sensors/obstacle_sensor.cpp` — Actualizar mapeo de zonas a 5 (campo zone en CAN 0x208)
- `esp32/src/ui/obstacle_sensor.cpp` — Actualizar display de proximidad para 5 zonas

**Dependencias:** Ninguna. La infraestructura CAN 0x208 y el campo obstacle_scale ya existen.

**Verificación:**
1. Colocar obstáculo a cada distancia umbral y verificar factor de velocidad correcto
2. Verificar interpolación lineal en zona 4 (200-500 mm)
3. Verificar que zona 1 (>1500 mm) no reduce velocidad pero sí muestra aviso en pantalla
4. Verificar histéresis temporal (200 ms confirmación, 1000 ms limpieza)
5. Verificar que zona 5 sigue produciendo parada completa (factor 0.0)

---

### Paso 5 — Detección de reacción infantil

**Qué restaura:** Cuando el niño detecta un obstáculo y suelta el pedal (reducción >10% en 500 ms), el sistema modifica la respuesta en zonas 2-3 (reduce el factor de velocidad adicional por acción cooperativa del conductor).

**Archivos afectados:**
- `Core/Src/safety_system.c` — Añadir lógica de detección de soltar pedal rápido y modificar factores zonas 2-3
- `Core/Inc/safety_system.h` — Constantes: CHILD_REACTION_THRESHOLD=10%, CHILD_REACTION_WINDOW_MS=500

**Dependencias:** Paso 4 completado (5 zonas implementadas). `Pedal_GetPercent()` ya disponible en `sensor_manager.c`.

**Verificación:**
1. Con obstáculo en zona 3 (500-1000 mm), soltar pedal >10% en <500 ms → verificar factor ajustado
2. Con obstáculo en zona 2 (1000-1500 mm), soltar pedal >10% en <500 ms → verificar factor ajustado
3. Verificar que en zona 5/4 (emergencia/crítica) la reacción infantil no modifica comportamiento
4. Verificar que pedal lento (>500 ms) no activa la lógica de reacción

---

### Paso 6 — Iluminación LED (WS2812B)

**Qué restaura:** Control de 28 LEDs frontales + 16 LEDs traseros WS2812B con patrones dependientes del estado del vehículo (encendido, marcha, freno, error, giro).

**Archivos afectados:**
- `esp32/src/led_controller.cpp` (nuevo) — Driver WS2812B con patrones por estado
- `esp32/src/led_controller.h` (nuevo) — Interfaz
- `esp32/src/main.cpp` — Integrar led_controller, actualizar según estado del vehículo
- `esp32/platformio.ini` — Añadir lib_dep FastLED o Adafruit_NeoPixel

**Dependencias:** GPIO del ESP32 disponible para data line WS2812B. Estado del vehículo disponible vía `VehicleData`.

**Verificación:**
1. Estado ACTIVE → luces de posición frontales encendidas
2. Frenado (pedal baja) → luces traseras rojas intensas
3. Estado SAFE/ERROR → patrón de emergencia (parpadeo)
4. Marcha atrás → luces traseras blancas
5. Verificar que LEDs no interfieren con timing CAN ni display

---

### Paso 7 — Audio (DFPlayer Mini)

**Qué restaura:** Alertas acústicas mediante DFPlayer Mini: sonido de encendido, aviso de obstáculo por zona, alerta de error, aviso de batería baja, confirmación de cambio de marcha.

**Archivos afectados:**
- `esp32/src/audio_manager.cpp` (nuevo) — Driver DFPlayer Mini vía UART, cola de reproducción
- `esp32/src/audio_manager.h` (nuevo) — Interfaz con prioridades
- `esp32/src/main.cpp` — Integrar audio_manager, disparar sonidos según eventos

**Dependencias:** Paso 2 completado (gestión de potencia para secuencia de encendido con audio). UART del ESP32 disponible para DFPlayer. Archivos MP3 en tarjeta SD.

**Verificación:**
1. Encendido → sonido de bienvenida
2. Obstáculo zona 3-4 → aviso acústico proporcional
3. Error/SAFE → alerta sonora
4. Batería <20 V → aviso periódico
5. Verificar que reproducción es no bloqueante (cola con prioridades)

---

### Paso 8 — Menú de ingeniería oculto

**Qué restaura:** Acceso a calibración y diagnóstico avanzado mediante código secreto (8989): calibración de pedal, calibración de encoder, enable/disable de módulos individuales, factory restore, visor de log de errores.

**Archivos afectados:**
- `esp32/src/screens/engineering_screen.cpp` (nuevo) — Pantalla de menú con submenús
- `esp32/src/screens/engineering_screen.h` (nuevo) — Interfaz
- `esp32/src/screen_manager.cpp` — Añadir transición a engineering_screen por código secreto
- `esp32/src/touch_handler.cpp` — Añadir detección de secuencia 8989 en touch

**Dependencias:** Paso 3 completado (touch funcional). Paso 9 recomendado (NVS para guardar calibraciones).

**Verificación:**
1. Introducir código 8989 en pantalla → se abre menú de ingeniería
2. Calibración de pedal → verificar nuevos rangos aplicados en STM32 vía CAN 0x110
3. Enable/disable módulo → verificar CAN 0x110 SERVICE_CMD correcto
4. Factory restore → verificar que configuración vuelve a valores por defecto
5. Código incorrecto → no abre menú

---

### Paso 9 — Persistencia NVS en ESP32

**Qué restaura:** Almacenamiento no volátil de configuración del HMI: modo de conducción preferido, brillo de pantalla, umbrales de alerta personalizados, calibraciones del menú de ingeniería.

**Archivos afectados:**
- `esp32/src/config_store.cpp` (nuevo) — Wrapper NVS con validación CRC
- `esp32/src/config_store.h` (nuevo) — Interfaz
- `esp32/src/main.cpp` — Cargar configuración al arranque
- `esp32/src/screens/engineering_screen.cpp` — Guardar calibraciones en NVS

**Dependencias:** Paso 8 recomendado (menú de ingeniería genera datos a persistir).

**Verificación:**
1. Cambiar configuración → apagar → encender → verificar que configuración persiste
2. Corromper NVS manualmente → verificar que arranca con valores por defecto
3. Factory restore → verificar que NVS se limpia y vuelve a defaults

---

### Paso 10 — Persistencia de log de errores

**Qué restaura:** Historial de diagnóstico: registro de últimos N errores con timestamp, código de error, subsistema afectado y estado del sistema al momento del fallo. Consultable desde menú de ingeniería.

**Archivos afectados:**
- `Core/Src/error_logger.c` (nuevo) — Escritura circular en flash (última página disponible)
- `Core/Inc/error_logger.h` (nuevo) — Interfaz
- `Core/Src/safety_system.c` — Llamar a error_logger en cada cambio de estado de error
- `Core/Src/can_handler.c` — Añadir comando CAN para solicitar log
- `esp32/src/screens/engineering_screen.cpp` — Mostrar log de errores recibido vía CAN

**Dependencias:** Paso 8 completado (menú de ingeniería para visualización). Flash page disponible en STM32 (páginas 124-125 libres, 126 = steering cal, 127 = EPS).

**Verificación:**
1. Provocar error de sobrecorriente → verificar que se registra en flash
2. Reiniciar STM32 → solicitar log vía CAN → verificar que entrada persiste
3. Llenar log completo → verificar que sobrescribe entradas más antiguas (circular)
4. Verificar integridad CRC de cada entrada

---

### Paso 11 — Crucero adaptativo (ACC)

**Qué restaura:** Seguimiento automático de distancia con PID: mantiene 500 mm de distancia al obstáculo frontal. Emergencia a <300 mm. Timeout de objetivo 2000 ms. Coordina con sistema de obstáculos (ACC prioridad zonas 2-3, obstáculo override zonas 4-5).

**Archivos afectados:**
- `esp32/src/adaptive_cruise.cpp` (nuevo) — Controlador PID (Kp=0.3, Ki=0.05, Kd=0.15)
- `esp32/src/adaptive_cruise.h` (nuevo) — Interfaz
- `esp32/src/main.cpp` — Integrar ACC con sensor de obstáculo
- `esp32/include/can_ids.h` — Añadir CAN ID para comando ACC si necesario (podría usar CMD_THROTTLE 0x100 existente)

**Dependencias:** Paso 4 completado (5 zonas para coordinar ACC con obstáculo). Sensor de obstáculo funcional. Paso 3 recomendado (touch para activar/desactivar ACC).

**Verificación:**
1. Activar ACC → acercarse a obstáculo → verificar que vehículo mantiene ~500 mm
2. Obstáculo a <300 mm → parada completa inmediata
3. Obstáculo desaparece >2000 ms → ACC vuelve a standby
4. Verificar que obstáculo zona 4-5 tiene prioridad sobre ACC
5. Verificar estabilidad PID (sin oscilaciones)

---

### Paso 12 — Frenado regenerativo

**Qué restaura:** Recuperación de energía durante desaceleración mediante control de corriente inversa en motores. Lookup tables para corriente regen óptima según velocidad, aceleración, temperatura y tensión de batería. Contadores de energía recuperada (Wh).

**Archivos afectados:**
- `Core/Src/regen_braking.c` (nuevo) — Lógica regen con lookup tables
- `Core/Inc/regen_braking.h` (nuevo) — Interfaz y tablas
- `Core/Src/motor_control.c` — Integrar regen en pipeline de frenado (reemplazar/complementar frenado dinámico)
- `Core/Inc/safety_system.h` — Añadir umbrales de protección de batería para regen (voltaje máximo, corriente máxima regen)
- `Core/Src/safety_system.c` — Verificar que regen se inhibe si batería está llena o en fallo

**Dependencias:** `Voltage_GetBus()` ya existe en `sensor_manager.c`. `Current_GetAmps()` disponible para monitoreo. Requiere verificar que BTS7960 soporta corriente inversa de forma segura.

**Verificación:**
1. Acelerar y soltar pedal → verificar corriente negativa en INA226 del motor
2. Verificar que voltaje de batería no supera umbral máximo durante regen
3. Batería con voltaje alto → regen inhibido automáticamente
4. Temperatura motor alta → regen reducido progresivamente
5. Velocidad <3 km/h → regen desactivado (solo frenado mecánico)
6. Verificar contadores de energía recuperada en telemetría CAN

---

## 6. RESUMEN EJECUTIVO

### Estado actual
El firmware migrado cubre el **100% del control de vehículo crítico para seguridad** (tracción, dirección, frenos, ABS/TCS, máquina de estados, sensores, CAN). La arquitectura dual STM32+ESP32 es superior al monolítico original al eliminar los bootloops documentados (30+) y aislar el control de motor del display.

### Brechas principales
Las brechas están concentradas en **periféricos del ESP32**: entradas físicas (palanca de marchas, llave de contacto), periféricos de usuario (touch, LEDs, audio), y funciones avanzadas (ACC, regen). El STM32 solo requiere ajustes menores (5 zonas de obstáculo, reacción infantil, log de errores).

### Camino a equivalencia funcional
Completando los 12 pasos en orden se alcanza el 100% de equivalencia funcional con el firmware original. Los pasos 1-5 son prioritarios (entradas físicas y seguridad de obstáculos). Los pasos 6-10 restauran funcionalidad de usuario. Los pasos 11-12 implementan funciones avanzadas no críticas para seguridad.

| Tras completar | Equivalencia estimada |
|---|---|
| Pasos 1-2 | 80% |
| Pasos 3-5 | 88% |
| Pasos 6-10 | 96% |
| Pasos 11-12 | 100% |
