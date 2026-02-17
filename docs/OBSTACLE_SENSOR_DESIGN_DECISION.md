# Justificación Técnica — Sensor de Obstáculos en la ESP32-S3

**Fecha:** 2026-02-17
**Estado:** Documentación de decisión de diseño
**Alcance:** Justificación de la ubicación del sensor TOFSense-M S (ToF/LiDAR) en la ESP32-S3 en lugar de la STM32G474RE
**Referencia:** `docs/OBSTACLE_SYSTEM_ARCHITECTURE.md`, `docs/CAN_CONTRACT_FINAL.md` rev 1.1, `docs/SAFETY_SYSTEMS.md`

---

## 1. Función del sensor de obstáculos en el sistema

El sensor TOFSense-M S (LiDAR de tiempo de vuelo) cumple una **función de seguridad activa con múltiples capas**:

| Nivel | Función | Responsable |
|-------|---------|-------------|
| **Asistencia al conductor** | Muestra distancia frontal al obstáculo en pantalla TFT con zonas de color (verde >1.5 m, amarillo 0.5–1.5 m, rojo <0.5 m) | ESP32-S3 (HMI) |
| **Seguridad primaria** | Lógica de 5 zonas con reducción progresiva de velocidad, detección de reacción de niño, frenado de emergencia ESP32 | ESP32-S3 (lógica de obstáculo) |
| **Seguridad secundaria (backstop)** | Limitador de par de 3 niveles basado en distancia CAN: <200 mm → parada total + estado SAFE; 200–500 mm → 30% potencia; 500–1000 mm → 70% potencia | STM32G474RE (safety_system.c) |
| **Seguridad terciaria** | Máquina de estados de seguridad: transición a SAFE, corte de relés, watchdog independiente | STM32G474RE (relés + IWDG) |

El sensor **no es solo informativo ni solo de telemetría**: participa directamente en la cadena de decisiones de movimiento del vehículo a través de la variable `obstacle_scale` que multiplica el PWM base de todos los motores de tracción en `Traction_Update()`.

---

## 2. Por qué el sensor está conectado a la ESP32-S3 y no a la STM32

### 2.1 Compatibilidad de interfaz hardware

El TOFSense-M S se comunica por **UART a 921.600 bps** y transmite tramas con una matriz de 8×8 píxeles de profundidad. La ESP32-S3 dispone de múltiples UART hardware con buffers DMA y soporte nativo para estas velocidades. La STM32G474RE también soporta UART a esa velocidad, pero **todos sus periféricos UART disponibles ya están asignados** o requerirían reasignación de pines que impactaría otros subsistemas (el UART1 se usa para debug, y los restantes están comprometidos por el mapa de pines existente con TIM1, TIM2, TIM8, I²C1, FDCAN1 y ADC).

### 2.2 Carga computacional

El procesamiento del sensor involucra:

- Parsing de tramas UART a 921.6 kbps (15 Hz, ~64 bytes/trama con cabecera y checksum).
- Procesamiento de matriz 8×8 (64 celdas de profundidad) para extraer la distancia mínima relevante.
- Validación de checksum por trama.
- Lógica de 5 zonas con interpolación lineal entre umbrales.
- Detección de reacción de niño (análisis de patrón de movimiento).
- Cálculo de factor de reducción de velocidad (ACC PID).
- Generación de alertas audiovisuales (DFPlayer Mini + overlay TFT).

Esta carga —particularmente el procesamiento matricial y la lógica ACC— se ejecuta cómodamente en la ESP32-S3 (Xtensa LX7 dual-core @ 240 MHz) dentro de su loop principal Arduino a ~60 Hz. Migrar todo esto a la STM32G474RE (Cortex-M4 @ 170 MHz, single-core) **aumentaría la carga del bucle de control de 10 ms**, donde ya se ejecutan: `Obstacle_Update()`, `Traction_Update()`, `Steering_ControlLoop()`, PID de dirección, ABS/TCS, lectura de encoders, y gestión de relés. El riesgo de sobrepasar el deadline de 10 ms se volvería real.

### 2.3 Librerías y ecosistema

El sensor utiliza un protocolo UART propietario que se parsea directamente byte a byte. No existe una librería oficial HAL para STM32; el parsing se implementa sobre la API `Serial` de Arduino en la ESP32. En la STM32 se requeriría implementar un driver DMA-UART con máquina de estados para recepción asíncrona sin bloquear el bucle de control. Esto es factible pero representa un esfuerzo significativo sin beneficio funcional, ya que la ESP32 ya resuelve el problema.

### 2.4 Aislamiento de dominios de fallo

La arquitectura separa intencionalmente dos dominios:

```
┌────────────────────────────┐     CAN 500 kbps     ┌────────────────────────────┐
│       ESP32-S3             │◄──────────────────────►│      STM32G474RE           │
│  • Sensor ToF (UART)       │                        │  • PWM motores (TIM1/TIM8) │
│  • HMI (TFT SPI)           │                        │  • PID dirección (TIM2)    │
│  • Audio (DFPlayer)        │                        │  • Sensores corriente I²C  │
│  • Lógica obstáculo 5-zona │                        │  • Sensores temperatura    │
│  • ACC PID                 │                        │  • ABS/TCS                 │
│  Supervisory + HMI         │                        │  • Backstop obstáculo 3-tier│
│                            │                        │  • Máquina de estados      │
│                            │                        │  • Relés de potencia       │
│                            │                        │  Real-time control         │
└────────────────────────────┘                        └────────────────────────────┘
```

Si el procesamiento del sensor fallara (bug en el parsing, corrupción de trama, etc.), el fallo queda **contenido en la ESP32** y la STM32 lo detecta por timeout CAN o contador congelado. Si el sensor estuviera en la STM32 y su driver tuviera un bug (e.g., buffer overflow en recepción UART), podría corromper memoria del mismo microcontrolador que controla los motores, comprometiendo la seguridad de todo el sistema.

### 2.5 No requiere cambios de hardware

El sensor ya está físicamente cableado a la UART0 de la ESP32-S3. Moverlo a la STM32 requeriría:

- Recablear el sensor al conector de la STM32.
- Asignar un periférico UART libre (conflicto de pines potencial).
- Modificar la configuración CubeMX (.ioc).
- Implementar un driver UART-DMA completo en C bare-metal.

Ninguno de estos cambios aporta beneficio funcional: la arquitectura split ya garantiza la seguridad mediante la cadena CAN → backstop → máquina de estados → relés.

---

## 3. Qué ocurre si la ESP32 se reinicia, bloquea o pierde comunicación

El sistema está diseñado para operar de forma segura ante cualquier fallo de la ESP32. La STM32 aplica **tres mecanismos de detección independientes**:

### 3.1 Timeout de heartbeat CAN (250 ms)

La ESP32 envía un mensaje heartbeat (CAN ID 0x011) cada 100 ms. Si la STM32 no recibe este mensaje durante 250 ms:

- Transición inmediata al **estado SAFE**.
- Todos los motores se detienen (`obstacle_scale` pasa a ser irrelevante; el estado SAFE corta toda tracción).
- Los relés de potencia se abren.
- Este mecanismo es **independiente** del sistema de obstáculos.

### 3.2 Timeout de datos de obstáculo (500 ms)

Si la ESP32 deja de enviar mensajes 0x208 (OBSTACLE_DISTANCE) durante más de 500 ms tras haber recibido al menos uno:

- `obstacle_scale` = 0.0 (par motor = 0%).
- Transición al estado SAFE.
- Recuperación automática cuando los mensajes CAN se reanuden con contador válido, distancia > 500 mm durante > 1 segundo.

### 3.3 Detección de datos obsoletos (counter stale)

Si la ESP32 sigue enviando mensajes pero el rolling counter no incrementa durante ≥ 3 tramas consecutivas (indicativo de datos cacheados o loop bloqueado):

- `obstacle_scale` = 0.0.
- Transición al estado SAFE.

### 3.4 Resumen de modos de fallo

| Fallo de la ESP32 | Detección más rápida | Tiempo máximo de exposición | Respuesta |
|-------------------|---------------------|-----------------------------|-----------|
| Crash completo (reboot) | Heartbeat timeout | 250 ms | SAFE + relés abiertos |
| Loop bloqueado (freeze) | Counter stale (3 × 66 ms) | ~200 ms | SAFE + obstacle_scale = 0 |
| Pérdida de CAN bus | Heartbeat timeout | 250 ms | SAFE + relés abiertos |
| Sensor UART desconectado | ESP32 reporta `health = 0` | 66 ms (siguiente trama) | SAFE + obstacle_scale = 0 |
| Dato corrupto | Checksum fail en ESP32 (no se envía trama corrupta) | N/A | ESP32 descarta internamente |

**Conclusión:** el vehículo se detiene de forma segura en un máximo de 250 ms ante cualquier fallo de la ESP32. La STM32 nunca depende de la disponibilidad continua de la ESP32 para garantizar la seguridad.

---

## 4. ¿El sistema depende del sensor para decisiones críticas de movimiento?

**Sí, pero con salvaguardas de defensa en profundidad.**

### 4.1 Dependencia directa

La variable `obstacle_scale` se multiplica directamente en la fórmula de cálculo de PWM de tracción:

```
FinalPWM[i] = base_pwm × obstacle_scale × ackermann_diff[i] × wheel_scale[i]
```

Cuando `obstacle_scale = 0.0`, todos los motores de tracción reciben PWM = 0 independientemente de la posición del pedal. Esto convierte al sensor en un componente que **afecta directamente al movimiento**.

### 4.2 Independencia por capas

Sin embargo, el sensor **no es el único mecanismo de seguridad**. Aunque el sensor fallara por completo y la ESP32 nunca enviara un solo mensaje 0x208:

- La STM32 mantiene `obstacle_scale = 1.0` (sin reducción) hasta recibir el primer mensaje. Esto permite operar sin sensor instalado.
- Los sistemas ABS/TCS siguen activos (independientes del obstáculo).
- El heartbeat CAN sigue protegiendo contra crash total de la ESP32.
- Las protecciones de sobrecorriente, sobretemperatura y batería baja siguen activas.
- El watchdog hardware (IWDG 500 ms) reinicia la STM32 si su propio firmware se bloquea.
- Los relés de potencia se abren ante estado SAFE o ERROR.

### 4.3 Modo servicio

El módulo de detección de obstáculos (`MODULE_OBSTACLE_DETECT`, ID 24) puede desactivarse mediante comando CAN de servicio (0x110). Cuando está desactivado:

- `obstacle_scale` se fuerza a 1.0.
- No se realiza detección de timeout CAN para obstáculo.
- El operador asume la responsabilidad de vigilancia de obstáculos.

Esto confirma que el sistema **puede operar sin el sensor**, pero cuando está activo, participa en decisiones de movimiento.

---

## 5. Ventajas de la ubicación actual frente a conexión directa a la STM32

| Criterio | Sensor en ESP32-S3 (actual) | Sensor en STM32 (alternativa) |
|----------|----------------------------|-------------------------------|
| **Aislamiento de fallos** | Fallo del driver UART no puede corromper memoria del controlador de motores. Dominio de fallo separado. | Un bug en el driver UART comparte espacio de memoria con el control de motores, PID y ABS/TCS. |
| **Carga de CPU** | Procesamiento de matriz 8×8 + ACC PID en ESP32 dual-core a 240 MHz; la STM32 solo recibe 5 bytes CAN cada 66 ms. | El bucle de 10 ms de la STM32 (170 MHz single-core) absorbería todo el procesamiento adicional. Riesgo de overrun. |
| **Latencia de control** | ~70 ms end-to-end (66 ms sensor + ~4 ms CAN). Aceptable para obstáculos estáticos/lentos a las velocidades del vehículo (<10 km/h). | ~10 ms (directo en el bucle de control). Menor latencia pero innecesaria para el caso de uso. |
| **Cambios de hardware** | Ninguno. El sensor ya está cableado a la ESP32 UART0. | Requiere recablear el sensor, reasignar pines, y modificar la configuración CubeMX. |
| **Cambios de firmware** | ~200 líneas nuevas en C (STM32 backstop). La lógica del sensor ya existía en la ESP32. | Requiere implementar un driver UART-DMA completo, parser, lógica de zonas, y eliminación del código actual de la ESP32. Estimación: >800 líneas. |
| **Interfaz HMI** | La ESP32 tiene acceso directo al dato para renderizar el overlay de obstáculo en la pantalla TFT sin latencia adicional. | La STM32 debería enviar el dato procesado de vuelta a la ESP32 por CAN para visualización, añadiendo un mensaje CAN extra y latencia. |
| **Defensa en profundidad** | Tres capas independientes: (1) ESP32 lógica, (2) STM32 backstop, (3) STM32 máquina de estados + relés. | Dos capas: (1) lógica en STM32, (2) máquina de estados + relés. Se pierde una capa de redundancia al concentrar todo en un solo microcontrolador. |
| **Mantenibilidad** | Actualizaciones del algoritmo de obstáculo se despliegan en la ESP32 (Arduino/PlatformIO) sin tocar el firmware crítico de la STM32. | Cualquier cambio en la lógica del sensor requiere recompilar y reflashear la STM32, con riesgo de regresión en el control de motores. |
| **Compatibilidad retroactiva** | La STM32 opera normalmente si nunca recibe un mensaje 0x208 (grace period). Permite usar firmware ESP32 sin módulo de obstáculo. | N/A — el sensor estaría siempre presente en el firmware. |

### Resumen

La ubicación del sensor en la ESP32-S3 maximiza el **aislamiento de dominios de fallo**, minimiza la **carga sobre el controlador de tiempo real**, mantiene **tres capas de defensa en profundidad**, y no requiere ningún cambio de hardware. La latencia adicional de ~70 ms es aceptable para las velocidades operativas del vehículo. La STM32 retiene la **autoridad final de seguridad** a través de su backstop de 3 niveles, timeout CAN, detección de datos obsoletos y control directo de relés de potencia.

---

*Documento generado: 2026-02-17*
*Este documento justifica la decisión de diseño sin proponer modificaciones al sistema existente.*
