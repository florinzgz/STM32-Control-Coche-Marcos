# Procedimiento de Validación de Arranque Seguro

**Versión:** 1.0  
**Fecha:** 2026-02-23  
**Firmware:** STM32G474RE — ~89% funcional, persistencia de centrado implementada  
**Requisito:** Vehículo con ruedas en el aire (caballetes/elevador) para TODAS las pruebas estáticas  
**Idioma:** Español (documentación operativa de garaje)

---

## Índice

1. [Resumen del Estado del Firmware](#1-resumen-del-estado-del-firmware)
2. [Inventario de Condiciones de Par Inesperado](#2-inventario-de-condiciones-de-par-inesperado)
3. [Secuencia de Pruebas Físicas](#3-secuencia-de-pruebas-físicas)
4. [Criterios PASS/FAIL Medibles](#4-criterios-passfail-medibles)
5. [Señales CAN a Observar](#5-señales-can-a-observar)
6. [Criterios de Prohibición de Pruebas Dinámicas (NO-GO)](#6-criterios-de-prohibición-de-pruebas-dinámicas-no-go)

---

## 1. Resumen del Estado del Firmware

### 1.1 Arquitectura de Control

| Componente | Rol | Enlace |
|---|---|---|
| STM32G474RE | Autoridad de seguridad, control de motores, lectura de sensores | CAN 500 kbps |
| ESP32-S3 | HMI, pantalla, envío de comandos | CAN 500 kbps |

### 1.2 Estados del Sistema

| Estado | Descripción | Motores |
|---|---|---|
| BOOT | Inicialización de periféricos, centrado de dirección | Deshabilitados |
| STANDBY | Periféricos listos, esperando heartbeat ESP32 | Deshabilitados |
| ACTIVE | Control completo habilitado | Habilitados |
| DEGRADED | Fallo no crítico, capacidad reducida | Limitados |
| LIMP_HOME | Sin CAN, pedal local al 20% máx. | 20% máx. |
| SAFE | Fallo crítico, motores cortados, relés abiertos | Cortados |
| ERROR | Estado terminal, requiere reset | Cortados |

### 1.3 Protecciones de Arranque Implementadas

- **Inhibición de pedal al arranque** (`startup_inhibit`): Pedal debe estar <3% durante 400 ms continuos tras encendido para desbloquear tracción.
- **Validación de boot** (`boot_validation`): 6 chequeos obligatorios antes de transición a ACTIVE.
- **Centrado persistente**: Posición central almacenada en flash, validada contra sensor inductivo al arranque.
- **Watchdog IWDG**: ~4.1 s timeout — reset automático si el bucle principal se congela.
- **Secuencia de relés**: Main → 50 ms → Tracción → 20 ms → Dirección (previene picos de corriente).

### 1.4 Umbrales de Seguridad Críticos

| Parámetro | Umbral | Acción |
|---|---|---|
| Corriente motor | >25 A | → SAFE |
| Temperatura motor | >90 °C | → SAFE |
| Timeout CAN heartbeat | >250 ms | → SAFE |
| Timeout watchdog | >4.1 s | → BOOT (reset MCU) |
| Batería baja (warning) | <20.0 V | → DEGRADED |
| Batería baja (crítico) | <18.0 V | → SAFE |
| Pedal contradictorio | Dual-sample divergen >30 counts o rango/tasa inválido | → Demand=0 |
| Encoder perdido | Sin actualización 100 ms | → SAFE |

---

## 2. Inventario de Condiciones de Par Inesperado

Las siguientes condiciones pueden provocar movimiento no deseado de las ruedas. Cada una debe verificarse antes de pasar a pruebas con ruedas en el suelo.

### 2.1 Condiciones de Par en Ruedas de Tracción

| # | Condición | Mecanismo | Gravedad |
|---|---|---|---|
| T1 | Pedal pulsado al encender | Si `startup_inhibit` falla, el pedal genera demanda inmediata | **CRÍTICA** |
| T2 | Offset ADC del pedal | Lectura ADC >3% sin pisar → demanda fantasma | **ALTA** |
| T3 | Plausibilidad software falla + ADC con offset | Sin canal de plausibilidad, el offset pasa directamente | **ALTA** |
| T4 | Transición a LIMP_HOME con pedal pisado | Si `limp_home_pedal_armed` no se resetea correctamente | **ALTA** |
| T5 | Comando CAN 0x100 espurio | ESP32 envía throttle >0 sin intención del usuario | **MEDIA** |
| T6 | Cambio de marcha en movimiento | Gear PARK→FORWARD con ruedas girando | **ALTA** |
| T7 | Creep torque por zona muerta | `MOTOR_DEADZONE` 8% puede generar creep si demanda supera umbral | **MEDIA** |
| T8 | Recuperación de watchdog con pedal pisado | Reset IWDG → BOOT → si pedal sigue pisado y `startup_inhibit` falla | **CRÍTICA** |
| T9 | Fallo en ramp-down | `PEDAL_RAMP_DOWN` 100%/s no actúa → PWM no baja | **ALTA** |
| T10 | NaN/Inf en cálculo de demanda | Float corrompido genera PWM máximo | **ALTA** |

### 2.2 Condiciones de Par en Dirección

| # | Condición | Mecanismo | Gravedad |
|---|---|---|---|
| D1 | Centrado activo con manos en volante | Motor de dirección gira a 10% PWM durante sweep automático al encender. Comportamiento normal del centering, pero las manos NO deben estar en el volante durante el arranque. Advertir al ocupante. | **MEDIA** |
| D2 | Fallo de sensor central | Centering no encuentra centro, sigue girando | **ALTA** |
| D3 | Encoder desconectado durante centering | Sin feedback, motor gira hasta stall timeout (300 ms) | **ALTA** |
| D4 | Comando CAN 0x101 con ángulo extremo | Dirección salta a ±45° instantáneamente | **MEDIA** |
| D5 | PID de dirección con ganancia excesiva | Kp=0.09 en espacio de encoder, posible overshoot | **BAJA** |
| D6 | Calibración flash corrupta | Centro restaurado erróneo, volante descentrado | **MEDIA** |

### 2.3 Condiciones de Par por Estado del Sistema

| # | Condición | Mecanismo | Gravedad |
|---|---|---|---|
| S1 | ESP32 envía comandos antes de ACTIVE | STM32 en STANDBY aceptando throttle | **ALTA** |
| S2 | Transición SAFE→ACTIVE sin validación | Si boot_validation se omite por error | **CRÍTICA** |
| S3 | Service mode deshabilita módulo crítico | Módulo de seguridad deshabilitado vía CAN 0x110 | **ALTA** |
| S4 | Bus-Off CAN con motores activos | Pérdida de comunicación, demanda previa se mantiene | **ALTA** |

---

## 3. Secuencia de Pruebas Físicas

> **REQUISITO ABSOLUTO**: Todas las pruebas de la Fase 1 a la Fase 4 se ejecutan con el vehículo elevado y las 4 ruedas en el aire.

### Equipamiento Necesario

- Caballetes o elevador para 4 ruedas en el aire
- Analizador CAN (PCAN-USB, CANalyzer, o similar) conectado al bus
- Multímetro con pinza amperimétrica (≥30 A DC)
- Termómetro infrarrojo
- Fuente de alimentación regulada 24V / 30A (o batería cargada >23 V)
- Seta de emergencia externa cableada (corta alimentación general)
- Cronómetro
- Persona de seguridad junto a la seta de emergencia durante TODAS las pruebas

---

### FASE 1 — Validación Pre-Energización (Sin alimentación)

**Objetivo:** Verificar integridad de cableado antes de energizar.

| Paso | Acción | Verificar | PASS | FAIL |
|---|---|---|---|---|
| 1.1 | Inspeccionar visualmente TODO el cableado de potencia | Sin cables pelados, conectores firmes, fusibles correctos | Sin anomalías | Cualquier cable suelto o pelado |
| 1.2 | Medir continuidad motor FL → driver BTS7960 FL | Continuidad <1 Ω entre motor y driver | <1 Ω | >5 Ω o circuito abierto |
| 1.3 | Repetir 1.2 para FR, RL, RR, Steering | Cada motor correctamente conectado a su driver | Los 5 motores OK | Cualquiera falla |
| 1.4 | Verificar que relé principal está ABIERTO (sin alimentación) | Sin tensión en bus de tracción | 0 V en bus | >0 V |
| 1.5 | Medir tensión de batería | Batería cargada | >23 V | <20 V |
| 1.6 | Verificar conexión CAN: resistencia entre CAN_H y CAN_L | 2× terminadores 120 Ω en paralelo | ~60 Ω | <50 Ω o >80 Ω |
| 1.7 | Verificar que la seta de emergencia corta alimentación | Pulsar seta, medir 0 V en bus | 0 V con seta pulsada | >0 V con seta pulsada |
| 1.8 | Verificar sensor inductivo de centro (LJ12A3 en PB5) | Sensor montado, LED se activa al pasar metal | LED se enciende | Sin reacción |
| 1.9 | Verificar encoders de rueda (4× LJ12A3) | Sensores montados, gap correcto (~1-2 mm) | Todos montados | Alguno ausente o mal posicionado |
| 1.10 | Verificar encoder de dirección (TIM2) | Conector firme en PA0/PA1 | Conector firme | Suelto o ausente |

**GATE 1: Todos los pasos 1.1–1.10 deben ser PASS para continuar.**

---

### FASE 2 — Primer Encendido y Validación de Boot (Ruedas en el aire)

**Objetivo:** Verificar que el arranque es seguro y no genera movimiento.

| Paso | Acción | Verificar | PASS | FAIL |
|---|---|---|---|---|
| 2.1 | **NO pisar pedal.** Persona de seguridad junto a seta. Energizar STM32 | Ninguna rueda se mueve, dirección no gira | 0 movimiento durante 5 s | Cualquier movimiento |
| 2.2 | Observar CAN: mensaje 0x001 (heartbeat STM32) | Heartbeat presente, estado = BOOT luego STANDBY | Estado cambia BOOT→STANDBY en <2 s | Sin heartbeat o estado incorrecto |
| 2.3 | Observar CAN 0x001 byte 2 (fault_flags) | Sin fallos críticos al arranque | fault_flags = 0x00 | fault_flags ≠ 0x00 |
| 2.4 | Observar CAN 0x204 (steering status) | Centrado completado o restaurado de flash | `calibrated` = 1 | `calibrated` = 0 después de 15 s |
| 2.5 | Observar CAN 0x207 (battery) | Tensión de batería correcta | >20.0 V | <20.0 V |
| 2.6 | Observar CAN 0x202 (temperaturas) | Todas las temperaturas plausibles | Todas entre -10 °C y +50 °C (ambiente) | Alguna = 0.0 o >85 °C |
| 2.7 | Observar CAN 0x201 (corrientes) | Corrientes en reposo | Todas <1 A | Alguna >2 A sin carga |
| 2.8 | Verificar con multímetro: 0 V en bornes de cada motor | PWM al 0%, sin tensión residual | 0 V (±0.5 V) en los 5 motores | >1 V en algún motor |
| 2.9 | Esperar 10 s completos. Monitorear ruedas | Confirmar que `startup_inhibit` mantiene par en cero | 0 movimiento durante 10 s | Cualquier movimiento |
| 2.10 | Verificar en CAN 0x001 byte 3 (error_code) | Sin errores persistentes | error_code = 0 | error_code ≠ 0 |

**GATE 2: Todos los pasos 2.1–2.10 deben ser PASS para continuar.**

---

### FASE 3 — Validación de Inhibición de Pedal (Ruedas en el aire)

**Objetivo:** Confirmar que `startup_inhibit` y la plausibilidad de pedal funcionan correctamente.

| Paso | Acción | Verificar | PASS | FAIL |
|---|---|---|---|---|
| 3.1 | Apagar sistema. Pisar pedal a fondo. Encender sistema | Ruedas NO giran pese a pedal pisado | 0 movimiento durante 10 s | Cualquier movimiento |
| 3.2 | Mantener pedal pisado. Observar CAN 0x100 (si ESP32 conectado) | Throttle rechazado por `startup_inhibit` | demand = 0 en CAN 0x205 | demand > 0 en CAN 0x205 |
| 3.3 | Soltar pedal completamente. Esperar 500 ms | `startup_inhibit` debe desactivarse | — | — |
| 3.4 | Pisar pedal lentamente (~5%) | Ruedas deben girar (si en ACTIVE) | Ruedas giran suavemente | Sin movimiento o movimiento brusco |
| 3.5 | Soltar pedal completamente | Ruedas se detienen | Ruedas paran en <2 s | Ruedas siguen girando >3 s |
| 3.6 | Forzar fallo de plausibilidad (señal fuera de rango). Pisar pedal | Pedal debe rechazar señal y forzar 0% | Ruedas giran con advertencia en CAN | Sin movimiento o sin advertencia |
| 3.7 | Restaurar señal normal. Pisar pedal al 100% | Corriente por debajo de límite (ruedas en aire = sin carga) | Corriente <10 A por motor | >25 A (fallo de limitación) |
| 3.8 | Verificar rampa de subida: pisar pedal de 0→100% bruscamente | PWM sube gradualmente (50%/s ramp) | Transición suave ≥1 s hasta 100% | Salto instantáneo |

**GATE 3: Todos los pasos 3.1–3.8 deben ser PASS para continuar.**

---

### FASE 4 — Validación de Protecciones de Seguridad (Ruedas en el aire)

**Objetivo:** Provocar cada condición de seguridad y verificar la respuesta.

#### 4A — Timeout CAN (Simulación de pérdida de ESP32)

| Paso | Acción | Verificar | PASS | FAIL |
|---|---|---|---|---|
| 4A.1 | Con motor girando al 20%, desconectar ESP32 del bus CAN | Motores se detienen en <500 ms | Motores paran, estado → SAFE | Motores siguen girando |
| 4A.2 | Observar CAN 0x001 | Estado = SAFE, fault = CAN_TIMEOUT | Estado SAFE visible | Estado no cambia |
| 4A.3 | Reconectar ESP32. Enviar heartbeat | Sistema se recupera a STANDBY→ACTIVE | Recuperación en <5 s | Sin recuperación |

#### 4B — Watchdog (Simulación de congelamiento)

| Paso | Acción | Verificar | PASS | FAIL |
|---|---|---|---|---|
| 4B.1 | Si es posible con debugger: pausar ejecución del STM32 >4.1 s | MCU se resetea automáticamente (IWDG) | Reset detectado en CAN (heartbeat se pierde y reaparece) | Sin reset |
| 4B.2 | Tras reset por watchdog, verificar `startup_inhibit` | Pedal bloqueado hasta soltar y esperar 400 ms | 0 movimiento tras reset | Movimiento inmediato |

#### 4C — Centrado de Dirección

| Paso | Acción | Verificar | PASS | FAIL |
|---|---|---|---|---|
| 4C.1 | Girar volante a tope izquierdo manualmente (sin alimentación). Encender sistema | Centrado automático: motor mueve dirección al centro | Dirección centra en <10 s | Timeout o dirección en tope |
| 4C.2 | Observar CAN 0x204 durante centrado | Ángulo varía hasta `calibrated` = 1 | Secuencia sweep visible | Sin movimiento de dirección |
| 4C.3 | Apagar y re-encender sin mover volante | Centrado se restaura de flash (sin sweep) | `calibrated` = 1 inmediato, sin movimiento de motor | Sweep completo de nuevo |
| 4C.4 | Desconectar sensor inductivo de centro. Encender | Centrado falla → estado DEGRADED | SAFETY_ERROR_CENTERING en CAN 0x300 | Sin error reportado |

#### 4D — Protección de Corriente

| Paso | Acción | Verificar | PASS | FAIL |
|---|---|---|---|---|
| 4D.1 | Con ruedas en el aire: bloquear una rueda manualmente con una cuerda. Pisar pedal | Corriente sube. Si >25 A → sistema corta | Transición a SAFE si >25 A | Motor sigue forzando |
| 4D.2 | Observar CAN 0x201 durante la prueba | Corriente del motor bloqueado sube progresivamente | Lectura coherente en CAN | Lectura fija o 0 A |

#### 4E — Cambio de Marcha

| Paso | Acción | Verificar | PASS | FAIL |
|---|---|---|---|---|
| 4E.1 | Con ruedas girando a >2 km/h: enviar comando CAN 0x102 cambio de marcha | Cambio rechazado (speed gate: solo <1 km/h) | ACK con ACK_REJECTED en CAN 0x103 | Cambio aceptado |
| 4E.2 | Detener ruedas. Enviar cambio FORWARD→REVERSE | Cambio aceptado con ruedas paradas | ACK_OK | Rechazado sin motivo |

#### 4F — Seta de Emergencia Física

| Paso | Acción | Verificar | PASS | FAIL |
|---|---|---|---|---|
| 4F.1 | Con motor girando al 50%: pulsar seta de emergencia | Toda alimentación cortada, motores paran instantáneamente | 0 V en bus, ruedas paran en <1 s | Ruedas siguen girando |
| 4F.2 | Restablecer seta. Encender sistema | Boot limpio, `startup_inhibit` activo | Sin movimiento al encender | Movimiento al encender |

**GATE 4: Todos los pasos 4A–4F deben ser PASS para continuar a Fase 5.**

---

### FASE 5 — Pruebas Funcionales Completas (Ruedas en el aire)

**Objetivo:** Verificar comportamiento funcional completo antes de bajar el vehículo.

| Paso | Acción | Verificar | PASS | FAIL |
|---|---|---|---|---|
| 5.1 | Marcha FORWARD, pedal 20%: las 4 ruedas giran hacia adelante | Dirección correcta, velocidad similar en las 4 | Diferencia <10% entre ruedas | >20% diferencia |
| 5.2 | Marcha REVERSE, pedal 20%: las 4 ruedas giran hacia atrás | Inversión correcta | Todas hacia atrás | Alguna hacia adelante |
| 5.3 | Marcha PARK: pisar pedal | Ruedas no giran | 0 movimiento | Cualquier movimiento |
| 5.4 | Marcha NEUTRAL: pisar pedal | Ruedas no giran | 0 movimiento | Cualquier movimiento |
| 5.5 | Dirección: enviar CAN 0x101 ángulo +30° | Dirección gira a la derecha | Ángulo en 0x204 = +30° ±2° | Error >5° o sin movimiento |
| 5.6 | Dirección: enviar CAN 0x101 ángulo -30° | Dirección gira a la izquierda | Ángulo en 0x204 = -30° ±2° | Error >5° o sin movimiento |
| 5.7 | Dirección: enviar CAN 0x101 ángulo 0° | Dirección vuelve a centro | Ángulo en 0x204 = 0° ±2° | Error >5° |
| 5.8 | Modo 4x4: activar, pisar pedal 20% | 4 ruedas giran | Las 4 con tracción | Alguna sin tracción |
| 5.9 | FORWARD_D2 (máxima potencia): pedal 50% | Ruedas giran más rápido que en D1 | Velocidad D2 > velocidad D1 para mismo pedal | Velocidades iguales |
| 5.10 | Prueba térmica: mantener 50% durante 2 min | Temperaturas suben pero se mantienen seguras | Todas <70 °C | Alguna >90 °C |

**GATE 5: Todos los pasos 5.1–5.10 deben ser PASS para considerar pruebas dinámicas.**

---

## 4. Criterios PASS/FAIL Medibles

### 4.1 Criterios Generales (Aplican a TODA prueba)

| Criterio | PASS | FAIL Inmediato |
|---|---|---|
| Movimiento de rueda sin comando | Nunca | Cualquier movimiento espontáneo |
| Temperatura de motor | <90 °C | ≥90 °C |
| Corriente de motor | <25 A (sin carga) / <10 A (en aire) | >25 A |
| Tensión de batería | >20 V | <18 V |
| Heartbeat CAN 0x001 | Presente cada 100 ms ±20 ms | Ausente >250 ms |
| Estado del sistema | Coherente con la prueba | Estado inesperado |
| Respuesta a seta de emergencia | Corte total <1 s | Cualquier retraso >1 s |

### 4.2 Criterios de Tiempos

| Evento | Tiempo Máximo |
|---|---|
| Boot → STANDBY | 2 s |
| STANDBY → ACTIVE (con ESP32) | 5 s |
| Centrado de dirección (sweep completo) | 10 s |
| Detención tras soltar pedal (en aire) | 2 s |
| Transición a SAFE tras desconexión CAN | 500 ms |
| Reset por watchdog | ~4.1 s |
| Inhibición de pedal al arranque: desbloqueo | 400 ms tras soltar pedal |

### 4.3 Criterios Eléctricos

| Medición | PASS | FAIL |
|---|---|---|
| Corriente en reposo (motores apagados) | <0.5 A por motor | >1 A |
| Tensión en bornes motor (sin demanda) | 0 V ±0.5 V | >1 V |
| Corriente de batería en STANDBY | <2 A total | >5 A |
| Resistencia bus CAN (terminadores) | 55–65 Ω | <50 Ω o >80 Ω |

---

## 5. Señales CAN a Observar

### 5.1 Señales Obligatorias por Fase

#### Durante Boot (Fase 2)

| CAN ID | Nombre | Qué Observar | Frecuencia |
|---|---|---|---|
| 0x001 | HEARTBEAT_STM32 | byte 1 = estado (0=BOOT, 1=STANDBY), byte 2 = fault_flags, byte 3 = error_code | 100 ms |
| 0x204 | STATUS_STEERING | byte 2 = `calibrated` (0 o 1), bytes 0-1 = ángulo actual | 100 ms |
| 0x207 | STATUS_BATTERY | Tensión >20 V | 100 ms |
| 0x202 | STATUS_TEMP | 5 temperaturas en rango ambiente | 1000 ms |
| 0x201 | STATUS_CURRENT | Todas las corrientes ~0 A | 100 ms |

#### Durante Pruebas de Tracción (Fases 3–5)

| CAN ID | Nombre | Qué Observar | Frecuencia |
|---|---|---|---|
| 0x200 | STATUS_SPEED | 4× velocidades de rueda en mm/s | 100 ms |
| 0x201 | STATUS_CURRENT | 4× corrientes de motor + batería | 100 ms |
| 0x203 | STATUS_SAFETY | ABS flags, TCS flags, max slip | 100 ms |
| 0x205 | STATUS_TRACTION | 4× wheel_scale (0–100%), reducción ABS/TCS | 100 ms |
| 0x103 | CMD_ACK | Respuesta a comandos de modo/marcha | On-demand |

#### Durante Pruebas de Dirección (Fase 5)

| CAN ID | Nombre | Qué Observar | Frecuencia |
|---|---|---|---|
| 0x204 | STATUS_STEERING | Ángulo (0.1° resolución), flag calibrated | 100 ms |

#### Señales de Error (Monitorizar SIEMPRE)

| CAN ID | Nombre | Significado |
|---|---|---|
| 0x300 | DIAG_ERROR | Código de error + subsistema afectado |
| 0x001 byte 2 | FAULT_FLAGS | Bitmask de fallos activos |
| 0x001 byte 3 | ERROR_CODE | Código de error activo (ver tabla §5.3) |

### 5.3 Tabla de Códigos de Error (CAN 0x001 byte 3)

| Código | Nombre | Descripción |
|---|---|---|
| 0 | OK | Sin error |
| 1 | OVERCURRENT | Corriente >25 A |
| 2 | OVERTEMP | Temperatura >90 °C |
| 3 | CAN_TIMEOUT | Heartbeat ESP32 ausente >250 ms |
| 4 | SENSOR_FAULT | Sensor implausible |
| 5 | MOTOR_STALL | Motor bloqueado |
| 6 | EMERGENCY_STOP | Seta de emergencia activada |
| 7 | WATCHDOG | Reset por watchdog |
| 8 | CENTERING | Fallo en centrado de dirección |
| 9 | BATTERY_UV_WARNING | Batería <20 V |
| 10 | BATTERY_UV_CRITICAL | Batería <18 V |
| 11 | I2C_FAILURE | Fallo bus I2C (INA226) |
| 12 | OBSTACLE | Obstáculo detectado |

### 5.2 Configuración del Analizador CAN

- **Bitrate:** 500 kbps
- **Filtros de visualización activos:** 0x001, 0x200–0x207, 0x300
- **Registro (log):** Activar grabación a fichero para TODA la sesión de pruebas
- **Trigger de alarma:** Configurar alerta visual/sonora si:
  - 0x001 byte 1 cambia a 5 (SAFE) o 6 (ERROR)
  - 0x001 byte 2 ≠ 0x00 (cualquier fault)
  - 0x300 aparece (error diagnóstico)
  - 0x001 desaparece >300 ms (pérdida de heartbeat STM32)

---

## 6. Criterios de Prohibición de Pruebas Dinámicas (NO-GO)

### 6.1 Prohibición Absoluta — NO bajar el vehículo si:

| # | Condición NO-GO | Justificación |
|---|---|---|
| NG1 | Cualquier paso de Fases 1–5 es FAIL | Seguridad no verificada |
| NG2 | Se observó movimiento de rueda sin comando en cualquier momento | Par fantasma activo |
| NG3 | `startup_inhibit` no funcionó en prueba 3.1 | Riesgo de arranque involuntario |
| NG4 | Seta de emergencia no cortó alimentación instantáneamente | Sin último recurso de seguridad |
| NG5 | Timeout CAN no provocó transición a SAFE (prueba 4A) | Sin protección ante pérdida de comunicación |
| NG6 | Corriente >10 A en cualquier motor sin carga (ruedas en aire) | Posible cortocircuito o driver defectuoso |
| NG7 | Cualquier temperatura >70 °C durante pruebas sin carga | Problema térmico de base |
| NG8 | Centrado de dirección no completa o falla repetidamente | Dirección impredecible |
| NG9 | Pedal genera demanda estando en PARK o NEUTRAL | Fallo en gate de marcha |
| NG10 | Cambio de marcha aceptado con ruedas girando >1 km/h | Speed gate no funciona |
| NG11 | Heartbeat CAN 0x001 tiene gaps >200 ms durante operación normal | Bucle principal inestable |
| NG12 | Error_code ≠ 0 en CAN 0x001 sin causa provocada | Fallo latente no resuelto |
| NG13 | Algún motor gira en dirección contraria a la marcha seleccionada | Cableado invertido |
| NG14 | Watchdog no reseteó MCU tras congelamiento (prueba 4B) | Sin recuperación de software |

### 6.2 Condiciones para Autorizar Pruebas Dinámicas (Ruedas en el suelo)

**TODOS** los siguientes criterios deben cumplirse simultáneamente:

- [ ] Todas las Fases 1–5 completadas con resultado PASS
- [ ] Ninguna condición NO-GO (NG1–NG14) presente
- [ ] Log CAN completo revisado sin anomalías
- [ ] Seta de emergencia probada y funcional
- [ ] Persona de seguridad designada con acceso a seta
- [ ] Área de pruebas despejada, sin personas en trayectoria
- [ ] Velocidad máxima de primera prueba dinámica: 2 km/h (marcha D1, pedal <10%)
- [ ] Primera prueba dinámica: avance recto 2 metros y frenado

### 6.3 Protocolo de Primera Prueba Dinámica

Si y solo si se cumplen TODOS los criterios de §6.2:

1. Bajar vehículo al suelo
2. Zona despejada, mínimo 10 m libres delante
3. Persona de seguridad junto a seta de emergencia, fuera de trayectoria
4. Marcha FORWARD (D1), pedal <5% → avanzar ~2 m
5. Soltar pedal → verificar que el vehículo se detiene
6. Si se detiene correctamente → repetir a 10%, 20%, hasta 50% en tramos de 5 m
7. Si en cualquier momento el comportamiento es inesperado → **SETA INMEDIATA** y volver a Fase 4

---

## Registro de Ejecución

| Fase | Fecha | Resultado | Ejecutado por | Observaciones |
|---|---|---|---|---|
| Fase 1 | ____/____/____ | PASS / FAIL | ______________ | |
| Fase 2 | ____/____/____ | PASS / FAIL | ______________ | |
| Fase 3 | ____/____/____ | PASS / FAIL | ______________ | |
| Fase 4 | ____/____/____ | PASS / FAIL | ______________ | |
| Fase 5 | ____/____/____ | PASS / FAIL | ______________ | |
| Autorización dinámica | ____/____/____ | SÍ / NO | ______________ | |

---

*Documento generado a partir de la revisión del firmware STM32G474RE v1.0. No modifica código. Solo seguridad funcional.*
