# Remote Control — Estrategia de Failsafe en 4 capas

> Estrategia profesional de failsafe para el control remoto inalámbrico del
> vehículo MarcosDashboard v10 Final. **NO IMPLEMENTADO** — documento de diseño.
> Verificado contra el firmware real.

---

## 1. Principio de diseño

El failsafe se construye en **4 capas independientes en cascada**. Cada capa
opera de forma autónoma; ningún fallo único puede dejar el vehículo sin protección.

> **Invariante absoluto:** El STM32G474RE es la **única autoridad de seguridad**.
> El mando remoto solo puede **solicitar** demanda. Todos los frenos de
> emergencia, watchdogs, validaciones de rango, gating por estado y abortos
> siguen siendo competencia exclusiva del STM32.

---

## 2. Las 4 capas de failsafe (en cascada)

```
┌───────────────────────────────────────────────────────────────────┐
│ CAPA 1 — Failsafe HARDWARE del receptor FS-iA6B                   │
│   Disparo: pérdida señal RF > ~1 s (configurable en mando)         │
│   Acción: todos los canales → posiciones preconfiguradas en TX     │
│            (CH1=1500, CH2=1500, CH5=kill, CH10=LOCAL)              │
│   Independiente: SÍ — ocurre dentro del receptor, sin software     │
└────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌────────────────────────────────────────────────────────────────────┐
│ CAPA 2 — Failsafe SOFTWARE en el ESP32 (parser iBUS)               │
│   Disparo:                                                         │
│     a) > 150 ms sin trama válida (≈21 tramas perdidas)             │
│     b) > 5 fallos de checksum consecutivos                         │
│     c) CH10 en LOCAL                                               │
│     d) Cualquier canal fuera de rango sano [900, 2100]             │
│   Acción: parser entra en estado FAILSAFE → ESP32 deja de enviar   │
│           frames 0x100 / 0x101 al STM32                            │
│   Independiente: SÍ — corre en Core 1 del ESP32                    │
└────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌────────────────────────────────────────────────────────────────────┐
│ CAPA 3 — CAN timeout (STM32, ya existente, NO se modifica)         │
│   Disparo: > 250 ms sin heartbeat ESP32 (CAN ID 0x011)             │
│   Acción: SYS_STATE → LIMP_HOME → pedal local con clamp 20%        │
│   Independiente: SÍ — corre en STM32 con su propio reloj           │
│   Código: Core/Src/can_handler.c (heartbeat 0x011 / 100 ms)        │
│   Referencia: Core/Src/safety_system.c                             │
└────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌────────────────────────────────────────────────────────────────────┐
│ CAPA 4 — IWDG hardware STM32 (ya existente, NO se modifica)        │
│   Disparo: > 4.1 s sin refresh del Independent Watchdog           │
│   Acción: reset completo del MCU → vuelve a BOOT en SAFE           │
│   Independiente: SÍ — periférico hardware dedicado en LSI 32 kHz   │
└────────────────────────────────────────────────────────────────────┘
```

---

## 3. Escenarios de fallo y respuesta esperada

### 3.1 Pérdida total de señal RF (mando apagado / fuera de rango)

| t | Evento | Capa actuante | Estado del vehículo |
|---|---|---|---|
| 0 ms | Mando se apaga | — | Normal, mando enviando |
| 0–1000 ms | Receptor sin señal | — | Receptor congela últimos valores |
| ~1000 ms | Failsafe HW del receptor | **Capa 1** | Receptor envía CH a posición preconfigurada |
| 1000+150 ms | Parser ESP32 ve CH5=kill o CH10=LOCAL o ausencia | **Capa 2** | ESP32 deja de enviar 0x100/0x101 |
| — | STM32 sigue recibiendo heartbeat 0x011 | — | El STM32 sigue en ACTIVE (no entra LIMP_HOME) |
| — | Pedal local toma control vía tick_50ms | — | Si conductor no pisa el pedal → demand = 0 |

**Resultado:** vehículo se detiene suavemente por EMA + ramp limiter del motor.

### 3.2 ESP32 reboot inesperado (con mando activo)

| t | Evento | Capa actuante | Estado del vehículo |
|---|---|---|---|
| 0 ms | ESP32 cuelga / reboot | — | Frames 0x100, 0x101, heartbeat 0x011 cesan |
| 0–250 ms | STM32 sigue procesando último frame | — | EMA + ramp suavizan |
| 250 ms | STM32 detecta CAN timeout (heartbeat) | **Capa 3** | SYS_STATE → LIMP_HOME |
| — | STM32 ignora CAN throttle | — | Pedal local con clamp 20% — usuario controla freno |
| ~3–5 s | ESP32 termina reboot, vuelve a enviar | — | STM32 vuelve a ACTIVE tras recibir heartbeat |

**Resultado:** vehículo no se desboca. LIMP_HOME mantiene movilidad limitada.

### 3.3 Falla CAN bus completa (rotura cable, transceiver muerto)

| t | Evento | Capa actuante | Estado del vehículo |
|---|---|---|---|
| 0 ms | CAN bus muerto | — | Ningún frame llega al STM32 |
| 250 ms | STM32 heartbeat timeout | **Capa 3** | SYS_STATE → LIMP_HOME |
| — | Pedal local sigue funcionando | — | El STM32 no depende del CAN para leer el pedal |

**Resultado:** independiente del mando — el STM32 cubre el escenario con su lógica existente.

### 3.4 Joystick congelado (sticky)

| t | Evento | Capa actuante | Estado del vehículo |
|---|---|---|---|
| 0 ms | Joystick se queda atascado en 80 % | — | Mando envía 80 % constante |
| — | ESP32 lo retransmite | — | STM32 acelera al 80 % validado |
| — | Conductor pulsa **kill switch CH5** | **Manual** | Mando envía CH2=0, CH5=kill → ESP32 deja de enviar |
| — | Pedal local toma control | — | Si pedal en reposo → demand = 0 |

**Mitigación adicional propuesta:** el parser ESP32 puede detectar "frozen joystick"
(mismo valor exacto por > N segundos con receptor activo) y entrar en DEGRADED.
Esto se discute pero **NO es obligatorio para la primera versión**.

### 3.5 EMI / paquetes corruptos durante uso intenso del motor

| t | Evento | Capa actuante | Estado del vehículo |
|---|---|---|---|
| 0 ms | EMI fuerte del BTS7960 a 20 kHz | — | Algunos paquetes iBUS llegan con checksum malo |
| — | Parser ESP32 descarta paquetes corruptos | **Capa 2 (a)** | Mantiene último valor válido |
| — | Si > 5 fallos consecutivos | **Capa 2 (b)** | Estado DEGRADED → no enviar |
| — | Si > 150 ms sin trama válida | **Capa 2 (a)** | Estado FAILSAFE → no enviar |

**Resultado:** EMI transitoria no produce comportamiento errático.

### 3.6 Bus-off CAN del ESP32 (al transmitir antes de que el STM32 esté listo)

Este escenario ya está cubierto por el código existente en `esp32/src/main.cpp:1542+`
(bus-off recovery con `twai_initiate_recovery()`). El nuevo módulo de remote control
no introduce frames adicionales fuera del ciclo normal y no requiere modificar este
mecanismo.

### 3.7 Pérdida del enlace UART entre receptor y ESP32 (cable roto)

| t | Evento | Capa actuante | Estado del vehículo |
|---|---|---|---|
| 0 ms | Cable iBUS se desconecta | — | UART RX del ESP32 deja de recibir |
| 150 ms | Parser entra en FAILSAFE | **Capa 2** | ESP32 no envía 0x100/0x101 |
| — | STM32 sigue recibiendo heartbeat | — | ACTIVE — pedal local |

**Mitigación adicional:** el receptor también soporta línea "iBUS Sensor" para
detectar conectividad. No imprescindible.

### 3.8 Doble fallo: mando activo + ESP32 colgado

Caso peor analizado:

| t | Evento | Capa actuante | Estado del vehículo |
|---|---|---|---|
| 0 ms | Mando en 70 %, ESP32 cuelga sin reboot | — | Frames 0x100 dejan de llegar inmediatamente |
| 250 ms | STM32 timeout CAN | **Capa 3** | LIMP_HOME |
| ~4.1 s (si STM32 también se cuelga) | IWDG | **Capa 4** | Reset completo MCU → SAFE |

**Resultado:** vehículo desactiva motores en < 500 ms en el peor caso.

---

## 4. Parámetros configurables (resumen)

| Parámetro | Valor propuesto | Capa | Notas |
|---|---|---|---|
| Failsafe HW receptor | ~1 s sin RF | 1 | Configurable desde menú del mando |
| Posición failsafe CH1 | 1500 (centro) | 1 | Steering neutro |
| Posición failsafe CH2 | < 1450 (≤ 0 %) | 1 | Throttle a 0 |
| Posición failsafe CH5 | kill switch ON | 1 | Activar emergency stop lógico |
| Posición failsafe CH10 | LOCAL | 1 | Forzar source = pedal local |
| Timeout parser sin trama | 150 ms | 2 | ≈21 tramas perdidas |
| Threshold checksum consecutivos | 5 | 2 | Antes de marcar DEGRADED |
| Rango sano canal | [900, 2100] | 2 | Fuera → descarte |
| Heartbeat STM32 timeout | 250 ms | 3 | Ya existente, no cambia |
| IWDG STM32 | ~4.1 s | 4 | Ya existente, no cambia |
| Deadzone ESP32 | ±30 cuentas (≈3 %) | (procesamiento) | Igual que pedal |
| EMA α | 0,15 | (procesamiento) | Igual que motor_control.c |
| Ramp up | 50 %/s | (procesamiento) | Igual que motor_control.c |
| Ramp down | 100 %/s | (procesamiento) | Igual que motor_control.c |

---

## 5. Source arbitration explícito (pedal local vs mando)

> Detalle completo en `docs/REMOTE_CONTROL_ARCHITECTURE.md §7`.

Recomendación: **Opción A** (source select por CH10 desde el mando).
- Si el mando está en **LOCAL** → el ESP32 NO envía 0x100/0x101 → el STM32 funciona
  exactamente como hoy.
- Si el mando está en **REMOTE** → el ESP32 envía 0x100/0x101 cada 50 ms; el pedal
  local del STM32 sigue siendo válido y compite, pero el EMA + ramp suavizan.

Si tras testing se requiere control absoluto del mando, **Opción B** (5 líneas en
`Core/Src/main.c`) añade un flag `can_throttle_override` con timeout 200 ms que
suspende la lectura del pedal mientras el mando está activo. Este es el único
cambio en STM32 contemplado, y solo si la Opción A resulta insuficiente.

---

## 6. Lo que NO se modifica nunca (invariantes de seguridad)

| Sistema | Razón |
|---|---|
| `Safety_ValidateThrottle()` | Validación de rango y estado se aplica SIEMPRE, venga de pedal o de CAN |
| `Safety_ValidateSteering()` | Idem para steering |
| `Startup_IsInhibited()` | Power-On Movement Prevention sigue intacto |
| `Pedal_IsContradictory()` / `Pedal_IsPlausible()` | Validación dual ADC sigue intacta |
| ABS / TCS (`Safety_GetTractionCapFactor()`) | Modulación de seguridad por encima del demand |
| Overcurrent INA226 | Lectura y respuesta intactas |
| Emergency stop | Mismo handler, mismas señales |
| IWDG | Periodo y refresh intactos |
| BTS7960 EN pin | Gestión por safety_system intacta |

**Conclusión:** ningún failsafe nuevo bypassa los existentes. Todos se suman.

---

## 7. Test cases de validación (a ejecutar antes de release)

| # | Test | Resultado esperado |
|---|---|---|
| F1 | Apagar mando con vehículo a 50 % throttle | Vehículo para suavemente en < 1,5 s |
| F2 | Mover mando fuera de alcance | Idem F1 |
| F3 | Activar kill switch CH5 con throttle 70 % | Throttle cae a 0 en < 200 ms |
| F4 | Forzar CH10 a LOCAL durante uso | Mando ignorado, pedal local toma control |
| F5 | Desconectar cable iBUS en caliente | ESP32 detecta failsafe < 200 ms |
| F6 | Reset ESP32 manual | STM32 entra LIMP_HOME < 250 ms |
| F7 | Cortar CAN_H mientras mando activo | STM32 entra LIMP_HOME |
| F8 | Acelerar BTS7960 al máximo cerca del receptor | Sin pérdida de control / sin checksum > 5/s |
| F9 | Joystick atascado en 80 % | Kill switch detiene; sin kill, ramp limiter del STM32 limita |
| F10 | IWDG: bloquear loopTask del ESP32 con `while(1);` artificial | STM32 → LIMP_HOME |
| F11 | Conducir 30 min con mando | Sin glitches, sin reboots, sin LIMP_HOME espurios |

---

**Estado del documento:** Estrategia documentada y verificada contra firmware real.
Pendiente ejecución en fase de implementación autorizada por el usuario.
