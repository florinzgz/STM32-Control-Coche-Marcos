# Remote Control — Análisis de Riesgos completo

> Análisis profesional de riesgos para la integración del control remoto
> en MarcosDashboard v10 Final. **NO IMPLEMENTADO** — documento previo
> a la fase de implementación.

---

## 1. Matriz de riesgos (resumen)

Escalas:

- **Probabilidad:** Muy baja (1) / Baja (2) / Media (3) / Alta (4) / Muy alta (5)
- **Severidad:** Insignificante (1) / Menor (2) / Moderada (3) / Mayor (4) / Crítica (5)
- **Riesgo = P × S** (1–25)

| # | Riesgo | P | S | R | Mitigación | Categoría |
|---|---|---|---|---|---|---|
| R1 | EMI del BTS7960 corrompe tramas iBUS | 3 | 2 | 6 | FHSS 142 canales + checksum + separación física + ferritas | EMI |
| R2 | UART conflict en ESP32 | 1 | 4 | 4 | Verificación explícita en `REMOTE_CONTROL_GPIO_USAGE.md` — UART0 libre | UART |
| R3 | Reboot ESP32 con mando activo | 2 | 4 | 8 | CAN timeout STM32 250 ms → LIMP_HOME | Software |
| R4 | Pérdida total RF | 3 | 4 | 12 | 4 capas failsafe en cascada | RF |
| R5 | Joystick mecánicamente atascado | 2 | 4 | 8 | Kill switch CH5 manual + ramp limiter STM32 | Mecánico |
| R6 | Conductor olvida CH10=LOCAL al salir del coche | 3 | 3 | 9 | Documentar procedimiento + buzzer audible si mando activo en arranque | Operacional |
| R7 | Doble fuente de demanda (pedal + mando) compite | 3 | 3 | 9 | Opción A (source select CH10) o Opción B (override flag) | Software |
| R8 | Saturación CAN bus | 1 | 3 | 3 | Cadencia 50 ms y carga < 5 % bus a 500 kbps | CAN |
| R9 | Latencia total > 50 ms | 2 | 3 | 6 | Medir bench: ~20 ms total esperado | Latencia |
| R10 | Glitches durante transición LOCAL→REMOTE en marcha | 2 | 3 | 6 | EMA + ramp suavizan; documentar handoff | Software |
| R11 | Frozen joystick no detectado | 2 | 4 | 8 | Kill switch manual + opcional detection en parser | Sensor |
| R12 | Receptor pierde alimentación durante uso | 2 | 4 | 8 | Failsafe HW + Capa 2 SW | Hardware |
| R13 | Race condition entre tick_50ms y CAN_ProcessMessages | 3 | 2 | 6 | EMA + ramp absorben; arbitraje claro Opción A/B | Software |
| R14 | Bus-off CAN del ESP32 | 2 | 3 | 6 | Recovery existente `esp32/src/main.cpp:1542+` | CAN |
| R15 | Watchdog STM32 (IWDG) reset inesperado | 1 | 4 | 4 | Parser ESP32 no afecta IWDG STM32; ya gestionado | Watchdog |
| R16 | Interferencia con WiFi/BT del ESP32 (futuro) | 1 | 2 | 2 | El receptor RC tiene su propia radio; no comparte | RF |
| R17 | Comprar FS-iA6 (sin "B") por error | 4 | 4 | 16 | Documento explícito: NO COMPRAR el de la foto | Compra |
| R18 | Driver Arduino HardwareSerial cuelga | 1 | 4 | 4 | Test bench prolongado + IWDG ESP32 | Software |
| R19 | Antena del receptor rota / suelta | 2 | 4 | 8 | Fijación mecánica + revisión periódica | Hardware |
| R20 | Pérdida de bind transmisor-receptor | 1 | 3 | 3 | Rebind documentado, failsafe HW protege | Operacional |

---

## 2. Análisis detallado de los riesgos críticos (R ≥ 8)

### R17 — Comprar hardware incorrecto (FS-iA6 sin "B")  **— RIESGO MÁS ALTO**

**Probabilidad: 4 (alta)** — el bundle más vendido es FS-i6 + FS-iA6, y visualmente
son casi idénticos al FS-i6X + FS-iA6B.

**Severidad: 4 (mayor)** — el FS-iA6 **no tiene iBUS**, lo que invalida toda la
arquitectura propuesta. Forzaría:

- Captura PWM por interrupciones de 4–6 GPIOs (consume pines que están reservados).
- Sin checksum a nivel de protocolo.
- Susceptible a jitter / EMI.
- Sin telemetría inversa.

**Mitigación:**
- ✅ Documentos explícitos `REMOTE_CONTROL_ARCHITECTURE.md §2` señalan el problema.
- ✅ Lista de compras en `REMOTE_CONTROL_WIRING.md §4` indica modelo exacto.
- ✅ Verificar al recibir: el receptor correcto tiene un puerto adicional rotulado
  "SENS / iBUS" además de los 6 canales PWM. Si solo hay 6 canales y nada más,
  es el FS-iA6 (incorrecto).

### R4 — Pérdida total RF

**Probabilidad: 3 (media)** — sucederá ocasionalmente por alcance, obstáculos.
**Severidad: 4 (mayor)** si no hubiera failsafe.

**Mitigación:**
- Capa 1 (HW receptor) → canales a posición segura preconfigurada (~1 s).
- Capa 2 (SW ESP32) → deja de enviar 0x100/0x101 en < 150 ms.
- Capa 3 (CAN heartbeat) → LIMP_HOME en 250 ms si todo falla.
- Capa 4 (IWDG) → reset en 500 ms en peor caso.

**Riesgo residual:** vehículo continúa unos segundos en inercia. Aceptable a baja
velocidad.

### R3 — Reboot ESP32 con mando activo

**Probabilidad: 2 (baja)** — reboots por watchdog / brownout son raros pero ocurren.
**Severidad: 4 (mayor)** — pérdida total de comunicación con el STM32.

**Mitigación existente (sin tocar nada):**
- Heartbeat 0x011 cesa inmediatamente al reboot.
- STM32 ve timeout en 250 ms → LIMP_HOME.
- Pedal local recupera control con clamp 20 %.
- Si el conductor no pisa pedal → demand = 0 → vehículo se detiene.

**Riesgo residual:** ventana de 250 ms donde el último demand del mando podría seguir
aplicándose en el STM32. EMA + ramp lo absorben.

### R7 — Doble fuente de demanda (pedal + mando)

**Probabilidad: 3 (media)** — si ambos están activos simultáneamente.
**Severidad: 3 (moderada)** — comportamiento errático posible.

**Mitigación:**
- **Opción A** (recomendada): source select por CH10. El conductor decide quién manda.
- **Opción B** (si Opción A insuficiente): flag override en STM32 con timeout 200 ms.

**Riesgo residual:** en Opción A con ambos activos, oscilación máxima de 50 ms entre
valores. Filtrado por EMA α=0,15 + ramp 50/100 %/s del motor (efecto despreciable
a baja velocidad).

### R11 — Frozen joystick no detectado

**Probabilidad: 2 (baja)** — fallo mecánico raro pero conocido en joysticks baratos.
**Severidad: 4 (mayor)** — el mando enviaría demanda constante alta.

**Mitigación:**
- Kill switch manual CH5 — el conductor puede activar.
- Ramp limiter STM32 limita aceleración máxima del demand.
- ABS / TCS limita si el vehículo derrapa.
- (Opcional) Implementar en el parser una detección de "frozen" (mismo valor exacto
  por > 30 s con throttle > 30 %) y marcarla como DEGRADED.

**Riesgo residual:** moderado. Recomendable implementar detection en el parser.

### R6 — Conductor olvida CH10=LOCAL al salir del coche

**Probabilidad: 3 (media)** — error humano frecuente.
**Severidad: 3 (moderada)** — si alguien enciende el coche luego con el mando en
REMOTE, podría arrancar con demanda inesperada.

**Mitigación:**
- `Startup_IsInhibited()` ya bloquea arranque hasta que el pedal se libere.
- Documentar procedimiento: apagar mando antes de salir.
- (Opcional) Buzzer audible si en arranque el mando está activo y CH10=REMOTE.

### R5 — Joystick mecánicamente atascado

Ver R11 — escenario equivalente con causa mecánica.

### R12 — Receptor pierde alimentación durante uso

**Mitigación:**
- Receptor alimentado del rail 5 V proyecto — mismo rail que DFPlayer y MCP23017.
- Si pierde alimentación, el ESP32 deja de recibir UART → Capa 2 actúa < 150 ms.
- Capa 3 actúa < 250 ms si el ESP32 también cae.

### R19 — Antena del receptor rota / suelta

**Probabilidad: 2 (baja)** — depende de montaje.
**Severidad: 4 (mayor)** — pérdida total RF.

**Mitigación:**
- Fijación mecánica con tornillos M2 nylon o adhesivo permanente.
- Revisión visual periódica en mantenimiento.
- Receptor en orientación protegida dentro del compartimento lógico.

### R18 — Driver Arduino HardwareSerial cuelga

**Probabilidad: 1 (muy baja)** — fallo raro reportado en condiciones de UART overflow.
**Severidad: 4 (mayor)** — el módulo dejaría de procesar.

**Mitigación:**
- `setRxBufferSize(256)` adecuado para 4 tramas iBUS.
- Drenar el buffer en cada `update()` para evitar overflow.
- TWDT del ESP32 cubre cuelgues de tarea.
- Tests prolongados en Fase 7.

---

## 3. Riesgos NO mitigables completamente

| Riesgo | Carácter | Mitigación residual |
|---|---|---|
| Pérdida de control en el ms 0–249 tras reboot ESP32 | Inherente al diseño CAN heartbeat | EMA + ramp del motor; bajar velocidad operacional |
| Pico de EMI específico que coincide con saltos FHSS | Probabilidad muy baja | Aceptado |
| Error humano del operador del mando | No técnico | Formación y procedimiento documentado |

---

## 4. Riesgos relacionados con `safety_system.c`, `motor_control.c`, ABS/TCS, encoder

**Ninguno.** Estos sistemas NO se tocan en absoluto. El mando es un productor más
de frames CAN 0x100/0x101 — exactamente los mismos que ya procesa el STM32 cuando
viene del pedal CAN existente (ahora mismo no se usa, pero el handler ya existe y
está validado).

Cualquier validación de seguridad existente (`Safety_ValidateThrottle()`,
`Safety_ValidateSteering()`, `Startup_IsInhibited()`, ABS/TCS, overcurrent, IWDG)
**actúa exactamente igual** independientemente del origen del frame CAN. Esta es
la garantía estructural de que el mando RC NO bypasa la safety authority del STM32.

---

## 5. Riesgos relacionados con `renderTask` Core 0

**Ninguno.** El parser iBUS vive en Core 1 (dentro de `loop()`). Core 0 sigue
ejecutando exclusivamente la renderTask de TFT + touch. No hay nuevas locks ni
nuevas tareas FreeRTOS añadidas.

---

## 6. Riesgos relacionados con FreeRTOS / watchdogs ESP32

| Aspecto | ¿Afectado? | Notas |
|---|---|---|
| TWDT (Task Watchdog) | ❌ No | El `update()` del parser retorna en < 100 µs |
| IDLE0 / IDLE1 tasks | ❌ No | Sin cambios de prioridades |
| Stack del loopTask | ⚠️ Mínimo | Buffer de 256 bytes en parser; tamaño total del módulo < 1 KB |
| Heap fragmentation | ❌ No | El módulo no usa `malloc`/`new` |
| ISR latency | ❌ No | El parser no usa ISRs personalizados — solo lee buffer UART |

---

## 7. Riesgos de saturación del CAN bus

CAN actualmente lleva (medido en `docs/CAN_CONTRACT_FINAL.md`):

- Heartbeats: 2 × 100 ms (ESP32 + STM32) = 20 frames/s
- Status frames (0x200-0x209): variable, ~30 frames/s
- Diag frames: bajo
- Comandos on-demand: muy bajo

**Carga total estimada actual:** < 50 frames/s × ~110 bits/frame = ~5500 bps ≈ **1,1 %**
del bus a 500 kbps.

**Adición del mando:** 2 frames × (1000 ms / 50 ms) = 40 frames/s ≈ +4400 bps ≈ +0,9 %.

**Carga total post-mando:** ~2 %. **Sin riesgo de saturación.**

---

## 8. Conclusión del análisis de riesgos

- **Riesgos altos (R ≥ 12):** solo R17 (comprar el modelo incorrecto) y R4 (pérdida RF).
  Ambos completamente mitigados por diseño y documentación.
- **Riesgos moderados (R 8–11):** 5 ítems. Todos con mitigación clara y, en algunos
  casos, mitigación adicional opcional (frozen joystick detection, buzzer de arranque).
- **Riesgos bajos (R < 8):** 13 ítems. Aceptados con las mitigaciones existentes.

**Veredicto:** la integración es viable con riesgo residual bajo, **siempre que**:

1. Se compre el hardware correcto (FS-i6X + FS-iA6B, **no** FS-i6 + FS-iA6).
2. Se respeten las 4 capas de failsafe documentadas.
3. NO se modifique `safety_system.c`, `motor_control.c`, PWM, watchdogs, encoder,
   ABS/TCS ni renderTask.
4. El STM32 siga siendo la única safety authority.

---

**Estado del documento:** Análisis completo. Riesgo R17 (compra incorrecta) es el
más alto — destacado para atención del usuario antes de pedir el hardware.
