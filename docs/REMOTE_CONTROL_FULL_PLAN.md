# Remote Control — Plan profesional ampliado (todas las funciones posibles)

> Extensión del plan base `REMOTE_CONTROL_IMPLEMENTATION_PLAN.md` aprovechando
> los **10 canales nativos** del FlySky FS-i6X y todas las capacidades que
> ofrece el ESP32-S3 + STM32G474RE actual sin romper invariantes de seguridad.
>
> **NO IMPLEMENTADO.** Documento de planificación previo a autorización.
>
> Última revisión: 2026-05-24.

---

## 0. Principio rector

> Cada función nueva del mando se mapea a **un canal iBUS** y se traduce a:
> - Un **frame CAN existente** que el STM32 ya entiende, o
> - Una acción **local en el ESP32** que ya existe en el firmware actual.
>
> **No se inventa hardware.** **No se cambian timers, IWDG, ABS/TCS, PWM,
> safety_system.c ni motor_control.c.** El STM32 sigue siendo la única
> autoridad de seguridad. El mando solo puede *solicitar*.

---

## 1. Mapa completo de canales (FS-i6X, 10 canales)

| CH | Control físico FS-i6X       | Función propuesta                    | Tipo        | Implementación |
|----|------------------------------|--------------------------------------|-------------|----------------|
| 1  | Joystick D, eje X            | Steering (±30°)                      | Analógico   | CAN 0x101 (existente) |
| 2  | Joystick I, eje Y            | Throttle 0–100 %                     | Analógico   | CAN 0x100 (existente) |
| 3  | Joystick I, eje X            | Trim de centrado steering (±5°)      | Analógico   | Ajuste local ESP32 sobre CH1 |
| 4  | Joystick D, eje Y            | (Reservado — sin uso seguro)         | —           | Ignorado |
| 5  | SWA (2 pos)                  | **Kill switch / habilitación**       | Digital     | Local: si OFF, ESP32 no emite 0x100/0x101 |
| 6  | SWB (3 pos)                  | **Modo de conducción** ECO / NORMAL / SPORT | Discreto | CAN 0x10A (sub-opcode existente o nuevo `MODE_SET`) |
| 7  | SWC (3 pos)                  | **Marcha** D / N / R                 | Discreto    | CAN 0x102 byte1 (mismo formato que palanca actual) |
| 8  | SWD (2 pos)                  | **Luces** ON/OFF                     | Digital     | CAN existente o vía relé ESP32 |
| 9  | VRA (potenciómetro)          | **Volumen audio DFPlayer**           | Analógico   | Local ESP32 → `audio::setVolume()` |
| 10 | VRB (potenciómetro)          | **Selector LOCAL ↔ REMOTE**          | Discreto    | Local: gate maestro de emisión CAN del mando |

> CH4 se reserva sin uso porque mover sin querer el joystick derecho en
> el eje Y no debe disparar nada.

> CH10 funciona como **interruptor maestro de soberanía**: con CH10 en
> LOCAL (umbral < 1300 µs) el ESP32 **nunca** envía frames del mando,
> independientemente del resto de canales. Esto coexiste con CH5 como kill
> switch redundante (CH5 OFF → throttle forzado a 0, steering a centro).

---

## 2. Funciones detalladas

### 2.1 Throttle (CH2) — *prioridad alta*
- Lectura iBUS 1000–2000 µs → mapeo 0–100 %.
- Deadzone ±30 µs alrededor del centro 1500.
- Filtro EMA α = 0.15 y rampa 50 %/s subida, 100 %/s bajada
  (refuerzo, ya que el STM32 también aplica suavizado idéntico —
  ver memoria *throttle smoothing*).
- Emisión: CAN `0x100` byte 0 = throttle %, cadencia 50 ms.

### 2.2 Steering (CH1 + CH3 trim) — *prioridad alta*
- CH1 1000–2000 µs → ±30° pedidos.
- CH3 actúa como trim sumado en runtime (±5°), guardado en NVS del ESP32
  para que persista entre reinicios.
- Emisión: CAN `0x101` con int16 LE en bytes 0-1 (formato existente).

### 2.3 Kill switch + selector (CH5 + CH10) — *prioridad crítica*
- CH5 OFF **o** CH10 LOCAL → el ESP32 deja de emitir 0x100/0x101.
- El STM32 detecta el silencio: tras 250 ms aplica timeout LIMP_HOME ya
  existente.
- CH5 es un override duro: aunque CH10 esté en REMOTE, CH5 OFF gana.

### 2.4 Modo de conducción (CH6) — *opcional, baja prioridad*
- 3 posiciones: ECO / NORMAL / SPORT.
- Implementación recomendada: emitir un frame CAN nuevo o reutilizar
  un sub-opcode ya existente del menú de servicio para ajustar el
  ramp limit en el STM32 (sin tocar `motor_control.c`, solo cambiando
  parámetros ya configurables).
- Si no existe sub-opcode adecuado, la función se aplaza y CH6 queda
  sin asignar — **no se añaden caminos críticos al firmware del STM32**.

### 2.5 Selector de marcha (CH7) — *opcional, prioridad media*
- 3 posiciones: D / N / R.
- Convención CAN ya existente (`0x102` byte 1, ver memoria *gear shifter*).
- Conflicto con la palanca física por MCP23017:
  - Política propuesta: **la palanca física tiene prioridad**.
    El mando solo cambia marcha si la palanca está en N.
  - Documentar el comportamiento. No replicar lógica de seguridad.

### 2.6 Luces (CH8) — *opcional, baja prioridad*
- Si existe ya el control de luces vía CAN: mapear al frame correspondiente.
- Si no, **NO se añade hardware ni GPIO nuevo**: queda como función futura.

### 2.7 Volumen audio (CH9) — *opcional, sin coste de seguridad*
- 100 % local al ESP32: VRA → `audio::setVolume(0–30)` con histéresis.
- No toca STM32, no toca CAN.

### 2.8 Telemetría inversa (futura, fase 9+) — *opcional*
- El FS-iA6B soporta sensores externos por iBUS-Telem.
- Idea: enviar batería principal (V/I) y velocidad como pseudo-sensores
  (no implementado, no planificado para esta fase).

---

## 3. Encaje arquitectónico

```
   FS-i6X TX  ─── AFHDS 2A ──►  FS-iA6B RX  ── iBUS UART 115200 ──►  ESP32-S3 GPIO 16
                                                                          │
                                                                          ▼
                                                  remote_control parser + FSM failsafe
                                                                          │
                                              ┌───────────────────────────┼──────────────────────┐
                                              ▼                           ▼                      ▼
                                       Acciones locales            CAN frames hacia STM32   NVS persistente
                                       (audio vol, trim,           (0x100, 0x101, 0x102,    (trim steering)
                                        modo display)               opcional 0x10A)
                                                                          │
                                                                          ▼
                                                          STM32G474RE (autoridad de seguridad)
                                                          - Valida rangos
                                                          - Aplica safety_system / ABS / TCS
                                                          - Timeout 250 ms → LIMP_HOME
                                                          - IWDG 4.1 s
```

---

## 4. Fases (refinadas para esta versión ampliada)

> Cada fase mantiene la disciplina del plan base: gate explícita, sin avanzar
> sin autorización del usuario.

| Fase | Contenido | Toca FW STM32 | Toca FW ESP32 | Hardware |
|------|-----------|---------------|---------------|----------|
| **−1** | **Copia de seguridad firmware actual** (ESTE PR ya prepara las herramientas) | NO | NO | Bench |
| 0    | Adquisición FS-i6X + FS-iA6B, validar tramas iBUS en bench con FTDI | NO | NO | Banco |
| 1    | Crear `remote_control.{h,cpp}` aislado + test host con fixtures | NO | SÍ (nuevo, desactivado) | NO |
| 2    | 3 líneas en `esp32/src/main.cpp` (include + init + update) + flag a 0 | NO | SÍ (mínimo) | NO |
| 3a   | Mapeo CH1 (steering) + CH2 (throttle) + CH5 + CH10. Bench con motor desconectado | NO | SÍ | Banco |
| 3b   | Añadir CH3 (trim), CH9 (volumen) — acciones puramente locales ESP32 | NO | SÍ | Banco |
| 3c   | Añadir CH7 (marcha) coordinado con palanca física, CH6 (modo) si hay sub-opcode existente | NO | SÍ | Banco |
| 4    | (Opcional) Override CAN throttle en STM32 si Fase 3 demuestra necesidad | SÍ (≤7 líneas) | NO | NO |
| 5    | Bench con motor conectado, ruedas en aire | NO | NO | Banco |
| 6    | Vehículo a baja velocidad en terreno controlado, conductor de seguridad | NO | NO | Vehículo |
| 7    | Test de alcance + EMI + duración | NO | NO | Vehículo |
| 8    | Documentación final (PIN_USAGE_INVENTORY, CONEXIONES_COMPLETAS, etc.) | NO | NO | NO |

---

## 5. Estimación de impacto en código

| Archivo | Tipo | Líneas estimadas |
|---|---|---|
| `esp32/src/remote_control.h`         | Nuevo | ~80 |
| `esp32/src/remote_control.cpp`       | Nuevo | ~350 |
| `esp32/src/test_remote_control.cpp`  | Nuevo (host test) | ~150 |
| `esp32/src/main.cpp`                 | Modificado | +5 a +15 |
| `Core/Inc/can_handler.h` (Fase 4)    | Modificado opcional | +2 |
| `Core/Src/can_handler.c` (Fase 4)    | Modificado opcional | +3 |
| `Core/Src/main.c` (Fase 4)           | Modificado opcional | +5 |
| `tools/backup/*` (Fase −1, este PR)  | Nuevo | ya entregado |
| `docs/REMOTE_CONTROL_*.md`           | Actualización | varios |

**Total firmware tocado:** < 600 líneas, 90 % en módulos *nuevos y aislados*.
Toda la lógica del STM32 existente queda intacta excepto, opcionalmente, una
ventana de 5 líneas en `main.c` (Fase 4) si Fase 3 lo justifica.

---

## 6. Invariantes que se mantienen (idénticos al plan base)

1. **NO** modificar `Core/Src/safety_system.c`.
2. **NO** modificar `Core/Src/motor_control.c`.
3. **NO** modificar TIM1 / TIM8 ni ningún PWM.
4. **NO** modificar IWDG (4.1 s).
5. **NO** modificar `encoder_reader.c` ni `steering_centering.c`.
6. **NO** modificar ABS / TCS.
7. **NO** modificar `renderTask` Core 0 ni nada del TFT.
8. **NO** cambiar GPIOs ya asignados (solo añadir GPIO 16 ESP32 como iBUS RX).
9. **NO** inventar hardware ni pines no documentados.
10. **STM32 sigue siendo la única safety authority.**

---

## 7. Reversibilidad

- Tag git `pre-remote-control-v10` antes de Fase 1.
- Snapshot binario (STM32 512 KB + ESP32 16 MB) con
  `tools/backup/backup_firmware.sh` antes de Fase 1.
- Cada fase = 1 commit pequeño y autocontenido.
- Reversión total = `git revert <commits>` + `tools/backup/restore_firmware.sh`.

---

## 8. Estado actual

- [x] **Fase −1 preparada**: herramientas de backup creadas en `tools/backup/`
      y procedimiento en `docs/BACKUP_FIRMWARE_PROCEDURE.md`.
      Pendiente que el usuario las ejecute físicamente en el coche.
- [ ] Tag `pre-remote-control-v10` (a crear cuando el usuario confirme
      que el snapshot se ha hecho y guardado).
- [ ] Fase 0 a Fase 8 — **bloqueadas a la espera de autorización
      explícita del usuario**.

---

## 9. Preguntas para el usuario antes de continuar

1. ¿Confirmas que vas a ejecutar `tools/backup/backup_firmware.sh` en tu
   PC con el coche conectado y guardar el directorio `backups/firmware_*/`
   en dos sitios distintos?
2. ¿Pido el FS-i6X + FS-iA6B y los pasivos (100 nF, 10 µF, ferrita
   snap-on, cable Dupont 3 pin) ya, o esperas?
3. ¿Quieres que mapee **las 10 funciones** propuestas (CH1–CH10) o
   prefieres empezar con el subconjunto mínimo CH1+CH2+CH5+CH10 y
   añadir el resto incrementalmente?
4. ¿Habilitamos CH7 (marcha por mando) o lo dejamos exclusivamente en la
   palanca física para evitar conflictos?

**Hasta recibir tus respuestas no se toca ni una línea de firmware del
mando.**
