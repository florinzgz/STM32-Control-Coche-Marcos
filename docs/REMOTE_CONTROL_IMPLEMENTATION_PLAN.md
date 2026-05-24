# Remote Control — Plan de implementación por fases

> Plan profesional por fases para integrar el control remoto en MarcosDashboard v10 Final.
> **Cada fase es una puerta (gate) que debe pasar antes de avanzar.**
> NINGUNA fase se ejecuta sin autorización explícita del usuario.
>
> Estado actual: **Fase 0 NO iniciada**. Documento de planificación.

---

## Resumen de fases

| Fase | Nombre | Toca firmware | Toca hardware | Duración estimada |
|---|---|---|---|---|
| 0 | Adquisición y validación de hardware | NO | SÍ (banco) | 3–5 días |
| 1 | Parser iBUS aislado (módulo nuevo, desactivado) | SÍ (solo nuevo módulo) | NO | 2–3 días |
| 2 | Integración ESP32 (3 líneas en main.cpp) | SÍ (mínimo) | NO | 1 día |
| 3 | Bench testing motor desconectado | NO | SÍ | 1–2 días |
| 4 | (Opcional) Override STM32 — 5 líneas main.c | SÍ (mínimo) | NO | 1 día |
| 5 | Bench testing motor conectado, ruedas en aire | NO | SÍ | 1–2 días |
| 6 | Test vehículo a baja velocidad, terreno controlado | NO | SÍ | 2–3 días |
| 7 | Test alcance + EMI + duración | NO | SÍ | 1–2 días |
| 8 | Documentación final + integración en `PIN_USAGE_INVENTORY.md` y BOM | NO | NO | 1 día |

> Las duraciones son orientativas. **No están aprobadas — no comprometerse a tiempos.**

---

## Fase 0 — Adquisición y validación de hardware

**Objetivo:** confirmar que el hardware comprado es el correcto antes de tocar
una sola línea de firmware.

### Tareas

- [ ] Pedir **FlySky FS-i6X + FS-iA6B** (bundle, ~50 €).
- [ ] **NO COMPRAR** el conjunto de la foto (FS-i6 + FS-iA6) — el FS-iA6 no tiene iBUS.
- [ ] Comprar también: 1× condensador 100 nF X7R, 1× condensador 10 µF, 1× ferrita
  snap-on, 1 cable Dupont 3 pin.
- [ ] Verificar visualmente que el receptor tiene el puerto **SENS/iBUS** (separado
  de los 6 canales PWM).
- [ ] Encender mando y receptor en bench, hacer bind.
- [ ] Configurar failsafe en el menú del mando:
  - CH1 = 1500 (steering centro)
  - CH2 = 1450 (throttle 0%)
  - CH5 = kill switch ON
  - CH10 = LOCAL
- [ ] Alimentar receptor con 5 V de bench power supply.
- [ ] Conectar pin S del puerto iBUS a un USB-Serial (FTDI) a 115200 8N1.
- [ ] Usar PuTTY / minicom / pyserial para capturar bytes RX.
- [ ] Confirmar tramas: header `0x20 0x40`, longitud 32, cadencia ~7 ms, checksum válido.
- [ ] Probar failsafe: apagar mando → verificar que tras ~1 s los canales saltan a las
  posiciones configuradas.

### Criterio de salida (gate)

- [ ] Tramas iBUS válidas leídas en PC.
- [ ] Failsafe hardware del receptor verificado.
- [ ] Sin avanzar a Fase 1 hasta validar esto.

---

## Fase 1 — Parser iBUS aislado (módulo nuevo, desactivado por defecto)

**Objetivo:** crear el módulo `remote_control` sin enlazarlo al sistema.

### Tareas

- [ ] Crear `esp32/src/remote_control.h` con:
  - `void init(int rxPin)`
  - `void update()`
  - `bool isActive()` — true si parser ACTIVE + CH10=REMOTE
  - `float getThrottlePct()` (0..100)
  - `float getSteeringDeg()` (−30..+30)
  - `bool isKillSwitchActive()`
  - `enum FailsafeState { IDLE, ACTIVE, DEGRADED, FAILSAFE }`
  - `FailsafeState getState()`
  - Diagnostic counters (frames OK, checksum fails, timeouts)
- [ ] Crear `esp32/src/remote_control.cpp`:
  - Buffer circular interno para los 32 bytes
  - Sincronización con header `0x20 0x40`
  - Cálculo y verificación de checksum
  - FSM de failsafe con timestamps
  - Deadzone ±30, EMA α=0,15, ramp 50/100 %/s
- [ ] Compile flag: `#define REMOTE_CONTROL_ENABLED 0` por defecto — desactivado.
- [ ] Añadir tests host opcionales en `esp32/src/test_remote_control.cpp` (paralelo a
  los `test_*.cpp` existentes), que validen parser con fixtures de tramas conocidas.

### Criterio de salida

- [ ] Módulo compila sin tocar nada más del proyecto.
- [ ] Test host con fixtures pasa (checksum válido / inválido / header desincronizado).
- [ ] Sin avanzar a Fase 2 hasta esto.

---

## Fase 2 — Integración ESP32 (3 líneas)

**Objetivo:** llamar al módulo desde `main.cpp`.

### Tareas

- [ ] En `esp32/src/main.cpp`:
  - `#include "remote_control.h"` (1 línea)
  - `remote_control::init(16)` en `setup()` (1 línea)
  - `remote_control::update()` en `loop()` (1 línea)
- [ ] Bajo `#if REMOTE_CONTROL_ENABLED`, añadir el envío de frames 0x100/0x101
  cuando `remote_control::isActive()` sea `true`:
  - Frame 0x100 con throttle byte
  - Frame 0x101 con steering int16 LE
  - Cadencia 50 ms (junto al heartbeat actual)
- [ ] Activar `REMOTE_CONTROL_ENABLED 1` solo en compilación de bench.

### Criterio de salida

- [ ] Firmware compila sin warnings nuevos.
- [ ] Con `REMOTE_CONTROL_ENABLED 0` el binario es idéntico funcionalmente al actual.
- [ ] Logs del ESP32 (Serial monitor) muestran tramas iBUS recibidas cuando el mando
  está encendido.

---

## Fase 3 — Bench testing motor desconectado

**Objetivo:** verificar generación de frames CAN sin riesgo.

### Tareas

- [ ] Desconectar físicamente el motor del BTS7960 (o desconectar el habilitador EN).
- [ ] Conectar STM32 + ESP32 por CAN, alimentar normalmente.
- [ ] Sniffer CAN en el bus (analizador lógico o segundo MCU).
- [ ] Encender mando, mover joysticks, verificar:
  - Frame 0x100 aparece con bytes correctos
  - Frame 0x101 aparece con bytes correctos
  - Cadencia regular ~50 ms
  - STM32 ACK / valores reflejados en telemetría 0x200+
- [ ] Verificar failsafe:
  - Apagar mando → frames 0x100/0x101 cesan en < 200 ms
  - STM32 heartbeat se mantiene
- [ ] Verificar arbitraje pedal vs mando (Opción A):
  - Mando en LOCAL → no se envían frames; pedal local funciona
  - Mando en REMOTE + pedal en reposo → demand sigue al mando
  - Mando en REMOTE + pedal apretado → competencia (último valor 50 ms)

### Criterio de salida

- [ ] Frames CAN correctos en todas las condiciones.
- [ ] Failsafe verificado < 200 ms.
- [ ] Decisión: ¿Opción A es suficiente o se necesita Opción B?

---

## Fase 4 (opcional) — Override STM32 (5 líneas)

**Solo si Fase 3 demostró necesidad de override.**

### Tareas

- [ ] En `Core/Inc/can_handler.h`: declarar `extern volatile uint32_t can_throttle_last_rx_ms;`
  y `extern volatile uint8_t can_throttle_override;`.
- [ ] En `Core/Src/can_handler.c` handler de `CAN_ID_CMD_THROTTLE`: tras la validación,
  setear `can_throttle_last_rx_ms = HAL_GetTick(); can_throttle_override = 1;`.
- [ ] En `Core/Src/main.c` tick_50ms, sustituir la línea `Traction_SetDemand(...)` del
  branch ACTIVE por:
  ```c
  if (can_throttle_override && (HAL_GetTick() - can_throttle_last_rx_ms) < 200) {
      /* CAN throttle override activo */
  } else {
      can_throttle_override = 0;
      Traction_SetDemand(Safety_ValidateThrottle(Pedal_GetPercent()));
  }
  ```
- [ ] Verificar que `safety_system.c`, `motor_control.c`, watchdogs, encoder, ABS/TCS
  **no se tocan**.

### Criterio de salida

- [ ] Cambio quirúrgico, ≤ 7 líneas totales sumando los dos archivos.
- [ ] Build STM32 sin warnings nuevos.
- [ ] Reversión = revertir el commit.

---

## Fase 5 — Bench testing motor conectado, ruedas en aire

**Objetivo:** verificar comportamiento dinámico con motor real, sin riesgo de tracción.

### Tareas

- [ ] Vehículo elevado, ruedas libres de girar en el aire.
- [ ] Verificar:
  - Aceleración con mando responde, suave
  - Steering responde con E6B2 encoder, sin overshoot
  - EMA + ramp del motor suavizan la entrada del mando
  - ABS/TCS no se disparan espuriamente
  - Kill switch CH5 → motor a 0 inmediato
- [ ] Test EMI: motor al 80 % → verificar que el parser no pierde sync.

### Criterio de salida

- [ ] Sin glitches mecánicos.
- [ ] Sin pérdida de checksum > 1 % bajo carga.

---

## Fase 6 — Test vehículo a baja velocidad

**Objetivo:** validar en operación real, terreno controlado.

### Tareas

- [ ] Espacio amplio, sin obstáculos cercanos, conductor de seguridad.
- [ ] Persona con kill switch (mando) lista para intervenir.
- [ ] Maniobras lentas: avanzar, frenar, girar, marcha atrás.
- [ ] Verificar transición LOCAL ↔ REMOTE en marcha.
- [ ] Provocar pérdida de RF (cubrir antena del mando) y observar comportamiento.
- [ ] Provocar reboot del ESP32 (botón reset accesible) y observar LIMP_HOME.

### Criterio de salida

- [ ] Vehículo controlable, sin desbocamientos.
- [ ] Failsafe demostrado en condiciones reales.

---

## Fase 7 — Test alcance + EMI + duración

**Objetivo:** caracterizar límites.

### Tareas

- [ ] Test de alcance progresivo: 10, 30, 50, 100, 200 m.
- [ ] Registrar contador de checksum fails por minuto.
- [ ] Test EMI: conducir con BTS7960 a máxima corriente sostenida.
- [ ] Test de duración: 30 min continuos, monitorizar reboots, LIMP_HOME, error counters.
- [ ] Registrar todo en log CSV.

### Criterio de salida

- [ ] Alcance útil documentado.
- [ ] Sin reboots espurios en 30 min.
- [ ] Tasa de checksum fail < 5 % en peor caso.

---

## Fase 8 — Documentación final

**Objetivo:** dejar el sistema documentado profesionalmente.

### Tareas

- [ ] Actualizar `docs/PIN_USAGE_INVENTORY.md` — GPIO 16 ahora ocupado por iBUS RX.
- [ ] Actualizar `docs/CONEXIONES_COMPLETAS.md` con la sección del receptor FS-iA6B.
- [ ] Añadir pasivos del receptor a `docs/COMPONENTES_PASIVOS_REFERENCIA.md`
  (sin crear listas duplicadas — integrar en las secciones existentes según el tipo
  de componente).
- [ ] Actualizar `docs/CAN_CONTRACT_FINAL.md` si se añade algún campo nuevo (no debería).
- [ ] Entrada en `PROJECT_CHANGELOG.md` con narrativa de PR.
- [ ] Entrada en `CHANGELOG.md` (rollup conciso).
- [ ] Marcar los documentos `REMOTE_CONTROL_*.md` como "implementado y validado".

### Criterio de salida

- [ ] Toda la documentación al día.
- [ ] Sin listas inventariadas duplicadas (respetando preferencia del usuario).

---

## Resumen de archivos creados / modificados por fase

| Fase | Archivos creados | Archivos modificados | Total líneas estimadas |
|---|---|---|---|
| 0 | — | — | 0 |
| 1 | `esp32/src/remote_control.h`, `.cpp`, `test_remote_control.cpp` | — | ~400 |
| 2 | — | `esp32/src/main.cpp` | +3 |
| 3 | — | — | 0 |
| 4 (opt) | — | `Core/Inc/can_handler.h`, `Core/Src/can_handler.c`, `Core/Src/main.c` | +7 |
| 5 | — | — | 0 |
| 6 | — | — | 0 |
| 7 | logs CSV en `tools/` | — | 0 |
| 8 | — | varios docs | (no code) |

**Total firmware tocado:** ~410 líneas máximo. **Todo reversible** revirtiendo
los commits correspondientes.

---

## Reglas absolutas durante toda la implementación

1. **NO modificar** `Core/Src/safety_system.c`.
2. **NO modificar** `Core/Src/motor_control.c`.
3. **NO modificar** TIM1 / TIM8 ni ningún PWM.
4. **NO modificar** IWDG.
5. **NO modificar** `Core/Src/encoder_reader.c` ni `steering_centering.c`.
6. **NO modificar** ABS / TCS.
7. **NO modificar** `renderTask` Core 0 ni nada de UI/TFT.
8. **NO cambiar GPIOs** ya asignados. Solo añadir GPIO 16 como RX iBUS.
9. **NO inventar hardware** ni pines no documentados.
10. **El STM32 sigue siendo la única safety authority.**

Cualquier desviación a estas reglas requiere autorización explícita del usuario antes
de proceder.

---

**Estado del documento:** Plan de fases listo. Pendiente autorización del usuario para
iniciar Fase 0 (compra del hardware correcto).
