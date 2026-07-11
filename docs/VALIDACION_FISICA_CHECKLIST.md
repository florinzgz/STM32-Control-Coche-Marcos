# Validación Física — Checklist (Segundo PR de instrumentación)

**Alcance:** Item 8 del PR "instrumentación primero". Checklist de banco/vehículo para verificar en
hardware real cada comportamiento cubierto por este PR. **Este documento no cambia el firmware**:
enlaza cada prueba con la instrumentación añadida (0x315 MOTION_INHIBIT, `mode_sync` 4x4, recuperación
BUS_OFF, línea `[BOOTDIAG]`, límites EPS server-side) para que el bloqueo o el fallo sea *observable*,
no adivinado.

Todos los identificadores/valores citados están trazados al firmware de este PR; no se inventan datos.
Rellena las casillas en el banco.

---

## Preparación

- [ ] Monitor CAN conectado (decodificando 0x300 estado, **0x315 DIAG_MOTION_INHIBIT**, 0x30x diagnósticos).
- [ ] Consola serie del ESP32 abierta (para la línea `[BOOTDIAG]` y `[BOOT][INFO] Reset reason`).
- [ ] Batería con capacidad de simular caídas de tensión (opcional para pruebas UV).
- [ ] Selector físico 4x4 accesible; tira LED frontal (GPIO47) y trasera (GPIO48) visibles.

---

## 1. DEGRADED permite movimiento en fallo *degradable*

Referencia: `docs/FAULT_STATE_TORQUE_MATRIX.md` §3; `Safety_GetTractionCapFactor` (DEGRADED L1=0.80 …).

- [ ] Provoca un fallo degradable (p. ej. aviso térmico > 80 °C, o aviso de batería < 20 V, o un solo
      evento de sobrecorriente).
- [ ] 0x300 reporta **DEGRADED (3)**.
- [ ] Con pedal aplicado, **el vehículo se mueve** con par reducido según el nivel L1/L2/L3.
- [ ] **0x315**: `reason` NO contiene `PWM_ZERO (0x100)` mientras hay demanda; `effective` (byte5) y
      `max PWM %` (byte6) son > 0; `TORQUE_LIMITED (0x200)` puede estar activo (informativo) y byte7
      bits4-7 indican el nivel degradado.

Resultado: ☐ Pasa ☐ Falla — Notas: ____________________

---

## 2. Fallo *peligroso* mantiene demanda/PWM a cero

Referencia: matriz §3 (SAFE/ERROR → factor 0).

- [ ] Provoca un fallo peligroso (sobretemp crítico > 90 °C, batería crítica < 18 V, fallo I²C
      Código 11, ≥3 sobrecorrientes consecutivas, o paro de emergencia).
- [ ] 0x300 reporta **SAFE (4)** o **ERROR (5)**.
- [ ] Con el pedal aplicado, **NO hay movimiento**; par/PWM en cero.
- [ ] **0x315** captura el bit de bloqueo exacto: `STATE_SAFE (0x01)` o `STATE_ERROR (0x02)`, con
      `max PWM %` (byte6) = 0. *(Este es el punto clave del PR: si alguna vez se observa*
      *`DEGRADED` con demanda ≥ 3 % y `PWM_ZERO (0x100)` a la vez, ese es el bit "40 % con PWM a cero"*
      *que hay que corregir con un test host que reproduzca el estado capturado — no antes.)*

Resultado: ☐ Pasa ☐ Falla — Bits `reason` capturados: 0x______  Notas: ____________________

---

## 3. Selector 4x4 mantenido al encender arranca en 4x4

Referencia: `esp32/src/mode_sync.h` (FSM ACK + reintentos), integración en `esp32/src/main.cpp`.

- [ ] Mantén el selector físico en **4x4** durante el encendido.
- [ ] El ESP32 **no** envía el modo hasta confirmar el heartbeat del STM32 (gate por heartbeat).
- [ ] Tras el heartbeat, se envía el modo y se recibe **ACK** (`CMD_MODE`); si no llega, hay
      reintentos acotados (`MODE_SYNC_MAX_RETRIES = 3`) y luego se marca fallo (sin spamear el bus).
- [ ] El vehículo arranca en **4x4** (modo confirmado == modo del selector).

Resultado: ☐ Pasa ☐ Falla — Nº de reintentos observados: ____  Notas: ____________________

---

## 4. Rojo frontal se ve rojo (no verde)

Referencia (memoria verificada): tira frontal GPIO47 en orden **RGB**, trasera GPIO48 en **GRB**
(`esp32/src/led_controller.cpp`). El KITT/policía rojo se veía verde cuando la frontal usaba GRB.

- [ ] Activa un modo decorativo rojo (KNIGHT_RIDER = 8, o policía).
- [ ] La **tira frontal** muestra **rojo** (no verde).
- [ ] La tira trasera también muestra el color correcto.

Resultado: ☐ Pasa ☐ Falla — Notas: ____________________

---

## 5. CAN desconectado / reconectado se recupera

Referencia: `busoff_recovery.c` (Item 3); `Safety_CheckCANTimeout` → LIMP_HOME; ventana
`CAN_BUSOFF_RECOVERY_WINDOW_MS = 1000 ms`, `RECOVERY_HOLD_MS = 500 ms`.

- [ ] Desconecta el bus CAN (o el nodo ESP32) en marcha.
- [ ] 0x300 pasa a **LIMP_HOME (6)**: el vehículo sigue moviéndose a paso de peatón (20 % par,
      5 km/h, dirección completa).
- [ ] Reconecta el bus.
- [ ] La recuperación **exige heartbeats sostenidos** durante la ventana estable antes de declarar
      recuperación; **no** vuelve a ACTIVE solo por ver RUNNING un instante.
- [ ] 0x300 vuelve a **ACTIVE (2)** tras heartbeats estables.
- [ ] Con un fallo físico persistente (corto/nodo atascado) el contador de reintentos **no** se
      reinicia en bucle (no reintentos infinitos).

Resultado: ☐ Pasa ☐ Falla — Tiempo hasta recuperación: ____ s  Notas: ____________________

---

## 6. Sin pantalla blanca ni reset inesperado (ESP32)

Referencia: instrumentación de arranque `esp32/src/boot_diag.h` (Item 4); línea `[BOOTDIAG]`.

- [ ] En cada arranque, la consola serie imprime `[BOOT][INFO] Reset reason: <causa>` y la línea
      `[BOOTDIAG] reset=… abnormal=… heap_free=… heap_min=… heap_size=… heap_maxblk=…
      stack_loop=… stack_render=…`.
- [ ] Arranque normal: `reset=PowerOn abnormal=0`. **Ningún** `abnormal=1` (Panic/Watchdog/Brownout)
      en operación normal.
- [ ] `heap_min` (mínimo histórico de heap) se mantiene con margen sano; `stack_loop`/`stack_render`
      (high-watermark) no se acercan a cero.
- [ ] La pantalla TFT muestra la UI (no pantalla en blanco) de forma estable tras el arranque y tras
      reinicios repetidos.
- [ ] Ciclar la alimentación varias veces no produce `Brownout` ni `Panic` inesperados.

Resultado: ☐ Pasa ☐ Falla — `heap_min` mín. observado: ____ B  Notas: ____________________

---

## 7. Límites EPS forzados en el servidor (STM32)

Referencia: `Core/Src/eps_params.c` tabla `eps_limits[]` (Item 5).

- [ ] Envía `EPS_PARAM_OP_SET_PARAM` con un valor **fuera de rango** desde el HMI.
- [ ] El STM32 **rechaza** el valor server-side (no confía en los límites del HMI); el parámetro no
      se aplica.
- [ ] Un valor válido dentro de rango sí se acepta.

Resultado: ☐ Pasa ☐ Falla — Notas: ____________________

---

## Resumen

| Nº | Prueba | Resultado |
|----|--------|-----------|
| 1 | DEGRADED permite movimiento (fallo degradable) | ☐ |
| 2 | Fallo peligroso mantiene PWM a cero | ☐ |
| 3 | 4x4 mantenido arranca en 4x4 | ☐ |
| 4 | Rojo frontal se ve rojo | ☐ |
| 5 | CAN desconectado/reconectado se recupera | ☐ |
| 6 | Sin pantalla blanca ni reset inesperado | ☐ |
| 7 | Límites EPS server-side | ☐ |

**Referencias:** `docs/FAULT_STATE_TORQUE_MATRIX.md`, `docs/STM32_BOOT_TIME_VS_IWDG.md`,
`docs/CAN_CONTRACT_FINAL.md` (0x315), `esp32/src/mode_sync.h`, `esp32/src/boot_diag.h`,
`Core/Src/busoff_recovery.c`, `Core/Src/eps_params.c`.
