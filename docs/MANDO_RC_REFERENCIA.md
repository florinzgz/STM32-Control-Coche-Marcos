# Mando RC — Referencia rápida: canales, funciones y cableado

> **Solo datos verificados en el firmware.**  
> Fuentes: `esp32/src/remote_control.h`, `esp32/src/remote_control.cpp`, `esp32/src/main.cpp`, `docs/REMOTE_CONTROL_WIRING.md`.

---

## 1. Receptor y transmisor usados

| Componente | Modelo | Notas |
|---|---|---|
| Transmisor | **FlySky FS-i6X** | 10 canales, protocolo AFHDS 2A |
| Receptor | **FlySky FS-iA6B** | Salida iBUS UART 3,3 V TTL en puerto `SENS/iBUS` |

---

## 2. Tabla de canales del mando

> Los valores de µs nominales son: mínimo 1000 µs · centro 1500 µs · máximo 2000 µs.  
> Umbral "LOW" = < 1300 µs · umbral "HIGH" = > 1700 µs (definidos en `remote_control.cpp:36-37`).

| Canal | Tipo de control | Función en el coche | Rango / posiciones | CAN / acción |
|---|---|---|---|---|
| **CH1** | Joystick analógico (izq/der) | **Dirección** (volante) | −30°…+30° (deadzone ±30 µs) | `CMD_RC_OVERRIDE` 0x10A, bytes 2-3 = ángulo×10 int16 LE |
| **CH2** | Joystick analógico (arriba) | **Acelerador** | 0…100 % (solo 1500→2000 µs; <1500 = 0 %) | `CMD_RC_OVERRIDE` 0x10A, byte 1 = % uint8 |
| **CH3** | Potenciómetro o trim | **Trim de dirección** | ±5° sobre CH1 | Sumado internamente a CH1 antes de emitir CAN |
| **CH4** | — | **Reservado / ignorado** | — | No se usa en firmware |
| **CH5** | Interruptor 2 posiciones | **Kill switch (corte de movimiento)** | LOW (<1300) = ACTIVO (corta tracción y dirección) · HIGH = permitido | `remoteMotionAuthorityActive = false` |
| **CH6** | Interruptor 3 posiciones | **Modo tracción** | LOW = 2WD · MID = 4WD · HIGH = 4WD+Tank | `CMD_MODE` 0x102, flags byte: 0x00 / 0x01 / 0x03 |
| **CH7** | Interruptor 3 posiciones | **Marcha** | LOW = Reversa(1) · MID = Neutro(2) · HIGH = Adelante(3) | `CMD_MODE` 0x102, byte1 = gear; confirma con audio GEAR_* |
| **CH8** | Interruptor 2 posiciones | **Luces** ON/OFF | HIGH = ON · LOW = OFF | `CMD_LED` 0x120, bytes 0-1 = front/rear; confirma con audio LIGHTS_ON/OFF |
| **CH9** | Potenciómetro rotativo | **Volumen de audio** | 0…30 (rango DFPlayer) | `audio::setVolume()` local en ESP32; debounce 250 ms, histeresis 2 niveles |
| **CH10** | Interruptor 2 posiciones | **Soberanía LOCAL / REMOTE** (palanca maestra) | HIGH (>1700) = REMOTE (mando activo) · LOW = LOCAL (coche físico manda) | Gate global: si LOW se anula la autoridad RC; el ESP32 sigue emitiendo `0x10A` con `override_flag=0` para que el STM32 vuelva al pedal físico |

### Notas de seguridad (firmware verificado)

- **Sin CH10 en REMOTE**: no se emite ningún CAN de movimiento aunque muevas los sticks.  
- **CH5 kill switch activo**: corta `remoteMotionAuthorityActive`; CH6/7 (modo y marcha) también se bloquean pero luces y volumen (CH8/9) siguen activos porque dependen de `remoteAuthorityActive`, no del kill.  
- **Pérdida de señal RF > 150 ms**: el FSM pasa a `FAILSAFE`; `isActive()` devuelve `false`; el STM32 ve `override_flag=0` en 0x10A y vuelve al pedal físico.  
- El STM32 es siempre la autoridad de seguridad final: el ESP32 solo envía los valores, el STM32 puede rechazarlos con sus propias validaciones.

---

## 3. Cableado del receptor al ESP32-S3

Solo se necesitan **3 hilos** desde el puerto `SENS/iBUS` del receptor.

| Pin receptor `SENS/iBUS` | Destino ESP32-S3 | Notas |
|---|---|---|
| **S** (señal) | **GPIO 16** (UART0 RX reasignado) | 3,3 V TTL — sin level shifter |
| **+** (VCC) | **Rail +5 V** del proyecto | NO conectar al 3,3 V del ESP32 (receptor especificado 4,0–6,5 V) |
| **−** (GND) | **GND común** del proyecto | Trenzar este hilo con el de señal si es posible |

### Diagrama de conexión

```
FS-iA6B                           ESP32-S3
puerto SENS/iBUS
                                  ┌──────────────────────┐
  [−] GND ────────────────────────┤ GND                  │
  [+] VCC ── +5 V rail proyecto   │                      │
  [S] señal ──────────────────────┤ GPIO 16  (UART0 RX)  │
              (3,3 V TTL)         └──────────────────────┘
```

### Filtros recomendados (del doc `REMOTE_CONTROL_WIRING.md`)

| Componente | Valor | Dónde | Para qué |
|---|---|---|---|
| C1 | 100 nF cerámico X7R | Entre VCC y GND del receptor, lo más cerca posible | Filtra alta frecuencia del BTS7960 |
| C2 | 10 µF electrolítico (o cerámico) | En paralelo con C1 | Filtra ripple de alimentación |
| Ferrita snap-on | ≥100 Ω @ 100 MHz | Abrazando el cable VCC+GND a la salida del receptor | Reduce EMI del cableado |

---

## 4. Parámetros de la UART (firmware)

| Parámetro | Valor | Fuente |
|---|---|---|
| Pin RX | GPIO 16 | `remote_control.h:64`, `Config::rxPin = 16` |
| UART index | UART0 | `remote_control.h:65`, `Config::uartIndex = 0` |
| Baudrate | 115200 bps | `remote_control.h:66` |
| Protocolo | iBUS-Servo, trama 32 bytes, header 0x20 0x40 | `remote_control.cpp:22-24` |
| Periodo de trama | ~7 ms (≈143 Hz) | comentario en `remote_control.h:14` |
| Timeout failsafe | 150 ms (~21 tramas) | `remote_control.h:67`, `Config::timeoutMs = 150` |
| Canales por trama | 14 (se usan CH1–CH10; CH11–CH14 ignorados) | `remote_control.cpp:25` |

---

## 5. Suavizado aplicado a CH1 y CH2 (firmware)

Los valores brutos de los sticks no van directamente al CAN; pasan por:

1. **Deadzone**: ±30 µs alrededor de 1500 µs — elimina deriva del joystick en reposo.  
2. **EMA** (media exponencial): α = 0,15 — suaviza variaciones rápidas.  
3. **Ramp limiter** (acelerador): subida máx. 50 %/s · bajada máx. 100 %/s.  
4. **Ramp limiter** (dirección): ±90°/s (recorre ±30° en ~330 ms a tope).  

Fuente: `remote_control.cpp:28-31` y comentario `remote_control.h:108`.

---

## 6. Estados del FSM (lo que ve el coche)

| Estado | Significado | Qué pasa en el coche |
|---|---|---|
| `IDLE` | Sin ninguna trama recibida todavía | Como LOCAL, mando inactivo |
| `ACTIVE` | Tramas válidas llegando normalmente | Mando funcional (si CH10 = REMOTE) |
| `DEGRADED` | >5 checksums consecutivos malos | Emite 0 en throttle/steering; modo degradado |
| `FAILSAFE` | >150 ms sin trama válida | STM32 vuelve al pedal físico |
