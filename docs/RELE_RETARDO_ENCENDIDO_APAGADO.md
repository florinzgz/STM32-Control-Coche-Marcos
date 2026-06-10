# ~~Relé de Retardo (Retención) — Puesta en Marcha y Apagado~~

> # ⛔ DOCUMENTO OBSOLETO
>
> **Este documento describe un circuito discreto con transistores NPN (Q1, Q2 tipo
> 2N2222), diodos OR (1N4148) y diodo flyback (1N4007) para controlar la bobina
> del relé de retención desde la llave y desde GPIO 41 (POWER_HOLD).**
>
> **DECISIÓN DE ARQUITECTURA ADOPTADA (2026-04-29): OPCIÓN A — MÓDULO TEMPORIZADOR AUTÓNOMO**
>
> El circuito discreto descrito en este documento ha sido **reemplazado por un módulo de
> retardo hardware autónomo** (potenciómetro, delay-OFF). El módulo funciona completamente
> en hardware sin componentes discretos adicionales:
>
> - `GPIO 41 (POWER_HOLD)` — uso **interno del firmware** únicamente. NO se conecta a ningún relé físico externo.
> - NO se construye ningún circuito con Q1, Q2, R4, R_base, D_OR1, D_OR2.
>
> **Referencia válida:** `docs/LLAVE_CONTACTO_ENCENDIDO_APAGADO.md` §3 (módulo retardo hardware).
>
> El contenido siguiente se conserva solo como referencia histórica del diseño anterior.
> **NO implementar.**

---

~~**Resumen de referencia rápida — Circuito y lógica completa**~~

> **Fuentes (históricas):**
> - `esp32/src/power_manager.h` / `power_manager.cpp` (GPIO 40/41, state machine)
> - `Core/Src/safety_system.c` (Relay_PowerUp / Relay_PowerDown)
> - `docs/LLAVE_CONTACTO_ENCENDIDO_APAGADO.md` (circuito completo + diagramas)
> - `docs/IGNITION_KEY_CIRCUIT_VALIDATION.md` (validación y correcciones de diseño)
> - `docs/POWER_DISTRIBUTION.md` (temporizaciones firmware)

---

## 1. ¿Qué es el relé de retardo / retención?

El **relé de retención** (`delay relay` en la documentación inglesa) es el componente
externo que **mantiene la alimentación del sistema encendida** durante el proceso de
apagado, dando tiempo al ESP32-S3 para:

1. Reproducir el audio de despedida (~3 segundos)
2. Guardar la configuración en flash (`config_store::flush()`)
3. Ejecutar la secuencia de apagado ordenada

Sin este relé, al girar la llave a OFF el sistema perdería alimentación
instantáneamente y no podría hacer ninguna tarea de cierre.

---

## 2. Principio de funcionamiento

El relé de retención tiene **dos caminos de activación en paralelo** (circuito OR):

```
Camino 1 — ENCENDIDO INICIAL (desde la llave):
  Llave ON → R4 (4.7 kΩ) → Base Q1 (2N2222) → Q1 saturado → Bobina energizada

Camino 2 — RETENCIÓN DURANTE APAGADO (desde ESP32):
  GPIO 41 (POWER_HOLD) HIGH → R_base (1 kΩ) → Base Q2 (2N2222) → Q2 saturado → Bobina energizada

Combinación OR (sin retroalimentación):
  Salida Q1 ──► D_OR1 (1N4148) ──┐
                                   ├──► Bobina relé (+)
  Salida Q2 ──► D_OR2 (1N4148) ──┘

Protección kickback:
  D2 (1N4007) EN PARALELO con bobina (cátodo → bobina+, ánodo → bobina−/GND)
```

**Resultado:** El relé permanece cerrado (sistema alimentado) mientras
**cualquiera** de los dos caminos esté activo. Solo se abre cuando
**ambos** se desactivan (llave OFF Y GPIO 41 LOW).

---

## 3. Pines ESP32-S3 involucrados

| Pin | Nombre firmware | Dirección | Función |
|-----|----------------|-----------|---------|
| **GPIO 40** | `PIN_IGNITION_SENSE` | Entrada (pull-down) | Lee posición de la llave (HIGH = ON, LOW = OFF) |
| **GPIO 41** | `PIN_POWER_HOLD` | Salida | Mantiene el relé energizado (HIGH = retener alimentación) |

**Tensión en GPIO 40 con divisor (R1=33 kΩ, R2=10 kΩ):**
- Llave ON (12 V): GPIO 40 = **2.79 V** ✅ (dentro del rango HIGH del ESP32-S3)
- Llave OFF: GPIO 40 = **0 V** (pull-down interno)

---

## 4. Estados de la máquina de estados (power_manager.cpp)

```
                    ┌──────────────────────────────────────┐
                    │         POWER STATE MACHINE          │
                    └──────────────────────────────────────┘

  LLAVE ON detectada           STARTUP_DELAY (200 ms)        3000 ms transcurridos
  GPIO 40 HIGH (50ms debounce) │                             │
       │                       │                             │
       ▼                       ▼                             ▼
  ┌──────────┐           ┌──────────┐           ┌──────────────┐           ┌─────┐
  │ POWER    │──────────►│ STARTING │──────────►│   RUNNING    │──────────►│ OFF │
  │  HOLD    │           │          │           │              │           │     │
  └──────────┘           └──────────┘           └──────┬───────┘           └─────┘
  GPIO 41 → HIGH                                       │
  (retención activa)                           LLAVE OFF detectada
                                               GPIO 40 LOW (50ms debounce)
                                                         │
                                                         ▼
                                                ┌────────────────┐
                                                │ SHUTTING_DOWN  │
                                                │                │
                                                │ Flush config   │
                                                │ Audio despedida│
                                                │ GPIO 41 HIGH   │
                                                │ (sigue retenido│
                                                │  durante 3 s)  │
                                                └────────────────┘
```

---

## 5. Secuencia completa de ENCENDIDO

### Línea temporal

```
t = 0        t ~ 50ms     t ~ 100ms    t ~ 300ms    t ~ 560ms    t ~ 800ms    t ~ 850ms
   │             │             │            │            │            │            │
   ▼             ▼             ▼            ▼            ▼            ▼            ▼
LLAVE ON     Relé de       ESP32 y      ESP32       ESP32       STM32        STM32
             retención     STM32        GPIO 41     RUNNING     Relay        relés
             cierra        arrancan     HIGH        Audio       PowerUp()    completos
             → 5 V OK      (HW reset)   (retención  bienvenida  TRAC→DIR     ACTIVE
                                        activa)
```

### Paso a paso

| Paso | t | Responsable | Acción |
|------|---|-------------|--------|
| 1 | 0 | Llave | Gira a ON → 12 V llega a R4 → Q1 saturado → Bobina relé energizada |
| 2 | 0 | Relé retención | Cierra contactos COM→NO → 12 V a regulador → 5 V disponible |
| 3 | 0 | Regulador 5 V | Convierte 12 V → 5 V → alimenta ESP32-S3 + STM32 Nucleo |
| 4 | ~50 ms | ESP32 | `setup()` ejecuta: init pantalla TFT, CAN, PSRAM (510 ms total) |
| 5 | ~50 ms | STM32 | Arranca, inicia periféricos (ADC, I2C, CAN, TIM, GPIO) |
| 6 | ~100 ms | ESP32 | `power_mgr::init()` → lee GPIO 40 HIGH → detecta llave ON |
| 7 | ~100 ms | ESP32 | Estado: `POWER_HOLD` → **GPIO 41 = HIGH** → retención activa |
| 8 | ~100 ms | ESP32 | Estado: `STARTING` (inmediato tras POWER_HOLD) |
| 9 | ~300 ms | ESP32 | Estado: `RUNNING` (200 ms `STARTUP_DELAY` cumplidos) |
| 10 | ~300 ms | ESP32 | Reproduce audio de bienvenida via DFPlayer Mini |
| 11 | ~560 ms | ESP32 | Entra en `loop()` → envía heartbeat CAN 0x011 (cada 100 ms) |
| 12 | — | STM32 | Recibe primer heartbeat → BOOT → STANDBY → ACTIVE |
| 13 | +0 ms | STM32 | `Relay_PowerUp()` → **RELAY_TRAC ON** (PC11 HIGH) |
| 14 | +50 ms | STM32 | Espera 50 ms inrush → **RELAY_STEER_PWR ON** (PC12 HIGH) |
| 16 | — | STM32 | Estado ACTIVE → motores disponibles |

### Protecciones durante el encendido

- **Inhibición de pedal:** Si el pedal está pisado al arrancar, los motores
  permanecen bloqueados hasta que el pedal baje a < 3 % durante 400 ms
  continuos (`startup_inhibit` en `main.c:94-96`)
- **Boot validation:** 6 chequeos de sensores (batería, INA226, etc.) antes
  de permitir el estado ACTIVE (`boot_validation.h`)

---

## 6. Secuencia completa de APAGADO

### Línea temporal

```
t = 0        t ~ 50ms     t ~ 50ms     t ~ 3050ms   t ~ 3100ms
   │             │             │            │            │
   ▼             ▼             ▼            ▼            ▼
LLAVE OFF    ESP32            ESP32       ESP32        Relé
             GPIO 40          SHUTTING    GPIO 41      retención
             LOW              _DOWN       LOW          se abre
             (50ms debounce)  Audio       (POWER_HOLD  → Sin
                              despedida   liberado)    alimentación
```

### Paso a paso

| Paso | t | Responsable | Acción |
|------|---|-------------|--------|
| 1 | 0 | Llave | Gira a OFF → 12 V cortado → Q1 se apaga (camino llave desactivado) |
| 2 | 0 | Relé | Q2 (GPIO 41 sigue HIGH) mantiene bobina → relé permanece cerrado |
| 3 | ~50 ms | ESP32 | GPIO 40 LOW (tras debounce 50 ms) → estado: `SHUTTING_DOWN` |
| 4 | ~50 ms | ESP32 | `config_store::flush()` → guarda configuración en flash |
| 5 | ~50 ms | ESP32 | Reproduce audio de despedida (`FAREWELL`, ~3 s) |
| 6 | ~50 ms | ESP32 | GPIO 41 sigue HIGH → relé retención sigue cerrado → sistema alimentado |
| 7 | ~3050 ms | ESP32 | `SHUTDOWN_DELAY_MS` (3000 ms) cumplidos |
| 8 | ~3050 ms | ESP32 | **GPIO 41 → LOW** → Q2 se apaga → bobina relé sin señal |
| 9 | ~3050 ms | ESP32 | Estado: `OFF` |
| 10 | ~3100 ms | Relé | Sin señal en ningún camino → bobina se desenerga → D2 absorbe kickback |
| 11 | ~3100 ms | Relé | Contactos COM→NO se abren → 12 V cortado al regulador |
| 12 | ~3100 ms | Regulador | 5 V cae a 0 V → ESP32 y STM32 pierden alimentación |
| 13 | inmediato | STM32 | GPIOs a LOW → RELAY_TRAC/DIR se abren → motores desconectados (**fail-safe**) |

### Apagado de relés de potencia (orden inverso)

```c
// safety_system.c
void Relay_PowerDown(void) {
    GPIO_RESET(PIN_RELAY_STEER_PWR);   // PC12 → Dirección OFF primero
    GPIO_RESET(PIN_RELAY_TRAC);  // PC11 → Tracción OFF último
}
// Orden INVERSO al encendido: DIR → TRAC (sin retardos entre ellos)
```

### ¿Qué pasa si se apaga con la llave durante la marcha?

1. ESP32 detecta llave OFF → `SHUTTING_DOWN` → deja de enviar heartbeats CAN
2. STM32 detecta timeout CAN (250 ms) → entra en **LIMP_HOME** (20 % potencia, 5 km/h)
3. El vehículo sigue operativo a velocidad reducida mientras el ESP32 hace el shutdown
4. Tras 3 s → GPIO 41 LOW → sistema apagado completamente

---

## 7. Componentes del relé de retención

### Lista de componentes

| Ref. | Componente | Valor | Función |
|------|-----------|-------|---------|
| K1 | Relé de retención | 12 V DC, contactos ≥ 5 A | Cierra/abre alimentación 12 V → regulador 5 V |
| Q1 | Transistor NPN | **2N2222** (disponible en inventario) | Driver bobina desde señal de llave |
| Q2 | Transistor NPN | **2N2222** (disponible en inventario) | Driver bobina desde GPIO 41 (POWER_HOLD) |
| R4 | Resistencia | **4.7 kΩ**, ¼ W | Limita I_base Q1 ≈ 2.4 mA desde señal llave 12 V |
| R_base | Resistencia | **1 kΩ**, ¼ W | Limita I_base Q2 ≈ 2.6 mA desde GPIO 41 (3.3 V) |
| D2 | Diodo flyback | **1N4007** (1000 V, 1 A) | **EN PARALELO** con bobina → absorbe kickback inductivo |
| D_OR1, D_OR2 | Diodos OR | **1N4148** × 2 | Combinan los dos caminos sin retroalimentación |
| Regulador 5 V | LM7805 o módulo buck | 12 V → 5 V, ≥ 1.5 A | Alimenta ESP32-S3 y STM32 Nucleo |

### ⚠️ Advertencia crítica — Flyback PARALELO, no en serie

```
✅ CORRECTO:                    ❌ INCORRECTO (destruye GPIO):
                                
  +12V ──► bobina+ ──►           +12V ──► D2 ──► bobina+ ──►
                │                         (serie)
              ┌─┴─┐                                │
              │D2 │  (flyback                    bobina−
              │   │   en paralelo)                  │
              └─┬─┘                               GND
                │
             bobina−
                │
               GND

  Al cortar: energía inductiva     Al cortar: pico de -60V a -200V
  se disipa en D2 (circuito         se propaga al GPIO 40 del ESP32
  cerrado bobina + D2).             → destrucción del pin ESP32.
```

---

## 8. Temporizaciones firmware (resumen)

| Constante | Valor | Archivo | Descripción |
|-----------|-------|---------|-------------|
| `STARTUP_DELAY_MS` | 200 ms | `power_manager.cpp` | Delay ESP32 entre POWER_HOLD y RUNNING |
| `SHUTDOWN_DELAY_MS` | 3000 ms | `power_manager.cpp` | Tiempo retención tras llave OFF (audio despedida) |
| Debounce llave | 50 ms | `power_manager.cpp` | Filtro software anti-rebote en GPIO 40 |
| `RELAY_TRAC_SETTLE_MS` | 50 ms | `project_config.h` | Espera inrush antes de activar RELAY_STEER_PWR |
| `RELAY_TRACTION_SETTLE_MS` | 20 ms | `project_config.h` | Espera supresión arco entre RELAY_TRAC y RELAY_STEER_PWR |
| Timeout heartbeat CAN | 250 ms | `safety_system.c` | Sin heartbeat ESP32 → LIMP_HOME |

---

## 9. Esquema de conexión resumido

```
                    ┌──────────────────────────────────────────────────────────┐
                    │              CIRCUITO RELÉ DE RETENCIÓN                  │
                    │                                                          │
   BAT 12V (+) ─────┼──── Llave contacto (SPST) ────────────── R4 (4.7 kΩ) ───┤
                    │                                                │         │
                    │                    R1 (33 kΩ)                 │         │
                    │                        │                   Base Q1      │
                    │                    R3 (10kΩ) + C1 (100nF)    │(2N2222)  │
                    │                        │                   Col ← Bobina │
                    │                   GPIO 40 ESP32                │    (+)  │
                    │                   (IGNITION_SENSE)          Emisor       │
                    │                    R2 (10 kΩ)                 │         │
                    │                        │                      GND        │
                    │                       GND                                │
                    │                                                          │
                    │   GPIO 41 ESP32 ─── R_base (1 kΩ) ─── Base Q2 ─────────┤
                    │   (POWER_HOLD)                              │            │
                    │                                         Col ← Bobina(−) │
                    │                                             │            │
                    │                                          Emisor          │
                    │                                             │            │
                    │                                            GND           │
                    │                                                          │
                    │   D_OR1 (1N4148): Q1 colector → Bobina(+)              │
                    │   D_OR2 (1N4148): Q2 colector → Bobina(+)              │
                    │   D2 (1N4007): EN PARALELO con bobina (flyback)        │
                    │                                                          │
                    │   Bobina(+) ← BAT 12V (+)                              │
                    │                                                          │
                    │   Contactos relé K1:                                    │
                    │     COM ← BAT 12V (+)                                   │
                    │     NO  ──► Regulador 5V ──► ESP32-S3 + STM32           │
                    │     NC  ── (sin conectar)                               │
                    └──────────────────────────────────────────────────────────┘
```

---

## 10. Referencias en el código fuente

| Archivo | Líneas relevantes | Descripción |
|---------|------------------|-------------|
| `esp32/src/power_manager.h` | 10, 11 | `PIN_IGNITION_SENSE = 40`, `PIN_POWER_HOLD = 41` |
| `esp32/src/power_manager.cpp` | 52–73 | `init()` — lectura inicial de llave, setup GPIO 41 |
| `esp32/src/power_manager.cpp` | 83–88 | Transición a POWER_HOLD, GPIO 41 HIGH |
| `esp32/src/power_manager.cpp` | 91–95 | Transición a STARTING |
| `esp32/src/power_manager.cpp` | 98–104 | Transición a RUNNING (tras STARTUP_DELAY) |
| `esp32/src/power_manager.cpp` | 115–118 | Detección llave OFF → SHUTTING_DOWN |
| `esp32/src/power_manager.cpp` | 126–129 | GPIO 41 LOW → estado OFF |
| `esp32/src/main.cpp` | 593–596 | Reproducción audio bienvenida |
| `esp32/src/main.cpp` | 602–603 | flush config + audio despedida |
| `Core/Src/safety_system.c` | 467 | `RELAY_TRAC` ON |
| `Core/Src/safety_system.c` | 482–484 | Espera 50 ms → `RELAY_TRAC` ON |
| `Core/Src/safety_system.c` | 490–492 | Espera 20 ms → `RELAY_STEER_PWR` ON |
| `Core/Src/safety_system.c` | 503 | `Relay_PowerDown()` — orden inverso DIR→TRAC→MAIN |
| `Core/Inc/project_config.h` | — | `RELAY_TRAC_SETTLE_MS = 50`, `RELAY_STEER_PWR_SETTLE_MS = 20` |

---

## 11. Documentos relacionados

> ⛔ Este documento es OBSOLETO. Ver el banner al inicio.

- `docs/LLAVE_CONTACTO_ENCENDIDO_APAGADO.md` — **Referencia válida** (módulo retardo autónomo)
- ~~`docs/IGNITION_KEY_CIRCUIT_VALIDATION.md`~~ — **OBSOLETO** (diseño discreto Q1/Q2 supersedido)
- `docs/POWER_DISTRIBUTION.md` — Distribución de potencia y temporizaciones de relés
- `docs/ALIMENTACION_BUCK_INRUSH_PROTECTION.md` — Protección inrush para Bucks 5V lógica y LEDs
- `Documentos/SISTEMA_ALIMENTACION_COMPLETO.md` — BOM completo del sistema de alimentación
