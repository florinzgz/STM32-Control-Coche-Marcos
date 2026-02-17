# Análisis Técnico — Propuesta de Corte Duro por Proximidad en la STM32

**Fecha:** 2026-02-17  
**Estado:** Análisis técnico (sin modificaciones propuestas)  
**Alcance:** Evaluación de la viabilidad y coherencia de añadir un límite duro de tracción en la STM32 basado exclusivamente en distancia CAN, independiente de la máquina de estados SAFE y los mecanismos de timeout  
**Referencia:** `docs/OBSTACLE_SYSTEM_ARCHITECTURE.md`, `docs/OBSTACLE_SENSOR_DESIGN_DECISION.md`, `Core/Src/safety_system.c` (Obstacle_Update), `Core/Src/motor_control.c` (Traction_Update)

---

## Resumen de la propuesta

Se propone añadir en la STM32 una validación física directa de proximidad:

- La ESP32 seguiría calculando zonas, ACC y comportamiento progresivo.
- La STM32 **no confiaría en el `obstacle_scale` final calculado por la lógica actual**.
- La STM32 aplicaría un **límite duro independiente**: si `distancia < umbral_crítico` → corte inmediato de tracción, **sin depender del estado SAFE ni de timeouts**.

La intención declarada es evolucionar de *"parada por fallo del sistema"* a *"parada garantizada por proximidad física"*.

---

## 1. ¿Rompe o contradice la arquitectura de separación de dominios?

**No la rompe; la refuerza.**

La arquitectura actual separa dos dominios de fallo: la ESP32 como sistema supervisor/HMI y la STM32 como controlador de tiempo real con autoridad final. El principio rector es que **la STM32 decide, la ESP32 sugiere** (documentado en `SAFETY_SYSTEMS.md`, sección "Autoridad Final del STM32").

Un corte duro por proximidad directa en la STM32 **no viola esta separación** porque:

1. **No mueve funcionalidad de la ESP32 a la STM32.** La ESP32 sigue calculando zonas, ACC y reducción progresiva. El corte duro es un mecanismo adicional, no un reemplazo.

2. **No introduce dependencia cruzada.** El corte duro usaría exactamente el mismo campo `distance_mm` ya recibido por CAN (mensaje 0x208, bytes 0–1). No requiere datos adicionales de la ESP32.

3. **Fortalece el principio de autoridad final.** Actualmente la STM32 *ya* aplica `obstacle_scale = 0.0` cuando `distance < 200 mm` (línea 1269 de `safety_system.c`), pero lo hace **dentro** de `Obstacle_Update()`, que depende de varias precondiciones:
   - Que el módulo de obstáculo esté habilitado (service mode check, línea 1217).
   - Que `obstacle_data_valid == 1` (primera trama ya recibida, línea 1237).
   - Que no haya timeout CAN previo que ya haya disparado SAFE por otra vía (línea 1225).

   Un corte duro independiente eliminaría estas precondiciones para el caso más crítico (proximidad extrema), haciéndolo **incondicional**.

4. **Mantiene la separación CAN.** El dato sigue viniendo de la ESP32 por CAN. La STM32 no lee el sensor directamente. El aislamiento de dominios de fallo hardware se conserva intacto.

**Conclusión:** La propuesta es **coherente con la arquitectura** y no introduce acoplamiento nuevo entre dominios.

---

## 2. ¿Mejora realmente la seguridad o solo duplica funciones existentes?

**Mejora la seguridad de forma significativa, aunque superficialmente parezca una duplicación.**

### Lo que ya existe

El sistema actual tiene esta cadena de protección ante proximidad (implementada en `Obstacle_Update()`, líneas 1214–1323):

```
CAN 0x208 → Obstacle_ProcessCAN() → obstacle_distance_mm actualizado
                                          ↓
                              Obstacle_Update() (cada 10 ms)
                                          ↓
                              ¿service mode deshabilitado? → skip (scale = 1.0)
                              ¿CAN timeout? → SAFE
                              ¿primera trama recibida? → skip (scale = 1.0)
                              ¿datos stale? → SAFE
                              ¿sensor unhealthy? → SAFE
                              ¿distancia < 200 mm? → scale = 0.0 + SAFE
                                          ↓
                              safety_status.obstacle_scale = scale
                                          ↓
                              Traction_Update() → base_pwm × obstacle_scale
```

### Lo que la propuesta añadiría conceptualmente

Un corte duro **no mediado por la máquina de estados**:

```
CAN 0x208 → distance_mm actualizado
                  ↓
     ¿distance < umbral_crítico? → PWM = 0 inmediato
                                   (independiente de obstacle_scale,
                                    independiente de service mode,
                                    independiente de system_state)
```

### Por qué no es una simple duplicación

| Aspecto | Sistema actual | Con corte duro adicional |
|---------|---------------|--------------------------|
| **Dependency on service mode** | Si `MODULE_OBSTACLE_DETECT` está deshabilitado, `Obstacle_Update()` fuerza `obstacle_scale = 1.0` y **no evalúa distancia**. Un obstáculo a 100 mm no provocaría parada. | El corte duro operaría **independientemente del service mode**. Incluso con el módulo deshabilitado, una proximidad extrema seguiría provocando corte de tracción. |
| **Dependency on SAFE state machine** | El corte por distancia dispara `Safety_SetState(SYS_STATE_SAFE)` + `Safety_SetError(SAFETY_ERROR_OBSTACLE)`. Si la máquina de estados tuviera un bug (e.g., transición bloqueada, error code ya ocupado por otra falla), el efecto podría no ser inmediato. | El corte duro actuaría directamente sobre el PWM o sobre un flag que `Traction_Update()` respeta incondicionalmente, sin pasar por la máquina de estados. |
| **First-message grace period** | Antes de recibir el primer 0x208, `obstacle_data_valid == 0` → `obstacle_scale = 1.0`. Si la ESP32 enviara un primer mensaje con distancia < 200 mm, ese mensaje se procesaría normalmente. Pero si la variable no estuviera correctamente inicializada, habría una ventana. | El corte duro dependería solo de `distance_mm < umbral` con un valor por defecto seguro (e.g., `0xFFFF` = sin obstáculo). |
| **Latencia de actuación** | `Obstacle_Update()` se ejecuta cada 10 ms. En el peor caso, el CAN RX ocurre 1 µs después de la última invocación → 10 ms hasta evaluar la distancia. | Si el corte duro se evaluara en el ISR de CAN o directamente en `Traction_Update()`, la latencia sería ≤ 10 ms (igual o menor). |

La diferencia clave es de **independencia**: el corte duro no dependería de la correcta operación de la máquina de estados SAFE, de los flags de error, del service mode, ni del flujo de `Obstacle_Update()`. Es un **disyuntor de último recurso** que reduce la superficie de fallo del propio firmware de la STM32.

**Conclusión:** No es una duplicación trivial. Es una capa de defensa contra fallos **internos** del firmware de la STM32, no solo contra fallos de la ESP32 o del sensor.

---

## 3. Efectos sobre latencia, determinismo y control de motores

### 3.1 Latencia

| Métrica | Valor actual | Con corte duro |
|---------|-------------|----------------|
| CAN RX → `obstacle_distance_mm` actualizado | < 10 µs (ISR) | Sin cambio |
| `obstacle_distance_mm` → `obstacle_scale` evaluado | ≤ 10 ms (Obstacle_Update en bucle principal) | ≤ 10 ms (sin cambio si se evalúa en Traction_Update) |
| `obstacle_scale` → PWM aplicado | ≤ 10 ms (Traction_Update en el mismo ciclo) | ≤ 10 ms (mismo ciclo) |
| **End-to-end CAN RX → PWM = 0** | **≤ 20 ms** (peor caso: CAN RX justo después de Obstacle_Update, esperar al siguiente ciclo + Traction_Update) | **≤ 10 ms** (si el check se inserta directamente en Traction_Update, que ya lee la distancia) |

La latencia máxima mejoraría de ~20 ms a ~10 ms en el peor caso, si la evaluación del corte duro se realiza directamente donde se calcula el PWM. En la práctica, esta mejora de 10 ms es poco significativa a las velocidades operativas del vehículo (< 10 km/h ≈ 2.8 m/s → en 10 ms recorre 2.8 cm), pero es técnicamente correcta.

### 3.2 Determinismo

El corte duro **mejora el determinismo**:

- El flujo actual de `Obstacle_Update()` tiene múltiples ramas condicionales (`service_mode`, `data_valid`, `stale_count`, `sensor_healthy`) antes de llegar a la evaluación de distancia. Si alguna de estas condiciones tiene un estado inconsistente (e.g., race condition entre ISR de CAN y bucle principal), la evaluación de distancia podría no ejecutarse.

- Un corte duro sería una **evaluación atómica**: `if (distance_mm < umbral) → PWM = 0`. Sin ramas previas, sin dependencias de estado. Determinista por diseño.

- **Coste computacional:** Una comparación `uint16_t` adicional por ciclo de 10 ms. Impacto en tiempo de ejecución: despreciable (< 1 µs en Cortex-M4 @ 170 MHz).

### 3.3 Control de motores

El único efecto es que el corte duro produciría una **transición abrupta de PWM** (del valor actual a 0) sin rampa de deceleración. Esto es idéntico al comportamiento actual cuando `obstacle_scale = 0.0`, que también produce un corte instantáneo. A las velocidades del vehículo (< 10 km/h), la inercia mecánica del sistema de transmisión absorbe esta transición sin efecto adverso significativo (no hay riesgo de inestabilidad mecánica ni de corriente regenerativa peligrosa en los motores BTS7960).

**Conclusión:** El corte duro no tendría efectos negativos sobre la latencia, el determinismo ni el control de motores. Mejoraría marginalmente la latencia worst-case y significativamente el determinismo al eliminar dependencias de estado previas.

---

## 4. ¿Es coherente con una arquitectura supervisada?

**Sí, y de hecho es un patrón estándar en sistemas safety-critical.**

### 4.1 Analogía con sistemas industriales

En controladores industriales (IEC 61508, ISO 13849), es habitual tener:

- Un **sistema de control** que gestiona el comportamiento normal (en este caso: la cadena completa ESP32 → CAN → Obstacle_Update → obstacle_scale → Traction_Update).
- Un **circuito de seguridad independiente** que actúa como disyuntor final, evaluando una condición simple (e.g., sensor de proximidad + contacto de parada) y cortando la alimentación de actuadores sin pasar por el controlador.

La propuesta emula este patrón en software: el "circuito de seguridad" sería la evaluación directa `distance < umbral → PWM = 0`, y el "controlador" sería toda la lógica existente de zonas, escalas y máquina de estados.

### 4.2 No contradice la supervisión

La propuesta no implica que la STM32 "deje de confiar" en la ESP32. La STM32 sigue usando la distancia reportada por la ESP32 vía CAN — el dato de entrada es el mismo. Lo que cambia es que la STM32 **no confía en su propia lógica de estado** para el caso más crítico (proximidad extrema). Esto es una forma de **auto-verificación**: el controlador desconfía de su propia complejidad y establece un atajo incondicionado.

### 4.3 Principio de "Safety Integrity Level"

En terminología SIL (Safety Integrity Level), un mecanismo más simple tiene mayor integridad que uno complejo, porque tiene menos modos de fallo. La cadena actual (CAN → Obstacle_ProcessCAN → Obstacle_Update → 7 ramas condicionales → obstacle_scale → Traction_Update → PWM) tiene más puntos de fallo potencial que un check directo `distance < umbral → PWM = 0`. Aunque ambos actúan sobre el mismo dato de entrada (CAN), la evaluación simplificada tiene una **superficie de fallo más pequeña**.

**Conclusión:** El enfoque es plenamente coherente con la filosofía supervisada y de defensa en profundidad documentada en el proyecto.

---

## 5. ¿Capa adicional válida o antipatrón?

**Capa adicional válida.**

### 5.1 No es un antipatrón

Los antipatrones comunes en seguridad embebida son:

| Antipatrón | ¿Aplica a la propuesta? |
|------------|------------------------|
| **Redundancia sin independencia** (dos sistemas que fallan por la misma causa) | No. El corte duro y `Obstacle_Update()` comparten el dato de entrada (CAN 0x208), pero el corte duro **no comparte la lógica de evaluación** (no depende de service mode, data_valid, stale_count, sensor_healthy, ni de la máquina de estados). Los modos de fallo que bloquearían `Obstacle_Update()` no bloquearían el corte duro. |
| **Complejidad accidental** (añadir código que no aporta seguridad real y dificulta el mantenimiento) | Parcialmente. El corte duro añade ~10 líneas de código. La complejidad es mínima. El riesgo de confusión para un mantenedor se mitiga documentando claramente que el corte duro es una capa diferenciada de `Obstacle_Update()`. |
| **Bypass no controlado** (mecanismo que puede causar comportamiento inesperado) | Riesgo bajo. Si el corte duro actuara en un punto diferente de la cadena de PWM (e.g., directamente en el registro de timer), podría crear un conflicto con `Traction_Update()`. Pero si actúa **dentro** de `Traction_Update()` como un clamp final, el flujo es predecible y auditable. |
| **Latch-up** (un corte que no se puede recuperar) | Depende del diseño. Si el corte duro no tiene histéresis ni recuperación, podría crear oscilación en el límite del umbral. Sería necesario aplicar el mismo patrón de histéresis (200 mm → 500 mm) que ya usa `Obstacle_Update()`, o bien confiar en que la recuperación la gestione `Obstacle_Update()` y el corte duro solo actúe como clamp unidireccional (corta, no restaura). |

### 5.2 Es un patrón reconocido

El patrón de **"safety clamp"** o **"hard limit"** es estándar en:

- **Automotive (ISO 26262):** Torque limiters independientes del control de tracción principal.
- **Robótica industrial:** Zonas de seguridad hardware que cortan actuadores independientemente del PLC.
- **Aviación (DO-178C):** Monitor de envolvente de vuelo que actúa independientemente del autopiloto.

En todos estos casos, el clamp de seguridad:
- Usa la entrada más cruda posible (dato físico mínimamente procesado).
- No depende de la máquina de estados del sistema de control.
- Actúa de forma incondicional cuando se supera el umbral.
- Es lo más simple posible (una comparación, una acción).

La propuesta encaja exactamente en este patrón.

### 5.3 Limitación inherente

La limitación principal de esta propuesta es que **el dato sigue viniendo de la ESP32 por CAN**. Si la ESP32 envía una distancia falsa (e.g., 5000 mm cuando el obstáculo real está a 100 mm), ningún mecanismo en la STM32 puede detectar esa inconsistencia, porque la STM32 no tiene acceso directo al sensor. Esto no es un defecto de la propuesta — es una limitación de la arquitectura split que ya existía antes y que se mitiga por:

1. El rolling counter y checksum que validan la integridad de las tramas CAN.
2. La topología punto-a-punto del bus CAN (sin terceros que puedan inyectar datos falsos).
3. La imposibilidad práctica de un bug en la ESP32 que envíe distancias consistentemente falsas sin ser detectado por otros mecanismos (heartbeat, stale counter).

---

## Diagrama: flujo actual vs. propuesta

```
                    FLUJO ACTUAL
                    ════════════

CAN RX (0x208) ──► Obstacle_ProcessCAN()
                         │
                         ▼
                   obstacle_distance_mm
                   obstacle_sensor_healthy
                   obstacle_stale_count
                         │
                         ▼
                   Obstacle_Update()  [cada 10 ms]
                   ┌─ service mode? → skip (scale=1.0)
                   ├─ CAN timeout?  → SAFE
                   ├─ data valid?   → skip (scale=1.0)
                   ├─ stale data?   → SAFE
                   ├─ unhealthy?    → SAFE
                   └─ distance mapping:
                      <200  → scale=0.0 + SAFE
                      <500  → scale=0.3
                      <1000 → scale=0.7
                      ≥1000 → scale=1.0
                         │
                         ▼
                   safety_status.obstacle_scale
                         │
                         ▼
                   Traction_Update()
                   base_pwm × obstacle_scale × ackermann × wheel_scale
                         │
                         ▼
                      Motor PWM


          FLUJO CONCEPTUAL CON CORTE DURO ADICIONAL
          ═══════════════════════════════════════════

CAN RX (0x208) ──► Obstacle_ProcessCAN()
                         │
                    ┌─────┴─────────────────────┐
                    ▼                             ▼
          obstacle_distance_mm             (flujo existente)
                    │                        Obstacle_Update()
                    │                             │
                    ▼                             ▼
          Traction_Update()              obstacle_scale
                    │                             │
                    ▼                             ▼
          ┌─────────────────────────────────────────────┐
          │  if (distance < UMBRAL_CRITICO)              │  ◄── CORTE DURO
          │      final_pwm = 0;     /* incondicional */  │      (nuevo)
          │  else                                        │
          │      final_pwm = base_pwm × obstacle_scale   │  ◄── flujo existente
          │                 × ackermann × wheel_scale;   │
          └─────────────────────────────────────────────┘
                         │
                         ▼
                      Motor PWM
```

---

## Resumen ejecutivo

| Pregunta | Evaluación |
|----------|------------|
| 1. ¿Rompe la arquitectura de separación de dominios? | **No.** La refuerza al hacer que la STM32 aplique su autoridad final de forma incondicional. |
| 2. ¿Mejora la seguridad o duplica funciones? | **Mejora la seguridad.** Protege contra fallos internos del firmware STM32 (bugs en service mode, máquina de estados, flags de error) que la capa actual no cubre. |
| 3. ¿Efectos sobre latencia/determinismo/control? | **Neutros a positivos.** Latencia worst-case mejora ~10 ms. Determinismo mejora al eliminar dependencias de estado. Coste computacional despreciable. |
| 4. ¿Coherente con arquitectura supervisada? | **Sí.** Es un patrón estándar en safety-critical (ISO 26262, IEC 61508). No contradice la supervisión existente. |
| 5. ¿Capa válida o antipatrón? | **Capa válida.** Encaja en el patrón "safety clamp". La única precaución es el diseño de la histéresis/recuperación para evitar oscilación en el límite del umbral. |

---

*Documento generado: 2026-02-17*  
*Este documento evalúa una propuesta conceptual sin implementar ni proponer cambios al código existente.*
