# 🛡️ Arquitectura de Seguridad — STM32G474RE + ESP32-S3

> Documento de referencia para todos los mecanismos de seguridad implementados en el
> firmware del vehículo eléctrico. Cada sección describe **qué activa** la protección,
> **qué hace**, **qué ocurre con motores y relés**, **cómo se recupera** y **por qué**
> se diseñó así.

---

## 📑 Índice

1. [Protección Hardware BREAK2/LOCKUP (TIM1 / TIM8)](#1-protección-hardware-break2lockup-tim1--tim8)
2. [Protección Software TIM3 (Dirección)](#2-protección-software-tim3-dirección)
3. [Watchdog Independiente (IWDG)](#3-watchdog-independiente-iwdg)
4. [Relés como Corte Final de Seguridad](#4-relés-como-corte-final-de-seguridad)
5. [Heartbeat CAN Mutuo](#5-heartbeat-can-mutuo)
6. [Modo LIMP_HOME](#6-modo-limp_home)
7. [Máquina de Estados del Sistema](#7-máquina-de-estados-del-sistema)
8. [Protección contra Sobrecorriente](#8-protección-contra-sobrecorriente)
9. [Protección contra Sobretemperatura](#9-protección-contra-sobretemperatura)
10. [Protección contra Subtensión de Batería](#10-protección-contra-subtensión-de-batería)
11. [Prevención de Movimiento al Encender](#11-prevención-de-movimiento-al-encender)
12. [Límites de Potencia en Modo Degradado](#12-límites-de-potencia-en-modo-degradado)
13. [Detección de Obstáculos](#13-detección-de-obstáculos)
14. [Funciones de Parada de Emergencia](#14-funciones-de-parada-de-emergencia)

---

## 1. Protección Hardware BREAK2/LOCKUP (TIM1 / TIM8)

### 🔎 Qué lo activa

La señal interna **Cortex LOCKUP** del núcleo ARM se activa cuando el procesador entra
en un estado irrecuperable (doble fault, error de bus en handler, etc.). Esta señal
está conectada por hardware a la entrada **BREAK2** de los timers avanzados TIM1 y TIM8.

### ⚙️ Qué hace

| Timer | Ruedas afectadas | Entrada BREAK |
|-------|-------------------|---------------|
| TIM1  | FL (Front-Left) / FR (Front-Right) | BREAK2 ← Cortex LOCKUP |
| TIM8  | RL (Rear-Left) / RR (Rear-Right)   | BREAK2 ← Cortex LOCKUP |

- El hardware **borra MOE** (Master Output Enable) instantáneamente, sin intervención
  del software.
- **OSSR** (Off-State Selection for Run mode) = ENABLE → las salidas se fuerzan a su
  estado inactivo durante el funcionamiento normal cuando MOE se borra.
- **OSSI** (Off-State Selection for Idle mode) = ENABLE → las salidas mantienen su
  estado inactivo incluso en modo idle, evitando que floten.
- **AutomaticOutput** = DISABLE → MOE **no** se reactiva automáticamente en el
  siguiente ciclo de actualización; requiere intervención explícita del software.

### 🔌 Qué ocurre con los motores

Los PWM de las cuatro ruedas caen a **cero** en el mismo ciclo de reloj del timer.
Los motores se detienen por inercia (frenado pasivo) sin recibir ningún pulso de
potencia adicional.

### 🔌 Qué ocurre con los relés

Los relés permanecen en su último estado hasta que el firmware (si se recupera) o el
watchdog fuerce un reset. En la práctica, si LOCKUP es permanente, el IWDG expira en
~500 ms y provoca un reset completo del MCU, lo que apaga todos los relés (estado por
defecto = OFF).

### 🔄 Cómo se recupera

1. El IWDG expira (~500 ms) y reinicia el MCU.
2. Tras el reset, todos los GPIOs vuelven a su estado por defecto (relés OFF, MOE
   desactivado).
3. El sistema arranca en estado **BOOT** y debe completar la secuencia normal para
   llegar a **ACTIVE**.

No existe recuperación sin reset porque `AutomaticOutput = DISABLE`.

### 💡 Por qué se diseñó así

Un LOCKUP indica corrupción grave del estado del procesador. Ningún handler de software
puede ejecutarse de forma fiable. La protección por hardware (BREAK2) es la única
garantía de que los PWM se desactivan en tiempo determinista (un ciclo de timer,
nanosegundos). Deshabilitar AutomaticOutput evita que un glitch transitorio reactive
los PWM antes de que el sistema se haya verificado por completo.

---

## 2. Protección Software TIM3 (Dirección)

### 🔎 Qué lo activa

TIM3 controla el motor de dirección y **no dispone de entrada BREAK** por hardware
(es un timer de propósito general, no avanzado). Por tanto, los handlers de excepción
del Cortex-M4 actúan como sustituto software:

- **HardFault_Handler**
- **BusFault_Handler**
- **UsageFault_Handler**

### ⚙️ Qué hace

Cada handler escribe directamente en los registros del timer:

```c
TIM3->CCR1 = 0;
TIM3->CCR2 = 0;
```

Esto pone el duty cycle de ambos canales a 0 %, deteniendo el motor de dirección.

### 🔌 Qué ocurre con los motores

El motor de dirección deja de recibir PWM y se detiene por inercia. No se aplica
frenado activo, pero la carga mecánica del sistema de dirección lo inmoviliza
rápidamente.

### 🔌 Qué ocurre con los relés

Los handlers de fault no modifican relés directamente. Si el fault es irrecuperable y
escala a LOCKUP, el mecanismo BREAK2 + IWDG se encarga del corte completo.

### 🔄 Cómo se recupera

- Si el fault es aislado (e.g., acceso a dirección inválida puntual), el handler puede
  registrar el error y el sistema transiciona a **SAFE** o **ERROR**.
- Si el fault es irrecuperable, el IWDG reinicia el MCU en ~500 ms.

### 💡 Por qué se diseñó así

Los timers de propósito general (TIM3) no tienen mecanismo BREAK por hardware. La
escritura directa al registro CCR desde el handler de excepción es la alternativa más
rápida posible en software (dos instrucciones store). Se cubren los tres tipos de fault
del Cortex-M4 para garantizar cobertura completa.

---

## 3. Watchdog Independiente (IWDG)

### 🔎 Qué lo activa

El IWDG se activa si el bucle principal del firmware **no refresca** el contador dentro
del periodo de timeout.

| Parámetro | Valor |
|-----------|-------|
| Prescaler | 32 |
| Reload    | 4095 |
| Reloj     | ~32 kHz (LSI) |
| Timeout   | **~500 ms** |
| Refresco  | Cada **10 ms** en el bucle principal |

### ⚙️ Qué hace

Si el contador llega a cero sin ser refrescado, el IWDG genera un **reset completo del
MCU**. Esto es una función hardware del periférico, independiente del Cortex-M4.

### 🔌 Qué ocurre con los motores

Tras el reset:

- Todos los GPIOs vuelven a su configuración por defecto (entrada flotante / analógico).
- Los timers se reinician → PWM a 0 %.
- Los motores se detienen por inercia.

### 🔌 Qué ocurre con los relés

Todos los pines de relé (PC10, PC11, PC12, PB10, PB11) vuelven a su estado por defecto
(entrada / baja impedancia) → los relés se **desactivan** → se corta la alimentación
de potencia a motores.

### 🔄 Cómo se recupera

El MCU se reinicia y arranca en estado **BOOT**. Debe completar toda la secuencia de
inicialización (periféricos OK, detección ESP32, heartbeat, sensores plausibles) antes
de permitir movimiento.

### 💡 Por qué se diseñó así

El IWDG usa un oscilador independiente (LSI) del reloj principal del sistema. Si el
HSE falla o el PLL se desbloquea, el IWDG sigue funcionando. El margen entre refresco
(10 ms) y timeout (500 ms) da 50× de margen, permitiendo jitter considerable en el
bucle principal sin falsas alarmas, pero detectando bloqueos reales rápidamente.

---

## 4. Relés como Corte Final de Seguridad

### 🔎 Qué lo activa

Los relés actúan como **última barrera física** entre la electrónica de potencia y los
motores. Se activan/desactivan por software o por reset del MCU.

### Mapa de relés

| Pin  | Función | Tensión controlada |
|------|---------|--------------------|
| PC10 | RELAY_MAIN | Potencia general |
| PC11 | RELAY_TRAC | Tracción 24 V |
| PC12 | RELAY_DIR  | Dirección 12 V |
| PB10 | RELAY_LED_L | Tiras LED 5 V (izquierda) |
| PB11 | RELAY_LED_R | Tiras LED 5 V (derecha) |

### Secuencia de activación

```
RELAY_MAIN ON → esperar 50 ms → RELAY_TRAC ON → esperar 20 ms → RELAY_DIR ON
```

La secuencia garantiza que la alimentación general esté estable antes de energizar
subsistemas de potencia.

### ⚙️ Qué hace

- En **reset del MCU**: todos los GPIOs en estado por defecto → relés **OFF** → motores
  sin alimentación.
- `Safety_PowerDown()`: borra MOE de TIM1/TIM8, fuerza todos los relés a OFF.
- Los relés cortan físicamente la alimentación, independientemente del estado de los
  PWM o del software.

### Protección del circuito de relé

- **Diodo flyback 1N4007** obligatorio en cada bobina de relé: suprime el pico de
  tensión inversa al desactivar la bobina, protegiendo el transistor de mando.
- **Snubber RC (100 Ω + 100 nF / 250 V)** en los contactos de potencia: suprime arcos
  eléctricos al abrir/cerrar, prolongando la vida del contacto y reduciendo EMI.

### 🔌 Qué ocurre con los motores

Sin alimentación de potencia, los motores no pueden girar independientemente de la
señal PWM. Esto proporciona un corte **galvánico** (físico).

### 🔄 Cómo se recupera

El firmware debe ejecutar la secuencia de activación completa (MAIN → TRAC → DIR) tras
verificar que el sistema está en condiciones seguras (estado STANDBY o superior).

### 💡 Por qué se diseñó así

Los MOSFETs del puente H pueden fallar en cortocircuito (failure mode más común). Un
relé electromecánico proporciona aislamiento galvánico que ningún semiconductor puede
garantizar al 100 %. El estado por defecto OFF en reset asegura que un MCU con firmware
corrupto no pueda energizar los motores.

---

## 5. Heartbeat CAN Mutuo

### 🔎 Qué lo activa

La pérdida de comunicación entre STM32 y ESP32 se detecta por dos mecanismos:

1. **Timeout**: no se recibe heartbeat en 250 ms.
2. **Freeze detection**: 5 tramas consecutivas con el mismo valor de contador.

### Tramas de heartbeat

| Dirección | CAN ID | Periodo | Payload |
|-----------|--------|---------|---------|
| STM32 → ESP32 | 0x001 | 100 ms | 4 bytes: `[counter, state, faults, statusFlags]` |
| ESP32 → STM32 | 0x011 | 100 ms | 1 byte: `[counter]` |

### ⚙️ Qué hace

- Si el STM32 no recibe el heartbeat del ESP32 durante 250 ms → transición a
  **LIMP_HOME**.
- Si el contador del ESP32 se repite 5 veces seguidas → transición a **LIMP_HOME**.

**Importante**: la pérdida de CAN causa LIMP_HOME, **no** SAFE. El vehículo sigue
siendo móvil con capacidades reducidas.

### 🔌 Qué ocurre con los motores

Los motores siguen operativos pero con restricciones severas (ver sección LIMP_HOME).

### 🔌 Qué ocurre con los relés

Los relés permanecen **activos**. No se corta la alimentación porque LIMP_HOME permite
movimiento limitado.

### 🔄 Cómo se recupera

- Cuando el heartbeat del ESP32 se restablece con contadores válidos e incrementales,
  el sistema puede transicionar de vuelta a **ACTIVE** (si todos los sensores son
  plausibles).

### Recuperación Bus-Off CAN

| Parámetro | Valor |
|-----------|-------|
| Intervalo de reintento | 500 ms |
| Reintentos máximos     | 10 |

### 💡 Por qué se diseñó así

Un timeout de 250 ms (2.5× el periodo de heartbeat) tolera la pérdida de 1-2 tramas
por ruido EMI sin falsa alarma. La detección de freeze cubre el caso de un ESP32
bloqueado que sigue transmitiendo desde un buffer DMA sin actualizar el contador. Se
eligió LIMP_HOME en lugar de SAFE porque una parada total en medio del tráfico puede
ser más peligrosa que continuar a velocidad reducida.

---

## 6. Modo LIMP_HOME

### 🔎 Qué lo activa

- Timeout de heartbeat CAN (250 ms sin respuesta del ESP32).
- Detección de freeze (5 tramas con contador idéntico).

### ⚙️ Qué hace

| Restricción | Valor |
|-------------|-------|
| Límite de torque | 20 % del máximo |
| Velocidad máxima | 5 km/h |
| Rampa de aceleración | 10 %/s |
| Control | Solo pedal local (sin torque vectoring de dirección) |
| Armado del pedal | 300 ms con pedal < 3 % antes de aceptar comandos |

### 🔌 Qué ocurre con los motores

Los motores de tracción operan con torque limitado al 20 %. No hay torque vectoring de
dirección: todas las ruedas reciben el mismo comando. La rampa de 10 %/s impide
aceleraciones bruscas.

### 🔌 Qué ocurre con los relés

Todos los relés permanecen **activos**. La alimentación de potencia no se corta.

### 🔄 Cómo se recupera

1. El enlace CAN se restablece (heartbeat válido con contadores incrementales).
2. Se verifica que los sensores reportan valores plausibles.
3. El sistema transiciona a **ACTIVE**.

### Armado del pedal en LIMP_HOME

Al entrar en LIMP_HOME, el sistema requiere que el pedal esté por debajo del 3 %
durante 300 ms continuos antes de aceptar cualquier comando de torque. Esto previene
aceleraciones involuntarias si el conductor tenía el pedal presionado en el momento de
la transición.

### 💡 Por qué se diseñó así

LIMP_HOME existe porque una parada total (SAFE) al perder comunicación CAN puede ser
más peligrosa que mantener movilidad mínima: el vehículo podría estar en una
intersección o en una pendiente. El límite de 5 km/h y 20 % de torque permite al
conductor mover el vehículo a un lugar seguro sin riesgo significativo.

---

## 7. Máquina de Estados del Sistema

### Estados

| Estado | Condición de entrada | Comandos permitidos | Movimiento |
|--------|---------------------|---------------------|------------|
| **BOOT** | Power-on | Ninguno | No |
| **STANDBY** | Periféricos OK, ESP32 detectado | Limitados | No |
| **ACTIVE** | Heartbeat ESP32 + sensores plausibles | Completos | Sí |
| **DEGRADED** | Fallo menor (térmico, glitch de sensor) | Limitados (70 % potencia) | Sí |
| **SAFE** | Sobrecorriente, sobretemperatura, watchdog | Ninguno | No |
| **ERROR** | Irrecuperable | Ninguno | No |
| **LIMP_HOME** | Timeout CAN (no SAFE) | Solo pedal local (20 %, 5 km/h) | Sí |

### Transiciones críticas de seguridad

```
ACTIVE ──── fallo menor ────────────► DEGRADED
ACTIVE ──── sobrecorriente/sobretemp ► SAFE
ACTIVE ──── CAN timeout ────────────► LIMP_HOME
DEGRADED ── fallo agravado ─────────► SAFE
DEGRADED ── fallo resuelto ─────────► ACTIVE
LIMP_HOME ─ CAN restaurado ─────────► ACTIVE
SAFE ────── reset MCU ──────────────► BOOT
ERROR ───── reset MCU ──────────────► BOOT
```

### 🔌 Qué ocurre con motores y relés en cada estado

| Estado | PWM motores | Relés |
|--------|-------------|-------|
| BOOT | 0 % | OFF |
| STANDBY | 0 % | ON (secuencia completada) |
| ACTIVE | Según comando | ON |
| DEGRADED | Limitado (ver niveles) | ON |
| SAFE | 0 %, MOE borrado | OFF (Safety_PowerDown) |
| ERROR | 0 %, MOE borrado | OFF (Safety_PowerDown) |
| LIMP_HOME | ≤ 20 % | ON |

### 💡 Por qué se diseñó así

Siete estados proporcionan granularidad suficiente para distinguir entre situaciones
que requieren parada total (SAFE), movilidad reducida (DEGRADED, LIMP_HOME) y
operación normal (ACTIVE). La separación entre SAFE y ERROR permite que SAFE sea
potencialmente recuperable (tras resolver la causa) mientras que ERROR requiere
intervención humana (reset).

---

## 8. Protección contra Sobrecorriente

### 🔎 Qué lo activa

- Corriente de un motor individual > **25 A**.
- Corriente total del sistema > **50 A**.

### ⚙️ Qué hace

1. Establece un flag de fallo de sobrecorriente.
2. Transiciona el sistema a **DEGRADED** o **SAFE** según severidad.

### 🔌 Qué ocurre con los motores

- **DEGRADED**: se reduce la potencia según el nivel de degradación (ver sección 12).
- **SAFE**: PWM a 0 %, MOE borrado. Motores sin señal.

### 🔌 Qué ocurre con los relés

- **DEGRADED**: relés permanecen ON.
- **SAFE**: `Safety_PowerDown()` fuerza todos los relés a OFF. Corte galvánico total.

### 🔄 Cómo se recupera

- Si la corriente baja por debajo del umbral, el sistema puede transicionar de DEGRADED
  a ACTIVE.
- Desde SAFE, se requiere verificar que la causa del fallo se ha resuelto. Dependiendo
  de la implementación, puede requerir reset del MCU.

### 💡 Por qué se diseñó así

25 A por motor y 50 A total protegen tanto los MOSFETs del puente H como el cableado.
Un cortocircuito en un motor puede superar estos umbrales en milisegundos; la detección
rápida y la transición a SAFE previenen daños al cableado y riesgo de incendio.

---

## 9. Protección contra Sobretemperatura

### 🔎 Qué lo activa

Sensores de temperatura monitorizados continuamente con los siguientes umbrales:

#### Umbrales generales del sistema

| Temperatura | Acción |
|-------------|--------|
| < 70 °C | Normal, sin restricciones |
| 70 – 90 °C | **DEGRADED**, reducción de potencia al 50 % |
| ≥ 90 °C | **SAFE**, parada completa |

#### Umbral por motor individual

| Temperatura | Acción |
|-------------|--------|
| ≥ 130 °C | Corte de emergencia: `wheel_scale = 0` |
| < 115 °C | Recuperación permitida (histéresis de 15 °C) |

### ⚙️ Qué hace

- Entre 70-90 °C: limita la potencia al 50 % para reducir la generación de calor.
- A 90 °C: transiciona a SAFE (parada completa).
- A 130 °C por motor: corte individual inmediato (`wheel_scale = 0`), independiente del
  estado general del sistema.

### 🔌 Qué ocurre con los motores

- **DEGRADED (70-90 °C)**: potencia reducida al 50 %, motores operativos.
- **SAFE (≥ 90 °C)**: PWM a 0 %, MOE borrado. Todos los motores detenidos.
- **Corte individual (≥ 130 °C)**: solo el motor afectado se detiene (`wheel_scale = 0`),
  los demás continúan.

### 🔌 Qué ocurre con los relés

- **DEGRADED**: relés ON.
- **SAFE**: `Safety_PowerDown()` → relés OFF.
- **Corte individual**: relés ON (el corte es por PWM, no por relé).

### 🔄 Cómo se recupera

- De DEGRADED: cuando la temperatura baja de 70 °C → ACTIVE.
- De SAFE: cuando la temperatura baja de 90 °C (puede requerir reset).
- De corte individual: cuando la temperatura del motor baja de 115 °C, se restaura
  `wheel_scale` gradualmente.

### 💡 Por qué se diseñó así

La histéresis (15 °C entre corte a 130 °C y recuperación a 115 °C) evita oscilaciones
rápidas encendido/apagado que degradan los contactos de los relés y causan estrés
térmico cíclico en los bobinados. El escalón intermedio a 70 °C permite seguir
operando con potencia reducida, dando tiempo al conductor para detenerse de forma
controlada antes de alcanzar el corte total a 90 °C.

---

## 10. Protección contra Subtensión de Batería

### 🔎 Qué lo activa

Monitorización continua de la tensión de batería (sistema 24 V):

| Tensión | Estado |
|---------|--------|
| > 20.0 V | Normal |
| 19.5 – 20.0 V | **DEGRADED** (histéresis 0.5 V) |
| < 18.0 V | **SAFE** |
| > 18.5 V | Recuperación desde SAFE permitida |

### ⚙️ Qué hace

- Entre 19.5-20.0 V: reduce la potencia para disminuir el consumo y preservar la
  batería.
- Por debajo de 18.0 V: transiciona a SAFE para evitar que la tensión caiga tanto que
  el MCU pierda alimentación de forma incontrolada.

### 🔌 Qué ocurre con los motores

- **DEGRADED**: potencia reducida según nivel de degradación.
- **SAFE**: PWM a 0 %, MOE borrado. Motores detenidos.

### 🔌 Qué ocurre con los relés

- **DEGRADED**: relés ON.
- **SAFE**: `Safety_PowerDown()` → relés OFF.

### 🔄 Cómo se recupera

- De DEGRADED: cuando la tensión supera 20.0 V (histéresis de 0.5 V).
- De SAFE: cuando la tensión supera 18.5 V (histéresis de 0.5 V).

### 💡 Por qué se diseñó así

Si la tensión de batería cae por debajo del mínimo de funcionamiento del regulador del
MCU (~16-17 V típico), el procesador se resetea de forma incontrolada. Cortando la
carga a 18.0 V se preserva suficiente tensión para que el MCU ejecute un apagado
ordenado. La histéresis de 0.5 V en cada umbral evita oscilaciones cuando la batería
está en el límite (la tensión sube al quitar carga y vuelve a caer al reponerla).

---

## 11. Prevención de Movimiento al Encender

### 🔎 Qué lo activa

Se activa **automáticamente en cada arranque** (estado BOOT). Impide cualquier comando
de torque hasta que se verifique que el pedal está en posición neutra.

### ⚙️ Qué hace

- Inhibe **todo** torque a los motores.
- Requiere que el pedal esté por debajo del **3 %** durante **400 ms continuos** para
  desbloquear.
- Una vez desbloqueado, solo se reactiva en el **siguiente reset del MCU**.

### 🔌 Qué ocurre con los motores

PWM a 0 % hasta que se complete la condición de desbloqueo. Los motores no reciben
ninguna señal de potencia.

### 🔌 Qué ocurre con los relés

Los relés se activan durante la secuencia de arranque (transición BOOT → STANDBY),
pero los motores no pueden girar porque el torque está inhibido a nivel de software.

### 🔄 Cómo se recupera

Mantener el pedal en posición neutra (< 3 %) durante 400 ms continuos. No hay otra
forma de desbloquear. Si el sensor de pedal está averiado y reporta > 3 %, el sistema
**nunca** permite movimiento.

### 💡 Por qué se diseñó así

Previene el escenario donde el conductor enciende el vehículo con el pedal presionado
(accidentalmente o por un objeto sobre el pedal). Los 400 ms son suficientes para
confirmar que la lectura es estable y no un transitorio del ADC durante el arranque.
La reactivación solo por reset MCU simplifica la lógica y evita que un fallo de
software pueda desactivar esta protección en tiempo de ejecución.

---

## 12. Límites de Potencia en Modo Degradado

### Niveles de degradación

| Nivel | Potencia general | Dirección | Tracción |
|-------|-----------------|-----------|----------|
| NONE (normal) | 100 % | 100 % | 100 % |
| L1 | 70 % | 85 % | 80 % |
| L2 | 50 % | 70 % | 60 % |
| L3 (suelo SAFE) | 40 % | 60 % | 50 % |

### 🔎 Qué lo activa

El nivel de degradación se selecciona automáticamente según la severidad del fallo:

- **L1**: fallo menor (glitch de sensor aislado, temperatura entre 70-80 °C).
- **L2**: fallo moderado (sobretemperatura 80-90 °C, subtensión 19.5-20.0 V).
- **L3**: fallo severo pero aún operable (múltiples fallos L1/L2 simultáneos, límite
  inferior antes de SAFE).

### ⚙️ Qué hace

Multiplica todos los comandos de PWM por los factores correspondientes antes de
aplicarlos a los timers. La dirección mantiene un factor mayor que la tracción porque
perder dirección es más peligroso que perder aceleración.

### 🔌 Qué ocurre con los motores

Los motores operan con potencia reducida. La respuesta del pedal se siente más lenta
y la velocidad máxima disminuye proporcionalmente.

### 🔌 Qué ocurre con los relés

Todos los relés permanecen **ON**. La reducción de potencia es exclusivamente por PWM.

### 🔄 Cómo se recupera

Al resolver la causa del fallo, el nivel de degradación baja progresivamente
(L3 → L2 → L1 → NONE).

### 💡 Por qué se diseñó así

La degradación progresiva (graceful degradation) permite que el vehículo siga operando
con capacidad reducida en lugar de detenerse completamente ante fallos menores. Los
factores diferentes para dirección y tracción priorizan la controlabilidad sobre la
velocidad: un vehículo lento pero dirigible es más seguro que uno rápido sin dirección.

---

## 13. Detección de Obstáculos

### 🔎 Qué lo activa

El sistema combina una **máquina de estados autónoma en el STM32** con información
consultiva recibida del ESP32 por CAN. La detección se basa en distancia al obstáculo.

### Zonas y factores de escala

| Zona | Distancia | Factor (`wheel_scale`) |
|------|-----------|------------------------|
| Emergency | < 200 mm | 0.0 (parada total) |
| Critical | 200 – 500 mm | 0.3 |
| Warning | 500 – 1000 mm | 0.7 |
| Caution | 1000 – 1500 mm | 0.85 |
| Alert | 1500 – 4000 mm | 0.95 |
| Clear | > 4000 mm | 1.0 (sin restricción) |

### Temporizaciones

| Parámetro | Valor |
|-----------|-------|
| Retardo de confirmación | 200 ms (evita falsos positivos por lecturas espurias) |
| Retardo de despeje | 1000 ms (evita oscilar si el obstáculo aparece/desaparece) |

### ⚙️ Qué hace

El factor `wheel_scale` multiplica el comando de torque de las ruedas en la dirección
del obstáculo. En zona Emergency (< 200 mm), el factor es 0.0: torque cero, parada
completa en esa dirección.

### Escape en reversa

Cuando el movimiento hacia adelante está bloqueado por un obstáculo, el sistema
**permite reversa** para que el conductor pueda retroceder y alejarse del obstáculo.

### 🔌 Qué ocurre con los motores

- El torque se reduce proporcionalmente al factor de escala.
- En Emergency: torque cero en la dirección del obstáculo.
- Los motores en dirección opuesta no se ven afectados.

### 🔌 Qué ocurre con los relés

Los relés permanecen **ON**. La reducción es exclusivamente por `wheel_scale` aplicado
al PWM.

### 🔄 Cómo se recupera

Cuando el obstáculo se aleja (o el vehículo retrocede), la distancia aumenta y el
factor de escala sube progresivamente. El retardo de despeje de 1000 ms previene
restauración prematura.

### 💡 Por qué se diseñó así

El retardo de confirmación de 200 ms filtra lecturas espurias de los sensores
ultrasónicos/ToF (reflexiones parásitas, interferencia). El retardo de despeje de
1000 ms es asimétrico (5× mayor que la confirmación) porque es preferible mantener
una restricción falsa durante 1 segundo que eliminar prematuramente una restricción
real. La reducción gradual por zonas permite al conductor aproximarse a baja velocidad
en maniobras de aparcamiento sin activar la parada de emergencia a distancias
excesivas.

---

## 14. Funciones de Parada de Emergencia

### `Safety_EmergencyStop()`

| Aspecto | Comportamiento |
|---------|---------------|
| **Activa** | Llamada por software ante condición de emergencia |
| **Motores** | Escribe 0 en todos los registros CCR de TIM1, TIM3 y TIM8 |
| **Relés** | No modifica relés |
| **Torque** | Inhibe flag de torque (impide nuevos comandos) |
| **Recuperación** | Requiere resolución del fallo + desbloqueo explícito |
| **Diseño** | Parada rápida por software manteniendo alimentación; permite diagnóstico post-evento con los relés activos |

### `Safety_FailSafe()`

| Aspecto | Comportamiento |
|---------|---------------|
| **Activa** | Fallo que requiere corte de potencia |
| **Motores** | Borra MOE de TIM1 y TIM8 (desactiva salidas hardware) |
| **Relés** | Fuerza todos los relés a OFF |
| **Torque** | Inhibido implícitamente (sin alimentación) |
| **Recuperación** | Reset del MCU |
| **Diseño** | Corte completo: MOE por seguridad inmediata + relés por aislamiento galvánico |

### `Safety_PowerDown()`

| Aspecto | Comportamiento |
|---------|---------------|
| **Activa** | Apagado ordenado o fallo irrecuperable |
| **Motores** | Borra MOE de TIM1 y TIM8 |
| **Relés** | Fuerza todos los relés a OFF |
| **Torque** | **Bloqueo permanente** (no se puede desbloquear sin reset) |
| **Recuperación** | Solo reset del MCU |
| **Diseño** | Nivel máximo de parada: combina corte eléctrico + bloqueo lógico permanente. Garantiza que incluso si un glitch reactiva MOE, el flag de bloqueo permanente impide que el software genere PWM |

### Jerarquía de parada

```
Safety_EmergencyStop()    ← Nivel 1: PWM a cero, diagnóstico posible
        ↓ (si no basta)
Safety_FailSafe()         ← Nivel 2: MOE + relés OFF
        ↓ (si irrecuperable)
Safety_PowerDown()        ← Nivel 3: MOE + relés OFF + bloqueo permanente
```

### 💡 Por qué tres niveles

Tres niveles permiten respuesta proporcional:

1. **EmergencyStop** es la más rápida (solo escrituras a registros) y permite
   diagnóstico post-evento con los relés activos.
2. **FailSafe** añade corte galvánico por relé cuando el diagnóstico no es prioritario.
3. **PowerDown** es irreversible sin reset, cubriendo el caso donde el firmware puede
   estar parcialmente corrupto y no se confía en ningún mecanismo de desbloqueo por
   software.

---

## 📊 Resumen de Capas de Protección

```
┌─────────────────────────────────────────────────────┐
│  Capa 4: BREAK2 Hardware (TIM1/TIM8)                │ ← Nanosegundos
│  Respuesta: instantánea, sin software               │
├─────────────────────────────────────────────────────┤
│  Capa 3: Handlers de Excepción (TIM3)               │ ← Microsegundos
│  Respuesta: HardFault/BusFault/UsageFault → CCR=0   │
├─────────────────────────────────────────────────────┤
│  Capa 2: Software de Seguridad                      │ ← Milisegundos
│  EmergencyStop / FailSafe / PowerDown                │
│  Máquina de estados, detección de obstáculos         │
├─────────────────────────────────────────────────────┤
│  Capa 1: Watchdog IWDG                              │ ← ~500 ms
│  Reset completo si todo lo anterior falla            │
├─────────────────────────────────────────────────────┤
│  Capa 0: Relés (corte galvánico)                    │ ← Estado por defecto
│  OFF en reset → motores sin alimentación             │
└─────────────────────────────────────────────────────┘
```

Cada capa es independiente de las superiores. Si la Capa 2 (software) falla, la Capa 1
(IWDG) fuerza un reset. Si el procesador entra en LOCKUP, la Capa 4 (BREAK2) actúa
sin esperar al IWDG. Los relés (Capa 0) proporcionan la última línea de defensa
mediante aislamiento físico.
