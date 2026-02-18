# Modelo Matemático de Par EPS — Dirección Asistida Eléctrica

> **Alcance**: Modelo matemático exclusivamente — sin código.
> **Objetivo**: Ecuación final de par motor en función de las variables
> medibles del sistema, con explicación física de cada término.
>
> **Hardware de referencia**: E6B2-CWZ6C (4800 CPR) en TIM2,
> BTS7960/IBT-2 en TIM8 CH3 (20 kHz, ARR=4249), motor DC brushed 24 V,
> 4× sensores de velocidad de rueda (6 pulsos/rev), INA226 ch 5 (corriente motor).
>
> **Documentos previos**: `docs/EPS_TORQUE_ASSIST_ANALYSIS.md` (viabilidad hardware),
> `docs/BTS7960_MOTOR_DRIVER_AUDIT.md` (topología del driver).

---

## 1. Filosofía: EPS vs. Servo de Posición

### Lo que NO es este modelo

Un servo de posición calcula:

```
error = θ_objetivo − θ_medido
par = Kp × error + Ki × ∫error + Kd × ḋerror
```

El motor persigue un ángulo. El conductor es una perturbación.
El resultado es un volante rígido, con vibraciones microscópicas y
sensación artificial.

### Lo que SÍ es este modelo

Una EPS real calcula:

```
par_motor = f(ω_volante, θ_volante, v_vehículo, fricción, geometría)
```

El motor **ayuda** al conductor. El conductor manda. El motor amplifica
la intención humana y genera fuerzas de retorno naturales. No hay ángulo
objetivo. No hay error de posición. No hay PID.

---

## 2. Variables del Sistema

### 2.1 Variables medidas (sensores disponibles)

| Símbolo | Descripción | Sensor | Unidad | Resolución / Rango |
|---------|-------------|--------|--------|---------------------|
| θ | Ángulo del volante respecto al centro | TIM2 encoder, 4800 CPR | ° (grados) | 0.075°/count, ±54° max |
| ω | Velocidad angular del volante | Derivada numérica de θ | °/s | 7.5°/s min @ 100 Hz |
| v | Velocidad del vehículo | Promedio de 4 sensores de rueda | km/h | ~0.5 km/h mínima |
| I_motor | Corriente del motor de dirección | INA226 ch 5, TCA9548A | A | ±50 A, 20 Hz |

### 2.2 Variables derivadas (calculadas en firmware)

| Símbolo | Descripción | Cálculo | Unidad |
|---------|-------------|---------|--------|
| ω_filt | Velocidad angular filtrada | EMA sobre ω_raw | °/s |
| α_accel | Aceleración angular del volante | Derivada de ω_filt | °/s² |

### 2.3 Parámetros del modelo (constantes calibrables)

| Símbolo | Descripción | Valor sugerido | Unidad |
|---------|-------------|----------------|--------|
| K_assist | Ganancia de asistencia al giro | 0.8–2.0 | % / (°/s) |
| K_center | Rigidez del muelle virtual de autocentrado | 0.15–0.40 | % / ° |
| K_damp | Amortiguación viscosa | 0.03–0.08 | % / (°/s) |
| K_friction | Compensación de fricción estática | 2.0–4.0 | % (constante) |
| v_ref | Velocidad de referencia para escalado | 15.0 | km/h |
| ω_threshold | Umbral de velocidad angular para detectar intención | 10.0–20.0 | °/s |
| ω_blend | Ancho de la zona de transición suave | 5.0–10.0 | °/s |
| duty_min | Duty mínimo efectivo (zona muerta del motor) | 6.0–8.0 | % |

---

## 3. Ecuación Final de Par

### 3.1 Forma general

```
τ_motor = τ_assist + τ_center + τ_damp + τ_friction
```

Cada término tiene una función física distinta y solo se activa en las
condiciones apropiadas. La suma produce un par que se siente natural
para un conductor humano.

### 3.2 Ecuación completa

```
τ_motor(t) = λ(ω) × K_assist × g(v) × ω_filt
            + [1 − λ(ω)] × K_center × h(v) × θ
            + K_damp × ω_filt
            + K_friction × σ(θ, ω)
```

Donde:
- **λ(ω)** = función de mezcla basada en velocidad angular (0 a 1)
- **g(v)** = escalado de asistencia por velocidad del vehículo
- **h(v)** = escalado de autocentrado por velocidad del vehículo
- **σ(θ, ω)** = función de compensación de fricción

### 3.3 Resultado en duty cycle para el BTS7960

```
duty_pct = |τ_motor|                          (ya en %)
duty_pct = clamp(duty_pct, 0, MAX_TORQUE_PCT) (límite de seguridad)
pwm = duty_pct × PWM_PERIOD / 100            (conversión a counts)
dir = sign(τ_motor)                           (dirección del par)
en  = duty_pct > 0.5                          (habilitación del H-bridge)
```

Si `duty_pct < duty_min` y `duty_pct > 0`:
```
duty_pct = duty_min × sign_no_zero(duty_pct)  (saltar zona muerta)
```

---

## 4. Descripción Física de Cada Término

### 4.1 Par de Asistencia — τ_assist

```
τ_assist = λ(ω) × K_assist × g(v) × ω_filt
```

#### Qué hace

Amplifica la fuerza que el conductor aplica al volante. Cuando el
conductor gira, el motor empuja en la misma dirección, reduciendo
el esfuerzo necesario.

#### Física

El par de asistencia es proporcional a la **velocidad angular del
volante** (ω), no al ángulo. Esta elección es deliberada:

- **Proporcional a ω (velocidad)**: El motor ayuda cuando el conductor
  está activamente girando. Si el conductor mantiene el volante quieto
  en una curva (ω ≈ 0), el motor no empuja. Esto es correcto: en una
  curva estable, las fuerzas de contacto neumático-asfalto mantienen
  el ángulo. El motor no necesita hacer nada.

- **No proporcional a θ (posición)**: Si fuera proporcional al ángulo,
  el motor empujaría permanentemente mientras el volante esté girado,
  consumiendo energía y calentándose sin necesidad.

- **No proporcional a par del conductor (sin sensor de par)**: Un EPS
  real de coche usa un sensor de par en la columna. Este vehículo no
  lo tiene. La velocidad angular es el mejor proxy disponible: cuando
  el conductor aplica fuerza, el volante gira; la velocidad angular
  refleja la intención del conductor.

#### Unidades

```
τ_assist [%] = [adimensional] × [% / (°/s)] × [adimensional] × [°/s]
```

Ejemplo: ω_filt = 50°/s, K_assist = 1.5, g(v) = 0.8, λ(ω) = 1.0:
```
τ_assist = 1.0 × 1.5 × 0.8 × 50 = 60 %
```

El motor aplica 60 % de duty (14.4 V en bus de 24 V) en la dirección
del giro del conductor.

#### Función de mezcla λ(ω) — Detección de intención

```
         ⎧ 0                                    si |ω_filt| < ω_threshold − ω_blend
λ(ω) =  ⎨ smoothstep(|ω_filt|, ω_th−ω_bl, ω_th)  si ω_threshold − ω_blend ≤ |ω_filt| ≤ ω_threshold
         ⎩ 1                                    si |ω_filt| > ω_threshold
```

Donde `smoothstep(x, a, b) = t² × (3 − 2t)` con `t = (x − a) / (b − a)`.

**Propósito**: Evitar que ruido del encoder o vibraciones mecánicas
activen la asistencia. La transición suave (smoothstep) previene
cambios bruscos de par.

Con `ω_threshold = 15°/s` y `ω_blend = 8°/s`:
- `|ω| < 7°/s` → λ = 0 (sin asistencia, solo autocentrado)
- `|ω| = 11°/s` → λ = 0.5 (transición)
- `|ω| > 15°/s` → λ = 1 (asistencia completa)

Un giro deliberado del conductor (> 15°/s ≈ ~2 counts/ciclo @ 100 Hz)
activa asistencia completa. Vibraciones y ruido (< 7°/s) no la activan.

---

### 4.2 Par de Autocentrado Virtual — τ_center

```
τ_center = [1 − λ(ω)] × K_center × h(v) × θ
```

#### Qué hace

Crea un "muelle virtual" que empuja el volante hacia el centro (θ = 0)
cuando el conductor no está girando activamente. Simula la fuerza de
autocentrado que en un coche real proviene de la geometría de la
suspensión (caster, kingpin inclination).

#### Física

- **Proporcional a θ**: Cuanto más girado está el volante, más fuerte
  el par de retorno. Esto es idéntico a un muelle: F = −k × x. El signo
  negativo está implícito: si θ > 0 (girado a la derecha), el par
  empuja a la izquierda (hacia centro).

- **Modulado por [1 − λ(ω)]**: El autocentrado solo se activa cuando
  el conductor NO está girando. Si el conductor gira activamente
  (λ = 1), el autocentrado se desactiva para no oponerse al giro.
  Transición suave gracias a λ(ω).

  Esto resuelve el problema fundamental: sin esta modulación, el muelle
  virtual lucharía contra el conductor. Con ella, el muelle solo actúa
  cuando el conductor suelta el volante.

#### Comportamiento

| Situación | |ω| | λ(ω) | τ_center |
|-----------|-----|------|----------|
| Conductor girando rápido | 80°/s | 1.0 | 0 — sin autocentrado |
| Conductor girando lento | 12°/s | 0.5 | 50 % del autocentrado |
| Conductor soltó el volante | 0°/s | 0.0 | 100 % autocentrado |
| Volante centrado, parado | 0°/s, θ=0 | 0.0 | 0 — nada que centrar |

Ejemplo: Conductor suelta volante a θ = 25°, v = 10 km/h:
```
τ_center = (1 − 0) × 0.3 × h(10) × 25 = 0.3 × 0.67 × 25 = 5.0 %
```

El motor aplica 5 % de duty hacia el centro. Suave, sin violencia.

---

### 4.3 Par de Amortiguación — τ_damp

```
τ_damp = K_damp × ω_filt
```

#### Qué hace

Se opone al movimiento del volante proporcionalmente a la velocidad.
Es un amortiguador viscoso virtual: cuanto más rápido gira el volante,
más resistencia siente el conductor.

#### Física

La amortiguación viscosa (F = −b × v) es el mecanismo fundamental de
estabilización dinámica. Sin ella:

1. El muelle de autocentrado (τ_center) haría que el volante oscilara
   alrededor del centro como un péndulo. La amortiguación absorbe energía
   y el volante converge al centro sin rebotes.

2. Giros muy rápidos del conductor (maniobra de emergencia) producirían
   un retorno violento al soltar. La amortiguación limita la velocidad
   de retorno.

#### Signo

El signo del par de amortiguación es **igual** al de la velocidad angular,
lo que significa que el motor empuja en la dirección del movimiento...
¿Por qué?

**Porque la amortiguación física ya existe en el mecanismo** (fricción
de cremallera, juntas, columna). El término K_damp es *pequeño* y
actúa como ajuste fino. El efecto neto es:

- **Durante asistencia**: τ_damp se suma a τ_assist → ligero incremento
  de ayuda. Imperceptible por ser K_damp << K_assist.
- **Durante autocentrado**: τ_damp se opone a τ_center (porque ω tiene
  signo opuesto a θ cuando el volante retorna al centro). Esto es
  exactamente el amortiguamiento deseado: el muelle empuja hacia centro,
  la amortiguación frena el retorno para que sea progresivo.

En la práctica, durante el retorno al centro:
```
θ = +20° (girado a derecha)
ω = −30°/s (volviendo hacia centro, movimiento hacia izquierda)
τ_center = K_center × (+20) = positivo... NO: el muelle empuja hacia izquierda = NEGATIVO
τ_damp   = K_damp × (−30) = negativo (frena el retorno)
```

Corrección: el signo del autocentrado ya apunta hacia el centro. El
damping con el mismo signo que ω efectivamente FRENA el retorno.
Veamos el caso completo:

```
θ > 0 (girado derecha), queremos que τ_center → negativo (empuje a izquierda):
τ_center = −K_center × θ  (el signo negativo está en la dirección: DIR pin)

En la implementación:
τ_total se calcula como escalar, y sign(τ_total) determina DIR.
θ = +20° → τ_center = K_center × θ = +6 → DIR positivo...
```

**Aclaración de convención de signos**: En este modelo, el par se
define como valor con signo donde:
- τ > 0 → motor empuja en dirección de θ creciente (hacia la derecha)
- τ < 0 → motor empuja en dirección de θ decreciente (hacia la izquierda)

Para el autocentrado:
```
τ_center = −K_center × θ
```
Si θ = +20° → τ_center = −K_center × 20 → negativo → empuja a izquierda ✓
Si θ = −15° → τ_center = −K_center × (−15) = +K_center × 15 → empuja a derecha ✓

Para la amortiguación:
```
τ_damp = −K_damp × ω
```
Si ω = −30°/s (retornando al centro desde derecha):
τ_damp = −K_damp × (−30) = +K_damp × 30 → positivo → frena el retorno ✓

Si ω = +50°/s (conductor girando a derecha):
τ_damp = −K_damp × 50 → negativo → opone resistencia al giro ✓
```

**El amortiguador siempre se opone al movimiento.** Aporta sensación
de peso al volante y evita oscilaciones en el retorno al centro.

#### Valor de K_damp

K_damp debe ser **mucho menor** que K_assist:
```
K_damp / K_assist ≈ 0.03–0.05
```

Si K_assist = 1.5 → K_damp ≈ 0.05. A 50°/s:
```
|τ_damp| = 0.05 × 50 = 2.5 %
|τ_assist| = 1.5 × 50 = 75 %
```

La amortiguación es ~3 % del par de asistencia. Imperceptible durante
el giro, pero crítica durante el retorno al centro.

---

### 4.4 Compensación de Fricción — τ_friction

```
τ_friction = K_friction × σ(θ, ω)
```

#### Qué hace

Compensa la fricción estática del mecanismo de dirección (cremallera,
juntas, cojinetes, escobillas del motor). Sin esta compensación, el
autocentrado no podría mover el volante de vuelta al centro porque el
par del muelle virtual no superaría la fricción estática.

#### Función σ(θ, ω) — Activación de compensación

```
         ⎧ sign(−θ)     si |θ| > θ_dead AND |ω_filt| < ω_friction_thresh
σ(θ,ω) = ⎨
         ⎩ 0            en cualquier otro caso
```

Donde:
- `θ_dead` = 1.0° (zona muerta angular — no compensar si el volante
  está prácticamente centrado)
- `ω_friction_thresh` = 5.0°/s (solo compensar fricción cuando el volante
  está casi parado — si ya se mueve, la fricción dinámica es menor y
  se supera naturalmente)

**Propósito de la dirección `sign(−θ)`**: El par de compensación
siempre apunta hacia el centro, igual que el autocentrado. Es un
"empujón constante" que supera la fricción estática.

#### Física

La fricción estática es un fenómeno de umbral: el mecanismo no se mueve
hasta que la fuerza supera un valor crítico. El muelle virtual genera
un par proporcional al ángulo:
```
τ_center(θ = 5°) = −K_center × 5 = −0.3 × 5 = −1.5 %
```

Si la fricción estática requiere > 2 % para mover la cremallera, el
volante se queda atascado a 5°. La compensación de fricción añade:
```
τ_friction = K_friction × sign(−θ) = 3.0 × (−1) = −3.0 %
```

Total: −1.5 − 3.0 = −4.5 % → supera la fricción → el volante se mueve.

Una vez que el volante empieza a moverse (|ω| > ω_friction_thresh),
σ = 0 y la compensación se desactiva. La fricción dinámica (menor que
la estática) es superada por el muelle virtual solo.

#### Riesgo

Si K_friction es demasiado alto, el volante "salta" bruscamente al
empezar a moverse (la compensación aporta más que lo necesario para
superar la fricción). La calibración debe encontrar el valor mínimo
de K_friction que permita el retorno al centro desde cualquier ángulo.

---

## 5. Escalado por Velocidad del Vehículo

### 5.1 Asistencia: g(v)

```
g(v) = 1 / (1 + v / v_ref)
```

Donde `v_ref = 15 km/h` (velocidad de referencia).

| v (km/h) | g(v) | Efecto |
|-----------|------|--------|
| 0 (parado) | 1.00 | Asistencia máxima — maniobra de aparcamiento |
| 5 | 0.75 | 75 % asistencia — velocidad de paseo |
| 10 | 0.60 | 60 % asistencia |
| 15 | 0.50 | 50 % asistencia |
| 30 | 0.33 | 33 % asistencia — volante más pesado |

#### Física

A mayor velocidad, la dirección necesita menos asistencia:

1. **Fuerzas laterales del neumático**: A alta velocidad, los neumáticos
   generan fuerzas de autocorrección mayores (slip angle → fuerza
   lateral → momento de autoalineamiento). El conductor necesita menos
   ayuda del motor.

2. **Seguridad**: Un volante ligero a alta velocidad es peligroso. Una
   pequeña corrección del conductor produce un cambio de dirección
   grande. Reducir la asistencia a alta velocidad aumenta la "sensación"
   del volante y mejora la precisión.

3. **Estabilidad**: Con asistencia máxima a alta velocidad, cualquier
   ruido del encoder o vibración del motor podría generar oscilaciones
   del volante perceptibles. Reducir g(v) reduce la ganancia del sistema
   y mejora la estabilidad.

### 5.2 Autocentrado: h(v)

```
h(v) = 0.3 + 0.7 × (v / v_ref) / (1 + v / v_ref)
```

| v (km/h) | h(v) | Efecto |
|-----------|------|--------|
| 0 (parado) | 0.30 | 30 % autocentrado — el conductor maneja el retorno |
| 5 | 0.53 | Moderado |
| 10 | 0.63 | El muelle se fortalece |
| 15 | 0.65 | Autocentrado firme |
| 30 | 0.77 | Autocentrado fuerte |

#### Física

A mayor velocidad, el autocentrado debe ser más fuerte:

1. **Estabilidad direccional**: Un vehículo a alta velocidad necesita
   volver al centro rápidamente cuando el conductor suelta el volante.
   A baja velocidad (aparcamiento), un autocentrado fuerte sería molesto
   — el conductor quiere mantener el volante girado mientras maniobra.

2. **Complemento a g(v)**: A alta velocidad la asistencia baja (g
   decrece) y el autocentrado sube (h crece). El resultado: el volante
   se siente más estable y "centrado" a alta velocidad.

3. **Parado (v = 0)**: h(0) = 0.30 (no cero). Incluso parado, hay un
   autocentrado residual que devuelve el volante al centro si el
   conductor lo suelta. El factor 0.30 es suficientemente bajo para
   no molestar durante maniobras de aparcamiento.

---

## 6. Ecuación Final Completa con Signos

Reuniendo todos los términos con la convención de signos correcta:

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  τ_motor =  + λ(ω) × K_assist × g(v) × ω_filt                     │
│             − K_center × h(v) × θ × [1 − λ(ω)]                    │
│             − K_damp × ω_filt                                       │
│             + K_friction × σ(θ, ω)                                  │
│                                                                     │
│  Donde:                                                             │
│    τ > 0 → motor empuja en dirección θ creciente (derecha)          │
│    τ < 0 → motor empuja en dirección θ decreciente (izquierda)      │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Desglose de signos

| Término | Signo en la ecuación | Efecto físico |
|---------|---------------------|---------------|
| τ_assist = +K_assist × ω | **Mismo signo que ω** | Empuja en la dirección del giro → **asiste** |
| τ_center = −K_center × θ | **Opuesto a θ** | Empuja hacia centro → **retorna** |
| τ_damp = −K_damp × ω | **Opuesto a ω** | Se opone al movimiento → **amortigua** |
| τ_friction = +K_friction × sign(−θ) | **Opuesto a θ** | Empuja hacia centro → **supera fricción** |

### Verificación de escenarios

#### Escenario 1: Conductor gira a la derecha, velocidad moderada

```
θ = +10°, ω_filt = +60°/s, v = 8 km/h
λ(60) = 1.0, g(8) = 0.65, h(8) = 0.59

τ_assist  = +1.0 × 1.5 × 0.65 × 60  = +58.5 %   ← Ayuda al giro ✓
τ_center  = −0.3 × 0.59 × 10 × (1−1) = 0.0 %     ← Desactivado ✓
τ_damp    = −0.05 × 60                = −3.0 %     ← Resistencia suave ✓
τ_friction = 0 (|ω| > thresh)         = 0.0 %      ← No necesario ✓

τ_total = 58.5 − 3.0 = +55.5 %  → motor empuja a la derecha
```

El conductor siente el volante ligero: el motor aporta el 55 % del
esfuerzo. El 3 % de amortiguación da sensación de peso, imperceptible
frente al 58 % de asistencia.

#### Escenario 2: Conductor suelta el volante girado, vehículo en movimiento

```
θ = +25°, ω_filt = 0°/s, v = 12 km/h
λ(0) = 0.0, g(12) = 0.56, h(12) = 0.63

τ_assist  = 0                                = 0.0 %
τ_center  = −0.3 × 0.63 × 25 × (1−0)        = −4.7 %   ← Empuja a izquierda ✓
τ_damp    = −0.05 × 0                        = 0.0 %
τ_friction = 3.0 × sign(−25) = 3.0 × (−1)   = −3.0 %   ← Supera fricción ✓

τ_total = −4.7 − 3.0 = −7.7 %  → motor empuja a la izquierda (hacia centro)
```

El volante empieza a retornar al centro con un par de 7.7 %.
A medida que θ disminuye, τ_center disminuye proporcionalmente.
El retorno es progresivo, sin violencia.

#### Escenario 3: Volante parado en centro, vehículo parado

```
θ = 0°, ω_filt = 0°/s, v = 0 km/h
λ(0) = 0.0, g(0) = 1.0, h(0) = 0.30

τ_assist  = 0
τ_center  = −0.3 × 0.30 × 0 = 0
τ_damp    = 0
τ_friction = 0 (|θ| < θ_dead)

τ_total = 0 %  → motor apagado (EN=LOW, coast)
```

**Sin vibración. Sin corriente. Motor completamente libre.** El conductor
puede mover el volante sin resistencia del motor. Exactamente lo deseado.

#### Escenario 4: Retorno al centro, volante en movimiento

```
θ = +8°, ω_filt = −20°/s (retornando), v = 10 km/h
λ(20) = 1.0 (|ω| > threshold), g(10) = 0.60, h(10) = 0.63

τ_assist  = +1.0 × 1.5 × 0.60 × (−20) = −18.0 %   ← Asiste el retorno ✓
τ_center  = −0.3 × 0.63 × 8 × (1−1)   = 0.0 %      ← Desactivado (λ=1) ✓
τ_damp    = −0.05 × (−20)              = +1.0 %      ← Frena retorno ✓
τ_friction = 0 (|ω| > thresh)          = 0.0 %

τ_total = −18.0 + 1.0 = −17.0 %  → motor empuja a izquierda (hacia centro)
```

Cuando el volante retorna rápidamente (ω = −20°/s), el término de
asistencia detecta movimiento y AYUDA el retorno. El damping frena
ligeramente para que el retorno sea suave, no un latigazo.

#### Escenario 5: Volante girado, velocidad alta

```
θ = +15°, ω_filt = +30°/s, v = 25 km/h
λ(30) = 1.0, g(25) = 0.38, h(25) = 0.74

τ_assist  = +1.0 × 1.5 × 0.38 × 30  = +17.1 %   ← Menos ayuda ✓
τ_center  = −0.3 × 0.74 × 15 × 0    = 0.0 %
τ_damp    = −0.05 × 30               = −1.5 %     ← Resistencia ✓
τ_friction = 0                        = 0.0 %

τ_total = +17.1 − 1.5 = +15.6 %
```

A 25 km/h, la asistencia se reduce a 38 % de la nominal. El conductor
siente el volante más pesado → mayor precisión y estabilidad.

#### Escenario 6: Aparcamiento, volante girado al máximo, muy lento

```
θ = +50°, ω_filt = +10°/s (giro lento), v = 2 km/h
λ(10) ≈ 0.35 (en zona de transición), g(2) = 0.88, h(2) = 0.38

τ_assist  = 0.35 × 1.5 × 0.88 × 10  = +4.6 %
τ_center  = −0.3 × 0.38 × 50 × 0.65 = −3.7 %  ← Autocentrado parcial
τ_damp    = −0.05 × 10               = −0.5 %
τ_friction = 0 (|ω| > ω_friction)    = 0.0 %

τ_total = +4.6 − 3.7 − 0.5 = +0.4 %
```

Giro lento durante aparcamiento: el motor aporta una ayuda neta mínima.
El autocentrado parcial (λ ≈ 0.35 → 65 % activo) proporciona una
resistencia progresiva que el conductor percibe como la "rigidez
natural" de la dirección. No impide el giro, solo lo modera.

---

## 7. Zona Muerta del Motor y Transición

### 7.1 Problema

El motor DC brushed tiene una zona muerta: por debajo de ~6 % duty
(~1.4 V en bus de 24 V), las escobillas no vencen la fricción estática
y el motor no gira. Consumo sin efecto útil.

### 7.2 Solución: Salto de zona muerta

```
if |τ_motor| < deadband_cancel:
    → motor en coast (EN=LOW, PWM=0)
elif |τ_motor| < duty_min:
    → saltar a duty_min con sign(τ_motor)
else:
    → aplicar τ_motor directamente
```

Donde:
- `deadband_cancel = 1.0 %` → por debajo de esto, no vale la pena activar el motor
- `duty_min = 7.0 %` → duty mínimo efectivo (calibrable)

### 7.3 Transición suave

Para evitar un salto brusco de 0 % a 7 %:
```
duty_effective = deadband_cancel + (duty_min − deadband_cancel)
                × smoothstep(|τ_motor|, deadband_cancel, duty_min)
                + (|τ_motor| − duty_min) × step(|τ_motor| − duty_min)
```

Esto mapea |τ_motor| ∈ [1, 7] a un duty que sube suavemente de 0 a 7,
evitando el tirón de arranque.

---

## 8. Condiciones Especiales

### 8.1 Motor en coast (sin par)

```
Condición: |τ_motor| < deadband_cancel (1 %)
Acción: Motor_Enable(motor_steer, 0)  → EN=LOW, H-bridge desconectado
        Motor_SetPWM(motor_steer, 0)
```

El motor no frena, no asiste, flota libremente. El conductor siente
solo la fricción mecánica del mecanismo. Esto ocurre cuando:
- Volante centrado y parado (escenario 3)
- Todos los términos de par se cancelan
- Zona muerta del cálculo

### 8.2 Freno de estacionamiento (GEAR_PARK)

```
Condición: current_gear == GEAR_PARK
Acción: Motor_SetPWM(motor_steer, BTS7960_BRAKE_PWM)  → 100 % duty
        Motor_Enable(motor_steer, 1)
```

En PARK, el EPS se desactiva y se aplica freno electromagnético total.
El volante queda bloqueado. Idéntico al comportamiento actual.

### 8.3 Emergencia

```
Condición: Safety_EmergencyStop() o enc_fault
Acción: Motor_Enable(motor_steer, 0)  → coast
        Motor_SetPWM(motor_steer, 0)
```

En emergencia, el motor se desconecta. El conductor mantiene control
manual completo (dirección mecánica sin asistencia).

### 8.4 Sistema degradado

```
Condición: Safety_IsDegraded()
Acción: τ_motor *= Safety_GetSteeringLimitFactor()
```

El par de asistencia se reduce proporcionalmente al nivel de degradación.
Mismo mecanismo que el PID actual. El conductor siente el volante más
pesado pero mantiene control.

---

## 9. Diagrama de Flujo del Cálculo (100 Hz)

```
┌────────────────────────────┐
│ Leer encoder: θ_raw (counts)│
└─────────────┬──────────────┘
              │
              ▼
┌────────────────────────────┐
│ θ_deg = θ_raw × 360 / 4800 │
│ Δθ = θ_raw − θ_prev        │
│ ω_raw = Δθ × 360/4800 / dt │
│ θ_prev = θ_raw              │
└─────────────┬──────────────┘
              │
              ▼
┌────────────────────────────┐
│ ω_filt = α×ω_raw           │
│        + (1−α)×ω_filt_prev │
│ (α = 0.3, fc ≈ 4.8 Hz)     │
└─────────────┬──────────────┘
              │
              ▼
┌────────────────────────────┐
│ v = avg(wheel_speeds) km/h  │
│ g(v) = 1 / (1 + v/v_ref)   │
│ h(v) = 0.3 + 0.7×...       │
└─────────────┬──────────────┘
              │
              ▼
┌────────────────────────────┐
│ λ = smoothstep(|ω_filt|,   │
│     ω_th−ω_bl, ω_th)       │
└─────────────┬──────────────┘
              │
              ▼
┌────────────────────────────────────────────────────────────┐
│ τ_assist  = +λ × K_assist × g(v) × ω_filt                 │
│ τ_center  = −(1−λ) × K_center × h(v) × θ_deg              │
│ τ_damp    = −K_damp × ω_filt                               │
│ τ_friction = K_friction × σ(θ_deg, ω_filt)                 │
│                                                            │
│ τ_motor = τ_assist + τ_center + τ_damp + τ_friction        │
└─────────────┬──────────────────────────────────────────────┘
              │
              ▼
┌────────────────────────────┐
│ if Safety_IsDegraded():     │
│   τ_motor *= limit_factor   │
│ clamp(τ, −MAX, +MAX)       │
└─────────────┬──────────────┘
              │
              ▼
┌────────────────────────────┐
│ if |τ_motor| < 1.0%:       │
│   → coast (EN=LOW)         │
│ elif |τ_motor| < duty_min:  │
│   → smoothstep to duty_min │
│ else:                       │
│   → apply directly          │
└─────────────┬──────────────┘
              │
              ▼
┌────────────────────────────┐
│ Motor_SetPWM(pwm)          │
│ Motor_SetDirection(sign(τ)) │
│ Motor_Enable(en)            │
└────────────────────────────┘
```

---

## 10. Resumen de Parámetros Calibrables

| Parámetro | Símbolo | Rango sugerido | Efecto si se aumenta |
|-----------|---------|----------------|---------------------|
| Ganancia de asistencia | K_assist | 0.8–2.5 %/(°/s) | Volante más ligero al girar |
| Rigidez de autocentrado | K_center | 0.15–0.50 %/° | Retorno al centro más rápido |
| Amortiguación | K_damp | 0.02–0.10 %/(°/s) | Retorno más lento, menos oscilaciones |
| Compensación de fricción | K_friction | 1.5–5.0 % | Retorno completa al centro desde ángulos grandes |
| Velocidad de referencia | v_ref | 10–25 km/h | Rango de velocidad donde la asistencia se reduce a la mitad |
| Umbral de intención | ω_threshold | 8–25 °/s | Sensibilidad a giros lentos |
| Ancho de transición | ω_blend | 3–12 °/s | Suavidad de la transición assist↔center |
| Zona muerta angular | θ_dead | 0.5–2.0 ° | Zona alrededor del centro donde no hay autocentrado |
| Duty mínimo motor | duty_min | 5–9 % | Depende del motor: mínimo para vencer stiction |
| Umbral de cancelación | deadband_cancel | 0.5–1.5 % | Por debajo, motor en coast |
| Coeficiente EMA | α | 0.2–0.5 | Suavizado de velocidad angular |
| Par máximo | MAX_TORQUE_PCT | 50–80 % | Límite de seguridad |

Todos estos parámetros son independientes del hardware — dependen del
peso del vehículo, tipo de neumáticos, fricción del mecanismo, y
preferencia del conductor. Requieren calibración empírica.
