# AISLAMIENTO DE AUDIO — DFPlayer Mini y Amplificador Externo

**Sistema de Control Vehicular — STM32G474RE + ESP32-S3**
**Documentos relacionados:** `CABLEADO_AISLAMIENTO_DEFINITIVO.md`, `VALIDACION_ELECTRICA_AISLAMIENTO.md`, `VALIDACION_CAN_PULLUP_PC817.md`
**Estado: Recomendación técnica previa al cableado definitivo — Firmware SIN modificar**

---

## Índice

1. [Cadena de audio y entorno eléctrico](#1-cadena-de-audio-y-entorno-eléctrico)
2. [¿Es necesario aislamiento galvánico en la señal de audio?](#2-es-necesario-aislamiento-galvánico-en-la-señal-de-audio)
3. [Soluciones recomendadas](#3-soluciones-recomendadas)
4. [Punto de inserción del aislamiento](#4-punto-de-inserción-del-aislamiento)
5. [Impacto en la detección de fallos y la seguridad del sistema](#5-impacto-en-la-detección-de-fallos-y-la-seguridad-del-sistema)
6. [Interacción con la topología de masas definida](#6-interacción-con-la-topología-de-masas-definida)
7. [Tabla de acciones y componentes recomendados](#7-tabla-de-acciones-y-componentes-recomendados)

---

## 1. Cadena de audio y entorno eléctrico

### 1.1 Cadena de señal

```
ESP32-S3                      DFPlayer Mini              Amplificador         Altavoz
────────────                  ─────────────              ───────────          ────────
GPIO43 (TX) ──UART2 9600bd──► RX  (control)
GPIO44 (RX) ◄──UART2 9600bd── TX  (busy/status)

3.3V ────────────────────────► VCC
GND_ESP32 ───────────────────► GND        DAC_R ──►[R_entrada]──► IN+  ──►[AMP]──► Altavoz (+)
                                          DAC_L    (opcional)       GND_amp                     │
                                          (señal analógica SE,                     Altavoz (−) ─┘
                                           ~0.5–2 Vpp no balanceada)
```

**Características relevantes de la señal:**
- Salida del DFPlayer: analógica no balanceada (single-ended SE), referenciada a GND_DFPlayer = GND_ESP32
- Nivel de señal: ~0.5–1.0 Vpp a plena escala
- Contenido: mensajes de voz (300 Hz – 3 kHz), no música ni alta fidelidad
- Control: UART2 digital (GPIO 43/44), enteramente dentro del dominio GND_ESP32

### 1.2 Fuentes de ruido eléctrico en el entorno

| Fuente | Frecuencia | Amplitud potencial | Mecanismo de acoplamiento |
|--------|-----------|-------------------|--------------------------|
| BTS7960 × 5 (tracción + dirección) | **20 kHz** PWM + armónicos | 1–5 V sobre GND_chasis | Corrientes de conmutación en GND_chasis; acoplamiento inductivo/capacitivo |
| Relés RELAY_TRAC / RELAY_STEER_PWR (legacy: RELAY_DIR) | Transitorios al abrir/cerrar | Picos de 50–200 V sobre GND_chasis (sin snubber) | Acoplamiento por inductancia parásita del cableado |
| WS2812B (LED strips, PB10/PB11) | ~800 kHz + armónicos | Variable | Transitorios en el bus 5V de las tiras |
| Retorno de corriente de motores 24 V | DC + AC ripple | Decenas de amperios pico | Tensión de caída en GND_chasis entre puntos de conexión |

La característica crítica es el **ruido de 20 kHz** de los BTS7960: es permanente mientras los motores están activos, está directamente en el rango audible y tiene suficiente energía para aparecer como zumbido audible en el altavoz.

---

## 2. ¿Es necesario aislamiento galvánico en la señal de audio?

### 2.1 Respuesta directa

**Sí, con condiciones.** El aislamiento galvánico en la señal de audio **no es obligatorio** si se cumple una condición de cableado fundamental: que el terminal negativo del altavoz no contacte con GND_chasis en ningún punto. Si esta condición se viola, el aislamiento se vuelve imprescindible para evitar un bucle de masa de alto impacto.

La respuesta no es "siempre sí" ni "siempre no" — depende del tipo de amplificador utilizado y de la instalación física del altavoz.

### 2.2 El bucle de masa crítico: cómo se forma

El riesgo específico de este sistema es la creación de un **segundo puente GND_ESP32 ↔ GND_chasis** a través de la cadena de audio.

La topología de masas ya definida tiene exactamente **un único punto de conexión** entre GND_STM32 y GND_chasis (punto de estrella), y la conexión GND_ESP32 ↔ GND_chasis ya existe por el CAN bus de referencia. Si el altavoz añade una segunda conexión GND_ESP32 ↔ GND_chasis a través del cable del altavoz, se forma un bucle:

```
Bucle de masa con amplificador SE y altavoz sobre chasis:

GND_chasis ───────[cable de señal largo]──────► Altavoz (−) ──► Amplificador GND
                                                                           │
                                                                     GND_ESP32
                                                                           │
                               (ya conectado a GND_chasis vía punto CAN) ◄┘

Area del bucle = distancia entre el punto de conexión CAN y la posición del altavoz
```

Toda tensión de ruido entre esos dos puntos de GND_chasis (generada por corrientes de retorno de los motores) aparece directamente en la entrada del amplificador en serie con la señal de audio del DFPlayer.

```
Ruido inyectado en la entrada del amplificador:
  V_ruido = I_retorno_motor × R_GND_chasis (entre punto CAN y punto altavoz)

Estimación:
  I_motor_peak ≈ 20–50 A (transitorio BTS7960)
  R_GND_chasis (bastidor metálico) ≈ 1–10 mΩ (muy baja, pero...)
  V_ruido ≈ 20 A × 5 mΩ = 100 mV

  Ganancia del amplificador ≈ 20–40 dB
  V_altavoz_ruido ≈ 100 mV × 100 = 10 V pico → perfectamente audible como zumbido fuerte
```

Este escenario no es teórico. Es el mecanismo exacto del "hum de masa" conocido en instalaciones automotrices. En banco (con motores parados) no existe — solo aparece al mover el vehículo.

### 2.3 Cuándo NO es necesario aislamiento adicional

El bucle de masa no existe si se cumple **cualquiera** de las siguientes condiciones:
1. El amplificador es **BTL (Bridge-Tied Load)**: ningún terminal del altavoz conecta a GND
2. El terminal negativo del altavoz se mantiene conectado a GND_ESP32 (no a GND_chasis) y el cuerpo/cesta del altavoz no toca el chasis metálico
3. Se instala un **transformador de aislamiento 1:1** en la señal de audio (siempre válido, independientemente del amplificador o instalación)

---

## 3. Soluciones recomendadas

### 3.1 Solución primaria: amplificador BTL clase D (RECOMENDADO)

Un amplificador BTL (Bridge-Tied Load) conduce la bobina del altavoz de forma diferencial: ambos terminales del altavoz están conectados a salidas push-pull del amplificador, y ninguno conecta a GND. Esto elimina físicamente el bucle de masa en el altavoz.

```
Amplificador BTL (por ejemplo, PAM8403):

  DFPlayer DAC_R ──► IN+   OUT+ ──────────────────► Altavoz (+)
  GND_ESP32 ──────► IN−   OUT− ──────────────────► Altavoz (−)
                                  ↑
                         Ambas salidas son activas (push-pull)
                         Ninguna sale a GND_chasis
```

**Ventajas del BTL en este contexto:**
- Elimina el bucle de masa sin componentes adicionales
- Duplica la tensión sobre el altavoz respecto a un amplificador SE (más potencia con la misma alimentación)
- Solución estándar en aplicaciones automotrices de bajo coste
- No afecta al firmware ni al DFPlayer

**Precaución: no todos los módulos etiquetados como "PAM8403" son iguales.** El PAM8403 en modo estéreo es BTL en cada canal (el canal L conduce el altavoz L de forma diferencial, el canal R conduce el altavoz R). Si se usa en modo mono, un canal debe ser el "+" y el otro el "−" del mismo altavoz. Verificar que la salida del módulo específico no tenga ningún terminal conectado a GND en el esquema del módulo.

**Módulos con salida BTL garantizada:**
- PAM8403 (configuración estéreo estándar)
- NS8002 / NS8003
- TPA2012D2
- MAX98357A (I2S, diferente protocolo, no compatible directamente con DFPlayer DAC)

**La condición suficiente para que el BTL sea efectivo:**
- El cuerpo metálico/cesta del altavoz no debe contactar GND_chasis (montar con arandelas plásticas o en panel no conductivo)
- La alimentación del amplificador BTL debe estar referenciada a GND_ESP32, no a GND_chasis

---

### 3.2 Solución secundaria: transformador de aislamiento 1:1 (siempre válido)

Si se usa un amplificador SE (single-ended, con el terminal negativo del altavoz a GND del amplificador) o si no se puede garantizar la instalación aislada del altavoz, un transformador de audio 1:1 entre el DFPlayer y el amplificador rompe el camino galvánico completamente.

```
DFPlayer DAC_R ──[R_serie 1kΩ]──► [Primario] │ TRANSFORMADOR │ [Secundario] ──► Amplificador IN+
GND_ESP32 ──────────────────────► [Primario] │  1:1 audio   │ [Secundario] ──► Amplificador IN− (o GND_amp)
                                  ↑                              ↑
                          Lado DFPlayer                  Lado amplificador
                          (GND_ESP32)                    (puede ser GND_chasis
                                                          si el amp es SE)
```

**Especificación del transformador:**

| Parámetro | Valor requerido | Justificación |
|-----------|----------------|---------------|
| Relación de transformación | 1:1 | No se requiere ganancia de tensión |
| Impedancia nominal | 600Ω:600Ω o 1kΩ:1kΩ | Compatibilidad con salida del DFPlayer |
| Respuesta en frecuencia | 300 Hz – 8 kHz (-3 dB) | Rango de voz; el DFPlayer ya ecualiza para voz |
| Tensión de aislamiento | ≥ 500 V rms | Suficiente para aislar GND_ESP32 de GND_chasis |
| Corriente continua | 0 mA (transformadores de audio no pasan DC) | Correcto para señal de audio |
| Tamaño | Módulo tipo EI-14 o equivalente | Pequeño, ligero |

**Productos específicos:**

| Componente | Referencia | Descripción |
|------------|-----------|-------------|
| Módulo aislador de audio | ALiExpress "audio isolator module" | Incluye transformador 1:1 + condensadores de filtro |
| Transformador standalone | Triad Magnetics TY-250P | 1kΩ:1kΩ, rango 200Hz–20kHz |
| Transformador económico | EPCOS / TDK B78108-S1001 | 600Ω:600Ω, en stock en Mouser |
| Módulo completo | Topping "ground loop isolator" | Resuelve en caja compacta |

**Resistencia de serie (R_serie = 1 kΩ):** Necesaria en la entrada del primario del transformador para limitar la corriente de pico y adaptar la impedancia de salida del DFPlayer (~0 Ω) a la impedancia del transformador (600–1000 Ω). Sin esta resistencia, el DFPlayer puede saturar el primario del transformador en frecuencias bajas.

---

### 3.3 Solución inadecuada: amplificador SE con GND al chasis

Si se usa un amplificador SE con el terminal negativo del altavoz o el GND del amplificador conectado a GND_chasis, el bucle de masa está garantizado. No se recomienda bajo ninguna circunstancia en este sistema.

### 3.4 Resumen de comparación de soluciones

| Solución | Coste | Componentes adicionales | Calidad audio en vehículo | Robustez automotriz | Recomendado |
|----------|-------|------------------------|--------------------------|---------------------|-------------|
| **Amplificador BTL** | Bajo | 0 (el amplificador ya es BTL) | ✅ Alta | ✅ Alta | **✅ Primaria** |
| **Transformador 1:1** | Bajo-Medio | 1 transformador + 1 resistor | ✅ Media-Alta | ✅ Muy alta | ✅ Secundaria |
| Amplificador SE + GND_ESP32 (sin chasis) | Bajo | 0 | ⚠️ Dependiente de instalación | ⚠️ Media | Solo si altavoz bien aislado |
| Amplificador SE + GND_chasis | Bajo | 0 | ❌ Zumbido 20 kHz garantizado | ❌ Mala | ❌ No recomendado |
| Interfaz balanceada (opamp) | Medio | 2 opamps, pasivos | ✅ Muy alta | ✅ Muy alta | Overkill para voz |

---

## 4. Punto de inserción del aislamiento

### 4.1 Posición óptima: entre DFPlayer DAC y amplificador

Si se decide instalar un transformador de aislamiento, la posición correcta es **entre la salida DAC del DFPlayer y la entrada del amplificador**.

```
GND_ESP32 domain                  │ BARRERA │   Dominio libre (GND_amp puede ser lo que sea)
─────────────────────────────     │         │   ─────────────────────────────────────────

DFPlayer DAC_R ──[R_serie 1kΩ]──►│~~~~~~~~~│──► Amplificador IN+
GND_ESP32 ──────────────────────►│~~~~~~~~~│──► Amplificador IN− / GND_amp
                                 │         │
ESP32-S3 (GPIO43 UART TX) ───────────────────► DFPlayer RX  (no aislar — dentro de GND_ESP32)
ESP32-S3 (GPIO44 UART RX) ◄──────────────────── DFPlayer TX  (no aislar — dentro de GND_ESP32)
```

**Por qué aquí y no en otro punto:**

| Posición | Señal en ese punto | Por qué es correcta/incorrecta |
|---------|-------------------|-------------------------------|
| ✅ **Entre DFPlayer y AMP entrada** | Analógica débil (~0.5 Vpp), alta impedancia | Punto de mínima corriente → transformador pequeño y barato. Rompe el bucle antes de la amplificación. |
| ❌ Entre amplificador y altavoz | Analógica de potencia (1–10 Vpp, 0.5–2 W) | El transformador debe soportar potencia; es más grande, más caro, introduce más distorsión armónica |
| ❌ En el UART ESP32-DFPlayer | Digital UART 9600 baud | No existe bucle de masa aquí. El UART está 100% dentro de GND_ESP32. Aislar sería innecesario y complicaría la comunicación |
| ❌ En la alimentación 3.3V/5V del DFPlayer | Alimentación DC | Un transformador no aísla DC. Requeriría un DC-DC aislado, que es excesivo para el DFPlayer |

---

## 5. Impacto en la detección de fallos y la seguridad del sistema

### 5.1 Relación funcional entre audio y seguridad

El sistema de audio está diseñado como **capa de interfaz de usuario (HMI informacional)**, no como capa de control o seguridad. La autoridad de seguridad es exclusivamente el STM32.

```
Cadena de mando de seguridad (NO incluye audio):

  Sensores físicos (ruedas, encoder, corriente, temperatura, pedal)
       │
       ▼
  STM32G474RE (safety_system.c, motor_control.c)
       │ ← Todos los actuadores: BTS7960 × 5, relés, PWM
       │ ← Todos los paros de emergencia, LIMP_HOME, SAFE, WDG
       │
       ▼
  CAN bus (FDCAN1 ← aislador digital → TJA1051_A ── bus ── TJA1051_B → TWAI ESP32)
       │
       ▼
  ESP32-S3 (HMI)
       │
       ▼
  DFPlayer Mini (audio) ──► Amplificador ──► Altavoz
                  ↑
         Solo recibe comandos del ESP32.
         No tiene ninguna vía de retorno hacia el STM32.
```

**El audio no puede:**
- Enviar comandos al STM32 (la comunicación es unidireccional: STM32 → ESP32 → DFPlayer)
- Inhibir o modificar el comportamiento del STM32
- Afectar a los sensores, actuadores, relés ni al watchdog
- Generar o suprimir estados de seguridad (SAFE, DEGRADED, LIMP_HOME)

**Consecuencias de un fallo total del audio:**
- El vehículo sigue funcionando normalmente en todos sus modos
- El STM32 no detecta ni registra el fallo de audio (no existe ningún CAN frame de confirmación de audio)
- El usuario pierde los mensajes de voz de alerta, pero la pantalla TFT del ESP32 sigue mostrando el estado del sistema → la información visual no se pierde

### 5.2 Único riesgo indirecto identificado

Un bucle de masa de alta amplitud (zumbido de 20 kHz a varios vatios) podría, teóricamente, inyectar EMI de RF en el plano de GND_ESP32 y provocar reinicios del ESP32 o errores en el UART2 del DFPlayer. En tal caso:

| Evento | Respuesta del STM32 | Impacto en seguridad |
|--------|-------------------|---------------------|
| ESP32 se reinicia | CAN timeout en 250 ms → STM32 entra en `LIMP_HOME` | ⚠️ Degradación operacional, NO paro de seguridad. Vehículo sigue mobile. |
| UART2 corrupto | DFPlayer reproduce archivo incorrecto o silencio | Ninguno — solo afecta al audio |
| DFPlayer se resetea | Silencio de audio | Ninguno — vehículo opera normalmente |

La estrategia de LIMP_HOME (vehículo mobile a ≤5 km/h, 20% torque, con pedal local) ya cubre el peor caso de pérdida de ESP32. El audio nunca puede causar un estado SAFE ni cortar los motores.

### 5.3 Conclusión de seguridad

Un fallo en el sistema de audio, incluyendo un bucle de masa severo, **no puede degradar la seguridad del sistema por debajo del nivel LIMP_HOME**. La arquitectura de seguridad del STM32 es inmune a cualquier fallo del subsistema de audio.

---

## 6. Interacción con la topología de masas definida

### 6.1 Dominio de masa del sistema de audio

Según la documentación de masas ya establecida, el DFPlayer Mini pertenece al dominio **GND_ESP32** (confirmado en `VALIDACION_ELECTRICA_AISLAMIENTO.md`).

La topología actual incluye:
- GND_ESP32 conectado a GND_chasis en **un punto único** (referencia CAN de modo común)
- GND_STM32 conectado a GND_chasis en **un punto único** (punto de estrella de la placa de control)
- Ambos puntos son distintos pero ambos están en GND_chasis

Esto significa que ya existe una conexión GND_ESP32 ↔ GND_chasis. El audio no introduce una nueva unión de dominios. **El problema no es la existencia de esa unión, sino la creación de un bucle de corriente de gran área.**

### 6.2 Cómo el altavoz puede crear un bucle inadvertido

```
Topología SIN problema (BTL o transformador):

GND_ESP32 ──[punto CAN]──► GND_chasis
                                │
                          bastidor chasis
                                │
                    (sin conexión desde altavoz)
                                │
                          Altavoz (−) ──► Amplificador OUT− ──► (solo GND_ESP32 o BTL: ninguna)

Topología CON bucle (amplificador SE, altavoz sobre chasis):

GND_ESP32 ──[punto CAN]──► GND_chasis ──────────────────────────────────────────────────┐
                                                                                         │
                                         [Corriente ruido 20kHz fluyendo por este bucle]│
                                                                                         │
GND_ESP32 ◄─[GND_amp]◄──── Amplificador OUT− ◄── Altavoz (−) ◄── bastidor chasis ───────┘

Area del bucle = [desde punto CAN hasta posición del altavoz] × altura del altavoz
```

### 6.3 Reglas de instalación para mantener la topología de masas

Estas reglas son independientes de si se usa transformador o BTL:

| Regla | Descripción | Si se viola |
|-------|-------------|------------|
| **A1** | El GND del amplificador de audio se conecta a GND_ESP32 exclusivamente | Se crea un segundo punto GND_ESP32↔GND_chasis, formando un bucle |
| **A2** | El terminal negativo del altavoz no contacta con el chasis metálico | Se crea un puente GND_amp↔GND_chasis vía altavoz, bucle de masa |
| **A3** | La cesta/cuerpo del altavoz se monta con aislante (plástico, nylon) respecto al panel metálico | La cesta metálica puede contactar con GND_chasis accidentalmente |
| **A4** | El cable del altavoz no discurre pegado a los cables de potencia de los motores | Acoplamiento inductivo del 20 kHz PWM en el cable del altavoz |
| **A5** | La alimentación del amplificador viene de la misma fuente que el ESP32 (5V referenciada a GND_ESP32) | Si viene de GND_chasis, el ruido de los motores entra por el rail de alimentación |

### 6.4 Diagrama final con audio integrado en la topología de masas

```
╔══════════════════════════════════════════════════════════════════════════════╗
║              TOPOLOGÍA DE MASAS COMPLETA CON AUDIO                           ║
╠══════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║  GND_STM32 ──[estrella]──┬──► GND_chasis                                    ║
║                          └──── (único punto)                                ║
║                                    │                                        ║
║  GND_encoder ──[6N137]──────────── │ ──(barrera)── GND_STM32               ║
║  GND_sensores ──[PC817]──────────── │ ──(barrera)── GND_STM32               ║
║                                    │                                        ║
║  GND_ESP32 ──[punto CAN]──────────► GND_chasis                              ║
║      │                                                                      ║
║      ├── ESP32-S3                                                           ║
║      ├── TJA1051T/3_B                                                       ║
║      ├── Display TFT                                                        ║
║      ├── WS2812B LEDs (5V desde relé, GND referenciado a GND_ESP32)        ║
║      ├── DFPlayer Mini (VCC 3.3V, GND → GND_ESP32)                         ║
║      │       │                                                              ║
║      │       └── DAC_R ──[R_serie 1kΩ]──► [TRAFO 1:1] ──► Amplificador IN ║
║      │            ó                                                         ║
║      │            └── DAC_R ──────────────────────────► Amplificador IN    ║
║      │                                                        │ (BTL)      ║
║      │                                                   OUT+ ──► Altavoz (+) ║
║      │                                                   OUT− ──► Altavoz (−) ║
║      │                                    (sin conexión a GND_chasis) ↑    ║
║      │                                                                      ║
║      └── Amplificador GND ──► GND_ESP32 (NO a GND_chasis)                  ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

---

## 7. Tabla de acciones y componentes recomendados

### 7.1 Decisión de implementación

| Escenario | Acción recomendada | Justificación |
|-----------|-------------------|---------------|
| Se usa amplificador BTL (PAM8403 u otro BTL) | **Usar BTL directamente, sin transformador** | Ningún terminal del altavoz toca GND. El bucle de masa es imposible. |
| Se usa amplificador SE de baja potencia (< 1W) | **Añadir transformador 1:1 entre DFPlayer y amplificador** | Rompe el bucle galvánico en el punto de menor señal. |
| Amplificador desconocido | **Verificar si es BTL; si no, añadir transformador** | Siempre más seguro con transformador. |
| Amplificador integrado en panel con carcasa metálica a chasis | **Transformador obligatorio** | La carcasa metálica conecta GND_amp a GND_chasis inevitablemente. |

### 7.2 Lista de acciones de hardware

| # | Acción | Componente | Obligatorio | Coste estimado |
|---|--------|-----------|-------------|---------------|
| **H1** | Verificar que el amplificador elegido es BTL | — | ✅ Antes de montar | 0 € |
| **H2** | Montar el altavoz con aislante (arandelas nylon) respecto al panel metálico | Arandelas M3 nylon × 4 | ✅ Siempre | < 1 € |
| **H3** | Conectar GND del amplificador a GND_ESP32 (no a chasis) | Cable corto, sección suficiente | ✅ Siempre | 0 € |
| **H4** | Alimentar el amplificador desde 5V_ESP32 (misma fuente que ESP32) | — | ✅ Siempre | 0 € |
| **H5** | Si amplificador NO es BTL: añadir transformador 1:1 | Transformador 600Ω:600Ω + R 1kΩ | Solo si SE | 2–8 € |
| **H6** | Separar el cable del altavoz de los cables de potencia de motores | Cable separado ≥ 5 cm de cables BTS7960 | ✅ Siempre | 0 € |
| **H7** | Añadir ferrita en el cable de señal DFPlayer → amplificador (opcional) | Ferrita snap-on Ø5 | Opcional | < 1 € |

### 7.3 El UART2 del DFPlayer NO necesita aislamiento

| Señal | Tipo | Acción |
|-------|------|--------|
| GPIO43 (ESP32 TX → DFPlayer RX) | UART digital 9600 baud | **No aislar** — dentro de GND_ESP32 |
| GPIO44 (ESP32 RX ← DFPlayer TX) | UART digital 9600 baud | **No aislar** — dentro de GND_ESP32 |
| VCC DFPlayer 3.3V | Alimentación DC | De regulador ESP32, referenciada a GND_ESP32 |
| GND DFPlayer | Masa | GND_ESP32 exclusivamente |

### 7.4 Verificaciones antes del primer arranque con motores

| # | Verificación | Método | Resultado esperado |
|---|-------------|--------|-------------------|
| **V1** | Sin zumbido/ruido con motores parados | Escuchar con volumen al 50% | Silencio o ruido < -40 dB |
| **V2** | Sin zumbido con motores al 50% de potencia | Escuchar durante marcha estable | El audio de voz es inteligible |
| **V3** | GND amplificador ≠ GND_chasis | Óhmetro entre GND_amp y bastidor del vehículo | Resistencia ≥ 1 MΩ |
| **V4** | Terminal (−) altavoz ≠ GND_chasis | Óhmetro entre altavoz (−) y bastidor | Resistencia ≥ 1 MΩ (solo si BTL) |
| **V5** | Mensaje de bienvenida audible | Encender sistema | "Bienvenido Marcos…" claro, sin distorsión |
| **V6** | Mensajes de alerta durante conducción | Simular fallo (temperatura, batería) | Audio claro y sin interferencias al 50% potencia |

---

> **Estado del firmware:** NINGÚN archivo de código fuente ha sido modificado.
> Este documento define la estrategia de hardware para el sistema de audio.
> La implementación es exclusivamente en cableado físico y selección de componentes.
>
> El firmware del ESP32 (audio_manager.h, DFPlayer UART2) y del STM32 son completamente
> compatibles con cualquiera de las soluciones propuestas sin ninguna modificación.

---

**Fecha:** 2026-02-25
**Basado en auditoría de:**
- `esp32/src/audio_manager.h` — PIN_DFPLAYER_TX=43, PIN_DFPLAYER_RX=44, UART2 9600 baud
- `docs/VALIDACION_ELECTRICA_AISLAMIENTO.md` — mapa de dominios de masa, GND_ESP32 incluye DFPlayer
- `docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md` — inventario de aisladores, arquitectura de masas
- `docs/VALIDACION_CAN_PULLUP_PC817.md` — topología GND_ESP32=GND_CAN, conexión a GND_chasis
- `docs/HARDWARE_WIRING_MANUAL.md` — tensiones del sistema: 24V tracción, 12V dirección, 5V lógica
- `Core/Src/safety_system.c` — autoridad de seguridad STM32, estados LIMP_HOME/SAFE/ERROR
- `Core/Src/can_handler.c` — CAN timeout 250ms → LIMP_HOME (no SAFE)
- Datasheet BTS7960 (Infineon): frecuencia PWM recomendada 20-25 kHz
- Datasheet DFPlayer Mini: salida DAC ~0.5-2Vpp single-ended, GND común
