# AUDIO_SYSTEM_WIRING — Sistema de Audio Definitivo

**Proyecto:** MarcosDashboard v10  
**Versión:** 1.0  
**Fecha:** 2026-06-12  
**Estado:** Arquitectura final — pendiente de prueba de line-out en hardware  

---

## LEYENDA

> ✅ **VERIFICADO** — confirmado en código fuente o prueba real  
> 🔵 **HIPÓTESIS PROBABLE** — razonamiento técnico, sin medición directa  
> ⚠️ **REQUIERE MEDICIÓN** — no confirmar hasta verificar con instrumento  

---

## 1. OBJETIVO

Todo el audio del vehículo pasa por el **PAM8403** como único amplificador final.  
El reproductor original (Bluetooth + USB + MicroSD + AUX) proporciona su señal
de **línea** (pre-amplificador) al PAM8403.  
El DFPlayer Mini proporciona su señal **DAC_L / DAC_R** (pre-amplificador) al PAM8403.  
Un relé físico conmuta entre las dos fuentes.  
**Ningún amplificador interno de ningún módulo conduce el altavoz directamente.**

---

## 2. VERIFICACIÓN DEL FIRMWARE ACTUAL

### 2.1 relay_audio — verificado en código

| Parámetro | Valor | Fuente |
|-----------|-------|--------|
| GPIO de control | **GPIO 11** (ESP32-S3) | `relay_audio.h:47` |
| Driver intermedio | **ULN2803A** canal 3 (3B→3C) | `relay_audio.h:9-11`, `AUDIO_WIRING_GUIDE.md` |
| Relé físico | **Songle SRD-05VDC-SL-C**, canal CH3 | `POWER_DISTRIBUTION.md:152-160` |
| Estado reposo (GPIO LOW) | **Relé OFF → radio original conectada al altavoz** | `relay_audio.h:6` |
| Estado activo (GPIO HIGH) | **Relé ON → DFPlayer conectado al altavoz** | `relay_audio.h:7` |
| Establecimiento ON | 20 ms (RELAY_ESTABLISH_MS) | `relay_audio.h:54` |
| Cooldown OFF | 150 ms (RELAY_RELEASE_MS) | `relay_audio.h:59` |
| Watchdog máximo ON | 7 000 ms (RELAY_MAX_ON_MS) | `relay_audio.h:68` |
| Anti-click | Sí — audio solo fluye cuando `isReady()` = true | `audio_manager.cpp:130` |

### 2.2 DFPlayer Mini — verificado en código

| Parámetro | Valor | Fuente |
|-----------|-------|--------|
| UART | **UART2**, 9 600 baud, 8N1 | `audio_manager.cpp:85` |
| GPIO TX (ESP32→DFPlayer) | **GPIO 43** | `audio_manager.h:66` |
| GPIO RX (DFPlayer→ESP32) | **GPIO 44** | `audio_manager.h:67` |
| Salida de audio a usar | **DAC_L / DAC_R** (NO SPK1/SPK2) | ver §5 |
| Tracks | 68 (0001.mp3–0068.mp3) | `audio_manager.h` |
| Volumen por defecto | 20 / 30 | `audio_manager.cpp:98` |
| Persistencia de volumen | NVS ESP32 (`config_store`, clave "vol") | `config_store.cpp` |

### 2.3 Lógica GPIO → ULN → Songle

```
ESP32 GPIO11 ──► ULN2803A 3B
                  3C (ULN, open-collector) ──► Songle IN3
GND común ────► ULN GND, Songle GND

GPIO11 = LOW  → ULN 3C en alta impedancia → IN3 ≈ 5V (pull-up interno) → Relé OFF
GPIO11 = HIGH → ULN 3C hunde a GND       → IN3 ≈ 0V                    → Relé ON
```

### 2.4 ¿Hay que modificar firmware?

✅ **No. El firmware ya implementa exactamente la arquitectura correcta.**  
Todos los cambios son únicamente de hardware (cableado y circuito pasivo).

---

## 3. ANÁLISIS DEL REPRODUCTOR ORIGINAL (P5470R251)

### 3.1 Prueba real a 5 V — verificada por el usuario

| Condición | Resultado |
|-----------|-----------|
| Alimentación a 5.10 V, sin altavoz | ✅ Arranca, pantalla funciona, estable indefinidamente |
| Alimentación a 5.10 V, con altavoz conectado | ❌ Se reinicia |

**Diagnóstico:**  
El reinicio ocurre únicamente cuando el altavoz está conectado.  
La lógica digital (BT + USB + MicroSD + pantalla) funciona correctamente a 5 V.  
El problema es la **etapa amplificadora interna** de la placa, que al intentar
entregar corriente al altavoz (picos de hasta 500 mA–1 A) provoca una caída
de tensión en el rail de 5 V, lo que activa el supervisor de tensión del SoC
y fuerza el reinicio.

**Solución:** No conectar el altavoz a la salida de la placa original.
Tomar la señal de **audio de línea** (antes del amplificador interno) y
conducirla al PAM8403 externo.

### 3.2 Condensador C12 (100 µF / 10 V) — verificado en foto

✅ El condensador más grande de la cara frontal de la placa está marcado
**100 µF / 10 V** (VTE). Según estándares IEC 60384, un electrolítico de 10 V
en un nodo real de 12 V incumpliría el margen mínimo 1,5×. Esto confirma
que la tensión de trabajo real del nodo donde está C12 es **≤ 6,5 V**.

🔵 Hipótesis: C12 está en el nodo de salida del convertidor DC/DC interno
(o en el nodo de alimentación de entrada, que es 5 V, no 12 V).

### 3.3 Convertidor DC/DC interno — verificado en foto

✅ Presentes en la cara frontal: inductor toroidal L2, transistor Q1 (SOT-23),
CI controlador de 8 pines SMD, red de feedback R16/R17/C8.  
🔵 Topología probable: boost de 5 V → ~9–12 V para el amplificador de audio
(NS8002 o similar, clase AB o D de 2–3 W).

### 3.4 Cómo localizar la señal de línea del reproductor original

**⚠️ REQUIERE MEDICIÓN física. Los pasos siguientes son procedimientos a seguir.**

#### Método A — Con multímetro (AC mV)

1. Alimentar la placa a 5 V sin altavoz. Reproducir música por BT o USB.
2. Poner el multímetro en modo **AC mV** (rango 200 mV o 2 V AC).
3. Conectar GND del multímetro a GND de la placa.
4. Explorar con la punta los pines del conector blanco K2 (cara posterior, 4 pines)
   y los conectores JST blancos de 3 y 4 pines.
5. El nodo que muestre **50–500 mV AC** con música reproduciéndose
   y **< 10 mV AC** en silencio es la señal de audio de línea.

#### Método B — Con osciloscopio

1. Mismas condiciones. Escala: 200 mV/div, tiempo: 1 ms/div.
2. Buscar forma de onda sinusoidal de audio (~1 kHz durante música).
3. El nodo con señal AC de **0,2–2 Vpp** sin offset DC mayor de ±1,65 V
   es DAC/pre-amplificador (línea).
4. El nodo con señal AC de **> 4 Vpp** es la salida del amplificador (NO usar).

#### Método C — Con auricular y condensador de 10 µF

1. Conectar en serie: nodo sospechoso → condensador 10 µF (polo + hacia el nodo) → auricular → GND.
2. Aplicar el condensador como bloqueo DC para proteger el oído.
3. Si se escucha música: es señal de audio de línea.
4. **NUNCA conectar auricular directamente sin condensador de bloqueo.**

#### Nodos más probables donde encontrar line-out

| Ref / ubicación | Probabilidad | Por qué |
|-----------------|-------------|---------|
| Pines del conector blanco K2 (cara posterior) | Alta | Es el conector de altavoz/salida |
| Unión entre U4 (DAC) y U_main (SoC) — cara posterior | Alta | Los SoC de la familia Jieli sacan DAC en pines dedicados |
| Pines del conector JST blanco de 3 pines — cara frontal | Media | Puede ser línea AUX out |
| Nodo entre el DAC y la resistencia de serie antes del amplificador | Alta | La red RC de acoplamiento suele ser accesible |

---

## 4. COMPARATIVA DE ARQUITECTURAS

| | **A: Relé antes del PAM8403** ← ELEGIDA | **B: Mezcla pasiva** | **C: Mezcla activa** | **D: Relé después del PAM8403** |
|---|---|---|---|---|
| Señal conmutada | Línea (baja potencia) | Línea (sumada) | Línea (buffer op-amp) | Altavoz (alta potencia) |
| Ruido | Bajo | Medio | Muy bajo | Medio |
| Saturación | No (señales independientes) | Riesgo alto si ambas fuentes activas | No | No |
| Bucle de masa | Bajo (un solo amp) | Medio | Muy bajo | Bajo |
| Compatibilidad firmware | ✅ 100% (ya implementado) | ❌ Sin conmutación | ❌ Sin conmutación | Posible (mayor complejidad) |
| Complejidad | Baja | Muy baja | Alta (CI op-amp) | Media (relay de potencia) |
| Riesgo relé | Mínimo (señal, no potencia) | — | — | Alto (corriente de altavoz) |
| Conclusión | ✅ **RECOMENDADA** | ❌ | ❌ | ❌ |

**Justificación de la elección A:**  
La arquitectura de relé antes del PAM8403 ya está implementada en el firmware
(`relay_audio.h/cpp`), usa componentes de bajo coste ya presentes en el vehículo,
y garantiza que solo una fuente de audio esté activa en cualquier momento,
eliminando el riesgo de saturación por mezcla y el riesgo de bucles de masa.

---

## 5. ESQUEMA ELÉCTRICO DEFINITIVO (ASCII)

```
╔════════════════════════════════════════════════════════════════════════╗
║  ALIMENTACIÓN 5V (LM2596 — rail existente del vehículo)               ║
║                                                                        ║
║  5V_RAW ──[FERRITA 100Ω@100MHz / 600mA]── 5V_AUDIO ──┬──[C_bulk 470µF/16V]── GND_AUDIO
║                                                        ├──[C_dec 100nF X7R]─── GND_AUDIO
║                                                        │
║     ┌──────────────────────────────────────────────────┤
║     │             ┌────────────────────────────────────┤
║     │             │                 ┌──────────────────┤
╚═════╪═════════════╪═════════════════╪══════════════════╝
      │             │                 │
      ▼             ▼                 ▼
 ┌─────────┐  ┌──────────┐     ┌───────────┐
 │REPR.ORIG│  │DFPlayer  │     │ PAM8403   │
 │P5470R251│  │  Mini    │     │ Clase D   │
 │         │  │          │     │           │
 │ VCC  ←──┼──┼──────────┼─────┤ VCC (5V) │
 │         │  │VCC ←─────┼─────┤           │
 │ GND  ←──┼──┼──────────┼─────┤ GND       │
 │         │  │GND ←─────┼─────┤           │
 │         │  │          │     │           │
 │LINE_OUT_L├──► [C_AC   │     │           │
 │         │   10µF/16V]─►[R_at 10kΩ]──►LINP─►[C_bootstrap 1µF]──►LOUT+──►SPK_L+
 │LINE_OUT_R├──►(véase    │     │   ┌──[R_gnd 10kΩ]──►GND_AUDIO   LOUT-──►SPK_L-
 │         │   §6 para   │     │   │                              │
 │         │   conexión  │     │   │  RINP─►[C_bootstrap 1µF]──►ROUT+──►SPK_R+
 │         │   al relé)  │     │   │                              ROUT-──►SPK_R-
 │         │  │          │     │   │                              │
 │         │  │ RX ←─────┼─────┼───┼──────────────── ESP32 GPIO43 (TX) ─[R 1kΩ]
 │         │  │ TX ──────┼─────┼───┼──────────────── ESP32 GPIO44 (RX)
 │         │  │          │     │   │
 │         │  │DAC_L ────┼─►[C_AC 10µF/16V]─►[R_at 10kΩ]─┐
 │         │  │DAC_R ────┼─►[C_AC 10µF/16V]─►[R_at 10kΩ]─┤
 │         │  │          │     │   │                        │
 └─────────┘  └──────────┘     │   │                        │
                                │   │                        │
     ┌────────────────────────────────────────────────────────────────────┐
     │               RELÉ SONGLE SRD-05VDC-SL-C (CH3)                   │
     │                                                                    │
     │  Bobina (+)──────────────────────── 5V_AUDIO                     │
     │  Bobina (-)──[D_flyback 1N4148]─┐── ULN2803A 3C                  │
     │                                  └──► 5V_AUDIO (cátodo diodo)     │
     │                                                                    │
     │  COM_L ────── LINE_OUT_L del reproductor original [via C_AC+R_at] │
     │  COM_R ────── LINE_OUT_R del reproductor original [via C_AC+R_at] │
     │                                                                    │
     │  NO_L  ────── DAC_L del DFPlayer [via C_AC+R_at]                 │
     │  NO_R  ────── DAC_R del DFPlayer [via C_AC+R_at]                 │
     │                                                                    │
     │  NC_L  ────── NO CONECTAR (fuente inactiva en reposo)             │
     │  NC_R  ────── NO CONECTAR                                         │
     │                                                                    │
     │  OUT_L ────── PAM8403 LINP                                        │
     │  OUT_R ────── PAM8403 RINP                                        │
     └────────────────────────────────────────────────────────────────────┘

              ESP32 GPIO11 ──► ULN2803A 3B
                                3C (open-collector sink) ──► Bobina relé (-)
              ULN2803A VCC ──► 5V_AUDIO
              ULN2803A GND ──► GND_AUDIO

              Lógica de conmutación:
              GPIO11=LOW  → Relé OFF → COM conectado a NC (radio, reposo)
              GPIO11=HIGH → Relé ON  → COM conectado a NO (DFPlayer, voz activa)
```

**Nota sobre el relé:** El Songle SRD-05VDC-SL-C es SPDT (Single Pole Double Throw).
Para canales L y R se usan **dos relés independientes** del mismo módulo de 4 canales
Songle 5V que ya existe en el vehículo (CH3 y — si disponible — CH4), o bien
un solo relé DPDT si se encuentran disponibles.  
Si solo hay un canal libre en el módulo, se puede usar un único canal SPDT
para el canal mono mezclado (L+R mezclados con resistencias antes del relé).

---

## 6. TABLA DE CONEXIONES PIN A PIN

### 6.1 Reproductor original → Circuito de audio

| Origen | Componente intermedio | Destino |
|--------|-----------------------|---------|
| VCC reproductor | — | 5V_AUDIO |
| GND reproductor | — | GND_AUDIO |
| LINE_OUT_L (⚠️ a verificar) | C_AC1 (10µF/16V) + R_at1 (10kΩ) | Relé COM_L |
| LINE_OUT_R (⚠️ a verificar) | C_AC2 (10µF/16V) + R_at2 (10kΩ) | Relé COM_R |

### 6.2 DFPlayer Mini → Circuito de audio

| Pin DFPlayer | Descripción | Componente intermedio | Destino |
|--------------|-------------|----------------------|---------|
| Pin 1 (VCC) | Alimentación | Ferrita 33Ω (opcional) | 5V_AUDIO |
| Pin 10 (GND) | Masa | — | GND_AUDIO |
| Pin 2 (RX) | UART recepción | R_ser 1kΩ | ESP32 GPIO43 (TX) |
| Pin 3 (TX) | UART transmisión | — | ESP32 GPIO44 (RX) |
| Pin 7 (DAC_R) | Línea derecha | C_AC3 (10µF/16V) + R_at3 (10kΩ) | Relé NO_R |
| Pin 8 (DAC_L) | Línea izquierda | C_AC4 (10µF/16V) + R_at4 (10kΩ) | Relé NO_L |
| Pin 11 (SPK1) | ❌ NO CONECTAR | — | — |
| Pin 13 (SPK2) | ❌ NO CONECTAR | — | — |

### 6.3 Relé de audio → PAM8403

| Terminal relé | Señal | Destino PAM8403 |
|---------------|-------|-----------------|
| COM_L | Señal L activa | LINP (pin 2) |
| COM_R | Señal R activa | RINP (pin 7) |
| NO_L | DFPlayer DAC_L (activo durante voz) | conectado a COM_L |
| NO_R | DFPlayer DAC_R | conectado a COM_R |
| NC_L | Radio LINE_OUT_L (activo en reposo) | conectado a COM_L |
| NC_R | Radio LINE_OUT_R | conectado a COM_R |

### 6.4 Control del relé

| Origen | Componente | Destino |
|--------|-----------|---------|
| ESP32 GPIO11 | — | ULN2803A pin 3B |
| ULN2803A pin 3C | D_flyback 1N4148 (entre 3C y 5V_AUDIO) | Bobina relé (−) |
| 5V_AUDIO | — | Bobina relé (+) |

### 6.5 PAM8403 → Altavoz

| Pin PAM8403 | Conexión |
|-------------|---------|
| VDD (pin 1) | 5V_AUDIO |
| GND (pin 5) | GND_AUDIO |
| LINP (pin 2) | Señal L (del relé COM_L) + R_gnd_L (10kΩ a GND_AUDIO) |
| RINP (pin 7) | Señal R (del relé COM_R) + R_gnd_R (10kΩ a GND_AUDIO) |
| /SD (pin 3) | 5V_AUDIO (siempre activo) o GPIO ESP32 si se quiere control SW |
| LOUT+ (pin 8) | Altavoz izquierdo (+) |
| LOUT− (pin 6) | Altavoz izquierdo (−) |
| ROUT+ (pin 9) | Altavoz derecho (+) |
| ROUT− (pin 4) | Altavoz derecho (−) |

> **Nota:** El PAM8403 es amplificador de clase D BTL (Bridge Tied Load).
> Los pines LOUT+/LOUT− llevan señales diferenciales: **nunca conectar LOUT− a GND**.
> Siempre conectar directamente al altavoz (sin condensador de salida).

---

## 7. LISTA DE COMPONENTES (BOM AUDIO)

| Ref | Componente | Valor / Tipo | Cant | Función |
|-----|-----------|-------------|------|---------|
| C_bulk | Condensador electrolítico | **470 µF / 16 V** | 1 | Bulk VCC audio; ver justificación §8.2 |
| C_dec_PAM | Condensador cerámico X7R | **100 nF / 16 V** | 2 | Desacoplo VDD PAM8403 (uno por nodo de alimentación) |
| C_dec_DF | Condensador cerámico X7R | **100 nF / 16 V** | 1 | Desacoplo VCC DFPlayer |
| C_dec_radio | Condensador cerámico X7R | **100 nF / 16 V** | 1 | Desacoplo VCC reproductor original |
| C_AC1–4 | Condensador electrolítico o cerámico X5R | **10 µF / 16 V** | 4 | Bloqueo DC en las 4 líneas de audio (L/R × 2 fuentes) |
| C_bootstrap | Condensador cerámico X7R | **1 µF / 16 V** | 2 | Bootstrap interno PAM8403 (BSTL/BSTR si aplica a la versión) |
| R_at1–4 | Resistencia | **10 kΩ** (±1%, 0402/0603) | 4 | Atenuación de señal de línea a nivel de entrada PAM8403 |
| R_gnd_L, R_gnd_R | Resistencia | **10 kΩ** (±1%, 0402/0603) | 2 | Referencia DC en entradas PAM8403 (impide flotación) |
| R_ser | Resistencia | **1 kΩ** (0402/0603) | 1 | Protección GPIO43 → DFPlayer RX |
| FB1 | Ferrita EMI | **100 Ω @ 100 MHz, 600 mA**, SMD 0805 o THT | 1 | Filtro EMI en 5V → 5V_AUDIO |
| D_flyback | Diodo | **1N4148** (DO-35) | 1 | Protección flyback bobina relé Songle |
| D_prot | Diodo Schottky | **SS14** (SOD-123, 1A/40V) | 1 | Protección polaridad inversa VCC audio |
| RELAY | Songle SRD-05VDC-SL-C | ya presente en el vehículo | (CH3) | Conmutación audio — ya cableado en firmware |
| DFPlayer | DFPlayer Mini | — | ya presente | Mensajes de voz |
| PAM8403 | Módulo PAM8403 | Clase D, 3W×2, 5V | 1 | Amplificador final |

### 7.1 Justificación del condensador bulk: 470 µF vs 100 µF vs 1 000 µF

| Valor | Razón de descarte / elección |
|-------|------------------------------|
| 100 µF | Insuficiente para el PAM8403 clase D: hold-up de ~1 ms. Los transitorios de carga de los cuatro motores BTS7960 pueden causar micro-caídas de tensión que generan clics en el audio. |
| **470 µF** ✅ | Hold-up ~4–5 ms. Cubre los transitorios de carga habituales de BTS7960 sin exceso de tamaño. Buen equilibrio entre ESR, volumen físico y coste. |
| 1 000 µF | Solo si la fuente LM2596 tiene regulación lenta o si los transitorios de carga superan los 100 ms. No necesario con el LM2596 (tipico 10 µs de respuesta). |

---

## 8. ALIMENTACIÓN DEFINITIVA

### 8.1 Rail compartido 5V_AUDIO

Todos los módulos de audio comparten el mismo rail de 5 V extraído del
LM2596 ya existente en el vehículo (`project_config.h:474`), aislado del
resto del sistema mediante la ferrita FB1.

| Módulo | VCC | Corriente típica | Corriente máx |
|--------|-----|-----------------|---------------|
| Reproductor original | 5V_AUDIO | ~100 mA (sin altavoz) | ~150 mA |
| DFPlayer Mini | 5V_AUDIO | ~20 mA idle | ~40 mA reproduciendo |
| PAM8403 (4Ω, 3W) | 5V_AUDIO | ~200 mA @ 50% vol | ~600 mA pico por canal |
| Relé Songle 5V | 5V_AUDIO | ~70 mA bobina ON | ~70 mA |
| **Total estimado** | | **~390 mA normal** | **~860 mA pico** |

**Corriente mínima recomendada del rail 5V hacia el subsistema de audio: 1 A.**

### 8.2 Distribución de masas

```
GND_BATTERY (punto de estrella del vehículo)
    │
    ├── GND_MOTOR  (BTS7960, corrientes > 10 A) ── cable 4 mm²
    ├── GND_RELAY  (bobinas relés de potencia)  ── cable 1,5 mm²
    ├── GND_DIGITAL (STM32 + ESP32 + sensores)  ── cable 0,5 mm²
    └── GND_AUDIO   (PAM8403 + DFPlayer + radio) ── cable 0,5 mm² SEPARADO

                    ▲ GND_AUDIO llega directo al punto de estrella.
                    ▲ NO encadenar a través de GND_MOTOR ni GND_RELAY.
```

> ⚠️ **CRÍTICO:** La corriente de retorno de los motores BTS7960 genera caídas de
> mV en la resistencia del cobre del GND_MOTOR. Si GND_AUDIO comparte tramo con
> GND_MOTOR, esas caídas se acoplan al amplificador PAM8403 como zumbido (hum 50 Hz)
> o ruido PWM (~20–50 kHz). El punto de estrella único elimina este problema.

---

## 9. IDENTIFICACIÓN DEL LINE-OUT DEL REPRODUCTOR ORIGINAL

### 9.1 Situación actual (verificada)

✅ El reproductor se ha comprobado que arranca correctamente a 5,10 V sin altavoz.  
✅ Se reinicia cuando el altavoz está conectado (etapa de potencia interna sobrecarga el rail).  
⚠️ El punto exacto de la señal de línea en la PCB P5470R251 **no ha sido medido**.

### 9.2 Procedimiento de localización

**Material necesario:** Multímetro (modo AC mV), fuente 5V/1A, cable USB/BT para música.

1. Alimentar la placa a 5V. Reproducir música continua por BT o USB.
2. Conectar GND del multímetro al GND de la placa.
3. En el conector **K2** (cara posterior, 4 pines):
   - Medir entre cada pin y GND en AC mV.
   - Si algún par de pines muestra 50–500 mV AC → línea de altavoz interna
     (salida del amplificador; tiene DC bias, NO conectar directamente).
4. En el conector **blanco de 3 pines** (cara frontal):
   - Mismo procedimiento.
   - Un pin con AC < 500 mV y DC < 2 V puede ser line-out real.
5. En los pads de prueba o vías accesibles cerca de **U4** (cara posterior):
   - El SoC Jieli suele tener salidas DAC accesibles en pads de test.
   - Buscar pad con AC 100–500 mV y DC ≈ 1,6 V (mitad de 3,3 V → referencia ratiométrica).

### 9.3 Cómo distinguir salida de amplificador vs line-out

| Característica | Salida amplificador (NO usar) | Line-out (usar) |
|----------------|-------------------------------|-----------------|
| Tensión AC con música | > 4 Vpp | 0,2–2 Vpp |
| Tensión DC en reposo | ~VCC/2 = ~2,5 V con filtro, o 0 V BTL | ~1,6 V (3,3 V/2) |
| Impedancia de salida | < 1 Ω | 100 Ω – 1 kΩ |
| ¿Soporta carga directa 4Ω? | Sí (es su función) | No (distorsiona) |

> Si no se puede localizar un line-out accesible en la PCB, ver alternativa §9.4.

### 9.4 Alternativa si no hay line-out accesible

🔵 Hipótesis: Si el amplificador de la placa original es BTL (bridge tied load),
no hay referencia de GND en su salida y es imposible tomar señal directamente.  
En ese caso, la alternativa es usar un **transformador de audio de línea** (1:1, impedancia
de alta) conectado a los terminales de altavoz de la placa, con la bobina primaria
en la salida y la bobina secundaria como señal de línea al relé.  
Valor típico: **EI14 / EI19 de audio, relación 1:1, impedancia 1 kΩ:1 kΩ**.  
⚠️ Esta alternativa requiere que el amplificador interno tenga altavoz siempre
conectado (la carga), por lo que el problema de reinicio persiste.

**En ese caso, la solución definitiva es:** alimentar la placa original a **12 V**
(o encontrar un suministro de corriente suficiente a 5 V) y
usar el transformador de aislamiento como line-out. Esto es el escenario de
respaldo; se evalúa solo si §9.2 no encuentra señal de línea accesible.

---

## 10. PRUEBAS DE VALIDACIÓN

### 10.1 Prueba 1 — Confirmar line-out del reproductor original

- [ ] Seguir procedimiento §9.2.
- [ ] Anotar pin/pad y tensión AC medida.
- [ ] Verificar que el nivel es < 2 Vpp (compatible con entrada PAM8403 sin saturar).
- [ ] Confirmar que no hay DC offset > 2 V (si lo hay, el C_AC de 10 µF lo bloqueará igualmente).

### 10.2 Prueba 2 — Verificar ausencia de saturación del PAM8403

- [ ] Conectar la señal de línea localizada con C_AC (10 µF) + R_at (10 kΩ) al PAM8403 LINP.
- [ ] Reproducir música al máximo volumen del reproductor original.
- [ ] Ajustar volumen hasta que el sonido sea limpio (sin distorsión).
- [ ] Si distorsiona a volumen bajo: reducir R_at a 22 kΩ o añadir un divisor adicional.
- [ ] Si el nivel es demasiado bajo: reducir R_at a 4,7 kΩ.

### 10.3 Prueba 3 — Confirmar funcionamiento del relé de audio

- [ ] Enviar desde HMI un comando de reproducción de cualquier track.
- [ ] Verificar que GPIO11 va a HIGH durante la reproducción (medir con multímetro DC).
- [ ] Verificar que ULN2803A 3C va a ~0 V cuando GPIO11=HIGH.
- [ ] Verificar que el altavoz reproduce la voz del DFPlayer (y no la radio).
- [ ] Verificar que tras terminar la voz (5 s), el relé vuelve a OFF y la radio se escucha de nuevo.

### 10.4 Prueba 4 — Anti-click

- [ ] Escuchar el altavoz durante la conmutación ON/OFF del relé.
- [ ] No debe haber clic audible. Si lo hay, aumentar RELAY_RELEASE_MS en `relay_audio.h`.

### 10.5 Prueba 5 — Sin reinicios del reproductor

- [ ] Conectar el reproductor original a 5V con altavoz SIN conectar a su salida de potencia.
- [ ] Verificar que NO se reinicia al reproducir música (la carga ahora es solo la entrada del relé, que es de alta impedancia).

### 10.6 Prueba 6 — Ausencia de ruido de motores

- [ ] Con el vehículo en movimiento (motores activos), reproducir música.
- [ ] Verificar ausencia de zumbido o ruido PWM en el altavoz.
- [ ] Si hay ruido: verificar punto de estrella de masas y la ferrita FB1.

---

## 11. RIESGOS DETECTADOS

| Riesgo | Severidad | Estado | Mitigación |
|--------|-----------|--------|-----------|
| Line-out del reproductor no accesible | Alta | ⚠️ A verificar | Ver §9.4 alternativa con transformador |
| Reproductor se reinicia incluso sin altavoz en su salida | Media | ⚠️ A verificar | Si ocurre: el SoC necesita > 5 V; alimentar a 5,5 V o usar un LDO ajustable |
| Offset DC en DAC_L/R del DFPlayer | Alta | ✅ Mitigado | Condensadores C_AC (10 µF) en serie obligatorios |
| Saturación PAM8403 | Alta | ✅ Mitigado | Divisor R_at (10 kΩ) + R_gnd (10 kΩ) en cada entrada |
| Conectar SPK1/SPK2 del DFPlayer al PAM8403 | Alta | ✅ Evitado en diseño | Usar solo DAC_L/DAC_R |
| Bucle de masa GND_AUDIO ↔ GND_MOTOR | Alta | ✅ Mitigado en diseño | Punto de estrella separado |
| Ruido BTS7960 en rail 5V audio | Media | ✅ Mitigado | Ferrita FB1 100Ω@100MHz |
| Clic al conmutar relé | Baja | ✅ Mitigado en firmware | 20 ms establecimiento + 150 ms cooldown |
| Polaridad inversa VCC audio | Baja | ✅ Mitigado | Diodo Schottky SS14 en serie |
| Flyback de bobina relé daña ULN2803A | Media | ✅ Mitigado | Diodo 1N4148 en paralelo con bobina |

---

## 12. COMPATIBILIDAD CON EL FIRMWARE ACTUAL

| Aspecto | Resultado |
|---------|-----------|
| relay_audio.h / relay_audio.cpp | ✅ Ningún cambio necesario |
| audio_manager.cpp | ✅ Ningún cambio necesario |
| GPIO11 (relay control) | ✅ Correcto, ya asignado |
| GPIO43/44 (DFPlayer UART) | ✅ Correcto, ya asignados |
| STM32 firmware | ✅ Sin cambios |
| CubeMX / .ioc | ✅ Sin cambios |
| CAN protocol | ✅ Sin cambios |
| ESP32 firmware (HMI) | ✅ Sin cambios |
| Librerías platformio.ini | ✅ Sin cambios |

**Conclusión: toda la solución es puramente de hardware.**

---

## 13. ARCHIVOS RELACIONADOS

| Archivo | Contenido |
|---------|-----------|
| `esp32/src/relay_audio.h` | Definición GPIO11, timing, lógica de estado |
| `esp32/src/relay_audio.cpp` | Implementación máquina de estados relé |
| `esp32/src/audio_manager.h` | GPIO43/44 DFPlayer, prioridades, tracks |
| `esp32/src/audio_manager.cpp` | Implementación DFPlayer UART, comandos |
| `docs/AUDIO_RELAY_INTEGRATION.md` | Detalles del control GPIO→ULN→Songle |
| `docs/AUDIO_WIRING_GUIDE.md` | Mapeo completo ULN2803A y relés |
| `docs/AUDIO_SCHEMATIC_ASCII.md` | Esquema de control del relé |
| `docs/AUDIO_TRACKS_GUIDE.md` | Lista de 68 tracks MP3 |
| `docs/POWER_DISTRIBUTION.md` | Rail 5V, relés, tabla de pines |

---

*Documento basado en verificación directa del código fuente del firmware y pruebas
reales reportadas por el usuario. Los aspectos marcados con ⚠️ requieren medición
con multímetro u osciloscopio antes de la conexión definitiva.*
