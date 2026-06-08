# AUDIO_WIRING_GUIDE — Cableado Pin a Pin

**Versión:** 1.0  
**Fecha:** 2026-06-08  
**Estado:** Documentación permanente — NO modifica código ni hardware  
**Prerequisito:** Leer `docs/AUDIO_ARCHITECTURE.md` antes de este documento  
**Documentos relacionados:**
- `docs/AUDIO_ARCHITECTURE.md` — arquitectura y decisiones
- `docs/AUDIO_SCHEMATIC_ASCII.md` — esquemas eléctricos
- `docs/AUDIO_RISKS_AND_LIMITATIONS.md` — errores a evitar

---

## Índice

1. [Leyenda de estados](#1-leyenda-de-estados)
2. [Conexión DFPlayer Mini ↔ ESP32-S3](#2-conexión-dfplayer-mini--esp32-s3)
3. [Conexión DFPlayer Mini ↔ PAM8403](#3-conexión-dfplayer-mini--pam8403)
4. [Conexión módulo original ↔ PAM8403](#4-conexión-módulo-original--pam8403)
5. [Conexión relé de audio](#5-conexión-relé-de-audio)
6. [Conexión PAM8403 ↔ altavoz](#6-conexión-pam8403--altavoz)
7. [Verificación del módulo original con multímetro](#7-verificación-del-módulo-original-con-multímetro)
8. [Alimentación completa](#8-alimentación-completa)
9. [Tabla resumen de conexiones](#9-tabla-resumen-de-conexiones)
10. [Procedimiento de primera conexión (orden de pasos)](#10-procedimiento-de-primera-conexión-orden-de-pasos)

---

## 1. Leyenda de estados

| Etiqueta | Significado |
|---|---|
| ✅ CONFIRMADO | Verificado en firmware o en documentación existente |
| ⚠️ HIPÓTESIS | Deducido por análisis técnico, no verificado con medición física |
| ❌ PENDIENTE DE MEDIR | Obligatorio medir antes de conectar |
| ❌ PROHIBIDO | No hacer bajo ninguna circunstancia |

---

## 2. Conexión DFPlayer Mini ↔ ESP32-S3

### 2.1 Pines asignados (CONFIRMADO en firmware)

```
Fuente: esp32/src/audio_manager.h líneas 24-25
inline constexpr int PIN_DFPLAYER_TX = 43;   // ESP32 GPIO43 TX → DFPlayer RX
inline constexpr int PIN_DFPLAYER_RX = 44;   // DFPlayer TX → ESP32 GPIO44 RX
```

### 2.2 Tabla de conexiones UART

| # | De | A | Cable | Resistor serie | Notas |
|---|---|---|---|---|---|
| 1 | ESP32-S3 **GPIO43** (UART2 TX, 3.3V) | DFPlayer **RX** (pin 2) | Verde | **1 kΩ en serie** | Protege el GPIO y el DFPlayer |
| 2 | DFPlayer **TX** (pin 3) | ESP32-S3 **GPIO44** (UART2 RX) | Amarillo | No necesario | DFPlayer TX es 3.3V compatible |
| 3 | ESP32-S3 **GND** | DFPlayer **GND** (pin 7, pin 10) | Negro | — | GND común obligatorio |
| 4 | ESP32-S3 **3.3V** o **5V** | DFPlayer **VCC** (pin 1) | Rojo | — | Ver nota alimentación |

### 2.3 Nota sobre alimentación del DFPlayer

| Opción VCC | Funcionamiento | Riesgo en GPIO43 (TX) |
|---|---|---|
| 3.3V | ✅ Funciona en la mayoría de módulos | ✅ Sin riesgo (misma tensión) |
| 5V | ✅ Siempre compatible | ⚠️ El pin RX del DFPlayer podría estar en lógica 5V → resistor 1kΩ es obligatorio |

> El resistor de 1kΩ en GPIO43→DFPlayer_RX es obligatorio independientemente de la tensión de VCC elegida.  
> Ver análisis en `docs/AUDIO_HARDWARE_AUDIT.md` §6.4 Nota 1.

### 2.4 Verificación de tensión TX del DFPlayer

> ⚠️ PENDIENTE: La tensión de salida del pin TX del DFPlayer Mini varía entre lotes.

**Procedimiento de verificación:**
1. Alimentar el DFPlayer con la tensión elegida (3.3V o 5V).
2. Con el multímetro en V DC, medir entre DFPlayer_TX (pin 3) y GND en reposo.
3. Si mide ≤ 3.3V → conexión directa a GPIO44 es segura.
4. Si mide > 3.3V (próximo a 5V) → añadir divisor resistivo 2kΩ/1kΩ antes de GPIO44.

**Divisor opcional si TX del DFPlayer es 5V:**
```
DFPlayer_TX ──[2kΩ]──┬──► GPIO44 (ESP32, 3.3V max)
                    [1kΩ]
                      │
                     GND
```
Esto reduce 5V → 1.67V (bien por debajo del umbral 3.3V del ESP32).

---

## 3. Conexión DFPlayer Mini ↔ PAM8403

### 3.1 Mezcla estéreo a mono

El altavoz es único. Los canales L y R del DFPlayer deben sumarse a mono mediante resistencias antes del PAM8403.

```
DFPlayer DAC_L (pin 5) ──[1kΩ]──┐
                                 ├──► MONO_DFPLAYER ──[1µF]──► PAM8403 IN_L
DFPlayer DAC_R (pin 4) ──[1kΩ]──┘
```

### 3.2 Tabla de conexiones DFPlayer → PAM8403

| # | De | Componente intermedio | A | Notas |
|---|---|---|---|---|
| 1 | DFPlayer **DAC_R** (pin 4) | R 1kΩ | Nodo MONO_DF | Suma a mono |
| 2 | DFPlayer **DAC_L** (pin 5) | R 1kΩ | Nodo MONO_DF | Suma a mono |
| 3 | Nodo MONO_DF | C 1µF | PAM8403 **IN_L** (o IN_R) | Desacoplo DC y anti-pop |
| 4 | DFPlayer **GND** (pin 7) | — | PAM8403 **GND** | Masa común |
| 5 | PAM8403 **GND** | R 100kΩ a GND | PAM8403 **IN_L** | Pull-down anti-pop (ver nota) |

### 3.3 Nota sobre el condensador de acoplo

El condensador de 1µF elimina cualquier offset DC presente en la salida DAC del DFPlayer y evita el "pop" al conectar. Usar cerámico X5R/X7R 25V o superior, o condensador de película.

### 3.4 Nota sobre el pull-down anti-pop

La resistencia de 100kΩ entre IN_L y GND mantiene la entrada del PAM8403 en 0V cuando ninguna fuente está conectada, evitando un "thump" al activar el amplificador.

### 3.5 No conectar SPK_1 / SPK_2 al PAM8403

> ❌ PROHIBIDO: NO conectar SPK_1 (pin 6) ni SPK_2 (pin 8) del DFPlayer a ninguna entrada del PAM8403.
> 
> SPK_1/SPK_2 son salidas de potencia del amplificador interno del DFPlayer (~3W BTL).
> Conectar dos salidas amplificadas en paralelo destruye ambos amplificadores.

---

## 4. Conexión módulo original ↔ PAM8403

### 4.1 Punto de extracción de audio

> ✅ RECOMENDADO: Usar el **jack 3.5mm** del módulo original.  
> ❌ PENDIENTE DE MEDIR: El mazo de cables NO debe usarse sin verificación.

### 4.2 Tabla de conexiones jack 3.5mm → PAM8403

| # | De | Componente intermedio | A | Notas |
|---|---|---|---|---|
| 1 | Jack **Tip** (canal L) | R 1kΩ | Nodo MONO_ORIG | Suma a mono |
| 2 | Jack **Ring** (canal R) | R 1kΩ | Nodo MONO_ORIG | Suma a mono |
| 3 | Nodo MONO_ORIG | C 1µF | Polo NC del relé (Fuente A) | Desacoplo DC |
| 4 | Jack **Sleeve** (GND) | — | GND audio | Masa común |

> El jack 3.5 mm del módulo original puede ser salida de auriculares (línea baja impedancia) o amplificada. Verificar con multímetro antes de conectar. Ver §7.

### 4.3 Nivel esperado del jack del módulo original

| Tipo de jack | Nivel típico | Compatible con PAM8403 |
|---|---|---|
| Headphone out | 0.5–1.5 Vpp | ✅ Sí |
| Line out (si existe) | 0.5–2.0 Vpp | ✅ Sí |
| Salida amplificada de altavoz | 2–5 Vpp a carga | ❌ NO — satura el PAM8403 |

> ⚠️ Si el jack resulta ser salida de altavoz amplificada, no es posible usarlo directamente en el PAM8403 sin atenuación fuerte (divisor de al menos 10:1).

---

## 5. Conexión relé de audio

### 5.1 Tipo de relé recomendado

**Relé DPDT de señal, bobina 5V, break-before-make**

Opciones válidas:
- Omron G5V-2-5V (recomendado)
- Panasonic TQ2-5V
- Módulo relé de dos canales 5V optoacoplado (opción económica)

> El firmware ya maneja activo LOW en GPIO11. Si se usa módulo con optoacoplador y entrada activa LOW → compatible directamente.

### 5.2 Conexión de control

| # | De | A | Notas |
|---|---|---|---|
| 1 | ESP32-S3 **GPIO11** | Relé **IN** (o bobina −) | Control activo LOW |
| 2 | **5V** | Relé **VCC** (o bobina +) | Alimentación bobina |
| 3 | **GND** | Relé **GND** | Masa |

### 5.3 Conexión de contactos (DPDT — usar polo 1 + polo 2 para señal y retorno)

#### Polo 1 — señal de audio

| Contacto | Conectar a |
|---|---|
| **COM1** | Entrada PAM8403 (IN_L) — después del condensador de acoplo |
| **NC1** | Fuente A: módulo original (señal MONO_ORIG tras 1µF) |
| **NO1** | Fuente B: DFPlayer Mini (señal MONO_DF tras 1µF) |

#### Polo 2 — retorno de audio (masa de audio)

| Contacto | Conectar a |
|---|---|
| **COM2** | GND de entrada del PAM8403 (AGND_PAM) |
| **NC2** | GND del módulo original (jack sleeve) |
| **NO2** | GND del DFPlayer (pin 7) |

> Usar el segundo polo del relé (DPDT) para conmutar también el retorno de audio es importante para evitar bucles de masa entre las dos fuentes.

### 5.4 Estado en reposo y activo

| Estado firmware | GPIO11 | Relé | COM conectado a | Audio activo |
|---|---|---|---|---|
| Sin aviso (reposo) | HIGH | OFF | NC | Módulo original (BT/USB/SD) |
| Aviso DFPlayer | LOW | ON | NO | DFPlayer Mini |

---

## 6. Conexión PAM8403 ↔ altavoz

### 6.1 Usar un solo canal del PAM8403

Para altavoz único, usar exclusivamente el canal L (o R, indistintamente).

| # | De | A | Notas |
|---|---|---|---|
| 1 | PAM8403 **OUTL+** | Altavoz **+** | Señal positiva canal L |
| 2 | PAM8403 **OUTL−** | Altavoz **−** | Señal negativa canal L (BTL) |

> ❌ NO conectar OUTL− a GND. Es una salida activa (BTL). Conectarla a GND destruye el PAM8403.

### 6.2 Canal R sin uso

Dejar las salidas OUTR+ y OUTR− **sin conectar** si no se usa segundo altavoz.  
Conectar la entrada IN_R a GND mediante la resistencia de 100kΩ para evitar ruido en el canal no usado.

### 6.3 Impedancia del altavoz

| Impedancia | Potencia máxima (5V) | Recomendado |
|---|---|---|
| 4Ω | 3W | ✅ Óptimo |
| 8Ω | 1.5W | ✅ Aceptable |
| < 4Ω | — | ❌ No usar — sobrecalienta el PAM8403 |

---

## 7. Verificación del módulo original con multímetro

### 7.1 Procedimiento de verificación del mazo de cables

> ⚠️ Realizar este procedimiento ANTES de conectar cualquier cable del mazo.

**Herramientas necesarias:** Multímetro digital, puntas de prueba.

#### Paso 1 — Identificar GND

1. Con el módulo original **alimentado normalmente a 12V** (su funcionamiento habitual).
2. Poner el multímetro en V DC, escala 20V.
3. Pinchar la punta negra (COM) en el chasis del vehículo o en la carcasa del módulo.
4. Tocar cada cable del mazo con la punta roja.
5. El cable que mida **0.0V** (o < 0.5V) es **GND**. ✅ Anotar su color.

#### Paso 2 — Identificar la alimentación

1. Con GND identificado, medir entre punta roja y cada otro cable.
2. El cable que mida **+12V** (±1V) es la **alimentación**.
3. Con el módulo apagado, el cable +12V puede medir 0V o el mismo valor. Medir con módulo encendido.

#### Paso 3 — Identificar si los cables restantes son altavoz

1. Los cables restantes (probablemente dos) son la salida de altavoz.
2. Con el módulo reproduciendo audio, medir en AC entre los dos cables restantes.
3. Si hay tensión AC > 0.5V RMS → son salida de altavoz BTL. ⚠️ Salida amplificada → **no usar directamente** en PAM8403.
4. Si tensión AC ≈ 0V con audio → son cables inactivos o no son altavoz.

### 7.2 Procedimiento de verificación del jack 3.5mm

#### Paso 1 — Verificar que el jack tiene señal de audio

1. Insertar un jack macho 3.5mm con cables de prueba.
2. Conectar punta roja del multímetro al tip o ring, punta negra a sleeve.
3. Poner multímetro en V AC, escala 2V.
4. Con música reproduciendo, verificar que hay tensión AC > 0.1V RMS en el tip y en el ring.
5. Si hay señal AC → el jack es salida de audio. ✅

#### Paso 2 — Verificar offset DC

1. Con el módulo encendido pero **sin audio reproduciendo**.
2. Poner multímetro en V DC.
3. Medir entre tip y sleeve.
4. Resultado esperado: **< 50 mV DC** (normal en headphone out o line out).
5. Si mide > 200 mV DC → hay offset DC significativo → el condensador de acoplo (1µF) es obligatorio.
6. Si mide > 2V DC → el jack podría no ser salida de audio estándar. Verificar más antes de proceder.

#### Paso 3 — Verificar nivel de señal

1. Con audio reproduciendo a volumen máximo del módulo.
2. Medir V AC entre tip y sleeve.
3. Si mide 0.3–1.5V RMS → señal de línea / headphone. ✅ Compatible con PAM8403.
4. Si mide > 2V RMS → señal fuerte que puede saturar PAM8403. Añadir atenuador (divisor 4.7kΩ/1kΩ).

---

## 8. Alimentación completa

### 8.1 Árbol de alimentación

```
Batería / Alimentación electrónica
12V ──────────────────────────────────────────────────────────────
  │
  ├──► Módulo original BT/USB/SD (12V directo, alimentación normal)
  │
  ├──► Buck converter A (12V → 5V, mínimo 2A)
  │       │
  │       ├──► DFPlayer Mini VCC (5V)
  │       ├──► PAM8403 VCC (5V)
  │       ├──► Relé de audio VCC (5V, bobina)
  │       └──► ESP32-S3 5V (si no tiene su propio regulador)
  │
  └──► Buck converter B (12V → 3.3V, o 5V→3.3V del ESP32 interno)
          └──► ESP32-S3 lógica (3.3V)
```

### 8.2 Masas

| Masa | Qué incluye | Restricciones |
|---|---|---|
| **GND_ESP32** | ESP32-S3, DFPlayer, relé, PAM8403 (señal) | No conectar al GND_chasis por retorno de motores |
| **GND_AUDIO** | PAM8403 AGND, jack sleeve módulo original | Debe ser la misma que GND_ESP32 |
| **GND_CHASIS** | Chasis metálico, baterías, motores | No usar como referencia de audio |

> El sistema ya tiene una topología de masas definida. Ver `docs/AISLAMIENTO_AUDIO_DFPLAYER.md` para el análisis completo del bucle de masa.

### 8.3 Condensadores de desacoplo — alimentación de audio

| Posición | Valor | Tipo | Función |
|---|---|---|---|
| Junto a PAM8403 VCC | 470µF / 10V | Electrolítico | Bulk de energía, suprime caídas de tensión |
| Junto a PAM8403 VCC | 100nF / 50V | Cerámico | Bypass de alta frecuencia |
| Junto a DFPlayer VCC | 100µF / 10V | Electrolítico | Bulk DFPlayer |
| Junto a DFPlayer VCC | 100nF / 50V | Cerámico | Bypass DFPlayer |

Los condensadores deben montarse **lo más cerca posible de los pines VCC y GND** de cada módulo.

---

## 9. Tabla resumen de conexiones

### 9.1 DFPlayer Mini — pinout completo

| Pin | Función | Conectar a |
|---|---|---|
| 1 — VCC | Alimentación | 5V (o 3.3V) |
| 2 — RX | UART entrada | GPIO43 ESP32 **con R 1kΩ en serie** |
| 3 — TX | UART salida | GPIO44 ESP32 (directo) |
| 4 — DAC_R | Audio analógico R | R 1kΩ → nodo MONO_DF |
| 5 — DAC_L | Audio analógico L | R 1kΩ → nodo MONO_DF |
| 6 — SPK_1 | Salida amplificada + | ❌ No conectar |
| 7 — GND | Masa | GND_ESP32 |
| 8 — SPK_2 | Salida amplificada − | ❌ No conectar |
| 16 — BUSY | Estado reproduciendo | ❓ Opcional (no usado en firmware actual) |

### 9.2 ESP32-S3 — pines de audio

| GPIO | Función | Conectar a |
|---|---|---|
| GPIO43 | UART2 TX → DFPlayer RX | DFPlayer pin 2 (con 1kΩ serie) |
| GPIO44 | UART2 RX ← DFPlayer TX | DFPlayer pin 3 |
| GPIO11 | Audio relay control (active LOW) | Relé IN |

### 9.3 PAM8403 — pinout (módulo típico)

| Pin / Pad | Función | Conectar a |
|---|---|---|
| VCC | Alimentación | 5V |
| GND | Masa | GND_ESP32 (GND_AUDIO) |
| IN_L | Entrada señal L | COM1 del relé (tras 1µF) |
| IN_R | Entrada señal R | R 100kΩ a GND (no usado) |
| OUTL+ | Salida L positiva | Altavoz + |
| OUTL− | Salida L negativa | Altavoz − |
| OUTR+/OUTR− | Salida R | Sin conectar |

### 9.4 Relé DPDT — conexiones

| Terminal | Conectar a |
|---|---|
| Bobina (+) | 5V |
| Bobina (−) | GPIO11 ESP32 (active LOW) |
| COM1 | PAM8403 IN_L |
| NC1 | Módulo original jack (tras 1µF) |
| NO1 | DFPlayer MONO_DF (tras 1µF) |
| COM2 | PAM8403 GND |
| NC2 | Jack sleeve del módulo original |
| NO2 | GND del DFPlayer |

---

## 10. Procedimiento de primera conexión (orden de pasos)

> Seguir este orden para evitar daños.

1. **Medir el mazo del módulo original** con multímetro (§7.1) — NO conectar nada del mazo todavía.
2. **Medir el jack del módulo original** con multímetro (§7.2) — confirmar tipo y nivel de señal.
3. **Montar el circuito de mezcla (resistencias + condensadores)** en protoboard — sin alimentación.
4. **Conectar DFPlayer ← ESP32** (UART, alimentación) — verificar funcionamiento por separado.
5. **Conectar DFPlayer → nodo MONO_DF** (resistencias 1kΩ + condensador 1µF).
6. **Conectar el relé** (alimentación 5V + GPIO11).
7. **Conectar PAM8403** (alimentación 5V + entrada desde COM1).
8. **Conectar altavoz** (solo a OUTL+ / OUTL−).
9. **Probar solo DFPlayer**: activar una pista desde firmware, verificar audio en altavoz.
10. **Conectar el módulo original** al polo NC1 del relé — con relé en reposo, verificar que el módulo original suena.
11. **Verificar conmutación**: reproducir una pista DFPlayer mientras el módulo original está activo. El audio debe cambiar automáticamente.

---

*Documento creado: 2026-06-08*  
*Basado en: firmware verificado (esp32/src/audio_manager.h, relay_audio.h, audio_manager.cpp), fotos físicas del hardware real, docs/AUDIO_HARDWARE_AUDIT.md*
