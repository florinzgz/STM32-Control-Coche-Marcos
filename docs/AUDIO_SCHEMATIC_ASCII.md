# AUDIO_SCHEMATIC_ASCII — Esquemas Eléctricos ASCII

**Versión:** 1.0  
**Fecha:** 2026-06-08  
**Estado:** Documentación permanente — NO modifica código ni hardware  
**Prerequisito:** Leer `docs/AUDIO_ARCHITECTURE.md` y `docs/AUDIO_WIRING_GUIDE.md`  
**Convenciones:**
- `[Xk]` → resistencia de X kΩ
- `[XnF]` / `[XµF]` → condensador de ese valor
- `──` → cable / señal analógica
- `──►` → dirección del flujo o señal
- `┤` `├` → nodo de conexión
- `===` → pantalla / cableado de señal diferencial
- `╔╗╚╝║═` → cuadros de módulo

---

## Índice

1. [Esquema completo del sistema de audio](#1-esquema-completo-del-sistema-de-audio)
2. [Esquema DFPlayer ↔ ESP32-S3 (UART + alimentación)](#2-esquema-dfplayer--esp32-s3-uart--alimentación)
3. [Esquema mezcla estéreo a mono DFPlayer](#3-esquema-mezcla-estéreo-a-mono-dfplayer)
4. [Esquema mezcla estéreo a mono módulo original](#4-esquema-mezcla-estéreo-a-mono-módulo-original)
5. [Esquema relé DPDT de conmutación](#5-esquema-relé-dpdt-de-conmutación)
6. [Esquema PAM8403 completo](#6-esquema-pam8403-completo)
7. [Esquema de alimentación y masas](#7-esquema-de-alimentación-y-masas)
8. [Esquema de protección anti-pop](#8-esquema-de-protección-anti-pop)
9. [Esquema de verificación con multímetro](#9-esquema-de-verificación-con-multímetro)
10. [Esquema integrado final](#10-esquema-integrado-final)

---

## 1. Esquema completo del sistema de audio

Vista de alto nivel de todo el sistema:

```
╔══════════════════════════════════════════════════════════════════════════════════╗
║                      SISTEMA DE AUDIO — MARCOSDASHBOARD                          ║
╠══════════════════════════════════════════════════════════════════════════════════╣
║                                                                                  ║
║  ┌───────────────────┐                                                           ║
║  │  MÓDULO ORIGINAL  │                                                           ║
║  │  BT / USB / SD    │                                                           ║
║  │  JieLi SoC        │                                                           ║
║  │  FM               │                                                           ║
║  │  P5470R2S1 (PCB2) │                                                           ║
║  │  Entrada: 12V     │                                                           ║
║  └──────┬────────────┘                                                           ║
║         │ Jack 3.5mm (PENDIENTE VERIFICAR)                                       ║
║         │ Tip (L) + Ring (R) + Sleeve (GND)                                      ║
║         │                                                                        ║
║         │  [1kΩ] [1kΩ]  (suma L+R a mono)                                       ║
║         └──┤                              ├──► NC1                               ║
║            └── MONO_ORIG ──[1µF]──────────┘    │                                ║
║                                                 │   RELÉ DPDT                   ║
║  ┌────────────────────────────────┐              │   (bobina 5V)                 ║
║  │  DFPLAYER MINI                 │   COM1 ─────►│──► PAM8403 IN_L              ║
║  │  UART2 GPIO43(TX)/GPIO44(RX)  │              │                               ║
║  │  microSD: 0001–0068.mp3       │              │   GPIO11 (ESP32-S3)           ║
║  │                                │           NO1│   active LOW                  ║
║  │ DAC_L(p5)──[1kΩ]──┐           │              │   HIGH=NC (módulo original)   ║
║  │                    ├──MONO_DF──┼──[1µF]──► NO1│   LOW=NO  (DFPlayer)          ║
║  │ DAC_R(p4)──[1kΩ]──┘           │                                               ║
║  │ GND(p7)───────────────────────┼──────────► COM2                               ║
║  └────────────────────────────────┘              │                               ║
║                                                  │                               ║
║  ┌────────────────────────────────┐              │                               ║
║  │  PAM8403                       │◄─────────────┘                               ║
║  │  Clase D BTL, 5V               │                                              ║
║  │  IN_L ← COM1                   │  OUTL+ ──────────────────► Altavoz +        ║
║  │  GND  ← COM2                   │  OUTL− ──────────────────► Altavoz −        ║
║  │  VCC  ← 5V                     │                                              ║
║  └────────────────────────────────┘                                              ║
║                                                                                  ║
╚══════════════════════════════════════════════════════════════════════════════════╝
```

---

## 2. Esquema DFPlayer ↔ ESP32-S3 (UART + alimentación)

```
╔═══════════════════════════════════════════════════════════════════════╗
║          CONEXIÓN DFPLAYER MINI ↔ ESP32-S3 (CONFIRMADO en firmware)   ║
╠═══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║    ESP32-S3 DevKitC-1                    DFPlayer Mini                ║
║    ─────────────────                     ───────────────              ║
║                                                                       ║
║    3.3V (o 5V) ─────────────────────────► VCC  (pin 1)               ║
║                                                                       ║
║    GND ──────────────────────────────────► GND  (pin 7)               ║
║                                                                       ║
║    GPIO43 (UART2 TX) ──[1kΩ]────────────► RX   (pin 2)               ║
║                                           ↑                           ║
║                                     OBLIGATORIO: protege GPIO         ║
║                                     y DFPlayer independientemente     ║
║                                     de la tensión VCC elegida         ║
║                                                                       ║
║    GPIO44 (UART2 RX) ◄──────────────────── TX   (pin 3)               ║
║                        ↑                                              ║
║                  Verificar tensión TX con multímetro                  ║
║                  Si > 3.3V: añadir divisor 2kΩ/1kΩ                   ║
║                                                                       ║
╠═══════════════════════════════════════════════════════════════════════╣
║  DIVISOR OPCIONAL si TX del DFPlayer es 5V:                           ║
║                                                                       ║
║  DFPlayer_TX ──[2kΩ]──┬──► GPIO44 (ESP32, 3.3V tolerante)            ║
║                       │                                               ║
║                      [1kΩ]    → reduce 5V a 1.67V < 3.3V ✅           ║
║                       │                                               ║
║                      GND                                              ║
╚═══════════════════════════════════════════════════════════════════════╝
```

**Protocolo UART DFPlayer (CONFIRMADO en `esp32/src/audio_manager.cpp`):**

```
Trama de 10 bytes, 9600 baud, 8N1:
┌──────┬──────┬──────┬──────┬──────┬───────┬───────┬───────┬───────┬──────┐
│ 0x7E │ 0xFF │ 0x06 │ CMD  │ 0x00 │PARAM1 │PARAM2 │CHKHI  │CHKLO  │ 0xEF │
│Start │ Ver  │ Len  │      │ FBK  │       │       │Checksum (2's) │  End │
└──────┴──────┴──────┴──────┴──────┴───────┴───────┴───────┴───────┴──────┘

Comandos usados en firmware:
  0x03 → PLAY_TRACK (param: número de pista)
  0x06 → SET_VOLUME  (param: 0–30)
  0x0C → RESET
```

---

## 3. Esquema mezcla estéreo a mono DFPlayer

```
╔═══════════════════════════════════════════════════════════════════╗
║          MEZCLA ESTÉREO A MONO — DFPLAYER MINI                    ║
╠═══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  DFPlayer Mini                                      al relé NO1  ║
║                                                                   ║
║  DAC_L (pin 5) ──[1kΩ]──────┐                                    ║
║                              ├──── MONO_DF ──[1µF]──────────────►║
║  DAC_R (pin 4) ──[1kΩ]──────┘                                    ║
║                              ↑                                    ║
║                         Nodo de suma                              ║
║                         (ambas resistencias iguales               ║
║                          mantienen relación estéreo)              ║
║                                                                   ║
║  Niveles esperados:                                               ║
║  - DAC_L / DAC_R: ~0.5–1.0 Vpp (a vol máximo)                    ║
║  - MONO_DF: ~0.5–1.0 Vpp (igual, sin pérdida significativa)      ║
║  - Tras 1µF: sin offset DC, señal AC pura                        ║
║                                                                   ║
║  GND (pin 7 DFPlayer) ─────────────────────────────────────────► ║
║                                                          al relé  ║
║                                                          NO2/GND  ║
╚═══════════════════════════════════════════════════════════════════╝
```

---

## 4. Esquema mezcla estéreo a mono módulo original

```
╔═══════════════════════════════════════════════════════════════════╗
║          MEZCLA ESTÉREO A MONO — MÓDULO ORIGINAL                  ║
║          (PENDIENTE VERIFICAR nivel con multímetro)               ║
╠═══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  Jack 3.5mm                                        al relé NC1   ║
║  (módulo original)                                                ║
║                                                                   ║
║  Tip  (canal L) ──[1kΩ]──────┐                                   ║
║                               ├──── MONO_ORIG ──[1µF]──────────►║
║  Ring (canal R) ──[1kΩ]──────┘                                   ║
║                                                                   ║
║  Sleeve (GND) ─────────────────────────────────────────────────► ║
║                                                          al relé  ║
║                                                          NC2/GND  ║
║                                                                   ║
║  ⚠️  Verificar con multímetro ANTES de conectar:                   ║
║     - Presencia de señal AC en Tip y Ring                        ║
║     - Offset DC < 200mV (si mayor → 1µF obligatorio)             ║
║     - Nivel < 1.5V RMS (si mayor → añadir atenuador)             ║
║                                                                   ║
╚═══════════════════════════════════════════════════════════════════╝
```

---

## 5. Esquema relé DPDT de conmutación

```
╔═══════════════════════════════════════════════════════════════════════╗
║          RELÉ DPDT — CONMUTACIÓN DE FUENTES DE AUDIO                  ║
║          Estado reposo: módulo original   Estado activo: DFPlayer     ║
╠═══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║         POLO 1 (señal de audio)                                       ║
║                                                                       ║
║   NC1 ◄──── MONO_ORIG (módulo original, tras [1µF])                   ║
║         │                                                             ║
║         │   Estado reposo (GPIO11=HIGH, relé OFF):                    ║
║   COM1 ─┤   COM conectado a NC → módulo original activo               ║
║         │                                                             ║
║         │   Estado activo (GPIO11=LOW, relé ON):                      ║
║   NO1 ◄──── MONO_DF  (DFPlayer Mini, tras [1µF])   COM conectado a NO║
║                                                                       ║
║   COM1 ──────────────────────────────────────────► PAM8403 IN_L      ║
║                                                                       ║
╠═══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║         POLO 2 (retorno de audio / masa)                              ║
║                                                                       ║
║   NC2 ◄──── GND jack sleeve (módulo original)                         ║
║         │                                                             ║
║   COM2 ─┤                                                             ║
║         │                                                             ║
║   NO2 ◄──── GND DFPlayer (pin 7)                                      ║
║                                                                       ║
║   COM2 ──────────────────────────────────────────► PAM8403 GND       ║
║                                                                       ║
╠═══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║         BOBINA DEL RELÉ                                               ║
║                                                                       ║
║   +5V ──────────────────────────────────────────► Bobina (+)         ║
║   GPIO11 (ESP32) ───────────────────────────────► Bobina (−)         ║
║         ↑                                                             ║
║         active LOW (relay_audio.h: PIN_AUDIO_RELAY = 11)             ║
║                                                                       ║
║   DIODO FLYBACK (si relé no tiene optoacoplador interno):             ║
║                                                                       ║
║   +5V ──┬──────────────────────────────────────► Bobina (+)          ║
║         │ [1N4148 o 1N4007 en antiparalelo con bobina]               ║
║   GND ──┴──────────────────────────────────────► Bobina (−) → GPIO11║
║                                                                       ║
╚═══════════════════════════════════════════════════════════════════════╝
```

---

## 6. Esquema PAM8403 completo

```
╔═══════════════════════════════════════════════════════════════════════╗
║          PAM8403 — AMPLIFICADOR CLASE D BTL                           ║
╠═══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║   +5V ──[100nF]──[470µF]──► VCC (desacoplo junto al módulo)          ║
║                                                                       ║
║   COM1 (relé) ──────────────► IN_L (+)                                ║
║   GND  ─────────────────────► IN_L (−) o AGND                        ║
║                                                                       ║
║   GND ──[100kΩ]──► IN_L     (pull-down anti-pop)                      ║
║                                                                       ║
║   IN_R (+) ──[100kΩ]──► GND  (canal R no usado, referenciado a GND)  ║
║                                                                       ║
║                        PAM8403                                        ║
║                        ┌──────────────────┐                           ║
║           5V ──────────┤VCC            GND├──── GND                  ║
║        IN_L ──────────►┤INA+         OUTA+├──────────────────────►   ║
║         GND ──────────►┤INA−         OUTA−├──────────────────────►   ║
║        IN_R ──[100kΩ]─►┤INB+         OUTB+├── sin conectar            ║
║         GND ──────────►┤INB−         OUTB−├── sin conectar            ║
║                        └──────────────────┘                           ║
║                               │        │                             ║
║                            OUTA+     OUTA−                           ║
║                               │        │                             ║
║                               └──[8Ω]──┘                             ║
║                              Altavoz único                            ║
║                         (NO conectar a GND)                          ║
║                                                                       ║
╠═══════════════════════════════════════════════════════════════════════╣
║  IMPORTANTE:                                                          ║
║  OUTA+ y OUTA− son salidas BTL (diferencial).                        ║
║  Ninguna de las dos debe conectarse a GND ni a chasis.               ║
║  Conectar OUTA− a GND destruirá el PAM8403.                          ║
╚═══════════════════════════════════════════════════════════════════════╝
```

---

## 7. Esquema de alimentación y masas

```
╔═══════════════════════════════════════════════════════════════════════════╗
║          ÁRBOL DE ALIMENTACIÓN Y MASAS — SISTEMA DE AUDIO                 ║
╠═══════════════════════════════════════════════════════════════════════════╣
║                                                                           ║
║  Batería/Fuente                                                           ║
║  12V ─────────────────────────────────────────────────────────────────   ║
║       │                                                                   ║
║       ├─────────────────────────────────────────────────────────────►    ║
║       │                                              Módulo original 12V ║
║       │                                                                   ║
║       │   Buck converter A (12V → 5V, ≥ 2A)                             ║
║       ├──► [BUCK] ──────────────────────────────────────────────────►    ║
║       │      │                                         5V_AUDIO_RAIL     ║
║       │      │   [470µF/10V + 100nF] (junto al buck)                     ║
║       │      ├──────────────────────────────────────► PAM8403 VCC       ║
║       │      ├──────────────────────────────────────► DFPlayer VCC      ║
║       │      ├──────────────────────────────────────► Relé audio VCC    ║
║       │      └──────────────────────────────────────► ESP32-S3 5V pin   ║
║       │                                                                   ║
║       └──► [BUCK o LDO] ─────────────────────────────────────────────►  ║
║                                                              3.3V_LOGIC  ║
║                                                         ESP32-S3 lógica  ║
║                                                                           ║
╠═══════════════════════════════════════════════════════════════════════════╣
║                                                                           ║
║          TOPOLOGÍA DE MASAS                                               ║
║                                                                           ║
║  GND_CHASIS (chasis metálico, retorno de motores)                         ║
║       │                                                                   ║
║       │  [único punto de estrella de masa]                               ║
║       │                                                                   ║
║  GND_ESP32 ────────────────────────────────────────────────────────────  ║
║       │   ESP32-S3 │ DFPlayer │ Relé │ PAM8403 │ Buck conv │ módulo orig  ║
║       │                                                                   ║
║  GND_AUDIO = GND_ESP32 (misma referencia, no conectar a GND_CHASIS       ║
║                          salvo en único punto de estrella)               ║
║                                                                           ║
║  ⚠️  NO crear segundo puente GND_ESP32 ↔ GND_CHASIS a través del         ║
║      altavoz ni del cableado de audio.                                   ║
║      Usar PAM8403 BTL: ningún terminal del altavoz va a GND_CHASIS.      ║
║                                                                           ║
╚═══════════════════════════════════════════════════════════════════════════╝
```

---

## 8. Esquema de protección anti-pop

```
╔═══════════════════════════════════════════════════════════════════════════╗
║          PROTECCIÓN ANTI-POP — CADENA COMPLETA                            ║
╠═══════════════════════════════════════════════════════════════════════════╣
║                                                                           ║
║  Fuente de audio (DFPlayer DAC o módulo original jack)                    ║
║       │                                                                   ║
║       │  [1kΩ + 1kΩ] suma estéreo a mono                                  ║
║       │       │                                                           ║
║       │  MONO_SRC                                                         ║
║       │       │                                                           ║
║       │  [1µF] desacoplo DC ← elimina offset DC y pop de energización     ║
║       │       │                                                           ║
║       │  ────┼──────────────────► al relé (NC o NO)                       ║
║       │   [100kΩ a GND]                                                   ║
║       │       ↑ pull-down: mantiene entrada en 0V cuando relé abre        ║
║       │       ↑ evita "thump" al conectar                                 ║
║       │                                                                   ║
║  Relé DPDT conmuta (20ms establecimiento en firmware)                     ║
║       │                                                                   ║
║  COM1 ────────────────────────────────────────────► PAM8403 IN_L          ║
║                                                                           ║
║  150ms cooldown firmware antes de desactivar relé (relay_audio.h)         ║
║       ↑ evita click al soltar el relé cuando el audio aún tiene cola       ║
║                                                                           ║
║  PAM8403 (BTL) ──────────────────────────────────► Altavoz                ║
║                                                                           ║
║  Secuencia anti-pop:                                                      ║
║  1. requestOn() → GPIO11 LOW → relé se activa                            ║
║  2. 20ms → relay_audio::ESTABLISHING → contacto cerrado                   ║
║  3. relay_audio::isReady() = true → DFPlayer envía comando PLAY           ║
║  4. Audio suena en altavoz                                                ║
║  5. playback timeout / fin pista → release() llamado                      ║
║  6. 150ms → relay_audio::RELEASING → cooldown                            ║
║  7. GPIO11 HIGH → relé desactiva → módulo original vuelve                 ║
║                                                                           ║
╚═══════════════════════════════════════════════════════════════════════════╝
```

---

## 9. Esquema de verificación con multímetro

```
╔═══════════════════════════════════════════════════════════════════════════╗
║          VERIFICACIÓN JACK 3.5mm CON MULTÍMETRO                           ║
╠═══════════════════════════════════════════════════════════════════════════╣
║                                                                           ║
║   Jack 3.5mm TRS (estéreo):                                              ║
║                                                                           ║
║      ┌────────────────────────────────┐                                  ║
║      │ TIP   ← Canal L (Izquierda)   │──► Multímetro punta roja (+)     ║
║      │ RING  ← Canal R (Derecha)     │──► Multímetro punta roja (+)     ║
║      │ SLEEVE← GND / Masa            │──► Multímetro punta negra (−)    ║
║      └────────────────────────────────┘                                  ║
║                                                                           ║
║   Medición 1 — DC offset (multímetro en V DC, escala 2V):               ║
║     TIP − SLEEVE:                                                        ║
║       < 50 mV → normal ✅                                                 ║
║       50–200 mV → offset moderado, 1µF obligatorio ⚠️                    ║
║       > 200 mV → offset alto, verificar más ❌                            ║
║                                                                           ║
║   Medición 2 — Señal AC con audio (multímetro en V AC, escala 2V):      ║
║     TIP − SLEEVE con audio reproduciendo:                                ║
║       0.1–1.5V RMS → línea / headphone ✅ compatible con PAM8403         ║
║       > 2V RMS → señal fuerte, añadir atenuador ⚠️                        ║
║       0V RMS sin audio, > 0.1V con audio → jack funcional ✅              ║
║       0V RMS siempre → jack no activo o sin señal ❌                     ║
║                                                                           ║
╠═══════════════════════════════════════════════════════════════════════════╣
║          VERIFICACIÓN MAZO DE CABLES CON MULTÍMETRO                       ║
╠═══════════════════════════════════════════════════════════════════════════╣
║                                                                           ║
║   Módulo encendido, funcionando normalmente a 12V.                       ║
║   Multímetro punta negra en GND_CHASIS o carcasa.                       ║
║                                                                           ║
║   Cable negro (hipótesis GND):                                           ║
║     Medir V DC → esperar 0.0–0.5V → confirma GND ✅                      ║
║                                                                           ║
║   Cables amarillos y rojo/magenta:                                       ║
║     Medir V DC respecto a GND confirmado:                                ║
║       ~12V → alimentación (+12V) ⚠️ VERIFICADO                            ║
║       0–0.5V en reposo, variable con audio → altavoz (BTL) ⚠️            ║
║       0V siempre → cable pasivo o inactivo                               ║
║                                                                           ║
║   Si cables restantes muestran tensión AC (escala 2V AC):               ║
║     > 0.5V RMS con audio → salida de altavoz AMPLIFICADA                ║
║     Notas:                                                               ║
║     - Salida de altavoz amplificada NO conectar a PAM8403                ║
║     - Usar SOLO el jack 3.5mm para audio hacia PAM8403                  ║
║                                                                           ║
╚═══════════════════════════════════════════════════════════════════════════╝
```

---

## 10. Esquema integrado final

Esquema completo de todo el sistema en un solo diagrama:

```
╔══════════════════════════════════════════════════════════════════════════════════════╗
║                  ESQUEMA INTEGRADO COMPLETO — MARCOSDASHBOARD AUDIO                  ║
╚══════════════════════════════════════════════════════════════════════════════════════╝

12V ───────────────────────────┬──────────────────────────────────────────────────────
                               │                                                       
                        [Módulo original]                                              
                        BT/USB/SD/FM                                                   
                        JieLi SoC                                                      
                        P5470R2S1 (PCB secundaria)                                     
                               │                                                       
                        Jack 3.5mm                                                     
                        (PENDIENTE VERIFICAR)                                          
                        Tip (L) ──[1kΩ]──┐                                            
                        Ring(R) ──[1kΩ]──┼── MONO_ORIG ──[1µF]──► NC1(relé)          
                        Sleeve──────────────────────────────────► NC2(relé)           
                                                                                       
12V ──[BUCK 12→5V]───────┬─────────────────────────────────────────────────────────── 
     [470µF + 100nF]     │                                                             
                         │                                                             
                    [DFPlayer Mini]                                                    
                    GPIO43→RX [1kΩ]                                                   
                    GPIO44←TX                                                          
                         │ DAC_L(p5)──[1kΩ]──┐                                       
                         │ DAC_R(p4)──[1kΩ]──┼── MONO_DF ──[1µF]──► NO1(relé)       
                         │ GND(p7)────────────────────────────────► NO2(relé)         
                         │ VCC ← 5V                                                   
                         │                                                             
                    [ESP32-S3]                                                         
                    GPIO43 TX → DFPlayer                                               
                    GPIO44 RX ← DFPlayer                                               
                    GPIO11 ──────────────────────────────────────► Bobina relé (−)    
                                                                   (active LOW)        
5V ──────────────────────────────────────────────────────────────► Bobina relé (+)   
                                                                                       
                                        ┌───────────────────────────────────────────┐ 
                                        │         RELÉ DPDT 5V                      │ 
                                        │  NC1 ◄── MONO_ORIG (módulo orig)          │ 
                                        │  COM1 ──────────────────────────────────► │→ PAM8403 IN_L
                                        │  NO1 ◄── MONO_DF (DFPlayer)              │ 
                                        │                                           │ 
                                        │  NC2 ◄── GND módulo original             │ 
                                        │  COM2 ──────────────────────────────────► │→ PAM8403 GND
                                        │  NO2 ◄── GND DFPlayer                    │ 
                                        └───────────────────────────────────────────┘ 
                                                                                       
                                                 [PAM8403]                            
                                         IN_L ◄── COM1 (relé)    OUTL+ ──► Altavoz + 
                                         GND  ◄── COM2 (relé)    OUTL− ──► Altavoz − 
                                         VCC  ◄── 5V                                  
                                         [100kΩ pull-down IN_L a GND]                 
                                         [470µF + 100nF en VCC]                       
                                                                                       
                                                  [Altavoz]                           
                                         4–8Ω, máximo 3W (con PAM8403 a 5V)          
                                         NO conectar ningún terminal a GND_CHASIS     
```

---

*Documento creado: 2026-06-08*  
*Basado en: firmware verificado (esp32/src/audio_manager.h, relay_audio.h, audio_manager.cpp), análisis de fotos físicas del hardware, docs/AUDIO_HARDWARE_AUDIT.md §6, docs/AISLAMIENTO_AUDIO_DFPLAYER.md*
