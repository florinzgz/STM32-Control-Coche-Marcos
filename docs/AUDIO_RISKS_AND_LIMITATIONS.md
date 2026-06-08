# AUDIO_RISKS_AND_LIMITATIONS — Riesgos y Limitaciones del Sistema de Audio

**Versión:** 1.0  
**Fecha:** 2026-06-08  
**Estado:** Documentación permanente — NO modifica código ni hardware  
**Prerequisito:** Leer antes `docs/AUDIO_ARCHITECTURE.md`, `docs/AUDIO_WIRING_GUIDE.md`, `docs/AUDIO_SCHEMATIC_ASCII.md`

---

## Índice

1. [Errores que NO deben cometerse NUNCA](#1-errores-que-no-deben-cometerse-nunca)
2. [Riesgos eléctricos](#2-riesgos-eléctricos)
3. [Riesgos de integración firmware](#3-riesgos-de-integración-firmware)
4. [Limitaciones conocidas del hardware](#4-limitaciones-conocidas-del-hardware)
5. [Elementos pendientes de medir](#5-elementos-pendientes-de-medir)
6. [Restricciones absolutas del proyecto](#6-restricciones-absolutas-del-proyecto)
7. [Síntomas de problemas comunes](#7-síntomas-de-problemas-comunes)
8. [Lista de verificación antes de energizar](#8-lista-de-verificación-antes-de-energizar)

---

## 1. Errores que NO deben cometerse NUNCA

### 1.1 Errores en el PAM8403

| # | Error | Consecuencia |
|---|-------|--------------|
| E1 | Conectar OUTL− (o OUTR−) a GND o chasis | **Destrucción inmediata del PAM8403.** Topología BTL: OUTL− es una salida activa, no una referencia. |
| E2 | Conectar un terminal del altavoz a GND_CHASIS | Equivalente a E1. Cortocircuito en la etapa BTL. |
| E3 | Conectar ambos canales de salida (L y R) al mismo altavoz en paralelo | Cortocircuito entre salidas. Destrucción del PAM8403. |
| E4 | Alimentar a más de 5.5V | Fuera de especificación. Daño progresivo o inmediato. |
| E5 | Usar altavoz de impedancia < 4Ω a plena potencia | Exceso de corriente, sobretemperatura, activación de protección térmica o daño. |
| E6 | Conectar el módulo original al canal R del PAM8403 y DFPlayer al canal L sin relé | Las masas de audio de ambas fuentes se conectarían, posible bucle de tierra y degradación de audio. |

### 1.2 Errores en la conexión DFPlayer ↔ ESP32-S3

| # | Error | Consecuencia |
|---|-------|--------------|
| E7 | Conectar DFPlayer TX directamente a GPIO ESP32 sin verificar el voltaje TX | Si TX es 5V, puede dañar los GPIO del ESP32-S3 (máx. 3.6V tolerante VIH). |
| E8 | Omitir la resistencia serie de 1kΩ en GPIO43→DFPlayer_RX | Sin protección contra sobrecorriente. Pequeño riesgo pero buena práctica obligatoria. |
| E9 | Usar la biblioteca Arduino DFPlayerMini_Fast o similar | El firmware ya implementa protocolo propio (10 bytes manual). Añadir biblioteca produciría conflicto y comportamiento impredecible. |
| E10 | Modificar UART2 o los GPIO43/44 para otro propósito | Rompe el audio sin advertencia de compilación. GPIO43=TX fijo en `audio_manager.h`. |

### 1.3 Errores en la conexión del módulo original

| # | Error | Consecuencia |
|---|-------|--------------|
| E11 | Conectar la salida de altavoz amplificada del mazo de cables directamente al PAM8403 | El PAM8403 espera señal de línea (~1V). Una señal amplificada (≥ 5V) saturará la entrada y posiblemente dañará el módulo. |
| E12 | Conectar el mazo de cables sin medir primero | Las funciones de los cables están sin confirmar. Un cable de +12V confundido con GND o señal destruirá el PAM8403 y posiblemente el ESP32. |
| E13 | Usar el jack 3.5mm sin verificar offset DC | Si hay offset DC y no hay condensador de desacoplo, el PAM8403 recibirá corriente continua en la entrada. |
| E14 | Conectar el jack del módulo original mientras está encendido a 12V sin condensador de desacoplo | Riesgo de descarga de energía almacenada hacia la entrada del PAM8403. |

### 1.4 Errores en el relé

| # | Error | Consecuencia |
|---|-------|--------------|
| E15 | Usar relé de potencia (12V, 10A) en lugar de relé de señal (5V, señal analógica) | Los relés de potencia tienen contactos en miniatura con recubrimiento de plata optimizado para corriente, no para señal AC de baja amplitud. Introduce ruido e impedancia. |
| E16 | Omitir el diodo flyback en la bobina del relé | El transistor de conmutación del ESP32 (o GPIO directo) recibirá el pico inductivo al apagar el relé. Puede dañar el GPIO. |
| E17 | Controlar la bobina del relé directamente desde GPIO11 sin transistor si el relé consume > 8mA | GPIO del ESP32-S3 máx. 12mA. Verificar datasheet del relé elegido. Si consume > 8mA usar MOSFET o transistor NPN. |
| E18 | Usar relé SPDT en lugar de DPDT | Con SPDT se conmuta la señal pero no el retorno de masa, creando posibles bucles de tierra entre fuentes. |

---

## 2. Riesgos eléctricos

### 2.1 Bucle de tierra (Ground Loop)

**Descripción:** Si la masa del módulo original (12V, conectado a chasis) y la masa del ESP32/DFPlayer (electrónica 5V/3.3V) se conectan por más de un camino, se forma un bucle que actúa como antena.

**Síntoma:** Zumbido de 50/100Hz en el altavoz, peor cuando el coche se mueve o los motores están activos.

**Causa en este proyecto:** El PAM8403 en configuración BTL rompe el bucle de tierra hacia el altavoz. Sin embargo, si el GND del jack del módulo original se conecta directamente a GND_chasis Y el GND del DFPlayer también llega a GND_chasis por otro camino, el bucle existe antes del PAM8403.

**Mitigación:** Usar el Polo 2 del relé DPDT para conmutar también el retorno de masa. Cada fuente solo conecta su GND al PAM8403 cuando está activa.

**Estado:** CONFIRMADO en `docs/AISLAMIENTO_AUDIO_DFPLAYER.md`. PAM8403 BTL documentado como solución principal.

### 2.2 Ruido de motores (PWM 20kHz)

**Descripción:** Los motores de tracción usan PWM a ~20kHz. En un entorno donde la masa de potencia y la masa de audio no están separadas, este ruido puede aparecer como portadora de alta frecuencia en el audio.

**Síntoma:** Tono agudo o sibilancia en el altavoz, proporcional a la velocidad/carga del motor.

**Mitigación:**
- Condensadores de desacoplo en 5V (470µF electrolítico + 100nF cerámico, junto al PAM8403).
- Masa de audio separada de masa de potencia (motores), unidas solo en un único punto de estrella.
- Cableado de audio alejado físicamente del cableado de motores.

**Estado:** RIESGO CONOCIDO. Ver `docs/AISLAMIENTO_AUDIO_DFPLAYER.md` para análisis completo.

### 2.3 Picos inductivos del relé

**Descripción:** Al desactivar el relé, la bobina genera un pico de voltaje inverso que puede alcanzar 50–100V en microsegundos.

**Mitigación obligatoria:** Diodo flyback (1N4148 o 1N4007) en antiparalelo con la bobina del relé.

**Estado:** PENDIENTE — confirmar si el relé físico elegido incluye protección interna o si requiere diodo externo.

### 2.4 Tensión TX del DFPlayer desconocida

**Descripción:** El DFPlayer Mini puede funcionar a 3.3V o a 5V según el lote de fabricación. El pin TX puede generar señal de 5V aunque el GPIO del ESP32-S3 tolere máximo 3.6V.

**Mitigación:** Medir con multímetro antes de conectar. Si es 5V, añadir divisor de tensión: 2kΩ (serie DFPlayer_TX) + 1kΩ (a GND) → ~1.67V entrada ESP32.

**Estado:** PENDIENTE DE MEDIR.

### 2.5 Offset DC en el jack del módulo original

**Descripción:** Algunos módulos de audio económicos no incluyen condensadores de desacoplo en sus salidas. La tensión DC de polarización del DAC interno puede aparecer en el jack (típicamente 50–300mV).

**Mitigación:** Condensador de desacoplo de 1µF en serie entre el jack y el relé. Obligatorio en este diseño independientemente del resultado de la medición.

**Estado:** PENDIENTE DE MEDIR. 1µF en el diseño como protección por defecto.

---

## 3. Riesgos de integración firmware

### 3.1 Relé watchdog (7000ms)

**Descripción:** El firmware en `relay_audio.h/cpp` tiene un watchdog que fuerza el relé a OFF después de 7000ms máximo. Si una pista de audio dura más de 7 segundos, el relé se desactivará aunque el DFPlayer siga reproduciendo.

**Estado:** CONFIRMADO en `esp32/src/relay_audio.cpp` (RELAY_MAX_ON_MS = 7000).

**Implicación:** Ninguna pista de las 68 actuales debe durar más de 7 segundos. Verificar antes de añadir nuevas pistas.

### 3.2 Cooldown de reproducción (4000ms)

**Descripción:** El firmware implementa un cooldown de 4000ms entre pistas en `audio_manager.cpp`. No se puede reproducir una segunda voz antes de que expire el temporizador.

**Estado:** CONFIRMADO en `esp32/src/audio_manager.cpp`.

**Implicación:** Si se acumulan eventos simultáneos (por ejemplo, cambio de marcha + alerta de batería simultáneos), solo el de mayor prioridad sonará. El otro se descartará.

### 3.3 Tiempo de establecimiento del relé (20ms)

**Descripción:** El firmware espera 20ms desde requestOn() hasta que isReady() = true. El DFPlayer no debe recibir el comando PLAY antes de que isReady() sea true.

**Estado:** CONFIRMADO en `relay_audio.h` (RELAY_ESTABLISH_MS = 20). El firmware ya lo gestiona correctamente.

**Riesgo:** Si el firmware se modifica en el futuro y se llama a play() antes de isReady(), el primer frame de audio se perderá o el relé aún no habrá cerrado el contacto.

### 3.4 GPIO11 es active LOW

**Descripción:** El relé se activa con GPIO11 = LOW (0V). En reposo o en fallo del firmware, GPIO11 = HIGH → relé OFF → módulo original activo.

**Estado:** CONFIRMADO en `relay_audio.h` y `relay_audio.cpp`.

**Implicación positiva:** Fail-safe correcto. Si el ESP32 se resetea o el firmware falla, el módulo original seguirá sonando normalmente. El DFPlayer no producirá sonido indeseado.

### 3.5 No hay retroalimentación del estado del relé hacia el STM32

**Descripción:** El estado del relé (ON/OFF) no se transmite por CAN al STM32. El STM32 no sabe si el audio está activo o no.

**Implicación:** No hay consecuencias funcionales actuales. Limitación a documentar si en el futuro se desea telemetría de audio en el STM32.

---

## 4. Limitaciones conocidas del hardware

### 4.1 DFPlayer Mini — calidad de audio

- DAC interno de 16 bits, pero relación señal/ruido real ~60-65dB (fabricación económica).
- Nivel de salida DAC_L/DAC_R variable según lote.
- No garantiza calidad audiófila. Adecuado para voces y alertas, no para música de alta fidelidad.
- **Conclusión:** Suficiente para el uso previsto (avisos de vehículo).

### 4.2 Módulo original — funciones desconocidas

- **CONFIRMADO:** BT, USB, microSD, FM (visible en display), jack 3.5mm, botones K2-K4.
- **HIPÓTESIS:** El jack 3.5mm es LINE OUT o headphone out.
- **PENDIENTE:** Función del mazo de cables de 4 hilos (amarillo/negro/rojo-magenta/amarillo).
- **PENDIENTE:** Si el jack tiene offset DC significativo.
- **PENDIENTE:** Nivel exacto de señal en el jack.
- **DESCONOCIDO:** Si el módulo original tiene UART o algún protocolo de control.

### 4.3 PAM8403 — mono forzado

- Solo se usa el canal L (OUTA+/OUTA−). El canal R no está conectado.
- Altavoz único mono: esto es correcto para el uso previsto.
- El canal R se referencia a GND mediante 100kΩ para evitar oscilaciones.

### 4.4 ESP32-S3 — sin DAC de audio nativo

- El ESP32-S3 no tiene DAC de audio disponible en GPIO accesibles (los DAC de los ESP32 originales no están disponibles en S3).
- No es posible reproducir audio directamente desde el ESP32-S3 sin hardware adicional (MAX98357A, PCM5102, etc.), lo cual requeriría cambios de firmware no permitidos.
- **Conclusión:** DFPlayer Mini es la solución correcta para generación de audio desde ESP32-S3.

### 4.5 MAX98357A — explícitamente descartado

- Requiere interfaz I2S.
- El firmware actual no implementa I2S de audio.
- Implementarlo requeriría reescritura significativa del firmware.
- **Motivo de descarte:** Incompatible con la restricción de no modificar firmware.

---

## 5. Elementos pendientes de medir

Estos elementos son **HIPÓTESIS** hasta que se midan físicamente. NO conectar nada sin completar estas verificaciones.

| # | Elemento | Medición necesaria | Riesgo si no se mide |
|---|----------|-------------------|---------------------|
| M1 | Tensión TX del DFPlayer Mini | V DC en pin TX con multímetro | Daño GPIO44 del ESP32-S3 si TX es 5V |
| M2 | Offset DC en jack 3.5mm del módulo original | V DC entre Tip y Sleeve | Offset alto puede degradar PAM8403 o crear pop |
| M3 | Nivel de señal AC en jack del módulo original | V AC (escala 2V) entre Tip y Sleeve durante reproducción | Señal excesiva saturará PAM8403 |
| M4 | Función cable negro del mazo | V DC respecto a chasis | Podría ser GND, alimentación o señal |
| M5 | Función cables amarillos del mazo | V DC y V AC respecto a GND | Podrían ser altavoz amplificado (+12V pico) |
| M6 | Función cable rojo/magenta del mazo | V DC y V AC respecto a GND | Igual que M5 |
| M7 | Consumo de bobina del relé elegido | mA de la bobina a 5V | > 12mA requiere transistor; sin transistor daña GPIO11 |
| M8 | Impedancia del altavoz disponible | Ohmímetro | < 4Ω sobrecarga el PAM8403 a plena potencia |

---

## 6. Restricciones absolutas del proyecto

Las siguientes restricciones son **ABSOLUTAS** y no admiten excepciones:

### 6.1 Código y firmware

- ❌ NO modificar `Core/Src/safety_system.c` ni ningún archivo relacionado con el sistema de seguridad STM32.
- ❌ NO modificar la configuración CAN (`Core/Src/can_handler.c`, IDs, timing, DLC).
- ❌ NO modificar los algoritmos PID ni el control de tracción.
- ❌ NO modificar el control de los relés de potencia (motores, dirección).
- ❌ NO modificar `esp32/src/shifter_input.cpp` ni la lectura de la palanca de cambios.
- ❌ NO añadir periféricos I2C nuevos al bus TCA9548A sin auditoría de address conflicts.
- ❌ NO cambiar GPIO43 ni GPIO44 (asignados permanentemente al DFPlayer Mini).
- ❌ NO cambiar GPIO11 (asignado permanentemente al relé de audio).

### 6.2 Hardware

- ❌ NO conectar ningún terminal del altavoz a GND o a chasis metálico del vehículo.
- ❌ NO conectar el mazo de cables del módulo original sin haber completado todas las mediciones de la sección M4–M6.
- ❌ NO alimentar el PAM8403 a más de 5.5V.
- ❌ NO usar altavoz de impedancia < 4Ω.
- ❌ NO conectar la salida de altavoz amplificada del módulo original a ningún punto del sistema de audio electrónico.

---

## 7. Síntomas de problemas comunes

### 7.1 Zumbido constante en el altavoz

**Posibles causas:**
1. Bucle de tierra entre el módulo original y la electrónica 5V.
2. Condensadores de desacoplo ausentes o incorrectos en PAM8403 VCC.
3. Cableado de audio paralelo al cableado de motores.

**Diagnóstico:**
- Desconectar el módulo original → si el zumbido desaparece, es bucle de tierra.
- Desconectar la alimentación de los motores → si mejora, es ruido EMI de motores.

**Solución:** Revisar topología de masas. Usar Polo 2 del DPDT para conmutar también el retorno.

### 7.2 El DFPlayer no reproduce (silencio)

**Posibles causas:**
1. Relé no activo cuando se envía PLAY (isReady() = false).
2. microSD no formateada en FAT32 o archivos no en carpeta `01/`.
3. UART2 configurada incorrectamente (baud, pin swap).
4. VCC del DFPlayer < 4.2V (tensión insuficiente).

**Diagnóstico:**
- Medir VCC del DFPlayer: debe ser 4.2–5.5V.
- Verificar que GPIO11 esté LOW cuando debe sonar.
- Verificar microSD con lector de tarjetas en PC.

### 7.3 Pop o clic al activar el relé

**Posibles causas:**
1. Condensador de desacoplo de 1µF ausente antes del relé.
2. Offset DC elevado en el jack del módulo original.
3. Pull-down de 100kΩ en IN_L del PAM8403 ausente.

**Solución:** Añadir los condensadores indicados en `docs/AUDIO_WIRING_GUIDE.md`. Verificar offset DC del jack.

### 7.4 Corte de audio antes de fin de pista

**Posibles causas:**
1. Pista de audio dura más de 7000ms → watchdog del relé desactiva.
2. Cooldown de 4000ms no expirado → nueva pista descartada.

**Diagnóstico:**
- Verificar duración de todas las pistas. Ninguna debe superar 6.5 segundos.
- Verificar logs UART del ESP32 para mensajes de audio_manager.

### 7.5 GPIO44 del ESP32 se comporta erráticamente

**Posible causa:** TX del DFPlayer está a 5V y está dañando el GPIO del ESP32-S3 progresivamente.

**Acción inmediata:** Desconectar GPIO44 del DFPlayer TX. Medir tensión TX del DFPlayer con multímetro antes de reconectar. Añadir divisor 2kΩ/1kΩ si es necesario.

---

## 8. Lista de verificación antes de energizar

Completar esta lista en orden antes de encender el sistema por primera vez con las modificaciones de audio:

### Fase 0 — Mediciones previas (SIN energizar)

- [ ] M1: Medir tensión TX del DFPlayer Mini (documentar resultado)
- [ ] M2: Medir offset DC en jack 3.5mm del módulo original (documentar resultado)
- [ ] M3: Medir nivel AC en jack del módulo original durante reproducción (documentar resultado)
- [ ] M4–M6: Identificar función de cada cable del mazo (documentar resultado)
- [ ] M7: Verificar consumo de bobina del relé elegido (documentar resultado)
- [ ] M8: Verificar impedancia del altavoz (documentar resultado)

### Fase 1 — Verificaciones visuales (SIN energizar)

- [ ] Condensador 1µF en serie entre DFPlayer DAC_L y relé NO1 ✓
- [ ] Condensador 1µF en serie entre DFPlayer DAC_R y nodo de suma ✓
- [ ] Resistencias 1kΩ en suma estéreo-mono DFPlayer ✓
- [ ] Condensador 1µF en serie entre jack módulo original y relé NC1 ✓
- [ ] Resistencias 1kΩ en suma estéreo-mono módulo original ✓
- [ ] Pull-down 100kΩ en IN_L del PAM8403 ✓
- [ ] 470µF + 100nF en VCC del PAM8403 ✓
- [ ] Resistencia 1kΩ en GPIO43 → DFPlayer RX ✓
- [ ] Diodo flyback en bobina del relé (o confirmado que el relé lo incluye) ✓
- [ ] Ningún terminal del altavoz conectado a GND ✓
- [ ] PAM8403 OUTA− solo conectado al altavoz, no a GND ✓

### Fase 2 — Prueba inicial (A bajo volumen)

- [ ] Reproducir track 0001 desde DFPlayer → voz audible, sin distorsión
- [ ] Verificar que GPIO11 pasa a LOW durante reproducción (LED o multímetro)
- [ ] Verificar retorno a módulo original (GPIO11 HIGH) al finalizar la pista
- [ ] Verificar ausencia de zumbido en reposo con módulo original activo
- [ ] Verificar ausencia de pop al conmutar relé

### Fase 3 — Prueba con sistema completo

- [ ] Hacer circular el vehículo → verificar ausencia de ruido de motores en audio
- [ ] Reproducir audio con BT activo en módulo original y luego DFPlayer → verificar conmutación
- [ ] Verificar que el relé watchdog no corta pistas < 6.5s
- [ ] Verificar que el módulo BT/USB/SD/FM sigue funcionando normalmente

---

*Documento creado: 2026-06-08*  
*Basado en: firmware verificado (esp32/src/audio_manager.h, relay_audio.h/cpp, audio_manager.cpp), docs/AISLAMIENTO_AUDIO_DFPLAYER.md, docs/AUDIO_HARDWARE_AUDIT.md, análisis de fotos del hardware físico*
