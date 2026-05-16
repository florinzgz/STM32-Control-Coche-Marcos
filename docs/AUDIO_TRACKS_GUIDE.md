# 🔊 Guía de Audios para DFPlayer Mini — SD Card

**Versión:** 1.0  
**Última actualización:** 2026-02-23  
**Referencia firmware original:** [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos) → `include/alerts.h`, `docs/AUDIO_TRACKS_GUIDE.md`

---

## ⚠️ IMPORTANTE: Los archivos MP3 NO están incluidos

Debes grabarlos tú y copiarlos a la tarjeta SD del DFPlayer Mini.

---

## 📝 Pasos Rápidos para Grabar y Subir

### 1. Grabar los audios

#### Recomendación rápida

- **Mejor opción general:** **[TTSMaker](https://ttsmaker.com/)** → gratis, rápido y suficiente para todo el proyecto.
- **Mejor calidad de voz:** **[ElevenLabs](https://elevenlabs.io/)** → suele sonar más natural, pero normalmente requiere cuenta o plan de pago.
- **Mejor opción si quieres una voz humana real:** grabar con micrófono en una sala silenciosa y editar después con **Audacity**.

#### Opciones para grabar las frases

| Opción | Sitio / herramienta | Cuándo usarla | Ventajas | Inconvenientes |
|---|---|---|---|---|
| A | **[TTSMaker](https://ttsmaker.com/)** | Si quieres hacerlo rápido y gratis | Fácil, online, sin complicarte | Voz menos natural que opciones premium |
| B | **[ElevenLabs](https://elevenlabs.io/)** | Si buscas la mejor calidad | Voz muy natural, buena entonación | Puede requerir pago |
| C | **[Narakeet](https://www.narakeet.com/)** o **[PlayHT](https://play.ht/)** | Si quieres comparar varias voces | Buenas voces y exportación simple | Menos directo que TTSMaker |
| D | **Script Python con gTTS** | Si quieres generar muchos audios de golpe | Automatizable, repetible | Voz bastante robótica |
| E | **Micrófono + Audacity** | Si prefieres voz humana personalizada | Resultado muy personal | Requiere más tiempo y edición |

#### Opción A — TTSMaker (recomendada)

1. Ir a **[ttsmaker.com](https://ttsmaker.com/)**
2. Seleccionar idioma: **Español (España)**
3. Elegir una voz clara e infantil o amable
4. Copiar el texto de la columna **"Texto Sugerido"** de las tablas de abajo
5. Configurar las opciones de exportación exactamente así:

| Opción en TTSMaker | ✅ Valor correcto | ❌ Evitar |
|---|---|---|
| **Formato** | **MP3** | WAV (el DFPlayer usa `0001.mp3`…`0068.mp3`) |
| **MP3 Audio Quality** | **Standard (128 kbps)** | High Quality (large size, slow sync) |
| **Voice Speed** | **1.0x (Default)** | — |
| **Voice Volume** | **100% (Default)** | — |
| **Pitch Adjustment** | **Default (normal pitch)** | — |
| **Pause per paragraph** | **0 ms** (mínimo disponible) | Default 300 ms (añade silencio al final) |

6. Clic en **"Start to Convert"**
7. Descargar el MP3
8. Renombrar el archivo a `XXXX.mp3` (ejemplo: `0001.mp3`, `0039.mp3`)

**Ejemplos para copiar y pegar:**

- `0001.mp3` → **"¡Hola Marcos, piloto estrella! Tu coche está listo para la aventura."**
- `0014.mp3` → **"Freno puesto. Coche quieto y seguro."**
- `0031.mp3` → **"Modo emergencia activado. Motor parado para protegerte."**

#### Opción B — ElevenLabs (mejor calidad)

1. Ir a **[elevenlabs.io](https://elevenlabs.io/)**
2. Elegir una voz en español o una voz neutra clara
3. Pegar una frase de la tabla
4. Generar el audio
5. Descargarlo en MP3
6. Renombrarlo con el número de track correspondiente

**Consejo:** úsalo para las frases más importantes (`0001`, `0002`, `0031`, `0054`) si quieres que el sistema suene más profesional.

#### Opción C — Script Python con gTTS (automatizado)

```bash
pip install gTTS
python3 generar_audios.py
```

El script Python completo se encuentra al final de este documento.

#### Opción D — Grabación con micrófono

- Usar micrófono de buena calidad
- Grabar en ambiente silencioso, con cortinas o material blando alrededor
- Hablar a unos **10–15 cm** del micrófono
- Exportar a MP3: **mono, 128 kbps, 22050 Hz**
- Normalizar volumen entre archivos
- Recortar silencios al inicio y al final

#### Mejor sitio para grabar las frases

Si vas a usar **voz humana real**, el mejor sitio es:

- **una habitación pequeña, silenciosa y con poca reverberación**
- con **cortinas, sofá, colchón, ropa o paneles blandos**
- lejos de calle, ventiladores, aire acondicionado y eco

**Ejemplos de buen sitio:**

- un dormitorio con cortinas y cama
- un armario vestidor con ropa alrededor
- una oficina pequeña con alfombra y puerta cerrada

**Evitar:**

- cocina
- garaje vacío
- salón grande con paredes desnudas
- grabar al aire libre con viento

### 2. Preparar la tarjeta SD

| Requisito | Valor |
|-----------|-------|
| **Formato** | FAT32 |
| **Capacidad** | 1 GB – 32 GB |
| **Velocidad** | Clase 4 o superior |
| **Ubicación archivos** | **Raíz** de la SD (NO en carpetas) |

### 3. Copiar archivos a la SD

Copiar **todos los archivos MP3** directamente a la raíz de la tarjeta SD:

```
SD Card (FAT32)
├── 0001.mp3    (Bienvenida / inicio)
├── 0002.mp3    (Apagado / despedida)
├── 0003.mp3    (Error general)
├── ...
└── 0068.mp3    (Beep de confirmación)
```

### 4. Insertar la SD en el DFPlayer Mini

- Apagar el sistema
- Insertar la tarjeta microSD en el módulo DFPlayer Mini
- Encender el sistema
- El primer audio que sonará es `0001.mp3` (bienvenida)

---

## ✅ Pistas que SÍ usa el firmware ahora (generar primero)

Revisado directamente contra firmware:

- `esp32/src/main.cpp` (llamadas reales a `audio::play(...)`)
- `esp32/src/audio_manager.h` (mapeo oficial `Sound -> track`)

Si quieres ir a lo seguro y **no generar de más**, empieza por estas pistas:

| Track | Archivo | Constante Firmware | Cuándo suena (resumen real) | Texto divertido para Marcos |
|---|---|---|---|---|
| 0001 | `0001.mp3` | `WELCOME` | Arranque del sistema | "¡Hola Marcos, piloto estrella! Tu coche está listo para la aventura." |
| 0002 | `0002.mp3` | `FAREWELL` | Apagado del sistema | "Misión cumplida, Marcos. Guardamos el coche y descansamos." |
| 0003 | `0003.mp3` | `ERROR_GENERAL` | DEGRADED/LIMP_HOME y fallos genéricos | "Ups, algo no va bien. Vamos a revisarlo juntos, campeón." |
| 0009 | `0009.mp3` | `ENCODER_ERROR` | Error de centrado/dirección (`CENTERING`) | "La dirección está confundida. Revisemos el encoder." |
| 0010 | `0010.mp3` | `TEMP_HIGH` | Temperatura alta | "Motor calentito. Bajamos ritmo para cuidarlo." |
| 0011 | `0011.mp3` | `TEMP_NORMAL` | Temperatura vuelve a normal | "¡Perfecto! Temperatura del motor en zona segura." |
| 0012 | `0012.mp3` | `BATTERY_LOW` | Batería baja o `BATTERY_OV_WARN` | "Batería bajita. Hora de recargar para seguir jugando." |
| 0013 | `0013.mp3` | `BATTERY_CRITICAL` | Batería crítica o `BATTERY_OV_CRIT` | "Batería súper baja. Paramos tracción para proteger el coche." |
| 0016 | `0016.mp3` | `LIGHTS_ON` | Luces ON (touch o eco CAN) | "Luces encendidas. ¡Brilla, mini piloto!" |
| 0017 | `0017.mp3` | `LIGHTS_OFF` | Luces OFF (touch o eco CAN) | "Luces apagadas. Ahorro de energía activado." |
| 0020 | `0020.mp3` | `GEAR_D1` | Cambio a D1 | "Marcha D1 activada. Salida suave de campeón." |
| 0021 | `0021.mp3` | `GEAR_D2` | Cambio a D2 | "Marcha D2 activada. Un poquito más de alegría." |
| 0022 | `0022.mp3` | `GEAR_REVERSE` | Cambio a R | "Marcha atrás activada. Miramos bien y vamos despacio." |
| 0023 | `0023.mp3` | `GEAR_NEUTRAL` | Cambio a N | "Punto muerto activado. Coche relajado." |
| 0024 | `0024.mp3` | `GEAR_PARK` | Cambio a P | "Modo parking activado. Coche aparcado con seguridad." |
| 0029 | `0029.mp3` | `TEST_SYSTEM` | Recordatorio de mantenimiento | "Comienza revisión total. ¡Chequeo de súper coche!" |
| 0031 | `0031.mp3` | `EMERGENCY` | SAFE/ERROR | "Modo emergencia activado. Motor parado para protegerte." |
| 0032 | `0032.mp3` | `SAFETY_RESET` | Recuperación SAFE/ERROR -> ACTIVE | "Seguridad reiniciada. Volvemos al control." |
| 0033 | `0033.mp3` | `SENSOR_TEMP_ERROR` | `OVERTEMP` como error de sensor | "El sensor de temperatura no responde. Hay que revisarlo." |
| 0034 | `0034.mp3` | `SENSOR_CURRENT_ERROR` | `I2C_FAILURE` | "Lectura de corriente rara. Revisemos el sistema." |
| 0035 | `0035.mp3` | `SENSOR_SPEED_ERROR` | `SENSOR_FAULT` | "No veo velocidad. Comprobemos sensores de rueda." |
| 0037 | `0037.mp3` | `TRACTION_4X4` | Cambio modo tracción 4x4 | "Tracción 4x4 activada. Máximo agarre para la aventura." |
| 0038 | `0038.mp3` | `TRACTION_4X2` | Cambio modo tracción 4x2 | "Tracción 4x2 activada. Conducción suave y eficiente." |
| 0039 | `0039.mp3` | `ABS_ON` | ABS ON | "ABS activado. Frenadas más seguras." |
| 0040 | `0040.mp3` | `ABS_OFF` | ABS OFF | "ABS desactivado. Conduce con extra cuidado." |
| 0041 | `0041.mp3` | `TCS_ON` | TCS ON | "Control de tracción activado. Ruedas bajo control." |
| 0042 | `0042.mp3` | `TCS_OFF` | TCS OFF | "Control de tracción desactivado. Suavidad al acelerar." |
| 0053 | `0053.mp3` | `OVERCURRENT` | `OVERCURRENT` | "Corriente alta detectada. Bajamos fuerza para cuidar el sistema." |
| 0054 | `0054.mp3` | `OBSTACLE_WARN` | Obstáculo crítico / error obstacle | "¡Cuidado! Obstáculo delante. Frenamos suave." |
| 0068 | `0068.mp3` | `BEEP` | Confirmación de cambio en modo tanque | *(beep corto de confirmación)* |

👉 **Total mínimo real para firmware actual: 30 pistas.**

---

## 📋 Lista Completa de Audios (68 Tracks)

### Sistema Principal (Tracks 1-3)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0001 | `0001.mp3` | `WELCOME` | "¡Hola Marcos, piloto estrella! Tu coche está listo para la aventura." |
| 0002 | `0002.mp3` | `FAREWELL` | "Misión cumplida, Marcos. Guardamos el coche y descansamos." |
| 0003 | `0003.mp3` | `ERROR_GENERAL` | "Ups, algo no va bien. Vamos a revisarlo juntos, campeón." |

### Calibración de Pedal (Tracks 4-5)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0004 | `0004.mp3` | `PEDAL_OK` | "¡Genial! Pedal calibrado y listo para jugar en modo seguro." |
| 0005 | `0005.mp3` | `PEDAL_ERROR` | "El pedal necesita ayuda. Revisemos la conexión con calma." |

### Sensores de Corriente (Tracks 6-7)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0006 | `0006.mp3` | `INA_OK` | "Sensores de energía listos. ¡Equipo preparado!" |
| 0007 | `0007.mp3` | `INA_ERROR` | "No leo bien la energía. Comprobemos sensores y cables." |

### Encoder de Dirección (Tracks 8-9)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0008 | `0008.mp3` | `ENCODER_OK` | "Volante sincronizado. ¡Dirección perfecta, piloto!" |
| 0009 | `0009.mp3` | `ENCODER_ERROR` | "La dirección está confundida. Revisemos el encoder." |

### Temperatura (Tracks 10-11)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0010 | `0010.mp3` | `TEMP_HIGH` | "Motor calentito. Bajamos ritmo para cuidarlo." |
| 0011 | `0011.mp3` | `TEMP_NORMAL` | "¡Perfecto! Temperatura del motor en zona segura." |

### Batería (Tracks 12-13)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0012 | `0012.mp3` | `BATTERY_LOW` | "Batería bajita. Hora de recargar para seguir jugando." |
| 0013 | `0013.mp3` | `BATTERY_CRITICAL` | "Batería súper baja. Paramos tracción para proteger el coche." |

### Freno de Estacionamiento (Tracks 14-15)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0014 | `0014.mp3` | `PARKING_BRAKE_ON` | "Freno puesto. Coche quieto y seguro." |
| 0015 | `0015.mp3` | `PARKING_BRAKE_OFF` | "Freno liberado. Listos para movernos con cuidado." |

### Luces (Tracks 16-17)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0016 | `0016.mp3` | `LIGHTS_ON` | "Luces encendidas. ¡Brilla, mini piloto!" |
| 0017 | `0017.mp3` | `LIGHTS_OFF` | "Luces apagadas. Ahorro de energía activado." |

### Radio/Multimedia (Tracks 18-19)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0018 | `0018.mp3` | `RADIO_ON` | "Multimedia encendida. ¡Que empiece la diversión!" |
| 0019 | `0019.mp3` | `RADIO_OFF` | "Multimedia apagada. Nos centramos en conducir." |

### Marchas (Tracks 20-24)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0020 | `0020.mp3` | `GEAR_D1` | "Marcha D1 activada. Salida suave de campeón." |
| 0021 | `0021.mp3` | `GEAR_D2` | "Marcha D2 activada. Un poquito más de alegría." |
| 0022 | `0022.mp3` | `GEAR_REVERSE` | "Marcha atrás activada. Miramos bien y vamos despacio." |
| 0023 | `0023.mp3` | `GEAR_NEUTRAL` | "Punto muerto activado. Coche relajado." |
| 0024 | `0024.mp3` | `GEAR_PARK` | "Modo parking activado. Coche aparcado con seguridad." |

### Menú Oculto y Calibración (Tracks 25-28)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0025 | `0025.mp3` | `MENU_HIDDEN` | "Menú secreto abierto. ¡Modo ingeniero Marcos!" |
| 0026 | `0026.mp3` | `CAL_PEDAL` | "Empezamos calibración de pedal. Pisa suave hasta el final." |
| 0027 | `0027.mp3` | `CAL_INA` | "Calibrando energía. Espera un poquito, casi está." |
| 0028 | `0028.mp3` | `CAL_ENCODER` | "Buscando centro del volante. Déjalo rectito, por favor." |

### Test del Sistema (Tracks 29-30)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0029 | `0029.mp3` | `TEST_SYSTEM` | "Comienza revisión total. ¡Chequeo de súper coche!" |
| 0030 | `0030.mp3` | `TEST_OK` | "¡Todo correcto! Sistemas listos para rodar." |

### Emergencia y Seguridad (Tracks 31-32)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0031 | `0031.mp3` | `EMERGENCY` | "Modo emergencia activado. Motor parado para protegerte." |
| 0032 | `0032.mp3` | `SAFETY_RESET` | "Seguridad reiniciada. Volvemos al control." |

### Errores de Sensores Específicos (Tracks 33-35)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0033 | `0033.mp3` | `SENSOR_TEMP_ERROR` | "El sensor de temperatura no responde. Hay que revisarlo." |
| 0034 | `0034.mp3` | `SENSOR_CURRENT_ERROR` | "Lectura de corriente rara. Revisemos el sistema." |
| 0035 | `0035.mp3` | `SENSOR_SPEED_ERROR` | "No veo velocidad. Comprobemos sensores de rueda." |

### Estado de Módulos (Track 36)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0036 | `0036.mp3` | `MODULE_OK` | "Módulo comprobado. ¡Funciona perfecto!" |

### Tracción 4x4/4x2 (Tracks 37-38)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0037 | `0037.mp3` | `TRACTION_4X4` | "Tracción 4x4 activada. Máximo agarre para la aventura." |
| 0038 | `0038.mp3` | `TRACTION_4X2` | "Tracción 4x2 activada. Conducción suave y eficiente." |

### Sistemas de Seguridad Avanzados (Tracks 39-44)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0039 | `0039.mp3` | `ABS_ON` | "ABS activado. Frenadas más seguras." |
| 0040 | `0040.mp3` | `ABS_OFF` | "ABS desactivado. Conduce con extra cuidado." |
| 0041 | `0041.mp3` | `TCS_ON` | "Control de tracción activado. Ruedas bajo control." |
| 0042 | `0042.mp3` | `TCS_OFF` | "Control de tracción desactivado. Suavidad al acelerar." |
| 0043 | `0043.mp3` | `REGEN_ON` | "Regeneración activada. ¡Recuperamos energía!" |
| 0044 | `0044.mp3` | `REGEN_OFF` | "Regeneración desactivada." |

### WiFi y Conectividad (Tracks 45-48) — Reservado

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0045 | `0045.mp3` | `WIFI_CONNECTED` | "WiFi conectado. ¡Listos para comunicar!" |
| 0046 | `0046.mp3` | `WIFI_DISCONNECTED` | "WiFi desconectado. Seguimos en modo local." |
| 0047 | `0047.mp3` | `OTA_STARTED` | "Actualización iniciada. No apagues el coche, por favor." |
| 0048 | `0048.mp3` | `OTA_COMPLETED` | "Actualización completada. Reiniciando para seguir." |

### Bluetooth (Tracks 49-51) — Reservado

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0049 | `0049.mp3` | `BT_CONNECTED` | "Mando Bluetooth conectado. ¡A jugar!" |
| 0050 | `0050.mp3` | `BT_DISCONNECTED` | "Mando Bluetooth desconectado." |
| 0051 | `0051.mp3` | `BT_PAIRING` | "Buscando mando Bluetooth. Mantén pulsado emparejar." |

### Estados del Vehículo (Tracks 52-56)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0052 | `0052.mp3` | `MAX_SPEED` | "Velocidad máxima alcanzada. ¡Muy bien, piloto!" |
| 0053 | `0053.mp3` | `OVERCURRENT` | "Corriente alta detectada. Bajamos fuerza para cuidar el sistema." |
| 0054 | `0054.mp3` | `OBSTACLE_WARN` | "¡Cuidado! Obstáculo delante. Frenamos suave." |
| 0055 | `0055.mp3` | `PARKING_ASSIST` | "Asistente de parking activado. Te ayudo a aparcar." |
| 0056 | `0056.mp3` | `SOFT_START` | "Arranque suave activado. Salida tranquila." |

### Información de Telemetría (Tracks 57-60)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0057 | `0057.mp3` | `BATTERY_50` | "Batería al 50%. ¡Vamos muy bien!" |
| 0058 | `0058.mp3` | `BATTERY_25` | "Batería al 25%. Mejor recargar pronto." |
| 0059 | `0059.mp3` | `DISTANCE_1KM` | "¡Ya hiciste 1 kilómetro, Marcos!" |
| 0060 | `0060.mp3` | `ENERGY_SAVE` | "Modo ahorro activado. Cuidamos la batería." |

### Modos de Conducción (Tracks 61-63)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0061 | `0061.mp3` | `MODE_ECO` | "Modo Eco activado. Conducción suave y eficiente." |
| 0062 | `0062.mp3` | `MODE_NORMAL` | "Modo Normal activado. Equilibrio perfecto." |
| 0063 | `0063.mp3` | `MODE_SPORT` | "Modo Sport activado. Respuesta más rápida, siempre con control." |

### Feedback de Configuración (Tracks 64-68)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0064 | `0064.mp3` | `CONFIG_SAVED` | "Configuración guardada. ¡Buen trabajo, ingeniero!" |
| 0065 | `0065.mp3` | `CONFIG_RESTORED` | "Ajustes de fábrica restaurados." |
| 0066 | `0066.mp3` | `ERRORS_CLEARED` | "Registro de errores limpio." |
| 0067 | `0067.mp3` | `REGEN_ADJUSTED` | "Nivel de regeneración actualizado." |
| 0068 | `0068.mp3` | `BEEP` | *(Sonido corto de confirmación — buscar "beep sound" en YouTube o generar un tono de 800 Hz, 200 ms)* |

---

## 📊 Resumen de Tracks por Categoría

| Categoría | Rango | Cantidad |
|-----------|-------|----------|
| Sistema principal | 1–3 | 3 |
| Calibración pedal/sensores | 4–9 | 6 |
| Temperatura / Batería | 10–13 | 4 |
| Freno / Luces / Media | 14–19 | 6 |
| Marchas | 20–24 | 5 |
| Menú oculto | 25–28 | 4 |
| Test sistema | 29–30 | 2 |
| Emergencia | 31–32 | 2 |
| Errores sensores | 33–35 | 3 |
| Módulos / Tracción | 36–38 | 3 |
| Seguridad (ABS/TCS/Regen) | 39–44 | 6 |
| WiFi/OTA (reservado) | 45–48 | 4 |
| Bluetooth (reservado) | 49–51 | 3 |
| Estados vehículo | 52–56 | 5 |
| Telemetría | 57–60 | 4 |
| Modos conducción | 61–63 | 3 |
| Config feedback | 64–68 | 5 |
| **TOTAL** | **1–68** | **68** |

---

## 🔧 Gestión de Avisos Superpuestos

El firmware implementa las siguientes protecciones para evitar que los avisos de audio se superpongan:

### Cooldown por sonido (4 segundos)
Cada sonido tiene un cooldown individual de 4 segundos. Si el mismo sonido intenta reproducirse antes de que expire su cooldown, se ignora. **Excepción:** los sonidos de prioridad HIGH (errores, emergencia) siempre se reproducen.

### Prioridad de audio
- **LOW** (0): Marchas, luces, modos, beeps — se interrumpen por cualquier sonido de mayor prioridad
- **MEDIUM** (1): Obstáculo, batería baja, temperatura — reemplazan a LOW, interrumpidos por HIGH
- **HIGH** (2): Errores, emergencia, bienvenida, despedida — siempre se reproducen

### Colapso de errores múltiples
Si más de 1 error/aviso se produce en una ventana de 2 segundos, el sistema reproduce automáticamente `0003.mp3` (ERROR_GENERAL: "Ups, algo no va bien. Vamos a revisarlo juntos, campeón.") en lugar de apilar avisos individuales. Esto evita un bombardeo de mensajes consecutivos que serían confusos.

---

## 🐍 Script Python para Generar Todos los Audios

```python
from gtts import gTTS

textos = {
    "0001": "¡Hola Marcos, piloto estrella! Tu coche está listo para la aventura.",
    "0002": "Misión cumplida, Marcos. Guardamos el coche y descansamos.",
    "0003": "Ups, algo no va bien. Vamos a revisarlo juntos, campeón.",
    "0004": "¡Genial! Pedal calibrado y listo para jugar en modo seguro.",
    "0005": "El pedal necesita ayuda. Revisemos la conexión con calma.",
    "0006": "Sensores de energía listos. ¡Equipo preparado!",
    "0007": "No leo bien la energía. Comprobemos sensores y cables.",
    "0008": "Volante sincronizado. ¡Dirección perfecta, piloto!",
    "0009": "La dirección está confundida. Revisemos el encoder.",
    "0010": "Motor calentito. Bajamos ritmo para cuidarlo.",
    "0011": "¡Perfecto! Temperatura del motor en zona segura.",
    "0012": "Batería bajita. Hora de recargar para seguir jugando.",
    "0013": "Batería súper baja. Paramos tracción para proteger el coche.",
    "0014": "Freno puesto. Coche quieto y seguro.",
    "0015": "Freno liberado. Listos para movernos con cuidado.",
    "0016": "Luces encendidas. ¡Brilla, mini piloto!",
    "0017": "Luces apagadas. Ahorro de energía activado.",
    "0018": "Multimedia encendida. ¡Que empiece la diversión!",
    "0019": "Multimedia apagada. Nos centramos en conducir.",
    "0020": "Marcha D1 activada. Salida suave de campeón.",
    "0021": "Marcha D2 activada. Un poquito más de alegría.",
    "0022": "Marcha atrás activada. Miramos bien y vamos despacio.",
    "0023": "Punto muerto activado. Coche relajado.",
    "0024": "Modo parking activado. Coche aparcado con seguridad.",
    "0025": "Menú secreto abierto. ¡Modo ingeniero Marcos!",
    "0026": "Empezamos calibración de pedal. Pisa suave hasta el final.",
    "0027": "Calibrando energía. Espera un poquito, casi está.",
    "0028": "Buscando centro del volante. Déjalo rectito, por favor.",
    "0029": "Comienza revisión total. ¡Chequeo de súper coche!",
    "0030": "¡Todo correcto! Sistemas listos para rodar.",
    "0031": "Modo emergencia activado. Motor parado para protegerte.",
    "0032": "Seguridad reiniciada. Volvemos al control.",
    "0033": "El sensor de temperatura no responde. Hay que revisarlo.",
    "0034": "Lectura de corriente rara. Revisemos el sistema.",
    "0035": "No veo velocidad. Comprobemos sensores de rueda.",
    "0036": "Módulo comprobado. ¡Funciona perfecto!",
    "0037": "Tracción 4x4 activada. Máximo agarre para la aventura.",
    "0038": "Tracción 4x2 activada. Conducción suave y eficiente.",
    "0039": "ABS activado. Frenadas más seguras.",
    "0040": "ABS desactivado. Conduce con extra cuidado.",
    "0041": "Control de tracción activado. Ruedas bajo control.",
    "0042": "Control de tracción desactivado. Suavidad al acelerar.",
    "0043": "Regeneración activada. ¡Recuperamos energía!",
    "0044": "Regeneración desactivada.",
    "0045": "WiFi conectado. ¡Listos para comunicar!",
    "0046": "WiFi desconectado. Seguimos en modo local.",
    "0047": "Actualización iniciada. No apagues el coche, por favor.",
    "0048": "Actualización completada. Reiniciando para seguir.",
    "0049": "Mando Bluetooth conectado. ¡A jugar!",
    "0050": "Mando Bluetooth desconectado.",
    "0051": "Buscando mando Bluetooth. Mantén pulsado emparejar.",
    "0052": "Velocidad máxima alcanzada. ¡Muy bien, piloto!",
    "0053": "Corriente alta detectada. Bajamos fuerza para cuidar el sistema.",
    "0054": "¡Cuidado! Obstáculo delante. Frenamos suave.",
    "0055": "Asistente de parking activado. Te ayudo a aparcar.",
    "0056": "Arranque suave activado. Salida tranquila.",
    "0057": "Batería al 50%. ¡Vamos muy bien!",
    "0058": "Batería al 25%. Mejor recargar pronto.",
    "0059": "¡Ya hiciste 1 kilómetro, Marcos!",
    "0060": "Modo ahorro activado. Cuidamos la batería.",
    "0061": "Modo Eco activado. Conducción suave y eficiente.",
    "0062": "Modo Normal activado. Equilibrio perfecto.",
    "0063": "Modo Sport activado. Respuesta más rápida, siempre con control.",
    "0064": "Configuración guardada. ¡Buen trabajo, ingeniero!",
    "0065": "Ajustes de fábrica restaurados.",
    "0066": "Registro de errores limpio.",
    "0067": "Nivel de regeneración actualizado.",
    # 0068 es un beep - descargar de internet
}

for num, texto in textos.items():
    print(f"Generando {num}.mp3...")
    tts = gTTS(text=texto, lang='es')
    tts.save(f"{num}.mp3")

print("¡Completado! Generados 67 archivos MP3")
print("Nota: 0068.mp3 (beep) debe descargarse por separado")
```

**Para ejecutar:**
```bash
pip install gTTS
python3 generar_audios.py
```

---

## ✅ Checklist de Grabación

- [ ] Tracks 1–3 (Sistema principal: bienvenida, apagado, error general)
- [ ] Tracks 4–9 (Calibración: pedal, corriente, encoder)
- [ ] Tracks 10–13 (Temperatura y batería)
- [ ] Tracks 14–19 (Freno, luces, multimedia)
- [ ] Tracks 20–24 (Marchas: D1, D2, R, N, P)
- [ ] Tracks 25–28 (Menú oculto y calibración)
- [ ] Tracks 29–30 (Test del sistema)
- [ ] Tracks 31–32 (Emergencia y seguridad)
- [ ] Tracks 33–36 (Errores de sensores y módulos)
- [ ] Tracks 37–38 (Tracción 4x4/4x2)
- [ ] Tracks 39–44 (ABS, TCS, regenerativo)
- [ ] Tracks 45–48 (WiFi, OTA — reservado)
- [ ] Tracks 49–51 (Bluetooth — reservado)
- [ ] Tracks 52–56 (Estados del vehículo)
- [ ] Tracks 57–60 (Telemetría)
- [ ] Tracks 61–63 (Modos: Eco, Normal, Sport)
- [ ] Tracks 64–68 (Configuración y beep)
- [ ] Formatear tarjeta SD en FAT32
- [ ] Copiar todos los MP3 a la raíz de la SD
- [ ] Insertar SD en DFPlayer Mini
- [ ] Probar reproducción encendiendo el coche

---

*Documento generado: 2026-02-23*  
*Constantes definidas en: `esp32/src/audio_manager.h` (enum `audio::Sound`)*  
*Eventos de audio implementados en: `esp32/src/main.cpp`*
