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

**Opción A — TTSMaker (RECOMENDADO, gratis):**

1. Ir a **[ttsmaker.com](https://ttsmaker.com/)**
2. Seleccionar idioma: **Español (España)**
3. Copiar el texto de la columna "Texto Sugerido" de las tablas de abajo
4. Clic en **"Start to Convert"**
5. Descargar el MP3
6. Renombrar el archivo a `XXXX.mp3` (ejemplo: `0001.mp3`, `0039.mp3`)

**Opción B — Script Python con gTTS (automatizado):**

```bash
pip install gTTS
python3 generar_audios.py
```

El script Python completo se encuentra al final de este documento.

**Opción C — Grabación con micrófono:**

- Usar micrófono de buena calidad
- Grabar en ambiente silencioso
- Exportar a MP3: **mono, 128 kbps, 22050 Hz**
- Normalizar volumen entre archivos

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

## 📋 Lista Completa de Audios (68 Tracks)

### Sistema Principal (Tracks 1-3)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0001 | `0001.mp3` | `WELCOME` | "Bienvenido Marcos. El sistema está listo para comenzar." |
| 0002 | `0002.mp3` | `FAREWELL` | "Cerrando sistemas. Hasta pronto." |
| 0003 | `0003.mp3` | `ERROR_GENERAL` | "Atención. Se ha detectado un error general." |

### Calibración de Pedal (Tracks 4-5)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0004 | `0004.mp3` | `PEDAL_OK` | "Calibración del pedal completada correctamente." |
| 0005 | `0005.mp3` | `PEDAL_ERROR` | "Error en el sensor del pedal. Revise la conexión." |

### Sensores de Corriente (Tracks 6-7)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0006 | `0006.mp3` | `INA_OK` | "Calibración de sensores de corriente finalizada." |
| 0007 | `0007.mp3` | `INA_ERROR` | "Error en sensores de corriente o shunt desconectado." |

### Encoder de Dirección (Tracks 8-9)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0008 | `0008.mp3` | `ENCODER_OK` | "Encoder sincronizado correctamente." |
| 0009 | `0009.mp3` | `ENCODER_ERROR` | "Error en el sensor de dirección. Compruebe el encoder." |

### Temperatura (Tracks 10-11)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0010 | `0010.mp3` | `TEMP_HIGH` | "Temperatura del motor elevada. Reduzca la velocidad." |
| 0011 | `0011.mp3` | `TEMP_NORMAL` | "Temperatura del motor normalizada." |

### Batería (Tracks 12-13)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0012 | `0012.mp3` | `BATTERY_LOW` | "Nivel de batería bajo. Conecte el cargador, por favor." |
| 0013 | `0013.mp3` | `BATTERY_CRITICAL` | "Advertencia. Batería en nivel crítico. Desconectando tracción." |

### Freno de Estacionamiento (Tracks 14-15)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0014 | `0014.mp3` | `PARKING_BRAKE_ON` | "Freno de estacionamiento activado." |
| 0015 | `0015.mp3` | `PARKING_BRAKE_OFF` | "Freno de estacionamiento desactivado." |

### Luces (Tracks 16-17)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0016 | `0016.mp3` | `LIGHTS_ON` | "Luces encendidas." |
| 0017 | `0017.mp3` | `LIGHTS_OFF` | "Luces apagadas." |

### Radio/Multimedia (Tracks 18-19)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0018 | `0018.mp3` | `RADIO_ON` | "Sistema multimedia activado." |
| 0019 | `0019.mp3` | `RADIO_OFF` | "Sistema multimedia desactivado." |

### Marchas (Tracks 20-24)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0020 | `0020.mp3` | `GEAR_D1` | "Marcha D uno activada." |
| 0021 | `0021.mp3` | `GEAR_D2` | "Marcha D dos activada." |
| 0022 | `0022.mp3` | `GEAR_REVERSE` | "Marcha atrás activada." |
| 0023 | `0023.mp3` | `GEAR_NEUTRAL` | "Punto muerto." |
| 0024 | `0024.mp3` | `GEAR_PARK` | "Vehículo en posición de estacionamiento." |

### Menú Oculto y Calibración (Tracks 25-28)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0025 | `0025.mp3` | `MENU_HIDDEN` | "Menú de calibración avanzado activado." |
| 0026 | `0026.mp3` | `CAL_PEDAL` | "Iniciando calibración del pedal. Presione lentamente hasta el fondo." |
| 0027 | `0027.mp3` | `CAL_INA` | "Calibrando sensores de corriente. Espere unos segundos." |
| 0028 | `0028.mp3` | `CAL_ENCODER` | "Calibrando el punto central del volante. Manténgalo recto." |

### Test del Sistema (Tracks 29-30)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0029 | `0029.mp3` | `TEST_SYSTEM` | "Iniciando comprobación completa del sistema." |
| 0030 | `0030.mp3` | `TEST_OK` | "Comprobación finalizada. Todos los módulos operativos." |

### Emergencia y Seguridad (Tracks 31-32)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0031 | `0031.mp3` | `EMERGENCY` | "Modo de emergencia activado. Motor deshabilitado." |
| 0032 | `0032.mp3` | `SAFETY_RESET` | "Reinicio de seguridad completado." |

### Errores de Sensores Específicos (Tracks 33-35)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0033 | `0033.mp3` | `SENSOR_TEMP_ERROR` | "Error en sensor de temperatura." |
| 0034 | `0034.mp3` | `SENSOR_CURRENT_ERROR` | "Anomalía en lectura de corriente." |
| 0035 | `0035.mp3` | `SENSOR_SPEED_ERROR` | "Sin señal de velocidad. Revise sensores de rueda." |

### Estado de Módulos (Track 36)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0036 | `0036.mp3` | `MODULE_OK` | "Módulo verificado correctamente." |

### Tracción 4x4/4x2 (Tracks 37-38)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0037 | `0037.mp3` | `TRACTION_4X4` | "Tracción 4x4 inteligente activada." |
| 0038 | `0038.mp3` | `TRACTION_4X2` | "Tracción 4x2 inteligente activada." |

### Sistemas de Seguridad Avanzados (Tracks 39-44)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0039 | `0039.mp3` | `ABS_ON` | "Sistema antibloqueo de frenos activado." |
| 0040 | `0040.mp3` | `ABS_OFF` | "Sistema antibloqueo de frenos desactivado." |
| 0041 | `0041.mp3` | `TCS_ON` | "Control de tracción activado." |
| 0042 | `0042.mp3` | `TCS_OFF` | "Control de tracción desactivado." |
| 0043 | `0043.mp3` | `REGEN_ON` | "Frenado regenerativo activado." |
| 0044 | `0044.mp3` | `REGEN_OFF` | "Frenado regenerativo desactivado." |

### WiFi y Conectividad (Tracks 45-48) — Reservado

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0045 | `0045.mp3` | `WIFI_CONNECTED` | "Conexión WiFi establecida." |
| 0046 | `0046.mp3` | `WIFI_DISCONNECTED` | "Conexión WiFi perdida." |
| 0047 | `0047.mp3` | `OTA_STARTED` | "Actualización remota iniciada. No desconecte el vehículo." |
| 0048 | `0048.mp3` | `OTA_COMPLETED` | "Actualización completada. Reiniciando sistema." |

### Bluetooth (Tracks 49-51) — Reservado

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0049 | `0049.mp3` | `BT_CONNECTED` | "Mando Bluetooth conectado." |
| 0050 | `0050.mp3` | `BT_DISCONNECTED` | "Mando Bluetooth desconectado." |
| 0051 | `0051.mp3` | `BT_PAIRING` | "Buscando mando Bluetooth. Mantenga pulsado el botón de emparejamiento." |

### Estados del Vehículo (Tracks 52-56)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0052 | `0052.mp3` | `MAX_SPEED` | "Velocidad máxima alcanzada." |
| 0053 | `0053.mp3` | `OVERCURRENT` | "Advertencia. Corriente excesiva detectada." |
| 0054 | `0054.mp3` | `OBSTACLE_WARN` | "Atención. Obstáculo detectado." |
| 0055 | `0055.mp3` | `PARKING_ASSIST` | "Modo asistencia de estacionamiento activado." |
| 0056 | `0056.mp3` | `SOFT_START` | "Iniciando arranque suave de motores." |

### Información de Telemetría (Tracks 57-60)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0057 | `0057.mp3` | `BATTERY_50` | "Nivel de batería al 50 por ciento." |
| 0058 | `0058.mp3` | `BATTERY_25` | "Nivel de batería al 25 por ciento. Considere recargar." |
| 0059 | `0059.mp3` | `DISTANCE_1KM` | "Ha recorrido un kilómetro en esta sesión." |
| 0060 | `0060.mp3` | `ENERGY_SAVE` | "Modo ahorro de energía activado." |

### Modos de Conducción (Tracks 61-63)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0061 | `0061.mp3` | `MODE_ECO` | "Modo eco activado. Máxima eficiencia." |
| 0062 | `0062.mp3` | `MODE_NORMAL` | "Modo normal activado." |
| 0063 | `0063.mp3` | `MODE_SPORT` | "Modo deportivo activado. Máxima potencia." |

### Feedback de Configuración (Tracks 64-68)

| Track | Archivo | Constante Firmware | Texto Sugerido |
|-------|---------|-------------------|----------------|
| 0064 | `0064.mp3` | `CONFIG_SAVED` | "Configuración guardada correctamente." |
| 0065 | `0065.mp3` | `CONFIG_RESTORED` | "Configuración de fábrica restaurada." |
| 0066 | `0066.mp3` | `ERRORS_CLEARED` | "Registro de errores borrado." |
| 0067 | `0067.mp3` | `REGEN_ADJUSTED` | "Nivel de regeneración ajustado." |
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
Si más de 1 error/aviso se produce en una ventana de 2 segundos, el sistema reproduce automáticamente `0003.mp3` (ERROR_GENERAL: "Atención. Se ha detectado un error general.") en lugar de apilar avisos individuales. Esto evita un bombardeo de mensajes consecutivos que serían confusos.

---

## 🐍 Script Python para Generar Todos los Audios

```python
from gtts import gTTS

textos = {
    "0001": "Bienvenido Marcos. El sistema está listo para comenzar.",
    "0002": "Cerrando sistemas. Hasta pronto.",
    "0003": "Atención. Se ha detectado un error general.",
    "0004": "Calibración del pedal completada correctamente.",
    "0005": "Error en el sensor del pedal. Revise la conexión.",
    "0006": "Calibración de sensores de corriente finalizada.",
    "0007": "Error en sensores de corriente o shunt desconectado.",
    "0008": "Encoder sincronizado correctamente.",
    "0009": "Error en el sensor de dirección. Compruebe el encoder.",
    "0010": "Temperatura del motor elevada. Reduzca la velocidad.",
    "0011": "Temperatura del motor normalizada.",
    "0012": "Nivel de batería bajo. Conecte el cargador, por favor.",
    "0013": "Advertencia. Batería en nivel crítico. Desconectando tracción.",
    "0014": "Freno de estacionamiento activado.",
    "0015": "Freno de estacionamiento desactivado.",
    "0016": "Luces encendidas.",
    "0017": "Luces apagadas.",
    "0018": "Sistema multimedia activado.",
    "0019": "Sistema multimedia desactivado.",
    "0020": "Marcha D uno activada.",
    "0021": "Marcha D dos activada.",
    "0022": "Marcha atrás activada.",
    "0023": "Punto muerto.",
    "0024": "Vehículo en posición de estacionamiento.",
    "0025": "Menú de calibración avanzado activado.",
    "0026": "Iniciando calibración del pedal. Presione lentamente hasta el fondo.",
    "0027": "Calibrando sensores de corriente. Espere unos segundos.",
    "0028": "Calibrando el punto central del volante. Manténgalo recto.",
    "0029": "Iniciando comprobación completa del sistema.",
    "0030": "Comprobación finalizada. Todos los módulos operativos.",
    "0031": "Modo de emergencia activado. Motor deshabilitado.",
    "0032": "Reinicio de seguridad completado.",
    "0033": "Error en sensor de temperatura.",
    "0034": "Anomalía en lectura de corriente.",
    "0035": "Sin señal de velocidad. Revise sensores de rueda.",
    "0036": "Módulo verificado correctamente.",
    "0037": "Tracción 4x4 inteligente activada.",
    "0038": "Tracción 4x2 inteligente activada.",
    "0039": "Sistema antibloqueo de frenos activado.",
    "0040": "Sistema antibloqueo de frenos desactivado.",
    "0041": "Control de tracción activado.",
    "0042": "Control de tracción desactivado.",
    "0043": "Frenado regenerativo activado.",
    "0044": "Frenado regenerativo desactivado.",
    "0045": "Conexión WiFi establecida.",
    "0046": "Conexión WiFi perdida.",
    "0047": "Actualización remota iniciada. No desconecte el vehículo.",
    "0048": "Actualización completada. Reiniciando sistema.",
    "0049": "Mando Bluetooth conectado.",
    "0050": "Mando Bluetooth desconectado.",
    "0051": "Buscando mando Bluetooth. Mantenga pulsado el botón de emparejamiento.",
    "0052": "Velocidad máxima alcanzada.",
    "0053": "Advertencia. Corriente excesiva detectada.",
    "0054": "Atención. Obstáculo detectado.",
    "0055": "Modo asistencia de estacionamiento activado.",
    "0056": "Iniciando arranque suave de motores.",
    "0057": "Nivel de batería al 50 por ciento.",
    "0058": "Nivel de batería al 25 por ciento. Considere recargar.",
    "0059": "Ha recorrido un kilómetro en esta sesión.",
    "0060": "Modo ahorro de energía activado.",
    "0061": "Modo eco activado. Máxima eficiencia.",
    "0062": "Modo normal activado.",
    "0063": "Modo deportivo activado. Máxima potencia.",
    "0064": "Configuración guardada correctamente.",
    "0065": "Configuración de fábrica restaurada.",
    "0066": "Registro de errores borrado.",
    "0067": "Nivel de regeneración ajustado.",
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
