# OEM Audio Pack — MarcosDashboard v10 Final

Pipeline **automático** de generación y preparación del banco de audio para el
DFPlayer Mini que cuelga del ESP32-S3 en MarcosDashboard v10 Final.

> ⚠️ **No toca firmware.** No modifica el enum `Sound`, ni la arquitectura, ni
> las prioridades HI/MEDIUM/LO, ni la lógica del DFPlayer. Es puramente
> *tooling* de contenido. El catálogo de 68 pistas refleja 1:1
> `esp32/src/audio_manager.h`.

---

## 1. Qué genera

```
tools/audio_pack/output_sd/
├── 0001.mp3   WELCOME            "Bienvenido, Marcos. Sistema listo."
├── 0002.mp3   FAREWELL           "Cerrando sistemas. Hasta pronto."
├── ...
├── 0044.mp3   REGEN_OFF          "Frenado regenerativo desactivado."
├── 0045.mp3   WIFI_CONNECTED     (dummy · 100 ms silencio)
├── ...
├── 0051.mp3   BT_PAIRING         (dummy · 100 ms silencio)
├── 0052.mp3   MAX_SPEED          "Velocidad máxima alcanzada."
├── ...
├── 0067.mp3   REGEN_ADJUSTED     "Nivel de regeneración ajustado."
└── 0068.mp3   BEEP               (sine 1 kHz · 200 ms · fade 10 ms)
```

68 archivos en total, todos en el **mismo formato OEM**:

- **mono, 44.1 kHz, MP3 CBR 128 kbps**
- high-pass 80 Hz · low-pass 10 kHz
- loudnorm `I=-16 LUFS · TP=-1 dB`
- 40 ms de silencio al inicio · 100 ms al final
- sin ID3v2, sin metadata
- duración ≤ 4.5 s

---

## 2. Estructura de carpetas

```
tools/audio_pack/
├── config_audio.py        # config centralizada (proveedor, voz, frases, pistas)
├── generar_pack_oem.py    # pipeline principal
├── copiar_a_sd.sh         # copia ordenada 0001→0068 a la SD
├── requirements.txt       # dependencias pip
├── README.md              # este archivo
├── output_sd/             # (generado) listo para copiar a la SD
└── .cache_tts/            # (generado) MP3 crudos TTS, se reutilizan entre runs
```

---

## 3. Dependencias

**Sistema (no pip):**

```bash
sudo apt update
sudo apt install -y ffmpeg          # incluye ffprobe
```

**Python:**

```bash
cd tools/audio_pack
python3 -m venv .venv && source .venv/bin/activate     # opcional
pip install -r requirements.txt
```

---

## 4. Configurar proveedor TTS

Edita `config_audio.py` o exporta por entorno. Prioridad recomendada:

| Prioridad | Proveedor      | Calidad        | API key                 |
|-----------|----------------|----------------|-------------------------|
| 1         | `elevenlabs`   | premium OEM    | `ELEVENLABS_API_KEY`    |
| 2         | `openai`       | muy buena      | `OPENAI_API_KEY`        |
| 3         | `gtts`         | fallback libre | — (acceso a Google)     |

```bash
export TTS_PROVIDER=elevenlabs
export ELEVENLABS_API_KEY=sk_live_xxxxxxxxxxxxxxxxxx
# (opcional) elegir un voice_id concreto del catálogo de tu cuenta
export ELEVENLABS_VOICE_ID=onwK4e9ZLuTAKqWW03F9   # Daniel (es)
```

Voz objetivo: **masculina, medio-grave, calmada, premium tipo Tesla/Mercedes**,
velocidad ligeramente lenta. Parámetros ya ajustados en `config_audio.py`
(`stability=0.55`, `style=0.25` para baja exageración emocional, etc.).

---

## 5. Generar el pack completo (1 comando)

```bash
cd tools/audio_pack
python3 generar_pack_oem.py
```

Salida típica:

```
[oem-audio] [0001] tts   WELCOME
[oem-audio] [0002] tts   FAREWELL
...
[oem-audio] [0068] beep  BEEP
[oem-audio] Salida: .../tools/audio_pack/output_sd
[oem-audio] Pistas procesadas: 68 / 68
[oem-audio] Validación OK · duración ≤ 4.5 s · 44.1 kHz mono · CBR 128 kbps.
```

### Opciones útiles

```bash
# Cambiar proveedor sin tocar config:
python3 generar_pack_oem.py --provider openai

# Regenerar solo unas pistas (rapidísimo, sin tocar el resto):
python3 generar_pack_oem.py --only 1 2 59 68

# Ignorar caché y volver a sintetizar TTS:
python3 generar_pack_oem.py --no-cache
```

---

## 6. Copiar a la SD (manteniendo indexado físico)

El DFPlayer Mini indexa por **orden de creación en FAT**, no por nombre. Si
copias todos los archivos en bloque puedes acabar con `0001.mp3` reproduciendo
otra pista. El script copia uno a uno con `sync`:

```bash
# Monta la SD (FAT32, p.ej. en /media/$USER/SD)
./copiar_a_sd.sh /media/$USER/SD
sync && udisksctl unmount -b /dev/sdX1     # o "eject"
```

Recomendaciones DFPlayer:

- SD **FAT32**, 4–32 GB. Evita FAT16 y exFAT.
- **Sin carpetas extra** ni archivos ocultos en la raíz.
- No reordenes archivos manualmente en el explorador.
- Si reemplazas pistas sueltas, copia siempre por orden numérico ascendente.

---

## 7. Cadena FFmpeg aplicada

### Pistas TTS (44 reales + frases motivacionales)

```bash
ffmpeg -y -i raw.mp3 \
  -af "highpass=f=80,\
       lowpass=f=10000,\
       loudnorm=I=-16:LRA=7:TP=-1,\
       adelay=40|40,\
       apad=pad_dur=0.1" \
  -ac 1 -ar 44100 \
  -c:a libmp3lame -b:a 128k \
  -write_xing 0 -id3v2_version 0 -map_metadata -1 \
  0001.mp3
```

### Beep 0068 (sine 1 kHz)

```bash
ffmpeg -y -f lavfi \
  -i "sine=frequency=1000:sample_rate=44100:duration=0.2" \
  -af "afade=t=in:st=0:d=0.01,\
       afade=t=out:st=0.19:d=0.01,\
       loudnorm=I=-16:LRA=7:TP=-1" \
  -ac 1 -ar 44100 \
  -c:a libmp3lame -b:a 128k \
  -write_xing 0 -id3v2_version 0 -map_metadata -1 \
  0068.mp3
```

### Dummies (reservados WiFi/BT 45–51)

```bash
ffmpeg -y -f lavfi -i "anullsrc=channel_layout=mono:sample_rate=44100" \
  -t 0.1 -ac 1 -ar 44100 \
  -c:a libmp3lame -b:a 128k \
  -write_xing 0 -id3v2_version 0 -map_metadata -1 \
  0045.mp3
```

---

## 8. Frases definitivas (resumen)

Las 68 frases viven en `config_audio.py → TRACKS`. Cumplen:

- estilo OEM premium (Tesla / Mercedes / BMW),
- breves (1–2 s típico, máximo 4.5 s),
- sin infantilismos, sin chistes, sin frases largas,
- sin repetición machacona,
- con **toques motivacionales puntuales** y nunca spam:
  - `0036 MODULE_OK`     → "Módulo verificado. Buen trabajo."
  - `0059 DISTANCE_1KM`  → "Un kilómetro recorrido. Buen ritmo, Marcos."
  - `0030 TEST_OK`       → "Comprobación finalizada. Todo operativo."

Banco adicional de motivacionales en
`config_audio.py → MOTIVATIONAL_PHRASES`, listo para sustituir manualmente la
frase de cualquier pista corta sin tocar el indexado.

---

## 9. Checklist de validación final

Tras `python3 generar_pack_oem.py` confirma que:

- [x] Existen exactamente **68** archivos `0001.mp3` … `0068.mp3`.
- [x] El script no imprime ningún `[WARN]`.
- [x] `ffprobe output_sd/0001.mp3` muestra: `mono`, `44100 Hz`, `128 kb/s`, `mp3`.
- [x] Ninguna pista supera **4.5 s** de duración.
- [x] No hay ID3v2 (`ffprobe` no lista `TAG: …`).
- [x] Pistas dummy (45–51) duran ~100 ms y son silencio.
- [x] `0068.mp3` es un beep limpio sin clicks (fade-in/out 10 ms).
- [x] En la SD, los nombres son `0001.mp3`, no `1.mp3` ni `0001 (copy).mp3`.

---

## 10. Recomendaciones OEM finales

1. **Volumen final del DFPlayer**: ajusta en firmware en `0..30`; con el
   loudnorm a -16 LUFS, un volumen de 18–22 suele dar resultado natural a
   través del amplificador del coche.
2. **Anti-fatiga**: motivacionales ≤ 1 cada varios minutos. El catálogo está
   pensado para que la mayoría de pistas sean informativas, no celebradoras.
3. **Iteración rápida**: cambia una frase en `config_audio.py`, borra solo
   ese fichero de `.cache_tts/` y vuelve a ejecutar con `--only N`.
4. **Sustitución por escena**: si quieres reescribir un mensaje (p.ej. cambiar
   `0036 MODULE_OK` por una motivacional distinta), edita solo el `text` de
   esa pista — no toques `kind`, `name` ni el índice.
5. **No renumerar nunca.** El firmware ya tiene los índices fijos en
   `enum class Sound`. Cambiar el orden rompe la coherencia ESP32 ↔ SD.
6. **Backup**: guarda `output_sd/` versionado fuera del repo (por tamaño) o
   en un release artifact; los `.cache_tts/` se pueden borrar sin perder
   nada definitivo.

---

## 11. Resolución rápida de problemas

| Síntoma                                       | Solución                                              |
|-----------------------------------------------|-------------------------------------------------------|
| `Falta la herramienta 'ffmpeg'`               | `sudo apt install ffmpeg`                             |
| `ELEVENLABS_API_KEY no definida`              | `export ELEVENLABS_API_KEY=...` o usa `--provider gtts` |
| El DFPlayer reproduce la pista equivocada     | Reformatea SD a FAT32 limpio y usa `copiar_a_sd.sh`   |
| Una pista suena más alta o más baja           | Borra su raw en `.cache_tts/` y regenera con `--only` |
| `duración X.XXs > 4.5s` en validación         | Acorta la frase en `config_audio.py → TRACKS[N]`      |
