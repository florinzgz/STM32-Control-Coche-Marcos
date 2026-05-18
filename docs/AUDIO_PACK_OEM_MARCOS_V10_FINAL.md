# PACK DEFINITIVO DE AUDIO OEM DIVERTIDO — MARCOSDASHBOARD V10 FINAL

## 1) PISTAS ACTIVAS EN FIRMWARE (confirmadas)

0001.mp3 — "Sistema listo."
0002.mp3 — "Apagando sistema."
0003.mp3 — "Atención. Error general."
0009.mp3 — "Error en sensor de dirección."
0010.mp3 — "Temperatura motor elevada."
0011.mp3 — "Temperatura normal."
0012.mp3 — "Batería baja."
0013.mp3 — "Batería crítica."
0016.mp3 — "Luces encendidas."
0017.mp3 — "Luces apagadas."
0020.mp3 — "Marcha D."
0021.mp3 — "Marcha D dos."
0022.mp3 — "Marcha atrás."
0023.mp3 — "Punto muerto."
0024.mp3 — "Estacionamiento."
0029.mp3 — "Mantenimiento pendiente."
0031.mp3 — "Modo emergencia. Motor desactivado."
0032.mp3 — "Sistema restablecido."
0033.mp3 — "Error de temperatura."
0034.mp3 — "Error de corriente."
0035.mp3 — "Sin señal de velocidad."
0037.mp3 — "Tracción cuatro por cuatro."
0038.mp3 — "Tracción cuatro por dos."
0039.mp3 — "Antibloqueo activado."
0040.mp3 — "Antibloqueo desactivado."
0041.mp3 — "Control de tracción activado."
0042.mp3 — "Control de tracción desactivado."
0053.mp3 — "Sobrecorriente detectada."
0054.mp3 — "Obstáculo cercano."
0068.mp3 — "Beep corto de confirmación."

## 2) PISTAS EXTRA FUTURAS (opcionales, no activas hoy en flujo principal)

0043.mp3 — "Modo aventura activado."
0055.mp3 — "Asistencia de aparcamiento."
0056.mp3 — "Arranque suave."
0057.mp3 — "Batería al cien por cien."
0058.mp3 — "Batería al cincuenta por ciento."
0059.mp3 — "Un kilómetro recorrido."
0060.mp3 — "Modo ahorro de energía."
0061.mp3 — "Modo eco."
0062.mp3 — "Modo normal."
0063.mp3 — "Modo sport."
0064.mp3 — "Configuración guardada."
0065.mp3 — "Configuración restaurada."
0066.mp3 — "Errores borrados."
0067.mp3 — "Sigue así, Marcos."

## 3) CON QUÉ PROGRAMA GRABARLAS

Programa recomendado (rápido): **TTSMaker** (voz masculina ES, tono calmado).
Programa recomendado (edición): **Audacity** (normalizar y exportar final).

Alternativa premium: **ElevenLabs** + Audacity.

## 4) CÓMO GRABARLAS (paso a paso)

1. Genera cada frase en TTSMaker (Español España, voz masculina medio-grave).
2. Descarga MP3 de cada frase.
3. Abre en Audacity y ajusta:
   - Duración objetivo: 1–2 s por frase
   - Volumen uniforme (normalizar a -16 LUFS aprox.)
   - Pico máximo: -1 dB
4. Exporta como MP3:
   - 44.1 kHz, mono, 128 kbps CBR
   - Sin metadatos extra
5. Nombra exactamente `0001.mp3`, `0002.mp3`, etc.
6. Copia todos los MP3 a la **raíz** de la SD (FAT32).

## 5) EJEMPLOS DE GRABACIÓN

Ejemplo A:
- Archivo: `0001.mp3`
- Texto: "Sistema listo."
- Estilo: serio, amable, tecnológico
- Duración: ~1.0 s

Ejemplo B:
- Archivo: `0031.mp3`
- Texto: "Modo emergencia. Motor desactivado."
- Estilo: técnico y firme
- Duración: ~1.4 s

Ejemplo C:
- Archivo: `0067.mp3` (extra opcional)
- Texto: "Sigue así, Marcos."
- Estilo: motivador suave, no caricaturesco
- Duración: ~1.2 s

## 6) CONFIRMACIÓN DE QUE ESTÁN ACTIVAS EN EL FIRMWARE

Confirmación realizada contra:
- `esp32/src/audio_manager.h` (enum `audio::Sound`, tracks 1..68)
- `esp32/src/main.cpp` (llamadas reales a `audio::play(...)`)

IDs de la lista activa confirmados en ejecución/eventos del firmware:
0001, 0002, 0003, 0009, 0010, 0011, 0012, 0013, 0016, 0017, 0020, 0021, 0022, 0023, 0024, 0029, 0031, 0032, 0033, 0034, 0035, 0037, 0038, 0039, 0040, 0041, 0042, 0053, 0054, 0068.
