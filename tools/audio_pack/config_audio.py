"""
Configuración centralizada del OEM Audio Pack — MarcosDashboard v10 Final.

Todos los parámetros (proveedor TTS, voz, audio engineering, frases por pista,
dummies, beep) se gestionan desde este único archivo.

NO modifica firmware. Solo afecta a la generación de los MP3 que se copian a la
SD del DFPlayer Mini conectado al ESP32-S3.

Las APIs (ElevenLabs, OpenAI) NO se invocan desde aquí; este archivo solo
expone configuración leída por ``generar_pack_oem.py``.
"""

from __future__ import annotations

import os
from typing import Dict, Optional


# ─────────────────────────────────────────────────────────────────────────────
# 1. PROVEEDOR TTS
# ─────────────────────────────────────────────────────────────────────────────
# Prioridad: elevenlabs (premium) > openai > espeak (offline local) > gtts.
# Cambia aquí o vía variable de entorno TTS_PROVIDER.
TTS_PROVIDER: str = os.environ.get("TTS_PROVIDER", "elevenlabs").lower()

# API keys (preferentemente vía entorno; nunca commitear claves reales).
ELEVENLABS_API_KEY: Optional[str] = os.environ.get("ELEVENLABS_API_KEY")
OPENAI_API_KEY: Optional[str] = os.environ.get("OPENAI_API_KEY")


# ─────────────────────────────────────────────────────────────────────────────
# 2. ESTILO DE VOZ
# ─────────────────────────────────────────────────────────────────────────────
# Voz masculina medio-grave, calmada, premium, tipo Tesla/Mercedes.
# Velocidad ligeramente lenta (0.95), pronunciación clara, energía moderada.

VOICE_LANGUAGE: str = "es-ES"

# ElevenLabs: "Adam" / "Antoni" / "Daniel" son masculinos calmados.
# Sustituir por el voice_id real de la cuenta. Por defecto: Daniel (es-ES amigable).
ELEVENLABS_VOICE_ID: str = os.environ.get(
    "ELEVENLABS_VOICE_ID", "onwK4e9ZLuTAKqWW03F9"  # Daniel
)
ELEVENLABS_MODEL: str = "eleven_multilingual_v2"
ELEVENLABS_STABILITY: float = 0.55
ELEVENLABS_SIMILARITY: float = 0.80
ELEVENLABS_STYLE: float = 0.25  # baja exageración emocional
ELEVENLABS_SPEAKER_BOOST: bool = True

# OpenAI TTS: voces "onyx" (grave) / "echo" (medio) — masculinas calmadas.
OPENAI_TTS_MODEL: str = "tts-1-hd"
OPENAI_VOICE: str = "onyx"
OPENAI_SPEED: float = 0.95  # ligeramente lenta

# gTTS: español de España, velocidad normal (slow=False para naturalidad).
GTTS_LANG: str = "es"
GTTS_TLD: str = "es"
GTTS_SLOW: bool = False

# eSpeak NG: fallback totalmente local, sin API ni red.
ESPEAK_VOICE: str = os.environ.get("ESPEAK_VOICE", "es")
ESPEAK_SPEED_WPM: int = int(os.environ.get("ESPEAK_SPEED_WPM", "145"))
ESPEAK_PITCH: int = int(os.environ.get("ESPEAK_PITCH", "45"))
ESPEAK_AMPLITUDE: int = int(os.environ.get("ESPEAK_AMPLITUDE", "170"))


# ─────────────────────────────────────────────────────────────────────────────
# 3. AUDIO ENGINEERING (FFmpeg)
# ─────────────────────────────────────────────────────────────────────────────
SAMPLE_RATE_HZ: int = 44100
CHANNELS: int = 1               # mono obligatorio para DFPlayer
MP3_BITRATE: str = "128k"       # CBR 128 kbps
MP3_BITRATE_MODE: str = "cbr"

# Filtrado para timbre OEM:
HIGHPASS_HZ: int = 80
LOWPASS_HZ: int = 10000

# Loudness target (broadcast/automotive friendly).
LOUDNORM_I: float = -16.0       # LUFS
LOUDNORM_LRA: float = 7.0       # rango dinámico
LOUDNORM_TP: float = -1.0       # true peak dBTP

# Silencios artificiales para evitar "click" del DFPlayer.
PAD_START_MS: int = 40
PAD_END_MS: int = 100

# Validación.
MAX_DURATION_S: float = 4.5


# ─────────────────────────────────────────────────────────────────────────────
# 4. BEEP 0068
# ─────────────────────────────────────────────────────────────────────────────
BEEP_FREQ_HZ: int = 1000
BEEP_DURATION_MS: int = 200
BEEP_FADE_MS: int = 10


# ─────────────────────────────────────────────────────────────────────────────
# 5. DUMMY (silencio para tracks reservados)
# ─────────────────────────────────────────────────────────────────────────────
DUMMY_DURATION_MS: int = 100


# ─────────────────────────────────────────────────────────────────────────────
# 6. ESTRUCTURA DE SALIDA
# ─────────────────────────────────────────────────────────────────────────────
OUTPUT_DIR: str = "output_sd"
CACHE_DIR: str = ".cache_tts"   # TTS crudo antes de procesar (re-uso entre runs)


# ─────────────────────────────────────────────────────────────────────────────
# 7. CATÁLOGO DE PISTAS (1..68)
# ─────────────────────────────────────────────────────────────────────────────
# Debe coincidir 1:1 con esp32/src/audio_manager.h enum Sound.
#
# kind:
#   "tts"   → frase TTS, se procesa con loudnorm.
#   "beep"  → sine 1 kHz generado por FFmpeg.
#   "dummy" → silencio (reserva indexado físico DFPlayer).
#
# text: frase OEM definitiva (estilo Tesla/Mercedes, breve, premium).

TRACKS: Dict[int, Dict[str, str]] = {
    # Sistema principal (1-3)
    1:  {"kind": "tts",   "name": "WELCOME",              "text": "Bienvenido, Marcos. Sistema listo."},
    2:  {"kind": "tts",   "name": "FAREWELL",             "text": "Cerrando sistemas. Hasta pronto."},
    3:  {"kind": "tts",   "name": "ERROR_GENERAL",        "text": "Atención. Error general detectado."},

    # Calibración pedal (4-5)
    4:  {"kind": "tts",   "name": "PEDAL_OK",             "text": "Pedal calibrado."},
    5:  {"kind": "tts",   "name": "PEDAL_ERROR",          "text": "Error en sensor de pedal."},

    # INA226 corriente (6-7)
    6:  {"kind": "tts",   "name": "INA_OK",               "text": "Sensores de corriente listos."},
    7:  {"kind": "tts",   "name": "INA_ERROR",            "text": "Error en sensores de corriente."},

    # Encoder dirección (8-9)
    8:  {"kind": "tts",   "name": "ENCODER_OK",           "text": "Dirección sincronizada."},
    9:  {"kind": "tts",   "name": "ENCODER_ERROR",        "text": "Error en sensor de dirección."},

    # Temperatura (10-11)
    10: {"kind": "tts",   "name": "TEMP_HIGH",            "text": "Temperatura del motor elevada."},
    11: {"kind": "tts",   "name": "TEMP_NORMAL",          "text": "Temperatura normalizada."},

    # Batería (12-13)
    12: {"kind": "tts",   "name": "BATTERY_LOW",          "text": "Batería baja."},
    13: {"kind": "tts",   "name": "BATTERY_CRITICAL",     "text": "Batería crítica. Desconectando tracción."},

    # Freno estacionamiento (14-15)
    14: {"kind": "tts",   "name": "PARKING_BRAKE_ON",     "text": "Freno de estacionamiento activado."},
    15: {"kind": "tts",   "name": "PARKING_BRAKE_OFF",    "text": "Freno de estacionamiento desactivado."},

    # Luces (16-17)
    16: {"kind": "tts",   "name": "LIGHTS_ON",            "text": "Luces encendidas."},
    17: {"kind": "tts",   "name": "LIGHTS_OFF",           "text": "Luces apagadas."},

    # Multimedia (18-19)
    18: {"kind": "tts",   "name": "RADIO_ON",             "text": "Sistema multimedia activado."},
    19: {"kind": "tts",   "name": "RADIO_OFF",            "text": "Sistema multimedia desactivado."},

    # Marchas (20-24)
    20: {"kind": "tts",   "name": "GEAR_D1",              "text": "Marcha D uno."},
    21: {"kind": "tts",   "name": "GEAR_D2",              "text": "Marcha D dos."},
    22: {"kind": "tts",   "name": "GEAR_REVERSE",         "text": "Marcha atrás."},
    23: {"kind": "tts",   "name": "GEAR_NEUTRAL",         "text": "Punto muerto."},
    24: {"kind": "tts",   "name": "GEAR_PARK",            "text": "Vehículo estacionado."},

    # Menú oculto / calibración (25-28)
    25: {"kind": "tts",   "name": "MENU_HIDDEN",          "text": "Menú avanzado activado."},
    26: {"kind": "tts",   "name": "CAL_PEDAL",            "text": "Calibrando pedal."},
    27: {"kind": "tts",   "name": "CAL_INA",              "text": "Calibrando sensores de corriente."},
    28: {"kind": "tts",   "name": "CAL_ENCODER",          "text": "Calibrando centro del volante."},

    # Test sistema (29-30)
    29: {"kind": "tts",   "name": "TEST_SYSTEM",          "text": "Iniciando comprobación del sistema."},
    30: {"kind": "tts",   "name": "TEST_OK",              "text": "Comprobación finalizada. Todo operativo."},

    # Emergencia / seguridad (31-32)
    31: {"kind": "tts",   "name": "EMERGENCY",            "text": "Emergencia. Motor deshabilitado."},
    32: {"kind": "tts",   "name": "SAFETY_RESET",         "text": "Reinicio de seguridad completado."},

    # Errores sensores específicos (33-35)
    33: {"kind": "tts",   "name": "SENSOR_TEMP_ERROR",    "text": "Error en sensor de temperatura."},
    34: {"kind": "tts",   "name": "SENSOR_CURRENT_ERROR", "text": "Anomalía en lectura de corriente."},
    35: {"kind": "tts",   "name": "SENSOR_SPEED_ERROR",   "text": "Sin señal de velocidad."},

    # Módulo OK (36) — usado como confirmación motivadora
    36: {"kind": "tts",   "name": "MODULE_OK",            "text": "Módulo verificado. Buen trabajo."},

    # Tracción (37-38)
    37: {"kind": "tts",   "name": "TRACTION_4X4",         "text": "Tracción cuatro por cuatro activada."},
    38: {"kind": "tts",   "name": "TRACTION_4X2",         "text": "Tracción cuatro por dos activada."},

    # Seguridad avanzada (39-44)
    39: {"kind": "tts",   "name": "ABS_ON",               "text": "Antibloqueo de frenos activado."},
    40: {"kind": "tts",   "name": "ABS_OFF",              "text": "Antibloqueo de frenos desactivado."},
    41: {"kind": "tts",   "name": "TCS_ON",               "text": "Control de tracción activado."},
    42: {"kind": "tts",   "name": "TCS_OFF",              "text": "Control de tracción desactivado."},
    43: {"kind": "tts",   "name": "REGEN_ON",             "text": "Frenado regenerativo activado."},
    44: {"kind": "tts",   "name": "REGEN_OFF",            "text": "Frenado regenerativo desactivado."},

    # WiFi (45-48) — RESERVED
    45: {"kind": "dummy", "name": "WIFI_CONNECTED",       "text": ""},
    46: {"kind": "dummy", "name": "WIFI_DISCONNECTED",    "text": ""},
    47: {"kind": "dummy", "name": "OTA_STARTED",          "text": ""},
    48: {"kind": "dummy", "name": "OTA_COMPLETED",        "text": ""},

    # Bluetooth (49-51) — RESERVED
    49: {"kind": "dummy", "name": "BT_CONNECTED",         "text": ""},
    50: {"kind": "dummy", "name": "BT_DISCONNECTED",      "text": ""},
    51: {"kind": "dummy", "name": "BT_PAIRING",           "text": ""},

    # Estado vehículo (52-56)
    52: {"kind": "tts",   "name": "MAX_SPEED",            "text": "Velocidad máxima alcanzada."},
    53: {"kind": "tts",   "name": "OVERCURRENT",          "text": "Corriente excesiva detectada."},
    54: {"kind": "tts",   "name": "OBSTACLE_WARN",        "text": "Atención. Obstáculo detectado."},
    55: {"kind": "tts",   "name": "PARKING_ASSIST",       "text": "Asistencia de estacionamiento activada."},
    56: {"kind": "tts",   "name": "SOFT_START",           "text": "Arranque suave en curso."},

    # Telemetría (57-60) — con toque motivacional ocasional
    57: {"kind": "tts",   "name": "BATTERY_50",           "text": "Batería al cincuenta por ciento."},
    58: {"kind": "tts",   "name": "BATTERY_25",           "text": "Batería al veinticinco por ciento."},
    59: {"kind": "tts",   "name": "DISTANCE_1KM",         "text": "Un kilómetro recorrido. Buen ritmo, Marcos."},
    60: {"kind": "tts",   "name": "ENERGY_SAVE",          "text": "Modo ahorro de energía."},

    # Modos conducción (61-63)
    61: {"kind": "tts",   "name": "MODE_ECO",             "text": "Modo eco."},
    62: {"kind": "tts",   "name": "MODE_NORMAL",          "text": "Modo normal."},
    63: {"kind": "tts",   "name": "MODE_SPORT",           "text": "Modo deportivo."},

    # Configuración (64-67)
    64: {"kind": "tts",   "name": "CONFIG_SAVED",         "text": "Configuración guardada."},
    65: {"kind": "tts",   "name": "CONFIG_RESTORED",      "text": "Configuración de fábrica restaurada."},
    66: {"kind": "tts",   "name": "ERRORS_CLEARED",       "text": "Registro de errores borrado."},
    67: {"kind": "tts",   "name": "REGEN_ADJUSTED",       "text": "Nivel de regeneración ajustado."},

    # Beep confirmación (68)
    68: {"kind": "beep",  "name": "BEEP",                 "text": ""},
}


# ─────────────────────────────────────────────────────────────────────────────
# 8. FRASES MOTIVACIONALES OEM (banco de referencia)
# ─────────────────────────────────────────────────────────────────────────────
# No se mapean automáticamente a pistas; son la fuente de inspiración para
# reemplazar puntualmente alguna frase corta (p.ej. DISTANCE_1KM, MODULE_OK)
# manteniendo el catálogo de 68 pistas estable.
MOTIVATIONAL_PHRASES = (
    "Excelente conducción, Marcos.",
    "Todo funcionando correctamente.",
    "Buen trabajo, piloto.",
    "Conducción suave y estable.",
    "Ruta perfecta.",
    "Muy buena maniobra.",
    "Todo bajo control.",
    "Sistema estable.",
    "Buen giro.",
    "Conducción eficiente.",
)


def expected_track_count() -> int:
    """Devuelve el número total de pistas esperadas en la SD (68)."""
    return len(TRACKS)
