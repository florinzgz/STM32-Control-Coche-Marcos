#!/usr/bin/env python3
"""
generar_pack_oem.py
===================

Pipeline automático de generación del OEM Audio Pack para MarcosDashboard v10
Final (ESP32-S3 + DFPlayer Mini).

Hace TODO sin pasos manuales:

  1. Lee ``config_audio.py`` (frases, proveedor TTS, parámetros de audio).
  2. Genera TTS por pista (ElevenLabs → OpenAI → gTTS según ``TTS_PROVIDER``).
  3. Genera beep 0068 (sine 1 kHz, 200 ms, fade) con FFmpeg.
  4. Genera dummies (silencio 100 ms) para todas las pistas reservadas.
  5. Procesa cada MP3:
        mono · 44.1 kHz · MP3 CBR 128 kbps
        high-pass 80 Hz · low-pass 10 kHz
        loudnorm I=-16 LUFS · TP=-1 dB
        pad 40 ms inicio / 100 ms final
        sin ID3v2 / sin metadata
  6. Valida duración (≤ 4.5 s), sample rate, mono y bitrate.
  7. Exporta a ``output_sd/0001.mp3 … 0068.mp3``.

Uso típico::

    cd tools/audio_pack
    pip install -r requirements.txt
    export ELEVENLABS_API_KEY=...           # opcional según proveedor
    python3 generar_pack_oem.py             # genera output_sd/
    ./copiar_a_sd.sh /media/usuario/SD      # copia ordenado a la SD

El script NO toca firmware. Es una herramienta puramente de tooling/contenido.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Optional

import config_audio as cfg


# ─────────────────────────────────────────────────────────────────────────────
# Utilidades
# ─────────────────────────────────────────────────────────────────────────────

class AudioPackError(RuntimeError):
    """Error específico del pipeline OEM."""


def _log(msg: str) -> None:
    print(f"[oem-audio] {msg}", flush=True)


def _warn(msg: str) -> None:
    print(f"[oem-audio][WARN] {msg}", file=sys.stderr, flush=True)


def _run(cmd: list[str]) -> None:
    """Ejecuta un proceso y lanza AudioPackError si falla."""
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise AudioPackError(
            f"Comando fallido: {' '.join(cmd)}\nSTDERR:\n{result.stderr}"
        )


def _require_tool(name: str) -> None:
    if shutil.which(name) is None:
        raise AudioPackError(
            f"Falta la herramienta '{name}'. Instálala antes de continuar."
        )


# ─────────────────────────────────────────────────────────────────────────────
# Generadores TTS
# ─────────────────────────────────────────────────────────────────────────────

def _tts_elevenlabs(text: str, out_mp3: Path) -> None:
    if not cfg.ELEVENLABS_API_KEY:
        raise AudioPackError("ELEVENLABS_API_KEY no definida en el entorno.")
    try:
        import requests  # type: ignore
    except ImportError as exc:
        raise AudioPackError("Falta dependencia 'requests' (pip install requests).") from exc

    url = (
        f"https://api.elevenlabs.io/v1/text-to-speech/"
        f"{cfg.ELEVENLABS_VOICE_ID}?output_format=mp3_44100_128"
    )
    headers = {
        "xi-api-key": cfg.ELEVENLABS_API_KEY,
        "Content-Type": "application/json",
        "Accept": "audio/mpeg",
    }
    payload = {
        "text": text,
        "model_id": cfg.ELEVENLABS_MODEL,
        "voice_settings": {
            "stability": cfg.ELEVENLABS_STABILITY,
            "similarity_boost": cfg.ELEVENLABS_SIMILARITY,
            "style": cfg.ELEVENLABS_STYLE,
            "use_speaker_boost": cfg.ELEVENLABS_SPEAKER_BOOST,
        },
    }
    resp = requests.post(url, headers=headers, data=json.dumps(payload), timeout=60)
    if resp.status_code != 200:
        raise AudioPackError(
            f"ElevenLabs {resp.status_code}: {resp.text[:300]}"
        )
    out_mp3.write_bytes(resp.content)


def _tts_openai(text: str, out_mp3: Path) -> None:
    if not cfg.OPENAI_API_KEY:
        raise AudioPackError("OPENAI_API_KEY no definida en el entorno.")
    try:
        import requests  # type: ignore
    except ImportError as exc:
        raise AudioPackError("Falta dependencia 'requests' (pip install requests).") from exc

    url = "https://api.openai.com/v1/audio/speech"
    headers = {
        "Authorization": f"Bearer {cfg.OPENAI_API_KEY}",
        "Content-Type": "application/json",
    }
    payload = {
        "model": cfg.OPENAI_TTS_MODEL,
        "voice": cfg.OPENAI_VOICE,
        "input": text,
        "response_format": "mp3",
        "speed": cfg.OPENAI_SPEED,
    }
    resp = requests.post(url, headers=headers, data=json.dumps(payload), timeout=60)
    if resp.status_code != 200:
        raise AudioPackError(f"OpenAI {resp.status_code}: {resp.text[:300]}")
    out_mp3.write_bytes(resp.content)


def _tts_gtts(text: str, out_mp3: Path) -> None:
    try:
        from gtts import gTTS  # type: ignore
    except ImportError as exc:
        raise AudioPackError("Falta dependencia 'gtts' (pip install gTTS).") from exc
    tts = gTTS(text=text, lang=cfg.GTTS_LANG, tld=cfg.GTTS_TLD, slow=cfg.GTTS_SLOW)
    tts.save(str(out_mp3))


def _tts_espeak(text: str, out_mp3: Path) -> None:
    _require_tool("espeak-ng")
    with tempfile.TemporaryDirectory() as tmp_dir:
        wav_path = Path(tmp_dir) / "espeak_raw.wav"
        _run([
            "espeak-ng",
            "-v", cfg.ESPEAK_VOICE,
            "-s", str(cfg.ESPEAK_SPEED_WPM),
            "-p", str(cfg.ESPEAK_PITCH),
            "-a", str(cfg.ESPEAK_AMPLITUDE),
            "-w", str(wav_path),
            text,
        ])
        _run([
            "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
            "-i", str(wav_path),
            "-ac", str(cfg.CHANNELS),
            "-ar", str(cfg.SAMPLE_RATE_HZ),
            "-c:a", "libmp3lame",
            "-b:a", cfg.MP3_BITRATE,
            "-write_xing", "0",
            "-id3v2_version", "0",
            "-map_metadata", "-1",
            str(out_mp3),
        ])


def _generate_tts(text: str, out_mp3: Path, provider: str) -> None:
    """Genera el MP3 TTS crudo (sin procesar). Cachea para no regenerar."""
    if out_mp3.exists() and out_mp3.stat().st_size > 0:
        return
    out_mp3.parent.mkdir(parents=True, exist_ok=True)
    if provider == "elevenlabs":
        _tts_elevenlabs(text, out_mp3)
    elif provider == "openai":
        _tts_openai(text, out_mp3)
    elif provider == "espeak":
        _tts_espeak(text, out_mp3)
    elif provider == "gtts":
        _tts_gtts(text, out_mp3)
    else:
        raise AudioPackError(f"TTS_PROVIDER desconocido: {provider}")


# ─────────────────────────────────────────────────────────────────────────────
# Procesado FFmpeg (cadena OEM)
# ─────────────────────────────────────────────────────────────────────────────

def _ffmpeg_filter_chain() -> str:
    """Filtro audio común: HP/LP, loudnorm, pad."""
    return (
        f"highpass=f={cfg.HIGHPASS_HZ},"
        f"lowpass=f={cfg.LOWPASS_HZ},"
        f"loudnorm=I={cfg.LOUDNORM_I}:LRA={cfg.LOUDNORM_LRA}:TP={cfg.LOUDNORM_TP},"
        f"adelay={cfg.PAD_START_MS}|{cfg.PAD_START_MS},"
        f"apad=pad_dur={cfg.PAD_END_MS / 1000.0}"
    )


def _process_to_oem(src: Path, dst: Path) -> None:
    """Convierte src → dst aplicando la cadena OEM completa (mono, 44.1k, CBR 128k)."""
    dst.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
        "-i", str(src),
        "-af", _ffmpeg_filter_chain(),
        "-ac", str(cfg.CHANNELS),
        "-ar", str(cfg.SAMPLE_RATE_HZ),
        "-c:a", "libmp3lame",
        "-b:a", cfg.MP3_BITRATE,
        "-write_xing", "0",
        "-id3v2_version", "0",
        "-map_metadata", "-1",
        str(dst),
    ]
    _run(cmd)


def _generate_beep(dst: Path) -> None:
    """Genera 0068 (sine 1 kHz, 200 ms, fade in/out)."""
    dst.parent.mkdir(parents=True, exist_ok=True)
    dur_s = cfg.BEEP_DURATION_MS / 1000.0
    fade_s = cfg.BEEP_FADE_MS / 1000.0
    fade_out_start = max(0.0, dur_s - fade_s)
    af = (
        f"afade=t=in:st=0:d={fade_s},"
        f"afade=t=out:st={fade_out_start}:d={fade_s},"
        f"loudnorm=I={cfg.LOUDNORM_I}:LRA={cfg.LOUDNORM_LRA}:TP={cfg.LOUDNORM_TP}"
    )
    cmd = [
        "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
        "-f", "lavfi",
        "-i", f"sine=frequency={cfg.BEEP_FREQ_HZ}:sample_rate={cfg.SAMPLE_RATE_HZ}:duration={dur_s}",
        "-af", af,
        "-ac", str(cfg.CHANNELS),
        "-ar", str(cfg.SAMPLE_RATE_HZ),
        "-c:a", "libmp3lame",
        "-b:a", cfg.MP3_BITRATE,
        "-write_xing", "0",
        "-id3v2_version", "0",
        "-map_metadata", "-1",
        str(dst),
    ]
    _run(cmd)


def _generate_dummy(dst: Path) -> None:
    """Genera silencio mono al formato OEM (preserva indexado del DFPlayer)."""
    dst.parent.mkdir(parents=True, exist_ok=True)
    dur_s = cfg.DUMMY_DURATION_MS / 1000.0
    cmd = [
        "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
        "-f", "lavfi",
        "-i", f"anullsrc=channel_layout=mono:sample_rate={cfg.SAMPLE_RATE_HZ}",
        "-t", f"{dur_s}",
        "-ac", str(cfg.CHANNELS),
        "-ar", str(cfg.SAMPLE_RATE_HZ),
        "-c:a", "libmp3lame",
        "-b:a", cfg.MP3_BITRATE,
        "-write_xing", "0",
        "-id3v2_version", "0",
        "-map_metadata", "-1",
        str(dst),
    ]
    _run(cmd)


# ─────────────────────────────────────────────────────────────────────────────
# Validación
# ─────────────────────────────────────────────────────────────────────────────

def _ffprobe_json(path: Path) -> dict:
    cmd = [
        "ffprobe", "-v", "error", "-print_format", "json",
        "-show_format", "-show_streams", str(path),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise AudioPackError(f"ffprobe falló sobre {path}: {result.stderr}")
    return json.loads(result.stdout)


def _validate(path: Path) -> list[str]:
    """Devuelve lista de warnings (vacía si todo OK)."""
    warnings: list[str] = []
    info = _ffprobe_json(path)
    streams = info.get("streams") or []
    if not streams:
        return [f"{path.name}: sin streams"]
    audio = next((s for s in streams if s.get("codec_type") == "audio"), None)
    if audio is None:
        return [f"{path.name}: sin stream de audio"]

    duration = float(info.get("format", {}).get("duration", 0.0) or 0.0)
    sample_rate = int(audio.get("sample_rate", 0))
    channels = int(audio.get("channels", 0))
    bit_rate = int(audio.get("bit_rate") or info.get("format", {}).get("bit_rate") or 0)

    if duration > cfg.MAX_DURATION_S:
        warnings.append(f"{path.name}: duración {duration:.2f}s > {cfg.MAX_DURATION_S}s")
    if sample_rate != cfg.SAMPLE_RATE_HZ:
        warnings.append(f"{path.name}: sample_rate {sample_rate} ≠ {cfg.SAMPLE_RATE_HZ}")
    if channels != cfg.CHANNELS:
        warnings.append(f"{path.name}: channels {channels} ≠ {cfg.CHANNELS}")
    # CBR 128 kbps → tolerar pequeñas desviaciones (±10 %).
    target = 128_000
    if bit_rate and abs(bit_rate - target) > target * 0.10:
        warnings.append(f"{path.name}: bitrate {bit_rate} fuera de ±10% de {target}")
    return warnings


# ─────────────────────────────────────────────────────────────────────────────
# Orquestador
# ─────────────────────────────────────────────────────────────────────────────

def _build_track(index: int, spec: dict, *, provider: str,
                 cache_dir: Path, out_dir: Path) -> list[str]:
    out_path = out_dir / f"{index:04d}.mp3"
    kind = spec["kind"]
    name = spec["name"]
    _log(f"[{index:04d}] {kind:<5} {name}")

    if kind == "beep":
        _generate_beep(out_path)
    elif kind == "dummy":
        _generate_dummy(out_path)
    elif kind == "tts":
        text = spec["text"]
        if not text:
            raise AudioPackError(f"Track {index} {name} marcado tts sin texto.")
        raw = cache_dir / f"{index:04d}_raw.mp3"
        _generate_tts(text, raw, provider)
        _process_to_oem(raw, out_path)
    else:
        raise AudioPackError(f"kind desconocido en track {index}: {kind}")

    return _validate(out_path)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Genera el OEM Audio Pack completo (68 pistas) para DFPlayer Mini."
    )
    parser.add_argument(
        "--provider", default=cfg.TTS_PROVIDER,
        choices=("elevenlabs", "openai", "espeak", "gtts"),
        help=f"Proveedor TTS (por defecto: {cfg.TTS_PROVIDER}).",
    )
    parser.add_argument(
        "--out", default=cfg.OUTPUT_DIR,
        help=f"Directorio de salida (por defecto: {cfg.OUTPUT_DIR}).",
    )
    parser.add_argument(
        "--cache", default=cfg.CACHE_DIR,
        help=f"Directorio de caché TTS (por defecto: {cfg.CACHE_DIR}).",
    )
    parser.add_argument(
        "--only", type=int, nargs="*", default=None,
        help="Genera solo las pistas indicadas (p.ej. --only 1 2 68).",
    )
    parser.add_argument(
        "--no-cache", action="store_true",
        help="Borra la caché TTS antes de empezar (fuerza regenerar).",
    )
    args = parser.parse_args(argv)

    _require_tool("ffmpeg")
    _require_tool("ffprobe")

    base = Path(__file__).resolve().parent
    out_dir = (base / args.out).resolve()
    cache_dir = (base / args.cache).resolve()

    if args.no_cache and cache_dir.exists():
        shutil.rmtree(cache_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cache_dir.mkdir(parents=True, exist_ok=True)

    indices = sorted(cfg.TRACKS.keys()) if not args.only else sorted(set(args.only))

    total_warnings: list[str] = []
    failed: list[int] = []
    for idx in indices:
        spec = cfg.TRACKS.get(idx)
        if spec is None:
            _warn(f"Track {idx} no está en TRACKS; se omite.")
            continue
        try:
            total_warnings.extend(
                _build_track(idx, spec,
                             provider=args.provider,
                             cache_dir=cache_dir,
                             out_dir=out_dir)
            )
        except AudioPackError as exc:
            failed.append(idx)
            _warn(f"Track {idx:04d}: {exc}")

    _log("")
    _log(f"Salida: {out_dir}")
    _log(f"Pistas procesadas: {len(indices) - len(failed)} / {len(indices)}")
    if failed:
        _warn(f"Pistas con error: {failed}")
    if total_warnings:
        _warn("Avisos de validación:")
        for w in total_warnings:
            _warn(f"  - {w}")
    else:
        _log("Validación OK · duración ≤ 4.5 s · 44.1 kHz mono · CBR 128 kbps.")

    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
