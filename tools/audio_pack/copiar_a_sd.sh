#!/usr/bin/env bash
# copiar_a_sd.sh
# ----------------------------------------------------------------------------
# Copia los 68 MP3 generados en output_sd/ a la SD del DFPlayer Mini en orden
# estricto (0001 → 0068) y forzando sync entre archivos. Esto evita que el
# DFPlayer indexe los archivos por orden de inserción FAT y desordene la
# numeración lógica.
#
# Uso:
#   ./copiar_a_sd.sh /ruta/al/punto/de/montaje/SD
#
# Recomendado: SD FAT32, 4–32 GB, sin carpetas extra.
# ----------------------------------------------------------------------------

set -euo pipefail

SRC_DIR="$(cd "$(dirname "$0")" && pwd)/output_sd"

if [[ $# -lt 1 ]]; then
  echo "Uso: $0 <ruta_SD_montada>" >&2
  exit 2
fi

DEST="$1"

if [[ ! -d "$SRC_DIR" ]]; then
  echo "[ERROR] No existe $SRC_DIR. Ejecuta primero: python3 generar_pack_oem.py" >&2
  exit 1
fi
if [[ ! -d "$DEST" ]]; then
  echo "[ERROR] Destino no existe o no está montado: $DEST" >&2
  exit 1
fi

echo "[oem-audio] Limpiando MP3 previos en $DEST"
# Borra solo los 0001..0068.mp3, no toca otras carpetas/archivos del usuario.
for i in $(seq -w 1 68); do
  rm -f "$DEST/${i}.mp3"
done
sync

echo "[oem-audio] Copiando 0001 → 0068 con sync..."
count=0
for i in $(seq -w 1 68); do
  src="$SRC_DIR/${i}.mp3"
  dst="$DEST/${i}.mp3"
  if [[ ! -f "$src" ]]; then
    echo "[WARN] Falta $src, se omite." >&2
    continue
  fi
  cp -- "$src" "$dst"
  sync
  count=$((count + 1))
done

echo "[oem-audio] Copiadas $count pistas a $DEST"
echo "[oem-audio] Expulsa la SD con seguridad (umount/eject) antes de retirarla."
