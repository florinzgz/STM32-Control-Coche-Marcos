# Procedimiento de copia de seguridad del firmware

> Procedimiento oficial para crear un **snapshot completo** del firmware
> grabado en el coche **antes** de cualquier cambio relevante
> (típicamente: antes de integrar el mando RC FlySky FS-i6X + FS-iA6B).
>
> Las herramientas viven en `tools/backup/`. Este documento es la guía
> operativa.

---

## 1. ¿Por qué un snapshot?

El firmware grabado en el coche tiene dos componentes que **no están** en Git:

1. **Calibraciones específicas del coche** (NVM del STM32, páginas 123-127):
   - Pedal (`pedal_cal_store.c` → página 124).
   - Mapa de sensores DS18B20 por rol (`sensor_map_store.c` → página 123).
   - Log persistente de errores (`error_log.c` → página 125).
   - Calibración de centrado del steering (`steering_cal_store.c` → página 126).
   - Parámetros EPS (`eps_params.c` → página 127).

2. **Datos del ESP32-S3**: bootloader + particiones + NVS + SPIFFS/LittleFS.

Si se reflashea solo el `.elf` compilado desde Git, **se pierde todo lo anterior**.
El snapshot binario las preserva tal cual están.

---

## 2. Antes de empezar

| Requisito | Cómo verificar |
|---|---|
| Repo limpio | `git status` debe decir "nothing to commit" |
| ST-Link conectado al STM32 (SWD) | LED rojo del ST-Link encendido |
| ESP32-S3 en modo bootloader | Pulsar `BOOT` mientras se hace reset |
| `STM32_Programmer_CLI` instalado | `STM32_Programmer_CLI --help` |
| `esptool.py` instalado | `esptool.py version` |

---

## 3. Procedimiento

### Paso 1 — Etiquetar el código

```bash
git tag -a pre-remote-control-v10 -m "Snapshot pre-mando RC ($(date -u +%Y-%m-%d))"
git push origin pre-remote-control-v10
```

Esto deja un punto de retorno inmutable en el repositorio. Para volver:
`git checkout pre-remote-control-v10`.

### Paso 2 — Dump binario del firmware grabado

**Linux/macOS:**
```bash
chmod +x tools/backup/backup_firmware.sh
./tools/backup/backup_firmware.sh /dev/ttyUSB0
```

**Windows (PowerShell):**
```powershell
.\tools\backup\backup_firmware.ps1 -Esp32Port COM3
```

### Paso 3 — Verificación

El directorio `backups/firmware_<UTC>/` debe contener:

```
metadata.txt
stm32_full.bin                   (524 288 bytes = 512 KB)
stm32_nvm_pages_123_127.bin      ( 20 480 bytes =  20 KB)
esp32_full.bin                   (16 777 216 bytes = 16 MB)
SHA256SUMS
```

Verificar tamaños y SHA256:
```bash
sha256sum -c backups/firmware_<UTC>/SHA256SUMS
```

### Paso 4 — Almacenamiento seguro

- Copiar el directorio `backups/firmware_<UTC>/` a **dos sitios** distintos:
  - Un disco externo / USB.
  - Un almacenamiento en la nube cifrado (Drive, OneDrive, etc.).
- **No** subir al repositorio (el `.gitignore` lo excluye).

### Paso 5 — Restauración (si fuera necesaria)

```bash
./tools/backup/restore_firmware.sh backups/firmware_<UTC>/ /dev/ttyUSB0
```

El script verifica el SHA256, borra y reprograma la Flash del STM32
(recuperando calibraciones) y reprograma la Flash del ESP32-S3.

---

## 4. Relación con la integración del mando RC

Este snapshot es el **paso 0 absoluto** antes de iniciar cualquier fase de
`docs/REMOTE_CONTROL_IMPLEMENTATION_PLAN.md`. La integración del mando es
**puramente aditiva** (un módulo nuevo aislado con un compile flag a 0
por defecto), pero la copia es una salvaguarda contra cualquier imprevisto:
errores humanos, flasheo cruzado entre coches, regresiones detectadas
tarde, etc.

Con el snapshot guardado y el tag `pre-remote-control-v10` en remoto,
cualquier paso posterior es **reversible** sin pérdida de calibraciones.

---

## 5. Comprobación de "¿funciona el resto sin el mando?"

Sí, el coche sigue funcionando al 100 % sin el mando porque:

- Ningún archivo existente (STM32 o ESP32) se modifica en las fases 0 y 1
  del plan del mando.
- El módulo `esp32/src/remote_control.{h,cpp}` (cuando se cree) está
  bajo `#define REMOTE_CONTROL_ENABLED 0` — desactivado por defecto.
- Los frames CAN `0x100`/`0x101` que usaría el mando son **los mismos**
  que ya emite la HMI hoy. El STM32 ni siquiera ve diferencia.
- El protocolo CAN existente no cambia (`docs/CAN_PROTOCOL.md`).

Por tanto, mientras llega el hardware del mando, todo el firmware actual
sigue funcionando exactamente igual.
