# Backup / Restore del firmware del coche

Herramientas para crear una **copia de seguridad completa** del firmware
actualmente grabado en el STM32G474RE y en el ESP32-S3 **antes** de
introducir cualquier cambio (típicamente, antes de integrar el mando RC).

Garantiza que, si algo va mal, se puede volver al estado de funcionamiento
actual reflasheando los binarios — **incluyendo las calibraciones**
guardadas en las páginas NVM 123-127 del STM32.

---

## Contenido

| Archivo | Propósito |
|---|---|
| `backup_firmware.sh`  | Linux/macOS — crea el snapshot en `backups/firmware_<UTC>/` |
| `backup_firmware.ps1` | Windows PowerShell — equivalente |
| `restore_firmware.sh` | Restaura un snapshot completo |
| `README.md` | Este documento |

> **El agente Copilot no tiene acceso al hardware**: estos scripts se
> ejecutan en el equipo del usuario, conectado físicamente al coche por
> ST-Link (STM32) y USB-Serial (ESP32-S3).

---

## Requisitos previos

- **STM32CubeProgrammer** (proporciona `STM32_Programmer_CLI`).
- **esptool.py** (`pip install esptool`) — versión ≥ 4.x.
- ST-Link V2/V3 conectado al SWD del STM32G474RE.
- Cable USB-Serial al ESP32-S3 con el bootloader accesible
  (pulsar BOOT mientras se hace reset).

---

## Qué se copia

| Componente | Rango | Tamaño | Incluye |
|---|---|---|---|
| STM32 código + datos | `0x08000000`..`0x0807AFFF` | 492 KB | `.text`, `.rodata`, `.data` init image |
| STM32 NVM completa   | `0x0807B000`..`0x0807FFFF` | 20 KB  | sensor_map, pedal_cal, error_log, steering_cal, eps_params |
| STM32 dump conjunto  | `0x08000000`..`0x0807FFFF` | 512 KB | Todo lo anterior en un solo `.bin` |
| ESP32-S3 Flash       | `0x000000`..`0xFFFFFF`     | 16 MB  | Bootloader, particiones, app, NVS, SPIFFS/LittleFS |

> Las páginas NVM están documentadas en `STM32G474RETX_FLASH.ld:36-50`.
> Sin ellas, una restauración perdería: la calibración del pedal, el mapa
> DS18B20, el log de errores, la calibración del steering y los parámetros EPS.

---

## Flujo recomendado antes de la integración del mando

1. **Asegurar repositorio limpio y rama sincronizada** (`git status` limpio).
2. Ejecutar el backup:
   ```bash
   chmod +x tools/backup/backup_firmware.sh
   ./tools/backup/backup_firmware.sh /dev/ttyUSB0
   ```
3. El script crea `backups/firmware_<UTC>/` con:
   - `stm32_full.bin`
   - `stm32_nvm_pages_123_127.bin`
   - `esp32_full.bin`
   - `metadata.txt` (commit SHA, rama, fecha, host)
   - `SHA256SUMS`
4. **Etiquetar el commit** correspondiente:
   ```bash
   git tag -a pre-remote-control-v10 -m "Snapshot pre-mando RC"
   git push origin pre-remote-control-v10
   ```
5. Guardar el directorio `backups/firmware_<UTC>/` fuera del repositorio
   (es un binario grande — añadido a `.gitignore`).

---

## Restauración

```bash
./tools/backup/restore_firmware.sh backups/firmware_<UTC>/ /dev/ttyUSB0
```

El script:
1. Verifica el SHA256 del snapshot.
2. Borra y reprograma la Flash del STM32 (recuperando calibraciones).
3. Reprograma la Flash del ESP32-S3.

---

## Notas de seguridad

- Los `.bin` contienen el firmware tal cual está en el chip. **No subir a Git.**
  El `.gitignore` de la raíz excluye `backups/`.
- Si el STM32 tiene Read-Out Protection (RDP) activo en nivel 2, el dump
  fallará. En este proyecto RDP está en nivel 0 (sin protección).
- La restauración de un dump capturado en otro coche perderá las
  calibraciones específicas del coche actual (porque las NVM son distintas).

---

## Relación con el plan del mando RC

Este backup es el **paso 0 absoluto** antes de iniciar cualquier fase de
`docs/REMOTE_CONTROL_IMPLEMENTATION_PLAN.md`. Si tras integrar el mando se
detecta una regresión que no se puede corregir, la restauración devuelve
el coche al estado de funcionamiento previo sin pérdida de calibraciones.
