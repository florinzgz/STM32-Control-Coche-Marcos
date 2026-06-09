# AUDIO_WIRING_GUIDE — Cableado Pin a Pin

**Versión:** 1.1  
**Fecha:** 2026-06-09  
**Estado:** Sincronizado con Songle 4CH real + ULN2803A

## 1) Regla principal de seguridad

Las entradas Songle `IN1..IN4` miden ~5V en reposo (pull-up interno).  
**No conectar GPIO de ESP32/STM32 directamente a INx.**

Siempre usar:

`GPIO MCU -> ULN2803A (Bx/Cx) -> INx Songle`

## 2) Cableado ULN2803A definitivo

| Señal MCU | ULN Bx | ULN Cx | Destino |
|---|---|---|---|
| PB10 | 1B | 1C | IN1 (relé LED frontal) |
| PB11 | 2B | 2C | IN2 (relé LED trasero) |
| GPIO11 | 3B | 3C | IN3 (relé audio S3) |
| PC11 | 4B | 4C | IN4/canal potencia tracción |
| PC12 | 5B | 5C | INx restante potencia steering |

Conexiones comunes:

- ULN GND -> GND común del sistema.
- ULN COM -> no obligatorio si solo se atacan entradas INx Songle.

## 3) Audio (analógico) — conexiones válidas

- DFPlayer `DAC_L` -> `1k` -> nodo MONO_DF.
- DFPlayer `DAC_R` -> `1k` -> nodo MONO_DF.
- Módulo original L/R -> `1k + 1k` -> nodo MONO_ORIG.
- Usar condensador **105 (1µF)** de acoplo en la rama de señal de entrada al relé de audio.
- Relé audio usado: **Songle S3 (IN3)**.
- Salida conmutada -> entrada PAM8403.
- Masa de audio común.

## 4) Lógica de activación por firmware (actualizada)

Para audio con ULN2803A:

- `GPIO11 LOW` = reposo audio (relé OFF).
- `GPIO11 HIGH` = audio DFPlayer activo (relé ON).

Para STM32 (sin cambio):

- `PB10/PB11/PC11/PC12` con `GPIO_PIN_SET` activan su relé objetivo.

## 5) Componentes usados en la ruta audio

- 2 × resistencias 1k (mezcla L+R de DFPlayer).
- 2 × resistencias 1k (mezcla L+R de módulo original).
- 1 × condensador cerámico **105 = 1µF** (acoplo principal de señal).
- PAM8403 como etapa final.
