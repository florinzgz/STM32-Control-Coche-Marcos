# AUDIO_SCHEMATIC_ASCII — Esquema eléctrico ASCII (actualizado)

**Versión:** 1.1  
**Fecha:** 2026-06-09

## 1) Control de relé de audio con ULN2803A

```
ESP32 GPIO11 ----> 3B (ULN2803A)
                   3C (ULN2803A) ----> Songle IN3
GND comun -------- GND (ULN) -----+--> Songle GND
                                  +--> ESP32 GND
                                  +--> DFPlayer GND
```

Lógica:

- GPIO11=HIGH -> ULN hunde IN3 -> Songle S3 ON.
- GPIO11=LOW  -> IN3 vuelve a 5V por pull-up -> Songle S3 OFF.

## 2) Ruta de audio previa al PAM8403

```
DFPlayer DAC_L --[1k]--+
                        +-- MONO_DF --[105=1uF]--+
DFPlayer DAC_R --[1k]--+                        |
                                                 +--> contacto relé Songle S3 --> PAM8403 IN
Modulo original L --[1k]--+
                           +-- MONO_ORIG --------+
Modulo original R --[1k]--+

GND comun ---------------------------------------> PAM8403 GND
```

## 3) Mapeo ULN completo usado en vehículo

```
1B/1C: PB10  -> Relay LED frontal
2B/2C: PB11  -> Relay LED trasero
3B/3C: GPIO11-> Relay audio (Songle IN3)
4B/4C: PC11  -> Relay potencia traccion
5B/5C: PC12  -> Relay potencia steering
6B/6C: libre
7B/7C: libre
8B/8C: libre
```
