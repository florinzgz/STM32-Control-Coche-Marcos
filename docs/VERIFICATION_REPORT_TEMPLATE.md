# Informe de verificación UART/CAN — ESP32‑S3 ↔ NUCLEO‑G474RE

> **Fecha:** ____/____/________
> **Operador:** ____________________
> **Versión firmware STM32:** ____________________
> **Versión firmware ESP32:** ____________________
> **Interfaz activa:** CAN bus (FDCAN1 @ 500 kbps)

---

## Resultados

```
GND diff:       _____ V  (esperado: 0–0.05 V)
3V3 STM:        _____ V  (esperado: 3.20–3.40 V)
3V3 ESP:        _____ V  (esperado: 3.20–3.40 V)
BOOT0:          _____ V  (esperado: 0.00 V con jumper)
VCC xcvr STM:   _____ V  (esperado: 3.20–3.40 V)
VCC xcvr ESP:   _____ V  (esperado: 3.20–3.40 V)

JP7:            puesto / puenteado / ausente

LD2:            parpadea sí/no  (frecuencia: ___ Hz)
                patrón: startup / heartbeat / error / fault

ESP32 Serial:   actividad sí/no
                logs: "..."

UART:           N/A (proyecto usa CAN)
                TX waveform: N/A
                datos recibidos: N/A

CAN bus:
  Terminación total:    _____ Ω  (esperado: ~60 Ω)
  CANH quiescente:      _____ V  (esperado: ~2.5 V)
  CANL quiescente:      _____ V  (esperado: ~2.5 V)
  CANH dominante:       _____ V  (esperado: ~3.5 V)
  CANL dominante:       _____ V  (esperado: ~1.5 V)

CAN loopback STM32:
  FDCAN init OK:        sí / no
  Loopback OK:          sí / no
  tx_count:             _____
  rx_count:             _____
  Errores:              _____

CAN loopback ESP32:
  TWAI install OK:      sí / no
  TWAI start OK:        sí / no
  Loopback OK:          sí / no
  Frame recibido:       ID=0x___ DLC=_ data=0x__
  Errores:              _____

CAN comunicación inter-placa:
  STM32 rx_count:       _____  (esperado: ≥10/s)
  STM32 tx_count:       _____  (esperado: ≥10/s)
  STM32 tx_errors:      _____  (esperado: 0)
  STM32 rx_errors:      _____  (esperado: 0)
  STM32 busoff_count:   _____  (esperado: 0)
  STM32 fifo_overflow:  _____  (esperado: 0)
  ESP32 heartbeat RX:   sí / no
  ESP32 system state:   _____  (esperado: BOOT→STANDBY→ACTIVE)
  Frames RX en STM32:   _____
  Frames RX en ESP32:   _____

Debugger STM32:  PC=0x__________ (función: ____________)
                 hfdcan1.State: __________

Fotos:          sí / no
                [Adjuntar: JP7/CN7 zona pines, conexiones ESP32↔STM32]

Observaciones:  ____________________________________________
                ____________________________________________
                ____________________________________________
```

---

## Veredicto

- [ ] **PASS** — Todas las comprobaciones dentro de tolerancia, comunicación CAN establecida.
- [ ] **FAIL** — Fallo en paso: _____ . Detalle: ___________________________

---

## Referencia

Guía completa de verificación: [UART_CAN_CONNECTION_VERIFICATION.md](UART_CAN_CONNECTION_VERIFICATION.md)
