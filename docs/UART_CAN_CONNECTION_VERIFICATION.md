# Verificación de conexión UART/CAN — ESP32‑S3 ↔ NUCLEO‑G474RE

> **Interfaz activa en este proyecto:** CAN bus (FDCAN1 @ 500 kbps, CAN 2.0A clásico).
> No se utiliza UART entre las dos placas; la sección UART se incluye como
> referencia para pruebas alternativas.

---

## Documentos relacionados

| Documento | Contenido |
|-----------|-----------|
| [ESP32_STM32_CAN_CONNECTION.md](ESP32_STM32_CAN_CONNECTION.md) | Esquema de cableado CAN y TJA1051 |
| [VALIDACION_CONEXION_FISICA_CAN.md](VALIDACION_CONEXION_FISICA_CAN.md) | Checklist de 13 puntos antes de alimentar |
| [HARDWARE_VALIDATION_PROCEDURE.md](HARDWARE_VALIDATION_PROCEDURE.md) | Plan de test general (boot, CAN, sensores) |
| [CAN_CONTRACT_FINAL.md](CAN_CONTRACT_FINAL.md) | Protocolo CAN congelado (IDs, DLC, timing) |

---

## Resumen de pines y hardware

### STM32G474RE (NUCLEO‑G474RE)

| Señal | Pin | AF | Notas |
|-------|-----|----|-------|
| FDCAN1_TX | PA12 | AF9 | Hacia TJA1051T/3 pin 1 (TXD) |
| FDCAN1_RX | PA11 | AF9 | Desde TJA1051T/3 pin 4 (RXD) |
| LD2 (user LED) | PA5 | GPIO | Heartbeat 5 Hz en funcionamiento normal |
| BOOT0 | JP7 | — | Jumper GND = arranque desde Flash |

### ESP32‑S3‑DevKitC‑1

| Señal | GPIO | Notas |
|-------|------|-------|
| CAN TX (TWAI) | GPIO 4 | Hacia TJA1051 pin 1 (TXD) |
| CAN RX (TWAI) | GPIO 5 | Desde TJA1051 pin 4 (RXD) |
| USB‑CDC | Nativo | Serial Monitor @ 115 200 bps |

### Transceptores CAN

| Parámetro | Valor |
|-----------|-------|
| Chip | TJA1051T/3 (versión 3.3 V) |
| VCC | 3.3 V (alimentado desde el regulador de cada placa) |
| Pin 8 (S/SLNT) | **Conectado a GND** (modo normal, no silencioso) |
| Terminación | 120 Ω entre CANH y CANL en **cada** extremo |
| Desacoplo | 100 nF entre VCC y GND en cada transceptor |

### Parámetros CAN bus

| Parámetro | Valor |
|-----------|-------|
| Velocidad | 500 kbps |
| Formato | CAN 2.0A clásico (11‑bit ID) |
| Prescaler (STM32) | 10 (PCLK1 = 170 MHz → TQ = 58.8 ns) |
| TimeSeg1 | 29 TQ |
| TimeSeg2 | 4 TQ |
| SJW | 4 TQ |
| Punto de muestreo | 88.2 % |
| Auto-retransmisión | Habilitada |

---

## Paso 1 — Estado inicial

| Comprobación | Resultado |
|--------------|-----------|
| STM32 alimentada por USB (CN1 / ST‑LINK) | ☐ sí / ☐ no |
| ESP32‑S3 alimentada por USB (conector nativo o UART) | ☐ sí / ☐ no |
| GND(ESP32) ↔ GND(STM32) cable conectado | ☐ sí / ☐ no |
| LD1 (ST‑LINK) encendido verde/rojo | ☐ sí / ☐ no |
| ESP32 LED de alimentación encendido | ☐ sí / ☐ no |

---

## Paso 2 — Comprobaciones eléctricas (multímetro)

> **Precaución:** Ambas placas deben estar **alimentadas** para estas mediciones
> (excepto continuidad, que se hace sin alimentación).

| Medición | Puntos de medida | Esperado | Medido |
|----------|------------------|----------|--------|
| GND diff | GND(STM32) vs GND(ESP32) | 0–0.05 V | _____ V |
| 3V3 STM | 3V3(Nucleo CN7 pin 16) vs GND(Nucleo) | 3.20–3.40 V | _____ V |
| 3V3 ESP | 3V3(ESP32 pin 3V3) vs GND(ESP32) | 3.20–3.40 V | _____ V |
| BOOT0 | JP7 pin superior vs GND (jumper puesto) | 0.00 V (GND) | _____ V |
| VCC transceptor STM | VCC TJA1051 (STM side) vs GND | 3.20–3.40 V | _____ V |
| VCC transceptor ESP | VCC TJA1051 (ESP side) vs GND | 3.20–3.40 V | _____ V |

### Continuidad (modo beep, placas **apagadas**)

| Prueba | Puntos de medida | Esperado | Resultado |
|--------|------------------|----------|-----------|
| JP7 a GND | Pin inferior JP7 → GND conocido Nucleo | Beep (sí) | ☐ sí / ☐ no |
| GND común | GND(ESP32) → GND(STM32) | Beep (sí) | ☐ sí / ☐ no |
| Bus CANH | CANH transceptor STM → CANH transceptor ESP | Beep (sí) | ☐ sí / ☐ no |
| Bus CANL | CANL transceptor STM → CANL transceptor ESP | Beep (sí) | ☐ sí / ☐ no |
| Terminación STM | CANH → CANL en transceptor STM | ~120 Ω | _____ Ω |
| Terminación ESP | CANH → CANL en transceptor ESP | ~120 Ω | _____ Ω |
| Terminación total | CANH → CANL en el bus (ambas R en paralelo) | ~60 Ω | _____ Ω |

---

## Paso 3 — Verificación física de JP7 (BOOT0)

JP7 en la NUCLEO‑G474RE controla la señal **BOOT0**:
- **Jumper puesto** (ambos pines cubiertos): BOOT0 = GND → arranque desde Flash ✅
- **Jumper ausente**: BOOT0 flotante → puede arrancar desde System Memory (bootloader) ❌

| Acción | Estado |
|--------|--------|
| Capuchón JP7 cubre ambos pines | ☐ sí / ☐ no |
| Si no hay capuchón: puenteado con cable corto | ☐ sí / ☐ no |
| BOOT0 medido = 0.00 V | ☐ sí / ☐ no |

> **Nota:** Los pines CAN del STM32 son **PA11 (RX)** y **PA12 (TX)**, NO PB8/PB9.
> PB8/PB9 ahora se usan para el bus I2C1 (SCL/SDA). JP7 no afecta al bus CAN.

---

## Paso 4 — Comprobaciones de pines en la ESP32‑S3

### 4.1 Pines CAN/TWAI

| GPIO | Función | Boot‑strapping | Conflicto |
|------|---------|----------------|-----------|
| GPIO 4 | CAN TX (TWAI) | No | Ninguno ✅ |
| GPIO 5 | CAN RX (TWAI) | No | Ninguno ✅ |

> GPIO 4 y GPIO 5 no son pines de boot‑strapping en el ESP32‑S3 y pueden
> usarse libremente para TWAI.

### 4.2 Transceptor TJA1051 (lado ESP32)

| Comprobación | Resultado |
|--------------|-----------|
| TJA1051 físicamente conectado a GPIO 4/5 | ☐ sí / ☐ no |
| VCC del transceptor medido (3.20–3.40 V) | _____ V |
| Pin 8 (S/SLNT) conectado a GND | ☐ sí / ☐ no |
| Resistencia terminación 120 Ω presente | ☐ sí / ☐ no |
| Condensador 100 nF desacoplo presente | ☐ sí / ☐ no |

---

## Paso 5 — Prueba de ejecución y firmware

### 5.1 STM32 — LED diagnóstico (LD2 / PA5)

El firmware tiene tres patrones de parpadeo en LD2:

| Patrón | Significado | Frecuencia |
|--------|-------------|------------|
| 3 parpadeos rápidos al arrancar | Startup OK | Secuencia única tras reset |
| Toggle continuo | Heartbeat normal (main loop) | ~5 Hz |
| Parpadeo lento | `Error_Handler()` activo | ~2 Hz |
| Parpadeo muy rápido | Fault handler (HardFault, etc.) | ~10 Hz |

| Comprobación | Resultado |
|--------------|-----------|
| LD2 parpadea tras reset | ☐ sí / ☐ no |
| Patrón observado | ☐ startup → heartbeat / ☐ error / ☐ fault |
| Frecuencia aproximada | _____ Hz |

> **Si LD2 no parpadea:** verificar JP7 (BOOT0=GND), comprobar que el
> firmware está flasheado, y probar Debug → Connect under reset en CubeIDE.

### 5.2 ESP32‑S3 — Serial Monitor

| Comprobación | Resultado |
|--------------|-----------|
| Serial Monitor abierto (115 200 bps) | ☐ sí / ☐ no |
| Velocidad correcta (sin caracteres basura) | ☐ sí / ☐ no |
| Actividad (mensajes de arranque/logs) | ☐ sí / ☐ no |
| Líneas relevantes | (pegar abajo) |

```
Líneas del Serial Monitor:
___________________________________________
___________________________________________
___________________________________________
```

---

## Paso 6 — Prueba UART entre placas

> **NOTA:** Este proyecto NO utiliza UART para comunicación ESP32↔STM32.
> La interfaz de comunicación es **CAN bus**.
> Esta sección se incluye solo como referencia si se necesita una prueba
> UART alternativa para diagnóstico.

### 6.1 Si se desea probar UART (diagnóstico temporal)

Para probar UART requeriría:
1. Habilitar un USART en el STM32 (ej. USART1 en PA9/PA10 o USART2 en PA2/PA3).
   - **PA2/PA3 están en uso** (PA2 = EXTI wheel RL, PA3 = ADC pedal).
   - PA9/PA10 están en uso para TIM1 (PWM motor FR).
   - Alternativa segura: USART3 en PC10/PC11 (**pero PC11 es relay RELAY_TRAC; PC10 está libre aunque no conectado**). PC10 = INPUT_PULLDOWN, libre para expansión.
2. Configurar un Serial1 en la ESP32 con pines libres.

**Conclusión:** No hay pines UART disponibles sin conflicto. Utilizar CAN como
interfaz principal (Paso 7).

---

## Paso 7 — Prueba CAN entre placas (interfaz principal)

### 7.1 Checklist pre-test

| Comprobación | Estado |
|--------------|--------|
| Transceptor TJA1051 en STM32 alimentado y cableado | ☐ |
| Transceptor TJA1051 en ESP32 alimentado y cableado | ☐ |
| Pin 8 (S/SLNT) a GND en **ambos** transceptores | ☐ |
| Terminación 120 Ω en extremo STM32 | ☐ |
| Terminación 120 Ω en extremo ESP32 | ☐ |
| GND común conectado | ☐ |
| Ambos nodos: 500 kbps | ☐ |
| Condensadores 100 nF en ambos transceptores | ☐ |

### 7.2 Loopback interno STM32

El firmware soporta loopback interno con la macro `CAN_LOOPBACK_TEST`.

**Para activar:**
1. En `Core/Src/main.c`, línea 38, cambiar:
   ```c
   #define CAN_LOOPBACK_TEST  1
   ```
   O compilar con `-DCAN_LOOPBACK_TEST=1` en las opciones del compilador.

2. Flashear y verificar:
   - En modo loopback, el transceptor se bypasea: los frames TX se rutan
     directamente al Rx FIFO.
   - El heartbeat STM32 (0x001) se envía cada 100 ms y debe recibirse
     internamente.

| Resultado loopback | Estado |
|--------------------|--------|
| FDCAN init OK (`hfdcan1` inicializado sin error) | ☐ sí / ☐ no |
| Frames TX (heartbeat 0x001) transmitidos | ☐ sí / ☐ no |
| Frames recibidos en Rx FIFO (loopback) | ☐ sí / ☐ no |
| `can_stats.tx_count` incrementa | ☐ sí / ☐ no |
| `can_stats.rx_count` incrementa | ☐ sí / ☐ no |

> **Si loopback OK:** El periférico FDCAN funciona correctamente.
> El problema está en el bus físico o transceptores.
>
> **Si loopback NOK:** Comprobar inicialización FDCAN, reloj PCLK1,
> y que `HAL_FDCAN_Init()` no devuelve error.
> Revisar si `fdcan_init_ok` es `false` en `CAN_Init()`.

### 7.3 Test de comunicación CAN entre placas

**Procedimiento:**
1. Asegurar `CAN_LOOPBACK_TEST = 0` en STM32 (modo normal).
2. Flashear firmware en ambas placas.
3. La ESP32 envía heartbeat (0x011) cada 100 ms automáticamente.
4. El STM32 envía heartbeat (0x001) cada 100 ms.

**Verificar en STM32 (debug o variables):**

| Variable / indicador | Esperado | Observado |
|----------------------|----------|-----------|
| `can_stats.rx_count` | Incrementa (≥10/s) | _____ |
| `can_stats.tx_count` | Incrementa (≥10/s) | _____ |
| `can_stats.tx_errors` | 0 | _____ |
| `can_stats.rx_errors` | 0 | _____ |
| `can_stats.busoff_count` | 0 | _____ |
| `can_stats.fifo_overflow_count` | 0 | _____ |
| `CAN_IsESP32Alive()` | `true` | ☐ sí / ☐ no |
| LD2 toggle en ISR (recepción CAN) | Parpadea rápido | ☐ sí / ☐ no |

**Verificar en ESP32 (Serial Monitor):**

| Indicador | Esperado | Observado |
|-----------|----------|-----------|
| Heartbeat STM32 recibido | sí (counter incrementa) | ☐ sí / ☐ no |
| System state recibido | BOOT → STANDBY → ACTIVE | _____ |
| Errores CAN (bus-off, FIFO overflow) | Ninguno | _____ |

### 7.4 Diagnóstico CAN con osciloscopio

Si la comunicación falla, medir con osciloscopio:

| Punto de medida | Esperado | Medido |
|-----------------|----------|--------|
| CANH (quiescente, sin tráfico) | ~2.5 V | _____ V |
| CANL (quiescente, sin tráfico) | ~2.5 V | _____ V |
| CANH (dominante, con tráfico) | ~3.5 V | _____ V |
| CANL (dominante, con tráfico) | ~1.5 V | _____ V |
| Diferencial (CANH − CANL, dominante) | ~2.0 V | _____ V |
| Forma de onda TX (PA12 STM lado MCU) | Tramas 500 kbps | ☐ sí / ☐ no |
| Forma de onda TX (GPIO4 ESP lado MCU) | Tramas 500 kbps | ☐ sí / ☐ no |

---

## Paso 8 — Loopback y auto-diagnóstico ESP32‑S3

### 8.1 TWAI/CAN loopback ESP32

El driver TWAI del ESP32 soporta modo loopback (no‑ACK mode).

**Sketch de prueba (Arduino):**
```cpp
#include <driver/twai.h>

#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== TWAI Loopback Test ===");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    Serial.printf("TWAI install: %s\n", esp_err_to_name(err));

    err = twai_start();
    Serial.printf("TWAI start: %s\n", esp_err_to_name(err));
}

void loop() {
    // Enviar frame de prueba (heartbeat ESP32, ID 0x011)
    twai_message_t tx_msg = {};
    tx_msg.identifier = 0x011;
    tx_msg.data_length_code = 1;
    tx_msg.data[0] = 0xAA;  // Byte de test

    esp_err_t tx_err = twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
    Serial.printf("TX: %s\n", esp_err_to_name(tx_err));

    // Intentar recibir (en loopback debería recibir su propio frame)
    twai_message_t rx_msg = {};
    esp_err_t rx_err = twai_receive(&rx_msg, pdMS_TO_TICKS(200));
    if (rx_err == ESP_OK) {
        Serial.printf("RX OK: ID=0x%03X DLC=%d data[0]=0x%02X\n",
                       rx_msg.identifier, rx_msg.data_length_code,
                       rx_msg.data[0]);
    } else {
        Serial.printf("RX: %s\n", esp_err_to_name(rx_err));
    }

    delay(1000);
}
```

| Resultado loopback ESP32 | Estado |
|--------------------------|--------|
| `twai_driver_install()` OK | ☐ sí / ☐ no |
| `twai_start()` OK | ☐ sí / ☐ no |
| TX sin error | ☐ sí / ☐ no |
| RX recibe frame propio | ☐ sí / ☐ no |
| ID y datos correctos | ☐ sí / ☐ no |

> **Si no transmite:** verificar que GPIO 4/5 no están retenidos por otro
> driver (SPI, I2C, etc.), que el TWAI está correctamente inicializado, y
> que no hay conflicto con el USB‑CDC en modo debug.

---

## Paso 9 — Verificación de niveles y protecciones

| Comprobación | Estado |
|--------------|--------|
| STM32 señales lógicas: 3.3 V | ☐ sí |
| ESP32‑S3 señales lógicas: 3.3 V | ☐ sí |
| TJA1051**T/3** (versión 3.3 V, no la de 5 V) | ☐ sí / ☐ no |
| Si TJA1051 estándar (5 V): level shifter instalado | ☐ N/A / ☐ sí / ☐ no |
| Resistencias en serie en líneas CAN (si hay) | ☐ N/A / ☐ valor: ___ Ω |

> **PELIGRO:** Si se usa un TJA1051 estándar (no /3), su salida RXD es 5 V.
> GPIO 5 del ESP32‑S3 **NO** es tolerante a 5 V. Se necesita un divisor
> resistivo (1 kΩ + 2 kΩ) o un level shifter bidireccional.

---

## Paso 10 — Debug bajo reset (si algo falla)

### 10.1 STM32 (CubeIDE)

1. **Debug Configurations → Debugger → Connect under reset** (o "Halt on startup").
2. Iniciar debug y pausar inmediatamente.

| Dato | Valor |
|------|-------|
| PC (Program Counter) | 0x__________ |
| Función donde se detuvo | __________________ |
| FDCAN init completado | ☐ sí / ☐ no |
| `hfdcan1.State` | __________________ |
| `can_stats` (todos los campos) | tx=___ rx=___ err_tx=___ err_rx=___ |

**Funciones típicas donde puede detenerse:**
- `Reset_Handler` → no ha arrancado correctamente
- `SystemClock_Config` → problema de reloj
- `MX_FDCAN1_Init` → problema de inicialización CAN
- `main` (loop) → funcionamiento normal
- `Error_Handler` → error fatal en inicialización

### 10.2 ESP32‑S3 (monitor de logs)

```bash
# Con PlatformIO
pio device monitor -b 115200

# Con ESP-IDF
idf.py monitor
```

| Dato | Valor |
|------|-------|
| Errores de inicialización | __________________ |
| TWAI driver status | __________________ |
| Stack trace (si hay crash) | __________________ |

---

## Paso 11 — Informe final

Copiar y rellenar:

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

CAN bus:
  Terminación:  ~60 Ω total (sí/no)
  CANH quiesc.: _____ V  (esperado: ~2.5 V)
  CANL quiesc.: _____ V  (esperado: ~2.5 V)

CAN loopback STM32:
  FDCAN init OK:  sí/no
  Loopback OK:    sí/no
  tx_count:       _____
  rx_count:       _____

CAN loopback ESP32:
  TWAI init OK:   sí/no
  Loopback OK:    sí/no

CAN comunicación inter-placa:
  STM32 rx_count:       _____  (esperado: ≥10/s)
  STM32 tx_count:       _____
  ESP32 heartbeat RX:   sí/no
  Errores bus-off:      _____
  Errores FIFO overflow: _____

Debugger STM32:  PC=0x__________ (función: ____________)

Fotos:          sí/no
                [JP7 y CN7 zona pines]
                [Conexiones ESP32↔STM32 cables/pines]

Observaciones:  ____________________________________________
```

---

## Criterios de éxito

| Criterio | Valor OK |
|----------|----------|
| GND común | 0–0.05 V |
| BOOT0 | 0.00 V (jumper GND) |
| 3V3 ambas placas | 3.20–3.40 V |
| VCC transceptores | 3.20–3.40 V |
| Terminación bus | ~60 Ω (2 × 120 Ω en paralelo) |
| LD2 parpadeando | Sí, heartbeat ~5 Hz |
| ESP32 Serial Monitor | Muestra logs de arranque |
| CAN loopback STM32 | OK (tx y rx incrementan) |
| CAN loopback ESP32 | OK (recibe frame propio) |
| Comunicación CAN | STM32 recibe heartbeat ESP32 y viceversa |
| Sin errores CAN | bus-off = 0, FIFO overflow = 0 |

---

## Árbol de diagnóstico

```
¿LD2 parpadea?
├─ NO → Verificar JP7, flashear firmware, debug under reset
└─ SÍ
   ├─ Patrón = error (2 Hz) → Error en init (revisar MX_FDCAN1_Init)
   ├─ Patrón = fault (10 Hz) → HardFault (debug, leer PC)
   └─ Patrón = heartbeat (5 Hz) → Firmware OK
      │
      ¿CAN loopback STM32 OK?
      ├─ NO → FDCAN periférico con problema (reloj, init, RAM)
      └─ SÍ → Periférico OK, problema en bus físico
         │
         ¿CAN loopback ESP32 OK?
         ├─ NO → TWAI driver ESP32 con problema (pines, init)
         └─ SÍ → Ambos periféricos OK
            │
            ¿Comunicación entre placas?
            ├─ NO → Bus físico:
            │  ├─ Verificar terminación (120 Ω × 2)
            │  ├─ Verificar CANH/CANL niveles con osciloscopio
            │  ├─ Verificar S/SLNT pin a GND
            │  ├─ Verificar GND común
            │  └─ Verificar VCC transceptores
            └─ SÍ → ✅ Comunicación establecida
```

---

## Notas de seguridad

- Desconectar periféricos antes de puentear pines.
- No modificar alimentación mientras se mide continuidad en modo beep.
- Si se sospecha cortocircuito, desconectar USB inmediatamente.
- GPIO del ESP32‑S3 **NO** son tolerantes a 5 V.
- Verificar que el TJA1051 es la versión **/3** (3.3 V) antes de conectar al ESP32.
