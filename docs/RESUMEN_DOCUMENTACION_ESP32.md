# RESUMEN: Documentación de Pines ESP32-S3 Completada

## ✅ Documentación Creada

Se ha creado documentación completa sobre las conexiones de pines del ESP32-S3 para:
1. **Pantalla TFT** (Display ST7796 480×320 con touch panel)
2. **CAN-Bus** (Conexión ESP32-S3 → TJA1051 → STM32G474RE)

---

## 📄 Archivos Disponibles

### 🌟 Documento Principal
**`docs/ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md`**
- Guía completa y detallada (429 líneas)
- Incluye todos los pines de la pantalla y CAN-Bus
- Especificaciones técnicas completas
- Lista de materiales necesarios
- Procedimiento paso a paso
- Solución de problemas
- Referencias a datasheets

### 📊 Diagramas Visuales
**`docs/DIAGRAMA_PINES_VISUAL.md`**
- Diagramas ASCII de las conexiones (370 líneas)
- Esquemas visuales del display y transceiver
- Mapa de GPIOs del ESP32-S3
- Código de colores para cables
- Checklist de verificación

### ⚡ Referencia Rápida
**`docs/CONEXIONES_RAPIDAS_ESP32.md`**
- Tablas compactas para consulta rápida (138 líneas)
- Valores de verificación con multímetro
- Diagnóstico rápido de problemas

### 📑 Índice
**`docs/ESP32_PIN_DOCUMENTATION_INDEX.md`**
- Guía de navegación de toda la documentación
- Instrucciones para convertir a PDF
- Enlaces a documentos relacionados

---

## 🔌 RESUMEN DE PINES

### Pantalla TFT (según tu lista de pines)

| Pin de tu Pantalla | Pin ESP32-S3 | Función |
|-------------------|--------------|---------|
| **vcc** | 3.3V | Alimentación |
| **gnd** | GND | Tierra |
| **cs** | GPIO 15 | Chip Select Display |
| **reset** | GPIO 17 | Reset Display |
| **dc/rs** | GPIO 16 | Data/Command |
| **sdi (mosi)** | GPIO 13 | Datos SPI |
| **sck** | GPIO 14 | Reloj SPI |
| **led** | GPIO 42 | Retroiluminación |
| **sdo (miso)** | No conectar | No usado |
| **t_cs** | GPIO 21 | Chip Select Touch |
| **t_din** | GPIO 13 | Touch Data (compartido con MOSI) |
| **t_clk** | GPIO 14 | Touch Clock (compartido con SCK) |
| **t_do** | No conectar | No usado (modo polling) |
| **t_irq** | No conectar | No usado (modo polling) |

### CAN-Bus (ESP32-S3 → TJA1051)

| ESP32-S3 | TJA1051 | Cable CAN |
|----------|---------|-----------|
| GPIO 4 (TX) | Pin 1 (TXD) | — |
| GPIO 5 (RX) | Pin 4 (RXD) | — |
| 5V | Pin 3 (VCC) | — |
| GND | Pin 2 (GND) | — |
| GND | Pin 8 (S) | Modo normal |
| — | Pin 7 (CANH) | Cable naranja (par trenzado) |
| — | Pin 6 (CANL) | Cable blanco (par trenzado) |

**Importante:** Instalar resistencia de 120Ω entre CANH y CANL en ambos extremos del bus.

---

## 📦 Cómo Convertir a PDF

### Opción 1: Pandoc (Recomendado)
```bash
# Instalar pandoc (si no lo tienes)
# Ubuntu/Debian: sudo apt install pandoc
# Windows: descargar de https://pandoc.org/

# Convertir documento principal
pandoc docs/ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md -o conexiones_esp32.pdf

# Convertir diagramas visuales
pandoc docs/DIAGRAMA_PINES_VISUAL.md -o diagramas_visuales.pdf

# Convertir referencia rápida
pandoc docs/CONEXIONES_RAPIDAS_ESP32.md -o referencia_rapida.pdf
```

### Opción 2: Herramientas Online (Fácil, sin instalar nada)
1. Ir a: https://www.markdowntopdf.com/
2. Copiar y pegar el contenido de cualquier archivo .md
3. Click en "Convert" → Descargar PDF

### Opción 3: Visual Studio Code
1. Instalar extensión "Markdown PDF" en VS Code
2. Abrir el archivo .md
3. Presionar `Ctrl+Shift+P` → "Markdown PDF: Export (pdf)"

---

## ✅ Verificación Rápida con Multímetro

Antes de encender todo, verificar con multímetro:

1. **ESP32 3.3V → GND**: 3.30V ± 0.1V
2. **Display VCC → GND**: 3.30V ± 0.1V  
3. **TJA1051 VCC → GND**: 5.00V ± 0.2V
4. **CANH → GND (bus idle)**: ~2.5V
5. **CANL → GND (bus idle)**: ~2.5V
6. **Resistencia CANH ↔ CANL**: 60Ω (con terminación instalada)

---

## 🚀 Pasos para Empezar

1. **Leer primero**: `docs/ESP32_PIN_DOCUMENTATION_INDEX.md`
2. **Montar**: Seguir `docs/DIAGRAMA_PINES_VISUAL.md`
3. **Consultar**: Tener a mano `docs/CONEXIONES_RAPIDAS_ESP32.md`
4. **Detalles técnicos**: Ver `docs/ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md`

---

## 📍 Ubicación de los Archivos

Todos los documentos están en el directorio `docs/` del repositorio:

```
STM32-Control-Coche-Marcos/
└── docs/
    ├── ESP32_PIN_DOCUMENTATION_INDEX.md      ← EMPEZAR AQUÍ
    ├── ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md  ← Guía completa
    ├── DIAGRAMA_PINES_VISUAL.md              ← Diagramas
    └── CONEXIONES_RAPIDAS_ESP32.md           ← Referencia rápida
```

---

## 💡 Información Adicional

### Configuración en el Código
Los pines están definidos en:
- `esp32/platformio.ini` - Definiciones de compilación (#define)
- `esp32/src/main.cpp` - Constantes CAN_TX_PIN y CAN_RX_PIN

### Documentación Relacionada
- `docs/ESP32_STM32_CAN_CONNECTION.md` - Conexión CAN detallada
- `docs/PINOUT_DEFINITIVO.md` - Pinout del STM32
- `docs/CONEXIONES_COMPLETAS.md` - Sistema completo

---

## ❓ Preguntas Frecuentes

**Q: ¿Por qué T_DO y T_IRQ no se conectan?**  
A: El touch panel usa polling en lugar de interrupciones, por lo que estos pines no son necesarios.

**Q: ¿Puedo usar MISO del display?**  
A: No es necesario. El display solo recibe comandos y datos, no envía nada de vuelta al ESP32.

**Q: ¿Por qué el TJA1051 necesita 5V si el ESP32 es 3.3V?**  
A: El TJA1051 necesita 5V para alimentación, pero sus pines de entrada/salida son compatibles con 3.3V.

**Q: ¿Cuánta corriente consume el sistema?**  
A: ESP32 + Display ≈ 500mA @ 3.3V, TJA1051 ≈ 70mA @ 5V

---

## 🎯 Próximos Pasos Sugeridos

1. ✅ Leer documentación (ya creada)
2. ⏭️ Imprimir o convertir a PDF `CONEXIONES_RAPIDAS_ESP32.md`
3. ⏭️ Verificar que tienes todos los componentes (ver lista de materiales)
4. ⏭️ Seguir el procedimiento de conexión paso a paso
5. ⏭️ Verificar con multímetro antes de encender
6. ⏭️ Cargar firmware y probar

---

**¡Toda la documentación está lista y disponible!**

Si necesitas más aclaraciones o tienes preguntas específicas, puedes crear un issue en el repositorio o consultar los documentos listados arriba.

---

_Documentación creada: 2026-02-19_  
_Repositorio: florinzgz/STM32-Control-Coche-Marcos_
