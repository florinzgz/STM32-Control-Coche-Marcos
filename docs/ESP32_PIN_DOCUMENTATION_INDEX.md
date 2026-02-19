# Índice de Documentación - Conexiones ESP32-S3

Este directorio contiene documentación completa sobre las conexiones de pines del ESP32-S3 para el sistema HMI del Control Coche Marcos.

## 📄 Documentos Disponibles

### 1. **ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md** ⭐ DOCUMENTO PRINCIPAL
**Guía completa y detallada** de todas las conexiones ESP32-S3.

**Contenido:**
- ✅ Mapa completo de pines de pantalla TFT (ST7796)
- ✅ Conexiones del touch panel
- ✅ Configuración SPI detallada
- ✅ Diagrama completo de CAN-Bus
- ✅ Conexión ESP32-S3 ↔ TJA1051 ↔ STM32
- ✅ Especificaciones técnicas
- ✅ Lista de materiales
- ✅ Procedimiento paso a paso
- ✅ Solución de problemas
- ✅ Referencias y datasheets

**Ideal para:** Montaje completo, troubleshooting avanzado, especificaciones técnicas

---

### 2. **DIAGRAMA_PINES_VISUAL.md** 📊 DIAGRAMAS VISUALES
**Guía visual con diagramas ASCII** de las conexiones.

**Contenido:**
- ✅ Diagramas de conexión física
- ✅ Esquemas del display y transceiver
- ✅ Mapa visual de GPIOs en DevKitC-1
- ✅ Código de colores para cables
- ✅ Checklist de verificación
- ✅ Vista general del sistema

**Ideal para:** Conexión física paso a paso, verificación visual, montaje de cables

---

### 3. **CONEXIONES_RAPIDAS_ESP32.md** ⚡ REFERENCIA RÁPIDA
**Guía de consulta rápida** en formato tabla.

**Contenido:**
- ✅ Tablas compactas de pines
- ✅ Display: pines principales + touch
- ✅ CAN-Bus: ESP32 ↔ TJA1051
- ✅ Valores de verificación con multímetro
- ✅ Diagnóstico rápido de problemas
- ✅ Diagrama simplificado

**Ideal para:** Consulta rápida durante el montaje, verificación con multímetro

---

## 🎯 ¿Qué Documento Usar?

### Para Montar desde Cero
1. Leer **ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md** (completo)
2. Seguir **DIAGRAMA_PINES_VISUAL.md** (paso a paso con diagramas)
3. Tener **CONEXIONES_RAPIDAS_ESP32.md** a mano (referencia rápida)

### Para Verificar Conexiones
- Usar **CONEXIONES_RAPIDAS_ESP32.md** (tablas de pines y valores)

### Para Solucionar Problemas
- Ver sección de troubleshooting en **ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md**

### Para Comprar Materiales
- Ver lista en **ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md** → Sección 3

---

## 📌 Resumen de Pines

### Display TFT ST7796 (480×320)
| Pin Display | GPIO ESP32 | Función |
|-------------|------------|---------|
| CS | 15 | Chip Select |
| RESET | 17 | Reset |
| DC/RS | 16 | Data/Command |
| MOSI (SDI) | 13 | Datos SPI |
| SCK | 14 | Reloj SPI |
| LED | 42 | Backlight |
| T_CS | 21 | Touch CS |

### CAN-Bus TJA1051
| ESP32 | TJA1051 |
|-------|---------|
| GPIO 4 | Pin 1 (TXD) |
| GPIO 5 | Pin 4 (RXD) |
| 5V | Pin 3 (VCC) |
| GND | Pin 2, 8 (GND, S) |

---

## 🔗 Documentos Relacionados

- `ESP32_STM32_CAN_CONNECTION.md` - Conexión CAN entre ESP32 y STM32
- `PINOUT_DEFINITIVO.md` - Pinout completo del STM32G474RE
- `CONEXIONES_COMPLETAS.md` - Conexiones del sistema completo
- `esp32/platformio.ini` - Configuración de compilación con defines de pines
- `esp32/src/main.cpp` - Código fuente principal del firmware ESP32

---

## 📦 Para Convertir a PDF

Estos documentos están en formato Markdown y pueden convertirse a PDF usando:

### Opción 1: Pandoc (Linux/Mac/Windows)
```bash
pandoc ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md -o conexiones_esp32.pdf
pandoc DIAGRAMA_PINES_VISUAL.md -o diagramas_visuales.pdf
pandoc CONEXIONES_RAPIDAS_ESP32.md -o referencia_rapida.pdf
```

### Opción 2: Herramientas Online
- [Markdown to PDF](https://www.markdowntopdf.com/)
- [Dillinger.io](https://dillinger.io/) (exportar como PDF)
- [CloudConvert](https://cloudconvert.com/md-to-pdf)

### Opción 3: Visual Studio Code
1. Instalar extensión "Markdown PDF"
2. Abrir archivo .md
3. Ctrl+Shift+P → "Markdown PDF: Export (pdf)"

---

## ✅ Estado de la Documentación

- [x] Conexiones de Display TFT documentadas
- [x] Conexiones de Touch Panel documentadas
- [x] Configuración SPI documentada
- [x] Conexiones CAN-Bus documentadas
- [x] Diagramas visuales creados
- [x] Referencia rápida creada
- [x] Lista de materiales incluida
- [x] Procedimiento de verificación incluido
- [x] Troubleshooting incluido

---

## 📞 Soporte

**Repositorio:** [florinzgz/STM32-Control-Coche-Marcos](https://github.com/florinzgz/STM32-Control-Coche-Marcos)

**Issues:** Para reportar problemas o solicitar aclaraciones sobre la documentación

---

_Última actualización: 2026-02-19_  
_Versión: 1.0_
