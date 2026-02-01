# 🚀 Guía de Inicio Rápido - STM32G474RE Control

Esta guía te ayudará a configurar y compilar el firmware de control para el STM32G474RE en pocos pasos.

## ⚡ Inicio Rápido (5 minutos)

### Paso 1: Requisitos Previos

**Software necesario:**
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (recomendado) **O**
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) + [ARM GCC Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)

**Hardware necesario:**
- Placa NUCLEO-G474RE (recomendada) o STM32G474RE personalizada
- Cable USB para programación
- (Opcional) Transreceptor CAN TJA1051T/3 para conectar con ESP32-S3

### Paso 2: Clonar el Repositorio

```bash
git clone https://github.com/florinzgz/STM32-Control-Coche-Marcos.git
cd STM32-Control-Coche-Marcos
```

### Paso 3A: Compilar con STM32CubeIDE (Recomendado)

1. Abrir STM32CubeIDE
2. **File → Open Projects from File System**
3. Seleccionar la carpeta del repositorio
4. El proyecto se importará automáticamente
5. Hacer clic derecho en el proyecto → **Build Project** (Ctrl+B)
6. Conectar la placa NUCLEO-G474RE
7. Hacer clic en **Run → Debug** (F11) para flashear

### Paso 3B: Compilar con Makefile

```bash
# Instalar drivers HAL primero (ver Paso 4)

# Compilar
make

# El binario estará en build/STM32G474RE-Control.bin
```

### Paso 4: Instalar STM32 HAL Drivers

Los drivers HAL no están incluidos en el repositorio. Tienes dos opciones:

#### Opción A: Automática (Recomendada)

1. Abrir `STM32G474RE-Control.ioc` con STM32CubeMX
2. Click en **Project → Generate Code** (Alt+K)
3. Los drivers se descargarán automáticamente a `Drivers/`

#### Opción B: Manual

1. Descargar [STM32CubeG4](https://www.st.com/en/embedded-software/stm32cubeg4.html)
2. Extraer el archivo
3. Copiar las siguientes carpetas a `Drivers/`:
   - `STM32Cube_FW_G4_VX.X.X/Drivers/STM32G4xx_HAL_Driver/`
   - `STM32Cube_FW_G4_VX.X.X/Drivers/CMSIS/`

### Paso 5: Verificar la Compilación

Si todo está correcto, deberías ver:

```
arm-none-eabi-size build/STM32G474RE-Control.elf
   text    data     bss     dec     hex filename
  45678    1234    5678   52590    cd6e build/STM32G474RE-Control.elf
```

## 📋 Estructura del Proyecto

```
STM32-Control-Coche-Marcos/
├── Core/
│   ├── Inc/                    # Headers
│   │   ├── main.h
│   │   ├── motor_control.h
│   │   ├── can_handler.h
│   │   ├── sensor_manager.h
│   │   └── safety_system.h
│   └── Src/                    # Código fuente
│       ├── main.c
│       ├── motor_control.c
│       ├── can_handler.c
│       ├── sensor_manager.c
│       └── safety_system.c
├── Drivers/                    # HAL drivers (no incluidos, ver Paso 4)
├── docs/                       # Documentación
├── STM32G474RE-Control.ioc     # Configuración STM32CubeMX
├── Makefile                    # Build con make
└── README.md
```

## 🔧 Configuración Inicial

### Verificar Configuración de Pines

El archivo `.ioc` ya incluye la configuración completa de pines para:
- **PWM**: 4 motores de tracción + 1 motor de dirección (TIM1, TIM8)
- **CAN**: FDCAN1 @ 500 kbps (PB8/PB9)
- **I2C**: Sensores de corriente INA226 (PB6/PB7)
- **Encoder**: Dirección incremental (PA0/PA1, TIM2)
- **ADC**: Pedal analógico (PA3)
- **GPIO**: Relés, enable, dirección

Ver `docs/PINOUT.md` para detalles completos.

### Configurar Comunicación CAN con ESP32-S3

Si vas a conectar el STM32G474RE con un ESP32-S3 vía CAN:

1. Leer `docs/ESP32_STM32_CAN_CONNECTION.md`
2. Obtener 2× transreceptores TJA1051T/3
3. Conectar según el diagrama en la documentación
4. Verificar protocolo CAN en `docs/CAN_PROTOCOL.md`

## 🐛 Solución de Problemas

### Error: "cannot find stm32g4xx_hal.h"

**Causa**: Los drivers HAL no están instalados.

**Solución**: Seguir el Paso 4 para instalar los drivers HAL.

### Error: "arm-none-eabi-gcc: command not found"

**Causa**: ARM GCC Toolchain no está instalado o no está en el PATH.

**Solución**: 
- **Windows**: Instalar STM32CubeIDE (incluye toolchain)
- **Linux/Mac**: `sudo apt install gcc-arm-none-eabi` o descargar desde ARM

### Error al compilar: "No rule to make target 'Core/Src/main.c'"

**Causa**: El Makefile no encuentra los archivos fuente.

**Solución**: Verificar que estás en el directorio raíz del proyecto.

### El firmware no arranca en la placa

**Posibles causas**:
1. Configuración de reloj incorrecta → Verificar en `.ioc` que PLL está configurado para 170 MHz
2. Watchdog demasiado corto → Verificar timeout IWDG en main.c
3. Falta inicialización HAL → Verificar que `HAL_Init()` se llama en main.c

## 📚 Documentación Adicional

### Documentación del Proyecto

| Documento | Descripción |
|-----------|-------------|
| [README.md](../README.md) | Visión general del proyecto |
| [PROJECT_STATUS.md](../PROJECT_STATUS.md) | Estado actual del desarrollo |
| [docs/PINOUT.md](PINOUT.md) | Configuración completa de pines |
| [docs/CAN_PROTOCOL.md](CAN_PROTOCOL.md) | Protocolo CAN ESP32↔STM32 |
| [docs/MOTOR_CONTROL.md](MOTOR_CONTROL.md) | Control de motores |
| [docs/SAFETY_SYSTEMS.md](SAFETY_SYSTEMS.md) | Sistemas de seguridad ABS/TCS |
| [docs/BUILD_GUIDE.md](BUILD_GUIDE.md) | Guía de compilación avanzada |
| [docs/ESP32_STM32_CAN_CONNECTION.md](ESP32_STM32_CAN_CONNECTION.md) | Conexión CAN con ESP32-S3 |

### Repositorio Relacionado

- **ESP32-S3 HMI**: [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos) - Firmware de interfaz gráfica y HMI

## 🎯 Próximos Pasos

Después de compilar exitosamente:

1. **Configurar Hardware**: Conectar sensores, motores y relés según `docs/PINOUT.md`
2. **Calibrar Sensores**: 
   - Encoder de dirección (centro)
   - Sensores de corriente INA226 (offsets)
   - Temperaturas DS18B20 (ROMs)
3. **Probar CAN**: Conectar con ESP32-S3 y verificar heartbeat
4. **Testing**: Probar cada subsistema individualmente
   - PWM motores
   - Lectura de sensores
   - Comunicación CAN
   - Sistemas de seguridad

## ⚠️ Notas Importantes

- **Seguridad**: Este es un sistema de control vehicular. Probar en banco antes de instalación final.
- **Calibración**: Los valores de PID y thresholds de seguridad deben ajustarse según el hardware real.
- **HAL Drivers**: Asegurar que la versión de HAL sea compatible (v1.2.0 o superior).
- **Watchdog**: El IWDG está configurado para 500 ms. Alimentarlo regularmente en el bucle principal.

## 📞 Soporte

- **Issues**: [GitHub Issues](https://github.com/florinzgz/STM32-Control-Coche-Marcos/issues)
- **Autor**: Florin Zgureanu (@florinzgz)

## 📄 Licencia

MIT License - Ver [LICENSE](../LICENSE) para detalles.

---

**¿Todo listo?** Si has completado estos pasos, tu entorno de desarrollo está configurado y puedes empezar a trabajar con el firmware de control STM32G474RE. ¡Buena suerte! 🚗⚡
