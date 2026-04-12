# 🔨 Guía de Compilación y Deployment

**Compilación, Flasheo y Debugging del Firmware STM32**

---

## 📋 Tabla de Contenidos

1. [Requisitos Previos](#-requisitos-previos)
2. [Instalación de Herramientas](#-instalación-de-herramientas)
3. [Configuración del Proyecto](#-configuración-del-proyecto)
4. [Compilación](#-compilación)
5. [Flasheo del Firmware](#-flasheo-del-firmware)
6. [Debugging](#-debugging)
7. [Solución de Problemas](#-solución-de-problemas)

---

## ✅ Requisitos Previos

### Hardware Necesario

| Componente | Especificación | Notas |
|------------|----------------|-------|
| **MCU** | STM32G474RE (NUCLEO-G474RE) | ARM Cortex-M4F @ 170 MHz |
| **Programador** | ST-Link V2/V3 | Incluido en NUCLEO board |
| **Cable USB** | USB-A a Mini/Micro-USB | Para ST-Link |
| **Fuente 5V** | 2A mínimo | Alimentación externa si se usan motores |
| **Transceptor CAN** | TJA1051T/3 (×2) | High-speed CAN transceiver (NXP) — VCC=5V, VIO=3.3V |
| **Resistencias CAN** | 2× 120Ω | Terminación del bus CAN |

### Software Necesario

| Software | Versión Mínima | Propósito |
|----------|----------------|-----------|
| **STM32CubeIDE** | 1.14.0 | IDE, compilador, debugger |
| **STM32CubeMX** | 6.17.0 | Generador de código de inicialización |
| **Git** | 2.30+ | Control de versiones |
| **ST-Link Utility** | 4.6+ (opcional) | Flasheo standalone |

---

## 🛠️ Instalación de Herramientas

### 1. Instalar STM32CubeIDE

#### Windows
1. Descargar desde: [https://www.st.com/en/development-tools/stm32cubeide.html](https://www.st.com/en/development-tools/stm32cubeide.html)
2. Ejecutar instalador `st-stm32cubeide_<version>_windows_x64.exe`
3. Seguir asistente (instalar drivers ST-Link incluidos)
4. Reiniciar PC

#### Linux (Ubuntu/Debian)
```bash
# Descargar instalador
wget https://www.st.com/content/ccc/resource/technical/software/sw_development_suite/group0/fa/5b/11/21/e9/f6/4b/5e/stm32cubeide_deb/files/st-stm32cubeide_<version>_amd64.deb

# Instalar dependencias
sudo apt update
sudo apt install libusb-1.0-0 libwebkit2gtk-4.0-37

# Instalar STM32CubeIDE
sudo dpkg -i st-stm32cubeide_*.deb

# Configurar udev rules para ST-Link
sudo cp /opt/st/stm32cubeide_*/resource/com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.linux64_*/config/udev/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

#### macOS
```bash
# Descargar desde web de ST
# Instalar .dmg arrastrando a /Applications

# Instalar Homebrew si no está instalado
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Instalar dependencias
brew install libusb
```

### 2. Instalar Git

```bash
# Windows (usar Git for Windows)
# Descargar desde: https://git-scm.com/download/win

# Linux
sudo apt install git

# macOS
brew install git
```

### 3. Verificar Instalación

```bash
# Verificar STM32CubeIDE
st-flash --version

# Verificar Git
git --version
```

---

## 📦 Configuración del Proyecto

### 1. Clonar el Repositorio

```bash
# Clonar desde GitHub
git clone https://github.com/florinzgz/STM32-Control-Coche-Marcos.git
cd STM32-Control-Coche-Marcos
```

### 2. Importar en STM32CubeIDE

#### Método 1: Importar Proyecto Existente
1. Abrir STM32CubeIDE
2. `File → Open Projects from File System...`
3. Seleccionar carpeta raíz del repositorio
4. Click `Finish`

#### Método 2: Crear Proyecto Nuevo (si no existe .project)
1. `File → New → STM32 Project`
2. Seleccionar MCU: **STM32G474RE**
3. Nombre del proyecto: `STM32-Control-Coche-Marcos`
4. Click `Finish`
5. Copiar archivos del repositorio a la carpeta del proyecto

### 3. Configurar Pinout en STM32CubeMX

Si el proyecto no tiene configuración `.ioc`, crear una nueva:

1. Abrir STM32CubeMX desde STM32CubeIDE
2. Configurar pines según [docs/PINOUT.md](PINOUT.md):

```
Pinout & Configuration:
├─ TIM1 (Internal Clock, PWM Generation CH1-CH4)
│  ├─ CH1: PA8 (PWM_FL)
│  ├─ CH2: PA9 (PWM_FR)
│  ├─ CH3: PA10 (PWM_RL)
│  └─ CH4: PA11 (PWM_RR)
├─ TIM8 (Internal Clock, PWM Generation CH3)
│  └─ CH3: PC8 (PWM_STEER)
├─ TIM2 (Encoder Mode)
│  ├─ CH1: PA15 (ENC_A)
│  └─ CH2: PB3 (ENC_B)
├─ FDCAN1
│  ├─ TX: PB9
│  └─ RX: PB8
├─ I2C1
│  ├─ SCL: PB6
│  └─ SDA: PB7
├─ ADC1
│  └─ IN1: PA0 (PEDAL)
└─ GPIOs
   ├─ PB0-PB2, PB10 (Sensores rueda)
   ├─ PC0-PC7, PC9-PC10 (Control motores)
   ├─ PC11-PC12, PD2 (Relés)
   ├─ PB12-PB13 (Shifter), PB14 (LED_DIAG)
   └─ PB5 (OneWire temperatura)
```

3. Generar código: `Project → Generate Code`

### 4. Añadir Archivos de Código

Copiar archivos del repositorio a las carpetas correspondientes:

```
STM32-Control-Coche-Marcos/
├─ Core/
│  ├─ Inc/
│  │  ├─ main.h
│  │  ├─ motor_control.h
│  │  ├─ can_handler.h
│  │  ├─ sensor_manager.h
│  │  ├─ safety_system.h
│  │  └─ stm32g4xx_it.h
│  └─ Src/
│     ├─ main.c
│     ├─ motor_control.c
│     ├─ can_handler.c
│     ├─ sensor_manager.c
│     ├─ safety_system.c
│     └─ stm32g4xx_it.c
└─ docs/
   └─ (archivos de documentación)
```

---

## 🔨 Compilación

### Desde STM32CubeIDE (GUI)

1. **Build Debug:**
   - Click en `Project → Build Project` (Ctrl+B)
   - O click en icono 🔨 en toolbar

2. **Build Release:**
   - Click derecho en proyecto → `Build Configurations → Set Active → Release`
   - `Project → Build Project`

### Desde Terminal (Makefile)

Si el proyecto tiene `Makefile` generado:

```bash
# Compilar Debug
make clean
make all

# Compilar Release
make clean
make BUILD_TYPE=Release all

# Compilar con máximo nivel de optimización
make OPTIMIZATION=-O3 all
```

### Verificar Compilación Exitosa

Salida esperada:

```
arm-none-eabi-size build/STM32-Control-Coche-Marcos.elf
   text    data     bss     dec     hex filename
  45632    2048   12288   59968    ea20 build/STM32-Control-Coche-Marcos.elf

Finished building: default.size.stdout
```

**Límites del STM32G474RE:**
- **Flash:** 512 KB (525,288 bytes)
- **RAM:** 128 KB (131,072 bytes)

---

## 📥 Flasheo del Firmware

### Desde STM32CubeIDE

1. Conectar NUCLEO-G474RE vía USB (ST-Link)
2. Click en `Run → Debug` (F11)
3. O `Run → Run` (Ctrl+F11) para flashear sin debug

### Desde Terminal (ST-Link CLI)

#### Windows
```cmd
"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" ^
  -c port=SWD ^
  -w build\STM32-Control-Coche-Marcos.bin 0x08000000 ^
  -v -rst
```

#### Linux/macOS
```bash
st-flash write build/STM32-Control-Coche-Marcos.bin 0x08000000

# O usando STM32CubeProgrammer
/opt/st/stm32cubeide_1.14.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.0.0/tools/bin/STM32_Programmer_CLI \
  -c port=SWD \
  -w build/STM32-Control-Coche-Marcos.bin 0x08000000 \
  -v -rst
```

### Verificar Flasheo

1. LED verde del NUCLEO debe parpadear
2. Verificar salida UART (si está configurado):

```bash
# Linux
sudo minicom -D /dev/ttyACM0 -b 115200

# macOS
screen /dev/tty.usbmodemXXXXX 115200

# Windows (usar PuTTY o TeraTerm)
```

---

## 🐞 Debugging

### Configuración de Debug

1. Click derecho en proyecto → `Debug As → Debug Configurations...`
2. Crear nueva configuración `STM32 C/C++ Application`
3. Configurar:
   - **C/C++ Application:** `Debug/STM32-Control-Coche-Marcos.elf`
   - **Debugger:**
     - Debug probe: ST-Link (ST-Link GDB server)
     - Interface: SWD
     - Reset Mode: Software system reset
   - **Startup:**
     - ☑ Load symbols
     - ☑ Load executable
     - ☑ Set breakpoint at: `main`

### Breakpoints y Watchpoints

```c
// Ejemplo: Breakpoint condicional en main.c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    // Breakpoint aquí para verificar inicialización
    __asm("NOP");  // <-- Click izquierdo en número de línea
    
    while (1) {
        // Watchpoint en variable crítica
        IWDG_Refresh();  // Observar si se ejecuta
    }
}
```

### Live Expressions

Monitorear variables en tiempo real:

1. `Window → Show View → Live Expressions`
2. Añadir expresiones:
   - `motor_FL.power_pct`
   - `wheel_speed.speed_avg`
   - `temperature.temp_max`

### SWV (Serial Wire Viewer)

Visualizar printf() sin UART:

1. En `Debug Configurations → Debugger → Serial Wire Viewer (SWV)`:
   - ☑ Enable
   - Core Clock: 170 MHz
2. Añadir en código:

```c
#include <stdio.h>

int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar((*ptr++));
    }
    return len;
}

// Usar printf() normalmente
printf("Velocidad: %d mm/s\n", wheel_speed.speed_avg);
```

3. Durante debug: `Window → Show View → SWV → ITM Data Console`

---

## 🔍 Solución de Problemas

### Problema: No se detecta ST-Link

**Síntomas:** `Error: No ST-Link detected`

**Soluciones:**
1. Verificar cable USB (usar cable de datos, no solo carga)
2. Reinstalar drivers ST-Link:
   - Windows: Ejecutar `dpinst_amd64.exe` desde `STM32CubeIDE/drivers/`
   - Linux: Verificar udev rules (ver sección instalación)
3. Verificar LED rojo en NUCLEO (debe estar encendido)
4. Probar otro puerto USB o cable

### Problema: Error de Compilación "Region FLASH Overflowed"

**Síntomas:** `region 'FLASH' overflowed by XXXX bytes`

**Soluciones:**
1. Cambiar a modo Release (optimización -O2)
2. Reducir tamaño de buffers:
```c
// Antes
uint8_t large_buffer[10000];

// Después
uint8_t large_buffer[1000];
```
3. Habilitar Link Time Optimization (LTO):
   - `Project Properties → C/C++ Build → Settings → MCU GCC Linker → Optimization`
   - ☑ Link-time optimizer (-flto)

### Problema: Hard Fault durante ejecución

**Síntomas:** Programa entra en `HardFault_Handler`

**Depuración:**
1. Activar FPU (Floating Point Unit):
```c
// En SystemInit() o main()
SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // CP10 y CP11 Full Access
```

2. Verificar stack overflow:
```c
// En stm32g4xx_it.c
void HardFault_Handler(void) {
    __asm volatile (
        "TST LR, #4 \n"
        "ITE EQ \n"
        "MRSEQ R0, MSP \n"
        "MRSNE R0, PSP \n"
        "B hard_fault_handler_c \n"
    );
}

void hard_fault_handler_c(uint32_t *hardfault_args) {
    volatile uint32_t stacked_r0 = hardfault_args[0];
    volatile uint32_t stacked_r1 = hardfault_args[1];
    volatile uint32_t stacked_pc = hardfault_args[6];
    
    // Breakpoint aquí para inspeccionar
    while(1);
}
```

### Problema: CAN no funciona

**Verificar:**
1. **Hardware:**
   - Resistencias 120Ω en ambos extremos del bus
   - CANH y CANL conectados correctamente
   - Alimentación del transceptor TJA1051T/3: VCC (pin 3) = **5 V**, VIO (pin 5) = **3.3 V** ⚠️ OBLIGATORIO

2. **Software:**
```c
// Verificar configuración de pines
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // PA11=RX, PA12=TX

// Verificar bitrate
hfdcan1.NominalPrescaler = 20;  // 500 kbps @ 170 MHz
```

3. **Loopback Test:**
```c
// Modo loopback para test sin hardware
hfdcan1.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
```

### Problema: I2C Timeout

**Síntomas:** `HAL_I2C_Master_Transmit()` retorna `HAL_TIMEOUT`

**Soluciones:**
1. Verificar pull-ups (4.7kΩ en SCL y SDA)
2. Reducir velocidad I2C:
```c
hi2c1.Init.Timing = 0x10909CEC;  // 100 kHz en lugar de 400 kHz
```
3. Verificar dirección I2C del slave (scan):
```c
for (uint8_t addr = 0; addr < 128; addr++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
        printf("Device found at 0x%02X\n", addr);
    }
}
```

---

## 📊 Métricas de Compilación

### Tamaño de Código Típico

| Configuración | Flash (KB) | RAM (KB) | Velocidad |
|---------------|------------|----------|-----------|
| **Debug (-Og)** | ~80 | ~20 | Lenta |
| **Release (-O2)** | ~45 | ~15 | Media |
| **Release (-O3 + LTO)** | ~40 | ~15 | Rápida |

### Tiempo de Compilación

| Sistema | Debug (clean) | Release (clean) | Incremental |
|---------|---------------|-----------------|-------------|
| **Workstation (i7)** | ~15s | ~25s | ~5s |
| **Laptop (i5)** | ~30s | ~45s | ~10s |
| **CI/CD Server** | ~20s | ~35s | - |

---

## 📖 Referencias

- [STM32CubeIDE User Manual](https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-guide-stmicroelectronics.pdf)
- [Getting Started with STM32G4](https://www.st.com/resource/en/application_note/an5044-getting-started-with-stm32g4-series-hardware-development-stmicroelectronics.pdf)
- [ST-Link User Manual](https://www.st.com/resource/en/user_manual/um2448-stlink-v3set-debuggerprogrammer-for-stm8-and-stm32-stmicroelectronics.pdf)

---

**Última actualización:** 2026-02-01  
**Autor:** florinzgz  
**Proyecto:** STM32-Control-Coche-Marcos
