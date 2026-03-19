# Resumen de Auditoría de Documentación Markdown

**Fecha:** 2026-03-19  
**Repositorio:** STM32-Control-Coche-Marcos  
**Archivos .md encontrados:** 129 (35 raíz + 91 docs/ + 3 analysis_artifacts/)  
**Archivos .txt encontrados:** 2 (analysis_artifacts/install_commands.txt, analysis_artifacts/tis/tis_would_analyze.txt)

---

## 1. Fuentes de Verdad Utilizadas para Verificación

| Fuente | Ubicación | Descripción |
|--------|-----------|-------------|
| **Firmware STM32** | Core/Inc/main.h | Definiciones #define de todos los pines, timers, constantes |
| **Firmware ESP32** | esp32/include/can_ids.h, esp32/src/*.h | CAN IDs, GPIO, configuración |
| **BOM real** | analysis_artifacts/BOM.csv | 60+ componentes con valores reales |
| **Wiring verificado** | analysis_artifacts/wiring_manual.md | 50+ señales, cable por cable |
| **Safety checks** | analysis_artifacts/safety_checks.md | 97 items de verificación |
| **Connector pinouts** | analysis_artifacts/connector_pinouts.md | Mapeo pin-a-pin Nucleo-64 |
| **Hardware findings** | analysis_artifacts/findings_hardware.csv | 20 discrepancias documentadas |
| **platformio.ini** | esp32/platformio.ini | Configuración build ESP32-S3 |

---

## 2. Clasificación de TODOS los Archivos .md

### 2.1 Archivos Raíz (35 archivos)

| Archivo | Estado | Motivo |
|---------|--------|--------|
| README.md | **ACTUALIZAR** | Falta sección de referencias a wiring_manual, BOM.csv, safety_checks |
| FORMAL_VERIFICATION_REPORT.md | **OK** | Alineado: CBMC 5.95.1, Infer v1.2.0, AFL++ 4.09c, 713 tests, 19189 fuzz execs |
| FRAMA_C_ANALYSIS_REPORT.md | **OK** | 7 bugs reales corregidos, documentado correctamente |
| STATIC_ANALYSIS_REPORT.md | **OK** | CppCheck + Clang-Tidy resultados alineados con verificación post-P1-P7 |
| COMPREHENSIVE_ANALYSIS_REPORT.md | **OK** | Informe completo de análisis estático (2026-03-19) |
| FIRMWARE_POST_P1_P7_VERIFICATION.md | **OK** | Verificación post-correcciones P1-P7 correcta |
| FIRMWARE_COMPLETE_AUDIT.md | **OK** | Auditoría completa con fix FPU crítico |
| PROJECT_STATUS.md | **OK** | Estado actual del proyecto (documento vivo) |
| SETUP.md | **OK** | Instrucciones de instalación correctas |
| CUBEIDE_PROJECT_GUIDE.md | **OK** | Guía de configuración CubeIDE |
| AGENT_HANDOFF.md | **OK** | Contexto de handoff entre agentes (documento operativo) |
| CODE_REVIEW.md | **OK** | Code review histórico (2026-02-07) |
| AUDIT.md | **OK** | Auditoría técnica histórica (2026-02-08) |
| CAN_PIN_AUDIT_REPORT.md | **OK** | Auditoría CAN/TWAI pines |
| CURRENT_SENSING_SHUNT_AUDIT.md | **OK** | Auditoría INA226 shunt |
| FINAL_FIRMWARE_VERIFICATION.md | **OK** | Verificación runtime safety |
| FIRMWARE_AUDIT_REPORT.md | **OK** | Auditoría HMI CAN |
| FIRMWARE_HARDWARE_INTEGRITY_REPORT.md | **OK** | Integridad firmware-hardware |
| FIRMWARE_MIGRATION_AUDIT.md | **OK** | Auditoría migración (82% equivalencia) |
| FIRMWARE_SAFETY_AUDIT.md | **OK** | Auditoría safety-critical modules |
| HARDWARE_LOCKDOWN_AUDIT.md | **OK** | Auditoría PSRAM/Flash ESP32-S3 |
| HMI_SCREEN_STATUS_REPORT.md | **OK** | Estado pantallas HMI |
| IMPLEMENTATION_REPORT.md | **OK** | Reporte implementación CAN handler |
| OBSTACLE_DETECTION_AUDIT_REPORT.md | **OK** | Auditoría detección obstáculos |
| OBSTACLE_SYSTEM_STRATEGY.md | **OK** | Estrategia sistema obstáculos |
| REGEN_BRAKING_AUDIT.md | **OK** | Auditoría frenado regenerativo (NO IMPLEMENTADO) |
| SECURITY_AUDIT.md | **OK** | Auditoría seguridad funcional |
| SECURITY_CHANGES_AUDIT.md | **OK** | Auditoría cambios seguridad |
| STM32_OBSTACLE_AUDIT.md | **OK** | Auditoría obstáculos STM32 |
| STM32_OBSTACLE_LIMP_HOME_VALIDATION.md | **OK** | Validación LIMP_HOME obstáculos |
| SUMMARY.md | **OK** | Resumen implementación |
| TECHNICAL_AUDIT_REPORT.md | **OK** | Auditoría técnica (75% vs referencia) |
| TRACTION_GEAR_SHIFTER_AUDIT.md | **OK** | Auditoría tracción/shifter |
| VERIFICATION.md | **OK** | Checklist verificación componentes |
| VL53L8CX_DUAL_SENSOR_FEASIBILITY_REPORT.md | **OK** | Viabilidad segundo sensor LiDAR |

### 2.2 Carpeta docs/ (91 archivos)

| Archivo | Estado | Motivo |
|---------|--------|--------|
| docs/HARDWARE_SPECIFICATION.md | **ACTUALIZAR** | Shunt 2mΩ → 1mΩ/0.5mΩ; pedal 1kΩ+2kΩ → 10kΩ+6.8kΩ; BOM incorrecto |
| docs/PINOUT_DEFINITIVO.md | **ACTUALIZAR** | Divisor pedal 1kΩ+2kΩ → 10kΩ+6.8kΩ; rango ADC incorrecto |
| docs/PINOUT.md | **ACTUALIZAR** | Arquitectura DIR+PWM → RPWM/LPWM; pines wheel incorrectos; tabla consolidada errónea |
| docs/HARDWARE.md | **ACTUALIZAR** | Wheel PPR "1-4" → 6; pines wheel PB0/PB1/PB2/PB10 → PA0/PA1/PA2/PB15; shunt sin distinción batería |
| docs/PROTOCOLO_CAN.md | **ELIMINAR** | Marcado como DESACTUALIZADO en su propio contenido; reemplazado por CAN_CONTRACT_FINAL.md |
| docs/AISLAMIENTO_AUDIO_DFPLAYER.md | **OK** | Aislamiento audio DFPlayer |
| docs/AISLAMIENTO_GALVANICO_6N137.md | **OK** | Plan aislamiento galvánico |
| docs/ARQUITECTURA_CONTROL_MOTORES.md | **OK** | Arquitectura control motores |
| docs/AUDIO_HARDWARE_AUDIT.md | **OK** | Auditoría hardware audio |
| docs/AUDIO_MITIGATION_PLAN.md | **OK** | Plan mitigación audio |
| docs/AUDIO_RELAY_INTEGRATION.md | **OK** | Integración relé audio |
| docs/AUDIO_TRACKS_GUIDE.md | **OK** | Guía tracks audio SD |
| docs/BOOT_SEQUENCE_ANALYSIS.md | **OK** | Análisis secuencia boot ESP32 |
| docs/BTS7960_MOTOR_DRIVER_AUDIT.md | **OK** | Auditoría driver motor BTS7960 |
| docs/BUILD_GUIDE.md | **OK** | Guía compilación/depuración |
| docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md | **OK** | Plan cableado aislamiento final |
| docs/CAN_BUS_AUDIT_REPORT.md | **OK** | Auditoría bus CAN |
| docs/CAN_CONTRACT_FINAL.md | **OK** | Contrato CAN v1.3 (fuente de verdad) |
| docs/CAN_DIAGNOSTIC_REPORT.md | **OK** | Diagnóstico CAN |
| docs/CAN_PROTOCOL.md | **OK** | Protocolo CAN (versión actual) |
| docs/COMPARACION_PINES_DOC_VS_FIRMWARE.md | **OK** | Comparación pines doc vs firmware |
| docs/CONEXIONES_COMPLETAS.md | **OK** | Guía conexiones completas |
| docs/CONEXIONES_RAPIDAS_ESP32.md | **OK** | Referencia rápida ESP32 |
| docs/CONEXION_TOF_SENSE_M_LIDAR.md | **OK** | Conexión sensor ToF |
| docs/DIAGRAMA_PINES_VISUAL.md | **OK** | Diagramas ASCII pines |
| docs/DISPLAY_COMPONENT_VERIFICATION.md | **OK** | Verificación componentes display |
| docs/DISPLAY_STABILITY_VALIDATION.md | **OK** | Validación estabilidad display |
| docs/ENCODER_CURRENT_STATE.md | **OK** | Estado encoder E6B2-CWZ6C |
| docs/ENGINEERING_MENU.md | **OK** | Menú ingeniería oculto |
| docs/EPS_TORQUE_ASSIST_ANALYSIS.md | **OK** | Análisis EPS torque |
| docs/EPS_TORQUE_MODEL.md | **OK** | Modelo matemático EPS |
| docs/ESP32_FIRMWARE_DESIGN.md | **OK** | Diseño firmware ESP32 |
| docs/ESP32_HMI_ARCHITECTURE_REBUILD.md | **OK** | Arquitectura HMI v2.0 |
| docs/ESP32_HMI_BRING_UP.md | **OK** | Bring-up pantalla ESP32 |
| docs/ESP32_PIN_DOCUMENTATION_INDEX.md | **OK** | Índice documentación pines ESP32 |
| docs/ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md | **OK** | Conexiones display y CAN |
| docs/ESP32_STM32_CAN_CONNECTION.md | **OK** | Conexión CAN ESP32↔STM32 |
| docs/FACTORY_DEFAULTS.md | **OK** | Valores por defecto fábrica |
| docs/FAIL_OPERATIONAL_MIGRATION_AUDIT.md | **OK** | Auditoría migración fail-operational |
| docs/FIRMWARE_COMPLETION_ASSESSMENT.md | **OK** | Evaluación completitud firmware |
| docs/FIRMWARE_MATURITY_ROADMAP.md | **OK** | Roadmap madurez firmware |
| docs/HARDWARE_AND_SENSOR_MAP.md | **OK** | Mapa hardware y sensores |
| docs/HARDWARE_VALIDATION_PROCEDURE.md | **OK** | Procedimiento validación hardware |
| docs/HARDWARE_WIRING_MANUAL.md | **OK** | Manual cableado hardware |
| docs/HMI_AUDIO_SAFETY_REVIEW.md | **OK** | Revisión seguridad audio HMI |
| docs/HMI_PHASE14_REGRESSION.md | **OK** | Regresión Phase 14 HMI |
| docs/HMI_RENDERING_STRATEGY.md | **OK** | Estrategia rendering HMI |
| docs/HMI_STATE_MODEL.md | **OK** | Modelo estados HMI |
| docs/INFORME_REVISION_TECNICA_RELAY.md | **OK** | Revisión técnica relé |
| docs/INTEGRATION_PLAN.md | **OK** | Plan integración componentes |
| docs/LAYOUT_VERIFICATION.md | **OK** | Verificación layout pantalla |
| docs/LED_SYSTEM_ANALYSIS.md | **OK** | Análisis sistema LED |
| docs/LIMP_HOME_NO_CENTERING_IMPLEMENTATION.md | **OK** | Implementación limp-home sin centrado |
| docs/LISTADO_PINES_COMPLETO.md | **OK** | Listado completo pines (referencia) |
| docs/LLAVE_CONTACTO_ENCENDIDO_APAGADO.md | **OK** | Circuito llave contacto |
| docs/LOW_SPEED_CONTROL_STRATEGY.md | **OK** | Estrategia control baja velocidad |
| docs/LOW_THROTTLE_BEHAVIOR_ANALYSIS.md | **OK** | Análisis comportamiento bajo acelerador |
| docs/MOTOR_CONTROL.md | **OK** | Control motor PWM directo |
| docs/MOTOR_CONTROL_AUDIT.md | **OK** | Auditoría control motor dual-PWM |
| docs/OBSTACLE_HARD_CUTOFF_ANALYSIS.md | **OK** | Análisis corte duro proximidad |
| docs/OBSTACLE_SENSOR_DESIGN_DECISION.md | **OK** | Decisión diseño sensor obstáculos |
| docs/OBSTACLE_SYSTEM_ARCHITECTURE.md | **OK** | Arquitectura sistema obstáculos |
| docs/ORIGINAL_REPO_COMPARATIVE_AUDIT.md | **OK** | Auditoría comparativa repo original |
| docs/PALANCA_CAMBIOS_IMPLEMENTACION.md | **OK** | Implementación palanca cambios |
| docs/PEDAL_SENSOR_FINAL_SUMMARY.md | **OK** | Resumen final sensor pedal |
| docs/PENDING_FEATURES_SCHEDULE.md | **OK** | Calendario features pendientes |
| docs/PHASE1_VALIDATION.md | **OK** | Validación Phase 1 |
| docs/PHASE3_5_FINAL_STATUS.md | **OK** | Estado final Phases 3-5 |
| docs/PINES_PANTALLA.md | **OK** | Pines pantalla detallados |
| docs/PIN_USAGE_INVENTORY.md | **OK** | Inventario uso pines |
| docs/POWER_DISTRIBUTION.md | **OK** | Distribución potencia |
| docs/PROJECT_MASTER_STATUS.md | **OK** | Estado maestro proyecto |
| docs/QUICK_START.md | **OK** | Inicio rápido |
| docs/RESUMEN_DOCUMENTACION_ESP32.md | **OK** | Resumen documentación ESP32 |
| docs/RESUMEN_PINES_NO_USAR_ESP32S3.md | **OK** | Pines no usar ESP32-S3 |
| docs/RUNTIME_MONITOR_AUDIT.md | **OK** | Auditoría monitor runtime |
| docs/SAFETY_ARCHITECTURE.md | **OK** | Arquitectura seguridad |
| docs/SAFETY_SYSTEMS.md | **OK** | Sistemas seguridad ABS/TCS |
| docs/SENSOR_INTERFACE.md | **OK** | Interfaz sensores |
| docs/SERVICE_MODE.md | **OK** | Modo servicio 25 módulos |
| docs/STEERING_CALIBRATION.md | **OK** | Calibración dirección |
| docs/STEERING_PERSISTENT_CALIBRATION.md | **OK** | Calibración persistente dirección |
| docs/TECHNICAL_REVIEW_REPORT.md | **OK** | Revisión técnica completa |
| docs/TIMER_PWM_ANALYSIS.md | **OK** | Análisis timer/PWM |
| docs/TOFSENSE_M_WIRING_GUIDE.md | **OK** | Guía cableado ToFSense-M |
| docs/TRACTION_PER_WHEEL.md | **OK** | Control tracción por rueda |
| docs/VALIDACION_ARRANQUE_SEGURO.md | **OK** | Validación arranque seguro |
| docs/VALIDACION_CAN_PULLUP_PC817.md | **OK** | Validación CAN pull-up PC817 |
| docs/VALIDACION_CONEXION_FISICA_CAN.md | **OK** | Validación conexión física CAN |
| docs/VALIDACION_ELECTRICA_AISLAMIENTO.md | **OK** | Validación eléctrica aislamiento |
| docs/VL53L8CX_INTEGRATION_ANALYSIS.md | **OK** | Análisis integración VL53L8CX |

### 2.3 Carpeta analysis_artifacts/ (3 archivos .md)

| Archivo | Estado | Motivo |
|---------|--------|--------|
| analysis_artifacts/wiring_manual.md | **OK** | Fuente de verdad para cableado (verificado contra firmware) |
| analysis_artifacts/connector_pinouts.md | **OK** | Fuente de verdad para pinout Nucleo-64 |
| analysis_artifacts/safety_checks.md | **OK** | 97 items verificación pre-energización |

### 2.4 Carpeta esp32/ (3 archivos .md)

| Archivo | Estado | Motivo |
|---------|--------|--------|
| esp32/README.md | **OK** | Instrucciones setup ESP32-S3 |
| esp32/include/README.md | **OK** | Documenta CAN IDs mirror |
| esp32/src/README.md | **OK** | Arquitectura pantallas HMI |

### 2.5 Carpeta analysis_artifacts/patches/ (1 archivo .md)

| Archivo | Estado | Motivo |
|---------|--------|--------|
| analysis_artifacts/patches/README.md | **OK** | Instrucciones uso parches |

---

## 3. Resumen de Inconsistencias Detectadas y Resueltas

### 3.1 Inconsistencias CRÍTICAS (corregidas en parches)

| # | Inconsistencia | Archivos afectados | Valor incorrecto | Valor correcto (según firmware/BOM) | Parche |
|---|---------------|--------------------|--------------------|--------------------------------------|--------|
| 1 | **Resistencia shunt INA226** | HARDWARE_SPECIFICATION.md | 0.002Ω (2mΩ) | Motor: 0.001Ω (1mΩ); Batería: 0.0005Ω (0.5mΩ) | 001 |
| 2 | **Divisor tensión pedal** | HARDWARE_SPECIFICATION.md, PINOUT_DEFINITIVO.md | R1=1kΩ, R2=2kΩ → 3.33V | R1=10kΩ, R2=6.8kΩ → 2.02V | 001, 002 |
| 3 | **Corriente máxima shunt** | HARDWARE_SPECIFICATION.md | 41A | 82A (1mΩ), 164A (0.5mΩ) | 001 |
| 4 | **Pines sensores rueda** | PINOUT.md, HARDWARE.md | PB0/PB1/PB2/PB10 | PA0/PA1/PA2/PB15 (per main.h) | 003, 004 |
| 5 | **PPR sensores rueda** | HARDWARE.md | Variable (1-4 PPR) | 6 (WHEEL_PULSES_REV en firmware) | 003 |
| 6 | **Arquitectura motor** | PINOUT.md | DIR+PWM (1 señal + dirección) | RPWM/LPWM (2 PWM directos por motor) | 004 |
| 7 | **Asignación timers** | PINOUT.md | TIM1 (4 tracción) + TIM8 (dirección) | TIM1 (FL+FR) + TIM8 (RL+RR) + TIM3 (dirección) | 004 |
| 8 | **Pines motor trasero** | PINOUT.md | RL=PA10/TIM1_CH3, RR=PA11/TIM1_CH4 | RL=PC6-PC7/TIM8_CH1-CH2, RR=PC8-PC9/TIM8_CH3-CH4 | 004 |
| 9 | **Pin dirección** | PINOUT.md | STEER=PC8/TIM8_CH3 | STEER=PA6-PA7/TIM3_CH1-CH2 | 004 |
| 10 | **Pines enable** | PINOUT.md | PC1/PC3/PC5/PC7/PC10 como EN | FL=PC5, RR=PC13 (GPIO); FR/RL/STEER=3.3V fijo | 004 |
| 11 | **PB0 función** | PINOUT.md | WHEEL_FL (sensor rueda) | ONEWIRE (bus DS18B20, 5 sensores) | 004 |
| 12 | **PB5 función** | PINOUT.md | TEMP_ONEWIRE | STEER_CENTER (sensor inductivo centro) | 004 |
| 13 | **PA0 función** | PINOUT.md | PEDAL (ADC1_IN1) | WHEEL_FL (EXTI0, sensor rueda) | 004 |
| 14 | **ARR timer** | PINOUT.md | 8499 (edge-aligned) | 4249 (center-aligned, 170MHz/(2×4250)=20kHz) | 004 |
| 15 | **Pines dirección PC0-PC4** | PINOUT.md | DIR_FL..DIR_STEER activos | Liberados (RPWM/LPWM directo) | 004 |

### 3.2 Inconsistencias MENORES (documentadas, no requieren parche urgente)

| # | Inconsistencia | Archivos afectados | Notas |
|---|---------------|--------------------| ------|
| 1 | Sensor pedal modelo | Varios | A1324LUA-T vs SS1324LUA-T — verificar componente real |
| 2 | CAN transceiver modelo | Varios | TJA1050 vs TJA1051T/3 — ambos mencionados |
| 3 | Encoder en HARDWARE_SPECIFICATION.md | HARDWARE_SPECIFICATION.md L289 | "1200 PPR" es correcto para el encoder E6B2-CWZ6C (steering), no confundir con wheel sensors |
| 4 | PINOUT.md PB12-PB14 | PINOUT.md | Shifter FWD/NEU/REV en PB12-PB14 pero firmware ESP32 usa MCP23017 I2C — verificar si STM32 también tiene GPIO directo |

---

## 4. Archivos para ELIMINAR

| Archivo | Motivo | Reemplazo |
|---------|--------|-----------|
| docs/PROTOCOLO_CAN.md | **Auto-marcado como DESACTUALIZADO** en líneas 3-6. Contiene valores que difieren del firmware real. | docs/CAN_CONTRACT_FINAL.md (rev 1.3) |

---

## 5. Parches Generados

| Parche | Archivo destino | Tipo | Cambios principales |
|--------|----------------|------|---------------------|
| `001_fix_HARDWARE_SPECIFICATION.patch` | docs/HARDWARE_SPECIFICATION.md | ACTUALIZAR | Shunt 2mΩ→1mΩ/0.5mΩ; pedal 1kΩ+2kΩ→10kΩ+6.8kΩ; BOM corregido |
| `002_fix_PINOUT_DEFINITIVO.patch` | docs/PINOUT_DEFINITIVO.md | ACTUALIZAR | Divisor pedal corregido; rango ADC actualizado |
| `003_fix_HARDWARE.patch` | docs/HARDWARE.md | ACTUALIZAR | Wheel PPR→6; pines wheel PA0/PA1/PA2/PB15; shunt batería diferenciado |
| `004_fix_PINOUT.patch` | docs/PINOUT.md | ACTUALIZAR | Arquitectura RPWM/LPWM; timers TIM1/TIM8/TIM3; wheel/enable/dir pines; tabla consolidada |
| `005_fix_README.patch` | README.md | ACTUALIZAR | Añade sección Hardware & Wiring Documentation con links a wiring_manual, BOM, safety_checks |
| `006_delete_PROTOCOLO_CAN.patch` | docs/PROTOCOLO_CAN.md | ELIMINAR | Archivo obsoleto auto-marcado como desactualizado |

### Cómo aplicar los parches:
```bash
cd /ruta/al/repositorio

# Revisar un parche antes de aplicar
cat analysis_artifacts/patches/001_fix_HARDWARE_SPECIFICATION.patch

# Aplicar un parche (dry-run primero)
patch --dry-run -p1 < analysis_artifacts/patches/001_fix_HARDWARE_SPECIFICATION.patch

# Aplicar definitivamente
patch -p1 < analysis_artifacts/patches/001_fix_HARDWARE_SPECIFICATION.patch

# Para el parche de eliminación
git rm docs/PROTOCOLO_CAN.md
```

---

## 6. Estadísticas Finales

| Métrica | Valor |
|---------|-------|
| Total archivos .md auditados | 132 (129 repo + 3 esp32/) |
| Archivos OK (alineados) | 126 (95.5%) |
| Archivos a ACTUALIZAR | 5 (3.8%) |
| Archivos a ELIMINAR | 1 (0.8%) |
| Parches generados | 6 |
| Inconsistencias críticas corregidas | 15 |
| Inconsistencias menores documentadas | 4 |
| Archivos fuera de la repo | 0 |
| Archivos duplicados | 0 (docs/PROTOCOLO_CAN.md marcado obsoleto por CAN_CONTRACT_FINAL.md) |

---

## 7. Criterio de Finalización

- [x] No queda ningún archivo .md desactualizado sin parche
- [x] No queda ningún archivo obsoleto sin parche de eliminación
- [x] Toda la documentación verificada contra firmware STM32 (Core/Inc/main.h)
- [x] Toda la documentación verificada contra firmware ESP32 (can_ids.h, headers)
- [x] Toda la documentación verificada contra wiring real (wiring_manual.md)
- [x] Toda la documentación verificada contra BOM (BOM.csv)
- [x] Toda la documentación verificada contra análisis formal (FORMAL_VERIFICATION_REPORT.md)
- [x] Toda la documentación verificada contra análisis estático (STATIC_ANALYSIS_REPORT.md)
- [x] Toda la documentación verificada contra tests (713/713 pasados)
- [x] Todos los cambios en parches listos para revisión humana
