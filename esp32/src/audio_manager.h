// =============================================================================
// ESP32-S3 HMI — Audio Manager (DFPlayer Mini)
//
// Non-blocking audio playback via DFPlayer Mini module over UART.
// Priority-based queue: higher priority sounds preempt lower ones.
// The DFPlayer reads MP3 files from an SD card (numbered 0001–0068).
//
// UART2 on GPIO 43 (TX) / GPIO 44 (RX), 9600 baud.
//
// Full 68-track sound mapping — see docs/AUDIO_TRACKS_GUIDE.md
//
// Reference: FIRMWARE_MIGRATION_AUDIT.md Step 7
//            Original firmware: include/alerts.h (Audio::Track enum)
// =============================================================================

#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include <cstdint>

namespace audio {

/// GPIO assignments for DFPlayer Mini UART
inline constexpr int PIN_DFPLAYER_TX = 43;   // ESP32 TX → DFPlayer RX
inline constexpr int PIN_DFPLAYER_RX = 44;   // DFPlayer TX → ESP32 RX

/// Sound file indices (must match SD card file numbering 0001.mp3–0068.mp3)
/// Matches original firmware Audio::Track enum from include/alerts.h
enum class Sound : uint8_t {
    // Sistema principal (1-3)
    WELCOME              = 1,   // "Bienvenido Marcos. El sistema está listo."
    FAREWELL             = 2,   // "Cerrando sistemas. Hasta pronto."
    ERROR_GENERAL        = 3,   // "Atención. Se ha detectado un error general."

    // Calibración de pedal (4-5)
    PEDAL_OK             = 4,   // "Calibración del pedal completada."
    PEDAL_ERROR          = 5,   // "Error en el sensor del pedal."

    // Sensores de corriente INA226 (6-7)
    INA_OK               = 6,   // "Calibración de sensores de corriente finalizada."
    INA_ERROR            = 7,   // "Error en sensores de corriente."

    // Encoder de dirección (8-9)
    ENCODER_OK           = 8,   // "Encoder sincronizado correctamente."
    ENCODER_ERROR        = 9,   // "Error en el sensor de dirección."

    // Temperatura (10-11)
    TEMP_HIGH            = 10,  // "Temperatura del motor elevada."
    TEMP_NORMAL          = 11,  // "Temperatura del motor normalizada."

    // Batería (12-13)
    BATTERY_LOW          = 12,  // "Nivel de batería bajo."
    BATTERY_CRITICAL     = 13,  // "Batería en nivel crítico. Desconectando tracción."

    // Freno de estacionamiento (14-15)
    PARKING_BRAKE_ON     = 14,  // "Freno de estacionamiento activado."
    PARKING_BRAKE_OFF    = 15,  // "Freno de estacionamiento desactivado."

    // Luces (16-17)
    LIGHTS_ON            = 16,  // "Luces encendidas."
    LIGHTS_OFF           = 17,  // "Luces apagadas."

    // Radio/Multimedia (18-19)
    RADIO_ON             = 18,  // "Sistema multimedia activado."
    RADIO_OFF            = 19,  // "Sistema multimedia desactivado."

    // Marchas (20-24)
    GEAR_D1              = 20,  // "Marcha D uno activada."
    GEAR_D2              = 21,  // "Marcha D dos activada."
    GEAR_REVERSE         = 22,  // "Marcha atrás activada."
    GEAR_NEUTRAL         = 23,  // "Punto muerto."
    GEAR_PARK            = 24,  // "Vehículo en posición de estacionamiento."

    // Menú oculto y calibración (25-28)
    MENU_HIDDEN          = 25,  // "Menú de calibración avanzado activado."
    CAL_PEDAL            = 26,  // "Iniciando calibración del pedal."
    CAL_INA              = 27,  // "Calibrando sensores de corriente."
    CAL_ENCODER          = 28,  // "Calibrando el punto central del volante."

    // Test del sistema (29-30)
    TEST_SYSTEM          = 29,  // "Iniciando comprobación completa del sistema."
    TEST_OK              = 30,  // "Comprobación finalizada. Todos los módulos operativos."

    // Emergencia y seguridad (31-32)
    EMERGENCY            = 31,  // "Modo de emergencia activado. Motor deshabilitado."
    SAFETY_RESET         = 32,  // "Reinicio de seguridad completado."

    // Errores de sensores específicos (33-35)
    SENSOR_TEMP_ERROR    = 33,  // "Error en sensor de temperatura."
    SENSOR_CURRENT_ERROR = 34,  // "Anomalía en lectura de corriente."
    SENSOR_SPEED_ERROR   = 35,  // "Sin señal de velocidad."

    // Estado de módulos (36)
    MODULE_OK            = 36,  // "Módulo verificado correctamente."

    // Tracción 4x4/4x2 (37-38)
    TRACTION_4X4         = 37,  // "Tracción 4x4 inteligente activada."
    TRACTION_4X2         = 38,  // "Tracción 4x2 inteligente activada."

    // Sistemas de seguridad avanzados (39-44)
    ABS_ON               = 39,  // "Sistema antibloqueo de frenos activado."
    ABS_OFF              = 40,  // "Sistema antibloqueo de frenos desactivado."
    TCS_ON               = 41,  // "Control de tracción activado."
    TCS_OFF              = 42,  // "Control de tracción desactivado."
    REGEN_ON             = 43,  // "Frenado regenerativo activado."
    REGEN_OFF            = 44,  // "Frenado regenerativo desactivado."

    // WiFi y conectividad (45-48) — reserved for future use
    WIFI_CONNECTED       = 45,
    WIFI_DISCONNECTED    = 46,
    OTA_STARTED          = 47,
    OTA_COMPLETED        = 48,

    // Bluetooth (49-51) — reserved for future use
    BT_CONNECTED         = 49,
    BT_DISCONNECTED      = 50,
    BT_PAIRING           = 51,

    // Estados del vehículo (52-56)
    MAX_SPEED            = 52,  // "Velocidad máxima alcanzada."
    OVERCURRENT          = 53,  // "Corriente excesiva detectada."
    OBSTACLE_WARN        = 54,  // "Atención. Obstáculo detectado."
    PARKING_ASSIST       = 55,  // "Modo asistencia de estacionamiento activado."
    SOFT_START           = 56,  // "Iniciando arranque suave de motores."

    // Información de telemetría (57-60)
    BATTERY_50           = 57,  // "Nivel de batería al 50 por ciento."
    BATTERY_25           = 58,  // "Nivel de batería al 25 por ciento."
    DISTANCE_1KM         = 59,  // "Ha recorrido un kilómetro en esta sesión."
    ENERGY_SAVE          = 60,  // "Modo ahorro de energía activado."

    // Modos de conducción (61-63)
    MODE_ECO             = 61,  // "Modo eco activado."
    MODE_NORMAL          = 62,  // "Modo normal activado."
    MODE_SPORT           = 63,  // "Modo deportivo activado."

    // Feedback de configuración (64-68)
    CONFIG_SAVED         = 64,  // "Configuración guardada correctamente."
    CONFIG_RESTORED      = 65,  // "Configuración de fábrica restaurada."
    ERRORS_CLEARED       = 66,  // "Registro de errores borrado."
    REGEN_ADJUSTED       = 67,  // "Nivel de regeneración ajustado."
    BEEP                 = 68   // (Short confirmation beep)
};

/// Priority levels (higher number = higher priority, preempts lower)
/// Note: names LO/HI avoid conflict with Arduino macros HIGH and LOW.
enum class Priority : uint8_t {
    LO      = 0,   // Gear clicks, info beeps, mode changes
    MEDIUM  = 1,   // Obstacle warnings, battery alerts, temperature
    HI      = 2    // Errors, emergency, welcome, farewell
};

/// Initialize UART and DFPlayer module
void init();

/// Update audio state machine — call from main loop
/// Handles DFPlayer busy detection and queue processing.
void update();

/// Play a sound. If a higher or equal priority sound is already playing,
/// the new sound is queued. If a lower priority sound is playing, it is
/// preempted.
/// Includes per-sound cooldown to prevent rapid repetition of the same alert.
void play(Sound sound, Priority priority = Priority::MEDIUM);

/// Set master volume (0–30, DFPlayer range)
void setVolume(uint8_t vol);

/// Returns true if audio is currently playing
bool isPlaying();

} // namespace audio

#endif // AUDIO_MANAGER_H
