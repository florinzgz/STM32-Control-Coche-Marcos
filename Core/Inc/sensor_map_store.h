/**
  ****************************************************************************
  * @file    sensor_map_store.h
  * @brief   Persistent DS18B20 temperature sensor mapping storage (flash NVM)
  *
  * Stores the physIdx→role mapping for the 5 DS18B20 sensors in flash so
  * that the assignment survives power cycles without reprogramming.
  *
  * The user assigns roles from the engineering hidden menu on the ESP32
  * display.  The ESP32 sends the mapping via CAN (CAN_ID_CMD_SENSOR_MAP_TEMP,
  * 0x112) and the STM32 stores it here, then applies it when building the
  * STATUS_TEMP_MAP (0x206) CAN frame so the ESP32 receives correctly-labelled
  * temperatures.
  *
  * Flash layout:
  *   Page 123 (0x0807B000, 4 KB) — dedicated to sensor mapping.
  *   Single slot with magic + CRC32 integrity check.
  *
  * Roles (position indices):
  *   0 = FL motor    1 = FR motor    2 = RL motor
  *   3 = RR motor    4 = Ambient
  *   0xFF = unset (sensor is present but not yet assigned)
  *
  * API:
  *   SensorMapStore_Init()          — load from flash on boot
  *   SensorMapStore_Save(map)       — persist a new 5-byte mapping
  *   SensorMapStore_GetMap()        — read pointer to active mapping
  *   SensorMapStore_IsValid()       — true if a valid mapping is stored
  ****************************************************************************
  */

#ifndef SENSOR_MAP_STORE_H
#define SENSOR_MAP_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Number of DS18B20 sensors — must match NUM_DS18B20 in project_config.h */
#define SMAP_NUM_SENSORS    5U

/** Sentinel: sensor not yet assigned to a position */
#define SMAP_ROLE_UNSET     0xFFU

/**
 * @brief  Initialise the sensor map store module.
 *         Reads flash page 123 and validates CRC / magic.
 *         If valid, the loaded mapping is immediately active.
 *         If invalid or absent, the identity mapping (0,1,2,3,4) is used.
 */
void SensorMapStore_Init(void);

/**
 * @brief  Persist a new physIdx→role mapping to flash.
 *
 * @param  map  Array of SMAP_NUM_SENSORS bytes.
 *              map[physIdx] = role (0-4) or SMAP_ROLE_UNSET.
 * @retval true  on success.
 * @retval false on flash erase/write error.
 */
bool SensorMapStore_Save(const uint8_t map[SMAP_NUM_SENSORS]);

/**
 * @brief  Get a pointer to the currently active physIdx→role map.
 *         Always returns a valid SMAP_NUM_SENSORS-element array.
 *         Falls back to identity mapping (0,1,2,3,4) if no valid
 *         flash slot was found.
 */
const uint8_t *SensorMapStore_GetMap(void);

/**
 * @brief  Query whether a valid mapping was loaded from flash.
 * @retval true  — flash contained a valid slot (user has assigned sensors).
 * @retval false — no valid slot; identity mapping is in use.
 */
bool SensorMapStore_IsValid(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MAP_STORE_H */
