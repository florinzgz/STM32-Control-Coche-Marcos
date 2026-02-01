#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

// Prototypes for sensor management functions
void Sensor_Init(void);

// Interrupt handler prototypes for the wheels
void Wheel_FL_IRQHandler(void);
void Wheel_FR_IRQHandler(void);
void Wheel_RL_IRQHandler(void);
void Wheel_RR_IRQHandler(void);

// Wheel speed retrieval prototypes
float Wheel_GetSpeed_FL(void);
float Wheel_GetSpeed_FR(void);
float Wheel_GetSpeed_RL(void);
float Wheel_GetSpeed_RR(void);

// RPM retrieval prototypes
float Wheel_GetRPM_FL(void);

// Pedal management prototypes
void Pedal_Update(void);
float Pedal_GetValue(void);
float Pedal_GetPercent(void);

// Temperature management prototypes
void Temperature_ReadAll(void);
float Temperature_Get(void);

// Current management prototypes
void Current_ReadAll(void);
float Current_Get(void);

// Voltage management prototype
float Voltage_Get(void);

// Power management prototype
float Power_Get(void);

#endif // SENSOR_MANAGER_H
