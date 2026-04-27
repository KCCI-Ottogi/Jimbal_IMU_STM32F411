#ifndef GYRO_H
#define GYRO_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
} Gyro_Data_t;

bool Gyro_Init(void);
void Gyro_StartAuto(void);
void Gyro_StopAuto(void);
bool Gyro_Read(Gyro_Data_t *pData);

#endif // GYRO_H

