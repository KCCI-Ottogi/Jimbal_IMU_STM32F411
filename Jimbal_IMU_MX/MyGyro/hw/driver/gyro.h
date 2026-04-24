#ifndef GYRO_H
#define GYRO_H

#include "stm32f411xe.h"

#define MPU6050_ADDR (0x68 << 1)

void Gyro_Init(void);
void Gyro_Task(void);

#endif // GYRO_H