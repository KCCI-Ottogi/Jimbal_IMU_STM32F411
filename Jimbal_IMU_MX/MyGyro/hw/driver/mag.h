#ifndef MAG_H
#define MAG_H

#include "stm32f411xe.h"

#define HMC5883L_ADDR (0x1E << 1)

void Mag_Init(void);
void Mag_Task(void);

#endif // MAG_H