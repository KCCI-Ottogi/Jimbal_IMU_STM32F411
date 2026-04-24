#ifndef I2C_H
#define I2C_H

#include "stm32f411xe.h"
#include <stdint.h>

void I2C1_Init(void);
void I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
void I2C_ReadBurst(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t size);

#endif // I2C_H