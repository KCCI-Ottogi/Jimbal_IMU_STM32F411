#ifndef __SERVO_H__
#define __SERVO_H__

#include "hw_def.h"

#define SERVO_CH0_INIT   110.0f
#define SERVO_CH1_INIT   65.0f 
#define SERVO_CH2_INIT   70.0f 

#define SERVO_CH0_MAX    180
#define SERVO_CH1_MAX    130
#define SERVO_CH2_MAX    180

void servoInit(void);
void servoWrite(uint8_t ch, uint8_t angle);

#endif // __SERVO_H__