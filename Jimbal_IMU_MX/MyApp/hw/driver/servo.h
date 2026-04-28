/* servo.h */
#ifndef SERVO_H_
#define SERVO_H_
#include "hw_def.h"
#include "cli.h" //추가함

void servoInit(void);
void servoWrite(uint8_t ch, uint8_t angle);
void servoScan(uint8_t ch);
void servoDualTest(void);

#endif