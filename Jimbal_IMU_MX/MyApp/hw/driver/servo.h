/* servo.h */
#ifndef SERVO_H_
#define SERVO_H_
#include "hw_def.h"
#include "cli.h" //추가함

void servoInit(void);
void servoWrite(uint8_t ch, uint8_t angle);
void servoScan(uint8_t ch);
void servoDualTest(void);
void servoTotalTest(void); // 전체 테스트 함수 추가


// 보간 제어용 인터페이스
void servoSetTarget(uint8_t ch, float target, float k);
void servoSmoothUpdate(void);



#endif