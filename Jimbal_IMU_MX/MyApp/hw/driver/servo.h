/* servo.h */
#ifndef __SERVO_H__
#define __SERVO_H__
#include "hw_def.h"
#include "cli.h" //추가함


typedef struct {
    float current_angle; // 현재 각도
    float target_angle;  // 목표 각도
    float k;             // 부드러움 계수
} servo_smooth_t;

void servoInit(void);
void servoWrite(uint8_t ch, uint8_t angle);


void servoScan(uint8_t ch);
void servoDualTest(void);
void servoTotalTest(void); // 전체 테스트 함수 추가


// 보간 제어용 인터페이스
void servoSmoothUpdate(void);
void servoSweep(uint8_t ch, float k);


void servoSetTarget(uint8_t ch, float target, float k);
void servoSetTargetAll(float target0, float target1, float target2, float k);

void servoRunToTarget(uint8_t ch);

float servoGetCurrentAngle(uint8_t ch);
float servoGetTargetAngle(uint8_t ch);
float servoGetK(uint8_t ch);
void servoSetCurrentAngle(uint8_t ch, float angle);
float servoGetInitAngle(uint8_t ch) ;
#endif