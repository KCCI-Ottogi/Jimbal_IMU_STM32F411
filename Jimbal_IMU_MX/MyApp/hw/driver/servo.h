/* servo.h */
#ifndef __SERVO_H__
#define __SERVO_H__
#include "hw_def.h"
#include "cli.h" //추가함


/**
 * [하드웨어 설정값 정의]
 * 지정하신 각 채널별 초기 정렬 위치 및 물리적 임계치
 */

 
#define SERVO_CH0_MIN    0
#define SERVO_CH0_INIT   110.0f
#define SERVO_CH0_MAX    180

#define SERVO_CH1_MIN    0
#define SERVO_CH1_INIT   65.0f
#define SERVO_CH1_MAX    130   // Pitch 축 하드웨어 제한

#define SERVO_CH2_MIN    0
#define SERVO_CH2_INIT   70.0f
#define SERVO_CH2_MAX    180



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