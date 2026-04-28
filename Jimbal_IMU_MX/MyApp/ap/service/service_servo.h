#ifndef AP_SERVICE_SERVICE_SERVO_H_
#define AP_SERVICE_SERVICE_SERVO_H_

#include "hw_def.h"

// 서보 상태 구조체
typedef struct {
  float current_angle;
  float target_angle;
  float k;          // 보간 계수 (0.05 ~ 0.2)
  bool  is_moving;  // 현재 이동 중인지 확인용
} service_servo_t;

bool serviceServoInit(void);
void serviceServoSetTarget(uint8_t ch, float angle, float speed);
void serviceServoUpdate(void); // 태스크에서 호출할 함수

#endif