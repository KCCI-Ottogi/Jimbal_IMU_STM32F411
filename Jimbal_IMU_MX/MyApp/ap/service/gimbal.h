/* gimbal.h */

#ifndef AP_SERVICE_GIMBAL_H_
#define AP_SERVICE_GIMBAL_H_

#include "hw_def.h"

// 서보 상태 구조체
typedef struct {
  float current_angle;
  float target_angle;
  float k;          // 보간 계수 (0.05 ~ 0.2)
  bool  is_moving;  // 현재 이동 중인지 확인용
} service_servo_t;

// 카메라 데이터 구조체 추가
typedef struct {
  int16_t x;
  int16_t y;
  bool is_detected;
} camera_data_t;

bool serviceServoInit(void);
void serviceServoSetTarget(uint8_t ch, float angle, float speed);
void serviceServoUpdate(void); // 태스크에서 호출할 함수


// UART에서 호출할 데이터 업데이트 함수 추가
void gimbalSetCamData(int16_t x, int16_t y, bool detected);
void gimbalParseCamData(void);

#endif