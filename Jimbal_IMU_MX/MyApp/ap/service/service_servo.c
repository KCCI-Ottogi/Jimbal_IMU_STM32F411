#include "service_servo.h"
#include "servo.h" // 하위 드라이버 호출

static service_servo_t servo_list[2];

bool serviceServoInit(void) {
  for (int i = 0; i < 2; i++) {
    servo_list[i].current_angle = 90.0f;
    servo_list[i].target_angle = 90.0f;
    servo_list[i].k = 0.1f; // 기본 부드러움 정도
    servo_list[i].is_moving = false;
    
    servoWrite(i, (uint8_t)servo_list[i].current_angle);
  }
  return true;
}

// 부드럽게 목표값 설정
void serviceServoSetTarget(uint8_t ch, float angle, float speed) {
  if (ch >= 2) return;
  
  servo_list[ch].target_angle = angle;
  if (speed > 0) servo_list[ch].k = speed; 
}

// 20ms마다 호출되어 각도를 갱신하는 핵심 로직
void serviceServoUpdate(void) {
  for (int i = 0; i < 2; i++) {
    float diff = servo_list[i].target_angle - servo_list[i].current_angle;
    
    // 차이가 아주 작으면 이동 완료로 간주
    if (fabs(diff) > 0.1f) {
      servo_list[i].current_angle += diff * servo_list[i].k;
      servoWrite(i, (uint8_t)servo_list[i].current_angle);
      servo_list[i].is_moving = true;
    } else {
      servo_list[i].is_moving = false;
    }
  }
}