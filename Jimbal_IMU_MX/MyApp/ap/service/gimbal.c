/* gimbal.c */

#include "gimbal.h"
#include "servo.h" // 하위 드라이버 호출
#include <math.h>

static service_servo_t servo_list[2];
static camera_data_t cam_data; // 카메라 데이터 저장용

// UART 인터럽트에서 이 함수를 호출하여 좌표를 넘겨줌
void gimbalSetCamData(int16_t x, int16_t y, bool detected) {
  cam_data.x = x;
  cam_data.y = y;
  cam_data.is_detected = detected;
}



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
  

  // 서보 보호를 위한 각도 제한 (하드웨어 사양에 맞춰 조절) //추가함
    if (angle < 10.0f)  angle = 10.0f;
    if (angle > 170.0f) angle = 170.0f;

  servo_list[ch].target_angle = angle;
  if (speed > 0) servo_list[ch].k = speed; 
}


void serviceServoUpdate(void) {
  // 1. 카메라가 객체를 잡았다면, 좌표를 목표 각도로 환산
  if (cam_data.is_detected) {
    // 예: 중심(160, 120)에 오도록 제어 (좌표 0~320 -> 각도 180~0 반전 매핑 예시)
    // float targetX = 180.0f - ((float)cam_data.x * 180.0f / 320.0f);
    // float targetY = (float)cam_data.y * 180.0f / 240.0f;
    
    // serviceServoSetTarget(0, targetX, 0.15f); // Pan
    // serviceServoSetTarget(1, targetY, 0.15f); // Tilt

        // [Pan 제어 - 채널 0]
        float error_x = (float)(160 - cam_data.x); // 중심(160)과의 차이
        float next_target_x = servo_list[0].current_angle + (error_x * 0.07f); // 0.07은 Pan 감도
        serviceServoSetTarget(0, next_target_x, 0.15f);

        // [Tilt 제어 - 채널 1]
        float error_y = (float)(120 - cam_data.y); // 중심(120)과의 차이
        float next_target_y = servo_list[1].current_angle + (error_y * 0.07f); // 0.07은 Tilt 감도
        serviceServoSetTarget(1, next_target_y, 0.15f);
  }

  // 2. 기존 보간 및 출력 로직
  for (int i = 0; i < 2; i++) {// 20ms마다 호출되어 각도를 갱신하는 핵심 로직
    float diff = servo_list[i].target_angle - servo_list[i].current_angle;
    if (fabs(diff) > 0.1f) {
      servo_list[i].current_angle += diff * servo_list[i].k;
      servoWrite(i, (uint8_t)servo_list[i].current_angle);
      servo_list[i].is_moving = true;
    } else {
      servo_list[i].is_moving = false;
    }
  }
}

// ESP32-CAM 전용 UART 파싱 함수
void gimbalParseCamData(void) {
    uint8_t data;
    static uint8_t buf[8]; // ESP32에서 보내는 패킷은 8바이트
    static uint8_t idx = 0;

    // 채널 1(ESP32-CAM 연결부)에서 데이터 읽기
    while(uartAvailable(1) > 0) {
        data = uartRead(1);

        // 1. 시작 바이트 확인 (FRAME_STX)
        if (idx == 0 && data == 0x02) {
            buf[idx++] = data;
            continue;
        }

        // 2. 패킷 데이터 저장
        if (idx > 0 && idx < 8) {
            buf[idx++] = data;
        }

        // 3. 패킷 완료 및 검증
        if (idx == 8) {
            // 마지막 바이트가 ETX(0x03)이고 체크섬이 맞는지 확인
            if (buf[7] == 0x03) {
                // 체크섬 계산: CX_H ^ CX_L ^ CY_H ^ CY_L ^ DET
                uint8_t checksum = buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5];
                
                if (checksum == buf[6]) {
                    // 데이터 복원 (Big Endian)
                    int16_t cx = (int16_t)((buf[1] << 8) | buf[2]);
                    int16_t cy = (int16_t)((buf[3] << 8) | buf[4]);
                    bool detected = (buf[5] == 0x01);

                    // 카메라 데이터 업데이트 호출
                    gimbalSetCamData(cx, cy, detected);
                }
            }
            // 패킷 처리 완료 후 인덱스 초기화 (다음 패킷 대기)
            idx = 0;
        }
    }
}