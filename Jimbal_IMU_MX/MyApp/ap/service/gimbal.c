/* gimbal.c */

#include "gimbal.h"
#include <math.h>

// static service_servo_t servo_list[3];
// static camera_data_t cam_data; // 카메라 데이터 저장용

// // UART 인터럽트에서 이 함수를 호출하여 좌표를 넘겨줌
// void gimbalSetCamData(int16_t x, int16_t y, bool detected) {
//   cam_data.x = x;
//   cam_data.y = y;
//   cam_data.is_detected = detected;
// }



// bool serviceServoInit(void) {
//   for (int i = 0; i < 3; i++) {
//     servo_list[i].current_angle = 90.0f;
//     servo_list[i].target_angle = 90.0f;
//     servo_list[i].k = 0.1f; // 기본 부드러움 정도
//     servo_list[i].is_moving = false;
    
//     servoWrite(i, (uint8_t)servo_list[i].current_angle);
//   }
//   return true;
// }

// // 부드럽게 목표값 설정
// void serviceServoSetTarget(uint8_t ch, float angle, float speed) {
//   if (ch >= 2) return;
  

//   // 서보 보호를 위한 각도 제한 (하드웨어 사양에 맞춰 조절) //추가함
//     if (angle < 10.0f)  angle = 10.0f;
//     if (angle > 170.0f) angle = 170.0f;

//   servo_list[ch].target_angle = angle;
//   if (speed > 0) servo_list[ch].k = speed; 
// }




// void serviceServoUpdate(void) {
//   // 1. 카메라가 객체를 잡았다면, 좌표를 목표 각도로 환산
//   if (cam_data.is_detected) {
//     // 예: 중심(160, 120)에 오도록 제어 (좌표 0~320 -> 각도 180~0 반전 매핑 예시)
//     // float targetX = 180.0f - ((float)cam_data.x * 180.0f / 320.0f);
//     // float targetY = (float)cam_data.y * 180.0f / 240.0f;
    
//     // serviceServoSetTarget(0, targetX, 0.15f); // Pan
//     // serviceServoSetTarget(1, targetY, 0.15f); // Tilt

//         // [Pan 제어 - 채널 0]
//         float error_x = (float)(160 - cam_data.x); // 중심(160)과의 차이
//         float next_target_x = servo_list[0].current_angle + (error_x * 0.07f); // 0.07은 Pan 감도
//         serviceServoSetTarget(0, next_target_x, 0.15f);

//         // [Tilt 제어 - 채널 1]
//         float error_y = (float)(120 - cam_data.y); // 중심(120)과의 차이
//         float next_target_y = servo_list[1].current_angle + (error_y * 0.07f); // 0.07은 Tilt 감도
//         serviceServoSetTarget(1, next_target_y, 0.15f);
//   }

//   // 2. 기존 보간 및 출력 로직
//   for (int i = 0; i < 3; i++) {// 20ms마다 호출되어 각도를 갱신하는 핵심 로직
//     float diff = servo_list[i].target_angle - servo_list[i].current_angle;
//     if (fabs(diff) > 0.1f) {
//       servo_list[i].current_angle += diff * servo_list[i].k;
//       servoWrite(i, (uint8_t)servo_list[i].current_angle);
//       servo_list[i].is_moving = true;
//     } else {
//       servo_list[i].is_moving = false;
//     }
//   }
// }


void gimbalUpdate(void) {
    camera_data_t* p_cam = cameraGetLatestData();

    // 1. 카메라 추적 제어 (Object Tracking)
    if (p_cam->is_detected) {
        // Pan(채널 0) 오차 계산 및 목표값 설정
        float error_x = (float)(160 - p_cam->x);
        float next_target_x = servoGetCurrentAngle(0) + (error_x * 0.07f);
        servoSetTarget(0, next_target_x, 0.15f);

        // Tilt(채널 1) 오차 계산 및 목표값 설정
        float error_y = (float)(120 - p_cam->y);
        float next_target_y = servoGetCurrentAngle(1) + (error_y * 0.07f);
        servoSetTarget(1, next_target_y, 0.15f);
    }
}



void gimbalUpdateFromIMU(float roll, float pitch, float yaw) {
    // 1. 수평 유지를 위한 부호 전환 로직 (0도 기준)
    // 기체가 +10도 기울면 서보는 90-10=80도로 움직여 수평 유지
    float target_r = 90.0f - roll;
    float target_p = 90.0f + pitch;
    float target_y = 90.0f - (yaw - 180.0f); // Yaw축 보정 로직 (필요시 주석 해제)

    // 2. 서보 모터 하드웨어 제한 (10도 ~ 170도)
    if (target_r < 10.0f) target_r = 10.0f;
    if (target_r > 170.0f) target_r = 170.0f;

    if (target_p < 10.0f) target_p = 10.0f;
    if (target_p > 170.0f) target_p = 170.0f;

    // 3. 목표 각도 업데이트
    // servo_list[0].target_angle = target_r;
    // servo_list[1].target_angle = target_p;
    // servo_list[2].target_angle = target_y; // 3번째 축 주석 처리
    //직접접근 못해서 함수로 변경
    servoSetTarget(0, target_r, 0.1f); 
    servoSetTarget(1, target_p, 0.1f);
    servoSetTarget(2, target_y, 0.1f);
}

