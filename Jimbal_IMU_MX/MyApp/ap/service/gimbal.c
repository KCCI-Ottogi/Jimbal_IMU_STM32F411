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

    // 1. 타겟이 인식되었을 때만 계산 진행
    if (p_cam->is_detected && p_cam->area > 50) {
        
        // [Pan 계산] 중심 160 기준 오차 계산
        float error_x = (float)(160 - p_cam->x);
        float next_target_x = servoGetCurrentAngle(0) + (error_x * 0.07f);

        // [Tilt 계산] 중심 120 기준 오차 계산
        float error_y = (float)(120 - p_cam->y);
        float next_target_y = servoGetCurrentAngle(1) + (error_y * 0.07f);

        // 2. 서보 목표값 설정
        servoSetTarget(0, next_target_x, 0.15f);
        servoSetTarget(1, next_target_y, 0.15f);

        // 3. [최종 로그 확인] 이 로그가 터미널에 찍히면 통신+계산 모두 성공!
        // CX/CY: 카메라입력, AREA: 물체크기, TRG_X/Y: 짐벌이 계산한 목표각도
        cliPrintf("[GIMBAL] IN(X:%d, Y:%d, A:%d) -> OUT(TRG_X:%.1f, TRG_Y:%.1f)\r\n", 
                  p_cam->x, p_cam->y, p_cam->area, next_target_x, next_target_y);

    } else if (p_cam->is_detected && p_cam->area <= 50) {
        // 면적이 너무 작으면 노이즈로 간주하고 무시 로그
        cliPrintf("[GIMBAL] Noise Ignored (Area: %d)\r\n", p_cam->area);
    }
}


// // ESP32-CAM 전용 UART 파싱 함수
// void gimbalParseCamData(void) {
//     uint8_t data;
//     static uint8_t buf[8]; // ESP32에서 보내는 패킷은 8바이트
//     static uint8_t idx = 0;

//     // 채널 1(ESP32-CAM 연결부)에서 데이터 읽기
//     while(uartAvailable(1) > 0) {
//         data = uartRead(1);

//         // 1. 시작 바이트 확인 (FRAME_STX)
//         if (idx == 0 && data == 0x02) {
//             buf[idx++] = data;
//             continue;
//         }

//         // 2. 패킷 데이터 저장
//         if (idx > 0 && idx < 8) {
//             buf[idx++] = data;
//         }

//         // 3. 패킷 완료 및 검증
//         if (idx == 8) {
//             // 마지막 바이트가 ETX(0x03)이고 체크섬이 맞는지 확인
//             if (buf[7] == 0x03) {
//                 // 체크섬 계산: CX_H ^ CX_L ^ CY_H ^ CY_L ^ DET
//                 uint8_t checksum = buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5];
                
//                 if (checksum == buf[6]) {
//                     // 데이터 복원 (Big Endian)
//                     int16_t cx = (int16_t)((buf[1] << 8) | buf[2]);
//                     int16_t cy = (int16_t)((buf[3] << 8) | buf[4]);
//                     bool detected = (buf[5] == 0x01);

//                     // 카메라 데이터 업데이트 호출
//                     gimbalSetCamData(cx, cy, detected);
//                 }
//             }
//             // 패킷 처리 완료 후 인덱스 초기화 (다음 패킷 대기)
//             idx = 0;
//         }
//     }
// }


// ================ gimbal.c IMU Control 부분 START ==================
void gimbalUpdateFromIMU(float roll, float pitch, float yaw) {
    // 1. 수평 유지를 위한 부호 전환 로직 (0도 기준)
    // 기체가 +10도 기울면 서보는 90-10=80도로 움직여 수평 유지
    float target_r = 90.0f - roll;
    float target_p = 90.0f + pitch;
    float target_y = 0.0f - (yaw - 180.0f); // Yaw축 보정 로직 (필요시 주석 해제)
    // float target_y = 90.0f - (yaw - 180.0f); // Yaw축 보정 로직 (필요시 주석 해제)

    // 2. 서보 모터 하드웨어 제한 (10도 ~ 170도)
    if (target_r < 10.0f) target_r = 10.0f;
    if (target_r > 170.0f) target_r = 170.0f;

    if (target_p < 10.0f) target_p = 10.0f;
    if (target_p > 170.0f) target_p = 170.0f;

    // 3. 목표 각도 업데이트// servo_list[0].target_angle = target_r;
    // servo_list[1].target_angle = target_p;
    // servo_list[2].target_angle = target_y; // 3번째 축 주석 처리
    //직접접근 못해서 함수로 변경
    servoSetTarget(0, target_r, 0.1f); 
    servoSetTarget(1, target_p, 0.1f);
    servoSetTarget(2, target_y, 0.1f);
    
}

// 태스크에서 10ms 마다 호출될 서보 부드러운 구동 루프
void gimbalTaskLoop(void) {
    for (int i = 0; i < 3; i++) {
        
        // float diff = servo_list[i].target_angle - servo_list[i].current_angle;
        //[수정] servo_list[ch].target_angle 대신 Getter 함수 사용
        float diff = servoGetTargetAngle(i) - servoGetCurrentAngle(i);
        if (fabs(diff) > 0.2f) { // 0.2도 이상 차이날 때만 제어 (지터링 방지)
            // servo_list[i].current_angle += diff * servo_list[i].k;
            //[수정] servo_list[i].current_angle 대신 Getter/Setter 함수 사용
            float current_angle = servoGetCurrentAngle(i);
            float k = servoGetK(i);
            current_angle += diff * k;
            servoSetCurrentAngle(i, current_angle);

            // 여기서 실제로 모터로 신호가 나갑니다.
            // servoWrite(i, (uint8_t)roundf(servo_list[i].current_angle));
            // [수정]   servo_list[i].current_angle 대신 Getter 함수 사용
            // servoWrite(i, (uint8_t)roundf(current_angle));
            // [수정2] 부드럽게 동작하도록
            servoSmoothUpdate(); 
        }
    }
}

float gimbalGetCurrentAngle(uint8_t ch) {
    if (ch >= 3) return 0.0f; // 예외 처리
    // return servo_list[ch].current_angle;
    // [수정] servo_list[ch] 대신 Getter 함수 사용
    return servoGetCurrentAngle(ch); 
}

void gimbalReturnHome(void) {
    // servo_list[0].target_angle = 90.0f;
    // servo_list[1].target_angle = 90.0f;
    // servo_list[2].target_angle = 0.0f;
    // [수정] servo_list[ch].target_angle = 90.0f 대신 Setter 함수 사용
    servoSetTarget(0, 90.0f, 0.1f); // Yaw
    servoSetTarget(1, 90.0f, 0.1f); // Pitch
    servoSetTarget(2, 90.0f, 0.1f); // Roll
}
// ================ gimbal.c IMU Control 부분 END ==================