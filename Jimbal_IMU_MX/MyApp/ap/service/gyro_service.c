

/* gyro_service.c */

#include "gyro_service.h"
#include <math.h>


#include "gimbal_control.h" // gimbalUpdateFromIMU 보고용
#include "cli.h"            // cliPrintf, isMonitoringOn 사용
#include "gyro.h"           // Gyro_Read, Gyro_Data_t 사용



// void gimbalUpdate(void) {
//     camera_data_t* p_cam = cameraGetLatestData();

//     if (p_cam->is_detected) {
//         // 1. 화면 중심점 설정 (160x120 해상도 기준)
//         const int16_t center_x = 80; 
//         const int16_t center_y = 60;

//         // 2. 오차 계산 (중심에서 얼마나 떨어져 있는가)
//         float error_x = (float)(center_x - p_cam->x); // 왼쪽이면 양수, 오른쪽이면 음수
//         // [수정] Y축(Tilt) 부호 반전: (center_y - p_cam->y) -> (p_cam->y - center_y)
//         float error_y = (float)(p_cam->y - center_y);

        
//         // 3. 각도 변환 및 업데이트
//         // 감도(0.1f ~ 0.2f)를 곱해 현재 각도에서 조금씩 이동합니다.
//         // TRG 값이 180을 넘지 않도록 하는 것이 핵심입니다.
//         float next_target_x = servoGetCurrentAngle(2) + (error_x * 0.15f); 
//         float next_target_y = servoGetCurrentAngle(1) + (error_y * 0.15f);

//         // 4. 서보 목표값 설정
//         servoSetTarget(2, next_target_x, 0.15f);
//         servoSetTarget(1, next_target_y, 0.15f);

//         // [디버그 로그] 이제 TRG 값이 90도 근처에서 안정적으로 움직일 겁니다.
//         cliPrintf("[GIMBAL] IN(%d,%d) -> ERR(%.1f,%.1f) -> TRG(%.1f,%.1f)\r\n", 
//                   p_cam->x, p_cam->y, error_x, error_y, next_target_x, next_target_y);
//     }
// }



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

    if (target_p < 40.0f) target_p = 40.0f;
    if (target_p > 140.0f) target_p = 140.0f;

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

// float gimbalGetCurrentAngle(uint8_t ch) {
//     if (ch >= 3) return 0.0f; // 예외 처리
//     // return servo_list[ch].current_angle;
//     // [수정] servo_list[ch] 대신 Getter 함수 사용
//     return servoGetCurrentAngle(ch); 
// }

// void gimbalReturnHome(void) {
//     // servo_list[0].target_angle = 90.0f;
//     // servo_list[1].target_angle = 90.0f;
//     // servo_list[2].target_angle = 0.0f;
//     // [수정] servo_list[ch].target_angle = 90.0f 대신 Setter 함수 사용
//     servoSetTarget(0, 90.0f, 0.1f); // Yaw
//     servoSetTarget(1, 90.0f, 0.1f); // Pitch
//     servoSetTarget(2, 90.0f, 0.1f); // Roll
// }


///////////////////


// /* ap.c에 있는 변수를 가져다 쓰겠다는 선언 */
// extern volatile uint32_t gimbal_report_period;

void gyroServiceUpdate(void) {
    static Gyro_Data_t imu_data; 

    if (Gyro_Read(&imu_data)) {
        gimbalSettingIMU(imu_data.roll, imu_data.pitch, imu_data.yaw);
    }

    // [CLI 출력 로직] ap.c에서 이사 옴
    // static uint32_t last_print_tick = 0;
    
    // if (gimbal_report_period > 0) {
    //     uint32_t now = osKernelGetTickCount();
    //     if (now - last_print_tick >= gimbal_report_period) {
    //         last_print_tick = now;

    //         // 모니터링 모드가 아닐 때만 예쁘게 출력
    //         if (!isMonitoringOn()) {
    //             cliPrintf("GYRO [R: %3d | P: %3d | Y: %3d]\r\n", 
    //                       (int)imu_data.roll, (int)imu_data.pitch, (int)imu_data.yaw);
    //         }
    //     }
    // }
}