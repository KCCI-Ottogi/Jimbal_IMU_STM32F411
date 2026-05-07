

/* gyro_service.c */

#include "gyro_service.h"
#include <math.h>


#include "gimbal_control.h" // gimbalUpdateFromIMU 보고용
#include "cli.h"            // cliPrintf, isMonitoringOn 사용
#include "gyro.h"           // Gyro_Read, Gyro_Data_t 사용




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

#include "gyro_service.h"
#include "gimbal_control.h"
#include "cli.h"
#include "monitor.h"
#include "cmsis_os2.h"

/* --- 내부 정적 변수 --- */
static bool is_gyro_ok = false;
static uint32_t imu_report_period = 0;
static Mag_Data_t current_magData; // 지자기 데이터 저장용
static uint32_t last_calc_tick = 0;
static uint32_t last_print_tick = 0;

extern osSemaphoreId_t GyroReadySemHandle;


// [추가] 외부(magSystemTask)에서 지자기 데이터를 업데이트해줄 함수
// void gyroServiceSetMagData(Mag_Data_t *p_mag) {
//     if (p_mag != NULL) {
//         current_magData = *p_mag;
//     }
// }
static uint32_t   mag_updated_tick = 0; // 데이터 신선도 체크용

void gyroServiceSetMagData(Mag_Data_t *p_mag) {
    if (p_mag == NULL) return;
    
    // Mutex가 있다면 여기서 Lock
    current_magData = *p_mag;
    mag_updated_tick = osKernelGetTickCount();
    // Mutex Unlock
}

/**
 * @brief 서비스 초기화
 */
bool gyroServiceInit(void) {
    is_gyro_ok = Gyro_Init();
    last_calc_tick = osKernelGetTickCount();
    return is_gyro_ok;
}


// 외부에서 상태를 물어볼 때 답해주는 함수
bool gyroServiceIsOk(void) {
    return is_gyro_ok;
}

/**
 * @brief 하드웨어 재시도 로직 (cli imu on 시 호출)
 */
bool gyroServiceReInit(void) {
    cliPrintf("Re-initializing I2C & Sensors...\r\n");
    is_gyro_ok = Gyro_Init();
    return is_gyro_ok;
}
/**
 * @brief 리포트 주기 및 센서 동작 설정
 */
void gyroServiceSetPeriod(uint32_t period) {
    imu_report_period = period;
    if (period > 0) {
        Gyro_StartAuto();
    } else {
        Gyro_StopAuto();
    }
}

// /**
//  * @brief 자이로 데이터를 읽어 짐벌 시스템에 전달 (자이로 태스크 전용)
//  */
// void gyroServiceUpdate(void) {
//     if (!is_gyro_ok) return;

//     static Gyro_Data_t imu_data; 
//     // 1. 센서 데이터 읽기
//     if (Gyro_Read(&imu_data)) {
//         // 2. 통합 제어 모듈(gimbal_control)에 데이터 전달 (가장 중요)
//         gimbalSettingIMU(imu_data.roll, imu_data.pitch, imu_data.yaw);
//     }


//     // [CLI 출력 로직] ap.c에서 이사 옴
//     // static uint32_t last_print_tick = 0;
    
//     // if (gimbal_report_period > 0) {
//     //     uint32_t now = osKernelGetTickCount();
//     //     if (now - last_print_tick >= gimbal_report_period) {
//     //         last_print_tick = now;

//     //         // 모니터링 모드가 아닐 때만 예쁘게 출력
//     //         if (!isMonitoringOn()) {
//     //             cliPrintf("GYRO [R: %3d | P: %3d | Y: %3d]\r\n", 
//     //                       (int)imu_data.roll, (int)imu_data.pitch, (int)imu_data.yaw);
//     //         }
//     //     }
//     // }
// }

/**
 * @brief 핵심 로직: 센서 읽기 -> 각도 계산 -> 짐벌 전달 -> 리포팅
 */
void gyroServiceUpdate(void) {
    if (!is_gyro_ok || GyroReadySemHandle == NULL) {
        return;
    }

    // 세마포어 획득 (센서 데이터 준비 완료 대기)
    if (osSemaphoreAcquire(GyroReadySemHandle, osWaitForever) != osOK) return;

    static Gyro_Data_t gyroData;

    // 1. 센서 데이터 읽기 및 연결 확인
    if (!Gyro_Read(&gyroData)) {
        is_gyro_ok = false;
        cliPrintf("\r\n[ALARM] Sensor Disconnected! Type 'imu off' then 'imu on' to recover.\r\n");
        return;
    }

    // 2. 시간 간격(dt) 계산
    uint32_t now = osKernelGetTickCount();
    float dt = (now - last_calc_tick) / 1000.0f;
    last_calc_tick = now;


    // 데이터 신선도 확인 (예: 500ms 이상 갱신 안 되면 가짜 데이터나 무시)
    Mag_Data_t *p_mag_to_use = &current_magData;
    if (osKernelGetTickCount() - mag_updated_tick > 500) {
        p_mag_to_use = NULL; // IMU_ComputeAngles 내부에서 NULL 체크를 하도록 설계
    }


    // 3. 센서 융합 및 각도 계산 (가속도 + 자이로 + 지자기)
    // IMU_ComputeAngles(&gyroData, &current_magData, dt);
    IMU_ComputeAngles(&gyroData, p_mag_to_use, dt); // NULL 처리가 반영된 포인터를 넘겨야 함


    // 4. 계산된 각도를 짐벌 제어기로 즉시 전송 (가장 중요)
    gimbalSettingIMU(gyroData.roll, gyroData.pitch, gyroData.yaw);

    // 5. 주기적 출력 제어 (CLI 또는 모니터링 툴)
    if (imu_report_period > 0 && (now - last_print_tick >= imu_report_period)) {
        last_print_tick = now;

        if (!isMonitoringOn()) {
            cliPrintf("Angle [Roll: %6.1f | Pitch: %6.1f | Yaw: %6.1f]\r\n", 
                      gyroData.roll, gyroData.pitch, gyroData.yaw);
        } else {
            monitorUpdateValue(ID_IMU_GYRO_X, TYPE_FLOAT, &gyroData.roll);
            monitorUpdateValue(ID_IMU_GYRO_Y, TYPE_FLOAT, &gyroData.pitch);
            monitorUpdateValue(ID_IMU_GYRO_Z, TYPE_FLOAT, &gyroData.yaw);
        }
    }
}
