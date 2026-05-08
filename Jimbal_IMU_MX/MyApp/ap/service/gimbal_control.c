#include "gimbal_control.h"

/* --- 내부 정적 변수 (데이터 저장소) --- */
// 자이로 서비스로부터 받는 기체 각도
static float imu_roll = 0.0f;
static float imu_pitch = 0.0f;
static float imu_yaw = 0.0f;

// 카메라 서비스로부터 받는 추적 보정치 (PID 결과값)
static float cam_offset_x = 0.0f;
static float cam_offset_y = 0.0f;

/**
 * @brief 자이로 서비스(gyro_service.c)가 호출하는 데이터 갱신 함수
 */
void gimbalSettingIMU(float roll, float pitch, float yaw) {
    imu_roll = roll;
    imu_pitch = pitch;
    imu_yaw = yaw;
}

/**
 * @brief 카메라 서비스(camera_service.c)가 호출하는 데이터 갱신 함수
 */
void gimbalSettingCamOffset(float x, float y) {
    cam_offset_x = x;
    cam_offset_y = y;
}

/**
 * @brief [통합 제어부] ap.c에서 주기적으로 호출하여 서보에 최종 명령을 내림
 */void gimbalExecuteCombinedControl(void) {
    // 이전 목표 각도를 저장하기 위한 정적 변수 (차이값 계산용)
    static float last_final_roll = 90.0f;
    static float last_final_pitch = 90.0f;
    static float last_final_yaw = 90.0f;

    // 1. IMU 기반 수평 유지 각도 계산 (Base)
    float base_roll  = 90.0f - imu_roll;
    float base_pitch = 90.0f - imu_pitch;
    float base_yaw   = 90.0f; // 기본 정면

    // 2. 카메라 PID 보정치 합산 (Base + Offset)
    float final_roll  = base_roll;
    float final_pitch = base_pitch + cam_offset_y;
    float final_yaw   = base_yaw + cam_offset_x;

    // 3. 서보 모터 하드웨어 보호를 위한 리미트 (Clamping)
    if (final_roll < 10.0f)   final_roll = 10.0f;
    if (final_roll > 170.0f)  final_roll = 170.0f;

    if (final_pitch < 40.0f)  final_pitch = 40.0f;
    if (final_pitch > 140.0f) final_pitch = 140.0f;

    if (final_yaw < 10.0f)    final_yaw = 10.0f;
    if (final_yaw > 170.0f)   final_yaw = 170.0f;

    // [CLI 모니터링 추가] 0.1초마다 한 줄로 출력
    static uint32_t last_log_time = 0;
    if (osKernelGetTickCount() - last_log_time >= 100) {
        last_log_time = osKernelGetTickCount();

        float diff_r = final_roll - last_final_roll;
        float diff_p = final_pitch - last_final_pitch;
        float diff_y = final_yaw - last_final_yaw;

        cliPrintf("R:%.1f->%.1f(%.2f) | P:%.1f+%.1f->%.1f(%.2f) | Y:90+%.1f->%.1f(%.2f)\r\n",
                  imu_roll, final_roll, diff_r,
                  imu_pitch, cam_offset_y, final_pitch, diff_p,
                  cam_offset_x, final_yaw, diff_y);

        // 현재 값을 다음 비교를 위해 저장
        last_final_roll = final_roll;
        last_final_pitch = final_pitch;
        last_final_yaw = final_yaw;
    }

    // 4. 최종 목표 각도를 서보 드라이버에 전달
    servoSetTarget(0, final_roll,  0.15f); // Roll 축
    servoSetTarget(1, final_pitch, 0.15f); // Pitch 축
    servoSetTarget(2, final_yaw,   0.15f); // Yaw 축
    
    // 5. 실제 보간 이동 실행
    servoSmoothUpdate();
}

/**
 * @brief 짐벌을 초기 정면 위치로 복귀
 */
void gimbalReturnHome(void) {
    cam_offset_x = 0.0f;
    cam_offset_y = 0.0f;
    servoSetTarget(0, 90.0f, 0.1f);
    servoSetTarget(1, 90.0f, 0.1f);
    servoSetTarget(2, 90.0f, 0.1f);
}

/**
 * @brief 현재 특정 채널의 서보 각도를 반환
 */
float gimbalGetCurrentAngle(uint8_t ch) {
    
    if (ch >= 3) return 0.0f; // 예외 처리
    return servoGetCurrentAngle(ch);
}