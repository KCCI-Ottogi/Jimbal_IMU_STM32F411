#include "gimbal_control.h"
#include "servo.h"
#include <math.h>

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
 */
void gimbalExecuteCombinedControl(void) {
    // 1. IMU 기반 수평 유지 각도 계산 (Base)
    // 기체가 +10도 기울면 서보는 90-10=80도로 움직여 수평 유지
    float base_roll  = 90.0f - imu_roll;
    float base_pitch = 90.0f + imu_pitch;
    float base_yaw   = 90.0f; // 기본 정면

    // 2. 카메라 PID 보정치 합산 (Base + Offset)
    // 자이로 값이 0이면(센서 없음) 90도 정면 기준으로 카메라 추적만 동작함
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

    // 4. 최종 목표 각도를 서보 드라이버에 전달
    // 응답성을 위해 k값은 0.15f 정도로 설정
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