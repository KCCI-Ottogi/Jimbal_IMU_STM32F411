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
    imu_roll = roll *-1 ;
    imu_pitch = roll *-1;
    imu_yaw = roll *-1;
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
    // 1. servo.c에서 정의된 초기값(Init)을 가져옵니다.
    float init_r = servoGetInitAngle(0);
    float init_p = servoGetInitAngle(1);
    float init_y = servoGetInitAngle(2);

    // 이전 목표 각도를 저장하기 위한 정적 변수 (최초 1회만 init값으로 설정)
    static float last_final_roll = -1.0f;
    static float last_final_pitch = -1.0f;
    static float last_final_yaw = -1.0f;

    if (last_final_roll < 0) { // 초기화
        last_final_roll = init_r;
        last_final_pitch = init_p;
        last_final_yaw = init_y;
    }

    // 2. IMU 기반 수평 유지 각도 계산 (Base)
    // 90.0f 대신 받아온 init_r, init_p, init_y를 사용합니다.
    float base_roll  = init_r + imu_roll;
    float base_pitch = init_p + imu_pitch;
    float base_yaw   = init_y + imu_yaw; // imu_yaw 반영됨

    // 3. 카메라 PID 보정치 합산 (Base + Offset)
    float final_roll  = base_roll;
    float final_pitch = base_pitch + cam_offset_y;
    float final_yaw   = base_yaw + cam_offset_x;

    // 4. 서보 모터 하드웨어 보호를 위한 리미트 (Clamping)
    if (final_roll < 10.0f)   final_roll = 10.0f;
    if (final_roll > 170.0f)  final_roll = 170.0f;

    if (final_pitch < 40.0f)  final_pitch = 40.0f;
    if (final_pitch > 140.0f) final_pitch = 140.0f;

    if (final_yaw < 10.0f)    final_yaw = 10.0f;
    if (final_yaw > 170.0f)   final_yaw = 170.0f;

    // [CLI 모니터링] 기존 형식을 유지하되, 숫자 90 부분만 실제 init값으로 출력
    static uint32_t last_log_time = 0;
    if (osKernelGetTickCount() - last_log_time >= 100) {
        last_log_time = osKernelGetTickCount();

        float diff_r = final_roll - last_final_roll;
        float diff_p = final_pitch - last_final_pitch;
        float diff_y = final_yaw - last_final_yaw;

        // 원하신 대로 기존 CLI 포맷을 그대로 유지합니다.
        // R(초기값):현재IMU->최종목표(차이) | P(초기값):현재IMU+오프셋->최종목표(차이) | ...
        cliPrintf("R(%.1f):%.1f->%.1f(%.2f) | P(%.1f):%.1f+%.1f->%.1f(%.2f) | Y(%.1f):%.1f+%.1f->%.1f(%.2f)\r\n",
                  init_r, imu_roll, final_roll, diff_r,
                  init_p, imu_pitch, cam_offset_y, final_pitch, diff_p,
                  init_y, imu_yaw, cam_offset_x, final_yaw, diff_y);

        // 현재 값을 다음 비교를 위해 저장
        last_final_roll = final_roll;
        last_final_pitch = final_pitch;
        last_final_yaw = final_yaw;
    }

    // 5. 최종 목표 각도를 서보 드라이버에 전달
    servoSetTarget(0, final_roll,  0.15f); // Roll 축
    servoSetTarget(1, final_pitch, 0.15f); // Pitch 축
    servoSetTarget(2, final_yaw,   0.15f); // Yaw 축
    
    // 6. 실제 보간 이동 실행
    servoSmoothUpdate();
}

/**
 * @brief 짐벌을 초기 정면 위치로 복귀
 */
void gimbalReturnHome(void) {
    cam_offset_x = 0.0f;
    cam_offset_y = 0.0f;
    // servo.c의 설정값을 그대로 가져와서 복귀
    servoSetTarget(0, servoGetInitAngle(0), 0.1f);
    servoSetTarget(1, servoGetInitAngle(1), 0.1f);
    servoSetTarget(2, servoGetInitAngle(2), 0.1f);
}
/**
 * @brief 현재 특정 채널의 서보 각도를 반환
 */
float gimbalGetCurrentAngle(uint8_t ch) {
    
    if (ch >= 3) return 0.0f; // 예외 처리
    return servoGetCurrentAngle(ch);
}