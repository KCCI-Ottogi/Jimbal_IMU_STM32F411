#include "gimbal.h"
#include "gyro.h"
#include "mag.h"
#include "servo.h"
#include "camera_uart.h"

static Gyro_Data_t imu_data;
static Mag_Data_t mag_data;

static float target_yaw = 0.0f;
static float target_pitch = 0.0f;

// 카메라 오차를 각도로 변환하는 게인값 (진동이 심하면 줄이세요)
#define KP_TRACKING_X  0.05f 
#define KP_TRACKING_Y  0.03f

void gimbalInit(void) {
    target_yaw = 0.0f;
    target_pitch = 0.0f;
}

void gimbalUpdateSensors(void) {
    if (Gyro_Read(&imu_data) && Mag_Read(&mag_data)) {
        IMU_ComputeAngles(&imu_data, &mag_data, 0.01f);
    }
}

void gimbalExecuteControl(void) {
    int err_x = cameraGetErrorX(); 
    int err_y = cameraGetErrorY();

    target_yaw -= (float)err_x * KP_TRACKING_X;
    target_pitch += (float)err_y * KP_TRACKING_Y;

    if (target_pitch > 30.0f) target_pitch = 30.0f;
    if (target_pitch < -30.0f) target_pitch = -30.0f;

    // 최종 서보 각도 = 초기각도 + 추적각도 - IMU기울기(역보상)
    float out_yaw   = SERVO_CH0_INIT + target_yaw - imu_data.yaw;
    float out_pitch = SERVO_CH1_INIT + target_pitch - imu_data.pitch;
    float out_roll  = SERVO_CH2_INIT - imu_data.roll;

    servoWrite(0, (uint8_t)out_yaw);
    servoWrite(1, (uint8_t)out_pitch);
    servoWrite(2, (uint8_t)out_roll);
}