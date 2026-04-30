/* gimbal.h */

#ifndef __AP_SERVICE_GIMBAL_H__
#define __AP_SERVICE_GIMBAL_H__


#include "hw_def.h"
#include "gyro.h"   // 추가: IMU 데이터 참조용

#include "camera_service.h" // camera_data_t, cameraGetLatestData 정의 포함
#include "servo.h"          // servoSetTarget, servoGetCurrentAngle 정의 포함

// // 서보 상태 구조체
// typedef struct {
//   float current_angle;
//   float target_angle;
//   float k;          // 보간 계수 (0.05 ~ 0.2)
//   bool  is_moving;  // 현재 이동 중인지 확인용
// } service_servo_t;

// // 카메라 데이터 구조체 추가
// typedef struct {
//   int16_t x;
//   int16_t y;
//   bool is_detected;
// } camera_data_t;

// bool serviceServoInit(void);
// void serviceServoSetTarget(uint8_t ch, float angle, float speed);
// void serviceServoUpdate(void); // 태스크에서 호출할 함수


// // UART에서 호출할 데이터 업데이트 함수 추가
// void gimbalSetCamData(int16_t x, int16_t y, bool detected);
// void gimbalParseCamData(void);

/**
 * @brief 카메라 좌표를 바탕으로 짐벌의 목표 각도를 계산하고 업데이트합니다.
 *        내부적으로 cameraGetLatestData()를 호출하여 최신 데이터를 가져옵니다.
 */
void gimbalUpdate(void);

/**
 * @brief IMU(자이로/가속도) 센서 값을 바탕으로 짐벌의 수평 유지를 제어합니다.
 * @param roll  현재 기체의 Roll 각도
 * @param pitch 현재 기체의 Pitch 각도
 * @param yaw   현재 기체의 Yaw 각도
 */
void gimbalUpdateFromIMU(float roll, float pitch, float yaw);
void gimbalTaskLoop(void);
float gimbalGetCurrentAngle(uint8_t ch);
void gimbalReturnHome(void);
#endif