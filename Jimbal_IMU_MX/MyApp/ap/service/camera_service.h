/* camera_service.h */
#ifndef __CAMERA_SERVICE_H__
#define __CAMERA_SERVICE_H__

#include "hw_def.h"
#include "cli.h"
// #include "gimbal_control.h" // 보정치 전달을 위해 추가 (servo.h는 제거 가능)



// 카메라 데이터를 담는 구조체
typedef struct {
    int16_t x;          // 물체의 X 좌표 (0~160)
    int16_t y;          // 물체의 Y 좌표 (0~120)[cite: 3, 5]
    uint16_t area;      // [추가] 물체의 면적 (거리 판단용)[cite: 3, 5]
    bool is_detected;   // 물체 인식 여부
} camera_data_t;

/**
 * @brief 카메라 서비스 초기화 (변수 및 상태 초기화)
 */
void cameraServiceInit(void);


/**
 * @brief 최신 파싱된 카메라 데이터 주소 반환
 */
camera_data_t* cameraGetLatestData(void);

/**
 * @brief UART로부터 9바이트 패킷을 수신하고 즉시 PID 기반 짐벌 제어를 수행
 */
void cameraDataParsing(void);


void cameraServicePIDUpdate(void);

// [수정: 제어기가 데이터를 가져갈 수 있도록 Getter 추가]
void cameraServiceGetPIDOffset(float *out_x, float *out_y);
#endif /* __CAMERA_SERVICE_H__ */