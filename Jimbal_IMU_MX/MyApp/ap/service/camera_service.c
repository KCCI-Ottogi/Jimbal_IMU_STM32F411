#include "camera_service.h"

// 최신 카메라 데이터를 저장할 구조체
static camera_data_t cam_data;

// [PID 상태 변수] 이전 오차와 누적 오차를 기억하기 위함
static float error_x_sum = 0.0f, error_x_prev = 0.0f;
static float error_y_sum = 0.0f, error_y_prev = 0.0f;

void cameraServiceInit(void) {
    // 초기화 시 모든 데이터를 0으로 세팅
    cam_data.x = 80;
    cam_data.y = 60;
    cam_data.area = 0;
    cam_data.is_detected = false;

    error_x_sum = error_x_prev = 0.0f;
    error_y_sum = error_y_prev = 0.0f;
}

camera_data_t* cameraGetLatestData(void) {
    return &cam_data;
}



void cameraDataParsing(void) {
    uint8_t data;
    static uint8_t buf[8]; // 9바이트 패킷 버퍼[cite: 5]
    static uint8_t idx = 0;

    // UART로부터 데이터 수신 (Board A -> Board B)
    while(uartAvailable(1) > 0) {
        data = uartRead(1);

        // STX(0x02) 찾기
        if (idx == 0) {
            if (data == 0x02) buf[idx++] = data;
            continue;
        }

        // 데이터 채우기
        if (idx < 7) buf[idx++] = data;

        // 패킷 완성 (9바이트)
        if (idx >= 7) {
            if (buf[6] == 0x03) { // ETX 확인
                // 좌표 및 면적 파싱
                cam_data.x = (int16_t)((buf[1] << 8) | buf[2]);
                cam_data.y = (int16_t)((buf[3] << 8) | buf[4]);
                // cam_data.area = (uint16_t)((buf[5] << 8) | buf[6]);
                cam_data.is_detected = (buf[5] == 0x01);
                
                // [테스트 로그] STM32 터미널에서 확인용
                if (cam_data.is_detected) {
                    cliPrintf("[STM32_LOG] CX=%d CY=%d AREA=%d\r\n", cam_data.x, cam_data.y, cam_data.area);
                } else {
                    cliPrintf("[STM32_LOG] 타겟 없음\r\n");
                }
                // 파싱 완료 즉시 짐벌 제어 수행
                // cameraServicePIDUpdate(); 
                
            }
            idx = 0; // 다음 패킷 준비
        }
    }
}


void cameraServicePIDUpdate(void) {
    if (!cam_data.is_detected) {
        // 물체를 놓치면 급격한 회전을 방지하기 위해 누적 데이터 초기화[cite: 5]
        error_x_sum = error_x_prev = error_y_sum = error_y_prev = 0.0f;
        return;
    }

    // 기본 게인 설정 
    float Kp = 0.15f;
    float Ki = 0.005f; // 잔류 오차 제거용[cite: 5]
    float Kd = 0.02f;  // 진동 억제(브레이크)용

    // 면적이 크면(가까우면) 부드럽게, 작으면(멀면) 민감하게 반응
    if (cam_data.area > 5000) {      
        Kp = 0.10f; Kd = 0.04f; // 가까울 때: 감도 낮추고 브레이크 강화
    } else if (cam_data.area < 500) { 
        Kp = 0.22f; Ki = 0.008f; // 멀 때: 감도 높이고 끈기 있게 추적
    }

    float err_x = (float)(80 - cam_data.x);
    float err_y = (float)(cam_data.y - 60);

    error_x_sum += err_x;
    if (error_x_sum > 20.0f) error_x_sum = 20.0f;   // Anti-windup (한계 설정)
    if (error_x_sum < -20.0f) error_x_sum = -20.0f;
    float d_err_x = err_x - error_x_prev;
    float pid_x = (err_x * Kp) + (error_x_sum * Ki) + (d_err_x * Kd);
    error_x_prev = err_x;

    error_y_sum += err_y;
    if (error_y_sum > 20.0f) error_y_sum = 20.0f;
    if (error_y_sum < -20.0f) error_y_sum = -20.0f;
    float d_err_y = err_y - error_y_prev;
    float pid_y = (err_y * Kp) + (error_y_sum * Ki) + (d_err_y * Kd);
    error_y_prev = err_y;

    gimbalSettingCamOffset(pid_x, pid_y);
}