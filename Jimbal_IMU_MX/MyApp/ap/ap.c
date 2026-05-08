#include "ap.h"
#include "gimbal.h"
#include "camera_uart.h"
#include "cmsis_os2.h"
#include "hw.h"

void apInit(void)
{   
    hwInit(); 
    cameraUartInit();
    gimbalInit();
}

void apMain(void)
{
    while (1) {
        osDelay(100);
    }
}

// 100Hz (10ms) 주기로 실행되는 통합 제어 태스크
void gimbalSystemTask(void *argument)
{
    while (1)
    {
        cameraUartUpdate();     // 1. 카메라 데이터 수신
        gimbalUpdateSensors();  // 2. IMU 데이터 갱신
        gimbalExecuteControl(); // 3. 모터 제어 실행
        osDelay(10);
    }
}