#include "hw.h"

void hwInit(void)
{
    // 통신 및 모터 초기화
    uartInit();
    servoInit();
    
    // 센서 초기화
    Gyro_Init();
    Mag_Init();
}