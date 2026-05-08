#ifndef __AP_AP_H_
#define __AP_AP_H_  

// #include "def.h"
// #include "hw_def.h"
// #include "bsp.h"
// #include "hw.h"

// #include "gimbal_control.h"//없어도 됨
#include "gyro_service.h"

void apInit(void);
void apMain(void);
void apStopAutoTask(void);
void apSyncPeriods(uint32_t period);

// FreeRTOS Task 함수 선언
void ledSystemTask(void *argument);
void monitorSystemTask(void *argument);
void gyroSystemTask(void *argument); 
void magSystemTask(void *argument);  
void startMonitorStartTask(void *argument);
void gimbalSystemTask(void *argument);

void showStack(void);
#endif //__AP_AP_H_
