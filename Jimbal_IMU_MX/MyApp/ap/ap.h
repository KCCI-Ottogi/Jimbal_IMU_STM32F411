#ifndef __AP_AP_H_
#define __AP_AP_H_  

#include "hw_def.h"

void apInit(void);
void apMain(void);

// FreeRTOS Task 함수 선언
void gimbalSystemTask(void *argument);

#endif //__AP_AP_H_