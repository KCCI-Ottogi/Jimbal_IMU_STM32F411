#ifndef _HW_DRIVER_TEMP_H_
#define _HW_DRIVER_TEMP_H_

#include "hw_def.h"
// #include "adc.h"
// #include "cmsis_os2.h"
#include "stm32f4xx_hal_adc.h"
#include <sys/types.h>

bool tempInit();
float tempReadAuto();
float tempReadSingle();


void tempStartAuto();
void tempStopAuto();

#endif //_HW_DRIVER_TEMP_H_
