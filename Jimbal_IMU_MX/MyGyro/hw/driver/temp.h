#ifndef __HW_DRIVER_TEMP_H__
#define __HW_DRIVER_TEMP_H__

#include "hw_def.h"

bool tempInit(void);
float tempReadAuto(void);
float tempReadSingle(void);

void tempStartAuto(void);
void tempStopAuto(void);
#endif //__HW_DRIVER_TEMP_H__