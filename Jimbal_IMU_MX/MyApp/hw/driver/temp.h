<<<<<<< HEAD
// #ifndef __HW_DRIVER_TEMP_H__
// #define __HW_DRIVER_TEMP_H__

// #include "hw_def.h"

// bool tempInit(void);
// float tempReadAuto(void);
// float tempReadSingle(void);

// void tempStartAuto(void);
// void tempStopAuto(void);
// #endif //__HW_DRIVER_TEMP_H__
=======
#ifndef _HW_DRIVER_TEMP_H_
#define _HW_DRIVER_TEMP_H_

#include "hw_def.h"

bool tempInit();
float tempReadAuto();
float tempReadSingle();


void tempStartAuto();
void tempStopAuto();

#endif //_HW_DRIVER_TEMP_H_
>>>>>>> origin/es_servo
