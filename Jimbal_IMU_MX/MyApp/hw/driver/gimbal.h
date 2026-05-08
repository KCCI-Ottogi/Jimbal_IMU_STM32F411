#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "hw_def.h"

void gimbalInit(void);
void gimbalUpdateSensors(void);
void gimbalExecuteControl(void);

#endif // __GIMBAL_H__