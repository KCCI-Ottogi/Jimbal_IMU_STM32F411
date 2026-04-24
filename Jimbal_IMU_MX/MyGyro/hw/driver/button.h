#ifndef __HW_DRIVER_BUTTON_H_
#define __HW_DRIVER_BUTTON_H__
#include "hw_def.h"
#include "cli.h"
// extern void cliPrintf(const char *fmt, ...);

#define BUTTON_MAX_CH 1

void buttonInit(void);
void buttonEnable(bool enable);
bool buttonGetEnable(void);

#endif // __HW_DRIVER_BUTTON_H_