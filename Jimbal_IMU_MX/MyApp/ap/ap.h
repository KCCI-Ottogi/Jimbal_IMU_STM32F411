#ifndef AP_AP_H_
#define AP_AP_H_

#include "def.h"
#include "hw_def.h"
#include "bsp.h"
#include "hw.h"
#include "monitor.h"
// #include "servo.h" // 추가

void apInit(void);
void apMain(void);
void apStopAutoTask(void);

#endif // AP_AP_H_
