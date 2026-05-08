#ifndef __CAMERA_UART_H__
#define __CAMERA_UART_H__

#include "hw_def.h"

void cameraUartInit(void);
void cameraUartUpdate(void);
int cameraGetErrorX(void);
int cameraGetErrorY(void);

#endif