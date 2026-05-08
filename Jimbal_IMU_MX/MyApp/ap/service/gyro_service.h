/* gyro_service.h */

#ifndef __AP_SERVICE_GIMBAL_H__
#define __AP_SERVICE_GIMBAL_H__


#include "gimbal_control.h" // gimbalUpdateFromIMU 보고용

#include "monitor.h"

// gimbal_control.h 안에 있음 
// #include "hw_def.h"
// #include "cli.h"            // cliPrintf, isMonitoringOn 사용
// #include "gyro.h"           // Gyro_Read, Gyro_Data_t 사용


void gimbalUpdate(void);

void gyroServiceUpdate(void);

// 서비스 초기화 (MPU6050 초기화 포함)
bool gyroServiceInit(void);
bool gyroServiceReInit(void); // 하드웨어 재초기화
bool gyroServiceIsOk(void);
// CLI 등에서 사용할 설정 함수
void gyroServiceSetPeriod(uint32_t period);

void gyroServiceSetMagData(Mag_Data_t *p_mag);

#endif