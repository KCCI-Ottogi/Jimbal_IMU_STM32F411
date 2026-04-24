#ifndef AP_H
#define AP_H

#include <stdint.h>

// RTOS Task 관리를 위한 구조체
typedef struct {
    uint8_t mpu6050_ready_flag;
    uint8_t hmc5883l_time_flag;
    uint32_t system_tick;
} OS_Tasks_t;

// 외부 모듈(ISR)에서 접근할 수 있도록 extern 선언
extern OS_Tasks_t os_tasks;

// Application Functions
void apInit(void);
void apMain(void);

#endif /* AP_H */