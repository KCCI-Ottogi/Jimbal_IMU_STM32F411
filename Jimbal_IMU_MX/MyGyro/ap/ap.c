#include "ap.h"
#include "i2c.h"

// Task 구조체 메모리 할당 및 초기화
OS_Tasks_t os_tasks = {0, 0, 0};

uint8_t mpu_raw_data[14];
uint8_t hmc_raw_data[6];



// 애플리케이션 초기화 (main.c의 while(1) 진입 전 1회 실행)
void apInit(void) {
    // 하드웨어 계층 초기화
    SysTick_Init();
    I2C1_Init();
    EXTI_Init_For_MPU();
    
    // 센서 레지스터 초기화
    I2C_WriteReg(MPU6050_ADDR, 0x6B, 0x00); // Wake up
    I2C_WriteReg(MPU6050_ADDR, 0x38, 0x01); // Enable INT
    I2C_WriteReg(HMC5883L_ADDR, 0x02, 0x00); // Continuous mode
}

// 애플리케이션 메인 루프 (main.c의 while(1) 내에서 무한 반복)
void apMain(void) {
    // CPU는 평소 빈 루프를 돌다가(또는 Sleep), Flag가 세팅될 때만 연산을 수행합니다.
    MPU6050_Task();
    HMC5883L_Task();
}