#include "gyro.h"
#include "i2c.h"
#include "ap.h"

static void EXTI_Init_For_MPU(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    GPIOA->MODER &= ~GPIO_MODER_MODER0; 
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0; 

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; 
    EXTI->IMR |= EXTI_IMR_MR0;    
    EXTI->RTSR |= EXTI_RTSR_TR0;  

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_SetPriority(EXTI0_IRQn, 2); 
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR = EXTI_PR_PR0; 
        os_tasks.gyro_ready_flag = 1; // Wake up AP layer
    }
}

void Gyro_Init(void) {
    EXTI_Init_For_MPU();
    
    // Wake up & Enable Data Ready Interrupt
    I2C_WriteReg(MPU6050_ADDR, 0x6B, 0x00); 
    I2C_WriteReg(MPU6050_ADDR, 0x38, 0x01); 
}

void Gyro_Task(void) {
    uint8_t raw[14];
    I2C_ReadBurst(MPU6050_ADDR, 0x3B, raw, 14);
    
    // 데이터 파싱 후 ap.c의 글로벌 모니터링 구조체에 저장
    sensor_data.accel_x = (raw[0] << 8) | raw[1];
    sensor_data.accel_y = (raw[2] << 8) | raw[3];
    sensor_data.accel_z = (raw[4] << 8) | raw[5];
    sensor_data.gyro_x  = (raw[8] << 8) | raw[9];
    sensor_data.gyro_y  = (raw[10] << 8) | raw[11];
    sensor_data.gyro_z  = (raw[12] << 8) | raw[13];
}