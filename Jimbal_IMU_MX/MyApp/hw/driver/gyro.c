#include "gyro.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
#define MPU6050_ADDR (0x68 << 1)

bool Gyro_Init(void) {
    uint8_t check;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 100) != HAL_OK) return false;
    
    if (check == 0x68) {
        uint8_t data = 0x00; // 절전 모드 해제
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);
        return true;
    }
    return false;
}

void Gyro_StartAuto(void) {
    uint8_t data = 0x01; // Data Ready 인터럽트 활성화
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x38, 1, &data, 1, 100);
}

void Gyro_StopAuto(void) {
    uint8_t data = 0x00; // 인터럽트 비활성화
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x38, 1, &data, 1, 100);
}

bool Gyro_Read(Gyro_Data_t *pData) {
    uint8_t raw[14];
    // 이 함수가 호출되어 데이터를 읽어야 INT 핀이 LOW로 초기화됩니다.
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, raw, 14, 10) != HAL_OK) return false;
    
    pData->accel_x = (int16_t)(raw[0] << 8 | raw[1]);
    pData->accel_y = (int16_t)(raw[2] << 8 | raw[3]);
    pData->accel_z = (int16_t)(raw[4] << 8 | raw[5]);
    pData->gyro_x  = (int16_t)(raw[8] << 8 | raw[9]);
    pData->gyro_y  = (int16_t)(raw[10] << 8 | raw[11]);
    pData->gyro_z  = (int16_t)(raw[12] << 8 | raw[13]);
    return true;
}