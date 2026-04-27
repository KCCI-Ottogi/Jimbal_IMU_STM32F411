#include "mag.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
#define HMC5883L_ADDR (0x1E << 1)

bool Mag_Init(void) {
    uint8_t data = 0x70; // 8-average, 15Hz
    if (HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x00, 1, &data, 1, 100) != HAL_OK) return false;
    
    data = 0x03; // 초기엔 Idle 모드 (전력소모 방지)
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
    return true;
}

void Mag_StartAuto(void) {
    uint8_t data = 0x00; // Continuous 측정 모드
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
}

void Mag_StopAuto(void) {
    uint8_t data = 0x03; // Idle 모드
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
}

bool Mag_Read(Mag_Data_t *pData) {
    uint8_t raw[6];
    if (HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x03, 1, raw, 6, 10) != HAL_OK) return false;
    
    pData->mag_x = (int16_t)(raw[0] << 8 | raw[1]);
    pData->mag_z = (int16_t)(raw[2] << 8 | raw[3]);
    pData->mag_y = (int16_t)(raw[4] << 8 | raw[5]);
    return true;
}