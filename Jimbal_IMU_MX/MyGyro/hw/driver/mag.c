#include "mag.h"
#include "i2c.h"
#include "ap.h"

void Mag_Init(void) {
    // Continuous-Measurement Mode
    I2C_WriteReg(HMC5883L_ADDR, 0x02, 0x00); 
}

void Mag_Task(void) {
    uint8_t raw[6];
    I2C_ReadBurst(HMC5883L_ADDR, 0x03, raw, 6);
    
    // 데이터 파싱 후 ap.c의 글로벌 모니터링 구조체에 저장
    sensor_data.mag_x = (raw[0] << 8) | raw[1];
    sensor_data.mag_z = (raw[2] << 8) | raw[3]; // HMC5883L은 X, Z, Y 순서로 출력됨
    sensor_data.mag_y = (raw[4] << 8) | raw[5];
}