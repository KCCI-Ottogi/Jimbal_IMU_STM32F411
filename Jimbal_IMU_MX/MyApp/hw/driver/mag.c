/* mag.c */
#include "mag.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include "cli.h"

#define RAD_TO_DEG 57.29577951f

extern I2C_HandleTypeDef hi2c1;
#define HMC5883L_ADDR (0x1E << 1)

static uint32_t last_log_time = 0;

// 캘리브레이션 관련 변수
static bool is_calibrating = false;
static int16_t offset_x = 0, offset_y = 0, offset_z = 0; 
static int16_t max_x = -32000, min_x = 32000;
static int16_t max_y = -32000, min_y = 32000;
static int16_t max_z = -32000, min_z = 32000;

bool Mag_Init(void) {
    uint8_t data;
    data = 0x70; 
    if (HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x00, 1, &data, 1, 100) != HAL_OK) return false;
    
    data = 0x20; 
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x01, 1, &data, 1, 100);
    
    data = 0x00; 
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
    return true;
}

void Mag_StartAuto(void) {
    uint8_t data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
}

void Mag_StopAuto(void) {
    uint8_t data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
}

// CLI 명령어로 호출되는 캘리브레이션 모드 셋팅
void Mag_SetCalibrationMode(bool enable) {
    is_calibrating = enable;
    if (enable) {
        max_x = max_y = max_z = -32000;
        min_x = min_y = min_z = 32000;
        cliPrintf("\r\n[MAG] Calibration Started! 8자 모양으로 기체를 돌려주세요.\r\n");
    } else {
        offset_x = (max_x + min_x) / 2;
        offset_y = (max_y + min_y) / 2;
        offset_z = (max_z + min_z) / 2;
        cliPrintf("\r\n[MAG] Calibration Done!\r\n");
        cliPrintf("적용된 오프셋 -> X: %d, Y: %d, Z: %d\r\n", offset_x, offset_y, offset_z);
        cliPrintf("이 값을 mag.h의 MAG_OFFSET 매크로에 업데이트 하세요!\r\n");
    }
}

// 캘리브레이션 값 추적용 내부 함수
static void Mag_TrackCalibration(int16_t rx, int16_t ry, int16_t rz) {
    if (rx > max_x) max_x = rx;
    if (rx < min_x) min_x = rx;
    if (ry > max_y) max_y = ry;
    if (ry < min_y) min_y = ry;
    if (rz > max_z) max_z = rz;
    if (rz < min_z) min_z = rz;

    if (HAL_GetTick() - last_log_time >= 200) {
        last_log_time = HAL_GetTick();
        cliPrintf("[CALIB] X:[%5d ~ %5d] | Y:[%5d ~ %5d] | Z:[%5d ~ %5d]\r\n", 
                  min_x, max_x, min_y, max_y, min_z, max_z);
    }
}

bool Mag_Read(Mag_Data_t *pData) {
    uint8_t raw[6];
    uint8_t status = 0;

    HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x09, 1, &status, 1, 10);
    
    // ------------------------------------------------------------------
    // 🔨 [락 브레이커] 데이터가 없거나(0x01 == 0), 잠겼을 때(0x02 != 0)
    // ------------------------------------------------------------------
    if ((status & 0x01) == 0 || (status & 0x02) != 0) {
        uint8_t mode = 0x00; 
        HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, &mode, 1, 10);
    }

    if (HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x03, 1, raw, 6, 50) != HAL_OK) return false;
    
    int16_t raw_x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t raw_z = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t raw_y = (int16_t)((raw[4] << 8) | raw[5]);

    if (is_calibrating) {
        Mag_TrackCalibration(raw_x, raw_y, raw_z);
        pData->mag_x = raw_x; 
        pData->mag_y = raw_y; 
        pData->mag_z = raw_z;
    } else {
        // 1. 노이즈 제거 (영점 보정)
        int16_t clean_x = raw_x - MAG_OFFSET_X; 
        int16_t clean_y = raw_y - MAG_OFFSET_Y; 
        int16_t clean_z = raw_z - MAG_OFFSET_Z;
        
        // ---------------------------------------------------------
        // 🎯 2. [최종 정답] 기체 NED 좌표계 매핑
        // 데이터 분석 결과: 센서의 X가 정면, Y가 좌측, Z가 하늘을 보고 있음
        // ---------------------------------------------------------
        pData->mag_x =  clean_x;    // 정면(Front) : 그대로 X
        pData->mag_y =  clean_y;    // 우측(Right) : 센서 Y가 좌측이므로 부호를 반전시켜야 할 수도 있지만, 
                                    // 일단 현재 clean_y의 움직임상 방향을 뒤집어 줍니다.
        pData->mag_z =  clean_z;    // 바닥(Down)  : 센서 Z
        
        // ---------------------------------------------------------
        // 💡 3. 실제 각도(Yaw)가 제대로 도는지 확인하기 위한 출력
        // ---------------------------------------------------------
        if (HAL_GetTick() - last_log_time >= 200) {
            last_log_time = HAL_GetTick();
            
            // atan2를 이용한 나침반 각도 계산 (기울기 보정 전 단순 각도)
            float mag_yaw = atan2f((float)pData->mag_y, (float)pData->mag_x) * RAD_TO_DEG;
            if(mag_yaw < 0) mag_yaw += 360.0f;

            cliPrintf("[MAG CHECK] Yaw: %5.1f | F:%4d | R:%4d\r\n", mag_yaw, pData->mag_x, pData->mag_y);
        }
    }
    return true;
}