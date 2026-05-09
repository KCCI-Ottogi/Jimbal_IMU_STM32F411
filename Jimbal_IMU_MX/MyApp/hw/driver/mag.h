#ifndef MAG_H
#define MAG_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t mag_x, mag_y, mag_z;
} Mag_Data_t;

// [추가] 8자 캘리브레이션으로 획득한 하드 아이언 오프셋
// #define MAG_OFFSET_X (-270)
// #define MAG_OFFSET_Y (-9)
// #define MAG_OFFSET_Z (-445)

// [수정] 새롭게 구한 하드 아이언 오프셋 적용
#define MAG_OFFSET_X (192)
#define MAG_OFFSET_Y (46)
#define MAG_OFFSET_Z (-96)


bool Mag_Init(void);
void Mag_StartAuto(void);
void Mag_StopAuto(void);
bool Mag_Read(Mag_Data_t *pData);

void Mag_SetCalibrationMode(bool enable);

#endif // MAG_H