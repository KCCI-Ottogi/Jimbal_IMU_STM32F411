/* camera_service.h */
#ifndef __CAMERA_SERVICE_H__
#define __CAMERA_SERVICE_H__

#include "hw_def.h"

typedef struct {
    int16_t x;
    int16_t y;
    uint16_t area;      // [추가] 면적 데이터
    bool is_detected;
} camera_data_t;

void cameraServiceUpdate(void);
camera_data_t* cameraGetLatestData(void);

#endif