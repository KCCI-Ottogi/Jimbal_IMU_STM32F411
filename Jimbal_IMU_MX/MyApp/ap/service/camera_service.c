#include "camera_service.h"
#include "uart.h"
#include "cli.h" 

static camera_data_t cam_data;

camera_data_t* cameraGetLatestData(void) {
    return &cam_data;
}

void cameraServiceUpdate(void) {
    uint8_t data;
    static uint8_t buf[10]; // 패킷 길이를 10으로 확장
    static uint8_t idx = 0;

    while(uartAvailable(1) > 0) {
        data = uartRead(1);

        if (idx == 0 && data == 0x02) { // STX (시작)
            buf[idx++] = data;
            continue;
        }

        if (idx > 0 && idx < 10) {
            buf[idx++] = data;
        }

        if (idx == 10) {
            if (buf[9] == 0x03) { // ETX (종료)
                // 체크섬: X(2) ^ Y(2) ^ Detect(1) ^ Area(2)
                uint8_t checksum = buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ^ buf[6] ^ buf[7];
                
                if (checksum == buf[8]) {
                    cam_data.x = (int16_t)((buf[1] << 8) | buf[2]);
                    cam_data.y = (int16_t)((buf[3] << 8) | buf[4]);
                    cam_data.is_detected = (buf[5] == 0x01);
                    cam_data.area = (uint16_t)((buf[6] << 8) | buf[7]);

                    // [테스트 로그] STM32 터미널에서 확인용
                    if (cam_data.is_detected) {
                        cliPrintf("[STM32_LOG] CX=%d CY=%d AREA=%d\r\n", cam_data.x, cam_data.y, cam_data.area);
                    } else {
                        cliPrintf("[STM32_LOG] 타겟 없음\r\n");
                    }
                }
            }
            idx = 0;
        }
    }
}