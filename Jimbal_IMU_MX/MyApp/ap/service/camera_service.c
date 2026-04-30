/* camera_service.c */
#include "camera_service.h"
#include "uart.h"

static camera_data_t cam_data;

// 외부(gimbal)에서 데이터를 읽어갈 수 있도록 Getter 제공
camera_data_t* cameraGetLatestData(void) {
    return &cam_data;
}

// ESP32-CAM 전용 UART 파싱 (이전 gimbalParseCamData 로직 이동)
void cameraServiceUpdate(void) {
    uint8_t data;
    static uint8_t buf[8]; 
    static uint8_t idx = 0;

    while(uartAvailable(1) > 0) {
        data = uartRead(1);

        if (idx == 0 && data == 0x02) { // STX
            buf[idx++] = data;
            continue;
        }

        if (idx > 0 && idx < 8) {
            buf[idx++] = data;
        }

        if (idx == 8) {
            if (buf[7] == 0x03) { // ETX
                uint8_t checksum = buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5];
                if (checksum == buf[6]) {
                    cam_data.x = (int16_t)((buf[1] << 8) | buf[2]);
                    cam_data.y = (int16_t)((buf[3] << 8) | buf[4]);
                    cam_data.is_detected = (buf[5] == 0x01);
                }
            }
            idx = 0;
        }
    }
}