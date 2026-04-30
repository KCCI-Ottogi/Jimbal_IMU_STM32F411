#include "camera_service.h"
#include "uart.h"
#include "cli.h" 

static camera_data_t cam_data;

camera_data_t* cameraGetLatestData(void) {
    return &cam_data;
}
void cameraServiceUpdate(void) {
    uint8_t data;
    static uint8_t buf[7]; // 10 -> 7바이트로 변경
    static uint8_t idx = 0;

    while(uartAvailable(1) > 0) {
        data = uartRead(1);

        // 1. STX 찾기
        if (idx == 0) {
            if (data == 0x02) {
                buf[idx++] = data;
            }
            continue;
        }

        // 2. 데이터 채우기
        if (idx < 7) {
            buf[idx++] = data;
        }

        // 3. 패킷 완성 (7바이트)
        if (idx == 7) {
            // ETX(0x03) 확인
            if (buf[6] == 0x03) {
                // 상대방 패킷에는 체크섬이 없으므로 바로 파싱 진행
                cam_data.x = (int16_t)((buf[1] << 8) | buf[2]);
                cam_data.y = (int16_t)((buf[3] << 8) | buf[4]);
                cam_data.is_detected = (buf[5] == 0x01);
                // cam_data.area = 0; 

                // 로그 출력
                if (cam_data.is_detected) {
                    cliPrintf("[STM32_LOG] CX=%d CY=%d \r\n", cam_data.x, cam_data.y);
                } else {
                    cliPrintf("[STM32_LOG] Target Lost\r\n");
                }
            } else {
                // 마지막 바이트가 0x03이 아니면 데이터가 밀린 것이므로 초기화 후 재검색
                idx = 0; 
                continue;
            }
            idx = 0; // 다음 패킷을 위해 인덱스 초기화
        }
    }
}