#include "camera_service.h"
#include "uart.h"
#include "cli.h" 

static camera_data_t cam_data;

camera_data_t* cameraGetLatestData(void) {
    return &cam_data;
}

void cameraServiceUpdate(void) {
    uint8_t data;
    static uint8_t buf[9]; // 7 -> 9바이트로 변경 (STX, CX(2), CY(2), AREA(2), DET, ETX)
    static uint8_t idx = 0;

    // Board A가 보낸 데이터는 USART1(또는 보드 설정에 따른 채널)을 통해 들어옵니다.
    while(uartAvailable(1) > 0) {
        data = uartRead(1);

        // 1. STX(0x02) 찾기
        if (idx == 0) {
            if (data == 0x02) {
                buf[idx++] = data;
            }
            continue;
        }

        // 2. 데이터 채우기 (9바이트가 될 때까지)
        if (idx < 9) {
            buf[idx++] = data;
        }

        // 3. 패킷 완성 및 파싱
        if (idx == 9) {
            // ETX(0x03) 확인 (인덱스 8번)
            if (buf[8] == 0x03) {
                // 좌표 데이터 (High | Low)
                cam_data.x = (int16_t)((buf[1] << 8) | buf[2]);
                cam_data.y = (int16_t)((buf[3] << 8) | buf[4]);
                
                // [추가] 면적 데이터 파싱 (High | Low)
                cam_data.area = (uint16_t)((buf[5] << 8) | buf[6]);
                
                // 인식 여부
                cam_data.is_detected = (buf[7] == 0x01);

                // 로그 출력 (면적 정보 포함)
                if (cam_data.is_detected) {
                    cliPrintf("[BOARD_CAM] CX=%d CY=%d AREA=%d\r\n", cam_data.x, cam_data.y, cam_data.area);
                } else {
                    cliPrintf("[BOARD_CAM] Target Lost\r\n");
                }
            } else {
                // 싱크가 맞지 않으면 인덱스 초기화 후 재검색
                idx = 0; 
                continue;
            }
            idx = 0; // 다음 패킷을 위해 초기화
        }
    }
}