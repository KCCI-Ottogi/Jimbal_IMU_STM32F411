#include "camera_uart.h"
#include "uart.h"
#include "bsp.h" // millis() 사용

static int object_err_x = 0;
static int object_err_y = 0;
static uint32_t last_receive_time = 0;

void cameraUartInit(void) {
    object_err_x = 0;
    object_err_y = 0;
}

void cameraUartUpdate(void) {
    static char buf[64];
    static uint8_t idx = 0;

    while (uartAvailable(1) > 0) {
        char c = (char)uartRead(1);
        
        if (c == '\n' || c == '\r') {
            buf[idx] = '\0';
            if (idx > 0) {
                // "X:15,Y:-20" 포맷으로 수신
                if (sscanf(buf, "X:%d,Y:%d", &object_err_x, &object_err_y) == 2) {
                    last_receive_time = millis();
                }
            }
            idx = 0;
        } else {
            if (idx < sizeof(buf) - 1) buf[idx++] = c;
        }
    }

    // 0.5초 이상 데이터가 안 들어오면 추적 중지
    if (millis() - last_receive_time > 500) {
        object_err_x = 0;
        object_err_y = 0;
    }
}

int cameraGetErrorX(void) { return object_err_x; }
int cameraGetErrorY(void) { return object_err_y; }