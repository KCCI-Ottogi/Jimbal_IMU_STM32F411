/* servo.c */
#include "servo.h"

extern TIM_HandleTypeDef htim2; // CubeMX에서 설정한 타이머 핸들러

void servoInit(void) {
    // TIM2의 채널 1(PA0)과 채널 2(PA1) PWM 시작
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void servoWrite(uint8_t ch, uint8_t angle) {
    if (angle > 180) angle = 180;

    // 84MHz 클럭, PSC 84-1, ARR 20000-1 설정 기준
    // 0도(0.5ms) = 500, 180도(2.5ms) = 2500
    uint32_t pulse_value = 500 + (angle * (2500 - 500) / 180);

    cliPrintf("Servo CH: %d, Angle: %d\r\n", ch, angle); 

    if (ch == 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_value);
    } else if (ch == 1) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_value);
    }
}


void servoScan(uint8_t ch) {
    cliPrintf("Starting Scan for Channel %d...\r\n", ch);
    
    // 0도에서 180도로 서서히 이동
    for (int i = 0; i <= 180; i += 5) {
        servoWrite(ch, i);
        cliPrintf("CH %d - Current Angle: %d\r\n", ch, i);
        HAL_Delay(100); // 각도 변화를 관찰할 수 있는 시간
    }
    
    // 다시 90도(중앙)로 복귀
    HAL_Delay(500);
    servoWrite(ch, 90);
    cliPrintf("Scan Complete. Returned to 90 degrees.\r\n");
}

