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