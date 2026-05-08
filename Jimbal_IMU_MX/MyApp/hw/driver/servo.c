#include "servo.h"

extern TIM_HandleTypeDef htim2;

void servoInit(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); 

    servoWrite(0, (uint8_t)SERVO_CH0_INIT);
    servoWrite(1, (uint8_t)SERVO_CH1_INIT);
    servoWrite(2, (uint8_t)SERVO_CH2_INIT);
}

void servoWrite(uint8_t ch, uint8_t angle) {
    uint8_t max_limit = 180;
    if (ch == 1) max_limit = SERVO_CH1_MAX;

    if (angle > max_limit) angle = max_limit;
    if (angle < 0) angle = 0;

    uint32_t pulse_value = 500 + (angle * (2500 - 500) / 180);

    if (ch == 0)      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_value);
    else if (ch == 1) __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_value);
    else if (ch == 2) __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_value);
}