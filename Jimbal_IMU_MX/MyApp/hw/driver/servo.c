/* servo.c */
#include "servo.h"

extern TIM_HandleTypeDef htim2; // CubeMX에서 설정한 타이머 핸들러

void servoInit(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Yaw (PA0)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Pitch (PA1)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // Roll (PB10) 추가
}

void servoWrite(uint8_t ch, uint8_t angle) {
    if (angle > 180) angle = 180;

    // 84MHz 클럭, PSC 84-1, ARR 20000-1 설정 기준
    // 0도(0.5ms) = 500, 180도(2.5ms) = 2500
    uint32_t pulse_value = 500 + (angle * (2500 - 500) / 180);

    // cliPrintf("Servo CH: %d, Angle: %d\r\n", ch, angle); 

    if (ch == 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_value);
    } else if (ch == 1) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_value);
    } else if (ch == 2) { // Roll 축 추가 (TIM2_CH3)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_value);
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

void servoDualTest(void) {
    cliPrintf("Dual Servo Sync Test Start...\r\n");
    
    for(int i=0; i<=180; i++) { 
        servoWrite(0, i);      // Pan: 0 -> 180
        servoWrite(1, 180-i);  // Tilt: 180 -> 0
        
        // 너무 잦은 printf는 모터 동작을 방해하므로 20도마다만 출력
        if (i % 20 == 0) {
            cliPrintf("P:%d, T:%d\r\n", i, 180-i);
        }

        // 딜레이를 15~20ms 정도로 줄임 (이 수치가 작을수록 빨라짐)
        HAL_Delay(15); 
    }
    servoWrite(0, 90);
    servoWrite(1, 90);
    cliPrintf("Dual Test Done.\r\n");
}

void servoTotalTest(void) {
    cliPrintf("======= [Gimbal 3-Axis Full Range Test] =======\r\n");
    cliPrintf("Range: 0 -> 180 -> 0 degrees\r\n");
    cliPrintf("Press 'Ctrl+C' to stop (if implemented)\r\n");

    // 0도에서 180도까지 정방향 이동
    for (int i = 0; i <= 180; i++) {
        servoWrite(0, i);       // Yaw
        servoWrite(1, i);       // Pitch
        servoWrite(2, i);       // Roll
        
        // 30도마다 현재 각도 출력
        if (i % 30 == 0) {
            cliPrintf("Moving to: %d deg\r\n", i);
        }
        
        // 속도 조절: 15ms * 180 step = 약 2.7초 소요
        HAL_Delay(15); 
    }

    HAL_Delay(500); // 180도 지점에서 잠시 대기

    // 180도에서 0도까지 역방향 복귀
    for (int i = 180; i >= 0; i--) {
        servoWrite(0, i);
        servoWrite(1, i);
        servoWrite(2, i);
        
        if (i % 30 == 0) {
            cliPrintf("Returning to: %d deg\r\n", i);
        }
        
        HAL_Delay(15);
    }

    // 최종적으로 안정적인 중앙 위치(90도)로 이동
    cliPrintf("Test Complete. Moving to Home(90, 90, 90).\r\n");
    HAL_Delay(500);
    servoWrite(0, 90);
    servoWrite(1, 90);
    servoWrite(2, 90);
}