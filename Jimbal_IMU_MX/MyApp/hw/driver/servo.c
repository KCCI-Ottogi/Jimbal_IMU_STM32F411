/* servo.c */
#include "servo.h"
#include <math.h>

extern TIM_HandleTypeDef htim2; // CubeMX에서 설정한 타이머 핸들러

/**
 * [하드웨어 설정값 정의]
 * 지정하신 각 채널별 초기 정렬 위치 및 물리적 임계치
 */
#define SERVO_CH0_INIT   110.0f
#define SERVO_CH0_MAX    180

#define SERVO_CH1_INIT   65.0f
#define SERVO_CH1_MAX    130   // Pitch 축 하드웨어 제한
 
#define SERVO_CH2_INIT   70.0f
#define SERVO_CH2_MAX    180

// 3축 상태 저장소
static servo_smooth_t servo_list[3];

/**
 * @brief 서보 초기화: 지정된 초기값(110, 65, 70)으로 시작
 */
void servoInit(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); 

    float init_angles[3] = {SERVO_CH0_INIT, SERVO_CH1_INIT, SERVO_CH2_INIT};

    for(int i=0; i<3; i++) {
        servo_list[i].current_angle = init_angles[i];
        servo_list[i].target_angle = init_angles[i];
        servo_list[i].k = 0.1f; 
        servoWrite(i, (uint8_t)init_angles[i]);
    }
}

/**
 * @brief PWM 출력 및 임계치 보호
 */
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

/**
 * @brief [수정] 스캔 후 90도가 아닌 '초기값'으로 복귀
 */
void servoScan(uint8_t ch) {
    cliPrintf("Starting Scan for Channel %d...\r\n", ch);
    uint8_t max_l = (ch == 1) ? SERVO_CH1_MAX : 180;

    for (int i = 0; i <= max_l; i += 5) {
        servoWrite(ch, i);
        cliPrintf("CH %d - Current Angle: %d\r\n", ch, i);
        HAL_Delay(100);
    }
    
    // 복귀 시 채널별 초기값 사용
    float return_angle = SERVO_CH0_INIT;
    if(ch == 1) return_angle = SERVO_CH1_INIT;
    else if(ch == 2) return_angle = SERVO_CH2_INIT;

    HAL_Delay(500);
    servoWrite(ch, (uint8_t)return_angle);
    cliPrintf("Scan Complete. Returned to Init Position (%.1f).\r\n", return_angle);
}

/**
 * @brief [수정] 듀얼 테스트 후 초기값으로 복귀
 */
void servoDualTest(void) {
    cliPrintf("Dual Servo Sync Test Start...\r\n");
    
    for(int i=0; i<=130; i++) { 
        servoWrite(2, i);  
        servoWrite(1, 130-i); 
        if (i % 20 == 0) cliPrintf("Y:%d, P:%d\r\n", i, 130-i);
        HAL_Delay(15); 
    }

    // 초기값으로 복귀
    servoWrite(1, (uint8_t)SERVO_CH1_INIT);
    servoWrite(2, (uint8_t)SERVO_CH2_INIT);
    cliPrintf("Dual Test Done. Returned to Home.\r\n");
}

/**
 * @brief [수정] 전체 테스트 완료 후 초기값(110, 65, 70)으로 복귀
 */
void servoTotalTest(void) {
    cliPrintf("======= [Gimbal Full Range Test] =======\r\n");

    for (int i = 0; i <= 180; i++) {
        servoWrite(0, i);
        if (i <= SERVO_CH1_MAX) servoWrite(1, i);
        servoWrite(2, i);
        if (i % 30 == 0) cliPrintf("Moving... %d deg\r\n", i);
        HAL_Delay(15); 
    }

    HAL_Delay(500);

    // 최종 복귀 위치 수정
    cliPrintf("Test Complete. Moving to Home (110, 65, 70).\r\n");
    servoWrite(0, (uint8_t)SERVO_CH0_INIT);
    servoWrite(1, (uint8_t)SERVO_CH1_INIT);
    servoWrite(2, (uint8_t)SERVO_CH2_INIT);
}

void servoSetTarget(uint8_t ch, float target, float k) {
    if (ch > 2) return;
    float max_l = (ch == 1) ? (float)SERVO_CH1_MAX : 180.0f;

    if (target < 0.0f) target = 0.0f;
    if (target > max_l) target = max_l;

    servo_list[ch].target_angle = target;
    if (k > 0.0f) servo_list[ch].k = k;
}

void servoSmoothUpdate(void) {
    for (int i = 0; i < 3; i++) {
        float diff = servo_list[i].target_angle - servo_list[i].current_angle;
        if (fabsf(diff) > 0.05f) {
            servo_list[i].current_angle += diff * servo_list[i].k;
            servoWrite(i, (uint8_t)roundf(servo_list[i].current_angle));
        }
    }
}


/**
 * @brief 이미 설정된(Target) 값으로 도달할 때까지 루프를 돌며 이동
 */
void servoRunToTarget(uint8_t ch) {
    if (ch > 2) return;

    // 이미 servoSetTarget이나 servoSweep이 
    // servo_list[ch].target_angle과 k를 셋팅해놨으므로 그걸 그냥 씁니다.
    while (1) {
        float diff = servo_list[ch].target_angle - servo_list[ch].current_angle;

        // 거의 도달하면 정수 값 맞추고 종료
        if (fabsf(diff) < 0.1f) {
            servo_list[ch].current_angle = servo_list[ch].target_angle;
            servoWrite(ch, (uint8_t)roundf(servo_list[ch].current_angle));
            break;
        }

        // 보간 연산 및 출력
        servo_list[ch].current_angle += diff * servo_list[ch].k;
        servoWrite(ch, (uint8_t)roundf(servo_list[ch].current_angle));

        // 서보 이동 시간 확보
        HAL_Delay(20);
    }
}

void servoSetTargetAll(float target0, float target1, float target2, float k) {
    servoSetTarget(0, target0, k);
    servoSetTarget(1, target1, k);
    servoSetTarget(2, target2, k);
}
/* servo.c */

/**
 * @brief 목표 각도에 도달할 때까지 루프를 돌며 부드럽게 이동
 * @param ch: 채널, target: 목표 각도, k: 부드러움 계수 (0.01 ~ 0.2)
 */
void servoMoveToTarget(uint8_t ch, float target, float k) {
    if (ch > 2) return;

    // 1. 하드웨어 보호 및 목표값 저장
    float max_l = (ch == 1) ? (float)SERVO_CH1_MAX : 180.0f;
    if (target < 0.0f) target = 0.0f;
    if (target > max_l) target = max_l;
    
    servo_list[ch].target_angle = target;
    if (k > 0.0f) servo_list[ch].k = k;

    // 2. [핵심] 목표에 도달할 때까지 여기서 직접 돌립니다.
    while (1) {
        float diff = servo_list[ch].target_angle - servo_list[ch].current_angle;

        // 차이가 0.1도 미만이면 도착으로 간주하고 루프 탈출
        if (fabsf(diff) < 0.1f) {
            servo_list[ch].current_angle = servo_list[ch].target_angle;
            servoWrite(ch, (uint8_t)roundf(servo_list[ch].current_angle));
            break;
        }

        // 부드럽게 한 단계 이동
        servo_list[ch].current_angle += diff * servo_list[ch].k;
        
        // 실제 PWM 출력
        servoWrite(ch, (uint8_t)roundf(servo_list[ch].current_angle));

        // 서보가 물리적으로 움직일 시간을 줍니다 (매우 중요)
        HAL_Delay(20); 
    }
}
/**
 * @brief [수정] 스위핑 기준을 초기값으로 변경
 */
void servoSweep(uint8_t ch, float k) {
    if (ch > 2) return;
    
    float home = SERVO_CH0_INIT;
    float max_l = 180.0f;
    if(ch == 1) { home = SERVO_CH1_INIT; max_l = (float)SERVO_CH1_MAX; }
    else if(ch == 2) { home = SERVO_CH2_INIT; }

    // 현재 위치가 초기값보다 작으면 최대치로, 크면 0도로 보냄
    float target = (servo_list[ch].current_angle < home) ? max_l : 0.0f;
    servoSetTarget(ch, target, k);
}
/* servo.c 하단 보조 함수 부분 */

// 현재 각도 설정 (필요시 강제 보정용)
void servoSetCurrentAngle(uint8_t ch, float angle) {
    if (ch > 2) return;
    servo_list[ch].current_angle = angle;
}

// [수정] 현재 각도 읽기: 잘못된 채널일 경우 채널별 초기값 반환
float servoGetCurrentAngle(uint8_t ch) {
    if (ch == 0) return servo_list[0].current_angle;
    if (ch == 1) return servo_list[1].current_angle;
    if (ch == 2) return servo_list[2].current_angle;
    
    // 잘못된 채널 인덱스(ch > 2)가 들어온 경우 
    // 가장 안전한 기본값인 0번 채널 초기값으로 대응
    return SERVO_CH0_INIT; 
}

// [수정] 목표 각도 읽기: 잘못된 채널일 경우 채널별 초기값 반환
float servoGetTargetAngle(uint8_t ch) {
    if (ch == 0) return servo_list[0].target_angle;
    if (ch == 1) return servo_list[1].target_angle;
    if (ch == 2) return servo_list[2].target_angle;
    
    return SERVO_CH0_INIT;
}

// [수정] 부드러움 계수 읽기
float servoGetK(uint8_t ch) {
    if (ch > 2) return 0.1f; // k값은 공통 기본값 사용
    return servo_list[ch].k;
}

/**
 * @brief 채널별 설정된 초기(Home) 각도를 반환
 */
float servoGetInitAngle(uint8_t ch) {
    if (ch == 0) return SERVO_CH0_INIT;
    if (ch == 1) return SERVO_CH1_INIT;
    if (ch == 2) return SERVO_CH2_INIT;
    return 90.0f; // 예외 상황
}