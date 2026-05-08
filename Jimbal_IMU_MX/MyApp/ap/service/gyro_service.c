/* gyro_service.c */

#include "gyro_service.h"



/* --- 내부 정적 변수 --- */
static bool is_gyro_ok = false;
static uint32_t imu_report_period = 0;
static Mag_Data_t current_magData; // 지자기 데이터 저장용
static uint32_t last_calc_tick = 0;
static uint32_t last_print_tick = 0;
extern osSemaphoreId_t GyroReadySemHandle;
static uint32_t   mag_updated_tick = 0; // 데이터 신선도 체크용




void gyroServiceSetMagData(Mag_Data_t *p_mag) {
    if (p_mag == NULL) return;
    
    // Mutex가 있다면 여기서 Lock
    current_magData = *p_mag;
    mag_updated_tick = osKernelGetTickCount();
    // Mutex Unlock
}

/**
 * @brief 서비스 초기화
 */
bool gyroServiceInit(void) {
    is_gyro_ok = Gyro_Init();
    last_calc_tick = osKernelGetTickCount();
    return is_gyro_ok;
}


// 외부에서 상태를 물어볼 때 답해주는 함수
bool gyroServiceIsOk(void) {
    return is_gyro_ok;
}

/**
 * @brief 하드웨어 재시도 로직 (cli imu on 시 호출)
 */
bool gyroServiceReInit(void) {
    cliPrintf("Re-initializing I2C & Sensors...\r\n");
    is_gyro_ok = Gyro_Init();
    return is_gyro_ok;
}
/**
 * @brief 리포트 주기 및 센서 동작 설정
 */
void gyroServiceSetPeriod(uint32_t period) {
    imu_report_period = period;
    if (period > 0) {
        Gyro_StartAuto();
    } else {
        Gyro_StopAuto();
    }
}


/**
 * @brief 핵심 로직: 센서 읽기 -> 각도 계산 -> 짐벌 전달 -> 리포팅
 */
void gyroServiceUpdate(void) {
    if (!is_gyro_ok || GyroReadySemHandle == NULL) {
        return;
    }

    // 세마포어 획득 (센서 데이터 준비 완료 대기)
    if (osSemaphoreAcquire(GyroReadySemHandle, osWaitForever) != osOK) return;

    static Gyro_Data_t gyroData;

    // 1. 센서 데이터 읽기 및 연결 확인
    if (!Gyro_Read(&gyroData)) {
        is_gyro_ok = false;
        cliPrintf("\r\n[ALARM] Sensor Disconnected! Type 'imu off' then 'imu on' to recover.\r\n");
        return;
    }

    // 2. 시간 간격(dt) 계산
    uint32_t now = osKernelGetTickCount();
    float dt = (now - last_calc_tick) / 1000.0f;
    last_calc_tick = now;


    // 데이터 신선도 확인 (예: 500ms 이상 갱신 안 되면 가짜 데이터나 무시)
    Mag_Data_t *p_mag_to_use = &current_magData;
    if (osKernelGetTickCount() - mag_updated_tick > 500) {
        p_mag_to_use = NULL; // IMU_ComputeAngles 내부에서 NULL 체크를 하도록 설계
    }


    // 3. 센서 융합 및 각도 계산 (가속도 + 자이로 + 지자기)
    // IMU_ComputeAngles(&gyroData, &current_magData, dt);
    IMU_ComputeAngles(&gyroData, p_mag_to_use, dt); // NULL 처리가 반영된 포인터를 넘겨야 함


    // 4. 계산된 각도를 짐벌 제어기로 즉시 전송 (가장 중요)
    gimbalSettingIMU(gyroData.roll, gyroData.pitch, gyroData.yaw);

    // 5. 주기적 출력 제어 (CLI 또는 모니터링 툴)
    if (imu_report_period > 0 && (now - last_print_tick >= imu_report_period)) {
        last_print_tick = now;

        if (!isMonitoringOn()) {
            cliPrintf("Angle [Roll: %6.1f | Pitch: %6.1f | Yaw: %6.1f]\r\n", 
                      gyroData.roll, gyroData.pitch, gyroData.yaw);
        } else {
            monitorUpdateValue(ID_IMU_GYRO_X, TYPE_FLOAT, &gyroData.roll);
            monitorUpdateValue(ID_IMU_GYRO_Y, TYPE_FLOAT, &gyroData.pitch);
            monitorUpdateValue(ID_IMU_GYRO_Z, TYPE_FLOAT, &gyroData.yaw);
        }
    }
}
