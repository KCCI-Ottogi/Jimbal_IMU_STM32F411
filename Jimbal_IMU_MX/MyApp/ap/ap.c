#include "ap.h"
#include "cmsis_os2.h"
#include "service/camera_service.h"


// FreeRTOS 세마포어 핸들
extern osSemaphoreId_t GyroReadySemHandle;


static volatile uint32_t led_toggle_period = 0;
static volatile uint32_t imu_report_period = 0;
static volatile uint32_t gimbal_report_period = 0;

static volatile bool imu_report_enabled = false; 
// static bool is_gyro_ok = false;
static bool is_mag_ok = false;



void cliServo(uint8_t argc, char **argv) {
    // 1. 명령어 처리에 성공했는지 추적할 변수 (선택 사항, 여기선 조기 반환 사용)
    
    // 2. 인자가 아예 없는 경우 (servo만 입력)
    if (argc >= 2) {
        
        // --- [기존 명령어] ---
        if (strcmp(argv[1], "total") == 0) {
            servoTotalTest();
            return; // 성공 시 즉시 종료
        }

        if (strcmp(argv[1], "test") == 0 && argc == 4) {
            uint8_t ch = (uint8_t)atoi(argv[2]);
            uint8_t angle = (uint8_t)atoi(argv[3]);
            if (ch <= 2) {
                servoWrite(ch, (angle > 180) ? 180 : angle);
                cliPrintf("Servo[%d] set to %d deg\r\n", ch, angle);
                return; // 성공 시 즉시 종료
            }
        }

        if (strcmp(argv[1], "scan") == 0 && argc == 3) {
            servoScan((uint8_t)atoi(argv[2]));
            return;
        }

        if (strcmp(argv[1], "smooth") == 0 && argc == 5) {
            uint8_t ch = (uint8_t)atoi(argv[2]);
            float angle = (float)atof(argv[3]);
            float k = (float)atof(argv[4]);
            servoSetTarget(ch, angle, k);
            cliPrintf("Servo %d: Target %.1f, k %.2f\r\n", ch, angle, k);
            return;
        }

        // --- [새로 추가한 명령어] ---
        if (strcmp(argv[1], "sync") == 0 && argc == 6) {
            float a0 = (float)atof(argv[2]);
            float a1 = (float)atof(argv[3]);
            float a2 = (float)atof(argv[4]);
            float k  = (float)atof(argv[5]);
            servoSetTargetAll(a0, a1, a2, k);
            cliPrintf("Sync Target Set\r\n");
            return;
        }

        if (strcmp(argv[1], "sweep") == 0 && argc == 4) {
            uint8_t ch = (uint8_t)atoi(argv[2]);
            if (ch > 2) {
                cliPrintf("Error: Invalid Channel %d\r\n", ch);
                return; // 잘못된 채널이면 Usage를 보여주는 대신 에러 메시지 후 종료하는 게 명확할 때가 많습니다.
            }
            float k = (float)atof(argv[3]);
            servoSweep(ch, k);
            cliPrintf("CH %d Sweep\r\n", ch);
            return;
        }
    }

    // 3. 여기까지 내려왔다는 것은? 
    // 인자가 부족했거나(argc < 2), 위 if문 조건(명령어 오타, 인자 개수 틀림)에 하나도 안 걸렸다는 뜻입니다.
    cliPrintf("\r\n[Invalid Command or Arguments]\r\n");
    cliPrintf("Usage:\r\n");
    cliPrintf("  servo total\r\n");
    cliPrintf("  servo test [0-2] [0-180]\r\n");
    cliPrintf("  servo scan [0-2]\r\n");
    cliPrintf("  servo smooth [0-2] [0-180] [k]\r\n");
    cliPrintf("  servo sync [ang0] [ang1] [ang2] [k]\r\n");
    cliPrintf("  servo sweep [0-2] [k]\r\n");
}



// button on/off  => enable/disable
void cliButton(uint8_t argc, char **argv)
{
    if (argc == 2) {
        if (strcmp(argv[1], "on") == 0) {
            buttonEnable(true);
            cliPrintf("Button Interrupt Report: ON\r\n");
        } else if (strcmp(argv[1], "off") == 0) {
            buttonEnable(false);
            cliPrintf("Button Interrupt Report: OFF\r\n");
        }
    } else {
        cliPrintf("Usage: button [on/off]\r\n");
        cliPrintf("Current Status: %s\r\n", buttonGetEnable() ? "ON" : "OFF");
    }
}




// ==========================================
// CLI Commands
// ==========================================
// void cliImu(uint8_t argc, char **argv) {
//   if (argc >= 2) {
//     if (strcmp(argv[1], "on") == 0) {
//       uint32_t period = (argc == 3) ? atoi(argv[2]) : 500; 
//       if(!isMonitoringOn() && period < 100) period = 100;
      
//       // ★ 복구 로직: "imu on"을 입력할 때마다 하드웨어 초기화 재시도
//       cliPrintf("Re-initializing I2C & Sensors...\r\n");
//       // MX_I2C1_Init; // I2C 버스 복구 및 활성화
//       is_gyro_ok = Gyro_Init(); 
//       if(!is_gyro_ok) {
//           cliPrintf("ERROR: Sensor Init Failed. Check Wires.\r\n");
//           return; // 선이 아직 안 꽂혀있으면 실행 중지
//       }
//       // 성공하면 계속 진행
//       Gyro_StartAuto(); 
//       imu_report_period = period;
//       cliPrintf("IMU Report : ON (%d ms)\r\n", imu_report_period);
//     } else if (strcmp(argv[1], "off") == 0) {
//       imu_report_period = 0;
//       Gyro_StopAuto();
//       cliPrintf("IMU Report : OFF\r\n");
//     }
//   }
// }
void cliImu(uint8_t argc, char **argv) {
  if (argc >= 2) {
    if (strcmp(argv[1], "on") == 0) {
      uint32_t period = (argc == 3) ? atoi(argv[2]) : 500; 
      if (period < 100) period = 100;
      
      // 하드웨어 재초기화 및 주기 설정 위임
      if(!gyroServiceReInit()) {
          cliPrintf("ERROR: Sensor Init Failed. Check Wires.\r\n");
          return;
      }
      gyroServiceSetPeriod(period);
      cliPrintf("IMU Report : ON (%d ms)\r\n", period);

    } else if (strcmp(argv[1], "off") == 0) {
      gyroServiceSetPeriod(0);
      cliPrintf("IMU Report : OFF\r\n");
    }
  }
}


static bool isSafeAddress(uint32_t addr)
{
  // 1. f411 flash
  if (0x08000000 <= addr && addr <= 0x0807FFFF)
    return true;

  // 2. f411 ram
  if (0x20000000 <= addr && addr <= 0x20001FFF)
    return true;

  // 3. system memory
  if (0x1FFF0000 <= addr && addr <= 0x1FFF7A1F)
    return true;

  // 4. Peripheral register
  if (0x40000000 <= addr && addr <= 0x5FFFFFFF)
    return true;

  return false;
}

// md 0x8000-0000 32
void cliMd(uint8_t argc, char **argv)
{
  if (argc >= 2)
  {
    uint32_t addr = strtoul(argv[1], NULL, 16);
    uint32_t length = 16;

    if (argc >= 3)
    {
      length = strtoul(argv[2], NULL, 0);
    }

    for (uint32_t i = 0; i < length; i += 16)
    {
      cliPrintf("0x%08x : ", addr + i);

      for (uint32_t j = 0; j < 16; j++)
      {
        if (i + j < length)
        {
          uint32_t target_addr = (addr + i + j);
          if (isSafeAddress(target_addr))
          {
            uint8_t val = *((volatile uint8_t *)target_addr);
            cliPrintf("%02X ", val);
          }
          else
          {
            cliPrintf("Not valid address!!\r\n");
            break;
          }
        }
        else
        {
          cliPrintf("   ");
        }
      }
      //
      cliPrintf(" | ");

      for (uint32_t j = 0; j < 16; j++)
      {
        if (i + j < length)
        {
          uint32_t target_addr = (addr + i + j);
          if (isSafeAddress(target_addr))
          {
            uint8_t val = *((volatile uint8_t *)target_addr);
            if (val >= 0x20 && val <= 0x7E)
            {
              cliPrintf("%c", val);
            }
            else
            {
              cliPrintf(".");
            }
          }
          else
          {
            cliPrintf("Not valid address!!\r\n");
            break;
          }
        }
      }

      cliPrintf("\r\n");
    }
  }
  else
  {
    cliPrintf("Usage : md [add(hex)] [length]\r\n");
    cliPrintf("        md 08000000 32 \r\n");
  }
}

// argv[1] : "read" "write"
// argv[2] : pin "A5", "B12"
void cliGpio(uint8_t argc, char **argv)
{
  if (argc >= 3)
  {
    char port_char = tolower(argv[2][0]);
    int pin_num = atoi(&argv[2][1]);

    uint8_t port_idx = port_char - 'a';

    if (strcmp(argv[1], "read") == 0)
    {
      int8_t state = gpioExtRead(port_idx, pin_num);
      if (state < 0)
      {
        cliPrintf("Invalid Port or Pin (ex:a5, b12)\r\n");
      }
      else
      {
        cliPrintf("GPIO %c%d=%d\r\n", toupper(port_char), pin_num, state);
      }
    }
    else if (strcmp(argv[1], "write") == 0 && argc == 4)
    {
      int val = atoi(argv[3]);
      if (gpioExtWrite(port_idx, pin_num, val) == true)
      {
        cliPrintf("GPIO %c%d Set to %d\r\n", toupper(port_char), pin_num, val);
      }
      else
      {
        cliPrintf("Invalid Port or Pin (ex:a5, b12)\r\n");
      }
    }
    else
    {
      cliPrintf("Usage: gpio read [a~h][0~15]\r\n");
      cliPrintf("       gpio write [a~h][0~15] [0|1]\r\n");
    }
  }
  else
  {
    cliPrintf("Usage: gpio read [a~h][0~15]\r\n");
    cliPrintf("       gpio write [a~h][0~15] [0|1]\r\n");
  }
}

// static uint32_t led_toggle_period = 0;
void cliLed(uint8_t argc, char **argv)
{
  if (argc >= 2)
  {
    if (strcmp(argv[1], "on") == 0)
    {
      led_toggle_period=0;
      ledOn();
      LOG_INF("LED ON");
    }
    else if (strcmp(argv[1], "off") == 0)
    {
      led_toggle_period=0;
      ledOff();
      LOG_INF("LED OFF");
    }
    else if (strcmp(argv[1], "toggle") == 0)
    {
      if(argc==3){
        led_toggle_period=atoi(argv[2]);
        if(led_toggle_period>0){
          LOG_INF("LED  Auto-Toggled!!");
        }
        else{
          cliPrintf("Invalid Period\r\n");
        }

      }
      else{
        led_toggle_period=0;
        ledToggle();
        LOG_INF("LED TOGGLE");
      }
    }
    else
    {
      cliPrintf("Invalid Command\r\n");
    }
  }
  else
  {
    cliPrintf("Usage: led [on|off]\r\n");
    cliPrintf("     : led toggle\r\n");
    cliPrintf("     : led toggle [period]\r\n");
  }
}

void cliInfo(uint8_t argc, char **argv)
{
  if (argc == 1)
  {
    cliPrintf("===============================");
    cliPrintf("  HW Model   :  STM32F411\r\n");
    cliPrintf("  FW Version : V1.0.0\r\n");
    cliPrintf("  Build Date : %s %s\r\n", __DATE__, __TIME__);
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();
    uint32_t dev = HAL_GetDEVID();

    cliPrintf("  Serial Num : %08x-%08x-%08x\r\n", uid0, uid1, uid2);
    cliPrintf("  DevicID    : %08x\r\n", dev);

    cliPrintf("===============================\r\n");
  }
  else if (argc == 2 || strcmp(argv[1], "uptime") == 0)
  {
    cliPrintf("System Uptime: %d ms \r\n", millis());
  }
  else
  {
    cliPrintf("Usage: info\r\n");
    cliPrintf("       info [uptime]\r\n");
  }
}
void cliSys(uint8_t argc, char **argv)
{
  if ((argc == 2) && strcmp(argv[1], "reset") == 0)
  {
    NVIC_SystemReset();
  }
  else
  {
    cliPrintf("Usage: sys [reset]\r\n");
  }
}




// ==========================================
// Gimbal CLI Command
// ==========================================
void cliGimbal(uint8_t argc, char **argv) {
  if (argc >= 2) {
    if (strcmp(argv[1], "on") == 0) {
      uint32_t period = (argc == 3) ? atoi(argv[2]) : 500; // 기본 500ms
      
      // UART 병목 방지 (텍스트 모드 시 최소 100ms 제한)
      if(!isMonitoringOn() && period < 100) {
          period = 100;
          cliPrintf("Warning: Period limited to 100ms to prevent UART bottleneck.\r\n");
      }
      
      gimbal_report_period = period;
      cliPrintf("Gimbal Report : ON (%d ms)\r\n", gimbal_report_period);
    } 
    else if (strcmp(argv[1], "off") == 0) {
      gimbal_report_period = 0;
      gimbalReturnHome();
      cliPrintf("Gimbal Report : OFF & return Home\r\n");
    }
  } else {
    cliPrintf("Usage: gim [on|off] [period]\r\n");
  }
}

void gimbalSystemTask(void *argument)
{
    LOG_INF("Gimbal System Task Started!");

    // uint32_t last_print_tick = 0; // 출력 시간 계산용

    cameraServiceInit();   // 카메라 PID 변수(sum, prev) 초기화

    while (1) {


        // --------
        // 1. 서보 모터를 목표 각도로 부드럽게 제어 (10ms 주기 업데이트)
        // 위 gyroSystemTask에서 들어온 타겟을 향해 모터가 움직입니다.
        // gimbalTaskLoop();

        // /* 3. 짐벌 위치(각도) CLI 출력 로직 (멀티태스킹 최적화) */
        // if (gimbal_report_period > 0) {
        //     uint32_t now = osKernelGetTickCount();
        //     if (now - last_print_tick >= gimbal_report_period) {
        //         last_print_tick = now;
                
        //         // getter 함수로 현재 서보 각도를 가져옴
        //         float r_angle = gimbalGetCurrentAngle(0);
        //         float p_angle = gimbalGetCurrentAngle(1);
        //         float y_angle = gimbalGetCurrentAngle(2);

                

        //         if (!isMonitoringOn()) {
        //             cliPrintf("GIMBAL [Roll(0): %3d | Pitch(1): %3d | Yaw(2): %3d]\r\n", 
        //                       (int)r_angle, (int)p_angle, (int)y_angle);
        //         }
        //     }
        // }

        ///////////////////////////////////////////////////

        // 1. 자이로 업데이트 (읽기 + 보고 + 출력까지 내부에서 처리)
        // gyroServiceUpdate();

        // 2. 카메라 업데이트 (파싱 + PID 계산 + 보고)
        cameraDataParsing();
        cameraServicePIDUpdate();

        // 3. 통합 제어 (데이터 합산 및 서보 구동)
        gimbalExecuteCombinedControl();

        // if (gimbal_report_period > 0) {
        //     uint32_t now = osKernelGetTickCount();
        //     if (now - last_print_tick >= gimbal_report_period) {
        //         last_print_tick = now;
                
        //         // getter 함수로 현재 서보 각도를 가져옴
        //         float r_angle = gimbalGetCurrentAngle(0);
        //         float p_angle = gimbalGetCurrentAngle(1);
        //         float y_angle = gimbalGetCurrentAngle(2);

                

        //         if (!isMonitoringOn()) {
        //             cliPrintf("GIMBAL [Roll(0): %3d | Pitch(1): %3d | Yaw(2): %3d]\r\n", 
        //                       (int)r_angle, (int)p_angle, (int)y_angle);
        //         }
        //     }
        // }

        osDelay(10); // 100Hz
    }
}




void StartDefaultTask(void *argument)
{
  apInit();
  for(;;)
  {
    apMain();
  }
  
}



void ledSystemTask(void *argument)
{
  while (1)
  {
    if(led_toggle_period > 0){
      
      ledToggle();
      bool led_state=ledGetStatus();

      if(isMonitoringOn()){
        monitorUpdateValue(ID_OUT_LED_STATE,TYPE_BOOL,&led_state);
      }
      else{
        LOG_DBG("LED Toggle!");
      }
      
      osDelay(led_toggle_period);
    }
    else{
      bool led_state=ledGetStatus();
      if(isMonitoringOn()){
        monitorUpdateValue(ID_OUT_LED_STATE,TYPE_BOOL,&led_state);
      }
      osDelay(50);
    }
  }
}


static uint32_t monitor_period = 0;
void monitorSystemTask(void *argument){
  while(1){
    if(isMonitoringOn()){
      monitorSendPacket();
    }
    monitor_period=monitorGetPeriod();
    osDelay(monitor_period);
  }

}

// static Mag_Data_t current_magData;
// void gyroSystemTask(void *argument) {
//   Gyro_Data_t gyroData = {0}; // 구조체 초기화
//   uint32_t last_print_tick = 0;
//   uint32_t last_calc_tick = osKernelGetTickCount();

//   while (1) {
//     if (is_gyro_ok && GyroReadySemHandle != NULL) {
//       if (osSemaphoreAcquire(GyroReadySemHandle, osWaitForever) == osOK) {
//         // 읽기 실패 시 복구 대기 상태로 전환
//         if (!Gyro_Read(&gyroData)) {
//             is_gyro_ok = false;
//             cliPrintf("\r\n[ALARM] Sensor Disconnected! Type 'imu off' then 'imu on' to recover.\r\n");
//             continue; 
//         }
//         else if (Gyro_Read(&gyroData)) {
          
//           // 1. 시간 간격(dt) 계산
//           uint32_t now = osKernelGetTickCount();
//           float dt = (now - last_calc_tick) / 1000.0f; // 밀리초를 초(Second)로 변환
//           last_calc_tick = now;

//           // 2. 센서 융합 및 각도 계산 (가속도 + 자이로 + 지자기)
//           IMU_ComputeAngles(&gyroData, &current_magData, dt);
//           gimbalUpdateFromIMU(gyroData.roll, gyroData.pitch, gyroData.yaw);
          
//           // 3. 주기적 출력 제어
//           if (imu_report_period > 0 && (now - last_print_tick >= imu_report_period)) {
//             last_print_tick = now;

//             if (!isMonitoringOn()) {
//               // CLI에 위치 각도 출력 (소수점 1자리까지만 출력하여 가독성 및 속도 확보)
//               cliPrintf("Angle [Roll: %6.1f | Pitch: %6.1f | Yaw: %6.1f]\r\n", 
//                         gyroData.roll, gyroData.pitch, gyroData.yaw);
//             } else {
//               // Monitor 패킷으로 보낼 때는 실수(float) 그대로 전송
//               monitorUpdateValue(ID_IMU_GYRO_X, TYPE_FLOAT, &gyroData.roll);
//               monitorUpdateValue(ID_IMU_GYRO_Y, TYPE_FLOAT, &gyroData.pitch);
//               monitorUpdateValue(ID_IMU_GYRO_Z, TYPE_FLOAT, &gyroData.yaw);
//             }
//           }
//         }
//       }
//     } else {
//       osDelay(100); 
//     }
//   }
// }
void gyroSystemTask(void *argument) {
  while (1) {
    // 변수 직접 접근 대신 Getter 함수 사용
    if (gyroServiceIsOk() && GyroReadySemHandle != NULL) { 
        gyroServiceUpdate();
    } else {
        osDelay(100);
    }
  }
}


// static Mag_Data_t current_magData; // 지자기 데이터 저장용
// void magSystemTask(void *argument) {
//   while (1) {
//     if (is_mag_ok && imu_report_period > 0) {
//       // 읽은 지자기 데이터를 전역 구조체에 업데이트 (gyroSystemTask에서 가져다 쓰기 위함)
//       Mag_Read(&current_magData); 
//       osDelay(imu_report_period); 
//     } else {
//       osDelay(100);
//     }
//   }
// }
void magSystemTask(void *argument) {
    Mag_Data_t magData; // 지역 변수로 선언

    while(1) {
        if (is_mag_ok) {
            if (Mag_Read(&magData)) {
                // 읽어온 데이터를 gyro_service로 전달
                gyroServiceSetMagData(&magData);
            }
        }
        osDelay(20); // 지자기 센서는 자이로보다 느리게 갱신해도 무방합니다
    }
}

// extern osThreadId_t defaultTaskHandle;
// extern osThreadId_t myTaskLedHandle;;
// extern osThreadId_t myTaskGyroHandle;
// extern osThreadId_t myTaskMonitorHandle;
// extern osThreadId_t myTaskMagHandle;
// extern osThreadId_t myTaskGimbalHandle;
// extern osThreadId_t stackMonitorTasHandle;

// static uint32_t task1Stack, task2Stack, task3Stack, task4Stack, task5Stack, task6Stack, task7Stack;
// void showStack() {
//   task1Stack = osThreadGetStackSpace(defaultTaskHandle);
//   task2Stack = osThreadGetStackSpace(myTaskLedHandle);
//   task3Stack = osThreadGetStackSpace(myTaskGyroHandle);
//   task4Stack = osThreadGetStackSpace(myTaskMonitorHandle);
//   task5Stack = osThreadGetStackSpace(myTaskMagHandle);
//   task6Stack = osThreadGetStackSpace(myTaskGimbalHandle);
//   task7Stack = osThreadGetStackSpace(stackMonitorTasHandle);

//   printf("Stack free - Task1: %lu Byte\t Task2: %lu Byte\t Task3: %lu Byte\t \r\n", task1Stack, task2Stack, task3Stack);
//   printf("Task4: %lu Byte\t Task5: %lu Byte\t Task6: %lu Byte\t Task7: %lu Byte\r\n", task4Stack, task5Stack, task6Stack, task7Stack);
// }

// void startMonitorStartTask(void *argument){
//   for(;;){
//     osDelay(5000);
//     showStack();
//   }
// }

void apStopAutoTask(void){
  monitorOff();
  led_toggle_period = 0;
  imu_report_period = 0;
  gimbal_report_period = 0;
  Gyro_StopAuto();
  Mag_StopAuto();
  gimbalReturnHome();
  ledOff();
}

void apSyncPeriods(uint32_t period){
  if(period > 0) {
    led_toggle_period = period;
    imu_report_period = period;
    Gyro_StartAuto();
    Mag_StartAuto();
    LOG_INF("Task Synchronized to %d ms", period);
  } else {
    led_toggle_period = 0;
    imu_report_period = 0;
    Gyro_StopAuto();
    Mag_StopAuto();
  }
}

void apInit(void)
{   
  hwInit(); 
  LOG_INF("Application Init... Started");
  logInit();
  monitorInit();

  
  servoInit();

  monitorSetSyncHandler(apSyncPeriods);
  cliSetCtrlCHandler(apStopAutoTask);
  
// ==========================================
  // [강력 추천] I2C Scanner 디버깅 코드
  // ==========================================
  extern I2C_HandleTypeDef hi2c1;
  cliPrintf("-------------------------------\r\n");
  cliPrintf("I2C Bus Scanner Started...\r\n");
  int found_count = 0;
  for(uint16_t i = 1; i < 128; i++) {
      if(HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 3, 100) == HAL_OK) {
          cliPrintf(">>> Found I2C Device at 0x%02X\r\n", i);
          found_count++;
      }
  }
  if (found_count == 0) cliPrintf(">>> NO I2C DEVICES FOUND! Check Wiring.\r\n");
  cliPrintf("-------------------------------\r\n");

  // ==========================================
  // 서비스 및 센서 초기화 (Encapsulated)
  // ==========================================
  
  // 자이로 서비스 초기화 (내부에서 Gyro_Init 호출 및 상태 저장)
  if(!gyroServiceInit()) {
      LOG_ERR("MPU6050 Init Failed.");
  } else {
      LOG_INF("MPU6050 Init OK.");
  }
  
  // 지자기 센서 초기화 (필요시 magService로 분리 가능)
  is_mag_ok = Mag_Init(); 
  if(!is_mag_ok) LOG_ERR("HMC5883L Init Failed.");
  else LOG_INF("HMC5883L Init OK.");



  cliAdd("led", cliLed);
  cliAdd("info", cliInfo);
  cliAdd("sys", cliSys);
  cliAdd("gpio", cliGpio);
  cliAdd("md", cliMd);
    cliAdd("button", cliButton);
    // cliAdd("temp", cliTemp);    
  cliAdd("imu", cliImu); // IMU 명령어 추가
  cliAdd("servo", cliServo);
  cliAdd("gim", cliGimbal);


}
void apMain(void)
{
  
  uartPrintf(0, "LED Task Started!!\r\n");
  while (1)
  {
    cliMain();
    osDelay(1);
  }
}


// void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
// {
//    /* Run time stack overflow checking is performed if
//    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
//    called if a stack overflow is detected. */
//   HAL_UART_DeInit(&huart3);
//   HAL_UART_Init(&huart3);

//   char msg[100];
//   int len = snprintf(msg, sizeof(msg), "Stack Overflow detectedin Task: %s\r\n", pcTaskName);
//   HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, HAL_MAX_DELAY);
//   showStack();

//   while(1);
   
// }