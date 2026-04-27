#include "ap.h"
#include "gyro.h"
#include "mag.h"
#include "cmsis_os2.h"
#include "cli.h"

// FreeRTOS 세마포어 핸들
extern osSemaphoreId_t GyroReadySemHandle;


static volatile uint32_t led_toggle_period = 0;
static volatile uint32_t imu_report_period = 0;
static volatile bool imu_report_enabled = false;static bool is_gyro_ok = false;
static bool is_mag_ok = false;
// static uint32_t led_toggle_period = 0;
// static uint32_t monitor_period = 0;

// ==========================================
// CLI Commands
// ==========================================
void cliImu(uint8_t argc, char **argv) {
  if (argc >= 2) {
    if (strcmp(argv[1], "on") == 0) {
      imu_report_period = (argc == 3) ? atoi(argv[2]) : 500; // 기본값 500ms
      Gyro_StartAuto(); // MPU6050 인터럽트 켜기
      Mag_StartAuto();  // HMC5883L 연속측정 켜기
      LOG_INF("IMU Report : ON (%d ms)", imu_report_period);
    } else if (strcmp(argv[1], "off") == 0) {
      imu_report_period = 0;
      Gyro_StopAuto();
      Mag_StopAuto();
      LOG_INF("IMU Report : OFF");
    }
  } else {
    cliPrintf("Usage: imu [on|off] [period]\r\n");
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
// static uint32_t temp_read_period = 0;
// void cliTemp(uint8_t argc, char **argv){
//   if(argc==1)
//   {
//     if(temp_read_period >0){
//       tempStopAuto();
//     }
//     temp_read_period=0;
//     float t=tempReadSingle();
//     cliPrintf("Current Temp: %.2f *C\r\n", t);
//   }
//   else if(argc==2){
    
//     int period =atoi(argv[1]);
//     if(period>0){
//       tempStartAuto();
//       temp_read_period=period;
//       cliPrintf("Temperature Auto-Read Started (%d ms)\r\n",period);
//     }
//     else{
//       tempStopAuto();
//       cliPrintf("Invalid Period\r\n");
//     }
//   }
//   else{
//     tempStopAuto();
//     cliPrintf("Usage: temp\r\n");
//     cliPrintf("       temp [period]\r\n");
//   }
// }

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

void gyroSystemTask(void *argument) {
  Gyro_Data_t gyroData;
  uint32_t last_tick = 0;

  while (1) {
    if (is_gyro_ok && GyroReadySemHandle != NULL) {
      // MPU6050 인터럽트 신호가 들어올 때까지 대기
      if (osSemaphoreAcquire(GyroReadySemHandle, osWaitForever) == osOK) {
        
        // I2C를 읽어줘야 MPU6050의 INT핀이 LOW로 클리어됩니다.
        if (Gyro_Read(&gyroData)) {
          
          // 설정된 주기(imu_report_period)를 만족했을 때만 콘솔/모니터에 출력 (요청하신 gyro x,y,z 출력)
          if (imu_report_period > 0 && (osKernelGetTickCount() - last_tick >= imu_report_period)) {
            last_tick = osKernelGetTickCount();

            int32_t gx = gyroData.gyro_x; int32_t gy = gyroData.gyro_y; int32_t gz = gyroData.gyro_z;
            
            if (isMonitoringOn()) {
              monitorUpdateValue(ID_IMU_GYRO_X, TYPE_INT32, &gx);
              monitorUpdateValue(ID_IMU_GYRO_Y, TYPE_INT32, &gy);
              monitorUpdateValue(ID_IMU_GYRO_Z, TYPE_INT32, &gz);
            } else {
              cliPrintf("GYRO X: %5d | Y: %5d | Z: %5d\r\n", gx, gy, gz);
            }
          }
        }
      }
    } else {
      osDelay(1000); // 초기화 실패 시 자원 낭비 방지
    }
  }
}
void magSystemTask(void *argument) {
  Mag_Data_t magData;

  while (1) {
    if (is_mag_ok && imu_report_period > 0) {
      if (Mag_Read(&magData)) {
        int32_t mx = magData.mag_x; int32_t my = magData.mag_y; int32_t mz = magData.mag_z;

        if (isMonitoringOn()) {
          monitorUpdateValue(ID_IMU_MAG_X, TYPE_INT32, &mx);
          monitorUpdateValue(ID_IMU_MAG_Y, TYPE_INT32, &my);
          monitorUpdateValue(ID_IMU_MAG_Z, TYPE_INT32, &mz);
        } else {
          // cliPrintf("MAG  X: %5d | Y: %5d | Z: %5d\r\n", mx, my, mz);
        }
      }
      osDelay(imu_report_period); // 지자계는 설정된 주기만큼 딜레이하며 폴링
    } else {
      osDelay(100);
    }
  }
}

void apStopAutoTask(void){
  monitorOff();
  led_toggle_period = 0;
  imu_report_period = 0;
  Gyro_StopAuto();
  Mag_StopAuto();
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
      // 칩 주소(i)를 1비트 왼쪽으로 시프트하여 HAL 함수에 전달
      if(HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 3, 100) == HAL_OK) {
          cliPrintf(">>> Found I2C Device at 0x%02X\r\n", i);
          found_count++;
      }
  }
  if (found_count == 0) cliPrintf(">>> NO I2C DEVICES FOUND! Check Wiring.\r\n");
  cliPrintf("-------------------------------\r\n");
  // ==========================================

  // 센서 초기화 (HAL)
  is_gyro_ok = Gyro_Init();
  if(!is_gyro_ok) LOG_ERR("MPU6050 Init Failed.");
  else LOG_INF("MPU6050 Init OK.");
  
  is_mag_ok = Mag_Init();
  if(!is_mag_ok) LOG_ERR("HMC5883L Init Failed.");
  else LOG_INF("HMC5883L Init OK.");
  cliAdd("led", cliLed);
  cliAdd("info", cliInfo);
  cliAdd("sys", cliSys);
  cliAdd("gpio", cliGpio);
  cliAdd("md", cliMd);
  cliAdd("imu", cliImu); // IMU 명령어 추가
}
void apMain(void)
{
  
  uartPrintf(0, "LED Task Started!!\r\n");
  while (1)
  {
    cliMain();
  }
}
