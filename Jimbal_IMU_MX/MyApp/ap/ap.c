
#include "ap.h"

#include "cli.h"
#include "log_DEF.h"
#include "monitor.h"

// button on/off => enable/disable
void cliButton(uint8_t argc, char **argv)
{
    // 인자가 2개 들어왔을 때 (명령어 + on/off)
    if (argc == 2) {
        if (strcmp(argv[1], "on") == 0) {
            buttonEnable(true);
            cliPrintf("Button Interrupt Report: ON\r\n");
        } else if (strcmp(argv[1], "off") == 0) {
            buttonEnable(false);
            cliPrintf("Button Interrupt Report: OFF\r\n");
        } else {
            // on/off 외의 다른 인자가 들어온 경우
            cliPrintf("Usage: button [on/off]\r\n");
        }
    }
    // 인자가 없거나 잘못된 경우 현재 상태 출력
    else {
        cliPrintf("Usage: button [on/off]\r\n");

        // 현재 상태가 true면 "ON", false면 "OFF" 출력
        cliPrintf("Current Status: %s\r\n", buttonGetEnable() ? "ON" : "OFF");
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
    if (argc >= 2) {
        uint32_t addr = strtoul(argv[1], NULL, 16);
        uint32_t length = 16;

        if (argc >= 3) {
            length = strtoul(argv[2], NULL, 0);
        }

        for (uint32_t i = 0; i < length; i += 16) {

            cliPrintf("0x%08x : ", addr + i);
            for (uint32_t j = 0; j < 16; j++) {

                if (i + j < length) {

                    uint32_t target_addr = (addr + i + j);

                    if (isSafeAddress(target_addr)) {

                        // uint8_t val = *(uint8_t *)(addr + i + j);
                        uint8_t val = *((volatile uint8_t *)target_addr);

                        cliPrintf("%02X ", val);
                    } else {

                        cliPrintf("Not valid address!!\r\n");
                        break;
                    }
                } else {
                    cliPrintf("     ");
                }
            }

            cliPrintf(" | ");

            for (uint32_t j = 0; j < 16; j++) {

                if (i + j < length) {
                    uint32_t target_addr = (addr + i + j);

                    if (isSafeAddress(target_addr)) {
                        uint8_t val = *((volatile uint8_t *)target_addr);

                        if (val >= 0x20 && val <= 0x7E) {
                            cliPrintf("%c", val);
                        } else {

                            cliPrintf(".");
                        }
                    } else {

                        cliPrintf("Not valid address!!\r\n");
                        break;
                    }
                }
            }
            cliPrintf("\r\n"); // 내가 추가
        }
    } else {
        cliPrintf(" Usage : md [add(hex)] [length]\r\n");
        cliPrintf("         md 98000000 32 \r\n");
    }
}

// argv[1] : "read" "write"
// argv[2] : pin "A5", "B12"
void cliGpio(uint8_t argc, char **argv)
{

    if (argc >= 3) {
        char port_char = tolower(argv[2][0]);
        int pin_num = atoi(&argv[2][1]);

        uint8_t port_idx = port_char - 'a';

        if (strcmp(argv[1], "read") == 0) {

            int8_t state = gpioExtRead(port_idx, pin_num);
            if (state < 0) {
                cliPrintf("Invalid Port or Pin (ex:a5, b12)\r\n");
            } else {
                cliPrintf("GPIO %c%d=%d\r\n", toupper(port_char), pin_num, state);
            }
        } else if (strcmp(argv[1], "write") == 0 && argc == 4) {
            int val = atoi(argv[3]);
            if (gpioExtWrite(port_idx, pin_num, val) == true) {
                cliPrintf("GPIO %c%d Set to %d\r\n", toupper(port_char), pin_num, val);

            } else {
                cliPrintf("Invalid Port or Pin (ex:a5, b12)\r\n");
            }
        } else {
            cliPrintf("Usage: gpio read[a~h][0~15]\r\n");
            cliPrintf("       gpio write[a~h][0~15] [0|1]\r\n");
        }
    } else {
        cliPrintf("Usage: gpio read[a~h][0~15]\r\n");
        cliPrintf("       gpio write[a~h][0~15] [0|1]\r\n");
    }
}

static u_int32_t led_toggle_period = 0;
void cliLed(uint8_t argc, char **argv)
{
    if (argc >= 2) {
        if (strcmp(argv[1], "on") == 0) {
            led_toggle_period = 0;
            ledOn();
            LOG_INF("LED ON");
            // cliPrintf("LED ON\r\n");
        } else if (strcmp(argv[1], "off") == 0) {
            led_toggle_period = 0;
            ledOff();
            LOG_INF("LED OFF");
            // cliPrintf("LED OFF\r\n");
        } else if (strcmp(argv[1], "toggle") == 0) {
            if (argc == 3) {
                led_toggle_period = atoi(argv[2]);
                if (led_toggle_period >= 0) {

                    LOG_INF("LED Auto-Toggled!!");
                    // cliPrintf("LED Auto-Toggled!!\r\n");
                } else {
                    LOG_INF("Invalid Period");
                    // cliPrintf("Invalid Period\r\n");
                }
            }
            // led_toggle_period=strtoul(argv[2],null,0);

            ledToggle();

            LOG_INF("LED TOGGLE");
            // cliPrintf("LED TOGGLE\r\n");
        } else {
            LOG_INF("Invalid Command");
            // cliPrintf("Invalid Command\r\n");
        }
    } else {
        cliPrintf("Usage: led [on|off]\r\n");
        cliPrintf("     : led toggle\r\n");
        cliPrintf("     : led toggle [period]\r\n");
    }
}

void cliInfo(uint8_t argc, char **argv)
{

    if (argc == 1) {
        cliPrintf("=========================================\r\n");
        cliPrintf(" HW Model    :   STM32F411\r\n");
        cliPrintf(" FW Version  : V1.0.0\r\n");
        cliPrintf(" Build Date  : %s %s\r\n", __DATE__, __TIME__);

        uint32_t uid0 = HAL_GetUIDw0();
        uint32_t uid1 = HAL_GetUIDw1();
        uint32_t uid2 = HAL_GetUIDw2();
        uint32_t dev = HAL_GetDEVID();

        // uint32_t hal = HAL_GetHalVersion();
        // uint32_t rev = HAL_GetREVID();

        cliPrintf(" Serial Num  : %08x-%08x-%08x\r\n", uid0, uid1, uid2);
        cliPrintf(" DevicID     : %08x\r\n", dev);
        cliPrintf("=========================================\r\n");
    }

    if (argc == 2 || strcmp(argv[1], "uptime") == 0) {
        cliPrintf("System Uptime: %d ms \r\n", millis());
    } else {
        cliPrintf("Usage: info\r\n");
        cliPrintf("       info [uptime]\r\n");
    }
}

void cliSys(uint8_t argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "reset") == 0) {
        NVIC_SystemReset();
    } else {
        cliPrintf("Usage: sys [reset]\r\n");
    }
}
static uint32_t temp_report_period = 0; // 0이면 출력 정지

void cliTemp(uint8_t argc, char **argv)
{
    if (argc == 1) {

        if (temp_report_period > 0) {
            tempStopAuto();
        }
        temp_report_period = 0;
        float t = tempReadSingle();
        cliPrintf("Current Temp: %.2f C\r\n", t);

    } else if (argc == 2) {

        int period = atoi(argv[1]);
        if (period > 0) {
            tempStartAuto();
            temp_report_period = period;
            cliPrintf("Temperature Auto-Read Started (%d ms)\r\n", period);
        } else {
            tempStopAuto();
            cliPrintf("Invalid Period\r\n");
        }
    } else {
        cliPrintf("Usage: temp\r\n");
        cliPrintf("     : temp [period]\r\n");
    }
}

void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */

    apInit();

    /* Infinite loop */
    for (;;) {

        apMain();
        // osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}



void ledSystemTask(void *argument)
{

    while (1) {
        if (led_toggle_period > 0) {
            LOG_DBG("LED Toggle!");
            ledToggle();
            bool led_state = ledGetStatus();
            if(isMonitoringOn())
                monitorUpdateValue(ID_OUT_LED_STATE, TYPE_BOOL, &led_state);
            else
                LOG_DBG("LED Toggle!");

            osDelay(led_toggle_period);
        } else {
            bool led_state = ledGetStatus();
            if(isMonitoringOn())
                monitorUpdateValue(ID_OUT_LED_STATE, TYPE_BOOL, &led_state);
            
            osDelay(50);
        }
    }
}
void tempSystemTask(void *argument)
{
    while (1) {

        if (temp_report_period > 0) {
            float t = tempReadAuto(); 
            
            if(isMonitoringOn())
                monitorUpdateValue(ID_ENV_TEMP, TYPE_FLOAT, &t);
            else
                cliPrintf("Current Temp: %.2f C\r\n", t);
            
            osDelay(temp_report_period);
        } else {
            
            osDelay(50);
        }
    }
}
void monitorSystemTask(void *argument)
{
    while (1) {
        if(isMonitoringOn()){
            monitorSendPacket();
        }
        // osDelay(1000);
        osDelay(monitorGetPeriod());
    }
}

void apStopAutoTask(void)
{
    led_toggle_period = 0;
    temp_report_period  = 0;
    tempStopAuto();
    ledOff();
}

//내가 추가
void updateModulePeriods(uint32_t new_period) {
    // 1. 온도 센서 업데이트 주기 변수 변경
    temp_report_period = new_period; 
    
    // 2. LED 토글 주기 변수 변경 (동기화를 원할 경우)
    led_toggle_period = new_period; 

    // 필요하다면 하위 드라이버 설정 함수 호출 (만들어두셨다면)
    // tempSetUpdatePeriod(new_period); 
    
    // LOG_INF("All modules synchronized to %d ms", new_period);
}

void apInit()
{   
    
    hwInit();
    LOG_INF("Application Init... Started");
    monitorInit();

    cliSetCtrlHandler(apStopAutoTask);
    monitorSetSyncHandler(updateModulePeriods);


    cliAdd("led", cliLed);
    cliAdd("info", cliInfo);
    cliAdd("sys", cliSys);
    cliAdd("gpio", cliGpio);
    cliAdd("md", cliMd);
    cliAdd("button", cliButton);
    cliAdd("temp", cliTemp);
}
void apMain()
{
    // osThreadId_t ledSystemTaskHandle;

    // const osThreadAttr_t ledSystemTask_attributes = {
    //     .name = "ledSystemTask",
    //     .stack_size = 128 * 4,
    //     .priority = (osPriority_t)osPriorityNormal,

    // };
    // osThreadNew(ledSystemTask, NULL, &ledSystemTask_attributes);

    // ledSystemTaskHandle = osThreadNew(ledSystemTask, NULL, &ledSystemTask_attributes);

    // uartPrintf(0, "HEllOW world!! \r\n"); //(uint8_t *)
    uartPrintf(0, "LED Task Started!! \r\n"); //(uint8_t *)

    while (1) {

        // HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        //  HAL_Delay(1000);
        // Delay(1000);

        // ledOn();
        // HAL_Delay(1000);
        // ledOff();
        // HAL_Delay(1000);
        // uartWrite(0,(uint8_t *)"HEllOW", 7);

        // uartPrintf(0,(uint8_t *)"HEllOW %d\r\n", 10);

        // if(uartAvailable(0)>0){
        //     uint8_t ch = uartRead(0);

        //     uartPrintf(0,"%c", ch);
        // }
        // HAL_Delay(500);

        cliMain();
        // osDelay(1);
    }
}