
#include "monitor.h"

// #define LOG_TAG "MON"

typedef enum{
MONITOR_MODE_OFF=0,
MONITOR_MODE_BINARY,
MONITOR_MODE_ASCII
}monitor_mode_t;

static monitor_packet_t g_packet;
static osMutexId_t monitor_mtx=NULL;
static monitor_mode_t current_mointor_mode=MONITOR_MODE_OFF;



static uint32_t monitor_period = 1000; // 기본값 1초
static monitor_sync_cb_t sync_handler = NULL; // 콜백 함수 포인터

// 1. 모니터링 정지
void monitorOff(void) {
    current_mointor_mode = MONITOR_MODE_OFF;
    LOG_INF("Monitoring Stopped.");
}

// 2. 현재 설정된 주기 읽기
uint32_t monitorGetPeriod(void) {
    return monitor_period;
}

// 3. 주기 설정 및 동기화 알림
void monitorSetPeriod(uint32_t period) {
    if (period > 0) {
        monitor_period = period;
        // 주기가 변경되었음을 다른 모듈(핸들러)에 알림
        if (sync_handler != NULL) {
            sync_handler(monitor_period);
        }
        LOG_INF("Monitor Period Set to: %d ms", period);
    }
}

// 4. 동기화 핸들러 등록 (다른 모듈에서 호출)
void monitorSetSyncHandler(monitor_sync_cb_t handler) {
    sync_handler = handler;
}








static uint8_t calChecksum(uint8_t *data, uint32_t len)
{
    uint8_t sum = 0;
    for (uint32_t i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}

static void cliMonitor(uint8_t argc, char **argv){

    if (argc >= 2) {
        if (strcmp(argv[1], "on") == 0) {
            current_mointor_mode = MONITOR_MODE_ASCII;
            LOG_INF("Monitor mode: ON(ASCII)\n");
            return;
        }else if(strcmp(argv[1], "off") == 0){
        current_mointor_mode = MONITOR_MODE_OFF;
        LOG_INF("Monitoring Mod: OFF (Text mode Restored)");
        
        return;
        }// 주기 설정 (예: mon period 500)
        else if (strcmp(argv[1], "period") == 0) {
            if (argc == 3) {
                uint32_t period = (uint32_t)strtoul(argv[2], NULL, 10);
                if (period < 10) period = 10; // 너무 빠르면 시스템 마비됨
                
                monitorSetPeriod(period); // 여기서 sync_handler 호출됨
                LOG_INF("Monitor Period Changed to %d ms", period);
            } else {
                LOG_INF("Usage: mon period [ms]");
            }
            return; // 처리 완료 후 리턴
        }
    }
    // LOG_INF("Usage: monitor [on|off])");
    LOG_INF("Usage: mon [on|off|period]");
    
    if (current_mointor_mode == MONITOR_MODE_ASCII) {
        LOG_INF("Current Status: ON(ASCII)");
    } else {
        LOG_INF("Current Status: OFF");
    }
    
}

void monitorInit(void)
{
    memset(&g_packet, 0, sizeof(g_packet));


    if(monitor_mtx == NULL)
        monitor_mtx = osMutexNew(NULL);

    cliAdd("mon", cliMonitor);
}

bool isMonitoringOn(void){
    return current_mointor_mode != MONITOR_MODE_OFF;
}
void monitorUpdateValue(SensorID id, DataType type, void *p_val) {
    if(monitor_mtx == NULL)
        return;
    osMutexAcquire(monitor_mtx, osWaitForever);
    
    //sensor id check
    int target_idx = -1;
    for (int i = 0; i < g_packet.count; i++) {
        if (g_packet.nodes[i].id == (uint8_t)id)
        {
            target_idx = i;
            break;
        }
    }
    //new sensor
    if ( -1 == target_idx) {
        if (g_packet.count < MAX_SENSOR_NODES) {
            target_idx = g_packet.count;
            g_packet.nodes[target_idx].id = (uint8_t)id;
            g_packet.count++;
        } else {
            //LOG_WRN("Monitor packet full, cannot add new sensor data");
            osMutexRelease(monitor_mtx);
            return;
        }
    }
    //update
    g_packet.nodes[target_idx].type = (uint8_t)type;

    if(type==TYPE_UINT8||type==TYPE_BOOL){
        g_packet.nodes[target_idx].value.u8_val[0] = *(uint8_t *) p_val;


    }else{
        memcpy(&g_packet.nodes[target_idx].value, p_val, 4);
    }

    osMutexRelease(monitor_mtx);

}
void monitorSendPacket()
{

    if (current_mointor_mode == MONITOR_MODE_OFF || monitor_mtx == NULL)
        return;

    osMutexAcquire(monitor_mtx, osWaitForever);

    // uint8_t tx_buf[256];
    // uint32_t len = 0;

    char tx_buf[512] = {0};
    int len = 0;

    if (current_mointor_mode == MONITOR_MODE_ASCII) {

        // 시작 문자 $
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "$%d", g_packet.count);

        for (int i = 0; i < g_packet.count; i++) {
            uint8_t id = g_packet.nodes[i].id;
            uint8_t type = g_packet.nodes[i].type;

            len += snprintf(tx_buf + len, sizeof(tx_buf) - len, ",");

            // id, type
            len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "%d:%d:", id, type);

            // value
            if (type == TYPE_UINT8 || type == TYPE_BOOL) {
                len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "%u",
                                g_packet.nodes[i].value.u8_val[0]);
            } else if (type == TYPE_INT32) {
                len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "%ld",
                                g_packet.nodes[i].value.i_val);
            } else if (type == TYPE_FLOAT) {
                len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "%.2f",
                                g_packet.nodes[i].value.f_val);
            } else {
                len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "%lu",
                                g_packet.nodes[i].value.u_val);
            }
        }

        // else{

        // }

        // 종료#
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "#\r\n");

        uartWrite(0, (uint8_t *)tx_buf, len);

        osMutexRelease(monitor_mtx);
    }
} /////