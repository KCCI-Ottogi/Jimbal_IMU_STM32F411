#include "uart.h"
#include "cmsis_os.h"
// #include "cmsis_os2.h"
// #include "hw_def.h"
#include "def.h"



extern UART_HandleTypeDef huart2;

static osMessageQueueId_t uart_rx_q = NULL;
static osMutexId_t uart_tx_mutex = NULL; // UART 전송 시 상호 배제를 위한 뮤텍스 핸들 
  //  const char                   *name;   ///< name of the mutex
  //  uint32_t                 attr_bits;   ///< attribute bits
  //  void                      *cb_mem;    ///< memory for control block
  //  uint32_t                   cb_size;   ///< size of provided memory for control block

#define TIMEOUT 100

#define UART_RX_BUF_LENGTH 256


static uint8_t rx_data;




bool uartInit(void){
    if(uart_rx_q == NULL) {
        uart_rx_q = osMessageQueueNew(UART_RX_BUF_LENGTH, sizeof(uint8_t), NULL);
    }

    if(uart_tx_mutex == NULL) {
        uart_tx_mutex = osMutexNew(NULL); // UART 전송 시 상호 배제를 위한 뮤텍스 생성  , NULL은 기본 속성 사용, osMutexNew는 CMSIS-RTOS2 API로 뮤텍스 객체를 생성하는 함수입니다. 이 함수는 뮤텍스 객체에 대한 핸들을 반환하며, NULL을 인자로 전달하면 기본 속성을 사용하여 뮤텍스가 생성됩니다.
    }
    bool ret = uartOpen(0,9600);
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);

    return ret;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

    if(huart->Instance==USART2){
        if(uart_rx_q != NULL) {
            osMessageQueuePut(uart_rx_q, &rx_data, 0, 0);
        }
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}

uint32_t uartAvailable(uint8_t ch){
   if(ch == 0 && uart_rx_q != NULL) {
        return osMessageQueueGetCount(uart_rx_q); // 큐에 있는 메시지 수 반환
    } 
    return 0;
}

uint8_t uartRead(uint8_t ch){
    uint8_t ret=0;
    if(ch == 0 && uart_rx_q != NULL) {
        osMessageQueueGet(uart_rx_q, &ret, NULL, osWaitForever); // 큐에서 메시지 가져오기 (대기), osWaitForever는 메시지가 올 때까지 기다림
    }   

    return ret;

}

bool uartReadBlock(uint8_t ch, uint8_t *p_data, uint32_t timeout){
    if(ch == 0 && uart_rx_q != NULL) {
        if(osMessageQueueGet(uart_rx_q, p_data, NULL, timeout) == osOK) { // osOk => 메시지 가져오기 성공
            return true;
        }
    }   
    return false;
}
 

bool uartOpen(uint8_t ch, uint32_t baudrate){
    
    if(huart2.Init.BaudRate!=baudrate)
       huart2.Init.BaudRate=baudrate;

    if(HAL_UART_DeInit(&huart2)!=HAL_OK)
        return false;

    if(HAL_UART_Init(&huart2)!=HAL_OK)
        return false;



    return true;
}

bool uartClose(uint8_t ch){
    return true;
}


uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t len){

    if(ch != 0 || uart_tx_mutex == NULL) {
        return 0;
    }
    osMutexAcquire(uart_tx_mutex, osWaitForever); // UART 전송 시 상호 배제를 위해 뮤텍스 획득, osWaitForever는 뮤텍스가 사용 가능해질 때까지 기다림
    // 내부 버퍼 있음. 버퍼가 다 차면, HAL_UART_Transmit는 블로킹 됨. 블로킹 되는 동안 다른 태스크가 UART를 사용하지 못하게 됨.   

    if(HAL_UART_Transmit(&huart2,p_data,len, TIMEOUT)==HAL_OK)
    // else len = 0; // 전송 실패 시 잠시 대기 (예: 버퍼가 가득 찬 경우)  // 전송 실패 시 len을 0으로 설정하여 호출자에게 전송이 실패했음을 알림, timeout

    osMutexRelease(uart_tx_mutex); // UART 전송 완료 후 뮤텍스 해제

    return len;
}

uint32_t uartPrintf(uint8_t ch, const char *fmt, ...){

    char buf[128];
    uint32_t len;
    va_list args;

    va_start(args,fmt);

    len=vsnprintf(buf, 128, fmt, args);

    va_end(args);
    return uartWrite(ch,(uint8_t*)buf,len);
}
