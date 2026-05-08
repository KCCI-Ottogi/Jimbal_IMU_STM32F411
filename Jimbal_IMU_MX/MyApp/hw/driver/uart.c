#include "uart.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static osMessageQueueId_t uart_rx_q[2] = {NULL, NULL};
static osMutexId_t uart_tx_mutex = NULL;
static uint8_t rx_data[2] = {0, 0};

bool uartInit(void)
{
    if (uart_rx_q[0] == NULL) uart_rx_q[0] = osMessageQueueNew(256, sizeof(uint8_t), NULL);
    if (uart_rx_q[1] == NULL) uart_rx_q[1] = osMessageQueueNew(256, sizeof(uint8_t), NULL);
    if (uart_tx_mutex == NULL) uart_tx_mutex = osMutexNew(NULL);

    uartOpen(0, 115200); // USART2 (PC)
    uartOpen(1, 115200); // USART1 (Camera)

    HAL_UART_Receive_IT(&huart2, &rx_data[0], 1);
    HAL_UART_Receive_IT(&huart1, &rx_data[1], 1);

    return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        if (uart_rx_q[0] != NULL) osMessageQueuePut(uart_rx_q[0], &rx_data[0], 0, 0);
        HAL_UART_Receive_IT(&huart2, &rx_data[0], 1);
    }
    else if (huart->Instance == USART1) {
        if (uart_rx_q[1] != NULL) osMessageQueuePut(uart_rx_q[1], &rx_data[1], 0, 0);
        HAL_UART_Receive_IT(&huart1, &rx_data[1], 1);
    }
}

uint32_t uartAvailable(uint8_t ch)
{
    if (ch < 2 && uart_rx_q[ch] != NULL) return osMessageQueueGetCount(uart_rx_q[ch]);
    return 0;
}

uint8_t uartRead(uint8_t ch)
{
    uint8_t ret = 0;
    if (ch < 2 && uart_rx_q[ch] != NULL) osMessageQueueGet(uart_rx_q[ch], &ret, NULL, 0);
    return ret;
}

bool uartOpen(uint8_t ch, uint32_t baudrate)
{
    UART_HandleTypeDef *p_huart = (ch == 0) ? &huart2 : &huart1;
    p_huart->Init.BaudRate = baudrate;
    if (HAL_UART_DeInit(p_huart) != HAL_OK) return false;
    if (HAL_UART_Init(p_huart) != HAL_OK) return false;
    return true;
}

uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t len)
{
    if (uart_tx_mutex == NULL) return 0;
    UART_HandleTypeDef *p_huart = (ch == 0) ? &huart2 : &huart1;
    
    osMutexAcquire(uart_tx_mutex, osWaitForever);
    if (HAL_UART_Transmit(p_huart, p_data, len, 100) != HAL_OK) len = 0;
    osMutexRelease(uart_tx_mutex);
    return len;
}

uint32_t uartPrintf(uint8_t ch, const char *fmt, ...)
{
    char buf[128];
    uint32_t len;
    va_list args;
    va_start(args, fmt);
    len = vsnprintf(buf, 128, fmt, args);
    va_end(args);
    return uartWrite(ch, (uint8_t *)buf, len);
}