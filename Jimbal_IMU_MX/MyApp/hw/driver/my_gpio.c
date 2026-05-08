#include "my_gpio.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>
#include <sys/types.h>

// 모드 정의 (필요에 따라 헤더에 추가하세요. ex: 0=INPUT, 1=OUTPUT)
#define GPIO_DIR_INPUT  0
#define GPIO_DIR_OUTPUT 1

static GPIO_TypeDef* getPortPtr(uint8_t port_idx)
{
    switch (port_idx){
        case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); return GPIOA;
        case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); return GPIOB;
        case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); return GPIOC;
        case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); return GPIOD;
        case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); return GPIOE;
        // case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); return GPIOF;
        // case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); return GPIOG;
        case 7: __HAL_RCC_GPIOH_CLK_ENABLE(); return GPIOH;
        default: return NULL; // Invalid port index
    }
}

// 1. 초기화를 전담하는 함수 구현
void myGpioPinMode(uint8_t port_idx, uint8_t pin_num, uint8_t mode)
{
    if (pin_num > 15) return;

    GPIO_TypeDef* pPort = getPortPtr(port_idx);
    if (pPort == NULL) return;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = (1 << pin_num);
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    

    if (mode == GPIO_DIR_OUTPUT) {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
    } else {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 
    }

    HAL_GPIO_Init(pPort, &GPIO_InitStruct);
}

// 2. 가벼워진 Write 함수
bool gpioExtWrite(uint8_t port_idx, uint8_t pin_num, uint8_t val) {
    if (pin_num > 15) return false;

    GPIO_TypeDef* pPort = getPortPtr(port_idx); // 클럭 활성화 겸 포트 얻기
    if (pPort == NULL) return false;

    uint16_t pin_mask = (1 << pin_num);
    HAL_GPIO_WritePin(pPort, pin_mask, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    return true;
}

// 3. 가벼워진 Read 함수
int8_t gpioExtRead(uint8_t port_idx, uint8_t pin_num) {
    if (pin_num > 15) return -1;

    GPIO_TypeDef* pPort = getPortPtr(port_idx);
    if (pPort == NULL) return -1;

    uint16_t pin_mask = (1 << pin_num);
    return (HAL_GPIO_ReadPin(pPort, pin_mask) == GPIO_PIN_SET) ? 1 : 0;
}