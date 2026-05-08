#ifndef __HW_DRIVER_MY_GPIO_H__
#define __HW_DRIVER_MY_GPIO_H__

#include "hw_def.h"

// 1. 아두이노처럼 쓰기 쉽게 방향(Mode) 매크로 정의
#define GPIO_DIR_INPUT  0
#define GPIO_DIR_OUTPUT 1

// port number : 0 = A, 1 = B, 2 = C, 3 = D, 4 = E, 5 = F, 6 = G, 7 = H

// 2. 파라미터 이름 통일 (port_idx, pin_num)
void myGpioPinMode(uint8_t port_idx, uint8_t pin_num, uint8_t mode);
bool gpioExtWrite(uint8_t port_idx, uint8_t pin_num, uint8_t state);
int8_t gpioExtRead(uint8_t port_idx, uint8_t pin_num);  // 1(High) 또는 0(Low) 반환, 에러 시 -1

// 3. 불필요한 괄호 오타 수정
#endif // __HW_DRIVER_MY_GPIO_H__