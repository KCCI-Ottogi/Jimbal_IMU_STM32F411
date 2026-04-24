#include "i2c.h"

#define I2C_TIMEOUT 10000 

void I2C1_Init(void) {
    // PB6, PB7 AF4(I2C1) 설정 (이전 코드와 동일하므로 세부 레지스터 세팅 생략)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); 
    GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);        
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0);   
    
    GPIOB->AFR[0] &= ~((0xF << 24) | (0xF << 28));
    GPIOB->AFR[0] |= ((4 << 24) | (4 << 28)); 

    I2C1->CR1 = I2C_CR1_SWRST; 
    I2C1->CR1 = 0;
    I2C1->CR2 = 42; 
    I2C1->CCR = I2C_CCR_FS | 35; 
    I2C1->TRISE = 13; 
    I2C1->CR1 |= I2C_CR1_PE; 
}

void I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    uint32_t timeout = I2C_TIMEOUT;
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB)) if(--timeout == 0) return;
    
    I2C1->DR = dev_addr; 
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_ADDR)) if(--timeout == 0) return;
    (void)I2C1->SR2; 
    
    I2C1->DR = reg_addr;
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_TXE)) if(--timeout == 0) return;
    
    I2C1->DR = data;
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_BTF)) if(--timeout == 0) return;
    I2C1->CR1 |= I2C_CR1_STOP;
}

// 다중 바이트 읽기 함수 추가
void I2C_ReadBurst(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t size) {
    uint32_t timeout;
    // 1. Start & Send Register Address
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = dev_addr; 
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2; 
    I2C1->DR = reg_addr;
    while(!(I2C1->SR1 & I2C_SR1_TXE));

    // 2. Restart & Send Device Address with Read bit (1)
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = dev_addr | 0x01; 
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    
    if (size == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR2; 
        I2C1->CR1 |= I2C_CR1_STOP;
    } else {
        (void)I2C1->SR2; 
        I2C1->CR1 |= I2C_CR1_ACK;
    }

    // 3. Read Data
    while (size > 0) {
        if (size == 1) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        while(!(I2C1->SR1 & I2C_SR1_RXNE));
        *data++ = I2C1->DR;
        size--;
    }
}