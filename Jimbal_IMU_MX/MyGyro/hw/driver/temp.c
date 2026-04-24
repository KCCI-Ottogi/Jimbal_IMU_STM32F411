#include "temp.h"


static volatile uint32_t adc_dma_buf[1]; // 2초마다 온도 읽기
extern ADC_HandleTypeDef hadc1;

bool tempInit(void) {
    // Implementation for temperature sensor initialization
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, 1);
    return true;
}

float tempReadAuto(void) {
    // Implementation for reading temperature
    uint32_t adc_val = adc_dma_buf[0];

    float vsense = (adc_val / 4095.0f) * 3.3f; // Assuming a 12-bit ADC and 3.3V reference
    float temp_celsius = (vsense - 0.76f) / 0.0025f + 25.0f; // Example conversion formula for STM32 internal temperature sensor

    return temp_celsius;
}


float tempReadSingle(void){
    // Implementation for reading temperature
    uint32_t adc_val = 0; // Replace with actual ADC reading code
    tempStartAuto();
    HAL_Delay(10); // ADC 변환이 완료될 때까지 잠시 대기
    
    adc_val = adc_dma_buf[0]; // ADC에서 단일 변환 결과 읽기
    tempStopAuto(); // 자동 온도 읽기 중지
    
    float vsense = (adc_val / 4095.0f) * 3.3f; // Assuming a 12-bit ADC and 3.3V reference
    float temp_celsius = (vsense - 0.76f) / 0.0025f + 25.0f; // Example conversion formula for STM32 internal temperature sensor
    
    return temp_celsius;
}


void tempStartAuto(void) {
    // Implementation for starting automatic temperature reading
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, 1);
}

void tempStopAuto(void) {
    // Implementation for stopping automatic temperature reading
    HAL_ADC_Stop_DMA(&hadc1);
}