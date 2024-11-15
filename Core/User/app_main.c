#include "stm32g0xx_hal.h"
#include "main.h"
#include "dma.h"

#include <stdlib.h>

// Defines
#define ADC_BUFFER_LEN 3



// External Handles 
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2; 


// ADC Buffer
volatile uint16_t adc_buf[ADC_BUFFER_LEN]; // NOTE MOVE TO VCU STYLE BUFFER


void simple_precharge(){
    // NOTE FOR TEST CIRCUIT PIN7 = AIR+, PIN6 = AIR=
    // BUF[0] = VBATT, BUF[1] = VTS
    uint32_t vbatt, vts = 0;
    
    // Close AIR -
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    // Check current voltage of the battery
    vbatt = adc_buf[0];
    
    // Enable Pre-charge
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    // Check Voltage or Timer
    uint32_t startTime = HAL_GetTick();
    while ((HAL_GetTick() - startTime) < 5000){
        // 
    }
 
    // Close AIR +
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

    // Disable Pre-charge
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}


void app_init(){
    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUFFER_LEN);
    return;
}

void app_main(){
    uint32_t enablePrecharge = 0;
    uint32_t TSStatus = 0;

    uint32_t currentTime = 0;
    uint32_t prevTime = 0;

    while (1){
        
        currentTime = HAL_GetTick();    
        if (currentTime - prevTime > 500){
            prevTime = currentTime;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        }


        if (enablePrecharge == 1){
            simple_precharge();
            TSStatus = 1;
            enablePrecharge = 0;
        }

        if (1){
            uint32_t vbatt = adc_buf[0]; 
            uint32_t vts = adc_buf[1];
            // Check of TS voltage difference is greater then 10%
            if (abs(vbatt - vts) > 0.2 * vbatt){ 
                // Disable contactors
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  
                TSStatus = 0;
            }
        }
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	(void)hadc;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    (void)hadc;
}