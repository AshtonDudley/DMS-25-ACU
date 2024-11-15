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

// CANbus handle
extern FDCAN_HandleTypeDef hfdcan1;

// CANbus Data Frame
uint8_t ubKeyNumber = 0x0;
uint8_t ubKeyNumberValue = 0x0;

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

// CANbus Heartbeat 
FDCAN_TxHeaderTypeDef TxHeartbeat;
uint8_t TxHeartbeatData[8];


static void FDCAN_Config(void);

static void FDCAN_Config(void) {
    FDCAN_FilterTypeDef sFilterConfig;

    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // FDCAN_FILTER_TO_RXFIFO0
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure global filter:
        Filter all remote frames with STD and EXT ID
        Reject non matching frames with STD ID and EXT ID */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    { 
        Error_Handler();
    }

    /* Prepare Tx Header */
    TxHeader.Identifier = 0x323;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // Heartbeat Signal 
    TxHeartbeat.Identifier = 0x320;
    TxHeartbeat.IdType = FDCAN_STANDARD_ID;
    TxHeartbeat.TxFrameType = FDCAN_DATA_FRAME;
    TxHeartbeat.DataLength = FDCAN_DLC_BYTES_8;
    TxHeartbeat.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeartbeat.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeartbeat.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeartbeat.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeartbeat.MessageMarker = 0;
}

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
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void can_heartbeat(){
    // Transmit Heartbeat   
    TxHeartbeatData[0] = 0xFF;
    TxHeartbeatData[1] = 0xFF;
    TxHeartbeatData[2] = 0xFF;
    TxHeartbeatData[3] = 0xFF;
    TxHeartbeatData[4] = 0xFF;
    TxHeartbeatData[5] = 0xFF;
    TxHeartbeatData[6] = 0xFF;
    TxHeartbeatData[7] = 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeartbeat, TxHeartbeatData) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
}


void app_init(){

    FDCAN_Config();

    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUFFER_LEN);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
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

            can_heartbeat();

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
            // if (abs(vbatt - vts) > 0.2 * vbatt){ 
            //     // Disable contactors
            //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
            //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  
            //     TSStatus = 0;
            // }
        }
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	(void)hadc;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    (void)hadc;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
        {
        /* retrieve Rx messages from RX FIFO0 */
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
        {
            /* Reception Error */
            Error_Handler();
        }
        
        if ((RxHeader.Identifier == 0x321) && (RxHeader.IdType == FDCAN_STANDARD_ID))
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
        }
    }
}