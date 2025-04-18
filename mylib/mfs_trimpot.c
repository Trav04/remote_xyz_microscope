/** 
 **************************************************************
 * @file mylib/mfs_trimpot.c
 * @author Travis Graham
 * @date 15/07/2024
 * @brief mfs trimpot driver library
 ***************************************************************
  * EXTERNAL FUNCTIONS 
 ***************************************************************
 * void reg_mfs_trimpot_init(); - initialise GPIO pins for trimpot
 * int reg_mfs_trimpot_get(); - get the trimpot ADC value
 *************************************************************** 
 */

#include "mfs_trimpot.h"
#include "board.h"
#include "processor_hal.h"

ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef AdcChanConfig;

/**
 * reg_mfs_trimpot_init
 * 
 * Initialise GPIO pins for trimpot and ADC
 */
void reg_mfs_trimpot_init() {
    /* Initialise A0. (Trimpot input) */
    __GPIOA_CLK_ENABLE();

	// Initalise PA3 as an Analog input.
  	GPIOA->MODER |= (0x03 << (3 * 2)); //Set bits for Analog input mode

  	GPIOA->OSPEEDR &= ~(0x03 << (3 * 2));
	GPIOA->OSPEEDR |= 0x02 << (3 * 2); // Fast speed

	GPIOA->PUPDR &= ~(0x03 << (3 * 2)); //Clear bits for no push/pull

	__ADC1_CLK_ENABLE(); //Enable ADC1 clock

    // ADC1 - 12Bit resolution
    AdcHandle.Instance                   = (ADC_TypeDef *)(ADC1_BASE);
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle.Init.ScanConvMode          = DISABLE;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.NbrOfDiscConversion   = 0;
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.NbrOfConversion       = 1;
    AdcHandle.Init.DMAContinuousRequests = DISABLE;
    AdcHandle.Init.EOCSelection          = DISABLE;

    HAL_ADC_Init(&AdcHandle);

    // Configure ADC channel for Board pin A3
    AdcChanConfig.Channel = ADC_CHANNEL_3;
    AdcChanConfig.Rank = 1;
    AdcChanConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    AdcChanConfig.Offset = 0;

    HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConfig);

}

/** 
 * reg_mfs_trimpot_get()
 * 
 * Returns the current value of the trimpot from the ADC
 */
int reg_mfs_trimpot_get() {
    // ADC Value
    unsigned int adc_value;
    // Start and wait for ADC conversion
    HAL_ADC_Start(&AdcHandle);
    
    // Wait for ADC conversion to finish
    while (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK);
    adc_value = HAL_ADC_GetValue(&AdcHandle); // Get the ADC value

    return adc_value;
}