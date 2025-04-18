/**
 * **************************************************************
 * @file mylib/rgb.c
 * @author Travis Graham
 * @date 14/08/2024
 * @brief mylib driver file for rgb module
 ***************************************************************
  * EXTERNAL FUNCTIONS 
 ***************************************************************
 * extern void tsk_rgb_init(void) - initialiser
 *************************************************************** 
 */

#include "board.h"
#include "processor_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "rgb.h"

QueueHandle_t RgbStateQueue;
SemaphoreHandle_t xRgbMutex;


// Debugging
// #define DEBUG // Uncomment to turn on debug mode
#ifdef DEBUG
    #include "debug_log.h"
#endif

/** 
 * reg_rgb_init()
 * 
 * Initialise all hardware pins required for the RGB module
 * Initialise PWM on the brightness line
 */
void reg_rgb_init() {
    __GPIOE_CLK_ENABLE();
    __GPIOF_CLK_ENABLE();

    /* Initialise E11 as an output. (Red) */
    GPIOE->MODER &= ~(0x03 << (11 * 2));  //clear bits
    GPIOE->MODER |= (0x01 << (11 * 2));   //Set for push pull

    GPIOE->OSPEEDR &= ~(0x03<<(11 * 2));
    GPIOE->OSPEEDR |= (0x02<<(11 * 2));  // Set for Fast speed

    GPIOE->OTYPER &= ~(0x01 << 11);    //Clear Bit for Push/Pull Output

    // Activate the Pull-up or Pull down resistor for the current IO
    GPIOE->PUPDR &= ~(0x03 << (11 * 2));   //Clear Bits
    GPIOE->PUPDR |= (0x01 << (11 * 2));  //Set for Pull down output

    /* Initialise F14, F15 as an output. (Green and Blue respectively) */
    GPIOF->MODER &= ~(0x03 << (14 * 2) | (0x03 << (15 * 2)));  //clear bits
    GPIOF->MODER |= (0x01 << (14 * 2) | (0x01 << (15 * 2)));   //Set for push pull

    GPIOF->OSPEEDR &= ~(0x03<<(14 * 2) | (0x03 << (15 * 2)));
    GPIOF->OSPEEDR |=   (0x02<<(14 * 2) | (0x02 << (15 * 2)));  // Set for Fast speed

    GPIOF->OTYPER &= ~((0x01 << 14) | (0x01 << (15 * 2)));    //Clear Bit for Push/Pull Output

    // Activate the Pull-up or Pull down resistor for the current IO
    GPIOF->PUPDR &= ~(0x03 << (14 * 2) | (0x03 << (15 * 2)));   //Clear Bits
    GPIOF->PUPDR |= ((0x01 << (14 * 2)) | (0x01 << (15 * 2)));  //Set for Pull down output


    // /* Brightness Control (PE9) Init and PWM */
    // GPIOE->OSPEEDR |= (GPIO_SPEED_FAST << (9 * 2));		//Set fast speed.
    // GPIOE->PUPDR &= ~(0x03 << (9 * 2));				//Clear bits for no push/pull
    // GPIOE->MODER &= ~(0x03 << (9 * 2));				//Clear bits
    // GPIOE->MODER |= (GPIO_MODE_AF_PP << (9 * 2));  	//Set Alternate Function Push Pull Mode

    // GPIOE->AFR[1] &= ~((0x0F) << ((9-8) * 4));			//Clear Alternate Function for pin (lower ARF register)
    // GPIOE->AFR[1] |= (GPIO_AF1_TIM1 << ((9-8) * 4));	//Set Alternate Function for pin (lower ARF register)

    // __TIM1_CLK_ENABLE();

    // /* Compute the prescaler value
    //     Set the clock prescaler to 50kHz
    //     SystemCoreClock is the system clock frequency */
    // TIM1->PSC = ((SystemCoreClock / 2) / TIMER_COUNTER_FREQ) - 1;

    // // Counting direction: 0 = up-counting, 1 = down-counting (Timer Control Register 1)
    // TIM1->CR1 &= ~TIM_CR1_DIR;

    // TIM1->ARR = PWM_PULSE_WIDTH_TICKS; 		//Set pulse period to ~1s
    // TIM1->CCR1 = PWM_PERCENT2TICKS_DUTYCYCLE(0);	//Set pulse width to 0% duty cycle.

    // TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M); 	// Clear OC1M (Channel 1)
    // TIM1->CCMR1 |= (0x6 << 4); 		    // Enable PWM Mode 1, upcounting, on Channel 1
    // TIM1->CCMR1 |= (TIM_CCMR1_OC1PE); 	// Enable output preload bit for channel 1

    // TIM1->CR1  |= (TIM_CR1_ARPE); 	// Set Auto-Reload Preload Enable
    // TIM1->CCER |= TIM_CCER_CC1E; 	// Set CC1E Bit
    // TIM1->CCER &= ~TIM_CCER_CC1NE; 	// Clear CC1NE Bit for active high output

    // TIM1->CCER &= ~TIM_CCER_CC1P; 	// Polarity of PWM output waveform is active high.
    // // If set, reverses polarity (e.g. high becomes low).
    // // See p576 of the STM32F429 reference manual

    // /* Set Main Output Enable (MOE) bit
    //     Set Off-State Selection for Run mode (OSSR) bit
    //     Set Off-State Selection for Idle mode (OSSI) bit */
    // TIM1->BDTR |= TIM_BDTR_MOE | TIM_BDTR_OSSR | TIM_BDTR_OSSI;

    // TIM1->CR1 |= TIM_CR1_CEN;	// Enable the counter
}

/** 
 * reg_rgb_brightness_write()
 * 
 * Given a brightness bitmask, set the brightness of the RGB module. Each 
 * significant bit in the bitmask represents a brightness percentage, starting
 * at 10%. That is, 0x0001 represents 10%, 0x0002 represents 20%, 0x0010
 * represnets 50% and so on.
 * 
 * @param unsigned short level the bitmask representing the desired brightness
 */
void reg_rgb_brightness_write(unsigned short level) {
    switch (level){
        case BRIGHTNESS_0_PERCENT:
            // Set brightness to appropriate value using macros
            PWM_DUTYCYCLE_CHANGE(PWM_PERCENT2TICKS_DUTYCYCLE(0)); 
            break;
        case BRIGHTNESS_10_PERCENT:
            PWM_DUTYCYCLE_CHANGE(PWM_PERCENT2TICKS_DUTYCYCLE(5)); 
            break;
        case BRIGHTNESS_20_PERCENT:
            PWM_DUTYCYCLE_CHANGE(PWM_PERCENT2TICKS_DUTYCYCLE(10));
            break;
        case BRIGHTNESS_30_PERCENT:
            PWM_DUTYCYCLE_CHANGE(PWM_PERCENT2TICKS_DUTYCYCLE(15));
            break;
        case BRIGHTNESS_40_PERCENT:
            PWM_DUTYCYCLE_CHANGE(PWM_PERCENT2TICKS_DUTYCYCLE(20));
            break;
        case BRIGHTNESS_50_PERCENT:
            PWM_DUTYCYCLE_CHANGE(PWM_PERCENT2TICKS_DUTYCYCLE(25));
            break;
        case BRIGHTNESS_60_PERCENT:
            PWM_DUTYCYCLE_CHANGE(PWM_PERCENT2TICKS_DUTYCYCLE(30));
            break;
        case BRIGHTNESS_70_PERCENT:
            PWM_DUTYCYCLE_CHANGE(PWM_PERCENT2TICKS_DUTYCYCLE(35));
            break;
        case BRIGHTNESS_80_PERCENT:
            PWM_DUTYCYCLE_CHANGE(PWM_PERCENT2TICKS_DUTYCYCLE(40));
            break;
        case BRIGHTNESS_90_PERCENT:
            PWM_DUTYCYCLE_CHANGE(PWM_PERCENT2TICKS_DUTYCYCLE(45));
            break;
        case BRIGHTNESS_100_PERCENT:
            PWM_DUTYCYCLE_CHANGE(PWM_PERCENT2TICKS_DUTYCYCLE(50));
            break;
    }
}


/**
 * reg_rgb_brightness_read()
 * 
 * Return a bitmask representing the current brightness
 * 
 * @returns int the brightness bit mask representing the brightness
 * 
 */
int reg_rgb_brightness_read() {
    uint32_t pwmValue = TIM1->CCR1;

    // Convert back to percentage value
    int brightness_percent = (int)((float)pwmValue * 100.0f / 200.0f); 

    // Return corresponding bit mask
    switch (brightness_percent){
    case 0:
        return BRIGHTNESS_0_PERCENT;
        break;
    case 10:
        return BRIGHTNESS_10_PERCENT;
        break;
    case 20:
        return BRIGHTNESS_20_PERCENT;
        break;
    case 30:
        return BRIGHTNESS_30_PERCENT;
        break;
    case 40:
        return BRIGHTNESS_40_PERCENT;
        break;
    case 50:
        return BRIGHTNESS_50_PERCENT;
        break;
    case 60:
        return BRIGHTNESS_60_PERCENT;
        break;
    case 70:
        return BRIGHTNESS_70_PERCENT;
        break;
    case 80:
        return BRIGHTNESS_80_PERCENT;
        break;
    case 90:
        return BRIGHTNESS_90_PERCENT;
        break;
    case 100:
        return BRIGHTNESS_100_PERCENT;
        break;
    }
}

/** 
 * reg_rgb_colour_set
 * 
 * Given a 3 bit mask in the format of <R><G><B>, set the colour of the RGB
 * module. If the bit is 1, then the correspeonding colour of that position will
 * be set to high. If the bit is 0, it will be set to low.
 * 
 * @param unsigned char rgb_mask the 3 bit mask used to set the colour
 */
void reg_rgb_colour_set(unsigned char rgb_mask) {
    // debug_log("RGB_MASK: %x\n\r", rgb_mask);
        if ((rgb_mask & 0x04)) {
            // RGB Module is acive low
            GPIOE->ODR &= ~(1 << RGB_RED);
        } else {
            GPIOE->ODR |= (1 << RGB_RED);
        }

        if (xSemaphoreTake(xRgbMutex, 1000) == pdTRUE) {
            SYSMON_CHAN1_TOGGLE();
            if (rgb_mask & 0x02) {
                GPIOF->ODR &= ~(1 << RGB_GREEN);
            } else {
                GPIOF->ODR |= (1 << RGB_GREEN);
            }
            xSemaphoreGive(xRgbMutex);

        }

        if (rgb_mask & 0x01) {
            GPIOF->ODR &= ~(1 << RGB_BLUE);
        } else {
            GPIOF->ODR |= (1 << RGB_BLUE);
        }
}

/**
 * Controlling task for the RGB LED.
 */
void s4747355_ctrl_task_rgb_led() {
    reg_rgb_init();

    uint8_t rgbColourMask = BLACK;

    xRgbMutex = xSemaphoreCreateMutex();
    
    RgbStateQueue = xQueueCreate(10, sizeof(rgbColourMask));
    reg_rgb_colour_set(rgbColourMask);

    for (;;) {

        if (RgbStateQueue != NULL) {
            if (xQueueReceive(RgbStateQueue, &rgbColourMask, 10)) {
                    reg_rgb_colour_set(rgbColourMask);
            }

        }
        // reg_rgb_colour_set(rgbColourMask);

        vTaskDelay(10);
    }
}

/**
 * Initialiser task for the controlling task for the RGB LED
 */
extern void tsk_rgb_init(void)
{
    // Create the controlling task
    xTaskCreate(&s4747355_ctrl_task_rgb_led, "RGB_Control_Task", RGB_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
}