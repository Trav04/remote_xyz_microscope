 /** 
 *********************************************************************
 * @file mylib/led.h
 * @author Travis Graham
 * @date 25/08/2024
 * @brief MFS LED driver
 **********************************************************************
 * EXTERNAL FUNCTIONS 
 **********************************************************************
 * void reg_mfs_led_init()
 *      - Initialises the LEDs on the MFS
 **********************************************************************
 */

#include "board.h"
#include "processor_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "string.h"
#include "task.h"
#include "mfs_led.h"

/* Queue for sharing switchbank values*/
QueueHandle_t MfsLedStatesQueue;

/* LED States bitmask*/
unsigned short int mfsLedBitmask;

/** 
 * Initialises the MFS LEDs, D1, D2, D3, D4
 */
void reg_mfs_led_init() {
    __GPIOA_CLK_ENABLE();

    /* Initialise and set PA5, PA6, PA7 as outputs*/
    GPIOA->MODER &= ~(0x03 << (5 * 2) | 0x03 << (6 * 2) | 0x03 << (7 * 2));
    GPIOA->MODER |= (0x01 << (5 * 2) | 0x01 << (6 * 2) | 0x01 << (7 * 2));

    GPIOA->OSPEEDR &= ~(0x03 << (5 * 2) | 0x03 << (6 * 2) | 0x03 << (7 * 2));
    GPIOA->OSPEEDR |= (0x02 << (5 * 2) | 0x02 << (6 * 2) | 0x02 << (7 * 2));

    GPIOA->OTYPER &= ~(0x01 << 5 | 0x01 << 6 | 0x01 << 7);

    GPIOA->PUPDR &= ~(0x03 << (5 * 2) | 0x03 << (6 * 2) | 0x03 << (7 * 2));
    GPIOA->PUPDR |= (0x01 << (5 * 2) | 0x01 << (6 * 2) | 0x01 << (7 * 2));

    GPIOA->ODR |= (0x01 << PIN_LED_D1);
    GPIOA->ODR |= (0x01 << PIN_LED_D2);
    GPIOA->ODR |= (0x01 << PIN_LED_D3);

    /* Initialise and set PD14 as output*/
    __GPIOD_CLK_ENABLE();

    GPIOD->MODER &= ~(0x03 << (14 * 2));  //clear bits
    GPIOD->MODER |= (0x01 << (14 * 2));   //Set for push pull

    GPIOD->OSPEEDR &= ~(0x03<<(14 * 2));
    GPIOD->OSPEEDR |=   0x02<<(14 * 2);  // Set for Fast speed

    GPIOD->OTYPER &= ~(0x01 << 14);       //Clear Bit for Push/Pull Output

    // Activate the Pull-up or Pull down resistor for the current IO
    GPIOD->PUPDR &= ~(0x03 << (14 * 2));   //Clear Bits
    GPIOD->PUPDR |= ((0x01) << (14 * 2));  //Set for Pull down output
    
    GPIOD->ODR |= (0x01 << PIN_LED_D4);
}

/**
 * Controlling task for MFS leds. Initialises a queue that will read a bitmask
 * where each bit corresponds to the state value of each LED[0..4]. This 
 * controlling task reads from the queue. To set the value of the LEDs, write
 * to this queue a bitmask of size 4 where each bit is the desired LED state.
 * 
 * @param pvParameters cast to an int bitmask. LSB represents the high or low
 *                     state of an LED. D1 is represented by the LSB. D4 is 
 *                     represented by the MSB
 */
void mfs_led_ctrl_tsk(void *pvParameters) {
    reg_mfs_led_init(); // Initiate diagnostic LEDs

    MfsLedStatesQueue = xQueueCreate(10, sizeof(mfsLedBitmask));

    int ledBitmask = (int) pvParameters;
    uint8_t d1Led;
    uint8_t d2Led;
    uint8_t d3Led;
    uint8_t d4Led;

    for (;;) {
        if (MfsLedStatesQueue != NULL) {

            if (xQueueReceive(MfsLedStatesQueue, &ledBitmask, 10)) {
                d1Led = ledBitmask & 0x01;
                d2Led = ledBitmask & 0x02;
                d3Led = ledBitmask & 0x04;
                d4Led = ledBitmask & 0x08;

                // Set the LED State values 
                if (d1Led) {
                    REG_MFS_LED_D1_ON;
                } else {
                    REG_MFS_LED_D1_OFF;
                }

                if (d2Led) {
                    REG_MFS_LED_D2_ON;
                } else {
                    REG_MFS_LED_D2_OFF;
                }

                if (d3Led) {
                    REG_MFS_LED_D3_ON;
                } else {
                    REG_MFS_LED_D3_OFF;
                }

                if (d4Led) {
                    REG_MFS_LED_D4_ON;
                } else {
                    REG_MFS_LED_D4_OFF;
                }
            }
        }
        
        vTaskDelay(3);
    }
}

/**
 * MFS LED Initialisation task. Creates the controlling task and parse.
 */
extern void mfs_led_init_tsk() {
    xTaskCreate(
        &mfs_led_ctrl_tsk, "MFS_LED_Task", MFS_LED_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
}
