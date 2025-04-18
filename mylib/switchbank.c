/** 
 *********************************************************************
 * @file mylib/switchbank.c
 * @author Travis Graham - 47437553
 * @date 25/08/2024
 * @brief Swtichbank driver
 **********************************************************************
 * EXTERNAL FUNCTIONS 
 **********************************************************************
 * int reg_switchbank_read() 
        - Reads the current values of the switchbank
 * void reg_switchbank_init()
        - Initialises the switchbank GPIO pins
 **********************************************************************
 */
#include "board.h"
#include "processor_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "string.h"
#include "switchbank.h"

/* Queue for sharing switchbank values*/
QueueHandle_t SwitchBankDataQueue;

/* Switch bank bit mask*/
unsigned long switchBankData;


/** 
 * Enabled the switch bank GPIO pins as inputs at the same time.
 */
void reg_switchbank_init() { 
    __GPIOE_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    // Initialize PE8, PE7, PE10, PE12, PE14, PE15 as inputs
    GPIOE->MODER &= ~((0x03 << (8 * 2)) | 
                    (0x03 << (7 * 2)) | 
                    (0x03 << (10 * 2)) | 
                    (0x03 << (12 * 2)) | 
                    (0x03 << (14 * 2)) | 
                    (0x03 << (15 * 2))); // Clear bits for input mode

    // Set all pins to Fast speed
    GPIOE->OSPEEDR &= ~((0x03 << (8 * 2)) | 
                        (0x03 << (7 * 2)) | 
                        (0x03 << (10 * 2)) | 
                        (0x03 << (12 * 2)) | 
                        (0x03 << (14 * 2)) | 
                        (0x03 << (15 * 2))); 
    GPIOE->OSPEEDR |=  ((0x02 << (8 * 2)) | 
                        (0x02 << (7 * 2)) | 
                        (0x02 << (10 * 2)) | 
                        (0x02 << (12 * 2)) | 
                        (0x02 << (14 * 2)) | 
                        (0x02 << (15 * 2)));  // Fast speed

    // Set all pins to Pull-down
    GPIOE->PUPDR &= ~((0x03 << (8 * 2)) | 
                    (0x03 << (7 * 2)) | 
                    (0x03 << (10 * 2)) | 
                    (0x03 << (12 * 2)) | 
                    (0x03 << (14 * 2)) | 
                    (0x03 << (15 * 2))); 
    GPIOE->PUPDR |=  ((0x02 << (8 * 2)) | 
                    (0x02 << (7 * 2)) | 
                    (0x02 << (10 * 2)) | 
                    (0x02 << (12 * 2)) | 
                    (0x02 << (14 * 2)) | 
                    (0x02 << (15 * 2)));  // Pull-down

    // Initialize PB4, PB11 as inputs
    GPIOB->MODER &= ~((0x03 << (3 * 2)) | 
                    (0x03 << (11 * 2))); // Clear bits for input mode

    // Set PB4, PB11 to Fast speed
    GPIOB->OSPEEDR &= ~((0x03 << (3 * 2)) | 
                        (0x03 << (11 * 2))); 
    GPIOB->OSPEEDR |=  ((0x02 << (3 * 2)) | 
                        (0x02 << (11 * 2)));  // Fast speed

    // Set PB4, PB11 to Pull-down
    GPIOB->PUPDR &= ~((0x03 << (3 * 2)) | 
                    (0x03 << (11 * 2))); 
    GPIOB->PUPDR |=  ((0x02 << (3 * 2)) | 
                    (0x02 << (11 * 2)));  // Pull-down
}

/**
 * Returns a bit mask of the switch bank inputs. The LSB should correspond to 
 * first slide switch (Switch[0]) and MSB should to 8th slide switch 
 * (SwitchBank[7])
 * 
 * @returns unsigned char the bitmask of the switch bank inputs.
 */
int reg_switchbank_read() {
    uint8_t value;
    int b0, b1, b2, b3, b4, b5, b6, b7;

    // Read values from  PE8, PE7, PE10, PE12, PE14, PE15, PB4, PB11
    b0 = GPIOE->IDR & (1 << 8) ? 1 : 0;
    b1 = GPIOE->IDR & (1 << 7) ? 1 : 0;
    b2 = GPIOE->IDR & (1 << 10) ? 1 : 0;
    b3 = GPIOE->IDR & (1 << 12) ? 1 : 0;
    b4 = GPIOE->IDR & (1 << 14) ? 1 : 0;
    b5 = GPIOE->IDR & (1 << 15) ? 1 : 0;
    b6 = GPIOB->IDR & (1 << 3) ? 1 : 0;
    b7 = GPIOB->IDR & (1 << 11) ? 1 : 0;

    value = b0 << 0 | (b1 << 1) | (b2 << 2) | (b3 << 3) | (b4 << 4) | 
            (b5 << 5) | (b6 << 6) | (b7 << 7);
    return value;
}

/**
 * Controlling task. Initialises regiseters using init function and creates
 * queue for storing switchbank values.
 */
void task_switchbank(void *pvParameters) {
    // Initialise push button
    reg_switchbank_init();

    // Create semaphore for button press synchronization
    SwitchBankDataQueue = xQueueCreate (10, sizeof(switchBankData));

    char message[8];

    for (;;) {
        if (SwitchBankDataQueue != NULL) {	// Check if queue exists 
            // Send message to the front of the queue - wait atmost 10 ticks 
            switchBankData = reg_switchbank_read();
            xQueueSendToFront(
                SwitchBankDataQueue, ( void * ) &switchBankData, ( portTickType ) 10 );
        }
        vTaskDelay(3);
    }
}

/**
 * Hardware initialiser task. Creates a task and parses in the controlling task
 * function. Does not handle register intit directly.
 */
extern void tsk_switchbank_init(void) {
    // Create the task for the controlling task function
    xTaskCreate(&task_switchbank, "Switchbank_Task", SB_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
}