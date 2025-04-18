/**
 * **************************************************************
 * @file mylib/ssd.c
 * @author Travis Graham
 * @date 25/09/2024
 * @brief mylib driver file for the seven segment display
 * ***************************************************************
  * EXTERNAL FUNCTIONS 
 ***************************************************************
 * void init_tsk_ssd(void) - init task function for ssd
 *************************************************************** 
 */

#include "board.h"
#include "processor_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "string.h"
#include "semphr.h"
#include "task.h"
#include "math.h"
#include "ssd.h"

#define DEBUG // Uncomment to turn on debug mode
#ifdef DEBUG
    #include "debug_log.h"
#endif

/* Queue holds the next number to display on the SSD */
QueueHandle_t SevenSegQueue;

extern SemaphoreHandle_t xRgbMutex;

/* Controlling Cathode indicies */
const int CATHODE_CONTROL[NUM_CATHODES] = {0x01, 0x02, 0x04, 0x08};

/* Binary encoded digits to display on the SSD */
int digits[4] = {DIGIT_ZERO, DIGIT_ZERO, DIGIT_ZERO, DIGIT_ZERO}; // SSD is zero by default

/** 
 * Initialise shift register pins to control MFS seven segment display
 */
void reg_ssd_init() {
    __GPIOF_CLK_ENABLE();
    // Initialies PF12 (Storage Register), PF13 (Registe CLK), PF14 (Serial Data Input)
    GPIOF->MODER &= ~(0x03 << (12 * 2) | (0x03 << (13 * 2)) | (0x03 << (14 * 2)));  
    GPIOF->MODER |= (0x01 << (12 * 2) | (0x01 << (13 * 2)) | (0x01 << (14 * 2)));  

    GPIOF->OSPEEDR &= ~(0x03 << (12 * 2) | 0x03 << (13 * 2) | 0x03 << (14 * 2));  
    GPIOF->OSPEEDR |= (0x02 << (12 * 2) | 0x02 << (13 * 2) | 0x02 << (14 * 2));  

    GPIOF->OTYPER &= ~(0x01 << 12 | 0x01 << 13 | 0x01 << 14);  

    GPIOF->PUPDR &= ~(0x03 << (12 * 2) | 0x03 << (13 * 2) | 0x03 << (14 * 2));  
    GPIOF->PUPDR |= (0x01 << (12 * 2) | 0x01 << (13 * 2) | 0x01 << (14 * 2));

}

/**
 * Given an 8bit binary sequence of data, shift the data
 * into the shift register.
 * 
 * @param data the 8 bit sequence of data representing the 
 *
 */
void reg_ssd_shift_digit_out(int data) {
    for (int i = 7; i >= 0; i--) {
        if ((data >> i) & 0x01) {
            GPIOF->ODR |= (1 << PIN_SER);
        } else {
            GPIOF->ODR &= ~(1 << PIN_SER);
        }

        // Clock SRCLK to shift in bit
        GPIOF->ODR |= (1 << PIN_SRCLK);
        GPIOF->ODR &= ~(1 << PIN_SRCLK);
    }
}

/**
 * Given an 8bit sequence of data representing the controlling
 * cathode, shift the data into the shift register. 
 * 
 * Precondition: The most significant half byte should be all zeros.
 * 
 * @param cathode the 8 bit sequence of data reprenting the controlling 
 *                   cathode
 */
void reg_ssd_shift_cathode_out(int cathode) {
    for (int i = 7; i >= 0; i--) {
        if ((cathode >> i) & 0x01) {
            GPIOF->ODR |= (1 << PIN_SER);
        } else {
            GPIOF->ODR &= ~(1 << PIN_SER);
        }

        // Clock SRCLK to shift in bit
        GPIOF->ODR |= (1 << PIN_SRCLK);
        GPIOF->ODR &= ~(1 << PIN_SRCLK);
    }

}

/**
 * Display a digit on the seven segment display at a desired digit location
 * (cathode)
 * 
 * @param cathode 8 bit binary sequence representing the controlling cathode. 
 *                Most significant four bits must be zero. 
 * 
 * @param digit 8 bit binary sequence representing a digit to display on the 
 *              seven segment display.
 */
void reg_ssd_display_digit(int cathode, int digit) {

        reg_ssd_shift_digit_out(digit);
        reg_ssd_shift_cathode_out(cathode);
        
        int state = GPIOF->ODR & (1 << PIN_RCLK); // Check the value 
        if (xSemaphoreTake(xRgbMutex, portMAX_DELAY) == pdTRUE) {
            SYSMON_CHAN0_SET();
            // Pulse RCLK to update storage register (display data)
            GPIOF->ODR |= (1 << PIN_RCLK);
            GPIOF->ODR &= ~(1 << PIN_RCLK);

            if (state) {
                GPIOF->ODR |= (1 << PIN_RCLK);
            } else {
                GPIOF->ODR &= ~(1 << PIN_RCLK);
            }
            SYSMON_CHAN0_CLR();
            xSemaphoreGive(xRgbMutex);
        }
        


}

/**
 * Display the global digits on the seven segment display
 */
void ssd_display_number() {
    for (int i = 0; i < NUM_CATHODES; i++) {
        reg_ssd_display_digit(CATHODE_CONTROL[i], digits[i]);
    }
}

/**
 * Given a four digit number, isolate the digits and update the global
 * variables to display on the seven segment display.
 */
void ssd_update_digits(int fourDigitNum) {

    int isolatedNums[4] = {
        (int) floor(fourDigitNum / 1000) % 10,
        (int) floor(fourDigitNum / 100) % 10,
        (int) floor(fourDigitNum / 10) % 10,
        (int) floor(fourDigitNum / 1) % 10
    };

    for (int i = 0; i < NUM_CATHODES; i++) {
            // debug_log("Calc'ed digit: %d, i = %d\n\r", (int) floor(fourDigitNum / (1 << i)) % 10, i);

        switch (isolatedNums[i]) {
            case (0):
                digits[i] = DIGIT_ZERO;
                break;
            case (1):
                digits[i] = DIGIT_ONE;
                break;
            case (2):
                digits[i] = DIGIT_TWO;
                break;
            case (3):
                digits[i] = DIGIT_THREE;
                break;
            case (4):
                digits[i] = DIGIT_FOUR;
                break;
            case (5):
                digits[i] = DIGIT_FIVE;
                break;
            case (6):
                digits[i] = DIGIT_SIX;
                break;
            case (7):
                digits[i] = DIGIT_SEVEN;
                break;
            case (8):
                digits[i] = DIGIT_EIGHT;
                break;
            case (9):
                digits[i] = DIGIT_NINE;
                break;
        }
        vTaskDelay(10);
    }
}

/**
 * Control task for the seven segment display.
 */
void ctrl_task_ssd(void *pvParameters) {
    // Init pins for seven seg display
    reg_ssd_init();
    
    // Number to display
    int number;

    SevenSegQueue = xQueueCreate(5, sizeof(number));

    for (;;) {

        if (xQueueReceive(SevenSegQueue, &number, (portTickType) 1)) {
            ssd_update_digits(number);
        }

        // Display the number on the seven segment display
        ssd_display_number();
    }
}

/** 
 * Initialiser task for the seven segment display.
 */
extern void init_tsk_ssd(void) {
    // Create the controlling task
    xTaskCreate(
        &ctrl_task_ssd, "SSD_CTRL_TASK", SSD_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
}