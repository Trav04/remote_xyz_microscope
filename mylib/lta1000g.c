 /** 
 **************************************************************
 * @file mylib/lta1000g.c
 * @author Travis Graham
 * @date 26/07/2024
 * @brief lta1000g driver
 * 
 * Light bar numbers are defined as 0-9 when the cut corner is orientated
 * in the top right corner. The pin to number map is as follows,
 * 9: PG6
 * 8: PG5
 * 7: PG8
 * 6: PE0
 * 5: PF11
 * 4: PG11
 * 3: PG13
 * 2: PG10
 * 1: PG15
 * 0: PE6
 ***************************************************************
 * EXTERNAL FUNCTIONS 
 ***************************************************************
 * extern void tsk_lta1000g_init(void) - initialiser
 *************************************************************** 
 */
#include "board.h"
#include "processor_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "lta1000g.h"


/* Queue to read led light bar values form bitmask*/
QueueHandle_t lta1000gQueue;


/**
 * Initialise pins E0, E6, G15, G13, G11, G10, G8, G6, G5, F11
 * as outputs.
 */
void reg_lta1000g_init() {

    // Enable the GPIO Clock
    __GPIOE_CLK_ENABLE();
    __GPIOG_CLK_ENABLE();
    __GPIOF_CLK_ENABLE();

    // PORT E

    //Initialise E0, E6 as an output.
    GPIOE->MODER &= ~(0x03 << (6 * 2) | (0x03 << (0 * 2)));  //clear bits
    GPIOE->MODER |= (0x01 << (6 * 2) | (0x01 << (0 * 2)));   //Set for push pull

    GPIOE->OSPEEDR &= ~(0x03<<(6 * 2) | (0x03 << (0 * 2)));
    GPIOE->OSPEEDR |=   (0x02<<(6 * 2) | 0x02 << (0 * 2));  // Set for Fast speed

    GPIOE->OTYPER &= ~((0x01 << 6) | (0x01 << 0));       //Clear Bit for Push/Pull Output

    // Activate the Pull-up or Pull down resistor for the current IO
    GPIOE->PUPDR &= ~(0x03 << (6 * 2) | 0x03 << (0 * 2));   //Clear Bits
    GPIOE->PUPDR |= ((0x01) << (6 * 2) | (0x01 << (0 * 2)));  //Set for Pull down output

    // // PORT G

    // Clear bits for GPIO pins 15, 13, 11, 10, 8, 6, and 5
    GPIOG->MODER &= ~(0x03 << (15 * 2) | 0x03 << (13 * 2) | 0x03 << (11 * 2) | 0x03 << (10 * 2) | 
                    0x03 << (8 * 2) | 0x03 << (6 * 2) | 0x03 << (5 * 2));

    // Set mode for push-pull for GPIO pins 15, 13, 11, 10, 8, 6, and 5
    GPIOG->MODER |= (0x01 << (15 * 2) | 0x01 << (13 * 2) | 0x01 << (11 * 2) | 0x01 << (10 * 2) |
                    0x01 << (8 * 2) | 0x01 << (6 * 2) | 0x01 << (5 * 2));

    // Clear speed bits for GPIO pins 15, 13, 11, 10, 8, 6, and 5
    GPIOG->OSPEEDR &= ~(0x03 << (15 * 2) | 0x03 << (13 * 2) | 0x03 << (11 * 2) | 0x03 << (10 * 2) | 
                        0x03 << (8 * 2) | 0x03 << (6 * 2) | 0x03 << (5 * 2));

    // Set fast speed for GPIO pins 15, 13, 11, 10, 8, 6, and 5
    GPIOG->OSPEEDR |= (0x02 << (15 * 2) | 0x02 << (13 * 2) | 0x02 << (11 * 2) | 0x02 << (10 * 2) |
                    0x02 << (8 * 2) | 0x02 << (6 * 2) | 0x02 << (5 * 2));

    // Clear output type bits for GPIO pins 15, 13, 11, 10, 8, 6, and 5 for push-pull output
    GPIOG->OTYPER &= ~(0x01 << 15 | 0x01 << 13 | 0x01 << 11 | 0x01 << 10 |
                    0x01 << 8 | 0x01 << 6 | 0x01 << 5);

    // Clear pull-up/pull-down bits for GPIO pins 15, 13, 11, 10, 8, 6, and 5
    GPIOG->PUPDR &= ~(0x03 << (15 * 2) | 0x03 << (13 * 2) | 0x03 << (11 * 2) | 0x03 << (10 * 2) |
                    0x03 << (8 * 2) | 0x03 << (6 * 2) | 0x03 << (5 * 2));

    // Set pull-down for GPIO pins 15, 13, 11, 10, 8, 6, and 5
    GPIOG->PUPDR |= (0x01 << (15 * 2) | 0x01 << (13 * 2) | 0x01 << (11 * 2) | 0x01 << (10 * 2) |
                    0x01 << (8 * 2) | 0x01 << (6 * 2) | 0x01 << (5 * 2));

    // PORT F

    //Initialise F11 as an output.
    GPIOF->MODER &= ~(0x03 << (11 * 2));  //clear bits
    GPIOF->MODER |= (0x01 << (11 * 2));   //Set for push pull

    GPIOF->OSPEEDR &= ~(0x03<<(11 * 2));
    GPIOF->OSPEEDR |=   0x02<<(11 * 2);  // Set for Fast speed

    GPIOF->OTYPER &= ~(0x01 << 11);       //Clear Bit for Push/Pull Output

    // Activate the Pull-up or Pull down resistor for the current IO
    GPIOF->PUPDR &= ~(0x03 << (11 * 2));   //Clear Bits
    GPIOF->PUPDR |= ((0x01) << (11 * 2));  //Set for Pull down output
}

/** 
 * Sets an LED lightbar segment high or low
 * 
 * @param segment is the segment number from 0-9 that is to be controlled
 * @param unsigned char value is either 0 or 1 to turn the pin to high or low 
 */
void lta1000g_seg_set(int segment, unsigned char value) {
    switch (segment) {
        case PE6: 
            if (value) {
                GPIOE->ODR |= (0x01 << 6);   //Set the bit in the ODR
            } else {
                GPIOE->ODR &= ~(0x01 << 6);   //Clear the bit in the ODR
            }
            break;
        case PG15: 
            if (value) {
                GPIOG->ODR |= (0x01 << 15);
            } else {
                GPIOG->ODR &= ~(0x01 << 15);
            }
            break;
        case PG10:
            if (value) {
                GPIOG->ODR |= (0x01 << 10);
            } else {
                GPIOG->ODR &= ~(0x01 << 10);
            }
            break;
        case PG13:
            if (value) {
                GPIOG->ODR |= (0x01 << 13);
            } else {
                GPIOG->ODR &= ~(0x01 << 13);
            }
            break;
        case PG11:
            if (value) {
                GPIOG->ODR |= (0x01 << 11);
            } else {
                GPIOG->ODR &= ~(0x01 << 11);
            }
            break;
        case PF11:
            if (value) {
                GPIOF->ODR |= (0x01 << 11);
            } else {
                GPIOF->ODR &= ~(0x01 << 11);
            }
            break;
        case PE0:
            if (value) {
                GPIOE->ODR |= (0x01 << 0);
            } else {
                GPIOE->ODR &= ~(0x01 << 0);
            }
            break;
        case PG8:
            if (value) {
                GPIOG->ODR |= (0x01 << 8);
            } else {
                GPIOG->ODR &= ~(0x01 << 8);
            }
            break;
        case PG5:
            if (value) {
                GPIOG->ODR |= (0x01 << 5);
            } else {
                GPIOG->ODR &= ~(0x01 << 5);
            }
            break;
        case PG6:
            if (value) {
                GPIOG->ODR |= (0x01 << 6);
            } else {
                GPIOG->ODR &= ~(0x01 << 6);
            }
            break;
    }
}

/**
 * Writes the either high or low to each LED lightbar segment as determiend
 * by the bit sequence parsed into the function. i.e. LED Bar 0 will be ‘low’ 
 * if bit 0 of value is 0, LED B
 * 
 * @param unsigned short value is the LED light bar segment number
 */
void reg_lta1000g_write(unsigned short value) {
    for (int segment = 0; segment < 10; segment++) {
        // Shift the sequence by i and check the bit by &'ing it with 0x01
        unsigned short bit = (value >> segment) & 0x01;
        lta1000g_seg_set(segment, bit);
    }
}

/**
 * Performs a bitwise rotation(using only the 10 lower bits) and then write the 
 * LED Bar segments high or low, depending on the new value. i.e. if initial 
 * value is 0x289, new value is 0x344. The index will set the number of 
 * positions the value should rotated. e.g. index=1 will rotate the value 
 * by 1 position to the right. The minimum index is 0, maximum index is 8. 
 * If an index is outside of limits, no rotation takes place.
 * 
 * @param unsigned short value the bit sequence to be rotated
 * @param int index the number of rotations to take place
 * 
 * @returns unsigned short the new value after the bit rotation
 */
unsigned short reg_lta1000g_rrotate(unsigned short value, int index) {

    if (index < 0 || index > 8) {
        return value; // Index out of bounds
    }

    unsigned short wrapBit = 0; // The bit that wraps around

    unsigned short bitmask = 0b001111111111;
    unsigned short newValue = value & bitmask; // Only use lower 10 bits
    value &= 0b110000000000; // Only keep bits 11 & 12

    for (int shift = 0; shift < index; shift++) {
        // Check if the LSB is significant
        if (newValue & 0x0000000001) {
            wrapBit = 1; 
        } else {
            wrapBit = 0;
        }

        newValue >>= 1;
        newValue |= (wrapBit << 9);
    }

    reg_lta1000g_write(newValue);

    newValue |= value; // Keep bits 11 and 12 in tact

    return newValue;
}

/**
 * Controlling task for the lta1000g lightbar. Initialises and reads from a
 * a queue. A bitmask for length 8 should be put into the queue. This function
 * will read this bitmask and write the values to the lightbar.
 */
void ctrl_task_lta1000g() {
    unsigned long lta1000gBitMask;

    reg_lta1000g_init();

    lta1000gQueue = xQueueCreate (10, sizeof(lta1000gBitMask));

    for (;;) {
        if (lta1000gQueue != NULL) {	// Check if queue exists 
            // Send message to the front of the queue - wait atmost 10 ticks 
            if (xQueueReceive(lta1000gQueue, &lta1000gBitMask, 10)) {
                // Read from the queue and write bits to light bar
                reg_lta1000g_write(lta1000gBitMask);
            }
        }
        vTaskDelay(9);
    }
}

/** 
 * Initialiser task for the lta1000g light bar.
 */
extern void tsk_lta1000g_init(void) {
    xTaskCreate(&ctrl_task_lta1000g, "lta1000g_Task", LTA1000G_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
}
