/**
 * *************************************************************
 * @file mylib/ssd.h
 * @author Travis Graham
 * @date 25/09/2024
 * @brief header file for the ssd module
 ***************************************************************
  * EXTERNAL FUNCTIONS 
 ***************************************************************
 * void ssd_display_number() - initialiser
 *************************************************************** 
 */
#ifndef SSD_H
#define SSD_H

#define SSD_STACK_SIZE        ( configMINIMAL_STACK_SIZE * 8 )

/* Pins */
#define PIN_SRCLK   13
#define PIN_RCLK   14
#define PIN_SER     12

/* Digit Definitions */
#define DIGIT_ZERO      0b11000000
#define DIGIT_ONE       0b11111001
#define DIGIT_TWO       0b10100100
#define DIGIT_THREE     0b10110000
#define DIGIT_FOUR      0b10011001
#define DIGIT_FIVE      0b10010010
#define DIGIT_SIX       0b10000010
#define DIGIT_SEVEN     0b11111000
#define DIGIT_EIGHT     0b10000000
#define DIGIT_NINE      0b10010000

/* Cathode Definitions */
#define NUM_CATHODES 4

/* Function Prototypes */
void init_tsk_ssd();

/* Globals */
extern QueueHandle_t SevenSegQueue;

#endif // SSD_H