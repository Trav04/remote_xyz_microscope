/**
 * **************************************************************
 * @file mylib/rgb.h
 * @author Travis Graham
 * @date 14/08/2024
 * @brief header file for rgb module
 ***************************************************************
  * EXTERNAL FUNCTIONS 
 ***************************************************************
 * extern void tsk_rgb_init(void) - initialiser
 *************************************************************** 
 */
#ifndef RGB_H
#define RGB_H

#define TIMER_COUNTER_FREQ  				10000          				                            // Frequency (in Hz)
#define PWM_PULSE_WIDTH_SECONDS				0.005												// Max pulse width of 5ms
#define PWM_PULSE_WIDTH_TICKS				200 //2 * ((PWM_PULSE_WIDTH_SECONDS) / (1.0 / TIMER_COUNTER_FREQ))	// Period in timer ticks
#define PWM_PERCENT2TICKS_DUTYCYCLE(value)	(uint16_t)((value) * PWM_PULSE_WIDTH_TICKS / 100)		// Duty Cycle on time in timer ticks
#define PWM_DUTYCYCLE_CHANGE(value) 		TIM1->CCR1 = (uint16_t)(value)

// Brightness percentage bitmasks
#define BRIGHTNESS_0_PERCENT 0x0000
#define BRIGHTNESS_10_PERCENT 0x0001
#define BRIGHTNESS_20_PERCENT 0x0002
#define BRIGHTNESS_30_PERCENT 0x0004
#define BRIGHTNESS_40_PERCENT 0x0008
#define BRIGHTNESS_50_PERCENT 0x0010
#define BRIGHTNESS_60_PERCENT 0x0020
#define BRIGHTNESS_70_PERCENT 0x0040
#define BRIGHTNESS_80_PERCENT 0x0080
#define BRIGHTNESS_90_PERCENT 0x0100
#define BRIGHTNESS_100_PERCENT 0x0200

// RGB Colour bitmasks
#define RED 0x04
#define GREEN 0x02
#define BLUE 0x01
#define CYAN 0x03
#define MAGENTA 0x05
#define YELLOW 0x06
#define WHITE 0x07
#define BLACK 0x00

// Number of colours
#define NUM_COLOURS 8

// RGB Module Pinouts
#define RGB_RED 11
#define RGB_BLUE 15
#define RGB_GREEN 14

/* FreeRTOS Specifics */
#define RGB_STACK_SIZE        ( configMINIMAL_STACK_SIZE * 10 )


/* Globals */
extern QueueHandle_t RgbStateQueue;

/* Function Prototypes*/
void tsk_rgb_init();
void reg_rgb_init();
void reg_rgb_brightness_write(unsigned short level);
int reg_rgb_brightness_read();
void reg_rgb_colour_set(unsigned char rgb_mask);

#endif //RGB_H
