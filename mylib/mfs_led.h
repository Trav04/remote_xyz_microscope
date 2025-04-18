/***********************************************************************
 * @file mylib/led.h
 * @author Travis Graham
 * @date 25/08/2024
 * @brief MFS LED driver header file
 **********************************************************************
 * EXTERNAL FUNCTIONS 
 **********************************************************************
 * extern void mfs_led_init_tsk(void *pvParameters)
 *      - Initialises the MFS led controlling task
 **********************************************************************
 */

#ifndef MFS_LED_H
#define MFS_LED_H

/* PORTA MFS LEDs */
#define PIN_LED_D1 5
#define PIN_LED_D2 6
#define PIN_LED_D3 7

/* PORTD MFS LEDs*/
#define PIN_LED_D4 14

// Constants to turn MFS LEDs ON or OFF
#define REG_MFS_LED_D1_OFF      (GPIOA->ODR |= (0x01 << PIN_LED_D1))
#define REG_MFS_LED_D1_ON     (GPIOA->ODR &= ~(0x01 << PIN_LED_D1))
#define REG_MFS_LED_D2_OFF      (GPIOA->ODR |= (0x01 << PIN_LED_D2))
#define REG_MFS_LED_D2_ON     (GPIOA->ODR &= ~(0x01 << PIN_LED_D2))
#define REG_MFS_LED_D3_OFF      (GPIOA->ODR |= (0x01 << PIN_LED_D3))
#define REG_MFS_LED_D3_ON     (GPIOA->ODR &= ~(0x01 << PIN_LED_D3))
#define REG_MFS_LED_D4_OFF      (GPIOD->ODR |= (0x01 << PIN_LED_D4))
#define REG_MFS_LED_D4_ON     (GPIOD->ODR &= ~(0x01 << PIN_LED_D4))

#define MFS_LED_STACK_SIZE        ( configMINIMAL_STACK_SIZE * 2 )

/* Globals */
extern QueueHandle_t MfsLedStatesQueue;

/* Function prototypes */
void mfs_led_init_tsk();

#endif // MFS_LED_H