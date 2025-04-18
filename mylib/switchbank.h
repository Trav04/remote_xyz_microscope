/** 
 *********************************************************************
 * @file mylib/switchbank.h
 * @author Travis Graham - 47437553
 * @date 25/08/2024
 * @brief Swtichbank driver header file
 **********************************************************************
 * EXTERNAL FUNCTIONS 
 **********************************************************************
 * int reg_switchbank_read() 
        - Reads the current values of the switchbank
 * void reg_switchbank_init()
        - Initialises the switchbank GPIO pins
 **********************************************************************
 */

#ifndef SWITCHBANK_H
#define SWITCHBANK_H

// Stack size for tasks
#define SB_STACK_SIZE        ( configMINIMAL_STACK_SIZE * 2 )

// Number of bits read from the switch bank
#define SWITCHBANK_LEN 8

// ASCII Symbols
#define SYM_PLUS 0x2B       // +
#define SYM_MINUS 0x2D      // -

/* Function prototypes */
void reg_switchbank_init();
int reg_switchbank_read();
void tsk_switchbank_init();

/* Globals */
extern QueueHandle_t SwitchBankDataQueue;

#endif // SWITCHBANK_H