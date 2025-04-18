/** 
 **************************************************************
 * @file mylib/lta1000g.h
 * @author Travis Graham
 * @date 26/07/2024
 * @brief lta1000g library header file
 ***************************************************************
 * EXTERNAL FUNCTIONS 
 ***************************************************************
 *extern void tsk_lta1000g_init(void) - initialiser
 *************************************************************** 
 */

#ifndef LTA1000G_H
#define LTA1000G_H

// Pin to lightbar segment mapping definitions
#define PG6 9
#define PG5 8
#define PG8 7
#define PE0 6
#define PF11 5
#define PG11 4
#define PG13 3
#define PG10 2
#define PG15 1
#define PE6 0

/* FreeRTOS Definitions*/
#define LTA1000G_STACK_SIZE        ( configMINIMAL_STACK_SIZE * 2 )

/* Globals */
extern QueueHandle_t lta1000gQueue;

/* Functin prototypes */
void tsk_lta1000g_init();


#endif // LTA1000G_H