/** 
 **************************************************************
 * @file mylib/rcmext.h
 * @author Travis Graham
 * @date 15/10/2024
 * @brief header file for the rcm extra commands mylib driver
 ***************************************************************
  * EXTERNAL FUNCTIONS 
 ***************************************************************
 * extern void tsk_rcmext_init(void) - initialiser
 *************************************************************** 
 */
#ifndef RCMEXT_H
#define RCMEXT_H

#define RCMEXT_STACK_SIZE        ( configMINIMAL_STACK_SIZE * 10 )

/* Event Group Bit States */
#define RCMEXT_NEW 1 << 0
#define RCMEXT_DEL 1 << 1
#define RCMEXT_SYS 1 << 2

/* Global Variables */
extern EventGroupHandle_t rcmExtCmdsEventGroup;


/* Function Prototypes */
void tsk_rcmext_init(void);

#endif // RCMEXT_H