/** 
 **************************************************************
 * @file mylib/mfs_pb.h
 * @author Travis Graham
 * @date 26/07/2024
 * @brief header file for mfs
 ***************************************************************
  * EXTERNAL FUNCTIONS 
 ***************************************************************
 * extern void tsk_pb_init(void) - initialiser
 *************************************************************** 
 */
#ifndef MFS_PB_H
#define MFS_PB_H

// Push buttons 1-3 on the MFS shield
#define REG_MFA_PB_S1  1 << 0 // Selects s1 pushbutton 
#define REG_MFA_PB_S2  1 << 1 // Selects s2 pushbutton 
#define REG_MFA_PB_S3  1 << 2 // Selects s3 pushbutton
#define REG_BOARD_PB   1 << 3 // Selects blue board pushbutton 

#define DEBOUNCE_DELAY 100

#define PB_STACK_SIZE        ( configMINIMAL_STACK_SIZE * 2 )

/* Global Variables */
extern SemaphoreHandle_t pb1Semaphore;
extern SemaphoreHandle_t pb3Semaphore;
extern SemaphoreHandle_t boardPbSemaphore;
extern EventGroupHandle_t pushButtonEventGroup;


/* Function Prototypes */
void reg_mfs_pb_init(int pb_select);
void reg_mfs_pb_reset(int pb_select);
int reg_mfs_pb_press_get(int pb_select);
void tsk_pb_init();

#endif // MFS_PB_H