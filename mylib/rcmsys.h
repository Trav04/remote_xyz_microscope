/***********************************************************************
 * @file mylib/rcmsys.h
 * @author Travis Graham
 * @date 25/08/2024
 * @brief RCM System Staus Header File
 **********************************************************************
 * EXTERNAL FUNCTIONS 
 **********************************************************************
 * void rcm_sys_init_tsk(void); - task initialiser
 **********************************************************************
 */

// Stack size for tasks
#define RCM_SYS_STACK_SIZE        ( configMINIMAL_STACK_SIZE * 2 )

/* OPCODE */
#define OPCODE_IDLE 0x01
#define OPCODE_X    0x02
#define OPCODE_Y    0x03
#define OPCODE_Z    0x04
#define OPCODE_ROT  0x05
#define OPCODE_ZOOM 0x06
#define OPCODE_NEW  0x07
#define OPCODE_DEL  0x08
#define OPCODE_SYS  0x09
#define OPCODE_ORG  0x0F

/* Global Variables */
extern int exeSignalFlag;
extern int cmdSignalFlag;
extern int joinSignalFlag;

void rcm_sys_init_tsk(void);