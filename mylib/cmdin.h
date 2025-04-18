/**
 * **************************************************************
 * @file mylib/cmdin.h
 * @author Travis Graham
 * @date 26/07/2024
 * @brief cmdin library header file
 ***************************************************************
 * EXTERNAL FUNCTIONS 
 ***************************************************************
 * void tsk_cmdin_init(void) - initaliser task
 *************************************************************** 
 */

#include "board.h"
#include "processor_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "txradio.h"
#include "nrf24l01plus.h"

#define CMD_IN_STACK_SIZE        ( configMINIMAL_STACK_SIZE * 10 )

/* States */
#define S_INIT  0x00 // To send JOIN Packet before FSM Begins
#define S_IDLE  0x01
#define S_X     0x02
#define S_Y     0x03
#define S_Z     0x04
#define S_ROT   0x05
#define S_ZOOM  0x06
#define S_ORG   0x07
#define S_NEW   0x08
#define S_DEL   0x09
#define S_SYS   0x0A

// Join Flag
#define JOINED 1

/* Global Variables */
extern int currentState;


/* Function Prototypes */
void tsk_cmdin_init(void);
void hamming_encode_packet(uint8_t test[16]);
