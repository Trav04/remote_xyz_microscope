/**
 * **************************************************************
 * @file mylib/rcmsys.c
 * @author Travis Graham
 * @date 14/08/2024
 * @brief mylib driver for RCM system status indicators
 ***************************************************************
  * EXTERNAL FUNCTIONS 
 ***************************************************************
 * extern void rcm_sys_init_tsk(void) - initialiser
 *************************************************************** 
 */


#include "board.h"
#include "processor_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "rcmsys.h"
#include "mfs_pb.h"
#include "mfs_led.h"
#include "lta1000g.h"
#include "cmdin.h"

/* Command signal LED state */
uint8_t cmdSignalLed = 0;

/* Packet send signal LED state */
uint8_t packetSendLed = 0;

/* Command signal pushbutton flag */
int cmdSignalFlag = 0;

/* Packet send signal pushbutton flag */
int exeSignalFlag = 0;

/* Join signal pushbutton flag*/
int joinSignalFlag = 0;

/**
 * Given the current state of the FSM, return the corresponding
 * OPCODE.
 * 
 * @param int currentState represents the current state of the FSM
 * 
 * @returns int corresponding OPCODE
 */
int get_opcode(int currentState) {
    switch (currentState) {
        case S_IDLE:
            return OPCODE_IDLE;
        case S_X:
            return OPCODE_X;
        case S_Y:
            return OPCODE_Y;
        case S_Z:
            return OPCODE_Z;
        case S_ROT:
            return OPCODE_ROT;
        case S_ZOOM:
            return OPCODE_ZOOM;
        case S_NEW:
            return OPCODE_NEW;
        case S_DEL:
            return OPCODE_DEL;
        case S_SYS:
            return OPCODE_SYS;
        case S_ORG:
            return OPCODE_ORG;
    }
}

/**
 * Controlling task for the RCM system. Controls diagnostic outputs, i.e.
 * MFS LEDs and LTA1000G opcode display.
 */
void rcm_sys_ctrl_tsk(void *pvParameters) {

    int ledBitMask = 0;
    int lta1000gBitMask = 0;
    EventBits_t uxBits;

    for (;;) {

        uxBits = xEventGroupWaitBits(
            pushButtonEventGroup, 
            REG_MFA_PB_S3 | REG_MFA_PB_S1 | REG_BOARD_PB, 
            pdTRUE, 
            pdFALSE, 
            10
        );

        if ((uxBits & REG_MFA_PB_S3) != 0) {
                exeSignalFlag ^= 1; // Toggle the execution flag
                packetSendLed ^= 1; // Toggle D1 LED since packet was sent
                cmdSignalLed ^= 1; // Toggle D2 LED since pb was pressed
        }

        if ((uxBits & REG_MFA_PB_S1) != 0) {
                cmdSignalFlag ^= 1; // Toggle the flag
                cmdSignalLed ^= 1; // Toggle D1 LED since packet was sent
        }

        if ((uxBits & REG_BOARD_PB) != 0) {
                joinSignalFlag ^= 1; // Toggle join flag
                cmdSignalLed ^= 1; // Toggle D1 LED since packet was sent
        }


        // Display opcode on LTA1000g led light bar
        if (lta1000gQueue != NULL) {
            xQueueSendToFront(
                lta1000gQueue, (void *) &lta1000gBitMask, (portTickType) 1);
        }

        // Prepare LED bit mask
        ledBitMask = (packetSendLed << 0) | (cmdSignalFlag << 1);

        lta1000gBitMask = get_opcode(currentState);
        if (MfsLedStatesQueue != NULL) {
            xQueueSendToFront(
                MfsLedStatesQueue, (void *) &ledBitMask, (portTickType) 1);
        }

    }
}

/**
 * Hardware initialiser task. Creates a task and parses in the controlling task
 * function. Does not handle register intit directly.
 */
extern void rcm_sys_init_tsk(void) {
    xTaskCreate(&rcm_sys_ctrl_tsk, "RCM_SYS_Task", RCM_SYS_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
}
