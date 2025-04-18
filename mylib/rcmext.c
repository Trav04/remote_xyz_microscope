/** 
 **************************************************************
 * @file mylib/rcmext.c
 * @author Travis Graham
 * @date 26/07/2024
 * @brief Extra commands mylib driver
 ***************************************************************
 * EXTERNAL FUNCTIONS 
 ***************************************************************
 * extern void tsk_rcmext_init(void) - initialiser
 *************************************************************** 
 */
#include "board.h"
#include "processor_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "string.h"
#include "debug_log.h"
#include "txradio.h"
#include "rcmext.h"

EventGroupHandle_t rcmExtCmdsEventGroup;

/**
 * Controlling task.
 */
void ctrl_task_rcmext() {
    rcmExtCmdsEventGroup = xEventGroupCreate();

    EventBits_t uxBits;


    for (;;) {
        uxBits = xEventGroupWaitBits(
            rcmExtCmdsEventGroup, 
            RCMEXT_NEW | RCMEXT_DEL | RCMEXT_SYS, 
            pdTRUE, 
            pdFALSE, 
            10
        );

        if ((uxBits & RCMEXT_NEW) != 0) {
            // Create a new task and override the handle
            xTaskCreate(&txradio_ctrl_tsk, "txRadio Control Task", RADIO_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &txRadioTaskHandle);

        }

        if ((uxBits & RCMEXT_DEL) != 0) {
            // Check that a txRadio task exists to be deleted
            if( txRadioTaskHandle != NULL ) {
                // Use the handle to delete the task.
                vTaskDelete( txRadioTaskHandle ); 
                // Reset the xHandle
                txRadioTaskHandle = NULL;
            }
        }

        if ((uxBits & RCMEXT_SYS) != 0) {
            TaskStatus_t *pxTaskStatusArray;
            volatile UBaseType_t uxArraySize, x;
            unsigned long ulTotalRunTime, ulStatsAsPercentage;


            /* Take a snapshot of the number of tasks in case it changes while this
            function is executing. */
            uxArraySize = uxTaskGetNumberOfTasks();


            /* Allocate a TaskStatus_t structure for each task. An array could be
            allocated statically at compile time. */
            pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

                /* Generate raw status information about each task. */
                uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                        uxArraySize,
                                        &ulTotalRunTime );

                /* For each populated position in the pxTaskStatusArray array,
                format the raw data as human readable ASCII data. */
                for ( x = 0; x < uxArraySize; x++ ) {
                        debug_log("Task Name: %s\n\r"
                                    "Task ID: %u\n\r"
                                    "Task State: %d\n\r"
                                    "Task Priority: %u\n\r"
                                    "Task Memory Usage: %u\n\r",
                                    pxTaskStatusArray[x].pcTaskName,
                                    pxTaskStatusArray[x].xTaskNumber,
                                    pxTaskStatusArray[x].eCurrentState,
                                    pxTaskStatusArray[x].uxCurrentPriority,
                                    pxTaskStatusArray[x].usStackHighWaterMark);
                        debug_log("\r------------------------\n\r");
                }
        
        }
        vTaskDelay(10);
    }
}

/**
 * Hardware initialiser task. Creates a task and parses in the controlling task
 * function. Does not handle register intit directly.
 */
extern void tsk_rcmext_init(void) {
    // Create the task for the controlling task function
    xTaskCreate(&ctrl_task_rcmext, "RCMEXT_Task", RCMEXT_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
}