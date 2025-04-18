/**
  ******************************************************************************
  * @file    repo/pf/main.c
  * @author  Travis Graham
  * @date    25/09/2024
  * @brief   Project main.c
  ******************************************************************************
  *
  */

#include "board.h"
#include "processor_hal.h"
#include "FreeRTOS.h"
#include "debug_log.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "mfs_led.h"
#include "mfs_pb.h"
#include "txradio.h"
#include "hamming.h"
#include "lta1000g.h"
#include "cmdin.h"
#include "rcmsys.h"
#include "switchbank.h"
#include "rgb.h"
#include "ssd.h"
#include "rcmext.h"

// Debugging
// #define DEBUG // Uncomment to turn on debug mode
#ifdef DEBUG
    #include "debug_log.h"
#endif

void hardware_init();

/* Function prototypes */
void Main_Task();

uint8_t nrf24l01plus_rr(uint8_t reg_addr);

/*
 * Starts all the other tasks, then starts the scheduler. Must not have a Cyclic Executive.
 */
int main( void ) {

    HAL_Init();     //Only HAL_Init() must be called before task creation.

    //
    //Only functions that create tasks must be called in main.
    //
    
    // Start the task.
    xTaskCreate(
        (void *) &Main_Task,
        (const signed char *) "Main Task",
        configMINIMAL_STACK_SIZE * 4,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL);

    /* Start the scheduler.

    NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
    The processor MUST be in supervisor mode when vTaskStartScheduler is
    called.  The demo applications included in the FreeRTOS.org download switch
    to supervisor mode prior to main being called.  If you are not using one of
    these demo application projects then ensure Supervisor mode is used here. */

    vTaskStartScheduler();

    return 0;
}

void Main_Task (void) {
    hardware_init();

    for (;;) {
        // pass
    }
}

/*
 * Hardware Initialisation - MUST be alled from a task and not in main.
 */
void hardware_init( void ) {

    taskENTER_CRITICAL();           //Disable interrupts
    #ifdef DEBUG
        BRD_LEDInit();              // Init Debug LEDS
        BRD_debuguart_init();       // Init Debug log
    #endif

    tsk_pb_init();         // Init Push buttons
    mfs_led_init_tsk();    // Init LEDs and Queue for bitmask
    tsk_lta1000g_init();   // Init LTA1000G Light Bar
    tsk_switchbank_init(); // Init Switchbank 
    tsk_rgb_init();        // Init RGB Module 
    rcm_sys_init_tsk();    // Init RCM System
    init_tsk_txradio();    // Init txRadio Transmitter (nrf24l01+)
    init_tsk_ssd();        // Init Seven Segment Display
    tsk_cmdin_init();      // Init Cmd In Driver
    tsk_rcmext_init();     // Init extra commands (Challenge Task)

    BRD_sysmon_init();              // Initialise System Monitor
    SYSMON_CHAN0_CLR();
    SYSMON_CHAN1_CLR();

    taskEXIT_CRITICAL();            //Enable interrupts

}
