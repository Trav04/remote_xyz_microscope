/** 
 **************************************************************
 * @file mylib/mfs_pb.c
 * @author Travis Graham
 * @date 26/07/2024
 * @brief mfs_pb driver
 ***************************************************************
 * EXTERNAL FUNCTIONS 
 ***************************************************************
 * extern void tsk_pb_init(void) - initialiser
 *************************************************************** 
 */

#include "board.h"
#include "processor_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "mfs_pb.h"


/* Press count initially zero */
static int MfPbPressCounter[3] = {0, 0, 0};

/* Time variable for button debouncing */
int previousTick = 0; 

/* Selected Push button*/
int selectedPb;


/* Semaphore for pushbutton 1 interrupt */
SemaphoreHandle_t pb1Semaphore;

/* Semaphore for pushbutton 3 interrupt */
SemaphoreHandle_t pb3Semaphore;

/* Semaphore for shield pushbutton interrupt*/
SemaphoreHandle_t boardPbSemaphore;

/* Control event group for push buttons*/
EventGroupHandle_t pushButtonEventGroup; 


void s474743755_reg_board_pb_init() {
    __GPIOC_CLK_ENABLE();

    GPIOC->OSPEEDR |= (GPIO_SPEED_FAST << 13);  //Set fast speed.
    GPIOC->PUPDR &= ~(0x03 << (13 * 2));        //Clear bits for no push/pull
    GPIOC->MODER &= ~(0x03 << (13 * 2));        //Clear bits for input mode

    // Enable EXTI clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    //select trigger source (port c, pin 13) on EXTICR4.
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

    EXTI->RTSR |= EXTI_RTSR_TR13;   //enable rising dedge
    EXTI->FTSR &= ~EXTI_FTSR_TR13;  //disable falling edge
    EXTI->IMR |= EXTI_IMR_IM13;     //Enable external interrupt

    //Enable priority (10) and interrupt callback. Do not set a priority lower than 5.
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * Initilises the pin interrupts for the push button selected.
 * 
 * @param pb_select an integer representing the selected push button 

 */
void reg_mfs_pb_init(int pb_select) {
    selectedPb = pb_select;
    // Enable EXTI clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    // Enable GPIO Clock
    __GPIOC_CLK_ENABLE();
    __GPIOF_CLK_ENABLE();

    switch (pb_select) {
        case REG_MFA_PB_S1:
            //select trigger source (port c, pin 0).
            SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
            SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC;

            EXTI->RTSR |= EXTI_RTSR_TR0; //enable rising dedge
            EXTI->FTSR &= ~EXTI_FTSR_TR0; //disable falling edge
            EXTI->IMR |= EXTI_IMR_IM0;  //Enable external interrupt

            GPIOC->OSPEEDR |= (GPIO_SPEED_FAST << 0); //Set fast speed.
            GPIOC->PUPDR &= ~(0x03 << (0 * 2)); //Clear bits for no push/pull
            GPIOC->MODER &= ~(0x03 << (0 * 2)); //Clear bits for input mode
            //Enable priority (10) and interrupt callback. Do not set a priority lower than 5.
            HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 0);
            HAL_NVIC_EnableIRQ(EXTI0_IRQn);
            break;
        case REG_MFA_PB_S2:
            //select trigger source (port c, pin 3).
            SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3;
            SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC;

            EXTI->RTSR |= EXTI_RTSR_TR3; //enable rising dedge
            EXTI->FTSR &= ~EXTI_FTSR_TR3; //disable falling edge
            EXTI->IMR |= EXTI_IMR_IM3;  //Enable external interrupt

            GPIOC->OSPEEDR |= (GPIO_SPEED_FAST << (3 * 2)); //Set fast speed.
            GPIOC->PUPDR &= ~(0x03 << (3 * 2)); //Clear bits for no push/pull
            GPIOC->MODER &= ~(0x03 << (3 * 2)); //Clear bits for input mode
            HAL_NVIC_SetPriority(EXTI3_IRQn, 10, 0);
            HAL_NVIC_EnableIRQ(EXTI3_IRQn);
            break;
        case REG_MFA_PB_S3:
            //select trigger source (port f, pin 3).
            SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3;
            SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PF;

            EXTI->RTSR |= EXTI_RTSR_TR3; //enable rising dedge
            EXTI->FTSR &= ~EXTI_FTSR_TR3; //disable falling edge
            EXTI->IMR |= EXTI_IMR_IM3;  //Enable external interrupt

            GPIOF->OSPEEDR |= (GPIO_SPEED_FAST << (3 * 2)); //Set fast speed.
            GPIOF->PUPDR &= ~(0x03 << (3 * 2)); //Clear bits for no push/pull
            GPIOF->MODER &= ~(0x03 << (3 * 2)); //Clear bits for input mode
            HAL_NVIC_SetPriority(EXTI3_IRQn, 10, 0);
            HAL_NVIC_EnableIRQ(EXTI3_IRQn);
            break;
    }
}

/**
 * Getter that returns the number of presses for a specified push button
 * @param int pb_select the push button
 * 
 * @returns int the number of presses for the chosen push button
 */
int reg_mfs_pb_press_get(int pb_select) {
    switch (pb_select) {
        case (REG_MFA_PB_S1):
            return MfPbPressCounter[0];
            break;
        case (REG_MFA_PB_S2):
            return MfPbPressCounter[1];
            break;
        case (REG_MFA_PB_S3):
            return MfPbPressCounter[2];
            break;
        }
}

/**
 * Reset the MFS pushbutton event counter value to 0, 
 * for the specified pushbutton
 * @param pb_select the push button
 */
void reg_mfs_pb_reset(int pb_select) {
    switch (pb_select) {
        case (REG_MFA_PB_S1):
            MfPbPressCounter[0] = 0;
            break;
        case (REG_MFA_PB_S2):
            MfPbPressCounter[1] = 0;
            break;
        case (REG_MFA_PB_S3):
            MfPbPressCounter[2] = 0;
            break;
        }
}

/**
 * Interrupt service routine for EXTI line 0
 */
void EXTI0_IRQHandler(void) {
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    EventBits_t uxBits;
    uint32_t currentTick = HAL_GetTick();

    // PR: Pending register
    if (((EXTI->PR & EXTI_PR_PR0) == EXTI_PR_PR0)) {

        // cleared by writing a 1 to this bit
        EXTI->PR |= EXTI_PR_PR0; //Clear interrupt flag.

        // PB S1 PC0
        if (GPIOC->IDR & (1 << 0)) {
            if ((currentTick - previousTick) >= 200 &&  GPIOC->IDR & (1 << 0)) {
                uxBits = xEventGroupSetBitsFromISR(pushButtonEventGroup, REG_MFA_PB_S1, &xHigherPriorityTaskWoken);
                previousTick = currentTick;
            }
        }
    }
}

/**
 * Interrupt service routine for EXTI line 3
 */
void EXTI3_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken;
    EventBits_t uxBits;
    uint32_t currentTick = HAL_GetTick();

    // debug_log("PB2 Interrupt!\n\r");
    NVIC_ClearPendingIRQ(EXTI3_IRQn);

    // PR: Pending register
    if (((EXTI->PR & EXTI_PR_PR3) == EXTI_PR_PR3)) {

        // cleared by writing a 1 to this bit
        EXTI->PR |= EXTI_PR_PR3; //Clear interrupt flag.

        if (GPIOF->IDR & (1 << 3)) {
            if ((currentTick - previousTick) >= DEBOUNCE_DELAY && GPIOF->IDR & (1 << 3)) {
                uxBits = xEventGroupSetBitsFromISR(pushButtonEventGroup, REG_MFA_PB_S3, &xHigherPriorityTaskWoken);
                previousTick = currentTick;
            }
        }
    }
}

/*
 * Interrupt handler (ISR) for EXTI 15 to 10 IRQ Handler
 * Note ISR should only execute a callback
 */ 
void EXTI15_10_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken;
    EventBits_t uxBits;
    uint32_t currentTick = HAL_GetTick();

    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);

    // PR: Pending register
    if ((EXTI->PR & EXTI_PR_PR13) == EXTI_PR_PR13) {

        // cleared by writing a 1 to this bit
        EXTI->PR |= EXTI_PR_PR13;   //Clear interrupt flag.
        if (GPIOC->IDR & (1 << 13)) {
            if ((currentTick - previousTick) >= 300 && GPIOC->IDR & (1 << 13)) {
                uxBits = xEventGroupSetBitsFromISR(pushButtonEventGroup, REG_BOARD_PB, &xHigherPriorityTaskWoken);
                previousTick = currentTick;
            }
        }
    }
}

/**
 * Controlling task. Initialises regiseters using init function and creates
 * semaphore for push button control.
 */
void ctrl_task_pb() {

    reg_mfs_pb_init(REG_MFA_PB_S1);
    reg_mfs_pb_init(REG_MFA_PB_S3);
    s474743755_reg_board_pb_init();

    pushButtonEventGroup = xEventGroupCreate();

    for (;;) {

        vTaskDelay(10);

    }
}

/**
 * Hardware initialiser task. Creates a task and parses in the controlling task
 * function. Does not handle register intit directly.
 */
extern void tsk_pb_init(void) {
    // Create the task for the controlling task function
    xTaskCreate(&ctrl_task_pb, "PB_Task", PB_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
}
