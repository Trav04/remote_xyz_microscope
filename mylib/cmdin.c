/**
 * **************************************************************
 * @file mylib/cmdin.c
 * @author Travis Graham
 * @date 14/08/2024
 * @brief mylib driver file for the command input tasks
 ***************************************************************
 * EXTERNAL FUNCTIONS
 ***************************************************************
 * void tsk_cmdin_init(void) - initialiser function
 ***************************************************************
 */

#include "board.h"
#include "processor_hal.h"
#include "FreeRTOS.h"
#include "math.h"
#include "event_groups.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "nrf24l01plus.h"
#include "switchbank.h"
#include "txradio.h"
#include "hamming.h"
#include "mfs_pb.h"
#include "rcmsys.h"
#include "rgb.h"
#include "ssd.h"
#include "rcmext.h"
#include "cmdin.h"


extern uint8_t packet[32];
extern QueueHandle_t RadioPacketsQueue;

// #define DEBUG // Uncomment to turn on debug mode
#ifdef DEBUG
    #include "debug_log.h"
#endif

/* XYZ Values */
int xVal;
int yVal;
int zVal;

/* Zoom value */
int zoomVal = 1; // Zoom is relative and initially 1

/* Rotation value */
int rotVal;

/* Current FSM State */
int currentState;


/**
 * Updates the radio packet global variable to represent a JOIN packet.
 * Hamming encodes the packet and then adds the packet to the queue for
 * transmission.
 */
void rcm_send_join_packet(void) {
    // Modify the packet global variable and send
    uint8_t pckt_join[16] = {
        PCKT_TYPE_JOIN,
        SENDER_ADDR[0],
        SENDER_ADDR[1],
        SENDER_ADDR[2],
        SENDER_ADDR[3],
        PCKT_STR_JOIN[0],
        PCKT_STR_JOIN[1],
        PCKT_STR_JOIN[2],
        PCKT_STR_JOIN[3],
        0x00, // <Zero padding>
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
    };

    hamming_encode_packet(pckt_join); // Encode the global packet

    if (RadioPacketsQueue != NULL) {
        xQueueSendToFront(
            RadioPacketsQueue, (void *) &packet, (portTickType) 10);
    }
}

/**
 * Updates the radio packet global variable to represent a XYZ packet.
 * Hamming encodes the packet and then adds the packet to the queue for
 * transmission.
 */
void rcm_send_xyz_packet(int x, int y, int z) {
    uint8_t pckt_xyz[16] = {
        PCKT_TYPE_XYZ,
        SENDER_ADDR[0],
        SENDER_ADDR[1],
        SENDER_ADDR[2],
        SENDER_ADDR[3],
        PCKT_STR_XYZ[0],
        PCKT_STR_XYZ[1],
        PCKT_STR_XYZ[2],
        (int) (floor(x / 100)) % 10 + '0', // <3 digit X>  // TODO Isolate each integer properly
        (int) (floor(x / 10)) % 10 + '0',
        (int) (floor(x / 1)) % 10 + '0',
        (int) (floor(y / 100)) % 10 + '0', // <3 digit Y>
        (int) (floor(y / 10)) % 10 + '0',
        (int) (floor(y / 1)) % 10 + '0',
        (int) (floor(z / 10)) % 10 + '0', // <2 digit Z>
        (int) (floor(z / 1)) % 10 + '0'
    };

    hamming_encode_packet(pckt_xyz); // Encode the global packet

    if (RadioPacketsQueue != NULL) {
        xQueueSendToFront(
            RadioPacketsQueue, (void *) &packet, (portTickType) 10);
    }

}

/**
 * Updates the radio packet global variable to represent a ZOOM packet.
 * Hamming encodes the packet and then adds the packet to the queue for
 * transmission.
 */
void rcm_send_zoom_packet(int zoomFactor) {
    // Modify the packet global variable and send
    uint8_t pckt_zoom[16] = {
        PCKT_TYPE_ZOOM,
        SENDER_ADDR[0],
        SENDER_ADDR[1],
        SENDER_ADDR[2],
        SENDER_ADDR[3],
        PCKT_STR_ZOOM[0],
        PCKT_STR_ZOOM[1],
        PCKT_STR_ZOOM[2],
        PCKT_STR_ZOOM[3],
        zoomFactor + '0',   // <1 digit zoom factor>
        0x00,   // <0 padding>
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
    };

    hamming_encode_packet(pckt_zoom); // Encode the global packet

    if (RadioPacketsQueue != NULL) {
        xQueueSendToFront(
            RadioPacketsQueue, (void *) &packet, (portTickType) 10);
    }
}

/**
 * Updates the radio packet global variable to represent a ROT packet.
 * Hamming encodes the packet and then adds the packet to the queue for
 * transmission.
 */
void rcm_send_rot_packet(int angle) {
    // Modify the packet global variable and send
    uint8_t pckt_rot[16] = {
        PCKT_TYPE_ROT,
        SENDER_ADDR[0],
        SENDER_ADDR[1],
        SENDER_ADDR[2],
        SENDER_ADDR[3],
        PCKT_STR_ROT[0],
        PCKT_STR_ROT[1],
        PCKT_STR_ROT[2],
        (int) (floor(angle / 100)) % 10 + '0', // <3 digit angle>
        (int) (floor(angle / 10)) % 10 + '0',
        (int) (floor(angle / 1)) % 10 + '0',
        0x00, // <0 padding>
        0x00,
        0x00,
        0x00,
        0x00
    };
    hamming_encode_packet(pckt_rot); // Encode the global packet

    if (RadioPacketsQueue != NULL) {
        xQueueSendToFront(
            RadioPacketsQueue, (void *) &packet, (portTickType) 10);
    }
}

/**
 * Hamming encodes an unecoded 16byte packet and updates the 32byte packet
 * ready for transmission.
 *
 * @param packet the 16byte packet to be encoded
 */
void hamming_encode_packet(uint8_t unencodedPacket[16]) {
    for (int i = 0; i < 32; i++) {
        packet[i] = 0;  // Assigning 0 to each element
    }

    uint8_t firstHammingByte;
    uint8_t secondHammingByte;
    for (int i = 0; i < 16; i++) {
        uint16_t encodedBytes = lib_hamming_byte_encode(unencodedPacket[i]);

        firstHammingByte = encodedBytes & 0xFF;
        secondHammingByte = (encodedBytes >> 8) & 0xFF;

        packet[i * 2] = firstHammingByte;
        packet[i * 2 + 1] = secondHammingByte;
    }
}

/**
 * Update the RGB module colour.
 * 
 * @parm int currentState the current state of the FSM.
 * Sets the colour of the RGB module
 */
void set_rgb_colour(int currentState) {
    uint8_t rgbColourMask;
    switch (currentState) {
        case S_INIT:
            rgbColourMask = BLACK;
            break;

        case S_IDLE:
            rgbColourMask = GREEN;
            break;

        case S_X:
            rgbColourMask = BLUE;
            break;

        case S_Y:
            rgbColourMask = YELLOW;
            break;

        case S_Z:
            rgbColourMask = RED;
            break;

        case S_ROT:
            rgbColourMask = MAGENTA;
            break;

        case S_ZOOM:
            rgbColourMask = CYAN;
            break;

        case S_NEW:
            rgbColourMask = WHITE;
            break;

        case S_DEL:
            rgbColourMask = WHITE;
            break;

        case S_SYS:
            rgbColourMask = WHITE;
            break;

        case S_ORG:
            rgbColourMask = BLACK;
            break;
    }
            // Update the RGB LED with current state colour
    if (RgbStateQueue != NULL) {
        #ifdef DEBUG
            debug_log("CMDIN RGBMASK: %x\n\r", rgbColourMask);
        #endif
        xQueueSendToFront(
            RgbStateQueue, (void *) &rgbColourMask, (portTickType) 10);
    }
}

/** 
 * Processes the current state of the FSM and returns the next state.
 * If execute is HIGH, then execute the state function.
 */
int fsm_processing(int currentState, int execute) {
        int nextState = S_IDLE;
        unsigned long transmissionData;
        int rgbColourMask;
        int relativeZoomVal;
        int relativeRotVal;


        // FSM to process the current state 
        // and find the next state
        switch (currentState) {
            case S_INIT:
                nextState = S_IDLE;
                break;
                
            case S_IDLE:
                if (execute) {
                    nextState = S_IDLE;
                } else {
                    nextState = S_X;
                }
                break;

            case S_X:
                if (SwitchBankDataQueue != NULL) {
                    xQueueReceive(SwitchBankDataQueue, &transmissionData, 10);
                    
                }

                if (execute) { // Send the packet
                    xVal = transmissionData;
                    rcm_send_xyz_packet(xVal, yVal, zVal);
                    if (SevenSegQueue != NULL) {
                        xQueueSendToFront(SevenSegQueue, &xVal, 10);
                    }
                    nextState = S_X;
                } else {
                    nextState = S_Y;

                }
                break;

            case S_Y:
                xQueueReceive(SwitchBankDataQueue, &transmissionData, 10);

                if (execute) {
                    yVal = transmissionData;
                    rcm_send_xyz_packet(xVal, yVal, zVal);
                    if (SevenSegQueue != NULL) {
                        xQueueSendToFront(SevenSegQueue, &yVal, 10);
                    }
                    nextState = S_Y;
                } else {
                    nextState = S_Z;
                }
                break;

            case S_Z:
                xQueueReceive(SwitchBankDataQueue, &transmissionData, 10);

                if (execute) {
                    zVal = transmissionData % 100; // Only last two digits for Z
                    rcm_send_xyz_packet(xVal, yVal, zVal);
                    if (SevenSegQueue != NULL) {
                        xQueueSendToFront(SevenSegQueue, &zVal, 10);
                    }
                    nextState = S_Z;
                } else {
                    nextState = S_ROT;
                }
                break;

            case S_ROT:
                xQueueReceive(SwitchBankDataQueue, &transmissionData, 10);
                if (execute) {
                    relativeRotVal = (int) (transmissionData / 10 ) * 10 ;
                    rotVal = (rotVal + relativeRotVal) % 190;

                    rcm_send_rot_packet(rotVal);
                    if (SevenSegQueue != NULL) {
                        xQueueSendToFront(SevenSegQueue, &rotVal, 10);
                    }
                    nextState = S_ROT;
                } else {
                    nextState = S_ZOOM;
                }
                break;

            case S_ZOOM:
                xQueueReceive(SwitchBankDataQueue, &transmissionData, 10);
                if (execute) {
                    relativeZoomVal = transmissionData % 10; 
                    zoomVal = zoomVal + relativeZoomVal;
                    if (zoomVal > 9) {
                        zoomVal = 9;
                    }
                    rcm_send_zoom_packet(zoomVal);
                    if (SevenSegQueue != NULL) {
                        xQueueSendToFront(SevenSegQueue, &zoomVal, 10);
                    }
                    nextState = S_ZOOM;
                } else {
                    nextState = S_NEW;
                }
                break;

            case S_NEW:
                if (execute) {
                    EventBits_t uxBits;
                    uxBits = xEventGroupSetBits(rcmExtCmdsEventGroup, RCMEXT_NEW);
                    nextState = S_NEW;

                }
                else { 
                    nextState = S_DEL;
                }
                break;

            case S_DEL:
                if (execute) {
                    EventBits_t uxBits;
                    uxBits = xEventGroupSetBits(rcmExtCmdsEventGroup, RCMEXT_DEL);
                    nextState = S_DEL;

                }
                else { 
                    nextState = S_SYS;
                }
                break;

            case S_SYS:
                if (execute) {
                    EventBits_t uxBits;
                    uxBits = xEventGroupSetBits(rcmExtCmdsEventGroup, RCMEXT_SYS);
                    nextState = S_SYS;

                }
                else { 
                    nextState = S_ORG;
                }
                break;

            case S_ORG:

                rgbColourMask = BLACK;

                if (execute) {
                    xVal = 0;
                    yVal = 0;
                    zVal = 0;
                    rcm_send_xyz_packet(xVal, yVal, zVal); // Move back to the Origin
                    if (SevenSegQueue != NULL) {
                        xQueueSendToFront(SevenSegQueue, &xVal, 10);
                    } 
                    nextState = S_ORG;
                } else {
                    nextState = S_IDLE;
                }
                break;
        }

        return nextState;
}

/**
 * Controlling task for command inputs.
 */
void s4747355_ctrl_task_cmdin() {
    int prevCmdSignalFlag = -1;
    int prevExeSignalFlag = 0;
    int prevJoinSignalFlag = 0;

    currentState = S_INIT;  // Initalise state first

    for (;;) {
        // Await JOIN Signal to send JOIN packet
        if (joinSignalFlag != prevJoinSignalFlag) {
            BRD_LEDGreenOn();
            #ifdef DEBUG
                debug_log("Joining...\n\r");
            #endif
            rcm_send_join_packet();
            prevJoinSignalFlag = joinSignalFlag;
        }

        if (prevExeSignalFlag != exeSignalFlag) {
            BRD_LEDBlueOn();
            currentState = fsm_processing(currentState, 1); // Update the state
            prevExeSignalFlag = exeSignalFlag;
            set_rgb_colour(currentState);

            #ifdef DEBUG
                    debug_log("\n\nCMD EXECUTED!\n\n\rCurrent State: %d\n\r", currentState);
                    debug_log("xVal: %d, yVal: %d, zVal: %d, rotVal: %d, zoomVal: %d\n\r", xVal, yVal, zVal, rotVal, zoomVal);
            #endif

        } else {
            // Await CMD Signal to toggle states
            if (cmdSignalFlag != prevCmdSignalFlag) { 
                BRD_LEDRedOn();
                currentState = fsm_processing(currentState, 0); // Update the state
                prevCmdSignalFlag = cmdSignalFlag;
                set_rgb_colour(currentState);

                #ifdef DEBUG
                    debug_log("Current State: %d\n\r", currentState);
                    debug_log("xVal: %d, yVal: %d, zVal: %d, rotVal: %d, zoomVal: %d\n\r", xVal, yVal, zVal, rotVal, zoomVal);
                #endif
            }

        }

        vTaskDelay(3);
    }
}

/**
 * Initialiser task for the controlling task for command inputs.
 */
extern void tsk_cmdin_init(void)
{
    // Create the controlling task
    xTaskCreate(&s4747355_ctrl_task_cmdin, "CMDIN Control Task", CMD_IN_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
}