/** 
 *********************************************************************
 * @file mylib/txradio.h
 * @author Travis Graham
 * @date 25/09/2024
 * @brief RCM Radio Configuration File
 **********************************************************************
 * EXTERNAL FUNCTIONS 
 **********************************************************************
 * extern void init_tsk_txradio(void) - initialiser
 **********************************************************************
 */

#ifndef TXRADIO_H
#define TXRADIO_H

#define RADIO_STACK_SIZE        ( configMINIMAL_STACK_SIZE * 5 )

#define PCKT_TYPE_JOIN 0x20
#define PCKT_TYPE_XYZ 0x22
#define PCKT_TYPE_ROT 0x23
#define PCKT_TYPE_ZOOM 0x25

static const uint8_t PCKT_STR_ZOOM[4] = {0x5A, 0x4F, 0x4F, 0x4D};
static const uint8_t PCKT_STR_ROT[3] = {0x52, 0x4F, 0x54};
static const uint8_t PCKT_STR_XYZ[3] = {0x58, 0x59, 0x5A};
static const uint8_t PCKT_STR_JOIN[4] = {0x4A, 0x4F, 0x49, 0x4E}; 
static const uint8_t SENDER_ADDR[4] = {0x47, 0x43, 0x75, 0x53};

/* Globals */
extern TaskHandle_t txRadioTaskHandle;


void init_tsk_txradio(void);
void txradio_ctrl_tsk(void *pvParameters);

#endif // TXRADIO_H