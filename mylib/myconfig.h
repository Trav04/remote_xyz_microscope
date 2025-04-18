/** 
 *********************************************************************
 * @file mylib/myconfig.h
 * @author Travis Graham
 * @date 25/08/2024
 * @brief RCM Radio Configuration File
 **********************************************************************
 */

// Radio Channel used for transmitting to a RCM
// #define MYRADIOCHAN  51
#define MYRADIOCHAN  56

/* Address used for trasmitting */
// static uint8_t default_addr[] = {0x12, 0x34, 0x56, 0x78, 0x90}; 
static uint8_t default_addr[] = {0x30, 0x10, 0x00, 0x00, 0x56}; 