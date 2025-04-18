/** 
 **************************************************************
 * @file mylib/mfs_trimpot.h
 * @author Travis Graham
 * @date 15/07/2024
 * @brief header file for mfs trimpot
 ***************************************************************
  * EXTERNAL FUNCTIONS 
 ***************************************************************
 * void reg_mfs_trimpot_init(); - initialise GPIO pins for trimpot
 * int reg_mfs_trimpot_get(); - get the trimpot ADC value
 *************************************************************** 
 */

#ifndef MFS_TRIMPOT_H
#define MFS_TRIMPOT_H


// Full revolution difference in the ADC value
#define ADC_FULL_REVOLUTION_DIFF 220

void reg_mfs_trimpot_init();
int reg_mfs_trimpot_get();

#endif //MFS_TRIMPOT_H
