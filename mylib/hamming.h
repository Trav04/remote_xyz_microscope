/** 
 *********************************************************************
 * @file mylib/hamming.h
 * @author Travis Graham
 * @date 25/08/2024
 * @brief Hamming encoding header file
 **********************************************************************
 * EXTERNAL FUNCTIONS 
 **********************************************************************
 * unsigned short lib_hamming_byte_encode(unsigned char value)
 *              - encodes a byte using hamming encoding
 * unsigned char lib_hamming_byte_decode(unsigned char value) 
 *              - decodes a byte using hamming encoding
 * int lib_hamming_parity_error(unsigned char value) 
 *              - checks for parity error
 **********************************************************************
 */

#ifndef HAMMING_H
#define HAMMING_H

/* Function prototypes */
unsigned short lib_hamming_byte_encode(unsigned char value);
unsigned char lib_hamming_byte_decode(unsigned char value);
int lib_hamming_parity_error(unsigned char value);
unsigned char hamming_hbyte_encode(unsigned char value);


#endif // sHAMMING_H