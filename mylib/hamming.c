 /** 
 *********************************************************************
 * @file mylib/hamming.c
 * @author Travis Graham
 * @date 25/08/2024
 * @brief Hamming encoding driver
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
#include "hamming.h"
#include "board.h"
#include "processor_hal.h"

// #define DEBUG // Uncomment to turn on debug mode
#ifdef DEBUG
    #include "debug_log.h"
#endif

/* Global variable to track if a hamming correctio nis made*/
int hammingCorrected = 0;

/* Global to track if the parity bit was flipped */
int parityError;

/**
 * Return the 16bit encoded value of a byte.
 * 
 * @param unsigned char value the char value to be encoded
 * 
 * @return unsigned short, the hamming encoded 16 bytes 
 */
unsigned short lib_hamming_byte_encode(unsigned char value) {
    uint16_t out;

    /* first encode D0..D3 (first 4 bits),
     * then D4..D7 (second 4 bits).
     */
    out = hamming_hbyte_encode(value & 0xF) |
        (hamming_hbyte_encode(value >> 4) << 8);

    return(out);
}

/**
 * Internal function
 * Implement Hamming Code + parity checking
 * Hamming code is based on the following generator and parity check matrices
 * G = [ 0 1 1 | 1 0 0 0 ;
 *       1 0 1 | 0 1 0 0 ;
 *       1 1 0 | 0 0 1 0 ;
 *       1 1 1 | 0 0 0 1 ;
 *
 * hence H =
 * [ 1 0 0 | 0 1 1 1 ;
 *   0 1 0 | 1 0 1 1 ;
 *   0 0 1 | 1 1 0 1 ];
 *
 * y = x * G, syn = H * y'
 * 
 * @param unsigned char value the half byte to be encoded
 */
unsigned char hamming_hbyte_encode(unsigned char value) {
    uint8_t d0, d1, d2, d3;
    uint8_t p0 = 0, h0, h1, h2;
    uint8_t z;
    uint8_t out;

    // extract bits
    // convert non-zero bits to 1
    d0 = !!(value & (1 << 0));
    d1 = !!(value & (1 << 1));
    d2 = !!(value & (1 << 2));
    d3 = !!(value & (1 << 3));

    /* calculate hamming parity bits */
    h0 = d1 ^ d2 ^ d3;
    h1 = d0 ^ d2 ^ d3;
    h2 = d0 ^ d1 ^ d3;

    /* generate out byte without parity bit P0 */
    out = (h0 << 1) | (h1 << 2) | (h2 << 3) |
        (d0 << 4) | (d1 << 5) | (d2 << 6) | (d3 << 7);

    /* calculate even parity bit */
    for (z = 1; z < 8; z++)
        p0 = p0 ^ !!(out & (1 << z));

    out |= p0;

    return(out);
}

/**
 * Return the decoded half byte from a byte. Errors must be detected 
 * and corrected. Sets flag if correction was made.
 * 
 * @param unsigned char value - the hamming byte to be decoded
 * 
 * @returns unsigned char value - the half byte decoded from the input byte
 */
unsigned char lib_hamming_byte_decode(unsigned char value) {
    uint8_t b0, b1, b2, b3, b4, b5, b6, b7;
    uint8_t p0 = 0, h0, h1, h2;
    uint8_t s0, s1, s2;
    uint8_t z;
    uint8_t out;
    uint8_t syndrome;
    uint8_t decoded_value;

    // Extract all bits
    // Convert all non-zero bits to 1
    p0 = !!(value & (1 << 0)); // Parity bit
    b0 = !!(value & (1 << 1)); // Hamming bit 0
    b1 = !!(value & (1 << 2)); // Hamming bit 1
    b2 = !!(value & (1 << 3)); // Hamming bit 2
    b3 = !!(value & (1 << 4));
    b4 = !!(value & (1 << 5));
    b5 = !!(value & (1 << 6));
    b6 = !!(value & (1 << 7));

    // Calculated syndrome bits (matrix mulitiplication % 2)
    s0 = b0 ^ b4 ^ b5 ^ b6;
    s1 = b1 ^ b3 ^ b5 ^ b6;
    s2 = b2 ^ b3 ^ b4 ^ b6;

    syndrome = (s0 << 2) | (s1 << 1) | (s2 << 0); // Calc syndrom 3 bits

    #ifdef DEBUG
        debug_log("\n\nSyndrome Bits %d, %d, %d \n\r", s0, s1, s2);
        debug_log("Syndrome Hex %X\n\n\r", syndrome);
    #endif
    if (syndrome == 0) {
        decoded_value = (b3 << 0) | (b4 << 1) | (b5 << 2) | (b6 << 3);
        return decoded_value;
    } else {
        // Set flag that syndrome was non-zero
        hammingCorrected = 1;
        // Correct the error by flipping bit
        // Syndrome is representative of the bit error position
        value ^= (1 << (syndrome));
        // Redefine the data bits after correcting the error 
        p0 = !!(value & (1 << 0)); // Parity bit
        b0 = !!(value & (1 << 1)); // Hamming bit 0
        b1 = !!(value & (1 << 2)); // Hamming bit 1
        b2 = !!(value & (1 << 3)); // Hamming bit 2
        b3 = !!(value & (1 << 4));
        b4 = !!(value & (1 << 5));
        b5 = !!(value & (1 << 6));
        b6 = !!(value & (1 << 7));

        decoded_value = (b3 << 0) | (b4 << 1) | (b5 << 2) | (b6 << 3);
        return decoded_value;
    }
}

/**
 * Checks the 8 bit hamming byte for a parity error. Uses even parity. 
 * 
 * @param unsigned char value - the 8 bit hamming byte to be checked 
 *                              for a parity error
 * 
 * @return 1 if a parity error has occurred, else 0.
 */
int lib_hamming_parity_error(unsigned char value) {
    int p0 = 0;
    /* calculate even parity bit */
    for (int z = 1; z < 8; z++)
        p0 = p0 ^ !!(value & (1 << z));

    if (p0 == (value & 1)){
        return 0;
    } 
    return 1;
}
