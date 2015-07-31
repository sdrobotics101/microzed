/*
 *  KM3NeT CLB v2 Firmware
 *  ----------------------
 *
 *  Copyright 2013 KM3NeT Collaboration
 *
 *  All Rights Reserved.
 *
 *
 *    File    : utils.h
 *    Created : 22 feb. 2013
 *    Author  : Vincent van Beveren
 */


#ifndef MACRO_H_
#define MACRO_H_

/**
 * @file
 *
 * @ingroup util
 *
 * Provides common macros.
 */
#define _EVAL(MACRO)    MACRO

#define EVAL(MACRO)     _EVAL(MACRO)

#define _STR(EXPR)  #EXPR

/**
 * Stringyfies an expression.
 *
 * @param   EXPR    The expression to turn into a string
 */
#define STR(EXPR)   _STR(EXPR)



/// Private Cyclic redundancy macro, please don't use.
#define _CCMP(A, B, BITS) \
        (((A) - (B)) & (1 << (BITS - 1)))

/**
 * \brief Cyclic comparison function Greather Than.
 *
 * Evaluates if A > B, but only within bit bounds.
 *
 * @param   A       Value A
 * @param   B       Value B
 * @param   BITS    Bits to check.
 */
#define CGT(A, B, BITS) \
    (_CCMP(B, A, BITS) != 0)

/**
 * \brief Cyclic comparison function Greather Of Equal To.
 *
 * Evaluates if A >= B, but only within bit bounds.
 *
 * @param   A       Value A
 * @param   B       Value B
 * @param   BITS    Bits to check.
 */
#define CGE(A, B, BITS) \
    (_CCMP(A, B, BITS) == 0)

/**
 * \brief Cyclic comparison function Less Than.
 *
 * Evaluates if A < B, but only within bit bounds.
 *
 * @param   A       Value A
 * @param   B       Value B
 * @param   BITS    Bits to check.
 */
#define CLT(A, B, BITS) \
    (_CCMP(A, B, BITS) != 0)

/**
 * \brief Cyclic comparison function Less Or Equal To.
 *
 * Evaluates if A <= B, but only within bit bounds.
 *
 * @param   A       Value A
 * @param   B       Value B
 * @param   BITS    Bits to check.
 */
#define CLE(A, B, BITS) \
    (_CCMP(B, A, BITS) == 0)

/**
 * Makes a value with the specified bit set.
 */
#define BIT(N)              ( 1 << ( N ) )

/**
 * Creates a mask with the specified offset and length.
 */
#define MASK(LEN)           ( ( 1 < ( LEN ) ) - 1 )

/**
 * Extracts a bitfield
 */
#define EXT_BITFIELD(VAL, OFF, LEN) \
    ( ( ( VAL ) >> ( OFF ) ) & MASK( LEN ) )
#endif /* UTILS_H_ */
