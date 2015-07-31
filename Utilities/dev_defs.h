/*
 *  KM3NeT CLB v2 Firmware
 *  ----------------------
 *
 *  Copyright 2013 KM3NeT Collaboration
 *
 *  All Rights Reserved.
 *
 *
 *    File    : dev_defs.h
 *    Created : 15 feb. 2013
 *    Author  : Vincent van Beveren
 */


#ifndef DEV_DEFS_H_
#define DEV_DEFS_H_

/**
 * @file
 *
 * This module contains some very basic type definitions used for hardware mappings.
 *
 * @ingroup     devices
 */

#include "macro.h"

/** Basic register addr. */
#define     _reg_a      volatile uint32_t

/** Basic register type. */
#define     _reg_t      uint32_t

/** Input/Output register */
#define     reg_io      _reg_t

/** Write-only register */
#define     reg_o       _reg_t

/** Read-only register */
#define     reg_i       const _reg_t

#endif /* DEV_DEFS_H_ */
