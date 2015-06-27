/*
 *  KM3NeT CLB v2 Firmware
 *  ----------------------
 *
 *  Copyright 2013 KM3NeT Collaboration
 *
 *  All Rights Reserved.
 *
 *
 *    File    : dev_spi.h
 *    Created : 8 mrt. 2013
 *    Author  : Vincent van Beveren
 */


#ifndef DEV_SPI_H_
#define DEV_SPI_H_

/**
 * @file
 *
 * @ingroup devices
 *
 * OpenCores SPI device.
 *
 * @attention that SPI modes 2 and 3 are not really supported in the strict sense. This could
 * result in issues with devices that do require this mode.
 */

#include "dev_defs.h"


#define SPI_CTL_CHAR_LEN_MASK       0x7F        ///< Number of bits to transfer in one go MASK
#define SPI_CTL_CHAR_LEN_SHIFT      0           ///< Number of bits to transfer in one go SHIFT
#define SPI_CTL_GO_BSY              BIT( 8)     ///< Start Transfer / Busy
#define SPI_CTL_RX_NEG              BIT( 9)     ///< If set latch on falling edge.
#define SPI_CTL_TX_NEG              BIT(10)     ///< If set changes on falling edge.

#define SPI_CTL_MODE_MASK   ( SPI_CTL_TX_NEG | SPI_CTL_RX_NEG)  ///< SPI mode mask.
#define SPI_CTL_MODE_0      ( SPI_CTL_TX_NEG )  ///< SPI mode 0 (CPOL = 0, CHPA = 0)
#define SPI_CTL_MODE_1      ( SPI_CTL_RX_NEG )  ///< SPI mode 1 (CPOL = 0, CHPA = 1)
#define SPI_CTL_MODE_2      ( SPI_CTL_RX_NEG )  ///< SPI mode 2 (CPOL = 1, CHPA = 0) (See note)
#define SPI_CTL_MODE_3      ( SPI_CTL_TX_NEG )  ///< SPI mode 3 (CPOL = 1, CHPA = 1) (See node)

#define SPI_CTL_LSB                 BIT(11)     ///< Least Significant Bit first (opposed to MSB)
#define SPI_CTL_IE                  BIT(12)     ///< Interrupt enable
#define SPI_CTL_ASS                 BIT(13)     ///< Auto slave select

#define SPI_DIV_MASK        0xFFFF


/**
 * Structure defines OpenCores I2C Device.
 */

typedef _reg_a 	SPI_Device;

#define SPI_DEVICE_RX0(a)	(a + 0x00)
#define SPI_DEVICE_RX1(a)	(a + 0x04)
#define SPI_DEVICE_RX2(a)	(a + 0x08)
#define SPI_DEVICE_RX3(a)	(a + 0x0c)

#define SPI_DEVICE_TX0(a)	(a + 0x00)
#define SPI_DEVICE_TX1(a)	(a + 0x04)
#define SPI_DEVICE_TX2(a)	(a + 0x08)
#define SPI_DEVICE_TX3(a)	(a + 0x0c)

#define SPI_DEVICE_CTL(a)	(a + 0x10)
#define SPI_DEVICE_DIVIDER(a)	(a + 0x14)
#define SPI_DEVICE_SS(a)        (a + 0x18)

#endif /* DEV_SPI_H_ */
