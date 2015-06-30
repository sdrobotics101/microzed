/*
 *  KM3NeT CLB v2 Firmware
 *  ----------------------
 *
 *  Copyright 2013 KM3NeT Collaboration
 *
 *  All Rights Reserved.
 *
 *
 *    File    : spi.h
 *    Created : 22 mrt. 2013
 *    Author  : Vincent van Beveren
 */


#ifndef SPI_H_
#define SPI_H_

/**
 * @file
 *
 * This driver wraps the functions of the OpenCores SPI master. The driver allows for multiple
 * SPI devices in memory, thus the specific SPI device must be provided with every call.
 *
 * @see dev_soc.h
 *
 * The following code demonstrates how to use the SPI device, using a fictive SPI slave. So its
 * not so much what it does, but more how it does it. This should of course be translated to your
 * specific SPI slaves' need.
 * @code
 *
 * // Initialize default, no auto slave select, 1 MBit.
 * const SpiInit init = SPI_DEFAULT_INIT;
 *
 * // initialize SPI device.
 * if (!spiInit(SPI, &init)) {
 *      errPrint(true);
 *      return;
 * }
 *
 * // ! fictive SPI slave command build up.
 * const int cmdSize = 33;
 * uint8_t bufIn[cmdSize];
 * uint8_t bufOut[cmdSize];
 * bufIn[0] = 0x32;     // fictive read packet command
 *
 * spiSelect(3);        // select slave no. 3 (fictive device).
 *
 * if (!spiTxRx(SPI, bufIn, bufOut, packetSize)) {
 *      spiDeselect();
 *      errPrint(true);
 *      return;
 * }
 * spiDeselect();
 * printf("Success! received a packet!");
 *
 * @endcode
 */

#include <stdint.h>
#include "dev_spi.h"

#define WISHBONE_FREQ 	100000000

#define SPI_MAX_BYTES   16      ///< The maximum bytes the SPI driver can transfer at a time
#define SPI_MAX_SLAVE   8       ///< Maximum number of slaves (0 - SPI_MAX_SLAVE - 1)

/// Minimum SPI bitrate
#define SPI_MIN_BITRATE     ( WISHBONE_FREQ / ( ( SPI_DIV_MASK + 1 ) * 2 ) )
/// Maximum SPI bitrate
#define SPI_MAX_BITRATE     ( WISHBONE_FREQ / 2 )

#define E_SPI_TIMEOUT           ( E_SPI + 1 )       ///< SPI transmission timeout
#define E_SPI_TIMEOUT_DESCR     "SPI transmission timeout"

/**
 * SPI initialization structure.
 *
 * See OpenCores SPI device documentation for more information. Note that only byte-sized
 * transfers are supported by this driver.
 *
 * @attention SPI modes 2 and 3 are not well supported!
 */
typedef struct
{
    uint32_t        bitrate;            ///< Bit-rate.
    uint8_t         mode;               ///< SPI mode (0 - 3, see Wikipedia, 0 - default).
    bool            lsbFirst;           ///< Transfer LSB first.
    bool            intEnable;          ///< Interrupts enabled (not implemented yet).
    bool            autoSlaveSelect;    ///< Auto slave select.

} SpiInit;

/**
 * Initializes the specified SPI device with the specified parameters.
 *
 * @param   dev     The SPI device.
 * @param   init    The initialization structure.
 *
 * @retval  true    Operation was a success.
 * @retval  false   Operation failed, check errCode() for the error code.
 */
bool spiInit(SPI_Device dev, SpiInit * init, int verbose);

/**
 * Transfers a specific number of bytes in a synchronous way. Note that the underlying SPI device
 * can at most send SPI_MAX_BYTES. When the length of the data to transmit exceeds this value,
 * multiple transmits will be issued.
 *
 * @note    function will exit after all bytes have been send and received.
 * @note    if auto slave select is enabled, each SPI_MAX_BYTES the SS will be selected and
 *          deselected.
 *
 * @param   dev         The WB device.
 * @param   dataIn      The data to receive, may be NULL to send dummy data.
 * @param   dataOut     The data to transmit, may be NULL to not receive data.
 * @param   len         The no of bytes to transmit and receive.
 *
 * @retval  true    Operation was a success.
 * @retval  false   Operation failed, check errCode() for the error code.
 */
bool spiTxRx(SPI_Device dev, uint8_t * dataIn, uint8_t * dataOut, int len, int verbose);

/**
 * Start a asynchronous SPI transmission.
 *
 * @param dev       The SPI device.
 * @param dataIn    The buffer to write
 * @param len       The length of of the data to send. May not exceed SPI_MAX_BYTES.
 */
void spiAsyncTx(SPI_Device dev, uint8_t * dataIn, int length, int verbose);

/**
 * Returns whether or not the SPI driver is busy tranceiving data.
 *
 * @param dev   The device to query.
 * @retval      true        Its busy
 * @retval      false       Its ready.
 */
bool spiASyncBusy(SPI_Device dev, int verbose);

/**
 * Last lenght of data transmitted. Can be used for asynchronous communciation.
 *
 * @param dev   The SPIdevice.
 * @return      The no of bytes which can be read.
 */
int spiASyncLength(SPI_Device dev, int verbose);

/**
 * Reads the data received in the previous transmission.
 *
 * @param dev       The device to read.
 * @param dataOut   The buffer to write into
 *
 * @retval      true    All went ok.
 * @retval      false   Failed, see errCode() for error.
 *
 */
void spiASyncRx(SPI_Device dev, uint8_t * dataOut, int verbose);

/**
 * Selects a specific slave.
 *
 * @param   dev         The WB SPI device.
 * @param   slaveNo     The selected slave number.
 */
void spiSelect(SPI_Device dev, uint32_t slaveNo, int verbose);

/**
 * Deselects all slaves.
 *
 * @param   dev         The WB SPI device.
 */
void spiDeselect(SPI_Device dev, int verbose);


#endif /* SPI_H_ */
