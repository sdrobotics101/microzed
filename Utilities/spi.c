/*
 * spi.c
 *
 *  Created on: 2 apr. 2013
 *      Author: vincentb
 */

#include <stdio.h>
#include <assert.h>

#include "dev_defs.h"
#include "regio.h"
#include "spi.h"

#define _SPI_MASK_ALL       0xFFFFFFFF

static void spiRMW(_reg_a adr, _reg_t val, _reg_t mask, int verbose)
{
    _reg_t tmp;

    regio_rd32(adr, &tmp, verbose-1);

    if ( ( tmp & mask ) == ( val & mask ) ) return;
    tmp = (tmp & ~mask) | (val & mask);

    regio_wr32(adr,  tmp, verbose-1);
}

#define SPI_FLAG( REG, BOOL, FLAG, VERBOSE ) \
        spiRMW( REG, ( BOOL ) ? FLAG : 0, FLAG, VERBOSE)

bool spiInit(SPI_Device dev, SpiInit * init, int verbose)
{

    assert(init->mode <= 3);
    assert(init->bitrate <= SPI_MAX_BITRATE);               // maximum frequency
    assert(init->bitrate >= SPI_MIN_BITRATE);               // minimum frequency

    spiRMW( SPI_DEVICE_DIVIDER(dev), (WISHBONE_FREQ / ( init->bitrate * 2 )) - 1, SPI_DIV_MASK, verbose);

    const uint32_t mode[] = {
            SPI_CTL_MODE_0, SPI_CTL_MODE_1, SPI_CTL_MODE_2, SPI_CTL_MODE_3 };

    spiRMW( SPI_DEVICE_CTL(dev), mode[init->mode], SPI_CTL_MODE_MASK, verbose);

    SPI_FLAG(SPI_DEVICE_CTL(dev), init->autoSlaveSelect    , SPI_CTL_ASS, verbose);
    SPI_FLAG(SPI_DEVICE_CTL(dev), init->intEnable          , SPI_CTL_IE , verbose);
    SPI_FLAG(SPI_DEVICE_CTL(dev), init->lsbFirst           , SPI_CTL_LSB, verbose);

    return true;
}

void spiAsyncTx(SPI_Device dev, uint8_t * dataIn, int len, int verbose)
{
    _reg_t tmp;

    assert(len > 0 && len <= SPI_MAX_BYTES );

    spiRMW( SPI_DEVICE_CTL(dev), ( len * 8 ) & SPI_CTL_CHAR_LEN_MASK, SPI_CTL_CHAR_LEN_MASK, verbose);

    if (dataIn != NULL) {

        // write bytes in the correct manner.
        int clen = 0;
        int bcnt = 0;
        int wcnt = 0;

        uint32_t tx[4] = {0, 0, 0, 0};
        for (wcnt=0; wcnt<4; wcnt++) {
            for (bcnt=0; bcnt<4; bcnt++) {
                if (clen < len) {
                    tx[wcnt] = ((tx[wcnt] << 8) | (dataIn[clen] & 0xff));
                    clen++;
                }
            }
        }

        regio_wr32(SPI_DEVICE_TX0(dev), tx[0], verbose-1);
        regio_wr32(SPI_DEVICE_TX1(dev), tx[1], verbose-1);
        regio_wr32(SPI_DEVICE_TX2(dev), tx[2], verbose-1);
        regio_wr32(SPI_DEVICE_TX3(dev), tx[3], verbose-1);
    }

    regio_rd32(SPI_DEVICE_CTL(dev), &tmp, verbose-1);
    tmp |= SPI_CTL_GO_BSY;
    regio_wr32(SPI_DEVICE_CTL(dev),  tmp, verbose-1);
}

bool spiASyncBusy(SPI_Device dev, int verbose)
{
    _reg_t tmp;

    regio_rd32(SPI_DEVICE_CTL(dev), &tmp, verbose-1);

    return ( tmp & SPI_CTL_GO_BSY ) != 0;
}

int spiASyncLength(SPI_Device dev, int verbose)
{
    _reg_t tmp;

    regio_rd32(SPI_DEVICE_CTL(dev), &tmp, verbose-1);

    return ( tmp & SPI_CTL_CHAR_LEN_MASK ) / 8;
}

void spiASyncRx(SPI_Device dev, uint8_t * dataOut, int verbose)
{
    int len  = spiASyncLength(dev, verbose);

    int clen = 0;
    int bcnt = 0;
    int wcnt = 0;

    uint32_t rx[4] = {0, 0, 0, 0};

    regio_rd32(SPI_DEVICE_RX0(dev), &rx[0], verbose-1);
    regio_rd32(SPI_DEVICE_RX1(dev), &rx[1], verbose-1);
    regio_rd32(SPI_DEVICE_RX2(dev), &rx[2], verbose-1);
    regio_rd32(SPI_DEVICE_RX3(dev), &rx[3], verbose-1);
    
    for (wcnt=0; wcnt<4; wcnt++) {
        for (bcnt=0; bcnt<4; bcnt++) {
            if ((len - clen) > 0) {
                switch((len-clen-1) % 4) {
                    case 3 :
                        dataOut[clen+0] = (rx[wcnt] >> 24) & 0xff;
                        dataOut[clen+1] = (rx[wcnt] >> 16) & 0xff;
                        dataOut[clen+2] = (rx[wcnt] >>  8) & 0xff;
                        dataOut[clen+3] = (rx[wcnt] >>  0) & 0xff;
                        clen += 4;
                        break;

                    case 2 :
                        dataOut[clen+0] = (rx[wcnt] >> 16) & 0xff;
                        dataOut[clen+1] = (rx[wcnt] >>  8) & 0xff;
                        dataOut[clen+2] = (rx[wcnt] >>  0) & 0xff;
                        clen += 3;
                        break;

                    case 1 :
                        dataOut[clen+0] = (rx[wcnt] >>  8) & 0xff;
                        dataOut[clen+1] = (rx[wcnt] >>  0) & 0xff;
                        clen += 2;
                        break;

                    case 0 :
                        dataOut[clen+0] = (rx[wcnt] >>  0) & 0xff;
                        clen += 1;
                        break;

                    default :
                        break;
                }
            }
        }
    }
}

static bool spiTxRxPart(SPI_Device dev, uint8_t * dataIn, uint8_t * dataOut, int len, int verbose)
{
    _reg_t tmp;

    spiAsyncTx(dev, dataIn, len, verbose);

    /* poll for completion : ROHAN : NEED TO ADD TIMEOUT MECHANISM HERE */
    do {

        regio_rd32(SPI_DEVICE_CTL(dev), &tmp, verbose-1);

    } while ((tmp & SPI_CTL_GO_BSY)==1);

    if (dataOut != NULL) spiASyncRx(dev, dataOut, verbose);

    return true;
}

bool spiTxRx(SPI_Device dev, uint8_t * dataIn, uint8_t * dataOut, int len, int verbose)
{
    bool rc;

    while (len > SPI_MAX_BYTES) {
        if (!spiTxRxPart(dev, dataIn, dataOut, SPI_MAX_BYTES, verbose)) return false;
        len -= SPI_MAX_BYTES;
        if (dataIn  != NULL) dataIn  += SPI_MAX_BYTES;
        if (dataOut != NULL) dataOut += SPI_MAX_BYTES;
    }

    rc = spiTxRxPart(dev, dataIn, dataOut, len, verbose);

    if (verbose > 0) {
        int cnt;
        printf("SREG W 0x%08x ", dev);
        for (cnt=0; cnt<len; cnt++) {
            printf("%02x ",dataIn[cnt]);
        }
        printf("\n");
        printf("SREG R 0x%08x ", dev);
        for (cnt=0; cnt<len; cnt++) {
            printf("%02x ",dataOut[cnt]);
        }
        printf("\n");
    }

    return rc;
}

void spiSelect(SPI_Device dev, uint32_t slaveNo, int verbose)
{
    _reg_t tmp;

    assert(slaveNo < SPI_MAX_SLAVE);

    regio_rd32(SPI_DEVICE_SS(dev), &tmp, verbose-1);

    tmp |= 1 << slaveNo;
    regio_wr32(SPI_DEVICE_SS(dev),  tmp, verbose-1);
}

void spiDeselect(SPI_Device dev, int verbose)
{
    regio_wr32(SPI_DEVICE_SS(dev), 0, verbose-1);
}
