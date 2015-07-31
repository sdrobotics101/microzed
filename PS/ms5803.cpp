//
//  MS5803.cpp
//
//  This library is for reading and writing to the MS5803 pressure/temperature sensor.
//
//  Created by Victor Konshin on 4/10/13.
//
//
//  Copyright (c) 2013, Victor Konshin, info@ayerware.com
//  for the DIY Dive Computer project www.diydivecomputer.com
//  All rights reserved.

//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//  * Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the distribution.
//  * Neither the name of Ayerware Publishing nor the
//  names of its contributors may be used to endorse or promote products
//  derived from this software without specific prior written permission.

//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>

#include "ms5803.h"

// Sensor constants:
#define SENSOR_CMD_RESET      0x1E
#define SENSOR_CMD_ADC_READ   0x00
#define SENSOR_CMD_ADC_CONV   0x40
#define SENSOR_CMD_ADC_D1     0x00
#define SENSOR_CMD_ADC_D2     0x10
#define SENSOR_CMD_ADC_256    0x00
#define SENSOR_CMD_ADC_512    0x02
#define SENSOR_CMD_ADC_1024   0x04
#define SENSOR_CMD_ADC_2048   0x06
#define SENSOR_CMD_ADC_4096   0x08

MS5803::MS5803(uint32_t spi_device, int verbose) {
    _spi_device = spi_device;
    _verbose    = verbose;

    press         = 0;    // Stores actual pressure in mbars
    temp		  = 0;    // Stores actual temp in degrees C.

    D1            = 0;    // Stores uncompensated pressure value
    D2            = 0;    // Stores uncompensated temperature value
    deltaTemp     = 0;    // These three variable are used for the conversion.
    sensorOffset  = 0;
    sensitivity   = 0;

    int i;
    for(i=0; i<8; i++)
        sensorCoefficients[i] = 0;
}

// Sends a power on reset command to the sensor.
// Should be done at powerup and maybe on a periodic basis (needs to confirm with testing).
void MS5803::resetSensor() {

    uint8_t buf_rd[16];
    uint8_t buf_wr[16];

    int i;
    for(i=0; i<16; i++) {
        buf_rd[i] = 0;
        buf_wr[i] = 0;
    }

    buf_wr[0] = SENSOR_CMD_RESET;

    spiSelect  (_spi_device, 0, _verbose-1);
    spiTxRx    (_spi_device, buf_wr, buf_rd, 1, _verbose-1);
    spiDeselect(_spi_device, _verbose-1);

    if (_verbose > 1)
        printf("MS5803 : resetSensor completed\n");
}

bool MS5803::initSensor() {

    SpiInit spi_init = {
                .bitrate         = 1000000,
                .mode            = 0,
                .lsbFirst        = false,
                .intEnable       = false,
                .autoSlaveSelect = false};

    if (!spiInit(_spi_device, &spi_init, _verbose-1)) {
        printf("MS5803 : ERROR: FAILED TO INIT SPI DEVICE : 0x%08x\n", _spi_device);
        return false;
    }

    // resetting the sensor on startup is important
    resetSensor(); 
	
	// Read sensor coefficients - these will be used to convert sensor data into pressure and temp data
    for (uint8_t i = 0; i < 8; i++ ){
        sensorCoefficients[ i ] = ms5803ReadCoefficient( i );  // read coefficients
        
        if (_verbose > 0) {
            printf("MS5803 : 0x%08x : coeff        : %d : %u\n", _spi_device, i, sensorCoefficients[i]);
        }
    }
    
    unsigned char p_crc = sensorCoefficients[ 7 ];
    unsigned char n_crc = ms5803CRC4( sensorCoefficients ); // calculate the CRC
    
    // If the calculated CRC does not match the returned CRC, then there is a
    // data integrity issue. Check the connections for bad solder joints or
    // "flakey" cables.  If this issue persists, you may have a bad sensor.
    if ( p_crc != n_crc ) {
        return false;
    }

    if (_verbose > 1)
        printf("MS5803 : initSensor completed\n");
    
    return true;
}

void MS5803::readSensor() {

	// If power or speed are important, you can change the ADC resolution to a lower value.
	// Currently set to SENSOR_CMD_ADC_4096 - set to a lower defined value for lower resolution.
	D1 = ms5803CmdAdc( SENSOR_CMD_ADC_D1 + SENSOR_CMD_ADC_256 );    // read uncompensated pressure
    D2 = ms5803CmdAdc( SENSOR_CMD_ADC_D2 + SENSOR_CMD_ADC_256 );    // read uncompensated temperature

    // calculate 1st order pressure and temperature correction factors (MS5803 1st order algorithm). 
    deltaTemp    = D2 - sensorCoefficients[5] * pow( 2, 8 );
    sensorOffset = sensorCoefficients[2] * pow( 2, 16 ) + ( deltaTemp * sensorCoefficients[4] ) / pow( 2, 7 );
    sensitivity  = sensorCoefficients[1] * pow( 2, 15 ) + ( deltaTemp * sensorCoefficients[3] ) / pow( 2, 8 );

    if (_verbose > 1) {
        printf("MS5803 : 0x%08x : D1           : %lu\n", _spi_device, D1);
        printf("MS5803 : 0x%08x : D2           : %lu\n", _spi_device, D2);
        printf("MS5803 : 0x%08x : deltaTemp    : %f\n" , _spi_device, deltaTemp);
        printf("MS5803 : 0x%08x : sensorOffset : %f\n" , _spi_device, sensorOffset);
        printf("MS5803 : 0x%08x : sensitivity  : %f\n" , _spi_device, sensitivity);
    }
    
    // calculate 2nd order pressure and temperature (MS5803 2st order algorithm)
    temp = ( 2000 + (deltaTemp * sensorCoefficients[6] ) / pow( 2, 23 ) ) / 100; 
    press = ( ( ( ( D1 * sensitivity ) / pow( 2, 21 ) - sensorOffset) / pow( 2, 15 ) ) / 10 );

    if (_verbose > 0) {
        printf("MS5803 : 0x%08x : pressure     : %8.2f\n", _spi_device, press);
        printf("MS5803 : 0x%08x : temperature  : %8.2f\n", _spi_device, temp);
    }
    
    if (_verbose > 1)
        printf("MS5803 : readSensor completed\n");
}


// These sensors have coefficient values stored in ROM that are used to convert
// the raw temp/pressure data into degrees and mbars. This method reads the
// coefficient at the index value passed.  Valid values are 0-7. 
// See datasheet for more info.
unsigned int MS5803::ms5803ReadCoefficient(uint8_t index) {

    unsigned int result = 0;   // result to return
    
    uint8_t buf_rd[16];
    uint8_t buf_wr[16];

    int i;
    for(i=0; i<16; i++) {
        buf_rd[i] = 0;
        buf_wr[i] = 0;
    }

    buf_wr[0] = (0xA0 + (index * 2));

    spiSelect  (_spi_device, 0, _verbose-1);
    spiTxRx    (_spi_device, buf_wr, buf_rd, 3, _verbose-1);
    spiDeselect(_spi_device, _verbose-1);

    result = (((unsigned int) buf_rd[1]) << 8) + 
             (((unsigned int) buf_rd[2]) << 0);
    
    return( result );
}

// Coefficient at index 7 is a four bit CRC value for verifying the validity of the other coefficients.
// The value returned by this method should match the coefficient at index 7.
// If not there is something works with the sensor or the connection.
unsigned char MS5803::ms5803CRC4(unsigned int n_prom[]) {

    int cnt;
    unsigned int n_rem;
    unsigned int crc_read;
    unsigned char  n_bit;
    
    n_rem = 0x00;
    crc_read = sensorCoefficients[7];
    sensorCoefficients[7] = ( 0xFF00 & ( sensorCoefficients[7] ) );
    
    for (cnt = 0; cnt < 16; cnt++)
    { // choose LSB or MSB
        if ( cnt%2 == 1 ) n_rem ^= (unsigned short) ( ( sensorCoefficients[cnt>>1] ) & 0x00FF );
        else n_rem ^= (unsigned short) ( sensorCoefficients[cnt>>1] >> 8 );
        for ( n_bit = 8; n_bit > 0; n_bit-- )
        {
            if ( n_rem & ( 0x8000 ) )
            {
                n_rem = ( n_rem << 1 ) ^ 0x3000;
            }
            else {
                n_rem = ( n_rem << 1 );
            }
        }
    }
    
    n_rem = ( 0x000F & ( n_rem >> 12 ) );// // final 4-bit reminder is CRC code
    sensorCoefficients[7] = crc_read; // restore the crc_read to its original place
    
    return ( n_rem ^ 0x00 ); // The calculated CRC should match what the device initally returned.
}

// Use this method to send commands to the sensor.  Pretty much just used to read the pressure and temp data.
unsigned long MS5803::ms5803CmdAdc(uint8_t cmd) {

    unsigned int result = 0;
    
    uint8_t buf_rd[16];
    uint8_t buf_wr[16];

    int i;
    for(i=0; i<16; i++) {
        buf_rd[i] = 0;
        buf_wr[i] = 0;
    }

    buf_wr[0] = ( SENSOR_CMD_ADC_CONV + cmd );

    spiSelect  (_spi_device, 0, _verbose-1);
    spiTxRx    (_spi_device, buf_wr, buf_rd, 3, _verbose-1);
    spiDeselect(_spi_device, _verbose-1);

    switch ( cmd & 0x0f ) {
        case SENSOR_CMD_ADC_256  : usleep(625);   break;
        case SENSOR_CMD_ADC_512  : usleep(1250);  break;
        case SENSOR_CMD_ADC_1024 : usleep(2500);  break;
        case SENSOR_CMD_ADC_2048 : usleep(5000);  break;
        case SENSOR_CMD_ADC_4096 : usleep(10000); break;
        default:                                  break;
    }

    for(i=0; i<16; i++) {
        buf_rd[i] = 0;
        buf_wr[i] = 0;
    }

    buf_wr[0] = SENSOR_CMD_ADC_READ;
    
    spiSelect  (_spi_device, 0, _verbose-1);
    spiTxRx    (_spi_device, buf_wr, buf_rd, 4, _verbose-1);
    spiDeselect(_spi_device, _verbose-1);

    result = (((unsigned long) buf_rd[1]) << 16) +
             (((unsigned long) buf_rd[2]) <<  8) +
             (((unsigned long) buf_rd[3]) <<  0);

    return result;
}




