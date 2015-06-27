
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "mpu9250.h"

MPU9250::MPU9250(unsigned int spi_device, int verbose) {
    _spi_device = spi_device;
    _verbose    = verbose;
}

uint8_t MPU9250::wr_reg(uint8_t wr_addr, uint8_t wr_data)
{
    uint8_t buf_rd[16];
    uint8_t buf_wr[16];

    int i;
    for(i=0; i<16; i++) {
        buf_rd[i] = 0;
        buf_wr[i] = 0;
    }

    buf_wr[0] = wr_addr;
    buf_wr[1] = wr_data;

    spiSelect  (_spi_device, 0, _verbose-1);
    spiTxRx    (_spi_device, buf_wr, buf_rd, 2, _verbose-1);
    spiDeselect(_spi_device, _verbose-1);

    return buf_rd[1];
}

uint8_t MPU9250::rd_reg(uint8_t wr_addr, uint8_t wr_data)
{
    return wr_reg(wr_addr | READ_FLAG, wr_data);
}

void MPU9250::rd_reg_mult( uint8_t rd_addr, uint8_t * rd_buf, unsigned int bytes)
{
    unsigned int i;
    for (i=0; i<bytes; i++) {
        rd_buf[i] = rd_reg(((uint8_t)(rd_addr+i))|READ_FLAG, 0);
    }
}

/*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
low pass filter value; suitable values are:
BITS_DLPF_CFG_256HZ_NOLPF2
BITS_DLPF_CFG_188HZ
BITS_DLPF_CFG_98HZ
BITS_DLPF_CFG_42HZ
BITS_DLPF_CFG_20HZ
BITS_DLPF_CFG_10HZ 
BITS_DLPF_CFG_5HZ 
BITS_DLPF_CFG_2100HZ_NOLPF
returns 1 if an error occurred
-----------------------------------------------------------------------------------------------*/
#define MPU_InitRegNum 17

bool MPU9250::init(uint8_t sample_rate_div, uint8_t low_pass_filter)
{
    SpiInit spi_init = {
        .bitrate         = 1000000,
        .mode            = 0,
        .lsbFirst        = false,
        .intEnable       = false,
        .autoSlaveSelect = false};

    if (!spiInit(_spi_device, &spi_init, _verbose)) {
        printf("MS5803 : ERROR: FAILED TO INIT SPI DEVICE : 0x%08x\n", _spi_device);
        return false;
    }

    uint8_t i = 0;
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
        {0x80,              MPUREG_PWR_MGMT_1},         // Reset Device
        {0x01,              MPUREG_PWR_MGMT_1},         // Clock Source
        {0x00,              MPUREG_PWR_MGMT_2},         // Enable Acc & Gyro
        {low_pass_filter,   MPUREG_CONFIG},             // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {0x18,              MPUREG_GYRO_CONFIG},        // +-2000dps
        {0x08,              MPUREG_ACCEL_CONFIG},       // +-4G
        {0x09,              MPUREG_ACCEL_CONFIG_2},     // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x30,              MPUREG_INT_PIN_CFG},        //
      //{0x40,              MPUREG_I2C_MST_CTRL},       // I2C Speed 348 kHz
      //{0x20,              MPUREG_USER_CTRL},          // Enable AUX
        {0x20,              MPUREG_USER_CTRL},          // I2C Master mode
        {0x0D,              MPUREG_I2C_MST_CTRL},       // I2C configuration multi-master  IIC 400KHz
        
        {AK8963_I2C_ADDR,   MPUREG_I2C_SLV0_ADDR},      //Set the I2C slave addres of AK8963 and set for write.
      //{0x09,              MPUREG_I2C_SLV4_CTRL},
      //{0x81,              MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

        {AK8963_CNTL2,      MPUREG_I2C_SLV0_REG},       //I2C slave 0 register address from where to begin data transfer
        {0x01,              MPUREG_I2C_SLV0_DO},        // Reset AK8963
        {0x81,              MPUREG_I2C_SLV0_CTRL},      //Enable I2C and set 1 byte

        {AK8963_CNTL1,      MPUREG_I2C_SLV0_REG},       //I2C slave 0 register address from where to begin data transfer
        {0x12,              MPUREG_I2C_SLV0_DO},        // Register value to continuous measurement in 16bit
        {0x81,              MPUREG_I2C_SLV0_CTRL}       //Enable I2C and set 1 byte
    };

    for(i=0; i<MPU_InitRegNum; i++) {
        wr_reg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        usleep(1000);
    }

    set_accl_scale(BITS_FS_8G);
    set_gyro_scale(BITS_FS_250DPS);
    calib_accl();

    usleep(10000);
    
    //calib_magn();  //Can't load this function here , strange problem?
    return true;
}
/*-----------------------------------------------------------------------------------------------
                                ACCELEROMETER SCALE
usage: call this function at startup, after initialization, to set the right range for the
accelerometers. Suitable ranges are:
BITS_FS_2G
BITS_FS_4G
BITS_FS_8G
BITS_FS_16G
returns the range set (2,4,8 or 16)
-----------------------------------------------------------------------------------------------*/
unsigned int MPU9250::set_accl_scale(int scale){
    unsigned int temp_scale;
    wr_reg(MPUREG_ACCEL_CONFIG, scale);
    
    switch (scale){
        case BITS_FS_2G:
            accl_div=16384;
        break;
        case BITS_FS_4G:
            accl_div=8192;
        break;
        case BITS_FS_8G:
            accl_div=4096;
        break;
        case BITS_FS_16G:
            accl_div=2048;
        break;   
    }
    temp_scale=wr_reg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);
    
    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale=2;
        break;
        case BITS_FS_4G:
            temp_scale=4;
        break;
        case BITS_FS_8G:
            temp_scale=8;
        break;
        case BITS_FS_16G:
            temp_scale=16;
        break;   
    }
    return temp_scale;
}


/*-----------------------------------------------------------------------------------------------
                                GYROSCOPE SCALE
usage: call this function at startup, after initialization, to set the right range for the
gyroscopes. Suitable ranges are:
BITS_FS_250DPS
BITS_FS_500DPS
BITS_FS_1000DPS
BITS_FS_2000DPS
returns the range set (250,500,1000 or 2000)
-----------------------------------------------------------------------------------------------*/
unsigned int MPU9250::set_gyro_scale(int scale){
    unsigned int temp_scale;
    wr_reg(MPUREG_GYRO_CONFIG, scale);
    switch (scale){
        case BITS_FS_250DPS:
            gyro_div=131;
        break;
        case BITS_FS_500DPS:
            gyro_div=65.5;
        break;
        case BITS_FS_1000DPS:
            gyro_div=32.8;
        break;
        case BITS_FS_2000DPS:
            gyro_div=16.4;
        break;   
    }
    temp_scale=wr_reg(MPUREG_GYRO_CONFIG|READ_FLAG, 0x00);
    switch (temp_scale){
        case BITS_FS_250DPS:
            temp_scale=250;
        break;
        case BITS_FS_500DPS:
            temp_scale=500;
        break;
        case BITS_FS_1000DPS:
            temp_scale=1000;
        break;
        case BITS_FS_2000DPS:
            temp_scale=2000;
        break;   
    }
    return temp_scale;
}


/*-----------------------------------------------------------------------------------------------
                                WHO AM I?
usage: call this function to know if SPI is working correctly. It checks the I2C address of the
mpu9250 which should be 104 when in SPI mode.
returns the I2C address (104)
-----------------------------------------------------------------------------------------------*/
uint8_t MPU9250::chipid_mp9250(){
    return wr_reg(MPUREG_WHOAMI|READ_FLAG, 0x00);
}


/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
-----------------------------------------------------------------------------------------------*/
void MPU9250::read_accl()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    rd_reg_mult(MPUREG_ACCEL_XOUT_H,response,6);
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        accl_val[i]=data/accl_div;
    }
    
}

/*-----------------------------------------------------------------------------------------------
                                READ GYROSCOPE
usage: call this function to read gyroscope data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
-----------------------------------------------------------------------------------------------*/
void MPU9250::read_gyro()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    rd_reg_mult(MPUREG_GYRO_XOUT_H,response,6);
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        gyro_val[i]=data/gyro_div;
    }

}

/*-----------------------------------------------------------------------------------------------
                                READ TEMPERATURE
usage: call this function to read temperature data. 
returns the value in °C
-----------------------------------------------------------------------------------------------*/
void MPU9250::read_temp(){
    uint8_t response[2];
    int16_t bit_data;
    float data;
    rd_reg_mult(MPUREG_TEMP_OUT_H,response,2);

    bit_data=((int16_t)response[0]<<8)|response[1];
    data=(float)bit_data;
    temp_val=(data/340)+36.53;
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns Factory Trim value
-----------------------------------------------------------------------------------------------*/
void MPU9250::calib_accl()
{
    uint8_t response[4];
    int temp_scale;
    //READ CURRENT ACC SCALE
    temp_scale=wr_reg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);
    set_accl_scale(BITS_FS_8G);
    //ENABLE SELF TEST need modify
    //temp_scale=wr_reg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

    rd_reg_mult(MPUREG_SELF_TEST_X,response,4);
    accl_cal[0]=((response[0]&11100000)>>3)|((response[3]&00110000)>>4);
    accl_cal[1]=((response[1]&11100000)>>3)|((response[3]&00001100)>>2);
    accl_cal[2]=((response[2]&11100000)>>3)|((response[3]&00000011));

    set_accl_scale(temp_scale);
}

uint8_t MPU9250::chipid_ak8963(){
    uint8_t response;
    wr_reg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    wr_reg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    wr_reg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

    usleep(1000);

    //wr_reg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    response=wr_reg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);    //Read I2C 
    //rd_reg_mult(MPUREG_EXT_SENS_DATA_00,response,1);
    //response=wr_reg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C 

    return response;
}

void MPU9250::calib_magn(){
    uint8_t response[3];
    float data;
    int i;

    wr_reg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    wr_reg(MPUREG_I2C_SLV0_REG, AK8963_ASAX); //I2C slave 0 register address from where to begin data transfer
    wr_reg(MPUREG_I2C_SLV0_CTRL, 0x83); //Read 3 bytes from the magnetometer

    //wr_reg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    usleep(1000);
    //response[0]=wr_reg(MPUREG_EXT_SENS_DATA_01|READ_FLAG, 0x00);    //Read I2C 
    rd_reg_mult(MPUREG_EXT_SENS_DATA_00,response,3);
    
    //response=wr_reg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C 
    for(i=0; i<3; i++) {
        data=response[i];
        magn_asa[i]=((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
}

void MPU9250::read_magn(){
    uint8_t response[7];
    int16_t bit_data;
    float data;
    int i;

    wr_reg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    wr_reg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    wr_reg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 6 bytes from the magnetometer

    usleep(1000);
    rd_reg_mult(MPUREG_EXT_SENS_DATA_00,response,7);
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
        data=(float)bit_data;
        magn_val[i]=data*magn_asa[i];
    }
}

void MPU9250::read(float *ax, float *ay, float *az,
              float *gx, float *gy, float *gz,
              float *mx, float *my, float *mz,
              float *temp) 
{
    uint8_t response[21];
    int16_t bit_data;
    float data;
    int i;

    //Send I2C command at first
    wr_reg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    wr_reg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    wr_reg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

    usleep(1000);
    rd_reg_mult(MPUREG_ACCEL_XOUT_H,response,21);
    //Get accelerometer value
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        accl_val[i]=data/accl_div;
    }
    //Get temperature
    bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
    data=(float)bit_data;
    temp_val=((data-21)/333.87)+21;
    //Get gyroscop value
    for(i=4; i<7; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        gyro_val[i-4]=data/gyro_div;
    }
    //Get Magnetometer value
    for(i=7; i<10; i++) {
        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
        data=(float)bit_data;
        magn_val[i-7]=data*magn_asa[i-7];
    }

    *ax = accl_val[0];
    *ay = accl_val[1];
    *az = accl_val[2];

    *gx = gyro_val[0];
    *gy = gyro_val[1];
    *gz = gyro_val[2];

    *mx = magn_val[0];
    *my = magn_val[1];
    *mz = magn_val[2];

    *temp = temp_val;
}

