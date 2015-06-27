//
//  PWM.cpp
//
//  This library is for reading and writing to the PWM Controller
//
//  Created by Rahul Salvi on 06/24/15

#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "../Utilities/regio.h"
#include "pwm.h"

// PWM constants:
#define PWM_GCTL_BASE       0x00
#define PWM_MCTL_BASE       0x40
#define PWM_CCTL_BASE       0x80

#define PWM_GCTL_ADDR(a)    (a+PWM_GCTL_BASE)
#define PWM_MCTL_ADDR(a,b)  (a+PWM_MCTL_BASE+(4*b))
#define PWM_CCTL_ADDR(a,b)  (a+PWM_CCTL_BASE+(4*b))

PWM::PWM(unsigned int pwm_device, int verbose) 
{
    _pwm_device = pwm_device;
    _verbose    = verbose;
}

void PWM::enable() 
{
    regio_wr32(PWM_GCTL_ADDR(_pwm_device), 0x00000001, _verbose-1);
}

void PWM::disable() 
{
    regio_wr32(PWM_GCTL_ADDR(_pwm_device), 0x00000000, _verbose-1);
}

void PWM::setMap(int idx, int chn) 
{
    int reg_id  = (idx % 24) >> 2;
    int fld_id  = (idx %  4);

    unsigned int data, mask;

    regio_rd32(PWM_MCTL_ADDR(_pwm_device,reg_id), &data, _verbose-1);
    mask    = (((1<<5)-1) << (8*fld_id));
    data    = ((data & ~mask) | ((chn << 8*fld_id) & mask));
    regio_wr32(PWM_MCTL_ADDR(_pwm_device,reg_id),  data, _verbose-1);
}

void PWM::setMap(int * table) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        setMap(idx, table[idx]);
    }
}

void PWM::getMap(int idx, int * chn) 
{
    int reg_id  = (idx % 24) >> 2;
    int fld_id  = (idx %  4);

    unsigned int data, mask;

    regio_rd32(PWM_MCTL_ADDR(_pwm_device,reg_id), &data, _verbose-1);
    mask    = (((1<<5)-1) << (8*fld_id));
    data    = ((data & mask) >> (8*fld_id));
    *chn    = data;
}

void PWM::getMap(int * table) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        getMap(idx, &table[idx]);
    }
}

void PWM::setDuty(int idx, float percent) 
{
    unsigned int data = ((unsigned int)((percent/100.0)*8192.0));

    regio_wr32(PWM_CCTL_ADDR(_pwm_device,idx), ((data > 8191) ? 8191 : data), _verbose-1);
}

void PWM::setDuty(float * percents) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        setDuty(idx, percents[idx]);
    }
}

void PWM::setValue(int idx, unsigned int value) 
{
    regio_wr32(PWM_CCTL_ADDR(_pwm_device,idx), ((value > 8191) ? 8191 : value), _verbose-1);
}

void PWM::setValue(unsigned int * values) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        setValue(idx, values[idx]);
    }
}

void PWM::getDuty(int idx, float * percent) 
{
    unsigned int data;

    regio_rd32(PWM_CCTL_ADDR(_pwm_device,idx), &data, _verbose-1);

    *percent = ((((float)data)*100.0)/8192.0);
}

void PWM::getDuty(float * percents) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        getDuty(idx, &percents[idx]);
    }
}

void PWM::getValue(int idx, unsigned int * value) 
{
    unsigned int data;

    regio_rd32(PWM_CCTL_ADDR(_pwm_device,idx), &data, _verbose-1);

    *value = data;
}

void PWM::getValue(unsigned int * values) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        getValue(idx, &values[idx]);
    }
}

