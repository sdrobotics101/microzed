//
//  PWM.cpp
//
//  This library is for reading and writing to the PWM Controller
//
//  Created by Rahul Salvi on 06/24/15

#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "pwm.h"

// PWM constants:
#define PWM_GCTL_BASE       0x00
#define PWM_MCTL_BASE       0x40
#define PWM_CCTL_BASE       0x80

#define PWM_GCTL_ADDR(a)    (a+PWM_GCTL_BASE)
#define PWM_MCTL_ADDR(a,b)  (a+PWM_MCTL_BASE+(4*b))
#define PWM_CCTL_ADDR(a,b)  (a+PWM_CCTL_BASE+(4*b))

PWM::PWM(uint32_t pwm_device, int verbose) 
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

void PWM::setMap(int idx, uint32_t chn) 
{
    uint32_t reg_id  = (uint32_t)((idx % 24) >> 2);
    uint32_t fld_id  = (uint32_t)((idx %  4));

    uint32_t data, mask;

    regio_rd32(PWM_MCTL_ADDR(_pwm_device,reg_id), &data, _verbose-1);
    mask    = (((1<<5)-1) << (8*fld_id));
    data    = ((data & ~mask) | ((chn << 8*fld_id) & mask));
    regio_wr32(PWM_MCTL_ADDR(_pwm_device,reg_id),  data, _verbose-1);
}

void PWM::setMap(uint32_t * table) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        setMap(idx, table[idx]);
    }
}

void PWM::getMap(int idx, uint32_t * chn) 
{
    uint32_t reg_id  = (uint32_t)((idx % 24) >> 2);
    uint32_t fld_id  = (uint32_t)((idx %  4));

    uint32_t data, mask;

    regio_rd32(PWM_MCTL_ADDR(_pwm_device,reg_id), &data, _verbose-1);
    mask    = (((1<<5)-1) << (8*fld_id));
    data    = ((data & mask) >> (8*fld_id));
    *chn    = data;
}

void PWM::getMap(uint32_t * table) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        getMap(idx, &table[idx]);
    }
}

void PWM::setDuty(int idx, double percent) 
{
    uint32_t data = ((uint32_t)((percent/100.0)*8192.0));

    regio_wr32(PWM_CCTL_ADDR(_pwm_device,idx), ((data > 8191) ? 8191 : data), _verbose-1);
}

void PWM::setDuty(double * percents) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        setDuty(idx, percents[idx]);
    }
}

void PWM::setValue(int idx, uint32_t value) 
{
    regio_wr32(PWM_CCTL_ADDR(_pwm_device,idx), ((value > 8191) ? 8191 : value), _verbose-1);
}

void PWM::setValue(uint32_t * values) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        setValue(idx, values[idx]);
    }
}

void PWM::getDuty(int idx, double * percent) 
{
    uint32_t data;

    regio_rd32(PWM_CCTL_ADDR(_pwm_device,idx), &data, _verbose-1);

    *percent = ((((double)data)*100.0)/8192.0);
}

void PWM::getDuty(double * percents) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        getDuty(idx, &percents[idx]);
    }
}

void PWM::getValue(int idx, uint32_t * value) 
{
    uint32_t data;

    regio_rd32(PWM_CCTL_ADDR(_pwm_device,idx), &data, _verbose-1);

    *value = data;
}

void PWM::getValue(uint32_t * values) 
{
    int idx;
    for (idx=0; idx<24; idx++) {
        getValue(idx, &values[idx]);
    }
}

