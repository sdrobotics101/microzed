//
//  PWM.h
//
//  This library is for reading and writing to the PWM Controller
//
//  Created by Rahul Salvi on 06/24/15
//

#ifndef ____PWM__
#define ____PWM__

#include <stdint.h>

#include "../Utilities/regio.h"

class PWM
{
public:

    PWM(uint32_t pwm_device, int verbose = 0);

    void enable();
    void disable();

    void setMap(int idx, uint32_t chn);
    void setMap(uint32_t * table);

    void getMap(int idx, uint32_t * chn);
    void getMap(uint32_t * table);

    void setDuty(int idx, double percent);
    void setDuty(double * percents);

    void setValue(int idx, uint32_t value);
    void setValue(uint32_t * values);

    void getDuty(int idx, double * percent);
    void getDuty(double * percents);

    void getValue(int idx, uint32_t * value);
    void getValue(uint32_t * values);
    
private:
    
    uint32_t _pwm_device;                  // pointer to pwm device
    int      _verbose;                     // verbose printouts
};

#endif /* defined(____PWM__) */
