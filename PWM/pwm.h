//
//  PWM.h
//
//  This library is for reading and writing to the PWM Controller
//
//  Created by Rahul Salvi on 06/24/15
//

#ifndef ____PWM__
#define ____PWM__

class PWM
{
public:

    PWM(unsigned int pwm_device, int verbose = 0);

    void enable();
    void disable();

    void setMap(int idx, int chn);
    void setMap(int * table);

    void getMap(int idx, int * chn);
    void getMap(int * table);

    void setDuty(int idx, float percent);
    void setDuty(float * percents);

    void setValue(int idx, unsigned int value);
    void setValue(unsigned int * values);

    void getDuty(int idx, float * percent);
    void getDuty(float * percents);

    void getValue(int idx, unsigned int * value);
    void getValue(unsigned int * values);
    
private:
    
    unsigned int      _pwm_device;                  // pointer to pwm device
    int               _verbose;                     // verbose printouts
};

#endif /* defined(____PWM__) */
