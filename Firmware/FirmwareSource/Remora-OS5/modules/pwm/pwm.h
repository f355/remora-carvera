#ifndef PWM_H
#define PWM_H

#include "modules/module.h"

class PWM : public Module
{
    private:

        int pwmMax; // maximum PWM output
        int pwmSP; // PWM setpoint as a percentage of maxPwm

        PwmOut *pwmPin; // PWM out object

        volatile float *ptrPwmPeriod; // pointer to the data source
        volatile float *ptrPwmPulseWidth; // pointer to the data source

        int pwmPeriod; // Period (us)
        float pwmPulseWidth; // Pulse width (%)
        int pwmPulseWidth_us; // Pulse width (us)

        bool variablePeriod;

    public:

        PWM(int setPoint, Pin* pin, int period, volatile rxData_t* rxData);
        PWM(int setPoint, int setPointPeriod, Pin* pin, int period, volatile rxData_t* rxData);

        virtual void update(void);
};

#endif
