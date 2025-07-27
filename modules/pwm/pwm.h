#ifndef PWM_H
#define PWM_H

#include "module.h"

Module* createPWM(JsonObject module, Comms* comms);

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

        PWM(volatile float&, int, std::string);
        PWM(volatile float&, volatile float&, int, std::string);

        virtual void update(void);
};

#endif
