#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "modules/module.h"

class Thermistor : public Module
{
    private:
        volatile float* ptrFeedback; // pointer where to put the feedback
        
        float temperaturePV;

        AnalogIn *adc;
        float r0;
        int r1;
        int r2;
        float j;
        float k;

        float getTemperature();

    public:

        Thermistor(int processVariable, Pin* pin, float beta, int r0, int t0, int32_t threadFreq, volatile txData_t* txData);

        virtual void slowUpdate(void);
};


#endif
