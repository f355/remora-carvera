#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "modules/module.h"

Module* createThermistor(JsonObject module, PRUThread* thread, RemoraComms* comms);

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

        Thermistor(volatile float&, int32_t, int32_t, std::string, float, int, int);

        virtual void slowUpdate(void);
};


#endif
