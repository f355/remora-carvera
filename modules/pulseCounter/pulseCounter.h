#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include "modules/module.h"

#include "mbed.h"


Module* createPulseCounter(JsonObject module, RemoraComms* comms);

class PulseCounter : public Module
{
    private:

        volatile float *ptrPulseCount;
        volatile int32_t count;
        void interruptHandler();

    public:

        mbed::InterruptIn *interrupt;

        PulseCounter(volatile float&, std::string);

        virtual void update(void);
};

#endif
