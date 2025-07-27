#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include "module.h"

#include "mbed.h"


Module* createPulseCounter(JsonObject module, Comms* comms);

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
