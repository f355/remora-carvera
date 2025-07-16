#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include "modules/module.h"

class PulseCounter : public Module
{
    private:

        volatile float *ptrPulseCount;
        volatile int32_t count;
        void interruptHandler();

    public:

        mbed::InterruptIn *interrupt;

        PulseCounter(int processVariable, Pin* pin, volatile txData_t* txData);

        virtual void update(void);
};

#endif
