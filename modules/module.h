#ifndef MODULE_H
#define MODULE_H

#include <cstdint>
#include <string>

#include "thread/pruThread.h"
#include "drivers/pin/pin.h"
#include "drivers/comms/RemoraComms.h"

// Module base class
// All modules are derived from this base class

class Module
{
    protected:

        int32_t threadFreq;
        int32_t slowUpdateFreq;
        int32_t updateCount;
        int32_t counter;

    public:

        Module(); // constructor to run the module at the thread frequency
        Module(int32_t, int32_t); // constructor to run the module at a "slow update frequency" < thread frequency

        virtual ~Module();
        void runModule();
        virtual void update();
        virtual void slowUpdate();
};

#endif

