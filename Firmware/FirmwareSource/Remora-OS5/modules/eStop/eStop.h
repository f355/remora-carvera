#ifndef ESTOP_H
#define ESTOP_H

#include "modules/module.h"

Module* createEStop(JsonObject module, RemoraComms* comms);

class eStop : public Module
{
    private:

        volatile int32_t *ptrTxHeader;
        std::string portAndPin;

        Pin *pin;

    public:

        eStop(volatile int32_t&, std::string);

        virtual void update(void);
        virtual void slowUpdate(void);
};

#endif
