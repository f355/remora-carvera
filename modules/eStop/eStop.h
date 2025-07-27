#ifndef ESTOP_H
#define ESTOP_H

#include "module.h"

Module* createEStop(JsonObject module, Comms* comms);

class eStop : public Module
{
    private:

        volatile int32_t *ptrTxHeader;
        Pin *pin;

    public:

        eStop(volatile int32_t&, std::string);

        virtual void update(void);
};

#endif
