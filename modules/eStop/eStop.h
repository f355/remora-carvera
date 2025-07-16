#ifndef ESTOP_H
#define ESTOP_H

#include "modules/module.h"

class eStop : public Module
{
    private:

        volatile int32_t *ptrTxHeader;
        Pin *pin;

    public:

        eStop(volatile int32_t&, Pin* pin);

        virtual void update(void);
};

#endif
