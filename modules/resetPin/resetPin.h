#ifndef RESETPIN_H
#define RESETPIN_H

#include "modules/module.h"

class ResetPin : public Module
{
    private:

        volatile bool *ptrReset; // pointer to the data source

        Pin *pin;

    public:

        ResetPin(volatile bool& pruReset, Pin* pin);
        virtual void update(void);
};

#endif
