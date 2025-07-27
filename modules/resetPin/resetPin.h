#ifndef RESETPIN_H
#define RESETPIN_H

#include "module.h"

Module* createResetPin(JsonObject module, Comms* comms);

class ResetPin : public Module
{
    private:

        volatile bool *ptrReset; // pointer to the data source

        Pin *pin;

    public:

        ResetPin(volatile bool&, std::string);
        virtual void update(void);
};

#endif
