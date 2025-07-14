#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include "modules/module.h"

Module* createDigitalPin(JsonObject module, RemoraComms* comms);

class DigitalPin : public Module
{
    private:

        volatile uint16_t *ptrData;
        int mask;
        int mode;
        Pin *pin;

    public:

        DigitalPin(volatile uint16_t&, int, std::string, int);
        virtual void update(void);
};

#endif
