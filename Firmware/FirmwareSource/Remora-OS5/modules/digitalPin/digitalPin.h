#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include "modules/module.h"

Module* createDigitalPin(JsonObject module, RemoraComms* comms);

class DigitalPin : public Module
{
    private:

        volatile uint16_t *ptrData;
        int bitNumber;
        bool invert;
        int mask;

        int mode;
        int modifier;
        std::string portAndPin;

        Pin *pin;

    public:

        DigitalPin(volatile uint16_t&, int, std::string, int, bool, int);
        virtual void update(void);
        virtual void slowUpdate(void);
};

#endif
