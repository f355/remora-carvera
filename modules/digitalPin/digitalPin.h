#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include "modules/module.h"

class DigitalPin : public Module
{
    private:

        volatile uint16_t *ptrData;
        int mask;
        int mode;
        Pin *pin;

    public:

        DigitalPin(volatile uint16_t&, int, Pin* pin, int);
        virtual void update(void);
};

class InputPin : public DigitalPin
{
    public:
        InputPin(int bitNumber, Pin* pin, volatile txData_t* txData);
};

class OutputPin : public DigitalPin
{
    public:
        OutputPin(int bitNumber, Pin* pin, volatile rxData_t* rxData);
};

#endif
