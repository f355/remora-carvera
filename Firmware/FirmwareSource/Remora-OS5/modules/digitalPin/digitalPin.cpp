#include "digitalPin.h"

InputPin::InputPin(int bitNumber, Pin* pin, volatile txData_t* txData) :
    DigitalPin(txData->inputs, PIN_MODE_INPUT, pin, bitNumber) {}

OutputPin::OutputPin(int bitNumber, Pin* pin, volatile rxData_t* rxData) :
    DigitalPin(rxData->outputs, PIN_MODE_OUTPUT, pin, bitNumber) {}

DigitalPin::DigitalPin(volatile uint16_t &ptrData, int mode, Pin* pin, int bitNumber) :
    ptrData(&ptrData),
    mode(mode)
{
    this->pin = pin->set_mode(mode);
    this->mask = 1 << bitNumber;
}


void DigitalPin::update()
{
    if (this->mode == PIN_MODE_INPUT) // the pin is configured as an input
    {
        bool pinState = this->pin->get();

        if (pinState) // input is high
        {
            *(this->ptrData) |= this->mask;
        }
        else // input is low
        {
            *(this->ptrData) &= ~this->mask;
        }
    }
    else // the pin is configured as an output
    {
        this->pin->set(*(this->ptrData) & this->mask);
    }
}

