#include "switch.h"

// TODO refactor this to be in line with the rest of the modules
Switch::Switch(float SP, volatile float &ptrPV, std::string portAndPin, bool mode) :
    SP(SP),
    ptrPV(&ptrPV),
    mode(mode)
{
    this->pin = (new Pin(portAndPin))->as_output();
}

void Switch::update()
{
    bool pinState;

    pinState = this->mode;

    this->PV = *(this->ptrPV);

    if (this->PV > this->SP)
    {
        this->pin->set(pinState);
    }
    else
    {
        pinState = !pinState;
        this->pin->set(pinState);
    }

}
