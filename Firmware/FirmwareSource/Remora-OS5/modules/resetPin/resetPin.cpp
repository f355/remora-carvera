#include "resetPin.h"

Module* createResetPin(JsonObject module, RemoraComms* comms)
{
    const char* pin = module["pin"];

    return new ResetPin(comms->pruReset, pin);
}

ResetPin::ResetPin(volatile bool &ptrReset, std::string portAndPin) :
    ptrReset(&ptrReset)
{
    this->pin = (new Pin(portAndPin))->as_input();
}


void ResetPin::update()
{
    *(this->ptrReset) = this->pin->get();
}
