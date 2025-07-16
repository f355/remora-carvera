#include "resetPin.h"

ResetPin::ResetPin(volatile bool &ptrReset, Pin* pin) :
    ptrReset(&ptrReset),
    pin(pin->as_input()) { }


void ResetPin::update()
{
    *(this->ptrReset) = this->pin->get();
}
