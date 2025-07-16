#include "eStop.h"

eStop::eStop(volatile int32_t &ptrTxHeader, Pin* pin) :
    ptrTxHeader(&ptrTxHeader),
    pin(pin->as_input()) {}


void eStop::update()
{
    if (this->pin->get() == 1)
    {
        *ptrTxHeader = PRU_ESTOP;
    }
    else {
        *ptrTxHeader = PRU_DATA;
    }
}

