#include "eStop.h"

Module* createEStop(JsonObject module, RemoraComms* comms)
{
    const char* pin = module["pin"];

    return new eStop(comms->ptrTxData->header, pin);
}


eStop::eStop(volatile int32_t &ptrTxHeader, std::string portAndPin) :
    ptrTxHeader(&ptrTxHeader)
{
    this->pin = (new Pin(portAndPin))->as_input(); // Input 0x0, Output 0x1
}


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

