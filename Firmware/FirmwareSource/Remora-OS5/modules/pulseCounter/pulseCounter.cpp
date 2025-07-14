#include "pulseCounter.h"

#include "port_api.h"
#include "LPC17xx.h"

Module* createPulseCounter(JsonObject module, RemoraComms* comms)
{
    int pv = module["process_variable"];
    const char* pin = module["pin"];
    const char* modifier = module["modifier"];

    int mod = Module::parseModifier(modifier);

    return new PulseCounter(comms->ptrTxData->processVariable[pv], pin, mod);
}

PulseCounter::PulseCounter(volatile float &ptrPulseCount, std::string pin, int modifier) :
    ptrPulseCount(&ptrPulseCount),
    pin(pin),
    modifier(modifier)
{
    this->count = 0;

    // set up an interrupt handler
    Pin *dummy = new Pin(this->pin, INPUT, this->modifier);
    PinName pinname = dummy->pinToPinName();
    this->interrupt = new mbed::InterruptIn(pinname);
    interrupt->rise(this, &PulseCounter::interruptHandler);
    NVIC_SetPriority(EINT3_IRQn, 16);
}

void PulseCounter::update()
{
    *(this->ptrPulseCount) = this->count;
}

void PulseCounter::interruptHandler()
{
    this->count++;
}
