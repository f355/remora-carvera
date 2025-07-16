#include "pulseCounter.h"

#include "port_api.h"
#include "LPC17xx.h"

PulseCounter::PulseCounter(int processVariable, Pin* pin, volatile txData_t* txData) :
    ptrPulseCount(&txData->processVariable[processVariable]),
    count(0)
{
    this->interrupt = pin->interrupt_pin();
    interrupt->rise(callback(this, &PulseCounter::interruptHandler));
    NVIC_SetPriority(EINT3_IRQn, 4);
}

void PulseCounter::update()
{
    *(this->ptrPulseCount) = this->count;
}

void PulseCounter::interruptHandler()
{
    this->count++;
}
