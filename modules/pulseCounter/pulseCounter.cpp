#include "pulseCounter.h"

#include "LPC17xx.h"
#include "port_api.h"

Module *createPulseCounter(JsonObject module, Comms *comms) {
  int pv = module["process_variable"];
  const char *pin = module["pin"];

  return new PulseCounter(comms->ptr_tx_data->process_variable[pv], pin);
}

PulseCounter::PulseCounter(volatile float &ptrPulseCount, std::string portAndPin) : ptrPulseCount(&ptrPulseCount) {
  this->count = 0;

  this->interrupt = (new Pin(portAndPin))->interrupt_pin();
  interrupt->rise(callback(this, &PulseCounter::interruptHandler));
  NVIC_SetPriority(EINT3_IRQn, 4);
}

void PulseCounter::update() { *(this->ptrPulseCount) = this->count; }

void PulseCounter::interruptHandler() { this->count++; }
