#include "resetPin.h"

Module *createResetPin(const JsonObject module, Comms *comms) {
  const char *pin = module["pin"];

  return new ResetPin(comms->pru_reset, pin);
}

ResetPin::ResetPin(volatile bool &ptrReset, std::string portAndPin) : ptr_reset(&ptrReset) {
  this->pin = (new Pin(portAndPin))->as_input();
}

void ResetPin::update() { *(this->ptr_reset) = this->pin->get(); }
