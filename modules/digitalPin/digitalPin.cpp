#include "digitalPin.h"

Module *createDigitalPin(JsonObject module, Comms *comms) {
  const char *pin = module["pin"];
  const char *modeStr = module["mode"];
  int dataBit = module["data_bit"];

  int mode;
  volatile uint16_t *ptrData;

  if (!strcmp(modeStr, "in")) {
    mode = PIN_MODE_INPUT;
    ptrData = &comms->ptr_tx_data->inputs;
  } else if (!strcmp(modeStr, "out")) {
    mode = PIN_MODE_OUTPUT;
    ptrData = &comms->ptr_rx_data->outputs;
  } else {
    error("Error - unknown Digital Pin mode [%s]\n", mode);
  }

  return new DigitalPin(*ptrData, mode, pin, dataBit);
}

DigitalPin::DigitalPin(volatile uint16_t &ptrData, int mode, std::string portAndPin, int bitNumber)
    : ptrData(&ptrData), mode(mode) {
  this->pin = (new Pin(portAndPin))->set_mode(mode);
  this->mask = 1 << bitNumber;
}

void DigitalPin::update() {
  if (this->mode == PIN_MODE_INPUT)  // the pin is configured as an input
  {
    bool pinState = this->pin->get();

    if (pinState)  // input is high
    {
      *(this->ptrData) |= this->mask;
    } else  // input is low
    {
      *(this->ptrData) &= ~this->mask;
    }
  } else  // the pin is configured as an output
  {
    this->pin->set(*(this->ptrData) & this->mask);
  }
}
