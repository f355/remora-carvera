#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include "module.h"

Module* createDigitalPin(JsonObject module, Comms* comms);

class DigitalPin final : public Module {
  volatile uint16_t* ptrData;
  int mask;
  int mode;
  Pin* pin;

 public:
  DigitalPin(volatile uint16_t&, int, std::string, int);
  void update() override;
};

#endif
