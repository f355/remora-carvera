#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include "mbed.h"
#include "module.h"

Module* createPulseCounter(JsonObject module, Comms* comms);

class PulseCounter : public Module {
  volatile float* ptrPulseCount;
  volatile int32_t count;
  void interruptHandler();

 public:
  InterruptIn* interrupt;

  PulseCounter(volatile float&, std::string);

  void update() override;
};

#endif
