#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include "module.h"
#include "pin.h"

class PulseCounter final : public Module {
  volatile float* output_var;
  InterruptIn* interrupt;
  void interrupt_handler();

 public:
  PulseCounter(int var_number, Pin* pin, volatile txData_t* tx_data);
};

#endif
