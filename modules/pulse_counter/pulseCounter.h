#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include "module.h"

class PulseCounter final : public Module {
  volatile int32_t* output_var;
  InterruptIn* interrupt;
  void interrupt_handler();

 public:
  PulseCounter(int var_number, Pin* pin, volatile txData_t* tx_data);

  bool needs_periodic_update = false;
};

#endif
