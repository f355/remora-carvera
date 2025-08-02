#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "comms.h"
#include "module.h"
#include "pin.h"

class ADC final : public Module {
  volatile int32_t* variable;  // pointer where to put the feedback
  AnalogIn* adc;

 public:
  ADC(int var_number, Pin* pin, volatile txData_t* tx_data);

  void run() override;
};

#endif
