#ifndef ADC_H
#define ADC_H

#include "comms.h"
#include "module.h"
#include "pin.h"

class ADC final : public Module {
  volatile int32_t* variable;  // pointer where to put the feedback
  analogin_t adc;

 public:
  ADC(int var_number, Pin* pin, volatile txData_t* tx_data);

  void run() override;
};

#endif
