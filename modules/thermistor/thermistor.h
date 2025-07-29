#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "module.h"
#include "pin.h"

class Thermistor final : public Module {
  volatile float* output_var;  // pointer where to put the feedback

  AnalogIn* adc;
  int r0;
  int r1;
  int r2;
  float j;
  float k;

  float getTemperature() const;

 public:
  Thermistor(int var_number, Pin* pin, float beta, int r0, int t0, uint32_t thread_freq, volatile txData_t* tx_data);

  void slow_update() override;
};

#endif
