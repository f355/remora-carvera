#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "module.h"

Module* createThermistor(JsonObject module, PRUThread* thread, Comms* comms);

class Thermistor final : public Module {
  volatile float* ptr_feedback;  // pointer where to put the feedback

  AnalogIn* adc;
  float r0;
  int r1;
  int r2;
  float j;
  float k;

  float get_temperature() const;

 public:
  Thermistor(volatile float&, int32_t, int32_t, std::string, float, int, int);

  void slow_update() override;
};

#endif
