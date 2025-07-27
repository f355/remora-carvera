#ifndef PWM_H
#define PWM_H

#include "module.h"

Module *createPWM(JsonObject module, Comms *comms);

class PWM final : public Module {
  int pwmMax;  // maximum PWM output
  int pwmSP;   // PWM setpoint as a percentage of maxPwm

  PwmOut *pwm_pin;  // PWM out object

  volatile float *set_duty_cycle;    // pointer to the data source
  volatile float *ptrPwmPulseWidth;  // pointer to the data source

  int period_us;       // Period (us)
  float duty_cycle;    // Pulse width (%)
  int pulse_width_us;  // Pulse width (us)

  bool variablePeriod;

 public:
  PWM(volatile float &, int, std::string);
  PWM(volatile float &, volatile float &, int, std::string);

  void update() override;
};

#endif
