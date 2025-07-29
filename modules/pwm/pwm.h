#ifndef PWM_H
#define PWM_H

#include "module.h"
#include "pin.h"

class PWM final : public Module {
  PwmOut *pwm_pin;  // PWM out object

  volatile float *set_duty_cycle;

  int period_us;       // Period (us)
  float duty_cycle;    // Pulse width (%)
  int pulse_width_us;  // Pulse width (us)

 public:
  PWM(int var_number, Pin *pin, int period_us, volatile rxData_t *rx_data);

  void update() override;
};

#endif
