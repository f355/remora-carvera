#ifndef PWM_H
#define PWM_H

#include "modules/module.h"

class PWM final : public Module {
  PwmOut* pwm_pin;  // PWM out object

  volatile int32_t* set_duty_cycle;

  int period_us;           // Period (us)
  int32_t duty_cycle;      // Pulse width (%)
  int32_t pulse_width_us;  // Pulse width (us)

 public:
  PWM(int var_number, Pin* pin, int period_us, volatile rxData_t* rx_data);

  void update() override;
};

#endif
