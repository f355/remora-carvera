#include "pwm.h"

#define PWMPERIOD 200

PWM::PWM(const int var_number, const Pin* pin, const int period_us, volatile rxData_t* rx_data)
    : pwm_pin(new PwmOut(pin->to_pin_name())),
      set_duty_cycle(&rx_data->output_vars[var_number]),
      period_us(period_us),
      duty_cycle(0),
      pulse_width_us(0) {
  this->pwm_pin->period_us(this->period_us);
}

void PWM::run() {
  if (*this->set_duty_cycle != this->duty_cycle) {
    // PWM duty has changed
    this->duty_cycle = *this->set_duty_cycle;
    this->pulse_width_us = (this->period_us * this->duty_cycle) / 1000;
    this->pwm_pin->pulsewidth_us(this->pulse_width_us);
  }
}
