#include "pwm.h"

PWM::PWM(const int var_number, Pin* pin, const int period_us, volatile rxData_t* rx_data)
    : pwm_pin(pin->as_output()->hardware_pwm()),
      set_duty_cycle(&rx_data->vars[var_number]),
      period_us(period_us),
      duty_cycle(0),
      pulse_width_us(0) {
  this->pwm_pin->period_us(this->period_us);
}

void PWM::update() {
  if (*(this->set_duty_cycle) != this->duty_cycle) {
    // PWM duty has changed
    this->duty_cycle = *this->set_duty_cycle;
    this->pulse_width_us = this->period_us * this->duty_cycle / 100;
    this->pwm_pin->pulsewidth_us(this->pulse_width_us);
  }
}
