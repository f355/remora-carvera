#include "pwm.h"

PWM::PWM(const int var_number, const Pin* pin, const int period_us, volatile rxData_t* rx_data)
    : pwm_pin(), set_duty_cycle(&rx_data->output_vars[var_number]), period_us(period_us), duty_cycle(0) {
  pwmout_init(&this->pwm_pin, pin->to_pin_name());
  pwmout_period_us(&this->pwm_pin, this->period_us);
}

void PWM::run() {
  if (this->duty_cycle != *this->set_duty_cycle) {
    this->duty_cycle = *this->set_duty_cycle;
    pwmout_pulsewidth_us(&this->pwm_pin, this->period_us * this->duty_cycle / 1000);
  }
}
