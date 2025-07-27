#include "pwm.h"

Module *createPWM(JsonObject module, Comms *comms) {
  int sp = module["set_point"];
  const char *pin = module["pwm_pin"];

  bool variable = module["variable_frequency"];
  int period_sp = module["period_set_point"];
  int period = module["period_us"];

  if (variable) {
    // Variable frequency hardware PWM
    return new PWM(comms->ptr_rx_data->set_point[period_sp], comms->ptr_rx_data->set_point[sp], period, pin);
  } else {
    // Fixed frequency hardware PWM
    return new PWM(comms->ptr_rx_data->set_point[sp], period, pin);
  }
}

#define PWMPERIOD 200

PWM::PWM(volatile float &ptrPwmPulseWidth, int pwmPeriod, std::string pin)
    : ptrPwmPulseWidth(&ptrPwmPulseWidth), period_us(pwmPeriod) {
  this->variablePeriod = false;

  if (pwmPeriod == 0) {
    this->period_us = PWMPERIOD;
  }

  this->pwm_pin = (new Pin(pin))->as_output()->hardware_pwm();
  this->pwm_pin->period_us(this->period_us);
}

PWM::PWM(volatile float &ptrPwmPeriod, volatile float &ptrPwmPulseWidth, int pwmPeriod, std::string pin)
    : PWM(ptrPwmPulseWidth, pwmPeriod, pin) {
  variablePeriod = true;
  this->set_duty_cycle = &ptrPwmPeriod;
}

void PWM::update() {
  if (variablePeriod) {
    if (*(this->set_duty_cycle) != 0 && (*(this->set_duty_cycle) != this->period_us)) {
      // PWM period has changed
      this->period_us = *(this->set_duty_cycle);
      this->pwm_pin->period_us(this->period_us);
      this->pulse_width_us = (this->period_us * this->duty_cycle) / 100.0;
      this->pwm_pin->pulsewidth_us(this->pulse_width_us);
    }
  }

  if (*(this->ptrPwmPulseWidth) != this->duty_cycle) {
    // PWM duty has changed
    this->duty_cycle = *(this->ptrPwmPulseWidth);
    this->pulse_width_us = (this->period_us * this->duty_cycle) / 100.0;
    this->pwm_pin->pulsewidth_us(this->pulse_width_us);
  }
}
