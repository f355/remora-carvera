#include "pwm.h"

Module* createPWM(JsonObject module, Comms* comms)
{
    int sp = module["set_point"];
    const char* pin = module["pwm_pin"];

    bool variable = module["variable_frequency"];
    int period_sp = module["period_set_point"];
    int period = module["period_us"];

    if (variable)
    {
        // Variable frequency hardware PWM
        return new PWM(comms->ptrRxData->setPoint[period_sp], comms->ptrRxData->setPoint[sp], period, pin);
    }
    else
    {
        // Fixed frequency hardware PWM
        return new PWM(comms->ptrRxData->setPoint[sp], period, pin);
    }
}

#define PWMPERIOD 200

PWM::PWM(volatile float &ptrPwmPulseWidth, int pwmPeriod, std::string pin) :
    ptrPwmPulseWidth(&ptrPwmPulseWidth),
    pwmPeriod(pwmPeriod)
{
    this->variablePeriod = false;

    if (pwmPeriod == 0)
    {
        this->pwmPeriod = PWMPERIOD;
    }

    this->pwmPin = (new Pin(pin))->as_output()->hardware_pwm();
    this->pwmPin->period_us(this->pwmPeriod);
}


PWM::PWM(volatile float &ptrPwmPeriod, volatile float &ptrPwmPulseWidth, int pwmPeriod, std::string pin) :
    PWM(ptrPwmPulseWidth, pwmPeriod, pin)
{
    variablePeriod = true;
    this->ptrPwmPeriod = &ptrPwmPeriod;
}


void PWM::update()
{
    if (variablePeriod)
    {
        if (*(this->ptrPwmPeriod) != 0 && (*(this->ptrPwmPeriod) != this->pwmPeriod))
        {
            // PWM period has changed
            this->pwmPeriod = *(this->ptrPwmPeriod);
            this->pwmPin->period_us(this->pwmPeriod);
            this->pwmPulseWidth_us = (this->pwmPeriod * this->pwmPulseWidth) / 100.0;
            this->pwmPin->pulsewidth_us(this->pwmPulseWidth_us);
        }
    }

    if (*(this->ptrPwmPulseWidth) != this->pwmPulseWidth)
    {
        // PWM duty has changed
        this->pwmPulseWidth = *(this->ptrPwmPulseWidth);
        this->pwmPulseWidth_us = (this->pwmPeriod * this->pwmPulseWidth) / 100.0;
        this->pwmPin->pulsewidth_us(this->pwmPulseWidth_us);
    }
}
