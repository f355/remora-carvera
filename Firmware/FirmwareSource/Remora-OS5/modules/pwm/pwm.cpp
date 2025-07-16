#include "pwm.h"

#define PWMPERIOD 200

PWM::PWM(int setPoint, Pin* pin, int period, volatile rxData_t* rxData) :
    ptrPwmPulseWidth(&rxData->setPoint[setPoint]),
    pwmPeriod(period),
    variablePeriod(false)
{
    if (pwmPeriod == 0)
    {
        this->pwmPeriod = PWMPERIOD;
    }

    this->pwmPin = pin->as_output()->hardware_pwm();
    this->pwmPin->period_us(this->pwmPeriod);
}


PWM::PWM(int setPoint, int setPointPeriod, Pin* pin, int period, volatile rxData_t* rxData) :
    PWM(setPoint, pin, period, rxData)
{
    this->variablePeriod = true;
    this->ptrPwmPeriod = &rxData->setPoint[setPointPeriod];
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
