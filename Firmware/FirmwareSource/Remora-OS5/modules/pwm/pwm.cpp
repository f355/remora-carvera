#include "pwm.h"
#include "hardwarePwm.h"


#define PID_PWM_MAX 256		// 8 bit resolution

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

Module* createPWM(JsonObject module, RemoraComms* comms)
{
    int sp = module["set_point"];
    int pwmMax = module["pwm_max"];
    const char* pin = module["pwm_pin"];

    bool hardware = module["use_hardware"];
    bool variable = module["variable_frequency"];
    int period_sp = module["period_set_point"];
    int period = module["period_us"];

    if (hardware)
    {
        // Hardware PWM
        if (variable)
        {
            // Variable frequency hardware PWM
            return new HardwarePWM(comms->ptrRxData->setPoint[period_sp], comms->ptrRxData->setPoint[sp], period, pin);
        }
        else
        {
            // Fixed frequency hardware PWM
            return new HardwarePWM(comms->ptrRxData->setPoint[sp], period, pin);
        }
    }
    else
    {
        // Software PWM
        if (pwmMax != 0) // use configuration file value for pwmMax - useful for 12V on 24V systems
        {
            return new PWM(comms->ptrRxData->setPoint[sp], pin, pwmMax);
        }
        else // use default value of pwmMax
        {
            return new PWM(comms->ptrRxData->setPoint[sp], pin);
        }
    }
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

PWM::PWM(volatile float &ptrSP, std::string portAndPin) :
	ptrSP(&ptrSP),
	portAndPin(portAndPin)
{
	this->pwm = new SoftPWM(this->portAndPin);
	this->pwmMax = PID_PWM_MAX-1;
	this->pwm->setMaxPwm(this->pwmMax);
}

// use the following constructor when using 12v devices on a 24v system
PWM::PWM(volatile float &ptrSP, std::string portAndPin, int pwmMax) :
	ptrSP(&ptrSP),
	portAndPin(portAndPin),
	pwmMax(pwmMax)
{
	this->pwm = new SoftPWM(this->portAndPin);
	this->pwm->setMaxPwm(this->pwmMax);
}



void PWM::update()
{
	float SP;

	// update the speed SP
	this->SP = *(this->ptrSP);

    // ensure SP is within range. LinuxCNC PID can have -ve command value
	if (this->SP > 100) this->SP = 100;
    if (this->SP < 0) this->SP = 0;

	// the SP is as a percentage (%)
	// scale the pwm output range (0 - pwmMax) = (0 - 100%)

	SP = this->pwmMax * (this->SP / 100.0);

	this->pwm->setPwmSP(int(SP));

	this->pwm->update();
}

void PWM::slowUpdate()
{
	return;
}
