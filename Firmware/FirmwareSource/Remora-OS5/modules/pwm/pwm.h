#ifndef PWM_H
#define PWM_H

#include "modules/module.h"
#include "drivers/softPwm/softPwm.h"

Module* createPWM(JsonObject module, RemoraComms* comms);

class PWM : public Module
{

	private:

		volatile float* ptrSP; 			// pointer to the data source
		int 			SP;
		std::string 	portAndPin;
		int 			pwmMax;

		SoftPWM* 		pwm;			// pointer to PWM object - output


	public:

		PWM(volatile float&, std::string);
		PWM(volatile float&, std::string, int);

		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
