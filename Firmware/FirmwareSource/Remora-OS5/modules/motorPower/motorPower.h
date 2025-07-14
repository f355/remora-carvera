#ifndef MOTORPOWER_H
#define MOTORPOWER_H

#include <cstdint>

#include "modules/module.h"
#include "drivers/pin/pin.h"

void createMotorPower(JsonObject module);

class MotorPower : public Module
{
	private:

		std::string portAndPin;

		Pin *pin;

	public:

        MotorPower(std::string);
		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
