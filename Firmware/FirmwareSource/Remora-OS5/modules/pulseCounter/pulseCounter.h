#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include "modules/module.h"

#include "mbed.h"


Module* createPulseCounter(JsonObject module, RemoraComms* comms);

class PulseCounter : public Module
{
	private:

		std::string pin;			// physical pin connection. must be interrupt-capable

		volatile float *ptrPulseCount; 	// pointer to the data source
		
		int8_t  modifier;
        volatile int32_t count;

		void interruptHandler();

	public:

		mbed::InterruptIn *interrupt;

		PulseCounter(volatile float&, std::string, int);

		virtual void update(void);	// Module default interface
};

#endif
