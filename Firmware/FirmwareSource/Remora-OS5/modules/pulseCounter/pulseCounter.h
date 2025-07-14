#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include <cstdint>
#include <iostream>
#include <string>

#include "modules/module.h"
#include "drivers/pin/pin.h"

#include "mbed.h"
#include "port_api.h"
#include "LPC17xx.h"


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
