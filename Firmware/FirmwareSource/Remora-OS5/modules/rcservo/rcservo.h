#ifndef RCSERVO_H
#define RCSERVO_H

#include "modules/module.h"

Module* createRCServo(JsonObject module, pruThread* thread, RemoraComms* comms);

class RCServo : public Module
{

	private:

		std::string pin;			// physical pin connection
		int threadFreq;				// thread frequency
		int T_ms;							// servo pulse period
		int T_compare;				// thread period counts compare for 20ms (50hz) pulses
		int t_compare;				// thread period counts compare for pulse period
		int counter;

		bool pinState;				// the state of the output pin

		volatile float *ptrPositionCmd; 	// pointer to the data source
		float positionCommand;	// the current servo position command



	public:

		Pin* servoPin;

		RCServo(volatile float&, std::string, int32_t, int32_t);

		virtual void update(void);	// Module default interface
		virtual void slowUpdate();
};

#endif
