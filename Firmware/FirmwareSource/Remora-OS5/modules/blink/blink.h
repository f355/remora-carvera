#ifndef BLINK_H
#define BLINK_H

#include "modules/module.h"

Module* createBlink(JsonObject module, PRUThread* thread);

class Blink : public Module
{

	private:

		bool 		bState;
		uint32_t 	periodCount;
		uint32_t 	blinkCount;

		Pin *blinkPin;	// class object members - Pin objects

	public:

		Blink(std::string, uint32_t, uint32_t);

		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
