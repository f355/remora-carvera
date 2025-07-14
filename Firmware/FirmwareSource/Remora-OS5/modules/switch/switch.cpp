#include "switch.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

Module* createSwitch(JsonObject module, RemoraComms* comms)
{
    const char* pin = module["pin"];
    const char* mode = module["mode"];
    int pv = module["process_variable"];
    float sp = module["set_point"];

    if (!strcmp(mode,"on"))
    {
        return new Switch(sp, comms->ptrTxData->processVariable[pv], pin, 1);
    }
    else if (!strcmp(mode,"off"))
    {
        return new Switch(sp, comms->ptrTxData->processVariable[pv], pin, 0);
    }
    else
    {
        error("Error - incorrectly defined Switch\n");
    }
}

Switch::Switch(float SP, volatile float &ptrPV, std::string portAndPin, bool mode) :
	SP(SP),
	ptrPV(&ptrPV),
	portAndPin(portAndPin),
	mode(mode)
{
	int output = 0x1; // an output
	this->pin = new Pin(this->portAndPin, output);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

void Switch::update()
{
	bool pinState;

	pinState = this->mode;

	// update the SP
	this->PV = *(this->ptrPV);

	if (this->PV > this->SP)
	{
		this->pin->set(pinState);
	}
	else
	{
		pinState = !pinState;
		this->pin->set(pinState);
	}

}


void Switch::slowUpdate()
{
	return;
}
