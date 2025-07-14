#include "motorPower.h"

void createMotorPower(JsonObject module)
{
    const char* pin = module["pin"];

    Module* motPower = new MotorPower(pin);
    delete motPower;
}


MotorPower::MotorPower(std::string portAndPin) :
    portAndPin(portAndPin)
{
    this->pin = new Pin(this->portAndPin, 0x1); // Input 0x0, Output 0x1
    this->update();
}


void MotorPower::update()
{
    this->pin->set(true);
}

void MotorPower::slowUpdate()
{
    return;
}
