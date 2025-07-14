#ifndef MOTORPOWER_H
#define MOTORPOWER_H

#include "modules/module.h"

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
