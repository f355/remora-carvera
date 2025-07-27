#include "motorPower.h"

void createMotorPower(JsonObject module)
{
    const char* pin = module["pin"];

    (new Pin(pin))->as_output()->set(true);
}
