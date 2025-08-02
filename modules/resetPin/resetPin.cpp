#include "resetPin.h"

ResetPin::ResetPin(volatile bool& reset, Pin* pin) : reset(&reset), pin(pin->as_input()) {}

void ResetPin::run() { *this->reset = this->pin->get(); }
