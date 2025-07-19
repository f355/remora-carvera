#include "resetPin.h"

ResetPin::ResetPin(volatile bool& ptr_reset, Pin* pin) : ptr_reset(&ptr_reset), pin(pin->as_input()) {}

void ResetPin::update() { *this->ptr_reset = this->pin->get(); }
