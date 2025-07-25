#include "digital_pin.h"

InputPin::InputPin(const int bit_number, Pin* pin, volatile txData_t* tx_data)
    : inputs(&tx_data->inputs), mask(1 << bit_number), pin(pin->as_input()) {}

void InputPin::update() {
  if (this->pin->get()) {
    *this->inputs |= this->mask;
  } else {
    *this->inputs &= ~this->mask;
  }
}

OutputPin::OutputPin(const int bit_number, Pin* pin, volatile rxData_t* rx_data)
    : outputs(&rx_data->outputs), mask(1 << bit_number), pin(pin->as_output()) {}

void OutputPin::update() { this->pin->set(*this->outputs & this->mask); }
