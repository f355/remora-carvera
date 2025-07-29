#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include "module.h"
#include "pin.h"

class InputPin final : public Module {
  volatile uint16_t* inputs;
  uint16_t mask;
  Pin* pin;

 public:
  InputPin(int bit_number, Pin* pin, volatile txData_t* tx_data);

  void update() override;
};

class OutputPin final : public Module {
  volatile uint16_t* outputs;
  uint16_t mask;
  Pin* pin;

 public:
  OutputPin(int bit_number, Pin* pin, volatile rxData_t* rx_data);

  void update() override;
};

#endif
