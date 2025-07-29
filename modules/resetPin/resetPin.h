#ifndef RESETPIN_H
#define RESETPIN_H

#include "module.h"
#include "pin.h"

class ResetPin final : public Module {
  volatile bool* ptr_reset;  // pointer to the data source

  Pin* pin;

 public:
  ResetPin(volatile bool& ptr_reset, Pin* pin);
  void update() override;
};

#endif
