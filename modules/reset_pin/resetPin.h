#ifndef RESETPIN_H
#define RESETPIN_H

#include "modules/module.h"

class ResetPin final : public Module {
  volatile bool* ptr_reset;  // pointer to the data source

  Pin* pin;

 public:
  ResetPin(volatile bool& ptr_reset, Pin* pin);
  void update() override;
};

#endif
