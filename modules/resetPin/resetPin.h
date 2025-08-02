#ifndef RESETPIN_H
#define RESETPIN_H

#include "module.h"
#include "pin.h"

class ResetPin final : public Module {
  volatile bool* reset;  // pointer to the data source

  Pin* pin;

 public:
  ResetPin(volatile bool& reset, Pin* pin);
  void run() override;
};

#endif
