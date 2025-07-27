#ifndef RESETPIN_H
#define RESETPIN_H

#include "module.h"

Module* createResetPin(JsonObject module, Comms* comms);

class ResetPin final : public Module {
  volatile bool* ptr_reset;  // pointer to the data source

  Pin* pin;

 public:
  ResetPin(volatile bool&, std::string);
  void update() override;
};

#endif
