#ifndef ESTOP_H
#define ESTOP_H

#include "module.h"

class eStop final : public Module {
  volatile txData_t *ptr_tx_data;
  Pin *pin;

 public:
  eStop(volatile txData_t *ptr_tx_data, Pin *pin);

  void update() override;
};

#endif
