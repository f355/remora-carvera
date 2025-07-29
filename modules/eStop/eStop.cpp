#include "eStop.h"

eStop::eStop(volatile txData_t *ptr_tx_data, Pin *pin) : ptr_tx_data(ptr_tx_data), pin(pin->as_input()) {}

void eStop::update() {
  if (this->pin->get()) {
    ptr_tx_data->header = PRU_ESTOP;
  } else {
    ptr_tx_data->header = PRU_DATA;
  }
}
