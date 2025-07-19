#include "eStop.h"

eStop::eStop(volatile txData_t *ptr_tx_data, Pin *pin) : ptr_tx_data(ptr_tx_data), pin(pin->as_input()) {}

void eStop::update() { ptr_tx_data->e_stop_triggered = this->pin->get(); }
