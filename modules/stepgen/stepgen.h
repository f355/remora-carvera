#ifndef STEPGEN_H
#define STEPGEN_H

#include "module.h"
#include "pin.h"

class Stepgen : public Module {
  int joint_enable_mask;

  volatile int32_t *ptr_commanded_frequency;  // pointer to the data source where to get the frequency command
  volatile int32_t *ptr_feedback;             // pointer where to put the feedback
  volatile uint8_t *ptr_joint_enable;

  bool current_dir = false;  // direction on last iteration, used for dir setup
  bool is_stepping = false;  // true if the step pin is held high
  int32_t last_commanded_frequency = 0;
  int32_t increment = 0;
  int32_t accumulator = 0;  // Direct Digital Synthesis (DDS) accumulator
  uint32_t frequency_scale;

 public:
  Stepgen(int joint_number, Pin *step_pin, Pin *dir_pin, uint32_t thread_frequency, volatile rxData_t *rx_data,
          volatile txData_t *tx_data);

  Pin *step_pin, *dir_pin;

  void update() override;
};

#endif
