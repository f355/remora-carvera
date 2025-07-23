#ifndef STEPGEN_H
#define STEPGEN_H

#include "module.h"

class Stepgen : public Module {
  volatile int32_t* ptr_commanded_frequency;  // pointer to the data source where to get the frequency command
  volatile int32_t* ptr_feedback;             // pointer where to put the feedback

  volatile uint8_t* ptr_joint_enable;
  int joint_enable_mask;

  bool last_dir;            // direction on last iteration, used for dir setup
  bool is_stepping{};       // true if the step pin is being held high
  int32_t step_count;       // current position since the system startup in steps (can be negative)
  int32_t accumulator;      // Direct Digital Synthesis (DDS) accumulator
  int32_t frequency_scale;  // frequency scale, 24.8 fixed-point

 public:
  Stepgen(int joint_number, Pin* step_pin, Pin* dir_pin, int32_t thread_frequency, volatile rxData_t* rx_data,
          volatile txData_t* tx_data);

  Pin *step_pin, *dir_pin;

  void update() override;
};

#endif
