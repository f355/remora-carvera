#ifndef STEPGEN_H
#define STEPGEN_H

#include "module.h"

Module *createStepgen(JsonObject module, PRUThread *thread, Comms *comms);

class Stepgen : public Module {
  int enable_mask;

  volatile int32_t *ptr_commanded_frequency;  // pointer to the data source where to get the frequency command
  volatile int32_t *ptr_feedback;             // pointer where to put the feedback
  volatile uint8_t *ptr_joint_enable;

  bool last_dir;          // direction on last iteration, used for dir setup
  bool is_stepping;       // true if the step pin is held high
  int32_t step_count;     // current position raw count
  int32_t accumulator;    // Direct Digital Synthesis (DDS) accumulator
  float frequency_scale;  // frequency scale
  int32_t step_mask;      // value of the DDS accumulator that triggers a step pulse

 public:
  Stepgen(int32_t, int, std::string, std::string, int32_t, volatile int32_t &, volatile int32_t &,
          volatile uint8_t &);  // constructor

  Pin *step_pin, *direction_pin;

  void update() override;
};

#endif
