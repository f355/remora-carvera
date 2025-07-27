#include "stepgen.h"

Module *createStepgen(JsonObject module, PRUThread *thread, Comms *comms) {
  int joint = module["joint"];
  const char *step = module["step_pin"];
  const char *dir = module["direction_pin"];

  return new Stepgen(thread->frequency, joint, step, dir, STEP_MASK, comms->ptr_rx_data->joint_freq_command[joint],
                     comms->ptr_tx_data->joint_feedback[joint], comms->ptr_rx_data->joint_enable);
}

Stepgen::Stepgen(int32_t threadFreq, int jointNumber, std::string step, std::string direction, int32_t stepMask,
                 volatile int32_t &ptrFrequencyCommand, volatile int32_t &ptrFeedback, volatile uint8_t &ptrJointEnable)
    : ptr_commanded_frequency(&ptrFrequencyCommand),
      ptr_feedback(&ptrFeedback),
      ptr_joint_enable(&ptrJointEnable),
      step_mask(stepMask) {
  this->step_pin = (new Pin(step))->as_output();
  this->direction_pin = (new Pin(direction))->as_output();
  this->accumulator = 0;
  this->step_count = 0;
  this->frequency_scale = (float)this->step_mask / (float)threadFreq;
  this->enable_mask = 1 << jointNumber;
  this->last_dir = true;
}

void Stepgen::update() {
  if (this->is_stepping) {
    // bring down the step pin that was set high on the previous thread tick
    this->step_pin->set(false);
    this->is_stepping = false;
  }

  if ((*(this->ptr_joint_enable) & this->enable_mask) == 0) {
    return;  // joint is disabled, nothing to do
  }

  // Direct Digital Synthesis (DDS)
  // works by incrementing an accumulator on every thread tick with a value calculated such that the accumulator
  // goes over a certain bit at the commanded frequency.
  //
  // frequencyScale is set to the increment that needs to be added to the accumulator on each thread tick
  // for the accumulator to reach that bit in one second at the thread tick frequency.
  //
  // by multiplying frequencyScale with the commanded frequency, we get the increment
  // that's needed to reach stepMask at that frequency.
  int32_t increment = (*(this->ptr_commanded_frequency)) * this->frequency_scale;

  // save the old accumulator value and increment the accumulator
  int32_t stepNow = this->accumulator;
  this->accumulator += increment;
  // XOR the old and the new accumulator values to find the flipped bits
  stepNow ^= this->accumulator;
  // if the step bit has flipped, we need to drive the step pin
  stepNow &= this->step_mask;

  // The sign of the increment indicates the desired direction
  bool isForward = (increment > 0);

  if (this->last_dir != isForward) {
    // Direction has changed, flip dir pin and do not step this iteration to
    // give some setup time.
    // TODO: make hold time configurable.
    this->last_dir = isForward;
    // Set direction pin
    this->direction_pin->set(isForward);
    return;
  }

  if (stepNow) {
    // make a step
    this->step_pin->set(true);
    this->is_stepping = true;
    if (isForward) {
      this->step_count++;
    } else {
      this->step_count--;
    }
    *(this->ptr_feedback) = this->step_count;
  }
}
