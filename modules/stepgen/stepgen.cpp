#include "stepgen.h"

#define STEP_BIT 22  // the bit location in DDS accum
#define STEP_MASK (1 << STEP_BIT)

Stepgen::Stepgen(const int joint_number, Pin* step_pin, Pin* dir_pin, const int32_t thread_frequency,
                 volatile rxData_t* rx_data, volatile txData_t* tx_data)
    : ptr_commanded_frequency(&rx_data->joint_freq_command[joint_number]),
      ptr_feedback(&tx_data->joint_feedback[joint_number]),
      ptr_joint_enable(&rx_data->joint_enable),
      joint_enable_mask(1 << joint_number),
      last_dir(true),
      step_count(0),
      accumulator(0),
      frequency_scale((STEP_MASK << FRACTIONAL_BITS) / thread_frequency),
      step_pin(step_pin->as_output()),
      dir_pin(dir_pin->as_output()) {}

void Stepgen::update() {
  if (this->is_stepping) {
    // bring down the step pin that was set high on the previous thread tick
    this->step_pin->set(false);
    this->is_stepping = false;
  }

  if ((*(this->ptr_joint_enable) & this->joint_enable_mask) == 0) {
    return;  // joint is disabled, nothing to do
  }

  // Direct Digital Synthesis (DDS)
  // works by incrementing an accumulator on every thread tick with a value calculated such that the accumulator
  // goes over a certain bit at the commanded frequency.
  //
  // frequency_scale is set to the increment that needs to be added to the accumulator on each thread tick
  // for the accumulator to reach that bit in one second at the thread tick frequency.
  //
  // by multiplying frequency_scale with the commanded frequency, we get the increment
  // that's needed to reach stepMask at that frequency.

  // napkin math:
  // do we need the fractional part of the commanded_frequency?
  // the minimum resolution across all axes is 640 steps/mm
  // the absolute minimum speed at which we'd want to move an axis is probably about 0.05 mm/sec or 32 steps/sec,
  // which is plenty enough to NOT care about the fractions
  //
  // do we need 64 bits when calculating the increments?
  // commanded_frequency  (max across all axes) = 888.889 steps/unit * 40 units/sec = 35 555 Hz
  // thread_frequency (minimum sensible to accommodate the above) = 80 000 Hz
  // frequency_scale = (1 << 8 << 22) / 80 000 = 13 421
  // increment (before shifting) = 13 421 * 35 555 = 477 183 655
  // bits required = log2(477 183 655) = 29
  // it fits in 29 bits, so absolutely no need for 64-bit conversions

  const int32_t increment = (*this->ptr_commanded_frequency * this->frequency_scale) >> FRACTIONAL_BITS;

  // save the old accumulator value and increment the accumulator
  int32_t stepNow = this->accumulator;
  this->accumulator += increment;
  // XOR the old and the new accumulator values to find the flipped bits
  stepNow ^= this->accumulator;
  // if the step bit has flipped, we need to drive the step pin
  stepNow &= STEP_MASK;

  // The sign of the increment indicates the desired direction
  bool isForward = (increment > 0);

  if (this->last_dir != isForward) {
    // Direction has changed, flip dir pin and do not step this iteration to give some setup time.
    // TODO: make hold time configurable.
    this->last_dir = isForward;
    // Set direction pin
    this->dir_pin->set(isForward);
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
