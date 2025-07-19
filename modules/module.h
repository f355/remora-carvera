#ifndef MODULE_H
#define MODULE_H

#include <cstdint>
#include <string>

#include "drivers/comms/comms.h"
#include "drivers/pin/pin.h"
#include "thread/cncThread.h"

// Module base class
// All modules are derived from this base class

class Module {
 protected:
  uint32_t thread_freq;
  uint32_t slow_update_freq;
  uint32_t update_count;
  uint32_t counter;

 public:
  Module();
  Module(uint32_t thread_freq, uint32_t slow_update_freq);
  virtual ~Module() = default;

  bool needs_periodic_update = true;

  void run();
  virtual void update();
  virtual void slow_update();
};

#endif
