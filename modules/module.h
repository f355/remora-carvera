#ifndef MODULE_H
#define MODULE_H

#include "Comms.h"

// Module base class
// All modules are derived from this base class

class Module {
 protected:
  uint32_t update_period;
  uint32_t counter;

 public:
  Module();
  Module(uint32_t thread_freq, uint32_t slow_update_freq);

  virtual ~Module();
  void run();
  virtual void update();
  virtual void slow_update();
};

#endif
