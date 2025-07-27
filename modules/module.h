#ifndef MODULE_H
#define MODULE_H

#include "ArduinoJson.h"
#include "Comms.h"
#include "pin.h"
#include "pruThread.h"

// Module base class
// All modules are derived from this base class

class Module {
 protected:
  int32_t update_period;
  int32_t counter;

 public:
  Module();
  Module(int32_t thread_freq, int32_t slow_update_freq);

  virtual ~Module();
  void run();
  virtual void update();
  virtual void slow_update();
};

#endif
