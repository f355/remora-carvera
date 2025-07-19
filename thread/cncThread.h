#ifndef CNCTHREAD_H
#define CNCTHREAD_H

#include <iostream>
#include <vector>

#include "lpcTimer.h"

using namespace std;

class Module;

class CNCThread final : public InterruptHandler {
  LPCTimer *timer;
  vector<Module *> modules;

 public:
  CNCThread(uint8_t timer_number, int32_t frequency, uint32_t priority);

  int32_t frequency;

  void register_module(Module *module);
  void start() const;
  void handle_interrupt() override;
};

#endif
