#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include <vector>

#include "LPC17xx.h"
#include "lpcTimer.h"

using namespace std;

class Module;

class PRUThread final : public InterruptHandler {
  LPCTimer *timer;
  vector<Module *> modules;

 public:
  PRUThread(uint32_t timer_number, uint32_t frequency, uint32_t priority);

  uint32_t frequency;

  void register_module(Module *module);
  void start() const;
  void handle_interrupt() override;
};

#endif
