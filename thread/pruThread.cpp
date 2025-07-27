#include "PRUThread.h"

#include "module.h"

using namespace std;

// Thread constructor
PRUThread::PRUThread(const uint32_t timer_number, const uint32_t frequency, const uint32_t priority)
    : frequency(frequency) {
  this->timer = lpc_timers[timer_number];
  this->timer->configure(this, this->frequency, priority);
}

void PRUThread::start() const { this->timer->start(); }

void PRUThread::register_module(Module* module) { this->modules.push_back(module); }

void PRUThread::handle_interrupt() {
  for (const auto module : modules) module->run();
}
