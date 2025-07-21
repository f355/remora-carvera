#include "MachineThread.h"

#include "modules/module.h"

using namespace std;

// Thread constructor
MachineThread::MachineThread(const uint8_t timer_number, const int32_t frequency, const uint32_t priority)
    : frequency(frequency) {
  this->timer = lpc_timers[timer_number];
  this->timer->configure(this, this->frequency, priority);
}

void MachineThread::start() const { this->timer->start(); }

void MachineThread::register_module(Module* module) {
  if (module->needs_periodic_update) this->modules.push_back(module);
}

void MachineThread::handle_interrupt() {
  // iterate over the Thread pointer vector to run all instances of Module::runModule()
  for (const auto m : modules) m->run();
}
