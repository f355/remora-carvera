#include "module.h"

#include <cstdio>

#include "pin.h"

Module::Module() : update_period(0), counter(0) {}

Module::Module(const int32_t thread_freq, const int32_t slow_update_freq)
    : update_period(thread_freq / slow_update_freq), counter(0) {
  printf("\nCreating a slower module, updating every %d thread cycles\n", this->update_period);
}

Module::~Module() = default;

void Module::run() {
  ++this->counter;

  if (this->counter >= this->update_period) {
    this->slow_update();
    this->counter = 0;
  }

  this->update();
}

void Module::update() {}
void Module::slow_update() {}
