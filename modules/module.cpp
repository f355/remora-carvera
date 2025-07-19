#include "module.h"

Module::Module() : thread_freq(0), slow_update_freq(0) {
  this->counter = 0;
  this->update_count = 1;
}

Module::Module(const uint32_t thread_freq, const uint32_t slow_update_freq)
    : thread_freq(thread_freq),
      slow_update_freq(slow_update_freq),
      update_count(thread_freq / slow_update_freq),
      counter(0) {
  this->counter = 0;
  this->update_count = this->thread_freq / this->slow_update_freq;
}

void Module::run() {
  ++this->counter;

  if (this->counter >= this->update_count) {
    this->slow_update();
    this->counter = 0;
  }

  this->update();
}

void Module::update() {}

void Module::slow_update() {}
