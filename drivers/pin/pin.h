#ifndef PIN_H
#define PIN_H

#include "LPC17xx.h"
#include "mbed.h"

#define PIN_MODE_INPUT 0
#define PIN_MODE_OUTPUT 1

class Pin {
 public:
  Pin(unsigned char port, unsigned char pin);

  Pin* as_output() {
    this->port->FIODIR |= 1 << this->pin;
    return this;
  }

  Pin* as_input() {
    this->port->FIODIR &= ~(1 << this->pin);
    return this;
  }

  Pin* set_mode(const int mode) {
    if (mode == PIN_MODE_INPUT) {
      this->as_input();
    } else if (mode == PIN_MODE_OUTPUT) {
      this->as_output();
    } else {
      error("unknown pin mode %d\n", mode);
    }
    return this;
  }

  Pin* as_open_drain();
  Pin* pull_up();
  Pin* pull_down();
  Pin* pull_none();
  Pin* invert();

  bool get() const { return this->inverting ^ ((this->port->FIOPIN >> this->pin) & 1); }

  void set(const bool value) const {
    if (this->inverting ^ value)
      this->port->FIOSET = 1 << this->pin;
    else
      this->port->FIOCLR = 1 << this->pin;
  }

  PwmOut* hardware_pwm() const;

  InterruptIn* interrupt_pin();

  PinName to_pin_name() const;

 private:
  bool inverting;
  LPC_GPIO_TypeDef* port;

  unsigned char pin;
  unsigned char port_number;
};

#endif
