#include "pin.h"

#include "port_api.h"

Pin::Pin(const unsigned char port, const unsigned char pin) : inverting(false), pin(pin), port_number(port) {
  LPC_GPIO_TypeDef* gpios[5] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3, LPC_GPIO4};
  this->port = gpios[port];
  this->port->FIOMASK &= ~(1 << this->pin);
}

// Configure this pin as OD
Pin* Pin::as_open_drain() {
  if (this->port_number == 0) {
    LPC_PINCON->PINMODE_OD0 |= (1 << this->pin);
  }
  if (this->port_number == 1) {
    LPC_PINCON->PINMODE_OD1 |= (1 << this->pin);
  }
  if (this->port_number == 2) {
    LPC_PINCON->PINMODE_OD2 |= (1 << this->pin);
  }
  if (this->port_number == 3) {
    LPC_PINCON->PINMODE_OD3 |= (1 << this->pin);
  }
  if (this->port_number == 4) {
    LPC_PINCON->PINMODE_OD4 |= (1 << this->pin);
  }
  pull_none();  // no pull up by default
  return this;
}

// Configure this pin as no pullup or pulldown
Pin* Pin::pull_none() {
  // Set the two bits for this pin as 10
  if (this->port_number == 0 && this->pin < 16) {
    LPC_PINCON->PINMODE0 |= (2 << (this->pin * 2));
    LPC_PINCON->PINMODE0 &= ~(1 << (this->pin * 2));
  }
  if (this->port_number == 0 && this->pin >= 16) {
    LPC_PINCON->PINMODE1 |= (2 << (this->pin * 2));
    LPC_PINCON->PINMODE1 &= ~(1 << ((this->pin - 16) * 2));
  }
  if (this->port_number == 1 && this->pin < 16) {
    LPC_PINCON->PINMODE2 |= (2 << (this->pin * 2));
    LPC_PINCON->PINMODE2 &= ~(1 << (this->pin * 2));
  }
  if (this->port_number == 1 && this->pin >= 16) {
    LPC_PINCON->PINMODE3 |= (2 << (this->pin * 2));
    LPC_PINCON->PINMODE3 &= ~(1 << ((this->pin - 16) * 2));
  }
  if (this->port_number == 2 && this->pin < 16) {
    LPC_PINCON->PINMODE4 |= (2 << (this->pin * 2));
    LPC_PINCON->PINMODE4 &= ~(1 << (this->pin * 2));
  }
  if (this->port_number == 3 && this->pin >= 16) {
    LPC_PINCON->PINMODE7 |= (2 << (this->pin * 2));
    LPC_PINCON->PINMODE7 &= ~(1 << ((this->pin - 16) * 2));
  }
  if (this->port_number == 4 && this->pin >= 16) {
    LPC_PINCON->PINMODE9 |= (2 << (this->pin * 2));
    LPC_PINCON->PINMODE9 &= ~(1 << ((this->pin - 16) * 2));
  }
  return this;
}

// Configure this pin as a pullup
Pin* Pin::pull_up() {
  // Set the two bits for this pin as 00
  if (this->port_number == 0 && this->pin < 16) {
    LPC_PINCON->PINMODE0 &= ~(3 << (this->pin * 2));
  }
  if (this->port_number == 0 && this->pin >= 16) {
    LPC_PINCON->PINMODE1 &= ~(3 << ((this->pin - 16) * 2));
  }
  if (this->port_number == 1 && this->pin < 16) {
    LPC_PINCON->PINMODE2 &= ~(3 << (this->pin * 2));
  }
  if (this->port_number == 1 && this->pin >= 16) {
    LPC_PINCON->PINMODE3 &= ~(3 << ((this->pin - 16) * 2));
  }
  if (this->port_number == 2 && this->pin < 16) {
    LPC_PINCON->PINMODE4 &= ~(3 << (this->pin * 2));
  }
  if (this->port_number == 3 && this->pin >= 16) {
    LPC_PINCON->PINMODE7 &= ~(3 << ((this->pin - 16) * 2));
  }
  if (this->port_number == 4 && this->pin >= 16) {
    LPC_PINCON->PINMODE9 &= ~(3 << ((this->pin - 16) * 2));
  }
  return this;
}

// Configure this pin as a pulldown
Pin* Pin::pull_down() {
  // Set the two bits for this pin as 11
  if (this->port_number == 0 && this->pin < 16) {
    LPC_PINCON->PINMODE0 |= (3 << (this->pin * 2));
  }
  if (this->port_number == 0 && this->pin >= 16) {
    LPC_PINCON->PINMODE1 |= (3 << ((this->pin - 16) * 2));
  }
  if (this->port_number == 1 && this->pin < 16) {
    LPC_PINCON->PINMODE2 |= (3 << (this->pin * 2));
  }
  if (this->port_number == 1 && this->pin >= 16) {
    LPC_PINCON->PINMODE3 |= (3 << ((this->pin - 16) * 2));
  }
  if (this->port_number == 2 && this->pin < 16) {
    LPC_PINCON->PINMODE4 |= (3 << (this->pin * 2));
  }
  if (this->port_number == 3 && this->pin >= 16) {
    LPC_PINCON->PINMODE7 |= (3 << ((this->pin - 16) * 2));
  }
  if (this->port_number == 4 && this->pin >= 16) {
    LPC_PINCON->PINMODE9 |= (3 << ((this->pin - 16) * 2));
  }
  return this;
}

Pin* Pin::invert() {
  this->inverting = true;
  return this;
}

// Return mbed hardware pwm class for this pin
PwmOut* Pin::hardware_pwm() const {
  switch (this->port_number) {
    case 1:
      if (pin == 18) return new PwmOut(P1_18);
      if (pin == 20) return new PwmOut(P1_20);
      if (pin == 21) return new PwmOut(P1_21);
      if (pin == 23) return new PwmOut(P1_23);
      if (pin == 24) return new PwmOut(P1_24);
      if (pin == 26) return new PwmOut(P1_26);
      break;
    case 2:
      if (pin == 0) return new PwmOut(P2_0);
      if (pin == 1) return new PwmOut(P2_1);
      if (pin == 2) return new PwmOut(P2_2);
      if (pin == 3) return new PwmOut(P2_3);
      if (pin == 4) return new PwmOut(P2_4);
      if (pin == 5) return new PwmOut(P2_5);
      break;
    case 3:
      if (pin == 25) return new PwmOut(P3_25);
      if (pin == 26) return new PwmOut(P3_26);
      break;
    default:;
  }
  error("Pin %d.%d is not PWM-capable!", this->port_number, this->pin);
}

InterruptIn* Pin::interrupt_pin() {
  as_input();
  if (port_number != 0 && port_number != 2) {
    error("Pin is not interrupt-capable!");
  }
  return new InterruptIn(this->to_pin_name());
}

PinName Pin::to_pin_name() const { return port_pin(static_cast<PortName>(port_number), pin); }
