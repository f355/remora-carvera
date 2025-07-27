#include "mbed.h"
#include "port_api.h"
#include "pin.h"
#include <cstdio>
#include <cerrno>
#include <string>

#include "LPC17xx.h"

// taken from Smoothieware https://smoothieware.org/

Pin::Pin()
{
    this->inverting = false;
    this->valid = false;
    this->pin = 32;
    this->port = nullptr;
}

Pin::Pin(std::string portAndPin): Pin()
{
    this->from_string(portAndPin);
}

// Make a new pin object from a string
Pin* Pin::from_string(std::string value)
{
    if(value == "nc") {
        this->valid= false;
        return this; // optimize the nc case
    }

    LPC_GPIO_TypeDef* gpios[5] = { LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3, LPC_GPIO4 };

    // cs is the current position in the string
    const char* cs = value.c_str();
    // cn is the position of the next char after the number we just read
    char* cn = NULL;
    valid = true;

    // grab first integer as port. pointer to first non-digit goes in cn
    this->port_number = strtol(cs, &cn, 10);
    // if cn > cs then strtol read at least one digit
    if ((cn > cs) && (port_number <= 4)){
        // translate port index into something useful
        this->port = gpios[(unsigned int) this->port_number];
        // if the char after the first integer is a . then we should expect a pin index next
        if (*cn == '.'){
            // move pointer to first digit (hopefully) of pin index
            cs = ++cn;

            // grab pin index.
            this->pin = strtol(cs, &cn, 10);

            // if strtol read some numbers, cn will point to the first non-digit
            if ((cn > cs) && (pin < 32)){
                this->port->FIOMASK &= ~(1 << this->pin);

                // now check for modifiers:-
                // ! = invert pin
                // o = set pin to open drain
                // ^ = set pin to pull up
                // v = set pin to pull down
                // - = set pin to no pull up or down
                for (; *cn; cn++) {
                    switch(*cn) {
                        case '!':
                            this->inverting = true;
                            break;
                        case 'o':
                            this->as_open_drain();
                            break;
                        case '^':
                            this->pull_up();
                            break;
                        case 'v':
                            this->pull_down();
                            break;
                        case '-':
                            this->pull_none();
                            break;
                        default:
                            // skip any whitespace following the pin index
                            if ((*cn == ' ') || (*cn == '\t'))
                                return this;
                    }
                }
                return this;
            }
        }
    }

    // from_string failed. TODO: some sort of error
    valid= false;
    port_number = 0;
    port = gpios[0];
    pin = 32;
    inverting = false;
    return this;
}

// Configure this pin as OD
Pin* Pin::as_open_drain(){
    if (!this->valid) return this;
    if( this->port_number == 0 ){ LPC_PINCON->PINMODE_OD0 |= (1<<this->pin); }
    if( this->port_number == 1 ){ LPC_PINCON->PINMODE_OD1 |= (1<<this->pin); }
    if( this->port_number == 2 ){ LPC_PINCON->PINMODE_OD2 |= (1<<this->pin); }
    if( this->port_number == 3 ){ LPC_PINCON->PINMODE_OD3 |= (1<<this->pin); }
    if( this->port_number == 4 ){ LPC_PINCON->PINMODE_OD4 |= (1<<this->pin); }
    pull_none(); // no pull up by default
    return this;
}

// Configure this pin as no pullup or pulldown
Pin* Pin::pull_none(){
    if (!this->valid) return this;
    // Set the two bits for this pin as 10
    if( this->port_number == 0 && this->pin < 16  ){ LPC_PINCON->PINMODE0 |= (2<<( this->pin*2)); LPC_PINCON->PINMODE0 &= ~(1<<( this->pin    *2)); }
    if( this->port_number == 0 && this->pin >= 16 ){ LPC_PINCON->PINMODE1 |= (2<<( this->pin*2)); LPC_PINCON->PINMODE1 &= ~(1<<((this->pin-16)*2)); }
    if( this->port_number == 1 && this->pin < 16  ){ LPC_PINCON->PINMODE2 |= (2<<( this->pin*2)); LPC_PINCON->PINMODE2 &= ~(1<<( this->pin    *2)); }
    if( this->port_number == 1 && this->pin >= 16 ){ LPC_PINCON->PINMODE3 |= (2<<( this->pin*2)); LPC_PINCON->PINMODE3 &= ~(1<<((this->pin-16)*2)); }
    if( this->port_number == 2 && this->pin < 16  ){ LPC_PINCON->PINMODE4 |= (2<<( this->pin*2)); LPC_PINCON->PINMODE4 &= ~(1<<( this->pin    *2)); }
    if( this->port_number == 3 && this->pin >= 16 ){ LPC_PINCON->PINMODE7 |= (2<<( this->pin*2)); LPC_PINCON->PINMODE7 &= ~(1<<((this->pin-16)*2)); }
    if( this->port_number == 4 && this->pin >= 16 ){ LPC_PINCON->PINMODE9 |= (2<<( this->pin*2)); LPC_PINCON->PINMODE9 &= ~(1<<((this->pin-16)*2)); }
    return this;
}

// Configure this pin as a pullup
Pin* Pin::pull_up(){
    if (!this->valid) return this;
    // Set the two bits for this pin as 00
    if( this->port_number == 0 && this->pin < 16  ){ LPC_PINCON->PINMODE0 &= ~(3<<( this->pin    *2)); }
    if( this->port_number == 0 && this->pin >= 16 ){ LPC_PINCON->PINMODE1 &= ~(3<<((this->pin-16)*2)); }
    if( this->port_number == 1 && this->pin < 16  ){ LPC_PINCON->PINMODE2 &= ~(3<<( this->pin    *2)); }
    if( this->port_number == 1 && this->pin >= 16 ){ LPC_PINCON->PINMODE3 &= ~(3<<((this->pin-16)*2)); }
    if( this->port_number == 2 && this->pin < 16  ){ LPC_PINCON->PINMODE4 &= ~(3<<( this->pin    *2)); }
    if( this->port_number == 3 && this->pin >= 16 ){ LPC_PINCON->PINMODE7 &= ~(3<<((this->pin-16)*2)); }
    if( this->port_number == 4 && this->pin >= 16 ){ LPC_PINCON->PINMODE9 &= ~(3<<((this->pin-16)*2)); }
    return this;
}

// Configure this pin as a pulldown
Pin* Pin::pull_down(){
    if (!this->valid) return this;
    // Set the two bits for this pin as 11
    if( this->port_number == 0 && this->pin < 16  ){ LPC_PINCON->PINMODE0 |= (3<<( this->pin    *2)); }
    if( this->port_number == 0 && this->pin >= 16 ){ LPC_PINCON->PINMODE1 |= (3<<((this->pin-16)*2)); }
    if( this->port_number == 1 && this->pin < 16  ){ LPC_PINCON->PINMODE2 |= (3<<( this->pin    *2)); }
    if( this->port_number == 1 && this->pin >= 16 ){ LPC_PINCON->PINMODE3 |= (3<<((this->pin-16)*2)); }
    if( this->port_number == 2 && this->pin < 16  ){ LPC_PINCON->PINMODE4 |= (3<<( this->pin    *2)); }
    if( this->port_number == 3 && this->pin >= 16 ){ LPC_PINCON->PINMODE7 |= (3<<((this->pin-16)*2)); }
    if( this->port_number == 4 && this->pin >= 16 ){ LPC_PINCON->PINMODE9 |= (3<<((this->pin-16)*2)); }
    return this;
}

// Return mbed hardware pwm class for this pin
mbed::PwmOut* Pin::hardware_pwm()
{
    if (port_number == 1)
    {
        if (pin == 18) { return new mbed::PwmOut(P1_18); }
        if (pin == 20) { return new mbed::PwmOut(P1_20); }
        if (pin == 21) { return new mbed::PwmOut(P1_21); }
        if (pin == 23) { return new mbed::PwmOut(P1_23); }
        if (pin == 24) { return new mbed::PwmOut(P1_24); }
        if (pin == 26) { return new mbed::PwmOut(P1_26); }
    }
    else if (port_number == 2)
    {
        if (pin == 0) { return new mbed::PwmOut(P2_0); }
        if (pin == 1) { return new mbed::PwmOut(P2_1); }
        if (pin == 2) { return new mbed::PwmOut(P2_2); }
        if (pin == 3) { return new mbed::PwmOut(P2_3); }
        if (pin == 4) { return new mbed::PwmOut(P2_4); }
        if (pin == 5) { return new mbed::PwmOut(P2_5); }
    }
    else if (port_number == 3)
    {
        if (pin == 25) { return new mbed::PwmOut(P3_25); }
        if (pin == 26) { return new mbed::PwmOut(P3_26); }
    }
    error("Pin is not PWM-capable!");
}

mbed::InterruptIn* Pin::interrupt_pin()
{
    if(!this->valid) return nullptr;

    // set as input
    as_input();

    if (port_number == 0 || port_number == 2) {
        PinName pinname = this->to_pin_name();
        return new mbed::InterruptIn(pinname);

    } else {
        error("Pin is not interrupt-capable!");
    }
}

PinName Pin::to_pin_name() {
    return port_pin((PortName)port_number, pin);
}
