#ifndef PIN_H
#define PIN_H

#include "mbed.h"

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <string>

#include "LPC17xx.h"

#define PIN_MODE_INPUT 0
#define PIN_MODE_OUTPUT 1

class Pin {
    public:
        Pin();
        Pin(std::string portAndPin);

        Pin* from_string(std::string value);

        inline bool connected(){
            return this->valid;
        }

        inline bool equals(const Pin& other) const {
            return (this->pin == other.pin) && (this->port == other.port);
        }

        inline Pin* as_output(){
            if (this->valid)
                this->port->FIODIR |= 1<<this->pin;
            return this;
        }

        inline Pin* as_input(){
            if (this->valid)
                this->port->FIODIR &= ~(1<<this->pin);
            return this;
        }

        inline Pin* set_mode(int mode) {
            if (mode == PIN_MODE_INPUT) {
                this->as_input();
            } else if (mode == PIN_MODE_OUTPUT) {
                this->as_output();
            } else {
                error("unknown pin mode %d\n", mode);
            }
            return this;
        }

        Pin* as_open_drain(void);

        Pin* as_repeater(void);

        Pin* pull_up(void);

        Pin* pull_down(void);

        Pin* pull_none(void);

        inline bool get() const{
            if (!this->valid) return false;
            return this->inverting ^ (( this->port->FIOPIN >> this->pin ) & 1);
        }

        inline void set(bool value)
        {
            if (!this->valid) return;
            if ( this->inverting ^ value )
                this->port->FIOSET = 1 << this->pin;
            else
                this->port->FIOCLR = 1 << this->pin;
        }

        mbed::PwmOut *hardware_pwm();

        mbed::InterruptIn *interrupt_pin();

        PinName to_pin_name();

        // these should be private, and use getters
        LPC_GPIO_TypeDef* port;

        unsigned char pin;
        char port_number;

    private:
        struct {
            bool inverting:1;
            bool valid:1;
        };
};

#endif
