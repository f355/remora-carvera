#ifndef BLINK_H
#define BLINK_H

#include "modules/module.h"

class Blink : public Module
{
    private:

        bool bState;
        uint32_t periodCount;
        uint32_t blinkCount;

        Pin *blinkPin;

    public:

        Blink(Pin* pin, uint32_t freq, uint32_t threadFreq);

        virtual void update(void);
};

#endif
