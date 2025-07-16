#include "blink.h"

Blink::Blink(Pin* pin, uint32_t freq, uint32_t threadFreq)
{
    this->periodCount = threadFreq / freq;
    this->blinkCount = 0;
    this->bState = false;

    this->blinkPin = pin->as_output();
    this->blinkPin->set(bState);
}

void Blink::update(void)
{
    ++this->blinkCount;
    if (this->blinkCount >= this->periodCount / 2)
    {
        this->blinkPin->set(this->bState=!this->bState);
        this->blinkCount = 0;
    }
}
