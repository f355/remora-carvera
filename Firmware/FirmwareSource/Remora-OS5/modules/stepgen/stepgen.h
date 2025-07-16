#ifndef STEPGEN_H
#define STEPGEN_H

#include "modules/module.h"

class Stepgen : public Module
{
    private:

        int enableMask;

        volatile int32_t *ptrFrequencyCommand; // pointer to the data source where to get the frequency command
        volatile int32_t *ptrFeedback; // pointer where to put the feedback
        volatile uint8_t *ptrJointEnable;

        bool lastDir; // direction on last iteration, used for dir setup
        bool isStepping; // true if the step pin is held high
        int32_t rawCount; // current position raw count
        int32_t DDSaccumulator; // Direct Digital Synthesis (DDS) accumulator
        uint32_t frequencyScale; // frequency scale, 16.16 fixed-point

    public:

        Stepgen(int jointNumber, Pin* stepPin, Pin* dirPin, int32_t threadFrequency, volatile rxData_t* rxData, volatile txData_t* txData);

        Pin *stepPin, *directionPin;

        virtual void update(void);
};


#endif
