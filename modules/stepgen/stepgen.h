#ifndef STEPGEN_H
#define STEPGEN_H

#include "modules/module.h"

Module* createStepgen(JsonObject module, PRUThread* thread, RemoraComms* comms);

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
        float frequencyScale; // frequency scale
        int32_t stepMask; // value of the DDS accumulator that triggers a step pulse

    public:

        Stepgen(int32_t, int, std::string, std::string, int32_t, volatile int32_t&, volatile int32_t&, volatile uint8_t&);  // constructor

        Pin *stepPin, *directionPin;

        virtual void update(void);
};


#endif
