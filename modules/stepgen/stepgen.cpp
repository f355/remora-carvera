#include "stepgen.h"

#define STEPBIT 22 // bit location in DDS accum
#define STEP_MASK (1L<<STEPBIT)
#define FRACTIONAL_BITS 16

Stepgen::Stepgen(
    int joint,
    Pin* stepPin,
    Pin* dirPin,
    int32_t threadFrequency,
    volatile rxData_t* rxData,
    volatile txData_t* txData
) :
    stepPin(stepPin->as_output()),
    directionPin(dirPin->as_output()),
    ptrFrequencyCommand(&rxData->jointFreqCmd[joint]),
    ptrFeedback(&txData->jointFeedback[joint]),
    ptrJointEnable(&rxData->jointEnable),
    DDSaccumulator(0),
    rawCount(0),
    frequencyScale((((uint64_t)STEP_MASK) << FRACTIONAL_BITS) / threadFrequency),
    enableMask(1 << joint),
    lastDir(true) {}


void Stepgen::update()
{
    if (this->isStepping) {
        // bring down the step pin that was set high on the previous thread tick
        this->stepPin->set(false);
        this->isStepping = false;
    }

    if ((*(this->ptrJointEnable) & this->enableMask) == 0)
    {
        return; // joint is disabled, nothing to do
    }
    
    // Direct Digital Synthesis (DDS)
    // works by incrementing an accumulator on every thread tick with a value calculated such that the accumulator
    // goes over a certain bit at the commanded frequency.
    //
    // frequencyScale is set to the increment that needs to be added to the accumulator on each thread tick
    // for the accumulator to reach that bit in one second at the thread tick frequency.
    //
    // by multiplying frequencyScale with the commanded frequency, we get the increment
    // that's needed to reach stepMask at that frequency.

    int32_t increment = (((int64_t)*(this->ptrFrequencyCommand)) * this->frequencyScale) >> FRACTIONAL_BITS;

    // save the old accumulator value and increment the accumulator
    int32_t stepNow = this->DDSaccumulator;
    this->DDSaccumulator += increment;
    // XOR the old and the new accumulator values to find the flipped bits
    stepNow ^= this->DDSaccumulator;
    // if the step bit has flipped, we need to drive the step pin
    stepNow &= STEP_MASK;

    // The sign of the increment indicates the desired direction
    bool isForward = (increment > 0);

    if (this->lastDir != isForward)
    {
        // Direction has changed, flip dir pin and do not step this iteration to give some setup time.
        // TODO: make hold time configurable.
        this->lastDir = isForward;
        // Set direction pin
        this->directionPin->set(isForward);
        return;
    }

    if (stepNow)
    {
        // make a step
        this->stepPin->set(true);
        this->isStepping = true;
        if (isForward)
        {
            this->rawCount++;
        }
        else
        {
            this->rawCount--;
        }
        *(this->ptrFeedback) = this->rawCount;
    }
}
