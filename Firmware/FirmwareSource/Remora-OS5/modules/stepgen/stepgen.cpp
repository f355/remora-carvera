#include "stepgen.h"


/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

Module* createStepgen(JsonObject module, PRUThread* thread, RemoraComms* comms)
{
    int joint = module["joint"];
    const char* step = module["step_pin"];
    const char* dir = module["direction_pin"];
    bool invert = module["invert"];

    return new Stepgen(
        thread->frequency,
        joint,
        step,
        dir,
        invert,
        STEP_MASK,
        comms->ptrRxData->jointFreqCmd[joint],
        comms->ptrTxData->jointFeedback[joint],
        comms->ptrRxData->jointEnable
    );
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

Stepgen::Stepgen(
    int32_t threadFreq,
    int jointNumber,
    std::string step,
    std::string direction,
    bool inv,
    int32_t stepMask,
    volatile int32_t &ptrFrequencyCommand,
    volatile int32_t &ptrFeedback,
    volatile uint8_t &ptrJointEnable
) :
    jointNumber(jointNumber),
    step(step),
    direction(direction),
    inv(inv),
    stepMask(stepMask),
    ptrFrequencyCommand(&ptrFrequencyCommand),
    ptrFeedback(&ptrFeedback),
    ptrJointEnable(&ptrJointEnable)
{
    this->stepPin = new Pin(this->step, OUTPUT);
    this->directionPin = new Pin(this->direction, OUTPUT);
    this->DDSaccumulator = 0;
    this->rawCount = 0;
    this->frequencyScale = (float)this->stepMask / (float)threadFreq;
    this->enableMask = 1 << this->jointNumber;
    this->lastDir = true;

    this-> hasPost = true;
}


void Stepgen::update()
{
    if ((*(this->ptrJointEnable) & this->enableMask) == 0)
    {
        return; // joint is disabled, nothing to do
    }
    
    // Direct Digital Synthesis (DDS)
    // works by incrementing an accumulator on every thread tick with a value calculated such that the accumulator
    // goes over a certain bit at the commanded frequency.
    //
    // frequencyScale is set to the increment that needs to be added to the accumulator on each thread tick
    // for the accumulator to reach stepMask in one second.
    // by multiplying it with the commanded frequency, we get the increment that's needed to reach stepMask at that frequency.
    int32_t increment = (*(this->ptrFrequencyCommand)) * this->frequencyScale;

    // save the old accumulator value and increment the accumulator
    int32_t stepNow = this->DDSaccumulator;
    this->DDSaccumulator += increment;
    // XOR the old and the new accumulator values to find the flipped bits
    stepNow ^= this->DDSaccumulator;
    // if the step bit has flipped, we need to drive the step pin
    stepNow &= this->stepMask;

    // The sign of the increment indicates the desired direction
    bool isForward = (increment > 0);

    if (this->lastDir != isForward)
    {
        // Direction has changed, flip dir pin and do not step this iteration to give some setup time.
        // TODO: make hold time configurable.
        this->lastDir = isForward;
        // Set direction pin, possibly inverted
        this->directionPin->set(isForward != this->inv);
        return;
    }

    if (stepNow)
    {
        // make a step
        this->stepPin->set(true);
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

void Stepgen::updatePost()
{
    // TODO: the step pin is reset at the end of the same thread tick in updatePost - is that always enough time?
    this->stepPin->set(false);
}
