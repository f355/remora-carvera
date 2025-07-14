#ifndef STEPGEN_H
#define STEPGEN_H

#include <cstdint>
#include <string>
#include <iostream>

#include "modules/module.h"
#include "drivers/pin/pin.h"

Module* createStepgen(JsonObject module, pruThread* thread, RemoraComms* comms);

class Stepgen : public Module
{
  private:

    int jointNumber;              	// LinuxCNC joint number
    int enableMask;

    std::string step, direction;	 // physical pins connections

    bool inv; // true if high dir results in negative axis movement

    bool lastDir;            // direction on last iteration, used for dir setup

    volatile int32_t *ptrFrequencyCommand; 	// pointer to the data source where to get the frequency command
    int32_t rawCount;             	// current position raw count - not currently used - mirrors original stepgen.c
    volatile int32_t *ptrFeedback;       	// pointer where to put the feedback
    volatile uint8_t *ptrJointEnable;
    int32_t DDSaccumulator;       	// Direct Digital Synthesis (DDS) accumulator
    float   frequencyScale;		  	  // frequency scale
    int32_t stepMask;               // value of the DDS accumulator that triggers a step pulse

  public:

    Stepgen(int32_t, int, std::string, std::string, bool, int32_t, volatile int32_t&, volatile int32_t&, volatile uint8_t&);  // constructor

    Pin *stepPin, *directionPin;		// class object members - Pin objects

    virtual void update(void);           // Module default interface
    virtual void updatePost(void);
};


#endif
