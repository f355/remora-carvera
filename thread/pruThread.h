#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include "LPC17xx.h"
#include "lpcTimer.h"

// Standard Template Library (STL) includes
#include <iostream>
#include <vector>

using namespace std;

class Module;

class PRUThread: public InterruptHandler
{

    private:
        LPCTimer *timer;

        vector<Module*> vThread; // vector containing pointers to Thread modules
        vector<Module*>::iterator iter;

    public:

        PRUThread(uint32_t timerNumber, uint32_t frequency, uint32_t priority);

        uint32_t frequency;

        void registerModule(Module *module);
        void unregisterModule(Module *module);
        void start(void);
        void handleInterrupt(void);
};

#endif

