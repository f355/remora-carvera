#include "PRUThread.h"
#include "module.h"


using namespace std;

// Thread constructor
PRUThread::PRUThread(uint32_t timerNumber, uint32_t frequency, uint32_t priority) :
    frequency(frequency)
{
    this->timer = lpcTimers[timerNumber];
    this->timer->configure(this, this->frequency, priority);
}

void PRUThread::startThread(void)
{
    this->timer->start();
}

void PRUThread::stopThread(void)
{
    this->timer->stop();
}

void PRUThread::registerModule(Module* module)
{
    this->vThread.push_back(module);
}

void PRUThread::unregisterModule(Module* module)
{
    iter = std::remove(vThread.begin(),vThread.end(), module);
    vThread.erase(iter, vThread.end());
}

void PRUThread::handleInterrupt()
{
    // iterate over the Thread pointer vector to run all instances of Module::runModule()
    for (iter = vThread.begin(); iter != vThread.end(); ++iter) (*iter)->runModule();
}
