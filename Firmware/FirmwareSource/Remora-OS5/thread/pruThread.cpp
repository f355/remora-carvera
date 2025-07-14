#include "pruThread.h"
#include "modules/module.h"


using namespace std;

// Thread constructor
pruThread::pruThread(uint32_t timerNumber, uint32_t frequency, uint32_t priority) :
	frequency(frequency)
{
	printf("Creating thread %d\n", this->frequency);
	this->timer = lpcTimers[timerNumber];
	this->timer->configure(this, this->frequency, priority);
}

void pruThread::startThread(void)
{
	this->timer->start();
}

void pruThread::stopThread(void)
{
    this->timer->stop();
}

void pruThread::registerModule(Module* module)
{
	this->vThread.push_back(module);
	if (module->hasPost) {
		this->vThreadPost.push_back(module);
	}
}

void pruThread::unregisterModule(Module* module)
{
	iter = std::remove(vThread.begin(),vThread.end(), module);
    vThread.erase(iter, vThread.end());
}

void pruThread::handleInterrupt()
{
	// iterate over the Thread pointer vector to run all instances of Module::runModule()
	for (iter = vThread.begin(); iter != vThread.end(); ++iter) (*iter)->runModule();

    // iterate over the second vector that contains module pointers to run after (post) the main vector
	for (iter = vThreadPost.begin(); iter != vThreadPost.end(); ++iter) (*iter)->runModulePost();

}
