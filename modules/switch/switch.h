#ifndef SWITCH_H
#define SWITCH_H

#include "modules/module.h"

class Switch : public Module
{
    private:

        volatile float* ptrPV; // pointer to the data source
        float PV;
        float SP;
        bool mode; // 0 switch off, 1 switch on

        Pin *pin;


    public:

        Switch(float, volatile float&, std::string, bool);

        virtual void update(void);
};

#endif
