#ifndef LPCTIMER_H
#define LPCTIMER_H

#include "LPC17xx.h"

#define NUM_TIMERS 4

class InterruptHandler
{
    public:
        virtual void handleInterrupt() = 0;
};

class LPCTimer
{
    protected:
        LPC_TIM_TypeDef* timer;
        IRQn_Type irq;
        int8_t sbit;
        int32_t frequency;
        uint32_t priority;
        InterruptHandler* handler;

        void (*wrapper)();

    public:
        LPCTimer(LPC_TIM_TypeDef* timer, IRQn_Type irq, int8_t sbit, void (*wrapper)());

        void configure(InterruptHandler* handler, int32_t frequency, uint32_t priority);

        void start(void);
        void handleInterrupt(void);
        void stop(void);
};

extern LPCTimer* lpcTimers[NUM_TIMERS];

void createTimers(void);

#endif