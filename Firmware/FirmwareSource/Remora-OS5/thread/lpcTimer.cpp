#include "lpcTimer.h"

#define SBIT_MR0I    0
#define SBIT_MR0R    1
#define SBIT_CNTEN   0

LPCTimer* lpcTimers[NUM_TIMERS];

LPCTimer::LPCTimer(LPC_TIM_TypeDef* timer, IRQn_Type irq, int8_t sbit, void (*wrapper)()) :
    timer(timer),
    irq(irq),
    sbit(sbit),
    wrapper(wrapper)
    {};

void LPCTimer::configure(InterruptHandler* handler, int32_t frequency, uint32_t priority) {
    this->handler = handler;
    this->frequency = frequency;
    this->priority = priority;
};

void LPCTimer::start() {
    LPC_SC->PCONP |= (1 << this->sbit); // power on the timer
    this->timer->MCR  = (1<<SBIT_MR0I) | (1<<SBIT_MR0R);     // Clear TC on MR0 match and Generate Interrupt
    this->timer->PR   = 0x00;
    this->timer->MR0  = SystemCoreClock/4/this->frequency;
    this->timer->TCR  = (1<<SBIT_CNTEN);                     // Start timer by setting the Counter Enable

    NVIC_SetVector(this->irq, (uint32_t)this->wrapper);
    NVIC_SetPriority(this->irq, this->priority);
    NVIC_EnableIRQ(this->irq);
}

void LPCTimer::handleInterrupt() {
    unsigned int isrMask = this->timer->IR;
    this->timer->IR = isrMask; /* Clear the Interrupt Bit */

    this->handler->handleInterrupt();
}

void LPCTimer::stop() {
    NVIC_DisableIRQ(this->irq);
    this->timer->TCR = (0<<SBIT_CNTEN);
}

// need these to pass to NVIC_SetVector
void timer1Wrapper() {
    lpcTimers[1]->handleInterrupt();
}

void timer2Wrapper() {
    lpcTimers[2]->handleInterrupt();
}

void createTimers() {
    lpcTimers[1] = new LPCTimer(LPC_TIM1, TIMER1_IRQn, 2, timer1Wrapper);
    lpcTimers[2] = new LPCTimer(LPC_TIM2, TIMER2_IRQn, 22, timer2Wrapper);
}