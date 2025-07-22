#include "lpcTimer.h"

#include "mbed.h"

#define SBIT_MR0I 0
#define SBIT_MR0R 1
#define SBIT_CNTEN 0

LPCTimer* lpc_timers[NUM_TIMERS];

LPCTimer::LPCTimer(LPC_TIM_TypeDef* timer, const IRQn_Type irq, const int8_t sbit, void (*wrapper)())
    : timer(timer), irq(irq), sbit(sbit), frequency(0), priority(0), handler(nullptr), wrapper(wrapper) {}

void LPCTimer::configure(InterruptHandler* handler, const uint32_t frequency, const uint32_t priority) {
  this->handler = handler;
  this->frequency = frequency;
  this->priority = priority;
};

void LPCTimer::start() {
  LPC_SC->PCONP |= (1 << this->sbit);                      // power on the timer
  this->timer->MCR = (1 << SBIT_MR0I) | (1 << SBIT_MR0R);  // Clear TC on MR0 match and Generate Interrupt
  this->timer->PR = 0x00;
  this->timer->MR0 = SystemCoreClock / 4 / this->frequency;
  this->timer->TCR = (1 << SBIT_CNTEN);  // Start timer by setting the Counter Enable

  NVIC_SetVector(this->irq, reinterpret_cast<uint32_t>(this->wrapper));
  NVIC_SetPriority(this->irq, this->priority);
  NVIC_EnableIRQ(this->irq);
}

void LPCTimer::handle_interrupt() const {
  // Clear the Interrupt Bit
  const unsigned int isr_mask = this->timer->IR;
  this->timer->IR = isr_mask;

  this->handler->handle_interrupt();
}

// need these to pass to NVIC_SetVector
void timer_1_irq_wrapper() { lpc_timers[1]->handle_interrupt(); }
void timer_2_irq_wrapper() { lpc_timers[2]->handle_interrupt(); }

void create_timers() {
  lpc_timers[1] = new LPCTimer(LPC_TIM1, TIMER1_IRQn, 2, timer_1_irq_wrapper);
  lpc_timers[2] = new LPCTimer(LPC_TIM2, TIMER2_IRQn, 22, timer_2_irq_wrapper);
}