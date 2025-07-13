#include "interrupt.h"

void TIMER1_IRQHandler(void)
{
    // Servo thread interrupt handler
    unsigned int isrMask = LPC_TIM1->IR;
    LPC_TIM1->IR = isrMask; /* Clear the Interrupt Bit */

    Interrupt::TIMER1_Wrapper();
}


void TIMER2_IRQHandler(void)
{
    // Base thread interrupt handler
    unsigned int isrMask = LPC_TIM2->IR;
    LPC_TIM2->IR = isrMask; /* Clear the Interrupt Bit */

    Interrupt::TIMER2_Wrapper();
}
