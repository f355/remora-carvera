/*
 * This example was provided to support Mbed forum thread:-
 * http://mbed.org/forum/mbed/topic/1798
 */
#include "MODDMA.h"
#include "mbed.h"

#define SAMPLE_BUFFER_LENGTH 32

DigitalOut led1(LED1);
DigitalOut led2(LED2);

MODDMA dma;

// ISR set's this when transfer complete.
bool dmaTransferComplete = false;

// Function prototypes for IRQ callbacks.
// See definitions following main() below.
void TC0_callback(void);
void ERR0_callback(void);

/**
 * @brief
 * @note
 * @param
 * @retval
 */
int main() {
  // Create a buffer to hold the ADC samples and clear it.
  // Note, we are going to sample two ADC inputs so they
  // end up in this buffer "interleaved". So you will want
  // a buffer twice this size to a real life given sample
  // frequency. See the printf() output for details.
  uint32_t adcInputBuffer[SAMPLE_BUFFER_LENGTH];
  memset(adcInputBuffer, 0, sizeof(adcInputBuffer));

  // We use the ADC irq to trigger DMA and the manual says
  // that in this case the NVIC for ADC must be disabled.
  NVIC_DisableIRQ(ADC_IRQn);

  // Power up the ADC and set PCLK
  LPC_SC->PCONP |= (1UL << 12);
  LPC_SC->PCLKSEL0 &= ~(3UL << 24);  // PCLK = CCLK/4 96M/4 = 24MHz

  // Enable the ADC, 12MHz,  ADC0.0 & .1
  LPC_ADC->ADCR = (1UL << 21) | (1UL << 8) | (3UL << 0);

  // Set the pin functions to ADC
  LPC_PINCON->PINSEL1 &= ~(3UL << 14); /* P0.23, Mbed p15. */
  LPC_PINCON->PINSEL1 |= (1UL << 14);
  LPC_PINCON->PINSEL1 &= ~(3UL << 16); /* P0.24, Mbed p16. */
  LPC_PINCON->PINSEL1 |= (1UL << 16);

  // Setup the serial port to print out results.
  printf("ADC with DMA example\n");
  printf("====================\n");

  // Prepare an ADC configuration.
  MODDMA_Config* conf = new MODDMA_Config;
  conf->channelNum(MODDMA::Channel_0);
  conf->srcMemAddr(0);
  conf->dstMemAddr((uint32_t)adcInputBuffer);
  conf->transferSize(SAMPLE_BUFFER_LENGTH);
  conf->transferType(MODDMA::p2m);
  conf->transferWidth(MODDMA::word);
  conf->srcConn(MODDMA::ADC);
  conf->dstConn(0);
  conf->dmaLLI(0);
  conf->attach_tc(&TC0_callback);
  conf->attach_err(&ERR0_callback);

  // Prepare configuration.
  dma.Setup(conf);

  // Enable configuration.
  dma.Enable(conf);

  // Enable ADC irq flag (to DMA).
  // Note, don't set the individual flags,
  // just set the global flag.
  LPC_ADC->ADINTEN = 0x100;

  // Enable burst mode on inputs 0 and 1.
  LPC_ADC->ADCR |= (1UL << 16);

  while (1) {
    // When transfer complete do this block.
    if (dmaTransferComplete) {
      delete conf;  // No memory leaks, delete the configuration.
      dmaTransferComplete = false;
      for (int i = 0; i < SAMPLE_BUFFER_LENGTH; i++) {
        int channel = (adcInputBuffer[i] >> 24) & 0x7;
        int iVal = (adcInputBuffer[i] >> 4) & 0xFFF;
        double fVal = 3.3 * (double)((double)iVal) / ((double)0x1000);  // scale to 0v to 3.3v

        printf("Array index %02d : ADC input channel %d = 0x%03x %01.3f volts\n", i, channel, iVal, fVal);
      }
    }

    // Just flash LED1 for something to do.
    led1 = !led1;
    ThisThread::sleep_for(250ms);
  }
}

// Configuration callback on TC
void TC0_callback(void) {
  MODDMA_Config* config = dma.getConfig();

  // Disbale burst mode and switch off the IRQ flag.
  LPC_ADC->ADCR &= ~(1UL << 16);
  LPC_ADC->ADINTEN = 0;

  // Finish the DMA cycle by shutting down the channel.
  dma.haltAndWaitChannelComplete((MODDMA::CHANNELS)config->channelNum());
  dma.Disable((MODDMA::CHANNELS)config->channelNum());

  // Tell main() while(1) loop to print the results.
  dmaTransferComplete = true;

  // Switch on LED2 to show transfer complete.
  led2 = 1;

  // Clear DMA IRQ flags.
  if (dma.irqType() == MODDMA::TcIrq) dma.clearTcIrq();
  if (dma.irqType() == MODDMA::ErrIrq) dma.clearErrIrq();
}

// Configuration callback on Error
void ERR0_callback(void) {
  // Switch off burst conversions.
  LPC_ADC->ADCR |= ~(1UL << 16);
  LPC_ADC->ADINTEN = 0;
  error("Oh no! My Mbed EXPLODED! :( Only kidding, go find the problem");
}
