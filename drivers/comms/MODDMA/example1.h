#include "MODDMA.h"
#include "mbed.h"

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
MODDMA dma;

// Function prototypes for IRQ callbacks.
// See definitions following main() below.
void dmaTCCallback(void);
void dmaERRCallback(void);
void TC0_callback(void);
void ERR0_callback(void);

/**
 * @brief
 * @note
 * @param
 * @retval
 */
int main() {
  char s[] = "**DMA** ABCDEFGHIJKLMNOPQRSTUVWXYZ **DMA**";

  printf("\r\nStarting\r\n");

  dma.attach_tc(&dmaTCCallback);
  dma.attach_err(&dmaERRCallback);

  MODDMA_Config* config = new MODDMA_Config;

  config->channelNum(MODDMA::Channel_0);
  config->srcMemAddr((uint32_t)&s);
  config->dstMemAddr(0);
  config->transferSize(sizeof(s));
  config->transferType(MODDMA::m2p);
  config->transferWidth(0);
  config->srcConn(0);
  config->dstConn(MODDMA::UART0_Tx);
  config->dmaLLI(0)->attach_tc(&TC0_callback);
  config->attach_err(&ERR0_callback);

  // Setup the configuration.
  dma.Setup(config);

  // dma.Enable( MODDMA::Channel_0 );
  // dma.Enable( config->channelNum() );
  dma.Enable(config);

  while (1) {
    led1 = !led1;
    ThisThread::sleep_for(250ms);
  }
}

// Main controller TC IRQ callback
void dmaTCCallback(void) { led2 = 1; }

// Main controller ERR IRQ callback
void dmaERRCallback(void) { printf("Oh no! My Mbed exploded! :( Only kidding, find the problem"); }

// Configuration callback on TC
void TC0_callback(void) {
  MODDMA_Config* config = dma.getConfig();

  dma.haltAndWaitChannelComplete((MODDMA::CHANNELS)config->channelNum());
  dma.Disable((MODDMA::CHANNELS)config->channelNum());

  // Configurations have two IRQ callbacks for TC and Err so you
  // know which you are processing. However, if you want to use
  // a single callback function you can tell what type of IRQ
  // is being processed thus:-
  if (dma.irqType() == MODDMA::TcIrq) {
    led3 = 1;
    dma.clearTcIrq();
  }

  if (dma.irqType() == MODDMA::ErrIrq) {
    led4 = 1;
    dma.clearErrIrq();
  }
}

// Configuration cakllback on Error
void ERR0_callback(void) { printf("Oh no! My Mbed exploded! :( Only kidding, find the problem"); }
