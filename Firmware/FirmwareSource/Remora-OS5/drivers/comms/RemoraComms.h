#ifndef REMORASPI_H
#define REMORASPI_H

#include "mbed.h"
#include "configuration.h"
#include "remora.h"
#include "MODDMA.h"

// RPi SPI
#define MOSI1               P0_9
#define MISO1               P0_8
#define SCK1                P0_7
#define SSEL1               P0_6

#pragma pack(push, 1)

typedef union
{
  // this allow structured access to the incoming SPI data without having to move it
  struct
  {
    uint8_t rxBuffer[SPI_BUFF_SIZE];
  };
  struct
  {
    int32_t header;
    volatile int32_t jointFreqCmd[JOINTS]; 	// Base thread commands ?? - basically motion
    volatile float setPoint[VARIABLES];		  // Servo thread commands ?? - temperature SP, PWM etc
    uint8_t jointEnable;
    uint16_t outputs;
    uint8_t spare0;
  };
} rxData_t;

typedef union
{
  // this allow structured access to the out going SPI data without having to move it
  struct
  {
    uint8_t txBuffer[SPI_BUFF_SIZE];
  };
  struct
  {
    int32_t header;
    int32_t jointFeedback[JOINTS];	  // Base thread feedback ??
    float processVariable[VARIABLES];		     // Servo thread feedback ??
	uint16_t inputs;
  };
} txData_t;

#pragma pack(pop)

class RemoraComms
{
    private:

        SPISlave            spiSlave;

        MODDMA              dma;

        MODDMA_Config*      spiDMArx1;
        MODDMA_Config*      spiDMArx2;
        MODDMA_Config*      spiDMAtx1;
        MODDMA_Config*      spiDMAtx2;
        MODDMA_Config*      spiDMAmemcpy1;
        MODDMA_Config*      spiDMAmemcpy2;

        rxData_t            spiRxBuffer1;
        rxData_t            spiRxBuffer2;
        uint8_t             rejectCnt;
        bool                SPIdata;
        bool                SPIdataError;
        
    public:

        RemoraComms();

        volatile bool pruReset;
        rxData_t volatile * ptrRxData;
        txData_t volatile * ptrTxData;

        void tc0_callback(void);
        void tc1_callback(void);
        void tc2_callback(void);
        void tc3_callback(void);
        void err_callback(void);

        void init(void);
        void start(void);
        bool getStatus(void);
        void setStatus(bool);
        bool getError(void);
        void setError(bool);

};

#endif