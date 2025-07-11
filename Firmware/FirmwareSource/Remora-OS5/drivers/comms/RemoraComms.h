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

        volatile rxData_t*  ptrRxData;
        volatile txData_t*  ptrTxData;
        rxData_t            spiRxBuffer1;
        rxData_t            spiRxBuffer2;
        uint8_t             rejectCnt;
        bool                SPIdata;
        bool                SPIdataError;
        
    public:

        RemoraComms(volatile rxData_t*, volatile txData_t*);

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