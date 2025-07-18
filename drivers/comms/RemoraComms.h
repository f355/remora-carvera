#ifndef REMORASPI_H
#define REMORASPI_H

#include "mbed.h"
#include "configuration.h"
#include "MODDMA.h"

// RPi SPI
#define MOSI1               P0_9
#define MISO1               P0_8
#define SCK1                P0_7
#define SSEL1               P0_6

#pragma pack(push, 1)

typedef struct
{
    int32_t header;
    volatile int32_t jointFreqCmd[JOINTS];
    float setPoint[VARIABLES];
    uint16_t outputs;
    uint8_t jointEnable;
} rxData_t;

typedef struct
{
    int32_t header;
    int32_t jointFeedback[JOINTS];
    float processVariable[VARIABLES];
    uint16_t inputs;
    uint8_t padding;
} txData_t;

#pragma pack(pop)

class RemoraComms
{
    private:

        SPISlave spiSlave;

        MODDMA dma;

        MODDMA_Config* spiDMArx1;
        MODDMA_Config* spiDMArx2;
        MODDMA_Config* spiDMAtx1;
        MODDMA_Config* spiDMAtx2;
        MODDMA_Config* spiDMAmemcpy1;
        MODDMA_Config* spiDMAmemcpy2;

        rxData_t rxData;
        txData_t txData;
        rxData_t spiRxBuffer1;
        rxData_t spiRxBuffer2;
        uint8_t rejectCnt;
        bool SPIdata;
        bool SPIdataError;
        
    public:

        RemoraComms();

        volatile bool pruReset;
        volatile rxData_t* ptrRxData = &rxData;
        volatile txData_t* ptrTxData = &txData;

        void tx1_callback(void);
        void tx2_callback(void);
        void rx1_callback(void);
        void rx2_callback(void);
        void err_callback(void);

        void init(void);
        void start(void);
        bool getStatus(void);
        void setStatus(bool);
        bool getError(void);
        void setError(bool);

};

#endif