#ifndef COMMS_H
#define COMMS_H

#include "MODDMA.h"
#include "mbed.h"
#include "spi_data.h"

typedef linuxCncData_t rxData_t;
typedef machineData_t txData_t;

// RPi SPI
#define MOSI1 P0_9
#define MISO1 P0_8
#define SCK1 P0_7
#define SSEL1 P0_6

#define FRACTIONAL_BITS 8

class Comms {
  SPISlave spi_slave;
  MODDMA dma;

  MODDMA_Config* rx_dma;
  MODDMA_Config* tx_dma;

  volatile rxData_t rx_data{};
  volatile txData_t tx_data{};
  volatile rxData_t rx_temp_buffer{};

  uint8_t reject_cnt = 0;
  bool comms_error = false;

  void rx_callback();
  void tx_callback();
  void err_callback();

 public:
  Comms();

  volatile bool pru_reset{};
  volatile rxData_t* ptr_rx_data = &rx_data;
  volatile txData_t* ptr_tx_data = &tx_data;

  void start();
  bool get_error() const;
};

#endif