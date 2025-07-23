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

  rxData_t rx_data{};
  txData_t tx_data{};
  rxData_t rx_temp_buffer{};
  rxData_t tx_temp_buffer{};
  uint8_t reject_cnt = 0;
  bool comms_ready = false;
  bool comms_error = false;

 public:
  Comms();

  volatile bool pru_reset{};
  volatile rxData_t* ptr_rx_data = &rx_data;
  volatile txData_t* ptr_tx_data = &tx_data;

  void rx_callback();
  static void err_callback();

  void start();
  bool is_ready() const;
  void not_ready();
  bool get_error() const;
  void clear_error();
};

#endif