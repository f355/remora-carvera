#ifndef COMMS_H
#define COMMS_H

#include "MODDMA.h"
#include "configuration.h"
#include "mbed.h"

// RPi SPI
#define MOSI1 P0_9
#define MISO1 P0_8
#define SCK1 P0_7
#define SSEL1 P0_6

#pragma pack(push, 1)

#include "machine_definitions.h"

typedef union {
  // this allows structured access to the incoming SPI data without having to move it
  struct {
    uint8_t rx_buffer[SPI_BUFF_SIZE];
  };
  struct {
    int32_t header;
    volatile int32_t joint_freq_command[JOINTS];
    float set_point[VARIABLES];
    uint8_t joint_enable;
    uint16_t outputs;
    uint8_t padding;
  };
} rxData_t;

typedef union {
  // this allows structured access to the outgoing SPI data without having to move it
  struct {
    uint8_t tx_buffer[SPI_BUFF_SIZE];
  };
  struct {
    int32_t header;
    int32_t joint_feedback[JOINTS];
    float process_variable[VARIABLES];
    uint16_t inputs;
  };
} txData_t;

#pragma pack(pop)

class Comms {
  SPISlave spi_slave;

  MODDMA dma;

  MODDMA_Config* rx_dma1;
  MODDMA_Config* rx_dma2;
  MODDMA_Config* tx_dma1;
  MODDMA_Config* tx_dma2;
  MODDMA_Config* rx_memcpy_dma1;
  MODDMA_Config* rx_memcpy_dma2;

  rxData_t temp_rx_buffer1{};
  rxData_t temp_rx_buffer2{};
  uint8_t reject_count = 0;
  bool data_ready = false;
  bool spi_error = false;

 public:
  Comms();

  volatile bool pru_reset = false;
  rxData_t volatile* ptr_rx_data;
  txData_t volatile* ptr_tx_data;

  void tx1_callback();
  void tx2_callback();
  void rx1_callback();
  void rx2_callback();
  void err_callback();

  void init();
  void start();
  bool get_status() const;
  void set_status(bool);
  bool get_error() const;
  void set_error(bool);
};

#endif