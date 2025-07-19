#ifndef REMORASPI_H
#define REMORASPI_H

#include "MODDMA.h"
#include "mbed.h"

#define JOINTS 4     // Number of joints - set this the same as LinuxCNC HAL compenent. Max 8 joints
#define VARIABLES 3  // Number of command values - set this the same as the LinuxCNC HAL compenent

#define SPI_DATA_HEADER 0xDABE  // DAta BEgin
#define SPI_DATA_FOOTER 0xDA57  // DAta 57op (STop)

// RPi SPI
#define MOSI1 P0_9
#define MISO1 P0_8
#define SCK1 P0_7
#define SSEL1 P0_6

#define FRACTIONAL_BITS 8

#pragma pack(push, 1)

// better keep these two structs the same size to synchronize rx/tx - SPI communicates bit-by-bit
typedef struct {
  uint16_t header;                              // should be set to SPI_DATA_HEADER by LinuxCNC
  volatile int32_t joint_freq_command[JOINTS];  // at what frequency to generate steps for each joint
  int32_t vars[VARIABLES];                      // output variables (e.g. PWM duty)
  uint16_t outputs;                             // output pin states (bit mask)
  // TODO do we need this, or joint_freq_command=0 is enough?
  uint8_t joint_enable;  // enable/disable stepgens (bitmask)
  uint16_t footer;       // makes sure the struct size is correct on LinuxCNC side, should be set to SPI_DATA_FOOTER
} rxData_t;

typedef struct {
  uint16_t header = SPI_DATA_HEADER;  // LinuxCNC should check it
  int32_t joint_feedback[JOINTS];     // joint positions in steps relative to zero when the PRU has booted up
  int32_t vars[VARIABLES];            // input variables (e.g. thermistor values)
  uint16_t inputs;                    // input pin states (bitmask)
  uint8_t e_stop_triggered;           // separate from inputs so LinuxCNC can detect it on the driver level
  uint16_t footer = SPI_DATA_FOOTER;  // LinuxCNC should check it to make sure the struct size is correct
} txData_t;

#pragma pack(pop)

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