#ifndef SPI_DATA_H
#define SPI_DATA_H

#include "machine_definitions.h"

#define SPI_DATA_HEADER 0xDABE  // DAta BEgin
#define SPI_DATA_FOOTER 0xDA57  // DAta 57op (STop)

#pragma pack(push, 1)

// better keep these two structs the same size to synchronize rx/tx - SPI transceives data full-duplex bit-by-bit

// Data originating on the LinuxCNC side
typedef struct {
  uint16_t header = 0;  // should be set to SPI_DATA_HEADER by LinuxCNC, the machine should check it
  volatile int32_t joint_freq_command[JOINTS]{};  // at what frequency to generate steps for each joint, in Hz
  int32_t vars[OUTPUT_VARS]{};                    // output variables (PWM duty per mil (1/1000, 0.1%))
  uint16_t outputs = 0;                           // output pin states (bitmask)
  // TODO do we need this, or joint_freq_command=0 is enough?
  uint8_t joint_enable = 0;  // enable/disable stepgens (bitmask)
  uint16_t footer = 0;       // should be set to SPI_DATA_HEADER by LinuxCNC, the machine should
                             // check it to make sure the struct size is correct
} linuxCncData_t;

// Data originating on the machine side
typedef struct {
  uint16_t header = 0;               // should be set to SPI_DATA_HEADER by the machine, LinuxCNC should check it
  int32_t joint_feedback[JOINTS]{};  // joint positions in steps relative to zero when the PRU has booted up
  int32_t vars[INPUT_VARS]{};        // input variables (thermistor values: 1/100ºC, pulse counter: number of pulses)
  uint16_t inputs = 0;               // input pin states (bitmask)
  uint8_t e_stop_triggered = 0;      // separate from inputs so LinuxCNC can detect it on the driver level
  uint16_t footer = 0;               // should be set to SPI_DATA_HEADER by the machine, LinuxCNC should
                                     // check it to make sure the struct size is correct
} machineData_t;

#pragma pack(pop)

#endif
