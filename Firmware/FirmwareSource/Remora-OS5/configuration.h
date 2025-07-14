#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define STEPBIT 22 // bit location in DDS accum
#define STEP_MASK (1L<<STEPBIT)

// JSON dynamic buffer size
#define JSON_BUFF_SIZE 10000

#define JOINTS 4 // Number of joints - set this the same as LinuxCNC HAL compenent. Max 8 joints
#define VARIABLES 6 // Number of command values - set this the same as the LinuxCNC HAL compenent

#define PRU_DATA 0x64617461 // "data" SPI payload
#define PRU_READ 0x72656164 // "read" SPI payload
#define PRU_WRITE 0x77726974 // "writ" SPI payload
#define PRU_ESTOP 0x65737470 // "estp" SPI payload


// PRU reset will occur in SPI_ERR_MAX * LOOP_TIME = 0.5sec
#define LOOP_TIME 0.1
#define SPI_ERR_MAX 5

// SPI configuration
#define SPI_BUFF_SIZE 64 // Size of SPI recieve buffer - same as HAL component, 64

#define MOSI0 P0_18
#define MISO0 P0_17
#define SCK0 P0_15
#define SSEL0 P0_16

#endif
