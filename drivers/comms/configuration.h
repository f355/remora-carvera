#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define JOINTS 4     // Number of joints - set this the same as LinuxCNC HAL compenent. Max 8 joints
#define VARIABLES 3  // Number of command values - set this the same as the LinuxCNC HAL compenent

#define PRU_DATA 0x64617461   // "data" SPI payload
#define PRU_READ 0x72656164   // "read" SPI payload
#define PRU_WRITE 0x77726974  // "writ" SPI payload
#define PRU_ESTOP 0x65737470  // "estp" SPI payload

// SPI configuration
#define SPI_BUFF_SIZE 35  // Size of SPI recieve buffer - same as HAL component, 64

#endif
