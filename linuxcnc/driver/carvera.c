/********************************************************************
 * Description:  remora.c
 *               This file, 'remora.c', is a HAL component that
 *               provides and SPI connection to a external LPC1768 running Remora PRU firmware.
 *
 *				Initially developed for RaspberryPi -> Arduino Due.
 *				Further developed for RaspberryPi -> Smoothieboard and clones (LPC1768).
 *
 * Author: Scott Alford
 * License: GPL Version 3
 *
 *		Credit to GP Orcullo and PICnc V2 which originally inspired this
 *		and portions of this code is based on stepgen.c by John Kasunich
 *		and hm2_rpspi.c by Matsche
 *
 * Copyright (c) 2024	All rights reserved.
 *
 * Last change:
 ********************************************************************/

#include <fcntl.h>
#include <math.h>
#include <sys/mman.h>
#include <unistd.h>

#include "hal.h"       /* HAL public API decls */
#include "rtapi.h"     /* RTAPI realtime OS API */
#include "rtapi_app.h" /* RTAPI realtime module decls */

// Using BCM2835 driver library by Mike McCauley, why reinvent the wheel!
// http://www.airspayce.com/mikem/bcm2835/index.html
// Include these in the source directory when using "halcompile --install remora.c"
#include "bcm2835.c"
#include "bcm2835.h"

// Raspberry Pi 5 uses the RP1
#include <stdbool.h>

#include "gpiochip_rp1.c"
#include "gpiochip_rp1.h"
#include "rp1lib.c"
#include "rp1lib.h"
#include "spi-dw.c"
#include "spi-dw.h"
#include "spi_data.h"

#define MODNAME "carvera"
#define PREFIX "carvera"

MODULE_AUTHOR("Scott Alford AKA scotta");
MODULE_DESCRIPTION("Driver for the Carvera family of desktop milling machines");
MODULE_LICENSE("GPL v3");
#define RPI5_RP1_PERI_BASE 0x7c000000

/***********************************************************************
 *                STRUCTURES AND GLOBAL VARIABLES                       *
 ************************************************************************/

typedef struct {
  hal_bit_t *enable;
  hal_float_t *pos_cmd;   // pin: position command (position units)
  hal_float_t *pos_fb;    // pin: position feedback (position units)
  hal_s32_t *steps;       // pin: position feedback (raw counts)
  int32_t prev_steps;     // previous value of steps (for spike filtering)
  int32_t filter_count;   // the number of times spike filter fired in a row
  hal_float_t pos_scale;  // param: steps per position unit
  int32_t freq;           // param: frequency command sent to PRU
  hal_s32_t *freq_cmd;    // pin: frequency command monitoring, available in LinuxCNC
  hal_float_t maxvel;     // param: max velocity, (pos units/sec)
  hal_float_t maxaccel;   // param: max accel (pos units/sec^2)
  hal_float_t *pgain;
  hal_float_t *ff1gain;
  hal_float_t *deadband;
  float old_scale;  // stored scale value
  float prev_cmd;
  float cmd_d;  // command derivative
} joint_state_t;

typedef struct {
  hal_bit_t *spi_enable;
  hal_bit_t *spi_reset;
  hal_bit_t *hard_reset;
  bool spi_reset_old;
  hal_bit_t *spi_status;
  joint_state_t *joints[JOINTS];
  hal_float_t *output_vars[OUTPUT_VARS];
  hal_float_t *input_vars[INPUT_VARS];
  hal_bit_t *output_pins[OUTPUT_PINS];
  hal_bit_t *input_pins[INPUT_PINS * 2];  // multiplied by 2 for inverted 'not' pins passed through to LinuxCNC
} state_t;

static state_t *state;

typedef machineData_t rxData_t;
typedef linuxCncData_t txData_t;

static txData_t txData;
static uint8_t *txBuffer = (uint8_t *)&txData;
static rxData_t rxData;
static uint8_t *rxBuffer = (uint8_t *)&rxData;

/* other globals */
static int comp_id;  // component ID
static const char *modname = MODNAME;
static const char *prefix = PREFIX;
static bool bcm;         // use BCM2835 driver
static bool rp1;         // use RP1 driver
static long old_dtns;    // update_freq function period in nsec - (THIS IS RUNNING IN THE PI)
static double dt;        // update_freq period in seconds  - (THIS IS RUNNING IN THE PI)
static double recip_dt;  // recprocal of period, avoids divides

static int reset_gpio_pin = 25;  // RPI GPIO pin number used to force watchdog reset of the PRU

static int rt_peripheral_init(void);
static int rt_bcm2835_init(void);
static int rt_rp1lib_init(void);

static void calc_stepper_commands(void *arg, long period);
static void prepare_tx();
static void handle_rx();
static void spi_transceive();

bool pin_err(int retval) {
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin export failed with err=%i\n", modname, retval);
    hal_exit(comp_id);
    return true;
  }
  return false;
}

int rtapi_app_main(void) {
  if (sizeof(rxData) != sizeof(txData)) {
    rtapi_print_msg(RTAPI_MSG_ERR, "SPI buffer size mismatch, rx: %d , tx: %d", sizeof(rxData), sizeof(txData));
    return -1;
  }

  txData.header = SPI_DATA_HEADER;
  txData.footer = SPI_DATA_FOOTER;

  // connect to the HAL, initialise the driver
  comp_id = hal_init(modname);
  if (comp_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed \n", modname);
    return -1;
  }

  // allocate shared memory
  state = hal_malloc(sizeof(state_t));
  if (state == 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc(state) failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  for (int i = 0; i < JOINTS; i++) {
    state->joints[i] = hal_malloc(sizeof(joint_state_t));
    if (state->joints[i] == 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc(joint) failed\n", modname);
      hal_exit(comp_id);
      return -1;
    }
  }

  bcm = false;
  rp1 = false;

  // initialise the gpio and spi peripherals
  if (!rt_peripheral_init()) {
    rtapi_print_msg(RTAPI_MSG_ERR, "rt_peripheral_init failed.\n");
    return -1;
  }

  // export remoraPRU SPI enable and status bits
  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->spi_enable, comp_id, "%s.spi-enable", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->spi_reset, comp_id, "%s.spi-reset", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->spi_status, comp_id, "%s.spi-status", prefix))) return -1;

  if (bcm == true) {
    bcm2835_gpio_fsel(reset_gpio_pin, BCM2835_GPIO_FSEL_OUTP);
  } else if (rp1 == true) {
    gpio_set_fsel(reset_gpio_pin, GPIO_FSEL_OUTPUT);
  }
  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->hard_reset, comp_id, "%s.hard-reset", prefix))) return -1;

  // export all the variables for each joint and pin
  int n;
  for (n = 0; n < JOINTS; n++) {
    joint_state_t *joint = state->joints[n];

    if (pin_err(hal_pin_bit_newf(HAL_IN, &joint->enable, comp_id, "%s.joint.%01d.enable", prefix, n))) return -1;
    if (pin_err(hal_pin_float_newf(HAL_IN, &joint->pos_cmd, comp_id, "%s.joint.%01d.pos-cmd", prefix, n))) return -1;
    *joint->pos_cmd = 0.0;
    if (pin_err(hal_pin_s32_newf(HAL_OUT, &joint->freq_cmd, comp_id, "%s.joint.%01d.freq-cmd", prefix, n))) return -1;
    *joint->freq_cmd = 0;
    if (pin_err(hal_pin_float_newf(HAL_OUT, &joint->pos_fb, comp_id, "%s.joint.%01d.pos-fb", prefix, n))) return -1;
    *joint->pos_fb = 0.0;
    if (pin_err(hal_param_float_newf(HAL_RW, &joint->pos_scale, comp_id, "%s.joint.%01d.scale", prefix, n))) return -1;
    joint->pos_scale = 1.0;
    if (pin_err(hal_pin_s32_newf(HAL_OUT, &joint->steps, comp_id, "%s.joint.%01d.counts", prefix, n))) return -1;
    *joint->steps = 0;
    if (pin_err(hal_pin_float_newf(HAL_IN, &joint->pgain, comp_id, "%s.joint.%01d.pgain", prefix, n))) return -1;
    *joint->pgain = 0.0;
    if (pin_err(hal_pin_float_newf(HAL_IN, &joint->ff1gain, comp_id, "%s.joint.%01d.ff1gain", prefix, n))) return -1;
    *joint->ff1gain = 0.0;
    if (pin_err(hal_pin_float_newf(HAL_IN, &joint->deadband, comp_id, "%s.joint.%01d.deadband", prefix, n))) return -1;
    *joint->deadband = 0.0;
    if (pin_err(hal_param_float_newf(HAL_RW, &joint->maxaccel, comp_id, "%s.joint.%01d.maxaccel", prefix, n)))
      return -1;
    joint->maxaccel = 1.0;
    if (pin_err(hal_param_float_newf(HAL_RW, &joint->maxvel, comp_id, "%s.joint.%01d.maxvel", prefix, n))) return -1;
    joint->maxvel = 0.0;
  }

  const char *output_var_names[OUTPUT_VARS] = OUTPUT_VAR_NAMES;
  for (n = 0; n < OUTPUT_VARS; n++) {
    if (pin_err(hal_pin_float_newf(HAL_IN, &state->output_vars[n], comp_id, "%s.output_vars.%s", prefix,
                                   output_var_names[n])))
      return -1;
    *state->output_vars[n] = 0;
  }

  const char *input_var_names[INPUT_VARS] = INPUT_VAR_NAMES;
  for (n = 0; n < INPUT_VARS; n++) {
    if (pin_err(hal_pin_float_newf(HAL_OUT, &state->input_vars[n], comp_id, "%s.input_vars.%s", prefix,
                                   input_var_names[n])))
      return -1;
    *state->input_vars[n] = 0;
  }

  const char *output_pin_names[OUTPUT_PINS] = OUTPUT_PIN_NAMES;
  for (n = 0; n < OUTPUT_PINS; n++) {
    if (pin_err(hal_pin_bit_newf(HAL_IN, &state->output_pins[n], comp_id, "%s.output.%s", prefix, output_pin_names[n])))
      return -1;
    *state->output_pins[n] = 0;
  }

  const char *input_pin_names[INPUT_PINS] = INPUT_PIN_NAMES;
  for (n = 0; n < INPUT_PINS; n++) {
    if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->input_pins[n], comp_id, "%s.input.%s", prefix, input_pin_names[n])))
      return -1;
    // inverted 'not' pins, offset by the number of inputs we have
    if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->input_pins[n + INPUT_PINS], comp_id, "%s.input.%s.not", prefix,
                                 input_pin_names[n])))
      return -1;
    *state->input_pins[n] = 0;
  }

  // Export functions
  char name[HAL_NAME_LEN + 1];

  rtapi_snprintf(name, sizeof(name), "%s.calc-stepper-commands", prefix);
  int retval = hal_export_funct(name, calc_stepper_commands, state, 1, 0, comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: calc-stepper-commands function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  rtapi_snprintf(name, sizeof(name), "%s.prepare-tx", prefix);
  retval = hal_export_funct(name, prepare_tx, state, 1, 0, comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: prepare-tx function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  rtapi_snprintf(name, sizeof(name), "%s.handle-rx", prefix);
  retval = hal_export_funct(name, handle_rx, state, 1, 0, comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: handle-rx function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  rtapi_snprintf(name, sizeof(name), "%s.spi-transceive", prefix);
  retval = hal_export_funct(name, spi_transceive, state, 1, 0, comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: spi-transceive function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
  hal_ready(comp_id);
  return 0;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }

void calc_stepper_commands(void *arg, long period) {
  const state_t *state = arg;

  // precalculate timing constants
  const double periodfp = period * 0.000000001;
  double periodrecip = 1.0 / periodfp;

  // calc constants related to the period of this function (LinuxCNC SERVO_THREAD)
  // only recalc constants if period changes
  if (period != old_dtns)  // Note!! period = LinuxCNC SERVO_PERIOD
  {
    old_dtns = period;          // get ready to detect future period changes
    dt = period * 0.000000001;  // dt is the period of this thread, used for the position loop
    recip_dt = 1.0 / dt;        // calc the reciprocal once here, to avoid multiple divides later
  }

  // loop through generators
  for (int i = 0; i < JOINTS; i++) {
    joint_state_t *joint = state->joints[i];

    // check for scale change
    if (joint->pos_scale != joint->old_scale) {
      joint->old_scale = joint->pos_scale;  // get ready to detect future scale changes
      // scale must not be 0
      if (joint->pos_scale < 1e-20 && joint->pos_scale > -1e-20)
        joint->pos_scale = 1.0;  // value too small, divide by zero is a bad thing
    }

    // calculate frequency limit
    double max_freq = BASE_THREAD_FREQUENCY;

    // check for user specified frequency limit parameter
    if (joint->maxvel <= 0.0) {
      // set to zero if negative
      joint->maxvel = 0.0;
    } else {
      // parameter is non-zero, compare to max_freq
      double desired_freq = joint->maxvel * fabs(joint->pos_scale);

      if (desired_freq > max_freq) {
        // parameter is too high, limit it
        joint->maxvel = max_freq / fabs(joint->pos_scale);
      } else {
        // lower max_freq to match parameter
        max_freq = joint->maxvel * fabs(joint->pos_scale);
      }
    }

    /* set internal accel limit to its absolute max, which is
    zero to full speed in one thread period */
    double max_ac = max_freq * recip_dt;

    // check for user specified accel limit parameter
    if (joint->maxaccel <= 0.0) {
      // set to zero if negative
      joint->maxaccel = 0.0;
    } else {
      // parameter is non-zero, compare to max_ac
      if ((joint->maxaccel * fabs(joint->pos_scale)) > max_ac) {
        // parameter is too high, lower it
        joint->maxaccel = max_ac / fabs(joint->pos_scale);
      } else {
        // lower limit to match parameter
        max_ac = joint->maxaccel * fabs(joint->pos_scale);
      }
    }

    /* at this point, all scaling, limits, and other parameter
    changes have been handled - time for the main control */

    double vel_cmd;

    double command = *joint->pos_cmd;
    double error = command - *joint->pos_fb;

    // use Proportional control with feed forward (pgain, ff1gain and deadband)
    // if outside of error deadband, calculate vel_cmd values, else zero error and do nothing.
    const float deadband = *joint->deadband == 0 ? 1 / joint->pos_scale : *joint->deadband;
    if (fabs(error) > fabs(deadband)) {
      // calculate command and derivatives
      joint->cmd_d = (command - joint->prev_cmd) * periodrecip;

      // save old values
      joint->prev_cmd = command;

      // calculate the output value
      const float pgain = *joint->pgain == 0 ? 1.0 : *joint->pgain;
      const float ff1gain = *joint->ff1gain == 0 ? 1.0 : *joint->ff1gain;
      vel_cmd = pgain * error + joint->cmd_d * ff1gain;
    } else {
      error = 0;  // this doesn't actually do anything now because we're not bothering to calculate vel_cmd in
                  // deadband. But leaving here for now for readability.
      vel_cmd = 0;
    }

    vel_cmd = vel_cmd * joint->pos_scale;

    // apply frequency limit
    if (vel_cmd > max_freq) {
      vel_cmd = max_freq;
    } else if (vel_cmd < -max_freq) {
      vel_cmd = -max_freq;
    }

    // calc max change in frequency in one period
    const double dv = max_ac * dt;

    // apply accel limit
    if (vel_cmd > joint->freq + dv) {
      vel_cmd = joint->freq + dv;
    } else if (vel_cmd < joint->freq - dv) {
      vel_cmd = joint->freq - dv;
    }

    // test for disabled stepgen
    if (*joint->enable == 0) {
      // set velocity to zero
      vel_cmd = 0;
    }

    joint->freq = vel_cmd;           // to be sent to the PRU
    *joint->freq_cmd = joint->freq;  // feedback to LinuxCNC
  }
}

void prepare_tx() {
  int i;

  // prepare the data to write

  // Joint frequency commands
  for (i = 0; i < JOINTS; i++) {
    joint_state_t *joint = state->joints[i];
    txData.joint_freq_command[i] = joint->freq;

    if (*joint->enable == 1) {
      txData.joint_enable |= 1 << i;
    } else {
      txData.joint_enable &= ~(1 << i);
    }
  }

  for (i = 0; i < OUTPUT_VARS; i++) {
    txData.vars[i] = *(state->output_vars[i]);
  }

  for (i = 0; i < OUTPUT_PINS; i++) {
    if (*state->output_pins[i] == 1) {
      txData.outputs |= 1 << i;
    } else {
      txData.outputs &= ~(1 << i);
    }
  }
}

void set_hard_reset_pin() {
  if (*state->hard_reset) {
    if (bcm) {
      bcm2835_gpio_set(reset_gpio_pin);
    } else if (rp1) {
      gpio_set(reset_gpio_pin);
    }
  } else {
    if (bcm) {
      bcm2835_gpio_clr(reset_gpio_pin);
    } else if (rp1) {
      gpio_clear(reset_gpio_pin);
    }
  }
}

void handle_rx() {
  if (!*state->spi_enable) {
    *state->spi_status = 0;
    return;
  }

  if (rxData.header != SPI_DATA_HEADER || rxData.footer != SPI_DATA_FOOTER) {
    // we have received a BAD payload from the PRU
    rtapi_print("bad SPI payload!\n");
    *state->spi_status = 0;
    return;
  }

  if (rxData.e_stop_triggered) {
    // we have an eStop notification from the PRU
    *state->spi_status = 0;
    rtapi_print_msg(RTAPI_MSG_ERR, "An E-stop is active");
    return;
  }

  *state->spi_status = 1;

  int i;
  for (i = 0; i < JOINTS; i++) {
    joint_state_t *joint = state->joints[i];
    int steps = *joint->steps;
    const int prev_steps = joint->prev_steps;
    joint->prev_steps = steps;
    steps = rxData.joint_feedback[i];
    const int diff = steps - prev_steps;

    // spike filter
    const int M = 250;
    const int n = 2;
    if (abs(diff) > M && joint->filter_count < n) {
      // recent big change: hold previous value
      ++joint->filter_count;
      steps = prev_steps;
      rtapi_print("Spike filter active[%d][%d]: %d\n", i, joint->filter_count, diff);
    } else {
      // normal operation, or else the big change must be real after all
      joint->filter_count = 0;
    }

    *joint->steps = steps;
    *joint->pos_fb = (float)steps / joint->pos_scale;
  }

  for (i = 0; i < INPUT_VARS; i++) {
    *state->input_vars[i] = rxData.vars[i];
  }

  for (i = 0; i < INPUT_PINS; i++) {
    const int input = (rxData.inputs & (1 << i)) >> i;
    *state->input_pins[i] = input;
    *state->input_pins[i + INPUT_PINS] = !input;
  }
}

void spi_transceive() {
  set_hard_reset_pin();

  bool out_of_reset = *state->spi_reset && !state->spi_reset_old;
  if (out_of_reset) {
    rtapi_print("out of reset\n");
  }
  state->spi_reset_old = *state->spi_reset;

  if (!out_of_reset && !*state->spi_status) return;

  if (bcm == true) {
    // bcm2835_spi_transfernb(txBuffer, rxBuffer, sizeof(txData_t));
    for (int i = 0; i < sizeof(txData); i++) {
      rxBuffer[i] = bcm2835_spi_transfer(txBuffer[i]);
    }
  } else if (rp1 == true) {
    rp1spi_transfer(0, &txData, &rxData, sizeof(txData));
  }

  rtapi_print("transmitted %d bytes: ", sizeof(txData));
  for (int i = 0; i < sizeof(txData); i++) {
    rtapi_print("%02X ", (uint8_t)txBuffer[i]);
  }
  rtapi_print("\n");
  rtapi_print("received %d bytes: ", sizeof(rxData));
  for (int i = 0; i < sizeof(rxData); i++) {
    rtapi_print("%02X ", (uint8_t)rxBuffer[i]);
  }
  rtapi_print("\n");
}

int rt_peripheral_init(void) {
  FILE *fp;

  // assume we're only running on >RPi3

  if ((fp = fopen("/proc/device-tree/soc/ranges", "rb"))) {
    unsigned char buf[16];
    uint32_t base_address;
    uint32_t peri_size;
    if (fread(buf, 1, sizeof(buf), fp) >= 8) {
      base_address = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | (buf[7] << 0);

      peri_size = (buf[8] << 24) | (buf[9] << 16) | (buf[10] << 8) | (buf[11] << 0);

      if (!base_address) {
        /* looks like RPI 4 or 5 */
        base_address = (buf[8] << 24) | (buf[9] << 16) | (buf[10] << 8) | (buf[11] << 0);

        peri_size = (buf[12] << 24) | (buf[13] << 16) | (buf[14] << 8) | (buf[15] << 0);
      }

      // rtapi_print_msg(RTAPI_MSG_ERR,"\nRPi peripheral: Base address 0x%08x size 0x%08x\n", base_address, peri_size);
    }

    if (base_address == BCM2835_RPI2_PERI_BASE) {
      rtapi_print_msg(RTAPI_MSG_INFO, "Raspberry Pi 3, using BCM2835 driver\n\n");
      bcm = true;
    } else if (base_address == BCM2835_RPI4_PERI_BASE) {
      rtapi_print_msg(RTAPI_MSG_INFO, "Raspberry Pi 4, using BCM2835 driver\n\n");
      bcm = true;
    } else if (peri_size == RPI5_RP1_PERI_BASE) {
      // on the RPi 5, the base address is in the location of the peripheral size
      rtapi_print_msg(RTAPI_MSG_INFO, "Raspberry Pi 5, using RP1 driver\n\n");
      rp1 = true;
    } else {
      rtapi_print_msg(RTAPI_MSG_ERR, "Error, RPi not detected\n");
      return -1;
    }

    fclose(fp);
  }

  if (bcm == true) {
    // Map the RPi BCM2835 peripherals - uses "rtapi_open_as_root" in place of "open"
    if (!rt_bcm2835_init()) {
      rtapi_print_msg(RTAPI_MSG_ERR, "rt_bcm2835_init failed. Are you running with root privlages??\n");
      return -1;
    }

    // Set the SPI0 pins to the Alt 0 function to enable SPI0 access, setup CS register
    // and clear TX and RX fifos
    if (!bcm2835_spi_begin()) {
      rtapi_print_msg(RTAPI_MSG_ERR, "bcm2835_spi_begin failed. Are you running with root privlages??\n");
      return -1;
    }

    // Configure SPI0
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);  // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);               // The default

    // bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);
    //bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);
    // bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);

    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                  // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);  // the default

    /* RPI_GPIO_P1_19        = 10 		MOSI when SPI0 in use
     * RPI_GPIO_P1_21        =  9 		MISO when SPI0 in use
     * RPI_GPIO_P1_23        = 11 		CLK when SPI0 in use
     * RPI_GPIO_P1_24        =  8 		CE0 when SPI0 in use
     * RPI_GPIO_P1_26        =  7 		CE1 when SPI0 in use
     */

    // Configure pullups on SPI0 pins - source termination and CS high (does this allows for higher clock frequencies???
    // wiring is more important here)
    bcm2835_gpio_set_pud(RPI_GPIO_P1_19, BCM2835_GPIO_PUD_DOWN);  // MOSI
    bcm2835_gpio_set_pud(RPI_GPIO_P1_21, BCM2835_GPIO_PUD_DOWN);  // MISO
    bcm2835_gpio_set_pud(RPI_GPIO_P1_24, BCM2835_GPIO_PUD_UP);    // CS0
  } else if (rp1 == true) {
    if (!rt_rp1lib_init()) {
      rtapi_print_msg(RTAPI_MSG_ERR, "rt_rp1_init failed.\n");
      return -1;
    }

    // TODO: Allow user to select SPI number, CS number and frequency
    rp1spi_init(0, 0, SPI_MODE_0,
                40000000);  // SPIx, CSx, mode, freq. Clock frequency here is different than Pi4, this will get rounded
                            // to nearest clock divider. TODO Figure out exact value that works best with Flexi.
  } else {
    return -1;
  }

  return 1;
}
// This is the same as the standard bcm2835 library except for the use of
// "rtapi_open_as_root" in place of "open"

int rt_bcm2835_init(void) {
  if (debug) {
    bcm2835_peripherals = (uint32_t *)BCM2835_PERI_BASE;

    bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS / 4;
    bcm2835_clk = bcm2835_peripherals + BCM2835_CLOCK_BASE / 4;
    bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE / 4;
    bcm2835_pwm = bcm2835_peripherals + BCM2835_GPIO_PWM / 4;
    bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE / 4;
    bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE / 4;
    bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE / 4;
    bcm2835_st = bcm2835_peripherals + BCM2835_ST_BASE / 4;
    bcm2835_aux = bcm2835_peripherals + BCM2835_AUX_BASE / 4;
    bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE / 4;

    return 1; /* Success */
  }

  /* Figure out the base and size of the peripheral address block
  // using the device-tree. Required for RPi2/3/4, optional for RPi 1
  */
  FILE *fp;
  if ((fp = fopen(BMC2835_RPI2_DT_FILENAME, "rb"))) {
    unsigned char buf[16];
    if (fread(buf, 1, sizeof(buf), fp) >= 8) {
      uint32_t base_address = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | (buf[7] << 0);

      uint32_t peri_size = (buf[8] << 24) | (buf[9] << 16) | (buf[10] << 8) | (buf[11] << 0);

      if (!base_address) {
        /* looks like RPI 4 */
        base_address = (buf[8] << 24) | (buf[9] << 16) | (buf[10] << 8) | (buf[11] << 0);

        peri_size = (buf[12] << 24) | (buf[13] << 16) | (buf[14] << 8) | (buf[15] << 0);
      }
      /* check for valid known range formats */
      if ((buf[0] == 0x7e) && (buf[1] == 0x00) && (buf[2] == 0x00) && (buf[3] == 0x00) &&
          ((base_address == BCM2835_PERI_BASE) || (base_address == BCM2835_RPI2_PERI_BASE) ||
           (base_address == BCM2835_RPI4_PERI_BASE))) {
        bcm2835_peripherals_base = (off_t)base_address;
        bcm2835_peripherals_size = (size_t)peri_size;
        if (base_address == BCM2835_RPI4_PERI_BASE) {
          pud_type_rpi4 = 1;
        }
      }
    }

    fclose(fp);
  }
  /* else we are prob on RPi 1 with BCM2835, and use the hardwired defaults */

  /* Now get ready to map the peripherals block
   * If we are not root, try for the new /dev/gpiomem interface and accept
   * the fact that we can only access GPIO
   * else try for the /dev/mem interface and get access to everything
   */
  int memfd = -1;
  int ok = 0;
  if (geteuid() == 0) {
    /* Open the master /dev/mem device */
    if ((memfd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC)) < 0) {
      fprintf(stderr, "bcm2835_init: Unable to open /dev/mem: %s\n", strerror(errno));
      goto exit;
    }

    /* Base of the peripherals block is mapped to VM */
    bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
    if (bcm2835_peripherals == MAP_FAILED) goto exit;

    /* Now compute the base addresses of various peripherals,
    // which are at fixed offsets within the mapped peripherals block
    // Caution: bcm2835_peripherals is uint32_t*, so divide offsets by 4
    */
    bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE / 4;
    bcm2835_pwm = bcm2835_peripherals + BCM2835_GPIO_PWM / 4;
    bcm2835_clk = bcm2835_peripherals + BCM2835_CLOCK_BASE / 4;
    bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS / 4;
    bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE / 4;
    bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE / 4; /* I2C */
    bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE / 4; /* I2C */
    bcm2835_st = bcm2835_peripherals + BCM2835_ST_BASE / 4;
    bcm2835_aux = bcm2835_peripherals + BCM2835_AUX_BASE / 4;
    bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE / 4;

    ok = 1;
  } else {
    /* Not root, try /dev/gpiomem */
    /* Open the master /dev/mem device */
    if ((memfd = open("/dev/gpiomem", O_RDWR | O_SYNC)) < 0) {
      fprintf(stderr, "bcm2835_init: Unable to open /dev/gpiomem: %s\n", strerror(errno));
      goto exit;
    }

    /* Base of the peripherals block is mapped to VM */
    bcm2835_peripherals_base = 0;
    bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
    if (bcm2835_peripherals == MAP_FAILED) goto exit;
    bcm2835_gpio = bcm2835_peripherals;
    ok = 1;
  }

exit:
  if (memfd >= 0) close(memfd);

  if (!ok) bcm2835_close();

  return ok;
}

int rt_rp1lib_init(void) {
  uint64_t phys_addr = RP1_BAR1;

  DEBUG_PRINT("Initialising RP1 library: %s\n", __func__);

  // rp1_chip is declared in gpiochip_rp1.c
  chip = &rp1_chip;

  inst = rp1_create_instance(chip, phys_addr, NULL);
  if (!inst) return -1;

  inst->phys_addr = phys_addr;

  // map memory
  inst->mem_fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC);
  if (inst->mem_fd < 0) return errno;

  inst->priv = mmap(NULL, RP1_BAR1_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, inst->mem_fd, inst->phys_addr);

  DEBUG_PRINT("Base address: %11lx, size: %lx, mapped at address: %p\n", inst->phys_addr, RP1_BAR1_LEN, inst->priv);

  if (inst->priv == MAP_FAILED) return errno;

  return 1;
}
