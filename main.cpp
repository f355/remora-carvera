
/*
Remora PRU firmware for LinuxCNC
Copyright (C) 2021  Scott Alford (scotta)

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License version 2
of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

// MBED includes
#include <cstdio>

#include "machine_config.h"
#include "mbed.h"

// drivers
#include "comms.h"
#include "pin.h"

// threads
#include "ThisThread.h"
#include "cncThread.h"

// state machine
enum State { ST_SETUP = 0, ST_START, ST_IDLE, ST_RUNNING, ST_RESET, ST_WDRESET };

int main() {
  NVIC_SetPriority(EINT3_IRQn, 4);

  vector<CNCThread*> threads;
  bool threads_running = false;

  // Watchdog
  Watchdog& watchdog = Watchdog::get_instance();
  const auto comms = new Comms();

  uint8_t reset_count = 0;
  State current_state = ST_SETUP;
  State prev_state = ST_RESET;

  comms->not_ready();
  comms->clear_error();

  printf("\nRemora PRU - Programmable Realtime Unit\n");

  watchdog.start(2000);

  while (true) {
    // the main loop does very little, keeping the Watchdog serviced and
    // resetting the rxData buffer if there is a loss of SPI commmunication
    // with LinuxCNC. Everything else is done via DMA and within the
    // two threads - Base and Servo threads that run the Modules.

    watchdog.kick();

    switch (current_state) {
      case ST_SETUP: {
        // do setup tasks
        if (current_state != prev_state) {
          printf("\n## Entering SETUP state\n");
        }
        prev_state = current_state;

        printf("\nSetting up DMA and threads\n");

        comms->start();
        create_timers();
        threads = configure_threads(comms);

        current_state = ST_START;
        break;
      }
      case ST_START:
        // do start tasks
        if (current_state != prev_state) {
          printf("\n## Entering START state\n");
        }
        prev_state = current_state;

        if (!threads_running) {
          // Start the threads
          for (const auto thread : threads) {
            thread->start();
          }

          threads_running = true;

          // wait 1 second for threads to read IO before testing for PRUreset
          rtos::ThisThread::sleep_for(1000);
        }

        if (comms->pru_reset) {
          // RPi outputs default is high until configured when LinuxCNC Remora component is started, PRUreset pin will
          // be high stay in start state until LinuxCNC is started
          current_state = ST_START;
        } else {
          current_state = ST_IDLE;
        }

        break;

      case ST_IDLE:
        // do something when idle
        if (current_state != prev_state) {
          printf("\n## Entering IDLE state\n");
        }
        prev_state = current_state;

        if (comms->get_error()) {
          printf("Communication data error\n");
          comms->clear_error();
        }

        // wait for SPI data before changing to running state
        if (comms->is_ready()) {
          current_state = ST_RUNNING;
        }

        if (comms->pru_reset) {
          current_state = ST_WDRESET;
        }

        break;

      case ST_RUNNING:
        // do running tasks
        if (current_state != prev_state) {
          printf("\n## Entering RUNNING state\n");
        }
        prev_state = current_state;

        // check to see if there has been SPI errors
        if (comms->get_error()) {
          printf("Communication data error\n");
          comms->clear_error();
        }

        if (comms->is_ready()) {
          // SPI data received by DMA
          reset_count = 0;
          comms->not_ready();
        } else {
          // no good data received by DMA
          reset_count++;
        }

        if (reset_count > 5) {
          // reset threshold reached, reset the PRU
          printf("   Communication data error limit reached, resetting\n");
          reset_count = 0;
          current_state = ST_RESET;
        }

        if (comms->pru_reset) {
          current_state = ST_WDRESET;
        }

        break;

      case ST_RESET:
        // do reset tasks
        if (current_state != prev_state) {
          printf("\n## Entering RESET state\n");
        }
        prev_state = current_state;

        // zero out the rxData buffer
        // it is volatile, so memset cannot be used, and we need to do it the long way
        printf("   Resetting rxBuffer\n");
        {
          int n = sizeof(*comms->ptr_rx_data);
          volatile void* b = comms->ptr_rx_data;
          while (--n >= 0) {
            *((uint8_t*)b + n) = 0;
          }
        }

        current_state = ST_IDLE;
        break;

      case ST_WDRESET:
        printf("\n## Entering WDRESET state\n");

        // force a watchdog reset by looping here
        while (true) {
        }
    }

    rtos::ThisThread::sleep_for(100);  // wait for 0.1 seconds
  }
}
