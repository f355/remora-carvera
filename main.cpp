
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
#include <cerrno>
#include <cstdio>

#include "Comms.h"
#include "configuration.h"
#include "machine_config/machine_config.h"
#include "mbed.h"
#include "pruThread.h"

#define SPI_ERR_MAX 5

enum State { ST_SETUP = 0, ST_START, ST_IDLE, ST_RUNNING, ST_RESET, ST_WDRESET };

[[noreturn]] int main() {
  printf("\nPRU - Programmable Realtime Unit\n");

  uint8_t reset_count = 0;
  bool threads_running = false;

  vector<PRUThread*> threads;
  const auto comms = new Comms();
  comms->set_status(false);
  comms->set_error(false);

  State current_state = ST_SETUP;
  State prev_state = ST_RESET;

  Watchdog& watchdog = Watchdog::get_instance();
  watchdog.start(2000);

  while (true) {
    watchdog.kick();

    switch (current_state) {
      case ST_SETUP: {
        if (current_state != prev_state) {
          printf("\n## Entering SETUP state\n");
        }
        prev_state = current_state;

        printf("\nSetting up DMA and threads\n");

        comms->init();
        comms->start();

        create_timers();

        threads = configure_threads(comms);

        current_state = ST_START;
        break;
      }
      case ST_START:
        if (current_state != prev_state) {
          printf("\n## Entering START state\n");
        }
        prev_state = current_state;

        if (!threads_running) {
          for (const auto thread : threads) thread->start();

          threads_running = true;

          // wait for threads to read IO before testing for PRUreset
          wait_us(1000000);
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
        if (current_state != prev_state) {
          printf("\n## Entering IDLE state\n");
        }
        prev_state = current_state;

        if (comms->get_error()) {
          printf("Communication data error\n");
          comms->set_error(false);
        }

        if (comms->get_status()) {
          current_state = ST_RUNNING;
        }

        if (comms->pru_reset) {
          current_state = ST_WDRESET;
        }

        break;

      case ST_RUNNING:
        if (current_state != prev_state) {
          printf("\n## Entering RUNNING state\n");
        }
        prev_state = current_state;

        if (comms->get_error()) {
          printf("Communication data error\n");
          comms->set_error(false);
        }

        if (comms->get_status()) {
          reset_count = 0;
          comms->set_status(false);
        } else {
          reset_count++;
        }

        if (reset_count > SPI_ERR_MAX) {
          printf("   Communication data error limit reached, resetting\n");
          reset_count = 0;
          current_state = ST_RESET;
        }

        if (comms->pru_reset) {
          current_state = ST_WDRESET;
        }

        break;
      case ST_RESET:
        if (current_state != prev_state) {
          printf("\n## Entering RESET state\n");
        }
        prev_state = current_state;

        // set all of the rxData buffer to 0
        // rxData.rxBuffer is volatile so need to do this the long way. memset cannot be used for volatile
        printf("   Resetting rxBuffer\n");
        {
          int n = sizeof(comms->ptr_rx_data->rx_buffer);
          while (n-- > 0) {
            comms->ptr_rx_data->rx_buffer[n] = 0;
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

    wait_us(1000);
  }
}
