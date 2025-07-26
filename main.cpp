
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
#include "MachineThread.h"

// state machine
enum State { ST_START = 0, ST_RUNNING, ST_RESET };

[[noreturn]]
int main() {
  printf("\nPRU - Programmable Realtime Unit\n");

  NVIC_SetPriority(EINT3_IRQn, 4);

  Watchdog& watchdog = Watchdog::get_instance();

  State current_state = ST_START;
  State prev_state = ST_RESET;

  const auto comms = new Comms();
  comms->start();

  create_timers();

  // Start the threads
  for (const auto thread : configure_threads(comms)) {
    thread->start();
  }

  // wait 1 second for threads to read IO before testing for PRUreset
  wait_us(1000000);

  watchdog.start(2000);

  while (true) {
    watchdog.kick();

    switch (current_state) {
      case ST_START:
        // do start tasks
        if (current_state != prev_state) {
          printf("\nSTART: waiting for LinuxCNC...\n");
        }
        prev_state = current_state;

        if (!comms->pru_reset) {
          // RPi outputs default is high until configured when LinuxCNC Remora component is started, PRUreset pin will
          // be high stay in start state until LinuxCNC is started
          current_state = ST_RUNNING;
        }

        break;

      case ST_RUNNING:
        // do running tasks
        if (current_state != prev_state) {
          printf("\nRUNNING...\n");
        }
        prev_state = current_state;

        // check to see if there has been SPI errors
        if (comms->get_error()) {
          printf("Communication data error, resetting the PRU\n");
          current_state = ST_RESET;
        }

        // LinuxCNC wants us to reset
        if (comms->pru_reset) {
          printf("Reset requested by LinuxCNC\n");
          current_state = ST_RESET;
        }

        break;

      case ST_RESET:
        printf("\nRESET\n");

        // force a watchdog reset by looping here
        while (true) {
        }
    }

    wait_us(100000);  // wait for 0.1 seconds
  }
}
