
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
#include <string>

#include "configuration.h"
#include "mbed.h"

// libraries
#include "ArduinoJson.h"

// drivers
#include "Comms.h"
#include "pin.h"

// threads
#include "pruThread.h"

// modules
#include "digitalPin/digitalPin.h"
#include "eStop/eStop.h"
#include "module.h"
#include "pulseCounter/pulseCounter.h"
#include "pwm/pwm.h"
#include "resetPin/resetPin.h"
#include "stepgen/stepgen.h"
#include "thermistor/thermistor.h"

/***********************************************************************
 *                STRUCTURES AND GLOBAL VARIABLES                       *
 ************************************************************************/

// state machine
enum State { ST_SETUP = 0, ST_START, ST_IDLE, ST_RUNNING, ST_RESET, ST_WDRESET };

uint8_t reset_counter;

bool threads_running = false;

// Watchdog
Watchdog &watchdog = Watchdog::get_instance();

/***********************************************************************
        ROUTINES
************************************************************************/

string readJsonConfig() {
  return " {\"board\":\"Carvera Air "
         "CA1\",\"threads\":{\"base\":{\"timer_number\":1,\"frequency\":60000,"
         "\"priority\":2,\"modules\":[{\"comment\":\"X - Joint 0 step "
         "generator\",\"type\":\"stepgen\",\"joint\":0,\"step_pin\":\"1.28\","
         "\"direction_pin\":\"1.29!\"},{\"comment\":\"Y - Joint 1 step "
         "generator\",\"type\":\"stepgen\",\"joint\":1,\"step_pin\":\"1.26\","
         "\"direction_pin\":\"1.27\"},{\"comment\":\"Z - Joint 2 step "
         "generator\",\"type\":\"stepgen\",\"joint\":2,\"step_pin\":\"1.24\","
         "\"direction_pin\":\"1.25!\"},{\"comment\":\"A - Joint 3 step "
         "generator\",\"type\":\"stepgen\",\"joint\":3,\"step_pin\":\"1.21\","
         "\"direction_pin\":\"1.23!\"} "
         "]},\"servo\":{\"timer_number\":2,\"frequency\":1000,\"priority\":3,"
         "\"modules\":[{\"comment\":\"Reset "
         "pin\",\"type\":\"reset_pin\",\"pin\":\"2.10\"},{\"comment\":\"e-stop "
         "pin\",\"type\":\"e_stop\",\"pin\":\"0.20\"},{\"comment\":\"Spindle "
         "(PWM period is "
         "shared!)\",\"type\":\"pwm\",\"set_point\":0,\"pwm_pin\":\"2.5\","
         "\"period_us\":10000},{\"comment\":\"Spindle "
         "feedback\",\"type\":\"pulse_counter\",\"process_variable\":0,\"pin\":"
         "\"2.7\"},{\"comment\":\"Power fan (PWM period is "
         "shared!)\",\"type\":\"pwm\",\"set_point\":1,\"pwm_pin\":\"2.3\","
         "\"period_us\":10000},{\"comment\":\"Spindle fan (PWM period is "
         "shared!)\",\"type\":\"pwm\",\"set_point\":2,\"pwm_pin\":\"2.1\","
         "\"period_us\":10000},{\"comment\":\"Light "
         "switch\",\"type\":\"digital_pin\",\"mode\":\"out\",\"data_bit\":0,"
         "\"pin\":\"2.0\"},{\"comment\":\"Tool sensor "
         "switch\",\"type\":\"digital_pin\",\"mode\":\"out\",\"data_bit\":1,"
         "\"pin\":\"0.11\"},{\"comment\":\"Beep "
         "switch\",\"type\":\"digital_pin\",\"mode\":\"out\",\"data_bit\":2,"
         "\"pin\":\"1.14v\"},{\"comment\":\"EXT "
         "out\",\"type\":\"digital_pin\",\"mode\":\"out\",\"data_bit\":3,"
         "\"pin\":\"0.21\"},{\"comment\":\"12V power "
         "switch\",\"mode\":\"out\",\"data_bit\":4,\"type\":\"digital_pin\","
         "\"pin\":\"0.22\"},{\"comment\":\"24V power "
         "switch\",\"mode\":\"out\",\"data_bit\":5,\"type\":\"digital_pin\","
         "\"pin\":\"0.10\"},{\"comment\":\"A axis "
         "enable\",\"mode\":\"out\",\"data_bit\":6,\"type\":\"digital_pin\","
         "\"pin\":\"1.30!\"},{\"comment\":\"Spindle "
         "alarm\",\"mode\":\"in\",\"data_bit\":0,\"type\":\"digital_pin\","
         "\"pin\":\"0.19^\"},{\"comment\":\"X axis stall "
         "alarm\",\"type\":\"digital_pin\",\"mode\":\"in\",\"data_bit\":1,"
         "\"pin\":\"0.1^\"},{\"comment\":\"Y axis stall "
         "alarm\",\"type\":\"digital_pin\",\"mode\":\"in\",\"data_bit\":2,"
         "\"pin\":\"0.0^\"},{\"comment\":\"Z axis stall "
         "alarm\",\"type\":\"digital_pin\",\"mode\":\"in\",\"data_bit\":3,"
         "\"pin\":\"3.25^\"},{\"comment\":\"Lid "
         "sensor\",\"type\":\"digital_pin\",\"mode\":\"in\",\"data_bit\":4,"
         "\"pin\":\"1.8!^\"},{\"comment\":\"Main "
         "button\",\"type\":\"digital_pin\",\"mode\":\"in\",\"data_bit\":5,"
         "\"pin\":\"2.13!^\"},{\"comment\":\"EXT "
         "in\",\"type\":\"digital_pin\",\"mode\":\"in\",\"data_bit\":6,\"pin\":"
         "\"2.2v\"},{\"comment\":\"X "
         "endstop\",\"type\":\"digital_pin\",\"mode\":\"in\",\"data_bit\":7,"
         "\"pin\":\"0.24^\"},{\"comment\":\"Y "
         "endstop\",\"type\":\"digital_pin\",\"mode\":\"in\",\"data_bit\":8,"
         "\"pin\":\"0.25^\"},{\"comment\":\"Z "
         "endstop\",\"type\":\"digital_pin\",\"mode\":\"in\",\"data_bit\":9,"
         "\"pin\":\"1.1^\"},{\"comment\":\"A "
         "endstop\",\"type\":\"digital_pin\",\"mode\":\"in\",\"data_bit\":10,"
         "\"pin\":\"1.9^\"},{\"comment\":\"Probe\",\"type\":\"digital_pin\","
         "\"mode\":\"in\",\"data_bit\":11,\"pin\":\"2.6v\"},{\"comment\":"
         "\"Probe "
         "calibrate\",\"type\":\"digital_pin\",\"mode\":\"in\",\"data_bit\":12,"
         "\"pin\":\"0.5^\"},{\"comment\":\"Spindle "
         "temperature\",\"type\":\"thermistor\",\"process_variable\":1,\"pin\":"
         "\"1.31\",\"beta\":3950,\"r0\":100000,\"t0\":25},{\"comment\":\"Power "
         "supply "
         "temperature\",\"type\":\"thermistor\",\"process_variable\":2,\"pin\":"
         "\"0.26\",\"beta\":3950,\"r0\":100000,\"t0\":25} "
         "]}},\"run_on_load\":[]}";
}

DynamicJsonDocument deserialiseJSON(const char *json) {
  printf("\nParsing json configuration file\n");

  DynamicJsonDocument doc(JSON_BUFF_SIZE);
  const DeserializationError err = deserializeJson(doc, json);

  printf("Config deserialisation - ");

  switch (err.code()) {
    case DeserializationError::Ok:
      printf("Deserialization succeeded\n");
      return doc;
    case DeserializationError::InvalidInput:
      error("Invalid input!\n");
      break;
    case DeserializationError::NoMemory:
      error("Not enough memory\n");
      break;
    default:
      error("Deserialization failed\n");
  }
}

void loadModules(PRUThread *thread, const JsonArray modules, Comms *comms) {
  // create objects from json data
  for (auto it : modules) {
    JsonObject moduleDef = it;

    const char *type = moduleDef["type"];
    const char *comment = moduleDef["comment"];

    printf("creating %s module: [%s]\n", type, comment);

    Module *module = nullptr;

    if (!strcmp(type, "stepgen")) {
      module = createStepgen(moduleDef, thread, comms);
    } else if (!strcmp(type, "e_stop")) {
      module = createEStop(moduleDef, comms);
    } else if (!strcmp(type, "reset_pin")) {
      module = createResetPin(moduleDef, comms);
    } else if (!strcmp(type, "digital_pin")) {
      module = createDigitalPin(moduleDef, comms);
    } else if (!strcmp(type, "pulse_counter")) {
      module = createPulseCounter(moduleDef, comms);
    } else if (!strcmp(type, "pwm")) {
      module = createPWM(moduleDef, comms);
    } else if (!strcmp(type, "thermistor")) {
      module = createThermistor(moduleDef, thread, comms);
    } else {
      error("module [%s]: unknown type [%s]\n", comment, type);
    }

    if (module) {
      thread->register_module(module);
    }
  }
}

vector<PRUThread *> createThreads(DynamicJsonDocument doc, Comms *comms) {
  printf("\ncreating threads\n");

  vector<PRUThread *> threads;
  JsonObject threadDefs = doc["threads"];

  for (auto it : threadDefs) {
    const char *threadName = it.key().c_str();
    JsonObject threadDef = it.value();

    printf("creating thread %s\n", threadName);

    const uint32_t frequency = threadDef["frequency"];
    const uint32_t priority = threadDef["priority"];
    const uint32_t timerNum = threadDef["timer_number"];

    auto thread = new PRUThread(timerNum, frequency, priority);

    loadModules(thread, threadDef["modules"], comms);

    threads.push_back(thread);
  }

  const JsonArray runOnLoad = doc["run_on_load"];
  loadModules(nullptr, runOnLoad, comms);

  return threads;
}

[[noreturn]] int main() {
  vector<PRUThread *> threads;

  const auto comms = new Comms();

  comms->set_status(false);
  comms->set_error(false);
  State current_state = ST_SETUP;
  State prev_state = ST_RESET;

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

        // initialise the Remora comms
        comms->init();
        comms->start();

        string strJson = readJsonConfig();
        DynamicJsonDocument doc = deserialiseJSON(strJson.c_str());

        create_timers();

        threads = createThreads(doc, comms);

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
        // do something when idle
        if (current_state != prev_state) {
          printf("\n## Entering IDLE state\n");
        }
        prev_state = current_state;

        // check to see if there there has been SPI errors
        if (comms->get_error()) {
          printf("Communication data error\n");
          comms->set_error(false);
        }

        // wait for SPI data before changing to running state
        if (comms->get_status()) {
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

        // check to see if there there has been SPI errors
        if (comms->get_error()) {
          printf("Communication data error\n");
          comms->set_error(false);
        }

        if (comms->get_status()) {
          // SPI data received by DMA
          reset_counter = 0;
          comms->set_status(false);
        } else {
          // no data received by DMA
          reset_counter++;
        }

        if (reset_counter > SPI_ERR_MAX) {
          // reset threshold reached, reset the PRU
          printf("   Communication data error limit reached, resetting\n");
          reset_counter = 0;
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
        // do a watch dog reset
        printf("\n## Entering WDRESET state\n");

        // force a watchdog reset by looping here
        while (true) {
        }
    }

    wait_us(LOOP_TIME);
  }
}
