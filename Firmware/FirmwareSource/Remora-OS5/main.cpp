
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
#include "mbed.h"
#include <cstdio>
#include <cerrno>
#include <string>

#include "FATFileSystem.h"
#include "SDBlockDevice.h"

#include "configuration.h"

// libraries
#include "ArduinoJson.h"

// drivers
#include "RemoraComms.h"
#include "pin.h"

// threads
#include "pruThread.h"

// modules
#include "module.h"
#include "blink.h"
#include "digitalPin.h"
#include "eStop.h"
#include "motorPower.h"
#include "pulseCounter.h"
#include "pwm.h"
#include "rcservo.h"
#include "resetPin.h"
#include "stepgen.h"
#include "switch.h"
#include "thermistor.h"

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

// state machine
enum State {
    ST_SETUP = 0,
    ST_START,
    ST_IDLE,
    ST_RUNNING,
    ST_STOP,
    ST_RESET,
    ST_WDRESET
};

uint8_t resetCnt;

bool threadsRunning = false;

// Watchdog
Watchdog& watchdog = Watchdog::get_instance();

/***********************************************************************
        ROUTINES
************************************************************************/

string readJsonConfig()
{
    printf("1. Reading json configuration file\n");

    // Try to mount the filesystem
    printf("Mounting the filesystem... ");
    fflush(stdout);

    SDBlockDevice blockDevice(
        MBED_CONF_SD_SPI_MOSI,
        MBED_CONF_SD_SPI_MISO,
        MBED_CONF_SD_SPI_CLK,
        MBED_CONF_SD_SPI_CS
    );
    FATFileSystem fileSystem("fs");
    int err = fileSystem.mount(&blockDevice);

    printf("%s\n", (err ? "Fail :(" : "OK"));
    if (err) {
        error("No filesystem found... ");
     }

    // Open the config file
    printf("Opening \"/fs/config.json\"... ");
    fflush(stdout);
    FILE *jsonFile = fopen("/fs/config.json", "r+");
    if (!jsonFile) {
        error("Error opening config");
    }

    fseek (jsonFile, 0, SEEK_END);
    int32_t length = ftell (jsonFile);
    fseek (jsonFile, 0, SEEK_SET);

    printf("Json config file length = %2d\n", length);

    string strJson;

    strJson.reserve(length + 1);

    while (!feof(jsonFile)) {
        int c = fgetc(jsonFile);
        strJson.push_back(c);
    }

    // Remove comments from next line to print out the JSON config file
    //printf("%s\n", strJson.c_str());

    printf("\rClosing \"/fs/config.json\"... ");
    fflush(stdout);
    fclose(jsonFile);

    return strJson;
}


DynamicJsonDocument deserialiseJSON(const char *json)
{
    printf("\nParsing json configuration file\n");

    DynamicJsonDocument doc(JSON_BUFF_SIZE);
    DeserializationError err = deserializeJson(doc, json);

    printf("Config deserialisation - ");

    switch (err.code())
    {
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


void loadModules(PRUThread* thread, JsonArray modules, RemoraComms* comms)
{
    // create objects from json data
    for (JsonArray::iterator it = modules.begin(); it != modules.end(); ++it)
    {
        JsonObject moduleDef = *it;
        
        const char* type = moduleDef["type"];
        const char* comment = moduleDef["comment"];

        printf("creating %s module: [%s]\n", type, comment);

        Module* module = NULL;

        if (!strcmp(type, "stepgen"))
        {
            module = createStepgen(moduleDef, thread, comms);
        }
        else if (!strcmp(type, "rc_servo"))
        {
            module = createRCServo(moduleDef, thread, comms);
        }
        else if (!strcmp(type, "e_stop"))
        {
            module = createEStop(moduleDef, comms);
        }
        else if (!strcmp(type, "reset_pin"))
        {
            module = createResetPin(moduleDef, comms);
        }
        else if (!strcmp(type, "blink"))
        {
            module = createBlink(moduleDef, thread);
        }
        else if (!strcmp(type, "digital_pin"))
        {
            module = createDigitalPin(moduleDef, comms);
        }
        else if (!strcmp(type, "pulse_counter"))
        {
            module = createPulseCounter(moduleDef, comms);
        }
        else if (!strcmp(type, "pwm"))
        {
            module = createPWM(moduleDef, comms);
        }
        else if (!strcmp(type, "thermistor"))
        { 
            module = createThermistor(moduleDef, thread, comms);
        }
        else if (!strcmp(type, "switch"))
        {
            module = createSwitch(moduleDef, comms);
        }
        else if (!strcmp(type, "motor_power"))
        {
            createMotorPower(moduleDef);
        }
        else {
            error("module [%s]: unknown type [%s]\n", comment, type);
        }

        if (module) {
            thread->registerModule(module);
        }
    }
}

vector<PRUThread*> createThreads(DynamicJsonDocument doc, RemoraComms* comms)
{
    printf("\ncreating threads\n");

    vector<PRUThread*> threads;
    JsonObject threadDefs = doc["threads"];

    for (JsonObject::iterator it = threadDefs.begin(); it != threadDefs.end(); ++it)
    {
        const char* threadName = (*it).key().c_str();
        JsonObject threadDef = (*it).value();

        printf("creating thread %s\n", threadName);
        
        uint32_t frequency = threadDef["frequency"];
        uint32_t priority = threadDef["priority"];
        uint32_t timerNum = threadDef["timer_number"];

        PRUThread* thread = new PRUThread(timerNum, frequency, priority);

        loadModules(thread, threadDef["modules"], comms);

        threads.push_back(thread);
    }

    JsonArray runOnLoad = doc["run_on_load"];
    loadModules(NULL, runOnLoad, comms);

    return threads;
}

int main()
{
    vector<PRUThread*> threads;

    enum State currentState;
    enum State prevState;

    RemoraComms* comms = new RemoraComms();

    comms->setStatus(false);
    comms->setError(false);
    currentState = ST_SETUP;
    prevState = ST_RESET;

    printf("\nRemora PRU - Programmable Realtime Unit\n");

    watchdog.start(2000);

    while(1)
    {
      // the main loop does very little, keeping the Watchdog serviced and
      // resetting the rxData buffer if there is a loss of SPI commmunication
      // with LinuxCNC. Everything else is done via DMA and within the
      // two threads- Base and Servo threads that run the Modules.

    watchdog.kick();

    switch(currentState){
        case ST_SETUP:
        {
            // do setup tasks
            if (currentState != prevState)
            {
                printf("\n## Entering SETUP state\n");
            }
            prevState = currentState;

            printf("\nSetting up DMA and threads\n");

            // initialise the Remora comms 
            comms->init();
            comms->start();

            string strJson = readJsonConfig();
            DynamicJsonDocument doc = deserialiseJSON(strJson.c_str());

            createTimers();

            threads = createThreads(doc, comms);

            currentState = ST_START;
            break; 
        }
        case ST_START:
            // do start tasks
            if (currentState != prevState)
            {
                printf("\n## Entering START state\n");
            }
            prevState = currentState;

            if (!threadsRunning)
            {
                // Start the threads
                for (std::vector<PRUThread*>::const_iterator it = threads.begin(); it != threads.end(); ++it)
                {
                    (*it)->startThread();
                }

                threadsRunning = true;

                // wait for threads to read IO before testing for PRUreset
                wait(1);
            }

            if (comms->pruReset)
            {
                // RPi outputs default is high until configured when LinuxCNC Remora component is started, PRUreset pin will be high
                // stay in start state until LinuxCNC is started
                currentState = ST_START;
            }
            else
            {
                currentState = ST_IDLE;
            }
            
            break;


        case ST_IDLE:
            // do something when idle
            if (currentState != prevState)
            {
                printf("\n## Entering IDLE state\n");
            }
            prevState = currentState;

            // check to see if there there has been SPI errors
            if (comms->getError())
            {
                printf("Communication data error\n");
                comms->setError(false);
            }

            //wait for SPI data before changing to running state
            if (comms->getStatus())
            {
                currentState = ST_RUNNING;
            }

            if (comms->pruReset)
            {
                currentState = ST_WDRESET;
            }

            break;

        case ST_RUNNING:
            // do running tasks
            if (currentState != prevState)
            {
                printf("\n## Entering RUNNING state\n");
            }
            prevState = currentState;

            // check to see if there there has been SPI errors 
            if (comms->getError())
            {
                printf("Communication data error\n");
                comms->setError(false);
            }
            
            if (comms->getStatus())
            {
                // SPI data received by DMA
                resetCnt = 0;
                comms->setStatus(false);
            }
            else
            {
                // no data received by DMA
                resetCnt++;
            }

            if (resetCnt > SPI_ERR_MAX)
            {
                // reset threshold reached, reset the PRU
                printf("   Communication data error limit reached, resetting\n");
                resetCnt = 0;
                currentState = ST_RESET;
            }

            if (comms->pruReset)
            {
                currentState = ST_WDRESET;
            }

            break;

        case ST_STOP:
            // do stop tasks
            if (currentState != prevState)
            {
                printf("\n## Entering STOP state\n");
            }
            prevState = currentState;


            currentState = ST_STOP;
            break;

        case ST_RESET:
            // do reset tasks
            if (currentState != prevState)
            {
                printf("\n## Entering RESET state\n");
            }
            prevState = currentState;

            // set all of the rxData buffer to 0
            // rxData.rxBuffer is volatile so need to do this the long way. memset cannot be used for volatile
            printf("   Resetting rxBuffer\n");
            {
                int n = sizeof(comms->ptrRxData);
                volatile void* b = comms->ptrRxData;
                while(--n >= 0)
                {
                    *((uint8_t*)b + n) = 0;
                }
            }

            currentState = ST_IDLE;
            break;

        case ST_WDRESET:
            // do a watch dog reset
            printf("\n## Entering WDRESET state\n");

            // force a watchdog reset by looping here
            while(1){}

            break;
      }

    wait(LOOP_TIME);
    }
}
