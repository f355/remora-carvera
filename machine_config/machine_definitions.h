#ifndef MACHINE_DEFINITIONS_H
#define MACHINE_DEFINITIONS_H

#define BASE_THREAD_FREQUENCY 100000  // 100kHz

#define JOINTS 4
#define INPUT_PINS 13
#define INPUT_VARS 3
#define OUTPUT_PINS 7
#define OUTPUT_VARS 3

const char* input_pin_names[INPUT_PINS] = {
    "stall-alarm-spindle",  // 0
    "stall-alarm-x",        // 1
    "stall-alarm-y",        // 2
    "stall-alarm-z",        // 3
    "lid-sensor",           // 4
    "main-button",          // 5
    "ext",                  // 6
    "endstop-x",            // 7
    "endstop-y",            // 8
    "endstop-z",            // 9
    "endstop-a",            // 10
    "probe",                // 11
    "tool-length-sensor"    // 12
};

const char* input_var_names[INPUT_VARS] = {
    "spindle-feedback",         // 0
    "spindle-temperature",      // 1
    "power-supply-temperature"  // 2
};

const char* output_pin_names[OUTPUT_PINS] = {
    "work-light",    // 0
    "probe-power",   // 1
    "beeper",        // 2
    "ext",           // 3
    "power-12v",     // 4
    "power-24v",     // 5
    "axis-enable-a"  // 6
};

const char* output_var_names[OUTPUT_VARS] = {
    "spindle-duty",          // 0
    "spindle-fan-duty",      // 1
    "power-supply-fan-duty"  // 2
};

#endif