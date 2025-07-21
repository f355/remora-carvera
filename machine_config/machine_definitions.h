#ifndef MACHINE_DEFINITIONS_H
#define MACHINE_DEFINITIONS_H

#define BASE_THREAD_FREQUENCY 100000  // 100kHz

#define JOINTS 4
#define INPUT_PINS 13
#define INPUT_VARS 3
#define OUTPUT_PINS 7
#define OUTPUT_VARS 3

#define INPUT_PIN_NAMES                                                                                                \
  {"stall-alarm-spindle", "stall-alarm-x", "stall-alarm-y", "stall-alarm-z", "lid-sensor", "main-button",       "ext", \
   "endstop-x",           "endstop-y",     "endstop-z",     "endstop-a",     "probe",      "tool-length-sensor"}
#define OUTPUT_PIN_NAMES {"work-light", "probe-power", "beeper", "ext", "power-12v", "power-24v", "axis-enable-a"}

#define INPUT_VAR_NAMES {"spindle-feedback", "spindle-temperature", "power-supply-temperature"}
#define OUTPUT_VAR_NAMES {"spindle-duty", "spindle-fan-duty", "power-supply-fan-duty"}

#endif