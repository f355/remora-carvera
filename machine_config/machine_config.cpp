#include "machine_config.h"

#include "digital_pin/digital_pin.h"
#include "e_stop/eStop.h"
#include "module.h"
#include "pulse_counter/pulseCounter.h"
#include "pwm/pwm.h"
#include "reset_pin/resetPin.h"
#include "stepgen/stepgen.h"
#include "thermistor/thermistor.h"

// Carvera Air CA1 configuration

vector<MachineThread*> configure_threads(Comms* comms) {
  printf("\ncreating threads for CA1\n");

  // Args: frequency, IRQ priority, timer number
  auto base_thread = new MachineThread(1, BASE_THREAD_FREQUENCY, 2);
  printf("created base thread\n");
  auto servo_thread = new MachineThread(2, 1000, 3);
  printf("created servo thread\n");

  const std::vector<Module*> base_modules = {
      // Axis step generators
      new Stepgen(0, new Pin(1, 28), (new Pin(1, 29))->invert(), base_thread->frequency, comms->ptr_rx_data,
                  comms->ptr_tx_data),  // X
      new Stepgen(1, new Pin(1, 26), new Pin(1, 27), base_thread->frequency, comms->ptr_rx_data,
                  comms->ptr_tx_data),  // Y
      new Stepgen(2, new Pin(1, 24), (new Pin(1, 25))->invert(), base_thread->frequency, comms->ptr_rx_data,
                  comms->ptr_tx_data),  // Z
      new Stepgen(3, new Pin(1, 21), (new Pin(1, 23))->invert(), base_thread->frequency, comms->ptr_rx_data,
                  comms->ptr_tx_data)  // A
  };
  printf("created base modules\n");

  for (const auto m : base_modules) base_thread->register_module(m);
  printf("registered base modules\n");

  // the beeper is obnoxious, shut it up ASAP
  Pin* beeper_pin = (new Pin(1, 14))->pull_down();
  beeper_pin->set(false);

  const std::vector<Module*> servo_modules = {
      // "special" pins
      new ResetPin(comms->pru_reset, new Pin(2, 10)),  // "hard" reset
      new eStop(comms->ptr_tx_data, new Pin(0, 20)),   // e-stop button

      // PWMs
      // on LPC1768, the period is shared among all PWMs,
      // so don't try setting it to different values - the last one wins.
      // many bothans died to bring us this information.

      new PWM(0, new Pin(2, 5), 10000, comms->ptr_rx_data),  // spindle
      new PWM(1, new Pin(2, 1), 10000, comms->ptr_rx_data),  // spindle fan
      new PWM(2, new Pin(2, 3), 10000, comms->ptr_rx_data),  // power supply fan

      // feedback
      new PulseCounter(0, new Pin(2, 7), comms->ptr_tx_data),  // spindle encoder feedback

      // thermistors
      new Thermistor(1, new Pin(1, 31), 3950, 100000, 25, servo_thread->frequency, comms->ptr_tx_data),  // spindle
      new Thermistor(2, new Pin(0, 26), 3950, 100000, 25, servo_thread->frequency, comms->ptr_tx_data),  // power supply

      // output pins
      new OutputPin(0, new Pin(2, 0), comms->ptr_rx_data),               // work light
      new OutputPin(1, new Pin(0, 11), comms->ptr_rx_data),              // probe power
      new OutputPin(2, beeper_pin, comms->ptr_rx_data),                  // beeper
      new OutputPin(3, new Pin(0, 21), comms->ptr_rx_data),              // EXT output
      new OutputPin(4, new Pin(0, 22), comms->ptr_rx_data),              // 12V power
      new OutputPin(5, new Pin(0, 10), comms->ptr_rx_data),              // 24V power
      new OutputPin(6, (new Pin(1, 30))->invert(), comms->ptr_rx_data),  // rotary A axis enable

      // input pins
      new InputPin(0, (new Pin(0, 19))->pull_up(), comms->ptr_tx_data),            // spindle stall alarm
      new InputPin(1, (new Pin(0, 1))->pull_up(), comms->ptr_tx_data),             // X stall alarm
      new InputPin(2, (new Pin(0, 0))->pull_up(), comms->ptr_tx_data),             // Y stall alarm
      new InputPin(3, (new Pin(3, 25))->pull_up(), comms->ptr_tx_data),            // Z stall alarm
      new InputPin(4, (new Pin(1, 8))->pull_up()->invert(), comms->ptr_tx_data),   // lid sensor
      new InputPin(5, (new Pin(2, 13))->pull_up()->invert(), comms->ptr_tx_data),  // main button
      new InputPin(6, (new Pin(2, 2))->pull_down(), comms->ptr_tx_data),           // EXT input
      new InputPin(7, (new Pin(0, 24))->pull_up(), comms->ptr_tx_data),            // X home switch/endstop
      new InputPin(8, (new Pin(0, 25))->pull_up(), comms->ptr_tx_data),            // Y home switch/endstop
      new InputPin(9, (new Pin(1, 1))->pull_up(), comms->ptr_tx_data),             // Z home switch/endstop
      new InputPin(10, (new Pin(1, 9))->pull_up(), comms->ptr_tx_data),            // A home switch
      new InputPin(11, (new Pin(2, 6))->pull_down(), comms->ptr_tx_data),          // probe signal
      new InputPin(12, (new Pin(0, 5))->pull_up(), comms->ptr_tx_data)             // tool length sensor
  };

  printf("created servo modules\n");

  for (const auto m : servo_modules) servo_thread->register_module(m);
  printf("registered servo modules\n");

  return {base_thread, servo_thread};
}
