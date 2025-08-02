#include "machine_config.h"

#include "digitalPin/digitalPin.h"
#include "eStop/eStop.h"
#include "module.h"
#include "pulseCounter/pulseCounter.h"
#include "pwm/pwm.h"
#include "resetPin/resetPin.h"
#include "stepgen/stepgen.h"
#include "thermistor/thermistor.h"

// Carvera Air CA1 configuration

vector<PRUThread*> configure_threads(Comms* comms) {
  // the beeper is obnoxious, shut it up first thing
  Pin* beeper_pin = (new Pin(1, 14))->pull_down();
  beeper_pin->set(false);
  // the addressable LED strip needs to be bit-banged at 800kHz and there's currently no support for that,
  // so for now let's just set it low and forget about it
  (new Pin(1, 15))->pull_down()->set(false);

  printf("\ncreating threads for CA1\n");

  // Args: timer number, frequency, IRQ priority
  auto base_thread = new PRUThread(0, BASE_THREAD_FREQUENCY, 2);
  printf("created base thread\n");

  const std::vector<Module*> base_modules = {
      // Axis step generators
      new Stepgen(0, new Pin(1, 28), (new Pin(1, 29))->invert(), base_thread->frequency, comms->rx_data,
                  comms->tx_data),  // X
      new Stepgen(1, new Pin(1, 26), new Pin(1, 27), base_thread->frequency, comms->rx_data,
                  comms->tx_data),  // Y
      new Stepgen(2, new Pin(1, 24), (new Pin(1, 25))->invert(), base_thread->frequency, comms->rx_data,
                  comms->tx_data),  // Z
      new Stepgen(3, new Pin(1, 21), (new Pin(1, 23))->invert(), base_thread->frequency, comms->rx_data,
                  comms->tx_data)  // A
  };
  printf("created base modules\n");

  for (const auto m : base_modules) base_thread->register_module(m);
  printf("registered base modules\n");

  auto servo_thread = new PRUThread(1, SERVO_THREAD_FREQUENCY, 3);
  printf("created servo thread\n");

  const std::vector<Module*> servo_modules = {
      // "special" pins
      new ResetPin(comms->pru_reset, new Pin(2, 10)),  // "hard" reset
      new eStop(comms->tx_data, new Pin(0, 20)),       // e-stop button

      // PWMs
      // on LPC1768, the period is shared among all PWMs,
      // so don't try setting it to different values - the last one wins.
      // many bothans died to bring us this information.

      new PWM(0, new Pin(2, 5), 10000, comms->rx_data),   // spindle
      new PWM(1, new Pin(2, 1), 10000, comms->rx_data),   // spindle fan
      new PWM(2, new Pin(2, 3), 10000, comms->rx_data),   // power supply fan
      new PWM(3, new Pin(0, 21), 10000, comms->rx_data),  // EXT port output

      // feedback
      new PulseCounter(0, new Pin(2, 7), comms->tx_data),  // spindle encoder feedback

      // thermistors
      new Thermistor(1, new Pin(1, 31), 3950, 100000, 25, servo_thread->frequency, comms->tx_data),  // spindle
      new Thermistor(2, new Pin(0, 26), 3950, 100000, 25, servo_thread->frequency, comms->tx_data),  // power supply

      // output pins
      new OutputPin(0, new Pin(2, 0), comms->rx_data),               // work light
      new OutputPin(1, new Pin(0, 11), comms->rx_data),              // probe power
      new OutputPin(2, beeper_pin, comms->rx_data),                  // beeper
      new OutputPin(3, new Pin(0, 22), comms->rx_data),              // 12V power
      new OutputPin(4, new Pin(0, 10), comms->rx_data),              // 24V power
      new OutputPin(5, (new Pin(1, 30))->invert(), comms->rx_data),  // rotary A axis enable

      // input pins
      new InputPin(0, (new Pin(0, 19))->pull_up(), comms->tx_data),            // spindle stall alarm
      new InputPin(1, (new Pin(0, 1))->pull_up(), comms->tx_data),             // X stall alarm
      new InputPin(2, (new Pin(0, 0))->pull_up(), comms->tx_data),             // Y stall alarm
      new InputPin(3, (new Pin(3, 25))->pull_up(), comms->tx_data),            // Z stall alarm
      new InputPin(4, (new Pin(1, 8))->pull_up()->invert(), comms->tx_data),   // lid sensor
      new InputPin(5, (new Pin(2, 13))->pull_up()->invert(), comms->tx_data),  // main button
      new InputPin(6, (new Pin(2, 2))->pull_down(), comms->tx_data),           // EXT port input
      new InputPin(7, (new Pin(0, 24))->pull_up(), comms->tx_data),            // X home switch/endstop
      new InputPin(8, (new Pin(0, 25))->pull_up(), comms->tx_data),            // Y home switch/endstop
      new InputPin(9, (new Pin(1, 1))->pull_up(), comms->tx_data),             // Z home switch/endstop
      new InputPin(10, (new Pin(1, 9))->pull_up(), comms->tx_data),            // A home switch
      new InputPin(11, (new Pin(2, 6))->pull_down(), comms->tx_data),          // probe signal
      new InputPin(12, (new Pin(0, 5))->pull_up(), comms->tx_data)             // tool length sensor
  };

  printf("created servo modules\n");

  for (const auto m : servo_modules) servo_thread->register_module(m);
  printf("registered servo modules\n");

  return {base_thread, servo_thread};
}
