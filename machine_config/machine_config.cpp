#include "machine_config.h"
#include "module.h"

#include "stepgen/stepgen.h"
#include "resetPin/resetPin.h"
#include "eStop/eStop.h"
#include "pwm/pwm.h"
#include "pulseCounter/pulseCounter.h"
#include "digitalPin/digitalPin.h"
#include "thermistor/thermistor.h"

#include "LPC17xx.h"

// Carvera Air CA1 configuration

vector<PRUThread*> configureThreads(RemoraComms* comms)
{
    printf("\ncreating threads for CA1\n");

    // Args: frequency, IRQ priority, timer number
    PRUThread* baseThread = new PRUThread(1, 60000, 2);
    printf("created base thread\n");
    PRUThread* servoThread = new PRUThread(2, 1000, 3);
    printf("created servo thread\n");

    std::vector<Module*> baseModules = {
        // Axis step generators, XYZA
        new Stepgen(0, new Pin(1, 28), (new Pin(1, 29))->invert(), baseThread->frequency, comms->ptrRxData, comms->ptrTxData),
        new Stepgen(1, new Pin(1, 26), new Pin(1, 27), baseThread->frequency, comms->ptrRxData, comms->ptrTxData),
        new Stepgen(2, new Pin(1, 24), (new Pin(1, 25))->invert(), baseThread->frequency, comms->ptrRxData, comms->ptrTxData),
        new Stepgen(3, new Pin(1, 21), (new Pin(1, 23))->invert(), baseThread->frequency, comms->ptrRxData, comms->ptrTxData)
    };
    printf("created base modules\n");

    for (auto m: baseModules) {
        baseThread->registerModule(m);
    }
    printf("registered base modules\n");

    // the beeper is obnoxious, shut it up ASAP
    Pin* beeperPin = (new Pin(1, 14))->pull_down();
    beeperPin->set(false);

    std::vector<Module*> servoModules = {
        new ResetPin(comms->pruReset, new Pin(2, 10)),
        new eStop(comms->ptrTxData->header, new Pin(0, 20)),

        // // PWMs
        // // many bothans died to bring us this information:
        // // on LPC1768, the period is shared among all PWMs,
        // // so don't try setting it to different values - the last one wins.

        // // spindle
        // new PWM(0, new Pin(2, 5), 10000, comms->ptrRxData),
        // //spindle fan
        // new PWM(1, new Pin(2, 1), 10000, comms->ptrRxData),
        // // power supply fan
        // new PWM(2, new Pin(2, 3), 10000, comms->ptrRxData),

        // // spindle encoder feedback
        // new PulseCounter(0, new Pin(2, 7), comms->ptrTxData),

        // // thermistors
        // // spindle
        // new Thermistor(1, new Pin(1, 31), 3950, 100000, 25, servoThread->frequency, comms->ptrTxData),
        // // power supply
        // new Thermistor(1, new Pin(0, 26), 3950, 100000, 25, servoThread->frequency, comms->ptrTxData),

        // output pins
        // light
        new OutputPin(0, new Pin(2, 0), comms->ptrRxData),
        // Probe power
        new OutputPin(1, new Pin(0, 11), comms->ptrRxData),
        // beeper
        new OutputPin(2, beeperPin, comms->ptrRxData),
        // EXT output
        new OutputPin(3, new Pin(0, 21), comms->ptrRxData),
        // 12V power
        new OutputPin(4, new Pin(0, 22), comms->ptrRxData),
        // 24V power
        new OutputPin(5, new Pin(0, 10), comms->ptrRxData),
        // rotary A axis enable pin
        new OutputPin(6, (new Pin(1, 30))->invert(), comms->ptrRxData),

        // input pins
        // spindle alarm
        new InputPin(0, (new Pin(0, 19))->pull_up(), comms->ptrTxData),
        // axis stall alarms, XYZ
        new InputPin(1, (new Pin(0, 1))->pull_up(), comms->ptrTxData),
        new InputPin(2, (new Pin(0, 0))->pull_up(), comms->ptrTxData),
        new InputPin(3, (new Pin(3, 25))->pull_up(), comms->ptrTxData),
        // lid sensor
        new InputPin(4, (new Pin(1, 8))->pull_up()->invert(), comms->ptrTxData),
        // main button
        new InputPin(5, (new Pin(2, 13))->pull_up()->invert(), comms->ptrTxData),
        // EXT input
        new InputPin(6, (new Pin(2, 2))->pull_down(), comms->ptrTxData),
        // axis endstops, XYZA
        new InputPin(7, (new Pin(0, 24))->pull_up(), comms->ptrTxData),
        new InputPin(8, (new Pin(0, 25))->pull_up(), comms->ptrTxData),
        new InputPin(9, (new Pin(1, 1))->pull_up(), comms->ptrTxData),
        new InputPin(10, (new Pin(1, 9))->pull_up(), comms->ptrTxData),
        // probe
        new InputPin(11, (new Pin(2, 6))->pull_down(), comms->ptrTxData),
        // tool length sensor
        new InputPin(12, (new Pin(0, 5))->pull_up(), comms->ptrTxData)
    };

    printf("created servo modules\n");

    for (auto m: servoModules) {
        servoThread->registerModule(m);
    }

    printf("registered servo modules\n");

    return {baseThread, servoThread};
}
