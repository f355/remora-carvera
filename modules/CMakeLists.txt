add_library(epho-module STATIC
        module.cpp
        digital_pin/digital_pin.cpp
        e_stop/eStop.cpp
        pulse_counter/pulseCounter.cpp
        pwm/pwm.cpp
        reset_pin/resetPin.cpp
        stepgen/stepgen.cpp
        thermistor/thermistor.cpp
)

target_include_directories(epho-module PUBLIC .)

target_link_libraries(epho-module PUBLIC epho-thread epho-driver-comms epho-driver-pin)