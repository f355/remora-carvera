add_library(epho-module STATIC
        module.cpp
        digitalPin/digitalPin.cpp
        eStop/eStop.cpp
        pulseCounter/pulseCounter.cpp
        pwm/pwm.cpp
        resetPin/resetPin.cpp
        stepgen/stepgen.cpp
        thermistor/thermistor.cpp
)

target_include_directories(epho-module PUBLIC .)

target_link_libraries(epho-module PUBLIC epho-thread epho-driver-comms epho-driver-pin)