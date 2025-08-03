add_library(epho-module STATIC
        module.cpp
        adc/adc.cpp
        eStop/eStop.cpp
        gpio/digitalIns.cpp
        gpio/digitalOuts.cpp
        pulseCounter/pulseCounter.cpp
        pwm/pwm.cpp
        resetPin/resetPin.cpp
        stepgen/stepgen.cpp
)

target_include_directories(epho-module PUBLIC .)

target_link_libraries(epho-module PUBLIC epho-thread epho-driver-comms epho-driver-pin)