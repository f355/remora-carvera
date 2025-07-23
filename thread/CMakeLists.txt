add_library(epho-thread STATIC lpcTimer.cpp MachineThread.cpp)

target_include_directories(epho-thread PUBLIC .)

target_link_libraries(epho-thread PUBLIC mbed-baremetal epho-module)
