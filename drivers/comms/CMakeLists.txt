add_subdirectory(MODDMA)

add_library(epho-driver-comms STATIC RemoraComms.cpp)

target_include_directories(epho-driver-comms PUBLIC .)

target_link_libraries(epho-driver-comms PUBLIC mbed-baremetal moddma)