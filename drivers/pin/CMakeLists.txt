add_library(epho-driver-pin STATIC pin.cpp)

target_include_directories(epho-driver-pin PUBLIC .)

target_link_libraries(epho-driver-pin PUBLIC mbed-baremetal)