add_library(epho-machine-config STATIC machine_config.cpp)

target_include_directories(epho-machine-config PUBLIC .)

target_link_libraries(epho-machine-config PUBLIC epho-thread)