cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

# cysecuretools fails to install on macOS and we don't need it anyway,
# so pretend we already have it
set(HAVE_PYTHON_CYSECURETOOLS TRUE)
set(MBED_APP_JSON_PATH mbed_app.json5)

include(mbed-os/tools/cmake/mbed_toolchain_setup.cmake)

project(firmware
        # VERSION 1.0.0
        LANGUAGES C CXX ASM)

include(mbed_project_setup)

add_subdirectory(mbed-os)

add_subdirectory(drivers)
add_subdirectory(thread)
add_subdirectory(modules)
add_subdirectory(machine_config)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} mbed-baremetal epho-machine-config epho-thread)

mbed_set_post_build(${PROJECT_NAME})
