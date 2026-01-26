# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Speech_Recognition_AudioCapture C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
# Set BUILD_TEST=1 to build test instead of main project
# TEST_TYPE options: scan (GPIO scan), gpio6 (pin 6 only), gpio (both pins), usb (USB test)
if(BUILD_TEST)
    if(TEST_TYPE STREQUAL "scan")
        add_executable(Speech_Recognition_AudioCapture test_gpio_scan.c)
    elseif(TEST_TYPE STREQUAL "gpio6")
        add_executable(Speech_Recognition_AudioCapture test_gpio6_only.c)
    elseif(TEST_TYPE STREQUAL "gpio")
        add_executable(Speech_Recognition_AudioCapture test_gpio_only.c)
    else()
        add_executable(Speech_Recognition_AudioCapture test_usb_only.c)
    endif()
    pico_enable_stdio_uart(Speech_Recognition_AudioCapture 0)
    pico_enable_stdio_usb(Speech_Recognition_AudioCapture 1)
    target_link_libraries(Speech_Recognition_AudioCapture
            pico_stdlib
            pico_stdio_usb
            hardware_gpio
            )
else()
    add_executable(Speech_Recognition_AudioCapture Speech_Recognition_AudioCapture.c )
    pico_enable_stdio_uart(Speech_Recognition_AudioCapture 0)
    pico_enable_stdio_usb(Speech_Recognition_AudioCapture 1)
    target_link_libraries(Speech_Recognition_AudioCapture 
            pico_stdlib
            pico_stdio_usb
            pico_multicore
            hardware_i2c
            hardware_dma
            hardware_adc
            hardware_uart
            m
            )
endif()

pico_set_program_name(Speech_Recognition_AudioCapture "Speech_Recognition_AudioCapture")
pico_set_program_version(Speech_Recognition_AudioCapture "0.1")

# Add the standard include files to the build
target_include_directories(Speech_Recognition_AudioCapture PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(Speech_Recognition_AudioCapture)

