# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(ENABLE_PSRAM "0" CACHE STRING "Enables PSRAM support (0/1)")
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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(camera C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(camera camera.c ov5640.c imagecapture/StateMachine.c imagecapture/ParallelImageCapture.c )

if(ENABLE_PSRAM)
    add_compile_definitions(ENABLE_PSRAM=1)
    message(STATUS "PSRAM enabled: Using custom linker script (memmap_custom.ld)")
    pico_set_linker_script(DualShot ${CMAKE_SOURCE_DIR}/memmap_custom.ld) # memory map linker script
endif()

pico_set_program_name(camera "camera")
pico_set_program_version(camera "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(camera 1)
pico_enable_stdio_usb(camera 0)

# Add the standard library to the build
target_link_libraries(camera
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(camera PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(camera 
        hardware_i2c
        hardware_gpio
        hardware_pwm
        hardware_pio
        hardware_dma
        hardware_clocks

        )

pico_add_extra_outputs(camera)

