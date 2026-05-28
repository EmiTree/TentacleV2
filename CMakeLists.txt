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

project(TentacleV2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(TentacleV2
        Main/TentacleV2.cpp
        Modules/PIDController.cpp
        Modules/ServoActuator.cpp
        Modules/MotorConverter.cpp
        Modules/MPU6050Sensor.cpp
        Modules/MotorDriver.cpp
        )

pico_set_program_name(TentacleV2 "TentacleV2")
pico_set_program_version(TentacleV2 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(TentacleV2 0)
pico_enable_stdio_usb(TentacleV2 1)

# Add the standard library to the build
target_link_libraries(TentacleV2
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(TentacleV2 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/Modules
)

# Add any user requested libraries
target_link_libraries(TentacleV2 
        hardware_i2c
        hardware_pwm
        )

pico_add_extra_outputs(TentacleV2)

add_executable(MotorActuation Troubleshooting/MotorActuation.cpp)

pico_set_program_name(MotorActuation "MotorActuation")
pico_set_program_version(MotorActuation "0.1")

pico_enable_stdio_uart(MotorActuation 0)
pico_enable_stdio_usb(MotorActuation 1)

target_link_libraries(MotorActuation
        pico_stdlib
        hardware_pwm
        )

pico_add_extra_outputs(MotorActuation)

add_executable(ServoActuation Troubleshooting/ServoActuation.cpp)

pico_set_program_name(ServoActuation "ServoActuation")
pico_set_program_version(ServoActuation "0.1")

pico_enable_stdio_uart(ServoActuation 0)
pico_enable_stdio_usb(ServoActuation 1)

target_link_libraries(ServoActuation
        pico_stdlib
        hardware_pwm
        )

pico_add_extra_outputs(ServoActuation)


add_executable(OnlyAngle Troubleshooting/AngleMeasurement.cpp)

pico_enable_stdio_uart(OnlyAngle 0)
pico_enable_stdio_usb(OnlyAngle 1)

target_link_libraries(OnlyAngle
        pico_stdlib
        hardware_i2c
        )

pico_add_extra_outputs(OnlyAngle)

add_executable(MotorMpuDebug Troubleshooting/MotorMpuDebug.cpp)

pico_enable_stdio_uart(MotorMpuDebug 0)
pico_enable_stdio_usb(MotorMpuDebug 1)

target_link_libraries(MotorMpuDebug
        pico_stdlib
        hardware_pwm
        hardware_i2c
        )

pico_add_extra_outputs(MotorMpuDebug)

add_executable(I2CScanner Troubleshooting/I2CScanner.cpp)

pico_enable_stdio_uart(I2CScanner 0)
pico_enable_stdio_usb(I2CScanner 1)

target_link_libraries(I2CScanner
        pico_stdlib
        hardware_i2c
        )

pico_add_extra_outputs(I2CScanner)