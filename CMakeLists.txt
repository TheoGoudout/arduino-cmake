#==============================================================================#
# Author: QueezyTheGreat                                                       #
# Date:   26.04.2011                                                           #
#                                                                              #
# Description: Arduino CMake example                                           #
#                                                                              #
#==============================================================================#
cmake_minimum_required(VERSION 2.8)

# Load Arduino Toolchain
set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake)

# Arduino borad and port options
set(ARDUINO_DEFAULT_BOARD diecimila    CACHE STRING "Default Board ID, 'diecimila' when not specified")
set(ARDUINO_DEFAULT_PORT  /dev/ttyUSB0 CACHE STRING "Default Port, '/dev/ttyUSB0' when not specified")

#==============================================================================#
# Add Arduino project                                                          #
#==============================================================================#
project(ArduinoDrone C CXX)

# Print list of supported boards and programmers
print_board_list()
print_programmer_list()

# Add include directories
# TODO : Add these directories automatically
include_directories(
    include
    ${ARDUINO_SDK_PATH}/libraries/Wire
    ${ARDUINO_SDK_PATH}/libraries/I2Cdev
    ${ARDUINO_SDK_PATH}/libraries/MPU6050
)

#==============================================================================#
# Add Arduino firmware generation informations                                 #
#==============================================================================#
generate_arduino_firmware(drone
    SRCS
        src/main.cpp
        src/imu.cpp
    HDRS
        include/config.h
        include/singleton.h
        include/imu.h
    LIBS
        nano328_Wire
        nano328_I2Cdev
        nano328_MPU6050
    BOARD ${ARDUINO_DEFAULT_BOARD})
