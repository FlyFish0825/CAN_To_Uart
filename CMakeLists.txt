cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME CAN_To_Uart)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/can_gateway_core.c
    Core/Src/can_gateway_priority.c
    Core/Src/firmware_flow.c
    Core/Src/imu_sensor.c
    Core/Src/imu_uart_debug.c
    Core/Src/system_heartbeat.c
    Core/Src/usb_can_gateway.c
    Core/Src/sensor_protocol.c
    Core/Src/sensor_service.c
    Core/Src/sensor_board.c
    Core/Src/imu_sensor.c
    Core/Src/ms5837.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Passive RX is safe for observation; TX/configuration remains separately guarded.
option(SENSOR_IMU_UART_ENABLED "Enable passive IMU USART1 receive on PA10" ON)
option(SENSOR_IMU_UART_TX_ENABLED "Enable IMU PA9 TX only after wiring is confirmed free" ON)  # 2026-10-06 接线确认空闲（WCH 拔除），经用户授权开启
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    SENSOR_IMU_UART_ENABLED=$<BOOL:${SENSOR_IMU_UART_ENABLED}>
    SENSOR_IMU_UART_TX_ENABLED=$<BOOL:${SENSOR_IMU_UART_TX_ENABLED}>)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
