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
set(CMAKE_PROJECT_NAME fly)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

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
    # Add user sources here
    Core/Src/task.c
    Core/Src/pid.c
    Hardware/Src/mpu6050.c
    Libs/anotc/Src/anotc.c
    Libs/eMPL/Src/inv_mpu_dmp_motion_driver.c
    Libs/eMPL/Src/inv_mpu.c
    Libs/kalman/Src/kalman.c
    Libs/RTT/Src/SEGGER_RTT.c
    Libs/RTT/Src/SEGGER_RTT_printf.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Hardware/Inc
    Libs/anotc/Inc
    Libs/eMPL/Inc
    Libs/kalman/Inc
    Libs/RTT/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    EMPL_TARGET_STM32XX
    MPU6050
    # USE_DMP
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Script to convert ELF to HEX
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND arm-none-eabi-objcopy -O ihex ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Converting ELF to HEX"
)

add_custom_target(ELF2HEX ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)

set(JLINK_PATH "C:/Program Files (x86)/SEGGER/JLink_V612c")
set(JLINK_TASK_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.jlink/auto.task")

add_custom_command(
    TARGET ELF2HEX POST_BUILD
    COMMAND "${JLINK_PATH}/JLink.exe" "${JLINK_TASK_FILE}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Running JLink with task file ${JLINK_TASK_FILE}"
)
