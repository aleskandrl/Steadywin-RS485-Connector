cmake_minimum_required(VERSION 3.10)
project(steadywin_rs485_connector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Source files for the library
set(LIB_SOURCES
    src/steadywin_protocol.cpp
    src/steadywin_motor.cpp
)

# Add Windows serial port implementation only on Windows
if(WIN32)
    list(APPEND LIB_SOURCES src/windows_serial_port.cpp)
endif()

# Create the library
add_library(steadywin STATIC ${LIB_SOURCES})

# --- Main Executable ---
if(WIN32)
    add_executable(steadywin_rs485_connector src/main.cpp)
    target_link_libraries(steadywin_rs485_connector steadywin setupapi ws2_32)
endif()
