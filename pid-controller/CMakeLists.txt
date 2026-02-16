cmake_minimum_required(VERSION 3.10)

# PID Controller Library
add_library(pid-lib STATIC
    pid.cpp
    pid.hpp
)

target_include_directories(pid-lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# PID Test executable
add_executable(pid-test pid_test.cpp)
target_link_libraries(pid-test pid-lib)

# Set C++ standard
set_property(TARGET pid-lib PROPERTY CXX_STANDARD 11)
set_property(TARGET pid-test PROPERTY CXX_STANDARD 11)
