cmake_minimum_required(VERSION 3.10)
project(kalman-test)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# Enable debug symbols for Debug builds
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0")

# Include Eigen header-only library
include_directories(${CMAKE_SOURCE_DIR}/eigen)

# Add GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
add_subdirectory(glfw)

# Add ImGui library
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/imgui)
add_library(imgui
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(imgui glfw)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Add kalman-cpp library
add_library(kalman-lib
    kalman-cpp/kalman.cpp
    kalman-cpp/kalman.hpp
)

# Include directory for kalman-cpp headers
target_include_directories(kalman-lib PUBLIC
    ${CMAKE_SOURCE_DIR}/kalman-cpp
    ${CMAKE_SOURCE_DIR}/eigen
)

# Add PID Controller subdirectory
add_subdirectory(pid-controller)

# Add main executable
file(GLOB MAIN_SOURCES
    "src/*.cpp"
    "src/**/*.cpp"
    "pid-controller/pid.cpp"
)
add_executable(main ${MAIN_SOURCES})

# Link kalman library to main executable
target_link_libraries(main kalman-lib imgui glfw OpenGL::GL ${CMAKE_DL_LIBS})

# Include directories for main
target_include_directories(main PRIVATE
    ${CMAKE_SOURCE_DIR}/kalman-cpp
    ${CMAKE_SOURCE_DIR}/eigen
    ${CMAKE_SOURCE_DIR}/glfw/deps
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)
