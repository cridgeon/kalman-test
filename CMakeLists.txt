cmake_minimum_required(VERSION 3.10)
project(kalman-test)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
add_subdirectory(avcpp)

# Add main executable
add_executable(main
    src/main.cpp
    src/video_encoder.cpp
    pid-controller/pid.cpp
)

# Link kalman library to main executable
target_link_libraries(main 
    kalman-lib 
    imgui 
    glfw 
    OpenGL::GL 
    avcpp::avcpp
    ${CMAKE_DL_LIBS}
)

# Include directories for main
target_include_directories(main PRIVATE
    ${CMAKE_SOURCE_DIR}/kalman-cpp
    ${CMAKE_SOURCE_DIR}/eigen
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)
