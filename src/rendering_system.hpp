#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <functional>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/gl.h>
#include "circle_renderer.hpp"

class RenderingSystem {
public:
    RenderingSystem(int window_width, int window_height, 
                   const std::string& window_title = "Kalman Test - ImGui Demo");
    ~RenderingSystem();

    // Initialize the rendering system
    bool initialize();
    
    // Main rendering loop - returns true while window should remain open
    bool shouldContinue();
    
    // Begin frame - call before rendering
    void beginFrame();
    
    // End frame and swap buffers
    void endFrame();
    
    // Set background color
    void setClearColor(const unsigned char col[4]);
    
    // Get window dimensions
    int getWindowWidth() const { return window_width_; }
    int getWindowHeight() const { return window_height_; }
    
    // Cleanup
    void shutdown();

    GLFWwindow* getWindow() const { return window_; }
    const char* getGLSLVersion() const { return glsl_version_; }

    CircleRenderer& getCircleRenderer() { return circle_renderer_; }

private:
    // GLFW callback
    static void glfwErrorCallback(int error, const char* description);
    
    // Setup functions
    bool setupGLFW();
    
private:
    int window_width_;
    int window_height_;
    std::string window_title_;
    
    GLFWwindow* window_;
    CircleRenderer circle_renderer_;
    unsigned char clear_color_[4];
    const char* glsl_version_;
    
    bool initialized_;
};