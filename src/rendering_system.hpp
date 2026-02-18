#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <functional>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/gl.h>

class RenderingSystem {
public:
    // Singleton access
    static RenderingSystem& getInstance();
    
    // Delete copy constructor and assignment operator
    RenderingSystem(const RenderingSystem&) = delete;
    RenderingSystem& operator=(const RenderingSystem&) = delete;
    
    ~RenderingSystem();

    // Initialize the rendering system with window parameters
    bool initialize(int window_width, int window_height, 
                   const std::string& window_title = "Kalman Test - ImGui Demo");
    
    // Main rendering loop - returns true while window should remain open
    bool shouldContinue();
    
    // Begin frame - call before rendering
    void beginFrame();
    
    // End frame and swap buffers
    void endFrame();
    
    // Set background color
    void setClearColor(const float col[4]);
    
    // Get window dimensions
    int getWindowWidth() const { return window_width_; }
    int getWindowHeight() const { return window_height_; }
    
    // Cleanup
    void shutdown();

    GLFWwindow* getWindow() const { return window_; }
    const char* getGLSLVersion() const { return glsl_version_; }

private:
    // Private constructor for singleton
    RenderingSystem();
    
    // GLFW callback
    static void glfwErrorCallback(int error, const char* description);
    
    // Setup functions
    bool setupGLFW();
    
private:
    int window_width_;
    int window_height_;
    std::string window_title_;
    
    GLFWwindow* window_;
    float clear_color_[4];
    const char* glsl_version_;
    
    bool initialized_;
};