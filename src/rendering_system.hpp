#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <functional>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "gl_extensions.hpp"
#include "postprocessor.hpp"
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
    
    // Get ImGui draw list for drawing primitives
    ImDrawList* getDrawList();
    
    // Create canvas window for drawing
    void beginCanvas();
    void endCanvas();
    
    // Get ImGui IO for input handling
    ImGuiIO& getIO();
    
    // Set background color
    void setClearColor(const ImVec4& color);
    
    // Get window dimensions
    int getWindowWidth() const { return window_width_; }
    int getWindowHeight() const { return window_height_; }
    
    // Post-processing effects
    PostProcessor& getPostProcessor() { return post_processor_; }
    
    // Circle rendering
    CircleRenderer& getCircleRenderer() { return circle_renderer_; }
    
    // Cleanup
    void shutdown();

private:
    // GLFW callback
    static void glfwErrorCallback(int error, const char* description);
    
    // Setup functions
    bool setupGLFW();
    bool setupImGui();
    bool setupPostProcessing();
    void setupShaderEffects();
    
private:
    int window_width_;
    int window_height_;
    std::string window_title_;
    
    GLFWwindow* window_;
    PostProcessor post_processor_;
    CircleRenderer circle_renderer_;
    ImVec4 clear_color_;
    const char* glsl_version_;
    
    bool initialized_;
};