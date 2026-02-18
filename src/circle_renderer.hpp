#pragma once

#include <vector>
#include <glad/gl.h>
#include "shader.hpp"

struct CircleData {
    float x, y;          // Position (normalized coordinates -1 to 1)
    float radius;        // Radius in pixels
    float r, g, b, a;    // Color components
};

class CircleRenderer {
public:
    CircleRenderer();
    ~CircleRenderer();

    // Initialize the circle renderer
    bool initialize(int window_width, int window_height);
    
    // Update window dimensions
    void resize(int window_width, int window_height);
    
    // Clear all circles
    void clearCircles();
    
    // Add a circle to be rendered
    void addCircle(float x, float y, float radius, float r, float g, float b, float a = 1.0f);
    
    // Render all circles
    void render();
    
    // Cleanup
    void cleanup();

private:
    // Setup fullscreen quad geometry
    void setupQuad();
    
    // Create and compile shaders
    bool createShaders();

private:
    std::vector<CircleData> circles_;
    
    // OpenGL objects
    GLuint vao_;
    GLuint vbo_;
    GLuint shader_program_;
    
    // Uniform locations
    GLint u_resolution_;
    GLint u_circle_count_;
    GLint u_circles_;
    
    // Window dimensions
    int window_width_;
    int window_height_;
    
    bool initialized_;
    
    // Maximum number of circles we can render
    static const int MAX_CIRCLES = 32;
};