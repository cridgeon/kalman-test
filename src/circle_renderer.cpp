#include "circle_renderer.hpp"
#include <iostream>
#include <cstring>

// Vertex shader for fullscreen quad
static const char* vertex_shader_source = R"(
#version 130
in vec2 position;
out vec2 fragCoord;

void main() {
    gl_Position = vec4(position, 0.0, 1.0);
    fragCoord = position * 0.5 + 0.5; // Convert from [-1,1] to [0,1]
}
)";

// Fragment shader for circle rendering
static const char* fragment_shader_source = R"(
#version 130
in vec2 fragCoord;
out vec4 fragColor;

uniform vec2 u_resolution;
uniform int u_circle_count;
uniform float u_circles[192]; // 6 floats per circle (x, y, radius, r, g, b) * 32 max circles = 192 floats

void main() {
    vec2 pixelCoord = fragCoord * u_resolution;
    vec4 finalColor = vec4(0.0, 0.0, 0.0, 0.0); // Transparent background
    
    for (int i = 0; i < u_circle_count && i < 32; i++) {
        int base = i * 6;
        
        // Get circle data (positions in normalized coordinates [-1,1])
        float circle_x = u_circles[base + 0];
        float circle_y = u_circles[base + 1]; 
        float radius = u_circles[base + 2];
        
        // Convert normalized coordinates to pixel coordinates
        vec2 center = vec2(
            (circle_x + 1.0) * 0.5 * u_resolution.x,
            (1.0 - (circle_y + 1.0) * 0.5) * u_resolution.y  // Flip Y coordinate
        );
        
        // Calculate distance from pixel to circle center
        float dist = length(pixelCoord - center);
        
        // Create filled circle with anti-aliasing
        float edge_softness = 1.0; // Softness of circle edge for anti-aliasing
        float alpha = 1.0 - smoothstep(radius - edge_softness, radius + edge_softness, dist);
        
        if (alpha > 0.0) {
            vec4 circleColor = vec4(u_circles[base + 3], u_circles[base + 4], u_circles[base + 5], alpha);
            
            // Alpha blending
            finalColor = mix(finalColor, circleColor, circleColor.a);
        }
    }
    
    fragColor = finalColor;
}
)";

CircleRenderer::CircleRenderer()
    : vao_(0), vbo_(0), shader_program_(0), window_width_(0), window_height_(0), 
      initialized_(false) {
}

CircleRenderer::~CircleRenderer() {
    cleanup();
}

bool CircleRenderer::initialize(int window_width, int window_height) {
    if (initialized_) {
        return true;
    }
    
    window_width_ = window_width;
    window_height_ = window_height;
    
    if (!createShaders()) {
        std::cerr << "Failed to create circle shaders" << std::endl;
        return false;
    }
    
    setupQuad();
    
    // Get uniform locations
    u_resolution_ = glGetUniformLocation(shader_program_, "u_resolution");
    u_circle_count_ = glGetUniformLocation(shader_program_, "u_circle_count");
    u_circles_ = glGetUniformLocation(shader_program_, "u_circles");
    
    initialized_ = true;
    return true;
}

void CircleRenderer::setupQuad() {
    // Fullscreen quad vertices (2 triangles)
    float vertices[] = {
        // First triangle
        -1.0f, -1.0f,
         1.0f, -1.0f,
         1.0f,  1.0f,
        // Second triangle  
        -1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f,  1.0f
    };
    
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    
    // Position attribute
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    glBindVertexArray(0);
}

bool CircleRenderer::createShaders() {
    // Create and compile vertex shader
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_source, NULL);
    glCompileShader(vertex_shader);
    
    // Check vertex shader compilation
    GLint success;
    GLchar info_log[512];
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertex_shader, 512, NULL, info_log);
        std::cerr << "Vertex shader compilation failed: " << info_log << std::endl;
        return false;
    }
    
    // Create and compile fragment shader
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_source, NULL);
    glCompileShader(fragment_shader);
    
    // Check fragment shader compilation
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragment_shader, 512, NULL, info_log);
        std::cerr << "Fragment shader compilation failed: " << info_log << std::endl;
        return false;
    }
    
    // Create shader program
    shader_program_ = glCreateProgram();
    glAttachShader(shader_program_, vertex_shader);
    glAttachShader(shader_program_, fragment_shader);
    
    // Bind attribute location
    glBindAttribLocation(shader_program_, 0, "position");
    
    glLinkProgram(shader_program_);
    
    // Check program linking
    glGetProgramiv(shader_program_, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shader_program_, 512, NULL, info_log);
        std::cerr << "Shader program linking failed: " << info_log << std::endl;
        return false;
    }
    
    // Clean up individual shaders
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    
    return true;
}

void CircleRenderer::resize(int window_width, int window_height) {
    window_width_ = window_width;
    window_height_ = window_height;
}

void CircleRenderer::clearCircles() {
    circles_.clear();
}

void CircleRenderer::addCircle(float x, float y, float radius, float r, float g, float b, float a) {
    if (circles_.size() >= MAX_CIRCLES) {
        std::cerr << "Warning: Maximum number of circles exceeded" << std::endl;
        return;
    }
    
    CircleData circle;
    circle.x = x;
    circle.y = y;
    circle.radius = radius;
    circle.r = r;
    circle.g = g;
    circle.b = b;
    circle.a = a;
    
    circles_.push_back(circle);
}

void CircleRenderer::render() {
    if (!initialized_ || circles_.empty()) {
        return;
    }
    
    // Enable blending for transparent circles
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glUseProgram(shader_program_);
    
    // Set uniforms
    glUniform2f(u_resolution_, (float)window_width_, (float)window_height_);
    glUniform1i(u_circle_count_, (int)circles_.size());
    
    // Pack circle data into float array
    float circle_data[MAX_CIRCLES * 6]; // 6 floats per circle (x, y, radius, r, g, b)
    for (size_t i = 0; i < circles_.size() && i < MAX_CIRCLES; i++) {
        int base = i * 6;
        circle_data[base + 0] = circles_[i].x;
        circle_data[base + 1] = circles_[i].y;
        circle_data[base + 2] = circles_[i].radius;
        circle_data[base + 3] = circles_[i].r;
        circle_data[base + 4] = circles_[i].g;
        circle_data[base + 5] = circles_[i].b;
    }
    
    glUniform1fv(u_circles_, circles_.size() * 6, circle_data);
    
    // Render fullscreen quad
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    
    glDisable(GL_BLEND);
}

void CircleRenderer::cleanup() {
    if (vao_) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }
    if (vbo_) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }
    if (shader_program_) {
        glDeleteProgram(shader_program_);
        shader_program_ = 0;
    }
    initialized_ = false;
}