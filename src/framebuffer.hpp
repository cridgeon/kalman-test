#pragma once

#include "gl_extensions.hpp"

class Framebuffer {
public:
    Framebuffer();
    ~Framebuffer();

    // Disable copy constructor and assignment operator
    Framebuffer(const Framebuffer&) = delete;
    Framebuffer& operator=(const Framebuffer&) = delete;

    // Enable move constructor and assignment operator
    Framebuffer(Framebuffer&& other) noexcept;
    Framebuffer& operator=(Framebuffer&& other) noexcept;

    // Create framebuffer with specified dimensions
    bool create(int width, int height);

    // Bind framebuffer for rendering
    void bind() const;

    // Unbind framebuffer (restore default framebuffer)
    void unbind() const;

    // Get the color texture ID
    unsigned int getColorTexture() const { return colorTexture; }

    // Get framebuffer dimensions
    int getWidth() const { return width; }
    int getHeight() const { return height; }

    // Check if framebuffer is valid
    bool isValid() const { return framebufferID != 0; }

    // Resize the framebuffer
    bool resize(int newWidth, int newHeight);

private:
    unsigned int framebufferID;
    unsigned int colorTexture;
    unsigned int depthRenderbuffer;
    int width, height;

    void cleanup();
};