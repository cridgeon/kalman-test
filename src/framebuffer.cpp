#include "framebuffer.hpp"
#include <iostream>

Framebuffer::Framebuffer() : framebufferID(0), colorTexture(0), depthRenderbuffer(0), width(0), height(0) {}

Framebuffer::~Framebuffer() {
    cleanup();
}

Framebuffer::Framebuffer(Framebuffer&& other) noexcept 
    : framebufferID(other.framebufferID), 
      colorTexture(other.colorTexture),
      depthRenderbuffer(other.depthRenderbuffer),
      width(other.width),
      height(other.height) {
    other.framebufferID = 0;
    other.colorTexture = 0;
    other.depthRenderbuffer = 0;
    other.width = 0;
    other.height = 0;
}

Framebuffer& Framebuffer::operator=(Framebuffer&& other) noexcept {
    if (this != &other) {
        cleanup();
        
        framebufferID = other.framebufferID;
        colorTexture = other.colorTexture;
        depthRenderbuffer = other.depthRenderbuffer;
        width = other.width;
        height = other.height;
        
        other.framebufferID = 0;
        other.colorTexture = 0;
        other.depthRenderbuffer = 0;
        other.width = 0;
        other.height = 0;
    }
    return *this;
}

bool Framebuffer::create(int w, int h) {
    width = w;
    height = h;

    // Generate framebuffer
    glGenFramebuffers(1, &framebufferID);
    glBindFramebuffer(GL_FRAMEBUFFER, framebufferID);

    // Create color texture
    glGenTextures(1, &colorTexture);
    glBindTexture(GL_TEXTURE_2D, colorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTexture, 0);

    // Create depth renderbuffer
    glGenRenderbuffers(1, &depthRenderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, depthRenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRenderbuffer);

    // Check if framebuffer is complete
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "ERROR::FRAMEBUFFER:: Framebuffer not complete!" << std::endl;
        cleanup();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return true;
}

void Framebuffer::bind() const {
    if (framebufferID != 0) {
        glBindFramebuffer(GL_FRAMEBUFFER, framebufferID);
        glViewport(0, 0, width, height);
    }
}

void Framebuffer::unbind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

bool Framebuffer::resize(int newWidth, int newHeight) {
    if (newWidth <= 0 || newHeight <= 0) return false;
    
    cleanup();
    return create(newWidth, newHeight);
}

void Framebuffer::cleanup() {
    if (depthRenderbuffer != 0) {
        glDeleteRenderbuffers(1, &depthRenderbuffer);
        depthRenderbuffer = 0;
    }
    
    if (colorTexture != 0) {
        glDeleteTextures(1, &colorTexture);
        colorTexture = 0;
    }
    
    if (framebufferID != 0) {
        glDeleteFramebuffers(1, &framebufferID);
        framebufferID = 0;
    }
}