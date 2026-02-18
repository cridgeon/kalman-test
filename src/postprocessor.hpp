#pragma once

#include "shader.hpp"
#include "framebuffer.hpp"
#include <vector>
#include <memory>
#include <string>

class PostProcessor {
public:
    PostProcessor();
    ~PostProcessor();

    // Disable copy constructor and assignment operator
    PostProcessor(const PostProcessor&) = delete;
    PostProcessor& operator=(const PostProcessor&) = delete;

    // Initialize the post processor with screen dimensions
    bool initialize(int screenWidth, int screenHeight);

    // Add a post-processing shader effect
    bool addEffect(const std::string& name, const std::string& fragmentSource);
    
    // Load effect from file
    bool loadEffect(const std::string& name, const std::string& fragmentPath);

    // Remove an effect
    void removeEffect(const std::string& name);

    // Enable/disable an effect
    void setEffectEnabled(const std::string& name, bool enabled);
    
    // Check if an effect is enabled
    bool isEffectEnabled(const std::string& name) const;

    // Get list of available effects
    std::vector<std::string> getEffectNames() const;

    // Start rendering to the framebuffer (call before scene rendering)
    void beginRender();

    // End rendering and apply post-processing effects (call after scene rendering)
    void endRender();

    // Handle window resize
    void resize(int newWidth, int newHeight);

    // Set uniform values for effects
    void setEffectFloat(const std::string& effectName, const std::string& uniformName, float value);
    void setEffectVec2(const std::string& effectName, const std::string& uniformName, float x, float y);
    void setEffectVec3(const std::string& effectName, const std::string& uniformName, float x, float y, float z);
    void setEffectVec4(const std::string& effectName, const std::string& uniformName, float x, float y, float z, float w);

    // Get current screen dimensions
    int getWidth() const { return screenWidth; }
    int getHeight() const { return screenHeight; }

private:
    struct Effect {
        std::string name;
        Shader shader;
        bool enabled;
        
        Effect(const std::string& n) : name(n), enabled(false) {}
    };

    std::vector<std::unique_ptr<Effect>> effects;
    std::unique_ptr<Framebuffer> framebuffer;
    unsigned int quadVAO, quadVBO;
    int screenWidth, screenHeight;

    // Initialize the full-screen quad
    void setupQuad();

    // Render a full-screen quad
    void renderQuad();

    // Get default vertex shader source
    std::string getDefaultVertexShader() const;

    // Find effect by name
    Effect* findEffect(const std::string& name);
    const Effect* findEffect(const std::string& name) const;
};