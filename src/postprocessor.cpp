#include "postprocessor.hpp"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <GLFW/glfw3.h>

PostProcessor::PostProcessor() : quadVAO(0), quadVBO(0), screenWidth(0), screenHeight(0) {}

PostProcessor::~PostProcessor() {
    if (quadVBO != 0) {
        GLExtensionLoader::glDeleteBuffers(1, &quadVBO);
    }
    if (quadVAO != 0) {
        GLExtensionLoader::glDeleteVertexArrays(1, &quadVAO);
    }
}

bool PostProcessor::initialize(int width, int height) {
    screenWidth = width;
    screenHeight = height;

    // Create framebuffer
    framebuffer = std::make_unique<Framebuffer>();
    if (!framebuffer->create(width, height)) {
        std::cerr << "Failed to create framebuffer for post-processing" << std::endl;
        return false;
    }

    // Setup full-screen quad
    setupQuad();

    return true;
}

bool PostProcessor::addEffect(const std::string& name, const std::string& fragmentSource) {
    // Check if effect already exists
    if (findEffect(name) != nullptr) {
        std::cerr << "Effect '" << name << "' already exists" << std::endl;
        return false;
    }

    auto effect = std::make_unique<Effect>(name);
    
    std::string vertexSource = getDefaultVertexShader();
    
    if (!effect->shader.loadFromSource(vertexSource, fragmentSource)) {
        std::cerr << "Failed to compile shader for effect '" << name << "'" << std::endl;
        return false;
    }

    effects.push_back(std::move(effect));
    return true;
}

bool PostProcessor::loadEffect(const std::string& name, const std::string& fragmentPath) {
    std::ifstream file(fragmentPath);
    if (!file.is_open()) {
        std::cerr << "Failed to open shader file: " << fragmentPath << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    return addEffect(name, buffer.str());
}

void PostProcessor::removeEffect(const std::string& name) {
    auto it = std::remove_if(effects.begin(), effects.end(),
        [&name](const std::unique_ptr<Effect>& effect) {
            return effect->name == name;
        });
    effects.erase(it, effects.end());
}

void PostProcessor::setEffectEnabled(const std::string& name, bool enabled) {
    Effect* effect = findEffect(name);
    if (effect) {
        effect->enabled = enabled;
    }
}

bool PostProcessor::isEffectEnabled(const std::string& name) const {
    const Effect* effect = findEffect(name);
    return effect ? effect->enabled : false;
}

std::vector<std::string> PostProcessor::getEffectNames() const {
    std::vector<std::string> names;
    names.reserve(effects.size());
    for (const auto& effect : effects) {
        names.push_back(effect->name);
    }
    return names;
}

void PostProcessor::beginRender() {
    if (framebuffer) {
        framebuffer->bind();
        // Clear the framebuffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
}

void PostProcessor::endRender() {
    if (!framebuffer) return;

    // Restore default framebuffer
    framebuffer->unbind();
    glViewport(0, 0, screenWidth, screenHeight);

    // Disable depth testing for post-processing
    glDisable(GL_DEPTH_TEST);

    unsigned int currentTexture = framebuffer->getColorTexture();
    bool renderedSomething = false;

    // Apply each enabled effect in sequence
    for (const auto& effect : effects) {
        if (!effect->enabled) continue;

        effect->shader.use();
        
        // Bind the texture
        GLExtensionLoader::glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, currentTexture);
        effect->shader.setInt("screenTexture", 0);
        
        // Set common uniforms
        effect->shader.setVec2("resolution", static_cast<float>(screenWidth), static_cast<float>(screenHeight));
        effect->shader.setFloat("time", static_cast<float>(glfwGetTime()));

        // Render full-screen quad
        renderQuad();
        renderedSomething = true;
        break; // For now, only apply the first enabled effect
    }

    // If no effects were applied, render with passthrough
    if (!renderedSomething) {
        // Find and use the passthrough effect
        for (const auto& effect : effects) {
            if (effect->name == "passthrough") {
                effect->shader.use();
                GLExtensionLoader::glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, currentTexture);
                effect->shader.setInt("screenTexture", 0);
                renderQuad();
                break;
            }
        }
    }

    // Re-enable depth testing
    glEnable(GL_DEPTH_TEST);
}

void PostProcessor::resize(int newWidth, int newHeight) {
    screenWidth = newWidth;
    screenHeight = newHeight;
    
    if (framebuffer) {
        framebuffer->resize(newWidth, newHeight);
    }
}

void PostProcessor::setEffectFloat(const std::string& effectName, const std::string& uniformName, float value) {
    Effect* effect = findEffect(effectName);
    if (effect && effect->shader.isValid()) {
        effect->shader.use();
        effect->shader.setFloat(uniformName, value);
    }
}

void PostProcessor::setEffectVec2(const std::string& effectName, const std::string& uniformName, float x, float y) {
    Effect* effect = findEffect(effectName);
    if (effect && effect->shader.isValid()) {
        effect->shader.use();
        effect->shader.setVec2(uniformName, x, y);
    }
}

void PostProcessor::setEffectVec3(const std::string& effectName, const std::string& uniformName, float x, float y, float z) {
    Effect* effect = findEffect(effectName);
    if (effect && effect->shader.isValid()) {
        effect->shader.use();
        effect->shader.setVec3(uniformName, x, y, z);
    }
}

void PostProcessor::setEffectVec4(const std::string& effectName, const std::string& uniformName, float x, float y, float z, float w) {
    Effect* effect = findEffect(effectName);
    if (effect && effect->shader.isValid()) {
        effect->shader.use();
        effect->shader.setVec4(uniformName, x, y, z, w);
    }
}

void PostProcessor::setupQuad() {
    float quadVertices[] = {
        // positions   // texCoords
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,

        -1.0f,  1.0f,  0.0f, 1.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f
    };

    GLExtensionLoader::glGenVertexArrays(1, &quadVAO);
    GLExtensionLoader::glGenBuffers(1, &quadVBO);
    GLExtensionLoader::glBindVertexArray(quadVAO);
    GLExtensionLoader::glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    GLExtensionLoader::glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
    GLExtensionLoader::glEnableVertexAttribArray(0);
    GLExtensionLoader::glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    GLExtensionLoader::glEnableVertexAttribArray(1);
    GLExtensionLoader::glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    GLExtensionLoader::glBindVertexArray(0);
}

void PostProcessor::renderQuad() {
    GLExtensionLoader::glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    GLExtensionLoader::glBindVertexArray(0);
}

std::string PostProcessor::getDefaultVertexShader() const {
    return R"(
#version 130

in vec2 position;
in vec2 texCoord;

out vec2 TexCoords;

void main() {
    gl_Position = vec4(position.x, position.y, 0.0, 1.0);
    TexCoords = texCoord;
}
)";
}

PostProcessor::Effect* PostProcessor::findEffect(const std::string& name) {
    auto it = std::find_if(effects.begin(), effects.end(),
        [&name](const std::unique_ptr<Effect>& effect) {
            return effect->name == name;
        });
    return (it != effects.end()) ? it->get() : nullptr;
}

const PostProcessor::Effect* PostProcessor::findEffect(const std::string& name) const {
    auto it = std::find_if(effects.begin(), effects.end(),
        [&name](const std::unique_ptr<Effect>& effect) {
            return effect->name == name;
        });
    return (it != effects.end()) ? it->get() : nullptr;
}