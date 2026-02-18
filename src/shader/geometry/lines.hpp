#ifndef CRIDGEON_SHADER_LINES_HPP
#define CRIDGEON_SHADER_LINES_HPP

#include "rendering_system.hpp"
#include "../shader.hpp"
#include "../utility.hpp"
#include <vector>

namespace Render {

    static Shader linesShader;
    static bool linesVAOInitialized = false;
    static unsigned int linesVAO = UINT_MAX;
    static unsigned int linesVBO = UINT_MAX;

    inline void lines(const std::vector<float>& vertices, float r, float g, float b, float a) {
        if (vertices.size() < 4) return; // Need at least 2 vertices (4 floats) for one line
        if (vertices.size() > 512) return; // Max 128 lines * 2 vertices * 2 coords = 512 floats

        // Load shader if not already loaded
        if (!linesShader.isValid()) {
            linesShader.loadFromFile("resources/shaders/geometry/default.vert", "resources/shaders/geometry/color.frag");
            if (!linesShader.isValid()) {
                throw std::runtime_error("Failed to load lines shader");
            }
        }

        // Initialize VAO/VBO if needed
        if (!linesVAOInitialized) {
            glGenVertexArrays(1, &linesVAO);
            glGenBuffers(1, &linesVBO);
            linesVAOInitialized = true;
        }

        float w = RenderingSystem::getInstance().getWindowWidth();
        float h = RenderingSystem::getInstance().getWindowHeight();

        // Convert from pixel coordinates to normalized device coordinates [-1, 1]
        std::vector<float> screenCoords;
        screenCoords.reserve(vertices.size());
        
        for (size_t i = 0; i < vertices.size(); i += 2) {
            float x = (vertices[i] / w) * 2.0f - 1.0f;
            float y = (vertices[i + 1] / h) * 2.0f - 1.0f;
            screenCoords.push_back(x);
            screenCoords.push_back(y);
        }

        linesShader.use();
        glUniform2f(linesShader.getUniformLocation("resolution"), w, h);
        glUniform4f(linesShader.getUniformLocation("color"), r, g, b, a);

        glBindVertexArray(linesVAO);
        glBindBuffer(GL_ARRAY_BUFFER, linesVBO);
        glBufferData(GL_ARRAY_BUFFER, screenCoords.size() * sizeof(float), screenCoords.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glDrawArrays(GL_LINES, 0, screenCoords.size() / 2);
        
        glBindVertexArray(0);
    }

    inline void _destroyLines() {
        linesShader.destroy();
        if (linesVAOInitialized) {
            glDeleteVertexArrays(1, &linesVAO);
            glDeleteBuffers(1, &linesVBO);
            linesVAOInitialized = false;
        }
    }
}

#endif // CRIDGEON_SHADER_LINES_HPP
